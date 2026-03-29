// SPDX-License-Identifier: GPL-2.0

/*
 * Copyright (C) 2026 by RISCstar Solutions Corporation.  All rights reserved.
 */

/*
 * The Toshiba TC9564 implements a PCIe Gen 3 switch that connects an upstream
 * x4 port to two downstream PCIe x2 ports.  It incorporates an internal
 * endpoint as well, which implements two Synopsys XGMAC Ethernet interface.
 * functions.
 *
 * The TC9564 incorporates other functionality, including an embedded
 * MCU, a UART, a GPIO controller, internal resets and clocks, and
 * interrupt handling.  These components are shared by both Ethernet
 * XGMACs.  Each Ethernet MAC must have be attached to a working PHY
 * for it to be functional, and for this reason either of them (or both!)
 * might not be usable/used.
 *
 * To support the non-XGMAC functionality on the TC9564 regardless of
 * the presense of either Ethernet PHY, the Ethernet functions are
 * treated as two parts:  a PCI function; and a Synopsys XGMAC component.
 * The PCI function has access to the BARs used by the XGMAC, and maps
 * them for use.  The XGMAP is treated as an auxiliary sub-device of
 * the PCI function, which is probed and bound separate from its
 * associated (parent) PCI function.
 *
 * This PCI driver matches the Toshiba TC956X (physical) PCI endpoint
 * device, (VID 0x1179, DID 0x0220).  There are two of these present
 * on the TC9564 SoC.  The first (PCI function 0) is the "primary"
 * function, which generally handles "chip" activities that are used
 * by both XGMACs.  This includes creating and registering the GPIO
 * auxiliary device, as well as asserting and deasserting internal
 * reset signals and enabling and disabling internal clocks.
 *
 * The PCI function driver maps the PCI BARs and performs other initial
 * setup, then registers auxiliary devices.  The GPIO device is registered
 * first (function 0 only), then the XGMAC device is registered.  A "chip"
 * data structure is shared with the XGMAC devices.
 *
 * And at this point I think I need to implement this thing to see how
 * it's really, REALLY going to work.
 */

#include <linux/auxiliary_bus.h>
#include <linux/device.h>
#include <linux/dev_printk.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/pci.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/types.h>

#include "soc-tc9564-chip.h"

#define DRIVER_NAME			"tc9564-chip"

#define GPIO_DEVICE_NAME		"tc9564-gpio"
#define XGMAC_DEVICE_NAME		"dwmac-tc9564x"

#define PCI_DEVICE_ID_TOSHIBA_TC9564	0x0220

/* PCI BAR assignments */
#define PCI_BAR_BRIDGE_CONFIG		0
#define PCI_BAR_SFR			4

/* Chip and revision ID register */
#define NCID_OFFSET			0x0000
#define NCID_REV_ID_MASK		GENMASK(7, 0)

/* Reset and clock register offsets.  Chip resets and clocks are controlled
 * by bits in register 0.  MAC resets and clocks are controlled by bits in
 * register 0 for MAC0, register 1 for MAC1.
 *
 * These are relative to the base of the clock/reset regmap.
 */
#define RSTCTRL0_OFFSET			0x0008
#define RSTCTRL1_OFFSET			0x0010
#define CLKCTRL0_OFFSET			0x0004
#define CLKCTRL1_OFFSET			0x000c

/* There are 4 AXI translation table entries each with 8 4-byte register */
#define ATR_AXI4_SLV0_OFFSET		0x0800

#define AXI4_TABLE_ENTRY_COUNT		4
#define AXI4_ENTRY_BASE(id)		((id) * AXI4_TABLE_STRIDE)
#define AXI4_TABLE_STRIDE               0x20

/* Address translation space parameters (entry 0) */
#define SLV00_ATR_SIZE			35	/* 2^36 (64 gigabytes) */
#define SLV00_SRC_ADDR			0x0000001000000000ULL
#define SLV00_TRSL_ADDR			0x0000000000000000ULL

/* Address translation space parameters (entries 1-3); SRC and TRSL are 0x0 */
#define SLV00_ATR_SIZE_DEFAULT		63	/* 2^64 (16 exabytes) */

/* Translation entry registers, fields, and values used */
#define SRC_ADDR_LO_OFFSET		0x0000
#define ATR_IMPL			BIT(0)		/* 1 = enabled */
#define ATR_SIZE_MASK			GENMASK(6, 1)	/* size 2^(ATR + 1) */
#define SRC_ADDR_HI_OFFSET		0x0004
#define TRSL_ADDR_LO_OFFSET		0x0008
#define TRSL_ADDR_HI_OFFSET		0x000c
#define TRSL_PARAM_OFFSET		0x0010
#define TRSL_ID_MASK			GENMASK(3, 0)
#define TRSL_ID_PCIE_TX_RX		0
#define TRSF_PARAM_MASK			GENMASK(27, 16)

/*
 * struct tc9564_chip - Common information related to the TC9564 chip
 * @dev:		Device structure
 * @rev_id:		Chip revision ID (for quirks)
 * @sfr:		Mapped SFR region (BAR 4)
 * @bridge_config:	Regmap used for bridge configuration
 * @reset_clock_regmap:	Regmap used for resets and clocks
 */
struct tc9564_chip {
	struct device *dev;
	void __iomem *sfr;
	u8 rev_id;
	void __iomem *bridge_config;
	struct regmap *reset_clock_regmap;
};

static const struct regmap_config gpio_regmap_config = {
	.name		= "tc9564-gpio",
	.reg_bits	= 32,
	.reg_stride	= 4,
	.reg_base	= 0x1200,	/* Register GPIOI0 */
	.val_bits	= 32,
	.max_register	= 0x1214,	/* Register GPIOO1 */
};

static const struct regmap_config reset_clock_regmap_config = {
	.name		= "tc956x-clk-reset",
	.reg_bits	= 32,
	.reg_stride	= 4,
	.reg_base	= 0x1000,	/* Register NCTLSTS */
	.val_bits	= 32,
	.max_register	= 0x1010,	/* Register NRSTCTRL1 */
};

static void
regmap_log_mmio(const char *name, const struct regmap_config *config)
{
#if IS_ENABLED(CONFIG_TRACE_MMIO_ACCESS)
	void __iomem *range_base;
	unsigned long range_len;

	range_base = base + config->reg_base,
	len = config->max_register;
	len -= config->reg_base,
	len += config->reg_bits / BITS_PER_BYTE;

	log_mmio_register_range(range_base, range_len, name);
#endif
}

/* Common clock/reset register update function */
void tc9564_chip_reset_clock_set(struct tc9564_chip *chip, bool reset,
				 bool reg0, bool set, u8 bit)
{
	u32 offset = reset ? reg0 ? RSTCTRL0_OFFSET : RSTCTRL1_OFFSET
			   : reg0 ? CLKCTRL0_OFFSET : CLKCTRL1_OFFSET;
	u32 mask = BIT(bit);

	/* Note: no need to check for errors on read/write for MMIO regmap */
	(void)regmap_update_bits(chip->reset_clock_regmap, offset, mask,
				 set ? mask : 0);
}
EXPORT_SYMBOL_GPL(tc9564_chip_reset_clock_set);

static void adev_release(struct device *dev)
{
	struct auxiliary_device *adev = to_auxiliary_dev(dev);

	of_node_put(adev->dev.of_node);
	kfree(adev);
}

static void adev_remove(void *data)
{
	struct auxiliary_device *adev = data;

	auxiliary_device_delete(adev);
	auxiliary_device_uninit(adev);
}

static void auxiliary_device_set_dma_from_dev(struct auxiliary_device *adev,
					      struct device *dev)
{
	adev->dev.dma_mask = dev->dma_mask;
	adev->dev.dma_parms = dev->dma_parms;
	adev->dev.coherent_dma_mask = dev->coherent_dma_mask;

	dma_set_max_seg_size(&adev->dev, dma_get_max_seg_size(dev));
	dma_set_seg_boundary(&adev->dev, dma_get_seg_boundary(dev));
}

static int adev_device_add(struct device *dev, const char *name, u32 id,
			   void *platform_data)
{
	struct auxiliary_device *adev;
	int ret;

	adev = kzalloc_obj(*adev);
	if (!adev)
		return -ENOMEM;

	adev->id = id;
	adev->name = name;
	adev->dev.parent = dev;
	adev->dev.platform_data = platform_data;
	adev->dev.release = adev_release;
	device_set_of_node_from_dev(&adev->dev, dev);
	auxiliary_device_set_dma_from_dev(adev, dev);

	ret = auxiliary_device_init(adev);
	if (ret) {
		of_node_put(adev->dev.of_node);
		kfree(adev);
		return ret;
	}

	ret = auxiliary_device_add(adev);
	if (ret) {
		auxiliary_device_uninit(adev);
		return ret;
	}

	return devm_add_action_or_reset(dev, adev_remove, adev);
}

/* The embedded GPIO controller has an auxiliary device driver */
static int chip_gpio_adev_add(struct tc9564_chip *chip)
{
	struct device *dev = chip->dev;
	void __iomem *base = chip->sfr;
	struct regmap *regmap;

	/*
	 * We might be able to enforce "gpio-controller is only present
	 * in function 0 or 1, not both" via devicetree.  If not, it
	 * would be nice to be able to preclude both specifying it
	 * at this point.
	 */
	if (!device_property_present(dev, "gpio-controller"))
		return 0;

	regmap = devm_regmap_init_mmio(dev, base, &gpio_regmap_config);
	if (IS_ERR(regmap))
		return PTR_ERR(regmap);

	regmap_log_mmio("misc-gpio", &gpio_regmap_config);

	return adev_device_add(dev, GPIO_DEVICE_NAME, 0, regmap);
}

/* The two embedded XGMAC controllers have an auxiliary device driver */
static int function_xgmac_adev_add(struct pci_dev *pdev,
				   struct tc9564_chip *chip,
				   unsigned int irq)
{
	bool fn0 = !PCI_FUNC(pdev->devfn);
	struct device *dev = &pdev->dev;
	void __iomem *base = chip->sfr;
	struct tc9564_dwmac_data *data;
	int ret;

	/* The stmmac code wants an I/O pointer, not a regmap */
	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->sfr = base;
	data->rev_id = chip->rev_id;
	if (fn0) {
		data->dwmac_addr = base + 0x40000;
		data->msigen_addr = base + 0xf000;
		data->id = 0;
	} else {
		data->dwmac_addr = base + 0x48000;
		data->msigen_addr = base + 0xf100;
		data->id = 1;
	}
	data->msigen_irq = irq;

	ret = adev_device_add(dev, XGMAC_DEVICE_NAME, data->id, data);
	if (ret)
		return ret;

#if IS_ENABLED(CONFIG_TRACE_MMIO_ACCESS)
	log_mmio_register_range(data->dwmac_addr, 0x8000,
				fn0 ? "xgmac0" : "xgmac1");
	log_mmio_register_range(data->msigen_addr, 0x0100,
				fn0 ? "msigen0" : "msigen1");
#endif
	return 0;
}

static int chip_reset_clock_init(struct tc9564_chip *chip)
{
	struct device *dev = chip->dev;
	void __iomem *base = chip->sfr;
	struct regmap *regmap;

	regmap = devm_regmap_init_mmio(dev, base, &reset_clock_regmap_config);
	if (IS_ERR(regmap))
		return PTR_ERR(regmap);
	chip->reset_clock_regmap = regmap;

	regmap_log_mmio("misc-reset-clock", &reset_clock_regmap_config);

	return 0;
}

static int chip_translation_init(struct tc9564_chip *chip, struct pci_dev *pdev)
{
	void __iomem *base;

	dev_info(chip->dev, " === %s\n", __func__);

	base = pcim_iomap_region(pdev, PCI_BAR_BRIDGE_CONFIG, DRIVER_NAME);
	if (IS_ERR(base))
		return PTR_ERR(base);

	chip->bridge_config = base + ATR_AXI4_SLV0_OFFSET;

#if IS_ENABLED(CONFIG_TRACE_MMIO_ACCESS)
	log_mmio_register_range(chip->bridge_config, 0x0080, "translation");
#endif

	return 0;
}

/**
 * chip_translation_config() - Configure the table address map registers
 * @chip:	The TC9564 chip pointer
 *
 * Populate the registers used to convert the AXI bus accesses to PCI TLPs.
 */
static void chip_translation_config(struct tc9564_chip *chip)
{
	void __iomem *table_base = chip->bridge_config;
	void __iomem *entry_base;
	u32 trsf_param_val;
	u32 atr_size_val;
	u32 val;
	u32 i;

	dev_info(chip->dev, " === %s\n", __func__);

	/*
	 * The lower bits of the source address must be zero, because the
	 * SRC_ADDR_LO register encodes the address translation space size
	 * and "implmented" bit there.  The size field defines the size of
	 * the translation space (2^(ATR_SIZE + 1)).  The minimum size is
	 * 4096 bytes, so ATR_SIZE value must be 11 or more.
	 */
	BUILD_BUG_ON(!!u32_get_bits(lower_32_bits(SLV00_SRC_ADDR),
						  ATR_SIZE_MASK));
	BUILD_BUG_ON(SLV00_SRC_ADDR & ATR_IMPL);
	BUILD_BUG_ON(SLV00_ATR_SIZE < 11);

	/*
	 * We only use the first AXI4 slave translation table entry:
	 *	EDMA address region:	0x10 0000 0000 - 0x1f ffff ffff
	 *	is translated to:	0x00 0000 0000 - 0x0f ffff ffff
	 */
	entry_base = table_base + AXI4_ENTRY_BASE(0);

	atr_size_val = u32_encode_bits(SLV00_ATR_SIZE, ATR_SIZE_MASK);
	atr_size_val |= ATR_IMPL;

	val = lower_32_bits(SLV00_SRC_ADDR) | atr_size_val;
	writel(val, entry_base + SRC_ADDR_LO_OFFSET);

	val = upper_32_bits(SLV00_SRC_ADDR);
	writel(val, entry_base + SRC_ADDR_HI_OFFSET);

	val = lower_32_bits(SLV00_TRSL_ADDR);
	writel(val, entry_base + TRSL_ADDR_LO_OFFSET);

	val = upper_32_bits(SLV00_TRSL_ADDR);
	writel(val, entry_base + TRSL_ADDR_HI_OFFSET);

	/* This is value assigned to *all* TRSL_PARAM registers */
	trsf_param_val = u32_encode_bits(TRSL_ID_PCIE_TX_RX, TRSL_ID_MASK);
	trsf_param_val |= u32_encode_bits(0, TRSF_PARAM_MASK);

	writel(trsf_param_val, entry_base + TRSL_PARAM_OFFSET);

	/* Set all other unused entries to default values (no translation) */
	BUILD_BUG_ON(SLV00_ATR_SIZE_DEFAULT < 11);
	atr_size_val = u32_encode_bits(SLV00_ATR_SIZE_DEFAULT, ATR_SIZE_MASK);
	atr_size_val |= ATR_IMPL;
	for (i = 1; i < AXI4_TABLE_ENTRY_COUNT; i++) {
		entry_base = table_base + AXI4_ENTRY_BASE(i);

		writel(0x0 | atr_size_val, entry_base + SRC_ADDR_LO_OFFSET);
		writel(0x0, entry_base + SRC_ADDR_HI_OFFSET);
		writel(0x0, entry_base + TRSL_ADDR_LO_OFFSET);
		writel(0x0, entry_base + TRSL_ADDR_HI_OFFSET);
		writel(trsf_param_val, entry_base + TRSL_PARAM_OFFSET);
	}
}

static void chip_start(struct tc9564_chip *chip)
{
	chip_translation_config(chip);

	tc9564_chip_clock_enable(chip, CHIP_CLOCK_MSIGEN);
	tc9564_chip_reset_deassert(chip, CHIP_RESET_MSIGEN);
}

static void chip_stop(struct tc9564_chip *chip)
{
	tc9564_chip_reset_assert(chip, CHIP_RESET_MSIGEN);
	tc9564_chip_clock_disable(chip, CHIP_CLOCK_MSIGEN);
}

static void chip_init_state(struct tc9564_chip *chip)
{
	tc9564_chip_reset_assert(chip, CHIP_RESET_MCU);
	tc9564_chip_reset_assert(chip, CHIP_RESET_MCU1);
	tc9564_chip_reset_assert(chip, CHIP_RESET_INTC);
	tc9564_chip_reset_assert(chip, CHIP_RESET_UART0);

	tc9564_chip_clock_disable(chip, CHIP_CLOCK_MCU);
	tc9564_chip_clock_disable(chip, CHIP_CLOCK_SRAM);
	tc9564_chip_clock_disable(chip, CHIP_CLOCK_PLL);
	tc9564_chip_clock_disable(chip, CHIP_CLOCK_SGMII);
	tc9564_chip_clock_disable(chip, CHIP_CLOCK_REFCLK);
	tc9564_chip_clock_disable(chip, CHIP_CLOCK_INTC);
	tc9564_chip_clock_disable(chip, CHIP_CLOCK_UART0);

	chip_stop(chip);
}

/*
 * Function 1 will first look up its peer device (function 0).  If
 * its driver data is NULL, it hasn't yet probed, so function 1
 * will return -EPROBE_DEFER.  Otherwise function 0's platform data
 * pointer is returned.
 *
 * Returns a chip structure pointer, or a pointer-coded error.
 */
static void chip_link_del(void *data)
{
	struct device_link *link = data;

	device_link_del(link);
}

static struct tc9564_chip *chip_get_function1(struct pci_dev *pdev)
{
	struct device *dev = &pdev->dev;
	struct tc9564_chip *chip;
	struct device_link *link;
	struct pci_dev *peer;
	unsigned int devfn;

	/* Look up the PCI device for function 0 */
	devfn = PCI_DEVFN(PCI_SLOT(pdev->devfn), 0);
	peer = pci_get_slot(pdev->bus, devfn);
	if (!peer)
		return ERR_PTR(-ENXIO);

	chip = dev_get_platdata(&peer->dev);
	if (!chip)
		return ERR_PTR(-EPROBE_DEFER);

	/* XXX Can't we DL_FLAG_AUTOREMOVE_SUPPLIER instead? */
	/* Mark this device as dependent on function 0 */
	link = device_link_add(dev, &peer->dev, DL_FLAG_STATELESS);
	if (link) {
		devm_add_action_or_reset(&peer->dev, chip_link_del, link);

		dev->platform_data = chip;
	}

	return link ? chip : ERR_PTR(-ENODEV);
}

/*
 * Function 0 will allocate the chip structure that is shared by both
 * functions.  Once it has allocated the structure it assigns it as
 * the PCI device platform data.
 *
 * Returns a chip structure pointer, or a pointer-coded error.
 */
static struct tc9564_chip *chip_get(struct pci_dev *pdev)
{
	struct device *dev = &pdev->dev;
	struct tc9564_chip *chip;

	if (PCI_FUNC(pdev->devfn))
		return chip_get_function1(pdev);

	chip = devm_kzalloc(dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return ERR_PTR(-ENOMEM);

	chip->dev = dev;
	dev->platform_data = chip;

	return chip;
}

static int chip_init(struct tc9564_chip *chip, struct pci_dev *pdev)
{
	u32 val;
	int ret;

	/* Only function 0 does chip initialization */
	if (PCI_FUNC(pdev->devfn))
		return 0;

	ret = chip_translation_init(chip, pdev);
	if (ret)
		return ret;

	chip->sfr = pcim_iomap_region(pdev, PCI_BAR_SFR, DRIVER_NAME);
	if (IS_ERR(chip->sfr))
		return PTR_ERR(chip->sfr);

	/* Get the revision ID */
	val = readl(chip->sfr + NCID_OFFSET);
	chip->rev_id = u32_get_bits(val, NCID_REV_ID_MASK);

	ret = chip_reset_clock_init(chip);
	if (ret)
		return ret;

	chip_init_state(chip);

	ret = chip_gpio_adev_add(chip);
	if (ret)
		return ret;

	return 0;
}

static int
tc9564_chip_probe(struct pci_dev *pdev, const struct pci_device_id *id)
{
	bool fn0 = !PCI_FUNC(pdev->devfn);
	struct device *dev = &pdev->dev;
	struct tc9564_chip *chip;
	unsigned int irq;
	int ret;

	dev_info(dev, " === %s\n", __func__);

	if (!dev->of_node)
		return -EINVAL;

	ret = pcim_enable_device(pdev);
	if (ret)
		return ret;

	pci_set_master(pdev);

	chip = chip_get(pdev);
	if (IS_ERR(chip))
		return dev_err_probe(dev, PTR_ERR(chip), "failed to get chip\n");

	ret = chip_init(chip, pdev);
	if (ret)
		return dev_err_probe(dev, ret, "failed to initialize chip\n");

	/* pcim_enable_device() causes this to be freed automatically */
	ret = pci_alloc_irq_vectors(pdev, 1, 1, PCI_IRQ_MSI);
	if (ret < 1)
		return dev_err_probe(dev, ret ? : -EIO,
				     "failed to allocate IRQ vectors\n");

	ret = pci_irq_vector(pdev, 0);
	if (ret < 1)
		return dev_err_probe(dev, ret ? : -EIO, "failed to get IRQ\n");
	irq = ret;

	if (fn0)
		chip_start(chip);

	ret = function_xgmac_adev_add(pdev, chip, irq);
	if (ret)
		return dev_err_probe(dev, ret, "failed to add xgmap device\n");

	dev_info(dev, " === %s success\n", __func__);

	return 0;
}

static void tc9564_chip_remove(struct pci_dev *pdev)
{
	struct tc9564_chip *chip = dev_get_platdata(&pdev->dev);
	bool fn0 = &pdev->dev == chip->dev;

	dev_info(&pdev->dev, " === %s success\n", __func__);

	pci_clear_master(pdev);

	if (fn0)
		chip_stop(chip);
}

static const struct pci_device_id tc9564_chip_id_table[] = {
	{ PCI_DEVICE(PCI_VENDOR_ID_TOSHIBA, PCI_DEVICE_ID_TOSHIBA_TC9564), },
	{ },
};
#if !(IS_ENABLED(CONFIG_TC956X_NET) || IS_ENABLED(CONFIG_DWMAC_TC956X))
/* Only autoload if neither of these other drivers is enabled */
MODULE_DEVICE_TABLE(pci, tc9564_chip_id_table);
#endif

static int tc9564_chip_suspend_noirq(struct device *dev)
{
	struct tc9564_chip *chip = dev_get_platdata(dev);
	struct pci_dev *pdev = to_pci_dev(dev);
	bool fn0 = dev == chip->dev;
	int ret;

	dev_info(dev, " === %s\n", __func__);

	ret = pci_save_state(pdev);
	if (ret)
		return ret;

	pci_wake_from_d3(pdev, true);

	if (fn0)
		chip_stop(chip);

	return 0;
}

static int tc9564_chip_resume_noirq(struct device *dev)
{
	struct tc9564_chip *chip = dev_get_platdata(dev);
	struct pci_dev *pdev = to_pci_dev(dev);
	bool fn0 = dev == chip->dev;

	dev_info(dev, " === %s\n", __func__);

	if (fn0)
		chip_start(chip);

	pci_wake_from_d3(pdev, false);

	pci_set_power_state(pdev, PCI_D0);
	pci_restore_state(pdev);

	return 0;
}

static DEFINE_NOIRQ_DEV_PM_OPS(tc9564_chip_pm_ops,
			       tc9564_chip_suspend_noirq,
			       tc9564_chip_resume_noirq);

static struct pci_driver tc9564_chip_driver = {
	.name		= DRIVER_NAME,
	.id_table	= tc9564_chip_id_table,
	.probe		= tc9564_chip_probe,
	.remove		= tc9564_chip_remove,
	.driver		= {
		.name	= DRIVER_NAME,
		.owner	= THIS_MODULE,
		.pm	= pm_sleep_ptr(&tc9564_chip_pm_ops),
	},
};

module_pci_driver(tc9564_chip_driver);

MODULE_DESCRIPTION("Toshiba TC9564 PCIe Embedded Function Driver");
MODULE_LICENSE("GPL");
