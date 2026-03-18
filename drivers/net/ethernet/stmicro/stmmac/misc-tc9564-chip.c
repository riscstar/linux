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
#include <linux/init.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/printk.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/types.h>

#include "soc-tc9564-chip.h"

#define DRIVER_NAME			"tc9564-chip"

#define GPIO_DEVICE_NAME		"tc9564-gpio"

#define PCI_DEVICE_ID_TOSHIBA_TC9564	0x0220

/* PCI BAR assignments */
#define PCI_BAR_SFR			4

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

/*
 * struct tc9564_chip - Common information related to the TC9564 chip
 * @dev:		Device structure
 * @sfr:		Mapped SFR region (BAR 4)
 * @reset_clock_regmap:	Regmap used for resets and clocks
 */
struct tc9564_chip {
	struct device *dev;
	void __iomem *sfr;
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

static void regmap_log(const char *name, const struct regmap_config *config)
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

/* The embedded GPIO controller has an auxiliary device driver */
static int gpio_auxiliary_device_add(struct tc9564_chip *chip)
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

	regmap_log("misc-gpio", &gpio_regmap_config);

	if (!devm_auxiliary_device_create(dev, GPIO_DEVICE_NAME, regmap))
		return -ENOMEM;

	return 0;
}

static int reset_clock_init(struct tc9564_chip *chip)
{
	struct device *dev = chip->dev;
	void __iomem *base = chip->sfr;
	struct regmap *regmap;

	regmap = devm_regmap_init_mmio(dev, base, &reset_clock_regmap_config);
	if (IS_ERR(regmap))
		return PTR_ERR(regmap);
	chip->reset_clock_regmap = regmap;

	regmap_log("misc-reset-clock", &reset_clock_regmap_config);

	return 0;
}

/*
 * Function 1 will first look up its peer device (function 0).  If
 * its driver data is NULL, it hasn't yet probed, so function 1
 * will return -EPROBE_DEFER.  Otherwise function 0's driver data
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

	chip = dev_get_drvdata(&peer->dev);
	if (!chip)
		return ERR_PTR(-EPROBE_DEFER);

	/* XXX Can't we DL_FLAG_AUTOREMOVE_SUPPLIER instead? */
	/* Mark this device as dependent on function 0 */
	link = device_link_add(dev, &peer->dev, DL_FLAG_AUTOREMOVE_SUPPLIER);
	if (link) {
		devm_add_action_or_reset(&peer->dev, chip_link_del, link);

		dev_set_drvdata(dev, chip);
	}

	return link ? chip : ERR_PTR(-ENODEV);
}

/*
 * Function 0 will allocate the chip structure that is shared by both
 * functions.  Once it has allocated the structure it assigns it as
 * the PCI device driver data.
 *
 * Returns a chip structure pointer, or a pointer-coded error.
 */
static struct tc9564_chip *chip_get(struct pci_dev *pdev)
{
	struct device *dev = &pdev->dev;
	struct tc9564_chip *chip;
	int ret;

	if (PCI_FUNC(pdev->devfn))
		return chip_get_function1(pdev);

	chip = kzalloc_obj(*chip);
	if (!chip)
		return ERR_PTR(-ENOMEM);

	chip->dev = dev;

	chip->sfr = pcim_iomap_region(pdev, PCI_BAR_SFR, DRIVER_NAME);
	if (IS_ERR(chip->sfr))
		return ERR_CAST(chip->sfr);

	ret = reset_clock_init(chip);
	if (ret)
		return ERR_PTR(ret);

	ret = gpio_auxiliary_device_add(chip);
	if (ret)
		return ERR_PTR(ret);

	dev_set_drvdata(dev, chip);

	return chip;
}

static int tc9564_probe(struct pci_dev *pdev, const struct pci_device_id *id)
{
	struct device *dev = &pdev->dev;
	struct tc9564_chip *chip;

	printk(" === %s\n", __func__);

	if (!dev->of_node)
		return -EINVAL;

	chip = chip_get(pdev);
	if (IS_ERR(chip))
		return dev_err_probe(dev, PTR_ERR(chip), "failed to get chip\n");

	dev_info(dev, " === %s success\n", __func__);

	return 0;
}

static void tc9564_remove(struct pci_dev *pdev)
{
	dev_info(&pdev->dev, " === %s success\n", __func__);
}

static const struct pci_device_id tc9564_id_table[] = {
	{ PCI_DEVICE(PCI_VENDOR_ID_TOSHIBA, PCI_DEVICE_ID_TOSHIBA_TC9564), },
	{ },
};
MODULE_DEVICE_TABLE(pci, tc9564_id_table);

static struct pci_driver tc9564_pci_driver = {
	.name		= DRIVER_NAME,
	.id_table	= tc9564_id_table,
	.probe		= tc9564_probe,
	.remove		= tc9564_remove,
	.driver		= {
		.name	= DRIVER_NAME,
		.owner	= THIS_MODULE,
	},
};

module_pci_driver(tc9564_pci_driver);

MODULE_DESCRIPTION("Toshiba TC9564 PCIe Embedded Function Driver");
MODULE_LICENSE("GPL");
