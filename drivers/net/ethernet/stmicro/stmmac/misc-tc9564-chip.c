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

/* For now we'll just make this be an alias for the tc9564_function structure */
struct tc9564_chip {
};

/*
 * struct tc9564_function - Information related to the embedded GPIO controller
 * @pdev:		PCI device structure
 * @sfr:		Mapped SFR region (BAR 4)
 * @pci_fn:		Which PCI function this is (0 or 1)
 */
struct tc9564_function {
	struct pci_dev *pdev;
	void __iomem *sfr;
	struct regmap *reset_clock_regmap;
	u8 pci_fn;			/* XXX Redundant if we keep pdev */
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

static struct tc9564_function *chip_to_function(struct tc9564_chip *chip)
{
	return (struct tc9564_function *)chip;
}

/* Common clock/reset register update function */
void tc9564_chip_reset_clock_set(struct tc9564_chip *chip, bool reset,
				 bool reg0, bool set, u8 bit)
{
	u32 offset = reset ? reg0 ? RSTCTRL0_OFFSET : RSTCTRL1_OFFSET
			   : reg0 ? CLKCTRL0_OFFSET : CLKCTRL1_OFFSET;
	struct tc9564_function *function = chip_to_function(chip);
	u32 mask = BIT(bit);

	/* Note: no need to check for errors on read/write for MMIO regmap */
	(void)regmap_update_bits(function->reset_clock_regmap, offset, mask,
				 set ? mask : 0);
}
EXPORT_SYMBOL_GPL(tc9564_chip_reset_clock_set);

static void adev_release(struct device *dev)
{
	kfree(to_auxiliary_dev(dev));
}

static void adev_remove(void *data)
{
	struct auxiliary_device *adev = data;

	auxiliary_device_delete(adev);
	auxiliary_device_uninit(adev);
}

static int devm_adev_device_add(struct tc9564_function *function,
				const char *name, struct regmap *regmap)
{
	struct device *dev = &function->pdev->dev;
	struct auxiliary_device *adev;
	int ret;

	adev = devm_kzalloc(dev, sizeof(*adev), GFP_KERNEL);
	if (!adev)
		return -ENOMEM;

	adev->name = name;
	adev->dev.parent = dev;
	adev->dev.release = adev_release;
	adev->dev.of_node = dev->of_node;
	adev->dev.platform_data = regmap;
	adev->id = function->pci_fn;

	ret = auxiliary_device_init(adev);
	if (ret)
		return ret;

	ret = auxiliary_device_add(adev);
	if (ret) {
		auxiliary_device_uninit(adev);
		return ret;
	}

	return devm_add_action_or_reset(dev, adev_remove, adev);
}

/* The embedded GPIO controller has an auxiliary device driver */
static int devm_gpio_auxiliary_device_add(struct tc9564_function *function)
{
	struct device *dev = &function->pdev->dev;
	void __iomem *base = function->sfr;
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

	return devm_adev_device_add(function, GPIO_DEVICE_NAME, regmap);
}

static int reset_clock_init(struct tc9564_function *function)
{
	struct device *dev = &function->pdev->dev;
	void __iomem *base = function->sfr;
	struct regmap *regmap;

	regmap = devm_regmap_init_mmio(dev, base, &reset_clock_regmap_config);
	if (IS_ERR(regmap))
		return PTR_ERR(regmap);
	function->reset_clock_regmap = regmap;

	return 0;
}

static int tc9564_probe(struct pci_dev *pdev, const struct pci_device_id *id)
{
	struct tc9564_function *function;
	struct device *dev = &pdev->dev;
	int ret;

	printk(" === %s\n", __func__);

	if (!dev->of_node)
		return -EINVAL;

	function = devm_kzalloc(dev, sizeof(*function), GFP_KERNEL);
	if (!function)
		return -ENOMEM;

	function->pdev = pdev;

	function->pci_fn = PCI_FUNC(pdev->devfn);
	if (WARN_ON(function->pci_fn > 1))
		return -EINVAL;

	function->sfr = pcim_iomap_region(pdev, PCI_BAR_SFR, DRIVER_NAME);
	if (IS_ERR(function->sfr))
		return dev_err_probe(dev, PTR_ERR(function->sfr),
				     "failed to map sfr region\n");

	ret = reset_clock_init(function);
	if (ret)
		return dev_err_probe(dev, ret,
				     "failed to initialize reset/clock\n");

	ret = devm_gpio_auxiliary_device_add(function);
	if (ret)
		return dev_err_probe(dev, ret,
				     "failed to add GPIO device\n");

	pci_set_drvdata(pdev, function);

	printk(" === %s function %u has probed successfully\n", __func__,
		function->pci_fn);

	return 0;
}

static void tc9564_remove(struct pci_dev *pdev)
{
	struct tc9564_function *function = pci_get_drvdata(pdev);

	printk(" === %s function %u has been removed successfully\n", __func__,
		function->pci_fn);
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
