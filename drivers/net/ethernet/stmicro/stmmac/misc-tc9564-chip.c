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

#include <linux/device.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/printk.h>
#include <linux/types.h>

#define DRIVER_NAME			"tc9564-chip"

#define PCI_DEVICE_ID_TOSHIBA_TC9564	0x0220

/*
 * struct tc9564_function - Information related to the embedded GPIO controller
 * @pci_fn:		Which PCI function this is (0 or 1)
 */
struct tc9564_function {
	u8 pci_fn;
};

static int tc9564_probe(struct pci_dev *pdev, const struct pci_device_id *id)
{
	struct tc9564_function *function;
	struct device *dev = &pdev->dev;

	printk(" === %s\n", __func__);

	function = devm_kzalloc(dev, sizeof(*function), GFP_KERNEL);
	if (!function)
		return -ENOMEM;

	function->pci_fn = PCI_FUNC(pdev->devfn);
	if (WARN_ON(function->pci_fn > 1))
		return -EINVAL;

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
