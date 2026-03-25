// SPDX-License-Identifier: GPL-2.0

/*
 * Copyright (C) 2026 by RISCstar Solutions Corporation.  All rights reserved.
 *
 * Derived from code having the following copyrights:
 * Copyright (C) 2011-2012  Vayavya Labs Pvt Ltd
 * Copyright (C) 2025 Toshiba Electronic Devices & Storage Corporation
 */

#include <linux/auxiliary_bus.h>
#include <linux/dev_printk.h>
#include <linux/module.h>
#include <linux/pm.h>

#include "soc-tc9564-chip.h"

#define DRIVER_NAME		"dwmac-tc9564"

/*
 * struct tc9564_dwmac - Information related to an embedded XGMAC
 * @data:		Pointer to data passed from the parent
 * @chip:		Handle used for common chip operations
 */
struct tc9564_dwmac {
	struct tc9564_dwmac_data *data;
	void *chip;				/* Intentionally opaque */
};

static const struct auxiliary_device_id tc964_dwmac_ids[] = {
	{ .name = "misc_tc9564_chip." DRIVER_NAME, },
	{ }
};
MODULE_DEVICE_TABLE(auxiliary, tc964_dwmac_ids);

static int tc9564_dwmac_probe(struct auxiliary_device *adev,
			      const struct auxiliary_device_id *id)
{
	struct device *dev = &adev->dev;
	struct tc9564_dwmac *dwmac;

	dev_info(dev, " === %s\n", __func__);

	if (!dev->platform_data)
		return -EINVAL;

	dwmac = devm_kzalloc(dev, sizeof(*dwmac), GFP_KERNEL);
	if (!dwmac)
		return -ENOMEM;

	dwmac->chip = dev_get_platdata(dev->parent);
	dwmac->data = dev_get_platdata(dev);

	dev_set_drvdata(dev, dwmac);

	return 0;
}

static int tc9564_dwmac_suspend(struct device *dev)
{
	dev_info(dev, " === %s\n", __func__);

	return 0;
}

static int tc9564_dwmac_resume(struct device *dev)
{
	dev_info(dev, " === %s\n", __func__);

	return 0;
}

static SIMPLE_DEV_PM_OPS(tc9564_dwmac_pm_ops, tc9564_dwmac_suspend,
			 tc9564_dwmac_resume);

static struct auxiliary_driver tc9564_dwmac_driver = {
	.name		= DRIVER_NAME,
	.probe          = tc9564_dwmac_probe,
	.id_table       = tc964_dwmac_ids,
	.driver = {
		.name		= DRIVER_NAME,
		.pm		= &tc9564_dwmac_pm_ops,
		/* .owner	= THIS_MODULE, */
		/* .probe_type	= PROBE_PREFER_ASYNCHRONOUS, */
	},
};
module_auxiliary_driver(tc9564_dwmac_driver);

MODULE_DESCRIPTION("Toshiba TC956x PCIe XGMAC Network Driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("auxiliary:" DRIVER_NAME);
