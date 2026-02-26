// SPDX-License-Identifier: GPL-2.0

/*
 * Copyright (C) 2026 by RISCstar Solutions Corporation.  All rights reserved.
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/printk.h>
#include <linux/regmap.h>

static int tc956x_gpio_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct regmap *regmap;

	printk(" === %s\n", __func__);

	regmap = dev_get_regmap(dev->parent, "tc956x-gpio");
	if (!regmap)
		return -EINVAL;

	return 0;
}

static struct platform_driver tc956x_gpio_driver = {
	.probe = tc956x_gpio_probe,
	.driver = {
		.name = "tc956x-gpio",
	},
};

module_platform_driver(tc956x_gpio_driver);

MODULE_DESCRIPTION("Toshiba TC956x PCIe GPIO Driver");
MODULE_LICENSE("GPL");
