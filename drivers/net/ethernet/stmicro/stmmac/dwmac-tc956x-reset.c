// SPDX-License-Identifier: GPL-2.0

/*
 * Copyright (C) 2026 by RISCstar Solutions Corporation.  All rights reserved.
 */

/*
 * The Toshiba TC9564 implements a PCIe Gen 3 switch that connects an
 * upstream x4 port to two downstream PCIe x2 ports.  It incorporates
 * an internal endpoint as well, which implements two Synopsys XGMAC
 * Ethernet interfaces.
 *
 * In addition, a set of 13 reset signals are implemented.  One register
 * controls the first 10 of these, and a second controls three more.
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/reset-controller.h>

#include "dwmac-tc956x-reset.h"

/**
 * struct tc956x_reset_data - Information about a single reset
 * @offset:	Offset in the regmap for register to update
 * @bit:	Bit position to modify (0: deasserted; 1: asserted)
 */
struct tc956x_reset_data {
	u32 offset;
	u32 bit;
};

/**
 * struct tc956x_reset_controller - The embedded reset controller
 * @rcdev:	Generic reset controller structure
 * @regmap:	MMIO register map for SFR reset region access
 * @data_count:	Number of entries in the data[] array
 * @data:	Array of register offsets and bit numbers per reset
 */
struct tc956x_reset_controller {
	struct reset_controller_dev rcdev;
	struct regmap *regmap;
	size_t data_count;
	const struct tc956x_reset_data *data;		/* Array */
};

/* The reset (and clock) offsets are relative to 0x1000 in SFR space */
#define TC9564_NRSTCTRL0_OFFSET	0x08
#define TC9564_NRSTCTRL1_OFFSET	0x10

static const struct tc956x_reset_data tc9564_resets[] = {
	[TC9564_RESET_MCU] = {
		.offset = TC9564_NRSTCTRL0_OFFSET,
		.bit = 0,
	},
	[TC9564_RESET_MCU1] = {
		.offset = TC9564_NRSTCTRL0_OFFSET,
		.bit = 1,
	},
	[TC9564_RESET_INTC] = {
		.offset = TC9564_NRSTCTRL0_OFFSET,
		.bit = 4,
	},
	[TC9564_RESET_MSIGEN] = {
		.offset = TC9564_NRSTCTRL0_OFFSET,
		.bit = 18,
	},
	[TC9564_RESET_UART0] = {
		.offset = TC9564_NRSTCTRL0_OFFSET,
		.bit = 16,
	},
	[TC9564_RESET_MAC0] = {
		.offset = TC9564_NRSTCTRL0_OFFSET,
		.bit = 7,
	},
	[TC9564_RESET_MAC0_PMA] = {
		.offset = TC9564_NRSTCTRL0_OFFSET,
		.bit = 30,
	},
	[TC9564_RESET_MAC0_XPCS] = {
		.offset = TC9564_NRSTCTRL0_OFFSET,
		.bit = 31,
	},
	[TC9564_RESET_MAC1] = {
		.offset = TC9564_NRSTCTRL1_OFFSET,
		.bit = 7,
	},
	[TC9564_RESET_MAC1_PMA] = {
		.offset = TC9564_NRSTCTRL1_OFFSET,
		.bit = 30,
	},
	[TC9564_RESET_MAC1_XPCS] = {
		.offset = TC9564_NRSTCTRL1_OFFSET,
		.bit = 31,
	},
};

static int tc956x_reset_update(struct reset_controller_dev *rcdev,
			       unsigned long id, bool assert)
{
	struct tc956x_reset_controller *controller;
	const struct tc956x_reset_data *data;
	u32 mask;
	u32 val;

	controller = container_of(rcdev, struct tc956x_reset_controller, rcdev);
	if (id >= controller->data_count)
		return -EINVAL;

	data = &controller->data[id];
	mask = BIT(data->bit);
	val = assert ? mask : 0;

	return regmap_update_bits(controller->regmap, data->offset, mask, val);
}

static int
tc956x_reset_assert(struct reset_controller_dev *rcdev, unsigned long id)
{
	return tc956x_reset_update(rcdev, id, true);
}

static int
tc956x_reset_deassert(struct reset_controller_dev *rcdev, unsigned long id)
{
	return tc956x_reset_update(rcdev, id, false);
}

static const struct reset_control_ops tc956x_reset_control_ops = {
	.assert		= tc956x_reset_assert,
	.deassert	= tc956x_reset_deassert,
};

static int tc956x_reset_controller_probe(struct platform_device *pdev)
{
	struct tc956x_reset_controller *controller;
	struct reset_controller_dev *rcdev;
	struct device *dev = &pdev->dev;

	controller = devm_kzalloc(dev, sizeof(*controller), GFP_KERNEL);
	if (!controller)
		return -ENOMEM;

	controller->regmap = dev_get_regmap(dev->parent, "tc956x-clk-reset");
	if (!controller->regmap)
		return -EINVAL;

	controller->data = tc9564_resets;
	controller->data_count = ARRAY_SIZE(tc9564_resets);

	rcdev = &controller->rcdev;
	rcdev->ops = &tc956x_reset_control_ops;
	rcdev->owner = dev->driver->owner;
	rcdev->of_node = dev->parent->of_node;
	rcdev->nr_resets = controller->data_count;

	return devm_reset_controller_register(dev, &controller->rcdev);
}

/* XXX This should be specifically for TC9564 */
/* XXX And then we'll use auxiliary device to match instead */
static struct platform_driver tc956x_reset_controller_driver = {
	.probe = tc956x_reset_controller_probe,
	.driver = {
		.name = "tc956x-reset-controller",
	},
};

module_platform_driver(tc956x_reset_controller_driver);

MODULE_DESCRIPTION("Toshiba TC956x PCIe Reset Controller Driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:tc956x-reset-controller");
