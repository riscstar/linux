// SPDX-License-Identifier: GPL-2.0-only

/*
 * Copyright (C) 2026 by RISCstar Solutions Corporation.  All rights reserved.
 */

#include <linux/bits.h>
#include <linux/device-id/of.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/reset-controller.h>

#include <dt-bindings/reset/toshiba,tc9564.h>

#define RST_CTRL0_OFFSET	0x1008
#define RST_CTRL1_OFFSET	0x1010

struct tc9564_reset {
	u32 offset;		/* RST_CTRL0_OFFSET or RST_CTRL1_OFFSET */
	u32 mask;		/* Zero means undefined reset */
};

struct tc9564_resets {
	struct regmap *regmap;
	struct reset_controller_dev rcdev;
};

#define TC9564_RESET_INIT0(_name, _bit)	__TC9564_RESET_INIT(_name, 0, _bit)
#define TC9564_RESET_INIT1(_name, _bit)	__TC9564_RESET_INIT(_name, 1, _bit)

#define __TC9564_RESET_INIT(_name, _reg, _bit)		\
	[RESET_##_name] = {				\
		.offset	= RST_CTRL ## _reg ## _OFFSET,	\
		.mask = BIT(_bit),			\
	}

/*
 * The PCIe and I2C resets are controllable, but we depend on PCIe
 * and that relies on I2C, so we don't allow them to be asserted.
 */
static const struct tc9564_reset tc9564_reset[] = {
	TC9564_RESET_INIT0(MCU, 0),
	TC9564_RESET_INIT0(MCU1, 1),
	TC9564_RESET_INIT0(INTC, 4),
	/* TC9564_RESET_INIT0(PCIE, 9), */
	/* TC9564_RESET_INIT0(I2C, 12), */
	TC9564_RESET_INIT0(UART, 16),
	TC9564_RESET_INIT0(MSIGEN, 18),

	TC9564_RESET_INIT0(MAC0_MAC, 7),
	TC9564_RESET_INIT0(MAC0_PMA, 30),
	TC9564_RESET_INIT0(MAC0_XPCS, 31),

	TC9564_RESET_INIT1(MAC1_MAC, 7),
	TC9564_RESET_INIT1(MAC1_PMA, 30),
	TC9564_RESET_INIT1(MAC1_XPCS, 31),
};
#define TC9564_RESET_COUNT	ARRAY_SIZE(tc9564_reset)

static const struct tc9564_resets *
rcdev_to_tc9564_resets(struct reset_controller_dev *rcdev)
{
	return container_of_const(rcdev, struct tc9564_resets, rcdev);
}

static int tc9564_reset_manage(struct reset_controller_dev *rcdev,
			       unsigned long id, bool assert)
{
	const struct tc9564_resets *resets = rcdev_to_tc9564_resets(rcdev);

	if (id < rcdev->nr_resets) {
		const struct tc9564_reset *reset = &tc9564_reset[id];
		u32 mask = reset->mask;

		if (mask)
			return regmap_update_bits(resets->regmap,
						  reset->offset, mask,
						  assert ? mask : 0);
	}

	dev_err(rcdev->dev, "invalid reset (%s id %lu)\n",
		assert ? "assert" : "deassert", id);

	return -ENXIO;
}

static int tc9564_reset_assert(struct reset_controller_dev *rcdev,
			       unsigned long id)
{
	return tc9564_reset_manage(rcdev, id, true);
}

static int tc9564_reset_deassert(struct reset_controller_dev *rcdev,
				 unsigned long id)
{
	return tc9564_reset_manage(rcdev, id, false);
}

static const struct reset_control_ops tc9564_reset_control_ops = {
	.assert = tc9564_reset_assert,
	.deassert = tc9564_reset_deassert,
};

/* Assert all resets by updating both reset registers */
static void tc9564_reset_assert_all(struct tc9564_resets *resets)
{
	u32 mask0 = 0;
	u32 mask1 = 0;

	for (u32 id = 0; id < TC9564_RESET_COUNT; id++) {
		const struct tc9564_reset *reset = &tc9564_reset[id];

		if (!reset->mask)
			continue;

		if (reset->offset == RST_CTRL0_OFFSET)
			mask0 |= reset->mask;
		else
			mask1 |= reset->mask;
	}
	regmap_update_bits(resets->regmap, RST_CTRL0_OFFSET, mask0, mask0);
	regmap_update_bits(resets->regmap, RST_CTRL1_OFFSET, mask1, mask1);
}

static struct tc9564_resets *tc9564_reset_init(struct device *dev)
{
	struct reset_controller_dev *rcdev;
	struct tc9564_resets *resets;
	struct regmap *regmap;
	int ret;

	regmap = syscon_node_to_regmap(dev_of_node(dev->parent));
	if (IS_ERR(regmap)) {
		dev_err(dev, "failed to get config regmap\n");
		return ERR_CAST(regmap);
	}

	resets = devm_kzalloc(dev, sizeof(*resets), GFP_KERNEL);
	if (!resets)
		return ERR_PTR(-ENOMEM);
	resets->regmap = regmap;

	rcdev = &resets->rcdev;
	rcdev->ops = &tc9564_reset_control_ops;
	rcdev->owner = THIS_MODULE;
	rcdev->dev = dev;
	rcdev->of_node = dev_of_node(dev);
	rcdev->nr_resets = TC9564_RESET_COUNT;

	/* Before registering, force all resets to be initially asserted */
	tc9564_reset_assert_all(resets);

	ret = devm_reset_controller_register(dev, rcdev);
	if (ret)
		return ERR_PTR(ret);

	return resets;
}

static int tc9564_reset_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct tc9564_resets *resets;

	if (!dev_of_node(dev))
		return dev_err_probe(dev, -EINVAL, "no devicetree node\n");

	resets = tc9564_reset_init(dev);
	if (IS_ERR(resets))
		return dev_err_probe(dev, PTR_ERR(resets),
				     "failed to initialize resets\n");

	platform_set_drvdata(pdev, resets);

	return 0;
}

static const struct of_device_id tc9564_reset_ids[] = {
	{ .compatible = "toshiba,tc9564-reset" },
	{ }
};
MODULE_DEVICE_TABLE(of, tc9564_reset_ids);

static struct platform_driver tc9564_reset_driver = {
	.probe = tc9564_reset_probe,
	.driver = {
		.name = KBUILD_MODNAME,
		.of_match_table = tc9564_reset_ids,
		.probe_type = PROBE_PREFER_ASYNCHRONOUS,
	},
};
module_platform_driver(tc9564_reset_driver);

MODULE_DESCRIPTION("Toshiba TC9564 Reset Driver");
MODULE_LICENSE("GPL");
