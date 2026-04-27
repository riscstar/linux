// SPDX-License-Identifier: GPL-2.0

/*
 * Copyright (C) 2026 by RISCstar Solutions Corporation.  All rights reserved.
 */

/*
 * The Toshiba TC9564 implements a PCIe Gen 3 switch that connects an
 * upstream x4 port to three downstream PCIe ports--two external ones
 * and an internal one which implements an internal PCIe endpoint.  The
 * endpoint implements two PCIe functions, each having a Synopsys XGMAC
 * Ethernet interface.
 *
 * The XGMACs access the PCIe bus via an AXI bus.  The AXI bus treats
 * addresses above 64 GB (2^36) as PCIe addresses, and an address
 * translation unit translates between this AXI bus space and PCIe bus
 * space.
 */

#include <linux/device.h>
#include <linux/io.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>

/*
 * The bus translation function has four AXI translation table entries
 * each with eight 4-byte registers.  These entries translate between
 * an internal AXI bus address space and "external" PCIe address space.
 * The Ethernet MACs access the PCIe subsystem via this bus.  Currently
 * we only use the first translation table entry.
 */
#define AXI4_ENTRY_BASE(id)		((id) * AXI4_TABLE_STRIDE)
#define AXI4_TABLE_ENTRY_COUNT		4
#define AXI4_TABLE_STRIDE               0x20

/*
 * Address translation space parameters used for table entry 0.
 *
 * The source address defines the base address of a range of AXI addresses
 * that get translated to PCIe addresses.  The ATR_SIZE field determines
 * the size (2^(ATR_SIZE+1)) of that range.  Translation involves extracting
 * the offset of an address within the source range, and adding it to a
 * translated base address.  We use zero as the translated base address.
 */
#define TC9564_SLV00_SRC_ADDR		0x0000001000000000ULL
#define TRANSLATE_RANGE_BITS		36

/* Translation entry registers, fields, and values used */
#define SRC_ADDR_LO_OFFSET		0x0000
#define ATR_IMPL			BIT(0)		/* 1 = enabled */
#define ATR_SIZE_MASK			GENMASK(6, 1)	/* 2^(ATR_SIZE+1) */
#define SRC_ADDR_HI_OFFSET		0x0004
#define TRSL_ADDR_LO_OFFSET		0x0008
#define TRSL_ADDR_HI_OFFSET		0x000c
#define TRSL_PARAM_OFFSET		0x0010		/* Only 0 allowed */

/*
 * The lower bits of the source address must be zero, because the
 * "implemented" bit and the address translation space size are
 * encoded there in the SRC_ADDR_LO register.
 */
static_assert(!(TC9564_SLV00_SRC_ADDR & ATR_IMPL));
static_assert(!(lower_32_bits(TC9564_SLV00_SRC_ADDR) & ATR_SIZE_MASK));

/*
 * The ATR_SIZE field defines the size of the translation range as
 * (2^(ATR_SIZE + 1)).  The minimum size is 4096 bytes, so the value of
 * TRANSLATE_RANGE_BITS must be 12 or more.
 */
static_assert(TRANSLATE_RANGE_BITS >= ilog2(4096));

struct tc9564_translate {
	struct regmap *regmap;
	u32 offset;		/* Offset to the translation table base */
};

/**
 * tc9564_translate_config() - Configure the translation unit registers
 * @translate:	Private translation structure
 *
 * Define the translation between AXI bus accesses and PCI TLPs using only
 * the first translation table entry.  TC9564_SLV00_SRC_ADDR defines the
 * base address of the AXI address range.  AXI addresses are translated to
 * the PCIe address range, whose base address we set to be 0x0.
 */
static void tc9564_translate_config(struct tc9564_translate *translate)
{
	struct regmap *regmap = translate->regmap;
	u32 offset = translate->offset;
	u32 val;

	/* Disable all entries initially */
	for (u32 i = 0; i < AXI4_TABLE_ENTRY_COUNT; i++) {
		u32 reg = offset + AXI4_ENTRY_BASE(i) + SRC_ADDR_LO_OFFSET;

		regmap_write(regmap, reg, 0);
	}

	/* We only use table entry 0 */
	offset += AXI4_ENTRY_BASE(0);

	/* TC9564 only allows 0 to be written to this register */
	regmap_write(regmap, offset + TRSL_PARAM_OFFSET, 0);

	/* The translated base address is always just 0x0 */
	regmap_write(regmap, offset + TRSL_ADDR_HI_OFFSET, 0);
	regmap_write(regmap, offset + TRSL_ADDR_LO_OFFSET, 0);

	/* Encode the source base address and range size */
	val = upper_32_bits(TC9564_SLV00_SRC_ADDR);
	regmap_write(regmap, offset + SRC_ADDR_HI_OFFSET, val);

	val = lower_32_bits(TC9564_SLV00_SRC_ADDR);
	/* ATR_SIZE field defines the range size using range_bits-1 */
	val |= u32_encode_bits(TRANSLATE_RANGE_BITS - 1, ATR_SIZE_MASK);
	val |= ATR_IMPL;		/* Enable the entry */
	regmap_write(regmap, offset + SRC_ADDR_LO_OFFSET, val);
}

static int tc9564_translate_probe(struct platform_device *pdev)
{
	struct tc9564_translate *translate;
	struct device *dev = &pdev->dev;
	struct device_node *np;
	struct regmap *regmap;
	u32 min_size;
	u64 offset;
	u64 size;
	int ret;

	np = dev_of_node(dev);
	if (!np)
		return dev_err_probe(dev, -EINVAL, "no devicetree node\n");

	ret = of_property_read_reg(np, 0, &offset, &size);
	if (ret < 0)
		return dev_err_probe(dev, ret, "failed to get reg property\n");

	min_size = AXI4_TABLE_ENTRY_COUNT * AXI4_TABLE_STRIDE;
	if (size < min_size)
		return dev_err_probe(dev, -EINVAL,
				     "reg size too small (%llu < %u)\n",
				     size, min_size);

	regmap = syscon_node_to_regmap(dev->parent->of_node);
	if (IS_ERR(regmap))
		return dev_err_probe(dev, PTR_ERR(regmap),
				     "failed to get bridge regmap\n");

	translate = devm_kzalloc(dev, sizeof(*translate), GFP_KERNEL);
	if (!translate)
		return dev_err_probe(dev, -ENOMEM,
				     "failed to allocate translation data\n");

	translate->regmap = regmap;
	translate->offset = offset;

	dev_set_drvdata(dev, translate);

	/* Do the initial configuraiton */

	tc9564_translate_config(translate);

	return 0;
}

static int tc9564_translate_suspend_noirq(struct device *dev)
{
	return 0;
}

/* We need to reconfigure address translation when we resume */
static int tc9564_translate_resume_noirq(struct device *dev)
{
	struct tc9564_translate *translate = dev_get_drvdata(dev);

	tc9564_translate_config(translate);

	return 0;
}

static DEFINE_NOIRQ_DEV_PM_OPS(tc9564_translate_pm_ops,
			       tc9564_translate_suspend_noirq,
			       tc9564_translate_resume_noirq);

static const struct of_device_id tc9564_translate_match[] = {
	{ .compatible	= "toshiba,tc9564-translate", },
	{ },
};
MODULE_DEVICE_TABLE(of, tc9564_translate_match);

static struct platform_driver tc9564_translate_driver = {
	.probe		= tc9564_translate_probe,
	.driver		= {
		.name		= KBUILD_MODNAME,
		.of_match_table	= of_match_ptr(tc9564_translate_match),
		.pm		= pm_sleep_ptr(&tc9564_translate_pm_ops),
	},
};

module_platform_driver(tc9564_translate_driver);

MODULE_DESCRIPTION("Toshiba TC9564 Configuration Driver");
MODULE_LICENSE("GPL");
