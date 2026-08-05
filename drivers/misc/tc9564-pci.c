// SPDX-License-Identifier: GPL-2.0

/*
 * Copyright (C) 2026 by RISCstar Solutions Corporation.  All rights reserved.
 */

/*
 * The Toshiba TC9564 implements a PCIe Gen 3 switch that connects an
 * upstream x4 port to three downstream PCIe ports.  Two of the downstream
 * ports are external, and the third is internal, implementing a PCIe
 * endpoint which implements two PCIe functions.  Each PCIe function drives
 * a Synopsys XGMAC Ethernet interface capable of 10 Gbps operation.
 *
 * The TC9564 implements other functionality, including an embedded MCU,
 * a UART, a GPIO controller, a reset controller, a clock controller, and
 * interrupt handling.  These features are separate from (and in some
 * cases used by) both Ethernet XGMACs.  Each Ethernet MAC must be
 * attached to a working PHY for it to be functional, and for this
 * reason either of them (or both!) might not be usable/used.
 *
 * An address translation table within the TC9564 converts internal AXI
 * bus receive requests into operations on PCIe space.  There are four
 * "slots" that map address ranges from source to translated addresses.
 * The host uses the first slot, and because there are currently no other
 * users, we disable the others.  This must be programmed initially, and
 * during resume.
 *
 * This PCI driver binds to the Toshiba TC9564 (physical) PCI function
 * (VID 0x1179, DID 0x0220).
 */

#include <linux/bitfield.h>
#include <linux/device.h>
#include <linux/io.h>
#include <linux/irqdomain.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/pci.h>
#include <linux/wordpart.h>

struct tc9564_function {
	void __iomem *ioaddr;		/* Mapped BAR 0 */
};

/*
 * The bus translation function has four AXI translation table entries
 * each with eight 4-byte registers.  These entries translate between
 * an internal AXI bus address space and "external" PCIe address space.
 * The Ethernet MACs access the PCIe subsystem via this bus.  Currently
 * we only use the first translation table entry.
 */
#define AXI4_ENTRY_BASE(id)		((id) * AXI4_TABLE_STRIDE)
#define AXI4_TABLE_ENTRY_COUNT		4
#define AXI4_TABLE_STRIDE		0x20

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

/**
 * tc9564_function_translate_config() - Configure the translation unit
 * @pdev:	PCI device pointer
 *
 * Define the translation between internal AXI bus accesses and PCI TLPs
 * using the first translation table entry.  TC9564_SLV00_SRC_ADDR defines
 * the base address of the AXI address range.  AXI addresses are translated
 * to the PCIe address range, whose base address we set to be 0x0.
 *
 * By convention, the host uses table slot 0.  Currently nothing else needs
 * translation, so we disable all other slots.  (This might have to change
 * if firmware ever needs to use a slot, for example.)
 *
 * Note that we allow both functions to program the table.  The first one
 * to do so updates the values.  The second one will re-write exactly the
 * same values as the first, so it's harmless.  (This is much simpler than
 * tracking reference counts.)
 */
static int tc9564_function_translate_config(struct pci_dev *pdev)
{
	struct tc9564_function *function = pci_get_drvdata(pdev);
	void __iomem *ioaddr = function->ioaddr;
	u32 val;

	/* Disable all entries initially */
	for (u32 i = 0; i < AXI4_TABLE_ENTRY_COUNT; i++)
		writel(0, ioaddr + AXI4_ENTRY_BASE(i) + SRC_ADDR_LO_OFFSET);

	/* We only use table entry 0 */
	ioaddr += AXI4_ENTRY_BASE(0);

	/* TC9564 only allows 0 to be written to this register */
	writel(0, ioaddr + TRSL_PARAM_OFFSET);

	/* The translated base address is always just 0x0 */
	writel(0, ioaddr + TRSL_ADDR_HI_OFFSET);
	writel(0, ioaddr + TRSL_ADDR_LO_OFFSET);

	/* Encode the source base address and range size */
	val = upper_32_bits(TC9564_SLV00_SRC_ADDR);
	writel(val, ioaddr + SRC_ADDR_HI_OFFSET);

	val = lower_32_bits(TC9564_SLV00_SRC_ADDR);
	/* ATR_SIZE field defines the range size using range_bits-1 */
	val |= u32_encode_bits(TRANSLATE_RANGE_BITS - 1, ATR_SIZE_MASK);
	val |= ATR_IMPL;		/* Enable the entry */
	writel(val, ioaddr + SRC_ADDR_LO_OFFSET);

	return 0;
}

static void tc9564_function_remove(struct pci_dev *pdev)
{
	of_platform_depopulate(&pdev->dev);
	pci_clear_master(pdev);
}

static int
tc9564_function_probe(struct pci_dev *pdev, const struct pci_device_id *id)
{
	struct tc9564_function *function;
	struct device *dev = &pdev->dev;
	struct device_node *np;
	int ret;

	/* Despite being a PCI device, we require devicetree */
	np = dev_of_node(dev);
	if (!np)
		return dev_err_probe(dev, -EINVAL, "no devicetree node\n");

	ret = pcim_enable_device(pdev);
	if (ret)
		return dev_err_probe(dev, ret, "failed to enable device\n");

	function = devm_kzalloc_obj(dev, *function);
	if (!function)
		return -ENOMEM;

	/* Map BAR 0 for access to the internal address translation table */
	function->ioaddr = pcim_iomap_region(pdev, 0, pci_name(pdev));
	if (IS_ERR(function->ioaddr))
		return dev_err_probe(dev, PTR_ERR(function->ioaddr),
				     "failed to map BAR 0\n");

	pci_set_drvdata(pdev, function);

	/* Do the initial translation table configuration */
	(void)tc9564_function_translate_config(pdev);

	pci_set_master(pdev);

	/* Scan for pci-ep-bus (simple-bus) nodes and probe their sub-devices */
	ret = of_platform_default_populate(np, NULL, dev);
	if (ret)
		goto err_depopulate;

	return 0;

err_depopulate:
	tc9564_function_remove(pdev);

	return dev_err_probe(dev, ret, "failed to populate platform bus\n");
}

static const struct pci_device_id tc9564_function_id_table[] = {
	{ PCI_DEVICE(PCI_VENDOR_ID_TOSHIBA, 0x0220), },
	{ },
};
MODULE_DEVICE_TABLE(pci, tc9564_function_id_table);

static int tc9564_resume_noirq(struct device *dev)
{
	return tc9564_function_translate_config(to_pci_dev(dev));
}
static DEFINE_SIMPLE_DEV_PM_OPS(tc9564_pm, NULL, tc9564_resume_noirq);

static struct pci_driver tc9564_function_driver = {
	.name		= KBUILD_MODNAME,
	.id_table	= tc9564_function_id_table,
	.probe		= tc9564_function_probe,
	.remove		= tc9564_function_remove,
	.driver.pm	= pm_sleep_ptr(&tc9564_pm),
};
module_pci_driver(tc9564_function_driver);

MODULE_DESCRIPTION("Toshiba TC9564 PCIe Embedded Function Driver");
MODULE_LICENSE("GPL");
