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
#include <linux/irq.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/irqdesc.h>
#include <linux/irqdomain.h>
#include <linux/module.h>
#include <linux/pm.h>

#include "soc-tc9564-chip.h"

#define DRIVER_NAME		"dwmac-tc9564"

/* Bits in MSI_OUT_EN and MSI_INT_STS are defined by tc956x_msigen_hwirq */
#define MSI_OUT_EN_OFFSET               0x0000	/* 1: interrupt enabled */
/* Bits in MSI_MASK_CLR correspond to MSIs 0..31 */
#define MSI_MASK_CLR_OFFSET             0x000c	/* Write 1: disable MSI */
#define CLR_VECTOR_UNMASK		BIT(0)	/* MSIs unmasked if written */
#define MSI_INT_STS_OFFSET              0x0010

enum tc956x_msigen_hwirq {
	HWIRQ_LPI		= 0,	/* Per-function */
	HWIRQ_PMT		= 1,	/* Per-function */
	HWIRQ_EVENT		= 2,	/* Per-function */
	HWIRQ_TX0		= 3,	/* 4..10 are TX1..TX7 */
	HWIRQ_RX0		= 11,	/* 12..18 are RX1..RX7 */
	HWIRQ_XPCS		= 19,	/* Per-function */
	HWIRQ_ETH_INT		= 20,	/* PHY interrupt */
	HWIRQ_MAILBOX		= 21,	/* Per-function */
	HWIRQ_SOFTWARE_MSI	= 24,
	HWIRQ_COUNT		= 25,	/* Not an IRQ bit */
};

/*
 * struct tc9564_dwmac - Information related to an embedded XGMAC
 * @dev:		Device pointer
 * @data:		Pointer to data passed from the parent
 * @chip:		Handle used for common chip operations
 */
struct tc9564_dwmac {
	struct device *dev;
	void *chip;				/* Intentionally opaque */
	struct tc9564_dwmac_data *data;
};

static const struct auxiliary_device_id tc964_dwmac_ids[] = {
	{ .name = "misc_tc9564_chip." DRIVER_NAME, },
	{ }
};
MODULE_DEVICE_TABLE(auxiliary, tc964_dwmac_ids);

static void msigen_irq_handler(struct irq_desc *desc)
{
	struct irq_domain *domain = irq_desc_get_handler_data(desc);
	struct irq_chip *chip = irq_desc_get_chip(desc);
	struct irq_chip_generic *gc;
	u32 status;

	gc = irq_get_domain_generic_chip(domain, 0);

	chained_irq_enter(chip, desc);

	status = irq_reg_readl(gc, MSI_INT_STS_OFFSET);
	while (status) {
		u32 hwirq = __ffs(status);

		status ^= BIT(hwirq);
		generic_handle_domain_irq(domain, hwirq);
	}

	/* XXX Shouldn't we do this right after reading MSI_INT_STS? */
	irq_reg_writel(gc, CLR_VECTOR_UNMASK, MSI_MASK_CLR_OFFSET);

	chained_irq_exit(chip, desc);
}

static int msigen_chip_init(struct irq_chip_generic *gc)
{
	struct tc9564_dwmac *dwmac = gc->domain->host_data;

	gc->reg_base = dwmac->data->msigen_addr;
	gc->chip_types[0].regs.mask = MSI_OUT_EN_OFFSET;
	gc->chip_types[0].chip.irq_mask = irq_gc_mask_clr_bit;
	gc->chip_types[0].chip.irq_unmask = irq_gc_mask_set_bit;

	/* Disable all interrupts */
	irq_reg_writel(gc, 0, MSI_OUT_EN_OFFSET);

	return 0;
}

static void msigen_chip_exit(struct irq_chip_generic *gc)
{
	/* Disable all interrupts */
	irq_reg_writel(gc, 0, MSI_OUT_EN_OFFSET);
}

static int msigen_domain_init(struct irq_domain *domain)
{
	struct tc9564_dwmac *dwmac = domain->host_data;
	unsigned int irq = dwmac->data->msigen_irq;

	irq_set_chained_handler_and_data(irq, msigen_irq_handler, domain);

	return 0;
}

static void msigen_domain_exit(struct irq_domain *domain)
{
	struct tc9564_dwmac *dwmac = domain->host_data;
	unsigned int irq = dwmac->data->msigen_irq;

	irq_set_chained_handler_and_data(irq, NULL, NULL);
}

/* We have one IRQ chip instance, 25 IRQs per chip and in the domain */
static struct irq_domain *msigen_domain_instantiate(struct tc9564_dwmac *dwmac)
{
	struct irq_domain_chip_generic_info dgc_info;
	struct irq_domain_info info;

	dgc_info.name = "tc9564-msigen";
	dgc_info.handler= handle_level_irq;
	dgc_info.irqs_per_chip = HWIRQ_COUNT;
	dgc_info.num_ct = 1;
	dgc_info.init = msigen_chip_init;
	dgc_info.exit = msigen_chip_exit;

	info.domain_flags = IRQ_DOMAIN_FLAG_DESTROY_GC;
	info.size = HWIRQ_COUNT;
	info.hwirq_max = HWIRQ_COUNT;
	info.ops = &irq_generic_chip_ops;
	info.host_data = dwmac;
	info.dgc_info = &dgc_info;
	info.init = msigen_domain_init;
	info.exit = msigen_domain_exit;

	return devm_irq_domain_instantiate(dwmac->dev, &info);
}

static int tc9564_dwmac_probe(struct auxiliary_device *adev,
			      const struct auxiliary_device_id *id)
{
	struct device *dev = &adev->dev;
	struct tc9564_dwmac *dwmac;
	struct irq_domain *domain;

	dev_info(dev, " === %s\n", __func__);

	if (!dev->platform_data)
		return -EINVAL;

	dwmac = devm_kzalloc(dev, sizeof(*dwmac), GFP_KERNEL);
	if (!dwmac)
		return -ENOMEM;

	dwmac->dev = dev;
	dwmac->chip = dev_get_platdata(dev->parent);
	dwmac->data = dev_get_platdata(dev);

	domain = msigen_domain_instantiate(dwmac);
	if (IS_ERR(domain))
		return dev_err_probe(dev, PTR_ERR(domain),
				     "failed to instantiate MSIGEN domain\n");

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
