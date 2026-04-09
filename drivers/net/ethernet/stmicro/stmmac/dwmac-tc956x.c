// SPDX-License-Identifier: GPL-2.0

/*
 * Copyright (C) 2026 by RISCstar Solutions Corporation.  All rights reserved.
 *
 * Derived from code having the following copyrights:
 * Copyright (C) 2011-2012  Vayavya Labs Pvt Ltd
 * Copyright (C) 2025 Toshiba Electronic Devices & Storage Corporation
 */

#include <linux/auxiliary_bus.h>
#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/iopoll.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/of_irq.h>
#include <linux/pcs/pcs-xpcs-regmap.h>
#include <linux/pcs/pcs-xpcs.h>
#include <linux/phy.h>
#include <linux/pinctrl/consumer.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/stmmac.h>
#include <linux/types.h>

#include "common.h"
#include "dwxgmac2.h"
#include "stmmac.h"

#include "soc-tc956x-chip.h"

#define DRIVER_NAME		"dwmac-tc956x"

/* Address translation space size */
#define SLV00_SRC_ADDR			0x0000001000000000ULL

#define CM3_TAMAP_COUNT			4

#define TC956X_RX_QUEUE_COUNT		8	/* Supported by hardware */
#define TC956X_RX_FIFO_KB		46	/* Shared by all RX queues */

#define TC956X_TX_QUEUE_COUNT		8	/* Supported by hardware */
#define TC956X_TX_FIFO_KB		46	/* Shared by all TX queues */

/* EMAC control registers for ports 0 and 1 (both have same format) */
#define NEMAC0CTL_OFFSET		0x1070
#define NEMAC1CTL_OFFSET		0x1074

/* Fields and values for the NEMACxCTL registers */
#define EMAC_SP_SEL_MASK		GENMASK(3, 0)
#define SP_SEL_SGMII_2500M		4
#define SP_SEL_SGMII_1000M		5
#define SP_SEL_SGMII_100M		6
#define SP_SEL_SGMII_10M		7
#define SP_SEL_USXGMII_10G_10G		8
#define EMAC_PHY_INF_SEL_MASK		GENMASK(5, 4)
#define PCS_CLK_PLL			0	/* Clock from internal PLL */
#define PCS_CLK_PHY			1	/* Clock from PHY */
#define EMAC_INV_SGM_SIG_DET		BIT(6)	/* 1 = polarity inverted */
#define EMAC_LPIHWCLKEN			BIT(8)	/* 1 = low power mode */
#define EMAC_INIT_DONE			BIT(21)

/* MSIGEN Registers */
#define TC956X_MSIGEN_BASE(pf_id)	(0x00f000 + (pf_id) * 0x0100)
#define MSI_OUT_EN_OFFSET		0x0000
#define MSI_MASK_SET_OFFSET		0x0008
#define MSI_MASK_CLR_OFFSET		0x000c
#define MSI_INT_STS_OFFSET		0x0010
#define MSI_VECT_SET_OFFSET(x)		(0x0020 + (x) * 4)
#define SW_MSI_CLR			0x0054

enum tc956x_msigen_hwirq {
	HWIRQ_LPI		= 0,
	HWIRQ_PMT		= 1,
	HWIRQ_EVENT		= 2,
	HWIRQ_TX0		= 3,
	HWIRQ_RX0		= 11,
	HWIRQ_XPCS		= 19,
	HWIRQ_ETH		= 20, /* PHY interrupt */
	HWIRQ_PFMAILBOX		= 21,
	HWIRQ_MSIREQ_PLS	= 24
};
#define TC956X_NR_HWIRQ		25

#define XGMAC_BASE(td) \
		((td)->data->sfr + ((td)->data->mac_id ? 0x48000 : 0x40000))

/* The next two are relative to XGMAC_BASE() */
#define XPCS_XGMAC_OFFSET			0x3a00
#define PMA_XGMAC_OFFSET			0x4000

/* All PMA registers are relative to PMA_XGMAC_OFFSET */
#define PMA_CML_GL_PM_CFG0			0x01b8
#define PMA_COMM_CFG_0_1_R0			0x1888
#define PMA_COMM_CFG_0_1_R1			0x1890
#define PMA_COMM_CFG_0_1_R2			0x1898
#define PMA_COMM_CFG_0_1_R3			0x18a0
#define PMA_COMM_CFG_0_1_R4			0x18a8

/* This is a mask for a field called write_mask */
#define COMM_CFG_WRITE_MASK_MASK		GENMASK(16, 9)
#define COMM_CFG_ENABLE				BIT(8)
#define COMM_CFG_WRITE_DATA_MASK		GENMASK(7, 0)

#define PMA_HWT_REFCK_R_EN_R0			0x1080
#define PMA_HWT_REFCK_TERM_EN_R0		0x1090
#define PMA_HWT_REFCK_R_EN_R1			0x1094
#define PMA_HWT_REFCK_TERM_EN_R1		0x10a4
#define PMA_HWT_REFCK_R_EN_R2			0x10a8
#define PMA_HWT_REFCK_TERM_EN_R2		0x10b8
#define PMA_HWT_REFCK_R_EN_R3			0x10bc
#define PMA_HWT_REFCK_TERM_EN_R3		0x10cc
#define PMA_HWT_REFCK_R_EN_R4			0x10d0
#define PMA_HWT_REFCK_TERM_EN_R4		0x10e0

/**
 * struct tc956x_data - Toshiba-specific platform data
 * @dev:		Device pointer
 * @data:		Pointer to data passed from the parent device
 * @plat:		Pointer to our stmmac platform data
 * @bridge_config:	Mapped bridge config data (BAR 0)
 * @phy_supply:		PHY supply regulator
 * @phy_reset:		Descriptor for GPIO used for PHY reset
 * @phy_reset_delay:	Delay (milliseconds) after PHY reset
 * @wol_irq:		Wake-on-LAN IRQ number
 * @chip:		Pointer to the containing chip information
 * @dma_cfg:		DMA config buffer used by plat_stmmacenet_data
 * @mdio_bus_data:	MDIO bus data used by plat_stmmacenet_data
 * @axi:		AXI data used by plat_stmmacenet_data
 * @desc:		DMA descriptor data used by mac_device_info
 * @dma:		DMA operations data used by mac_device_info
 */
struct tc956x_data {
	struct device *dev;
	struct tc956x_dwmac_data *data;
	struct plat_stmmacenet_data *plat;
	void __iomem *bridge_config;
	struct regulator *phy_supply;
	struct gpio_desc *phy_reset;
	u32 phy_reset_delay;
	int wol_irq;
	struct tc956x_chip *chip;

	/* These three fields are used by the plat_stmmacenet_data structure */
	struct stmmac_dma_cfg dma_cfg;
	struct stmmac_mdio_bus_data mdio_bus_data;
	struct stmmac_axi axi;
	/* This field is used by the mac_device_info structure */
	struct stmmac_desc_ops desc;
	struct stmmac_dma_ops dma;
};

struct tc956x_mac_speed {
	phy_interface_t phy_interface;
	int speed;
	u32 sp_sel;
};

static struct tc956x_mac_speed tc956x_chipcfg_mac_speed[] = {
	{ PHY_INTERFACE_MODE_10GBASER,	SPEED_10000, SP_SEL_USXGMII_10G_10G, },
	{ PHY_INTERFACE_MODE_SGMII,	SPEED_2500,  SP_SEL_SGMII_2500M, },
	{ PHY_INTERFACE_MODE_2500BASEX,	SPEED_2500,  SP_SEL_SGMII_2500M, },
	{ PHY_INTERFACE_MODE_SGMII,	SPEED_1000,  SP_SEL_SGMII_1000M, },
	{ PHY_INTERFACE_MODE_SGMII,	SPEED_100,   SP_SEL_SGMII_100M, },
	{ PHY_INTERFACE_MODE_SGMII,	SPEED_10,    SP_SEL_SGMII_10M, },
};

struct tc956x_msigen_data {
	void __iomem *regs;
	int irq;
};

static void tc956x_msigen_irq_handler(struct irq_desc *desc)
{
	struct irq_domain *d = irq_desc_get_handler_data(desc);
	struct irq_chip_generic *gc = irq_get_domain_generic_chip(d, 0);
	struct irq_chip *chip = irq_desc_get_chip(desc);
	unsigned long sts;
	unsigned int hwirq;

	chained_irq_enter(chip, desc);

	sts = irq_reg_readl(gc, MSI_INT_STS_OFFSET);
	if (sts)
		for_each_set_bit(hwirq, &sts, 32)
			generic_handle_domain_irq(d, hwirq);

	/*
	 * Clear the MSI flag. All interrupts within TC956X are level-high type.
	 * If any interrupts are still asserted then clearing this flag will
	 * cause the (edge-triggered) MSI to be regenerated.
	 */
	irq_reg_writel(gc, BIT(0), MSI_MASK_CLR_OFFSET);

	chained_irq_exit(chip, desc);
}

static int tc956x_msigen_chip_init(struct irq_chip_generic *gc)
{
	struct tc956x_msigen_data *tc956x_msigen = gc->domain->host_data;

	gc->reg_base = tc956x_msigen->regs;
	gc->chip_types[0].regs.mask = MSI_OUT_EN_OFFSET;
	gc->chip_types[0].chip.irq_mask = irq_gc_mask_clr_bit;
	gc->chip_types[0].chip.irq_unmask = irq_gc_mask_set_bit;

	/* Disable all interrupts */
	irq_reg_writel(gc, 0, MSI_OUT_EN_OFFSET);

	return 0;
}

static void tc956x_msigen_chip_exit(struct irq_chip_generic *gc)
{
	irq_reg_writel(gc, 0, MSI_OUT_EN_OFFSET);
}

static int tc956x_msigen_domain_init(struct irq_domain *d)
{
	struct tc956x_msigen_data *tc956x_msigen = d->host_data;

	irq_set_chained_handler_and_data(tc956x_msigen->irq,
					 tc956x_msigen_irq_handler, d);

	return 0;
}

static void tc956x_msigen_domain_exit(struct irq_domain *d)
{
	struct tc956x_msigen_data *tc956x_msigen = d->host_data;

	irq_set_chained_handler_and_data(tc956x_msigen->irq, NULL, NULL);
}

static struct irq_domain *devm_tc956x_msigen_register(struct tc956x_data *td)
{
	struct irq_domain_chip_generic_info dgc_info = {
		.name		= "tc956x-msigen",
		.handler	= handle_level_irq,
		.irqs_per_chip	= TC956X_NR_HWIRQ,
		.num_ct		= 1,
		.init		= tc956x_msigen_chip_init,
		.exit		= tc956x_msigen_chip_exit,
	};
	struct irq_domain_info d_info = {
		.domain_flags	= IRQ_DOMAIN_FLAG_DESTROY_GC,
		.size		= TC956X_NR_HWIRQ,
		.hwirq_max	= TC956X_NR_HWIRQ,
		.ops		= &irq_generic_chip_ops,
		.dgc_info	= &dgc_info,
		.init		= tc956x_msigen_domain_init,
		.exit		= tc956x_msigen_domain_exit,
	};
	struct tc956x_msigen_data *tc956x_msigen;
	struct device *dev = td->dev;
	struct irq_domain *domain;

	tc956x_msigen = devm_kmalloc(dev, sizeof(*tc956x_msigen), GFP_KERNEL);
	if (!tc956x_msigen)
		return ERR_PTR(-ENOMEM);

	tc956x_msigen->regs = td->data->msigen_addr;
	tc956x_msigen->irq = td->data->msigen_irq;
	d_info.host_data = tc956x_msigen;

	domain = devm_irq_domain_instantiate(dev, &d_info);
	if (IS_ERR(domain))
		return dev_err_cast_probe(
			dev, domain, "failed to instantiate the IRQ domain\n");
	return domain;
}

static int tc956x_phy_power_on(struct tc956x_data *td)
{
	int ret;

	ret = regulator_enable(td->phy_supply);
	if (ret)
		dev_warn(td->dev, "Failed to enable PHY supply with error %d\n", ret);

	(void)gpiod_set_value(td->phy_reset, 1);
	fsleep(td->phy_reset_delay);

	return ret;
}

static int tc956x_phy_power_off(struct tc956x_data *td)
{
	int ret = 0;

	/* make this function safe to call unconditionally from error paths */
	if (!td->phy_reset)
		return 0;

	ret = gpiod_set_value(td->phy_reset, 0);
	if (ret)
		return ret;

	return regulator_disable(td->phy_supply);
}

/**
 * tc956x_mac_pma_init() - Initialize PMA
 * @td:	bsp_priv pointer
 *
 * Initialize (or re-initialize) the PMA, configure the clocks and wait for the
 * MAC to be ready.
 */
static void tc956x_mac_pma_init(struct tc956x_data *td)
{
	void __iomem *pma_base = XGMAC_BASE(td) + PMA_XGMAC_OFFSET;
	void __iomem *emac_ctl_reg;
	u32 id = td->data->mac_id;
	u32 val;

	/*
	 * When we re-initialize the PMA then the reset will already have
	 * been deasserted. We must make sure the PMA reset is asserted before
	 * we change the clock settings.
	 */
	tc956x_mac_reset_assert(td->chip, id, MAC_RESET_PMA);

	/* Power on CML buffer (0 = normal mode, 1 = power down) */
	writel(0, pma_base + PMA_CML_GL_PM_CFG0);

	/* XXX Any documentation on what these values (4 in particular) do? */
	/* Switch clock from C0_REFCK to CLK_REF_I */
	val = u32_encode_bits(0xf7, COMM_CFG_WRITE_MASK_MASK);
	val |= COMM_CFG_ENABLE;
	val |= u32_encode_bits(4, COMM_CFG_WRITE_DATA_MASK);
	writel(val, pma_base + PMA_COMM_CFG_0_1_R0);
	writel(val, pma_base + PMA_COMM_CFG_0_1_R1);
	writel(val, pma_base + PMA_COMM_CFG_0_1_R2);
	writel(val, pma_base + PMA_COMM_CFG_0_1_R3);
	writel(val, pma_base + PMA_COMM_CFG_0_1_R4);

	/* Disable C0_REFCK and 100 ohm termination */
	writel(0, pma_base + PMA_HWT_REFCK_R_EN_R0);
	writel(0, pma_base + PMA_HWT_REFCK_TERM_EN_R0);
	writel(0, pma_base + PMA_HWT_REFCK_R_EN_R1);
	writel(0, pma_base + PMA_HWT_REFCK_TERM_EN_R1);
	writel(0, pma_base + PMA_HWT_REFCK_R_EN_R2);
	writel(0, pma_base + PMA_HWT_REFCK_TERM_EN_R2);
	writel(0, pma_base + PMA_HWT_REFCK_R_EN_R3);
	writel(0, pma_base + PMA_HWT_REFCK_TERM_EN_R3);
	writel(0, pma_base + PMA_HWT_REFCK_R_EN_R4);
	writel(0, pma_base + PMA_HWT_REFCK_TERM_EN_R4);

	tc956x_mac_reset_deassert(td->chip, id, MAC_RESET_PMA);

	emac_ctl_reg = td->data->sfr + (id ? NEMAC1CTL_OFFSET
					   : NEMAC0CTL_OFFSET);

	WARN_ON(readl_poll_timeout(emac_ctl_reg, val, val & EMAC_INIT_DONE, 50, 1000000));
}

static int tc956x_mac_speed_select(struct tc956x_data *td, int speed)
{
	phy_interface_t phy_interface = td->plat->phy_interface;
	int i;

	for (i = 0; i < ARRAY_SIZE(tc956x_chipcfg_mac_speed); i++) {
		if (tc956x_chipcfg_mac_speed[i].speed != speed)
			continue;

		if (tc956x_chipcfg_mac_speed[i].phy_interface == phy_interface)
			return tc956x_chipcfg_mac_speed[i].sp_sel;
	}
	dev_warn(td->dev, "%s/%d unsupported\n",
		 phy_modes(phy_interface), speed);

	return -ENOTSUPP;
}

static int tc956x_chipcfg_mac_configure(struct tc956x_data *td, int speed)
{
	void __iomem *emac_ctl_reg;
	u32 id = td->data->mac_id;
	int sp_sel;
	u32 val;

	sp_sel = tc956x_mac_speed_select(td, speed);
	if (sp_sel < 0)
		return sp_sel;

	/* Speeds up to 1Gbps require the 125 MHz clock to be enabled */
	if (speed < SPEED_2500)
		tc956x_mac_clock_enable(td->chip, id, MAC_CLOCK_125M);
	else
		tc956x_mac_clock_disable(td->chip, id, MAC_CLOCK_125M);

	emac_ctl_reg = td->data->sfr + (id ? NEMAC1CTL_OFFSET
					   : NEMAC0CTL_OFFSET);
	val = readl(emac_ctl_reg);
	val |= EMAC_LPIHWCLKEN;
	val &= ~EMAC_INV_SGM_SIG_DET;
	val = u32_replace_bits(val, PCS_CLK_PHY, EMAC_PHY_INF_SEL_MASK);
	val = u32_replace_bits(val, sp_sel, EMAC_SP_SEL_MASK);
	writel(val, emac_ctl_reg);

	return 0;
}

static int tc956x_chipcfg_mac_init(struct tc956x_data *td)
{
	struct plat_stmmacenet_data *plat = td->plat;
	u32 id = td->data->mac_id;
	int ret;

	tc956x_mac_clock_enable(td->chip, id, MAC_CLOCK_TX);
	tc956x_mac_clock_enable(td->chip, id, MAC_CLOCK_RX);
	tc956x_mac_clock_enable(td->chip, id, MAC_CLOCK_ALL);
	if (id)
		tc956x_mac_clock_enable(td->chip, id, MAC_CLOCK_RMII);

	/* Set the speed related registers */
	ret = tc956x_chipcfg_mac_configure(td, plat->max_speed);
	if (ret)
		return ret;

	tc956x_mac_reset_deassert(td->chip, id, MAC_RESET_MAC);

	tc956x_mac_pma_init(td);

	tc956x_mac_reset_deassert(td->chip, id, MAC_RESET_XPCS);

	return 0;
}

static void tc956x_stop_mac(struct tc956x_data *td)
{
	u32 id = td->data->mac_id;

	tc956x_mac_reset_assert(td->chip, id, MAC_RESET_MAC);
	tc956x_mac_reset_assert(td->chip, id, MAC_RESET_PMA);
	tc956x_mac_reset_assert(td->chip, id, MAC_RESET_XPCS);

	tc956x_mac_clock_disable(td->chip, id, MAC_CLOCK_ALL);
	tc956x_mac_clock_disable(td->chip, id, MAC_CLOCK_RX);
	tc956x_mac_clock_disable(td->chip, id, MAC_CLOCK_TX);
	tc956x_mac_clock_disable(td->chip, id, MAC_CLOCK_125M);
	if (id)
		tc956x_mac_clock_disable(td->chip, id, MAC_CLOCK_RMII);
}

static void tc956x_mac_init_state(struct tc956x_data *td)
{
	tc956x_mac_clock_disable(td->chip, td->data->mac_id, MAC_CLOCK_312_5M);

	tc956x_stop_mac(td);
}

/* Extra fields for XGMAC_DMA_MODE */
/* Descriptor posted write */
#define XGMAC_DSPW			BIT(8)	/* 1: All Rx DMA posted */
/* Interrupt mode */
#define XGMAC_DMA_MODE_INTM		GENMASK(13, 12)
#define DMA_MODE_INTM_LEVEL_ONCE	1

#define XGMAC_DMA_CH_RX_CONTROL2(_x)	(0x00003134 + (0x80 * (_x)))
#define XGMAC_OWRQ			GENMASK(25, 24)

static void tc956x_dma_init(void __iomem *ioaddr,
			    struct stmmac_dma_cfg *dma_cfg)
{
	u32 value;

	/*
	 * Set the DMA completion interrupt-mode (INTM) to level signals without
	 * automatic re-assertion on new events.
	 *
	 * Ensure descriptor posted write (DSPW) is disabled. This is needed for
	 * XGMAC 3.01a errata.
	 */
	value = readl(ioaddr + XGMAC_DMA_MODE);
	value = u32_replace_bits(value, DMA_MODE_INTM_LEVEL_ONCE,
				 XGMAC_DMA_MODE_INTM);
	value &= ~XGMAC_DSPW;
	writel(value, ioaddr + XGMAC_DMA_MODE);

	value = readl(ioaddr + XGMAC_DMA_SYSBUS_MODE);

	if (dma_cfg->aal)
		value |= XGMAC_AAL;

	if (dma_cfg->eame)
		value |= XGMAC_EAME;

	writel(value, ioaddr + XGMAC_DMA_SYSBUS_MODE);
}

static void tc956x_dma_init_rx_chan(struct stmmac_priv *priv,
				    void __iomem *ioaddr,
				    struct stmmac_dma_cfg *dma_cfg,
				    dma_addr_t phy, u32 chan)
{
	struct tc956x_data *td = priv->plat->bsp_priv;
	u32 setting;
	u32 value;

	/* RX programmable burst length */
	setting = dma_cfg->rxpbl ? : dma_cfg->pbl;
	value = readl(ioaddr + XGMAC_DMA_CH_RX_CONTROL(chan));
	value = u32_replace_bits(value, setting, XGMAC_RxPBL);
	writel(value, ioaddr + XGMAC_DMA_CH_RX_CONTROL(chan));

	/*
	 * Reduce the number of outstanding write requests to 3.  Needed
	 * for XGMAC 3.01a errata (value 0 means 4 outstanding writes).
	 */
	setting = td->data->rev_id == 1 ? 3 : 0;
	value = readl(ioaddr + XGMAC_DMA_CH_RX_CONTROL2(chan));
	value = u32_replace_bits(value, setting, XGMAC_OWRQ);
	writel(value, ioaddr + XGMAC_DMA_CH_RX_CONTROL2(chan));

	/* Set BIT(36) in the physical address */
	writel(upper_32_bits(phy) | upper_32_bits(SLV00_SRC_ADDR),
	       ioaddr + XGMAC_DMA_CH_RxDESC_HADDR(chan));
	writel(lower_32_bits(phy), ioaddr + XGMAC_DMA_CH_RxDESC_LADDR(chan));
}

static void tc956x_dma_init_tx_chan(struct stmmac_priv *priv,
				    void __iomem *ioaddr,
				    struct stmmac_dma_cfg *dma_cfg,
				    dma_addr_t phy, u32 chan)
{
	u32 txpbl = dma_cfg->txpbl ? : dma_cfg->pbl;
	u32 value;

	value = readl(ioaddr + XGMAC_DMA_CH_TX_CONTROL(chan));
	value = u32_replace_bits(value, txpbl, XGMAC_TxPBL);
	writel(value, ioaddr + XGMAC_DMA_CH_TX_CONTROL(chan));

	/* Set BIT(36) in the physical address */
	writel(upper_32_bits(phy) | upper_32_bits(SLV00_SRC_ADDR),
	       ioaddr + XGMAC_DMA_CH_TxDESC_HADDR(chan));
	writel(lower_32_bits(phy), ioaddr + XGMAC_DMA_CH_TxDESC_LADDR(chan));
}

static void tc956x_desc_set_addr(struct dma_desc *p, dma_addr_t addr)
{
	p->des0 = cpu_to_le32(lower_32_bits(addr));
	/* Set BIT(36) in the physical address */
	p->des1 = cpu_to_le32(upper_32_bits(addr) |
			      upper_32_bits(SLV00_SRC_ADDR));
}

static void tc956x_desc_set_sec_addr(struct dma_desc *p, dma_addr_t addr, bool is_valid)
{
	p->des2 = cpu_to_le32(lower_32_bits(addr));
	/* Set BIT(36) in the physical address */
	p->des3 = cpu_to_le32(upper_32_bits(addr) |
			      upper_32_bits(SLV00_SRC_ADDR));
}

static int tc956x_mac_setup(void *apriv, struct mac_device_info *mac)
{
	struct stmmac_priv *priv = apriv;
	struct stmmac_desc_ops *desc;
	struct stmmac_dma_ops *dma;
	struct tc956x_data *td;

	td = priv->plat->bsp_priv;

	/* We can mostly use dwxgmac210_dma_ops, overwriting just a few */
	dma = &td->dma;
	*dma = dwxgmac210_dma_ops;
	dma->init = tc956x_dma_init;
	dma->init_rx_chan = tc956x_dma_init_rx_chan;
	dma->init_tx_chan = tc956x_dma_init_tx_chan;
	mac->dma = dma;

	/* dwxgmac210_desc_ops has most of what we want */
	desc = &td->desc;
	*desc = dwxgmac210_desc_ops;
	desc->set_addr = tc956x_desc_set_addr;
	desc->set_sec_addr = tc956x_desc_set_sec_addr;
	mac->desc = desc;

	priv->hw = mac;

	return dwxgmac2_setup(priv);
}

static int tc956x_pcs_init(struct stmmac_priv *priv)
{
	struct tc956x_data *td = priv->plat->bsp_priv;
	struct xpcs_regmap_config xpcs_regmap_cfg = {
		.reg_indir = true,
	};
	struct regmap_config regmap_cfg = {
		.reg_bits = 32,
		.val_bits = 32,
		.reg_shift = REGMAP_UPSHIFT(2),
	};
	struct dw_xpcs *xpcs;

	xpcs_regmap_cfg.regmap = devm_regmap_init_mmio(
		priv->device, XGMAC_BASE(td) + XPCS_XGMAC_OFFSET, &regmap_cfg);
	if (IS_ERR(xpcs_regmap_cfg.regmap))
		return PTR_ERR(xpcs_regmap_cfg.regmap);

	xpcs = devm_xpcs_regmap_register(priv->device, &xpcs_regmap_cfg);
	if (IS_ERR(xpcs))
		return PTR_ERR(xpcs);

	xpcs_config_eee_mult_fact(xpcs, priv->plat->mult_fact_100ns);
	priv->hw->phylink_pcs = xpcs_to_phylink_pcs(xpcs);

	return 0;
}

static struct phylink_pcs *tc956x_select_pcs(struct stmmac_priv *priv,
					     phy_interface_t interface)
{
	return priv->hw->phylink_pcs;
}

static void tc956x_fix_mac_speed(void *bsp_priv, int speed, unsigned int mode)
{
	struct tc956x_data *td = bsp_priv;

	/*
	 * XXX Q: Are we certain we need to provide this callback?
	 *     A: Yes.
	 */
	(void)tc956x_chipcfg_mac_configure(td, speed);
	tc956x_mac_pma_init(td);
}

static int tc956x_xgmac3_suspend(struct device *dev, void *bsp_priv)
{
	struct net_device *ndev = dev_get_drvdata(dev);
	struct stmmac_priv *priv = netdev_priv(ndev);
	struct tc956x_data *td = bsp_priv;
	int ret;

	if (priv->wolopts) {
		ret = enable_irq_wake(priv->wol_irq);
		if (unlikely(ret))
			dev_warn(priv->device, "Failed to set WOL IRQ %d as wake up capable with error %d\n",
				priv->wol_irq, ret);
	} else {
		ret = tc956x_phy_power_off(td);
		if (ret)
			dev_warn(priv->device, "Failed to power off PHY with error %d\n", ret);
	}

	tc956x_stop_mac(td);

	return 0;
}

static int tc956x_xgmac3_resume(struct device *dev, void *bsp_priv)
{
	struct net_device *ndev = dev_get_drvdata(dev);
	struct stmmac_priv *priv = netdev_priv(ndev);
	struct tc956x_data *td = priv->plat->bsp_priv;
	int ret;

	if (priv->wolopts) {
		ret = disable_irq_wake(priv->wol_irq);
		if (ret)
			dev_warn(priv->device, "Failed to set WOL IRQ %d as a wake-disabled irq with error %d\n",
				priv->wol_irq, ret);
	} else {
		ret = tc956x_phy_power_on(td);
		if (ret) {
			/* Let's hope this error is symmetrical with a failure to turn the PHY off! */
			dev_warn(priv->device, "Failed to power on the PHY with error %d\n", ret);
		}
	}

	return tc956x_chipcfg_mac_init(td);
}

/* Called by tc956x_dwmac_probe(); return errors with dev_err_probe() */
static int devicetree_init(struct tc956x_data *td)
{
	struct device *dev = td->dev;
	struct regulator *regulator;
	struct device_node *np;
	struct gpio_desc *gpio;
	u32 delay;
	int ret;

	regulator = devm_regulator_get(dev, "phy");
	if (IS_ERR(regulator))
		return dev_err_probe(dev, PTR_ERR(regulator),
				     "failed to get phy-supply\n");

	gpio = devm_gpiod_get(dev, "phy-reset", GPIOD_OUT_LOW);
	if (IS_ERR(gpio))
		return dev_err_probe(dev, PTR_ERR(gpio),
				     "failed to get phy-reset\n");

	np = dev_of_node(dev);
	ret = of_property_read_u32(np, "qcom,phy-reset-delay", &delay);
	if (ret)
		return dev_err_probe(dev, ret,
				      "failed to get qcom,phy-reset-delay property\n");

	ret = of_irq_get_byname(np, "wake-on-lan");
	if (ret <= 0)
		return dev_err_probe(dev, ret ? : -EINVAL,
				     "failed to get wake-on-lan property\n");

	td->phy_supply = regulator;
	td->phy_reset = gpio;
	td->phy_reset_delay = delay;
	td->wol_irq = ret;

	return 0;
}

/* XXX This should be populated further, and represented symbolically */
static int plat_clk_csr_value(phy_interface_t phy_interface)
{
	switch (phy_interface) {
	case PHY_INTERFACE_MODE_SGMII:
		return 0;		/* STMMAC_CSR_60_100M? */

	case PHY_INTERFACE_MODE_10GBASER:	/* XXX set CRS bit? */
		return 4;		/* STMMAC_CSR_150_250M? */

	default:
		return -EINVAL;
	}
}

static int plat_speed(phy_interface_t phy_interface)
{
	switch (phy_interface) {
	case PHY_INTERFACE_MODE_10GBASER:
		return SPEED_10000;

	case PHY_INTERFACE_MODE_SGMII:
	case PHY_INTERFACE_MODE_2500BASEX:
		return SPEED_2500;

	default:
		return -ENOTSUPP;
	}
}

static int plat_stmmacenet_data_init(struct tc956x_data *td)
{
	struct plat_stmmacenet_data *plat;
	phy_interface_t phy_interface;
	struct device *dev = td->dev;
	struct stmmac_axi *axi;
	u32 filter_size_kb;
	u32 clk_csr;
	u32 speed;
	int ret;
	u32 i;

	/* The platform structure is allocated with devm_kzalloc() */
	plat = stmmac_plat_dat_alloc(dev);
	if (!plat)
		return -ENOMEM;

	ret = device_get_phy_mode(dev);
	if (ret < 0)
		return ret;
	phy_interface = ret;

	ret = plat_clk_csr_value(phy_interface);
	if (ret < 0)
		return ret;
	clk_csr = ret;

	ret = plat_speed(phy_interface);
	if (ret < 0)
		return ret;
	speed = ret;

	plat->core_type = DWMAC_CORE_XGMAC;
	plat->bus_id = td->data->mac_id;
	/* phy_addr */
	plat->phy_interface = phy_interface;
	plat->mdio_bus_data = &td->mdio_bus_data;
	/* mdio_bus_data->probed_phy_irq is set in tc956x_xgmac3_probe() */
	/* phy_node */
	/* port_node */
	/* mdio_node */
	plat->dma_cfg = &td->dma_cfg;
	plat->dma_cfg->pbl = 32;
	plat->dma_cfg->pblx8 = true;
	/* safety_feat_cfg */
	plat->clk_csr = clk_csr;
	/* enh_desc */
	/* tx_coe */
	/* rx_coe */
	/* bugged_jumbo */
	/* pmt */
	plat->force_sf_dma_mode = 1;
	/* force_thresh_dma_mode */
	/* riwt_off */
	plat->max_speed = speed;
	/* XXX
	 * We use the default maxmtu (JUMBO_LEN = 9000).  Toshiba used 9024
	 * instead:  plat->maxmtu = ALIGN(9000, SMP_CACHE_BYTES);
	 */
	/* multicast_filter_bins */
	plat->unicast_filter_entries = 32;
	/*
	 * Oversized FIFOs result in reduced performance in bandwidth tests.
	 * Limit them to 8KiB per queue, or the total available.
	 */
	filter_size_kb = min(TC956X_TX_FIFO_KB, 8 * plat->tx_queues_to_use);
	plat->tx_fifo_size = SZ_1K * filter_size_kb;
	filter_size_kb = min(TC956X_RX_FIFO_KB, 8 * plat->rx_queues_to_use);
	plat->rx_fifo_size = SZ_1K * filter_size_kb;
	plat->host_dma_width = 36;

	/* XXX
	 * TC956x has 8 RX queues but we observe significantly reduced RX
	 * bandwidth if we don't have at least 8k FIFO space per queue, so
	 * by default we avoid using all the queues.
	 */
	plat->rx_queues_to_use = 4;

	/* XXX
	 * TX956x has 8 TX queues. However failures are observed (DHCP does not
	 * get an IP address or ping does fails) if tx_queues_to_use >3
	 */
	plat->tx_queues_to_use = 3;

	plat->rx_sched_algorithm = MTL_RX_ALGORITHM_SP;
	plat->tx_sched_algorithm = MTL_TX_ALGORITHM_WRR;

	for (i = 0; i < plat->rx_queues_to_use; i++)
		plat->rx_queues_cfg[i].mode_to_use = MTL_QUEUE_DCB;

	for (i = 0; i < plat->tx_queues_to_use; i++) {
		plat->tx_queues_cfg[i].weight = 12;
		plat->tx_queues_cfg[i].mode_to_use = MTL_QUEUE_DCB;

		/* Tx Queues 0-4 don't support TBS on TC956X */
		if (i >= 5)
			plat->tx_queues_cfg[i].tbs_en = true;
	}

	/* get_interfaces */
	/* set_phy_intf_sel */
	/* set_clk_tx_rate */
	plat->fix_mac_speed = tc956x_fix_mac_speed;
	/* fix_soc_reset */
	/* serdes_powerup */
	/* serdes_powerdown */
	/* mac_finish */
	/* ptp_clk_freq_config */
	/* init */
	/* exit */
	plat->suspend = tc956x_xgmac3_suspend;
	plat->resume = tc956x_xgmac3_resume;
	plat->mac_setup = tc956x_mac_setup;
	/* clks_config */
	/* crosststamp */
	/* dump_debug_regs */
	plat->pcs_init = tc956x_pcs_init;
	/* pcs_exit */
	plat->select_pcs = tc956x_select_pcs;

	plat->bsp_priv = td;
	/* stmmac_clk */
	/* pclk */
	/* clk_ptp_ref */
	/* clk_tx_i */
	plat->clk_ptp_rate = 250000000;
	/* clk_ref_rate */
	/* clks */
	/* num_clks */
	/* mult_fact_100ns */
	/* ptp_max_adj */
	/* cdc_error_adj */
	/* stmmac_rst */
	/* stmmac_ahb_rst */

	/* AXI Configuration */
	axi = &td->axi;
	axi->axi_lpi_en = 1;
	axi->axi_wr_osr_lmt = 31;
	axi->axi_rd_osr_lmt = 31;
	/* All sizes (2^2..2^8) are supported */
	axi->axi_blen_regval = field_max(DMA_AXI_BLEN_MASK);
	plat->axi = axi;
	/* rss_en */
	plat->mac_port_sel_speed = speed;
	/* vlan_fail_q */
	/* pdev * */
	/* int_snapshot_num */
	/* msi_mac_vec */
	/* msi_wol_vec */
	/* msi_sfty_ce_vec */
	/* msi_sfty_ue_vec */
	/* msi_rx_base_vec */
	/* msi_tx_base_vec */
	/* dwmac4_addrs */
	plat->flags = STMMAC_FLAG_MULTI_MSI_EN | STMMAC_FLAG_TSO_EN;

	td->plat = plat;

	return 0;
}

static int tc956x_xgmac3_probe(struct tc956x_data *td)
{
	struct stmmac_resources res = { };
	struct irq_domain *irq_domain;
	struct device *dev = td->dev;
	struct pinctrl *pinctrl;
	int ret;
	u32 i;

	/* Put the MAC in a known initial state */
	tc956x_mac_init_state(td);

	irq_domain = devm_tc956x_msigen_register(td);
	if (IS_ERR(irq_domain)) {
		ret = PTR_ERR(irq_domain);
		goto err;
	}

	res.addr = XGMAC_BASE(td);
	/* Problems creating mappings will be reported by stmmac_dvr_probe */
	res.irq = irq_create_mapping(irq_domain, HWIRQ_EVENT);
	for (i = 0; i < MTL_MAX_TX_QUEUES; i++)
		res.tx_irq[i] = irq_create_mapping(irq_domain, HWIRQ_TX0 + i);
	for (i = 0; i < MTL_MAX_RX_QUEUES; i++)
		res.rx_irq[i] = irq_create_mapping(irq_domain, HWIRQ_RX0 + i);
	res.wol_irq = td->wol_irq;

	/*
	 * Hook up the PHY interrupt.
	 *
	 * TODO: This probably wants to be made optional in the DT (if the
	 *       interrupt is not connected we need to fall back to polling)
	 */
	td->plat->mdio_bus_data->probed_phy_irq =
		irq_create_mapping(irq_domain, HWIRQ_ETH);

	/* XXX Can't we do this in the DTS file? */
	pinctrl = devm_pinctrl_get_select_default(dev);
	if (IS_ERR(pinctrl)) {
		ret = PTR_ERR(pinctrl);
		dev_err(dev, "error %d selecting PHY reset state\n", ret);
		goto err;
	}

	ret = tc956x_phy_power_on(td);
	if (ret) {
		dev_err(dev, "error %d powering on PHY\n", ret);
		goto err;
	}

	/* Won't fail; we know the phy_interface and max_speed are valid */
	(void)tc956x_chipcfg_mac_init(td);

	ret = stmmac_dvr_probe(dev, td->plat, &res);
	if (ret) {
		dev_set_drvdata(dev, NULL);
		goto err;
	}

	return 0;

err:
	tc956x_stop_mac(td);
	(void)tc956x_phy_power_off(td);

	return ret;
}

static void tc956x_xgmac3_remove(struct tc956x_data *td)
{
	stmmac_dvr_remove(td->dev);
	(void)tc956x_phy_power_off(td);
	tc956x_stop_mac(td);
}

static int tc956x_dwmac_probe(struct auxiliary_device *adev,
			       const struct auxiliary_device_id *id)
{
	struct device *dev = &adev->dev;
	int has_gpio_controller;
	struct tc956x_data *td;
	int ret;

	if (!dev_of_node(dev))
		return -EINVAL;

	/* XXX This will come from adev->driver_data instead */

	td = devm_kzalloc(dev, sizeof(*td), GFP_KERNEL);
	if (!td)
		return dev_err_probe(dev, -ENOMEM, "cannot create data\n");

	td->dev = dev;
	td->data = dev_get_platdata(dev);

	/* XXX And this might come from the data pointer */
	td->chip = dev_get_platdata(dev->parent);

	ret = devicetree_init(td);
	if (ret)
		return ret;

	ret = plat_stmmacenet_data_init(td);
	if (ret)
		return ret;

	ret = tc956x_xgmac3_probe(td);
	if (ret)
		dev_warn(td->dev, "Cannot initialize xgmac3 (%pe)%s\n",
			 ERR_PTR(ret),
			 has_gpio_controller ? " but GPIO will be kept" : "");

	return ret;
}

static void tc956x_dwmac_remove(struct auxiliary_device *adev)
{
	struct device *dev = &adev->dev;
	struct net_device *ndev = dev_get_drvdata(dev);

	if (ndev) {
		struct stmmac_priv *priv = netdev_priv(ndev);
		struct tc956x_data *td = priv->plat->bsp_priv;

		tc956x_xgmac3_remove(td);
	}
}

static const struct auxiliary_device_id tc956x_dwmac_ids[] = {
	{ .name = "tc956x_pci." DRIVER_NAME, },
	{ }
};
MODULE_DEVICE_TABLE(auxiliary, tc956x_dwmac_ids);

static struct auxiliary_driver tc956x_dwmac_driver = {
	.name		= DRIVER_NAME,
	.probe		= tc956x_dwmac_probe,
	.remove		= tc956x_dwmac_remove,
	.id_table	= tc956x_dwmac_ids,
	.driver = {
		.name	= DRIVER_NAME,
		.pm	= &stmmac_simple_pm_ops,
		.owner	= THIS_MODULE,
		/* .probe_type	= PROBE_PREFER_ASYNCHRONOUS, */
	},
};
module_auxiliary_driver(tc956x_dwmac_driver);

MODULE_DESCRIPTION("Toshiba TC956X PCIe Ethernet Network Driver");
MODULE_LICENSE("GPL");
