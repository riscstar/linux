// SPDX-License-Identifier: GPL-2.0

/*
 * Copyright (C) 2026 by RISCstar Solutions Corporation.  All rights reserved.
 *
 * Derived from code having the following copyrights:
 * Copyright (C) 2011-2012  Vayavya Labs Pvt Ltd
 * Copyright (C) 2025 Toshiba Electronic Devices & Storage Corporation
 */

#define pr_fmt(fmt) "dwmac-tc956x: " fmt

#include <linux/auxiliary_bus.h>
#include <linux/bitops.h>
#include <linux/cleanup.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/iopoll.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/of_irq.h>
#include <linux/pci.h>
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
#include "stmmac_libpci.h"

#include "soc-tc9564-chip.h"

#define DRIVER_NAME		"dwmac-tc956x"

#define GPIO_DEVICE_NAME	"tc9564-gpio"

/* PCI BAR assignments */
#define PCI_BAR_BRIDGE_CONFIG	0
#define PCI_BAR_SFR		4

/* XXX TC9564? Also, this is a physical function; virtual is 0x0221 */
#define PCI_DEVICE_ID_TOSHIBA_TC956X	0x0220

#define AXI4_SLV_TABLE_OFFSET		0x0800

/* Each AXI translation entry has has a block of registers this far apart */
#define AXI4_TABLE_STRIDE		0x20
#define AXI4_SLV_BASE(tid)						\
		(AXI4_SLV_TABLE_OFFSET + (tid) * AXI4_TABLE_STRIDE)

#define SRC_ADDR_LO_OFFSET		0x0000
#define ATR_IMPL			BIT(0)		/* 1 = enabled */
#define ATR_SIZE_MASK			GENMASK(6, 1)	/* size 2^(ATR + 1) */
#define SRC_ADDR_HI_OFFSET		0x0004
#define TRSL_ADDR_LO_OFFSET		0x0008
#define TRSL_ADDR_HI_OFFSET		0x000c
#define TRSL_PARAM_OFFSET		0x0010
#define TRSL_ID_MASK			GENMASK(3, 0)
#define TRSL_ID_PCIE_TX_RX		0
#define TRSF_PARAM_MASK			GENMASK(27, 16)

/* Address translation space size */
#define SLV00_ATR_SIZE_DEFAULT		63	/* 2^64 (16 exabytes) */
#define SLV00_ATR_SIZE			35	/* 2^36 (64 gigabytes) */
#define SLV00_SRC_ADDR			0x0000001000000000ULL
#define SLV00_TRSL_ADDR			0x0000000000000000ULL

#define CM3_TAMAP_COUNT			4

/* Configuration Register Address */
#define NCID_OFFSET			0x0000
#define NCID_REV_ID_MASK		GENMASK(7, 0)
#define NCID_CHIP_ID_MASK		GENMASK(15, 8)

#define NCTLSTS_OFFSET			0x1000
/* The next four are relative to the base of the clock/reset regmap (NCTLSTS) */
#define RSTCTRL0_OFFSET			0x0008
#define RSTCTRL1_OFFSET			0x0010
#define CLKCTRL0_OFFSET			0x0004
#define CLKCTRL1_OFFSET			0x000c

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

#define XGMAC_BASE(td)	((td)->sfr + ((td)->pci_fn ? 0x48000 : 0x40000))

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
 * @plat:		Pointer to our stmmac platform data
 * @bridge_config:	Mapped bridge config data (BAR 0)
 * @sfr:		Mapped SFR region (BAR 4)
 * @pci_fn:		Which PCI function this is (0 or 1)
 * @phy_supply:		PHY supply regulator
 * @phy_reset:		Descriptor for GPIO used for PHY reset
 * @phy_reset_delay:	Delay (milliseconds) after PHY reset
 * @wol_irq:		Wake-on-LAN IRQ number
 * @chip:		Pointer to the containing chip information
 * @dma_cfg:		DMA config buffer used by plat_stmmacenet_data
 * @mdio_bus_data:	MDIO bus data used by plat_stmmacenet_data
 */
struct tc956x_data {
	struct device *dev;
	struct plat_stmmacenet_data *plat;
	void __iomem *bridge_config;
	void __iomem *sfr;
	unsigned int pci_fn;
	struct regulator *phy_supply;
	struct gpio_desc *phy_reset;
	u32 phy_reset_delay;
	int wol_irq;
	struct tc9564_chip *chip;

	/* Remaining fields are used by the plat_stmmacenet_data structure */
	struct stmmac_dma_cfg dma_cfg;
	struct stmmac_mdio_bus_data mdio_bus_data;
};

/**
 * struct tc9564_chip - Common chip support information
 * @reset_clock_regmap:	Register map used for clocks and resets
 * @primary:		Data pointer for the primary eMAC interface
 * @secondary:		Device link between secondary (consumer) and primary
 * @pci_bus_num:	PCI bus this chip is on
 * @pci_slot:		PCI slot on its bus this chip fills
 * @rev_id:		Revision ID
 * @chip_id:		Chip ID
 * @links:		Links in the list of all chips
 *
 * A single tc9564_chip structure represents the chip as a whole,
 * collecting resources that are common to both eMAC interfaces.
 * The first eMAC probed will create one of these when it creates
 * its tc956x_data structure; this will be the *primary* interface.
 * The second eMAC (which could be eMAC 0 or 1, assuming it's probed)
 * will be the *secondary* interface.  The primary interface provides
 * the mapped SFR memory, and for that reason, cannot be removed
 * (and unmapped) unless the secondary interface is not in use.
 */
struct tc9564_chip {
	struct regmap *reset_clock_regmap;

	struct tc956x_data *primary;

	struct device_link *secondary;

	u8 pci_bus_num;
	u8 pci_slot;
	u8 rev_id;
	u8 chip_id;

	struct list_head links;		/* Protected by tc9564_chips_lock */
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

static LIST_HEAD(tc9564_chips);		/* List of TC956x chips */
static DEFINE_MUTEX(tc9564_chips_lock); /* Don't rely on synchronous probing */

static const struct regmap_config tc956x_gpio_regmap_config = {
	.name		= "tc956x-gpio",
	.reg_bits	= 32,
	.reg_stride	= 4,
	.reg_base	= 0x1200,	/* Register GPIOI0 */
	.val_bits	= 32,
	.max_register	= 0x1214,	/* Register GPIOO1 */
};

static const struct regmap_config tc956x_reset_clock_regmap_config = {
	.name		= "tc956x-clk-reset",
	.reg_bits	= 32,
	.reg_stride	= 4,
	.reg_base	= 0x1000,	/* Register NCTLSTS */
	.val_bits	= 32,
	.max_register	= 0x1010,	/* Register NRSTCTRL1 */
};

void tc9564_chip_reset_clock_set(struct tc9564_chip *chip, bool reset,
				 bool reg0, bool set, u8 bit)
{
	u32 offset = reset ? reg0 ? RSTCTRL0_OFFSET : RSTCTRL1_OFFSET
			   : reg0 ? CLKCTRL0_OFFSET : CLKCTRL1_OFFSET;
	u32 mask = BIT(bit);

	/* Note: no need to check for errors on read/write for MMIO regmap */
	(void)regmap_update_bits(chip->reset_clock_regmap, offset, mask,
				 set ? mask : 0);
}

static int tc956x_reset_clock_init(struct tc9564_chip *chip)
{
	struct tc956x_data *td = chip->primary;
	struct regmap *regmap;

	regmap = devm_regmap_init_mmio(td->dev, td->sfr,
				       &tc956x_reset_clock_regmap_config);
	if (IS_ERR(regmap))
		return PTR_ERR(regmap);
	chip->reset_clock_regmap = regmap;

	return 0;
}

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
	 * Clear the MSI flag. All interrupts within TC956x are level-high type.
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

static struct irq_domain *devm_tc956x_msigen_register(struct pci_dev *pdev,
						      struct tc956x_data *td)
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
	struct device *dev = &pdev->dev;
	struct irq_domain *domain;

	tc956x_msigen = devm_kmalloc(dev, sizeof(*tc956x_msigen), GFP_KERNEL);
	if (!tc956x_msigen)
		return ERR_PTR(-ENOMEM);

	tc956x_msigen->regs = td->sfr + TC956X_MSIGEN_BASE(td->pci_fn);
	tc956x_msigen->irq = pdev->irq;
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

static int tc956x_reset_gpio_get(struct tc956x_data *td)
{
	struct device *dev = td->dev;
	struct gpio_desc *gpio;
	int retries = 10;
	int ret;

	/*
	 * When we created the chip it registers a GPIO device and that device
	 * may be used to supply the phy-reset. It may not have finished probing
	 * by the time we get here and sadly we can't return -EPROBE_DEFER
	 * (because that would cause the GPIO device to de-register). Thus we
	 * must wait for a short period before failing.
	 */
	/* XXX
	 * This loop might not be required.  Daniel will do some power cycle
	 * testing overnight with the loop removed, to provide a convincing
	 * argument for its removal.
	 */
	do {
		gpio = devm_gpiod_get(dev, "phy-reset", GPIOD_OUT_LOW);
		msleep(10);
	} while (IS_ERR(gpio) && retries--);

	if (retries < 0)
		return PTR_ERR(td->phy_reset);
	td->phy_reset = gpio;

	/* XXX
	 * We can use the Ethernet PHY reset-assert-us and reset-deassert-us
	 * properties to specify some delays.  In addition, Ayaan's message
	 * said there were different delays:
	 *   10Gbps PHY (Marvell)
	 *     RST_OUT delay1 time:	21 msec		Not sure what this
	 *     RST_OUT delay2 time:	21 msec		Not sure what this
	 *     MDIO access wait time:	221 msec
	 * "Minimum time the user should wait before accessing MDIO"
	 *
	 *   2.5Gbps PHY (Qualcomm)
	 *     Wait time:		10 msec
	 * "Reset must be asserted for at least 10 ms after all power
	 * supplies and reference clock becomes stable."  They updated
	 * the time to 20 msec after they found PHY attach was failing.
	 *
	 * Note also that there is a reset-post-delay-us property defined
	 * in "mdio.yaml" that sounds like it's what should be used for the
	 * 221 msec delay specified above.
	 */
	ret = of_property_read_u32(dev_of_node(dev), "qcom,phy-reset-delay",
				   &td->phy_reset_delay);
	if (ret)
		dev_err(dev, "failed to get qcom,phy-reset-delay property\n");

	return ret;
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
	u32 val;

	/*
	 * When we re-initialize the PMA then the reset will already have
	 * been deasserted. We must make sure the PMA reset is asserted before
	 * we change the clock settings.
	 */
	tc9564_mac_reset_assert(td->chip, td->pci_fn, MAC_RESET_PMA);

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

	tc9564_mac_reset_deassert(td->chip, td->pci_fn, MAC_RESET_PMA);

	emac_ctl_reg = td->sfr + (td->pci_fn ? NEMAC1CTL_OFFSET
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
	int sp_sel;
	u32 val;

	sp_sel = tc956x_mac_speed_select(td, speed);
	if (sp_sel < 0)
		return sp_sel;

	/* Speeds up to 1Gbps require the 125 MHz clock to be enabled */
	if (speed < SPEED_2500)
		tc9564_mac_clock_enable(td->chip, td->pci_fn, MAC_CLOCK_125M);
	else
		tc9564_mac_clock_disable(td->chip, td->pci_fn, MAC_CLOCK_125M);

	emac_ctl_reg = td->sfr + (td->pci_fn ? NEMAC1CTL_OFFSET
					     : NEMAC0CTL_OFFSET);
	val = readl(emac_ctl_reg);
	val |= EMAC_LPIHWCLKEN;
	val &= ~EMAC_INV_SGM_SIG_DET;
	val = u32_replace_bits(val, PCS_CLK_PHY, EMAC_PHY_INF_SEL_MASK);
	val = u32_replace_bits(val, sp_sel, EMAC_SP_SEL_MASK);
	writel(val, emac_ctl_reg);

	return 0;
}

/**
 * tc956x_config_tamap() - Populate the table address map registers
 * @td:		TC956x driver private data pointer
 *
 * Populate the registers used to convert the AXI bus access to PCI TLP.
 */
static void tc956x_config_tamap(struct tc956x_data *td)
{
	void __iomem *base;
	u32 trsf_param_val;
	u32 atr_size_val;
	u32 val;
	u32 i;

	/* This is value assigned to *all* TRSL_PARAM registers */
	trsf_param_val = u32_encode_bits(TRSL_ID_PCIE_TX_RX, TRSL_ID_MASK);
	trsf_param_val |= u32_encode_bits(0, TRSF_PARAM_MASK);

	/*
	 * AXI4 slave 0 translation table 0
	 * We only use the first AXI4 slave translation table entry:
	 *	EDMA address region:	0x10 0000 0000 - 0x1f ffff ffff
	 *	is translated to:	0x00 0000 0000 - 0x0f ffff ffff
	 */
	BUILD_BUG_ON(SLV00_ATR_SIZE < 11);
	BUILD_BUG_ON(!!u32_get_bits(lower_32_bits(SLV00_SRC_ADDR),
						  ATR_SIZE_MASK));
	BUILD_BUG_ON(SLV00_SRC_ADDR & ATR_IMPL);

	base = td->bridge_config + AXI4_SLV_BASE(0);

	atr_size_val = u32_encode_bits(SLV00_ATR_SIZE, ATR_SIZE_MASK);
	atr_size_val |= ATR_IMPL;

	val = lower_32_bits(SLV00_SRC_ADDR) | atr_size_val;
	writel(val, base + SRC_ADDR_LO_OFFSET);

	val = upper_32_bits(SLV00_SRC_ADDR);
	writel(val, base + SRC_ADDR_HI_OFFSET);

	val = lower_32_bits(SLV00_TRSL_ADDR);
	writel(val, base + TRSL_ADDR_LO_OFFSET);

	val = upper_32_bits(SLV00_TRSL_ADDR);
	writel(val, base + TRSL_ADDR_HI_OFFSET);

	writel(trsf_param_val, base + TRSL_PARAM_OFFSET);

	/* Set all other unused entries to default values */
	BUILD_BUG_ON(SLV00_ATR_SIZE_DEFAULT < 11);
	atr_size_val = u32_encode_bits(SLV00_ATR_SIZE_DEFAULT, ATR_SIZE_MASK);
	atr_size_val |= ATR_IMPL;
	for (i = 1; i < CM3_TAMAP_COUNT; i++) {
		base = td->bridge_config + AXI4_SLV_BASE(i);
		writel(atr_size_val, base + SRC_ADDR_LO_OFFSET);
		writel(0x0, base + SRC_ADDR_HI_OFFSET);
		writel(0x0, base + TRSL_ADDR_LO_OFFSET);
		writel(0x0, base + TRSL_ADDR_HI_OFFSET);
		writel(trsf_param_val, base + TRSL_PARAM_OFFSET);
	}
}

static int tc956x_chipcfg_mac_init(struct tc956x_data *td)
{
	struct plat_stmmacenet_data *plat = td->plat;
	int ret;

	tc9564_mac_clock_enable(td->chip, td->pci_fn, MAC_CLOCK_TX);
	tc9564_mac_clock_enable(td->chip, td->pci_fn, MAC_CLOCK_RX);
	tc9564_mac_clock_enable(td->chip, td->pci_fn, MAC_CLOCK_ALL);
	if (td->pci_fn)
		tc9564_mac_clock_enable(td->chip, td->pci_fn, MAC_CLOCK_RMII);

	/* Set the speed related registers */
	ret = tc956x_chipcfg_mac_configure(td, plat->max_speed);
	if (ret)
		return ret;

	tc9564_mac_reset_deassert(td->chip, td->pci_fn, MAC_RESET_MAC);

	tc956x_mac_pma_init(td);

	tc9564_mac_reset_deassert(td->chip, td->pci_fn, MAC_RESET_XPCS);

	return 0;
}

static void tc956x_stop_mac(struct tc956x_data *td)
{
	tc9564_mac_reset_assert(td->chip, td->pci_fn, MAC_RESET_MAC);
	tc9564_mac_reset_assert(td->chip, td->pci_fn, MAC_RESET_PMA);
	tc9564_mac_reset_assert(td->chip, td->pci_fn, MAC_RESET_XPCS);

	tc9564_mac_clock_disable(td->chip, td->pci_fn, MAC_CLOCK_ALL);
	tc9564_mac_clock_disable(td->chip, td->pci_fn, MAC_CLOCK_RX);
	tc9564_mac_clock_disable(td->chip, td->pci_fn, MAC_CLOCK_TX);
	tc9564_mac_clock_disable(td->chip, td->pci_fn, MAC_CLOCK_125M);
	if (td->pci_fn)
		tc9564_mac_clock_disable(td->chip, td->pci_fn, MAC_CLOCK_RMII);
}

static void tc956x_mac_init_state(struct tc956x_data *td)
{
	tc9564_mac_clock_disable(td->chip, td->pci_fn, MAC_CLOCK_312_5M);

	tc956x_stop_mac(td);
}

static int tc956x_xgmac3_default_data(struct pci_dev *pdev,
				struct plat_stmmacenet_data *plat)
{
	struct tc956x_data *td = plat->bsp_priv;
	int speed;
	u32 i;

	switch (plat->phy_interface) {
	case PHY_INTERFACE_MODE_10GBASER:
		speed = SPEED_10000;
		break;
	case PHY_INTERFACE_MODE_SGMII:
	case PHY_INTERFACE_MODE_2500BASEX:
		speed = SPEED_2500;
		break;
	default:
		dev_err(&pdev->dev, "Unexpected PHY interface mode\n");
		return -ENOTSUPP;
	}

	/* AXI Configuration */
	plat->axi = devm_kzalloc(&pdev->dev, sizeof(*plat->axi), GFP_KERNEL);
	if (!plat->axi)
		return -ENOMEM;

	plat->axi->axi_lpi_en = 1;
	plat->axi->axi_wr_osr_lmt = 31;
	plat->axi->axi_rd_osr_lmt = 31;
	plat->axi->axi_blen_regval =
		DMA_AXI_BLEN256 | DMA_AXI_BLEN128 | DMA_AXI_BLEN64 |
		DMA_AXI_BLEN32 | DMA_AXI_BLEN16 | DMA_AXI_BLEN8 | DMA_AXI_BLEN4;

	plat->mac_port_sel_speed = speed;
	plat->max_speed = speed;

	/* Set common default data */
	plat->core_type = DWMAC_CORE_XGMAC;
	plat->force_sf_dma_mode = 1;
	plat->flags |= STMMAC_FLAG_MULTI_MSI_EN | STMMAC_FLAG_TSO_EN;

	plat->pdev = pdev;
	plat->clk_ptp_rate = 250000000;
	plat->host_dma_width = 36;

	/* For TC956X, clk_csr_i = 125MHz */
	if (td->pci_fn)			/* emac1: SGMII */
		plat->clk_csr = STMMAC_CSR_60_100M;
	else				/* emac0: XFI */
		plat->clk_csr = STMMAC_CSR_150_250M;	/* XXX set CRS bit? */

	plat->unicast_filter_entries = 32;

	plat->dma_cfg->pbl = 32;
	plat->dma_cfg->pblx8 = true;

	/*
	 * TC956x has 8 RX queues but we observe significantly reduced RX
	 * bandwidth if we don't have at least 8k FIFO space per queue, so
	 * by default we avoid using all the queues.
	 */
	plat->rx_queues_to_use = 4;
	plat->rx_sched_algorithm = MTL_RX_ALGORITHM_SP;

	for (i = 0; i < plat->rx_queues_to_use; i++)
		plat->rx_queues_cfg[i].mode_to_use = MTL_QUEUE_DCB;

	/*
	 * TX956x has 8 TX queues. However failures are observed (DHCP does not
	 * get an IP address or ping does fails) if tx_queues_to_use >3
	 */
	plat->tx_queues_to_use = 3;
	plat->tx_sched_algorithm = MTL_TX_ALGORITHM_WRR;

	for (i = 0; i < plat->tx_queues_to_use; i++) {
		plat->tx_queues_cfg[i].weight = 12;
		plat->tx_queues_cfg[i].mode_to_use = MTL_QUEUE_DCB;

		/* Tx Queues 0 - 4 don't support TBS on TC956x */
		if (i >= 5)
			plat->tx_queues_cfg[i].tbs_en = true;
	}

	/*
	 * Oversized FIFOs result in reduced performance in bandwidth tests.
	 * Let's limit them to 8KiB unless they must be smaller.
	 */
	plat->tx_fifo_size = min(plat->tx_queues_to_use * 8, 46) * SZ_1K;
	plat->rx_fifo_size = min(plat->rx_queues_to_use * 8, 46) * SZ_1K;

	return 0;
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
	setting = td->chip->rev_id == 1 ? 3 : 0;
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
	struct {
		struct stmmac_dma_ops dma;
		struct stmmac_desc_ops desc;
	} *ops;

	ops = devm_kzalloc(priv->device, sizeof(*ops), GFP_KERNEL);
	if (!ops)
		return -ENOMEM;

	ops->dma = dwxgmac210_dma_ops;
	ops->dma.init = tc956x_dma_init;
	ops->dma.init_rx_chan = tc956x_dma_init_rx_chan;
	ops->dma.init_tx_chan = tc956x_dma_init_tx_chan;
	mac->dma = &ops->dma;

	ops->desc = dwxgmac210_desc_ops;
	ops->desc.set_addr = tc956x_desc_set_addr;
	ops->desc.set_sec_addr = tc956x_desc_set_sec_addr;
	mac->desc = &ops->desc;

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

	/* Configure TA map registers whenever the primary MAC is initialized */
	if (td == td->chip->primary)
		tc956x_config_tamap(td);

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

static void tc956x_adev_release(struct device *dev)
{
	kfree(to_auxiliary_dev(dev));
}

static void tc956x_adev_remove(void *data)
{
	struct auxiliary_device *adev = data;

	auxiliary_device_delete(adev);
	auxiliary_device_uninit(adev);
}

static int tc956x_devm_adev_device_add(struct tc956x_data *td,
				       const char *name, struct regmap *regmap)
{
	struct auxiliary_device *adev;
	struct device *dev = td->dev;
	int ret;

	adev = devm_kzalloc(dev, sizeof(*adev), GFP_KERNEL);
	if (!adev)
		return -ENOMEM;

	adev->name = name;
	adev->dev.parent = dev;
	adev->dev.release = tc956x_adev_release;
	adev->dev.of_node = dev->of_node;
	adev->dev.platform_data = regmap;
	adev->id = PCI_FUNC(to_pci_dev(dev)->devfn);

	ret = auxiliary_device_init(adev);
	if (ret)
		return ret;

	ret = auxiliary_device_add(adev);
	if (ret) {
		auxiliary_device_uninit(adev);
		return ret;
	}

	ret = devm_add_action_or_reset(dev, tc956x_adev_remove, adev);
	if (ret)
		return ret;

	return 1;
}

/* The embedded GPIO controller has an auxiliary device driver */
static int tc956x_devm_gpio_device_add(struct tc956x_data *td)
{
	struct device *dev = td->dev;
	void __iomem *base = td->sfr;
	struct regmap *regmap;

	/* XXX It might be nice to preclude both MACs defining this */
	if (!device_property_present(dev, "gpio-controller"))
		return 0;

	regmap = devm_regmap_init_mmio(dev, base, &tc956x_gpio_regmap_config);
	if (IS_ERR(regmap))
		return PTR_ERR(regmap);

	return tc956x_devm_adev_device_add(td, GPIO_DEVICE_NAME, regmap);
}

static struct plat_stmmacenet_data *
tc956x_plat_dat_alloc(struct tc956x_data *td, struct pci_dev *pdev)
{
	struct plat_stmmacenet_data *plat;
	struct device *dev = &pdev->dev;

	/* The platform structure is allocated with devm_kzalloc() */
	plat = stmmac_plat_dat_alloc(dev);
	if (!plat)
		return NULL;

	plat->bsp_priv = td;
	plat->bus_id = pci_dev_id(pdev);
	plat->mac_setup = tc956x_mac_setup;
	plat->fix_mac_speed = tc956x_fix_mac_speed;
	plat->pcs_init = tc956x_pcs_init;
	plat->select_pcs = tc956x_select_pcs;
	plat->suspend = tc956x_xgmac3_suspend;
	plat->resume = tc956x_xgmac3_resume;

	/* The probed_phy_irq field is set in tc956x_xgmac3_probe() */
	plat->mdio_bus_data = &td->mdio_bus_data;

	/* Initialized in tc956x_xgmac3_default_data() and tc956x_dma_init() */
	plat->dma_cfg = &td->dma_cfg;

	return plat;
}

static void tc956x_start_chip(struct tc9564_chip *chip)
{
	tc9564_chip_reset_deassert(chip, CHIP_RESET_MSIGEN);
	tc9564_chip_clock_enable(chip, CHIP_CLOCK_MSIGEN);
}

static void tc956x_stop_chip(struct tc9564_chip *chip)
{
	tc9564_chip_reset_assert(chip, CHIP_RESET_MSIGEN);
	tc9564_chip_clock_disable(chip, CHIP_CLOCK_MSIGEN);
}

static void tc9564_chip_init_state(struct tc9564_chip *chip)
{
	tc9564_chip_reset_assert(chip, CHIP_RESET_MCU);
	tc9564_chip_reset_assert(chip, CHIP_RESET_MCU1);
	tc9564_chip_reset_assert(chip, CHIP_RESET_INTC);
	tc9564_chip_reset_assert(chip, CHIP_RESET_UART0);

	tc9564_chip_clock_disable(chip, CHIP_CLOCK_MCU);
	tc9564_chip_clock_disable(chip, CHIP_CLOCK_SRAM);
	tc9564_chip_clock_disable(chip, CHIP_CLOCK_PLL);
	tc9564_chip_clock_disable(chip, CHIP_CLOCK_SGMII);
	tc9564_chip_clock_disable(chip, CHIP_CLOCK_REFCLK);
	tc9564_chip_clock_disable(chip, CHIP_CLOCK_INTC);
	tc9564_chip_clock_disable(chip, CHIP_CLOCK_UART0);

	tc956x_stop_chip(chip);
}

static struct tc9564_chip *tc9564_chip_get(struct tc956x_data *td)
{
	struct device *dev = td->dev;
	struct pci_dev *pdev = to_pci_dev(dev);
	u8 pci_bus_num = PCI_BUS_NUM(pdev->devfn);
	u8 pci_slot = PCI_SLOT(pdev->devfn);
	struct tc9564_chip *chip;
	u32 val;
	int ret;

	/* Use the existing chip structure if it's already been created */
	list_for_each_entry(chip, &tc9564_chips, links) {
		if (chip->pci_bus_num != pci_bus_num)
			continue;
		if (chip->pci_slot != pci_slot)
			continue;

		/* Make sure the secondary hasn't already been recorded */
		if (WARN_ON(chip->secondary))
			return ERR_PTR(-EINVAL);

		chip->secondary = device_link_add(td->dev, chip->primary->dev, DL_FLAG_STATELESS);
		if (!chip->secondary)
			return ERR_PTR(-ENODEV);

		return chip;
	}

	/* We're operating on the primary MAC, and need a new chip structure */
	chip = devm_kzalloc(dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return ERR_PTR(-ENOMEM);

	chip->pci_bus_num = pci_bus_num;
	chip->pci_slot = pci_slot;
	chip->primary = td;

	/* Get the chip and revision IDs */
	val = readl(td->sfr + NCID_OFFSET);
	chip->rev_id = u32_get_bits(val, NCID_REV_ID_MASK);
	chip->chip_id = u32_get_bits(val, NCID_CHIP_ID_MASK);
	dev_dbg(dev, "NCID Register value: %x\n", val);

	ret = tc956x_reset_clock_init(chip);
	if (ret)
		return ERR_PTR(ret);

	tc9564_chip_init_state(chip);
	tc956x_start_chip(chip);

	list_add(&chip->links, &tc9564_chips);

	return chip;
}

static void tc9564_chip_put(struct tc956x_data *td)
{
	struct tc9564_chip *chip = td->chip;

	td->chip = NULL;

	if (chip->secondary && td->dev == chip->secondary->consumer) {
		device_link_del(chip->secondary);
		chip->secondary = NULL;
		return;
	}

	/*
	 * The primary interface needs to be the last to go. The driver uses
	 * device links to guarantee that... so if this warning fires then
	 * things have gone pretty badly wrong!
	 */
	WARN(chip->secondary, "tc9564_chip_put() calls are incorrectly ordered");

	list_del(&chip->links);

	tc956x_stop_chip(chip);

	chip->primary = NULL;
	chip->pci_slot = 0;
	chip->pci_bus_num = 0;
}

static int tc956x_xgmac3_probe(struct tc956x_data *td)
{
	struct device *dev = td->dev;
	struct pci_dev *pdev = to_pci_dev(dev);
	struct stmmac_resources res = { };
	struct irq_domain *irq_domain;
	struct pinctrl *pinctrl;
	int ret;
	u32 i;

	td->plat = tc956x_plat_dat_alloc(td, pdev);
	if (!td->plat)
		return -ENOMEM;

	ret = of_irq_get_byname(dev_of_node(dev), "wake-on-lan");
	if (ret <= 0) {
		dev_err(dev, "failed to get wake-on-lan property\n");
		return ret ? : -EINVAL;
	}
	td->wol_irq = ret;

	td->phy_supply = devm_regulator_get(dev, "phy");
	if (IS_ERR(td->phy_supply)) {
		dev_err(dev, "failed to get phy-supply\n");
		return PTR_ERR(td->phy_supply);
	}

	/*
	 * We must hold the chips lock until we have decided whether or not we
	 * will be the primary device. This is a relatively long held lock
	 * because we cannot fully commit to being the primary until right at
	 * end of the probe function.
	 */
	guard(mutex)(&tc9564_chips_lock);

	td->chip = tc9564_chip_get(td);
	if (IS_ERR(td->chip))
		return dev_err_probe(dev, PTR_ERR(td->chip), "cannot get chip\n");

	/* Put the MAC in a known initial state */
	tc956x_mac_init_state(td);

	// TODO: this needs to come from devicetree
	td->plat->phy_interface = td->pci_fn ? PHY_INTERFACE_MODE_SGMII
					     : PHY_INTERFACE_MODE_10GBASER;

	ret = tc956x_xgmac3_default_data(pdev, td->plat);
	if (ret)
		goto err;

	pci_set_master(pdev);

	/*
	 * Enable MSI and Allocate Vectors. Despite the spelling (no pcim) the
	 * free will be handled by devres due to the prior pcim_enable_device()
	 */
	ret = pci_alloc_irq_vectors(pdev, 1, 1, PCI_IRQ_MSI);

	if (ret < 1) {
		dev_err(dev, "%s:Enable MSI error\n", DRIVER_NAME);
		goto err;
	}

	irq_domain = devm_tc956x_msigen_register(pdev, td);
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

	ret = tc956x_reset_gpio_get(td);
	if (ret)
		goto err;

	/* Configure TA map registers whenever the primary MAC is initialized */
	if (td == td->chip->primary)
		tc956x_config_tamap(td);

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
	tc9564_chip_put(td);

	return ret;
}

static void tc956x_xgmac3_remove(struct tc956x_data *td)
{
	stmmac_dvr_remove(td->dev);
	(void)tc956x_phy_power_off(td);
	tc956x_stop_mac(td);

	scoped_guard(mutex, &tc9564_chips_lock)
	{
		tc9564_chip_put(td);
	}
}

static int tc9564x_dwmac_probe(struct auxiliary_device *adev,
			       const struct auxiliary_device_id *id)
{
	struct device *dev = &adev->dev;
	struct tc9564_dwmac_data *data;
	int has_gpio_controller;
	struct tc956x_data *td;
	int ret;

	if (!dev_of_node(dev))
		return -EINVAL;

	/* XXX This will come from adev->driver_data instead */
	data = dev_get_platdata(dev);

	td = devm_kzalloc(dev, sizeof(*td), GFP_KERNEL);
	if (!td)
		return dev_err_probe(dev, -ENOMEM, "cannot create data\n");

	td->dev = dev;
	td->pci_fn = data->id;
	td->sfr = data->sfr;
	/* XXX And this might come from the data pointer */
	td->chip = dev_get_platdata(dev->parent);

	has_gpio_controller = tc956x_devm_gpio_device_add(td);
	if (has_gpio_controller < 0)
		return dev_err_probe(td->dev, has_gpio_controller, "GPIO add failed\n");

	ret = tc956x_xgmac3_probe(td);
	if (ret)
		dev_warn(td->dev, "Cannot initialize xgmac3 (%pe)%s\n",
			 ERR_PTR(ret),
			 has_gpio_controller ? " but GPIO will be kept" : "");

	/*
	 * If we created a GPIO controller then the probe has succeeded even if
	 * we cannot initialize the eMAC.
	 */
	return has_gpio_controller ? 0 : ret;
}

static void tc9564x_dwmac_remove(struct auxiliary_device *adev)
{
	struct device *dev = &adev->dev;
	struct net_device *ndev = dev_get_drvdata(dev);

	if (ndev) {
		struct stmmac_priv *priv = netdev_priv(ndev);
		struct tc956x_data *td = priv->plat->bsp_priv;

		tc956x_xgmac3_remove(td);
	}
}

static const struct auxiliary_device_id tc964x_dwmac_ids[] = {
	{ .name = "misc_tc9564_chip." DRIVER_NAME, },
	{ }
};

#if !(IS_ENABLED(CONFIG_TC956X_NET) || IS_ENABLED(CONFIG_DWMAC_TC9564))
MODULE_DEVICE_TABLE(auxiliary, tc9564x_dwmac_ids);
#endif

/**
 * tc956x_suspend - suspend callback
 * @dev: device pointer
 * Description: Most of the TC956x MAC suspend is handled from via stmmac
 * callbacks (tc956x_xgmac3_suspend). This "outer" suspend function simply helps
 * us cope when a PCI device provides a GPIO controller but the MAC is inactive.
 */
static int tc956x_suspend(struct device *dev)
{
	struct net_device *ndev = dev_get_drvdata(dev);
	int ret;

	/* If we are a GPIO-only device then there will be no device data */
	if (ndev) {
		struct stmmac_priv *priv = netdev_priv(ndev);
		struct tc956x_data *td = priv->plat->bsp_priv;

		ret = stmmac_suspend(dev);
		if (ret)
			return ret;

		if (td == td->chip->primary)
			tc956x_stop_chip(td->chip);
	}

	return stmmac_pci_plat_suspend(dev, NULL);
}

static int tc956x_resume(struct device *dev)
{
	struct net_device *ndev = dev_get_drvdata(dev);
	int ret;

	ret = stmmac_pci_plat_resume(dev, NULL);
	if (ret)
		return ret;

	/* If we are a GPIO-only device then there will be no device data */
	if (ndev) {
		struct stmmac_priv *priv = netdev_priv(ndev);
		struct tc956x_data *td = priv->plat->bsp_priv;

		if (td == td->chip->primary)
			tc956x_start_chip(td->chip);

		return stmmac_resume(dev);
	}

	return 0;
}

static SIMPLE_DEV_PM_OPS(tc9564x_pm_ops, tc956x_suspend, tc956x_resume);

static struct auxiliary_driver tc9564x_dwmac_driver = {
	.name		= DRIVER_NAME,
	.probe		= tc9564x_dwmac_probe,
	.remove		= tc9564x_dwmac_remove,
	.id_table	= tc964x_dwmac_ids,
	.driver = {
		.name	= DRIVER_NAME,
		.pm	= &tc9564x_pm_ops,
		.owner	= THIS_MODULE,
		/* .probe_type	= PROBE_PREFER_ASYNCHRONOUS, */
	},
};
module_auxiliary_driver(tc9564x_dwmac_driver);

MODULE_DESCRIPTION("Toshiba TC956x PCIe Ethernet Network Driver");
MODULE_LICENSE("GPL");
