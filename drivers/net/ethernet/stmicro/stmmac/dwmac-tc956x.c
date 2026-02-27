// SPDX-License-Identifier: GPL-2.0

/*
 * Copyright (C) 2026 by RISCstar Solutions Corporation.  All rights reserved.
 *
 * Derived from code having the following copyrights:
 * Copyright (C) 2011-2012  Vayavya Labs Pvt Ltd
 * Copyright (C) 2025 Toshiba Electronic Devices & Storage Corporation
 */

#define pr_fmt(fmt) "dwmac-tc956x: " fmt

#include <linux/stmmac.h>
#include <linux/clk-provider.h>
#include <linux/pci.h>
#include <linux/dmi.h>
#include <linux/firmware.h>
#include <linux/version.h>
#include <linux/aer.h>
#include <linux/iopoll.h>
#include <linux/gpio/consumer.h>
#include <linux/gpio/driver.h>
#include <linux/gpio/machine.h>
#include <linux/mfd/core.h>
#include <linux/pcs/pcs-xpcs.h>
#include <linux/pcs/pcs-xpcs-regmap.h>
#include <linux/pinctrl/consumer.h>
#include <linux/phy.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/of_irq.h>
#include <linux/delay.h>

#include "stmmac.h"
#include "dwxgmac2.h"
#include "common.h"
#include "dwmac-tc956x-reset.h"

#define DRIVER_NAME "dwmac-tc956x"

/*
 * XXX TC956X_AXI4_SLV00_ATR_SIZE (36) defines the source translation
 * XXX region size.  The value held in the field is one less than this,
 * XXX so we subtract one when filling it.
 * XXX
 * XXX That *might* also be related to the value recorded as the
 * XXX base of the translated space:
 * XXX	TC956X_AXI4_SLV00_SRC_ADDR_HI_VAL  0x00000010U
 * XXX	TC956X_AXI4_SLV00_SRC_ADDR_LO_VAL  0x00000000U
 * XXX
 * XXX Start working on isolating the clocks and resets (and GPIOs and
 * XXX pinctrl), possibly implementing them using the proper common clock
 * XXX and reset interfaces.
 */

//
// (Long term) TODOs
//
// * tc956x_qcom.c included a `qcom,link-down-macrst` property to provide
//   "MAC reset for PHY Clock loss during Link Down". It was removed from
//   this driver when unused code was deleted. It is an important property?
// * tc956x_qcom.c also has `qcom,c45_needed` which was removed because there
//   was nothing in stmmac core to connect it to (FWIW the register logs shows
//   clause 45 activity on port 1/2.5G)
// * TC956x has support for faster MDIO bus scanning (by increasing CSR
//   clock rate). Should we replicate that?
// * TC956x has support for phy interrupts. Should that be re-enabled?
//

/* PCI BAR assignments */
#define PCI_BAR_BRIDGE_CONFIG		0
#define PCI_BAR_SFR			4

/**
 * struct tc956x_data - Toshiba-specific platform data
 * @dev:		Device pointer
 * @devfn:		PCI device/function id
 * @plat:		Pointer to our stmmac platform data
 * @bridge_config:	Mapped bridge config data (BAR 0)
 * @sfr:		Mapped SFR region (BAR 4)
 * @emac0:		Which eMAC port this is (true: port 0; false: port 1)
 * @pm_saved_emac_rst:	Saved eMAC reset control register value
 * @pm_saved_emac_clk:	Saved eMAC clock control register value
 * @pci_bd:		PCIe bus and device ID
 * @pinctrl:		Pin control structure
 * @pinctrl_default:	Pin control default value
 * @phy_supply:		PHY supply egulator
 * @phy_reset:		Descriptor for GPIO used for PHY reset
 * @phy_reset_delay:	Delay (milliseconds) after PHY reset
 * @reset_asserted:	Whether reset on this PHY is currently asserted
 * @wol_irq:		Wake-on-LAN IRQ number
 * @chip:		Pointer to the containing chip information
 * @regmap:		Register map for SFR region access
 */
struct tc956x_data {
	struct device *dev;
	unsigned int devfn;
	struct plat_stmmacenet_data *plat;
	void __iomem *bridge_config;
	void __iomem *sfr;
	bool emac0;
	u32 pm_saved_emac_rst;
	u32 pm_saved_emac_clk;
	uint16_t pci_bd;
	struct pinctrl *pinctrl;
	struct pinctrl_state *pinctrl_default;
	struct regulator *phy_supply;
	struct gpio_desc *phy_reset;
	u32 phy_reset_delay;
	u32 reset_asserted;
	int wol_irq;
	struct tc956x_chip *chip;
	struct regmap *regmap;
};

/**
 * struct tc956x_chip - Common chip support information
 * @pci_bus_num:	PCI bus this chip is on
 * @pci_slot:		PCI slot on its bus this chip fills
 * @primary:		Data pointer for the primary eMAC interface
 * @secondary:		Data pointer for the secondary eMAC interface
 * @gpio:		Pointer to GPIO information
 * @regmap:		Register map for SFR region access
 * @links:		Links in the list of all chips
 *
 * A single tc956x_chip structure represents the chip as a whole,
 * collecting resources that are common to both eMAC interfaces.
 * The first eMAC probed will create one of these when it creates
 * its tc956x_data structure; this will be the *primary* interface.
 * The second eMAC (which could be eMAC 0 or 1, assuming it's probed)
 * will be the *secondary* interface.  The primary interface provides
 * the mapped SFR memory, and for that reason, cannot be removed
 * (and unmapped) unless the secondary interface is not in use.
 */
struct tc956x_chip {
	u8 pci_bus_num;
	u8 pci_slot;
	struct tc956x_data *primary;
	struct tc956x_data *secondary;
	struct list_head links;		/* XXX any locking needed? */
};

static LIST_HEAD(tc956x_chips);		/* List of TC956x chips */

static const struct regmap_config tc956x_gpio_regmap_config = {
	.name		= "tc956x-gpio",
	.reg_bits	= 32,
	.reg_stride	= 4,
	.reg_base	= 0x1200,	/* Register GPIOI0 */
	.val_bits	= 32,
	.max_register	= 0x1214,	/* Register GPIOO1 */
};

static const struct regmap_config tc956x_clk_reset_regmap_config = {
	.name		= "tc956x-clk-reset",
	.reg_bits	= 32,
	.reg_stride	= 4,
	.reg_base	= 0x1000,	/* Register NCTLSTS */
	.val_bits	= 32,
	.max_register	= 0x1010,	/* Register NRSTCTRL1 */
};

static const struct regmap_config tc956x_regmap_config = {
	.name		= "tc956x",
	.reg_bits	= 32,
	.reg_stride	= 4,
	.reg_base	= 0,
	.val_bits	= 32,
	/* .max_register	= xxx, */
};

/* XXX TC9564? Also, this is a physical function; virtual is 0x0221 */
#define PCI_DEVICE_ID_TOSHIBA_TC956X		0x0220

//
// Definitions taken from tc956xmac.h in vendor driver
//

#define AXI4_SLV_TABLE_OFFSET		0x0800

/* Each AXI translation entry has has a block of registers this far apart */
#define AXI4_TABLE_STRIDE		0x20
#define AXI4_SLV_BASE(tid) \
		(AXI4_SLV_TABLE_OFFSET + (tid) * AXI4_TABLE_STRIDE)

#define SRC_ADDR_LO_OFFSET		0x00
#define ATR_IMPL_MASK		BIT(0)		/* 1 = enabled */
#define ATR_SIZE_MASK		GENMASK(6, 1)	/* size 2^(ATR + 1) */
#define SRC_ADDR_HI_OFFSET		0x04
#define TRSL_ADDR_LO_OFFSET		0x08
#define TRSL_ADDR_HI_OFFSET		0x0C
#define TRSL_PARAM_OFFSET		0x10
#define TRSL_ID_MASK		GENMASK(3, 0)
#define TRSL_ID_PCIE_TX_RX		0
#define TRSF_PARAM_MASK		GENMASK(27, 16)
#define TRSL_MASK_LO_OFFSET		0x18
#define TRSL_MASK_HI_OFFSET		0x1C

#define TC956X_AXI4_SLV00_ATR_SIZE	36U	/* log2(Addr transl size) */
#define TC956X_AXI4_SLV00_SRC_ADDR	0x0000001000000000ULL
#define TC956X_AXI4_SLV00_TRSL_ADDR	0x0000000000000000ULL

/* XXX This is an invalid value; 0x00000017 is the minimum allowed */
#define TC956X_AXI4_SLV00_SRC_ADDR_LO_VAL_DEFAULT  (0x0000007FU)

#define CM3_TAMAP_COUNT			4

//
// Definitions taken from tc956xmac_inc.h in vendor driver
//

struct tx956x_shrd_mem {
	uint16_t pci_bd;
	uint16_t pci_dev_active_cnt;
	uint16_t eth_link_down_cnt;
};

//
// Definitions taken from common.h in vendor driver
//

#define TC956X_TOT_CASCADE_DEV	7 /* Maximum number of devices for 2 Level cascade setup */
#define TC956X_PCI_BD_MASK	0xFFF8

// TODO: this was unifdef'ed (some build options result in the value being two)
#define TC956X_TOT_MSI_VEC	1

#define TC956X_DA_MAP		0xF

#define XGMAC_BASE(td)	((td)->sfr + ((td)->emac0 ? 0x40000 : 0x48000))

/*	Configuration Register Address	*/
#define NCID_OFFSET			(0x0000) /* TC956X Chip and revision ID */
#define NMODESTS_OFFSET		(0x0004) /* TC956X current operation mode */
#define NMODESTS_MODE2		BIT(10)	/* PCIe lanes: 0:  x4x1x1; 1: x2x2x1 */

#define NCLKCTRL0_OFFSET	0x1004	/* Clock control register 0 */
#define CLK0_MCUCEN		BIT(0)		/* COMMON */
#define CLK0_INTCEN		BIT(4)		/* individual */
#define CLK0_MAC0TXCEN		BIT(7)		/* IO */
#define CLK0_PCIECEN		BIT(9)		/* COMMON */
#define CLK0_I2SSPIEN		BIT(12)		/* COMMON */
#define CLK0_SRMCEM		BIT(13)		/* COMMON */
#define CLK0_MAC0RXCEN		BIT(14)		/* IO */
#define CLK0_UARTOCEN		BIT(16)		/* OTHER */
#define CLK0_MSIGENCEN		BIT(18)		/* OTHER */
#define CLK0_POEPLLCEN		BIT(24)		/* BUS */
#define CLK0_SGMPCIEN		BIT(25)		/* BUS */
#define CLK0_REFCLKOCEN		BIT(26)		/* BUS */
#define CLK0_MAC0125CLKEN	BIT(29)		/* CORE */
#define CLK0_MAC0312CLKEN	BIT(30)		/* CORE */
#define CLK0_MAC0ALLCLKEN	BIT(31)		/* IO */

#define CLK0_MAC0_CORE_MASK \
		(CLK0_MAC0125CLKEN | CLK0_MAC0312CLKEN)
#define CLK0_MAC0_IO_MASK \
		(CLK0_MAC0TXCEN | CLK0_MAC0RXCEN | CLK0_MAC0ALLCLKEN)
#define CLK0_COMMON_MASK \
		(CLK0_MCUCEN | CLK0_PCIECEN | CLK0_I2SSPIEN | CLK0_SRMCEM)
#define CLK0_OTHER_MASK	\
		(CLK0_UARTOCEN | CLK0_MSIGENCEN)
#define CLK0_BUS_MASK \
		(CLK0_POEPLLCEN | CLK0_SGMPCIEN | CLK0_REFCLKOCEN)

#define NCLKCTRL1_OFFSET	0x100c	/* Clock control register 1 */
#define CLK1_MAC1TXCEN		BIT(7)		/* IO */
#define CLK1_MAC1RXCEN		BIT(14)		/* IO */
#define CLK1_MAC1RMCEN		BIT(15)		/* individual */
#define CLK1_MAC1125CLKEN	BIT(29)		/* CORE */
#define CLK1_MAC1312CLKEN	BIT(30)		/* CORE */
#define CLK1_MAC1ALLCLKEN	BIT(31)		/* IO */

#define CLK1_MAC1_CORE_MASK \
		(CLK1_MAC1125CLKEN | CLK1_MAC1312CLKEN)
#define CLK1_MAC1_IO_MASK \
		(CLK1_MAC1TXCEN | CLK1_MAC1RXCEN | CLK1_MAC1ALLCLKEN)

#define NRSTCTRL0_OFFSET	0x1008	/* Reset control register 0 */
#define RST0_MCURST		BIT(0)		/* M3 system reset */
#define RST0_MCU1RST		BIT(1)		/* M3 cold reset */
#define RST0_INTRST		BIT(4)
#define RST0_MAC0RST		BIT(7)
#define RST0_UART0RST		BIT(16)
#define RST0_MSIGENRST		BIT(18)
#define RST0_MAC0PMARST		BIT(30)
#define RST0_MAC0XPCSRST	BIT(31)

#define RST0_MCU_MASK \
		(RST0_MCURST | RST0_MCU1RST)
#define RST0_OTHER_MASK	\
		(RST0_UART0RST | RST0_MSIGENRST)

#define NRSTCTRL1_OFFSET	0x1010	/* Reset control register 1 */
#define RST1_MAC1RST		BIT(7)		/* individual */
#define RST1_MAC1PMARST		BIT(30)
#define RST1_MAC1XPCSRST	BIT(31)

/* Field in the NCLKCTRL0 register to enable the MSIGEN clock */
#define TC956X_MSIGENCEN	BIT(18)

/* Field in the NRSTCTRL0 register to assert the MSIGEN reset */
#define TC956X_MSIGENSRST	BIT(18)

#define NMISCCTL_OFFSET		(0x1800)

/* MSIGEN Registers */

#define TC956X_MSI_BASE		0xf000

/* Each PCIe function has a block of registers this far apart */
#define MSIGEN_STRIDE		0x0100
#define MSIGEN_BASE(pf_id)	(TC956X_MSI_BASE + (pf_id) * MSIGEN_STRIDE)

#define TC956X_MSI_OUT_EN_OFFSET	0x0000
#define TC956X_MSI_MASK_SET_OFFSET	0x0008
#define TC956X_MSI_MASK_CLR_OFFSET	0x000c
#define TC956X_MSI_INT_STS_OFFSET	0x0010
#define TC956X_MSI_VECT_SET0_OFFSET	0x0020
#define TC956X_MSI_VECT_SET1_OFFSET	0x0024
#define TC956X_MSI_VECT_SET2_OFFSET	0x0028
#define TC956X_MSI_VECT_SET3_OFFSET	0x002C
#define TC956X_MSI_VECT_SET4_OFFSET	0x0030
#define TC956X_MSI_VECT_SET5_OFFSET	0x0034
#define TC956X_MSI_VECT_SET6_OFFSET	0x0038
#define TC956X_MSI_VECT_SET7_OFFSET	0x003C

#define TC956X_MSI_OUT_EN_CLR	(0x00000000)

#define TC956X_MSI_OUT_EN (0x0017FFF8)

#define TC956X_MSI_MASK_SET	(0xFFFFFFFE)
#define TC956X_MSI_MASK_CLR	(0x00000001)
#define TC956X_MSI_SET0		(0x00000000)
#define TC956X_MSI_SET1		(0x00000000)
#define TC956X_MSI_SET2		(0x00000000)
#define TC956X_MSI_SET3		(0x00000000)
#define TC956X_MSI_SET4		(0x00000000)
#define TC956X_MSI_SET5		(0x00000000)
#define TC956X_MSI_SET6		(0x00000000)
#define TC956X_MSI_SET7		(0x00000000)

/* EMAC control registers for ports 0 and 1 (both have same format) */
#define NEMAC0CTL_OFFSET		0x1070
#define NEMAC1CTL_OFFSET		0x1074

/* Fields and values for the NEMACxCTL registers */
#define EMAC_SP_SEL_MASK		GENMASK(3, 0)
#define SPEED_SGMII_2500M		4
#define SPEED_SGMII_1000M		5
#define SPEED_USXGMII_10G_10G		8
#define EMAC_PHY_INF_SEL_MASK		GENMASK(5, 4)
#define PCS_CLK_PLL			0	/* Clock from internal PLL */
#define PCS_CLK_PHY			1	/* Clock from PHY */
#define EMAC_INV_SGM_SIG_DET		BIT(6)	/* 1 = polarity inverted */
#define EMAC_LPIHWCLKEN			BIT(8)	/* 1 = low power mode */
#define EMAC_INIT_DONE			BIT(21)

#define SP_ETH1_SHIFT			24
#define SP_ETH_1G				1
#define SP_ETH_100M				3
#define SP_ETH_10M				7

/* Pin configuration for PHY resets; eMAC0 uses GPIO00, eMAC1 uses GPIO01 */
#define NFUNCEN4_OFFSET		0x1528
#define NFUNCEN4_GPIO_00_MASK	GENMASK(3, 0)
#define GPIO_00_FUNC		0
#define NFUNCEN4_GPIO_01_MASK	GENMASK(7, 4)
#define GPIO_01_FUNC		0

#define TC956X_SSREG_BRREG_REG_BASE		(0x00024000U)

#define TC956X_GLUE_LOGIC_BASE_OFST		(0x0002C000U)

#define TC956X_SSREG_K_PCICONF_021_021		(TC956X_SSREG_BRREG_REG_BASE \
						+ 0x000009E4U)
#define TC956X_SSREG_K_PCICONF_022_022		(TC956X_SSREG_BRREG_REG_BASE \
						+ 0x000009E8U)

#define TC956X_GLUE_SW_REG_ACCESS_CTRL		(TC956X_GLUE_LOGIC_BASE_OFST \
						+ 0x0000002CU)
#define SW_DSP1_ENABLE				BIT(1)
#define SW_DSP2_ENABLE				BIT(2)

#define ENABLE_CUT_THROUGH_ON_RX_PATH_MASK	0x1U
#define ENABLE_CUT_THROUGH_ON_TX_PATH_MASK	0x1U

/* MSIGEN Registers */

#define MSI_INT_TX_CH0		 3
#define MSI_INT_RX_CH0		11
#define MSI_INT_EXT_PHY		20

#ifdef TC956X_SW_MSI
#define MSI_INT_SW_MSI		24
#endif

//
// Code from tc956x_main.c in vendor driver
//

static void tc956x_reg_update(void __iomem *addr, u32 mask, u32 new)
{
	u32 old;
	u32 val;

	val = readl(addr);
	old = field_get(mask, val);
	if (old != new) {
		val &= ~mask;
		val |= field_prep(mask, new);
		writel(val, addr);
	}
}

/* XXX This shouldn't be required every time, and should go in the GPIO code */
static void tc956x_phy_reset_pin_config(struct tc956x_data *td)
{
	void __iomem *addr = td->sfr + NFUNCEN4_OFFSET;

	if (td->emac0)
		tc956x_reg_update(addr, NFUNCEN4_GPIO_00_MASK, GPIO_00_FUNC);
	else
		tc956x_reg_update(addr, NFUNCEN4_GPIO_01_MASK, GPIO_01_FUNC);

}

/**
 * tc956x_assert_phy_reset() - Assert or deassert the PHY resetn output
 *  @td: driver private structure
 *  @assert: true is assert the reset signal (drive low); false is deassert
 */
static int tc956x_assert_phy_reset(struct tc956x_data *td, bool assert)
{
	int ret;

	tc956x_phy_reset_pin_config(td);

	ret = gpiod_set_value(td->phy_reset, assert ? 0 : 1);
	if (ret)
		return ret;

	td->reset_asserted = assert;

	return 0;
}

//
// Code from tc956x_msigen.c in vendor driver
//

/**
 * tc956x_msigen_init() - Initialize and configure the MSIGEN module
 * @priv:	STMMAC driver private data pointer
 * @dev:	Net device pointer
 *
 * Configure clocks and resets, and sets the mask and interrupt source
 * to MSI vector mapping.
 */
static void tc956x_msigen_init(struct stmmac_priv *priv, struct net_device *dev)
{
	struct tc956x_data *td = priv->plat->bsp_priv;
	void __iomem *addr;
	void __iomem *base;
	u32 val;

	addr = td->sfr + NCLKCTRL0_OFFSET;

	/* XXX Is this conditional on eMAC0, or on the first to initialize? */
	if (td->emac0)
		val |= CLK0_MAC0_CORE_MASK;
	val |= CLK0_BUS_MASK;

	/* Enable MSIGEN Module */
	val = readl(addr);
	val |= TC956X_MSIGENCEN;
	writel(val, addr);

	addr = td->sfr + NRSTCTRL0_OFFSET;

	val = readl(addr);
	val &= ~TC956X_MSIGENSRST;	/* XXX Does this DEassert? */
	writel(val, addr);

	/* Initialize MSIGEN */

	base = td->sfr + MSIGEN_BASE(td->emac0 ? 0 : 1);
	writel(TC956X_MSI_OUT_EN_CLR, base + TC956X_MSI_OUT_EN_OFFSET);
	writel(TC956X_MSI_MASK_SET, base + TC956X_MSI_MASK_SET_OFFSET);
	writel(TC956X_MSI_MASK_CLR, base + TC956X_MSI_MASK_CLR_OFFSET);
	/* DMA Ch Tx-Rx Interrupt sources are assigned to Vector 0,
	 * All other Interrupt sources are assigned to Vector 1
	 */
	writel(TC956X_MSI_SET0, base + TC956X_MSI_VECT_SET0_OFFSET);
	writel(TC956X_MSI_SET1, base + TC956X_MSI_VECT_SET1_OFFSET);
	writel(TC956X_MSI_SET2, base + TC956X_MSI_VECT_SET2_OFFSET);
	writel(TC956X_MSI_SET3, base + TC956X_MSI_VECT_SET3_OFFSET);
	writel(TC956X_MSI_SET4, base + TC956X_MSI_VECT_SET4_OFFSET);
	writel(TC956X_MSI_SET5, base + TC956X_MSI_VECT_SET5_OFFSET);
	writel(TC956X_MSI_SET6, base + TC956X_MSI_VECT_SET6_OFFSET);
	writel(TC956X_MSI_SET7, base + TC956X_MSI_VECT_SET7_OFFSET);
}

static u32 tc956x_interrupt_sts(struct stmmac_priv *priv, struct net_device *dev)
{
	struct tc956x_data *td = priv->plat->bsp_priv;
	void __iomem *base;

	base = td->sfr + MSIGEN_BASE(td->emac0 ? 0 : 1);

	return readl(base + TC956X_MSI_INT_STS_OFFSET);
}

/**
 * tc956x_interrupt_en() - Enable or disable MSI interrupts
 * @priv:	STMMAC driver private data pointer
 * @dev:	Net device pointer
 * @en:		Whether to enable or disable MSI interrupts
 */
static void tc956x_interrupt_en(struct stmmac_priv *priv, struct net_device *dev, u32 en)
{
	struct tc956x_data *td = priv->plat->bsp_priv;
	void __iomem *base;
	u32 mask_val = 0;

#if 0
	// This table is copied from tc956xmac_main.c and is almost certainly
	// wrong in some way (subtle or otherwise)
	bool tx_ch_in_use[8];		bool rx_ch_in_use[8];
	tx_ch_in_use[0] = true;		rx_ch_in_use[0] = true;
	tx_ch_in_use[1] = false;	rx_ch_in_use[1] = false;
	tx_ch_in_use[2] = false;	rx_ch_in_use[2] = false;
	tx_ch_in_use[3] = false;	rx_ch_in_use[3] = true;
	tx_ch_in_use[4] = true;		rx_ch_in_use[4] = true;
	rx_ch_in_use[5] = true;		tx_ch_in_use[5] = true;
	rx_ch_in_use[6] = true;		tx_ch_in_use[6] = true;
	rx_ch_in_use[7] = true;		tx_ch_in_use[7] = true;
#endif

	base = td->sfr + MSIGEN_BASE(td->emac0 ? 0 : 1);
	if (en) {
		/*
		 * TODO: This logic was intended to avoid enabling interrupts
		 *       that shouldn't fire (and therefore won't be serviced
		 *       properly). When/if writing an irqchip driver it should
		 *       probably be ported... which is why isn't not been
		 *       deleted yet. The effect is that all interrupts are
		 *       jammed on.
		 */
#if 0
		u32 chan;
		/* Disable MSI for Tx/Rx channels that is not enabled in the Function */
		for (chan = 0; chan < priv->plat->tx_queues_to_use; chan++)
			if (!tx_ch_in_use[chan])
				mask_val |= (1 << (MSI_INT_TX_CH0 + chan));

		for (chan = 0; chan < priv->plat->rx_queues_to_use; chan++) {
			if (!rx_ch_in_use[chan])
				mask_val |= (1 << (MSI_INT_RX_CH0 + chan));
		}
		if (priv->dev->phydev != NULL) {
			/* PHY MSI interrupt enabled */
			mask_val &= ~(1 << MSI_INT_EXT_PHY);
		} else
			mask_val |= (1 << MSI_INT_EXT_PHY); /* Disable PHY interrupt on PHY absence */
#endif

		mask_val = TC956X_MSI_OUT_EN & (~mask_val);

		writel(mask_val, base + TC956X_MSI_OUT_EN_OFFSET);
	} else
		writel(TC956X_MSI_OUT_EN_CLR, base + TC956X_MSI_OUT_EN_OFFSET);
}

/**
 * tc956x_interrupt_clr() - Clear/acknowledge an MSI condition
 * @priv:	STMMAC driver private data pointer
 * @dev:	Net device pointer
 * @vector:	MSI number to clear
 *
 * Clear an interrupt condition for an MSI after handling it.
 */
static void tc956x_interrupt_clr(struct stmmac_priv *priv, struct net_device *dev, u32 vector)
{
	struct tc956x_data *td = priv->plat->bsp_priv;
	void __iomem *base;

	base = td->sfr + MSIGEN_BASE(td->emac0 ? 0 : 1);
	writel((1<<vector), base + TC956X_MSI_MASK_CLR_OFFSET);
}

const struct tc956x_msi_ops tc956x_msigen_ops = {
	.init		= tc956x_msigen_init,
	.interrupt_sts	= tc956x_interrupt_sts,
	.interrupt_en	= tc956x_interrupt_en,
	.interrupt_clr	= tc956x_interrupt_clr,
};

//
// Code from tc956x_qcom.c in vendor driver
//

static int tc956x_phy_power_on(struct tc956x_data *td)
{
	int ret = 0;

	ret = regulator_enable(td->phy_supply);
	if (ret) {
		dev_err(td->dev, "Failed to enable PHY supply with error %d\n", ret);
		return ret;
	}

	if (td->reset_asserted) {
		ret = tc956x_assert_phy_reset(td, false);
		if (ret) {
			(void)regulator_disable(td->phy_supply);
			dev_err(td->dev, "failed to deassert PHY reset\n");
			return ret;
		}

		msleep(td->phy_reset_delay);
	}

	return ret;
}

static int tc956x_phy_power_off(struct tc956x_data *td)
{
	int ret = 0;

	if (!td->reset_asserted) {
		ret = tc956x_assert_phy_reset(td, true);
		if (ret) {
			dev_err(td->dev, "failed to assert PHY reset\n");
			return ret;
		}
	}

	ret = regulator_disable(td->phy_supply);
	if (ret) {
		dev_err(td->dev, "Failed to disable PHY supply with error %d\n", ret);
		(void)tc956x_assert_phy_reset(td, false);
		/* XXX Any need for the phy_reset_delay here? */
	}

	return ret;
}

static int tc956x_reset_gpio_get(struct tc956x_data *td)
{
	struct device *dev = td->dev;
	struct device_node *np;
	int ret;

	np = dev_of_node(dev);
	if (!np)
		return -EINVAL;

	td->phy_reset = devm_gpiod_get(dev, "phy-reset", GPIOD_OUT_LOW);
	if (IS_ERR(td->phy_reset))
		return PTR_ERR(td->phy_reset);

	/* XXX Can we use a good constant and avoid having to specify this? * */
	ret = of_property_read_u32(np, "qcom,phy-reset-delay",
				   &td->phy_reset_delay);
	if (ret) {
		dev_err(dev, "failed to get qcom,phy-reset-delay property\n");
		return ret;
	}

	return 0;
}

static int tc956x_platform_of_parse(struct tc956x_data *td)
{
	struct device *dev = td->dev;
	struct device_node *np;
	int ret;

	np = dev_of_node(dev);
	if (!np)
		return -EINVAL;

	ret = tc956x_reset_gpio_get(td);
	if (ret)
		return ret;

	ret = of_irq_get_byname(np, "wake-on-lan");
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

	/* XXX Have to chase down why the following is necessary... */
	td->pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR(td->pinctrl)) {
		dev_err(dev, "failed to get pinctrl handle\n");
		return PTR_ERR(td->pinctrl);
	}

	td->pinctrl_default = pinctrl_lookup_state(td->pinctrl,
						   PINCTRL_STATE_DEFAULT);
	if (IS_ERR(td->pinctrl_default)) {
		dev_err(dev, "failed to look up default pinctrl state\n");
		return PTR_ERR(td->pinctrl_default);
	}

	return 0;
}

static int tc956x_platform_probe(struct tc956x_data *td,
				 struct stmmac_resources *res)
{
	int ret = 0;

	dev_dbg(td->dev, "QPS615 platform probing has started\n");

	ret = tc956x_platform_of_parse(td);
	if (ret)
		return ret;

	/* Hold the PHY in reset until we're ready */
	td->reset_asserted = false;
	ret = tc956x_assert_phy_reset(td, true);
	if (ret) {
		dev_err(td->dev, "failed to assert PHY reset\n");
		return ret;
	}

	ret = pinctrl_select_state(td->pinctrl, td->pinctrl_default);
	if (ret) {
		dev_err(td->dev, "Failed to select the 'default' pincrl state\n");
		goto err_pinctrl_select_state;
	}

	ret = tc956x_phy_power_on(td);
	if (ret) {
		dev_err(td->dev, "Failed to power on PHY with error %d\n", ret);
		goto err_power_on;
	}

	res->wol_irq = td->wol_irq;
	dev_dbg(td->dev, "QPS615 platform probing has finished successfully\n");

	return 0;

err_power_on:
	irq_set_irq_wake(td->wol_irq, 0);
err_pinctrl_select_state:
	return -EINVAL;
}

static int tc956x_platform_remove(struct tc956x_data *td)
{
	int ret = 0;

	dev_dbg(td->dev, "Freeing QPS615 platform resources\n");

	ret = tc956x_phy_power_off(td);
	if (ret)
		dev_err(td->dev, "Failed to power off PHY with error %d\n", ret);

	devm_regulator_put(td->phy_supply);

	devm_pinctrl_put(td->pinctrl);

	return ret;
}

static int tc956x_platform_suspend(struct stmmac_priv *priv)
{
	struct tc956x_data *td = priv->plat->bsp_priv;
	int ret = 0;

	if (priv->wolopts) {
		ret = enable_irq_wake(priv->wol_irq);
		if (unlikely(ret))
			dev_err(priv->device, "Failed to set WOL IRQ %d as wake up capable with error %d\n",
				priv->wol_irq, ret);
	} else {
		ret = tc956x_phy_power_off(td);
		if (ret)
			dev_err(priv->device, "Failed to power off PHY with error %d\n", ret);
	}

	return ret;
}

static int tc956x_platform_resume(struct stmmac_priv *priv)
{
	struct tc956x_data *td = priv->plat->bsp_priv;
	int ret = 0;


	if (priv->wolopts) {
		ret = disable_irq_wake(priv->wol_irq);
		if (unlikely(ret))
			dev_err(priv->device, "Failed to set WOL IRQ %d as a wake-disabled irq with error %d\n",
				priv->wol_irq, ret);
	} else {
		ret = tc956x_phy_power_on(td);
		if (ret)
			dev_err(priv->device, "Failed to power on the PHY with error %d\n", ret);
	}

	return ret;
}

//
// Code from tc956x_pma.h in vendor driver
//

#define XPCS_XGMAC_OFFSET  0x3A00
#define PMA_XGMAC_OFFSET   0x4000

/*PMA registers*/
#define XGMAC_PMA_GL_PM_CFG0				0x000001B8
#define XGMAC_PMA_CFG_0_1_R0				0x00001888
#define XGMAC_PMA_CFG_0_1_R1				0x00001890
#define XGMAC_PMA_CFG_0_1_R2				0x00001898
#define XGMAC_PMA_CFG_0_1_R3				0x000018A0
#define XGMAC_PMA_CFG_0_1_R4				0x000018A8

#define	XGMAC_PMA_HWT_REFCK_EN_R0			0x00001080
#define	XGMAC_PMA_HWT_REFCK_TERM_EN_R0		0x00001090
#define XGMAC_PMA_HWT_REFCK_R_EN_R1			0x00001094
#define XGMAC_PMA_HWT_REFCK_TERM_EN_R1		0x000010A4
#define XGMAC_PMA_HWT_REFCK_R_EN_R2			0x000010A8
#define XGMAC_PMA_HWT_REFCK_TERM_EN_R2		0x000010B8
#define XGMAC_PMA_HWT_REFCK_R_EN_R3			0x000010BC
#define XGMAC_PMA_HWT_REFCK_TERM_EN_R3		0x000010CC
#define XGMAC_PMA_HWT_REFCK_R_EN_R4			0x000010D0
#define XGMAC_PMA_HWT_REFCK_TERM_EN_R4		0x000010E0

/*PMA register values*/
#define XGMAC_PMA_OFFSET0					0x00000000
#define XGMAC_PMA_OFFSET1					0x0001EF04

//
// Code from tc956x_pma.c in vendor driver
//

static void tc956x_pma_init(struct tc956x_data *td)
{
	void __iomem *pmaaddr = XGMAC_BASE(td) + PMA_XGMAC_OFFSET;
	u32 val;

	/* Assertion of PMA reset  software Reset*/
	if (td->emac0) {
		val = readl(td->sfr + NRSTCTRL0_OFFSET);
		val |= RST0_MAC0PMARST;
		writel(val, td->sfr + NRSTCTRL0_OFFSET);
	} else {
		val = readl(td->sfr + NRSTCTRL1_OFFSET);
		val |= RST1_MAC1PMARST;
		writel(val, td->sfr + NRSTCTRL1_OFFSET);
	}

	/*Power on CML buffer*/
	val = readl(pmaaddr + XGMAC_PMA_GL_PM_CFG0);
	val = XGMAC_PMA_OFFSET0;
	writel(val, pmaaddr + XGMAC_PMA_GL_PM_CFG0);

	/*Switch clock from C0_REFCK to CLK_REF_I*/
	writel(XGMAC_PMA_OFFSET1, pmaaddr + XGMAC_PMA_CFG_0_1_R0);
	writel(XGMAC_PMA_OFFSET1, pmaaddr + XGMAC_PMA_CFG_0_1_R1);
	writel(XGMAC_PMA_OFFSET1, pmaaddr + XGMAC_PMA_CFG_0_1_R2);
	writel(XGMAC_PMA_OFFSET1, pmaaddr + XGMAC_PMA_CFG_0_1_R3);
	writel(XGMAC_PMA_OFFSET1, pmaaddr + XGMAC_PMA_CFG_0_1_R4);

	/*Disable C0_REFCK*/
	writel(XGMAC_PMA_OFFSET0, pmaaddr + XGMAC_PMA_HWT_REFCK_EN_R0);
	writel(XGMAC_PMA_OFFSET0, pmaaddr + XGMAC_PMA_HWT_REFCK_TERM_EN_R0);
	writel(XGMAC_PMA_OFFSET0, pmaaddr + XGMAC_PMA_HWT_REFCK_R_EN_R1);
	writel(XGMAC_PMA_OFFSET0, pmaaddr + XGMAC_PMA_HWT_REFCK_TERM_EN_R1);
	writel(XGMAC_PMA_OFFSET0, pmaaddr + XGMAC_PMA_HWT_REFCK_R_EN_R2);
	writel(XGMAC_PMA_OFFSET0, pmaaddr + XGMAC_PMA_HWT_REFCK_TERM_EN_R2);
	writel(XGMAC_PMA_OFFSET0, pmaaddr + XGMAC_PMA_HWT_REFCK_R_EN_R3);
	writel(XGMAC_PMA_OFFSET0, pmaaddr + XGMAC_PMA_HWT_REFCK_TERM_EN_R3);
	writel(XGMAC_PMA_OFFSET0, pmaaddr + XGMAC_PMA_HWT_REFCK_R_EN_R4);
	writel(XGMAC_PMA_OFFSET0, pmaaddr + XGMAC_PMA_HWT_REFCK_TERM_EN_R4);

	/* De-assertion of PMA reset  software Reset*/
	if (td->emac0) {
		val = readl(td->sfr + NRSTCTRL0_OFFSET);
		val &= ~RST0_MAC0PMARST;
		writel(val, td->sfr + NRSTCTRL0_OFFSET);
	} else {
		val = readl(td->sfr + NRSTCTRL1_OFFSET);
		val &= ~RST1_MAC1PMARST;
		writel(val, td->sfr + NRSTCTRL1_OFFSET);
	}

	/* TODO: Is this the right bit to poll for a PMA only reset? */
	WARN_ON(readl_poll_timeout(td->sfr + (td->emac0 ? NEMAC0CTL_OFFSET :
							  NEMAC1CTL_OFFSET),
				   val, val & EMAC_INIT_DONE, 50, 1000000));
}

//
// Code from tc956x_pci.c in vendor driver
//

struct tx956x_shrd_mem tx956x_pci_shrd_mem[TC956X_TOT_CASCADE_DEV];

static uint16_t tc956x_get_shared_mem_offset(struct pci_dev *pdev, uint16_t pci_bd)
{
	struct device *dev = &pdev->dev;
	uint16_t offset;

	for (offset = 0; offset < TC956X_TOT_CASCADE_DEV; offset++) {
		if (tx956x_pci_shrd_mem[offset].pci_bd == 0) {
			tx956x_pci_shrd_mem[offset].pci_bd = pci_bd;
			dev_dbg(dev, "New shared memory offset %d allocated\n", offset);
			return offset;	/* Free memory is available */
		} else if (tx956x_pci_shrd_mem[offset].pci_bd == pci_bd) {
			dev_dbg(dev, "Existing shared memory offset %d found\n", offset);
			return offset;	/* Allocated memory found */
		}
	}
	return 0xFFFF;
}

static int tc956x_chipcfg_mac_configure(struct tc956x_data *td, int speed)
{
	bool mac_312_clock = false, mac_125_clock = false;
	u32 nclkctrlx_offset, nemacxctl_offset;
	u32 macx312clken, macx125clken;
	u32 sp_sel, val;

	/*
	 * All paths much set sp_sel, any path that requires the 312/125
	 * clocks to be enabled must also set that appropriate booleans.
	 */
	switch (td->plat->phy_interface) {
	case PHY_INTERFACE_MODE_10GBASER:
		switch (speed) {
		case SPEED_10000:
			sp_sel = SPEED_USXGMII_10G_10G;
			break;
		default:
			return -ENOTSUPP;
		}
		break;
	case PHY_INTERFACE_MODE_SGMII:
		switch (speed) {
		case SPEED_2500:
			sp_sel = SPEED_SGMII_2500M;
			break;
		default:
			sp_sel = SPEED_SGMII_1000M;
			mac_125_clock = true;
			break;
		}
		break;
	case PHY_INTERFACE_MODE_2500BASEX:
		sp_sel = SPEED_SGMII_2500M;
		break;
	default:
		return -ENOTSUPP;
	}

	if (td->emac0) {
		nclkctrlx_offset = NCLKCTRL0_OFFSET;
		macx312clken = CLK0_MAC0312CLKEN;
		macx125clken = CLK0_MAC0125CLKEN;
		nemacxctl_offset = NEMAC0CTL_OFFSET;
	} else {
		nclkctrlx_offset = NCLKCTRL1_OFFSET;
		macx312clken = CLK1_MAC1312CLKEN;
		macx125clken = CLK1_MAC1125CLKEN;
		nemacxctl_offset = NEMAC1CTL_OFFSET;
	}

	val = readl(td->sfr + nclkctrlx_offset);
	FIELD_MODIFY(macx312clken, &val, mac_312_clock);
	FIELD_MODIFY(macx125clken, &val, mac_125_clock);
	writel(val, td->sfr + nclkctrlx_offset);

	val = readl(td->sfr + nemacxctl_offset);
	val |= EMAC_LPIHWCLKEN;
	val &= ~EMAC_INV_SGM_SIG_DET;
	FIELD_MODIFY(EMAC_PHY_INF_SEL_MASK, &val, PCS_CLK_PHY);
	FIELD_MODIFY(EMAC_SP_SEL_MASK, &val, sp_sel);
	writel(val, td->sfr + nemacxctl_offset);

	return 0;
}

static int tc956x_chipcfg_mac_init(struct tc956x_data *td)
{
	struct plat_stmmacenet_data *plat = td->plat;
	u32 val;


	if (td->emac0) {
		/* Assertion of EMAC Port0 software Reset */
		val = readl(td->sfr + NRSTCTRL0_OFFSET);
		val |= RST0_MAC0RST;
		writel(val, td->sfr + NRSTCTRL0_OFFSET);

		/* Enable all clocks to eMAC Port0 */
		val = readl(td->sfr + NCLKCTRL0_OFFSET);
		val |= CLK0_MAC0_IO_MASK;
		if (plat->phy_interface == PHY_INTERFACE_MODE_SGMII ||
		    plat->phy_interface == PHY_INTERFACE_MODE_2500BASEX)
			val &= ~CLK0_BUS_MASK;
		writel(val, td->sfr + NCLKCTRL0_OFFSET);

		/* Set the speed related registers */
		tc956x_chipcfg_mac_configure(td, td->plat->max_speed);

		/* De-assertion of EMAC Port0  software Reset*/
		val = readl(td->sfr + NRSTCTRL0_OFFSET);
		val &= ~RST0_MAC0RST;
		writel(val, td->sfr + NRSTCTRL0_OFFSET);
	} else {
		/* Assertion of EMAC Port1 software Reset*/
		val = readl(td->sfr + NRSTCTRL1_OFFSET);
		val |= RST1_MAC1RST;
		writel(val, td->sfr + NRSTCTRL1_OFFSET);

		/* Enable all clocks to eMAC Port1 */
		val = readl(td->sfr + NCLKCTRL1_OFFSET);
		val |= CLK1_MAC1_IO_MASK | CLK1_MAC1RMCEN;
		writel(val, td->sfr + NCLKCTRL1_OFFSET);

		/* Set the speed related registers */
		tc956x_chipcfg_mac_configure(td, td->plat->max_speed);

		/* De-assertion of EMAC Port1  software Reset */
		val = readl(td->sfr + NRSTCTRL1_OFFSET);
		val &= ~RST1_MAC1RST;
		writel(val, td->sfr + NRSTCTRL1_OFFSET);
	}

	return 0;
}

static void tc956x_chipcfg_xpcs_init(struct tc956x_data *td)
{
	u32 val;

	/* Take XPCS out of reset */
	if (td->emac0) {
		val = readl(td->sfr + NRSTCTRL0_OFFSET);
		val &= ~RST0_MAC0XPCSRST;
		writel(val, td->sfr + NRSTCTRL0_OFFSET);
	} else {
		val = readl(td->sfr + NRSTCTRL1_OFFSET);
		val &= ~RST1_MAC1XPCSRST;
		writel(val, td->sfr + NRSTCTRL1_OFFSET);
	}
}

/**
 * tc956x_pm_set_power() - Set clock and reset for suspend or resume
 * @priv:	STMMAC driver private data pointer
 * @suspend:	Whether we are being called during suspend
 *
 * Save the eMAC clock and reset settings before suspend, or restore those
 * settings during resume.
 */
static void tc956x_pm_set_power(struct stmmac_priv *priv, bool suspend)
{
	struct tc956x_data *td = priv->plat->bsp_priv;
	void __iomem *commonclk_reg;
	void __iomem *nrst_reg;
	void __iomem *nclk_reg;
	u32 nrst_mask;
	u32 nclk_mask;
	u32 val;

	/* Select register address by port */
	if (td->emac0) {
		nrst_reg = td->sfr + NRSTCTRL0_OFFSET;
		nrst_mask = RST0_MAC0RST | RST0_MAC0PMARST | RST0_MAC0XPCSRST;
		nclk_reg = td->sfr + NCLKCTRL0_OFFSET;
		nclk_mask = CLK0_MAC0_CORE_MASK | CLK0_MAC0_IO_MASK;
	} else {
		nrst_reg = td->sfr + NRSTCTRL1_OFFSET;
		nrst_mask = RST1_MAC1RST | RST1_MAC1PMARST | RST1_MAC1XPCSRST;
		nclk_reg = td->sfr + NCLKCTRL1_OFFSET;
		nclk_mask = CLK1_MAC1_CORE_MASK | CLK1_MAC1_IO_MASK;
		nclk_mask |= CLK1_MAC1RMCEN;
	}

	if (suspend) {
		/* Save current reset state, and assert resets */
		val = readl(nrst_reg);
		td->pm_saved_emac_rst = val & nrst_mask;
		val |= nrst_mask;
		writel(val, nrst_reg);

		/* Save current clock state, and disable clocks */
		val = readl(nclk_reg);
		td->pm_saved_emac_clk = val & nclk_mask;
		val &= ~nclk_mask;
		writel(val, nclk_reg);

		/* If zero are active, disable common clocks */
		if (!tx956x_pci_shrd_mem[td->pci_bd].pci_dev_active_cnt) {
			commonclk_reg = td->sfr + NCLKCTRL0_OFFSET;
			val = readl(commonclk_reg);
			val = val & ~CLK0_BUS_MASK;
			writel(val, commonclk_reg);
		}
	} else {
		/* If zero were active, re-enable common clocks */
		if (!tx956x_pci_shrd_mem[td->pci_bd].pci_dev_active_cnt) {
			commonclk_reg = td->sfr + NCLKCTRL0_OFFSET;
			val = readl(commonclk_reg);
			val = val | CLK0_BUS_MASK;
			writel(val, commonclk_reg);
		}

		/* Restore saved clock state */
		val = readl(nclk_reg);
		val = val | td->pm_saved_emac_clk;
		writel(val, nclk_reg);

		/* Restore saved reset state */
		val = readl(nrst_reg);
		/* XXX What's the point of clearing the non-mask bits? */
		val = (val & ~nrst_mask) | td->pm_saved_emac_rst;
		writel(val, nrst_reg);
	}
}

static void tc956x_get_interfaces(struct stmmac_priv *priv, void *bsp_priv,
				  unsigned long *interfaces)
{
	if (priv->plat->phy_interface != PHY_INTERFACE_MODE_SGMII)
		return;

	/*
	 * To handle 2.5G PHYs via (overclocked) SGMII then we need set both
	 * SGMII and 2500BASEX are supported interfaces.
	 */
	__set_bit(PHY_INTERFACE_MODE_SGMII, interfaces);
	__set_bit(PHY_INTERFACE_MODE_2500BASEX, interfaces);
}

static int tc956x_xgmac3_default_data(struct pci_dev *pdev,
				struct plat_stmmacenet_data *plat)
{
	struct tc956x_data *td = plat->bsp_priv;

	/* Set common default data first */
	plat->core_type = DWMAC_CORE_XGMAC;
	plat->force_sf_dma_mode = 1;
	plat->flags |= STMMAC_FLAG_TSO_EN;

	plat->pdev = pdev;
	plat->clk_ptp_rate = 50000000;

	/* For TC956X, clk_csr_i = 125MHz XXX any standard XGMAC values? */
	if (td->emac0)			/* emac0: XFI */
		plat->clk_csr = 0x4;	/* clk_csr_i / 12 XXX set CRS bit? */
	else				/* emac1: SGMII */
		plat->clk_csr = 0x0;	/* clk_csr_i / 62 */

	switch (plat->phy_interface) {
	case PHY_INTERFACE_MODE_10GBASER:
		plat->mac_port_sel_speed = 10000;
		plat->max_speed = 10000;
		break;
	case PHY_INTERFACE_MODE_SGMII:
	case PHY_INTERFACE_MODE_2500BASEX:
		plat->mac_port_sel_speed = 2500;
		plat->max_speed = 2500;
		break;
	default:
		dev_err(&pdev->dev, "Unexpected PHY interface mode\n");
		return -ENOTSUPP;
	}

	plat->get_interfaces = tc956x_get_interfaces;

	plat->unicast_filter_entries = 32;

	/*
	 * TODO:
	 * These values come from the QPS615 TRM. Sadly setting them results
	 * in a significant performance regression on iperf3 -R tests.
	 */
	//plat->tx_fifo_size = 46 * SZ_1K;
	//plat->rx_fifo_size = 46 * SZ_1K;

	plat->dma_cfg->pbl = 32;
	plat->dma_cfg->pblx8 = true;

	plat->rx_queues_to_use = 8;
	plat->rx_sched_algorithm = MTL_RX_ALGORITHM_SP;

	for (int i = 0; i < plat->rx_queues_to_use; i++) {
		/* Copied from socfpga_agilex5.dtsi */
		plat->rx_queues_cfg[i].mode_to_use = MTL_QUEUE_DCB;
	}

	/*
	 * TODO: tx_queues_to_use would normally be set to 8. However functional
	 *       reliability becomes poor (DHCP fails to get IP address or, if
	 *       it gets an address, ping does not work) if tx_queues_to_use >3
	 */
	plat->tx_queues_to_use = 3;
	plat->tx_sched_algorithm = MTL_TX_ALGORITHM_WRR;

	for (int i = 0; i < plat->tx_queues_to_use; i++) {
		/* Copied from socfpga_agilex5.dtsi */
		plat->tx_queues_cfg[i].weight = 9 + i;
		plat->tx_queues_cfg[i].mode_to_use = MTL_QUEUE_DCB;

		/* Tx Queues 0 - 4 doesn't support TBS on TC956x */
		if (i >= 5)
			plat->tx_queues_cfg[i].tbs_en = true;
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
	u32 val;
	u32 i;

	/* This is value assigned to the TRSL_PARAM register */
	trsf_param_val = FIELD_PREP(TRSL_ID_MASK, TRSL_ID_PCIE_TX_RX);
	trsf_param_val |= FIELD_PREP(TRSF_PARAM_MASK, 0);

	base = td->bridge_config + AXI4_SLV_BASE(td->emac0 ? 0 : 1);

	/* Set all entries to default values */
	for (i = 0; i < CM3_TAMAP_COUNT; i++) {
		writel(TC956X_AXI4_SLV00_SRC_ADDR_LO_VAL_DEFAULT,
		       base + SRC_ADDR_LO_OFFSET);
		writel(0x0, base + SRC_ADDR_HI_OFFSET);
		writel(0x0, base + TRSL_ADDR_LO_OFFSET);
		writel(0x0, base + TRSL_ADDR_HI_OFFSET);
		writel(trsf_param_val, base + TRSL_PARAM_OFFSET);
		/* XXX Not initializing the TRSL_MASK value? */
	}

	/* AXI4 Slave 0 - Table 0 Entry */
	/* EDMA address region 0x10 0000 0000 - 0x1F FFFF FFFF is
	 * translated to 0x0 0000 0000 - 0xF FFFF FFFF
	 */
	BUILD_BUG_ON(TC956X_AXI4_SLV00_SRC_ADDR & ATR_IMPL_MASK);
	BUILD_BUG_ON(FIELD_GET(ATR_SIZE_MASK, TC956X_AXI4_SLV00_SRC_ADDR));
	BUILD_BUG_ON(TC956X_AXI4_SLV00_ATR_SIZE < 12);

	base = td->bridge_config + AXI4_SLV_BASE(0);

	val = lower_32_bits(TC956X_AXI4_SLV00_SRC_ADDR);
	val |= FIELD_PREP(ATR_SIZE_MASK, TC956X_AXI4_SLV00_ATR_SIZE - 1);
	val |= ATR_IMPL_MASK;
	writel(val, base + SRC_ADDR_LO_OFFSET);

	val = upper_32_bits(TC956X_AXI4_SLV00_SRC_ADDR);
	writel(val, base + SRC_ADDR_HI_OFFSET);

	val = lower_32_bits(TC956X_AXI4_SLV00_TRSL_ADDR);
	writel(val, base + TRSL_ADDR_LO_OFFSET);

	val = upper_32_bits(TC956X_AXI4_SLV00_TRSL_ADDR);
	writel(val, base + TRSL_ADDR_HI_OFFSET);
	writel(trsf_param_val, base + TRSL_PARAM_OFFSET);

	pr_debug("SRC_ADDR HI = 0x%08x\n", readl(base + SRC_ADDR_HI_OFFSET));
	pr_debug("SRC_ADDR LO = 0x%08x\n", readl(base + SRC_ADDR_LO_OFFSET));
	pr_debug("TRSL_ADDR_HI = 0x%08x\n", readl(base + TRSL_ADDR_HI_OFFSET));
	pr_debug("TRSL_ADDR_LO = 0x%08x\n", readl(base + TRSL_ADDR_LO_OFFSET));
	pr_debug("TRSL_PARAM = 0x%08x\n", readl(base + TRSL_PARAM_OFFSET));
	pr_debug("TRSL_MASK_HI = 0x%08x\n", readl(base + TRSL_MASK_HI_OFFSET));
	pr_debug("TRSL_MASK_LO = 0x%08x\n", readl(base + TRSL_MASK_LO_OFFSET));
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

	WARN_ON(tc956x_chipcfg_mac_configure(td, speed));
	tc956x_pma_init(td);
}

static const struct mfd_cell tc956x_mfd_cells[] = {
	{ .name = "tc956x-gpio", },
	{ .name = "tc956x-reset-controller", },
};

static int tc956x_devm_mfd_init(struct tc956x_chip *tc)
{
	struct device *dev = tc->primary->dev;
	void __iomem *regs = tc->primary->sfr;
	struct regmap *regmap;

	/* Note: no need to check for errors on read/write for MMIO regmap */
	regmap = devm_regmap_init_mmio(dev, regs, &tc956x_gpio_regmap_config);
	if (IS_ERR(regmap))
		return PTR_ERR(regmap);

	regmap = devm_regmap_init_mmio(dev, regs,
				       &tc956x_clk_reset_regmap_config);
	if (IS_ERR(regmap))
		return PTR_ERR(regmap);

	return devm_mfd_add_devices(dev, PLATFORM_DEVID_AUTO,
				    tc956x_mfd_cells,
				    ARRAY_SIZE(tc956x_mfd_cells),
				    NULL, 0, NULL);
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
	plat->fix_mac_speed = tc956x_fix_mac_speed;
	plat->pcs_init = tc956x_pcs_init;
	plat->select_pcs = tc956x_select_pcs;

	/* XXX We don't initialize this; what is required? */
	plat->mdio_bus_data = devm_kzalloc(dev, sizeof(*plat->mdio_bus_data),
					   GFP_KERNEL);
	if (!plat->mdio_bus_data)
		return NULL;

	/* XXX We initialize two (four) fields here; what is required? */
	plat->dma_cfg = devm_kzalloc(dev, sizeof(*plat->dma_cfg), GFP_KERNEL);
	if (!plat->dma_cfg)
		return NULL;

	return plat;
}

static struct tc956x_data *tc956x_devm_data_create(struct pci_dev *pdev)
{
	struct device *dev = &pdev->dev;
	struct tc956x_data *td;
	void __iomem *virt;
	int ret;

	td = devm_kzalloc(dev, sizeof(*td), GFP_KERNEL);
	if (!td)
		return NULL;

	td->plat = tc956x_plat_dat_alloc(td, pdev);
	if (!td->plat)
		return NULL;

	td->dev = dev;
	td->devfn = pdev->devfn;

	ret = pcim_enable_device(pdev);
	if (ret) {
		dev_err(dev, "%s: ERROR: failed to enable device\n", __func__);
		return ERR_PTR(ret);
	}

	/* Request the PCI IO Memory for the device */
	virt = pcim_iomap_region(pdev, PCI_BAR_BRIDGE_CONFIG, DRIVER_NAME);
	if (IS_ERR(virt)) {
		dev_err(dev, "failed to map bridge config region\n");
		return ERR_CAST(virt);
	}
	td->bridge_config = virt;

	virt = pcim_iomap_region(pdev, PCI_BAR_SFR, DRIVER_NAME);
	if (IS_ERR(virt)) {
		dev_err(dev, "failed to map sfr region\n");
		return ERR_CAST(virt);
	}
	td->sfr = virt;

	td->regmap = devm_regmap_init_mmio(dev, virt, &tc956x_regmap_config);
	if (IS_ERR(td->regmap)) {
		dev_err(dev, "failed to initialize regmap\n");
		return ERR_CAST(td->regmap);
	}

	dev_dbg(dev, "BAR0 physical address = 0x%llx length 0x%llx\n",
		(u64)pci_resource_start(pdev, 0),
		(u64)pci_resource_len(pdev, 0));
	dev_dbg(dev, "BAR2 physical address = 0x%llx length 0x%llx\n",
		(u64)pci_resource_start(pdev, 2),
		(u64)pci_resource_len(pdev, 2));
	dev_dbg(dev, "BAR4 physical address = 0x%llx length 0x%llx\n",
		(u64)pci_resource_start(pdev, 4),
		(u64)pci_resource_len(pdev, 4));

	return td;
}

static struct tc956x_chip *tc956x_chip_get(struct tc956x_data *td)
{
	u8 pci_bus_num = PCI_BUS_NUM(td->devfn);
	u8 pci_slot = PCI_SLOT(td->devfn);
	struct device *dev = td->dev;
	struct tc956x_chip *tc;
	int ret;

	/* Use the existing chip structure if it's already been created */
	list_for_each_entry(tc, &tc956x_chips, links) {
		if (tc->pci_bus_num != pci_bus_num)
			continue;
		if (tc->pci_slot != pci_slot)
			continue;

		/* Make sure the secondary hasn't already been recorded */
		if (WARN_ON(tc->secondary))
			return ERR_PTR(-EINVAL);

		tc->secondary = td;

		return tc;
	}

	/* We need a new chip structure */
	tc = devm_kzalloc(dev, sizeof(*tc), GFP_KERNEL);
	if (!tc)
		return NULL;

	tc->pci_bus_num = pci_bus_num;
	tc->pci_slot = pci_slot;
	tc->primary = td;

	ret = tc956x_devm_mfd_init(tc);
	if (ret)
		return ERR_PTR(ret);

	list_add(&tc->links, &tc956x_chips);

	return tc;
}

static void tc956x_chip_put(struct tc956x_data *td)
{
	struct tc956x_chip *tc = td->chip;

	td->chip = NULL;

	/* The primary interface needs to be the last to go */
	if (tc->secondary) {
		if (td == tc->secondary)
			tc->secondary = NULL;
		return;
	}

	/*
	 * XXX This logic is insufficient.  We need to control when the
	 * XXX memory that the chip structure will point to gets unmapped.
	 * XXX We'll deal with this later.
	 */
	list_del(&tc->links);

	tc->primary = NULL;
	tc->pci_slot = 0;
	tc->pci_bus_num = 0;

	kfree(tc);
}

/**
 * tc956x_pci_probe() - PCI driver probe callback
 * @pdev:	PCI device pointer
 * @id:		Pointer to the matching PCI device ID table entry
 *
 * Set up a PCI device whose VID/PID match what we support.  This includes
 * allocating a driver private structure, requesting memory regions,
 * setting up interrupt handling, and so on.
 */
static int tc956x_pci_probe(struct pci_dev *pdev,
			    const struct pci_device_id *id)
{
	struct stmmac_resources res = { };
	struct device *dev = &pdev->dev;
	struct net_device *netdev;
	struct stmmac_priv *priv;
	struct tc956x_data *td;
	/* use signal from EMSPHY */
	uint16_t sh_mem_offset;
	bool mode2;
	u32 pfn;
	u32 val;
	int ret;

	td = tc956x_devm_data_create(pdev);
	if (IS_ERR_OR_NULL(td))
		return td ? PTR_ERR(td) : -ENOMEM;

	pci_set_master(pdev);

	td->chip = tc956x_chip_get(td);
	if (IS_ERR_OR_NULL(td->chip)) {
		ret = td->chip ? PTR_ERR(td->chip) : -ENOMEM;
		goto err_clear_master;
	}

#if IS_ENABLED(CONFIG_TRACE_MMIO_ACCESS)
	/*
	 *  TODO: This is the filtering/tagging support for MMIO tracing.
	 *
	 * Eventually it needs to be removed but not yet... it's too useful
	 * for feature development!
	 */
	log_mmio_register_range(td->bridge_config, pci_resource_len(pdev, 0),
				"bridge_cfg");
	log_mmio_register_range(td->sfr, pci_resource_len(pdev, 4), "sfr");
#endif

	/* The physical port number matches from the PCI function number */
	pfn = PCI_FUNC(pdev->devfn);
	if (WARN_ON(pfn > 1)) {
		ret = -EINVAL;
		goto err_chip_put;
	}
	td->emac0 = pfn == 0;

	// NCID_OFFSET gives the revision ID (and early revisions are limited
	// to 2.5G)
	pr_debug("NCID Register value: %x\n", readl(td->sfr + NCID_OFFSET));

	// TODO: this needs to come from devicetree
	td->plat->phy_interface = td->emac0 ? PHY_INTERFACE_MODE_10GBASER :
					      PHY_INTERFACE_MODE_SGMII;

	ret = tc956x_xgmac3_default_data(pdev, td->plat);
	if (ret)
		goto err_chip_put;

	/* XXX eMAC0, or first one probed? */
	if (td->emac0)
		tc956x_config_tamap(td);

	/* Enable MSI  and Allocate Vectors */
	ret = pci_alloc_irq_vectors(pdev, TC956X_TOT_MSI_VEC,
				TC956X_TOT_MSI_VEC, PCI_IRQ_MSI);

	if (ret < TC956X_TOT_MSI_VEC) {
		dev_err(dev, "%s:Enable MSI error\n", DRIVER_NAME);
		goto err_out_msi_failed;
	}

	dev_dbg(dev, "%s : Allocated MSI Vectors : %d", __func__, ret);
	dev_dbg(dev, "%s : pdev->irq %d  pci_irq_vector %d\n",
		__func__, pdev->irq, pci_irq_vector(pdev, 0));
	pci_write_config_dword(pdev, pdev->msi_cap + PCI_MSI_MASK_64, 0);

	if (td->emac0) {
		void __iomem *addr = td->sfr + NRSTCTRL0_OFFSET;

		val = readl(addr);
		val |= RST0_MCU_MASK;	/* Keep the M3 in reset */
		writel(val, addr);
	}

	res.addr = XGMAC_BASE(td);
	res.wol_irq = pdev->irq;
	res.irq = pdev->irq;

	sh_mem_offset = tc956x_get_shared_mem_offset(pdev, pci_dev_id(pdev) & TC956X_PCI_BD_MASK);
	if (sh_mem_offset < TC956X_TOT_CASCADE_DEV) {
		td->pci_bd  = sh_mem_offset;
	} else {
		dev_err(dev, "Error finding shared memory\n");
		goto err_out_msi_failed;
	}

	ret = tc956x_platform_probe(td, &res);
	if (ret) {
		dev_err(dev, "Platform (DT) code failed\n");
		goto err_platform_probe;
	}

	ret = tc956x_chipcfg_mac_init(td);
	if (ret < 0)
		goto err_platform_probe;

	tc956x_pma_init(td);
	tc956x_chipcfg_xpcs_init(td);

	ret = stmmac_dvr_probe(dev, td->plat, &res);
	if (ret) {
		void __iomem *nrst_reg;
		void __iomem *nclk_reg;
		u32 nrst_mask;
		u32 nclk_mask;
		u32 val;

		if (td->emac0) {
			nrst_reg = td->sfr + NRSTCTRL0_OFFSET;
			nrst_mask = RST0_MAC0RST | RST0_MAC0PMARST | RST0_MAC0XPCSRST;
			nclk_reg = td->sfr + NCLKCTRL0_OFFSET;
			nclk_mask = CLK0_MAC0_CORE_MASK | CLK0_MAC0_IO_MASK;
		} else {
			nrst_reg = td->sfr + NRSTCTRL1_OFFSET;
			nrst_mask = RST1_MAC1RST | RST1_MAC1PMARST | RST1_MAC1XPCSRST;
			nclk_reg = td->sfr + NCLKCTRL1_OFFSET;
			nclk_mask = CLK1_MAC1_CORE_MASK | CLK1_MAC1_IO_MASK;
			nclk_mask |= CLK1_MAC1RMCEN;
		}

		/* Assert resets */
		val = readl(nrst_reg);
		val |= nrst_mask;
		writel(val, nrst_reg);

		/* Disable clocks */
		val = readl(nclk_reg);
		val &= ~nclk_mask;
		writel(val, nclk_reg);

		if (ret != -ENODEV)
			goto err_dvr_probe;
	}

	/*
	 * Install the MSI ops. This is only needed until we have a proper
	 * irqchip driver for msigen)
	 */
	netdev = dev_get_drvdata(dev);
	priv = netdev_priv(netdev);
	priv->hw->msi = &tc956x_msigen_ops;

	dev_dbg(dev, "<--%s : Adding DSP Cut Through Settings", __func__);

	/*
	 * Determine the switch configuration from the MODE2 bit in the
	 * mode status register (number of lanes per port):
	 *   0: setting A: upstream x4, downstream 1 x1, downstream 2 x1
	 *   1: setting B: upstream x2, downstream 1 x2, downstream 2 x1
	 */
	val = readl(td->sfr + NMODESTS_OFFSET);
	mode2 = !!(val & NMODESTS_MODE2);
	dev_dbg(dev, "%s : Setting %c : Adding DSP Cut Through Settings for %sDSP2",
		__func__, mode2 ? 'B' : 'A', mode2 ? "" : "DSP1 & ");

	val = readl(td->sfr + TC956X_GLUE_SW_REG_ACCESS_CTRL);
	val |= SW_DSP2_ENABLE;
	if (!mode2)
		val |= SW_DSP1_ENABLE;
	writel(val, td->sfr + TC956X_GLUE_SW_REG_ACCESS_CTRL);

	/*Set 0x0 to Rx Bit enable_cut_through_on_receive_path*/
	val = readl(td->sfr + TC956X_SSREG_K_PCICONF_021_021);
	val &= ~ENABLE_CUT_THROUGH_ON_RX_PATH_MASK;
	writel(val, td->sfr + TC956X_SSREG_K_PCICONF_021_021);

	/*Set 0x00000000 to Tx Bit enable_cut_through_on_transmit_path*/
	val = readl(td->sfr + TC956X_SSREG_K_PCICONF_022_022);
	val &= ~ENABLE_CUT_THROUGH_ON_TX_PATH_MASK;
	writel(val, td->sfr + TC956X_SSREG_K_PCICONF_022_022);

	/* Increment device usage counter */
	tx956x_pci_shrd_mem[td->pci_bd].pci_dev_active_cnt++;

	return 0;

err_dvr_probe:
	(void)tc956x_platform_remove(td);
err_platform_probe:
err_out_msi_failed:
	pci_free_irq_vectors(pdev);
err_chip_put:
	tc956x_chip_put(td);
err_clear_master:
	pci_clear_master(pdev);

	return ret;
}

/**
 * tc956x_pci_remove() - PCI driver remove callback
 * @pdev: Pointer to the pci_dev structure
 *
 * Reverse operations performed at probe time, releasing resources
 * and returning things to original state.
 */
static void tc956x_pci_remove(struct pci_dev *pdev)
{
	struct device *dev = &pdev->dev;
	struct net_device *ndev = dev_get_drvdata(dev);
	struct stmmac_priv *priv = netdev_priv(ndev);
	struct tc956x_data *td = priv->plat->bsp_priv;
	void __iomem *nrst_reg;
	void __iomem *nclk_reg;
	u32 nrst_val;
	u32 nclk_val;

	/* phy_addr == -1 indicates that PHY was not found and
	 * device is registered as only PCIe device. So skip any
	 * ethernet device related uninitialization
	 */
	if (priv->dma_cap.sma_mdio == 1) {
		if (priv->plat->phy_addr != -1) {
			stmmac_dvr_remove(dev);
			tc956x_platform_remove(td);
		}
	} else {
		stmmac_dvr_remove(dev);
		tc956x_platform_remove(td);
	}

	/* Set reset value for CLK control and RESET Control registers */
	if (td->emac0) {
		nrst_reg = td->sfr + NRSTCTRL0_OFFSET;
		nrst_val = readl(nrst_reg);
		nrst_val |= RST0_MAC0XPCSRST | RST0_MAC0PMARST | RST0_MAC0RST;
		writel(nrst_val, nrst_reg);

		nclk_reg = td->sfr + NCLKCTRL0_OFFSET;
		nclk_val = readl(nclk_reg);
		nclk_val &= ~CLK0_MAC0_CORE_MASK;
		nclk_val &= ~CLK0_MAC0_IO_MASK;
		writel(nclk_val, nclk_reg);
	} else {
		nrst_reg = td->sfr + NRSTCTRL1_OFFSET;
		nrst_val = RST1_MAC1RST | RST1_MAC1PMARST | RST1_MAC1XPCSRST;
		writel(nrst_val, nrst_reg);

		nclk_reg = td->sfr + NCLKCTRL1_OFFSET;
		nclk_val = 0;
		writel(nclk_val, nclk_reg);
	}

	/* If exactly one MAC is in use... */
	if (tx956x_pci_shrd_mem[td->pci_bd].pci_dev_active_cnt == 1) {
		/* Set reset value for Common CLK control and Common RESET Control registers */
		nrst_reg = td->sfr + NRSTCTRL0_OFFSET;
		nrst_val = readl(nrst_reg);
		nrst_val |= RST0_MCU_MASK | RST0_INTRST | RST0_OTHER_MASK;
		writel(nrst_val, nrst_reg);

		nclk_reg = td->sfr + NCLKCTRL0_OFFSET;
		nclk_val = readl(nclk_reg);
		nclk_val |= CLK0_COMMON_MASK;
		nclk_val &= ~(CLK0_OTHER_MASK | CLK0_BUS_MASK | CLK0_INTCEN);
		nclk_val &= ~(CLK0_MAC0_CORE_MASK | CLK0_MAC0_IO_MASK);
		writel(nclk_val, nclk_reg);
	}
	pr_debug("%s : Port %d %s Wr RST Reg:%x, CLK Reg:%x", __func__,
		 td->emac0 ? 0 : 1, priv->dev->name,
		 readl(nrst_reg), readl(nclk_reg));

	pdev->irq = 0;

	/* Free allocated interrupt vectors for device */
	pci_free_irq_vectors(pdev);

	td->sfr = NULL;
	td->bridge_config = NULL;

	/* Decrement device usage counter */
	tx956x_pci_shrd_mem[td->pci_bd].pci_dev_active_cnt--;
}

/**
 * tc956x_pcie_pm_enable_pci() - Enable a PCI device
 * @pdev:	Pointer to the PCI device to enable
 *
 * Enable the PCI device passed as argument.
 *
 * Return:	0 if successful, or an error code if setting power state fails
 */
static int tc956x_pcie_pm_enable_pci(struct pci_dev *pdev)
{
	int ret;

	pci_set_power_state(pdev, PCI_D0);

	ret = pci_enable_device_mem(pdev);
	if (ret) {
		dev_err(&pdev->dev, "error %d enabling PCI device memory", ret);
		return ret;
	}

	pci_restore_state(pdev);
	pci_set_master(pdev);

	return 0;
}

/**
 * tc956x_pcie_pm_disable_pci() - Disable a PCI device
 * @pdev:	Pointer to the PCI device to disable
 *
 * Disable the PCI device passed as argument.
 */
static void tc956x_pcie_pm_disable_pci(struct pci_dev *pdev)
{
	pci_save_state(pdev);
	pci_disable_device(pdev);
	pci_prepare_to_sleep(pdev);
}

/**
 * tc956x_pcie_pm_pci() - Disable PCIe child devices
 * @pdev:	Pointer to the PCI device whose children are affected
 * @suspend:	Whether we are being called during suspend
 *
 * Disable PCI devices that are children of the given PCI device.
 *
 * Return:	0 if successful, or an error code if an error occurs
 */
static int tc956x_pcie_pm_pci(struct pci_dev *pdev, bool suspend)
{
	struct net_device *ndev = dev_get_drvdata(&pdev->dev);
	struct stmmac_priv *priv = netdev_priv(ndev);
	struct tc956x_data *td = priv->plat->bsp_priv;
	struct pci_dev *tc956x_port_pdev[2] = { };
	struct pci_dev *tc956x_dsp_ep;
	struct pci_dev *tc956x_pd;
	struct pci_bus *bus;
	int i = 0;
	int ret;
	int p;

	/* Zero active means are suspended */
	if (!tx956x_pci_shrd_mem[td->pci_bd].pci_dev_active_cnt) {
		tc956x_dsp_ep = pci_upstream_bridge(pdev);
		bus = tc956x_dsp_ep->subordinate;

		if (bus)
			list_for_each_entry(tc956x_pd, &bus->devices, bus_list)
				tc956x_port_pdev[i++] = tc956x_pd;

		for (p = 0; ((p < i) && (tc956x_port_pdev[p] != NULL)); p++) {
			/* Enter only if at least 1 Port Suspended */
			if (suspend) {
				tc956x_pcie_pm_disable_pci(tc956x_port_pdev[p]);
			} else {
				ret = tc956x_pcie_pm_enable_pci(tc956x_port_pdev[p]);
				if (ret < 0)
					return ret;
			}
		}
	}

	return 0;
}

/**
 * tc956x_pcie_suspend() - Device driver suspend callback
 * @dev:	Device pointer
 *
 * Perform the activities required to suspend the TC956x platform device.
 * This includes suspending the eMACs (and managing wake-on-LAN state)
 * and suspending the PCIe interfaces.
 *
 * Return:	0 if successful, or an error code if an error occurs
 */
static int tc956x_pcie_suspend(struct device *dev)
{
	struct pci_dev *pdev = to_pci_dev(dev);
	struct net_device *ndev = dev_get_drvdata(dev);
	struct stmmac_priv *priv = netdev_priv(ndev);
	struct tc956x_data *td = priv->plat->bsp_priv;
	int ret;

	tx956x_pci_shrd_mem[td->pci_bd].pci_dev_active_cnt--;

	stmmac_suspend(dev);
	ret = tc956x_platform_suspend(priv);
	if (ret) {
		dev_err(dev, "%s: error in calling tc956x_platform_suspend", pci_name(pdev));
		return ret;
	}

	tc956x_pm_set_power(priv, true);

	return tc956x_pcie_pm_pci(pdev, true);
}

/**
 * tc956x_pcie_resume_config() - Restore device configuration during resume
 * @pdev:	PCI device pointer
 *
 * Restore the state of the eMAC to functional state during resume.
 *
 * Return:	0 if successful, or an error code if an error occurs
 */
static int tc956x_pcie_resume_config(struct pci_dev *pdev)
{
	struct device *dev = &pdev->dev;
	struct net_device *ndev = dev_get_drvdata(dev);
	struct stmmac_priv *priv = netdev_priv(ndev);
	struct tc956x_data *td = priv->plat->bsp_priv;
	int ret = 0;

	/* Skip Config when Port unavailable */
	if (priv->dma_cap.sma_mdio == 1) {
		if ((priv->plat->phy_addr == -1) || (priv->mii == NULL)) {
			dev_dbg(dev, "%s : Invalid PHY Address (%d)\n", __func__, priv->plat->phy_addr);
			ret = -1;
			goto err_phy_addr;
		}
	}

	ret = tc956x_chipcfg_mac_init(td);
	WARN_ON(ret);

	tc956x_pma_init(td);
	tc956x_chipcfg_xpcs_init(td);

	return 0;

err_phy_addr:
	return ret;
}

/**
 * tc956x_pcie_resume() - Device driver resume callback
 * @dev:	Device pointer
 *
 * Perform the activities required to resume the TC956x platform device.
 * This includes resuming the PCIe interfaces, and disabling wake-on-LAN
 * and resuming the eMACs.
 *
 * Return:	0 if successful, or an error code if an error occurs
 */
static int tc956x_pcie_resume(struct device *dev)
{
	struct pci_dev *pdev = to_pci_dev(dev);
	struct net_device *ndev = dev_get_drvdata(dev);
	struct stmmac_priv *priv = netdev_priv(ndev);
	struct tc956x_data *td = priv->plat->bsp_priv;
	int ret;

	ret = tc956x_pcie_pm_enable_pci(pdev);
	if (ret < 0)
		return ret;

	tc956x_pm_set_power(priv, false);

	/* XXX Error handling in this function needs work */
	ret = tc956x_assert_phy_reset(td, td->reset_asserted);
	if (ret) {
		dev_err(dev, "error restoring PHY reset state");
		pci_disable_device(pdev);

		return ret;
	}

	ret = tc956x_platform_resume(priv);
	if (ret) {
		dev_err(dev, "%s: error in calling tc956x_platform_resume", pci_name(pdev));
		pci_disable_device(pdev);

		return ret;
	}

	/* Configure TA map registers: zero active means are suspended */
	if (!tx956x_pci_shrd_mem[td->pci_bd].pci_dev_active_cnt)
		tc956x_config_tamap(td);

	/* Configure EMAC Port */
	tc956x_pcie_resume_config(pdev);

	/* Call stmmac_resume() */
	stmmac_resume(dev);

	/* Increment device usage counter */
	tx956x_pci_shrd_mem[td->pci_bd].pci_dev_active_cnt++;

	return 0;
}

static const struct pci_device_id tc956x_id_table[] = {
	{ PCI_DEVICE(PCI_VENDOR_ID_TOSHIBA, PCI_DEVICE_ID_TOSHIBA_TC956X), },
	{ },
};

// TODO: During development it is very convenient to avoid auto-loading the
//       module if the vendor driver is also enabled.
#if !IS_ENABLED(CONFIG_TC956X_NET)
MODULE_DEVICE_TABLE(pci, tc956x_id_table);
#endif

static DEFINE_SIMPLE_DEV_PM_OPS(tc956x_pm_ops,
				tc956x_pcie_suspend,
				tc956x_pcie_resume);

static struct pci_driver tc956x_pci_driver = {
	.name		= DRIVER_NAME,
	.id_table	= tc956x_id_table,
	.probe		= tc956x_pci_probe,
	.remove		= tc956x_pci_remove,
	.driver		= {
		.name	= DRIVER_NAME,
		.owner	= THIS_MODULE,
		.pm	= pm_sleep_ptr(&tc956x_pm_ops),
	},
};

module_pci_driver(tc956x_pci_driver);

MODULE_DESCRIPTION("Toshiba TC956x PCIe Ethernet Network Driver");
MODULE_LICENSE("GPL");
