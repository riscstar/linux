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
#include <linux/pinctrl/consumer.h>
#include <linux/phy.h>
#include <linux/regulator/consumer.h>
#include <linux/of_irq.h>
#include <linux/delay.h>
#include "stmmac.h"
#include "dwxgmac2.h"
#include "common.h"

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
 * @plat:		Pointer to our stmmac platform data
 * @bridge_config:	Mapped bridge config data (BAR 0)
 * @sfr:		Mapped SFR region (BAR 4)
 * @emac0:		Which eMAC port this is (true: port 0; false: port 1
 * @is_sgmii_2p5g:	True if PHY uses SGMII and operating at 2.5 Gbps
 * @port_interface:	Operating more of the port (XFI or SGMII)
 * @pm_saved_emac_rst:	Saved eMAC reset control register value
 * @pm_saved_emac_clk:	Saved eMAC clock control register value
 * @pci_bd:		PCIe bus and device ID
 * @pinctrl:		Pin control structure
 * @pinctrl_default:	Pin control default value
 * @phy_supply:		PHY supply egulator
 * @phy_reset_gpio:	GPIO used for PHY reset
 * @phy_reset_delay:	Delay (milliseconds) after PHY reset
 * @reset_asserted:	Whether reset on this PHY is currently asserted
 * @wol_irq:		Wake-on-LAN IRQ number
 */
struct tc956x_data {
	struct device *dev;
	struct plat_stmmacenet_data *plat;
	void __iomem *bridge_config;
	void __iomem *sfr;
	bool emac0;
	bool is_sgmii_2p5g;
	u32 port_interface;
	u32 pm_saved_emac_rst;
	u32 pm_saved_emac_clk;
	uint16_t pci_bd;
	struct pinctrl *pinctrl;
	struct pinctrl_state *pinctrl_default;
	struct regulator *phy_supply;
	u32 phy_reset_gpio;
	u32 phy_reset_delay;
	u32 reset_asserted;
	int wol_irq;
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

#if 0
/* XXX What are these?  EMAC control */
#define TC9563_CFG_NEMACTXCDLY		0x1050U
#define TC9563_CFG_NEMACIOCTL		0x107CU

#define NEMACTXCDLY_DEFAULT		0x00000000U
#define NEMACIOCTL_DEFAULT		0xF300F300
#endif

#define ENABLE_XFI_INTERFACE			1 /* XFI/SFI, this is same as USXGMII, except XPCS autoneg disabled */
#define ENABLE_SGMII_INTERFACE			4

#define CM3_TAMAP_COUNT			4

#if 0
struct tc956x_version {
	unsigned char rel_dbg; /* 'R' for release, 'D' for debug */
	unsigned char major;
	unsigned char minor;
	unsigned char sub_minor;
	unsigned char patch_rel_major;
	unsigned char patch_rel_minor;
};
#endif

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

#if 0
/* PHY/MDIO configurations */
enum TC956X_PHY_MDIO_AVAILABILITY {
	PHY_ON_MDIO_ON = 0, /* PHY and MDIO available */
	PHY_ON_MDIO_OFF,    /* PHY available and MDIO not available */ /* Not supported currently */
	PHY_OFF_MDIO_ON,    /* PHY not available and MDIO available */ /* Not supported currrently */
	PHY_OFF_MDIO_OFF    /* PHY not available and MDIO not available */
};
#endif

// TODO: this was unifdef'ed (some build options result in the value being two)
#define TC956X_TOT_MSI_VEC	1

#define TC956X_DA_MAP		0xF

/************************ TC956X_SRIOV_PF Starts ************************/

#if 0
/* Unicast/Untagged packet */
#define LEG_UNTAGGED_PACKET	TC956X_DA_MAP
/* VLAN tagged packets */
#define LEG_TAGGED_PACKET	TC956X_DA_MAP
/* Untagged gPTP packet */
#define UNTAGGED_GPTP_PACKET	4
/* Untagged AV Control Packet */
#define UNTAGGED_AVCTRL_PACKET	3
/* Class B AVB Packet */
#define AVB_CLASS_B_PACKET	5
/* Class A AVB Packet */
#define AVB_CLASS_A_PACKET	6
/* TSN Class CDT Packet */
#define TSN_CLASS_CDT_PACKET	7
/* Broadcast/Multicast packet */
#define BC_MC_PACKET		TC956X_DA_MAP
#endif

/************************* TC956X_SRIOV_PF Ends *************************/

#define XGMAC_BASE(td)	((td)->sfr + ((td)->emac0 ? 0x40000 : 0x48000))

#define RSC_MNG_OFFSET		0x2000
#define RSCMNG_ID_REG		(RSC_MNG_OFFSET + 0x00000000)
#define RSCMNG_PFN_MASK		GENMASK(3, 0)

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
#define RST0_MAC0PMARST		BIT(30)		/* POWER */
#define RST0_MAC0PONRST		BIT(31)		/* POWER */

#define RST0_MCU_MASK \
		(RST0_MCURST | RST0_MCU1RST)
#define RST0_MAC0_POWER_MASK \
		(RST0_MAC0PMARST | RST0_MAC0PONRST)
#define RST0_OTHER_MASK	\
		(RST0_UART0RST | RST0_MSIGENRST)

#define NRSTCTRL1_OFFSET	0x1010	/* Reset control register 1 */
#define RST1_MAC1RST		BIT(7)		/* individual */
#define RST1_MAC1PMARST		BIT(30)		/* POWER */
#define RST1_MAC1PONRST		BIT(31)		/* POWER */

#define RST1_MAC1_POWER_MASK \
		(RST1_MAC1PMARST | RST1_MAC1PONRST)

#define NRSTCTRL0_DEFAULT	(RST0_MAC0PONRST | RST0_MAC0PMARST | \
					RST0_MAC0RST)
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

#define GPIOE0_OFFSET		0x1208	/* GPIO00 enable register */
#define GPIOE1_OFFSET		0x120C	/* GPIO01 enable register */
#define GPIOO0_OFFSET		0x1210	/* GPIO00 output register */
#define GPIOO1_OFFSET		0x1214	/* GPIO01 output register */

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

/* We only use GPIOs 00 and 01, which are managed by the NFUNCEN4 register */
static void tc956x_phy_reset_pin_config(struct tc956x_data *td)
{
	void __iomem *addr = td->sfr + NFUNCEN4_OFFSET;

	if (td->emac0)
		tc956x_reg_update(addr, NFUNCEN4_GPIO_00_MASK, GPIO_00_FUNC);
	else
		tc956x_reg_update(addr, NFUNCEN4_GPIO_01_MASK, GPIO_01_FUNC);

}

/**
 * __tc956x_assert_phy_reset() - Assert or deassert the PHY resetn output
 *  @td: driver private structure
 *  @assert: true is assert the reset signal (drive low); false is deassert
 */
static void __tc956x_assert_phy_reset(struct tc956x_data *td, bool assert)
{
	void __iomem *addr;
	u32 gpio_pin;

	if (assert == td->reset_asserted)
		return;

	tc956x_phy_reset_pin_config(td);

	/* Output value for both pins is in the GPIOO0 register */
	gpio_pin = td->phy_reset_gpio;
	addr = td->sfr + GPIOO0_OFFSET;
	tc956x_reg_update(addr, BIT(gpio_pin), assert ? 0 : 1);

	/* Configure the GPIO pin in output direction */
	addr = td->sfr + GPIOE0_OFFSET;
	tc956x_reg_update(addr, BIT(gpio_pin), 0);

	td->reset_asserted = assert;
}

static void tc956x_assert_phy_reset(struct tc956x_data *td)
{
	__tc956x_assert_phy_reset(td, true);
}

static void tc956x_deassert_phy_reset(struct tc956x_data *td)
{
	__tc956x_assert_phy_reset(td, false);
}

/**
 * tc956x_restore_phy_reset() - Restore the saved PHY reset configuration
 * @priv:	STMMAC driver private data pointer
 */
static void tc956x_restore_phy_reset(struct stmmac_priv *priv)
{
	struct tc956x_data *td = priv->plat->bsp_priv;
	u32 gpio_pin = td->phy_reset_gpio;
	void __iomem *addr;

	tc956x_phy_reset_pin_config(td);

	addr = td->sfr + GPIOO0_OFFSET;
	tc956x_reg_update(addr, BIT(gpio_pin), td->reset_asserted ? 0 : 1);

	/* Configure the GPIO pin in output direction */
	addr = td->sfr + GPIOE0_OFFSET;
	tc956x_reg_update(addr, BIT(gpio_pin), 0);
}

//
// Code from tc956x_xpcs.h in vendor driver
//

#define XPCS_XGMAC_OFFSET	0x3A00

/*XPCS registers*/
#define XGMAC_SR_MII_CTRL				0x7C0000
#define XGMAC_VR_MII_AN_CTRL			0x7e0004
#define XGMAC_VR_MII_DIG_CTRL1			0x7e0000
#define XGMAC_SR_XS_PCS_CTRL1			0xC0000
#define XGMAC_SR_XS_PCS_CTRL2			0xC001C
#define XGMAC_SR_XS_PCS_EEE_ABL			0xC0050
#define XGMAC_VR_XS_PCS_DIG_CTRL1		0xe0000
#define XGMAC_VR_XS_PCS_EEE_MCTRL0		0xe0018
#define XGMAC_VR_XS_PCS_EEE_MCTRL1		0xe002c
#define XGMAC_VR_XS_PCS_KR_CTRL			0xe001c
#define XGMAC_VR_XS_PCS_EEE_TXTIMER		0xe0020
#define XGMAC_VR_XS_PCS_EEE_RXTIMER		0xe0024
#define XGMAC_VR_XS_PCS_DIG_STS			0xe0040

#define XGMAC_LPI_ENABLE			0x0800
#define XGMAC_PSEQ_STATE			0x001C
#define XGMAC_KXEEE				0x0010
#define XGMAC_MULT_FACT_100NS			0x0F00
#define XGMAC_SIGN_BIT				0x40
#define XGMAC_TX_RX_EN				0x90
#define XGMAC_EEE_RX_TIMER			0x3FFF
#define XGMAC_EEE_TX_TIMER			0x1FFF
#define XGMAC_TX_RX_QUIET_EN			0x000F
#define XGMAC_MULT_FACT_100NS_MAC		0xB00
#define XGMAC_EEE_TX_TIMER_MAC_CONT		0x0543
#define XGMAC_EEE_RX_TIMER_MAC_CONT		0x062A
#define XGMAC_TRN_LPI				0x1

/*XPCS Register value*/
#define XGMAC_PCS_MODE_MASK				0xFFFFFFF9
#define XGMAC_SGMII_MODE				0x00000004
#define XGMAC_TX_CFIG_INTR_EN_MASK		0xFFFFFFF6/*Mask TX_CONFIG & MII_AN_INTR_EN*/
#define XGMAC_MII_AN_INTR_EN			0x00000001/*MII_AN_INTR_EN*/
#define XGMAC_MAC_AUTO_SW_EN			0x00000200/*MAC_AUTO_SW*/
#define XGMAC_AN_37_ENABLE				0x00001000/*AN_EN*/
#define XGMAC_PCS_TYPE_SEL				0xFFFFFFF0/*PCS_TYPE_SEL: 0x0000*/
#define XGMAC_USXG_EN					0x00000200/*USXG_EN enable*/
#define XGMAC_USXG_MODE					0x00001c00/*USXG_MODE: 0x000*/
#define XGMAC_VR_RST					0x00008000/*set VR_RST*/
#define XGMAC_SOFT_RST					0x00008000/*SOFT RST*/

#define XPCS_REG_BASE_ADDR_MASK				GENMASK(31, 10)
#define XPCS_REG_OFFSET_MASK				GENMASK(9, 0)
#define	XPCS_IND_ACCESS					0x3fc

#if 0
#define XPCS_USX_5G_MODE				(0x1 << 10)
#define XPCS_USX_2_5G_MODE				(0x2 << 10)
#endif

//
// Code from tc956x_xpcs.c in vendor driver
//

static u32 tc956x_xpcs_read(void __iomem *xpcsaddr, u32 pcs_reg_num)
{
	u16 base_address = FIELD_GET(XPCS_REG_BASE_ADDR_MASK, pcs_reg_num);
	u16 offset = FIELD_GET(XPCS_REG_OFFSET_MASK, pcs_reg_num);

	/* Write the base address into indirect access register */
	writel(base_address, (xpcsaddr + XPCS_IND_ACCESS));

	/* Then read the value from the offset register */

	return readl(xpcsaddr + offset);
}

static void
tc956x_xpcs_write(void __iomem *xpcsaddr, u32 pcs_reg_num, u32 value)
{
	u16 base_address = FIELD_GET(XPCS_REG_BASE_ADDR_MASK, pcs_reg_num);
	u16 offset = FIELD_GET(XPCS_REG_OFFSET_MASK, pcs_reg_num);

	/* Write the base address into indirect access register */
	writel(base_address, xpcsaddr + XPCS_IND_ACCESS);

	/* Then write the value to the offset register */
	writel(value, xpcsaddr + offset);
}

static int tc956x_xpcs_init(struct plat_stmmacenet_data *plat)
{
	struct tc956x_data *td = plat->bsp_priv;
	void __iomem *xpcs = XGMAC_BASE(td) + XPCS_XGMAC_OFFSET;
	u32 reg_value;

	reg_value = tc956x_xpcs_read(xpcs, XGMAC_SR_MII_CTRL);
	if (reg_value & XGMAC_SOFT_RST)
		return -1;

	/*Clause 37 autoneg related settings*/
	if (plat->phy_interface == PHY_INTERFACE_MODE_SGMII) {
		//DK2
		//PCS Type Select SR_XS_PCS_CTRL2  PCS_TYPE_SEL -> 1
		reg_value = tc956x_xpcs_read(xpcs, XGMAC_SR_XS_PCS_CTRL2);
		reg_value &= XGMAC_PCS_TYPE_SEL;
		reg_value |= 0x1;
		tc956x_xpcs_write(xpcs, XGMAC_SR_XS_PCS_CTRL2, reg_value);

		reg_value = tc956x_xpcs_read(xpcs, XGMAC_VR_MII_AN_CTRL);
		reg_value &= XGMAC_PCS_MODE_MASK;
		reg_value |= XGMAC_SGMII_MODE; /*SGMII PCS MODE*/
		tc956x_xpcs_write(xpcs, XGMAC_VR_MII_AN_CTRL, reg_value);

		if (td->is_sgmii_2p5g == true) {
			reg_value = tc956x_xpcs_read(xpcs,
						     XGMAC_VR_XS_PCS_DIG_CTRL1);
			reg_value &= ~(0x4);
			/* Enable only if SGMII 2.5G is enabled */
			reg_value |= 0x4; /*EN_2_5G_MODE*/
			tc956x_xpcs_write(xpcs, XGMAC_VR_XS_PCS_DIG_CTRL1,
					  reg_value);
		}
	}
	if ((plat->phy_interface == PHY_INTERFACE_MODE_USXGMII) ||
			(plat->phy_interface == PHY_INTERFACE_MODE_10GKR) ||
			(plat->phy_interface == PHY_INTERFACE_MODE_10GBASER)) {
		reg_value = tc956x_xpcs_read(xpcs, XGMAC_SR_XS_PCS_CTRL2);
		reg_value &= XGMAC_PCS_TYPE_SEL;/*PCS_TYPE_SEL as 10GBASE-R PCS */
		tc956x_xpcs_write(xpcs, XGMAC_SR_XS_PCS_CTRL2, reg_value);

		reg_value = tc956x_xpcs_read(xpcs, XGMAC_VR_XS_PCS_DIG_CTRL1);
		if (plat->phy_interface == PHY_INTERFACE_MODE_10GKR
			|| (plat->phy_interface == PHY_INTERFACE_MODE_10GBASER)
			) {
			reg_value &= (~XGMAC_USXG_EN); /*Disable USXG_EN*/
		} else {
			reg_value |= XGMAC_USXG_EN; /*set USXG_EN*/
		}

		tc956x_xpcs_write(xpcs, XGMAC_VR_XS_PCS_DIG_CTRL1, reg_value);

		reg_value = tc956x_xpcs_read(xpcs, XGMAC_VR_XS_PCS_KR_CTRL);
		reg_value &= ~XGMAC_USXG_MODE;/*USXG_MODE : 0x000*/
		tc956x_xpcs_write(xpcs, XGMAC_VR_XS_PCS_KR_CTRL, reg_value);

		reg_value = tc956x_xpcs_read(xpcs, XGMAC_VR_XS_PCS_DIG_CTRL1);
		reg_value |= XGMAC_VR_RST;/*set VR_RST*/
		tc956x_xpcs_write(xpcs, XGMAC_VR_XS_PCS_DIG_CTRL1, reg_value);

		/*Wait for Reset to clear*/
		do {
			reg_value = tc956x_xpcs_read(xpcs, XGMAC_VR_XS_PCS_DIG_CTRL1);
		} while ((XGMAC_VR_RST & reg_value) == XGMAC_VR_RST);

	}
	reg_value = tc956x_xpcs_read(xpcs, XGMAC_SR_XS_PCS_CTRL1);
	reg_value |= XGMAC_LPI_ENABLE;/* LPM : power down */
	tc956x_xpcs_write(xpcs, XGMAC_SR_XS_PCS_CTRL1, reg_value);

	reg_value = tc956x_xpcs_read(xpcs, XGMAC_VR_XS_PCS_DIG_STS);
	reg_value &= ~(XGMAC_PSEQ_STATE);/* PSEQ_STATE(B4:2)=3'b000 */
	tc956x_xpcs_write(xpcs, XGMAC_VR_XS_PCS_DIG_STS, reg_value);

	reg_value = tc956x_xpcs_read(xpcs, XGMAC_SR_XS_PCS_CTRL1);
	reg_value &= ~(XGMAC_LPI_ENABLE);/* LPM : Normal Operation */
	tc956x_xpcs_write(xpcs, XGMAC_SR_XS_PCS_CTRL1, reg_value);

	reg_value = tc956x_xpcs_read(xpcs, XGMAC_SR_XS_PCS_EEE_ABL);
	reg_value |= XGMAC_KXEEE;/* KXEEE */
	tc956x_xpcs_write(xpcs, XGMAC_SR_XS_PCS_EEE_ABL, reg_value);

	reg_value = tc956x_xpcs_read(xpcs, XGMAC_VR_XS_PCS_EEE_MCTRL0);
	reg_value &= ~(XGMAC_MULT_FACT_100NS);
	reg_value |= XGMAC_MULT_FACT_100NS_MAC; /* MULT_FACT_100NS */
	reg_value |= XGMAC_SIGN_BIT;/* SIGN_BIT */
	reg_value |= XGMAC_TX_RX_EN;/* TX_EN_CTRL, RX_EN_CTRL */
	tc956x_xpcs_write(xpcs, XGMAC_VR_XS_PCS_EEE_MCTRL0, reg_value);

	reg_value = tc956x_xpcs_read(xpcs, XGMAC_VR_XS_PCS_EEE_TXTIMER);
	reg_value &= ~(XGMAC_EEE_TX_TIMER);
	reg_value |= XGMAC_EEE_TX_TIMER_MAC_CONT; /* TWL_RES=0x5, T1U_RES=0x1, TSL_RES=0x3 */
	tc956x_xpcs_write(xpcs, XGMAC_VR_XS_PCS_EEE_TXTIMER, reg_value);

	reg_value = tc956x_xpcs_read(xpcs, XGMAC_VR_XS_PCS_EEE_RXTIMER);
	reg_value &= ~(XGMAC_EEE_RX_TIMER);
	reg_value |= XGMAC_EEE_RX_TIMER_MAC_CONT; /* TWR_RES=0x6, RES_100U=0x42 */
	tc956x_xpcs_write(xpcs, XGMAC_VR_XS_PCS_EEE_RXTIMER, reg_value);

	reg_value = tc956x_xpcs_read(xpcs, XGMAC_VR_XS_PCS_EEE_MCTRL1);
	reg_value |= XGMAC_TRN_LPI;
	tc956x_xpcs_write(xpcs, XGMAC_VR_XS_PCS_EEE_MCTRL1, reg_value);

	reg_value = tc956x_xpcs_read(xpcs, XGMAC_VR_XS_PCS_EEE_MCTRL0);

	reg_value &= ~XGMAC_TX_RX_QUIET_EN;
	reg_value |= XGMAC_TX_RX_QUIET_EN; /* RX_QUIET_EN, TX_QUIET_EN, LRX_EN, LTX_EN */

	tc956x_xpcs_write(xpcs, XGMAC_VR_XS_PCS_EEE_MCTRL0, reg_value);

	reg_value = tc956x_xpcs_read(xpcs, XGMAC_VR_MII_AN_CTRL);
	reg_value &= XGMAC_TX_CFIG_INTR_EN_MASK;/*TX_CONFIG MAC SIDE*/
	reg_value |= XGMAC_MII_AN_INTR_EN;/*MII_AN_INTR_EN enabe*/
	tc956x_xpcs_write(xpcs, XGMAC_VR_MII_AN_CTRL, reg_value);

	reg_value = tc956x_xpcs_read(xpcs, XGMAC_VR_MII_DIG_CTRL1);
	reg_value &= ~XGMAC_MAC_AUTO_SW_EN;/*MAC_AUTO_SW enable*/
	if (td->is_sgmii_2p5g != true)
		/* Enable only if SGMII 2.5G is not enabled. */
		reg_value |= XGMAC_MAC_AUTO_SW_EN;
	tc956x_xpcs_write(xpcs, XGMAC_VR_MII_DIG_CTRL1, reg_value);

	return 0;
}

static void tc956x_xpcs_ctrl_ane(struct tc956x_data *td, bool ane)
{
	void __iomem *xpcs = XGMAC_BASE(td) + XPCS_XGMAC_OFFSET;
	u32 reg_value;

	reg_value = tc956x_xpcs_read(xpcs, XGMAC_SR_MII_CTRL);
	if (ane) {
		reg_value |= XGMAC_AN_37_ENABLE;
		dev_dbg(td->dev, "%s Enable AN", __func__);
	} else {
		reg_value &= (~XGMAC_AN_37_ENABLE);
		dev_dbg(td->dev, "%s Disable AN", __func__);
	}

	tc956x_xpcs_write(xpcs, XGMAC_SR_MII_CTRL, reg_value);
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

	tc956x_deassert_phy_reset(td);

	msleep(td->phy_reset_delay);

	return ret;
}

static int tc956x_phy_power_off(struct tc956x_data *td)
{
	int ret = 0;

	tc956x_assert_phy_reset(td);

	ret = regulator_disable(td->phy_supply);
	if (ret) {
		dev_err(td->dev, "Failed to disable PHY supply with error %d\n", ret);
		tc956x_deassert_phy_reset(td);
		/* XXX Any need for the phy_reset_delay here? */
	}

	return ret;
}

static int tc956x_platform_of_parse(struct tc956x_data *td)
{
	struct device *dev = td->dev;
	struct device_node *np;
	int ret;

	np = dev_of_node(dev);
	if (!np)
		return -EINVAL;

	ret = of_property_read_u32(np, "qcom,phy-reset-gpio",
				   &td->phy_reset_gpio);
	if (ret) {
		dev_err(dev, "failed to get qcom,phy-reset-gpio property\n");
		return ret;
	}
	/* The only values used currently are 0 and 1; we'll generalize later */
	if (td->phy_reset_gpio && td->phy_reset_gpio != 1) {
		dev_err(dev, "bad qcom,phy-reset-gpio property (%u)\n",
			td->phy_reset_gpio);
		return ret;
	}

	/* XXX Can we use a good constant and avoid having to specify this? * */
	ret = of_property_read_u32(np, "qcom,phy-reset-delay",
				   &td->phy_reset_delay);
	if (ret) {
		dev_err(dev, "failed to get qcom,phy-reset-delay property\n");
		return ret;
	}

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
	tc956x_assert_phy_reset(td);

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

static int tc956x_pma_init(struct stmmac_priv *priv, void __iomem *pmaaddr)
{

	u32 reg_value;

	/*Power on CML buffer*/
	reg_value = readl(pmaaddr + XGMAC_PMA_GL_PM_CFG0);
	reg_value = XGMAC_PMA_OFFSET0;
	writel(reg_value, pmaaddr + XGMAC_PMA_GL_PM_CFG0);

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

	return 0;

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

#if 0
/*
 * This struct is used to associate PCI Function of MAC controller on a board,
 * discovered via DMI, with the address of PHY connected to the MAC. The
 * negative value of the address means that MAC controller is not connected
 * with PHY.
 */
struct tc956x_pci_func_data {
	unsigned int func;
	int phy_addr;
};

struct tc956x_pci_dmi_data {
	const struct tc956x_pci_func_data *func;
	size_t nfuncs;
};
#endif

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
		nrst_mask = RST0_MAC0RST | RST0_MAC0_POWER_MASK;
		nclk_reg = td->sfr + NCLKCTRL0_OFFSET;
		nclk_mask = CLK0_MAC0_CORE_MASK | CLK0_MAC0_IO_MASK;
	} else {
		nrst_reg = td->sfr + NRSTCTRL1_OFFSET;
		nrst_mask = RST1_MAC1RST | RST1_MAC1_POWER_MASK;
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
	struct tc956x_data *td = bsp_priv;

	if (td->port_interface != ENABLE_SGMII_INTERFACE)
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

	/* For TC956X, clk_csr_i = 125MHz XXX any standard XGMAC values? */
	if (td->emac0)			/* emac0: XFI */
		plat->clk_csr = 0x4;	/* clk_csr_i / 12 XXX set CRS bit? */
	else				/* emac1: SGMII */
		plat->clk_csr = 0x0;	/* clk_csr_i / 62 */

	if (td->port_interface == ENABLE_XFI_INTERFACE)
		plat->mac_port_sel_speed = 10000;

	if (td->port_interface == ENABLE_SGMII_INTERFACE)
		plat->mac_port_sel_speed = 2500;

	plat->bus_id = 1;
	plat->pdev = pdev;
	plat->clk_ptp_rate = 50000000;

	if (td->port_interface == ENABLE_XFI_INTERFACE) {
		plat->phy_interface = PHY_INTERFACE_MODE_10GBASER;
		plat->max_speed = 10000;
	}
	if (td->port_interface == ENABLE_SGMII_INTERFACE) {
		plat->phy_interface = PHY_INTERFACE_MODE_SGMII;
		plat->max_speed = 2500;
		td->is_sgmii_2p5g = true;
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

	/* Axi Configuration */
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

static void tc956x_fix_mac_speed(void *bsp_priv, int speed, unsigned int mode)
{
	struct tc956x_data *td = bsp_priv;
	void __iomem *xgmac = XGMAC_BASE(td);
	struct plat_stmmacenet_data *plat = td->plat;
	int ret, reg = 0, val, reg_value;
	void __iomem *xpcs = xgmac + XPCS_XGMAC_OFFSET;
	bool enable_an = true;

	// TODO: copied from vendor drivers customizations in
	//       tc956x_speed_change_init_mac()

	if (td->emac0) {
		/* XXX emac0: td->port_interface = ENABLE_XFI_INTERFACE */
		/* XXX plat->phy_interface = PHY_INTERFACE_MODE_10GBASER; */
		/* XXX plat->max_speed = 10000; */
		/* Enable all clocks to eMAC Port0 */
		/* Interface configuration for port0*/
		ret = readl(td->sfr + NEMAC0CTL_OFFSET);
		ret &= ~EMAC_SP_SEL_MASK;
		ret &= ~EMAC_PHY_INF_SEL_MASK;
		reg_value = tc956x_xpcs_read(xpcs, XGMAC_VR_XS_PCS_KR_CTRL);
		reg_value &= ~XGMAC_USXG_MODE; /*USXG_MODE : 0x000*/
		tc956x_xpcs_write(xpcs, XGMAC_VR_XS_PCS_KR_CTRL, reg_value);
		ret &= ~EMAC_INV_SGM_SIG_DET;
		ret |= FIELD_PREP(EMAC_PHY_INF_SEL_MASK, PCS_CLK_PHY);
		ret |= EMAC_LPIHWCLKEN;
		writel(ret, td->sfr + NEMAC0CTL_OFFSET);
		writel(reg, td->sfr + NMISCCTL_OFFSET);
	} else {
		/* XXX emac1: td->port_interface = ENABLE_SGMII_INTERFACE; */
		/* plat->phy_interface = PHY_INTERFACE_MODE_SGMII; */
		/* plat->max_speed = 2500; */
		/* td->is_sgmii_2p5g = true; */
		/* Enable all clocks to eMAC Port1 */
		ret = readl(td->sfr + NCLKCTRL1_OFFSET);
		if (td->plat->phy_interface == PHY_INTERFACE_MODE_SGMII &&
		    speed == SPEED_2500) {
			ret &= ~CLK1_MAC1_CORE_MASK;
		} else {
			ret &= ~CLK1_MAC1312CLKEN;
			ret |= CLK1_MAC1125CLKEN;
		}
		writel(ret, td->sfr + NCLKCTRL1_OFFSET);

		/* Interface configuration for port1*/
		ret = readl(td->sfr + NEMAC1CTL_OFFSET);
		ret &= ~EMAC_SP_SEL_MASK;
		ret &= ~EMAC_PHY_INF_SEL_MASK;
		if (plat->phy_interface == PHY_INTERFACE_MODE_SGMII) {
			if (speed == SPEED_2500)
				ret |= FIELD_PREP(EMAC_SP_SEL_MASK,
						  SPEED_SGMII_2500M);
			else
				ret |= FIELD_PREP(EMAC_SP_SEL_MASK,
						  SPEED_SGMII_1000M);
		} else {
			reg_value = tc956x_xpcs_read(xpcs, XGMAC_VR_XS_PCS_KR_CTRL);
			reg_value &= ~XGMAC_USXG_MODE; /*USXG_MODE : 0x000*/
			tc956x_xpcs_write(xpcs, XGMAC_VR_XS_PCS_KR_CTRL, reg_value);
		}

		ret &= ~EMAC_INV_SGM_SIG_DET;
		ret |= FIELD_PREP(EMAC_PHY_INF_SEL_MASK, PCS_CLK_PHY);
		ret |= EMAC_LPIHWCLKEN;
		writel(ret, td->sfr + NEMAC1CTL_OFFSET);
		writel(reg, td->sfr + NMISCCTL_OFFSET);

	}

	if (td->emac0) {
		/* Assertion of PMA & XPCS reset software Reset*/
		ret = readl(td->sfr + NRSTCTRL0_OFFSET);
		ret |= RST0_MAC0_POWER_MASK;
		writel(ret, td->sfr + NRSTCTRL0_OFFSET);
	} else {
		/* Assertion of PMA &  XPCS reset  software Reset*/
		ret = readl(td->sfr + NRSTCTRL1_OFFSET);
		ret |= RST1_MAC1_POWER_MASK;
		writel(ret, td->sfr + NRSTCTRL1_OFFSET);
	}

	ret = tc956x_pma_init(NULL, xgmac + PMA_XGMAC_OFFSET);
	if (ret < 0)
		pr_info("PMA switching to internal clock Failed\n");

	if (td->emac0) {
		/* De-assertion of PMA & XPCS reset software Reset*/
		ret = readl(td->sfr + NRSTCTRL0_OFFSET);
		ret &= ~RST0_MAC0_POWER_MASK;
		ret &= ~RST0_MAC0RST;
		writel(ret, td->sfr + NRSTCTRL0_OFFSET);
	} else {
		/* De-assertion of PMA &  XPCS reset software Reset*/
		ret = readl(td->sfr + NRSTCTRL1_OFFSET);
		ret &= ~RST1_MAC1_POWER_MASK;
		/* Is this missing?  ret &= ~RST1_MAC1RST; */
		writel(ret, td->sfr + NRSTCTRL1_OFFSET);
	}

	ret = readl_poll_timeout(td->sfr + (td->emac0 ? NEMAC0CTL_OFFSET
						           : NEMAC1CTL_OFFSET),
				 val, val & EMAC_INIT_DONE, 50, 1000000);
	if (ret < 0)
		dev_err(td->dev, "PMA/XPCS failed to come out of reset\n");

	/*
	 * TODO:
	 * This is probably wrong (or at least making a decision on a
	 * potentially outdated value): at 2500 the proper enum value is
	 *  PHY_INTERFACE_MODE_2500BASEX.
	 */
	if ((plat->phy_interface == PHY_INTERFACE_MODE_SGMII) &&
	    (speed == SPEED_2500)) {
		/* XPCS doesn't support AN for 2.5G SGMII.
		 * Disable AN only if SGMII 2.5G is Enabled.
		 */
		td->is_sgmii_2p5g = true;
		enable_an = false;
	} else {
		td->is_sgmii_2p5g = false;
		enable_an = true;
	}

	ret = tc956x_xpcs_init(td->plat);
	if (ret < 0)
		dev_err(td->dev, "XPCS initialization error\n");

	tc956x_xpcs_ctrl_ane(td, enable_an);
}

/* Assert or deassert the interrupt controller (INTC) */
static void tc956x_intc_reset(struct tc956x_data *td, bool assert)
{
	void __iomem *addr = td->sfr + NRSTCTRL0_OFFSET;
	u32 val;

	/* Note: 1 means assert */
	val = readl(addr);
	if (assert)
		val |= RST0_INTRST;
	else
		val &= ~RST0_INTRST;
	writel(val, addr);
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
	struct plat_stmmacenet_data *plat;
	struct stmmac_resources res = { };
	struct device *dev = &pdev->dev;
	struct net_device *netdev;
	struct stmmac_priv *priv;
	struct tc956x_data *td;
	/* use signal from EMSPHY */
	uint16_t sh_mem_offset;
	void __iomem *virt;
	bool mode2;
	u32 pfn;
	u32 val;
	int ret;

	/* The platform structure is allocated with devm_kzalloc() */
	plat = stmmac_plat_dat_alloc(dev);
	if (!plat)
		return -ENOMEM;

	td = devm_kzalloc(dev, sizeof(*td), GFP_KERNEL);
	if (!td)
		return -ENOMEM;

	plat->bsp_priv = td;
	td->plat = plat;
	td->dev = dev;

	/* XXX We don't initialize this; what is required? */
	plat->mdio_bus_data = devm_kzalloc(dev, sizeof(*plat->mdio_bus_data),
					   GFP_KERNEL);
	if (!plat->mdio_bus_data)
		return -ENOMEM;

	/* XXX We initialize two (four) fields here; what is required? */
	plat->dma_cfg = devm_kzalloc(dev, sizeof(*plat->dma_cfg), GFP_KERNEL);
	if (!plat->dma_cfg)
		return -ENOMEM;

	ret = pcim_enable_device(pdev);
	if (ret) {
		dev_err(dev, "%s: ERROR: failed to enable device\n", __func__);
		return ret;
	}
	pci_set_master(pdev);

	/* Request the PCI IO Memory for the device */
	virt = pcim_iomap_region(pdev, PCI_BAR_BRIDGE_CONFIG, DRIVER_NAME);
	if (IS_ERR(virt)) {
		ret = PTR_ERR(virt);
		dev_err(dev, "failed to map bridge config region\n");
		goto err_clear_master;
	}
	td->bridge_config = virt;

	virt = pcim_iomap_region(pdev, PCI_BAR_SFR, DRIVER_NAME);
	if (IS_ERR(td->sfr)) {
		ret = PTR_ERR(virt);
		dev_err(dev, "failed to map sfr region\n");
		goto err_clear_master;
	}
	td->sfr = virt;

	dev_dbg(dev, "BAR0 physical address = 0x%llx length 0x%llx\n",
		(u64)pci_resource_start(pdev, 0),
		(u64)pci_resource_len(pdev, 0));
	dev_dbg(dev, "BAR2 physical address = 0x%llx length 0x%llx\n",
		(u64)pci_resource_start(pdev, 2),
		(u64)pci_resource_len(pdev, 2));
	dev_dbg(dev, "BAR4 physical address = 0x%llx length 0x%llx\n",
		(u64)pci_resource_start(pdev, 4),
		(u64)pci_resource_len(pdev, 4));

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

	/* Determine physical port number from the resource manager */
	val = readl(td->bridge_config + RSCMNG_ID_REG);
	pfn = FIELD_GET(RSCMNG_PFN_MASK, val);
	if (WARN_ON(pfn > 1)) {
		ret = -EINVAL;
		goto err_clear_master;
	}
	td->emac0 = pfn == 0;

	plat->fix_mac_speed = &tc956x_fix_mac_speed;

	// NCID_OFFSET gives the revision ID (and early revisions are limited
	// to 2.5G)
	pr_debug("NCID Register value: %x\n", readl(td->sfr + NCID_OFFSET));

	td->port_interface = td->emac0 ? ENABLE_XFI_INTERFACE
				       : ENABLE_SGMII_INTERFACE;

	ret = tc956x_xgmac3_default_data(pdev, plat);
	if (ret)
		goto err_clear_master;

	dev_dbg(dev, "port_interface = %d\n", td->port_interface);

	/* XXX eMAC0, or first one probed? */
	if (td->emac0) {
		void __iomem *addr = td->sfr + NCLKCTRL0_OFFSET;

		/* Enable the interrupt controller */
		tc956x_intc_reset(td, true);
		val = readl(addr);
		val |= CLK0_INTCEN;
		writel(val, addr);
		tc956x_intc_reset(td, false);
		tc956x_config_tamap(td);
	}
	dev_dbg(dev, "Initialising eMAC Port %d bus number-%x\n",
		 td->emac0 ? 0 : 1, pdev->bus->number);
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

	if (td->emac0) {
		ret = readl(td->sfr + NRSTCTRL0_OFFSET);
		/* Assertion of EMAC Port0 software Reset */
		ret |= RST0_MAC0RST;
		writel(ret, td->sfr + NRSTCTRL0_OFFSET);

		/* Enable all clocks to eMAC Port0 */
		ret = readl(td->sfr + NCLKCTRL0_OFFSET);

		ret |= CLK0_MAC0_IO_MASK;

		/* Only if "current" port is SGMII 2.5G, configure below clocks. */
		if (td->port_interface == ENABLE_SGMII_INTERFACE) {
			ret &= ~CLK0_BUS_MASK;
			ret &= ~CLK0_MAC0_CORE_MASK;
		}
		writel(ret, td->sfr + NCLKCTRL0_OFFSET);

		/* Interface configuration for port0*/
		ret = readl(td->sfr + NEMAC0CTL_OFFSET);
		ret &= ~EMAC_SP_SEL_MASK;
		ret &= ~EMAC_PHY_INF_SEL_MASK;
		if (td->port_interface == ENABLE_SGMII_INTERFACE)
			ret |= FIELD_PREP(EMAC_SP_SEL_MASK, SPEED_SGMII_2500M);
		else if (td->port_interface == ENABLE_XFI_INTERFACE)
			ret |= FIELD_PREP(EMAC_SP_SEL_MASK,
					  SPEED_USXGMII_10G_10G);

		ret &= ~EMAC_INV_SGM_SIG_DET;

		ret |= FIELD_PREP(EMAC_PHY_INF_SEL_MASK, PCS_CLK_PHY);
		ret |= EMAC_LPIHWCLKEN;
		writel(ret, td->sfr + NEMAC0CTL_OFFSET);

		/* De-assertion of EMAC Port0  software Reset*/
		ret = readl(td->sfr + NRSTCTRL0_OFFSET);
		ret &= ~RST0_MAC0RST;
		writel(ret, td->sfr + NRSTCTRL0_OFFSET);
	} else {
		ret = readl(td->sfr + NRSTCTRL1_OFFSET);
		/* Assertion of EMAC Port1 software Reset*/
		ret |= RST1_MAC1RST;
		writel(ret, td->sfr + NRSTCTRL1_OFFSET);

		/* Enable all clocks to eMAC Port1 */
		ret = readl(td->sfr + NCLKCTRL1_OFFSET);

		ret |= CLK1_MAC1_IO_MASK | CLK1_MAC1RMCEN;
		if (td->port_interface == ENABLE_SGMII_INTERFACE)
			ret &= ~CLK1_MAC1_CORE_MASK;
		writel(ret, td->sfr + NCLKCTRL1_OFFSET);

		/* Interface configuration for port1*/
		ret = readl(td->sfr + NEMAC1CTL_OFFSET);
		ret &= ~EMAC_SP_SEL_MASK;
		ret &= ~EMAC_PHY_INF_SEL_MASK;
		if (td->port_interface == ENABLE_SGMII_INTERFACE)
			ret |= FIELD_PREP(EMAC_SP_SEL_MASK, SPEED_SGMII_2500M);
		else if (td->port_interface == ENABLE_XFI_INTERFACE)
			ret |= FIELD_PREP(EMAC_SP_SEL_MASK,
					  SPEED_USXGMII_10G_10G);

		ret &= ~EMAC_INV_SGM_SIG_DET;

		ret |= FIELD_PREP(EMAC_PHY_INF_SEL_MASK, PCS_CLK_PHY);
		ret |= EMAC_LPIHWCLKEN;
		writel(ret, td->sfr + NEMAC1CTL_OFFSET);

		/* De-assertion of EMAC Port1  software Reset */
		ret = readl(td->sfr + NRSTCTRL1_OFFSET);
		ret &= ~RST1_MAC1RST;
		writel(ret, td->sfr + NRSTCTRL1_OFFSET);
	}

	res.addr = XGMAC_BASE(td);
	res.wol_irq = pdev->irq;
	res.irq = pdev->irq;

	plat->bus_id = ((pdev->bus->number<<4) | (td->emac0 ? 0 : 1));

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

	if (td->emac0) {
		/* Assertion of PMA & XPCS reset software Reset*/
		ret = readl(td->sfr + NRSTCTRL0_OFFSET);
		ret |= RST0_MAC0_POWER_MASK;
		writel(ret, td->sfr + NRSTCTRL0_OFFSET);
	} else {
		/* Assertion of PMA &  XPCS reset  software Reset*/
		ret = readl(td->sfr + NRSTCTRL1_OFFSET);
		ret |= RST1_MAC1_POWER_MASK;
		writel(ret, td->sfr + NRSTCTRL1_OFFSET);
	}

	ret = tc956x_pma_init(NULL, res.addr + PMA_XGMAC_OFFSET);
	if (ret < 0)
		pr_info("PMA switching to internal clock Failed\n");

	if (td->emac0) {
		/* De-assertion of PMA & XPCS reset software Reset*/
		ret = readl(td->sfr + NRSTCTRL0_OFFSET);
		ret &= ~RST0_MAC0_POWER_MASK;
		ret &= ~RST0_MAC0RST;
		writel(ret, td->sfr + NRSTCTRL0_OFFSET);
	} else {
		/* De-assertion of PMA &  XPCS reset software Reset*/
		ret = readl(td->sfr + NRSTCTRL1_OFFSET);
		ret &= ~RST1_MAC1_POWER_MASK;
		/* XXX Is this missing?  ret &= ~RST1_MAC1RST; */
		writel(ret, td->sfr + NRSTCTRL1_OFFSET);
	}

	ret = readl_poll_timeout(td->sfr + (td->emac0 ? NEMAC0CTL_OFFSET :
							     NEMAC1CTL_OFFSET),
				 val, val & EMAC_INIT_DONE, 50, 1000000);
	if (ret < 0)
		dev_err(dev, "PMA/XPCS failed to come out of reset\n");

	ret = tc956x_xpcs_init(plat);
	if (ret < 0)
		dev_err(dev, "XPCS initialization error\n");

	ret = stmmac_dvr_probe(dev, plat, &res);
	if (ret) {
		void __iomem *nrst_reg;
		void __iomem *nclk_reg;
		u32 nrst_mask;
		u32 nclk_mask;
		u32 val;

		if (td->emac0) {
			nrst_reg = td->sfr + NRSTCTRL0_OFFSET;
			nrst_mask = RST0_MAC0RST | RST0_MAC0_POWER_MASK;
			nclk_reg = td->sfr + NCLKCTRL0_OFFSET;
			nclk_mask = CLK0_MAC0_CORE_MASK | CLK0_MAC0_IO_MASK;
		} else {
			nrst_reg = td->sfr + NRSTCTRL1_OFFSET;
			nrst_mask = RST1_MAC1RST | RST1_MAC1_POWER_MASK;
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

		/* Make sure probe() succeeds by returning 0 to caller of probe() */
		ret = 0;	/* XXX Why? */
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

	return ret;

err_dvr_probe:
	(void) tc956x_platform_remove(td);
err_platform_probe:
err_out_msi_failed:
	pci_free_irq_vectors(pdev);
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
		nrst_val |= NRSTCTRL0_DEFAULT;
		writel(nrst_val, nrst_reg);

		nclk_reg = td->sfr + NCLKCTRL0_OFFSET;
		nclk_val = readl(nclk_reg);
		nclk_val &= ~CLK0_MAC0_CORE_MASK;
		nclk_val &= ~CLK0_MAC0_IO_MASK;
		writel(nclk_val, nclk_reg);
	} else {
		nrst_reg = td->sfr + NRSTCTRL1_OFFSET;
		nrst_val = RST1_MAC1RST | RST1_MAC1_POWER_MASK;
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
	/* use signal from MSPHY */
	int ret = 0;
	u32 val;

	/* Skip Config when Port unavailable */
	if (priv->dma_cap.sma_mdio == 1) {
		if ((priv->plat->phy_addr == -1) || (priv->mii == NULL)) {
			dev_dbg(dev, "%s : Invalid PHY Address (%d)\n", __func__, priv->plat->phy_addr);
			ret = -1;
			goto err_phy_addr;
		}
	}

	if (td->emac0) {
		ret = readl(td->sfr + NRSTCTRL0_OFFSET);
		/* Assertion of EMAC Port0 software Reset */
		ret |= RST0_MAC0RST;
		writel(ret, td->sfr + NRSTCTRL0_OFFSET);

		/* Enable all clocks to eMAC Port0 */
		ret = readl(td->sfr + NCLKCTRL0_OFFSET);

		ret |= CLK0_MAC0_IO_MASK;

		if (td->port_interface == ENABLE_SGMII_INTERFACE) {
			/* Disable Clocks for 2.5Gbps SGMII */
			ret &= ~CLK0_BUS_MASK;
			ret &= ~CLK0_MAC0_CORE_MASK;
		}
		writel(ret, td->sfr + NCLKCTRL0_OFFSET);

		/* Interface configuration for port0*/
		ret = readl(td->sfr + NEMAC0CTL_OFFSET);
		ret &= ~EMAC_SP_SEL_MASK;
		ret &= ~EMAC_PHY_INF_SEL_MASK;
		if (td->port_interface == ENABLE_SGMII_INTERFACE)
			ret |= FIELD_PREP(EMAC_SP_SEL_MASK, SPEED_SGMII_2500M);
		else if (td->port_interface == ENABLE_XFI_INTERFACE)
			ret |= FIELD_PREP(EMAC_SP_SEL_MASK,
					  SPEED_USXGMII_10G_10G);

		ret &= ~EMAC_INV_SGM_SIG_DET;

		ret |= FIELD_PREP(EMAC_PHY_INF_SEL_MASK, PCS_CLK_PHY);
		ret |= EMAC_LPIHWCLKEN;
		writel(ret, td->sfr + NEMAC0CTL_OFFSET);

		/* De-assertion of EMAC Port0  software Reset*/
		ret = readl(td->sfr + NRSTCTRL0_OFFSET);
		ret &= ~RST0_MAC0RST;
		writel(ret, td->sfr + NRSTCTRL0_OFFSET);
	} else {
		ret = readl(td->sfr + NRSTCTRL1_OFFSET);
		/* Assertion of EMAC Port1 software Reset*/
		ret |= RST1_MAC1RST;
		writel(ret, td->sfr + NRSTCTRL1_OFFSET);

		/* Enable all clocks to eMAC Port1 */
		ret = readl(td->sfr + NCLKCTRL1_OFFSET);

		ret |= CLK1_MAC1_IO_MASK | CLK1_MAC1RMCEN;
		if (td->port_interface == ENABLE_SGMII_INTERFACE)
			ret &= ~CLK1_MAC1_CORE_MASK;
		writel(ret, td->sfr + NCLKCTRL1_OFFSET);

		/* Interface configuration for port1*/
		ret = readl(td->sfr + NEMAC1CTL_OFFSET);
		ret &= ~EMAC_SP_SEL_MASK;
		ret &= ~EMAC_PHY_INF_SEL_MASK;
		if (td->port_interface == ENABLE_SGMII_INTERFACE)
			ret |= FIELD_PREP(EMAC_SP_SEL_MASK, SPEED_SGMII_2500M);
		else if (td->port_interface == ENABLE_XFI_INTERFACE)
			ret |= FIELD_PREP(EMAC_SP_SEL_MASK,
					  SPEED_USXGMII_10G_10G);

		ret &= ~EMAC_INV_SGM_SIG_DET;

		ret |= FIELD_PREP(EMAC_PHY_INF_SEL_MASK, PCS_CLK_PHY);
		ret |= EMAC_LPIHWCLKEN;
		writel(ret, td->sfr + NEMAC1CTL_OFFSET);

		/* De-assertion of EMAC Port1  software Reset */
		ret = readl(td->sfr + NRSTCTRL1_OFFSET);
		ret &= ~RST1_MAC1RST;
		writel(ret, td->sfr + NRSTCTRL1_OFFSET);
	}

	if (priv->hw->xpcs) {
		if (td->emac0) {
			/* Assertion of PMA &  XPCS reset  software Reset*/
			ret = readl(td->sfr + NRSTCTRL0_OFFSET);
			ret |= RST0_MAC0_POWER_MASK;
			writel(ret, td->sfr + NRSTCTRL0_OFFSET);
		} else {
			/* Assertion of PMA &  XPCS reset  software Reset*/
			ret = readl(td->sfr + NRSTCTRL1_OFFSET);
			ret |= RST1_MAC1_POWER_MASK;
			writel(ret, td->sfr + NRSTCTRL1_OFFSET);
		}

		ret = tc956x_pma_init(priv, priv->ioaddr + PMA_XGMAC_OFFSET);
		if (ret < 0)
			pr_info("PMA switching to internal clock Failed\n");

		if (td->emac0) {
			/* De-assertion of PMA &  XPCS reset  software Reset*/
			ret = readl(td->sfr + NRSTCTRL0_OFFSET);
			ret &= ~RST0_MAC0_POWER_MASK;
			ret &= ~RST0_MAC0RST;
			writel(ret, td->sfr + NRSTCTRL0_OFFSET);
		} else {
			/* De-assertion of PMA &  XPCS reset  software Reset*/
			ret = readl(td->sfr + NRSTCTRL1_OFFSET);
			ret &= ~RST1_MAC1_POWER_MASK;
			/* XXX Is this missing?  ret &= ~RST1_MAC1RST; */
			writel(ret, td->sfr + NRSTCTRL1_OFFSET);
		}

		ret = readl_poll_timeout(
			td->sfr + (td->emac0 ? NEMAC0CTL_OFFSET : NEMAC1CTL_OFFSET),
			val, val & EMAC_INIT_DONE, 50, 1000000);
		if (ret < 0)
			dev_err(dev, "PMA/XPCS failed to come out of reset\n");

		ret = tc956x_xpcs_init(priv->plat);
		if (ret < 0)
			pr_info("XPCS initialization error\n");
	}

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

	tc956x_restore_phy_reset(priv);

	ret = tc956x_platform_resume(priv);
	if (ret) {
		dev_err(dev, "%s: error in calling tc956x_platform_resume", pci_name(pdev));
		pci_disable_device(pdev);

		return ret;
	}

	/* Configure TA map registers */

	/* Zero active means are suspended */
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
