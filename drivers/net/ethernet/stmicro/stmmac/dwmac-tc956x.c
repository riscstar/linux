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

#define DRIVER_NAME "dwmac-tc956x-pci"

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

/*
 * Used to store toshiba-specific context.
 *
 * It is stored in the bsp_priv field of struct plat_stmmacenet_data.
 *
 * Can be accessed as either plat->bsp_priv or priv->plat->bsp_priv depending
 * on which pointer you have in any particular part of the code.
 */
struct tc956x_data {
	/** @dev: Device pointer */
	struct device *dev;

	/** @dev: Back-pointer to our plat structure */
	struct plat_stmmacenet_data *plat;

	/** &bridge_cfg_addr: Bridge config base address (BAR0) */
	void __iomem *bridge_cfg_addr;

	/** &sram_addr: SRAM base address (BAR2) */
	void __iomem *sram_addr;

	/** @sfr_addr: SFR base address (BAR4) */
	void __iomem *sfr_addr;

	/** @emac0: Indicates which eMAC is assigned to this driver */
	bool emac0;		/* true: eMAC port 0; false: eMAC port 1 */

	/**
	 * @is_sgmii_2p5g: Controls XPCS AN enablement
	 *
	 * True if the PHY is interfaced via SGMII and is operating at
	 *  2.5G, false otherwise.
	 */
	bool is_sgmii_2p5g;

	/** @port_interface: Operating mode of the port (SGMII, XII, etc) */
	u32 port_interface;

	/** @tc956x_port_pm_suspend: Port Suspend Status (true if port suspended */
	bool tc956x_port_pm_suspend;

	/** @pm_saved_emac_rst: Preserves eMAC resets during suspend-resume */
	u32 pm_saved_emac_rst;

	/** @pm_saved_emac_clk: Preserves eMAC clock status during suspend-resume */
	u32 pm_saved_emac_clk;

	/** @pci_bd: PCI bus and device ID of self */
	uint16_t pci_bd;

	/*
	 * Remaining elements were copied from tc956x_qcom_priv (and all
	 * comments have been preserved)
	 */

	struct pinctrl *pinctrl;
	struct pinctrl_state *pinctrl_default;
	struct regulator *phy_supply;
	u32 phy_reset_gpio;
	u32 phy_reset_delay;
	u32 saved_phy_reset_value;
	int wol_irq;
};

/* XXX TC9564? Also, this is a physical function; virtual is 0x0221 */
#define PCI_DEVICE_ID_TOSHIBA_TC956X		0x0220

//
// Definitions taken from tc956xmac.h in vendor driver
//

#define FIRMWARE_NAME "TC956X_Firmware_PCIeBridge.bin"

#define TC956X_FW_MAX_SIZE	(64*1024)

#define ATR_AXI4_SLV_BASE		0x0800
#define ATR_AXI4_SLAVE_OFFSET		0x0100
#define ATR_AXI4_TABLE_OFFSET		0x20
#define TC956X_AXI4_SLV(ch, tid)	(ATR_AXI4_SLV_BASE +\
					(ch * ATR_AXI4_SLAVE_OFFSET) +\
					(tid * ATR_AXI4_TABLE_OFFSET))

#define SRC_ADDR_LO_OFFSET		0x00
#define SRC_ADDR_HI_OFFSET		0x04
#define TRSL_ADDR_LO_OFFSET		0x08
#define TRSL_ADDR_HI_OFFSET		0x0C
#define TRSL_PARAM_OFFSET		0x10
#define TRSL_MASK_OFFSET1		0x18
#define TRSL_MASK_OFFSET2		0x1C
#define TC956X_AXI4_SLV_SRC_ADDR_LO(ch, tid)	(TC956X_AXI4_SLV(ch, tid) + SRC_ADDR_LO_OFFSET)
#define TC956X_AXI4_SLV_SRC_ADDR_HI(ch, tid)	(TC956X_AXI4_SLV(ch, tid) + SRC_ADDR_HI_OFFSET)
#define TC956X_AXI4_SLV_TRSL_ADDR_LO(ch, tid)	(TC956X_AXI4_SLV(ch, tid) + TRSL_ADDR_LO_OFFSET)
#define TC956X_AXI4_SLV_TRSL_ADDR_HI(ch, tid)	(TC956X_AXI4_SLV(ch, tid) + TRSL_ADDR_HI_OFFSET)
#define TC956X_AXI4_SLV_TRSL_PARAM(ch, tid)	(TC956X_AXI4_SLV(ch, tid) + TRSL_PARAM_OFFSET)
#define TC956X_AXI4_SLV_TRSL_MASK1(ch, tid)	(TC956X_AXI4_SLV(ch, tid) + TRSL_MASK_OFFSET1)
#define TC956X_AXI4_SLV_TRSL_MASK2(ch, tid)	(TC956X_AXI4_SLV(ch, tid) + TRSL_MASK_OFFSET2)

#define TC956X_ATR_IMPL 1U
#define TC956X_ATR_SIZE(size) ((size - 1U) << 1U)

#define TC956X_AXI4_SLV00_ATR_SIZE 36U
#define TC956X_AXI4_SLV00_SRC_ADDR_LO_VAL  (0x00000000U)
#define TC956X_AXI4_SLV00_SRC_ADDR_HI_VAL  (0x00000010U)
#define TC956X_AXI4_SLV00_TRSL_ADDR_LO_VAL (0x00000000U)
#define TC956X_AXI4_SLV00_TRSL_ADDR_HI_VAL (0x00000000U)
#define TC956X_AXI4_SLV00_TRSL_PARAM_VAL   (0x00000000U)
#define TC956X_AXI4_SLV00_SRC_ADDR_LO_VAL_DEFAULT  (0x0000007FU)

#define NRSTCTRL0_RST_ASRT 0x1
#define NRSTCTRL0_RST_DE_ASRT 0x3

#define TC956X_BAR0 0
#define TC956X_BAR2 2
#define TC956X_BAR4 4

#define TC9563_CFG_NEMACTXCDLY		0x1050U
#define TC9563_CFG_NEMACIOCTL		0x107CU

#define NEMACTXCDLY_DEFAULT		0x00000000U
#define NEMACIOCTL_DEFAULT		0xF300F300

#define TC956X_M3_SRAM_EEPROM_OFFSET_ADDR	0x47050		/* DMEM addrs 0x20007050U */
#define TC956X_M3_SRAM_EEPROM_MAC_COUNT		0x47051		/* DMEM addrs 0x20007051U */
#define TC956X_M3_INIT_DONE			0x47054		/* DMEM addrs 0x20007054U */

#define ENABLE_XFI_INTERFACE			1 /* XFI/SFI, this is same as USXGMII, except XPCS autoneg disabled */
#define ENABLE_SGMII_INTERFACE			4

#define MAX_CM3_TAMAP_ENTRIES		3

struct tc956x_version {
	unsigned char rel_dbg; /* 'R' for release, 'D' for debug */
	unsigned char major;
	unsigned char minor;
	unsigned char sub_minor;
	unsigned char patch_rel_major;
	unsigned char patch_rel_minor;
};

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

/* Independent Suspend/Resume Debug */
#undef TC956X_PM_DEBUG
#define TC956X_ALL_MAC_PORT_SUSPENDED	0 /* All EMAC Port Suspended. To be used just after suspend and before resume. */
#define TC956X_SINGLE_MAC_DEVICE_IN_USE	1 /* One of the EMAC Port in use. To be used at remove. */
#define TC956X_TOT_CASCADE_DEV	7 /* Maximum number of devices for 2 Level cascade setup */
#define TC956X_PCI_BD_MASK	0xFFF8

/* Suspend-Resume Arguments */
enum TC956X_PORT_PM_STATE {
	SUSPEND = 0,
	RESUME,
};

/* PHY/MDIO configurations */
enum TC956X_PHY_MDIO_AVAILABILITY {
	PHY_ON_MDIO_ON = 0, /* PHY and MDIO available */
	PHY_ON_MDIO_OFF,    /* PHY available and MDIO not available */ /* Not supported currently */
	PHY_OFF_MDIO_ON,    /* PHY not available and MDIO available */ /* Not supported currrently */
	PHY_OFF_MDIO_OFF    /* PHY not available and MDIO not available */
};

// TODO: this was unifdef'ed (some build options result in the value being two)
#define TC956X_TOT_MSI_VEC	1

#define TC956X_DA_MAP		0xF

/************************ TC956X_SRIOV_PF Starts ************************/

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

#define EEPROM_OFFSET		0
#define EEPROM_MAC_COUNT	14

/************************* TC956X_SRIOV_PF Ends *************************/

#define MAC0_BASE_OFFSET 0x40000 /* eMAC0 Base Offset */
#define MAC1_BASE_OFFSET 0x48000 /* eMAC1 Base Offset */

#define RSC_MNG_OFFSET		0x2000
#define RSCMNG_ID_REG		(RSC_MNG_OFFSET + 0x00000000)
#define RSCMNG_PFN_MASK		GENMASK(3, 0)

/*	Configuration Register Address	*/
#define NCID_OFFSET			(0x0000) /* TC956X Chip and revision ID */
#define NMODESTS_OFFSET		(0x0004) /* TC956X current operation mode */
#define NMODESTS_MODE2		BIT(10)	/* PCIe lanes: 0:  x4x1x1; 1: x2x2x1 */

#define NCLKCTRL0_OFFSET	(0x1004)  /* TC956X clock control Register-0 */
#define NCLKCTRL0_MCUCEN	BIT(0)
#define NCLKCTRL0_INTCEN	BIT(4)
#define NCLKCTRL0_MAC0TXCEN	BIT(7)
#define NCLKCTRL0_PCIECEN	BIT(9)
#define NCLKCTRL0_I2SSPIEN	BIT(12)
#define NCLKCTRL0_SRMCEM	BIT(13)
#define NCLKCTRL0_MAC0RXCEN	BIT(14)
#define NCLKCTRL0_UARTOCEN	BIT(16)
#define NCLKCTRL0_MSIGENCEN	BIT(18)
#define NCLKCTRL0_POEPLLCEN	BIT(24)
#define NCLKCTRL0_SGMPCIEN	BIT(25)
#define NCLKCTRL0_REFCLKOCEN	BIT(26)
#define NCLKCTRL0_MAC0125CLKEN	BIT(29)
#define NCLKCTRL0_MAC0312CLKEN	BIT(30)
#define NCLKCTRL0_MAC0ALLCLKEN	BIT(31)
#define NRSTCTRL0_OFFSET	(0x1008)  /* TC956X reset control Register-0 */
#define NRSTCTRL0_MCURST	BIT(0)
#define NRSTCTRL0_INTRST	BIT(4)
#define NRSTCTRL0_MAC0RST	BIT(7)
#define NRSTCTRL0_UART0RST	BIT(16)
#define NRSTCTRL0_MSIGENRST	BIT(18)
#define NRSTCTRL0_MAC0PMARST	BIT(30)
#define NRSTCTRL0_MAC0PONRST	BIT(31)
#define NCLKCTRL1_OFFSET	(0x100C)  /* TC956X clock control Register-1 for eMAC Port-1*/
#define NCLKCTRL1_MAC1TXCEN	BIT(7)
#define NCLKCTRL1_MAC1RXCEN	BIT(14)
#define NCLKCTRL1_MAC1RMCEN	BIT(15)
#define NCLKCTRL1_MAC1125CLKEN1	BIT(29)
#define NCLKCTRL1_MAC1312CLKEN1	BIT(30)
#define NCLKCTRL1_MAC1ALLCLKEN1	BIT(31)
#define NRSTCTRL1_OFFSET	(0x1010)  /* TC956X reset control Register-1 for eMAC Port-1*/
#define NRSTCTRL1_MAC1RST1	BIT(7)
#define NRSTCTRL1_MAC1PMARST1	BIT(30)
#define NRSTCTRL1_MAC1PONRST1	BIT(31)
#define NRSTCTRL_EMAC_MASK     (NRSTCTRL0_MAC0RST | NRSTCTRL0_MAC0PMARST | \
				 NRSTCTRL0_MAC0PONRST)
#define NCLKCTRL_EMAC_MASK     (NCLKCTRL0_MAC0TXCEN | NCLKCTRL0_MAC0RXCEN | \
				 NCLKCTRL0_MAC0125CLKEN | NCLKCTRL0_MAC0312CLKEN | \
				 NCLKCTRL1_MAC1RMCEN | NCLKCTRL0_MAC0ALLCLKEN)
#define NCLKCTRL0_COMMON_EMAC_MASK     (NCLKCTRL0_POEPLLCEN | NCLKCTRL0_SGMPCIEN | \
				 NCLKCTRL0_REFCLKOCEN)
#define NRSTCTRL0_DEFAULT	(NRSTCTRL0_MAC0PONRST | NRSTCTRL0_MAC0PMARST | \
					NRSTCTRL0_MAC0RST)
#define NRSTCTRL_COMMON (NRSTCTRL0_MSIGENRST  | NRSTCTRL0_UART0RST | \
					NRSTCTRL0_INTRST | NRSTCTRL0_MCURST)
#define NCLKCTRL_ENABLE_COMMON_EMAC_MASK (NCLKCTRL0_SRMCEM | NCLKCTRL0_I2SSPIEN | \
					NCLKCTRL0_PCIECEN | NCLKCTRL0_MCUCEN)
#define NCLKCTRL_DISABLE_COMMON_EMAC_MASK (NCLKCTRL0_MAC0ALLCLKEN | NCLKCTRL0_MAC0312CLKEN | \
								NCLKCTRL0_MAC0125CLKEN | NCLKCTRL0_REFCLKOCEN | \
								NCLKCTRL0_SGMPCIEN | NCLKCTRL0_POEPLLCEN | \
								NCLKCTRL0_MSIGENCEN | NCLKCTRL0_UARTOCEN | \
								NCLKCTRL0_MAC0RXCEN | NCLKCTRL0_MAC0TXCEN | \
								NCLKCTRL0_INTCEN)
#define NCLKCTRL_PORT0_EMAC_MASK     (NCLKCTRL0_MAC0TXCEN | NCLKCTRL0_MAC0RXCEN | \
				 NCLKCTRL0_MAC0125CLKEN | NCLKCTRL0_MAC0312CLKEN | \
				 NCLKCTRL0_MAC0ALLCLKEN)

#define TC956X_MSIGENCEN	18

#define NMISCCTL_OFFSET		(0x1800)

/* MSIGEN Registers */

#define TC956X_MSI_BASE		(0xF000)

#define TC956X_MSI_F_OFFSET		(0x0100)
#define TC956X_MSI_OUT_EN_OFFSET(pf_id, vf_id)	(TC956X_MSI_BASE +\
						(vf_id * TC956X_MSI_F_OFFSET) + (pf_id * TC956X_MSI_F_OFFSET) + (0x0000))
#define TC956X_MSI_MASK_SET_OFFSET(pf_id, vf_id)	(TC956X_MSI_BASE +\
						(vf_id * TC956X_MSI_F_OFFSET) + (pf_id * TC956X_MSI_F_OFFSET) + (0x0008))
#define TC956X_MSI_MASK_CLR_OFFSET(pf_id, vf_id)	(TC956X_MSI_BASE +\
						(vf_id * TC956X_MSI_F_OFFSET) + (pf_id * TC956X_MSI_F_OFFSET) + (0x000c))
#define TC956X_MSI_INT_STS_OFFSET(pf_id, vf_id)	(TC956X_MSI_BASE +\
						(vf_id * TC956X_MSI_F_OFFSET) + (pf_id * TC956X_MSI_F_OFFSET) + (0x0010))
#define TC956X_MSI_VECT_SET0_OFFSET(pf_id, vf_id)	(TC956X_MSI_BASE +\
						(vf_id * TC956X_MSI_F_OFFSET) + (pf_id * TC956X_MSI_F_OFFSET) + (0x0020))
#define TC956X_MSI_VECT_SET1_OFFSET(pf_id, vf_id)	(TC956X_MSI_BASE +\
						(vf_id * TC956X_MSI_F_OFFSET) + (pf_id * TC956X_MSI_F_OFFSET) + (0x0024))
#define TC956X_MSI_VECT_SET2_OFFSET(pf_id, vf_id)	(TC956X_MSI_BASE +\
						(vf_id * TC956X_MSI_F_OFFSET) + (pf_id * TC956X_MSI_F_OFFSET) + (0x0028))
#define TC956X_MSI_VECT_SET3_OFFSET(pf_id, vf_id)	(TC956X_MSI_BASE +\
						(vf_id * TC956X_MSI_F_OFFSET) + (pf_id * TC956X_MSI_F_OFFSET) + (0x002C))
#define TC956X_MSI_VECT_SET4_OFFSET(pf_id, vf_id)	(TC956X_MSI_BASE +\
						(vf_id * TC956X_MSI_F_OFFSET) + (pf_id * TC956X_MSI_F_OFFSET) + (0x0030))
#define TC956X_MSI_VECT_SET5_OFFSET(pf_id, vf_id)	(TC956X_MSI_BASE +\
						(vf_id * TC956X_MSI_F_OFFSET) + (pf_id * TC956X_MSI_F_OFFSET) + (0x0034))
#define TC956X_MSI_VECT_SET6_OFFSET(pf_id, vf_id)	(TC956X_MSI_BASE +\
						(vf_id * TC956X_MSI_F_OFFSET) + (pf_id * TC956X_MSI_F_OFFSET) + (0x0038))
#define TC956X_MSI_VECT_SET7_OFFSET(pf_id, vf_id)	(TC956X_MSI_BASE +\
						(vf_id * TC956X_MSI_F_OFFSET) + (pf_id * TC956X_MSI_F_OFFSET) + (0x003C))

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

/* EMAC control registers for ports 0 and 1 */
#define NEMAC0CTL_OFFSET	0x1070
#define NEMAC1CTL_OFFSET	0x1074

/* Fields and values for the NEMACxCTL registers */
#define NEMACCTL_SP_SEL_MASK			GENMASK(3, 0)
#define NEMACCTL_SP_SEL_SGMII_2500M		0x4	/* SGMII 2500M */
#define NEMACCTL_SP_SEL_SGMII_1000M		0x5	/* SGMII 1000M */
#define NEMACCTL_SP_SEL_USXGMII_10G_10G		0x8	/* USXGMII 10G/10G */

#define NEMACCTL_PHY_INF_SEL_MASK		GENMASK(5, 4)
/* XXX Fix this to use u32_assign_bits() */
/*	NEMACCTL_PHY_INF_SEL_PLL		0x00	clock from internal PLL */
#define NEMACCTL_PHY_INF_SEL_PHY		0x10	/* clock from PHY */

#define NEMACCTL_LPIHWCLKEN			BIT(8)	/* 1 = low power mode */

#define NEMACCTL_INIT_DONE			BIT(21)

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

#define SW_DSP1_ENABLE				(0x00000002)
#define SW_DSP2_ENABLE				(0x00000004)

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
	void __iomem *addr = td->sfr_addr + NFUNCEN4_OFFSET;

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
	u32 gpio_pin = td->phy_reset_gpio;
	u32 out_value = assert ? 0 : 1;
	void __iomem *addr;

	tc956x_phy_reset_pin_config(td);

	/* Output value for both pins is in the GPIOO0 register */
	addr = td->sfr_addr + GPIOO0_OFFSET;
	tc956x_reg_update(addr, BIT(gpio_pin), out_value);

	td->saved_phy_reset_value = out_value;

	/* Configure the GPIO pin in output direction */
	addr = td->sfr_addr + GPIOE0_OFFSET;
	tc956x_reg_update(addr, BIT(gpio_pin), 0);
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
	u32 out_value;

	tc956x_phy_reset_pin_config(td);

	out_value = td->saved_phy_reset_value;

	addr = td->sfr_addr + GPIOO0_OFFSET;
	tc956x_reg_update(addr, BIT(gpio_pin), out_value);

	/* Configure the GPIO pin in output direction */
	addr = td->sfr_addr + GPIOE0_OFFSET;
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

#define XPCS_REG_BASE_ADDR				10
#define XPCS_REG_OFFSET					0x0003FF
#define XPCS_IND_ACCESS					0x3FC

#define XPCS_USX_5G_MODE				(0x1 << 10)
#define XPCS_USX_2_5G_MODE				(0x2 << 10)


//
// Code from tc956x_xpcs.c in vendor driver
//

static u32 tc956x_xpcs_read(void __iomem *xpcsaddr, u32 pcs_reg_num)
{
	u32 reg_value;
	u16 base_address, offset;

	base_address = pcs_reg_num >> XPCS_REG_BASE_ADDR;
	offset = pcs_reg_num & XPCS_REG_OFFSET;

	/*write base address to (PCS address + 0x3FC) register*/
	writel(base_address, (xpcsaddr + XPCS_IND_ACCESS));

	/*Access to offset address (PCS address + offset)*/
	reg_value = readl(xpcsaddr + offset);
	pr_debug("XPCS register %x (%x@%x) indirect read access value : %x",
		 pcs_reg_num, base_address, offset, reg_value);

	return reg_value;
}

static u32 tc956x_xpcs_write(void __iomem *xpcsaddr, u32 pcs_reg_num, u32 value)
{
	u16 base_address, offset;

	base_address = pcs_reg_num >> XPCS_REG_BASE_ADDR;
	offset = pcs_reg_num & XPCS_REG_OFFSET;

	/*write base address to (PCS address + 0x3FC) register*/
	writel(base_address, (xpcsaddr + XPCS_IND_ACCESS));

	/*Access to offset address (PCS address + offset)*/
	writel(value, xpcsaddr + offset);
	pr_debug("XPCS register %x (%x@%x) indirect write access value : %x",
		 pcs_reg_num, base_address, offset, value);

	return 0;
}


static int tc956x_xpcs_init(struct plat_stmmacenet_data *plat)
{
	struct tc956x_data *td = plat->bsp_priv;
	void __iomem *xpcsaddr = td->sfr_addr +
					(td->emac0 ? MAC0_BASE_OFFSET
						   : MAC1_BASE_OFFSET) +
					 XPCS_XGMAC_OFFSET;
	u32 reg_value;

	reg_value = tc956x_xpcs_read(xpcsaddr, XGMAC_SR_MII_CTRL);
	if (reg_value & XGMAC_SOFT_RST)
		return -1;

	/*Clause 37 autoneg related settings*/
	if (plat->phy_interface == PHY_INTERFACE_MODE_SGMII) {
		//DK2
		//PCS Type Select SR_XS_PCS_CTRL2  PCS_TYPE_SEL -> 1
		reg_value = tc956x_xpcs_read(xpcsaddr, XGMAC_SR_XS_PCS_CTRL2);
		reg_value &= XGMAC_PCS_TYPE_SEL;
		reg_value |= 0x1;
		tc956x_xpcs_write(xpcsaddr, XGMAC_SR_XS_PCS_CTRL2, reg_value);

		reg_value = tc956x_xpcs_read(xpcsaddr, XGMAC_VR_MII_AN_CTRL);
		reg_value &= XGMAC_PCS_MODE_MASK;
		reg_value |= XGMAC_SGMII_MODE; /*SGMII PCS MODE*/
		tc956x_xpcs_write(xpcsaddr, XGMAC_VR_MII_AN_CTRL, reg_value);

		if (td->is_sgmii_2p5g == true) {
			reg_value = tc956x_xpcs_read(xpcsaddr,
						     XGMAC_VR_XS_PCS_DIG_CTRL1);
			reg_value &= ~(0x4);
			/* Enable only if SGMII 2.5G is enabled */
			reg_value |= 0x4; /*EN_2_5G_MODE*/
			tc956x_xpcs_write(xpcsaddr, XGMAC_VR_XS_PCS_DIG_CTRL1,
					  reg_value);
		}
	}
	if ((plat->phy_interface == PHY_INTERFACE_MODE_USXGMII) ||
			(plat->phy_interface == PHY_INTERFACE_MODE_10GKR) ||
			(plat->phy_interface == PHY_INTERFACE_MODE_10GBASER)) {
		reg_value = tc956x_xpcs_read(xpcsaddr, XGMAC_SR_XS_PCS_CTRL2);
		reg_value &= XGMAC_PCS_TYPE_SEL;/*PCS_TYPE_SEL as 10GBASE-R PCS */
		tc956x_xpcs_write(xpcsaddr, XGMAC_SR_XS_PCS_CTRL2, reg_value);

		reg_value = tc956x_xpcs_read(xpcsaddr, XGMAC_VR_XS_PCS_DIG_CTRL1);
		if (plat->phy_interface == PHY_INTERFACE_MODE_10GKR
			|| (plat->phy_interface == PHY_INTERFACE_MODE_10GBASER)
			) {
			reg_value &= (~XGMAC_USXG_EN); /*Disable USXG_EN*/
		} else {
			reg_value |= XGMAC_USXG_EN; /*set USXG_EN*/
		}

		tc956x_xpcs_write(xpcsaddr, XGMAC_VR_XS_PCS_DIG_CTRL1, reg_value);

		reg_value = tc956x_xpcs_read(xpcsaddr, XGMAC_VR_XS_PCS_KR_CTRL);
		reg_value &= ~XGMAC_USXG_MODE;/*USXG_MODE : 0x000*/
		tc956x_xpcs_write(xpcsaddr, XGMAC_VR_XS_PCS_KR_CTRL, reg_value);

		reg_value = tc956x_xpcs_read(xpcsaddr, XGMAC_VR_XS_PCS_DIG_CTRL1);
		reg_value |= XGMAC_VR_RST;/*set VR_RST*/
		tc956x_xpcs_write(xpcsaddr, XGMAC_VR_XS_PCS_DIG_CTRL1, reg_value);

		/*Wait for Reset to clear*/
		do {
			reg_value = tc956x_xpcs_read(xpcsaddr, XGMAC_VR_XS_PCS_DIG_CTRL1);
		} while ((XGMAC_VR_RST & reg_value) == XGMAC_VR_RST);

	}
	reg_value = tc956x_xpcs_read(xpcsaddr, XGMAC_SR_XS_PCS_CTRL1);
	reg_value |= XGMAC_LPI_ENABLE;/* LPM : power down */
	tc956x_xpcs_write(xpcsaddr, XGMAC_SR_XS_PCS_CTRL1, reg_value);

	reg_value = tc956x_xpcs_read(xpcsaddr, XGMAC_VR_XS_PCS_DIG_STS);
	reg_value &= ~(XGMAC_PSEQ_STATE);/* PSEQ_STATE(B4:2)=3'b000 */
	tc956x_xpcs_write(xpcsaddr, XGMAC_VR_XS_PCS_DIG_STS, reg_value);

	reg_value = tc956x_xpcs_read(xpcsaddr, XGMAC_SR_XS_PCS_CTRL1);
	reg_value &= ~(XGMAC_LPI_ENABLE);/* LPM : Normal Operation */
	tc956x_xpcs_write(xpcsaddr, XGMAC_SR_XS_PCS_CTRL1, reg_value);

	reg_value = tc956x_xpcs_read(xpcsaddr, XGMAC_SR_XS_PCS_EEE_ABL);
	reg_value |= XGMAC_KXEEE;/* KXEEE */
	tc956x_xpcs_write(xpcsaddr, XGMAC_SR_XS_PCS_EEE_ABL, reg_value);

	reg_value = tc956x_xpcs_read(xpcsaddr, XGMAC_VR_XS_PCS_EEE_MCTRL0);
	reg_value &= ~(XGMAC_MULT_FACT_100NS);
	reg_value |= XGMAC_MULT_FACT_100NS_MAC; /* MULT_FACT_100NS */
	reg_value |= XGMAC_SIGN_BIT;/* SIGN_BIT */
	reg_value |= XGMAC_TX_RX_EN;/* TX_EN_CTRL, RX_EN_CTRL */
	tc956x_xpcs_write(xpcsaddr, XGMAC_VR_XS_PCS_EEE_MCTRL0, reg_value);

	reg_value = tc956x_xpcs_read(xpcsaddr, XGMAC_VR_XS_PCS_EEE_TXTIMER);
	reg_value &= ~(XGMAC_EEE_TX_TIMER);
	reg_value |= XGMAC_EEE_TX_TIMER_MAC_CONT; /* TWL_RES=0x5, T1U_RES=0x1, TSL_RES=0x3 */
	tc956x_xpcs_write(xpcsaddr, XGMAC_VR_XS_PCS_EEE_TXTIMER, reg_value);

	reg_value = tc956x_xpcs_read(xpcsaddr, XGMAC_VR_XS_PCS_EEE_RXTIMER);
	reg_value &= ~(XGMAC_EEE_RX_TIMER);
	reg_value |= XGMAC_EEE_RX_TIMER_MAC_CONT; /* TWR_RES=0x6, RES_100U=0x42 */
	tc956x_xpcs_write(xpcsaddr, XGMAC_VR_XS_PCS_EEE_RXTIMER, reg_value);

	reg_value = tc956x_xpcs_read(xpcsaddr, XGMAC_VR_XS_PCS_EEE_MCTRL1);
	reg_value |= XGMAC_TRN_LPI;
	tc956x_xpcs_write(xpcsaddr, XGMAC_VR_XS_PCS_EEE_MCTRL1, reg_value);

	reg_value = tc956x_xpcs_read(xpcsaddr, XGMAC_VR_XS_PCS_EEE_MCTRL0);

	reg_value &= ~XGMAC_TX_RX_QUIET_EN;
	reg_value |= XGMAC_TX_RX_QUIET_EN; /* RX_QUIET_EN, TX_QUIET_EN, LRX_EN, LTX_EN */

	tc956x_xpcs_write(xpcsaddr, XGMAC_VR_XS_PCS_EEE_MCTRL0, reg_value);

	reg_value = tc956x_xpcs_read(xpcsaddr, XGMAC_VR_MII_AN_CTRL);
	reg_value &= XGMAC_TX_CFIG_INTR_EN_MASK;/*TX_CONFIG MAC SIDE*/
	reg_value |= XGMAC_MII_AN_INTR_EN;/*MII_AN_INTR_EN enabe*/
	tc956x_xpcs_write(xpcsaddr, XGMAC_VR_MII_AN_CTRL, reg_value);

	reg_value = tc956x_xpcs_read(xpcsaddr, XGMAC_VR_MII_DIG_CTRL1);
	reg_value &= ~XGMAC_MAC_AUTO_SW_EN;/*MAC_AUTO_SW enable*/
	if (td->is_sgmii_2p5g != true)
		/* Enable only if SGMII 2.5G is not enabled. */
		reg_value |= XGMAC_MAC_AUTO_SW_EN;
	tc956x_xpcs_write(xpcsaddr, XGMAC_VR_MII_DIG_CTRL1, reg_value);

	return 0;
}

static void tc956x_xpcs_ctrl_ane(struct tc956x_data *td, bool ane)
{
	void __iomem *xpcsaddr = td->sfr_addr +
					(td->emac0 ? MAC0_BASE_OFFSET
						   : MAC1_BASE_OFFSET) +
					 XPCS_XGMAC_OFFSET;
	u32 reg_value;

	reg_value = tc956x_xpcs_read(xpcsaddr, XGMAC_SR_MII_CTRL);
	if (ane) {
		reg_value |= XGMAC_AN_37_ENABLE;
		dev_dbg(td->dev, "%s Enable AN", __func__);
	} else {
		reg_value &= (~XGMAC_AN_37_ENABLE);
		dev_dbg(td->dev, "%s Disable AN", __func__);
	}

	tc956x_xpcs_write(xpcsaddr, XGMAC_SR_MII_CTRL, reg_value);
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
	u8 pf_no = td->emac0 ? 0 : 1, vf_no = 0;
	u32 rd_val;

	// TODO: this is the #ifdef EEE_MAC_CONTROLLED_MODE stanza from
	//       tc965xmac_main.c. It's nothing to do with xpcs but it
	//       appears just before MSI initialization so this gets
	//       things turned on in the same order we see in the vendor
	//       driver
	rd_val = readl(td->sfr_addr + NCLKCTRL0_OFFSET);
	if (td->emac0)
		rd_val |= (NCLKCTRL0_MAC0312CLKEN | NCLKCTRL0_MAC0125CLKEN);
	rd_val |= (NCLKCTRL0_POEPLLCEN | NCLKCTRL0_SGMPCIEN | NCLKCTRL0_REFCLKOCEN);
	writel(rd_val, td->sfr_addr + NCLKCTRL0_OFFSET);

	/* Enable MSIGEN Module */
	rd_val = readl(td->sfr_addr + NCLKCTRL0_OFFSET);
	rd_val |= (1 << TC956X_MSIGENCEN);
	writel(rd_val, td->sfr_addr + NCLKCTRL0_OFFSET);
	rd_val = readl(td->sfr_addr + NRSTCTRL0_OFFSET);
	rd_val &= ~(1 << TC956X_MSIGENCEN);
	writel(rd_val, td->sfr_addr + NRSTCTRL0_OFFSET);

	/* Initialize MSIGEN */

	writel(TC956X_MSI_OUT_EN_CLR, td->sfr_addr + TC956X_MSI_OUT_EN_OFFSET(pf_no, vf_no));
	writel(TC956X_MSI_MASK_SET, td->sfr_addr + TC956X_MSI_MASK_SET_OFFSET(pf_no, vf_no));
	writel(TC956X_MSI_MASK_CLR, td->sfr_addr + TC956X_MSI_MASK_CLR_OFFSET(pf_no, vf_no));
	/* DMA Ch Tx-Rx Interrupt sources are assigned to Vector 0,
	 * All other Interrupt sources are assigned to Vector 1
	 */
	writel(TC956X_MSI_SET0, td->sfr_addr + TC956X_MSI_VECT_SET0_OFFSET(pf_no, vf_no));
	writel(TC956X_MSI_SET1, td->sfr_addr + TC956X_MSI_VECT_SET1_OFFSET(pf_no, vf_no));
	writel(TC956X_MSI_SET2, td->sfr_addr + TC956X_MSI_VECT_SET2_OFFSET(pf_no, vf_no));
	writel(TC956X_MSI_SET3, td->sfr_addr + TC956X_MSI_VECT_SET3_OFFSET(pf_no, vf_no));
	writel(TC956X_MSI_SET4, td->sfr_addr + TC956X_MSI_VECT_SET4_OFFSET(pf_no, vf_no));
	writel(TC956X_MSI_SET5, td->sfr_addr + TC956X_MSI_VECT_SET5_OFFSET(pf_no, vf_no));
	writel(TC956X_MSI_SET6, td->sfr_addr + TC956X_MSI_VECT_SET6_OFFSET(pf_no, vf_no));
	writel(TC956X_MSI_SET7, td->sfr_addr + TC956X_MSI_VECT_SET7_OFFSET(pf_no, vf_no));
}

static u32 tc956x_interrupt_sts(struct stmmac_priv *priv, struct net_device *dev)
{
	struct tc956x_data *td = priv->plat->bsp_priv;
	u8 pf_no = td->emac0 ? 0 : 1;
	u8 vf_no = 0;

	return readl(td->sfr_addr + TC956X_MSI_INT_STS_OFFSET(pf_no, vf_no));
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
	u8 pf_no = td->emac0 ? 0 : 1;
	u32 mask_val = 0;
	u8 vf_no = 0;

#if 0
	// This table is copied from tc956xmac_main.c and is almost certainly
	// wrong in some way (subtle or otherwise)
	bool tx_ch_in_use[8];
	bool rx_ch_in_use[8];
	tx_ch_in_use[0] = true;
	tx_ch_in_use[1] = false;
	tx_ch_in_use[2] = false;
	tx_ch_in_use[3] = false;
	tx_ch_in_use[4] = true;
	tx_ch_in_use[5] = true;
	tx_ch_in_use[6] = true;
	tx_ch_in_use[7] = true;
	rx_ch_in_use[0] = true;
	rx_ch_in_use[1] = false;
	rx_ch_in_use[2] = false;
	rx_ch_in_use[3] = true;
	rx_ch_in_use[4] = true;
	rx_ch_in_use[5] = true;
	rx_ch_in_use[6] = true;
	rx_ch_in_use[7] = true;
#endif

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

		writel(mask_val, td->sfr_addr + TC956X_MSI_OUT_EN_OFFSET(pf_no, vf_no));
	} else
		writel(TC956X_MSI_OUT_EN_CLR, td->sfr_addr + TC956X_MSI_OUT_EN_OFFSET(pf_no, vf_no));
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
	u8 pf_no = td->emac0 ? 0 : 1;
	u8 vf_no = 0;

	writel((1<<vector), td->sfr_addr + TC956X_MSI_MASK_CLR_OFFSET(pf_no, vf_no));
}

const struct tc956x_msi_ops tc956x_msigen_ops = {
	.init = tc956x_msigen_init,
	.interrupt_sts = tc956x_interrupt_sts,
	.interrupt_en = tc956x_interrupt_en,
	.interrupt_clr = tc956x_interrupt_clr,
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

/* XXX
 * I don't think this mutex is needed at all.  It mutually excludes the
 * probe, remove, suspend, and remove callbacks from being called
 * concurrently, but the driver core core ought to guarantee that
 * won't haeppn.
 */
static DEFINE_MUTEX(tc956x_pm_suspend_lock);

struct tx956x_shrd_mem tx956x_pci_shrd_mem[TC956X_TOT_CASCADE_DEV];

static uint16_t tc956x_get_shared_mem_offset(struct pci_dev *pdev, uint16_t pci_bd)
{
	uint16_t offset;

	for (offset = 0; offset < TC956X_TOT_CASCADE_DEV; offset++) {
		if (tx956x_pci_shrd_mem[offset].pci_bd == 0) {
			tx956x_pci_shrd_mem[offset].pci_bd = pci_bd;
			dev_dbg(&pdev->dev, "New shared memory offset %d allocated\n", offset);
			return offset;	/* Free memory is available */
		} else if (tx956x_pci_shrd_mem[offset].pci_bd == pci_bd) {
			dev_dbg(&pdev->dev, "Existing shared memory offset %d found\n", offset);
			return offset;	/* Allocated memory found */
		}
	}
	return 0xFFFF;
}

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

struct tc956x_pci_info {
	int (*setup)(struct pci_dev *pdev, struct plat_stmmacenet_data *plat);
};

/**
 * tc956x_pm_set_power() - Set clock and reset for suspend or resume
 * @priv:	STMMAC driver private data pointer
 * @state:	Whether we are being called during suspend
 *
 * Save the eMAC clock and reset settings before suspend, or restore those
 * settings during resume.
 */
static void tc956x_pm_set_power(struct stmmac_priv *priv, enum TC956X_PORT_PM_STATE state)
{
	struct tc956x_data *td = priv->plat->bsp_priv;

	void *nrst_reg = NULL, *nclk_reg = NULL, *commonclk_reg = NULL;
	u32 nrst_val = 0, nclk_val = 0, commonclk_val = 0;

	pr_debug("-->%s : Port %d interface %s", __func__, td->emac0 ? 0 : 1,
		 priv->dev->name);
	/* Select register address by port */
	if (td->emac0) {
		nrst_reg = td->sfr_addr + NRSTCTRL0_OFFSET;
		nclk_reg = td->sfr_addr + NCLKCTRL0_OFFSET;
	} else {
		nrst_reg = td->sfr_addr + NRSTCTRL1_OFFSET;
		nclk_reg = td->sfr_addr + NCLKCTRL1_OFFSET;
	}

	if (state == SUSPEND) {
		pr_debug("%s : Port %d interface %s Set Power for Suspend",
			 __func__, td->emac0 ? 0 : 1, priv->dev->name);
		/* Modify register for reset, clock and MSI_OUTEN */
		nrst_val = readl(nrst_reg);
		nclk_val = readl(nclk_reg);
		pr_debug("%s : Port %d interface %s Rd RST Reg:%x, CLK Reg:%x",
			 __func__, td->emac0 ? 0 : 1, priv->dev->name,
			 nrst_val, nclk_val);
		/* Save values before Asserting reset and Clock Disable */
		td->pm_saved_emac_rst = nrst_val & NRSTCTRL_EMAC_MASK;
		td->pm_saved_emac_clk = nclk_val & NCLKCTRL_EMAC_MASK;
		nrst_val = nrst_val | NRSTCTRL_EMAC_MASK;
		nclk_val = nclk_val & ~NCLKCTRL_EMAC_MASK;
		writel(nrst_val, nrst_reg);
		writel(nclk_val, nclk_reg);
		if (tx956x_pci_shrd_mem[td->pci_bd].pci_dev_active_cnt == TC956X_ALL_MAC_PORT_SUSPENDED) {
			commonclk_reg = td->sfr_addr + NCLKCTRL0_OFFSET;
			commonclk_val = readl(commonclk_reg);
			pr_debug("%s : Port %d interface %s Common CLK Rd Reg:%x",
				 __func__, td->emac0 ? 0 : 1,
				 priv->dev->name, commonclk_val);
			/* Clear Common Clocks only when both port suspends */
			commonclk_val = commonclk_val & ~NCLKCTRL0_COMMON_EMAC_MASK;
			writel(commonclk_val, commonclk_reg);
			pr_debug("%s : Port %d interface %s Common CLK Wr Reg:%x",
				 __func__, td->emac0 ? 0 : 1,
				 priv->dev->name, commonclk_val);
		}
	} else if (state == RESUME) {
		pr_debug("%s : Port %d interface %s Set Power for Resume",
			 __func__, td->emac0 ? 0 : 1, priv->dev->name);
		if (tx956x_pci_shrd_mem[td->pci_bd].pci_dev_active_cnt == TC956X_ALL_MAC_PORT_SUSPENDED) {
			commonclk_reg = td->sfr_addr + NCLKCTRL0_OFFSET;
			commonclk_val = readl(commonclk_reg);
			pr_debug("%s : Port %d interface %s Common CLK Rd Reg:%x",
				 __func__, td->emac0 ? 0 : 1, priv->dev->name,
				 commonclk_val);
			/* Clear Common Clocks only when both port suspends */
			commonclk_val = commonclk_val | NCLKCTRL0_COMMON_EMAC_MASK;
			writel(commonclk_val, commonclk_reg);
			pr_debug("%s : Port %d interface %s Common CLK WR Reg:%x",
				 __func__, td->emac0 ? 0 : 1,
				 priv->dev->name, commonclk_val);
		}
		nrst_val = readl(nrst_reg);
		nclk_val = readl(nclk_reg);
		pr_debug("%s : Port %d interface %s Rd RST Reg:%x, CLK Reg:%x",
			 __func__, td->emac0 ? 0 : 1, priv->dev->name,
			 nrst_val, nclk_val);
		/* Restore values same as before suspend */
		nrst_val = (nrst_val & ~NRSTCTRL_EMAC_MASK) | td->pm_saved_emac_rst;
		nclk_val = nclk_val | td->pm_saved_emac_clk; /* Restore Clock */
		writel(nclk_val, nclk_reg);
		writel(nrst_val, nrst_reg);
	}
	pr_debug("%s : Port %d interface %s td->pm_saved_emac_rst %x td->pm_saved_emac_clk %x",
		 __func__, td->emac0 ? 0 : 1, priv->dev->name,
		 td->pm_saved_emac_rst, td->pm_saved_emac_clk);
	pr_debug("%s : Port %d %s Wr RST Reg:%x, CLK Reg:%x", __func__,
		td->emac0 ? 0 : 1, priv->dev->name,
		readl(nrst_reg), readl(nclk_reg));
	pr_debug("<--%s : Port %d interface %s", __func__, td->emac0 ? 0 : 1,
		 priv->dev->name);
}

static void tc956x_get_interfaces(struct stmmac_priv *priv, void *bsp_priv,
				  unsigned long *interfaces)
{
	struct tc956x_data *td = bsp_priv;

	/*
	 * To handle 2.5G PHYs via (overclocked) SGMII then we need set both
	 * SGMII and 2500BASEX are supported interfaces.
	 */
	if (td->port_interface == ENABLE_SGMII_INTERFACE) {
		__set_bit(PHY_INTERFACE_MODE_SGMII, interfaces);
		__set_bit(PHY_INTERFACE_MODE_2500BASEX, interfaces);
	}
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
	 * TODO: tx_queues_to_use would normally be set to 8
	 *
	 * 1. ping stops working if we set tx_queues_to_use to 8
	 * 2. functional reliability is poor of tx_queues_to_use is >2
	 *    (DHCP fails to get IP address)
	 */
	plat->tx_queues_to_use = 2;
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
 * tc956x_zero_sram() - Reset SRAM region
 * @td:		TC956x driver private data pointer
 *
 * Reset the IMEM and DMEM memory in the tc956x.
 */
static void tc956x_zero_sram(struct tc956x_data *td)
{
	memset_io(td->sram_addr, 0x0, 0x10000);			/* IMEM */
	memset_io(td->sram_addr + 0x40000, 0x0, 0x10000);	/* DMEM */
}

/**
 * tc956x_load_firmware() - Load firmware for the embedded Cortex M3
 * @td:		TC956x driver private data pointer
 *
 * Load the firmware into the SRAM in the TC956x.  The embedded Cortex M3
 * starts executing once the firmware loading is complete.
 *
 * Return:	0 if successful, or an error code if an error occurs
 */
static s32 tc956x_load_firmware(struct tc956x_data *td)
{
	struct device *dev = td->dev;
	const struct firmware *pfw;
	u32 fw_init_sync;
	u32 adrs;
	u32 val;

	dev_dbg(dev,  "FW Loading: .bin\n");

	/* Get TC956X FW binary through kernel firmware interface request */
	if (request_firmware(&pfw, FIRMWARE_NAME, dev)) {
		dev_err(dev, "TC956X: Error in calling request_firmware");
		return -EINVAL;
	}

	/* Validate the size of the firmware */
	if (pfw->size > TC956X_FW_MAX_SIZE) {
		dev_err(dev, "Error : FW size exceeds the memory size\n");
		return -EINVAL;
	}

	dev_dbg(dev,  "FW Loading Start...\n");
	dev_dbg(dev,  "FW Size = %ld\n", (long)(pfw->size));

	/* Assert M3 reset */
	adrs = NRSTCTRL0_OFFSET;
	val = ioread32(td->sfr_addr + adrs);
	dev_dbg(dev,  "Reset Register value = %lx\n", (unsigned long)val);

	val |= NRSTCTRL0_RST_ASRT;
	iowrite32(val, td->sfr_addr + adrs);

	iowrite32(0, td->sram_addr + TC956X_M3_INIT_DONE);

	tc956x_zero_sram(td);

	mdelay(10);
	iowrite8(EEPROM_OFFSET,
		 td->sram_addr + TC956X_M3_SRAM_EEPROM_OFFSET_ADDR);
	iowrite8(EEPROM_MAC_COUNT,
		 td->sram_addr + TC956X_M3_SRAM_EEPROM_MAC_COUNT);

	/* Copy TC956X FW to SRAM */
	memcpy_toio(td->sram_addr, pfw->data, pfw->size);
	/* Release kernel firmware interface */
	release_firmware(pfw);

	dev_dbg(dev,  "FW Loading Finish.\n");

	/* De-assert M3 reset */
	adrs = NRSTCTRL0_OFFSET;
	val = ioread32(td->sfr_addr + adrs);
	val &= ~NRSTCTRL0_RST_DE_ASRT;
	iowrite32(val, td->sfr_addr + adrs);

	readl_poll_timeout(td->sram_addr + TC956X_M3_INIT_DONE,
				fw_init_sync, fw_init_sync, 100, 100000);
	if (!fw_init_sync)
		dev_alert(dev, "TC956x FW yet to start!!!");
	else
		dev_dbg(dev,  "TC956x M3 started.\n");

	return 0;
}

/*
 * brief API to populate the table address map registers.
 *
 * details This function pouplates the registers that are used to convert the
 * AXI bus access to PCI TLP.
 *
 * param[in] dev  - pointer to device structure.
 * param[in] id   - pointer to base address of registers.
 */
static void tc956x_config_tamap(struct device *dev,
				void __iomem *reg_pci_base_addr)
{
	u32 table_entry;

	/* Set all entries to default */
	for (table_entry = 0; table_entry <= MAX_CM3_TAMAP_ENTRIES; table_entry++) {

		writel(TC956X_AXI4_SLV00_TRSL_PARAM_VAL, reg_pci_base_addr +
					TC956X_AXI4_SLV_TRSL_PARAM(0, table_entry));
		writel(0x0, reg_pci_base_addr +
					TC956X_AXI4_SLV_TRSL_ADDR_HI(0, table_entry));
		writel(0x0, reg_pci_base_addr +
					TC956X_AXI4_SLV_TRSL_ADDR_LO(0, table_entry));
		writel(0x0, reg_pci_base_addr +
					TC956X_AXI4_SLV_SRC_ADDR_HI(0, table_entry));
		writel(TC956X_AXI4_SLV00_SRC_ADDR_LO_VAL_DEFAULT, reg_pci_base_addr +
					TC956X_AXI4_SLV_SRC_ADDR_LO(0, table_entry));

	}



	/* AXI4 Slave 0 - Table 0 Entry */
	/* EDMA address region 0x10 0000 0000 - 0x1F FFFF FFFF is
	 * translated to 0x0 0000 0000 - 0xF FFFF FFFF
	 */
	writel(TC956X_AXI4_SLV00_TRSL_PARAM_VAL, reg_pci_base_addr +
					TC956X_AXI4_SLV_TRSL_PARAM(0, 0));
	writel(TC956X_AXI4_SLV00_TRSL_ADDR_HI_VAL, reg_pci_base_addr +
					TC956X_AXI4_SLV_TRSL_ADDR_HI(0, 0));
	writel(TC956X_AXI4_SLV00_TRSL_ADDR_LO_VAL, reg_pci_base_addr +
					TC956X_AXI4_SLV_TRSL_ADDR_LO(0, 0));
	writel(TC956X_AXI4_SLV00_SRC_ADDR_HI_VAL, reg_pci_base_addr +
					TC956X_AXI4_SLV_SRC_ADDR_HI(0, 0));
	writel(TC956X_AXI4_SLV00_SRC_ADDR_LO_VAL |
				TC956X_ATR_SIZE(TC956X_AXI4_SLV00_ATR_SIZE) |
				TC956X_ATR_IMPL, reg_pci_base_addr +
				TC956X_AXI4_SLV_SRC_ADDR_LO(0, 0));

	pr_debug("SL00 TRSL_MASK = 0x%08x\n",
		readl(reg_pci_base_addr + TC956X_AXI4_SLV_TRSL_MASK1(0, 0)));
	pr_debug("SL00 TRSL_MASK = 0x%08x\n",
		readl(reg_pci_base_addr + TC956X_AXI4_SLV_TRSL_MASK2(0, 0)));
	pr_debug("SL00 TRSL_PARAM = 0x%08x\n",
		readl(reg_pci_base_addr + TC956X_AXI4_SLV_TRSL_PARAM(0, 0)));
	pr_debug("SL00 TRSL_ADDR HI = 0x%08x\n",
		readl(reg_pci_base_addr + TC956X_AXI4_SLV_TRSL_ADDR_HI(0, 0)));
	pr_debug("SL00 TRSL_ADDR LO = 0x%08x\n",
		readl(reg_pci_base_addr + TC956X_AXI4_SLV_TRSL_ADDR_LO(0, 0)));
	pr_debug("SL00 SRC_ADDR HI = 0x%08x\n",
		readl(reg_pci_base_addr + TC956X_AXI4_SLV_SRC_ADDR_HI(0, 0)));
	pr_debug("SL00 SRC_ADDR LO = 0x%08x\n",
		readl(reg_pci_base_addr + TC956X_AXI4_SLV_SRC_ADDR_LO(0, 0)));
}

static int tc956x_dwmac_setup(void *apriv, struct mac_device_info *mac)
{
	//struct stmmac_priv *priv = apriv;
	//struct tc956x_data *td = priv->plat->bsp_priv;

	mac->msi = &tc956x_msigen_ops;

	return 0;
}

static void tc956x_fix_mac_speed(void *bsp_priv, int speed, unsigned int mode)
{
	struct tc956x_data *td = bsp_priv;
	struct plat_stmmacenet_data *plat = td->plat;
	int ret, reg = 0, val, reg_value;
	void __iomem *ioaddr =
		td->sfr_addr +
		(td->emac0 ? MAC0_BASE_OFFSET : MAC1_BASE_OFFSET);
	void __iomem *xpcsaddr = ioaddr + XPCS_XGMAC_OFFSET;
	bool enable_an = true;

	// TODO: copied from vendor drivers customizations in
	//       tc956x_speed_change_init_mac()

	if (td->emac0) {
		// TODO
		BUG();
	} else {
		/* Enable all clocks to eMAC Port1 */
		ret = readl(td->sfr_addr + NCLKCTRL1_OFFSET);
		if (td->plat->phy_interface == PHY_INTERFACE_MODE_SGMII &&
		    speed == SPEED_2500) {
			ret &= ~NCLKCTRL1_MAC1125CLKEN1;
			ret &= ~NCLKCTRL1_MAC1312CLKEN1;
		} else {
			ret &= ~NCLKCTRL1_MAC1312CLKEN1;
			ret |= NCLKCTRL1_MAC1125CLKEN1;
		}
		writel(ret, td->sfr_addr + NCLKCTRL1_OFFSET);

		/* Interface configuration for port1*/
		ret = readl(td->sfr_addr + NEMAC1CTL_OFFSET);
		ret &= ~(NEMACCTL_SP_SEL_MASK | NEMACCTL_PHY_INF_SEL_MASK);
		if (plat->phy_interface == PHY_INTERFACE_MODE_SGMII) {
			if (speed == SPEED_2500)
				ret |= NEMACCTL_SP_SEL_SGMII_2500M;
			else
				ret |= NEMACCTL_SP_SEL_SGMII_1000M;
		} else {
			reg_value = tc956x_xpcs_read(xpcsaddr, XGMAC_VR_XS_PCS_KR_CTRL);
			reg_value &= ~XGMAC_USXG_MODE; /*USXG_MODE : 0x000*/
			tc956x_xpcs_write(xpcsaddr, XGMAC_VR_XS_PCS_KR_CTRL, reg_value);
		}

		ret &= ~(0x00000040); /* Mask Polarity */
		ret |= NEMACCTL_PHY_INF_SEL_PHY | NEMACCTL_LPIHWCLKEN;
		writel(ret, td->sfr_addr + NEMAC1CTL_OFFSET);
		writel(reg, td->sfr_addr + NMISCCTL_OFFSET);

	}

	if (td->emac0) {
		/* Assertion of PMA & XPCS reset software Reset*/
		ret = readl(td->sfr_addr + NRSTCTRL0_OFFSET);
		ret |= (NRSTCTRL0_MAC0PMARST | NRSTCTRL0_MAC0PONRST);
		writel(ret, td->sfr_addr + NRSTCTRL0_OFFSET);
	} else {
		/* Assertion of PMA &  XPCS reset  software Reset*/
		ret = readl(td->sfr_addr + NRSTCTRL1_OFFSET);
		ret |= (NRSTCTRL1_MAC1PMARST1 | NRSTCTRL1_MAC1PONRST1);
		writel(ret, td->sfr_addr + NRSTCTRL1_OFFSET);
	}

	ret = tc956x_pma_init(NULL, ioaddr + PMA_XGMAC_OFFSET);
	if (ret < 0)
		pr_info("PMA switching to internal clock Failed\n");

	if (td->emac0) {
		/* De-assertion of PMA & XPCS reset software Reset*/
		ret = readl(td->sfr_addr + NRSTCTRL0_OFFSET);
		ret &= ~(NRSTCTRL0_MAC0PMARST | NRSTCTRL0_MAC0PONRST);
		ret &= ~(NRSTCTRL0_MAC0RST | NRSTCTRL0_MAC0RST);

		writel(ret, td->sfr_addr + NRSTCTRL0_OFFSET);
	} else {
		/* De-assertion of PMA &  XPCS reset software Reset*/
		ret = readl(td->sfr_addr + NRSTCTRL1_OFFSET);
		ret &= ~(NRSTCTRL1_MAC1PMARST1 | NRSTCTRL1_MAC1PONRST1);
		writel(ret, td->sfr_addr + NRSTCTRL1_OFFSET);
	}

	ret = readl_poll_timeout(td->sfr_addr + (td->emac0 ? NEMAC0CTL_OFFSET
						           : NEMAC1CTL_OFFSET),
				 val, val & NEMACCTL_INIT_DONE, 50, 1000000);
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
	struct device *dev = &pdev->dev;
	struct stmmac_resources res;
	struct tc956x_data *td;
	/* use signal from EMSPHY */
	uint16_t sh_mem_offset;
	uint8_t SgmSigPol = 0;
	u32 pfn;
	u32 val;
	int ret;

	mutex_lock(&tc956x_pm_suspend_lock);

	plat = stmmac_plat_dat_alloc(dev);
	if (!plat) {
		ret = -ENOMEM;
		goto err_out_enb_failed;
	}

	td = devm_kzalloc(dev, sizeof(*td), GFP_KERNEL);
	plat->bsp_priv = td;
	td->plat = plat;

	plat->mdio_bus_data = devm_kzalloc(dev, sizeof(*plat->mdio_bus_data),
					   GFP_KERNEL);
	if (!plat->mdio_bus_data) {
		ret = -ENOMEM;
		goto err_out_enb_failed;
	}

	plat->dma_cfg = devm_kzalloc(dev, sizeof(*plat->dma_cfg), GFP_KERNEL);
	if (!plat->dma_cfg) {
		ret = -ENOMEM;
		goto err_out_enb_failed;
	}

	/* Enable pci device */
	ret = pci_enable_device(pdev);
	if (ret) {
		dev_err(dev, "%s: ERROR: failed to enable device\n", __func__);
		goto err_out_enb_failed;
	}

	/* Request the PCI IO Memory for the device */
	if (pci_request_regions(pdev, DRIVER_NAME)) {
		dev_err(dev, "%s:Failed to get PCI regions\n", DRIVER_NAME);
		ret = -ENODEV;
		goto err_out_req_reg_failed;
	}
	memset(&res, 0, sizeof(res));

	/* Enable the bus mastering */
	pci_set_master(pdev);

	dev_dbg(dev,
		"BAR0 length = %lld bytes\n", (u64)pci_resource_len(pdev, 0));
	dev_dbg(dev,
		"BAR2 length = %lld bytes\n", (u64)pci_resource_len(pdev, 2));
	dev_dbg(dev,
		"BAR4 length = %lld bytes\n", (u64)pci_resource_len(pdev, 4));
	dev_dbg(dev,
		"BAR0 physical address = 0x%llx\n", (u64)pci_resource_start(pdev, 0));
	dev_dbg(dev,
		"BAR2 physical address = 0x%llx\n", (u64)pci_resource_start(pdev, 2));
	dev_dbg(dev,
		"BAR4 physical address = 0x%llx\n", (u64)pci_resource_start(pdev, 4));

	// TODO: devm_pci_iomap?
	td->bridge_cfg_addr = ioremap(pci_resource_start(pdev, TC956X_BAR0),
				      pci_resource_len(pdev, TC956X_BAR0));
	if (!td->bridge_cfg_addr) {
		dev_err(dev, "%s: cannot map TC956X BAR0, aborting", pci_name(pdev));
		ret = -EIO;
		goto err_out_map_failed;
	}
	td->sram_addr = ioremap(pci_resource_start(pdev, TC956X_BAR2),
				pci_resource_len(pdev, TC956X_BAR2));
	if (!td->sram_addr) {
		pci_iounmap(pdev, td->bridge_cfg_addr);
		dev_err(dev, "%s: cannot map TC956X BAR2, aborting", pci_name(pdev));
		ret = -EIO;
		goto err_out_map_failed;
	}
	td->sfr_addr = ioremap(pci_resource_start(pdev, TC956X_BAR4),
			       pci_resource_len(pdev, TC956X_BAR4));
	if (!td->sfr_addr) {
		pci_iounmap(pdev, td->bridge_cfg_addr);
		pci_iounmap(pdev, td->sram_addr);
		dev_err(dev, "%s: cannot map TC956X BAR4, aborting", pci_name(pdev));
		ret = -EIO;
		goto err_out_map_failed;
	}

	dev_dbg(dev, "BAR0 virtual address = %p\n", td->bridge_cfg_addr);
	dev_dbg(dev, "BAR2 virtual address = %p\n", td->sram_addr);
	dev_dbg(dev, "BAR4 virtual address = %p\n", td->sfr_addr);

#if IS_ENABLED(CONFIG_TRACE_MMIO_ACCESS)
	/*
	 *  TODO: This is the filtering/tagging support for MMIO tracing.
	 *
	 * Eventually it needs to be removed but not yet... it's too useful
	 * for feature development!
	 */
	log_mmio_register_range(td->bridge_cfg_addr, pci_resource_len(pdev, 0), "bridge_cfg");
	//log_mmio_register_range(td->sram_addr, pci_resource_len(pdev, 2), "sram");
	log_mmio_register_range(td->sfr_addr, pci_resource_len(pdev, 4), "sfr");
#endif

	/* Determine physical port number from the resource manager */
	val = readl(td->bridge_cfg_addr + RSCMNG_ID_REG);
	pfn = FIELD_GET(RSCMNG_PFN_MASK, val);
	if (WARN_ON(pfn > 1))
		return -EINVAL;
	td->emac0 = pfn == 0;

	td->dev = dev;

	res.addr = td->sfr_addr +
		   (td->emac0 ? MAC0_BASE_OFFSET : MAC1_BASE_OFFSET);

	plat->mac_setup = &tc956x_dwmac_setup;
	plat->fix_mac_speed = &tc956x_fix_mac_speed;

	// NCID_OFFSET gives the revision ID (and early revisions are limited
	// to 2.5G)
	pr_debug("NCID Register value: %x\n", readl(td->sfr_addr + NCID_OFFSET));

	td->port_interface = td->emac0 ? ENABLE_XFI_INTERFACE
				       : ENABLE_SGMII_INTERFACE;

	ret = tc956x_xgmac3_default_data(pdev, plat);
	if (ret)
		goto err_out_enb_failed;

	dev_dbg(dev, "port_interface = %d\n", td->port_interface);

	if (td->emac0) {
		ret = readl(td->sfr_addr + NRSTCTRL0_OFFSET);
		ret |= (NRSTCTRL0_INTRST);
		writel(ret, td->sfr_addr + NRSTCTRL0_OFFSET);

		ret = readl(td->sfr_addr + NCLKCTRL0_OFFSET);
		ret |= NCLKCTRL0_INTCEN;
		writel(ret, td->sfr_addr + NCLKCTRL0_OFFSET);

		ret = readl(td->sfr_addr + NRSTCTRL0_OFFSET);
		ret &= (~(NRSTCTRL0_INTRST));
		writel(ret, td->sfr_addr + NRSTCTRL0_OFFSET);

		/* Configure Address Transslation block
		 * Bridge Base address to be passed for TC956X
		 */
		tc956x_config_tamap(dev, td->bridge_cfg_addr);
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
		ret = tc956x_load_firmware(td);
		if (ret)
			dev_err(dev, "Firmware load failed\n");
	}

	if (td->emac0) {
		ret = readl(td->sfr_addr + NRSTCTRL0_OFFSET);

		/* Assertion of EMAC Port0 software Reset */
		ret |= NRSTCTRL0_MAC0RST;

		writel(ret, td->sfr_addr + NRSTCTRL0_OFFSET);

		dev_dbg(dev, "Enabling all eMAC clocks for Port 0 Bus number %x\n", pdev->bus->number);
		/* Enable all clocks to eMAC Port0 */
		ret = readl(td->sfr_addr + NCLKCTRL0_OFFSET);

		ret |= ((NCLKCTRL0_MAC0TXCEN | NCLKCTRL0_MAC0ALLCLKEN | NCLKCTRL0_MAC0RXCEN));
		/* Only if "current" port is SGMII 2.5G, configure below clocks. */
		if (td->port_interface == ENABLE_SGMII_INTERFACE) {
			ret &= ~NCLKCTRL0_POEPLLCEN;
			ret &= ~NCLKCTRL0_SGMPCIEN;
			ret &= ~NCLKCTRL0_REFCLKOCEN;
			ret &= ~NCLKCTRL0_MAC0125CLKEN;
			ret &= ~NCLKCTRL0_MAC0312CLKEN;
		}
		writel(ret, td->sfr_addr + NCLKCTRL0_OFFSET);

		/* Interface configuration for port0*/
		ret = readl(td->sfr_addr + NEMAC0CTL_OFFSET);
		ret &= ~(NEMACCTL_SP_SEL_MASK | NEMACCTL_PHY_INF_SEL_MASK);
		if (td->port_interface == ENABLE_SGMII_INTERFACE)
			ret |= NEMACCTL_SP_SEL_SGMII_2500M;
		else if (td->port_interface == ENABLE_XFI_INTERFACE)
			ret |= NEMACCTL_SP_SEL_USXGMII_10G_10G;

		ret &= ~(0x00000040); /* Mask Polarity */
		if (SgmSigPol == 1)
			ret |= 0x00000040; /* Set Active low */

		ret |= NEMACCTL_PHY_INF_SEL_PHY | NEMACCTL_LPIHWCLKEN;
		writel(ret, td->sfr_addr + NEMAC0CTL_OFFSET);

		/* De-assertion of EMAC Port0  software Reset*/
		ret = readl(td->sfr_addr + NRSTCTRL0_OFFSET);
		ret &= ~(NRSTCTRL0_MAC0RST);
		writel(ret, td->sfr_addr + NRSTCTRL0_OFFSET);
	}

	if (!td->emac0) {
		ret = readl(td->sfr_addr + NRSTCTRL1_OFFSET);

		/* Assertion of EMAC Port1 software Reset*/
		ret |= NRSTCTRL1_MAC1RST1;
		writel(ret, td->sfr_addr + NRSTCTRL1_OFFSET);

		dev_dbg(dev, "Enabling all eMAC clocks for Port 1 Bus number-%x\n", pdev->bus->number);
		/* Enable all clocks to eMAC Port1 */
		ret = readl(td->sfr_addr + NCLKCTRL1_OFFSET);

		ret |= ((NCLKCTRL1_MAC1TXCEN | NCLKCTRL1_MAC1RXCEN |
		NCLKCTRL1_MAC1ALLCLKEN1 | 1 << 15));
		if (td->port_interface == ENABLE_SGMII_INTERFACE) {
			ret &= ~NCLKCTRL1_MAC1125CLKEN1;
			ret &= ~NCLKCTRL1_MAC1312CLKEN1;
		}
		writel(ret, td->sfr_addr + NCLKCTRL1_OFFSET);

		/* Interface configuration for port1*/
		ret = readl(td->sfr_addr + NEMAC1CTL_OFFSET);
		ret &= ~(NEMACCTL_SP_SEL_MASK | NEMACCTL_PHY_INF_SEL_MASK);
		if (td->port_interface == ENABLE_SGMII_INTERFACE)
			ret |= NEMACCTL_SP_SEL_SGMII_2500M;
		else if (td->port_interface == ENABLE_XFI_INTERFACE)
			ret |= NEMACCTL_SP_SEL_USXGMII_10G_10G;

		ret &= ~(0x00000040); /* Mask Polarity */
		if (SgmSigPol == 1)
			ret |= 0x00000040; /* Set Active low */

		ret |= NEMACCTL_PHY_INF_SEL_PHY | NEMACCTL_LPIHWCLKEN;
		writel(ret, td->sfr_addr + NEMAC1CTL_OFFSET);

		/* De-assertion of EMAC Port1  software Reset */
		ret = readl(td->sfr_addr + NRSTCTRL1_OFFSET);
		ret &= ~NRSTCTRL1_MAC1RST1;
		writel(ret, td->sfr_addr + NRSTCTRL1_OFFSET);
	}


	res.wol_irq = pdev->irq;
	res.irq = pdev->irq;
	res.lpi_irq = pdev->irq;

	plat->bus_id = ((pdev->bus->number<<4) | (td->emac0 ? 0 : 1));

	sh_mem_offset = tc956x_get_shared_mem_offset(pdev, pci_dev_id(pdev) & TC956X_PCI_BD_MASK);
	if (sh_mem_offset < TC956X_TOT_CASCADE_DEV)
		td->pci_bd  = sh_mem_offset;
	else {
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
		ret = readl(td->sfr_addr + NRSTCTRL0_OFFSET);
		ret |= (NRSTCTRL0_MAC0PMARST | NRSTCTRL0_MAC0PONRST);
		writel(ret, td->sfr_addr + NRSTCTRL0_OFFSET);
	} else {
		/* Assertion of PMA &  XPCS reset  software Reset*/
		ret = readl(td->sfr_addr + NRSTCTRL1_OFFSET);
		ret |= (NRSTCTRL1_MAC1PMARST1 | NRSTCTRL1_MAC1PONRST1);
		writel(ret, td->sfr_addr + NRSTCTRL1_OFFSET);
	}

	ret = tc956x_pma_init(NULL, res.addr + PMA_XGMAC_OFFSET);
	if (ret < 0)
		pr_info("PMA switching to internal clock Failed\n");

	if (td->emac0) {
		/* De-assertion of PMA & XPCS reset software Reset*/
		ret = readl(td->sfr_addr + NRSTCTRL0_OFFSET);
		ret &= ~(NRSTCTRL0_MAC0PMARST | NRSTCTRL0_MAC0PONRST);
		ret &= ~(NRSTCTRL0_MAC0RST | NRSTCTRL0_MAC0RST);

		writel(ret, td->sfr_addr + NRSTCTRL0_OFFSET);
	} else {
		/* De-assertion of PMA &  XPCS reset software Reset*/
		ret = readl(td->sfr_addr + NRSTCTRL1_OFFSET);
		ret &= ~(NRSTCTRL1_MAC1PMARST1 | NRSTCTRL1_MAC1PONRST1);
		writel(ret, td->sfr_addr + NRSTCTRL1_OFFSET);
	}

	ret = readl_poll_timeout(td->sfr_addr + (td->emac0 ? NEMAC0CTL_OFFSET :
							     NEMAC1CTL_OFFSET),
				 val, val & NEMACCTL_INIT_DONE, 50, 1000000);
	if (ret < 0)
		dev_err(dev, "PMA/XPCS failed to come out of reset\n");

	ret = tc956x_xpcs_init(plat);
	if (ret < 0)
		dev_err(dev, "XPCS initialization error\n");

	ret = stmmac_dvr_probe(dev, plat, &res);
	if (ret) {
		void *nrst_reg = NULL, *nclk_reg = NULL;
		u32 nrst_val = 0, nclk_val = 0;
		if (td->emac0) {
			nrst_reg = td->sfr_addr + NRSTCTRL0_OFFSET;
			nclk_reg = td->sfr_addr + NCLKCTRL0_OFFSET;
		} else {
			nrst_reg = td->sfr_addr + NRSTCTRL1_OFFSET;
			nclk_reg = td->sfr_addr + NCLKCTRL1_OFFSET;
		}
		nrst_val = readl(nrst_reg);
		nclk_val = readl(nclk_reg);

		/* Assert reset and Disable Clock for EMAC */
		nrst_val = nrst_val | NRSTCTRL_EMAC_MASK;
		nclk_val = nclk_val & ~NCLKCTRL_EMAC_MASK;
		writel(nrst_val, nrst_reg);
		writel(nclk_val, nclk_reg);

		if (ret == -ENODEV) {
			dev_dbg(dev, "Port%d Bus%x will be registered as PCIe device only",
				 td->emac0 ? 0 : 1, pdev->bus->number);
			/* Make sure probe() succeeds by returning 0 to caller of probe() */
			ret = 0;
		} else {
			dev_err(dev, "<--%s : ret: %d\n", __func__, ret);
			goto err_dvr_probe;
		}
	}

	dev_dbg(dev, "<--%s : Adding DSP Cut Through Settings", __func__);

	/*
	 * Determine the switch configuration from the MODE2 bit in the
	 * mode status register (number of lanes per port):
	 *   0: setting A: upstream x4, downstream 1 x1, downstream 2 x1
	 *   1: setting B: upstream x2, downstream 1 x2, downstream 2 x1
	 */
	val = readl(td->sfr_addr + NMODESTS_OFFSET);
	if (val & NMODESTS_MODE2) {
		dev_dbg(dev, "%s : Setting B : Adding DSP Cut Through Settings for DSP2", __func__);
		/* downstream port is selected*/
		val = readl(td->sfr_addr + TC956X_GLUE_SW_REG_ACCESS_CTRL);
		val &= ~(SW_DSP2_ENABLE);	/* XXX Not needed */
		val |= SW_DSP2_ENABLE;
		writel(val, td->sfr_addr + TC956X_GLUE_SW_REG_ACCESS_CTRL);
		/*Set 0x0 to Rx Bit enable_cut_through_on_receive_path*/
		val = readl(td->sfr_addr + TC956X_SSREG_K_PCICONF_021_021);
		val &= ~(ENABLE_CUT_THROUGH_ON_RX_PATH_MASK);
		writel(val, td->sfr_addr + TC956X_SSREG_K_PCICONF_021_021);
		/*Set 0x0 to Tx Bit enable_cut_through_on_transmit_path*/
		val = readl(td->sfr_addr + TC956X_SSREG_K_PCICONF_022_022);
		val &= ~(ENABLE_CUT_THROUGH_ON_TX_PATH_MASK);
		writel(val, td->sfr_addr + TC956X_SSREG_K_PCICONF_022_022);
	} else {
		dev_dbg(dev, "%s : Setting A : Adding DSP Cut Through Settings for DSP1 & DSP2", __func__);
		/*DSP1 & DSP2 is selected*/
		val = readl(td->sfr_addr + TC956X_GLUE_SW_REG_ACCESS_CTRL);
		val &= ~(SW_DSP1_ENABLE | SW_DSP2_ENABLE); /* XXX Not needed */
		val |= SW_DSP1_ENABLE | SW_DSP2_ENABLE;
		writel(val, td->sfr_addr + TC956X_GLUE_SW_REG_ACCESS_CTRL);
		/*Set 0x0 to Rx Bit enable_cut_through_on_receive_path*/
		val = readl(td->sfr_addr + TC956X_SSREG_K_PCICONF_021_021);
		val &= ~(ENABLE_CUT_THROUGH_ON_RX_PATH_MASK);
		writel(val, td->sfr_addr + TC956X_SSREG_K_PCICONF_021_021);
		/*Set 0x00000000 to Tx Bit enable_cut_through_on_transmit_path*/
		val = readl(td->sfr_addr + TC956X_SSREG_K_PCICONF_022_022);
		val &= ~(ENABLE_CUT_THROUGH_ON_TX_PATH_MASK);
		writel(val, td->sfr_addr + TC956X_SSREG_K_PCICONF_022_022);
	}

	/* Increment device usage counter */
	tx956x_pci_shrd_mem[td->pci_bd].pci_dev_active_cnt++;
	mutex_unlock(&tc956x_pm_suspend_lock);

	return ret;


err_dvr_probe:
	(void) tc956x_platform_remove(td);
err_platform_probe:
err_out_msi_failed:
	pci_free_irq_vectors(pdev);
	if (td->sfr_addr)
		pci_iounmap(pdev, td->sfr_addr);
	if (td->sram_addr)
		pci_iounmap(pdev, td->sram_addr);
	if (td->bridge_cfg_addr)
		pci_iounmap(pdev, td->bridge_cfg_addr);
err_out_map_failed:
	pci_release_regions(pdev);
err_out_req_reg_failed:
	pci_disable_device(pdev);
err_out_enb_failed:
	dev_dbg(dev, "<--%s Error return: %d\n", __func__, ret);
	mutex_unlock(&tc956x_pm_suspend_lock);

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
	struct net_device *ndev = dev_get_drvdata(&pdev->dev);
	struct stmmac_priv *priv = netdev_priv(ndev);
	struct tc956x_data *td = priv->plat->bsp_priv;
	void *nrst_reg, *nclk_reg;
	u32 nrst_val, nclk_val;
	mutex_lock(&tc956x_pm_suspend_lock);

	/* phy_addr == -1 indicates that PHY was not found and
	 * device is registered as only PCIe device. So skip any
	 * ethernet device related uninitialization
	 */
	if (priv->dma_cap.sma_mdio == 1) {
		if (priv->plat->phy_addr != -1) {
			stmmac_dvr_remove(&pdev->dev);
			tc956x_platform_remove(td);
		}
	} else {
		stmmac_dvr_remove(&pdev->dev);
		tc956x_platform_remove(td);
	}

	/* Set reset value for CLK control and RESET Control registers */
	if (td->emac0) {
		nrst_reg = td->sfr_addr + NRSTCTRL0_OFFSET;
		nclk_reg = td->sfr_addr + NCLKCTRL0_OFFSET;
		nrst_val = readl(nrst_reg);
		nclk_val = readl(nclk_reg);
		nrst_val |= NRSTCTRL0_DEFAULT;
		nclk_val &= ~NCLKCTRL_PORT0_EMAC_MASK;
	} else {
		nrst_reg = td->sfr_addr + NRSTCTRL1_OFFSET;
		nclk_reg = td->sfr_addr + NCLKCTRL1_OFFSET;
		nrst_val = NRSTCTRL_EMAC_MASK;
		nclk_val = 0;
	}
	writel(nrst_val, nrst_reg);
	writel(nclk_val, nclk_reg);
	if (tx956x_pci_shrd_mem[td->pci_bd].pci_dev_active_cnt == TC956X_SINGLE_MAC_DEVICE_IN_USE) {
		/* Set reset value for Common CLK control and Common RESET Control registers */
		nrst_reg = td->sfr_addr + NRSTCTRL0_OFFSET;
		nclk_reg = td->sfr_addr + NCLKCTRL0_OFFSET;
		nrst_val = readl(nrst_reg);
		nclk_val = readl(nclk_reg);
		nrst_val |= NRSTCTRL_COMMON;
		nclk_val |= NCLKCTRL_ENABLE_COMMON_EMAC_MASK;
		nclk_val &= ~NCLKCTRL_DISABLE_COMMON_EMAC_MASK;
		writel(nrst_val, nrst_reg);
		writel(nclk_val, nclk_reg);
	}
	pr_debug("%s : Port %d %s Wr RST Reg:%x, CLK Reg:%x", __func__,
			td->emac0 ? 0 : 1, priv->dev->name,
			readl(nrst_reg), readl(nclk_reg));

	pdev->irq = 0;

	/* Free allocated interrupt vectors for device */
	pci_free_irq_vectors(pdev);

	/* Un-map previously mapped BAR0/2/4 address memory */
	if (td->sfr_addr)
		pci_iounmap(pdev, td->sfr_addr);
	if (td->sram_addr)
		pci_iounmap(pdev, td->sram_addr);
	if (td->bridge_cfg_addr)
		pci_iounmap(pdev, td->bridge_cfg_addr);
	pci_release_regions(pdev);

	pci_disable_device(pdev);

	/* Decrement device usage counter */
	tx956x_pci_shrd_mem[td->pci_bd].pci_dev_active_cnt--;
	mutex_unlock(&tc956x_pm_suspend_lock);
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
 * tc956x_pcie_pm_pci() - Disable PCIe child devices
 * @pdev:	Pointer to the PCI device whose children are affected
 * @state:	Whether we are being called during suspend
 *
 * Disable PCI devices that are children of the given PCI device.
 *
 * Return:	0 if successful, or an error code if an error occurs
 */
static int tc956x_pcie_pm_pci(struct pci_dev *pdev, enum TC956X_PORT_PM_STATE state)
{
	static struct pci_dev *tc956x_pd = NULL, *tc956x_dsp_ep = NULL, *tc956x_port_pdev[2] = {NULL};
	struct pci_bus *bus = NULL;
	int ret = 0, i = 0, p = 0;
	struct net_device *ndev = dev_get_drvdata(&pdev->dev);
	struct stmmac_priv *priv = netdev_priv(ndev);
	struct tc956x_data *td = priv->plat->bsp_priv;

	if (tx956x_pci_shrd_mem[td->pci_bd].pci_dev_active_cnt == TC956X_ALL_MAC_PORT_SUSPENDED) {
		tc956x_dsp_ep = pci_upstream_bridge(pdev);
		bus = tc956x_dsp_ep->subordinate;

		if (bus)
			list_for_each_entry(tc956x_pd, &bus->devices, bus_list)
		tc956x_port_pdev[i++] = tc956x_pd;

		for (p = 0; ((p < i) && (tc956x_port_pdev[p] != NULL)); p++) {
			/* Enter only if at least 1 Port Suspended */
			if (state == SUSPEND) {
				tc956x_pcie_pm_disable_pci(tc956x_port_pdev[p]);
			} else if (state == RESUME) {
				ret = tc956x_pcie_pm_enable_pci(tc956x_port_pdev[p]);
				if (ret < 0)
					goto err;
			}
		}
	}
err:
	return ret;
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
	struct net_device *ndev = dev_get_drvdata(&pdev->dev);
	struct stmmac_priv *priv = netdev_priv(ndev);
	struct tc956x_data *td = priv->plat->bsp_priv;
	int ret = 0;

	if (td->tc956x_port_pm_suspend == true) {
		dev_dbg(&(pdev->dev), "<--%s : Port %d interface %s already Suspended\n",
			 __func__, td->emac0 ? 0 : 1, priv->dev->name);
		return -1;
	}
	/* Set flag to avoid queuing any more work */
	td->tc956x_port_pm_suspend = true;

	mutex_lock(&tc956x_pm_suspend_lock);

	/* Decrement device usage counter */
	tx956x_pci_shrd_mem[td->pci_bd].pci_dev_active_cnt--;
	dev_dbg(&(pdev->dev), "%s : (Number of Ports Left to Suspend = [%d])\n", __func__, tx956x_pci_shrd_mem[td->pci_bd].pci_dev_active_cnt);

	/* Call stmmac_suspend() */
	stmmac_suspend(&pdev->dev);
	dev_dbg(&(pdev->dev), "%s : Port %d %s- Platform Suspend",
			__func__, td->emac0 ? 0 : 1, priv->dev->name);
	ret = tc956x_platform_suspend(priv);
	if (ret) {
		dev_err(&(pdev->dev), "%s: error in calling tc956x_platform_suspend", pci_name(pdev));
		goto err;
	}

	tc956x_pm_set_power(priv, SUSPEND);
	ret = tc956x_pcie_pm_pci(pdev, SUSPEND);
	if (ret < 0)
		goto err;
err:
	mutex_unlock(&tc956x_pm_suspend_lock);
	return ret;
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
	struct net_device *ndev = dev_get_drvdata(&pdev->dev);
	struct stmmac_priv *priv = netdev_priv(ndev);
	struct tc956x_data *td = priv->plat->bsp_priv;
	/* use signal from MSPHY */
	uint8_t SgmSigPol = 0;
	int ret = 0;
	u32 val;

	/* Skip Config when Port unavailable */
	if (priv->dma_cap.sma_mdio == 1) {
		if ((priv->plat->phy_addr == -1) || (priv->mii == NULL)) {
			dev_dbg(&(pdev->dev), "%s : Invalid PHY Address (%d)\n", __func__, priv->plat->phy_addr);
			ret = -1;
			goto err_phy_addr;
		}
	}

	if (td->emac0) {
		ret = readl(td->sfr_addr + NRSTCTRL0_OFFSET);

		/* Assertion of EMAC Port0 software Reset */
		ret |= NRSTCTRL0_MAC0RST;

		writel(ret, td->sfr_addr + NRSTCTRL0_OFFSET);

		dev_dbg(&pdev->dev, "Enabling all eMAC clocks for Port 0 %s\n", priv->dev->name);
		/* Enable all clocks to eMAC Port0 */
		ret = readl(td->sfr_addr + NCLKCTRL0_OFFSET);

		ret |= ((NCLKCTRL0_MAC0TXCEN | NCLKCTRL0_MAC0ALLCLKEN | NCLKCTRL0_MAC0RXCEN));
		if (td->port_interface == ENABLE_SGMII_INTERFACE) {
			/* Disable Clocks for 2.5Gbps SGMII */
			ret &= ~NCLKCTRL0_POEPLLCEN;
			ret &= ~NCLKCTRL0_SGMPCIEN;
			ret &= ~NCLKCTRL0_REFCLKOCEN;
			ret &= ~NCLKCTRL0_MAC0125CLKEN;
			ret &= ~NCLKCTRL0_MAC0312CLKEN;
		}
		writel(ret, td->sfr_addr + NCLKCTRL0_OFFSET);

		/* Interface configuration for port0*/
		ret = readl(td->sfr_addr + NEMAC0CTL_OFFSET);
		ret &= ~(NEMACCTL_SP_SEL_MASK | NEMACCTL_PHY_INF_SEL_MASK);
		if (td->port_interface == ENABLE_SGMII_INTERFACE)
			ret |= NEMACCTL_SP_SEL_SGMII_2500M;
		else if (td->port_interface == ENABLE_XFI_INTERFACE)
			ret |= NEMACCTL_SP_SEL_USXGMII_10G_10G;

		ret &= ~(0x00000040); /* Mask Polarity */
		if (SgmSigPol == 1)
			ret |= 0x00000040; /* Set Active low */

		ret |= NEMACCTL_PHY_INF_SEL_PHY | NEMACCTL_LPIHWCLKEN;
		writel(ret, td->sfr_addr + NEMAC0CTL_OFFSET);

		/* De-assertion of EMAC Port0  software Reset*/
		ret = readl(td->sfr_addr + NRSTCTRL0_OFFSET);
		ret &= ~(NRSTCTRL0_MAC0RST);
		writel(ret, td->sfr_addr + NRSTCTRL0_OFFSET);
	}

	if (!td->emac0) {
		ret = readl(td->sfr_addr + NRSTCTRL1_OFFSET);

		/* Assertion of EMAC Port1 software Reset*/
		ret |= NRSTCTRL1_MAC1RST1;
		writel(ret, td->sfr_addr + NRSTCTRL1_OFFSET);

		dev_dbg(&pdev->dev, "Enabling all eMAC clocks for Port 1 %s\n", priv->dev->name);
		/* Enable all clocks to eMAC Port1 */
		ret = readl(td->sfr_addr + NCLKCTRL1_OFFSET);

		ret |= ((NCLKCTRL1_MAC1TXCEN | NCLKCTRL1_MAC1RXCEN |
		NCLKCTRL1_MAC1ALLCLKEN1 | 1 << 15));
		if (td->port_interface == ENABLE_SGMII_INTERFACE) {
			ret &= ~NCLKCTRL1_MAC1125CLKEN1;
			ret &= ~NCLKCTRL1_MAC1312CLKEN1;
		}
		writel(ret, td->sfr_addr + NCLKCTRL1_OFFSET);

		/* Interface configuration for port1*/
		ret = readl(td->sfr_addr + NEMAC1CTL_OFFSET);
		ret &= ~(NEMACCTL_SP_SEL_MASK | NEMACCTL_PHY_INF_SEL_MASK);
		if (td->port_interface == ENABLE_SGMII_INTERFACE)
			ret |= NEMACCTL_SP_SEL_SGMII_2500M;
		else if (td->port_interface == ENABLE_XFI_INTERFACE)
			ret |= NEMACCTL_SP_SEL_USXGMII_10G_10G;

		ret &= ~(0x00000040); /* Mask Polarity */
		if (SgmSigPol == 1)
			ret |= 0x00000040; /* Set Active low */

		ret |= NEMACCTL_PHY_INF_SEL_PHY | NEMACCTL_LPIHWCLKEN;
		writel(ret, td->sfr_addr + NEMAC1CTL_OFFSET);

		/* De-assertion of EMAC Port1  software Reset */
		ret = readl(td->sfr_addr + NRSTCTRL1_OFFSET);
		ret &= ~NRSTCTRL1_MAC1RST1;
		writel(ret, td->sfr_addr + NRSTCTRL1_OFFSET);
	}

	if (priv->hw->xpcs) {
		if (td->emac0) {
			/* Assertion of PMA &  XPCS reset  software Reset*/
			ret = readl(td->sfr_addr + NRSTCTRL0_OFFSET);
			ret |= (NRSTCTRL0_MAC0PMARST | NRSTCTRL0_MAC0PONRST);
			writel(ret, td->sfr_addr + NRSTCTRL0_OFFSET);
		} else {
			/* Assertion of PMA &  XPCS reset  software Reset*/
			ret = readl(td->sfr_addr + NRSTCTRL1_OFFSET);
			ret |= (NRSTCTRL1_MAC1PMARST1 | NRSTCTRL1_MAC1PONRST1);
			writel(ret, td->sfr_addr + NRSTCTRL1_OFFSET);
		}

		ret = tc956x_pma_init(priv, priv->ioaddr + PMA_XGMAC_OFFSET);
		if (ret < 0)
			pr_info("PMA switching to internal clock Failed\n");

		if (td->emac0) {
			/* De-assertion of PMA &  XPCS reset  software Reset*/
			ret = readl(td->sfr_addr + NRSTCTRL0_OFFSET);
			ret &= ~(NRSTCTRL0_MAC0PMARST | NRSTCTRL0_MAC0PONRST);
			ret &= ~(NRSTCTRL0_MAC0RST | NRSTCTRL0_MAC0RST);
			writel(ret, td->sfr_addr + NRSTCTRL0_OFFSET);
		} else {
			/* De-assertion of PMA &  XPCS reset  software Reset*/
			ret = readl(td->sfr_addr + NRSTCTRL1_OFFSET);
			ret &= ~(NRSTCTRL1_MAC1PMARST1 | NRSTCTRL1_MAC1PONRST1);
			writel(ret, td->sfr_addr + NRSTCTRL1_OFFSET);
		}

		ret = readl_poll_timeout(
			td->sfr_addr + (td->emac0 ? NEMAC0CTL_OFFSET : NEMAC1CTL_OFFSET),
			val, val & NEMACCTL_INIT_DONE, 50, 1000000);
		if (ret < 0)
			dev_err(&pdev->dev, "PMA/XPCS failed to come out of reset\n");

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
	struct net_device *ndev = dev_get_drvdata(&pdev->dev);
	struct stmmac_priv *priv = netdev_priv(ndev);
	struct tc956x_data *td = priv->plat->bsp_priv;
	int ret = 0;

	if (td->tc956x_port_pm_suspend == false) {
		dev_dbg(&(pdev->dev), "%s : Port %d %s already Resumed\n",
				__func__, td->emac0 ? 0 : 1, priv->dev->name);
		return -1;
	}
	mutex_lock(&tc956x_pm_suspend_lock);

	ret = tc956x_pcie_pm_enable_pci(pdev);
	if (ret < 0)
		goto err;

	tc956x_pm_set_power(priv, RESUME);

	tc956x_restore_phy_reset(priv);

	dev_dbg(&(pdev->dev), "%s : Port %d %s - Platform Resume",
			__func__, td->emac0 ? 0 : 1, priv->dev->name);
	ret = tc956x_platform_resume(priv);
	if (ret) {
		dev_err(&(pdev->dev), "%s: error in calling tc956x_platform_resume", pci_name(pdev));
		pci_disable_device(pdev);
		goto err;
	}

	/* Configure TA map registers */

	if (tx956x_pci_shrd_mem[td->pci_bd].pci_dev_active_cnt == TC956X_ALL_MAC_PORT_SUSPENDED) {
		dev_dbg(&(pdev->dev), "%s : Tamap Re-configuration", __func__);
		tc956x_config_tamap(&pdev->dev, td->bridge_cfg_addr);
	}

	/* Configure EMAC Port */
	tc956x_pcie_resume_config(pdev);

	/* Call stmmac_resume() */
	stmmac_resume(&pdev->dev);

	/* Increment device usage counter */
	tx956x_pci_shrd_mem[td->pci_bd].pci_dev_active_cnt++;
	dev_dbg(&(pdev->dev), "%s : (Number of Ports Resumed = [%d])\n", __func__, tx956x_pci_shrd_mem[td->pci_bd].pci_dev_active_cnt);

	td->tc956x_port_pm_suspend = false;

err:
	mutex_unlock(&tc956x_pm_suspend_lock);

	return ret;
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

static SIMPLE_DEV_PM_OPS(tc956x_pm_ops, tc956x_pcie_suspend, tc956x_pcie_resume);

static struct pci_driver tc956x_pci_driver = {
	.name = DRIVER_NAME,
	.id_table = tc956x_id_table,
	.probe = tc956x_pci_probe,
	.remove = tc956x_pci_remove,
	.driver		= {
		.name		= DRIVER_NAME,
		.owner		= THIS_MODULE,
		.pm		= &tc956x_pm_ops,
	},
};

module_pci_driver(tc956x_pci_driver);

MODULE_DESCRIPTION("Toshiba TC956x PCIe Ethernet Network Driver");
MODULE_LICENSE("GPL");
