/*
 * TC956X ethernet driver.
 *
 * tc956x_pci.c
 *
 * Copyright (C) 2011-2012  Vayavya Labs Pvt Ltd
 * Copyright (C) 2025 Toshiba Electronic Devices & Storage Corporation
 *
 * This file has been derived from the STMicro and Synopsys Linux driver,
 * and developed or modified for TC956X.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#define pr_fmt(fmt) "dwmac-toshiba: " fmt
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
#include <linux/regulator/consumer.h>
#include <linux/of_irq.h>
#include <linux/delay.h>
#include "stmmac.h"
#include "dwxgmac2.h"
//#include "tc956xmac.h"
//#include "tc956xmac_config.h"
//#include "tc956xmac_inc.h"
#include "common.h"
//#include "tc956x_pcie_logstat.h"

#define DRIVER_NAME "dwmac-toshiba-pci"

//
// Definitions taken from tc956xmac.h in vendor driver
//

#define DEVICE_ID 0x0220	/* PF - 0x0220, VF - 0x0221 */

#define MOD_PARAM_ACCESS 0444

#define FIRMWARE_NAME "TC956X_Firmware_PCIeBridge.bin"

#define TC956X_TOTAL_VFS 3

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
#define TC956X_AXI4_SLV_SRC_ADDR_LO(ch, tid)	(TC956X_AXI4_SLV(ch, tid) +\
							SRC_ADDR_LO_OFFSET)
#define TC956X_AXI4_SLV_SRC_ADDR_HI(ch, tid)	(TC956X_AXI4_SLV(ch, tid) +\
							SRC_ADDR_HI_OFFSET)
#define TC956X_AXI4_SLV_TRSL_ADDR_LO(ch, tid)	(TC956X_AXI4_SLV(ch, tid) +\
							TRSL_ADDR_LO_OFFSET)
#define TC956X_AXI4_SLV_TRSL_ADDR_HI(ch, tid)	(TC956X_AXI4_SLV(ch, tid) +\
							TRSL_ADDR_HI_OFFSET)
#define TC956X_AXI4_SLV_TRSL_PARAM(ch, tid)	(TC956X_AXI4_SLV(ch, tid) +\
							TRSL_PARAM_OFFSET)
#define TC956X_AXI4_SLV_TRSL_MASK1(ch, tid)	(TC956X_AXI4_SLV(ch, tid) +\
							TRSL_MASK_OFFSET1)
#define TC956X_AXI4_SLV_TRSL_MASK2(ch, tid)	(TC956X_AXI4_SLV(ch, tid) +\
							TRSL_MASK_OFFSET2)

#define TC956X_ATR_IMPL 1U
#define TC956X_ATR_SIZE(size) ((size - 1U) << 1U)
#define TC956X_ATR_SIZE_MASK		GENMASK(6, 1)
#define TC956x_ATR_SIZE_SHIFT		1
#define TC956X_SRC_LO_MASK		GENMASK(31, 12)
#define TC956X_SRC_LO_SHIFT		12

#define TC956X_AXI4_SLV00_ATR_SIZE 36U
#define TC956X_AXI4_SLV00_SRC_ADDR_LO_VAL  (0x00000000U)
#define TC956X_AXI4_SLV00_SRC_ADDR_HI_VAL  (0x00000010U)
#define TC956X_AXI4_SLV00_TRSL_ADDR_LO_VAL (0x00000000U)
#define TC956X_AXI4_SLV00_TRSL_ADDR_HI_VAL (0x00000000U)
#define TC956X_AXI4_SLV00_TRSL_PARAM_VAL   (0x00000000U)
#define TC956X_AXI4_SLV00_SRC_ADDR_LO_VAL_DEFAULT  (0x0000007FU)

#define NRSTCTRL0_RST_ASRT 0x1
#define NRSTCTRL0_RST_DE_ASRT 0x3

#define TC956X_OFFSET_TAMAP 0x00000010
#define TC956X_MASK_TAMAP 0xFFFFF000
#define TC956X_SHIFT_TAMAP 32
#define TC956X_OFFSET_OW 28
#define TC956X_OFFSET_OW_MAX 53
#define TC956X_HEX_ZERO 0x00000000

#define TC956X_BAR0 0
#define TC956X_BAR2 2
#define TC956X_BAR4 4

#define NMODESTS 0x0004
#define NMODESTS_MODE 0x200
#define NMODESTS_MODE2		0x400
#define NMODESTS_MODE2_SHIFT	10
#define TC956X_PCIE_SETTING_A	0 /* x4x1x1 mode */
#define TC956X_PCIE_SETTING_B	1 /* x2x2x1 mode */

#define TC9563_CFG_NEMACTXCDLY		0x1050U
#define TC9563_CFG_NEMACIOCTL		0x107CU

#define NEMACTXCDLY_DEFAULT		0x00000000U
#define NEMACIOCTL_DEFAULT		0xF300F300
/* Systick count SRAM  address  DMEM addrs 0x2000F83C, Check this value for any change */
#define SYSTCIK_SRAM_OFFSET		0x4F83C

/* Tx Timer count SRAM  address  DMEM addrs 0x2000F844, Check this value for any change */
#define TX_TIMER_SRAM_OFFSET_0		0x4F844

/* Tx Timer count SRAM  address  DMEM addrs 0x2000F848, Check this value for any change */
#define TX_TIMER_SRAM_OFFSET_1		0x4F848

#define TX_TIMER_SRAM_OFFSET(t) (((t) == RM_PF0_ID) ? (TX_TIMER_SRAM_OFFSET_0) : (TX_TIMER_SRAM_OFFSET_1))

#define TC956X_M3_SRAM_EEPROM_MAC_ADDR		0x47000		/* DMEM addrs 0x20007000U */
#define TC956X_M3_SRAM_EEPROM_OFFSET_ADDR	0x47050		/* DMEM addrs 0x20007050U */
#define TC956X_M3_SRAM_EEPROM_MAC_COUNT		0x47051		/* DMEM addrs 0x20007051U */
#define TC956X_M3_INIT_DONE					0x47054		/* DMEM addrs 0x20007054U */
#define TC956X_M3_FW_EXIT					0x47058		/* DMEM addrs 0x20007058U */

#define TC956X_M3_DBG_VER_START			0x4F900

#define ENABLE_USXGMII_INTERFACE		0 /* This value is passed to TSB AQR Sample driver as dev_flags, when this changed, AQR sample driver needs change */
#define ENABLE_XFI_INTERFACE			1 /* XFI/SFI, this is same as USXGMII, except XPCS autoneg disabled */
#define ENABLE_RGMII_INTERFACE			2
#define ENABLE_RGMII_ID_INTERFACE		3
#define ENABLE_SGMII_INTERFACE			4
#define ENABLE_2500BASE_X_INTERFACE		5
#define ENABLE_USXGMII_10G_INTERFACE	6
#define ENABLE_USXGMII_5G_INTERFACE		7 /* This value is passed to TSB AQR Sample driver as dev_flags, when this changed, AQR sample driver needs change */
#define ENABLE_USXGMII_2_5G_INTERFACE	8

#define MAX_CM3_TAMAP_ENTRIES		3
#define CM3_TAMAP_ATR_SIZE		28 /* ATR Size = 2 ^ (28 + 1) = 512MB */
#define CM3_TAMAP_SIZE			(1 << (CM3_TAMAP_ATR_SIZE + 1))
#define CM3_TAMAP_MASK			(CM3_TAMAP_SIZE - 1)
#define CM3_TAMAP_SRC_ADDR_START	0x60000000

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

/* For TC956X, clk_scr_i = 125MHz */
#define TC956XMAC_XGMAC_MDC_CSR_4		0x0 /*clk_csr_i/4 */
#define TC956XMAC_XGMAC_MDC_CSR_6		0x1 /* clk_csr_i/6 */
#define TC956XMAC_XGMAC_MDC_CSR_8		0x2 /* clk_csr_i/8 */
#define TC956XMAC_XGMAC_MDC_CSR_10		0x3 /* clk_csr_i/10 */
#define TC956XMAC_XGMAC_MDC_CSR_12		0x4 /* clk_csr_i/12 */
#define TC956XMAC_XGMAC_MDC_CSR_14		0x5 /* clk_csr_i/14 */
#define TC956XMAC_XGMAC_MDC_CSR_16		0x6 /* clk_csr_i/16 */
#define TC956XMAC_XGMAC_MDC_CSR_18		0x7 /* clk_csr_i/18 */
#define TC956XMAC_XGMAC_MDC_CSR_62		0x8 /* clk_csr_i/62 */
#define TC956XMAC_XGMAC_MDC_CSR_102		0x9 /* clk_csr_i/102 */
#define TC956XMAC_XGMAC_MDC_CSR_122		0xA /* clk_csr_i/122 */
#define TC956XMAC_XGMAC_MDC_CSR_142		0xB /* clk_csr_i/142 */
#define TC956XMAC_XGMAC_MDC_CSR_162		0xC /* clk_csr_i/162 */
#define TC956XMAC_XGMAC_MDC_CSR_202		0xD /* clk_csr_i/202 */

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
#define TC956X_MAX_PORT			2
#define TC956X_ALL_MAC_PORT_SUSPENDED	0 /* All EMAC Port Suspended. To be used just after suspend and before resume. */
#define TC956X_NO_MAC_DEVICE_IN_USE	0 /* No EMAC Port in use. To be used at probe and remove. */
#define TC956X_SINGLE_MAC_DEVICE_IN_USE	1 /* One of the EMAC Port in use. To be used at remove. */
#define TC956X_ALL_MAC_PORT_LINK_DOWN	2 /* All ports are Link Down */
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

#define DISABLE		0x0U
#define ENABLE		0x1U
#define SIZE_512B	0x200U
#define SIZE_1KB	0x400U

// TODO: this was unifdef'ed (some build options result in the value being two
#define TC956X_TOT_MSI_VEC	1

/*	Dual Port related Macros	*/
#define RM_PF0_ID		(0)
#define RM_PF1_ID		(1)
#define RM_IS_PF		(0)
#define RM_IS_VF		(1)
#define MAC_PORT_NUM_CHECK	(priv->port_num == RM_PF0_ID)

#define TC956X_AVB_PRIORITY_CLASS_A	(3)
#define TC956X_AVB_PRIORITY_CLASS_B	(2)
#define TC956X_PRIORITY_CLASS_CDT	(7)

#define MAX_RX_QUEUE_SIZE	47104 /* 46KB Maximun RX Queue size */
#define MAX_TX_QUEUE_SIZE	47104 /* 46KB Maximun TX Queue size */

#define TC956X_DA_MAP		0xF

/************************ TC956X_SRIOV_PF Starts ************************/

#define TC956X_DMA_RX_AIS  BIT(14)
#define TC956X_DMA_RX_FBE  BIT(12)
#define TC956X_DMA_RX_RPS  BIT(8)
#define MAX_TX_QUEUES_TO_USE	8
#define MAX_RX_QUEUES_TO_USE	8

/* Tx Queue Size*/
#define TX_QUEUE0_SIZE	4096
#define TX_QUEUE1_SIZE	4096
#define TX_QUEUE2_SIZE	18432
#define TX_QUEUE3_SIZE	4096
#define TX_QUEUE4_SIZE	1024
#define TX_QUEUE5_SIZE	4096
#define TX_QUEUE6_SIZE	4096
#define TX_QUEUE7_SIZE	409NEMAC0CTL_OFFSET6

/* TX Queue 0: Legacy and Jumbo packets */
#define TX_QUEUE0_MODE		MTL_QUEUE_DCB
/* TX Queue 1:Legacy*/
#define TX_QUEUE1_MODE		MTL_QUEUE_DCB
/* TX Queue 2: Legacy */
#define TX_QUEUE2_MODE		MTL_QUEUE_DCB
/* TX Queue 3: Legacy */
#define TX_QUEUE3_MODE		MTL_QUEUE_DCB
/* TX Queue 4: Untagged PTP */
#define TX_QUEUE4_MODE		MTL_QUEUE_DCB
/* TX Queue 5: AVB Class B AVTP packet */
#define TX_QUEUE5_MODE		MTL_QUEUE_AVB
/* TX Queue 6: AVB Class A AVTP packet */
#define TX_QUEUE6_MODE		MTL_QUEUE_AVB
/* TX Queue 7: TSN Class CDT packet */
#if defined(TSN_DEMO_AUTOMOTIVE)
#define TX_QUEUE7_MODE		MTL_QUEUE_DCB
#else
#define TX_QUEUE7_MODE		MTL_QUEUE_AVB
#endif

/* Tx Queue TBS Enable/Disable */
#define TX_QUEUE0_TBS		0
#define TX_QUEUE1_TBS		0
#define TX_QUEUE2_TBS		0
#define TX_QUEUE3_TBS		0
#define TX_QUEUE4_TBS		0
#define TX_QUEUE5_TBS		1
#define TX_QUEUE6_TBS		1
#define TX_QUEUE7_TBS		1

/* Tx Queue TSO Enable/Disable */
#define TX_QUEUE0_TSO		1
#define TX_QUEUE1_TSO		1
#define TX_QUEUE2_TSO		1
#define TX_QUEUE3_TSO		1
#define TX_QUEUE4_TSO		0
#define TX_QUEUE5_TSO		0
#define TX_QUEUE6_TSO		0
#define TX_QUEUE7_TSO		0

/* Configure TxQueue - Traffic Class mapping */
#define TX_QUEUE0_TC	0x0
#define TX_QUEUE1_TC	0x0
#define TX_QUEUE2_TC	0x0
#define TX_QUEUE3_TC	0x0
#define TX_QUEUE4_TC	0x1
#define TX_QUEUE5_TC	0x2
#define TX_QUEUE6_TC	0x3
#define TX_QUEUE7_TC	0x4


/*
 * RX Queue 0: Unicast/Untagged Packets - Packets with
 * unique MAC Address of Host/Guest OS DMA channel selection will be based on
 * MAC_Address(#i)_High.DCS
 */
#define RX_QUEUE0_MODE		MTL_QUEUE_DCB
/* RX Queue 1: VLAN Tagged Legacy packets- Pkt routing will be based on VLAN */
#define RX_QUEUE1_MODE		MTL_QUEUE_DCB
/* RX Queue 2: Untagged gPTP packets */
#define RX_QUEUE2_MODE		MTL_QUEUE_DCB
/* RX Queue 3: Filter Fail packet queue */
#define RX_QUEUE3_MODE		MTL_QUEUE_DCB
/* RX Queue 4: AVB Class B AVTP packets */
#define RX_QUEUE4_MODE		MTL_QUEUE_AVB
/* RX Queue 5: AVB Class A AVTP packets */
#define RX_QUEUE5_MODE		MTL_QUEUE_AVB
/* RX Queue 6:TSN  Class CDT packets */
#define RX_QUEUE6_MODE		MTL_QUEUE_AVB
/* RX Queue 7: Broadcast/Multicast packets */
#define RX_QUEUE7_MODE		MTL_QUEUE_DCB

/* Rx Queue Size */
#define RX_QUEUE0_SIZE	18432
#define RX_QUEUE1_SIZE	4096
#define RX_QUEUE2_SIZE	1024
#define RX_QUEUE3_SIZE	4096
#define RX_QUEUE4_SIZE	4096
#define RX_QUEUE5_SIZE	4096
#define RX_QUEUE6_SIZE	4096
#define RX_QUEUE7_SIZE	4096

/* Rx Queue Packet Routing */
#define RX_QUEUE0_PKT_ROUTE	PACKET_UPQ
#define RX_QUEUE1_PKT_ROUTE	0
#define RX_QUEUE2_PKT_ROUTE	PACKET_PTPQ
#define RX_QUEUE3_PKT_ROUTE	PACKET_FILTER_FAIL
/* Queue 4,5,6 Routed based on Packet Priority Configured in
 * MAC_RxQ_Ctrl2/MAC_RxQ_Ctrl3
 */
#define RX_QUEUE4_PKT_ROUTE	0
#define RX_QUEUE5_PKT_ROUTE	0
#define RX_QUEUE6_PKT_ROUTE	0
#define RX_QUEUE7_PKT_ROUTE	PACKET_MCBCQ

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

#define TX_QUEUE0_USE_PRIO	true
#define TX_QUEUE1_USE_PRIO	true
#define TX_QUEUE2_USE_PRIO	true
#define TX_QUEUE3_USE_PRIO	true
#define TX_QUEUE4_USE_PRIO	false
#define TX_QUEUE5_USE_PRIO	false
#define TX_QUEUE6_USE_PRIO	false
#define TX_QUEUE7_USE_PRIO	false

#define TX_QUEUE0_PRIO		0xFF	/* TC0 Priority */
#define TX_QUEUE1_PRIO		0xFF
#define TX_QUEUE2_PRIO		0xFF
#define TX_QUEUE3_PRIO		0xFF
#define TX_QUEUE4_PRIO		0x00
#define TX_QUEUE5_PRIO		0x00
#define TX_QUEUE6_PRIO		0x00
#define TX_QUEUE7_PRIO		0x00

/* Rx Queue Use Priority */
#define RX_QUEUE0_USE_PRIO		false
#define RX_QUEUE1_USE_PRIO		true
#define RX_QUEUE2_USE_PRIO		false
#define RX_QUEUE3_USE_PRIO		false
#define RX_QUEUE4_USE_PRIO		true
#define RX_QUEUE5_USE_PRIO		true
#define RX_QUEUE6_USE_PRIO		true
#define RX_QUEUE7_USE_PRIO		false

/* Rx Queue VLAN tagged Priority mapping */
#define RX_QUEUE0_PRIO		0
#define RX_QUEUE1_PRIO		0xFF
#define RX_QUEUE2_PRIO		0
#define RX_QUEUE3_PRIO		0
#define RX_QUEUE4_PRIO		(1 << TC956X_AVB_PRIORITY_CLASS_B)
#define RX_QUEUE5_PRIO		(1 << TC956X_AVB_PRIORITY_CLASS_A)
#define RX_QUEUE6_PRIO		(1 << TC956X_PRIORITY_CLASS_CDT)
#define RX_QUEUE7_PRIO		0

#define EEPROM_OFFSET		0
#define EEPROM_MAC_COUNT	14

/* DMA Ch allocation in tx-rx pairs for PF & VF */
#define TC956X_PF_CH_ALLOC	0x18	/* PF - Ch 3, 4 */
#define TC956X_VF0_CH_ALLOC	0x21	/* VF0 - Ch 0, 5 */
#define TC956X_VF1_CH_ALLOC	0x02	/* VF1 - Ch 1 */
#define TC956X_VF2_CH_ALLOC	0xC4	/* VF2 - Ch 2, 6, 7 */

#define TC956X_DMA0_VF_MAP  0U
#define TC956X_DMA1_VF_MAP  1U
#define TC956X_DMA2_VF_MAP  2U
#define TC956X_DMA3_VF_MAP  3U
#define TC956X_DMA4_VF_MAP  3U
#define TC956X_DMA5_VF_MAP  0U
#define TC956X_DMA6_VF_MAP  2U
#define TC956X_DMA7_VF_MAP  2U

#define TC956X_RXQ0_CH_MAP  3U
#define TC956X_RXQ1_CH_MAP  3U
#define TC956X_RXQ2_CH_MAP  4U
#define TC956X_RXQ3_CH_MAP  3U
#define TC956X_RXQ4_CH_MAP  8U
#define TC956X_RXQ5_CH_MAP  8U
#define TC956X_RXQ6_CH_MAP  8U
#define TC956X_RXQ7_CH_MAP  3U


/************************* TC956X_SRIOV_PF Ends *************************/

/* Default LPI timers */
#define TC956XMAC_DEFAULT_LIT_LS	0x3E8
#define TC956XMAC_DEFAULT_TWT_LS	0x1E
#define TC956XMAC_LIT_LS		0x0011
#define TC956XMAC_TWT_LS		0x0028
#define TC956XMAC_TIC_1US_CNTR		0x7c
#define TC956XMAC_LPIET_600US		0x258
#define TC956X_PHY_SPEED_5G		5000
#define TC956X_PHY_SPEED_2_5G		2500

#define TC956XMAC_CHAIN_MODE	0x1
#define TC956XMAC_RING_MODE	0x2

#define TC956X_TARGET_PTP_CLK	50000000

#define RSC_MNG_OFFSET		0x2000
#define RSCMNG_ID_REG		((RSC_MNG_OFFSET) + 0x00000000)
#define RSCMNG_PFN		GENMASK(3, 0)
#define RSCMNG_PFN_SHIFT	0

/*	Configuration Register Address	*/
#define NCID_OFFSET			(0x0000) /* TC956X Chip and revision ID */
#define NMODESTS_OFFSET		(0x0004) /* TC956X current operation mode */
#define NFUNCEN0_OFFSET		(0x0008) /* TC956X pin mux control */
#define NPCIEBOOT_OFFSET	(0x0018) /* TC956X PCIE Boot HW Sequence Status and Control */

#define NCTLSTS_OFFSET		(0x1000)  /* TC956X control and status */
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
#define NRSTCTRL0_PCIERST	BIT(9)
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
#define NCLKCTRL0_DEFAULT	(NCLKCTRL0_SRMCEM | NCLKCTRL0_I2SSPIEN | \
					NCLKCTRL0_PCIECEN | NCLKCTRL0_MCUCEN)
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

#define NEMAC0CTL_OFFSET	(0x1070) /* eMAC Port-0 Control */
#define NMISCCTL_OFFSET		(0x1800)

#define NEMAC1CTL_OFFSET	(0x1074) /* eMAC Port-1 Control */
#define NEMACSTS_OFFSET		(0x1078) /* eMAC status */
#define NEMACIOCTL_OFFSET	(0x107C) /* eMAC IO Control */

#define NEMACCTL_SP_SEL_MASK			GENMASK(3, 0)
#define NEMACCTL_INIT_DONE			0x200000
#define NEMACCTL_LPIHWCLKEN			(0x100)
#define NEMACCTL_PHY_INF_SEL_MASK		GENMASK(5, 4)
#define NEMACCTL_PHY_INF_SEL			(0x10)/* Phy_intf_sel : clock from PHY */
#define NEMACCTL_SP_SEL_SGMII_10M		(0x7) /* SGMII 10M */
#define NEMACCTL_SP_SEL_SGMII_100M		(0x6) /* SGMII 100M */
#define NEMACCTL_SP_SEL_SGMII_1000M		(0x5) /* SGMII 1000M */
#define NEMACCTL_SP_SEL_SGMII_2500M		(0x4) /* SGMII 2500M */
#define NEMACCTL_SP_SEL_USXGMII_2_5G_2_5G	(0xD) /* USXGMII 2.5G/2.5G */
#define NEMACCTL_SP_SEL_USXGMII_2_5G_5G		(0xC) /* USXGMII 2.5G/5G */
#define NEMACCTL_SP_SEL_USXGMII_2_5G_10G	(0xB) /* USXGMII 2.5G/10G */
#define NEMACCTL_SP_SEL_USXGMII_5G_5G		(0xA) /* USXGMII 5G/5G */
#define NEMACCTL_SP_SEL_USXGMII_5G_10G		(0x9) /* USXGMII 5G/10G */
#define NEMACCTL_SP_SEL_USXGMII_10G_10G		(0x8) /* USXGMII 10G/10G */
#define NEMACCTL_SP_SEL_USXGMII_1G_10G		(0x8) /* USXGMII 1G/10G */
#define NEMACCTL_SP_SEL_USXGMII_100M_10G	(0x9) /* USXGMII 100M/10G */
#define NEMACCTL_SP_SEL_USXGMII_10M_10G		(0x9) /* USXGMII 10M/10G */
#define NEMACCTL_SP_SEL_USXGMII_1G_5G		(0xC) /* USXGMII 1G/5G */
#define NEMACCTL_SP_SEL_USXGMII_100M_5G		(0xC) /* USXGMII 100M/5G */
#define NEMACCTL_SP_SEL_USXGMII_10M_5G		(0xA) /* USXGMII 10M/5G */
#define NEMACCTL_SP_SEL_USXGMII_1G_2_5G		(0xD) /* USXGMII 1G/2.5G */
#define NEMACCTL_SP_SEL_USXGMII_100M_2_5G	(0xD) /* USXGMII 100M/2.5G */
#define NEMACCTL_SP_SEL_USXGMII_10M_2_5G	(0xD) /* USXGMII 10M/2.5G */

#define SP_ETH0_SHIFT			16
#define SP_ETH1_SHIFT			24
#define SP_ETH_1G				1
#define SP_ETH_100M				3
#define SP_ETH_10M				7

#define REV_ID_MASK				0xF
#define REV_ID1					0x1
#define REV_ID2					0x2

#define NEMACCTL_SP_SEL_RGMII_10M		(0x2) /* RGMII 10M */
#define NEMACCTL_SP_SEL_RGMII_100M		(0x1) /* RGMII 100M */
#define NEMACCTL_SP_SEL_RGMII_1000M		(0x0) /* RGMII 1000M */

#define GPIOI0_OFFSET	(0x1200) /* GPIO Input-0 register */
#define GPIOI1_OFFSET	(0x1204) /* GPIO Input-1 register */
#define GPIOE0_OFFSET	(0x1208) /* GPIO Enable-0 register */
#define GPIOE1_OFFSET	(0x120C) /* GPIO Enable-1 register */
#define GPIOO0_OFFSET	(0x1210) /* GPIO Output-0 register */
#define GPIOO1_OFFSET	(0x1214) /* GPIO Output-1 register */

#define NPCIEPWR_OFFSET	(0x1300) /* PCIe power gating control */

#define I2CERRADD_OFFSET	(0x1400)
#define SPIERRADD_OFFSET	(0x1404)
#define NFUNCEN1_OFFSET		(0x1514)
#define NFUNCEN2_OFFSET		(0x151C)
#define NFUNCEN3_OFFSET		(0x1524)
#define NFUNCEN4_OFFSET		(0x1528)
#define NFUNCEN5_OFFSET		(0x152C)
#define NFUNCEN6_OFFSET		(0x1530)
#define NFUNCEN7_OFFSET		(0x153C)

#define NFUNCEN_FUNC0		(0)
#define NFUNCEN_FUNC1		(1)
#define NFUNCEN_FUNC2		(2)
#define NFUNCEN4_GPIO_00	GENMASK(3, 0)
#define NFUNCEN4_GPIO_00_SHIFT	(0)
#define NFUNCEN4_GPIO_01	GENMASK(7, 4)
#define NFUNCEN4_GPIO_01_SHIFT	(4)
#define NFUNCEN4_GPIO_02	GENMASK(11, 8)
#define NFUNCEN4_GPIO_02_SHIFT	(8)
#define NFUNCEN4_GPIO_03	GENMASK(15, 12)
#define NFUNCEN4_GPIO_03_SHIFT	(12)
#define NFUNCEN4_GPIO_04	GENMASK(19, 16)
#define NFUNCEN4_GPIO_04_SHIFT	(16)
#define NFUNCEN4_GPIO_05	GENMASK(23, 20)
#define NFUNCEN4_GPIO_05_SHIFT	(20)
#define NFUNCEN4_GPIO_06	GENMASK(27, 24)
#define NFUNCEN4_GPIO_06_SHIFT	(24)
#define NFUNCEN5_GPIO_10	GENMASK(3, 0)
#define NFUNCEN5_GPIO_10_SHIFT	(0)
#define NFUNCEN5_GPIO_11	GENMASK(7, 4)
#define NFUNCEN5_GPIO_11_SHIFT	(4)
#define NFUNCEN6_GPIO_12	GENMASK(19, 16)
#define NFUNCEN6_GPIO_12_SHIFT	(16)
#define NFUNCEN7_GPIO_13	GENMASK(3, 0)
#define NFUNCEN7_GPIO_13_SHIFT	(0)

#define NIOCFG1_OFFSET		(0x1614)
#define NIOCFG7_OFFSET		(0x163C)
#define NIOEN7_OFFSET		(0x173C)

#define GPIO_00			(0)
#define GPIO_01			(1)
#define GPIO_02			(2)
#define GPIO_03			(3)
#define GPIO_04			(4)
#define GPIO_05			(5)
#define GPIO_06			(6)
#define GPIO_10			(10)
#define GPIO_11			(11)
#define GPIO_12			(12)
#define GPIO_13			(13)
#define GPIO_32			(32)

#define TRIG00_SHIFT		4
#define TRIG10_SHIFT		12
#define TRIG00_MASK			0xF0
#define TRIG10_MASK			0xF000
#define TSIE_SHIFT			12

/* REV_ID1 does not support USXGMII_5G, USXGMII_2_5G interface type */
#define MAX_INTERFACE \
	((plat->RevID == REV_ID1) ? (ENABLE_USXGMII_10G_INTERFACE) : (ENABLE_USXGMII_2_5G_INTERFACE))

#define TC956X_SSREG_BRREG_REG_BASE		(0x00024000U)

#define TC956X_GLUE_LOGIC_BASE_OFST		(0x0002C000U)

/*All phy core use the same base address, glue register we need to select correct phy core*/
#define TC956X_PHY_CORE0_REG_BASE		(0x00028000U)
#define TC956X_PHY_CORE1_REG_BASE		(0x00028000U)
#define TC956X_PHY_CORE2_REG_BASE		(0x00028000U)
#define TC956X_PHY_CORE3_REG_BASE		(0x00028000U)


#define TC956X_SSREG_K_PCICONF_015_000		(TC956X_SSREG_BRREG_REG_BASE \
						+ 0x00000850U)
#define TC956X_SSREG_K_PCICONF_031_016		(TC956X_SSREG_BRREG_REG_BASE \
						+ 0x00000854U)
#define TC956X_SSREG_K_PCICONF_021_021		(TC956X_SSREG_BRREG_REG_BASE \
						+ 0x000009E4U)
#define TC956X_SSREG_K_PCICONF_022_022		(TC956X_SSREG_BRREG_REG_BASE \
						+ 0x000009E8U)

#define TC956X_GLUE_EFUSE_CTRL			(TC956X_GLUE_LOGIC_BASE_OFST \
						+ 0x0000001CU)
#define TC956X_GLUE_SW_REG_ACCESS_CTRL		(TC956X_GLUE_LOGIC_BASE_OFST \
						+ 0x0000002CU)
#define TC956X_GLUE_PHY_REG_ACCESS_CTRL		(TC956X_GLUE_LOGIC_BASE_OFST \
						+ 0x00000030U)
#define TC956X_GLUE_SW_RESET_CTRL		(TC956X_GLUE_LOGIC_BASE_OFST \
						+ 0x00000044U)
#define TC956X_GLUE_SW_DSP1_TEST_IN_31_00	(TC956X_GLUE_LOGIC_BASE_OFST \
						+ 0x0000006CU)
#define TC956X_GLUE_SW_DSP2_TEST_IN_31_00	(TC956X_GLUE_LOGIC_BASE_OFST \
						+ 0x00000074U)
#define TC956X_GLUE_SW_USP_TEST_OUT_127_096	(TC956X_GLUE_LOGIC_BASE_OFST \
						+ 0x00000098U)
#define TC956X_GLUE_TL_LINK_SPEED_MON		(TC956X_GLUE_LOGIC_BASE_OFST \
						+ 0x00000244U)
#define TC956X_GLUE_TL_NUM_LANES_MON		(TC956X_GLUE_LOGIC_BASE_OFST \
						+ 0x00000248U)
#define TC956X_GLUE_RSVD_RW0			(TC956X_GLUE_LOGIC_BASE_OFST \
						+ 0x0000024CU)

#define TC956X_GLUE_LTSSM_STATE_MASK		(0x0000001FU)
#define TC956X_GLUE_LTSSM_STATE_SHIFT		(0)
#define TC956X_GLUE_DLL_MASK			(0x00000020U)
#define TC956X_GLUE_DLL_SHIFT			(5)
#define TC956X_GLUE_SPEED_MASK(x)		(0x0000000FU << (8*x))
#define TC956X_GLUE_SPEED_SHIFT(x)		(8*x)

#define USP_LANE_WIDTH_MASK			(0x0000003F)
#define DSP1_LANE_WIDTH_MASK			(0x00003F00)
#define DSP1_LANE_WIDTH_SHIFT			(8)
#define TC956X_GLUE_LANE_WIDTH_MASK(x)		(0x0000003FU << (8*x))
#define TC956X_GLUE_LANE_WIDTH_SHIFT(x)		(8*x)

#define MAX_MAC_ADDR_FILTERS 32

#define TC956X_MIN_LPI_AUTO_ENTRY_TIMER		0
#define TC956X_MAX_LPI_AUTO_ENTRY_TIMER		0xFFFF8 /* LPI Entry timer is in the units of 8 micro second granularity. So mask the last 3 bits. */

//
// Definitions taken from hwif.h in vendor driver
//

/*PMA module*/
struct tc956xmac_pma_ops {
	int (*init)(struct stmmac_priv *priv, void __iomem *pmaaddr);
};

#define tc956x_pma_setup(__priv, __args...) \
	stmmac_do_callback(__priv, pma, init, __priv, __args)

//
// Definitions taken from tc956xmac_ioctl.h in vendor driver
//

/**
 * enum port - Enumeration for ports available
 */
enum ports {
	UPSTREAM_PORT     = 0U, /* Used for Calculating port Offset */
	DOWNSTREAM_PORT1  = 1U,
	DOWNSTREAM_PORT2  = 2U,
	INTERNAL_ENDPOINT = 3U,
};

/**
 * enum state_log_enable - Enumeration for State Log Enable
 */
enum state_log_enable {
	STATE_LOG_DISABLE = 0U,
	STATE_LOG_ENABLE  = 1U,
};

/**
 * struct tc956x_pcie_link_params - PCIe Link Parameters
 *
 * PCIe Link Parameters
 * ltssm : Link Training and Status State Machine(LTSSM) Value(0 to 0x1F).
 * dll : Data Link Layer Active/Inactive State Value(0, 1).
 * speed : Link Speed (Gen1 : 2.5GT/s, Gen2 : 5 GT/s, Gen3 : 8GT/s).
 * width : Number of Active Lanes(0, 1, 2, 3, 4).
 */
struct tc956x_pcie_link_params {
	__u8 ltssm; /* Current Link Training and Status State Machine(LTSSM) Value */
	__u8 dll; /* Current Data Link Layer State */
	__u8 speed; /* Current Link Speed */
	__u8 width; /* Current Link Width */
};

/**
 * struct tc956x_ioctl_state_log_summary - IOCTL arguments for
 * State Logging Summary.
 *
 * cmd - TC956X_PCIE_STATE_LOG_SUMMARY IOCTL.
 * port - USP/DSP1/DSP2/EP for which state logging enable/disbale to be done.
 */
struct tc956x_ioctl_state_log_summary {
	__u32 cmd;
	enum ports port; /* USP, DSP1, DSP2, EP*/
};

/**
 * struct tc956x_ioctl_state_log_enable - IOCTL arguments for
 * Enabling/Disabling State Logging.
 *
 * cmd - TC956X_PCIE_STATE_LOG_ENABLE IOCTL.
 * enable - enable/disable state log.
 * port - USP/DSP1/DSP2/EP for which state logging enable/disbale to be done.
 */
struct tc956x_ioctl_state_log_enable {
	__u32 cmd;
	enum state_log_enable enable; /* Enable/Disable */
	enum ports port; /* USP, DSP1, DSP2, EP*/
};

/**
 * struct tc956x_ioctl_pcie_link_params - IOCTL arguments for State log data
 *
 * cmd - TC956X_PCIE_GET_PCIE_LINK_PARAMS IOCTL.
 * link_param - structure for pcie link parameters to read.
 * port - USP/DSP1/DSP2/EP for which state link parameters to be read.
 */
struct tc956x_ioctl_pcie_link_params {
	__u32 cmd;
	struct tc956x_pcie_link_params *link_param;
	enum ports port;
};

//
// Definitions taken from tc956x_pcie_logstat.h in vendor driver
//

/* ===================================
 * Macros
 * ===================================
 */
/* Configuration Register Address */
#define TC956X_CONF_REG_NPCIEUSPLOGCFG				(0x00001320U)
#define TC956X_CONF_REG_NPCIEUSPLOGCTRL				(0x00001324U)
#define TC956X_CONF_REG_NPCIEUSPLOGST				(0x00001328U)
#define TC956X_CONF_REG_NPCIEUSPLOGRDCTRL			(0x0000132CU)
#define TC956X_CONF_REG_NPCIEUSPLOGD				(0x00001330U)

#define LTSSM_CONF_REG_OFFSET					(0x20U)

 /* LTSSM Enable Bit Mask Value*/
#define LTSSM_BIT_MASK						(0x00000001U)
#define LTSSM_PORT_EN_SHIFT					(0x4U)
/* Common NPCIEUSPLOGCFG, NPCIEDSP1LOGCFG, NPCIEDSP2LOGCFG, NPCIEEPLOGCFG
 * register Logging Configuration Bit Mask and Shift Value
 */
#define STOP_COUNT_VALUE_MASK					(0x00000FF0U)
#define STOP_COUNT_VALUE_SHIFT					(4U)
#define LINKWIDTH_DOWN_ST_MASK					(0x00000008U)
#define LINKWIDTH_DOWN_ST_SHIFT					(3U)
#define LINKSPEED_DOWN_ST_MASK					(0x00000004U)
#define LINKSPEED_DOWN_ST_SHIFT					(2U)
#define TIMEOUT_STOP_MASK					(0x00000002U)
#define TIMEOUT_STOP_SHIFT					(1U)
#define L0S_MASK_MASK						(0x00000001U)
#define L0S_MASK_SHIFT						(0U)
#define STATE_LOGGING_ENABLE_MASK				(0x00000001U)
#define STATE_LOGGING_ENABLE_SHIFT				(0U)
#define FIFO_READ_POINTER_MASK					(0x0000001FU)
#define FIFO_READ_POINTER_SHIFT					(0U)
#define STOP_STATUS_MASK					(0x00000001U)
#define STOP_STATUS_SHIFT					(0U)

#define COUNT_LTSSM_REG_STATES					(28U)

/* Common NPCIEUSPLOGD, NPCIEDSP1LOGD, NPCIEDSP2LOGD, NPCIEEPLOGD
 * Register Logging Read Data Bit Mask and Shift Value
 */
#define FIFO_READ_VALUE8_MASK					(0x20000000U)
#define FIFO_READ_VALUE8_SHIFT					(29U)
#define FIFO_READ_VALUE7_MASK					(0x10000000U)
#define FIFO_READ_VALUE7_SHIFT					(28U)
#define FIFO_READ_VALUE6_MASK					(0x03000000U)
#define FIFO_READ_VALUE6_SHIFT					(24U)
#define FIFO_READ_VALUE5_MASK					(0x00F00000U)
#define FIFO_READ_VALUE5_SHIFT					(20U)
#define FIFO_READ_VALUE4_MASK					(0x00070000U)
#define FIFO_READ_VALUE4_SHIFT					(16U)
#define FIFO_READ_VALUE3_MASK					(0x0000C000U)
#define FIFO_READ_VALUE3_SHIFT					(14U)
#define FIFO_READ_VALUE2_MASK					(0x00003000U)
#define FIFO_READ_VALUE2_SHIFT					(12U)
#define FIFO_READ_VALUE1_MASK					(0x00000300U)
#define FIFO_READ_VALUE1_SHIFT					(8U)
#define FIFO_READ_VALUE0_MASK					(0x0000001FU)
#define FIFO_READ_VALUE0_SHIFT					(0U)
#define STOP_STATUS_MASK					(0x00000001U)
#define STOP_STATUS_SHIFT					(0U)
/* Common NPCIEUSPLOGD, NPCIEDSP1LOGD, NPCIEDSP2LOGD, NPCIEEPLOGD
 * Register, Different Lanes Bit Mask Values
 */
#define LANE0_MASK						(0x1U)
#define LANE0_SHIFT						(0U)
#define LANE1_MASK						(0x2U)
#define LANE1_SHIFT						(1U)
#define LANE2_MASK						(0x4U)
#define LANE2_SHIFT						(2U)
#define LANE3_MASK						(0x8U)
#define LANE3_SHIFT						(3U)

#define MAX_STOP_CNT						(0xFFU)
#define MAX_FIFO_POINTER					(31U)

#define STATE_LOG_REG_OFFSET					(0x20U)
#define GLUE_REG_LTSSM_OFFSET					(0x40U)

#define STATE_LOG_STOP						(1U)
#define MAX_FIFO_READ_POINTER					(0x1F)
#define INVALID_STATE_LOG					(0x33F7F31FU)

#define LTSSM_TIMEOUT_NOT_OCCURRED				(0U)
#define LTSSM_TIMEOUT_OCCURRED					(1U)
#define DL_ACTIVE						(1U)
#define DL_NOT_ACTIVE						(0U)
#define ALL_LANES_INACTIVE					(0U)
#define INACTIVE_L0s						(0U)
#define EQ_PHASE0						(0U)
#define INACTIVE_L1						(0U)

#define LTSSM_MAX_VALUE						(0x1A)
#define LOGSTAT_DUMMY_VALUE					(0xFF)
#define ACTIVE_SINGLE_LANE_MASK					(1)
#define ACTIVE_SINGLE_LANE_SHIFT				(1)
#define ACTIVE_ALL_LANE_MASK					(0xF)
/* ===================================
 * Enumeration
 * ===================================
 */

/* ===================================
 * Structure/Union
 * ===================================
 */
union tc956x_logstat_State_Log_Data {
	struct {
		unsigned char fifo_read_value0 :5;
		unsigned char reserved1 :3;
		unsigned char fifo_read_value1 :2;
		unsigned char reserved2 :2;
		unsigned char fifo_read_value2 :2;
		unsigned char fifo_read_value3 :2;
		unsigned char fifo_read_value4 :3;
		unsigned char reserved3 :1;
		unsigned char fifo_read_value5 :4;
		unsigned char fifo_read_value6 :2;
		unsigned char reserved4 :2;
		unsigned char fifo_read_value7 :1;
		unsigned char fifo_read_value8 :1;
		unsigned char reserved5 :2;
	} bitfield;
	unsigned int reg_val;
};

//
// Code from tc956x_main.c in vendor driver
//

/**
 *  tc956x_GPIO_OutputConfigPin - to configure GPIO as output and write the value
 *  @priv: driver private structure
 *  @gpio_pin: GPIO pin number
 *  @out_value : value to write to the GPIO pin. Can be 0 or 1
 *  @remarks : Only GPIO0- GPIO06, GPI010-GPIO12 are allowed
 */
static int tc956x_GPIO_OutputConfigPin(struct stmmac_priv *priv, u32 gpio_pin, u8 out_value)
{
	u32 config, val;

	/* Only GPIO0- GPIO06, GPI010-GPIO12 are allowed */
	switch (gpio_pin) {
	case GPIO_00:
		val = readl(priv->ioaddr + NFUNCEN4_OFFSET);
		val &= ~NFUNCEN4_GPIO_00;
		val |= (NFUNCEN_FUNC0 << NFUNCEN4_GPIO_00_SHIFT);
		writel(val, priv->ioaddr + NFUNCEN4_OFFSET);
		break;
	case GPIO_01:
		val = readl(priv->ioaddr + NFUNCEN4_OFFSET);
		val &= ~NFUNCEN4_GPIO_01;
		val |= (NFUNCEN_FUNC0 << NFUNCEN4_GPIO_01_SHIFT);
		writel(val, priv->ioaddr + NFUNCEN4_OFFSET);
		break;
	case GPIO_02:
		val = readl(priv->ioaddr + NFUNCEN4_OFFSET);
		val &= ~NFUNCEN4_GPIO_02;
		val |= (NFUNCEN_FUNC0 << NFUNCEN4_GPIO_02_SHIFT);
		writel(val, priv->ioaddr + NFUNCEN4_OFFSET);
		break;
	case GPIO_03:
		val = readl(priv->ioaddr + NFUNCEN4_OFFSET);
		val &= ~NFUNCEN4_GPIO_03;
		val |= (NFUNCEN_FUNC0 << NFUNCEN4_GPIO_03_SHIFT);
		writel(val, priv->ioaddr + NFUNCEN4_OFFSET);
		break;
	case GPIO_04:
		val = readl(priv->ioaddr + NFUNCEN4_OFFSET);
		val &= ~NFUNCEN4_GPIO_04;
		val |= (NFUNCEN_FUNC0 << NFUNCEN4_GPIO_04_SHIFT);
		writel(val, priv->ioaddr + NFUNCEN4_OFFSET);
		break;
	case GPIO_05:
		val = readl(priv->ioaddr + NFUNCEN4_OFFSET);
		val &= ~NFUNCEN4_GPIO_05;
		val |= (NFUNCEN_FUNC0 << NFUNCEN4_GPIO_05_SHIFT);
		writel(val, priv->ioaddr + NFUNCEN4_OFFSET);
		break;
	case GPIO_06:
		val = readl(priv->ioaddr + NFUNCEN4_OFFSET);
		val &= ~NFUNCEN4_GPIO_06;
		val |= (NFUNCEN_FUNC0 << NFUNCEN4_GPIO_06_SHIFT);
		writel(val, priv->ioaddr + NFUNCEN4_OFFSET);
		break;
	case GPIO_10:
		val = readl(priv->ioaddr + NFUNCEN5_OFFSET);
		val &= ~NFUNCEN5_GPIO_10;
		val |= (NFUNCEN_FUNC0 << NFUNCEN5_GPIO_10_SHIFT);
		writel(val, priv->ioaddr + NFUNCEN5_OFFSET);
		break;
	case GPIO_11:
		val = readl(priv->ioaddr + NFUNCEN5_OFFSET);
		val &= ~NFUNCEN5_GPIO_11;
		val |= (NFUNCEN_FUNC0 << NFUNCEN5_GPIO_11_SHIFT);
		writel(val, priv->ioaddr + NFUNCEN5_OFFSET);
		break;
	case GPIO_12:
		val = readl(priv->ioaddr + NFUNCEN6_OFFSET);
		val &= ~NFUNCEN6_GPIO_12;
		val |= (NFUNCEN_FUNC0 << NFUNCEN6_GPIO_12_SHIFT);
		writel(val, priv->ioaddr + NFUNCEN6_OFFSET);
		break;
	case GPIO_13:
		val = readl(priv->ioaddr + NFUNCEN7_OFFSET);
		val &= ~NFUNCEN7_GPIO_13;
		val |= (NFUNCEN_FUNC2 << NFUNCEN7_GPIO_13_SHIFT);
		writel(val, priv->ioaddr + NFUNCEN7_OFFSET);
		break;
	default:
		netdev_err(priv->dev, "Invalid GPIO pin - %d\n", gpio_pin);
		return -EPERM;
	}

	priv->saved_gpio_config[gpio_pin].config = 1;

	/* Write data to GPIO pin */
	if (gpio_pin < GPIO_32) {
		config = 1 << gpio_pin;
		val = readl(priv->ioaddr + GPIOO0_OFFSET);
		val &= ~config;
		if (out_value)
			val |= config;

		writel(val, priv->ioaddr + GPIOO0_OFFSET);
	}  else {
		config = 1 << (gpio_pin - GPIO_32);
		val = readl(priv->ioaddr + GPIOO1_OFFSET);
		val &= ~config;
		if (out_value)
			val |= config;

		writel(val, priv->ioaddr + GPIOO1_OFFSET);
	}

	priv->saved_gpio_config[gpio_pin].out_val = out_value;

	/* Configure the GPIO pin in output direction */
	if (gpio_pin < GPIO_32) {
		config = ~(1 << gpio_pin);
		val = readl(priv->ioaddr + GPIOE0_OFFSET);
		writel(val & config, priv->ioaddr + GPIOE0_OFFSET);
	} else {
		config = ~(1 << (gpio_pin - GPIO_32));
		val = readl(priv->ioaddr + GPIOE1_OFFSET);
		writel(val & config, priv->ioaddr + GPIOE1_OFFSET);
	}

	return 0;
}

/**
 *  tc956x_gpio_restore_configuration - to restore the saved configuration of GPIO
 *  @priv: driver private structure
 *  @remarks : Only GPIO0- GPIO06, GPI010-GPIO12 are allowed
 */
static int tc956x_gpio_restore_configuration(struct stmmac_priv *priv)
{
	u32 config, val, gpio_pin, out_value;

	dev_dbg(priv->device, "-->%s", __func__);

	for (gpio_pin = 0; gpio_pin <= GPIO_13; gpio_pin++) {

		/* Restore only the GPIOs which were configured/saved */
		if (!(priv->saved_gpio_config[gpio_pin].config))
			continue;

		dev_dbg(priv->device, "%s : Restoring GPIO configuration for pin: %d, val: %d",
				__func__, gpio_pin, priv->saved_gpio_config[gpio_pin].out_val);

		/* Only GPIO0- GPIO06, GPI010-GPIO12 are allowed */
		switch (gpio_pin) {
		case GPIO_00:
			val = readl(priv->ioaddr + NFUNCEN4_OFFSET);
			val &= ~NFUNCEN4_GPIO_00;
			val |= (NFUNCEN_FUNC0 << NFUNCEN4_GPIO_00_SHIFT);
			writel(val, priv->ioaddr + NFUNCEN4_OFFSET);
			break;
		case GPIO_01:
			val = readl(priv->ioaddr + NFUNCEN4_OFFSET);
			val &= ~NFUNCEN4_GPIO_01;
			val |= (NFUNCEN_FUNC0 << NFUNCEN4_GPIO_01_SHIFT);
			writel(val, priv->ioaddr + NFUNCEN4_OFFSET);
			break;
		case GPIO_02:
			val = readl(priv->ioaddr + NFUNCEN4_OFFSET);
			val &= ~NFUNCEN4_GPIO_02;
			val |= (NFUNCEN_FUNC0 << NFUNCEN4_GPIO_02_SHIFT);
			writel(val, priv->ioaddr + NFUNCEN4_OFFSET);
			break;
		case GPIO_03:
			val = readl(priv->ioaddr + NFUNCEN4_OFFSET);
			val &= ~NFUNCEN4_GPIO_03;
			val |= (NFUNCEN_FUNC0 << NFUNCEN4_GPIO_03_SHIFT);
			writel(val, priv->ioaddr + NFUNCEN4_OFFSET);
			break;
		case GPIO_04:
			val = readl(priv->ioaddr + NFUNCEN4_OFFSET);
			val &= ~NFUNCEN4_GPIO_04;
			val |= (NFUNCEN_FUNC0 << NFUNCEN4_GPIO_04_SHIFT);
			writel(val, priv->ioaddr + NFUNCEN4_OFFSET);
			break;
		case GPIO_05:
			val = readl(priv->ioaddr + NFUNCEN4_OFFSET);
			val &= ~NFUNCEN4_GPIO_05;
			val |= (NFUNCEN_FUNC0 << NFUNCEN4_GPIO_05_SHIFT);
			writel(val, priv->ioaddr + NFUNCEN4_OFFSET);
			break;
		case GPIO_06:
			val = readl(priv->ioaddr + NFUNCEN4_OFFSET);
			val &= ~NFUNCEN4_GPIO_06;
			val |= (NFUNCEN_FUNC0 << NFUNCEN4_GPIO_06_SHIFT);
			writel(val, priv->ioaddr + NFUNCEN4_OFFSET);
			break;
		case GPIO_10:
			val = readl(priv->ioaddr + NFUNCEN5_OFFSET);
			val &= ~NFUNCEN5_GPIO_10;
			val |= (NFUNCEN_FUNC0 << NFUNCEN5_GPIO_10_SHIFT);
			writel(val, priv->ioaddr + NFUNCEN5_OFFSET);
			break;
		case GPIO_11:
			val = readl(priv->ioaddr + NFUNCEN5_OFFSET);
			val &= ~NFUNCEN5_GPIO_11;
			val |= (NFUNCEN_FUNC0 << NFUNCEN5_GPIO_11_SHIFT);
			writel(val, priv->ioaddr + NFUNCEN5_OFFSET);
			break;
		case GPIO_12:
			val = readl(priv->ioaddr + NFUNCEN6_OFFSET);
			val &= ~NFUNCEN6_GPIO_12;
			val |= (NFUNCEN_FUNC0 << NFUNCEN6_GPIO_12_SHIFT);
			writel(val, priv->ioaddr + NFUNCEN6_OFFSET);
			break;
		case GPIO_13:
			val = readl(priv->ioaddr + NFUNCEN7_OFFSET);
			val &= ~NFUNCEN7_GPIO_13;
			val |= (NFUNCEN_FUNC2 << NFUNCEN7_GPIO_13_SHIFT);
			writel(val, priv->ioaddr + NFUNCEN7_OFFSET);
			break;
		default:
			netdev_err(priv->dev, "Invalid GPIO pin - %d\n", gpio_pin);
			return -EPERM;
		}

		out_value = priv->saved_gpio_config[gpio_pin].out_val;

		/* Write data to GPIO pin */
		if (gpio_pin < GPIO_32) {
			config = 1 << gpio_pin;
			val = readl(priv->ioaddr + GPIOO0_OFFSET);
			val &= ~config;
			if (out_value)
				val |= config;

			writel(val, priv->ioaddr + GPIOO0_OFFSET);
		}  else {
			config = 1 << (gpio_pin - GPIO_32);
			val = readl(priv->ioaddr + GPIOO1_OFFSET);
			val &= ~config;
			if (out_value)
				val |= config;

			writel(val, priv->ioaddr + GPIOO1_OFFSET);
		}

		/* Configure the GPIO pin in output direction */
		if (gpio_pin < GPIO_32) {
			config = ~(1 << gpio_pin);
			val = readl(priv->ioaddr + GPIOE0_OFFSET);
		writel(val & config, priv->ioaddr + GPIOE0_OFFSET);
		} else {
			config = ~(1 << (gpio_pin - GPIO_32));
			val = readl(priv->ioaddr + GPIOE1_OFFSET);
			writel(val & config, priv->ioaddr + GPIOE1_OFFSET);
		}
	}
	return 0;
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
#define XGMAC_SR_XS_PCS_STS1			0xC0004
#define XGMAC_SR_XS_PCS_CTRL2			0xC001C
#define XGMAC_SR_XS_PCS_EEE_ABL			0xC0050
#define XGMAC_SR_XS_PCS_STS2			0xC0020
#define XGMAC_VR_XS_PCS_DIG_CTRL1		0xe0000
#define XGMAC_VR_XS_PCS_EEE_MCTRL0		0xe0018
#define XGMAC_VR_XS_PCS_EEE_MCTRL1		0xe002c
#define XGMAC_VR_XS_PCS_KR_CTRL			0xe001c
#define XGMAC_VR_XS_PCS_EEE_TXTIMER		0xe0020
#define XGMAC_VR_XS_PCS_EEE_RXTIMER		0xe0024
#define XGMAC_VR_XS_PCS_DIG_STS			0xe0040
#define XGMAC_VR_MII_AN_INTR_STS		0x7e0008
#define XGMAC_SR_XS_PCS_STS2			0xC0020

#define XGMAC_LTX_LRX_STATE			0xFC00
#define XGMAC_LPI_RECEIVE_STATE			0x1C00
#define XGMAC_LPI_TRANSMIT_STATE		0xE000
#define XGMAC_RX_LPI_RECEIVE			0x400
#define XGAMC_TX_LPI_RECEIVE			0x800
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
#define XGMAC_MULT_FACT_100NS_PHY		0xA00
#define XGMAC_EEE_TX_TIMER_MAC_CONT		0x0543
#define XGMAC_EEE_TX_TIMER_PHY_CONT		0x0E9C
#define XGMAC_EEE_RX_TIMER_MAC_CONT		0x062A
#define XGMAC_EEE_RX_TIMER_PHY_CONT		0x2888
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
#define XGMAC_USXG_AN_STS_SPEED_MASK	0x00001c00/*USXGMII autonegotiated speed*/
#define XGMAC_USXG_AN_STS_DUPLEX_MASK	0x00002000/*USXGMII autonegtiated duplex*/
#define XGMAC_USXG_AN_STS_LINK_MASK		0x00004000/*USXGMII link status*/
#define XGMAC_SGM_STS_LINK_MASK			0x00000010/*SGMII link status*/
#define XGMAC_SGM_STS_DUPLEX			0x00000002/*SGMII autonegotiated duplex*/
#define XGMAC_SGM_STS_SPEED_MASK		0x0000000c/*SGMII autonegotiated speed*/
#define XGMAC_SOFT_RST					0x00008000/*SOFT RST*/
#define XGMAC_C37_AN_COMPL				0x00000001/*C37 Autoneg complete*/
#define XGMAC_SR_MII_CTRL_SPEED			0x00002060/* SR_MII_CTRL Reg SPEED SS13, SS6, SS5 */
#define XGMAC_SR_MII_CTRL_SPEED_10G		0x00002040/* SR_MII_CTRL SPEED: 10G */
#define XGMAC_SR_MII_CTRL_SPEED_5G		0x00002020/* SR_MII_CTRL SPEED: 5G */
#define XGMAC_SR_MII_CTRL_SPEED_2_5G	0x00000020/* SR_MII_CTRL SPEED: 5G */
#define XGMAC_SR_MII_CTRL_SPEED_1G		0x00000040/* SR_MII_CTRL SPEED: 1G */
#define XGMAC_SR_MII_CTRL_SPEED_100M	0x00002000/* SR_MII_CTRL SPEED: 100M */
#define XGMAC_SR_MII_CTRL_SPEED_10M		0x00000000/* SR_MII_CTRL SPEED: 10M */

#define XGMAC_USRA_RST					0x400/* USRA_RST */
#define XGMAC_EEE_LRX_EN			BIT(1)		/* LPI Rx Enable */

#define XPCS_REG_BASE_ADDR				10
#define XPCS_REG_OFFSET					0x0003FF
#define XPCS_IND_ACCESS					0x3FC
#define XPCS_SS_SGMII_1G				0x40
#define XPCS_SS_SGMII_100M				0x2000
#define XPCS_SS_SGMII_10M				0x0

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

	pr_debug("XPCS Indirect Access Base Register : %x, offset : %x", base_address, offset);
	/*write base address to (PCS address + 0x3FC) register*/
	writel(base_address, (xpcsaddr + XPCS_IND_ACCESS));

	/*Access to offset address (PCS address + offset)*/
	reg_value = readl(xpcsaddr + offset);
	pr_debug("XPCS register %x indirect read access value : %x", pcs_reg_num, reg_value);

	return reg_value;
}

static u32 tc956x_xpcs_write(void __iomem *xpcsaddr, u32 pcs_reg_num, u32 value)
{
	u16 base_address, offset;

	base_address = pcs_reg_num >> XPCS_REG_BASE_ADDR;
	offset = pcs_reg_num & XPCS_REG_OFFSET;

	pr_debug("XPCS Indirect Access Base Register : %x, offset : %x", base_address, offset);
	/*write base address to (PCS address + 0x3FC) register*/
	writel(base_address, (xpcsaddr + XPCS_IND_ACCESS));

	/*Access to offset address (PCS address + offset)*/
	writel(value, xpcsaddr + offset);
	pr_debug("XPCS register %x indirect write access value : %x", pcs_reg_num, value);

	return 0;
}


static int tc956x_xpcs_init(struct stmmac_priv *priv, void __iomem *xpcsaddr)
{
	u32 reg_value;

	dev_dbg(priv->device, "-->%s\n", __func__);

	reg_value = tc956x_xpcs_read(xpcsaddr, XGMAC_SR_MII_CTRL);
	if (reg_value & XGMAC_SOFT_RST)
		return -1;

#ifdef CONFIG_TC956X_MAGIC_PACKET_WOL_CONF
	if (priv->wol_config_enabled != true) {
#endif /* #ifdef CONFIG_TC956X_MAGIC_PACKET_WOL_CONF */
		/*Clause 37 autoneg related settings*/
		if (priv->plat->phy_interface == PHY_INTERFACE_MODE_SGMII) {
			//DK2
			//PCS Type Select SR_XS_PCS_CTRL2  PCS_TYPE_SEL -> 1
			reg_value = tc956x_xpcs_read(xpcsaddr, XGMAC_SR_XS_PCS_CTRL2);
			reg_value &= XGMAC_PCS_TYPE_SEL;
			reg_value |= 0x1;
			tc956x_xpcs_write(xpcsaddr, XGMAC_SR_XS_PCS_CTRL2, reg_value);

			reg_value = tc956x_xpcs_read(xpcsaddr, XGMAC_VR_MII_AN_CTRL);
			reg_value &= XGMAC_PCS_MODE_MASK;
			reg_value |= XGMAC_SGMII_MODE;/*SGMII PCS MODE*/
			tc956x_xpcs_write(xpcsaddr, XGMAC_VR_MII_AN_CTRL, reg_value);

			if (priv->is_sgmii_2p5g == true) {
				reg_value = tc956x_xpcs_read(xpcsaddr, XGMAC_VR_XS_PCS_DIG_CTRL1);
				reg_value &= ~(0x4);
				/* Enable only if SGMII 2.5G is enabled */
				reg_value |= 0x4; /*EN_2_5G_MODE*/
				tc956x_xpcs_write(xpcsaddr, XGMAC_VR_XS_PCS_DIG_CTRL1, reg_value);
			}
		}
		if ((priv->plat->phy_interface == PHY_INTERFACE_MODE_USXGMII) ||
			(priv->plat->phy_interface == PHY_INTERFACE_MODE_10GKR)
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 6, 0)
			|| (priv->plat->phy_interface == PHY_INTERFACE_MODE_10GBASER)
#endif
			) {
			reg_value = tc956x_xpcs_read(xpcsaddr, XGMAC_SR_XS_PCS_CTRL2);
			reg_value &= XGMAC_PCS_TYPE_SEL;/*PCS_TYPE_SEL as 10GBASE-R PCS */
			tc956x_xpcs_write(xpcsaddr, XGMAC_SR_XS_PCS_CTRL2, reg_value);

			reg_value = tc956x_xpcs_read(xpcsaddr, XGMAC_VR_XS_PCS_DIG_CTRL1);
			if (priv->plat->phy_interface == PHY_INTERFACE_MODE_10GKR
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 6, 0)
				|| (priv->plat->phy_interface == PHY_INTERFACE_MODE_10GBASER)
#endif
				) {
				reg_value &= (~XGMAC_USXG_EN); /*Disable USXG_EN*/
			} else {
				reg_value |= XGMAC_USXG_EN; /*set USXG_EN*/
			}

			tc956x_xpcs_write(xpcsaddr, XGMAC_VR_XS_PCS_DIG_CTRL1, reg_value);

			reg_value = tc956x_xpcs_read(xpcsaddr, XGMAC_VR_XS_PCS_KR_CTRL);
			reg_value &= ~XGMAC_USXG_MODE;/*USXG_MODE : 0x000*/
			if (priv->plat->port_interface == ENABLE_USXGMII_5G_INTERFACE)
				reg_value |= XPCS_USX_5G_MODE;
			else if (priv->plat->port_interface == ENABLE_USXGMII_2_5G_INTERFACE)
				reg_value |= XPCS_USX_2_5G_MODE;
			tc956x_xpcs_write(xpcsaddr, XGMAC_VR_XS_PCS_KR_CTRL, reg_value);

			reg_value = tc956x_xpcs_read(xpcsaddr, XGMAC_VR_XS_PCS_DIG_CTRL1);
			reg_value |= XGMAC_VR_RST;/*set VR_RST*/
			tc956x_xpcs_write(xpcsaddr, XGMAC_VR_XS_PCS_DIG_CTRL1, reg_value);

			/*Wait for Reset to clear*/
			do {
				reg_value = tc956x_xpcs_read(xpcsaddr, XGMAC_VR_XS_PCS_DIG_CTRL1);
			} while ((XGMAC_VR_RST & reg_value) == XGMAC_VR_RST);

		}
#ifdef CONFIG_TC956X_MAGIC_PACKET_WOL_CONF
	} else { /* SerDES Configuration for WOL SGMII 1G when native interface other than SGMII. */
		pr_debug("%s Port %d %s: Entered with flag priv->wol_config_enabled %d", __func__, priv->port_num, priv->dev->name, priv->wol_config_enabled);
		reg_value = tc956x_xpcs_read(xpcsaddr, XGMAC_SR_XS_PCS_CTRL2);
			reg_value &= XGMAC_PCS_TYPE_SEL;
			reg_value |= 0x1;
			tc956x_xpcs_write(xpcsaddr, XGMAC_SR_XS_PCS_CTRL2, reg_value);

			reg_value = tc956x_xpcs_read(xpcsaddr, XGMAC_VR_MII_AN_CTRL);
			reg_value &= XGMAC_PCS_MODE_MASK;
			reg_value |= XGMAC_SGMII_MODE;/*SGMII PCS MODE*/
			tc956x_xpcs_write(xpcsaddr, XGMAC_VR_MII_AN_CTRL, reg_value);
	}
#endif /* #ifdef CONFIG_TC956X_MAGIC_PACKET_WOL_CONF */
#ifdef EEE
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
#ifdef EEE_MAC_CONTROLLED_MODE
	reg_value |= XGMAC_MULT_FACT_100NS_MAC; /* MULT_FACT_100NS */
#else
	reg_value |= XGMAC_MULT_FACT_100NS_PHY; /* MULT_FACT_100NS */
#endif
	reg_value |= XGMAC_SIGN_BIT;/* SIGN_BIT */
	reg_value |= XGMAC_TX_RX_EN;/* TX_EN_CTRL, RX_EN_CTRL */
	tc956x_xpcs_write(xpcsaddr, XGMAC_VR_XS_PCS_EEE_MCTRL0, reg_value);

	reg_value = tc956x_xpcs_read(xpcsaddr, XGMAC_VR_XS_PCS_EEE_TXTIMER);
	reg_value &= ~(XGMAC_EEE_TX_TIMER);
#ifdef EEE_MAC_CONTROLLED_MODE
	reg_value |= XGMAC_EEE_TX_TIMER_MAC_CONT; /* TWL_RES=0x5, T1U_RES=0x1, TSL_RES=0x3 */
#else
	reg_value |= XGMAC_EEE_TX_TIMER_PHY_CONT; /* TWL_RES=0xe, T1U_RES=0x8, TSL_RES=0x1c */
#endif
	tc956x_xpcs_write(xpcsaddr, XGMAC_VR_XS_PCS_EEE_TXTIMER, reg_value);

	reg_value = tc956x_xpcs_read(xpcsaddr, XGMAC_VR_XS_PCS_EEE_RXTIMER);
	reg_value &= ~(XGMAC_EEE_RX_TIMER);
#ifdef EEE_MAC_CONTROLLED_MODE
	reg_value |= XGMAC_EEE_RX_TIMER_MAC_CONT; /* TWR_RES=0x6, RES_100U=0x42 */
#else
	reg_value |= XGMAC_EEE_RX_TIMER_PHY_CONT; /* TWR_RES=0x88, RES_100U=0x28 */
#endif
	tc956x_xpcs_write(xpcsaddr, XGMAC_VR_XS_PCS_EEE_RXTIMER, reg_value);

	reg_value = tc956x_xpcs_read(xpcsaddr, XGMAC_VR_XS_PCS_EEE_MCTRL1);
	reg_value |= XGMAC_TRN_LPI;
	tc956x_xpcs_write(xpcsaddr, XGMAC_VR_XS_PCS_EEE_MCTRL1, reg_value);

	reg_value = tc956x_xpcs_read(xpcsaddr, XGMAC_VR_XS_PCS_EEE_MCTRL0);

	reg_value &= ~XGMAC_TX_RX_QUIET_EN;
	reg_value |= XGMAC_TX_RX_QUIET_EN; /* RX_QUIET_EN, TX_QUIET_EN, LRX_EN, LTX_EN */

	tc956x_xpcs_write(xpcsaddr, XGMAC_VR_XS_PCS_EEE_MCTRL0, reg_value);
#endif

	reg_value = tc956x_xpcs_read(xpcsaddr, XGMAC_VR_MII_AN_CTRL);
	reg_value &= XGMAC_TX_CFIG_INTR_EN_MASK;/*TX_CONFIG MAC SIDE*/
	reg_value |= XGMAC_MII_AN_INTR_EN;/*MII_AN_INTR_EN enabe*/
	tc956x_xpcs_write(xpcsaddr, XGMAC_VR_MII_AN_CTRL, reg_value);

	reg_value = tc956x_xpcs_read(xpcsaddr, XGMAC_VR_MII_DIG_CTRL1);
	reg_value &= ~XGMAC_MAC_AUTO_SW_EN;/*MAC_AUTO_SW enable*/
	if (priv->is_sgmii_2p5g != true)
		/* Enable only if SGMII 2.5G is not enabled. */
		reg_value |= XGMAC_MAC_AUTO_SW_EN;
	tc956x_xpcs_write(xpcsaddr, XGMAC_VR_MII_DIG_CTRL1, reg_value);

	return 0;
}

//
// Code from tc956x_qcom.c in vendor driver
//

struct tc956x_qcom_priv {
	struct pinctrl *pinctrl;
	struct pinctrl_state *pinctrl_default;
	struct regulator *phy_supply;
	u32 phy_rst_gpio;
	u32 phy_rst_delay_us;
	int wol_irq;
	bool has_always_on_supplies;
	struct gpio_desc *phy_rst_gpio_som;
};

#define to_priv(priv) \
	((struct tc956x_qcom_priv *)priv->plat_priv)

static int tc956x_assert_phy_reset(struct stmmac_priv *priv)
{
	return tc956x_GPIO_OutputConfigPin(priv, to_priv(priv)->phy_rst_gpio, 0);
}

static int tc956x_deassert_phy_reset(struct stmmac_priv *priv)
{
	return tc956x_GPIO_OutputConfigPin(priv, to_priv(priv)->phy_rst_gpio, 1);
}

static int tc956x_phy_power_on(struct stmmac_priv *priv)
{
	int ret = 0;
	struct tc956x_qcom_priv *qpriv = to_priv(priv);

	if(!qpriv->has_always_on_supplies) {
		ret = regulator_enable(to_priv(priv)->phy_supply);
		if (ret) {
			dev_err(priv->device, "Failed to enable PHY supply with error %d\n", ret);
			return ret;
		}

		ret = tc956x_deassert_phy_reset(priv);
		if (ret) {
			dev_err(priv->device, "Failed to deassert QPS615 GPIO0%d\n", qpriv->phy_rst_gpio);
			if (regulator_disable(qpriv->phy_supply))
				dev_err(priv->device, "Failed to disable regulator\n");
		}
	} else
		gpiod_set_value(qpriv->phy_rst_gpio_som, 1);

	dev_dbg(priv->device,"QPS615 PHY out of reset delay %d", qpriv->phy_rst_delay_us);
	usleep_range(qpriv->phy_rst_delay_us, qpriv->phy_rst_delay_us);

	return ret;
}

static int tc956x_phy_power_off(struct stmmac_priv *priv)
{
	int ret = 0;
	struct tc956x_qcom_priv *qpriv = to_priv(priv);

	if(!qpriv->has_always_on_supplies) {
		ret = tc956x_assert_phy_reset(priv);
		if (ret) {
			dev_err(priv->device, "Failed to assert QPS615 GPIO%02d\n", qpriv->phy_rst_gpio);
				return ret;
		}

		ret = regulator_disable(qpriv->phy_supply);
		if (ret) {
			dev_err(priv->device, "Failed to disable PHY supply with error %d\n", ret);
			if (tc956x_deassert_phy_reset(priv))
				dev_err(priv->device, "Failed to deassert PHY\n");
		}
	} else
		gpiod_set_value(qpriv->phy_rst_gpio_som, 0);

	return ret;
}

static int tc956x_platform_of_parse(struct device *dev,
				    struct tc956x_qcom_priv *qpriv)
{
	qpriv->has_always_on_supplies = of_property_read_bool(dev->of_node, "qcom,always-on-supply");

	if(!qpriv->has_always_on_supplies) {
		if (of_property_read_u32(dev->of_node,"qcom,phy-rst-gpio", &qpriv->phy_rst_gpio)) {
			if (of_property_read_u32(dev->of_node, "qcom,phy-rst-gpio-id",
				&qpriv->phy_rst_gpio)) {
				dev_err(dev, "Failed to get PHY reset GPIO\n");
				return -EINVAL;
			}
		}
	} else {
		qpriv->phy_rst_gpio_som = devm_gpiod_get(dev, "phy-rst-som", GPIOD_OUT_LOW);
		if (IS_ERR(qpriv->phy_rst_gpio_som)) {
			dev_err(dev, "Failed to get PHY reset GPIO: %ld\n", PTR_ERR(qpriv->phy_rst_gpio_som));
			return -EINVAL;
		}
	}

	if (of_property_read_u32(dev->of_node, "qcom,phy-rst-delay-us", &qpriv->phy_rst_delay_us)) {
		dev_err(dev, "Failed to get PHY reset delay time\n");
			return -EINVAL;
	}

	qpriv->wol_irq = of_irq_get_byname(dev->of_node, "wol_irq");
	if (qpriv->wol_irq <= 0) {
		dev_err(dev, "Failed to get 'wol_irq' IRQ with error %d\n", qpriv->wol_irq);
		return -EINVAL;
	}

	if(!qpriv->has_always_on_supplies) {
		qpriv->phy_supply = devm_regulator_get(dev, "phy");
		if (IS_ERR(qpriv->phy_supply)) {
			dev_err(dev, "Failed to acquire supply 'phy-supply': %ld\n", PTR_ERR(qpriv->phy_supply));
			return -EINVAL;
		}
	}

	qpriv->pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR_OR_NULL(qpriv->pinctrl)) {
		dev_err(dev, "Failed to get pinctrl handle\n");
		goto err_pinctrl_get;
	}

	qpriv->pinctrl_default = pinctrl_lookup_state(qpriv->pinctrl, PINCTRL_STATE_DEFAULT);
	if (IS_ERR_OR_NULL(qpriv->pinctrl_default)) {
		dev_err(dev, "Failed to look up '%s' pinctrl state\n", PINCTRL_STATE_DEFAULT);
		goto err_pinctrl_lookup_state;
	}

	return 0;

err_pinctrl_lookup_state:
	devm_pinctrl_put(qpriv->pinctrl);
err_pinctrl_get:
	devm_regulator_put(qpriv->phy_supply);
	return -EINVAL;
}

static int tc956x_platform_probe(struct stmmac_priv *priv,
			  struct stmmac_resources *res)
{
	int ret = 0;
	struct tc956x_qcom_priv *qpriv;

#ifdef RBTC9563_3MA
#ifdef RBTC9563_3DB
	tc956x_GPIO_OutputConfigPin(priv, GPIO_12, 0);
#else
	tc956x_GPIO_OutputConfigPin(priv, GPIO_12, 1);
	tc956x_GPIO_OutputConfigPin(priv, GPIO_13, 0);
#endif
#endif

	dev_dbg(priv->device, "QPS615 platform probing has started\n");

	qpriv = kzalloc(sizeof(*qpriv), GFP_KERNEL);
	if (!qpriv) {
		dev_dbg(priv->device, "Failed to allocate memory for qpriv, exiting\n");
		return -ENOMEM;
	}

	priv->plat_priv = qpriv;

	ret = tc956x_platform_of_parse(priv->device, qpriv);
	if (ret) {
		dev_err(priv->device, "Failed to parse platform device tree\n");
		goto err_parse_properties;
	}

	if(!qpriv->has_always_on_supplies) {

		ret = tc956x_assert_phy_reset(priv);
		if (ret) {
			dev_err(priv->device, "Failed to assert the PHY reset with error %d\n", ret);
			goto err_assert_phy_rst;
		}
	} else
		gpiod_set_value(qpriv->phy_rst_gpio_som, 0);

	ret = pinctrl_select_state(qpriv->pinctrl, qpriv->pinctrl_default);
	if (ret) {
		dev_err(priv->device, "Failed to select the 'default' pincrl state\n");
		goto err_pinctrl_select_state;
	}

	ret = tc956x_phy_power_on(priv);
	if (ret) {
		dev_err(priv->device, "Failed to power on PHY with error %d\n", ret);
		goto err_power_on;
	}

	res->wol_irq = qpriv->wol_irq;
	dev_dbg(priv->device, "QPS615 platform probing has finished successfully\n");

	return 0;

err_power_on:
	irq_set_irq_wake(qpriv->wol_irq, 0);
err_pinctrl_select_state:
err_assert_phy_rst:
err_parse_properties:
	kfree(qpriv);
	priv->plat_priv = NULL;
	return -EINVAL;
}

static int tc956x_platform_remove(struct stmmac_priv *priv)
{
	int ret = 0;
	struct tc956x_qcom_priv *qpriv = to_priv(priv);

	dev_dbg(priv->device, "Freeing QPS615 platform resources\n");

	ret = tc956x_phy_power_off(priv);
	if (ret)
		dev_err(priv->device, "Failed to power off PHY with error %d\n", ret);

	if (!qpriv->has_always_on_supplies)
		devm_regulator_put(qpriv->phy_supply);

	devm_pinctrl_put(qpriv->pinctrl);
	kfree(priv->plat_priv);
	priv->plat_priv = NULL;

	return ret;
}

static int tc956x_platform_suspend(struct stmmac_priv *priv)
{
	int ret = 0;

	if (priv->wolopts) {
		ret = enable_irq_wake(priv->wol_irq);
		if (unlikely(ret))
			dev_err(priv->device, "Failed to set WOL IRQ %d as wake up capable with error %d\n",
				priv->wol_irq, ret);
	} else {
		ret = tc956x_phy_power_off(priv);
		if (ret)
			dev_err(priv->device, "Failed to power off PHY with error %d\n", ret);
	}

	return ret;
}

static int tc956x_platform_resume(struct stmmac_priv *priv)
{
	int ret = 0;

#ifdef RBTC9563_3MA
#ifdef RBTC9563_3DB
	tc956x_GPIO_OutputConfigPin(priv, GPIO_12, 0);
#else
	tc956x_GPIO_OutputConfigPin(priv, GPIO_12, 1);
	tc956x_GPIO_OutputConfigPin(priv, GPIO_13, 0);
#endif
#endif

	if (priv->wolopts) {
		ret = disable_irq_wake(priv->wol_irq);
		if (unlikely(ret))
			dev_err(priv->device, "Failed to set WOL IRQ %d as a wake-disabled irq with error %d\n",
				priv->wol_irq, ret);
	} else {
		ret = tc956x_phy_power_on(priv);
		if (ret)
			dev_err(priv->device, "Failed to power on the PHY with error %d\n", ret);
	}

	return ret;
}

static int tc956x_platform_port_interface_overlay(struct device *dev, struct stmmac_resources *res)
{
	int ret = 0;
	u32 interface;
	u32 mdc_clk;
	u32 c45_state;
	u32 link_down_macrst;

	if (of_property_read_u32(dev->of_node, "qcom,phy-port-interface", &interface)) {
		dev_err(dev, "Failed to get phy port interface\n");
		return ret;
	} else {
		dev_err(dev, "phy port interface overlay to %d from %d\n", interface, res->port_interface);
		res->port_interface = interface;

		if (of_property_read_u32(dev->of_node, "qcom,mdc-clk", &mdc_clk)) {
			dev_err(dev, "Failed to get mdc clk\n");
			return ret;
		} else {
			dev_err(dev, "mdc clk overlay to %d\n", mdc_clk);
			res->mdc_clk = mdc_clk;
		}

		if (of_property_read_u32(dev->of_node, "qcom,c45-state", &c45_state)) {
			dev_err(dev, "Failed to get c45 state\n");
			return ret;
		} else {
			dev_err(dev, "c45 state overlay to %d\n", c45_state);
			res->c45_state = c45_state;
		}

		if (of_property_read_u32(dev->of_node, "qcom,link-down-macrst", &link_down_macrst)) {
			dev_err(dev, "Failed to get link down macrst\n");
			return ret;
		} else {
			dev_err(dev, "link down macrst overlay to %d\n", link_down_macrst);
			res->link_down_macrst = link_down_macrst;
		}
		ret = 1;
	}
	return ret;
}

//
// Code from tc956x_pcie_logstat.c in vendor driver
//

/* ===================================
 * Global Variables
 * ===================================
 */
/*
 * Array containing Different Available Ports.
 */
static uint8_t pcie_port[4][20] = {
	"Upstream Port",
	"Downstream Port1",
	"Downstream Port2",
	"Endpoint Port",
};

/* ===================================
 * Function Definition
 * ===================================
 */

/**
 * tc956x_logstat_set_state_log_enable
 *
 * \brief Function to Enable and Disable State Log.
 *
 * \details This function enable or disable State Logging based on mode passed.
 *
 * \param[in] pbase_addr - pointer to Bar4 base address.
 * \param[in] nport - log start/stop for port passed.
 * \param[in] mode - start or stop state logging.
 *
 * \return -EFAULT in case of bad address, otherwise 0
 */
static int tc956x_logstat_set_state_log_enable(void __iomem *pbase_addr, enum ports nport, enum state_log_enable enable)
{
	int ret = 0;
	uint32_t port_offset; /* Port Address Register Offset */

	if (pbase_addr == NULL) {
		ret = -EFAULT;
		pr_info("%s : Invalid Arguments\n", __func__);
	}

	if (ret == 0) {
		port_offset = nport * STATE_LOG_REG_OFFSET;

		if (enable == STATE_LOG_ENABLE) {
			/* Stop State Log */
			writel(STATE_LOG_DISABLE, pbase_addr + TC956X_CONF_REG_NPCIEUSPLOGCTRL + port_offset);
			/* Start State Log */
			writel(STATE_LOG_ENABLE, pbase_addr + TC956X_CONF_REG_NPCIEUSPLOGCTRL + port_offset);
			/* Verify Sate Log Enable */
			if (readl(pbase_addr + TC956X_CONF_REG_NPCIEUSPLOGCTRL + port_offset) == STATE_LOG_ENABLE)
				pr_debug("%s : Enabling State Logging for port %s\n", __func__, pcie_port[nport]);
		} else {
			/* Stop State Log */
			writel(STATE_LOG_DISABLE, pbase_addr + TC956X_CONF_REG_NPCIEUSPLOGCTRL + port_offset);
		}
		/* pr_debug("WR: Addr= 0x%08X, Val= 0x%08X\n", TC956X_CONF_REG_NPCIEUSPLOGCTRL + port_offset, enable); */
	}

	return ret;
}

//
// Code from tc956x_pci.c in vendor driver
//

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
#ifdef CONFIG_PCI_IOV
static int tc956x_no_of_vf;
#endif

#define DEF_FORCE_CONFIG_SPEED	3		/* 1Gbps */

unsigned int macX_force_speed_mode[(TC956X_TOT_CASCADE_DEV*2) + 1] = {
	DISABLE, DISABLE,
	DISABLE, DISABLE,
	DISABLE, DISABLE,
	DISABLE, DISABLE,
	DISABLE, DISABLE,
	DISABLE, DISABLE,
	DISABLE, DISABLE,
	DISABLE /* Not in use: This index value to be used when user passed BDF is not matched with probed device's bdf */
};

unsigned int macX_force_config_speed[(TC956X_TOT_CASCADE_DEV*2) + 1] = {
	DEF_FORCE_CONFIG_SPEED, DEF_FORCE_CONFIG_SPEED,
	DEF_FORCE_CONFIG_SPEED, DEF_FORCE_CONFIG_SPEED,
	DEF_FORCE_CONFIG_SPEED, DEF_FORCE_CONFIG_SPEED,
	DEF_FORCE_CONFIG_SPEED, DEF_FORCE_CONFIG_SPEED,
	DEF_FORCE_CONFIG_SPEED, DEF_FORCE_CONFIG_SPEED,
	DEF_FORCE_CONFIG_SPEED, DEF_FORCE_CONFIG_SPEED,
	DEF_FORCE_CONFIG_SPEED, DEF_FORCE_CONFIG_SPEED,
	DEF_FORCE_CONFIG_SPEED /* Not in use: This index value to be used when user passed BDF is not matched with probed device's bdf */
};


/* RFA RFD values initialized for CPE configuration and PF/VF configuration */
#if defined(TC956X_CPE_CONFIG)
#define RX_QUEUE0_RFD  24
#define RX_QUEUE0_RFA  24
#define RX_QUEUE1_RFD  24
#define RX_QUEUE1_RFA  24
#else
#define RX_QUEUE0_RFD  0xe
#define RX_QUEUE0_RFA  0x3
#define RX_QUEUE1_RFD  5
#define RX_QUEUE1_RFA  5
#endif

/* Set initial values for Array Module parameters; Need to increase this when total cascade is increased */
unsigned int tc956x_eth_ports_bdf[TC956X_TOT_CASCADE_DEV*2] = {
	0x0000, 0x0000, /* Change this to 0xFFFF, if other module parameters to be taken from array instead of user passed */
	0x0000, 0x0000,
	0x0000, 0x0000,
	0x0000, 0x0000,
	0x0000, 0x0000,
	0x0000, 0x0000,
	0x0000, 0x0000,
};
unsigned int macX_interface[(TC956X_TOT_CASCADE_DEV*2) + 1] = {
	0xFF, 0xFF,
	0xFF, 0xFF,
	0xFF, 0xFF,
	0xFF, 0xFF,
	0xFF, 0xFF,
	0xFF, 0xFF,
	0xFF, 0xFF,
	0xFF /* Not in use: This index value to be used when user passed BDF is not matched with probed device's bdf */
};
unsigned int portX_mdc[(TC956X_TOT_CASCADE_DEV*2) + 1] = {
	0xFF, 0xFF,
	0xFF, 0xFF,
	0xFF, 0xFF,
	0xFF, 0xFF,
	0xFF, 0xFF,
	0xFF, 0xFF,
	0xFF, 0xFF,
	0xFF /* Not in use: This index value to be used when user passed BDF is not matched with probed device's bdf */
};

unsigned int portX_c45_state[(TC956X_TOT_CASCADE_DEV*2) + 1] = {
	0xFF, 0xFF,
	0xFF, 0xFF,
	0xFF, 0xFF,
	0xFF, 0xFF,
	0xFF, 0xFF,
	0xFF, 0xFF,
	0xFF, 0xFF,
	0xFF /* Not in use: This index value to be used when user passed BDF is not matched with probed device's bdf */
};

unsigned int portX_phyaddr[(TC956X_TOT_CASCADE_DEV*2) + 1];
unsigned int macX_link_down_macrst[(TC956X_TOT_CASCADE_DEV*2) + 1] = {
	0xFF, 0xFF,
	0xFF, 0xFF,
	0xFF, 0xFF,
	0xFF, 0xFF,
	0xFF, 0xFF,
	0xFF, 0xFF,
	0xFF, 0xFF,
	0xFF /* Not in use: This index value to be used when user passed BDF is not matched with probed device's bdf */
};
unsigned int macX_no_mdio_no_phy[(TC956X_TOT_CASCADE_DEV*2) + 1];

unsigned int macX_rxq0_size[(TC956X_TOT_CASCADE_DEV*2) + 1] = {
	RX_QUEUE0_SIZE, RX_QUEUE0_SIZE,
	RX_QUEUE0_SIZE, RX_QUEUE0_SIZE,
	RX_QUEUE0_SIZE, RX_QUEUE0_SIZE,
	RX_QUEUE0_SIZE, RX_QUEUE0_SIZE,
	RX_QUEUE0_SIZE, RX_QUEUE0_SIZE,
	RX_QUEUE0_SIZE, RX_QUEUE0_SIZE,
	RX_QUEUE0_SIZE, RX_QUEUE0_SIZE,
	RX_QUEUE0_SIZE /* Not in use: This index value to be used when user passed BDF is not matched with probed device's bdf */
};

unsigned int macX_rxq1_size[(TC956X_TOT_CASCADE_DEV*2) + 1] = {
	RX_QUEUE1_SIZE, RX_QUEUE1_SIZE,
	RX_QUEUE1_SIZE, RX_QUEUE1_SIZE,
	RX_QUEUE1_SIZE, RX_QUEUE1_SIZE,
	RX_QUEUE1_SIZE, RX_QUEUE1_SIZE,
	RX_QUEUE1_SIZE, RX_QUEUE1_SIZE,
	RX_QUEUE1_SIZE, RX_QUEUE1_SIZE,
	RX_QUEUE1_SIZE, RX_QUEUE1_SIZE,
	RX_QUEUE1_SIZE /* Not in use: This index value to be used when user passed BDF is not matched with probed device's bdf */
};

unsigned int macX_txq0_size[(TC956X_TOT_CASCADE_DEV*2) + 1] = {
	TX_QUEUE0_SIZE, TX_QUEUE0_SIZE,
	TX_QUEUE0_SIZE, TX_QUEUE0_SIZE,
	TX_QUEUE0_SIZE, TX_QUEUE0_SIZE,
	TX_QUEUE0_SIZE, TX_QUEUE0_SIZE,
	TX_QUEUE0_SIZE, TX_QUEUE0_SIZE,
	TX_QUEUE0_SIZE, TX_QUEUE0_SIZE,
	TX_QUEUE0_SIZE, TX_QUEUE0_SIZE,
	TX_QUEUE0_SIZE /* Not in use: This index value to be used when user passed BDF is not matched with probed device's bdf */
};

unsigned int macX_txq1_size[(TC956X_TOT_CASCADE_DEV*2) + 1] = {
	TX_QUEUE1_SIZE, TX_QUEUE1_SIZE,
	TX_QUEUE1_SIZE, TX_QUEUE1_SIZE,
	TX_QUEUE1_SIZE, TX_QUEUE1_SIZE,
	TX_QUEUE1_SIZE, TX_QUEUE1_SIZE,
	TX_QUEUE1_SIZE, TX_QUEUE1_SIZE,
	TX_QUEUE1_SIZE, TX_QUEUE1_SIZE,
	TX_QUEUE1_SIZE, TX_QUEUE1_SIZE,
	TX_QUEUE1_SIZE/* Not in use: This index value to be used when user passed BDF is not matched with probed device's bdf */
};

unsigned int macX_rxq0_rfd[(TC956X_TOT_CASCADE_DEV*2) + 1] = {
	RX_QUEUE0_RFD, RX_QUEUE0_RFD,
	RX_QUEUE0_RFD, RX_QUEUE0_RFD,
	RX_QUEUE0_RFD, RX_QUEUE0_RFD,
	RX_QUEUE0_RFD, RX_QUEUE0_RFD,
	RX_QUEUE0_RFD, RX_QUEUE0_RFD,
	RX_QUEUE0_RFD, RX_QUEUE0_RFD,
	RX_QUEUE0_RFD, RX_QUEUE0_RFD,
	RX_QUEUE0_RFD /* Not in use: This index value to be used when user passed BDF is not matched with probed device's bdf */
};

unsigned int macX_rxq0_rfa[(TC956X_TOT_CASCADE_DEV*2) + 1] = {
	RX_QUEUE0_RFA, RX_QUEUE0_RFA,
	RX_QUEUE0_RFA, RX_QUEUE0_RFA,
	RX_QUEUE0_RFA, RX_QUEUE0_RFA,
	RX_QUEUE0_RFA, RX_QUEUE0_RFA,
	RX_QUEUE0_RFA, RX_QUEUE0_RFA,
	RX_QUEUE0_RFA, RX_QUEUE0_RFA,
	RX_QUEUE0_RFA, RX_QUEUE0_RFA,
	RX_QUEUE0_RFA /* Not in use: This index value to be used when user passed BDF is not matched with probed device's bdf */
};

unsigned int macX_rxq1_rfd[(TC956X_TOT_CASCADE_DEV*2) + 1] = {
	RX_QUEUE1_RFD, RX_QUEUE1_RFD,
	RX_QUEUE1_RFD, RX_QUEUE1_RFD,
	RX_QUEUE1_RFD, RX_QUEUE1_RFD,
	RX_QUEUE1_RFD, RX_QUEUE1_RFD,
	RX_QUEUE1_RFD, RX_QUEUE1_RFD,
	RX_QUEUE1_RFD, RX_QUEUE1_RFD,
	RX_QUEUE1_RFD, RX_QUEUE1_RFD,
	RX_QUEUE1_RFD /* Not in use: This index value to be used when user passed BDF is not matched with probed device's bdf */
};

unsigned int macX_rxq1_rfa[(TC956X_TOT_CASCADE_DEV*2) + 1] = {
	RX_QUEUE1_RFA, RX_QUEUE1_RFA,
	RX_QUEUE1_RFA, RX_QUEUE1_RFA,
	RX_QUEUE1_RFA, RX_QUEUE1_RFA,
	RX_QUEUE1_RFA, RX_QUEUE1_RFA,
	RX_QUEUE1_RFA, RX_QUEUE1_RFA,
	RX_QUEUE1_RFA, RX_QUEUE1_RFA,
	RX_QUEUE1_RFA, RX_QUEUE1_RFA,
	RX_QUEUE1_RFA /* Not in use: This index value to be used when user passed BDF is not matched with probed device's bdf */
};

unsigned int macX_eee_enable[(TC956X_TOT_CASCADE_DEV*2) + 1] = {
	DISABLE, DISABLE,
	DISABLE, DISABLE,
	DISABLE, DISABLE,
	DISABLE, DISABLE,
	DISABLE, DISABLE,
	DISABLE, DISABLE,
	DISABLE, DISABLE,
	DISABLE /* Not in use: This index value to be used when user passed BDF is not matched with probed device's bdf */
};

unsigned int macX_lpi_timer[(TC956X_TOT_CASCADE_DEV*2) + 1] = {
	TC956XMAC_LPIET_600US, TC956XMAC_LPIET_600US,
	TC956XMAC_LPIET_600US, TC956XMAC_LPIET_600US,
	TC956XMAC_LPIET_600US, TC956XMAC_LPIET_600US,
	TC956XMAC_LPIET_600US, TC956XMAC_LPIET_600US,
	TC956XMAC_LPIET_600US, TC956XMAC_LPIET_600US,
	TC956XMAC_LPIET_600US, TC956XMAC_LPIET_600US,
	TC956XMAC_LPIET_600US, TC956XMAC_LPIET_600US,
	TC956XMAC_LPIET_600US /* Not in use: This index value to be used when user passed BDF is not matched with probed device's bdf */
};

unsigned int macX_filter_phy_pause[(TC956X_TOT_CASCADE_DEV*2) + 1] = {
	DISABLE, DISABLE,
	DISABLE, DISABLE,
	DISABLE, DISABLE,
	DISABLE, DISABLE,
	DISABLE, DISABLE,
	DISABLE, DISABLE,
	DISABLE, DISABLE,
	DISABLE /* Not in use: This index value to be used when user passed BDF is not matched with probed device's bdf */
};


unsigned int macX_en_lp_pause_frame_cnt[(TC956X_TOT_CASCADE_DEV*2) + 1] = {
	DISABLE, DISABLE,
	DISABLE, DISABLE,
	DISABLE, DISABLE,
	DISABLE, DISABLE,
	DISABLE, DISABLE,
	DISABLE, DISABLE,
	DISABLE, DISABLE,
	DISABLE /* Not in use: This index value to be used when user passed BDF is not matched with probed device's bdf */
};

unsigned int macX_power_save_at_link_down[(TC956X_TOT_CASCADE_DEV*2) + 1] = {
	DISABLE, DISABLE,
	DISABLE, DISABLE,
	DISABLE, DISABLE,
	DISABLE, DISABLE,
	DISABLE, DISABLE,
	DISABLE, DISABLE,
	DISABLE, DISABLE,
	DISABLE /* Not in use: This index value to be used when user passed BDF is not matched with probed device's bdf */
};

static int mac0_tx_pbl = 16;
static int mac0_rx_pbl = 16;
static int mac1_tx_pbl = 16;
static int mac1_rx_pbl = 16;

#ifdef TC956X_PCIE_LINK_STATE_LATENCY_CTRL

unsigned int epX_l0s_delay[(TC956X_TOT_CASCADE_DEV*2) + 1] = {
	EP_L0s_ENTRY_DELAY, EP_L0s_ENTRY_DELAY,
	EP_L0s_ENTRY_DELAY, EP_L0s_ENTRY_DELAY,
	EP_L0s_ENTRY_DELAY, EP_L0s_ENTRY_DELAY,
	EP_L0s_ENTRY_DELAY, EP_L0s_ENTRY_DELAY,
	EP_L0s_ENTRY_DELAY, EP_L0s_ENTRY_DELAY,
	EP_L0s_ENTRY_DELAY, EP_L0s_ENTRY_DELAY,
	EP_L0s_ENTRY_DELAY, EP_L0s_ENTRY_DELAY,
	EP_L0s_ENTRY_DELAY /* Not in use: This index value to be used when user passed BDF is not matched with probed device's bdf */
};

unsigned int epX_l1_delay[(TC956X_TOT_CASCADE_DEV*2) + 1] = {
	EP_L1_ENTRY_DELAY, EP_L1_ENTRY_DELAY,
	EP_L1_ENTRY_DELAY, EP_L1_ENTRY_DELAY,
	EP_L1_ENTRY_DELAY, EP_L1_ENTRY_DELAY,
	EP_L1_ENTRY_DELAY, EP_L1_ENTRY_DELAY,
	EP_L1_ENTRY_DELAY, EP_L1_ENTRY_DELAY,
	EP_L1_ENTRY_DELAY, EP_L1_ENTRY_DELAY,
	EP_L1_ENTRY_DELAY, EP_L1_ENTRY_DELAY,
	EP_L1_ENTRY_DELAY /* Not in use: This index value to be used when user passed BDF is not matched with probed device's bdf */
};

unsigned int uspX_l0s_delay[(TC956X_TOT_CASCADE_DEV*2) + 1] = {
	USP_L0s_ENTRY_DELAY, USP_L0s_ENTRY_DELAY,
	USP_L0s_ENTRY_DELAY, USP_L0s_ENTRY_DELAY,
	USP_L0s_ENTRY_DELAY, USP_L0s_ENTRY_DELAY,
	USP_L0s_ENTRY_DELAY, USP_L0s_ENTRY_DELAY,
	USP_L0s_ENTRY_DELAY, USP_L0s_ENTRY_DELAY,
	USP_L0s_ENTRY_DELAY, USP_L0s_ENTRY_DELAY,
	USP_L0s_ENTRY_DELAY, USP_L0s_ENTRY_DELAY,
	USP_L0s_ENTRY_DELAY /* Not in use: This index value to be used when user passed BDF is not matched with probed device's bdf */
};

unsigned int uspX_l1_delay[(TC956X_TOT_CASCADE_DEV*2) + 1] = {
	USP_L1_ENTRY_DELAY, USP_L1_ENTRY_DELAY,
	USP_L1_ENTRY_DELAY, USP_L1_ENTRY_DELAY,
	USP_L1_ENTRY_DELAY, USP_L1_ENTRY_DELAY,
	USP_L1_ENTRY_DELAY, USP_L1_ENTRY_DELAY,
	USP_L1_ENTRY_DELAY, USP_L1_ENTRY_DELAY,
	USP_L1_ENTRY_DELAY, USP_L1_ENTRY_DELAY,
	USP_L1_ENTRY_DELAY, USP_L1_ENTRY_DELAY,
	USP_L1_ENTRY_DELAY /* Not in use: This index value to be used when user passed BDF is not matched with probed device's bdf */
};

#endif

static unsigned int mac0_axi_wr_osr_lmt = 31;
static unsigned int mac0_axi_rd_osr_lmt = 31;
static unsigned int mac1_axi_wr_osr_lmt = 31;
static unsigned int mac1_axi_rd_osr_lmt = 31;

static unsigned int mac0_axi_blen;
static unsigned int mac1_axi_blen;
static const struct tc956x_version tc956x_drv_version = {0, 6, 0, 0, 0, 0};
int tc956xmac_pm_usage_counter; /* Device Usage Counter */
int tc956x_dsp_count;
#ifdef CONFIG_TC956X_MAGIC_PACKET_WOL_GPIO
static void tc956x_wol_gpio_trigger(void __iomem *reg_base_addr, bool mode);
#endif
/*
 * This struct is used to associate PCI Function of MAC controller on a board,
 * discovered via DMI, with the address of PHY connected to the MAC. The
 * negative value of the address means that MAC controller is not connected
 * with PHY.
 */
struct tc956xmac_pci_func_data {
	unsigned int func;
	int phy_addr;
};

struct tc956xmac_pci_dmi_data {
	const struct tc956xmac_pci_func_data *func;
	size_t nfuncs;
};

struct tc956xmac_pci_info {
	int (*setup)(struct pci_dev *pdev, struct plat_stmmacenet_data *plat);
};

/* By default, Bypass FRP routing */
/* User to configure routing for FRP usecase */

static struct tc956xmac_rx_parser_entry snps_rxp_entries[] = {
	{
		.match_data = 0x00000000,
		.match_en = 0x00000000,
		.af = 1,
		.rf = 1,
		.im = 0,
		.nc = 0,
		.res1 = 0,
		.frame_offset = 0,
		.res2 = 0,
		.ok_index = 0,
		.res3 = 0,
		.dma_ch_no = 0x0,
		.res4 = 0, /* FRP Bypass */
	},
};

static struct tc956xmac_rx_parser_entry
	snps_rxp_entries_filter_phy_pause_frames[] = {
		/* 0th entry */ {
			.match_data = 0x00000888,
			.match_en = 0x0000FFFF,
			.af = 0,
			.rf = 0,
			.im = 0,
			.nc = 1,
			.res1 = 0,
			.frame_offset = 3,
			.res2 = 0,
			.ok_index = 3,
			.res3 = 0,
			.dma_ch_no = 1,
			.res4 = 0,
		},
		/* Checking SA Address 00:01:02:03:04:05 AQR PHYs SA Address as Ether type Match*/
		/* 1st entry */
		{
			.match_data = 0x01000000,
			.match_en = 0xFFFF0000,
			.af = 0,
			.rf = 0,
			.im = 0,
			.nc = 1,
			.res1 = 0,
			.frame_offset = 1,
			.res2 = 0,
			.ok_index = 3,
			.res3 = 0,
			.dma_ch_no = 1,
			.res4 = 0,
		},
		/* 2nd entry */
		{
			.match_data = 0x05040302,
			.match_en = 0xFFFFFFFF,
			.af = 0,
			.rf = 1,
			.im = 0,
			.nc = 0,
			.res1 = 0,
			.frame_offset = 2,
			.res2 = 0,
			.ok_index = 0,
			.res3 = 0,
			.dma_ch_no = 1,
			.res4 = 0,
		},
		/* Route all other packets to DMA channel-0 */
		/* 3rd entry */
		{
			.match_data = 0x00000000,
			.match_en = 0x00000000,
			.af = 1,
			.rf = 0,
			.im = 0,
			.nc = 0,
			.res1 = 0,
			.frame_offset = 0,
			.res2 = 0,
			.ok_index = 0,
			.res3 = 0,
			.dma_ch_no = 1,
			.res4 = 0,
		},
		/* 4th entry */
		{
			.match_data = 0x00000000,
			.match_en = 0x00000000,
			.af = 1,
			.rf = 0,
			.im = 0,
			.nc = 0,
			.res1 = 0,
			.frame_offset = 0,
			.res2 = 0,
			.ok_index = 0,
			.res3 = 0,
			.dma_ch_no = 1,
			.res4 = 0,
		},
	};

/*!
 * \brief API to save and restore clock and reset during suspend-resume.
 *
 * \details This fucntion saves the EMAC clock and reset bits before
 * suspend. And restores the same settings after resume.
 *
 * \param[in] priv - pointer to device private structure.
 * \param[in] state - identify SUSPEND and RESUME operation.
 *
 * \return None
 */
static void tc956xmac_pm_set_power(struct stmmac_priv *priv, enum TC956X_PORT_PM_STATE state)
{
	void *nrst_reg = NULL, *nclk_reg = NULL, *commonclk_reg = NULL;
	u32 nrst_val = 0, nclk_val = 0, commonclk_val = 0;

	pr_debug("-->%s : Port %d interface %s", __func__, priv->port_num, priv->dev->name);
	/* Select register address by port */
	if (priv->port_num == 0) {
		nrst_reg = priv->tc956x_SFR_pci_base_addr + NRSTCTRL0_OFFSET;
		nclk_reg = priv->tc956x_SFR_pci_base_addr + NCLKCTRL0_OFFSET;
	} else {
		nrst_reg = priv->tc956x_SFR_pci_base_addr + NRSTCTRL1_OFFSET;
		nclk_reg = priv->tc956x_SFR_pci_base_addr + NCLKCTRL1_OFFSET;
	}

	if (state == SUSPEND) {
		pr_debug("%s : Port %d interface %s Set Power for Suspend", __func__, priv->port_num, priv->dev->name);
		/* Modify register for reset, clock and MSI_OUTEN */
		nrst_val = readl(nrst_reg);
		nclk_val = readl(nclk_reg);
		pr_debug("%s : Port %d interface %s Rd RST Reg:%x, CLK Reg:%x", __func__, priv->port_num, priv->dev->name,
			nrst_val, nclk_val);
		/* Save values before Asserting reset and Clock Disable */
		priv->pm_saved_emac_rst = nrst_val & NRSTCTRL_EMAC_MASK;
		priv->pm_saved_emac_clk = nclk_val & NCLKCTRL_EMAC_MASK;
		nrst_val = nrst_val | NRSTCTRL_EMAC_MASK;
		nclk_val = nclk_val & ~NCLKCTRL_EMAC_MASK;
		writel(nrst_val, nrst_reg);
		writel(nclk_val, nclk_reg);
		if (tx956x_pci_shrd_mem[priv->pci_bd].pci_dev_active_cnt == TC956X_ALL_MAC_PORT_SUSPENDED) {
			commonclk_reg = priv->tc956x_SFR_pci_base_addr + NCLKCTRL0_OFFSET;
			commonclk_val = readl(commonclk_reg);
			pr_debug("%s : Port %d interface %s Common CLK Rd Reg:%x", __func__, priv->port_num, priv->dev->name,
				commonclk_val);
			/* Clear Common Clocks only when both port suspends */
			commonclk_val = commonclk_val & ~NCLKCTRL0_COMMON_EMAC_MASK;
			writel(commonclk_val, commonclk_reg);
			pr_debug("%s : Port %d interface %s Common CLK Wr Reg:%x", __func__, priv->port_num, priv->dev->name,
				commonclk_val);
		}
	} else if (state == RESUME) {
		pr_debug("%s : Port %d interface %s Set Power for Resume", __func__, priv->port_num, priv->dev->name);
		if (tx956x_pci_shrd_mem[priv->pci_bd].pci_dev_active_cnt == TC956X_ALL_MAC_PORT_SUSPENDED) {
			commonclk_reg = priv->tc956x_SFR_pci_base_addr + NCLKCTRL0_OFFSET;
			commonclk_val = readl(commonclk_reg);
			pr_debug("%s : Port %d interface %s Common CLK Rd Reg:%x", __func__, priv->port_num, priv->dev->name,
				commonclk_val);
			/* Clear Common Clocks only when both port suspends */
			commonclk_val = commonclk_val | NCLKCTRL0_COMMON_EMAC_MASK;
			writel(commonclk_val, commonclk_reg);
			pr_debug("%s : Port %d interface %s Common CLK WR Reg:%x", __func__, priv->port_num, priv->dev->name,
				commonclk_val);
		}
		nrst_val = readl(nrst_reg);
		nclk_val = readl(nclk_reg);
		pr_debug("%s : Port %d interface %s Rd RST Reg:%x, CLK Reg:%x", __func__, priv->port_num, priv->dev->name,
			nrst_val, nclk_val);
		/* Restore values same as before suspend */
		nrst_val = (nrst_val & ~NRSTCTRL_EMAC_MASK) | priv->pm_saved_emac_rst;
		nclk_val = nclk_val | priv->pm_saved_emac_clk; /* Restore Clock */
		writel(nclk_val, nclk_reg);
		writel(nrst_val, nrst_reg);
	}
	pr_debug("%s : Port %d interface %s priv->pm_saved_emac_rst %x priv->pm_saved_emac_clk %x", __func__,
		priv->port_num, priv->dev->name, priv->pm_saved_emac_rst, priv->pm_saved_emac_clk);
	pr_debug("%s : Port %d %s Wr RST Reg:%x, CLK Reg:%x", __func__, priv->port_num, priv->dev->name,
		readl(nrst_reg), readl(nclk_reg));
	pr_debug("<--%s : Port %d interface %s", __func__, priv->port_num, priv->dev->name);
}

static void xgmac_default_data(struct plat_stmmacenet_data *plat)
{
	//plat->has_xgmac = 1;
	plat->core_type = DWMAC_CORE_XGMAC;
	plat->force_sf_dma_mode = 1;
	//plat->tso_en = 1;
	plat->flags |= STMMAC_FLAG_TSO_EN;
	plat->cphy_read = NULL;
	plat->cphy_write = NULL;
	plat->mdio_bus_data->phy_mask = 0;

	switch (plat->mdc_clk) {
	case TC956XMAC_XGMAC_MDC_CSR_4:
		plat->clk_csr = 0x0;
		plat->clk_crs = 1;
		break;
	case TC956XMAC_XGMAC_MDC_CSR_6:
		plat->clk_csr = 0x1;
		plat->clk_crs = 1;
		break;
	case TC956XMAC_XGMAC_MDC_CSR_8:
		plat->clk_csr = 0x2;
		plat->clk_crs = 1;
		break;
	case TC956XMAC_XGMAC_MDC_CSR_10:
		plat->clk_csr = 0x3;
		plat->clk_crs = 1;
		break;
	case TC956XMAC_XGMAC_MDC_CSR_12:
		plat->clk_csr = 0x4;
		plat->clk_crs = 1;
		break;
	case TC956XMAC_XGMAC_MDC_CSR_14:
		plat->clk_csr = 0x5;
		plat->clk_crs = 1;
		break;
	case TC956XMAC_XGMAC_MDC_CSR_16:
		plat->clk_csr = 0x6;
		plat->clk_crs = 1;
		break;
	case TC956XMAC_XGMAC_MDC_CSR_18:
		plat->clk_csr = 0x7;
		plat->clk_crs = 1;
		break;
	case TC956XMAC_XGMAC_MDC_CSR_62:
		plat->clk_csr = 0x0;
		plat->clk_crs = 0;
		break;
	case TC956XMAC_XGMAC_MDC_CSR_102:
		plat->clk_csr = 0x1;
		plat->clk_crs = 0;
		break;
	case TC956XMAC_XGMAC_MDC_CSR_122:
		plat->clk_csr = 0x2;
		plat->clk_crs = 0;
		break;
	case TC956XMAC_XGMAC_MDC_CSR_142:
		plat->clk_csr = 0x3;
		plat->clk_crs = 0;
		break;
	case TC956XMAC_XGMAC_MDC_CSR_162:
		plat->clk_csr = 0x4;
		plat->clk_crs = 0;
		break;
	case TC956XMAC_XGMAC_MDC_CSR_202:
		plat->clk_csr = 0x5;
		plat->clk_crs = 0;
		break;

	};

	//plat->has_gmac = 0;
	//plat->has_gmac4 = 0;
	plat->force_thresh_dma_mode  = 0;
	plat->mdio_bus_data->needs_reset = false;
	if ((plat->port_interface == ENABLE_USXGMII_INTERFACE) ||
	   (plat->port_interface == ENABLE_XFI_INTERFACE) || (plat->port_interface == ENABLE_USXGMII_10G_INTERFACE))
		plat->mac_port_sel_speed = 10000;

	if (plat->port_interface == ENABLE_USXGMII_5G_INTERFACE)
		plat->mac_port_sel_speed = 5000;

	if (plat->port_interface == ENABLE_USXGMII_2_5G_INTERFACE)
		plat->mac_port_sel_speed = 2500;

	if ((plat->port_interface == ENABLE_RGMII_INTERFACE) ||
		(plat->port_interface == ENABLE_RGMII_ID_INTERFACE))
		plat->mac_port_sel_speed = 1000;

	if ((plat->port_interface == ENABLE_SGMII_INTERFACE) ||
		(plat->port_interface == ENABLE_2500BASE_X_INTERFACE)) {
		plat->mac_port_sel_speed = 2500;
	}

	plat->riwt_off = 0;
	plat->rss_en = 0;

	/*For RXP config */
#ifdef TC956X_FRP_ENABLE
	plat->rxp_cfg.enable = true;
#else
	plat->rxp_cfg.enable = false;
#endif

	plat->rxp_cfg.nve = ARRAY_SIZE(snps_rxp_entries);
	plat->rxp_cfg.npe = ARRAY_SIZE(snps_rxp_entries);
	memcpy(plat->rxp_cfg.entries, snps_rxp_entries,
			ARRAY_SIZE(snps_rxp_entries) *
			sizeof(struct tc956xmac_rx_parser_entry));
	/* Over writing the Default FRP table with FRP Table for Filtering PHY pause frames */
	if (plat->filter_phy_pause == ENABLE) {
		plat->rxp_cfg.nve = ARRAY_SIZE(snps_rxp_entries_filter_phy_pause_frames);
		plat->rxp_cfg.npe = ARRAY_SIZE(snps_rxp_entries_filter_phy_pause_frames);
		memcpy(plat->rxp_cfg.entries, snps_rxp_entries_filter_phy_pause_frames,
				ARRAY_SIZE(snps_rxp_entries_filter_phy_pause_frames) *
				sizeof(struct tc956xmac_rx_parser_entry));
	}
}

static int tc956xmac_xgmac3_default_data(struct pci_dev *pdev,
				struct plat_stmmacenet_data *plat)
{
	unsigned int queue0_rfd = 0, queue1_rfd = 0, queue0_rfa = 0, queue1_rfa = 0, temp_var = 0;
	unsigned int rxqueue0_size = 0, rxqueue1_size = 0, txqueue0_size = 0, txqueue1_size = 0;
	unsigned int forced_speed = 3; /* default 1Gbps */
	unsigned int axi_blen = 0; /* default */
	u32 axi_blen_array[AXI_BLEN];

	/* Set common default data first */
	xgmac_default_data(plat);

	plat->gate_mask = 1; /* 1: tc_to_txq gate mask enabled by default. Traffic control gate event mapped to respective queues in kernel and sent to Driver */
	plat->bus_id = 1;
	plat->phy_addr = -1;
	plat->pdev = pdev;
	plat->pse = 0;

	if ((plat->port_interface == ENABLE_USXGMII_INTERFACE) || (plat->port_interface == ENABLE_USXGMII_10G_INTERFACE)) {
		plat->phy_interface = PHY_INTERFACE_MODE_USXGMII;
		plat->max_speed = 10000;
	}

	if (plat->port_interface == ENABLE_USXGMII_5G_INTERFACE) {
		plat->phy_interface = PHY_INTERFACE_MODE_USXGMII;
		plat->max_speed = 5000;
	}

	if (plat->port_interface == ENABLE_USXGMII_2_5G_INTERFACE) {
		plat->phy_interface = PHY_INTERFACE_MODE_USXGMII;
		plat->max_speed = 2500;
	}

	if (plat->port_interface == ENABLE_XFI_INTERFACE) {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 6, 0)
		plat->phy_interface = PHY_INTERFACE_MODE_10GBASER;
#else
		plat->phy_interface = PHY_INTERFACE_MODE_10GKR;
#endif
		plat->max_speed = 10000;
	}
	if (plat->port_interface == ENABLE_RGMII_INTERFACE) {
		plat->phy_interface = PHY_INTERFACE_MODE_RGMII;
		plat->max_speed = 1000;
	}
	if (plat->port_interface == ENABLE_RGMII_ID_INTERFACE) {
		plat->phy_interface = PHY_INTERFACE_MODE_RGMII_ID;
		plat->max_speed = 1000;
	}
	if ((plat->port_interface == ENABLE_SGMII_INTERFACE) ||
		(plat->port_interface == ENABLE_2500BASE_X_INTERFACE)) {
		plat->phy_interface = PHY_INTERFACE_MODE_SGMII;
		plat->max_speed = 2500;
	}

	/* Configure forced speed based on the module param.
	 * This is applicable only for fixed phy mode.
	 */
	if (plat->port_num == RM_PF0_ID)
		forced_speed = plat->force_config_speed;

	if (plat->port_num == RM_PF1_ID)
		forced_speed = plat->force_config_speed;

	switch (forced_speed) {
	case 0:
		plat->forced_speed = SPEED_10000;
		break;
	case 1:
		plat->forced_speed = SPEED_5000;
		break;
	case 2:
		plat->forced_speed = SPEED_2500;
		break;
	case 3:
		plat->forced_speed = SPEED_1000;
		break;
	case 4:
		plat->forced_speed = SPEED_100;
		break;
	case 5:
		plat->forced_speed = SPEED_10;
		break;
	default:
		plat->forced_speed = SPEED_1000;
		break;
	}

	plat->clk_ptp_rate = TC956X_TARGET_PTP_CLK;

	/* Set default value for multicast hash bins */
	plat->multicast_filter_bins = HASH_TABLE_SIZE;
	/* Set default value for unicast filter entries */
	plat->unicast_filter_entries = MAX_MAC_ADDR_FILTERS;
#if defined(TC956X_CPE_CONFIG)
	plat->maxmtu = MAX_SUPPORTED_MTU/*XGMAC_JUMBO_LEN*/;
#else
	plat->maxmtu = XGMAC_JUMBO_LEN;
#endif

	/* Set default number of RX and TX queues to use */
	plat->tx_queues_to_use = MAX_TX_QUEUES_TO_USE;
	plat->rx_queues_to_use = MAX_RX_QUEUES_TO_USE;

	/* MTL Configuration */

	/*Rx queue configuration for VFs are done in PF driver, so skip setting it here*/
	/* Static Mapping */
	/* Note : Best Effort, Broadcast/Multicast packet routing based
	 * on DA filter Channel Mapping
	 */
	/* Unicast/Untagged Packets : Consider Jumbo packets */
	plat->rx_queues_cfg[0].chan = LEG_UNTAGGED_PACKET;
	/* VLAN Tagged Legacy packets */
	plat->rx_queues_cfg[1].chan = LEG_TAGGED_PACKET;
	/* Untagged gPTP packets  */
	plat->rx_queues_cfg[2].chan = UNTAGGED_GPTP_PACKET;

	/* Tagged/Untagged  AV control pkts */
	plat->rx_queues_cfg[3].chan = UNTAGGED_AVCTRL_PACKET;

	/* AVB Class B */
	plat->rx_queues_cfg[4].chan = AVB_CLASS_B_PACKET;
	/* AVB Class A */
	plat->rx_queues_cfg[5].chan = AVB_CLASS_A_PACKET;
	/* CDT */
	plat->rx_queues_cfg[6].chan = TSN_CLASS_CDT_PACKET;
	/* Broadcast/Multicast packets to support pkt duplication it should be highest queue */
	plat->rx_queues_cfg[7].chan = BC_MC_PACKET;

	/* Rx Queue Packet routing */
	plat->rx_queues_cfg[0].pkt_route = RX_QUEUE0_PKT_ROUTE;
	plat->rx_queues_cfg[1].pkt_route = RX_QUEUE1_PKT_ROUTE;
	plat->rx_queues_cfg[2].pkt_route = RX_QUEUE2_PKT_ROUTE;
	plat->rx_queues_cfg[3].pkt_route = RX_QUEUE3_PKT_ROUTE;
	plat->rx_queues_cfg[4].pkt_route = RX_QUEUE4_PKT_ROUTE;
	plat->rx_queues_cfg[5].pkt_route = RX_QUEUE5_PKT_ROUTE;
	plat->rx_queues_cfg[6].pkt_route = RX_QUEUE6_PKT_ROUTE;
	plat->rx_queues_cfg[7].pkt_route = RX_QUEUE7_PKT_ROUTE;
	/* MTL Scheduler for RX and TX */

	plat->rx_sched_algorithm = MTL_RX_ALGORITHM_SP;
	plat->tx_sched_algorithm = MTL_TX_ALGORITHM_WRR;

	/* Due to the erratum in XGMAC 3.01a, WRR weights not considered in
	 * TX DMA read data arbitration. Workaround is at set all weights for Tx Queues with
	 * WRR arbitration logic to 1
	 */

	 /* Best Effort weitghts are same as its mapped to TC0 */
	plat->tx_queues_cfg[0].weight = 0x11;
	plat->tx_queues_cfg[1].weight = 0x11;
	plat->tx_queues_cfg[2].weight = 0x11;
	plat->tx_queues_cfg[3].weight = 0x11;
#if defined(TC956X_CPE_CONFIG)
	plat->tx_queues_cfg[4].weight = 0x11;
#else
	plat->tx_queues_cfg[4].weight = 0x12;
#endif
	plat->tx_queues_cfg[5].weight = 0x13;
	plat->tx_queues_cfg[6].weight = 0x14;
	plat->tx_queues_cfg[7].weight = 0x15;

	/*Rx queue configuration for VFs are done in PF driver, so skip setting it here*/
	plat->rx_queues_cfg[0].mode_to_use = RX_QUEUE0_MODE;
	plat->rx_queues_cfg[1].mode_to_use = RX_QUEUE1_MODE;
	plat->rx_queues_cfg[2].mode_to_use = RX_QUEUE2_MODE;
	plat->rx_queues_cfg[3].mode_to_use = RX_QUEUE3_MODE;
	plat->rx_queues_cfg[4].mode_to_use = RX_QUEUE4_MODE;
	plat->rx_queues_cfg[5].mode_to_use = RX_QUEUE5_MODE;
	plat->rx_queues_cfg[6].mode_to_use = RX_QUEUE6_MODE;
	plat->rx_queues_cfg[7].mode_to_use = RX_QUEUE7_MODE;

	plat->tx_queues_cfg[0].mode_to_use = TX_QUEUE0_MODE;
	plat->tx_queues_cfg[1].mode_to_use = TX_QUEUE1_MODE;
	plat->tx_queues_cfg[2].mode_to_use = TX_QUEUE2_MODE;
	plat->tx_queues_cfg[3].mode_to_use = TX_QUEUE3_MODE;
	plat->tx_queues_cfg[4].mode_to_use = TX_QUEUE4_MODE;
	plat->tx_queues_cfg[5].mode_to_use = TX_QUEUE5_MODE;
	plat->tx_queues_cfg[6].mode_to_use = TX_QUEUE6_MODE;
	plat->tx_queues_cfg[7].mode_to_use = TX_QUEUE7_MODE;

	/* CBS are per TC basis in TC956X (total TC = 5) */
	/* CBS: queue 5 -> Class B traffic (25% BW) */
	/* plat->tx_queues_cfg[3].idle_slope = plat->est_cfg.enable ? 0x8e4 : 0x800; */

	/* CBS: queue 5 -> Class B traffic (25% BW) */
	plat->tx_queues_cfg[5].idle_slope = 0x800;
	plat->tx_queues_cfg[5].send_slope = 0x1800;
	plat->tx_queues_cfg[5].high_credit = 0x320000;
	plat->tx_queues_cfg[5].low_credit = 0xff6a0000;

	/* CBS: queue 6 -> Class A traffic (25% BW) */
	/* plat->tx_queues_cfg[5].idle_slope = plat->est_cfg.enable ? 0x8e4 : 0x800; */
	plat->tx_queues_cfg[6].idle_slope = 0x800;
	plat->tx_queues_cfg[6].send_slope = 0x1800;
	plat->tx_queues_cfg[6].high_credit = 0x320000;
	plat->tx_queues_cfg[6].low_credit = 0xff6a0000;

	/* CBS: queue 7 -> Class CDT traffic (40%) BW */
	/* plat->tx_queues_cfg[4].idle_slope = plat->est_cfg.enable ? 0xe38 : 0xccc; */
	plat->tx_queues_cfg[7].idle_slope = 0xccc;
	plat->tx_queues_cfg[7].send_slope = 0x1333;
	plat->tx_queues_cfg[7].high_credit = 0x500000;
	plat->tx_queues_cfg[7].low_credit = 0xff880000;

	/* Tx TC priority */
	plat->tx_queues_cfg[0].use_prio = TX_QUEUE0_USE_PRIO;
	plat->tx_queues_cfg[1].use_prio = TX_QUEUE1_USE_PRIO;
	plat->tx_queues_cfg[2].use_prio = TX_QUEUE2_USE_PRIO;
	plat->tx_queues_cfg[3].use_prio = TX_QUEUE3_USE_PRIO;
	plat->tx_queues_cfg[4].use_prio = TX_QUEUE4_USE_PRIO;
	plat->tx_queues_cfg[5].use_prio = TX_QUEUE5_USE_PRIO;
	plat->tx_queues_cfg[6].use_prio = TX_QUEUE6_USE_PRIO;
	plat->tx_queues_cfg[7].use_prio = TX_QUEUE7_USE_PRIO;

	plat->tx_queues_cfg[0].prio = TX_QUEUE0_PRIO;
	plat->tx_queues_cfg[1].prio = TX_QUEUE1_PRIO;
	plat->tx_queues_cfg[2].prio = TX_QUEUE2_PRIO;
	plat->tx_queues_cfg[3].prio = TX_QUEUE3_PRIO;
	plat->tx_queues_cfg[4].prio = TX_QUEUE4_PRIO;
	plat->tx_queues_cfg[5].prio = TX_QUEUE5_PRIO;
	plat->tx_queues_cfg[6].prio = TX_QUEUE6_PRIO;
	plat->tx_queues_cfg[7].prio = TX_QUEUE7_PRIO;

	/* Enable/Disable TBS */
	plat->tx_queues_cfg[0].tbs_en = TX_QUEUE0_TBS;
	plat->tx_queues_cfg[1].tbs_en = TX_QUEUE1_TBS;
	plat->tx_queues_cfg[2].tbs_en = TX_QUEUE2_TBS;
	plat->tx_queues_cfg[3].tbs_en = TX_QUEUE3_TBS;
	plat->tx_queues_cfg[4].tbs_en = TX_QUEUE4_TBS;
	plat->tx_queues_cfg[5].tbs_en = TX_QUEUE5_TBS;
	plat->tx_queues_cfg[6].tbs_en = TX_QUEUE6_TBS;
	plat->tx_queues_cfg[7].tbs_en = TX_QUEUE7_TBS;

	/* Enable/Disable TSO*/
	plat->tx_queues_cfg[0].tso_en = TX_QUEUE0_TSO;
	plat->tx_queues_cfg[1].tso_en = TX_QUEUE1_TSO;
	plat->tx_queues_cfg[2].tso_en = TX_QUEUE2_TSO;
	plat->tx_queues_cfg[3].tso_en = TX_QUEUE3_TSO;
	plat->tx_queues_cfg[4].tso_en = TX_QUEUE4_TSO;
	plat->tx_queues_cfg[5].tso_en = TX_QUEUE5_TSO;
	plat->tx_queues_cfg[6].tso_en = TX_QUEUE6_TSO;
	plat->tx_queues_cfg[7].tso_en = TX_QUEUE7_TSO;

	plat->tx_queues_cfg[0].traffic_class = TX_QUEUE0_TC;
	plat->tx_queues_cfg[1].traffic_class = TX_QUEUE1_TC;
	plat->tx_queues_cfg[2].traffic_class = TX_QUEUE2_TC;
	plat->tx_queues_cfg[3].traffic_class = TX_QUEUE3_TC;
	plat->tx_queues_cfg[4].traffic_class = TX_QUEUE4_TC;
	plat->tx_queues_cfg[5].traffic_class = TX_QUEUE5_TC;
	plat->tx_queues_cfg[6].traffic_class = TX_QUEUE6_TC;
	plat->tx_queues_cfg[7].traffic_class = TX_QUEUE7_TC;


	plat->rx_queues_cfg[0].use_prio = RX_QUEUE0_USE_PRIO;
	plat->rx_queues_cfg[0].prio = RX_QUEUE0_PRIO;

	plat->rx_queues_cfg[1].use_prio = RX_QUEUE1_USE_PRIO;
	plat->rx_queues_cfg[1].prio = RX_QUEUE1_PRIO;

	plat->rx_queues_cfg[2].use_prio = RX_QUEUE2_USE_PRIO;
	plat->rx_queues_cfg[2].prio = RX_QUEUE2_PRIO;

	plat->rx_queues_cfg[3].use_prio = RX_QUEUE3_USE_PRIO;
	plat->rx_queues_cfg[3].prio = RX_QUEUE3_PRIO;

	plat->rx_queues_cfg[4].use_prio = RX_QUEUE4_USE_PRIO;
	plat->rx_queues_cfg[4].prio = RX_QUEUE4_PRIO;

	plat->rx_queues_cfg[5].use_prio = RX_QUEUE5_USE_PRIO;
	plat->rx_queues_cfg[5].prio = RX_QUEUE5_PRIO;

	plat->rx_queues_cfg[6].use_prio = RX_QUEUE6_USE_PRIO;
	plat->rx_queues_cfg[6].prio = RX_QUEUE6_PRIO;

	plat->rx_queues_cfg[7].use_prio = RX_QUEUE7_USE_PRIO;
	plat->rx_queues_cfg[7].prio = RX_QUEUE7_PRIO;

	if (plat->port_num == RM_PF0_ID)
		plat->dma_cfg->txpbl = mac0_tx_pbl;

	if (plat->port_num == RM_PF1_ID)
		plat->dma_cfg->txpbl = mac1_tx_pbl;

	if (plat->port_num == RM_PF0_ID)
		plat->dma_cfg->rxpbl = mac0_rx_pbl;

	if (plat->port_num == RM_PF1_ID)
		plat->dma_cfg->rxpbl = mac1_rx_pbl;

	plat->dma_cfg->pblx8 = true;
	/* Axi Configuration */
	plat->axi = devm_kzalloc(&pdev->dev, sizeof(*plat->axi), GFP_KERNEL);
	if (!plat->axi)
		return -ENOMEM;

#ifdef EEE_MAC_CONTROLLED_MODE
	plat->axi->axi_lpi_en = 1;
	plat->axi->axi_xit_frm = 0;
	plat->en_tx_lpi_clockgating = 1;
#endif
	if (plat->port_num == RM_PF0_ID) {
		plat->axi->axi_wr_osr_lmt = mac0_axi_wr_osr_lmt;
		plat->axi->axi_rd_osr_lmt = mac0_axi_rd_osr_lmt;
	}

	if (plat->port_num == RM_PF1_ID) {
		plat->axi->axi_wr_osr_lmt = mac1_axi_wr_osr_lmt;
		plat->axi->axi_rd_osr_lmt = mac1_axi_rd_osr_lmt;
	}


	if (plat->port_num == RM_PF0_ID)
		axi_blen = mac0_axi_blen;

	if (plat->port_num == RM_PF1_ID)
		axi_blen = mac1_axi_blen;

	plat->axi->axi_fb = true;

	axi_blen_array[0] = 4;
	axi_blen_array[1] = 0;
	axi_blen_array[2] = 0;
	axi_blen_array[3] = 0;
	axi_blen_array[4] = 0;
	axi_blen_array[5] = 0;
	axi_blen_array[6] = 0;

	switch (axi_blen) {
	case 256:
		axi_blen_array[6] = 256;
		/* Falls through. */
		fallthrough;
	case 128:
		axi_blen_array[5] = 128;
		/* Falls through. */
		fallthrough;
	case 64:
		axi_blen_array[4] = 64;
		/* Falls through. */
		fallthrough;
	case 32:
		axi_blen_array[3] = 32;
		/* Falls through. */
		fallthrough;
	case 16:
		axi_blen_array[2] = 16;
		/* Falls through. */
		fallthrough;
	case 8:
		axi_blen_array[1] = 8;
		/* Falls through. */
		fallthrough;
	case 4:
		axi_blen_array[0] = 4;
		break;

	default:
		plat->axi->axi_fb = false;
		axi_blen_array[0] = 4;
		axi_blen_array[1] = 8;
		axi_blen_array[2] = 16;
		axi_blen_array[3] = 32;
		axi_blen_array[4] = 64;
		axi_blen_array[5] = 128;
		axi_blen_array[6] = 256;
		break;
	}
	stmmac_axi_blen_to_mask(&plat->axi->axi_blen_regval, axi_blen_array,
				AXI_BLEN);

	if (!plat->est) {
		plat->est = devm_kzalloc(&pdev->dev, sizeof(*plat->est),
					 GFP_KERNEL);
		if (!plat->est)
			return -ENOMEM;
	} else {
		memset(plat->est, 0, sizeof(*plat->est));
	}



	/* Configuration of PHY operating mode 1(true): for interrupt mode, 0(false): for polling mode */
	if (plat->port_num == RM_PF0_ID) {
#ifdef TC956X_PHY_INTERRUPT_MODE_EMAC0
		plat->phy_interrupt_mode = true;
#else
		plat->phy_interrupt_mode = false;
#endif
	}

	if (plat->port_num == RM_PF1_ID) {
#ifdef TC956X_PHY_INTERRUPT_MODE_EMAC1
		plat->phy_interrupt_mode = true;
#else
		plat->phy_interrupt_mode = false;
#endif
	}

	/* Rx Queue size and flow control thresholds configuration */
	rxqueue0_size = macX_rxq0_size[plat->device_num];
	rxqueue1_size = macX_rxq1_size[plat->device_num];

	queue0_rfd = macX_rxq0_rfd[plat->device_num];
	queue0_rfa = macX_rxq0_rfa[plat->device_num];

	queue1_rfd = macX_rxq1_rfd[plat->device_num];
	queue1_rfa = macX_rxq1_rfa[plat->device_num];

	txqueue0_size = macX_txq0_size[plat->device_num];
	txqueue1_size = macX_txq1_size[plat->device_num];

	/* Validation of Queue size and Flow control thresholds and configuring local parameters to update registers*/
	if ((rxqueue0_size + rxqueue1_size) <= MAX_RX_QUEUE_SIZE) {
		plat->rx_queues_cfg[0].size = rxqueue0_size;
		plat->rx_queues_cfg[1].size = rxqueue1_size;
	} else {
		plat->rx_queues_cfg[0].size = RX_QUEUE0_SIZE; /* Default configuration when invalid input given */
		plat->rx_queues_cfg[1].size = RX_QUEUE1_SIZE;
		dev_info(&(pdev->dev), "%s: ERROR Invalid Rx Queue sizes passed rxq0_size=%d, rxq1_size=%d,Restoring default to rxq0_size=%d, rxq1_size=%d of port=%d Bus number=%x\n",
			__func__, rxqueue0_size, rxqueue1_size, RX_QUEUE0_SIZE, RX_QUEUE1_SIZE, plat->port_num, pdev->bus->number);

	}

	if ((((queue0_rfd * SIZE_512B) + SIZE_1KB) < plat->rx_queues_cfg[0].size) &&
		(((queue0_rfa * SIZE_512B) + SIZE_1KB) < plat->rx_queues_cfg[0].size)) {
		plat->rx_queues_cfg[0].rfd = queue0_rfd;
		plat->rx_queues_cfg[0].rfa = queue0_rfa;
	} else {
		temp_var = ((plat->rx_queues_cfg[0].size - (((plat->rx_queues_cfg[0].size)*8)/10))/SIZE_512B); /* configuration to 20% of FIFO Size */
		if (temp_var >= 2) {
			temp_var = (temp_var - 2);
		} else {
			temp_var = 0;
		}
		plat->rx_queues_cfg[0].rfd = temp_var;
		plat->rx_queues_cfg[0].rfa = temp_var;
		dev_info(&(pdev->dev), "%s: ERROR Invalid Flow control threshold for Rx Queue-0 passed rxq0_rfd=%d, rxq0_rfa=%d,configuring to 20%% of Queue size, rxq0_rfd=%d, rxq0_rfa=%d of port=%d Bus number=%x\n",
			__func__, queue0_rfd, queue0_rfa, plat->rx_queues_cfg[0].rfd, plat->rx_queues_cfg[0].rfa, plat->port_num, pdev->bus->number);
	}

	if ((((queue1_rfd * SIZE_512B) + SIZE_1KB) < plat->rx_queues_cfg[1].size) &&
		(((queue1_rfa * SIZE_512B) + SIZE_1KB) < plat->rx_queues_cfg[1].size)) {
		plat->rx_queues_cfg[1].rfd = queue1_rfd;
		plat->rx_queues_cfg[1].rfa = queue1_rfa;
	} else {
		temp_var = ((plat->rx_queues_cfg[1].size - (((plat->rx_queues_cfg[1].size)*8)/10))/SIZE_512B); /* configuration to 20% of FIFO Size */
		if (temp_var >= 2) {
			temp_var = (temp_var - 2);
		} else {
			temp_var = 0;
		}
		plat->rx_queues_cfg[1].rfd = temp_var;
		plat->rx_queues_cfg[1].rfa = temp_var;
		dev_info(&(pdev->dev), "%s: ERROR Invalid Flow control threshold for Rx Queue-1 passed rxq1_rfd=%d, rxq1_rfa=%d,configuring to 20%% of Queue size, rxq1_rfd=%d, rxq1_rfa=%d of port=%d Bus number=%x\n",
			__func__, queue1_rfd, queue1_rfa, plat->rx_queues_cfg[1].rfd, plat->rx_queues_cfg[1].rfa, plat->port_num, pdev->bus->number);
	}

	if ((txqueue0_size + txqueue1_size) <= MAX_TX_QUEUE_SIZE) {
		plat->tx_queues_cfg[0].size = txqueue0_size;
		plat->tx_queues_cfg[1].size = txqueue1_size;
	} else {
		plat->tx_queues_cfg[0].size = TX_QUEUE0_SIZE; /* Default configuration when invalid input given */
		plat->tx_queues_cfg[1].size = TX_QUEUE1_SIZE;
		dev_info(&(pdev->dev), "%s: ERROR Invalid Rx Queue sizes passed txq0_size=%d, txq1_size=%d, Restoring default to txq0_size=%d, txq1_size=%d of port=%d Bus number=%x\n",
			__func__, rxqueue0_size, rxqueue1_size, TX_QUEUE0_SIZE, TX_QUEUE1_SIZE, plat->port_num, pdev->bus->number);
	}
	return 0;
}

static const struct tc956xmac_pci_info tc956xmac_xgmac3_pci_info = {
	.setup = tc956xmac_xgmac3_default_data,
};

/*!
 * \brief API to Reset SRAM Region.
 *
 * \details This function Resets both IMEM & DMEM sections of tc956x.
 *
 * \param[in] dev  - pointer to device structure.
 * \param[in] res  - pointer to stmmac_resources structure.
 *
 * \return none
 */
static void tc956x_reset_SRAM(struct device *dev, struct stmmac_resources *res)
{
	dev_dbg(dev,  "Resetting SRAM Region start\n");
	/* Resetting SRAM IMEM Region */
	memset_io(res->tc956x_SRAM_pci_base_addr, 0x0, 0x10000);
	/* Resetting SRAM DMEM Region */
	memset_io((res->tc956x_SRAM_pci_base_addr + 0x40000), 0x0, 0x10000);
	dev_dbg(dev,  "Resetting SRAM Region end\n");
}

/*!
 * \brief API to load firmware for CM3.
 *
 * \details This fucntion loads the firmware onto the SRAM of tc956x.
 * The tc956x CM3 starts executing once the firmware loading is complete.
 *
 * \param[in] dev  - pointer to device structure.
 * \param[in] id   - pointer to stmmac_resources structure.
 *
 * \return integer
 *
 * \retval 0 on success & -ve number on failure.
 */

static s32 tc956x_load_firmware(struct device *dev, struct stmmac_resources *res)
{
	u32 adrs = 0, val = 0;
	u32 fw_init_sync;
	const struct firmware *pfw = NULL;

	dev_dbg(dev,  "FW Loading: .bin\n");

	/* Get TC956X FW binary through kernel firmware interface request */
	if (request_firmware(&pfw, FIRMWARE_NAME, dev) != 0) {
		dev_err(dev,
		"TC956X: Error in calling request_firmware");
		return -EINVAL;
	}

	if (pfw == NULL) {
		dev_err(dev, "TC956X: request_firmware: pfw == NULL");
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
	val = ioread32((void __iomem *)(res->addr + adrs));
	dev_dbg(dev,  "Reset Register value = %lx\n", (unsigned long)val);

	val |= NRSTCTRL0_RST_ASRT;
	iowrite32(val, (void __iomem *)(res->addr + adrs));

	iowrite32(0, (void __iomem *)(res->tc956x_SRAM_pci_base_addr +
			TC956X_M3_INIT_DONE));

	tc956x_reset_SRAM(dev, res);

	mdelay(10);
	iowrite8(EEPROM_OFFSET, (void __iomem *)(res->tc956x_SRAM_pci_base_addr +
			TC956X_M3_SRAM_EEPROM_OFFSET_ADDR));
	iowrite8(EEPROM_MAC_COUNT, (void __iomem *)(res->tc956x_SRAM_pci_base_addr +
			TC956X_M3_SRAM_EEPROM_MAC_COUNT));

	/* Copy TC956X FW to SRAM */
	memcpy_toio(res->tc956x_SRAM_pci_base_addr, pfw->data, pfw->size);
	/* Release kernel firmware interface */
	release_firmware(pfw);

	dev_dbg(dev,  "FW Loading Finish.\n");

	/* De-assert M3 reset */
	adrs = NRSTCTRL0_OFFSET;
	val = ioread32((void __iomem *)(res->addr + adrs));
	val &= ~NRSTCTRL0_RST_DE_ASRT;
	iowrite32(val, (void __iomem *)(res->addr + adrs));

	readl_poll_timeout_atomic(res->tc956x_SRAM_pci_base_addr + TC956X_M3_INIT_DONE,
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

	dev_dbg(dev, "-->%s\n", __func__);

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

	dev_dbg(dev, "<--%s\n", __func__);
}

static uint8_t get_tc956x_index(struct pci_dev *pdev)
{
	uint8_t index;
	uint32_t pci_bdf = pci_dev_id(pdev); /* [15:8] Bus number, [7:3] Slot number and [2:0] Function number */

	if (tc956x_eth_ports_bdf[tc956xmac_pm_usage_counter] == 0xFFFF) /* This is required when module parameters cannot be used. Other module parameters should hard coded as per device probe order */
		return tc956xmac_pm_usage_counter;


	for (index = 0; index < (TC956X_TOT_CASCADE_DEV*2); index++) {
		if (pci_bdf  == tc956x_eth_ports_bdf[index]) /* Compare Bus number, Device number and Function number */
			return (index);
	}
	return 0xFF; /* no match found */
}

/**
 * tc956xmac_pci_probe
 *
 * @pdev: pci device pointer
 * @id: pointer to table of device id/id's.
 *
 * Description: This probing function gets called for all PCI devices which
 * match the ID table and are not "owned" by other driver yet. This function
 * gets passed a "struct pci_dev *" for each device whose entry in the ID table
 * matches the device. The probe functions returns zero when the driver choose
 * to take "ownership" of the device or an error code(-ve no) otherwise.
 */
static int tc956xmac_pci_probe(struct pci_dev *pdev,
			    const struct pci_device_id *id)
{
	struct tc956xmac_pci_info *info = (struct tc956xmac_pci_info *)id->driver_data;
	struct plat_stmmacenet_data *plat;
	struct stmmac_resources res;
#if (defined(TC956X_PCIE_DSP_CUT_THROUGH) || defined(CONFIG_TC956X_PCIE_GEN3_SETTING)) && defined(TC956X_SRIOV_PF)
	u32 val;
#endif
#if defined(TC956X_PCIE_LINK_STATE_LATENCY_CTRL) && defined(TC956X_SRIOV_PF)
	u32 reg_val;
#endif /* end of TC956X_PCIE_LINK_STATE_LATENCY_CTRL */
	/* use signal from EMSPHY */
	uint8_t SgmSigPol = 0;
	int ret, reg;
	int overlay;
	u32 offset;
	char version_str[32];
#if defined(TC956X_PCIE_DSP_CUT_THROUGH) && defined(TC956X_SRIOV_PF)
	u32 pcie_mode; /* Read Setting A/B */
#endif
	uint16_t sh_mem_offset;

	dev_dbg(&pdev->dev, "%s  >", __func__);
	mutex_lock(&tc956x_pm_suspend_lock);
	dev_dbg(&(pdev->dev), "-->%s\n", __func__);
	scnprintf(version_str, sizeof(version_str), "Host Driver Version %d%d-%d%d-%d%d",
		tc956x_drv_version.rel_dbg,
		tc956x_drv_version.major, tc956x_drv_version.minor,
		tc956x_drv_version.sub_minor,
		tc956x_drv_version.patch_rel_major, tc956x_drv_version.patch_rel_minor);
	dev_dbg(&pdev->dev, "%s\n", version_str);

	plat = devm_kzalloc(&pdev->dev, sizeof(*plat), GFP_KERNEL);
	if (!plat) {
		ret = -ENOMEM;
		goto err_out_enb_failed;
	}

	plat->mdio_bus_data = devm_kzalloc(&pdev->dev,
					   sizeof(*plat->mdio_bus_data),
					   GFP_KERNEL);
	if (!plat->mdio_bus_data) {
		ret = -ENOMEM;
		goto err_out_enb_failed;
	}

	plat->dma_cfg = devm_kzalloc(&pdev->dev, sizeof(*plat->dma_cfg),
				     GFP_KERNEL);
	if (!plat->dma_cfg) {
		ret = -ENOMEM;
		goto err_out_enb_failed;
	}

	/* Enable pci device */
	ret = pci_enable_device(pdev);
	if (ret) {
		dev_err(&pdev->dev, "%s: ERROR: failed to enable device\n",
			__func__);
		goto err_out_enb_failed;
	}

	/* Request the PCI IO Memory for the device */
	if (pci_request_regions(pdev, DRIVER_NAME)) {
		dev_err(&(pdev->dev), "%s:Failed to get PCI regions\n",
			DRIVER_NAME);
		ret = -ENODEV;
		dev_dbg(&(pdev->dev), "<--%s : ret: %d\n", __func__, ret);
		goto err_out_req_reg_failed;
	}
	memset(&res, 0, sizeof(res));
	if (tc956x_no_of_vf > 0) {
		tc956x_no_of_vf = 0;
		dev_info(&(pdev->dev),
		"Enabling SRIOV not allowed in Automotive configuration\n");
	}

	res.probe_seq_no = tc956xmac_pm_usage_counter;
#ifdef CONFIG_PCI_IOV


	/* Enable SRIOV with the requested no of VFs */
	if ((tc956x_no_of_vf != 0) && (pdev->is_physfn)) {

		s32 pos = 0;

		pos = pci_find_ext_capability(pdev, PCI_EXT_CAP_ID_SRIOV);
		if (pos) {
			dev_dbg(&(pdev->dev), "SR-IOV capability found\n");

			/* Validate and Enable the requested No. of VFs value */
			if (tc956x_no_of_vf <= TC956X_TOTAL_VFS) {

				ret = pci_enable_sriov(pdev, tc956x_no_of_vf);
				if (ret) {

					dev_err(&(pdev->dev),
					"%s : SRIOV enable failed.\n",
					DRIVER_NAME);
					dev_dbg(&(pdev->dev),
					"<--%s: ret: %d\n", __func__, ret);
					goto err_sriov_vf_en_failed;
				}

				res.sriov_enabled = 1;

				dev_dbg(&(pdev->dev),
				"Total SR-IOV VFs Enabled: %d\n",
					tc956x_no_of_vf);
			} else {
				dev_alert(&(pdev->dev),
				"%s : VFs Value Out of Range.\n",
					DRIVER_NAME);
			}
		} else {
			dev_dbg(&(pdev->dev),
			"SR-IOV capability not found\n");
		}
	}
#endif
	/* From the Kernel version 6.4.0, AER Error reporting is enabled by default.
	 * It is enabled in pci_device_add() Kernel Space API.
	 * So not required to enable from EMAC Driver.
	 */
#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 4, 0)
	/* Enable AER Error reporting, if device capability is detected */
	if (pci_find_ext_capability(pdev, PCI_EXT_CAP_ID_ERR)) {

		pci_enable_pcie_error_reporting(pdev);
		dev_dbg(&(pdev->dev), "AER Capability Enabled\n");
	}
#endif
	/* Enable the bus mastering */
	pci_set_master(pdev);

	dev_dbg(&(pdev->dev),
		"BAR0 length = %lld bytes\n", (u64)pci_resource_len(pdev, 0));
	dev_dbg(&(pdev->dev),
		"BAR2 length = %lld bytes\n", (u64)pci_resource_len(pdev, 2));
	dev_dbg(&(pdev->dev),
		"BAR4 length = %lld bytes\n", (u64)pci_resource_len(pdev, 4));
	dev_dbg(&(pdev->dev),
		"BAR0 physical address = 0x%llx\n", (u64)pci_resource_start(pdev, 0));
	dev_dbg(&(pdev->dev),
		"BAR2 physical address = 0x%llx\n", (u64)pci_resource_start(pdev, 2));
	dev_dbg(&(pdev->dev),
		"BAR4 physical address = 0x%llx\n", (u64)pci_resource_start(pdev, 4));

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 6, 0)
	res.tc956x_BRIDGE_CFG_pci_base_addr = ioremap
		(pci_resource_start(pdev, TC956X_BAR0), pci_resource_len(pdev, TC956X_BAR0));
#else
	res.tc956x_BRIDGE_CFG_pci_base_addr = ioremap_nocache
		(pci_resource_start(pdev, TC956X_BAR0), pci_resource_len(pdev, TC956X_BAR0));
#endif
	if (((void __iomem *)res.tc956x_BRIDGE_CFG_pci_base_addr == NULL)) {
		dev_err(&(pdev->dev), "%s: cannot map TC956X BAR0, aborting", pci_name(pdev));
		ret = -EIO;
		dev_dbg(&(pdev->dev), "<--%s : ret: %d\n", __func__, ret);
		goto err_out_map_failed;
	}
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 6, 0)
	res.tc956x_SRAM_pci_base_addr = ioremap
		(pci_resource_start(pdev, TC956X_BAR2), pci_resource_len(pdev, TC956X_BAR2));
#else
	res.tc956x_SRAM_pci_base_addr = ioremap_nocache
		(pci_resource_start(pdev, TC956X_BAR2), pci_resource_len(pdev, TC956X_BAR2));
#endif
	if (((void __iomem *)res.tc956x_SRAM_pci_base_addr == NULL)) {
		pci_iounmap(pdev, (void __iomem *)res.tc956x_BRIDGE_CFG_pci_base_addr);
		dev_err(&(pdev->dev), "%s: cannot map TC956X BAR2, aborting", pci_name(pdev));
		ret = -EIO;
		dev_dbg(&(pdev->dev), "<--%s : ret: %d\n", __func__, ret);
		goto err_out_map_failed;
	}
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 6, 0)
	res.tc956x_SFR_pci_base_addr = ioremap
		(pci_resource_start(pdev, TC956X_BAR4), pci_resource_len(pdev, TC956X_BAR4));
#else
	res.tc956x_SFR_pci_base_addr = ioremap_nocache
		(pci_resource_start(pdev, TC956X_BAR4), pci_resource_len(pdev, TC956X_BAR4));
#endif
	if (((void __iomem *)res.tc956x_SFR_pci_base_addr == NULL)) {
		pci_iounmap(pdev, (void __iomem *)res.tc956x_BRIDGE_CFG_pci_base_addr);
		pci_iounmap(pdev, (void __iomem *)res.tc956x_SRAM_pci_base_addr);
		dev_err(&(pdev->dev), "%s: cannot map TC956X BAR4, aborting", pci_name(pdev));
		ret = -EIO;
		dev_dbg(&(pdev->dev), "<--%s : ret: %d\n", __func__, ret);
		goto err_out_map_failed;
	}

	dev_dbg(&(pdev->dev), "BAR0 virtual address = %p\n", res.tc956x_BRIDGE_CFG_pci_base_addr);
	dev_dbg(&(pdev->dev), "BAR2 virtual address = %p\n", res.tc956x_SRAM_pci_base_addr);
	dev_dbg(&(pdev->dev), "BAR4 virtual address = %p\n", res.tc956x_SFR_pci_base_addr);

	res.addr = res.tc956x_SFR_pci_base_addr;

	res.port_num = readl(res.tc956x_BRIDGE_CFG_pci_base_addr + RSCMNG_ID_REG); /* Resource Manager ID */
	res.port_num &= RSCMNG_PFN;

	if (res.port_num == RM_PF0_ID) {

		if ((tc956x_logstat_set_state_log_enable((void __iomem *)res.addr, UPSTREAM_PORT, STATE_LOG_ENABLE) < 0)
		|| (tc956x_logstat_set_state_log_enable((void __iomem *)res.addr, DOWNSTREAM_PORT1, STATE_LOG_ENABLE) < 0)
		|| (tc956x_logstat_set_state_log_enable((void __iomem *)res.addr, DOWNSTREAM_PORT2, STATE_LOG_ENABLE) < 0)
		|| (tc956x_logstat_set_state_log_enable((void __iomem *)res.addr, INTERNAL_ENDPOINT, STATE_LOG_ENABLE) < 0)) {
			ret = -EFAULT; /* The returns returned by above functions are -EFAULT only */
			dev_dbg(&(pdev->dev), "<--%s : Error ret: %d\n", __func__, ret);
			goto err_dvr_logstat;
		}
	}


	/* Get the device index by comparing the user passed BDF (module param) with actual BDF */
	res.device_num = get_tc956x_index(pdev);
	dev_dbg(&(pdev->dev), "tc956x_eth_ports_bdf matched device index for this device is: %d and Port number: %d\n", res.device_num, res.port_num);

	if (res.device_num == 0xFF) {
		res.device_num = (TC956X_TOT_CASCADE_DEV*2); /* Use the slot at the end of array for non-matching devices */

		dev_dbg(&(pdev->dev), "Error: Module parameter tc956x_eth_ports_bdf not provided or\
			value provided in module param not matching with the device BDF.\
			Use the device number as %d and set other associated module parameter values to default\n", res.device_num);

		macX_interface[res.device_num]					= 0xFF;
		portX_mdc[res.device_num]						= 0xFF;
		portX_c45_state[res.device_num]					= 0xFF;
		portX_phyaddr[res.device_num]					= 0;
		macX_link_down_macrst[res.device_num]			= 0xFF;
		macX_no_mdio_no_phy[res.device_num]				= PHY_ON_MDIO_ON;

		macX_rxq0_size[res.device_num]					= RX_QUEUE0_SIZE;
		macX_rxq1_size[res.device_num]					= RX_QUEUE1_SIZE;
		macX_rxq0_rfd[res.device_num]					= RX_QUEUE0_RFD;
		macX_rxq0_rfa[res.device_num]					= RX_QUEUE0_RFA;
		macX_rxq1_rfd[res.device_num]					= RX_QUEUE1_RFD;
		macX_rxq1_rfa[res.device_num]					= RX_QUEUE1_RFA;
		macX_txq0_size[res.device_num]					= TX_QUEUE0_SIZE;
		macX_txq1_size[res.device_num]					= TX_QUEUE1_SIZE;
		macX_force_speed_mode[res.device_num]			= DISABLE;
		macX_force_config_speed[res.device_num]			= DEF_FORCE_CONFIG_SPEED;
		macX_eee_enable[res.device_num]					= DISABLE;
		macX_lpi_timer[res.device_num]					= TC956XMAC_LPIET_600US;
		macX_filter_phy_pause[res.device_num]			= DISABLE;
		macX_en_lp_pause_frame_cnt[res.device_num]		= DISABLE;
		macX_power_save_at_link_down[res.device_num]	= DISABLE;
#ifdef TC956X_PCIE_LINK_STATE_LATENCY_CTRL
		epX_l0s_delay[res.device_num]					= EP_L0s_ENTRY_DELAY;
		epX_l1_delay[res.device_num]					= EP_L1_ENTRY_DELAY;
		uspX_l0s_delay[res.device_num]					= USP_L0s_ENTRY_DELAY;
		uspX_l1_delay[res.device_num]					= USP_L1_ENTRY_DELAY;
#endif

	}

	plat->device_num = res.device_num;

#ifdef TC956X_PCIE_LINK_STATE_LATENCY_CTRL
	epX_l0s_delay[res.device_num] = ((epX_l0s_delay[res.device_num] <= EP_L0s_ENTRY_DELAY) &&
	(epX_l0s_delay[res.device_num] > INVALID_L0s_ENTRY_DELAY)) ?  epX_l0s_delay[res.device_num] : EP_L0s_ENTRY_DELAY;
	plat->ep_l0s_delay = epX_l0s_delay[res.device_num];

	epX_l1_delay[res.device_num] = ((epX_l1_delay[res.device_num] <= EP_L1_ENTRY_DELAY) &&
	(epX_l1_delay[res.device_num] > INVALID_L1_ENTRY_DELAY)) ? epX_l1_delay[res.device_num] : EP_L1_ENTRY_DELAY;
	plat->ep_l1_delay = epX_l1_delay[res.device_num];

	uspX_l0s_delay[res.device_num] = ((uspX_l0s_delay[res.device_num] <= USP_L0s_ENTRY_DELAY) &&
	(uspX_l0s_delay[res.device_num] > INVALID_L0s_ENTRY_DELAY)) ? uspX_l0s_delay[res.device_num] : USP_L0s_ENTRY_DELAY;
	plat->usp_l0s_delay = uspX_l0s_delay[res.device_num];

	uspX_l1_delay[res.device_num] = ((uspX_l1_delay[res.device_num] <= USP_L1_ENTRY_DELAY) &&
	(uspX_l1_delay[res.device_num] > INVALID_L1_ENTRY_DELAY)) ? uspX_l1_delay[res.device_num] : USP_L1_ENTRY_DELAY;
	plat->usp_l1_delay = uspX_l1_delay[res.device_num];
#endif /*#ifdef TC956X_PCIE_LINK_STATE_LATENCY_CTRL*/

#ifdef TC956X_PCIE_LINK_STATE_LATENCY_CTRL
	/* 0x4002_C02C SSREG_GLUE_SW_REG_ACCESS_CTRL.sw_port_reg_access_enable for USP Access enable */
	writel(SW_USP_ENABLE, res.addr + TC956X_GLUE_SW_REG_ACCESS_CTRL);

	/* 0x4002_496C K_PEXCONF_209_205.aspm_l0s_entry_delay in terms of 256ns */
	reg_val = readl(res.addr + TC956X_PCIE_S_L0s_ENTRY_LATENCY);
	reg_val &= ~(TC956X_PCIE_USP_L0s_ENTRY_MASK);
	reg_val |= ((plat->usp_l0s_delay << TC956X_PCIE_USP_L0s_ENTRY_SHIFT) & TC956X_PCIE_USP_L0s_ENTRY_MASK);
	writel(reg_val, res.addr + TC956X_PCIE_S_L0s_ENTRY_LATENCY);

	/* 0x4002_4970 K_PEXCONF_219_210.aspm_L1_entry_delay in terms of 256ns */
	reg_val = readl(res.addr + TC956X_PCIE_S_L1_ENTRY_LATENCY);
	reg_val &= ~(TC956X_PCIE_USP_L1_ENTRY_MASK);
	reg_val |= ((plat->usp_l1_delay << TC956X_PCIE_USP_L1_ENTRY_SHIFT) & TC956X_PCIE_USP_L1_ENTRY_MASK);
	writel(reg_val, res.addr + TC956X_PCIE_S_L1_ENTRY_LATENCY);

	/* 0x4002_C02C SSREG_GLUE_SW_REG_ACCESS_CTRL.sw_port_reg_access_enable for DSP1 Access enable */
	writel(SW_DSP1_ENABLE, res.addr + TC956X_GLUE_SW_REG_ACCESS_CTRL);
	/* 0x4002_496C K_PEXCONF_209_205.aspm_l0s_entry_delay in terms of 256ns */
	writel(DSP1_L0s_ENTRY_DELAY, res.addr + TC956X_PCIE_S_L0s_ENTRY_LATENCY);
	/* 0x4002_4970 K_PEXCONF_219_210.aspm_L1_entry_delay in terms of 256ns */
	writel(DSP1_L1_ENTRY_DELAY, res.addr + TC956X_PCIE_S_L1_ENTRY_LATENCY);

	/* 0x4002_C02C SSREG_GLUE_SW_REG_ACCESS_CTRL.sw_port_reg_access_enable for DSP2 Access enable */
	writel(SW_DSP2_ENABLE, res.addr + TC956X_GLUE_SW_REG_ACCESS_CTRL);
	/* 0x4002_496C K_PEXCONF_209_205.aspm_l0s_entry_delay in terms of 256ns */
	writel(DSP2_L0s_ENTRY_DELAY, res.addr + TC956X_PCIE_S_L0s_ENTRY_LATENCY);
	/* 0x4002_4970 K_PEXCONF_219_210.aspm_L1_entry_delay in terms of 256ns */
	writel(DSP2_L1_ENTRY_DELAY, res.addr + TC956X_PCIE_S_L1_ENTRY_LATENCY);

	/* 0x4002_C02C SSREG_GLUE_SW_REG_ACCESS_CTRL.sw_port_reg_access_enable
	 * for VDSP Access enable
	 */
	writel(SW_VDSP_ENABLE, res.addr + TC956X_GLUE_SW_REG_ACCESS_CTRL);
	/* 0x4002_496C K_PEXCONF_209_205.aspm_l0s_entry_delay in terms of 256ns */
	writel(VDSP_L0s_ENTRY_DELAY, res.addr + TC956X_PCIE_S_L0s_ENTRY_LATENCY);
	/* 0x4002_4970 K_PEXCONF_219_210.aspm_L1_entry_delay in terms of 256ns */
	writel(VDSP_L1_ENTRY_DELAY, res.addr + TC956X_PCIE_S_L1_ENTRY_LATENCY);

	/* 0x4002_00D8 Reading PCIE EP Capability setting Register */
	reg_val = readl(res.addr + TC956X_PCIE_EP_CAPB_SET);

	/* Clearing PCIE EP Capability setting of L0s & L1 entry delays */
	reg_val &= ~(TC956X_PCIE_EP_L0s_ENTRY_MASK | TC956X_PCIE_EP_L1_ENTRY_MASK);

	/* Updating PCIE EP Capability setting of L0s & L1 entry delays */
	reg_val |= (((plat->ep_l0s_delay << TC956X_PCIE_EP_L0s_ENTRY_SHIFT) &
				TC956X_PCIE_EP_L0s_ENTRY_MASK) |
			((plat->ep_l1_delay  << TC956X_PCIE_EP_L1_ENTRY_SHIFT) &
				TC956X_PCIE_EP_L1_ENTRY_MASK));

	/* 0x4002_00D8 PCIE EP Capability setting L0S & L1 entry delay in terms of 256ns */
	writel(reg_val, res.addr + TC956X_PCIE_EP_CAPB_SET);

	/* 0x4002_C02C SSREG_GLUE_SW_REG_ACCESS_CTRL.sw_port_reg_access_enable
	 * for All Switch Ports Access enable
	 */
	writel(TC956X_PCIE_S_EN_ALL_PORTS_ACCESS, res.addr + TC956X_GLUE_SW_REG_ACCESS_CTRL);
#endif /* end of TC956X_PCIE_LINK_STATE_LATENCY_CTRL */






	res.port_num = readl(res.tc956x_BRIDGE_CFG_pci_base_addr + RSCMNG_ID_REG); /* Resource Manager ID */
	res.port_num &= RSCMNG_PFN;
#ifdef CONFIG_TC956X_MAGIC_PACKET_WOL_GPIO
	if (res.port_num == RM_PF0_ID) {
		pr_debug("%s: Port %d Bus number %x - Configuring GPIO for WOL", __func__, res.port_num, pdev->bus->number);
		tc956x_wol_gpio_trigger(res.addr, false); /* Set to LOW */
	}
#endif
#ifdef DISABLE_EMAC_PORT1
	if (res.port_num == RM_PF1_ID) {

		dev_dbg(&pdev->dev, "Disabling all eMAC clocks for Port 1\n");
		/* Disable all clocks to eMAC Port1 */
		ret = readl(res.addr + NCLKCTRL1_OFFSET);

		ret &= (~(NCLKCTRL1_MAC1TXCEN | NCLKCTRL1_MAC1RXCEN |
			  NCLKCTRL1_MAC1ALLCLKEN1 | NCLKCTRL1_MAC1RMCEN));
		writel(ret, res.addr + NCLKCTRL1_OFFSET);

		ret = -ENODEV;
		goto disable_emac_port;
	}
#endif

	plat->port_num = res.port_num;

	reg = readl(res.addr + NCID_OFFSET);
	pr_debug("NCID Register value: %x\n", readl(res.addr + NCID_OFFSET));
	if ((reg & REV_ID_MASK) == REV_ID1)
		plat->RevID = REV_ID1;
	else if ((reg & REV_ID_MASK) == REV_ID2)
		plat->RevID = REV_ID2;

	dev_dbg(&pdev->dev, "Board Revision ID = %d\n", plat->RevID);

	/* User configured/Default Module parameters of TC956x*/
	dev_dbg(&pdev->dev, "User Configured/Default Module parameters of TC956x of Port-%d bus number-%x\n", plat->port_num, pdev->bus->number);
	dev_dbg(&pdev->dev, "macX_interface = %d\n", macX_interface[res.device_num]);
	dev_dbg(&pdev->dev, "macX_link_down_macrst = %d\n", macX_link_down_macrst[res.device_num]);
	dev_dbg(&pdev->dev, "portX_mdc = 0x%x\n", portX_mdc[res.device_num]);
	dev_dbg(&pdev->dev, "portX_c45_state = %d\n", portX_c45_state[res.device_num]);
	dev_dbg(&pdev->dev, "portX_phyaddr = %d\n", portX_phyaddr[res.device_num]);
	dev_dbg(&pdev->dev, "macX_no_mdio_no_phy = %d\n", macX_no_mdio_no_phy[res.device_num]);
	dev_dbg(&pdev->dev, "macX_force_speed_mode = %d\n", macX_force_speed_mode[res.device_num]);
	dev_dbg(&pdev->dev, "macX_force_config_speed = %d\n", macX_force_config_speed[res.device_num]);
	dev_dbg(&pdev->dev, "macX_eee_enable = %d\n", macX_eee_enable[res.device_num]);
	dev_dbg(&pdev->dev, "macX_lpi_timer = %d\n", macX_lpi_timer[res.device_num]);
	dev_dbg(&pdev->dev, "macX_filter_phy_pause = %d\n", macX_filter_phy_pause[res.device_num]);
	dev_dbg(&pdev->dev, "macX_rxq0_size = %d\n", macX_rxq0_size[res.device_num]);
	dev_dbg(&pdev->dev, "macX_rxq1_size = %d\n", macX_rxq1_size[res.device_num]);
	dev_dbg(&pdev->dev, "macX_rxq0_rfd  = %d\n", macX_rxq0_rfd[res.device_num]);
	dev_dbg(&pdev->dev, "macX_rxq0_rfa  = %d\n", macX_rxq0_rfa[res.device_num]);
	dev_dbg(&pdev->dev, "macX_rxq1_rfd  = %d\n", macX_rxq1_rfd[res.device_num]);
	dev_dbg(&pdev->dev, "macX_rxq1_rfa  = %d\n", macX_rxq1_rfa[res.device_num]);
	dev_dbg(&pdev->dev, "macX_txq0_size = %d\n", macX_txq0_size[res.device_num]);
	dev_dbg(&pdev->dev, "macX_txq1_size = %d\n", macX_txq1_size[res.device_num]);
	dev_dbg(&pdev->dev, "macX_en_lp_pause_frame_cnt = %d\n", macX_en_lp_pause_frame_cnt[res.device_num]);
	dev_dbg(&pdev->dev, "macX_power_save_at_link_down = %d \n", macX_power_save_at_link_down[res.device_num]);
#ifdef TC956X_PCIE_LINK_STATE_LATENCY_CTRL
	dev_dbg(&pdev->dev, "epX_l0s_delay = %d\n", epX_l0s_delay[res.device_num]);
	dev_dbg(&pdev->dev, "epX_l1_delay = %d\n", epX_l1_delay[res.device_num]);
	dev_dbg(&pdev->dev, "uspX_l0s_delay = %d\n", uspX_l0s_delay[res.device_num]);
	dev_dbg(&pdev->dev, "uspX_l1_delay = %d\n", uspX_l1_delay[res.device_num]);
#endif

	for (offset = 0; offset < TC956X_TOT_CASCADE_DEV*2; offset++)
		dev_dbg(&pdev->dev, "tc956x_eth_ports_bdf[%d] = 0x%x\n", offset, tc956x_eth_ports_bdf[offset]);

	if (res.port_num == RM_PF0_ID) {

		/* Set the PORT0 interface mode to default, in case of invalid input */
		if ((macX_interface[res.device_num] == ENABLE_RGMII_INTERFACE) || (macX_interface[res.device_num] == ENABLE_RGMII_ID_INTERFACE) ||
			(macX_interface[res.device_num] < ENABLE_USXGMII_INTERFACE) || (macX_interface[res.device_num] > ENABLE_USXGMII_2_5G_INTERFACE)) {
			macX_interface[res.device_num] = ENABLE_XFI_INTERFACE;
			dev_info(&(pdev->dev), "%s: ERROR Invalid macX_interface parameter passed. Restoring to default interface %d for the device index: %d\n",
			__func__, macX_interface[res.device_num], res.device_num);
		} else if ((macX_interface[res.device_num] > MAX_INTERFACE) && (macX_interface[res.device_num] <= ENABLE_USXGMII_2_5G_INTERFACE)) {
				macX_interface[res.device_num] = ENABLE_USXGMII_10G_INTERFACE;
			dev_info(&(pdev->dev), "%s: ERROR Un-supported USXGMII mode passed for macX_interface parameter. Restoring to default interface %d for the device index: %d\n",
			__func__, macX_interface[res.device_num], res.device_num);
		}
		res.port_interface = macX_interface[res.device_num];

		portX_mdc[res.device_num] = (portX_mdc[res.device_num] > TC956XMAC_XGMAC_MDC_CSR_202) ? TC956XMAC_XGMAC_MDC_CSR_12 : portX_mdc[res.device_num];
		plat->mdc_clk = portX_mdc[res.device_num];

		portX_c45_state[res.device_num] = (portX_c45_state[res.device_num] > 1) ? true : portX_c45_state[res.device_num];
		plat->c45_needed = portX_c45_state[res.device_num];

		macX_link_down_macrst[res.device_num] = (macX_link_down_macrst[res.device_num] > ENABLE) ? ENABLE : macX_link_down_macrst[res.device_num];
		plat->link_down_macrst = macX_link_down_macrst[res.device_num];

	}

	if (res.port_num == RM_PF1_ID) {

		/* Set the PORT1 interface mode to default, in case of invalid input */
		if ((macX_interface[res.device_num] <  ENABLE_USXGMII_INTERFACE) || (macX_interface[res.device_num] > ENABLE_USXGMII_2_5G_INTERFACE)) {
			macX_interface[res.device_num] = ENABLE_SGMII_INTERFACE;
			dev_info(&(pdev->dev), "%s: ERROR Invalid macX_interface parameter passed. Restoring to default interface %d for the device index: %d\n",
			__func__, macX_interface[res.device_num], res.device_num);
		} else if ((macX_interface[res.device_num] > MAX_INTERFACE) && (macX_interface[res.device_num] <= ENABLE_USXGMII_2_5G_INTERFACE)) {
			macX_interface[res.device_num] = ENABLE_USXGMII_10G_INTERFACE;
			dev_info(&(pdev->dev), "%s: ERROR Un-supported USXGMII mode passed for macX_interface parameter. Restoring to default interface %d for the device index: %d\n",
			__func__, macX_interface[res.device_num], res.device_num);
		}

		res.port_interface = macX_interface[res.device_num];

		portX_mdc[res.device_num] = (portX_mdc[res.device_num] > TC956XMAC_XGMAC_MDC_CSR_202) ? TC956XMAC_XGMAC_MDC_CSR_62 : portX_mdc[res.device_num];
		plat->mdc_clk = portX_mdc[res.device_num];

		portX_c45_state[res.device_num] = (portX_c45_state[res.device_num] > 1) ? false : portX_c45_state[res.device_num];
		plat->c45_needed = portX_c45_state[res.device_num];

		macX_link_down_macrst[res.device_num] = (macX_link_down_macrst[res.device_num] > ENABLE) ? DISABLE : macX_link_down_macrst[res.device_num];
		plat->link_down_macrst = macX_link_down_macrst[res.device_num];

	}

	macX_filter_phy_pause[res.device_num] = (macX_filter_phy_pause[res.device_num] > ENABLE) ? DISABLE : macX_filter_phy_pause[res.device_num];
	plat->filter_phy_pause = macX_filter_phy_pause[res.device_num];

	macX_en_lp_pause_frame_cnt[res.device_num] = (macX_en_lp_pause_frame_cnt[res.device_num] > ENABLE) ? DISABLE : macX_en_lp_pause_frame_cnt[res.device_num];
	plat->en_lp_pause_frame_cnt = macX_en_lp_pause_frame_cnt[res.device_num];

	macX_power_save_at_link_down[res.device_num] = (macX_power_save_at_link_down[res.device_num] > ENABLE) ? DISABLE : macX_power_save_at_link_down[res.device_num];
	plat->mac_power_save_at_link_down = macX_power_save_at_link_down[res.device_num];

	plat->start_phy_addr = portX_phyaddr[res.device_num] = portX_phyaddr[res.device_num] > PHY_MAX_ADDR ? 0 : portX_phyaddr[res.device_num];

	if (macX_no_mdio_no_phy[res.device_num] != PHY_OFF_MDIO_OFF)
		macX_no_mdio_no_phy[res.device_num] = PHY_ON_MDIO_ON; /* Currently only PHY OFF and MDIO OFF is supported for SFP+ case, others are invalid */

	plat->mac_no_mdio_no_phy = macX_no_mdio_no_phy[res.device_num];

	overlay = tc956x_platform_port_interface_overlay(&pdev->dev, &res);
	if (overlay) {
		plat->mdc_clk = res.mdc_clk;
		plat->c45_needed = res.c45_state;
		plat->link_down_macrst = (res.link_down_macrst == 1) ? ENABLE : DISABLE;
	}


	plat->port_interface = res.port_interface;

	if ((macX_force_speed_mode[res.device_num] != DISABLE) && (macX_force_speed_mode[res.device_num] != ENABLE)) {
		macX_force_speed_mode[res.device_num] = DISABLE;
		dev_info(&(pdev->dev), "%s: ERROR Invalid mac1_force_speed_mode parameter passed. Restoring default to %d. Supported Values are 0 and 1.\n",
		__func__, macX_force_speed_mode[res.device_num]);
	}
	plat->force_speed_mode = macX_force_speed_mode[res.device_num];

	if (macX_force_speed_mode[res.device_num] == ENABLE) {
		if (macX_force_config_speed[res.device_num] > 5) { /*Configuring default value on error*/
			macX_force_config_speed[res.device_num] = 3;
			dev_info(&(pdev->dev), "%s: ERROR Invalid mac1_force_config_speed parameter passed. Restoring default to %d. Supported Values are 0 to 5.\n",
			__func__, macX_force_config_speed[res.device_num]);
		}
	}
	plat->force_config_speed = macX_force_config_speed[res.device_num];

	if ((macX_eee_enable[res.device_num] != DISABLE) &&
	(macX_eee_enable[res.device_num] != ENABLE)) {
		macX_eee_enable[res.device_num] = DISABLE;
		dev_info(&(pdev->dev), "%s: ERROR Invalid mac1_eee_enable parameter passed. Restoring default to %d. Supported Values are 0 and 1.\n",
		__func__, macX_eee_enable[res.device_num]);
	}

	if ((macX_eee_enable[res.device_num] == ENABLE) &&
	(macX_lpi_timer[res.device_num] > TC956X_MAX_LPI_AUTO_ENTRY_TIMER)) {
		macX_lpi_timer[res.device_num] = TC956XMAC_LPIET_600US;
		dev_info(&(pdev->dev), "%s: ERROR Invalid mac1_lpi_timer parameter passed. Restoring default to %d. Supported Values between %d and %d.\n",
		__func__, macX_lpi_timer[res.device_num],
		TC956X_MIN_LPI_AUTO_ENTRY_TIMER, TC956X_MAX_LPI_AUTO_ENTRY_TIMER);
	}
	res.eee_enabled = macX_eee_enable[res.device_num];
	res.tx_lpi_timer = macX_lpi_timer[res.device_num];


	ret = info->setup(pdev, plat);

	if (ret)
		goto err_out_enb_failed;

	dev_dbg(&pdev->dev, "Re-Configured Module parameters of TC956x of Port-%d bus number-%x\n", plat->port_num, pdev->bus->number);
	dev_dbg(&pdev->dev, "macX_interface = %d\n", res.port_interface);
	dev_dbg(&pdev->dev, "macX_link_down_macrst = %d\n", plat->link_down_macrst);
	dev_dbg(&pdev->dev, "portX_mdc = 0x%x\n", plat->mdc_clk);
	dev_dbg(&pdev->dev, "portX_c45_state = %d\n", plat->c45_needed);
	dev_dbg(&pdev->dev, "portX_phyaddr = %d\n", plat->start_phy_addr);
	dev_dbg(&pdev->dev, "macX_no_mdio_no_phy = %d\n", plat->mac_no_mdio_no_phy);
	dev_dbg(&pdev->dev, "macX_force_speed_mode = %d \n", plat->force_speed_mode);
	dev_dbg(&pdev->dev, "macX_force_config_speed = %d\n", plat->force_config_speed);
	dev_dbg(&pdev->dev, "macX_eee_enable = %d\n", res.eee_enabled);
	dev_dbg(&pdev->dev, "macX_lpi_timer = %d\n", res.tx_lpi_timer);
	dev_dbg(&pdev->dev, "macX_filter_phy_pause = %d\n", plat->filter_phy_pause);
	dev_dbg(&pdev->dev, "macX_rxq0_size = %d\n", plat->rx_queues_cfg[0].size);
	dev_dbg(&pdev->dev, "macX_rxq1_size = %d\n", plat->rx_queues_cfg[1].size);
	dev_dbg(&pdev->dev, "macX_rxq0_rfd  = %d\n", plat->rx_queues_cfg[0].rfd);
	dev_dbg(&pdev->dev, "macX_rxq0_rfa  = %d\n", plat->rx_queues_cfg[0].rfa);
	dev_dbg(&pdev->dev, "macX_rxq1_rfd  = %d\n", plat->rx_queues_cfg[1].rfd);
	dev_dbg(&pdev->dev, "macX_rxq1_rfa  = %d\n", plat->rx_queues_cfg[1].rfa);
	dev_dbg(&pdev->dev, "macX_txq0_size = %d\n", plat->tx_queues_cfg[0].size);
	dev_dbg(&pdev->dev, "macX_txq1_size = %d\n", plat->tx_queues_cfg[1].size);
	dev_dbg(&pdev->dev, "macX_en_lp_pause_frame_cnt = %d\n", plat->en_lp_pause_frame_cnt);
	dev_dbg(&pdev->dev, "macX_power_save_at_link_down = %d\n", plat->mac_power_save_at_link_down);
#ifdef TC956X_PCIE_LINK_STATE_LATENCY_CTRL
	dev_dbg(&pdev->dev, "epX_l0s_delay = %d\n", plat->ep_l0s_delay);
	dev_dbg(&pdev->dev, "epX_l1_delay = %d\n", plat->ep_l1_delay);
	dev_dbg(&pdev->dev, "uspX_l0s_delay = %d\n", plat->usp_l0s_delay);
	dev_dbg(&pdev->dev, "uspX_l1_delay = %d\n", plat->usp_l1_delay);
#endif

	if (res.port_num == RM_PF0_ID) {
		ret = readl(res.addr + NRSTCTRL0_OFFSET);
		ret |= (NRSTCTRL0_INTRST);
		writel(ret, res.addr + NRSTCTRL0_OFFSET);

		ret = readl(res.addr + NCLKCTRL0_OFFSET);
		ret |= NCLKCTRL0_INTCEN;
		writel(ret, res.addr + NCLKCTRL0_OFFSET);

		ret = readl(res.addr + NRSTCTRL0_OFFSET);
		ret &= (~(NRSTCTRL0_INTRST));
		writel(ret, res.addr + NRSTCTRL0_OFFSET);

		/* Configure Address Transslation block
		 * Bridge Base address to be passed for TC956X
		 */
		tc956x_config_tamap(&pdev->dev, res.tc956x_BRIDGE_CFG_pci_base_addr);
	}
	dev_dbg(&(pdev->dev), "Initialising eMAC Port %d bus number-%x\n", res.port_num, pdev->bus->number);
	/* Enable MSI  and Allocate Vectors */
	ret = pci_alloc_irq_vectors(pdev, TC956X_TOT_MSI_VEC,
				TC956X_TOT_MSI_VEC, PCI_IRQ_MSI);

	if (ret < TC956X_TOT_MSI_VEC) {
		dev_err(&(pdev->dev),
		"%s:Enable MSI error\n", DRIVER_NAME);
		dev_dbg(&(pdev->dev), "<--%s : ret: %d\n", __func__, ret);
		goto err_out_msi_failed;
	}

	dev_dbg(&(pdev->dev), "%s : Allocated MSI Vectors : %d",
								__func__, ret);
	pci_write_config_dword(pdev, pdev->msi_cap + PCI_MSI_MASK_64, 0);

#ifdef EEPROM_MAC_ADDR

	iowrite8(EEPROM_OFFSET, (void __iomem *)(res.tc956x_SRAM_pci_base_addr +
			TC956X_M3_SRAM_EEPROM_OFFSET_ADDR));
	iowrite8(EEPROM_MAC_COUNT, (void __iomem *)(res.tc956x_SRAM_pci_base_addr +
			TC956X_M3_SRAM_EEPROM_MAC_COUNT));

#endif

	if (res.port_num == RM_PF0_ID) {
		ret = tc956x_load_firmware(&pdev->dev, &res);
		if (ret)
			dev_err(&(pdev->dev), "Firmware load failed\n");
	}


	if (res.port_num == RM_PF0_ID) {
		ret = readl(res.addr + NRSTCTRL0_OFFSET);

		/* Assertion of EMAC Port0 software Reset */
		ret |= NRSTCTRL0_MAC0RST;

		writel(ret, res.addr + NRSTCTRL0_OFFSET);

		dev_dbg(&pdev->dev, "Enabling all eMAC clocks for Port 0 Bus number %x\n", pdev->bus->number);
		/* Enable all clocks to eMAC Port0 */
		ret = readl(res.addr + NCLKCTRL0_OFFSET);

		ret |= ((NCLKCTRL0_MAC0TXCEN | NCLKCTRL0_MAC0ALLCLKEN | NCLKCTRL0_MAC0RXCEN));
		/* Only if "current" port is SGMII 2.5G, configure below clocks. */
		if ((res.port_interface == ENABLE_SGMII_INTERFACE) ||
			(res.port_interface == ENABLE_2500BASE_X_INTERFACE)) {
			ret &= ~NCLKCTRL0_POEPLLCEN;
			ret &= ~NCLKCTRL0_SGMPCIEN;
			ret &= ~NCLKCTRL0_REFCLKOCEN;
			ret &= ~NCLKCTRL0_MAC0125CLKEN;
			ret &= ~NCLKCTRL0_MAC0312CLKEN;
		}
		writel(ret, res.addr + NCLKCTRL0_OFFSET);

		/* Interface configuration for port0*/
		ret = readl(res.addr + NEMAC0CTL_OFFSET);
		ret &= ~(NEMACCTL_SP_SEL_MASK | NEMACCTL_PHY_INF_SEL_MASK);
		if ((res.port_interface == ENABLE_SGMII_INTERFACE) ||
			(res.port_interface == ENABLE_2500BASE_X_INTERFACE))
			ret |= NEMACCTL_SP_SEL_SGMII_2500M;
		else if ((res.port_interface == ENABLE_USXGMII_INTERFACE) || (res.port_interface == ENABLE_USXGMII_10G_INTERFACE) ||
			(res.port_interface == ENABLE_XFI_INTERFACE))
			ret |= NEMACCTL_SP_SEL_USXGMII_10G_10G;
		else if (res.port_interface == ENABLE_USXGMII_5G_INTERFACE)
			ret |= NEMACCTL_SP_SEL_USXGMII_5G_5G;
		else if (res.port_interface == ENABLE_USXGMII_2_5G_INTERFACE)
			ret |= NEMACCTL_SP_SEL_USXGMII_2_5G_2_5G;

		ret &= ~(0x00000040); /* Mask Polarity */
		if (SgmSigPol == 1)
			ret |= 0x00000040; /* Set Active low */

		ret |= NEMACCTL_PHY_INF_SEL | NEMACCTL_LPIHWCLKEN;
		writel(ret, res.addr + NEMAC0CTL_OFFSET);

		/* De-assertion of EMAC Port0  software Reset*/
		ret = readl(res.addr + NRSTCTRL0_OFFSET);
		ret &= ~(NRSTCTRL0_MAC0RST);
		writel(ret, res.addr + NRSTCTRL0_OFFSET);
	}

	if (res.port_num == RM_PF1_ID) {
		ret = readl(res.addr + NRSTCTRL1_OFFSET);

		/* Assertion of EMAC Port1 software Reset*/
		ret |= NRSTCTRL1_MAC1RST1;
		writel(ret, res.addr + NRSTCTRL1_OFFSET);

		dev_dbg(&pdev->dev, "Enabling all eMAC clocks for Port 1 Bus number-%x\n", pdev->bus->number);
		/* Enable all clocks to eMAC Port1 */
		ret = readl(res.addr + NCLKCTRL1_OFFSET);

		ret |= ((NCLKCTRL1_MAC1TXCEN | NCLKCTRL1_MAC1RXCEN |
		NCLKCTRL1_MAC1ALLCLKEN1 | 1 << 15));
		if ((res.port_interface == ENABLE_SGMII_INTERFACE) ||
			(res.port_interface == ENABLE_2500BASE_X_INTERFACE)) {
			ret &= ~NCLKCTRL1_MAC1125CLKEN1;
			ret &= ~NCLKCTRL1_MAC1312CLKEN1;
		}
		writel(ret, res.addr + NCLKCTRL1_OFFSET);

		/* Interface configuration for port1*/
		ret = readl(res.addr + NEMAC1CTL_OFFSET);
		ret &= ~(NEMACCTL_SP_SEL_MASK | NEMACCTL_PHY_INF_SEL_MASK);
		if ((res.port_interface == ENABLE_RGMII_INTERFACE) ||
			(res.port_interface == ENABLE_RGMII_ID_INTERFACE))
			ret |= NEMACCTL_SP_SEL_RGMII_1000M;
		else if ((res.port_interface == ENABLE_SGMII_INTERFACE) ||
			(res.port_interface == ENABLE_2500BASE_X_INTERFACE))
			ret |= NEMACCTL_SP_SEL_SGMII_2500M;
		else if ((res.port_interface == ENABLE_USXGMII_INTERFACE) || (res.port_interface == ENABLE_USXGMII_10G_INTERFACE) ||
			(res.port_interface == ENABLE_XFI_INTERFACE))
			ret |= NEMACCTL_SP_SEL_USXGMII_10G_10G;
		else if (res.port_interface == ENABLE_USXGMII_5G_INTERFACE)
			ret |= NEMACCTL_SP_SEL_USXGMII_5G_5G;
		else if (res.port_interface == ENABLE_USXGMII_2_5G_INTERFACE)
			ret |= NEMACCTL_SP_SEL_USXGMII_2_5G_2_5G;

		ret &= ~(0x00000040); /* Mask Polarity */
		if (SgmSigPol == 1)
			ret |= 0x00000040; /* Set Active low */

		ret |= NEMACCTL_PHY_INF_SEL | NEMACCTL_LPIHWCLKEN;
		writel(ret, res.addr + NEMAC1CTL_OFFSET);

		/* De-assertion of EMAC Port1  software Reset */
		ret = readl(res.addr + NRSTCTRL1_OFFSET);
		ret &= ~NRSTCTRL1_MAC1RST1;
		writel(ret, res.addr + NRSTCTRL1_OFFSET);
	}


	res.wol_irq = pdev->irq;
	res.irq = pdev->irq;
	res.lpi_irq = pdev->irq;

	plat->bus_id = ((pdev->bus->number<<4) | res.port_num);
	res.pci_bdf = pci_dev_id(pdev);

	dev_dbg(&(pdev->dev), "Port%d Bus%x BDF is 0x%x\n", res.port_num, pdev->bus->number, res.pci_bdf);

	sh_mem_offset = tc956x_get_shared_mem_offset(pdev, pci_dev_id(pdev) & TC956X_PCI_BD_MASK);
	if (sh_mem_offset < TC956X_TOT_CASCADE_DEV)
		res.pci_bd  = sh_mem_offset;
	else {
		dev_err(&(pdev->dev), "Error finding shared memory\n");
		goto err_out_msi_failed;
	}

	ret = stmmac_dvr_probe(&pdev->dev, plat, &res);
	if (ret) {
		if (ret == -ENODEV) {
			dev_dbg(&(pdev->dev), "Port%d Bus%x will be registered as PCIe device only", res.port_num, pdev->bus->number);
			/* Make sure probe() succeeds by returning 0 to caller of probe() */
			ret = 0;
		} else {
			dev_err(&(pdev->dev), "<--%s : ret: %d\n", __func__, ret);
			goto err_dvr_probe;
		}
	}

	if ((res.port_num == RM_PF1_ID) && ((res.port_interface == ENABLE_RGMII_INTERFACE) || (res.port_interface == ENABLE_RGMII_ID_INTERFACE))) {
		writel(0x00000000, res.addr + 0x1050);
		writel(0xF300F300, res.addr + 0x107C);
	}

#ifdef TC956X_PCIE_DSP_CUT_THROUGH
	dev_dbg(&(pdev->dev), "<--%s : Adding DSP Cut Through Settings", __func__);
	/* Read mode setting register
	 * Mode settings values 0:Setting A: x4x1x1, 1:Setting B: x2x2x1
	 */
	val = readl(res.addr + NMODESTS_OFFSET);
	pcie_mode = (val & NMODESTS_MODE2) >> NMODESTS_MODE2_SHIFT;

	switch (pcie_mode) {
	case TC956X_PCIE_SETTING_A: /* 0:Setting A: x4x1x1 mode */
		dev_dbg(&(pdev->dev), "%s : Setting A : Adding DSP Cut Through Settings for DSP1 & DSP2", __func__);
		/*DSP1 & DSP2 is selected*/
		val = readl(res.addr + TC956X_GLUE_SW_REG_ACCESS_CTRL);
		val &= ~(SW_DSP1_ENABLE|SW_DSP2_ENABLE);
		val |= (SW_DSP1_ENABLE|SW_DSP2_ENABLE);
		writel(val, res.addr + TC956X_GLUE_SW_REG_ACCESS_CTRL);
		/*Set 0x0 to Rx Bit enable_cut_through_on_receive_path*/
		val = readl(res.addr + TC956X_SSREG_K_PCICONF_021_021);
		val &= ~(ENABLE_CUT_THROUGH_ON_RX_PATH_MASK);
		writel(val, res.addr + TC956X_SSREG_K_PCICONF_021_021);
		/*Set 0x00000000 to Tx Bit enable_cut_through_on_transmit_path*/
		val = readl(res.addr + TC956X_SSREG_K_PCICONF_022_022);
		val &= ~(ENABLE_CUT_THROUGH_ON_TX_PATH_MASK);
		writel(val, res.addr + TC956X_SSREG_K_PCICONF_022_022);
		break;
	case TC956X_PCIE_SETTING_B: /* 1:Setting B: x2x2x1 mode */
		dev_dbg(&(pdev->dev), "%s : Setting B : Adding DSP Cut Through Settings for DSP2", __func__);
		/*DSP2 is selected*/
		val = readl(res.addr + TC956X_GLUE_SW_REG_ACCESS_CTRL);
		val &= ~(SW_DSP2_ENABLE);
		val |= (SW_DSP2_ENABLE);
		writel(val, res.addr + TC956X_GLUE_SW_REG_ACCESS_CTRL);
		/*Set 0x0 to Rx Bit enable_cut_through_on_receive_path*/
		val = readl(res.addr + TC956X_SSREG_K_PCICONF_021_021);
		val &= ~(ENABLE_CUT_THROUGH_ON_RX_PATH_MASK);
		writel(val, res.addr + TC956X_SSREG_K_PCICONF_021_021);
		/*Set 0x0 to Tx Bit enable_cut_through_on_transmit_path*/
		val = readl(res.addr + TC956X_SSREG_K_PCICONF_022_022);
		val &= ~(ENABLE_CUT_THROUGH_ON_TX_PATH_MASK);
		writel(val, res.addr + TC956X_SSREG_K_PCICONF_022_022);
		break;
	}
#endif /* #ifdef TC956X_PCIE_DSP_CUT_THROUGH */

	/* Increment device usage counter */
	tx956x_pci_shrd_mem[res.pci_bd].pci_dev_active_cnt++;
	tc956xmac_pm_usage_counter++;
	dev_dbg(&(pdev->dev), "%s : Device Usage Count = [%d] probe sequence number : %d\n", __func__, tx956x_pci_shrd_mem[res.pci_bd].pci_dev_active_cnt, res.probe_seq_no);
	dev_dbg(&(pdev->dev), "<--%s\n", __func__);
	mutex_unlock(&tc956x_pm_suspend_lock);

	return ret;


err_dvr_probe:
err_out_msi_failed:
#if defined(TC956X_SRIOV_PF) | defined(TC956X_SRIOV_VF)
	pci_free_irq_vectors(pdev);
#else
	pci_disable_msi(pdev);
#endif
#ifdef DISABLE_EMAC_PORT1
disable_emac_port:
#endif
	if (((void __iomem *)res.tc956x_SFR_pci_base_addr != NULL))
		pci_iounmap(pdev, (void __iomem *)res.tc956x_SFR_pci_base_addr);
	if (((void __iomem *)res.tc956x_SRAM_pci_base_addr != NULL))
		pci_iounmap(pdev, (void __iomem *)res.tc956x_SRAM_pci_base_addr);
	if (((void __iomem *)res.tc956x_BRIDGE_CFG_pci_base_addr != NULL))
		pci_iounmap(pdev, (void __iomem *)res.tc956x_BRIDGE_CFG_pci_base_addr);
err_dvr_logstat:
err_out_map_failed:
#ifdef CONFIG_PCI_IOV
	/* Disable SR-IOV */

	if ((res.sriov_enabled != 0) && pdev->is_physfn) {
		dev_dbg(&(pdev->dev), "Disabling sriov\n");
		res.sriov_enabled = 0;
		pci_disable_sriov(pdev);
	}
#endif
#ifdef CONFIG_PCI_IOV
err_sriov_vf_en_failed:
#endif
	pci_release_regions(pdev);
err_out_req_reg_failed:
	pci_disable_device(pdev);
err_out_enb_failed:
	dev_dbg(&(pdev->dev), "<--%s Error return: %d\n", __func__, ret);
	mutex_unlock(&tc956x_pm_suspend_lock);

	return ret;

}

/**
 * tc956x_pci_remove
 *
 * \brief API to release all the resources from the driver.
 *
 * \details The remove function gets called whenever a device being handled
 * by this driver is removed (either during deregistration of the driver or
 * when it is manually pulled out of a hot-pluggable slot). This function
 * should reverse operations performed at probe time. The remove function
 * always gets called from process context, so it can sleep.
 *
 * \param[in] pdev - pointer to pci_dev structure.
 *
 * \return void
 */
static void tc956xmac_pci_remove(struct pci_dev *pdev)
{
	struct net_device *ndev = dev_get_drvdata(&pdev->dev);
	struct stmmac_priv *priv = netdev_priv(ndev);
	void *nrst_reg, *nclk_reg;
	u32 nrst_val, nclk_val;
	mutex_lock(&tc956x_pm_suspend_lock);

	dev_dbg(&(pdev->dev), "-->%s\n", __func__);

#ifdef CONFIG_PCI_IOV
	/* Disable SR-IOV */
	if ((priv->sriov_enabled != 0) && pdev->is_physfn) {
		dev_dbg(&(pdev->dev), "Disabling sriov\n");
		priv->sriov_enabled = 0;
		pci_disable_sriov(pdev);
	}
#endif
	/* phy_addr == -1 indicates that PHY was not found and
	 * device is registered as only PCIe device. So skip any
	 * ethernet device related uninitialization
	 */
	if (priv->dma_cap.sma_mdio == 1) {
		if (priv->plat->phy_addr != -1)
			stmmac_dvr_remove(&pdev->dev);
	} else {
		stmmac_dvr_remove(&pdev->dev);
	}

	/* Set reset value for CLK control and RESET Control registers */
	if (priv->port_num == 0) {
		nrst_reg = priv->tc956x_SFR_pci_base_addr + NRSTCTRL0_OFFSET;
		nclk_reg = priv->tc956x_SFR_pci_base_addr + NCLKCTRL0_OFFSET;
		nrst_val = readl(nrst_reg);
		nclk_val = readl(nclk_reg);
		nrst_val |= NRSTCTRL0_DEFAULT;
		nclk_val &= ~NCLKCTRL_PORT0_EMAC_MASK;
	} else {
		nrst_reg = priv->tc956x_SFR_pci_base_addr + NRSTCTRL1_OFFSET;
		nclk_reg = priv->tc956x_SFR_pci_base_addr + NCLKCTRL1_OFFSET;
		nrst_val = NRSTCTRL_EMAC_MASK;
		nclk_val = 0;
	}
	writel(nrst_val, nrst_reg);
	writel(nclk_val, nclk_reg);
	if (tx956x_pci_shrd_mem[priv->pci_bd].pci_dev_active_cnt == TC956X_SINGLE_MAC_DEVICE_IN_USE) {
		/* Set reset value for Common CLK control and Common RESET Control registers */
		nrst_reg = priv->tc956x_SFR_pci_base_addr + NRSTCTRL0_OFFSET;
		nclk_reg = priv->tc956x_SFR_pci_base_addr + NCLKCTRL0_OFFSET;
		nrst_val = readl(nrst_reg);
		nclk_val = readl(nclk_reg);
		nrst_val |= NRSTCTRL_COMMON;
		nclk_val |= NCLKCTRL_ENABLE_COMMON_EMAC_MASK;
		nclk_val &= ~NCLKCTRL_DISABLE_COMMON_EMAC_MASK;
		writel(nrst_val, nrst_reg);
		writel(nclk_val, nclk_reg);
	}
	pr_debug("%s : Port %d %s Wr RST Reg:%x, CLK Reg:%x", __func__, priv->port_num, priv->dev->name,
		readl(nrst_reg), readl(nclk_reg));

	pdev->irq = 0;

#if defined(TC956X_SRIOV_PF) | defined(TC956X_SRIOV_VF)
	/* Free allocated interrupt vectors for device */
	pci_free_irq_vectors(pdev);
#else
	/* Disable MSI Operation */
	pci_disable_msi(pdev);
#endif

	if (priv->plat->tc956xmac_clk)
		clk_unregister_fixed_rate(priv->plat->tc956xmac_clk);


	/* Un-map previously mapped BAR0/2/4 address memory */
	if ((void __iomem *)priv->tc956x_SFR_pci_base_addr != NULL)
		pci_iounmap(pdev, (void __iomem *)
			priv->tc956x_SFR_pci_base_addr);
	if ((void __iomem *)priv->tc956x_SRAM_pci_base_addr != NULL)
		pci_iounmap(pdev, (void __iomem *)
			priv->tc956x_SRAM_pci_base_addr);
	if ((void __iomem *)priv->tc956x_BRIDGE_CFG_pci_base_addr != NULL)
		pci_iounmap(pdev, (void __iomem *)
			priv->tc956x_BRIDGE_CFG_pci_base_addr);
	pci_release_regions(pdev);

	pci_disable_device(pdev);

	/* Decrement device usage counter */
	tc956xmac_pm_usage_counter--;
	tx956x_pci_shrd_mem[priv->pci_bd].pci_dev_active_cnt--;
	dev_dbg(&(pdev->dev), "%s : Device Usage Count = [%d] probe sequence number : %d\n", __func__, tx956x_pci_shrd_mem[priv->pci_bd].pci_dev_active_cnt, priv->probe_seq_no);
	dev_dbg(&(pdev->dev), "<--%s\n", __func__);
	mutex_unlock(&tc956x_pm_suspend_lock);
}

/*!
 * \brief API to disable pci device.
 *
 * \details This api will be called during suspend operation.
 * This will disable pci device passed as argument.
 *
 * \param[in] pdev - pointer to pci_dev structure.
 *
 * \return int
 */
static int tc956x_pcie_pm_disable_pci(struct pci_dev *pdev)
{
	struct net_device *ndev = dev_get_drvdata(&pdev->dev);
	struct stmmac_priv *priv = netdev_priv(ndev);
	int ret = 0;

	dev_dbg(&(pdev->dev), "---->%s : Port %d %s - PCI Save State, Disable Device, Prepare to sleep", __func__, priv->port_num, ndev->name);
	pci_save_state(pdev);
	pci_disable_device(pdev);
	pci_prepare_to_sleep(pdev);
	dev_dbg(&(pdev->dev), "<----%s : Port %d %s- PCI Save State, Disable Device, Prepare to sleep", __func__, priv->port_num, ndev->name);
	return ret;
}

/*!
 * \brief API to enable pci device.
 *
 * \details This api will be called during resume operation.
 * This will enable pci device passed as argument.
 *
 * \param[in] pdev - pointer to pci_dev structure.
 *
 * \return int
 */
static int tc956x_pcie_pm_enable_pci(struct pci_dev *pdev)
{
	struct net_device *ndev = dev_get_drvdata(&pdev->dev);
	struct stmmac_priv *priv = netdev_priv(ndev);
	int ret = 0;

	dev_dbg(&(pdev->dev), "---->%s : Port %d %s - PCI Set Power, Enable Device, Restore State & Set Master", __func__, priv->port_num, ndev->name);
	pci_set_power_state(pdev, PCI_D0);
	ret = pci_enable_device_mem(pdev);
	if (ret) {
		dev_err(&(pdev->dev),
		"%s: error in calling pci_enable_device_mem", pci_name(pdev));
		dev_dbg(&(pdev->dev), "<--%s\n", __func__);
		return ret;
	}
	pci_restore_state(pdev);
	pci_set_master(pdev);
	dev_dbg(&(pdev->dev), "<----%s : Port %d %s - PCI Set Power, Enable Device, Restore State & Set Master", __func__, priv->port_num, ndev->name);
	return ret;
}

/*!
 * \brief API to extract child pci devices.
 *
 * \details This api will be called during suspend and resume operation.
 * This will find pci child devices by getting parent device of argument pci device.
 *
 * \param[in] pdev - pointer to pci_dev structure.
 * \param[in] state - identify SUSPEND and RESUME operation.
 *
 * \return int
 */
static int tc956x_pcie_pm_pci(struct pci_dev *pdev, enum TC956X_PORT_PM_STATE state)
{
	static struct pci_dev *tc956x_pd = NULL, *tc956x_dsp_ep = NULL, *tc956x_port_pdev[2] = {NULL};
	struct pci_bus *bus = NULL;
	int ret = 0, i = 0, p = 0;
	struct net_device *ndev = dev_get_drvdata(&pdev->dev);
	struct stmmac_priv *priv = netdev_priv(ndev);

	if (tx956x_pci_shrd_mem[priv->pci_bd].pci_dev_active_cnt == TC956X_ALL_MAC_PORT_SUSPENDED) {
		tc956x_dsp_ep = pci_upstream_bridge(pdev);
		bus = tc956x_dsp_ep->subordinate;

		if (bus)
			list_for_each_entry(tc956x_pd, &bus->devices, bus_list)
		tc956x_port_pdev[i++] = tc956x_pd;

		for (p = 0; ((p < i) && (tc956x_port_pdev[p] != NULL)); p++) {
			/* Enter only if at least 1 Port Suspended */
			if (state == SUSPEND) {
				ret = tc956x_pcie_pm_disable_pci(tc956x_port_pdev[p]);
				if (ret < 0)
					goto err;
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

/*!
 * \brief Routine to put the device in suspend mode
 *
 * \details This function is called whenever pm_generic_suspend() gets invoked.
 * This function invokes stmmac_suspend() to process MAC related suspend
 * operations during PORT_WIDE suspend.
 * This function handles PCI state during SYSTEM_WIDE suspend.
 *
 * \param[in] dev \96 pointer to device structure.
 *
 * \return int
 *
 * \retval 0
 */
static int tc956x_pcie_suspend(struct device *dev)
{
	struct pci_dev *pdev = to_pci_dev(dev);
	struct net_device *ndev = dev_get_drvdata(&pdev->dev);
	struct stmmac_priv *priv = netdev_priv(ndev);
	int ret = 0;

	dev_dbg(&(pdev->dev), "-->%s\n", __func__);
	if (priv->tc956x_port_pm_suspend == true) {
		dev_dbg(&(pdev->dev), "<--%s : Port %d interface %s already Suspended\n", __func__, priv->port_num, priv->dev->name);
		return -1;
	}
	/* Set flag to avoid queuing any more work */
	priv->tc956x_port_pm_suspend = true;

	mutex_lock(&tc956x_pm_suspend_lock);

	/* Decrement device usage counter */
	tx956x_pci_shrd_mem[priv->pci_bd].pci_dev_active_cnt--;
	dev_dbg(&(pdev->dev), "%s : (Number of Ports Left to Suspend = [%d])\n", __func__, tx956x_pci_shrd_mem[priv->pci_bd].pci_dev_active_cnt);

	/* Call stmmac_suspend() */
	stmmac_suspend(&pdev->dev);
	dev_dbg(&(pdev->dev), "%s : Port %d %s- Platform Suspend", __func__, priv->port_num, priv->dev->name);
	ret = tc956x_platform_suspend(priv);
	if (ret) {
		dev_err(&(pdev->dev), "%s: error in calling tc956x_platform_suspend", pci_name(pdev));
		goto err;
	}

	tc956xmac_pm_set_power(priv, SUSPEND);
#ifdef CONFIG_TC956X_MAGIC_PACKET_WOL_GPIO
	if (priv->port_num == RM_PF0_ID) {
		pr_debug("%s: Port %d %s - Configuring GPIO for WOL", __func__, priv->port_num, priv->dev->name);
		tc956x_wol_gpio_trigger(priv->ioaddr, true); /* Set to HIGH */
	}
#endif
	ret = tc956x_pcie_pm_pci(pdev, SUSPEND);
	if (ret < 0)
		goto err;
#ifdef CONFIG_TC956X_MAGIC_PACKET_WOL_CONF
	if (priv->wol_config_enabled == true) {
		/* Set Flag to configure original interface and speed after resume. */
		priv->wol_config_enabled = false; /* Note: QC can place this either at end of suspend or beginning of resume */
		pr_debug("%s Port %d %s : Updated flag priv->wol_config_enabled to %d", __func__, priv->port_num, priv->dev->name, priv->wol_config_enabled);
	}
#endif /* #ifdef CONFIG_TC956X_MAGIC_PACKET_WOL_CONF */
err:
	mutex_unlock(&tc956x_pm_suspend_lock);
	dev_dbg(&(pdev->dev), "<--%s\n", __func__);
	return ret;
}

/*!
 * \brief Routine to configure device during resume
 *
 * \details This function gets called by PCI core when the device is being
 * resumed. It is always called after suspend has been called. These function
 * reverse operations performed at suspend time. This function configure emac
 * port 0, 1 and xpcs to perform MAC realted resume operations.
 *
 * \param[in] pdev pointer to pci device structure.
 *
 * \return s32
 *
 * \retval 0
 */
static int tc956x_pcie_resume_config(struct pci_dev *pdev)
{
	struct net_device *ndev = dev_get_drvdata(&pdev->dev);
	struct stmmac_priv *priv = netdev_priv(ndev);
	/* use signal from MSPHY */
	uint8_t SgmSigPol = 0;
	int ret = 0;

	dev_dbg(&(pdev->dev), "---> %s", __func__);

	/* Skip Config when Port unavailable */
	if (priv->dma_cap.sma_mdio == 1) {
		if ((priv->plat->phy_addr == -1) || (priv->mii == NULL)) {
			dev_dbg(&(pdev->dev), "%s : Invalid PHY Address (%d)\n", __func__, priv->plat->phy_addr);
			ret = -1;
			goto err_phy_addr;
		}
	}

	if (priv->port_num == RM_PF0_ID) {
		ret = readl(priv->tc956x_SFR_pci_base_addr + NRSTCTRL0_OFFSET);

		/* Assertion of EMAC Port0 software Reset */
		ret |= NRSTCTRL0_MAC0RST;

		writel(ret, priv->tc956x_SFR_pci_base_addr + NRSTCTRL0_OFFSET);

		dev_dbg(&pdev->dev, "Enabling all eMAC clocks for Port 0 %s\n", priv->dev->name);
		/* Enable all clocks to eMAC Port0 */
		ret = readl(priv->tc956x_SFR_pci_base_addr + NCLKCTRL0_OFFSET);

		ret |= ((NCLKCTRL0_MAC0TXCEN | NCLKCTRL0_MAC0ALLCLKEN | NCLKCTRL0_MAC0RXCEN));
		if ((priv->port_interface == ENABLE_SGMII_INTERFACE) ||
			(priv->port_interface == ENABLE_2500BASE_X_INTERFACE)) {
			/* Disable Clocks for 2.5Gbps SGMII */
			ret &= ~NCLKCTRL0_POEPLLCEN;
			ret &= ~NCLKCTRL0_SGMPCIEN;
			ret &= ~NCLKCTRL0_REFCLKOCEN;
			ret &= ~NCLKCTRL0_MAC0125CLKEN;
			ret &= ~NCLKCTRL0_MAC0312CLKEN;
		}
		writel(ret, priv->tc956x_SFR_pci_base_addr + NCLKCTRL0_OFFSET);

		/* Interface configuration for port0*/
		ret = readl(priv->tc956x_SFR_pci_base_addr + NEMAC0CTL_OFFSET);
		ret &= ~(NEMACCTL_SP_SEL_MASK | NEMACCTL_PHY_INF_SEL_MASK);
		if ((priv->port_interface == ENABLE_SGMII_INTERFACE) ||
			(priv->port_interface == ENABLE_2500BASE_X_INTERFACE))
			ret |= NEMACCTL_SP_SEL_SGMII_2500M;
		else if ((priv->port_interface == ENABLE_USXGMII_INTERFACE) || (priv->port_interface == ENABLE_USXGMII_10G_INTERFACE) ||
			(priv->port_interface == ENABLE_XFI_INTERFACE))
			ret |= NEMACCTL_SP_SEL_USXGMII_10G_10G;
		else if (priv->port_interface == ENABLE_USXGMII_5G_INTERFACE)
			ret |= NEMACCTL_SP_SEL_USXGMII_5G_5G;
		else if (priv->port_interface == ENABLE_USXGMII_2_5G_INTERFACE)
			ret |= NEMACCTL_SP_SEL_USXGMII_2_5G_2_5G;

		ret &= ~(0x00000040); /* Mask Polarity */
		if (SgmSigPol == 1)
			ret |= 0x00000040; /* Set Active low */

		ret |= NEMACCTL_PHY_INF_SEL | NEMACCTL_LPIHWCLKEN;
		writel(ret, priv->tc956x_SFR_pci_base_addr + NEMAC0CTL_OFFSET);

		/* De-assertion of EMAC Port0  software Reset*/
		ret = readl(priv->tc956x_SFR_pci_base_addr + NRSTCTRL0_OFFSET);
		ret &= ~(NRSTCTRL0_MAC0RST);
		writel(ret, priv->tc956x_SFR_pci_base_addr + NRSTCTRL0_OFFSET);
	}

	if (priv->port_num == RM_PF1_ID) {
		ret = readl(priv->tc956x_SFR_pci_base_addr + NRSTCTRL1_OFFSET);

		/* Assertion of EMAC Port1 software Reset*/
		ret |= NRSTCTRL1_MAC1RST1;
		writel(ret, priv->tc956x_SFR_pci_base_addr + NRSTCTRL1_OFFSET);

		dev_dbg(&pdev->dev, "Enabling all eMAC clocks for Port 1 %s\n", priv->dev->name);
		/* Enable all clocks to eMAC Port1 */
		ret = readl(priv->tc956x_SFR_pci_base_addr + NCLKCTRL1_OFFSET);

		ret |= ((NCLKCTRL1_MAC1TXCEN | NCLKCTRL1_MAC1RXCEN |
		NCLKCTRL1_MAC1ALLCLKEN1 | 1 << 15));
		if ((priv->port_interface == ENABLE_SGMII_INTERFACE) ||
			(priv->port_interface == ENABLE_2500BASE_X_INTERFACE)) {
			ret &= ~NCLKCTRL1_MAC1125CLKEN1;
			ret &= ~NCLKCTRL1_MAC1312CLKEN1;
		}
		writel(ret, priv->tc956x_SFR_pci_base_addr + NCLKCTRL1_OFFSET);

		/* Interface configuration for port1*/
		ret = readl(priv->tc956x_SFR_pci_base_addr + NEMAC1CTL_OFFSET);
		ret &= ~(NEMACCTL_SP_SEL_MASK | NEMACCTL_PHY_INF_SEL_MASK);
		if ((priv->port_interface == ENABLE_RGMII_INTERFACE) ||
			(priv->port_interface == ENABLE_RGMII_ID_INTERFACE))
			ret |= NEMACCTL_SP_SEL_RGMII_1000M;
		else if ((priv->port_interface == ENABLE_SGMII_INTERFACE) ||
			(priv->port_interface == ENABLE_2500BASE_X_INTERFACE))
			ret |= NEMACCTL_SP_SEL_SGMII_2500M;
		else if ((priv->port_interface == ENABLE_USXGMII_INTERFACE) || (priv->port_interface == ENABLE_USXGMII_10G_INTERFACE) ||
			(priv->port_interface == ENABLE_XFI_INTERFACE))
			ret |= NEMACCTL_SP_SEL_USXGMII_10G_10G;
		else if (priv->port_interface == ENABLE_USXGMII_5G_INTERFACE)
			ret |= NEMACCTL_SP_SEL_USXGMII_5G_5G;
		else if (priv->port_interface == ENABLE_USXGMII_2_5G_INTERFACE)
			ret |= NEMACCTL_SP_SEL_USXGMII_2_5G_2_5G;

		ret &= ~(0x00000040); /* Mask Polarity */
		if (SgmSigPol == 1)
			ret |= 0x00000040; /* Set Active low */

		ret |= NEMACCTL_PHY_INF_SEL | NEMACCTL_LPIHWCLKEN;
		writel(ret, priv->tc956x_SFR_pci_base_addr + NEMAC1CTL_OFFSET);

		/* De-assertion of EMAC Port1  software Reset */
		ret = readl(priv->tc956x_SFR_pci_base_addr + NRSTCTRL1_OFFSET);
		ret &= ~NRSTCTRL1_MAC1RST1;
		writel(ret, priv->tc956x_SFR_pci_base_addr + NRSTCTRL1_OFFSET);
	}

/*PMA module init*/
	if (priv->hw->xpcs) {

		if (priv->port_num == RM_PF0_ID) {
			/* Assertion of PMA &  XPCS reset  software Reset*/
			ret = readl(priv->ioaddr + NRSTCTRL0_OFFSET);
			ret |= (NRSTCTRL0_MAC0PMARST | NRSTCTRL0_MAC0PONRST);
			writel(ret, priv->ioaddr + NRSTCTRL0_OFFSET);
		}

		if (priv->port_num == RM_PF1_ID) {
			/* Assertion of PMA &  XPCS reset  software Reset*/
			ret = readl(priv->ioaddr + NRSTCTRL1_OFFSET);
			ret |= (NRSTCTRL1_MAC1PMARST1 | NRSTCTRL1_MAC1PONRST1);
			writel(ret, priv->ioaddr + NRSTCTRL1_OFFSET);
		}

		ret = tc956x_pma_setup(priv, priv->pmaaddr);
		if (ret < 0)
			pr_info("PMA switching to internal clock Failed\n");

		if (priv->port_num == RM_PF0_ID) {
			/* De-assertion of PMA &  XPCS reset  software Reset*/
			ret = readl(priv->ioaddr + NRSTCTRL0_OFFSET);
			ret &= ~(NRSTCTRL0_MAC0PMARST | NRSTCTRL0_MAC0PONRST);
#ifdef EEE_MAC_CONTROLLED_MODE
			ret &= ~(NRSTCTRL0_MAC0RST | NRSTCTRL0_MAC0RST);
#endif
			writel(ret, priv->ioaddr + NRSTCTRL0_OFFSET);
		}

		if (priv->port_num == RM_PF1_ID) {
			/* De-assertion of PMA &  XPCS reset  software Reset*/
			ret = readl(priv->ioaddr + NRSTCTRL1_OFFSET);
			ret &= ~(NRSTCTRL1_MAC1PMARST1 | NRSTCTRL1_MAC1PONRST1);
			writel(ret, priv->ioaddr + NRSTCTRL1_OFFSET);
		}

		if (priv->port_num == RM_PF0_ID) {
			do {
				ret = readl(priv->ioaddr + NEMAC0CTL_OFFSET);
		} while ((NEMACCTL_INIT_DONE & ret) != NEMACCTL_INIT_DONE);
		}

		if (priv->port_num == RM_PF1_ID) {
			do {
				ret = readl(priv->ioaddr + NEMAC1CTL_OFFSET);
		} while ((NEMACCTL_INIT_DONE & ret) != NEMACCTL_INIT_DONE);
		}
		ret = tc956x_xpcs_init(priv, priv->xpcsaddr);
		if (ret < 0)
			pr_info("XPCS initialization error\n");
	}

err_phy_addr:
	dev_dbg(&(pdev->dev), "<--- %s", __func__);
	return ret;
}
/*!
 * \brief Routine to resume device operation
 *
 * \details This function gets called whenever pm_generic_resume() gets invoked.
 * This function reverse operations performed at suspend time. This function restores the
 * power state of the device and restores the PCI config space for SYSTEM_WIDE resume.
 * And it invokes stmmac_resume() to perform MAC realted resume operations
 * for PORT_WIDE resume.
 *
 * \param[in] dev pointer to device structure.
 *
 * \return int
 *
 * \retval 0
 */

static int tc956x_pcie_resume(struct device *dev)
{
	struct pci_dev *pdev = to_pci_dev(dev);
	struct net_device *ndev = dev_get_drvdata(&pdev->dev);
	struct stmmac_priv *priv = netdev_priv(ndev);
	int ret = 0;

	dev_dbg(&(pdev->dev), "-->%s\n", __func__);
	if (priv->tc956x_port_pm_suspend == false) {
		dev_dbg(&(pdev->dev), "%s : Port %d %s already Resumed\n", __func__, priv->port_num, priv->dev->name);
		return -1;
	}
	mutex_lock(&tc956x_pm_suspend_lock);

	ret = tc956x_pcie_pm_enable_pci(pdev);
	if (ret < 0)
		goto err;

	tc956xmac_pm_set_power(priv, RESUME);

	/* Restore the GPIO settings which was saved during GPIO configuration */
	ret = tc956x_gpio_restore_configuration(priv);
	if (ret < 0)
		pr_info("GPIO configuration restoration failed\n");

	dev_dbg(&(pdev->dev), "%s : Port %d %s - Platform Resume", __func__, priv->port_num, priv->dev->name);
	ret = tc956x_platform_resume(priv);
	if (ret) {
		dev_err(&(pdev->dev), "%s: error in calling tc956x_platform_resume", pci_name(pdev));
		pci_disable_device(pdev);
		goto err;
	}

	/* Configure TA map registers */

	if (tx956x_pci_shrd_mem[priv->pci_bd].pci_dev_active_cnt == TC956X_ALL_MAC_PORT_SUSPENDED) {
		dev_dbg(&(pdev->dev), "%s : Tamap Re-configuration", __func__);
		tc956x_config_tamap(&pdev->dev, priv->tc956x_BRIDGE_CFG_pci_base_addr);
	}

	/* Configure EMAC Port */
	tc956x_pcie_resume_config(pdev);

	/* Call stmmac_resume() */
	stmmac_resume(&pdev->dev);
	if ((priv->port_num == RM_PF1_ID) && ((priv->port_interface == ENABLE_RGMII_INTERFACE) || (priv->port_interface == ENABLE_RGMII_ID_INTERFACE))) {
		writel(NEMACTXCDLY_DEFAULT, priv->ioaddr + TC9563_CFG_NEMACTXCDLY);
		writel(NEMACIOCTL_DEFAULT, priv->ioaddr + TC9563_CFG_NEMACIOCTL);
	}

	/* Increment device usage counter */
	tx956x_pci_shrd_mem[priv->pci_bd].pci_dev_active_cnt++;
	dev_dbg(&(pdev->dev), "%s : (Number of Ports Resumed = [%d])\n", __func__, tx956x_pci_shrd_mem[priv->pci_bd].pci_dev_active_cnt);

	priv->tc956x_port_pm_suspend = false;

	/* Queue Work after resume complete to prevent MSI Disable */
	if (priv->tc956xmac_pm_wol_interrupt) {
		dev_dbg(&(pdev->dev), "%s : Clearing WOL and queuing phy work", __func__);
		/* Clear WOL Interrupt after resume, if WOL enabled */
		priv->tc956xmac_pm_wol_interrupt = false;
		/* Queue the work in system_wq */
		queue_work(system_wq, &priv->emac_phy_work);
	}

#ifdef CONFIG_TC956X_MAGIC_PACKET_WOL_GPIO
	if (priv->port_num == RM_PF0_ID) {
		pr_debug("%s: Port %d - Configuring GPIO for WOL", __func__, priv->port_num);
		tc956x_wol_gpio_trigger(priv->ioaddr, false); /* Set to LOW */
	}
#endif
	if (priv->port_num == RM_PF0_ID) {
		if ((tc956x_logstat_set_state_log_enable((void __iomem *)priv->ioaddr, UPSTREAM_PORT, STATE_LOG_ENABLE) < 0)
			|| (tc956x_logstat_set_state_log_enable((void __iomem *)priv->ioaddr, DOWNSTREAM_PORT1, STATE_LOG_ENABLE) < 0)
			|| (tc956x_logstat_set_state_log_enable((void __iomem *)priv->ioaddr, DOWNSTREAM_PORT2, STATE_LOG_ENABLE) < 0)
			|| (tc956x_logstat_set_state_log_enable((void __iomem *)priv->ioaddr, INTERNAL_ENDPOINT, STATE_LOG_ENABLE) < 0)) {
			ret = -EFAULT; /* The returns returned by above function are -EFAULT only */
			dev_err(&(pdev->dev),
			"%s: error in calling tc956x_logstat_set_state_log_enable", pci_name(pdev));
			dev_dbg(&(pdev->dev), "<--%s : Error ret: %d\n", __func__, ret);
			goto err_resume_logstat;
		}
	}
	dev_dbg(&(pdev->dev), "<--%s\n", __func__);
err_resume_logstat:
err:
	mutex_unlock(&tc956x_pm_suspend_lock);
	dev_dbg(&(pdev->dev), "<--%s\n", __func__);

	return ret;
}

#ifdef CONFIG_TC956X_MAGIC_PACKET_WOL_GPIO
/*!
 * \brief API to signal SUSPEND/RESUME to external Host Triggering Device.
 *
 * \details This is a api to configure GPIO04 and set its output value
 * based on mode to (HIGH/LOW) passed as argument.
 *
 * \param[in] reg_base_addr - pointer to BAR 4 base address.
 * \param[in] mode - true or false
 */
static void tc956x_wol_gpio_trigger(void __iomem *reg_base_addr, bool mode)
{
	u32 reg;

	pr_debug("-->%s\n", __func__);
	/* Set GPIO to Function.0*/
	reg = readl(reg_base_addr + NFUNCEN4_OFFSET);
	reg &= (~(BIT(4)));
	writel(reg, reg_base_addr + NFUNCEN4_OFFSET);
	pr_debug("%s: Setting GPIO04 to Function 0 (%x:%x)\n", __func__,
		NFUNCEN4_OFFSET, readl(reg_base_addr + NFUNCEN4_OFFSET));

	/* GPIO04:OUT Enable*/
	reg = readl(reg_base_addr + GPIOE0_OFFSET);
	reg = (reg & ~(BIT(4)));
	writel(reg, reg_base_addr + GPIOE0_OFFSET);
	pr_debug("%s: Setting GPIO04 Direction to Out Enable (%x:%x)\n", __func__,
		GPIOE0_OFFSET, readl(reg_base_addr + GPIOE0_OFFSET));

	reg = readl(reg_base_addr + GPIOO0_OFFSET);
	if (!mode) { /* Set GPIO04 to LOW */
		pr_debug("%s: Setting GPIO04 to LOW\n", __func__);
		reg &= (~(BIT(4)));
	} else { /* Set GPIO04 to HIGH */
		pr_debug("%s: Setting GPIO04 to HIGH\n", __func__);
		reg |= (BIT(4));
	}
	writel(reg, reg_base_addr + GPIOO0_OFFSET);
	pr_debug("%s: Setting GPIO04 Value (%x:%x)\n", __func__,
		GPIOO0_OFFSET, readl(reg_base_addr + GPIOO0_OFFSET));
	pr_debug("<--%s\n", __func__);
}
#endif /* #ifdef CONFIG_TC956X_MAGIC_PACKET_WOL_GPIO */
/*!
 * \brief API to shutdown the device.
 *
 * \details This is a dummy implementation for the shutdown feature of the
 * pci_driver structure.
 *
 * \param[in] pdev - pointer to pci_dev structure.
 */
static void tc956x_pcie_shutdown(struct pci_dev *pdev)
{

	dev_dbg(&(pdev->dev), "-->%s\n", __func__);
	dev_alert(&(pdev->dev), "Handle the shutdown\n");
	dev_dbg(&(pdev->dev), "<--%s\n", __func__);
}

/**
 * tc956x_pcie_error_detected
 *
 * \brief Function is called when PCI AER kernel module detects an error.
 *
 * \details This is a dummy implementation for the callback registration
 *
 * \param[in] pdev - pointer to pci_dev structure.
 *
 * \param[in] state - PCI error state.
 *
 * \return Error recovery state
 */
static pci_ers_result_t tc956x_pcie_error_detected(struct pci_dev *pdev,
						pci_channel_state_t state)
{
	dev_err(&(pdev->dev), "PCI AER Error detected : %d\n", state);

	/* No further error recovery to be carried out */
	return PCI_ERS_RESULT_DISCONNECT;
}

/**
 * tc956x_pcie_slot_reset
 *
 * \brief Function is called when PCI AER kernel module issues an slot reset.
 *
 * \details This is a dummy implementation for the callback registration
 *
 * \param[in] pdev - pointer to pci_dev structure.
 *
 * \return Error recovery state
 */
static pci_ers_result_t tc956x_pcie_slot_reset(struct pci_dev *pdev)
{
	dev_err(&(pdev->dev), "PCI AER Slot reset Invoked\n");

	/* No further error recovery to be carried out */
	return PCI_ERS_RESULT_DISCONNECT;
}


/**
 * tc956x_pcie_io_resume
 *
 * \brief Function is called when PCI AER kernel module requests for
 *	  device to resume.
 *
 * \details This is a dummy implementation for the callback registration
 *
 * \param[in] pdev - pointer to pci_dev structure.
 *
 * \return void
 */
static void tc956x_pcie_io_resume(struct pci_dev *pdev)
{
	dev_err(&(pdev->dev), "PCI AER Resume Invoked\n");
}

/* PCI AER Error handlers */
static struct pci_error_handlers tc956x_err_handler = {
	.error_detected = tc956x_pcie_error_detected,
	.slot_reset = tc956x_pcie_slot_reset,
	.resume = tc956x_pcie_io_resume,
};

/* synthetic ID, no official vendor */
#define PCI_VENDOR_ID_TC956XMAC 0x700

#define TC956XMAC_QUARK_ID  0x0937
#define TC956XMAC_DEVICE_ID 0x1108
#define TC956XMAC_EHL_RGMII1G_ID	0x4b30
#define TC956XMAC_EHL_SGMII1G_ID	0x4b31
#define TC956XMAC_TGL_SGMII1G_ID	0xa0ac
#define TC956XMAC_GMAC5_ID		0x7102
#define TC956XMAC_XGMAC3_10G	0x7203
#define TC956XMAC_XGMAC3_2_5G	0x7207
#define TC956XMAC_XGMAC3_2_5G_MDIO	0x7211

#define TC956XMAC_DEVICE(vendor_id, dev_id, info)	{	\
	PCI_VDEVICE(vendor_id, dev_id),			\
	.driver_data = (kernel_ulong_t)&info		\
	}

static const struct pci_device_id tc956xmac_id_table[] = {
	TC956XMAC_DEVICE(TOSHIBA, DEVICE_ID, tc956xmac_xgmac3_pci_info),
	{}
};
MODULE_DEVICE_TABLE(pci, tc956xmac_id_table);

static SIMPLE_DEV_PM_OPS(tc956xmac_pm_ops, tc956x_pcie_suspend, tc956x_pcie_resume);

static struct pci_driver tc956xmac_pci_driver = {
	.name = DRIVER_NAME,
	.id_table = tc956xmac_id_table,
	.probe = tc956xmac_pci_probe,
	.remove = tc956xmac_pci_remove,
	.shutdown	= tc956x_pcie_shutdown,
	.driver		= {
		.name		= DRIVER_NAME,
		.owner		= THIS_MODULE,
		.pm		= &tc956xmac_pm_ops,
	},
	.err_handler = &tc956x_err_handler
};


/*!
 * \brief API to register the driver.
 *
 * \details This is the first function called when the driver is loaded.
 * It register the driver with PCI sub-system
 *
 * \return void.
 */
static s32 __init tc956x_init_module(void)
{
	s32 ret = 0;

	pr_debug("-->%s", __func__);
	ret = pci_register_driver(&tc956xmac_pci_driver);
	if (ret) {
		pr_info("TC956X : Driver registration failed");
		return ret;
	}

	pr_debug("<--%s", __func__);
	return ret;
}

/*!
 * \brief API to unregister the driver.
 *
 * \details This is the first function called when the driver is removed.
 * It unregister the driver from PCI sub-system
 *
 * \return void.
 */
static void __exit tc956x_exit_module(void)
{
	pr_debug("%s", __func__);
	pci_unregister_driver(&tc956xmac_pci_driver);
	pr_debug("%s", __func__);
}

/*!
 * \brief Macro to register the driver registration function.
 *
 * \details A module always begin with either the init_module or the function
 * you specify with module_init call. This is the entry function for modules;
 * it tells the kernel what functionality the module provides and sets up the
 * kernel to run the module's functions when they're needed. Once it does this,
 * entry function returns and the module does nothing until the kernel wants
 * to do something with the code that the module provides.
 */
module_init(tc956x_init_module);

/*!
 * \brief Macro to register the driver un-registration function.
 *
 * \details All modules end by calling either cleanup_module or the function
 * you specify with the module_exit call. This is the exit function for modules;
 * it undoes whatever entry function did. It unregisters the functionality
 * that the entry function registered.
 */
module_exit(tc956x_exit_module);
#ifdef CONFIG_PCI_IOV
/* Input parameter for No of virtural functions to Enable per VF.
 * tc956x_no_of_vf - Valid vlaues are 0 to 3.
 */
module_param(tc956x_no_of_vf, int, MOD_PARAM_ACCESS);
#endif



module_param(mac0_tx_pbl, int, 0444);
MODULE_PARM_DESC(mac0_tx_pbl,
		"Port-0 Transmit Programmable Burst Length, default is 16,\
		Supports following values: 1, 2, 4, 8, 16, or 32.");

module_param(mac0_rx_pbl, int, 0444);
MODULE_PARM_DESC(mac0_rx_pbl,
		"Port-0 Receive Programmable Burst Length, default is 16,\
		Supports following values: 1, 2, 4, 8, 16, or 32.");

module_param(mac1_tx_pbl, int, 0444);
MODULE_PARM_DESC(mac1_tx_pbl,
		"Port-1 Transmit Programmable Burst Length, default is 16,\
		Supports following values: 1, 2, 4, 8, 16, or 32.");

module_param(mac1_rx_pbl, int, 0444);
MODULE_PARM_DESC(mac1_rx_pbl,
		"Port-1 Receive Programmable Burst Length, default is 16,\
		Supports following values: 1, 2, 4, 8, 16, or 32.");

module_param(mac0_axi_wr_osr_lmt, uint, 0444);
MODULE_PARM_DESC(mac0_axi_wr_osr_lmt,
		"Port-0 AXI Maximum Write Outstanding Request Limit");

module_param(mac0_axi_rd_osr_lmt, uint, 0444);
MODULE_PARM_DESC(mac0_axi_rd_osr_lmt,
		"Port-0 AXI Maximum Read Outstanding Request Limit");

module_param(mac1_axi_wr_osr_lmt, uint, 0444);
MODULE_PARM_DESC(mac1_axi_wr_osr_lmt,
		"Port-1 AXI Maximum Write Outstanding Request Limit");

module_param(mac1_axi_rd_osr_lmt, uint, 0444);
MODULE_PARM_DESC(mac1_axi_rd_osr_lmt,
		"Port-1 AXI Maximum Read Outstanding Request Limit");

module_param(mac0_axi_blen, uint, 0444);
MODULE_PARM_DESC(mac0_axi_blen,
		"Port-0 AXI DMA Burst Length\
		Supported values: 4,8,16,32,64,128,256");

module_param(mac1_axi_blen, uint, 0444);
MODULE_PARM_DESC(mac1_axi_blen,
		"Port-1 AXI DMA Burst Length\
		Supported values: 4,8,16,32,64,128,256");

/* From here Module params are supported in array format */
module_param_array(tc956x_eth_ports_bdf, uint, NULL, 0444);
MODULE_PARM_DESC(tc956x_eth_ports_bdf,
		"Array of BDFs (Bus number, Device number and Function number) for which interface configuration is required, default is 0 (None)\
		which means other associated array module parameters will assign their default values to the TC956x devices in cascade setup\
		Supported format: 0xBBDF, 'BB': one byte of Bus number, 'DF': one byte of Slot/Device number and Function number encoded as\
		[7:3] bits for Slot number and [2:0] bits for Function number\
		This is array module parameter in which maximum of 14 BDFs can be provided in comma seperated format.\
		Note that this is a mandatory parameter to associate other array module parameters with particular TC956x device in a cascade setup");

module_param_array(macX_interface, uint, NULL, 0444);
MODULE_PARM_DESC(macX_interface,
		"Array of MAC Interface arranged in order according to the BDFs provided in module parameter 'tc956x_eth_ports_bdf'\
		Following are supported values according to the port.\
		PORTX interface supported values, default is 1 (XFI) for Port0 and 4 (SGMII) for Port1\
		[0: USXGMII, 1: XFI, 2: RGMII*, 3: RGMII_ID*, 4: SGMII, 5: 2500Base-X, 6: USXGMII_10G, 7: USXGMII_5G, 8: USXGMII_2.5G]\
		* - Not supported for Port0 or Function0.\
		This is array module parameter in which maximum of 14 interface values can be provided in comma seperated format");

module_param_array(portX_mdc, uint, NULL, 0444);
MODULE_PARM_DESC(portX_mdc,
		"Array of MDC values arranged in order according to the BDFs provided in module parameter 'tc956x_eth_ports_bdf'\
		PORTX MDC clock setting supported values - default is 0x4 (clk_csr_i/12) for all Port0 and 0x8 (clk_csr_i/62) for all Port1,\
		[0x0 - clk_csr_i/4,\
		0x1 - clk_csr_i/6,\
		0x2 - clk_csr_i/8,\
		0x3 - clk_csr_i/10,\
		0x4 - clk_csr_i/12,\
		0x5 - clk_csr_i/14,\
		0x6 - clk_csr_i/16,\
		0x7 - clk_csr_i/18,\
		0x8 - clk_csr_i/62,\
		0x9 - clk_csr_i/102,\
		0xA - clk_csr_i/122,\
		0xB - clk_csr_i/142,\
		0xC - clk_csr_i/162,\
		0xD - clk_csr_i/202]\
		Note: Select the value based on the above mentioned MDIO clock settings\
		This is array module parameter in which maximum of 14 MDC values can be provided in comma seperated format");


module_param_array(portX_c45_state, uint, NULL, 0444);
MODULE_PARM_DESC(portX_c45_state,
		"Array of C45 state values arranged in order according to the BDFs provided in module parameter 'tc956x_eth_ports_bdf'\
		PORTX phy driver clause setting - default is 1 (true) for all Port0 and 0 (false) for all Port1,\
		Supported values: [1 - true, 0 - false]\
		This is array module parameter in which maximum of 14 C45 state can be provided in comma seperated format");


module_param_array(portX_phyaddr, uint, NULL, 0444);
MODULE_PARM_DESC(portX_phyaddr,
		"Array of Phy device addr arranged in order according to the BDFs provided in module parameter 'tc956x_eth_ports_bdf'\
		PORT0 Phy device addr for phy detection, default is 0 for both Port0 and Port1,\
		Supported values are [0 to 31]\
		This is array module parameter in which maximum of 14 Phy device addresses can be provided in comma seperated format");

module_param_array(macX_link_down_macrst, uint, NULL, 0444);
MODULE_PARM_DESC(macX_link_down_macrst,
		"Array of MAC Link down reset setting in order according to the BDFs provided in module parameter 'tc956x_eth_ports_bdf'\
		MAC reset for PHY Clock loss during Link Down - default is 1 (ENABLE) for all Port0 and 0 (DISABLE) for all Port1,\
		Supported values [0: DISABLE, 1: ENABLE]\
		This is array module parameter in which maximum of 14 MAC link down reset state can be provided in comma seperated format");

module_param_array(macX_no_mdio_no_phy, uint, NULL, 0444);
MODULE_PARM_DESC(macX_no_mdio_no_phy,
	"Array of PHY and MDIO configuration in order according to the BDFs provided in module parameter 'tc956x_eth_ports_bdf'\
	PHY and MDIO configuration - default is 0 (PHY ON and MDIO ON) for both Port0 and Port1,\
	Supported values [0: PHY ON and MDIO ON, 1: PHY ON and MDIO OFF*, 2: PHY OFF and MDIO ON*, 3: PHY OFF and MDIO OFF]\
	* - These modes are not supported in current version\
	This is array module parameter in which maximum of 14 PHY and MDIO configuration state can be provided in comma seperated format");

module_param_array(macX_rxq0_size, uint, NULL, 0444);
MODULE_PARM_DESC(macX_rxq0_size,
		"Array of Rx Queue-0 arranged in order according to the BDFs provided in module parameter 'tc956x_eth_ports_bdf'\
		 Rx Queue-0 size of BDfs provided - default is 18432 (bytes),\
		 [Range Supported : 3072..44032 (bytes)]");

module_param_array(macX_txq0_size, uint, NULL, 0444);
MODULE_PARM_DESC(macX_txq0_size,
		"Array of Tx Queue-0 arranged in order according to the BDFs provided in module parameter 'tc956x_eth_ports_bdf'\
		 Tx Queue-0 size of BDfs provided - default is 18432 (bytes),\
		 [Range Supported : 3072..44032 (bytes)]");

module_param_array(macX_rxq1_size, uint, NULL, 0444);
#ifdef TC956X_CPE_CONFIG
MODULE_PARM_DESC(macX_rxq1_size,
		"Array of Rx Queue-1 arranged in order according to the BDFs provided in module parameter 'tc956x_eth_ports_bdf'\
		 Rx Queue-1 size of BDfs provided - default is 18432 (bytes),\
		 [Range Supported : 3072..44032 (bytes)]");
#else
MODULE_PARM_DESC(macX_rxq1_size,
		"Array of Rx Queue-1 arranged in order according to the BDFs provided in module parameter 'tc956x_eth_ports_bdf'\
		 Rx Queue-1 size of BDfs provided - default is 4096 (bytes),\
		 [Range Supported : 3072..44032 (bytes)]");
#endif

module_param_array(macX_txq1_size, uint, NULL, 0444);
#ifdef TC956X_CPE_CONFIG
MODULE_PARM_DESC(macX_txq1_size,
		"Array of Tx Queue-1 arranged in order according to the BDFs provided in module parameter 'tc956x_eth_ports_bdf'\
		 Tx Queue-1 size of BDfs provided - default is 18432 (bytes),\
		 [Range Supported : 3072..44032 (bytes)]");
#else
MODULE_PARM_DESC(macX_txq1_size,
		"Array of Tx Queue-1 arranged in order according to the BDFs provided in module parameter 'tc956x_eth_ports_bdf'\
		 Tx Queue-1 size of BDfs provided - default is 14336 (bytes),\
		 [Range Supported : 3072..44032 (bytes)]");
#endif

module_param_array(macX_rxq0_rfd, uint, NULL, 0444);
MODULE_PARM_DESC(macX_rxq0_rfd,
		"Array of Flow control thresholds for Rx Queue-0 arranged in order according to the BDFs provided in module parameter 'tc956x_eth_ports_bdf'\
		 Flow control thresholds for Rx Queue-0 of BDfs provided\
		 for disable - default is 24 (13KB)\
		 [Range Supported : 0..84]");

module_param_array(macX_rxq1_rfd, uint, NULL, 0444);
MODULE_PARM_DESC(macX_rxq1_rfd,
		"Array of Flow control thresholds for Rx Queue-1 arranged in order according to the BDFs provided in module parameter 'tc956x_eth_ports_bdf'\
		 Flow control thresholds for Rx Queue-1 of BDfs provided\
		 for disable - default is 24 (13KB)\
		 [Range Supported : 0..84]");

module_param_array(macX_rxq0_rfa, uint, NULL, 0444);
MODULE_PARM_DESC(macX_rxq0_rfa,
		"Array of Flow control thresholds for Rx Queue-0 arranged in order according to the BDFs provided in module parameter 'tc956x_eth_ports_bdf'\
		 Flow control thresholds for Rx Queue-0 of BDfs provided\
		 for enable - default is 24 (13KB)\
		 [Range Supported : 0..84]");

module_param_array(macX_rxq1_rfa, uint, NULL, 0444);
MODULE_PARM_DESC(macX_rxq1_rfa,
		"Array of Flow control thresholds for Rx Queue-1 arranged in order according to the BDFs provided in module parameter 'tc956x_eth_ports_bdf'\
		 Flow control thresholds for Rx Queue-1 of BDfs provided\
		 for enable - default is 24 (13KB)\
		 [Range Supported : 0..84]");

module_param_array(macX_eee_enable, uint, NULL, 0444);
MODULE_PARM_DESC(macX_eee_enable,
		"Array of Enable/Disable EEE arranged in order according to the BDFs provided in module parameter 'tc956x_eth_ports_bdf'\
		 Enable/Disable EEE for BDfs provided - default is 0,\
		 [0 : DISABLE, 1 : ENABLE]");

module_param_array(macX_lpi_timer, uint, NULL, 0444);
MODULE_PARM_DESC(macX_lpi_timer,
		"Array of LPI Automatic Entry Timer arranged in order according to the BDFs provided in module parameter 'tc956x_eth_ports_bdf'\
		 LPI Automatic Entry Timer for BDfs provided - default is 600 (us),\
		 [Range Supported : 0..1048568 (us)]");

module_param_array(macX_filter_phy_pause, uint, NULL, 0444);
MODULE_PARM_DESC(macX_filter_phy_pause,
		"Array of Filter PHY pause frames arranged in order according to the BDFs provided in module parameter 'tc956x_eth_ports_bdf'\
		 Filter PHY pause frames alone and pass Link partner pause frames\
		 to application for BDfs provided - default is 0,\
		 [0 : DISABLE, 1 : ENABLE]");

module_param_array(macX_en_lp_pause_frame_cnt, uint, NULL, 0444);
MODULE_PARM_DESC(macX_en_lp_pause_frame_cnt,
		"Array of Enable counter to count Link Partner pause frames arranged in order according to the BDFs provided in module parameter 'tc956x_eth_ports_bdf'\
		 Enable counter to count Link Partner pause frames for BDfs provided - default is 0,\
		 [0 : DISABLE, 1 : ENABLE]");

module_param_array(macX_force_speed_mode, uint, NULL, 0444);
MODULE_PARM_DESC(mac0_force_speed_mode,
		"Array of Enable force speed mode arranged in order according to the BDFs provided in module parameter 'tc956x_eth_ports_bdf'\
		 Enable force speed mode for BDfs provided - default is 0,\
		 [0 : DISABLE, 1 : ENABLE]");

module_param_array(macX_force_config_speed, uint, NULL, 0444);
MODULE_PARM_DESC(macX_force_config_speed,
		"Array of Configure force speed arranged in order according to the BDFs provided in module parameter 'tc956x_eth_ports_bdf'\
		 Configure force speed for BDfs provided - default is 3,\
		 [0 : 10G, 1 : 5G, 2 : 2.5G, 3 : 1G, 4 : 100M, 5 : 10M]");

module_param_array(macX_power_save_at_link_down, uint, NULL, 0444);
MODULE_PARM_DESC(macX_power_save_at_link_down,
		"Array of Enable Power saving during Link down arranged in order according to the BDFs provided in module parameter 'tc956x_eth_ports_bdf'\
		 Same value to be assigned for Port-0 and Port-1 of a TC956x device - default is 0\
		 Note: If Port-0 and Port-1 have different values, power saving is not gauranteed\
		 [0 : DISABLE, 1 : ENABLE]");

#ifdef TC956X_PCIE_LINK_STATE_LATENCY_CTRL

module_param_array(epX_l0s_delay, uint, NULL, 0444);
MODULE_PARM_DESC(epX_l0s_delay,
		"Array of L0s Link state change delay arranged in order according to the BDFs provided in module parameter 'tc956x_eth_ports_bdf'\
		L0s Link state change delay configuration for\
		Internal Endpoint, Same value to be assigned for Port-0 and Port-1 - default is 31\
		Range: 1-31");

module_param_array(epX_l1_delay, uint, NULL, 0444);
MODULE_PARM_DESC(epX_l1_delay,
		"Array of L1 Link state change delay arranged in order according to the BDFs provided in module parameter 'tc956x_eth_ports_bdf'\
		L1 Link state change delay configuration for\
		Internal Endpoint, Same value to be assigned for Port-0 and Port-1- default is 1023\
		Range: 1-1023");

module_param_array(uspX_l0s_delay, uint, NULL, 0444);
MODULE_PARM_DESC(uspX_l0s_delay,
		"Array of L0s Link state change delay arranged in order according to the BDFs provided in module parameter 'tc956x_eth_ports_bdf'\
		L0s Link state change delay configuration for\
		Upstream Port, Same value to be assigned for Port-0 and Port-1 - default is 31\
		Range: 1-31");

module_param_array(uspX_l1_delay, uint, NULL, 0444);
MODULE_PARM_DESC(uspX_l1_delay,
		"Array of L1 Link state change delay arranged in order according to the BDFs provided in module parameter 'tc956x_eth_ports_bdf'\
		L1 Link state change delay configuration for\
		Upstream Port, Same value to be assigned for Port-0 and Port-1 - default is 1023\
		Range: 1-1023");

#endif

MODULE_DESCRIPTION("TC956X PCI Express Ethernet Network Driver");
MODULE_AUTHOR("Toshiba Electronic Devices & Storage Corporation");
MODULE_LICENSE("GPL v2");
//MODULE_VERSION(DRV_MODULE_VERSION);
