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
#include <linux/bitops.h>
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
#include <linux/types.h>
#include <linux/of_irq.h>
#include <linux/delay.h>

#include "common.h"
#include "dwmac-tc956x-reset.h"
#include "dwxgmac2.h"
#include "stmmac.h"
#include "stmmac_libpci.h"

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

/* These values are bit positions in struct tc956x_data->mac_state */
enum tc956x_mac_state {
	MAC_STATE_125_CLOCK,			/* set: enabled; clear: not */
	MAC_STATE_312_5_CLOCK,
	MAC_STATE_TX_CLOCK,
	MAC_STATE_RX_CLOCK,
	MAC_STATE_ALL_CLOCK,
	MAC_STATE_RGMII_CLOCK,

	MAC_STATE_COUNT,			/* Not a state */
};

/* XXX MCU, SRAM, PCIE, and I2C are always enabled/disabled together */
/* XXX MSIGEN and UART0 are always enabled/disabled together */
enum tc9564_chip_clock_id {
	CHIP_CLOCK_MCU		= 0,
	CHIP_CLOCK_SRAM		= 13,
	CHIP_CLOCK_PCIE		= 9,
	CHIP_CLOCK_I2C		= 12,
	CHIP_CLOCK_MSIGEN	= 18,
	CHIP_CLOCK_INTC		= 4,
	CHIP_CLOCK_UART0	= 16,
};

/* XXX MAC_TX, MAC_RX, and MAC_ALL are always enabled/disabled together */
/* XXX MAC_125M and MAC_312_5M are always enabled/disabled together */
/* XXX MAC_PLL, MAC_SGMII, and REFCLK are always enabled/disabled together */
enum tc9564_mac_clock_id {
	MAC_CLOCK_TX		= 7,
	MAC_CLOCK_RX		= 14,
	MAC_CLOCK_ALL		= 31,
	MAC_CLOCK_125M		= 29,
	MAC_CLOCK_312_5M	= 30,
	/* eMAC 0 only */		/* XXX Are these really MAC clocks? */
	MAC_CLOCK_PLL		= 24,
	MAC_CLOCK_SGMII		= 25,
	MAC_CLOCK_REFCLK	= 26,	/* XXX Probably CHIP_CLOCK_REFCLK */
	/* eMAC 1 only */
	MAC_CLOCK_RMII		= 15,
};

/**
 * struct tc956x_data - Toshiba-specific platform data
 * @dev:		Device pointer
 * @revid:		Revision ID (as read from the chipcfg registers)
 * @devfn:		PCI device/function id
 * @plat:		Pointer to our stmmac platform data
 * @bridge_config:	Mapped bridge config data (BAR 0)
 * @sfr:		Mapped SFR region (BAR 4)
 * @emac0:		Which eMAC port this is (true: port 0; false: port 1)
 * @pinctrl:		Pin control structure
 * @pinctrl_default:	Pin control default value
 * @phy_supply:		PHY supply egulator
 * @phy_reset:		Descriptor for GPIO used for PHY reset
 * @phy_reset_delay:	Delay (milliseconds) after PHY reset
 * @reset_asserted:	Whether reset on this PHY is currently asserted
 * @wol_irq:		Wake-on-LAN IRQ number
 * @chip:		Pointer to the containing chip information
 * @regmap:		Register map for SFR region access
 * @mac_reset:		MAC reset control
 * @pma_reset:		PMA reset control
 * @xpcs_reset:		XPCS reset control
 * @mac_state:		Bitmap tracking state of resets
 * @clock_offset:	Offset to clock control register
 */
struct tc956x_data {
	struct device *dev;
	u32 revid;
	unsigned int devfn;
	struct plat_stmmacenet_data *plat;
	void __iomem *bridge_config;
	void __iomem *sfr;
	bool emac0;
	struct pinctrl *pinctrl;
	struct pinctrl_state *pinctrl_default;
	struct regulator *phy_supply;
	struct gpio_desc *phy_reset;
	u32 phy_reset_delay;
	u32 reset_asserted;
	int wol_irq;
	struct tc956x_chip *chip;
	struct regmap *regmap;
	struct reset_control *mac_reset;
	struct reset_control *pma_reset;
	struct reset_control *xpcs_reset;
	u32 clock_offset;
	DECLARE_BITMAP(mac_state, MAC_STATE_COUNT);
};

/**
 * struct tc956x_chip - Common chip support information
 * @pci_bus_num:	PCI bus this chip is on
 * @pci_slot:		PCI slot on its bus this chip fills
 * @primary:		Data pointer for the primary eMAC interface
 * @secondary:		Device link between secondary (consumer) and primary
 * @gpio:		Pointer to GPIO information
 * @reset_clock_regmap:	Register map used for clocks and resets
 * @mcu_reset:		MCU reset control
 * @mcu1_reset:		MCU1 reset control
 * @intr_reset:		INTC reset control
 * @uart0_reset:	UART0 reset control
 * @msigen_reset:	MSIGEN reset control
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
	struct device_link *secondary;

	struct regmap *reset_clock_regmap;

	struct reset_control *mcu_reset;
	struct reset_control *mcu1_reset;
	struct reset_control *intr_reset;
	struct reset_control *uart0_reset;
	struct reset_control *msigen_reset;
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

static const struct regmap_config tc956x_reset_clock_regmap_config = {
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
// Definitions taken from common.h in vendor driver
//

#define TC956X_TOT_CASCADE_DEV	7 /* Maximum number of devices for 2 Level cascade setup */
#define TC956X_PCI_BD_MASK	0xFFF8

// TODO: this was unifdef'ed (some build options result in the value being two)
#define TC956X_TOT_MSI_VEC	1

#define TC956X_DA_MAP		0xF

#define XGMAC_BASE(td)	((td)->sfr + ((td)->emac0 ? 0x40000 : 0x48000))

/*	Configuration Register Address	*/
#define NCID_OFFSET		(0x0000) /* TC956X Chip and revision ID */
#define NCID_REV_ID		GENMASK(7, 0)
#define NMODESTS_OFFSET		(0x0004) /* TC956X current operation mode */
#define NMODESTS_MODE2		BIT(10)	/* PCIe lanes: 0:  x4x1x1; 1: x2x2x1 */

#define NMISCCTL_OFFSET		(0x1800)

/* MSIGEN Registers */

#define TC956X_MSIGEN_BASE(pf_id)	(0x00f000 + (pf_id) * 0x0100)

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
#define TC956X_SW_MSI_CLR		0x0054

#define TC956X_HWIRQ_LPI		0
#define TC956X_HWIRQ_PMT		1
#define TC956X_HWIRQ_EVENT		2
#define TC956X_HWIRQ_TX0		3
#define TC956X_HWIRQ_RX0		11
#define TC956X_HWIRQ_XPCS		19
#define TC956X_HWIRQ_ETH		20 /* PHY interrupt */
#define TC956X_HWIRQ_PFMAILBOX		21
#define TC956X_HWIRQ_MSIREQ_PLS		24

#define TC956X_NR_HWIRQ			25

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

#define CLKCTRL0_OFFSET		0x04	/* Relative to clock/reset regmap */
#define CLKCTRL1_OFFSET		0x0c	/* Relative to clock/reset regmap */

static void __reset_clock_set(struct tc956x_chip *chip, u32 offset,
			      u32 bit, bool set)
{
	u32 mask = BIT(bit);

	/* Note: no need to check for errors on read/write for MMIO regmap */
	(void)regmap_update_bits(chip->reset_clock_regmap, offset, mask,
				 set ? mask : 0);
}

#define tc956x_chip_clock_enable(_chip, _id) \
	__reset_clock_set((_chip), CLKCTRL0_OFFSET, (u32)(_id), true)
#define tc956x_chip_clock_disable(_chip, _id) \
	__reset_clock_set((_chip), CLKCTRL0_OFFSET, (u32)(_id), false)
#define tc956x_mac_clock_enable(_td, _id) \
	__reset_clock_set((_td)->chip, (_td)->clock_offset, (u32)(_id), true)
#define tc956x_mac_clock_disable(_td, _id) \
	__reset_clock_set((_td)->chip, (_td)->clock_offset, (u32)(_id), false)

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

	sts = irq_reg_readl(gc, TC956X_MSI_INT_STS_OFFSET);
	if (sts)
		for_each_set_bit(hwirq, &sts, 32)
			generic_handle_domain_irq(d, hwirq);

	/*
	 * Clear the MSI flag. All interrupts within TC956x are level-high type.
	 * If any interrupts are still asserted then clearing this flag will
	 * cause the (edge-triggered) MSI to be regenerated.
	 */
	irq_reg_writel(gc, 1, TC956X_MSI_MASK_CLR_OFFSET);

	chained_irq_exit(chip, desc);
}

static int tc956x_msigen_chip_init(struct irq_chip_generic *gc)
{
	struct tc956x_msigen_data *tc956x_msigen = gc->domain->host_data;

	gc->reg_base = tc956x_msigen->regs;
	gc->chip_types[0].regs.mask = TC956X_MSI_OUT_EN_OFFSET;
	gc->chip_types[0].chip.irq_mask = irq_gc_mask_clr_bit;
	gc->chip_types[0].chip.irq_unmask = irq_gc_mask_set_bit;

	/* Ensure no interrupts are raised */
	irq_reg_writel(gc, 0, TC956X_MSI_OUT_EN_OFFSET);
	irq_reg_writel(gc, 1, TC956X_SW_MSI_CLR);

	/*
	 * Enable only those MSI vectors that are routed by the VECT_SETx
	 * settings below (currently only vector #0 is used).
	 */
	irq_reg_writel(gc, ~0, TC956X_MSI_MASK_SET_OFFSET);
	irq_reg_writel(gc, BIT(0), TC956X_MSI_MASK_CLR_OFFSET);

	/* Assign everything to vector #0 */
	irq_reg_writel(gc, 0, TC956X_MSI_VECT_SET0_OFFSET);
	irq_reg_writel(gc, 0, TC956X_MSI_VECT_SET1_OFFSET);
	irq_reg_writel(gc, 0, TC956X_MSI_VECT_SET2_OFFSET);
	irq_reg_writel(gc, 0, TC956X_MSI_VECT_SET3_OFFSET);
	irq_reg_writel(gc, 0, TC956X_MSI_VECT_SET4_OFFSET);
	irq_reg_writel(gc, 0, TC956X_MSI_VECT_SET5_OFFSET);
	irq_reg_writel(gc, 0, TC956X_MSI_VECT_SET6_OFFSET);
	irq_reg_writel(gc, 0, TC956X_MSI_VECT_SET7_OFFSET);

	return 0;
}

static void tc956x_msigen_chip_exit(struct irq_chip_generic *gc)
{
	irq_reg_writel(gc, 0, TC956X_MSI_OUT_EN_OFFSET);
	irq_reg_writel(gc, 1, TC956X_MSI_MASK_CLR_OFFSET);
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
		.handler	= handle_simple_irq,
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

	tc956x_msigen->regs = td->sfr + TC956X_MSIGEN_BASE(td->emac0 ? 0 : 1);
	tc956x_msigen->irq = pdev->irq;
	d_info.host_data = tc956x_msigen;

	domain = devm_irq_domain_instantiate(dev, &d_info);
	if (IS_ERR(domain))
		return dev_err_cast_probe(
			dev, domain, "failed to instantiate the IRQ domain\n");
	return domain;
}

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
	int retries = 10;
	int ret;

	np = dev_of_node(dev);
	if (!np)
		return -EINVAL;

	do {
		td->phy_reset = devm_gpiod_get(dev, "phy-reset", GPIOD_OUT_LOW);
		msleep(10);
	} while (IS_ERR(td->phy_reset) && retries--);

	if (retries < 0)
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

	ret = tc956x_reset_gpio_get(td);
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

	reset_control_assert(td->pma_reset);

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

	reset_control_deassert(td->pma_reset);

	WARN_ON(readl_poll_timeout(td->sfr + (td->emac0 ? NEMAC0CTL_OFFSET :
							  NEMAC1CTL_OFFSET),
				   val, val & EMAC_INIT_DONE, 50, 1000000));
}

//
// Code from tc956x_pci.c in vendor driver
//

struct {
	phy_interface_t phy_interface;
	int speed;
	u32 sp_sel;
	bool mac_125_clock;
} tc956x_chipcfg_mac_speed[] = {
	{ PHY_INTERFACE_MODE_10GBASER, SPEED_10000, SP_SEL_USXGMII_10G_10G },
	{ PHY_INTERFACE_MODE_SGMII, SPEED_2500, SP_SEL_SGMII_2500M },
	{ PHY_INTERFACE_MODE_2500BASEX, SPEED_2500, SP_SEL_SGMII_2500M },
	{ PHY_INTERFACE_MODE_SGMII, SPEED_1000, SP_SEL_SGMII_1000M, true },
	{ PHY_INTERFACE_MODE_SGMII, SPEED_100, SP_SEL_SGMII_100M, true },
	{ PHY_INTERFACE_MODE_SGMII, SPEED_10, SP_SEL_SGMII_10M, true },
};

static int tc956x_chipcfg_mac_configure(struct tc956x_data *td, int speed)
{
	u32 sp_sel = EMAC_SP_SEL_MASK + 1;
	u32 nemacxctl_offset;
	bool mac_125_clock;
	u32 val;
	int i;

	for (i = 0; i < ARRAY_SIZE(tc956x_chipcfg_mac_speed); i++) {
		if (tc956x_chipcfg_mac_speed[i].phy_interface ==
			    td->plat->phy_interface &&
		    tc956x_chipcfg_mac_speed[i].speed == speed) {
			sp_sel = tc956x_chipcfg_mac_speed[i].sp_sel;
			mac_125_clock =
				tc956x_chipcfg_mac_speed[i].mac_125_clock;
			break;
		}
	}
	if (sp_sel & ~EMAC_SP_SEL_MASK)
		return -ENOTSUPP;

	nemacxctl_offset = td->emac0 ? NEMAC0CTL_OFFSET : NEMAC1CTL_OFFSET;

	if (mac_125_clock) {
		__set_bit(MAC_STATE_125_CLOCK, td->mac_state);
		tc956x_mac_clock_enable(td, MAC_CLOCK_125M);
	} else {
		__clear_bit(MAC_STATE_125_CLOCK, td->mac_state);
		tc956x_mac_clock_disable(td, MAC_CLOCK_125M);
	}
	__clear_bit(MAC_STATE_312_5_CLOCK, td->mac_state);
	tc956x_mac_clock_disable(td, MAC_CLOCK_312_5M);

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
	int ret;

	reset_control_assert(td->mac_reset);

	__set_bit(MAC_STATE_TX_CLOCK, td->mac_state);
	tc956x_mac_clock_enable(td, MAC_CLOCK_TX);
	__set_bit(MAC_STATE_RX_CLOCK, td->mac_state);
	tc956x_mac_clock_enable(td, MAC_CLOCK_RX);
	__set_bit(MAC_STATE_ALL_CLOCK, td->mac_state);
	tc956x_mac_clock_enable(td, MAC_CLOCK_ALL);
	if (td->emac0) {
		if (plat->phy_interface == PHY_INTERFACE_MODE_SGMII ||
		    plat->phy_interface == PHY_INTERFACE_MODE_2500BASEX) {
			/* XXX These are always re-enabled during resume */
			tc956x_mac_clock_disable(td, MAC_CLOCK_PLL);
			tc956x_mac_clock_disable(td, MAC_CLOCK_SGMII);
			tc956x_mac_clock_disable(td, MAC_CLOCK_REFCLK);
		}
	} else {
		__set_bit(MAC_STATE_RGMII_CLOCK, td->mac_state);
		tc956x_mac_clock_enable(td, MAC_CLOCK_RMII);
	}

	/* Set the speed related registers */
	ret = tc956x_chipcfg_mac_configure(td, plat->max_speed);
	if (ret)
		return dev_err_probe(td->dev, ret,
				     "Cannot configure %s@%dMb/s\n",
				     phy_modes(plat->phy_interface),
				     plat->max_speed);

	reset_control_deassert(td->mac_reset);

	return 0;
}

static void tc956x_stop_clocks(struct tc956x_data *td)
{
	tc956x_mac_clock_disable(td, MAC_CLOCK_ALL);
	tc956x_mac_clock_disable(td, MAC_CLOCK_TX);
	tc956x_mac_clock_disable(td, MAC_CLOCK_RX);

	tc956x_mac_clock_disable(td, MAC_CLOCK_125M);
	tc956x_mac_clock_disable(td, MAC_CLOCK_312_5M);

	if (td == td->chip->primary) {
		tc956x_mac_clock_disable(td, MAC_CLOCK_PLL);
		tc956x_mac_clock_disable(td, MAC_CLOCK_SGMII);
		tc956x_mac_clock_disable(td, MAC_CLOCK_REFCLK);
	}
}

static void tc956x_restart_clocks(struct tc956x_data *td)
{
	if (td == td->chip->primary) {
		tc956x_mac_clock_enable(td, MAC_CLOCK_REFCLK);
		tc956x_mac_clock_enable(td, MAC_CLOCK_SGMII);
		tc956x_mac_clock_enable(td, MAC_CLOCK_PLL);
	}

	if (test_bit(MAC_STATE_312_5_CLOCK, td->mac_state))
		tc956x_mac_clock_enable(td, MAC_CLOCK_312_5M);
	if (test_bit(MAC_STATE_125_CLOCK, td->mac_state))
		tc956x_mac_clock_enable(td, MAC_CLOCK_125M);

	if (test_bit(MAC_STATE_RX_CLOCK, td->mac_state))
		tc956x_mac_clock_enable(td, MAC_CLOCK_RX);
	if (test_bit(MAC_STATE_TX_CLOCK, td->mac_state))
		tc956x_mac_clock_enable(td, MAC_CLOCK_TX);
	if (test_bit(MAC_STATE_ALL_CLOCK, td->mac_state))
		tc956x_mac_clock_enable(td, MAC_CLOCK_ALL);
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
	plat->flags |= STMMAC_FLAG_MULTI_MSI_EN | STMMAC_FLAG_TSO_EN;

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

	plat->dma_cfg->pbl = 32;
	plat->dma_cfg->pblx8 = true;

	/*
	 * TC956x has 8 RX queues but we observe significantly reduced RX
	 * bandwidth if we don't have at least 8k FIFO space per queue, so
	 * by default we avoid using all the queues.
	 */
	plat->rx_queues_to_use = 4;
	plat->rx_sched_algorithm = MTL_RX_ALGORITHM_SP;

	for (int i = 0; i < plat->rx_queues_to_use; i++) {
		plat->rx_queues_cfg[i].mode_to_use = MTL_QUEUE_DCB;
	}

	/*
	 * TX956x has 8 TX queues. However failures are observed (DHCP does not
	 * get an IP address or ping does fails) if tx_queues_to_use >3
	 */
	plat->tx_queues_to_use = 3;
	plat->tx_sched_algorithm = MTL_TX_ALGORITHM_WRR;

	for (int i = 0; i < plat->tx_queues_to_use; i++) {
		plat->tx_queues_cfg[i].weight = 12;
		plat->tx_queues_cfg[i].mode_to_use = MTL_QUEUE_DCB;

		/* Tx Queues 0 - 4 doesn't support TBS on TC956x */
		if (i >= 5)
			plat->tx_queues_cfg[i].tbs_en = true;
	}

	/*
	 * Oversized FIFOs result in reduced performance in bandwidth tests.
	 * Let's limit them either to 8KiB unless they must be smaller.
	 */
	plat->tx_fifo_size = min(plat->tx_queues_to_use * 8, 46) * SZ_1K;
	plat->rx_fifo_size = min(plat->rx_queues_to_use * 8, 46) * SZ_1K;

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

/* Extra fields for XGMAC_DMA_MODE */
#define XGMAC_DSPW			BIT(8);
#define XGMAC_DMA_MODE_INTM		GENMASK(13, 12)

#define XGMAC_DMA_CH_RX_CONTROL2(x) 	(0x00003134 + (0x80 * (x)))
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
	value = u32_replace_bits(value, 1, XGMAC_DMA_MODE_INTM);
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
	u32 rxpbl = dma_cfg->rxpbl ?: dma_cfg->pbl;
	u32 value;

	value = readl(ioaddr + XGMAC_DMA_CH_RX_CONTROL(chan));
	value = u32_replace_bits(value, rxpbl, XGMAC_RxPBL);
	writel(value, ioaddr + XGMAC_DMA_CH_RX_CONTROL(chan));

	/*
	 * Reduce the number of outstanding write requests to 3. This is needed
	 * for XGMAC 3.01a errata.
	 */
	value = readl(ioaddr + XGMAC_DMA_CH_RX_CONTROL2(chan));
	value = u32_replace_bits(value, td->revid == 1 ? 3 : 0, XGMAC_OWRQ);
	writel(value, ioaddr + XGMAC_DMA_CH_RX_CONTROL2(chan));

	writel(upper_32_bits(phy) + upper_32_bits(TC956X_AXI4_SLV00_SRC_ADDR),
	       ioaddr + XGMAC_DMA_CH_RxDESC_HADDR(chan));
	writel(lower_32_bits(phy), ioaddr + XGMAC_DMA_CH_RxDESC_LADDR(chan));
}

static void tc956x_dma_init_tx_chan(struct stmmac_priv *priv,
				    void __iomem *ioaddr,
				    struct stmmac_dma_cfg *dma_cfg,
				    dma_addr_t phy, u32 chan)
{
	u32 txpbl = dma_cfg->txpbl ?: dma_cfg->pbl;
	u32 value;

	value = readl(ioaddr + XGMAC_DMA_CH_TX_CONTROL(chan));
	value = u32_replace_bits(value, txpbl, XGMAC_TxPBL);
	writel(value, ioaddr + XGMAC_DMA_CH_TX_CONTROL(chan));

	writel(upper_32_bits(phy) + upper_32_bits(TC956X_AXI4_SLV00_SRC_ADDR),
	       ioaddr + XGMAC_DMA_CH_TxDESC_HADDR(chan));
	writel(lower_32_bits(phy), ioaddr + XGMAC_DMA_CH_TxDESC_LADDR(chan));
}

static void tc956x_desc_set_addr(struct dma_desc *p, dma_addr_t addr)
{
	p->des0 = cpu_to_le32(lower_32_bits(addr));
	p->des1 = cpu_to_le32(upper_32_bits(addr) +
			      upper_32_bits(TC956X_AXI4_SLV00_SRC_ADDR));
}

static void tc956x_desc_set_sec_addr(struct dma_desc *p, dma_addr_t addr, bool is_valid)
{
	p->des2 = cpu_to_le32(lower_32_bits(addr));
	p->des3 = cpu_to_le32(upper_32_bits(addr) +
			      upper_32_bits(TC956X_AXI4_SLV00_SRC_ADDR));
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

	WARN(tc956x_chipcfg_mac_configure(td, speed),
	     "%s@%dMb/s is not supported",
	     phy_modes(td->plat->phy_interface), speed);
	tc956x_pma_init(td);
}

/**
 * tc956x_suspend() - Device driver suspend callback
 * @dev:		Device pointer
 * @bsp_priv:		stmmac dsp_priv pointer
 *
 * Perform the activities required to suspend the TC956x platform device.
 * This includes suspending the eMACs (and managing wake-on-LAN state)
 * and suspending the PCIe interfaces.
 *
 * Return:	0 if successful, or an error code if an error occurs
 */
static int tc956x_suspend(struct device *dev, void *bsp_priv)
{
	struct net_device *ndev = dev_get_drvdata(dev);
	struct stmmac_priv *priv = netdev_priv(ndev);
	struct pci_dev *pdev = to_pci_dev(dev);
	struct tc956x_data *td = bsp_priv;
	int ret;

	ret = tc956x_platform_suspend(priv);
	if (ret) {
		dev_err(dev, "%s: error in calling tc956x_platform_suspend", pci_name(pdev));
		return ret;
	}

	reset_control_assert(td->mac_reset);
	reset_control_assert(td->pma_reset);
	reset_control_assert(td->xpcs_reset);

	tc956x_stop_clocks(td);

	return stmmac_pci_plat_suspend(dev, bsp_priv);
}

/**
 * tc956x_resume() - Device driver resume callback
 * @dev:		Device pointer
 * @bsp_priv:		stmmac dsp_priv pointer
 *
 * Perform the activities required to resume the TC956x platform device.
 * This includes resuming the PCIe interfaces, and disabling wake-on-LAN
 * and resuming the eMACs.
 *
 * Return:	0 if successful, or an error code if an error occurs
 */
static int tc956x_resume(struct device *dev, void *bsp_priv)
{
	struct pci_dev *pdev = to_pci_dev(dev);
	struct net_device *ndev = dev_get_drvdata(dev);
	struct stmmac_priv *priv = netdev_priv(ndev);
	struct tc956x_data *td = priv->plat->bsp_priv;
	int ret;

	ret = stmmac_pci_plat_resume(dev, bsp_priv);
	if (ret < 0)
		return ret;

	tc956x_restart_clocks(td);

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

	/* Configure TA map registers when the primary MAC resumes */
	if (td == td->chip->primary)
		tc956x_config_tamap(td);

	/* Configure EMAC Port */
	ret = tc956x_chipcfg_mac_init(td);
	WARN_ON(ret);

	tc956x_pma_init(td);
	reset_control_deassert(td->xpcs_reset);

	return 0;
}

static const struct mfd_cell tc956x_mfd_cells[] = {
	{ .name = "tc956x-gpio", },
	{ .name = "tc956x-reset-controller", },
};

static int tc956x_devm_mfd_init(struct tc956x_chip *chip)
{
	struct device *dev = chip->primary->dev;
	void __iomem *regs = chip->primary->sfr;
	struct regmap *regmap;

	regmap = devm_regmap_init_mmio(dev, regs, &tc956x_gpio_regmap_config);
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
	plat->mac_setup = tc956x_mac_setup;
	plat->fix_mac_speed = tc956x_fix_mac_speed;
	plat->pcs_init = tc956x_pcs_init;
	plat->select_pcs = tc956x_select_pcs;
	plat->suspend = tc956x_suspend;
	plat->resume = tc956x_resume;

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

	td->dev = dev;
	td->devfn = pdev->devfn;

	ret = tc956x_platform_of_parse(td);
	if (ret)
		return ERR_PTR(ret);

	td->plat = tc956x_plat_dat_alloc(td, pdev);
	if (!td->plat)
		return NULL;

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

	return td;
}

/*
 * When successful all reset controls will be valid (no error), and
 * the underlying driver implements the assert and deassert callbacks.
 * So there is no need to test for errors when asserting or deasserting.
 */
static int tc956x_devm_chip_resets_get(struct tc956x_chip *chip)
{
	struct device *dev = chip->primary->dev;
	int retries = 10;

	/* XXX We don't need any reset except for MSIGEN */

	/*
	 * We cannot return -EPROBE_DEFER (at least not from function 0) because
	 * we have already created child devices (including the reset
	 * controller) so we just have to wait for it to appear.
	 */
	chip->mcu_reset = devm_reset_control_get_exclusive(dev, "MCU");
	while (IS_ERR(chip->mcu_reset) && retries-- > 0) {
		msleep(10);
		chip->mcu_reset = devm_reset_control_get_exclusive(dev, "MCU");
	}
	if (IS_ERR(chip->mcu_reset))
		return PTR_ERR(chip->mcu_reset);

	chip->mcu1_reset = devm_reset_control_get_exclusive(dev, "MCU1");
	if (IS_ERR(chip->mcu1_reset))
		return PTR_ERR(chip->mcu1_reset);

	chip->intr_reset = devm_reset_control_get_exclusive(dev, "INTC");
	if (IS_ERR(chip->intr_reset))
		return PTR_ERR(chip->intr_reset);

	chip->msigen_reset = devm_reset_control_get_exclusive(dev, "MSIGEN");
	if (IS_ERR(chip->msigen_reset))
		return PTR_ERR(chip->msigen_reset);

	chip->uart0_reset = devm_reset_control_get_exclusive(dev, "UART0");
	if (IS_ERR(chip->uart0_reset))
		return PTR_ERR(chip->uart0_reset);

	return 0;
}

static struct tc956x_chip *tc956x_chip_get(struct tc956x_data *td)
{
	u8 pci_bus_num = PCI_BUS_NUM(td->devfn);
	u8 pci_slot = PCI_SLOT(td->devfn);
	struct device *dev = td->dev;
	struct tc956x_chip *chip;
	struct regmap *regmap;
	int ret;

	/* Use the existing chip structure if it's already been created */
	list_for_each_entry(chip, &tc956x_chips, links) {
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
		return NULL;

	chip->pci_bus_num = pci_bus_num;
	chip->pci_slot = pci_slot;
	chip->primary = td;

	regmap = devm_regmap_init_mmio(td->dev, td->sfr,
				       &tc956x_reset_clock_regmap_config);
	if (IS_ERR(regmap))
		return ERR_CAST(regmap);
	chip->reset_clock_regmap = regmap;

	ret = tc956x_devm_mfd_init(chip);
	if (ret)
		return dev_err_ptr_probe(td->dev, ret, "mfd init failed\n");

	ret = tc956x_devm_chip_resets_get(chip);
	if (ret)
		return dev_err_ptr_probe(td->dev, ret, "no chip resets\n");

	list_add(&chip->links, &tc956x_chips);

	return chip;
}

static void tc956x_chip_put(struct tc956x_data *td)
{
	struct tc956x_chip *chip = td->chip;

	td->chip = NULL;

	/* The primary interface needs to be the last to go */
	if (chip->secondary) {
		/* XXX Why not just td == chip->secondary as done elsewhere? */
		if (td->dev == chip->secondary->consumer) {
			device_link_del(chip->secondary);
			chip->secondary = NULL;
		}
		return;
	}

	/*
	 * XXX This logic is insufficient.  We need to control when the
	 * XXX memory that the chip structure will point to gets unmapped.
	 * XXX We'll deal with this later.
	 */
	list_del(&chip->links);

	chip->primary = NULL;
	chip->pci_slot = 0;
	chip->pci_bus_num = 0;

	kfree(chip);
}

/*
 * When successful all reset controls will be valid (no error), and
 * the underlying driver implements the assert and deassert callbacks.
 * So there is no need to test for errors when asserting or deasserting.
 */
static int tc956x_devm_mac_resets_get(struct tc956x_data *td)
{
	struct device *dev = td->dev;

	td->mac_reset = devm_reset_control_get_exclusive(dev, "MAC");
	if (IS_ERR(td->mac_reset))
		return PTR_ERR(td->mac_reset);

	td->pma_reset = devm_reset_control_get_exclusive(dev, "PMA");
	if (IS_ERR(td->pma_reset))
		return PTR_ERR(td->pma_reset);

	td->xpcs_reset = devm_reset_control_get_exclusive(dev, "XPCS");
	if (IS_ERR(td->xpcs_reset))
		return PTR_ERR(td->xpcs_reset);

	return 0;
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
	struct irq_domain *irq_domain;
	struct tc956x_data *td;
	/* use signal from EMSPHY */
	u32 pfn, val;
	int ret;

	td = tc956x_devm_data_create(pdev);
	if (IS_ERR_OR_NULL(td))
		return dev_err_probe(dev, td ? PTR_ERR(td) : -ENOMEM,
				     "cannot create data");

	pci_set_master(pdev);

	td->chip = tc956x_chip_get(td);
	if (IS_ERR_OR_NULL(td->chip)) {
		ret = dev_err_probe(dev, td->chip ? PTR_ERR(td->chip) : -ENOMEM,
				    "cannot get chip\n");
		goto err_clear_master;
	}

	ret = tc956x_devm_mac_resets_get(td);
	if (ret) {
		ret = dev_err_probe(dev, -EPROBE_DEFER, "no mac resets\n");
		goto err_chip_put;
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

	/* NCLKCTRL0_OFFSET or NCLKCTRL1_OFFSET, relative to regmap */
	td->clock_offset = td->emac0 ? CLKCTRL0_OFFSET : CLKCTRL1_OFFSET;

	// NCID_OFFSET gives the revision ID (and early revisions are limited
	// to 2.5G)
	val = readl(td->sfr + NCID_OFFSET);
	dev_dbg(dev, "NCID Register value: %x\n", val);
	td->revid = FIELD_GET(NCID_REV_ID, val);

	// TODO: this needs to come from devicetree
	td->plat->phy_interface = td->emac0 ? PHY_INTERFACE_MODE_10GBASER :
					      PHY_INTERFACE_MODE_SGMII;

	ret = tc956x_xgmac3_default_data(pdev, td->plat);
	if (ret)
		goto err_chip_put;

	/* XXX eMAC0, or first one probed? */
	if (td->emac0)
		tc956x_config_tamap(td);

	/*
	 * Enable MSI and Allocate Vectors. Despite the spelling (no pcim) the
	 * free will be handled by devres due to the prior pcim_enable_device()
	 */
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
		reset_control_assert(td->chip->mcu_reset);
		reset_control_assert(td->chip->mcu1_reset);
	}

	/*
	 * Enable MSIGEN Module
	 *
	 * TODO: Ideally msigen_reset should be shared by each MAC (rather than
	 *       exclusively owned by the chip). That would make it possible to
	 *       deassert/assert the reset from the irqchip code.
	 */
	tc956x_chip_clock_enable(td->chip, CHIP_CLOCK_MSIGEN);
	reset_control_deassert(td->chip->msigen_reset);

	irq_domain = devm_tc956x_msigen_register(pdev, td);
	if (IS_ERR_OR_NULL(irq_domain)) {
		ret = PTR_ERR(irq_domain);
		goto err_out_msi_failed;
	}

	res.addr = XGMAC_BASE(td);
	/* Problems creating mappings will be reported by stmmac_dvr_probe */
	res.irq = irq_create_mapping(irq_domain, TC956X_HWIRQ_EVENT);
	for (int i=0; i<MTL_MAX_TX_QUEUES; i++)
		res.tx_irq[i] = irq_create_mapping(irq_domain, TC956X_HWIRQ_TX0 + i);
	for (int i=0; i<MTL_MAX_RX_QUEUES; i++)
		res.rx_irq[i] = irq_create_mapping(irq_domain, TC956X_HWIRQ_RX0 + i);

	/*
	 * Hook up the PHY interrupt.
	 *
	 * TODO: This probably wants to be made optional in the DT (if the
	 *       interrupt is not connected we need to fall back to polling)
	 */
	td->plat->mdio_bus_data->probed_phy_irq =
		irq_create_mapping(irq_domain, TC956X_HWIRQ_ETH);

	ret = tc956x_platform_probe(td, &res);
	if (ret) {
		dev_err(dev, "Platform (DT) code failed\n");
		goto err_platform_probe;
	}

	ret = tc956x_chipcfg_mac_init(td);
	if (ret < 0)
		goto err_platform_probe;

	tc956x_pma_init(td);
	reset_control_deassert(td->xpcs_reset);

	ret = stmmac_dvr_probe(dev, td->plat, &res);
	if (ret) {
		reset_control_assert(td->mac_reset);
		reset_control_assert(td->pma_reset);
		reset_control_assert(td->xpcs_reset);

		tc956x_mac_clock_disable(td, MAC_CLOCK_ALL);
		tc956x_mac_clock_disable(td, MAC_CLOCK_RX);
		tc956x_mac_clock_disable(td, MAC_CLOCK_TX);

		tc956x_mac_clock_disable(td, MAC_CLOCK_125M);
		tc956x_mac_clock_disable(td, MAC_CLOCK_312_5M);

		if (!td->emac0)
			tc956x_mac_clock_disable(td, MAC_CLOCK_RMII);

		if (ret != -ENODEV)
			goto err_dvr_probe;
	}

	return 0;

err_dvr_probe:
	(void)tc956x_platform_remove(td);
err_platform_probe:
err_out_msi_failed:
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

	reset_control_assert(td->mac_reset);
	reset_control_assert(td->pma_reset);
	reset_control_assert(td->xpcs_reset);

	/*
	 * XXX We could probably arrange things so this code can reuse the logic
	 *     in tc956x_stop_clocks()
	 */
	tc956x_mac_clock_disable(td, MAC_CLOCK_TX);
	tc956x_mac_clock_disable(td, MAC_CLOCK_RX);
	tc956x_mac_clock_disable(td, MAC_CLOCK_ALL);

	tc956x_mac_clock_disable(td, MAC_CLOCK_125M);
	tc956x_mac_clock_disable(td, MAC_CLOCK_312_5M);

	/* Close shared things down if the primary is removed */
	if (td == td->chip->primary) {
		struct tc956x_chip *chip = td->chip;

		/* XXX
		 * I think this still might not be enough.  If we happen
		 * to remove the primary interface but not the secondary,
		 * I'm not sure the device core will still perform actions
		 * (like suspending/resuming) on the primary device.
		 */
		reset_control_assert(chip->mcu_reset);
		reset_control_assert(chip->mcu1_reset);
		reset_control_assert(chip->intr_reset);
		reset_control_assert(chip->msigen_reset);
		reset_control_assert(chip->uart0_reset);

		tc956x_chip_clock_enable(chip, CHIP_CLOCK_MCU);
		tc956x_chip_clock_enable(chip, CHIP_CLOCK_SRAM);
		tc956x_chip_clock_enable(chip, CHIP_CLOCK_PCIE);
		tc956x_chip_clock_enable(chip, CHIP_CLOCK_I2C);

		tc956x_chip_clock_disable(chip, CHIP_CLOCK_MSIGEN);
		tc956x_chip_clock_disable(chip, CHIP_CLOCK_INTC);
		tc956x_chip_clock_disable(chip, CHIP_CLOCK_UART0);

		tc956x_mac_clock_disable(td, MAC_CLOCK_PLL);
		tc956x_mac_clock_disable(td, MAC_CLOCK_SGMII);
		tc956x_mac_clock_disable(td, MAC_CLOCK_REFCLK);
	}

	tc956x_chip_put(td);
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

static struct pci_driver tc956x_pci_driver = {
	.name		= DRIVER_NAME,
	.id_table	= tc956x_id_table,
	.probe		= tc956x_pci_probe,
	.remove		= tc956x_pci_remove,
	.driver		= {
		.name	= DRIVER_NAME,
		.owner	= THIS_MODULE,
		.pm     = &stmmac_simple_pm_ops,
	},
};

module_pci_driver(tc956x_pci_driver);

MODULE_DESCRIPTION("Toshiba TC956x PCIe Ethernet Network Driver");
MODULE_LICENSE("GPL");
