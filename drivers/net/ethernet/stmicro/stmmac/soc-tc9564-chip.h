/* SPDX-License-Identifier: GPL-2.0 */

/*
 * Copyright (C) 2026 by RISCstar Solutions Corporation.  All rights reserved.
 */

#ifndef __SOC_TOSHIBA_TC9564_CHIP_H__
#define __SOC_TOSHIBA_TC9564_CHIP_H__

#include <linux/types.h>

enum tc9564_chip_reset_id {
	CHIP_RESET_MCU		= 0,
	CHIP_RESET_MCU1		= 1,
	CHIP_RESET_MSIGEN	= 18,
	CHIP_RESET_INTC		= 4,
	CHIP_RESET_UART0	= 16,
};

enum tc9564_mac_reset_id {
	MAC_RESET_MAC		= 7,
	MAC_RESET_PMA		= 30,
	MAC_RESET_XPCS		= 31,
};

enum tc9564_chip_clock_id {
	CHIP_CLOCK_MCU		= 0,
	CHIP_CLOCK_SRAM		= 13,
	CHIP_CLOCK_MSIGEN	= 18,
	CHIP_CLOCK_PLL		= 24,
	CHIP_CLOCK_SGMII	= 25,
	CHIP_CLOCK_REFCLK	= 26,
	CHIP_CLOCK_INTC		= 4,
	CHIP_CLOCK_UART0	= 16,
};

enum tc9564_mac_clock_id {
	MAC_CLOCK_TX		= 7,
	MAC_CLOCK_RX		= 14,
	MAC_CLOCK_ALL		= 31,
	MAC_CLOCK_125M		= 29,
	MAC_CLOCK_312_5M	= 30,
	MAC_CLOCK_RMII		= 15,	/* eMAC 1 only */
};

struct tc9564_chip;

/**
 * struct tc9564_dwmac_data - Structure passed to stmmac auxiliary devices.
 * @sfr:		I/O mapped address of entire SFR region
 * @dwmac_addr:		I/O mapped address used by dwmac
 * @msigen_addr:	I/O mapped address used by MSIGEN
 * @msigen_irq:		IRQ number used by MSIGEN
 * @rev_id:		Chip revision ID (for quirks)
 * @id:			Unique device ID
 *
 * This structure is passed via platform data to the stmmac auxiliary devices.
 */
struct tc9564_dwmac_data {
	void __iomem *sfr;		/* XXX This should go away */
	void __iomem *dwmac_addr;
	void __iomem *msigen_addr;
	unsigned int msigen_irq;
	u32 rev_id;
	u32 id;
};

extern void tc9564_chip_reset_clock_set(struct tc9564_chip *chip, bool reset,
					bool reg0, bool set, u8 bit);

/* Chip and MAC reset assert/deassert */
static inline void tc9564_chip_reset_assert(struct tc9564_chip *chip,
					    enum tc9564_chip_reset_id id)
{
	tc9564_chip_reset_clock_set(chip, true, true, true, (u8)id);
}

static inline void tc9564_chip_reset_deassert(struct tc9564_chip *chip,
					      enum tc9564_chip_reset_id id)
{
	tc9564_chip_reset_clock_set(chip, true, true, false, (u8)id);
}

static inline void tc9564_mac_reset_assert(struct tc9564_chip *chip,
					   u8 pci_fn,
					   enum tc9564_mac_reset_id id)
{
	tc9564_chip_reset_clock_set(chip, true, !pci_fn, true, (u8)id);
}

static inline void tc9564_mac_reset_deassert(struct tc9564_chip *chip,
					     u8 pci_fn,
					     enum tc9564_mac_reset_id id)
{
	tc9564_chip_reset_clock_set(chip, true, !pci_fn, false, (u8)id);
}


/* Chip and MAC clock enable/disable */
static inline void tc9564_chip_clock_enable(struct tc9564_chip *chip,
					    enum tc9564_chip_clock_id id)
{
	tc9564_chip_reset_clock_set(chip, false, true, true, (u8)id);
}

static inline void tc9564_chip_clock_disable(struct tc9564_chip *chip,
					     enum tc9564_chip_clock_id id)
{
	tc9564_chip_reset_clock_set(chip, false, true, false, (u8)id);
}

static inline void tc9564_mac_clock_enable(struct tc9564_chip *chip,
					   u8 pci_fn,
					   enum tc9564_mac_clock_id id)
{
	tc9564_chip_reset_clock_set(chip, false, !pci_fn, true, (u8)id);
}

static inline void tc9564_mac_clock_disable(struct tc9564_chip *chip,
					    u8 pci_fn,
					    enum tc9564_mac_clock_id id)
{
	tc9564_chip_reset_clock_set(chip, false, !pci_fn, false, (u8)id);
}

#endif /* __SOC_TOSHIBA_TC9564_CHIP_H__*/
