/*
 * Copyright (c) 2021 Intel Corporation
 *
 * SPDX-License-Identifier:  BSD-3-Clause
 */

#ifndef _INTEL_TGPIO_REGS_H_
#define _INTEL_TGPIO_REGS_H_

/* Total pin numbers */
#define TGPIO_MAX_PIN_NUM (20U)
/* Total timers */
#define TGPIO_MAX_TIMERS (3U)
#define TGPIO_MAX_NS (999999999U)
#define TGPIO_MAX_ADJUST (999999896U)
#define TGPIO_MAX_TIMINCA (0x7fffffff)

/* GPIO/TGPIO Pinmux mapping */
/* 29:0 GPIO */
#define TGPIO_PINMUX_0_MASK (0x00000000)
/* 29:20 TGPIO, 19:10 TGPIO, 9:0 GPIO*/
#define TGPIO_PINMUX_1_MASK (0x3ffffc00)
/* 29:20 GPIO, 19:10 TGPIO, 9:0 TGPIO*/
#define TGPIO_PINMUX_2_MASK (0x000fffff)
/* 29:20 TGPIO, 19:10 GPIO, 9:0 TGPIO*/
#define TGPIO_PINMUX_3_MASK (0x3ff003ff)

#define BASE_OFFSET (0x1000)
#define LOWER_10_BITS_MASK (0x000003ff)
#define UPPER_10_BITS_MASK (0x3ff00000)
#define REPEAT_COUNT_MAX (0xff)
#define PULSE_WIDTH_MAX (15U)
#define PULSE_WIDTH_MIN (2U)
#define TS_SEL_REG_LIMIT (16U)
#define MUX_SHIFT_SIZE (10U)
#define BITS_PER_PIN (2U)
#define REG_WIDTH (32U)
#define DEFAULT_ART_MATCH_MASK (0U)
#define DEFAULT_TMT_MATCH_MASK (0x7)
#define NANO_SEC_WRAP_BITS_0_2 (0x7)
/* (2^32)/(10^9) = 4.29 */
#define PPB_TO_TIMINCA (4.294f)
#define MAX_PPB (500000000)
#define MIN_PPB (-500000000)

#define ART_PERIOD (52083)
#define NSEC_PER_SECOND 1000000000
#define KHz 1000

/* Enable timer */
#define TGPIO_TIMER_ENABLE (BIT(0))

/* Enable pin */
#define TGPIO_PIN_ENABLE (BIT(0))
/* Direction */
#define TGPIO_PIN_DIRECTION (BIT(1))
/* Periodic mode */
#define TGPIO_PIN_PERIODIC (BIT(4))
/* Input counter select */
#define TGPIO_PIN_INPUT_CNTR_SLCT (BIT(9))
/* Timestamp counter select */
#define TGPIO_PIN_TIMESTAMP_CNTR_SLCT (BIT(10))
/* Override Match mask */
#define TGPIO_PIN_OUTPUT_SKIP_MATCH (BIT(12))
/* Input repeat or immediate */
#define TGPIO_PIN_INPUT_REPEAT (BIT(14))
/* Event counter */
#define TGPIO_PIN_ENABLE_EC (BIT(16))
/* Sign bit mask for TIMINCA & TIMADJ */
#define TGPIO_TMT_TIME_ADJUST_SIGN (BIT(31))

/* Event Polariy bit */
#define TGPIO_EVENT_POLARITY_BIT (2)
/* Pulse width modulation bit */
#define TGPIO_PWS_BIT (5)
/* Repead count bit */
#define TGPIO_N_REPEATS_BIT (17)
/* Nanosecond wrap interrupt bit */
#define TGPIO_NANO_SEC_WRAP_BIT (23)
/* Time Adjust complete bit */
#define TGPIO_TADJ_TMT_BIT (20)

#define IPC_HOST_BASE 0x40400000
#define TGPIO_MUX_SEL_REG_0_LH2OSE (IPC_HOST_BASE + 0xB00)
#define TGPIO_MUX_SEL_REG_1_LH2OSE (IPC_HOST_BASE + 0xB04)

/**
 * Structure that represents a single pin
 */
typedef __IO_RW struct tgpio_single_pin_regs {
	/*
	 * Single pin control register
	 *  BIT(31-30): Reserved
	 *  BIT(29-28): Timer Select (Deprecated)
	 *  BIT(27-25): Reserved
	 *  BIT(24-17): Periodic Sequence Length,
	 *		     0 - Sequence continues indefinitely
	 *		     X - Lenght of periodic sequence
	 *  BIT(16)   : EC Counter
	 *  BIT(15-14): Input Event Control
	 *		    00 - Input alone
	 *		    01 - Output EC fedback to input EC
	 *		    10 - Reserved
	 *		    11 - Reserved
	 *  BIT(13)   : Freeze Input Time Stamp (Deprecated)
	 *  BIT(12)   : Output Event control, 1-Simple GPIO
	 *  BIT(11)   : Reserved
	 *  BIT(10)   : Timestamp Select, 0-Timestamp, 1-Event Counter
	 *		(Selected input is captured in TCV)
	 *  BIT(9)    : Input Counter Select, 0-Timestamp, 1-Event Counter
	 *		(Selected input is compared against COMPV)
	 *  BIT(8-5)  : Pulse width stretch in cycles
	 *  BIT(4)    : Periodic Mode
	 *  BIT(3-2)  : Event Polarity
	 *		    00-Raising edge
	 *		    01-Falling Edge
	 *		    10-Toggle mode
	 *		    11-Reserved
	 *  BIT(1)    : Direction: 1-Input; 0-Output
	 *  BIT(0)    : 1 - Enabled, 0 - Disabled
	 */
	__IO_RW uint32_t tgpio_ctl;
	/*
	 * Comparator value ns
	 * BIT(31)-BIT(30) don't care or zero
	 */
	__IO_RW uint32_t tgpio_compv_31_0;
	/* Comparator value secs */
	__IO_RW uint32_t tgpio_compv_63_32;
	/* Periodic interval value: next output after */
	__IO_RW uint32_t tgpio_piv_31_0;
	__IO_RW uint32_t tgpio_piv_63_32;
	/*
	 * Time capture value
	 * Input mode - ART is captured
	 * Output mode - COMPV is captured
	 */
	__IO_R uint32_t tgpio_tcv_31_0;
	__IO_R uint32_t tgpio_tcv_63_32;
	/* Number of events associated to last read TCV */
	__IO_R uint32_t tgpio_eccv_31_0;
	__IO_R uint32_t tgpio_eccv_63_32;
	/* Event Counter/Number of events */
	__IO_R uint32_t tgpio_ec_31_0;
	__IO_R uint32_t tgpio_ec_63_32;
	/* Mask reg - used for forcing a match b/w COMPV & input */
	__IO_RW uint32_t tgpio_mask_31_0;
	__IO_RW uint32_t tgpio_mask_63_32;
	__IO_R uint32_t reserved[3];
} tgpio_pin_regs_t;

/**
 * Structure for all interrupt configuration regs
 */
typedef __IO_RW struct interrupt_regs {
	/*
	 * Interrupt Control Register
	 * BIT(0): Interrupt Coalescing. 0-Enabled, 1-Disabled
	 * BIT(0) = 1 Not supported currently
	 */
	__IO_RW uint32_t tgpio_intr_ctl;
	/*
	 * Raw Interrupt Status Register
	 * BIT(31-26): Reserved
	 * BIT(25)   : TMT_2 nano second counter wrap
	 * BIT(24)   : TMT_1 nano second counter wrap
	 * BIT(23)   : TMT_0 nano second counter wrap
	 * BIT(22)   : Time adjust complete TMT_2
	 * BIT(21)   : Time adjust complete TMT_1
	 * BIT(20)   : Time adjust complete TMT_0
	 * BIT(19-0) : PIN event interrupt
	 *		0-No Interrupt
	 *		1-Interrupt pending
	 */
	__IO_R uint32_t tgpio_intr_ris;
	/*
	 * Interrupt Mask Register
	 * BIT(31-26): Reserved
	 * BIT(25)   : Mask TMT_2 nano second counter wrap
	 * BIT(24)   : Mask TMT_1 nano second counter wrap
	 * BIT(23)   : Mask TMT_0 nano second counter wrap
	 * BIT(22)   : Mask time adjust complete TMT_2
	 * BIT(21)   : Mask time adjust complete TMT_1
	 * BIT(20)   : Mask time adjust complete TMT_0
	 * BIT(19-0) : Mask PIN event interrupt
	 *		0-Interrupt masked
	 *		1-Interrupt Enabled
	 */
	__IO_RW uint32_t tgpio_intr_imsc;
	/*
	 * Masked Interrupt Status Register
	 * BIT(31-26): Reserved
	 * BIT(25)   : TMT_2 nano second counter wrap
	 * BIT(24)   : TMT_1 nano second counter wrap
	 * BIT(23)   : TMT_0 nano second counter wrap
	 * BIT(22)   : Time adjust complete TMT_2
	 * BIT(21)   : Time adjust complete TMT_1
	 * BIT(20)   : Time adjust complete TMT_0
	 * BIT(19-0) : PIN event interrupt
	 *		0 - No Interrupt
	 *		1 - Interrupt pending
	 */
	__IO_R uint32_t tgpio_intr_mis;
	/*
	 * Interrupt Clear Register
	 * BIT(31-26): Reserved
	 * BIT(25)   : Clear TMT_2 nano second counter wrap
	 * BIT(24)   : Clear TMT_1 nano second counter wrap
	 * BIT(23)   : Clear TMT_0 nano second counter wrap
	 * BIT(22)   : Clear time adjust complete TMT_2
	 * BIT(21)   : Clear time adjust complete TMT_1
	 * BIT(20)   : Clear time adjust complete TMT_0
	 * BIT(19-0) : Clear PIN event interrupt, HW clears the bit
	 *		0 - No change
	 *		1 - Clear interrupt
	 */
	__IO_W uint32_t tgpio_intr_icr;
} tgpio_intr_regs_t;

/**
 * Structure that represents a Tunable Monotonous Timer
 */
typedef __IO_RW struct tmt_timer_regs {
	/*
	 * TMT Control register
	 * BIT(31-1): Reserved
	 * BIT(0)   : 0-Disabled, 1-Enabled & Starts incrementing
	 */
	__IO_RW uint32_t tmt_ctl;
	/* TMT Residue register */
	__IO_RW uint32_t tmt_r;
	/* Lower half of TMT defined in nano seconds */
	__IO_RW uint32_t tmt_l;
	/* Upper half of TMT defined in seconds */
	__IO_RW uint32_t tmt_h;
	/*
	 * Timer increment attributes register
	 * BIT(31)  : Sign. 0-Increment, 1-Decrement
	 * BIT(30-0): Value in resolution of 2^-32ns
	 */
	__IO_RW uint32_t tmt_timinca;
	/*
	 * Time adjust offset register
	 * BIT(31)  : Sign. 0-Increment, 1-Decrement)
	 * BIT(30)  : Zero bit
	 * BIT(29-0): Time adjust value (ns)
	 */
	__IO_RW uint32_t tmt_timadj;
	/* TMTL - during local time stamp capture */
	__IO_R uint32_t tmt_local_high;
	/* TMTH - during local time stamp capture */
	__IO_R uint32_t tmt_local_low;
	/* ARTL - during local time stamp capture */
	__IO_R uint32_t tmt_art_high;
	/* ARTH - during local time stamp capture */
	__IO_R uint32_t tmt_art_low;
	/* TMTL - during remote time stamp capture */
	__IO_R uint32_t tmt_remote_high;
	/* TMTH - during remote time stamp capture */
	__IO_R uint32_t tmt_remote_low;
	__IO_R uint32_t reserved[4];

} tgpio_tmt_regs_t;

/**
 * Master instance TPGIO register structure
 */
typedef __IO_RW struct tgpio_master_instance_regs {
	tgpio_pin_regs_t tgpio_pin[TGPIO_MAX_PIN_NUM];
	tgpio_intr_regs_t tgpio_intr_regs;
	/*
	 * TGPIO Clock select register - Selects clock for a pin
	 * BIT(0-19) - TGPIO Pins
	 *	0 – ptp_clk_200
	 *	1 – xtal_clk
	 */
	__IO_RW uint32_t tgpio_clk_sel;
	/* TGPIO 19.2MHz Clock gate register */
	__IO_RW uint32_t tgpio_xtal_clk_gate;
	/* TGPIO 200MHz Clock gate register */
	__IO_RW uint32_t tgpio_ptp_clk_gate;
	/*
	 * Input Time Stamp select registers
	 * tgpio_ts_sel[0] : TGPIO0 to TGPIO15
	 * tgpio_ts_sel[2] : TGPIO16 to TGPIO19
	 *	2 bits per TGPIO pin
	 *	00 – TMT0 timestamp
	 *	01 – TMT1 timestamp
	 *	10 – TMT2 timestamp
	 *	11 – Local ART
	 */
	__IO_RW uint32_t tgpio_ts_sel[2];
	/*
	 * TMT Clock Select Register  Selects clock for TMT0-TMT2
	 * BIT(31:3) - Reserved
	 * BIT(2:0)  – TMT0-TMT2
	 *		0 - ptp_clk_200
	 *		1 – xtal_clk
	 */
	__IO_RW uint32_t tgpio_tmt_clk_sel;
	__IO_R uint32_t reserved_1;
	/**
	 * BIT(31:6) - Reserved
	 * BIT(5) - Enable CTS between PTM & TMT_2. Cleared by HW
	 * BIT(4) - Enable CTS between ART & TMT_2. Cleared by HW
	 * BIT(3) - Enable CTS between PTM & TMT_1. Cleared by HW
	 * BIT(2) - Enable CTS between ART & TMT_1. Cleared by HW
	 * BIT(1) - Enable CTS between PTM & TMT_0. Cleared by HW
	 * BIT(0) - Enable CTS between ART & TMT_0. Cleared by HW
	 */
	__IO_RW uint32_t tgpio_cts_enable;
	/**
	 * BIT(31:6) - Reserved
	 * BIT(5) - CTS between PTM & TMT_2 valid
	 * BIT(4) - CTS between ART & TMT_2 valid
	 * BIT(3) - CTS between PTM & TMT_1 valid
	 * BIT(2) - CTS between ART & TMT_1 valid
	 * BIT(1) - CTS between PTM & TMT_0 valid
	 * BIT(0) - CTS between ART & TMT_0 valid
	 */
	__IO_RW uint32_t tgpio_cts_valid;
	__IO_R uint32_t reserved_2[50];
	tgpio_tmt_regs_t tgpio_tmt[TGPIO_MAX_TIMERS];
} tgpio_regs_t;

#endif /* _INTEL_TGPIO_REGS_H_ */
