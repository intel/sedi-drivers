/*
 * Copyright (c) 2021 Intel Corporation
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _QEP_PRIV_H_
#define _QEP_PRIV_H_

#define SEDI_QEP_SFT_RST_REG (0x40600068)
#define SEDI_QEP_SFT_RST_MASK (0xF)

#define SEDI_QEP_CFG_EN (BIT(0))
#define SEDI_QEP_CFG_EN_OFFSET (0)

#define SEDI_QEP_CFG_FLT_EN (BIT(1))
#define SEDI_QEP_CFG_FLT_EN_OFFSET (1)

#define SEDI_QEP_CFG_EDGE_SEL_A (BIT(2))
#define SEDI_QEP_CFG_EDGE_SEL_A_OFFSET (2)

#define SEDI_QEP_CFG_EDGE_SEL_B (BIT(3))
#define SEDI_QEP_CFG_EDGE_SEL_B_OFFSET (3)

#define SEDI_QEP_CFG_EDGE_SEL_IDX (BIT(4))
#define SEDI_QEP_CFG_EDGE_SEL_IDX_OFFSET (4)

#define SEDI_QEP_CFG_SWAP_A_B (BIT(5))
#define SEDI_QEP_CFG_SWAP_A_B_OFFSET (5)

#define SEDI_QEP_CFG_OP_MODE (BIT(6))
#define SEDI_QEP_CFG_OP_MODE_OFFSET (6)

/*Read Only */
#define SEDI_QEP_CFG_PH_ERR (BIT(7))
#define SEDI_QEP_CFG_PH_ERR_OFFSET (7)

#define SEDI_QEP_CFG_CNT_RST_MODE (BIT(8))
#define SEDI_QEP_CFG_CNT_RST_MODE_OFFSET (8)

#define SEDI_QEP_CFG_IDX_GATE_SEL_OFFSET (9)
#define SEDI_QEP_CFG_IDX_GATE_SEL_MASK (3)

#define SEDI_QEP_CFG_CAP_MODE (BIT(11))
#define SEDI_QEP_CFG_CAP_MODE_OFFSET (11)

#define SEDI_QEP_CFG_CAP_FIFO_TH_OFFSET (12)
#define SEDI_QEP_CFG_CAP_FIFO_TH_MASK (7)

/*Read only */
#define SEDI_QEP_CFG_FIFO_EMPTY (BIT(15))
#define SEDI_QEP_CFG_FIFO_EMPTY_OFFSET (15)

/* PCLK frequency */
#define SEDI_QEP_PCLK_FREQ_HZ (100000000)
#define SEDI_QEP_NS_PER_SEC (1000000000)

#define SEDI_QEP_PCLK_PERIOD_NS (SEDI_QEP_NS_PER_SEC / SEDI_QEP_PCLK_FREQ_HZ)
#define SEDI_QEP_FRC_CLK_RES_NS	\
	(SEDI_QEP_PCLK_PERIOD_NS * (1 << SEDI_QEP_CAP_CLK_DIV))

#define SEDI_QEP_USEC_TO_PCLK_CYCLES(usec) \
	((usec * 1000) / SEDI_QEP_PCLK_PERIOD_NS)
#define SEDI_QEP_NS_TO_PCLK_CYCLES(ns) ((ns) / SEDI_QEP_PCLK_PERIOD_NS)

/* Time conversion for calculating rpm.*/
#define SEDI_QEP_USEC_PER_MINUTE (60 * 1000000)

/* Interrupt Status Register Fields */
#define SEDI_QEP_RAW_INT_STAT_WDT (BIT(0))
#define SEDI_QEP_RAW_INT_STAT_RST_DN (BIT(1))
#define SEDI_QEP_RAW_INT_STAT_RST_UP (BIT(2))
#define SEDI_QEP_RAW_INT_STAT_QEP_DIR (BIT(3))
#define SEDI_QEP_RAW_INT_STAT_FIFO_ENTRY (BIT(4))
#define SEDI_QEP_RAW_INT_STAT_FIFO_CRIT (BIT(5))

/* SEDI QEP Interrupts  */
#define SEDI_QEP_INT_WDT (BIT(0))
#define SEDI_QEP_INT_RST_DN (BIT(1))
#define SEDI_QEP_INT_RST_UP (BIT(2))
#define SEDI_QEP_INT_QEP_DIR (BIT(3))
#define SEDI_QEP_INT_FIFO_ENTRY (BIT(4))
#define SEDI_QEP_INT_FIFO_CRIT_PH_ERR (BIT(5))
#define SEDI_QEP_INT_ALL (0x3F)

#define SEDI_QEP_DECODE_INTS						\
	(SEDI_QEP_INT_WDT | SEDI_QEP_INT_RST_DN | SEDI_QEP_INT_RST_UP |	\
	 SEDI_QEP_INT_QEP_DIR | SEDI_QEP_INT_FIFO_CRIT_PH_ERR)

/* Counts per pulse*/
#define SEDI_QEP_COUNTS_PER_PULSE (4)

/* Max detectable RPM as per hardware design. */
#define SEDI_QEP_MAX_RPM (1000000)

/* Minimum filter width cycles supported. */
#define SEDI_QEP_MIN_FILTER_WIDTH_CYCLES (2)

/* Maximum filter width supported 20ms. */
#define SEDI_QEP_MAX_FILTER_WIDTH_NS (20000000)

/**
 *  QEP capture FIFO threshold.
 */

typedef enum {
	SEDI_QEP_CAP_FIFO_THRE_1        = 0x0,  /**< 1 Data entry threshold in FIFO. */
	SEDI_QEP_CAP_FIFO_THRE_2        = 0x1,  /**< 2 Data entry threshold in FIFO. */
	SEDI_QEP_CAP_FIFO_THRE_3        = 0x2,  /**< 3 Data entry threshold in FIFO. */
	SEDI_QEP_CAP_FIFO_THRE_4        = 0x3,  /**< 4 Data entry threshold in FIFO. */
	SEDI_QEP_CAP_FIFO_THRE_5        = 0x4,  /**< 5 Data entry threshold in FIFO. */
	SEDI_QEP_CAP_FIFO_THRE_6        = 0x5,  /**< 6 Data entry threshold in FIFO. */
	SEDI_QEP_CAP_FIFO_THRE_7        = 0x6,  /**< 7 Data entry threshold in FIFO. */
	SEDI_QEP_CAP_FIFO_THRE_8        = 0x7   /**< 8 Data entry threshold in FIFO. */
} sedi_qep_cap_fifo_thre_t;

/**
 *  QEP capture counter clock divider.
 */

typedef enum {
	SEDI_QEP_CAP_CLK_DIV_1          = 0x0,  /**< Clk div 1. */
	SEDI_QEP_CAP_CLK_DIV_2          = 0x1,  /**< Clk div 2. */
	SEDI_QEP_CAP_CLK_DIV_4          = 0x2,  /**< Clk div 4. */
	SEDI_QEP_CAP_CLK_DIV_8          = 0x3,  /**< Clk div 8. */
	SEDI_QEP_CAP_CLK_DIV_16         = 0x4,  /**< Clk div 16. */
	SEDI_QEP_CAP_CLK_DIV_32         = 0x5,  /**< Clk div 32. */
	SEDI_QEP_CAP_CLK_DIV_64         = 0x6,  /**< Clk div 64. */
	SEDI_QEP_CAP_CLK_DIV_128        = 0x7   /**< Clk div 128. */
} sedi_qep_cap_clk_div_t;

/* Fifo threshold for capture mode. */
#define SEDI_QEP_CAP_FIFO_THRESHOLD (SEDI_QEP_CAP_FIFO_THRE_3)

/* Clk divider for capture timestamp counter. */
#define SEDI_QEP_CAP_CLK_DIV (SEDI_QEP_CAP_CLK_DIV_1)

typedef struct {
	__IO_RW uint32_t qep_con;       /**< QEP Config register */
	__IO_RW uint32_t qep_flt;       /**< QEP Filter register */

	__IO_R uint32_t qep_count;      /**< QEP Count  register */
	__IO_RW uint32_t qep_max;       /**< QEP Max Count Register */
	__IO_RW uint32_t qep_wdt;       /**< QEP Watchdog timeout register */
	__IO_RW uint32_t qep_cap_div;   /**< QEP Capture Clk Divider */
	__IO_R uint32_t qep_cap_cntr;   /**< QEP Capture count register */
	__IO_R uint32_t qep_cap_buf;    /**< QEP Capture Buffer */
	__IO_RW uint32_t qep_int_stat;  /**< QEP Interrupt status register */
	__IO_RW uint32_t qep_int_mask;  /**< QEP Interrupt mask register */

} sedi_qep_reg_t;

typedef struct {
	__IO_RW uint32_t qep_con;       /**< QEP Config register */
	__IO_RW uint32_t qep_flt;       /**< QEP Filter register */
	__IO_RW uint32_t qep_max;       /**< QEP Max Count Register */
	__IO_RW uint32_t qep_wdt;       /**< QEP Watchdog timeout register */
	__IO_RW uint32_t qep_cap_div;   /**< QEP Capture Clk Divider */
	__IO_RW uint32_t qep_int_mask;  /**< QEP Interrupt mask register */
} sedi_qep_reg_ctxt_t;

typedef struct {
	sedi_qep_callback_t callback;   /**< User callback. */
	const void *usr_param;          /**< User param. */
	sedi_qep_dir_t last_known_dir;  /**< Last known direction of roation. */
	uint32_t cap_term_count;        /**< Term count for capture mode. */
	uint64_t *cap_buff;             /**< Capture buffer for capture mode. */
	uint32_t cap_current_count;     /**< Current capture count. */
	uint32_t cap_done_flag;         /**< Capture done flag. */
	uint32_t status_flag;           /**< Error flag. */
	sedi_qep_reg_ctxt_t saved_regs; /**< Saved registers for Power Mgt. */
	bool regs_valid;                /**< Saved regsiters valid. */
} sedi_qep_ctxt_t;

#endif /*_QEP_PRIV_H_*/
