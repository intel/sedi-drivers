/*
 * Copyright (c) 2021 Intel Corporation
 *
 * SPDX-License-Identifier:  BSD-3-Clause
 */

#ifndef _SEDI_DRIVER_CAN_REGDEF_H_
#define _SEDI_DRIVER_CAN_REGDEF_H_

#include "sedi_driver_common.h"

#define CAN_CLK_FREQ_HZ 20000000

#define CAN_SFT_RST_REG (0x40600074)
#define CAN_SFT_RST_MASK (0x3)

#define CAN0_REG_START 0x50300000
#define CAN_REG_OFFSET 0x10000
#define CAN1_REG_START (CAN0_REG_START + CAN_REG_OFFSET)

#define CAN_MSG_RAM_OFFSET 0x800
#define CAN0_MSG_RAM_START (CAN0_REG_START + CAN_MSG_RAM_OFFSET)
#define CAN1_MSG_RAM_START (CAN1_REG_START + CAN_MSG_RAM_OFFSET)

#define PARITY_STAT_OFFSET 0x500
#define PARITY0_STAT_REG_START (CAN0_REG_START + PARITY_STAT_OFFSET)

#define PARITY_CTRL_OFFSET 0x600
#define PARITY0_CTRL_REG_START (CAN0_REG_START + PARITY_CTRL_OFFSET)

#define PARITY1_STAT_REG_START (CAN1_REG_START + PARITY_STAT_OFFSET)
#define PARITY1_CTRL_REG_START (CAN1_REG_START + PARITY_CTRL_OFFSET)

#define CAN_MAX_STD_FILTER_WORDS (128)
#define CAN_MAX_EXT_FILTER_WORDS (128)
#define CAN_WORDS_PER_EXT_FITER (2)

/**
 * CAN registers
 */
struct can_regs_t {
	__IO_R uint32_t crel;
	__IO_R uint32_t endn;

	__IO_RW uint32_t cust;
	__IO_RW uint32_t dbtp;
	__IO_RW uint32_t test;
	__IO_RW uint32_t rwd;
	__IO_RW uint32_t cccr;
	__IO_RW uint32_t nbtp;
	__IO_RW uint32_t tscc;
	__IO_RW uint32_t tscv;
	__IO_RW uint32_t tocc;
	__IO_RW uint32_t tocv;
	__IO_R uint32_t reserved1[4];
	__IO_R uint32_t ecr;
	__IO_R uint32_t psr;
	__IO_R uint32_t tdcr;
	__IO_R uint32_t reserved2[1];
	__IO_RW uint32_t ir;
	__IO_RW uint32_t ie;
	__IO_RW uint32_t ils;
	__IO_RW uint32_t ile;
	__IO_R uint32_t reserved3[8];
	__IO_RW uint32_t gfc;
	__IO_RW uint32_t sidfc;
	__IO_RW uint32_t xidfc;
	__IO_R uint32_t reserved4[1];
	__IO_RW uint32_t xidam;
	__IO_R uint32_t hpms;
	__IO_RW uint32_t ndata1;
	__IO_RW uint32_t ndata2;
	__IO_RW uint32_t rxf0c;
	__IO_R uint32_t rxf0s;
	__IO_RW uint32_t rxf0a;
	__IO_RW uint32_t rxbc;
	__IO_RW uint32_t rxf1c;
	__IO_R uint32_t rxf1s;
	__IO_RW uint32_t rxf1a;
	__IO_RW uint32_t rxesc;
	__IO_RW uint32_t txbc;
	__IO_R uint32_t txfqs;
	__IO_RW uint32_t txesc;
	__IO_R uint32_t txbrp;
	__IO_RW uint32_t txbar;
	__IO_RW uint32_t txbcr;
	__IO_R uint32_t txbto;
	__IO_R uint32_t txbcf;
	__IO_RW uint32_t txbtie;
	__IO_RW uint32_t txbcie;
	__IO_R uint32_t reserved5[2];
	__IO_RW uint32_t txefc;
	__IO_R uint32_t txefs;
	__IO_RW uint32_t txefa;
	__IO_R uint32_t reserved6[1];
	__IO_RW uint32_t tttmc;
	__IO_RW uint32_t ttrmc;
	__IO_RW uint32_t ttocf;
	__IO_RW uint32_t ttmlm;
	__IO_RW uint32_t turcf;
	__IO_RW uint32_t ttocn;
	__IO_RW uint32_t ttgtp;
	__IO_RW uint32_t tttmk;
	__IO_RW uint32_t ttir;
	__IO_RW uint32_t ttie;
	__IO_RW uint32_t ttils;
	__IO_R uint32_t ttost;
	__IO_R uint32_t turna;
	__IO_R uint32_t ttlgt;
	__IO_R uint32_t ttctc;
	__IO_R uint32_t ttcpt;
	__IO_R uint32_t ttcsm;
};

/**
 * Parity stat registers
 */
struct parity_stat_regs_t {
	__IO_R uint32_t msg_ram_size;
	__IO_RW uint32_t ctl;
	__IO_RW uint32_t int_ctl;
	__IO_R uint32_t int_stat;
	__IO_R uint32_t msg_ram_conflict_state;
};

/**
 * Parity control registers
 */
struct parity_ctrl_regs_t {
	__IO_RW uint32_t csr;
	__IO_R uint32_t err_offset;
	__IO_RW uint32_t einj_ctl_stat;
	__IO_RW uint32_t einj_offset;
	__IO_RW uint32_t einj_data_mask;
	__IO_RW uint32_t einj_parity_mask;
};

/**
 * CAN pm_registers
 */
typedef struct {
	__IO_RW uint32_t cust;
	__IO_RW uint32_t dbtp;
	__IO_RW uint32_t test;
	__IO_RW uint32_t rwd;
	__IO_RW uint32_t cccr;
	__IO_RW uint32_t nbtp;
	__IO_RW uint32_t tscc;
	__IO_RW uint32_t tscv;
	__IO_RW uint32_t tocc;
	__IO_RW uint32_t tocv;
	__IO_RW uint32_t ie;
	__IO_RW uint32_t ils;
	__IO_RW uint32_t ile;
	__IO_RW uint32_t gfc;
	__IO_RW uint32_t sidfc;
	__IO_RW uint32_t xidfc;
	__IO_RW uint32_t xidam;
	__IO_RW uint32_t rxf0c;
	__IO_RW uint32_t rxbc;
	__IO_RW uint32_t rxf1c;
	__IO_RW uint32_t rxesc;
	__IO_RW uint32_t txbc;
	__IO_RW uint32_t txesc;
	__IO_RW uint32_t txbtie;
	__IO_RW uint32_t txbcie;
	__IO_RW uint32_t txefc;

	/**
	 * Parity control registers
	 */
	__IO_RW uint32_t csr;
	__IO_RW uint32_t einj_ctl_stat;
	__IO_RW uint32_t einj_offset;
	__IO_RW uint32_t einj_data_mask;
	__IO_RW uint32_t einj_parity_mask;

	/**
	 * Parity stat registers
	 */
	__IO_RW uint32_t ctl;
	__IO_RW uint32_t int_ctl;

	/* Filter config */
	__IO_RW uint32_t std_filter[CAN_MAX_STD_FILTER_WORDS];
	__IO_RW uint32_t ext_filter[CAN_MAX_EXT_FILTER_WORDS];

	__IO_RW bool ctxt_valid;
} can_pm_regs_t;

/* Registers definitions and bit fields */
/* CREL */
#define CAN_CREL_DAY_POS 0
#define CAN_CREL_DAY_MASK (0xffu << CAN_CREL_DAY_POS)
#define CAN_CREL_MON_POS 8
#define CAN_CREL_MON_MASK (0xffu << CAN_CREL_MON_POS)
#define CAN_CREL_YEAR_POS 16
#define CAN_CREL_YEAR_MASK (0xfu << CAN_CREL_YEAR_POS)
#define CAN_CREL_SUBSTEP_POS 20
#define CAN_CREL_SUBSTEP_MASK (0xfu << CAN_CREL_SUBSTEP_POS)
#define CAN_CREL_STEP_POS 24
#define CAN_CREL_STEP_MASK (0xfu << CAN_CREL_STEP_POS)
#define CAN_CREL_REL_POS 28
#define CAN_CREL_REL_MASK (0xfu << CAN_CREL_REL_POS)
/* ENDN */
#define CAN_ENDN_ETV_POS 0
#define CAN_ENDN_ETV_MASK (0xffffffffu << CAN_ENDN_ETV_POS)
/* CUST */
#define CAN_CUST_CSV_POS 0
#define CAN_CUST_CSV_MASK (0xffffffffu << CAN_CUST_CSV_POS)
#define CAN_CUST_CSV(value) \
	((CAN_CUST_CSV_MASK & ((value) << CAN_CUST_CSV_POS)))
/* DBTP */
#define CAN_DBTP_DSJW_POS 0
#define CAN_DBTP_DSJW_MASK (0xfu << CAN_DBTP_DSJW_POS)
#define CAN_DBTP_DSJW(value) \
	((CAN_DBTP_DSJW_MASK & ((value) << CAN_DBTP_DSJW_POS)))
#define CAN_DBTP_DTSEG2_POS 4
#define CAN_DBTP_DTSEG2_MASK (0xfu << CAN_DBTP_DTSEG2_POS)
#define CAN_DBTP_DTSEG2(value) \
	((CAN_DBTP_DTSEG2_MASK & ((value) << CAN_DBTP_DTSEG2_POS)))
#define CAN_DBTP_DTSEG1_POS 8
#define CAN_DBTP_DTSEG1_MASK (0x1fu << CAN_DBTP_DTSEG1_POS)
#define CAN_DBTP_DTSEG1(value) \
	((CAN_DBTP_DTSEG1_MASK & ((value) << CAN_DBTP_DTSEG1_POS)))
#define CAN_DBTP_DBRP_POS 16
#define CAN_DBTP_DBRP_MASK (0x1fu << CAN_DBTP_DBRP_POS)
#define CAN_DBTP_DBRP(value) \
	((CAN_DBTP_DBRP_MASK & ((value) << CAN_DBTP_DBRP_POS)))
#define CAN_DBTP_TDC (0x1u << 23)
#define CAN_DBTP_TDC_DISABLED ~(0x1u << 23)
#define CAN_DBTP_TDC_ENABLED (0x1u << 23)

/* TEST */
#define CAN_TEST_TAM (0x1u << 0)
#define CAN_TEST_TAT (0x1u << 1)
#define CAN_TEST_CAM (0x1u << 2)
#define CAN_TEST_CAT (0x1u << 3)
#define CAN_TEST_LBCK (0x1u << 4)
#define CAN_TEST_LBCK_DISABLED ~(0x1u << 4)
#define CAN_TEST_LBCK_ENABLED (0x1u << 4)
#define CAN_TEST_TX_POS 5
#define CAN_TEST_TX_MASK (0x3u << CAN_TEST_TX_POS)
#define CAN_TEST_TX(value) ((CAN_TEST_TX_MASK & ((value) << CAN_TEST_TX_POS)))
#define CAN_TEST_TX_RESET ~(0x3u << 5)
#define CAN_TEST_TX_SAMPLE_POINT_MONITORING (0x1u << 5)
#define CAN_TEST_TX_DOMINANT (0x2u << 5)
#define CAN_TEST_TX_RECESSIVE (0x3u << 5)
#define CAN_TEST_RX (0x1u << 7)
/* RWD */
#define CAN_RWD_WDC_POS 0
#define CAN_RWD_WDC_MASK (0xffu << CAN_RWD_WDC_POS)
#define CAN_RWD_WDC(value) ((CAN_RWD_WDC_MASK & ((value) << CAN_RWD_WDC_POS)))
#define CAN_RWD_WDV_POS 8
#define CAN_RWD_WDV_MASK (0xffu << CAN_RWD_WDV_POS)
#define CAN_RWD_WDV(value) ((CAN_RWD_WDV_MASK & ((value) << CAN_RWD_WDV_POS)))
/* CCCR */
#define CAN_CCCR_INIT (0x1u << 0)
#define CAN_CCCR_INIT_DISABLED ~(0x1u << 0)
#define CAN_CCCR_INIT_ENABLED (0x1u << 0)
#define CAN_CCCR_CCE (0x1u << 1)
#define CAN_CCCR_CCE_PROTECTED ~(0x1u << 1)
#define CAN_CCCR_CCE_CONFIGURABLE (0x1u << 1)
#define CAN_CCCR_ASM (0x1u << 2)
#define CAN_CCCR_ASM_NORMAL ~(0x1u << 2)
#define CAN_CCCR_ASM_RESTRICTED (0x1u << 2)
#define CAN_CCCR_CSA (0x1u << 3)
#define CAN_CCCR_CSR (0x1u << 4)
#define CAN_CCCR_CSR_NO_CLOCK_STOP ~(0x1u << 4)
#define CAN_CCCR_CSR_CLOCK_STOP (0x1u << 4)
#define CAN_CCCR_MON (0x1u << 5)
#define CAN_CCCR_MON_DISABLED ~(0x1u << 5)
#define CAN_CCCR_MON_ENABLED (0x1u << 5)
#define CAN_CCCR_DAR (0x1u << 6)
#define CAN_CCCR_DAR_AUTO_RETX ~(0x1u << 6)
#define CAN_CCCR_DAR_NO_AUTO_RETX (0x1u << 6)
#define CAN_CCCR_TEST (0x1u << 7)
#define CAN_CCCR_TEST_DISABLED ~(0x1u << 7)
#define CAN_CCCR_TEST_ENABLED (0x1u << 7)
#define CAN_CCCR_FDOE_ENABLE (0x1u << 8)
#define CAN_CCCR_FDOE_DISABLE ~(0x1u << 8)
#define CAN_CCCR_BRSE_ENABLE (0x1u << 9)
#define CAN_CCCR_BRSE_DISABLE ~(0x1u << 9)
#define CAN_CCCR_PXHD_ENABLE (0x1u << 12)
#define CAN_CCCR_PXHD_DISABLE ~(0x1u << 12)
#define CAN_CCCR_EFBI_ENABLE (0x1u << 13)
#define CAN_CCCR_EFBI_DISABLE ~(0x1u << 13)
#define CAN_CCCR_TXP_ENABLE (0x1u << 14)
#define CAN_CCCR_TXP_DISABLE ~(0x1u << 14)
#define CAN_CCCR_NISO_ENABLE (0x1u << 15)
#define CAN_CCCR_NISO_DISABLE ~(0x1u << 15)

/* NBTP */
#define CAN_NBTP_NTSEG2_POS 0
#define CAN_NBTP_NTSEG2_MASK (0x7fu << CAN_NBTP_NTSEG2_POS)
#define CAN_NBTP_NTSEG2(value) \
	((CAN_NBTP_NTSEG2_MASK & ((value) << CAN_NBTP_NTSEG2_POS)))
#define CAN_NBTP_NTSEG1_POS 8
#define CAN_NBTP_NTSEG1_MASK (0xffu << CAN_NBTP_NTSEG1_POS)
#define CAN_NBTP_NTSEG1(value) \
	((CAN_NBTP_NTSEG1_MASK & ((value) << CAN_NBTP_NTSEG1_POS)))
#define CAN_NBTP_NBRP_POS 16
#define CAN_NBTP_NBRP_MASK (0x1ffu << CAN_NBTP_NBRP_POS)
#define CAN_NBTP_NBRP(value) \
	((CAN_NBTP_NBRP_MASK & ((value) << CAN_NBTP_NBRP_POS)))
#define CAN_NBTP_NSJW_POS 25
#define CAN_NBTP_NSJW_MASK (0x7fu << CAN_NBTP_NSJW_POS)
#define CAN_NBTP_NSJW(value) \
	((CAN_NBTP_NSJW_MASK & ((value) << CAN_NBTP_NSJW_POS)))

/* TSCC */
#define CAN_TSCC_TSS_POS 0
#define CAN_TSCC_TSS_MASK (0x3u << CAN_TSCC_TSS_POS)
#define CAN_TSCC_TSS(value) \
	((CAN_TSCC_TSS_MASK & ((value) << CAN_TSCC_TSS_POS)))
#define CAN_TSCC_TSS_ALWAYS_0 (0x0u << 0)
#define CAN_TSCC_TSS_TCP_INC (0x1u << 0)
#define CAN_TSCC_TSS_EXT_TIMESTAMP (0x2u << 0)
#define CAN_TSCC_TCP_POS 16
#define CAN_TSCC_TCP_MASK (0xfu << CAN_TSCC_TCP_POS)
#define CAN_TSCC_TCP(value) \
	((CAN_TSCC_TCP_MASK & ((value) << CAN_TSCC_TCP_POS)))
/* TSCV */
#define CAN_TSCV_TSC_POS 0
#define CAN_TSCV_TSC_MASK (0xffffu << CAN_TSCV_TSC_POS)
#define CAN_TSCV_TSC(value) \
	((CAN_TSCV_TSC_MASK & ((value) << CAN_TSCV_TSC_POS)))
/* TOCC */
#define CAN_TOCC_ETOC (0x1u << 0)
#define CAN_TOCC_ETOC_NO_TIMEOUT (0x0u << 0)
#define CAN_TOCC_ETOC_TOS_CONTROLLED (0x1u << 0)
#define CAN_TOCC_TOS_POS 1
#define CAN_TOCC_TOS_MASK (0x3u << CAN_TOCC_TOS_POS)
#define CAN_TOCC_TOS(value) \
	((CAN_TOCC_TOS_MASK & ((value) << CAN_TOCC_TOS_POS)))
#define CAN_TOCC_TOS_CONTINUOUS (0x0u << 1)
#define CAN_TOCC_TOS_TX_EV_TIMEOUT (0x1u << 1)
#define CAN_TOCC_TOS_RX0_EV_TIMEOUT (0x2u << 1)
#define CAN_TOCC_TOS_RX1_EV_TIMEOUT (0x3u << 1)
#define CAN_TOCC_TOP_POS 16
#define CAN_TOCC_TOP_MASK (0xffffu << CAN_TOCC_TOP_POS)
#define CAN_TOCC_TOP(value) \
	((CAN_TOCC_TOP_MASK & ((value) << CAN_TOCC_TOP_POS)))
/* TOCV */
#define CAN_TOCV_TOC_POS 0
#define CAN_TOCV_TOC_MASK (0xffffu << CAN_TOCV_TOC_POS)
#define CAN_TOCV_TOC(value) \
	((CAN_TOCV_TOC_MASK & ((value) << CAN_TOCV_TOC_POS)))
/* ECR */
#define CAN_ECR_TEC_POS 0
#define CAN_ECR_TEC_MASK (0xffu << CAN_ECR_TEC_POS)
#define CAN_ECR_REC_POS 8
#define CAN_ECR_REC_MASK (0x7fu << CAN_ECR_REC_POS)
#define CAN_ECR_RP (0x1u << 15)
#define CAN_ECR_CEL_POS 16
#define CAN_ECR_CEL_MASK (0xffu << CAN_ECR_CEL_POS)
/* PSR */
#define CAN_PSR_LEC_POS 0
#define CAN_PSR_LEC_MASK (0x7u << CAN_PSR_LEC_POS)
#define CAN_PSR_LEC_NO_ERROR (0x0u << 0)
#define CAN_PSR_LEC_STUFF_ERROR (0x1u << 0)
#define CAN_PSR_LEC_FORM_ERROR (0x2u << 0)
#define CAN_PSR_LEC_ACK_ERROR (0x3u << 0)
#define CAN_PSR_LEC_BIT1_ERROR (0x4u << 0)
#define CAN_PSR_LEC_BIT0_ERROR (0x5u << 0)
#define CAN_PSR_LEC_CRC_ERROR (0x6u << 0)
#define CAN_PSR_LEC_NO_CHANGE (0x7u << 0)
#define CAN_PSR_ACT_POS 3
#define CAN_PSR_ACT_MASK (0x3u << CAN_PSR_ACT_POS)
#define CAN_PSR_ACT_SYNCHRONIZING (0x0u << 3)
#define CAN_PSR_ACT_IDLE (0x1u << 3)
#define CAN_PSR_ACT_RECEIVER (0x2u << 3)
#define CAN_PSR_ACT_TRANSMITTER (0x3u << 3)
#define CAN_PSR_EP (0x1u << 5)
#define CAN_PSR_EW (0x1u << 6)
#define CAN_PSR_BO (0x1u << 7)
#define CAN_PSR_DLEC_POS 8
#define CAN_PSR_DLEC_MASK (0x7u << CAN_PSR_DLEC_POS)
#define CAN_PSR_RESI (0x1u << 11)
#define CAN_PSR_RBRS (0x1u << 12)
#define CAN_PSR_RFDF (0x1u << 13)
#define CAN_PSR_PXE (0x1u << 14)
#define CAN_PSR_TDCV_POS 16
#define CAN_PSR_TDCV_MASK (0x7fu << CAN_PSR_TDCV_POS)
/* TDCR */
#define CAN_TDCR_TDCF_POS 0
#define CAN_TDCR_TDCF_MASK (0x7fu << CAN_TDCR_TDCF_POS)
#define CAN_TDCR_TDCO_POS 8
#define CAN_TDCR_TDCO_MASK (0x7fu << CAN_TDCR_TDCO_POS)
/* IR */
#define CAN_IR_RF0N (0x1u << 0)
#define CAN_IR_RF0W (0x1u << 1)
#define CAN_IR_RF0F (0x1u << 2)
#define CAN_IR_RF0L (0x1u << 3)
#define CAN_IR_RF1N (0x1u << 4)
#define CAN_IR_RF1W (0x1u << 5)
#define CAN_IR_RF1F (0x1u << 6)
#define CAN_IR_RF1L (0x1u << 7)
#define CAN_IR_HPM (0x1u << 8)
#define CAN_IR_TC (0x1u << 9)
#define CAN_IR_TCF (0x1u << 10)
#define CAN_IR_TFE (0x1u << 11)
#define CAN_IR_TEFN (0x1u << 12)
#define CAN_IR_TEFW (0x1u << 13)
#define CAN_IR_TEFF (0x1u << 14)
#define CAN_IR_TEFL (0x1u << 15)
#define CAN_IR_TSW (0x1u << 16)
#define CAN_IR_MRAF (0x1u << 17)
#define CAN_IR_TOO (0x1u << 18)
#define CAN_IR_DRX (0x1u << 19)
#define CAN_IR_BEC (0x1u << 20)
#define CAN_IR_BEU (0x1u << 21)
#define CAN_IR_ELO (0x1u << 22)
#define CAN_IR_EP (0x1u << 23)
#define CAN_IR_EW (0x1u << 24)
#define CAN_IR_BO (0x1u << 25)
#define CAN_IR_WDI (0x1u << 26)
#define CAN_IR_PEA (0x1u << 27)
#define CAN_IR_PED (0x1u << 28)
#define CAN_IR_ARA (0x1u << 29)

#define CAN_IR_ERR							\
	(CAN_IR_ARA | CAN_IR_PED | CAN_IR_PEA | CAN_IR_BO | CAN_IR_EP |	\
	 CAN_IR_ELO | CAN_IR_BEU | CAN_IR_TOO | CAN_IR_MRAF | CAN_IR_EW)

#define CAN_IR_TX_EVT							      \
	(CAN_IR_TEFL | CAN_IR_TEFF | CAN_IR_TEFW | CAN_IR_TEFN | CAN_IR_TCF | \
	 CAN_IR_TC)

#define CAN_IR_RX_EVT \
	(CAN_IR_RF1N | CAN_IR_RF0N | CAN_IR_RF1L | CAN_IR_RF0L | CAN_IR_DRX)

/* IE */
#define CAN_IE_RF0NE (0x1u << 0)
#define CAN_IE_RF0WE (0x1u << 1)
#define CAN_IE_RF0FE (0x1u << 2)
#define CAN_IE_RF0LE (0x1u << 3)
#define CAN_IE_RF1NE (0x1u << 4)
#define CAN_IE_RF1WE (0x1u << 5)
#define CAN_IE_RF1FE (0x1u << 6)
#define CAN_IE_RF1LE (0x1u << 7)
#define CAN_IE_HPME (0x1u << 8)
#define CAN_IE_TCE (0x1u << 9)
#define CAN_IE_TCFE (0x1u << 10)
#define CAN_IE_TFEE (0x1u << 11)
#define CAN_IE_TEFNE (0x1u << 12)
#define CAN_IE_TEFWE (0x1u << 13)
#define CAN_IE_TEFFE (0x1u << 14)
#define CAN_IE_TEFLE (0x1u << 15)
#define CAN_IE_TSWE (0x1u << 16)
#define CAN_IE_MRAFE (0x1u << 17)
#define CAN_IE_TOOE (0x1u << 18)
#define CAN_IE_DRXE (0x1u << 19)
#define CAN_IE_BECE (0x1u << 20)
#define CAN_IE_BEUE (0x1u << 21)
#define CAN_IE_ELOE (0x1u << 22)
#define CAN_IE_EPE (0x1u << 23)
#define CAN_IE_EWE (0x1u << 24)
#define CAN_IE_BOE (0x1u << 25)
#define CAN_IE_WDIE (0x1u << 26)
#define CAN_IE_PEAE (0x1u << 27)
#define CAN_IE_PEDE (0x1u << 28)
#define CAN_IE_ARAE (0x1u << 29)

#define CAN_IE_ALL (0xFFFFFFFF)

/* ILE */
#define CAN_ILE_EINT0 (0x1u << 0)
#define CAN_ILE_EINT1 (0x1u << 1)
/* GFC */
#define CAN_GFC_RRFE (0x1u << 0)
#define CAN_GFC_RRFE_FILTER (0x0u << 0)
#define CAN_GFC_RRFE_REJECT (0x1u << 0)
#define CAN_GFC_RRFS (0x1u << 1)
#define CAN_GFC_RRFS_FILTER (0x0u << 1)
#define CAN_GFC_RRFS_REJECT (0x1u << 1)
#define CAN_GFC_ANFE_POS 2
#define CAN_GFC_ANFE_MASK (0x3u << CAN_GFC_ANFE_POS)
#define CAN_GFC_ANFE(value) \
	((CAN_GFC_ANFE_MASK & ((value) << CAN_GFC_ANFE_POS)))
#define CAN_GFC_ANFE_RX_FIFO_0 (0x0u << 2)
#define CAN_GFC_ANFE_RX_FIFO_1 (0x1u << 2)
#define CAN_GFC_ANFS_POS 4
#define CAN_GFC_ANFS_MASK (0x3u << CAN_GFC_ANFS_POS)
#define CAN_GFC_ANFS(value) \
	((CAN_GFC_ANFS_MASK & ((value) << CAN_GFC_ANFS_POS)))
#define CAN_GFC_ANFS_RX_FIFO_0 (0x0u << 4)
#define CAN_GFC_ANFS_RX_FIFO_1 (0x1u << 4)
/* SIDFC */
#define CAN_SIDFC_FLSSA_POS 2
#define CAN_SIDFC_FLSSA_MASK (0x3fffu << CAN_SIDFC_FLSSA_POS)
#define CAN_SIDFC_FLSSA(value) \
	((CAN_SIDFC_FLSSA_MASK & ((value) << CAN_SIDFC_FLSSA_POS)))
#define CAN_SIDFC_LSS_POS 16
#define CAN_SIDFC_LSS_MASK (0xffu << CAN_SIDFC_LSS_POS)
#define CAN_SIDFC_LSS(value) \
	((CAN_SIDFC_LSS_MASK & ((value) << CAN_SIDFC_LSS_POS)))
/* XIDFC */
#define CAN_XIDFC_FLESA_POS 2
#define CAN_XIDFC_FLESA_MASK (0x3fffu << CAN_XIDFC_FLESA_POS)
#define CAN_XIDFC_FLESA(value) \
	((CAN_XIDFC_FLESA_MASK & ((value) << CAN_XIDFC_FLESA_POS)))
#define CAN_XIDFC_LSE_POS 16
#define CAN_XIDFC_LSE_MASK (0x7fu << CAN_XIDFC_LSE_POS)
#define CAN_XIDFC_LSE(value) \
	((CAN_XIDFC_LSE_MASK & ((value) << CAN_XIDFC_LSE_POS)))
/* XIDAM */
#define CAN_XIDAM_EIDM_POS 0
#define CAN_XIDAM_EIDM_MASK (0x1fffffffu << CAN_XIDAM_EIDM_POS)
#define CAN_XIDAM_EIDM(value) \
	((CAN_XIDAM_EIDM_MASK & ((value) << CAN_XIDAM_EIDM_POS)))
/* HPMS */
#define CAN_HPMS_BIDX_POS 0
#define CAN_HPMS_BIDX_MASK (0x3fu << CAN_HPMS_BIDX_POS)
#define CAN_HPMS_MSI_POS 6
#define CAN_HPMS_MSI_MASK (0x3u << CAN_HPMS_MSI_POS)
#define CAN_HPMS_MSI_NO_FIFO_SEL (0x0u << 6)
#define CAN_HPMS_MSI_LOST (0x1u << 6)
#define CAN_HPMS_MSI_FIFO_0 (0x2u << 6)
#define CAN_HPMS_MSI_FIFO_1 (0x3u << 6)
#define CAN_HPMS_FIDX_POS 8
#define CAN_HPMS_FIDX_MASK (0x7fu << CAN_HPMS_FIDX_POS)
#define CAN_HPMS_FLST (0x1u << 15)
/* RXF0C */
#define CAN_RXF0C_F0SA_POS 2
#define CAN_RXF0C_F0SA_MASK (0x3fffu << CAN_RXF0C_F0SA_POS)
#define CAN_RXF0C_F0SA(value) \
	((CAN_RXF0C_F0SA_MASK & ((value) << CAN_RXF0C_F0SA_POS)))
#define CAN_RXF0C_F0S_POS 16
#define CAN_RXF0C_F0S_MASK (0x7fu << CAN_RXF0C_F0S_POS)
#define CAN_RXF0C_F0S(value) \
	((CAN_RXF0C_F0S_MASK & ((value) << CAN_RXF0C_F0S_POS)))
#define CAN_RXF0C_F0WM_POS 24
#define CAN_RXF0C_F0WM_MASK (0x7fu << CAN_RXF0C_F0WM_POS)
#define CAN_RXF0C_F0WM(value) \
	((CAN_RXF0C_F0WM_MASK & ((value) << CAN_RXF0C_F0WM_POS)))
#define CAN_RXF0C_F0OM_POS 31
#define CAN_RXF0C_F0OM(value) (value << CAN_RXF0C_F0OM_POS)
/* RXF0S */
#define CAN_RXF0S_F0FL_POS 0
#define CAN_RXF0S_F0FL_MASK (0x7fu << CAN_RXF0S_F0FL_POS)
#define CAN_RXF0S_F0GI_POS 8
#define CAN_RXF0S_F0GI_MASK (0x3fu << CAN_RXF0S_F0GI_POS)
#define CAN_RXF0S_F0PI_POS 16
#define CAN_RXF0S_F0PI_MASK (0x3fu << CAN_RXF0S_F0PI_POS)
#define CAN_RXF0S_F0F (0x1u << 24)
#define CAN_RXF0S_RF0L (0x1u << 25)
/* RXF0A */
#define CAN_RXF0A_F0AI_POS 0
#define CAN_RXF0A_F0AI_MASK (0x3fu << CAN_RXF0A_F0AI_POS)
#define CAN_RXF0A_F0AI(value) \
	((CAN_RXF0A_F0AI_MASK & ((value) << CAN_RXF0A_F0AI_POS)))
/* RXBC */
#define CAN_RXBC_RBSA_POS 2
#define CAN_RXBC_RBSA_MASK (0x3fffu << CAN_RXBC_RBSA_POS)
#define CAN_RXBC_RBSA(value) \
	((CAN_RXBC_RBSA_MASK & ((value) << CAN_RXBC_RBSA_POS)))
/* RXF1C */
#define CAN_RXF1C_F1SA_POS 2
#define CAN_RXF1C_F1SA_MASK (0x3fffu << CAN_RXF1C_F1SA_POS)
#define CAN_RXF1C_F1SA(value) \
	((CAN_RXF1C_F1SA_MASK & ((value) << CAN_RXF1C_F1SA_POS)))
#define CAN_RXF1C_F1S_POS 16
#define CAN_RXF1C_F1S_MASK (0x7fu << CAN_RXF1C_F1S_POS)
#define CAN_RXF1C_F1S(value) \
	((CAN_RXF1C_F1S_MASK & ((value) << CAN_RXF1C_F1S_POS)))
#define CAN_RXF1C_F1WM_POS 24
#define CAN_RXF1C_F1WM_MASK (0x7fu << CAN_RXF1C_F1WM_POS)
#define CAN_RXF1C_F1WM(value) \
	((CAN_RXF1C_F1WM_MASK & ((value) << CAN_RXF1C_F1WM_POS)))
#define CAN_RXF1C_F1OM_POS 31
#define CAN_RXF1C_F1OM(value) (value << CAN_RXF1C_F1OM_POS)

/* RXF1S */
#define CAN_RXF1S_F1FL_POS 0
#define CAN_RXF1S_F1FL_MASK (0x7fu << CAN_RXF1S_F1FL_POS)
#define CAN_RXF1S_F1GI_POS 8
#define CAN_RXF1S_F1GI_MASK (0x3fu << CAN_RXF1S_F1GI_POS)
#define CAN_RXF1S_F1PI_POS 16
#define CAN_RXF1S_F1PI_MASK (0x3fu << CAN_RXF1S_F1PI_POS)
#define CAN_RXF1S_F1F (0x1u << 24)
#define CAN_RXF1S_RF1L (0x1u << 25)
#define CAN_RXF1S_DMS_POS 30
#define CAN_RXF1S_DMS_MASK (0x3u << CAN_RXF1S_DMS_POS)
#define CAN_RXF1S_DMS_IDLE (0x0u << 30)
#define CAN_RXF1S_DMS_MSG_A (0x1u << 30)
#define CAN_RXF1S_DMS_MSG_AB (0x2u << 30)
#define CAN_RXF1S_DMS_MSG_ABC (0x3u << 30)
/* RXFIA */
#define CAN_RXF1A_F1AI_POS 0
#define CAN_RXF1A_F1AI_MASK (0x3fu << CAN_RXF1A_F1AI_POS)
#define CAN_RXF1A_F1AI(value) \
	((CAN_RXF1A_F1AI_MASK & ((value) << CAN_RXF1A_F1AI_POS)))
/* RXESC */
#define CAN_RXESC_F0DS_POS 0
#define CAN_RXESC_F0DS_MASK (0x7u << CAN_RXESC_F0DS_POS)
#define CAN_RXESC_F0DS(value) \
	((CAN_RXESC_F0DS_MASK & ((value) << CAN_RXESC_F0DS_POS)))
#define CAN_RXESC_F1DS_POS 4
#define CAN_RXESC_F1DS_MASK (0x7u << CAN_RXESC_F1DS_POS)
#define CAN_RXESC_F1DS(value) \
	((CAN_RXESC_F1DS_MASK & ((value) << CAN_RXESC_F1DS_POS)))
#define CAN_RXESC_RBDS_POS 8
#define CAN_RXESC_RBDS_MASK (0x7u << CAN_RXESC_RBDS_POS)
#define CAN_RXESC_RBDS(value) \
	((CAN_RXESC_RBDS_MASK & ((value) << CAN_RXESC_RBDS_POS)))
/* TXBC */
#define CAN_TXBC_TBSA_POS 2
#define CAN_TXBC_TBSA_MASK (0x3fffu << CAN_TXBC_TBSA_POS)
#define CAN_TXBC_TBSA(value) \
	((CAN_TXBC_TBSA_MASK & ((value) << CAN_TXBC_TBSA_POS)))
#define CAN_TXBC_NDTB_POS 16
#define CAN_TXBC_NDTB_MASK (0x3fu << CAN_TXBC_NDTB_POS)
#define CAN_TXBC_NDTB(value) \
	((CAN_TXBC_NDTB_MASK & ((value) << CAN_TXBC_NDTB_POS)))
#define CAN_TXBC_TFQS_POS 24
#define CAN_TXBC_TFQS_MASK (0x3fu << CAN_TXBC_TFQS_POS)
#define CAN_TXBC_TFQS(value) \
	((CAN_TXBC_TFQS_MASK & ((value) << CAN_TXBC_TFQS_POS)))
#define CAN_TXBC_TFQM (0x1u << 30)
/* TXFQS */
#define CAN_TXFQS_TFFL_POS 0
#define CAN_TXFQS_TFFL_MASK (0x3fu << CAN_TXFQS_TFFL_POS)
#define CAN_TXFQS_TFGI_POS 8
#define CAN_TXFQS_TFGI_MASK (0x1fu << CAN_TXFQS_TFGI_POS)
#define CAN_TXFQS_TFQPI_POS 16
#define CAN_TXFQS_TFQPI_MASK (0x1fu << CAN_TXFQS_TFQPI_POS)
#define CAN_TXFQS_TFQF (0x1u << 21)
/* TXESC */
#define CAN_TXESC_TBDS_POS 0
#define CAN_TXESC_TBDS_MASK (0x7u << CAN_TXESC_TBDS_POS)
#define CAN_TXESC_TBDS(value) \
	((CAN_TXESC_TBDS_MASK & ((value) << CAN_TXESC_TBDS_POS)))
#define CAN_TXESC_TBDS_8_BYTE (0x0u << 0)
#define CAN_TXESC_TBDS_12_BYTE (0x1u << 0)
#define CAN_TXESC_TBDS_16_BYTE (0x2u << 0)
#define CAN_TXESC_TBDS_20_BYTE (0x3u << 0)
#define CAN_TXESC_TBDS_24_BYTE (0x4u << 0)
#define CAN_TXESC_TBDS_32_BYTE (0x5u << 0)
#define CAN_TXESC_TBDS_48_BYTE (0x6u << 0)
#define CAN_TXESC_TBDS_64_BYTE (0x7u << 0)
/* TXEFC */
#define CAN_TXEFC_EFSA_POS 2
#define CAN_TXEFC_EFSA_MASK (0x3fffu << CAN_TXEFC_EFSA_POS)
#define CAN_TXEFC_EFSA(value) \
	((CAN_TXEFC_EFSA_MASK & ((value) << CAN_TXEFC_EFSA_POS)))
#define CAN_TXEFC_EFS_POS 16
#define CAN_TXEFC_EFS_MASK (0x3fu << CAN_TXEFC_EFS_POS)
#define CAN_TXEFC_EFS(value) \
	((CAN_TXEFC_EFS_MASK & ((value) << CAN_TXEFC_EFS_POS)))
#define CAN_TXEFC_EFWM_POS 24
#define CAN_TXEFC_EFWM_MASK (0x3fu << CAN_TXEFC_EFWM_POS)
#define CAN_TXEFC_EFWM(value) \
	((CAN_TXEFC_EFWM_MASK & ((value) << CAN_TXEFC_EFWM_POS)))
/* TXEFS */
#define CAN_TXEFS_EFFL_POS 0
#define CAN_TXEFS_EFFL_MASK (0x3fu << CAN_TXEFS_EFFL_POS)
#define CAN_TXEFS_EFGI_POS 8
#define CAN_TXEFS_EFGI_MASK (0x1fu << CAN_TXEFS_EFGI_POS)
#define CAN_TXEFS_EFPI_POS 16
#define CAN_TXEFS_EFPI_MASK (0x1fu << CAN_TXEFS_EFPI_POS)
#define CAN_TXEFS_EFF (0x1u << 24)
#define CAN_TXEFS_TEFL (0x1u << 25)
/* TXEFA */
#define CAN_TXEFA_EFAI_POS 0
#define CAN_TXEFA_EFAI_MASK (0x1fu << CAN_TXEFA_EFAI_POS)
#define CAN_TXEFA_EFAI(value) \
	((CAN_TXEFA_EFAI_MASK & ((value) << CAN_TXEFA_EFAI_POS)))

/* parity state registers */
/* CTL */
#define PARITY_CTL_ERR_DATA_DISABLE (0x1)

/* INT_CTL */
#define PARITY_INT_CTL_PRIMARY_INTR_ENABLE (0x1 << 0x0)
#define PARITY_INT_CTL_PUNIT_INTR_ENABLE (0x1 << 0x1)

/* INT_STAT */
#define PARITY_INT_STAT_THIS_CAN_CONT_INT_MASK (0x1)
#define PARITY_INT_STAT_THIS_CAN_PERR_INT_MASK (0x2)
#define PARITY_INT_STAT_OTHER_CAN_CONT_INT_MASK (0x4)
#define PARITY_INT_STAT_OTHER_CAN_PERR_INT_MASK (0x8)
#define PARITY_INT_STAT_ERR			   \
	(PARITY_INT_STAT_OTHER_CAN_PERR_INT_MASK | \
	 PARITY_INT_STAT_THIS_CAN_PERR_INT_MASK)

/* MSG_RAM_CONFLICT_STAT */
#define PARITY_ADDR_STAT_OFFSET_MASK (0x7fff)
#define PARITY_ADDR_STAT_CONFLICT_OCCURED_MASK (0x1 << 16)

/* parity control register */
/* PAR_CTL_STAT */
#define PARITY_CTL_PAR_ENABLE (0x1 << 0)
#define PARITY_CTL_PAR_INIT_PROGRESS (0x1 << 1)
#define PARITY_CTL_PAR_ERR_OCCURED (0x1 << 2)

/* PAR_ERR_OFFSET */
#define PARITY_CTL_PAR_ERR_OFFSET_MASK (0x7fff)
#define PARITY_CTL_PAR_ERR_IN_LOWER_16_BITS (0x1 << 16)
#define PARITY_CTL_PAR_ERR_IN_UPPER_16_BITS (0x1 << 17)

/* PAR_EINJ_CTL_STAT */
#define PARITY_CTL_ERR_INJ_ENABLE (0x1 << 0)
#define PARITY_CTL_ERR_INJ_MODE_CONTINIOUS (0x1 << 1)
#define PARITY_CTL_ERR_INJ_MODE_ONETIME (0x0 << 1)
#define PARITY_CTL_ERR_INJ_ONETIME_OCCURED (0x1 << 2)

/* PAR_EINJ_OFFSET */
#define PARITY_CTL_INJ_OFFSET_POS 0
#define PARITY_CTL_INJ_OFFSET_MASK (0x7fff << PARITY_CTL_INJ_OFFSET_POS)
#define PARITY_CTL_INJ_OFFSET(value) \
	((PARITY_CTL_INJ_OFFSET_MASK & ((value) << PARITY_CTL_INJ_OFFSET_POS)))

/* PAR_EINJ_DATA_MASK */

/* PAR_EINJ_PARITY_MASK */
#define PARITY_CTL_INJ_PAR_POS 0
#define PARITY_CTL_INJ_PAR_MASK (0x3 << PARITY_CTL_INJ_PAR_POS)
#define PARITY_CTL_INJ_PAR(value) \
	((PARITY_CTL_INJ_PAR_MASK & ((value) << PARITY_CTL_INJ_PAR_POS)))

#endif /*_SEDI_CAN_DRIVER_REGDEF_H_*/
