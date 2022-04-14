/*
 * Copyright (c) 2021 Intel Corporation
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "sedi_driver_can_regdef.h"
#include "sedi_driver_can.h"

#define CAN_BIT_RATE_500BKBPS 0x08070f07
#define CAN_BIT_RATE_1MBPS 0x08030f07
#define CAN_BIT_RATE_2MBPS 0x08010f07

#define CAN_EXTENDED_ID_MASK 0x1FFFFFFF
#define CAN_DISABLE_ALL_INTR 0x00000000
#define CAN_SELECT_INTR_LINE_0 0x00000000
#define CAN_SELECT_INTR_LINE_1 0xFFFFFFFF
#define CAN_DISABLE_INTR_LINE 0x00
#define CAN_RESET_INTR_ALL 0xFFCFFFFF
#define CAN_RESET_NEW_DATA_REG 0xFFFFFFFF

#define TIMER_LIMIT (10000)

#define CAN0_MAX_STD_FILTER_CNT 128
#define CAN0_MAX_XTD_FILTER_CNT 64
#define CAN0_MAX_RX_FIFO0_CNT 64
#define CAN0_MAX_RX_FIFO1_CNT 64
#define CAN0_MAX_RX_BUF_CNT 64
#define CAN0_MAX_TX_EVTFIFO_CNT 32
#define CAN0_MAX_TX_BUF_CNT 32

#define CAN1_MAX_STD_FILTER_CNT 32
#define CAN1_MAX_XTD_FILTER_CNT 32
#define CAN1_MAX_RX_FIFO0_CNT 32
#define CAN1_MAX_RX_FIFO1_CNT 32
#define CAN1_MAX_RX_BUF_CNT 32
#define CAN1_MAX_TX_EVTFIFO_CNT 32
#define CAN1_MAX_TX_BUF_CNT 32

#define CAN_MAX_RESET_CNT 129

#define CAN_11_BIT_ID_MASK (0x7FF)
#define CAN_29_BIT_ID_MASK (0x1FFFFFFF)
#define CAN_ELMT_SIZE_MASK (0x1F)

#define CAN_STD_ID_POS 18
#define CAN_MSG_LEN_POS 16
#define CAN_MSG_FIDX_POS 24
#define CAN_MSG_MM_POS 24

#define CAN_MAX_NEW_DATA_NUM_1 32
#define CAN_MAX_NEW_DATA_NUM_2 64

#define CAN_SIDF_ELEM_SIZE 4
#define CAN_XIDF_ELEM_SIZE 8
#define CAN_RXB_MAX_ELEM_SIZE 72
#define CAN_TXB_MAX_ELEM_SIZE 72
#define CAN_TX_EVT_ELEM_SIZE 8
#define CAN_TT_MEM_ELEM_SIZE 8

#define CAN_MSGRAM_ADDR(addr) (0xFFFC & (addr))

/* RX/TX Buffer */
#define CAN_BUFFER_ESI_MASK (0x80000000)
#define CAN_BUFFER_RTR_MASK (0x20000000)
#define CAN_BUFFER_XTD_MASK (0x40000000)
#define CAN_BUFFER_EXT_ID_MASK (0x1FFFFFFF)
#define CAN_BUFFER_STD_ID_MASK (0x1FFC0000)
#define CAN_BUFFER_ANMF_MASK (0x80000000)
#define CAN_BUFFER_FIDX_MASK (0x3D000000)
#define CAN_BUFFER_FDF_MASK (0x00200000)
#define CAN_BUFFER_BRS_MASK (0x00100000)
#define CAN_BUFFER_DLC_MASK (0x000F0000)
#define CAN_BUFFER_RXTS_MASK (0x0000FFFF)
#define CAN_BUFFER_MM_MASK (0xFF000000)
#define CAN_BUFFER_EFC_MASK (0x00800000)
#define CAN_BUFFER_TIMESTAMP_MASK (0x0000FFFF)

/* TX Event */
#define CAN_TX_EVT_MASK (0x00C00000)

/* Standard Filter */
#define CAN_STD_FILTER_SFT_MASK (0x3 << 30)
#define CAN_STD_FILTER_SFT_RANGE (0x0 << 30)
#define CAN_STD_FILTER_SFT_DUAL_FILTER (0x1 << 30)
#define CAN_STD_FILTER_SFT_CLASSIC_FILTER (0x2 << 30)
#define CAN_STD_FILTER_SFEC_MASK (0x7 << 27)
#define CAN_STD_FILTER_SFEC_DISABLE ~(0x7 << 27)
#define CAN_STD_FILTER_SFEC_FIFO0 (0x1 << 27)
#define CAN_STD_FILTER_SFEC_FIFO1 (0x2 << 27)
#define CAN_STD_FILTER_SFEC_REJECT (0x3 << 27)
#define CAN_STD_FILTER_SFEC_PRIORITY (0x4 << 27)
#define CAN_STD_FILTER_SFEC_PRIORITY_FIFO0 (0x5 << 27)
#define CAN_STD_FILTER_SFEC_PRIORITY_FIFO1 (0x6 << 27)
#define CAN_STD_FILTER_SFEC_BUFFER (0x7 << 27)
#define CAN_STD_FILTER_SFID1_MASK (0x03FF << 16)
#define CAN_STD_FILTER_SFID2_MASK (0x3FF << 0)
#define CAN_STD_FILTER_SFID2_RX_BUFFER (0x0 << 9)

/* Extended Filter */
#define CAN_EXT_FILTER_EFEC_MASK (0x7 << 29)
#define CAN_EXT_FILTER_EFEC_DISABLE ~(0x7 << 29)
#define CAN_EXT_FILTER_EFEC_FIFO0 (0x1 << 29)
#define CAN_EXT_FILTER_EFEC_FIFO1 (0x2 << 29)
#define CAN_EXT_FILTER_EFEC_REJECT (0x3 << 29)
#define CAN_EXT_FILTER_EFEC_PRIORITY (0x4 << 29)
#define CAN_EXT_FILTER_EFEC_PRIORITY_FIFO0 (0x5 << 29)
#define CAN_EXT_FILTER_EFEC_PRIORITY_FIFO1 (0x6 << 29)
#define CAN_EXT_FILTER_EFEC_BUFFER (0x7 << 29)
#define CAN_EXT_FILTER_EFID1_MASK (0x1FFFFFFF)
#define CAN_EXT_FILTER_EFT_MASK (0x3 << 30)
#define CAN_EXT_FILTER_EFT_RANGE_FILTER (0x0 << 30)
#define CAN_EXT_FILTER_EFT_DUAL_FILTER (0x1 << 30)
#define CAN_EXT_FILTER_EFT_CLASSIC_FILTER (0x2 << 30)
#define CAN_EXT_FILTER_EFT_RANGE_NO_XIDAM (0x3 << 30)
#define CAN_EXT_FILTER_EFID2_MASK (0x1FFFFFFF)
#define CAN_EXT_FILTER_EFID2_RX_BUFFER (0x0 << 9)
#define CAN_EXT_FILTER_EFID2_DEBUG_A (0x1 << 9)
#define CAN_EXT_FILTER_EFID2_DEBUG_B (0x2 << 9)
#define CAN_EXT_FILTER_EFID2_DEBUG_C (0x3 << 9)
#define CAN_EXT_FILTER_EFID2_BUFFER(nmbr) (nmbr & 0x3F)
#define CAN_FILTER_ID1_POS 16
#define CAN_MEM_WORD_SIZE 4
#define CAN_HEADER_SIZE 2
#define CAN_INVALID_FILTER_NUM 255
/**
 * Message RAM structure
 */
struct can_msg_ram_t {
	__IO_RW uint32_t *std_filters;  /**< Start address for std filters */
	__IO_RW uint32_t *ext_filters;  /**< Start address for ext filters */
	__IO_RW uint32_t *rx_fifo_0;    /**< Start address for RX FIFO 0 */
	__IO_RW uint32_t *rx_fifo_1;    /**< Start address for RX FIFO 1 */
	__IO_RW uint32_t *rx_buf;       /**< Start address for RX Buf */
	__IO_RW uint32_t *tx_evt_fifo;  /**< Start address for TX Event FIFO */
	__IO_RW uint32_t *tx_buf;       /**< Start address for TX Buf & FIFO */
};

/**
 * CAN configuration structure
 */
struct can_config_t {
	/**< CAN IP registers */
	struct can_regs_t *regs;
	/**< CAN parity state registers */
	struct parity_stat_regs_t *parity_stat_regs;
	/**< CAN parity controller registers */
	struct parity_ctrl_regs_t *parity_ctrl_regs;
	/**< CAN parameters structure */
	struct can_params_t *params;
	/**< CAN message RAM structure */
	struct can_msg_ram_t *msg_ram;
	/**< Current state of CAN controller */
	enum can_states state;
	/**< Current mode of CAN controller */
	enum can_op_mode mode;
	/**< User callback for notifying events/errors */
	can_callback_t cb;
};
