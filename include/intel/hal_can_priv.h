/*
 * Copyright (c) 2021 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _INTEL_HAL_CAN_PRIV_H_
#define _INTEL_HAL_CAN_PRIV_H_

/**
 * Message RAM structure
 */
/**
 * Message RAM structure
 */
struct can_msg_ram_t {
	__IO_RW uint32_t *std_filters; /**< Start address for std filters */
	__IO_RW uint32_t *ext_filters; /**< Start address for ext filters */
	__IO_RW uint32_t *rx_fifo_0;   /**< Start address for RX FIFO 0 */
	__IO_RW uint32_t *rx_fifo_1;   /**< Start address for RX FIFO 1 */
	__IO_RW uint32_t *rx_buf;      /**< Start address for RX Buf */
	__IO_RW uint32_t *tx_evt_fifo; /**< Start address for TX Event FIFO */
	__IO_RW uint32_t *tx_buf;      /**< Start address for TX Buf & FIFO */
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
	struct can_msg_ram_t msg_ram;
	struct can_msg_ram_t msg_ram_phy;
	/**< Current state of CAN controller */
	enum can_states state;
	/**< Current mode of CAN controller */
	enum can_op_mode mode;
	/**< User callback for notifying events/errors */
	can_callback_t cb;
};

/* CAN context data structure */
typedef struct {
	intel_instance_t inst;
	enum can_id id;
	struct can_config_t cfg_regs;
} intel_can_context;

#endif /* _INTEL_HAL_CAN_PRIV_H_ */
