/*
 * Copyright (c) 2021 Intel Corporation
 *
 * SPDX-License-Identifier:  BSD-3-Clause
 */

#ifndef _INTEL_HAL_UART_PRIV_H_
#define _INTEL_HAL_UART_PRIV_H_

/* Transmit & Receive control. */
typedef struct {
	uint8_t tx_disable : 1;
	uint8_t rx_disable : 1;
} uart_xfer_ctrl_t;

/* Buffer pointers to store transmit / receive data for UART. */

typedef struct {
	uint32_t enable_unsol_rx;
	int32_t read_idx;
	int32_t write_idx;
	int32_t curr_len;
	const intel_uart_unsol_rx_t *unsol_rx;
} unsol_read_context_t;

typedef struct {
	intel_uart_transfer_t xfer;
	const intel_uart_io_vec_xfer_t *vec;
	uint32_t curr_count;
	uint8_t active;
} io_vec_cntxt_t;

typedef enum { WRITE, READ } dma_operation_type_t;

typedef struct {
	const intel_uart_dma_xfer_t *dma_xfer;
	dma_operation_type_t operation; /* READ/WRITE */
} uart_dma_ctxt_t;

/* UART context data structure */
struct intel_uart_context {
	intel_instance_t instance;
	__IO_RW void *regs;
	__IO_RW void *phy_regs;
	uint32_t write_pos;
	uint32_t read_pos;
	const intel_uart_transfer_t *uart_read_transfer;
	const intel_uart_transfer_t *uart_write_transfer;
	uint32_t iir_cache;
	uint32_t baud_rate_cache;
	uint32_t clk_speed_cache;
	uint32_t status_report_mask;
	uart_xfer_ctrl_t uart_xfer_ctrl;
	unsol_read_context_t unsol_read_ctxt;
	io_vec_cntxt_t vec_read_ctxt;
	io_vec_cntxt_t vec_write_ctxt;
	uint32_t uart_dma_hs_id; /**HW handshake id*/
	uart_dma_ctxt_t dma_ctxt;
};

#endif /* _INTEL_HAL_UART_PRIV_H_ */
