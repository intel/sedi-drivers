/*
 * Copyright (c) 2021 Intel Corporation
 *
 * SPDX-License-Identifier:  BSD-3-Clause
 */

#include <intel/hal_uart.h>
#include "dw_uart_priv.h"

/* DMA driver requires to have a callback to be provided during init even
 * when in polled mode. Adding a dummy callback for this.
 */
static void intel_dma_poll_dummy_cb(IN intel_instance_t *dma_dev,
				    IN int channel, IN int event,
				    INOUT void *param)
{
	PARAM_UNUSED(event);
	PARAM_UNUSED(param);
	PARAM_UNUSED(dma_dev);
	PARAM_UNUSED(channel);
}

static void intel_dma_event_cb(IN intel_instance_t *dma_device,
			       IN int channel_id, IN int event,
			       INOUT void *param)
{
	(void)dma_device;
	(void)channel_id;

	struct intel_uart_context *uart = CONTAINER_OF(
	    (intel_instance_t *)param, struct intel_uart_context, instance);

	const intel_uart_dma_xfer_t *xfer = uart->dma_ctxt.dma_xfer;
	/* Program next transfer. */
	volatile intel_uart_reg_t *regs = uart->regs;
	uint32_t line_err_status = (regs->lsr & INTEL_UART_LSR_ERROR_BITS);

	if (uart->dma_ctxt.operation == READ) {

		if (event == INTEL_DMA_EVENT_TRANSFER_DONE) {
			if (xfer->callback) {
				xfer->callback(xfer->cb_param, INTEL_DRIVER_OK,
					       line_err_status, xfer->len);
			}
		} else {
			if (xfer->callback) {
				xfer->callback(xfer->cb_param,
					       INTEL_DRIVER_ERROR,
					       line_err_status, xfer->len);
			}
		}

	} else if (uart->dma_ctxt.operation == WRITE) {
		if (event == INTEL_DMA_EVENT_TRANSFER_DONE) {
			/* wait for transfer to complete as data may
			 * still be in the fifo/ tx shift regs. */
			while (!(regs->lsr & INTEL_UART_LSR_TEMT)) {
			}
			if (xfer->callback) {
				xfer->callback(xfer->cb_param, INTEL_DRIVER_OK,
					       line_err_status, xfer->len);
			}
		} else {
			if (xfer->callback) {
				xfer->callback(xfer->cb_param,
					       INTEL_DRIVER_ERROR,
					       line_err_status, xfer->len);
			}
		}
	}
	uart->dma_ctxt.dma_xfer = NULL;
}

static void io_vec_write_callback(void *data, int error, uint32_t status,
				  uint32_t len)
{
	struct intel_uart_context *uart = CONTAINER_OF(
	    (intel_instance_t *)data, struct intel_uart_context, instance);

	const intel_uart_io_vec_xfer_t *const vec_xfer =
	    uart->vec_write_ctxt.vec;

	uint32_t current_count;

	/* Increment the next count */
	current_count = ++uart->vec_write_ctxt.curr_count;
	PARAM_UNUSED(len);

	/* Error in write or transfer completed , call user callback.*/
	if (status || (current_count == vec_xfer->count)) {
		if (vec_xfer->callback) {

			/* Call callack with error. */
			vec_xfer->callback(vec_xfer->cb_data, error, status,
					   current_count);
		}
		/* Set active xfer to false. */
		uart->vec_write_ctxt.active = false;
		return;
	}

	/* Program next transfer. */
	volatile intel_uart_reg_t *regs = uart->regs;
	uart->vec_write_ctxt.xfer.data = vec_xfer->vec[current_count].base;
	uart->vec_write_ctxt.xfer.data_len = vec_xfer->vec[current_count].len;
	uart->write_pos = 0;
	uart->uart_write_transfer = &uart->vec_write_ctxt.xfer;

	/* Wait for last write transfer to finish completely. */
	while (!(regs->lsr & INTEL_UART_LSR_TEMT)) {
	}

	regs->iir_fcr =
	    (INTEL_UART_FCR_FIFOE | INTEL_UART_FCR_TX_0_RX_1_2_THRESHOLD);

	/* Enable TX holding reg empty interrupt. */
	regs->ier_dlh |= INTEL_UART_IER_ETBEI;
}

static void io_vec_read_callback(void *data, int error, uint32_t status,
				 uint32_t len)
{
	struct intel_uart_context *uart = CONTAINER_OF(
	    (intel_instance_t *)data, struct intel_uart_context, instance);
	const intel_uart_io_vec_xfer_t *const vec_xfer =
	    uart->vec_read_ctxt.vec;
	uint32_t current_count;
	PARAM_UNUSED(len);

	/* Increment the next count */
	current_count = ++uart->vec_read_ctxt.curr_count;

	/* Error in read or read completes, call user callback.*/
	if (status || (current_count == vec_xfer->count)) {
		if (vec_xfer->callback) {
			/* Call callack with error. */
			vec_xfer->callback(vec_xfer->cb_data, error, status,
					   current_count);
		}
		/* Set active xfer to false. */
		uart->vec_read_ctxt.active = false;
		return;
	}

	/* Program next transfer */
	volatile intel_uart_reg_t *regs = uart->regs;
	uart->vec_read_ctxt.xfer.data = vec_xfer->vec[current_count].base;
	uart->vec_read_ctxt.xfer.data_len = vec_xfer->vec[current_count].len;
	uart->read_pos = 0;
	uart->uart_read_transfer = &uart->vec_read_ctxt.xfer;

	/* Set threshold. */
	regs->iir_fcr =
	    (INTEL_UART_FCR_FIFOE | INTEL_UART_FCR_TX_0_RX_1_2_THRESHOLD);

	/*
	 * Enable both 'Receiver Data Available' and 'Receiver
	 * Line Status' interrupts.
	 */
	regs->ier_dlh |= INTEL_UART_IER_ERBFI | INTEL_UART_IER_ELSI;
}

static bool is_read_xfer_complete(const intel_instance_t *inst)
{
	struct intel_uart_context *uart =
	    CONTAINER_OF(inst, struct intel_uart_context, instance);
	const intel_uart_transfer_t *const transfer = uart->uart_read_transfer;

	return uart->read_pos >= transfer->data_len;
}

static bool is_write_xfer_complete(const intel_instance_t *inst)
{
	struct intel_uart_context *uart =
	    CONTAINER_OF(inst, struct intel_uart_context, instance);
	const intel_uart_transfer_t *const transfer = uart->uart_write_transfer;

	return uart->write_pos >= transfer->data_len;
}

static void handle_unsol_rx_data(const intel_instance_t *inst)
{
	struct intel_uart_context *uart =
	    CONTAINER_OF(inst, struct intel_uart_context, instance);
	volatile intel_uart_reg_t *regs = uart->regs;
	uint32_t lsr = regs->lsr;
	const intel_uart_unsol_rx_t *const unsol_rx =
	    uart->unsol_read_ctxt.unsol_rx;
	int32_t write_idx = uart->unsol_read_ctxt.write_idx;
	int32_t read_idx = uart->unsol_read_ctxt.read_idx;

	while (lsr & INTEL_UART_LSR_DR) {
		write_idx++;
		if (write_idx == unsol_rx->size) {
			write_idx = 0;
		}
		unsol_rx->buffer[write_idx] = regs->rbr_thr_dll;
		lsr = regs->lsr;
	}
	uart->unsol_read_ctxt.write_idx = write_idx;
	if (read_idx < write_idx) {
		uart->unsol_read_ctxt.curr_len = write_idx - read_idx;
	} else {
		uart->unsol_read_ctxt.curr_len =
		    unsol_rx->size - read_idx + write_idx;
	}
	unsol_rx->unsol_rx_callback(unsol_rx->cb_data, INTEL_DRIVER_OK,
				    INTEL_UART_IDLE,
				    uart->unsol_read_ctxt.curr_len);
}

static void handle_unsol_rx_error(const intel_instance_t *inst,
				  uint32_t line_status)
{
	struct intel_uart_context *uart =
	    CONTAINER_OF(inst, struct intel_uart_context, instance);
	const intel_uart_unsol_rx_t *const unsol_rx =
	    uart->unsol_read_ctxt.unsol_rx;

	unsol_rx->unsol_rx_callback(unsol_rx->cb_data, INTEL_DRIVER_ERROR,
				    line_status,
				    uart->unsol_read_ctxt.curr_len);
}

static bool is_tx_disabled(const intel_instance_t *inst)
{
	struct intel_uart_context *uart =
	    CONTAINER_OF(inst, struct intel_uart_context, instance);

	return uart->uart_xfer_ctrl.tx_disable;
}

static bool is_rx_disabled(const intel_instance_t *inst)
{
	struct intel_uart_context *uart =
	    CONTAINER_OF(inst, struct intel_uart_context, instance);

	return uart->uart_xfer_ctrl.rx_disable;
}

void intel_uart_isr_handler(const intel_instance_t *inst)
{
	struct intel_uart_context *uart =
	    CONTAINER_OF(inst, struct intel_uart_context, instance);
	volatile intel_uart_reg_t *regs = uart->regs;
	uint8_t interrupt_id = regs->iir_fcr & INTEL_UART_IIR_IID_MASK;
	const intel_uart_transfer_t *const read_transfer =
	    uart->uart_read_transfer;
	const intel_uart_transfer_t *const write_transfer =
	    uart->uart_write_transfer;
	uint32_t line_status;

	/*
	 * Interrupt ID priority levels (from highest to lowest):
	 * 1: INTEL_UART_IIR_RECV_LINE_STATUS
	 * 2: INTEL_UART_IIR_RECV_DATA_AVAIL and INTEL_UART_IIR_CHAR_TIMEOUT
	 * 3: INTEL_UART_IIR_THR_EMPTY
	 */
	switch (interrupt_id) {
	/* Spurious interrupt */
	case INTEL_UART_IIR_NO_INTERRUPT_PENDING:
		break;

	case INTEL_UART_IIR_THR_EMPTY:
		if (write_transfer) {
			if (is_write_xfer_complete(inst)) {
				regs->ier_dlh &= ~INTEL_UART_IER_ETBEI;
				/*
				* At this point the FIFOs are empty, but the
				* shift
				* register still is transmitting the last 8
				* bits. So if
				 * we were to read LSR, it would say the device
				* is still
				* busy. Use the SCR Bit 0 to indicate an irq tx
				* is
				* complete.
				*/
				regs->scr |= INTEL_UART_SCR_STATUS_UPDATE;
				if (write_transfer->callback) {
					write_transfer->callback(
					    write_transfer->callback_data, 0,
					    INTEL_UART_IDLE, uart->write_pos);
				}

				if (uart->vec_write_ctxt.active == false) {
					uart->uart_write_transfer = NULL;
				}
				return;
			}
			/*
			 * If we are starting the transfer then the TX FIFO is
			 * empty.
			 * In that case we set 'count' variable to
			 * INTEL_UART_FIFO_DEPTH
			 * in order to take advantage of the whole FIFO
			 * capacity.
			 */
			int count = (uart->write_pos == 0)
					? INTEL_UART_FIFO_DEPTH
					: INTEL_UART_FIFO_HALF_DEPTH;
			while (count-- && !is_write_xfer_complete(inst)) {
				regs->rbr_thr_dll =
				    write_transfer->data[uart->write_pos++];
			}

			/*
			* Change the threshold level to trigger an interrupt
			* when the
			* TX buffer is empty.
			*/
			if (is_write_xfer_complete(inst)) {
				regs->iir_fcr =
				    INTEL_UART_FCR_TX_0_RX_1_2_THRESHOLD |
				    INTEL_UART_FCR_FIFOE;
			}
		}
		break;

	case INTEL_UART_IIR_CHAR_TIMEOUT:
	case INTEL_UART_IIR_RECV_DATA_AVAIL:
		if (read_transfer) {
			/*
			 * Copy data from RX FIFO to xfer buffer as long as the
			 * xfer
			 * has not completed and we have data in the RX FIFO.
			 */
			while (!is_read_xfer_complete(inst)) {
				uint32_t lsr = regs->lsr;
				/*
				 * A break condition may cause a line status
				 * interrupt to follow very closely after a
				 * char timeout interrupt, but reading the lsr
				 * effectively clears the pending interrupts so
				 * we issue here the callback
				 * instead, otherwise we would miss it.
				 * NOTE: Returned len is 0 for now, this might
				 * change in the future.
				 */
				if (lsr & uart->status_report_mask) {
					regs->ier_dlh &=
					    ~(INTEL_UART_IER_ERBFI |
					      INTEL_UART_IER_ELSI);
					if (read_transfer->callback) {
						read_transfer->callback(
						    read_transfer
							->callback_data,
						    INTEL_DRIVER_ERROR,
						    lsr &
							INTEL_UART_LSR_ERROR_BITS,
						    0);
					}
					uart->uart_read_transfer = NULL;
					return;
				}
				if (lsr & INTEL_UART_LSR_DR) {
					read_transfer->data[uart->read_pos++] =
					    regs->rbr_thr_dll;
				} else {
					/* No more data in the RX FIFO. */
					break;
				}
			}

			if (is_read_xfer_complete(inst)) {
				/*
				 * Disable both 'Receiver Data Available' and
				 * 'Receiver Line Status' interrupts.
				 */
				regs->ier_dlh &= ~(INTEL_UART_IER_ERBFI |
						   INTEL_UART_IER_ELSI);
				if (read_transfer->callback) {
					read_transfer->callback(
					    read_transfer->callback_data, 0,
					    INTEL_UART_IDLE, uart->read_pos);
				}

				if (uart->vec_read_ctxt.active == false) {
					uart->uart_read_transfer = NULL;
				}
			}
		} else {
			if (uart->unsol_read_ctxt.enable_unsol_rx) {
				handle_unsol_rx_data(inst);
			}
		}
		break;

	case INTEL_UART_IIR_RECV_LINE_STATUS:

		line_status = regs->lsr & (INTEL_UART_LSR_ADDR_RCVD |
					   INTEL_UART_LSR_ERROR_BITS);
		if (uart->status_report_mask & line_status) {
			if (read_transfer) {
				regs->ier_dlh &= ~(INTEL_UART_IER_ERBFI |
						   INTEL_UART_IER_ELSI);
				if (read_transfer->callback) {
					/*
					 * Return the number of bytes read
					 * a zero as a line status error
					 * was detected.
					 */
					read_transfer->callback(
					    read_transfer->callback_data,
					    INTEL_DRIVER_ERROR,
					    (uart->status_report_mask &
					     line_status),
					    0);
					uart->uart_read_transfer = NULL;
				}
			} else {
				if (uart->unsol_read_ctxt.enable_unsol_rx) {
					handle_unsol_rx_error(inst,
							      line_status);
				}
			}
		}
		if (line_status & INTEL_UART_LSR_ADDR_RCVD) {
			/* Remove the address from FIFO as address match is
			 * confirmed with hardware address match. */
			regs->rbr_thr_dll;
		}
		break;

	default:
		/* Unhandled interrupt occured,disable uart interrupts.
		 * and report error.
		 */
		if (read_transfer && read_transfer->callback) {
			regs->ier_dlh &=
			    ~(INTEL_UART_IER_ERBFI | INTEL_UART_IER_ELSI);
			read_transfer->callback(read_transfer->callback_data,
						INTEL_DRIVER_ERROR,
						INTEL_UART_UNHANDLED_INT, 0);
			uart->uart_read_transfer = NULL;
		}
		if (write_transfer && write_transfer->callback) {
			regs->ier_dlh &= ~INTEL_UART_IER_ETBEI;
			write_transfer->callback(write_transfer->callback_data,
						 INTEL_DRIVER_ERROR,
						 INTEL_UART_UNHANDLED_INT, 0);
			uart->uart_write_transfer = NULL;
		}
	}
}

int intel_uart_set_config(IN intel_instance_t *inst,
			  IN intel_uart_config_t *cfg)
{
	DBG_CHECK(cfg != NULL, INTEL_DRIVER_ERROR_PARAMETER);

	int32_t ret;
	struct intel_uart_context *uart =
	    CONTAINER_OF(inst, struct intel_uart_context, instance);

	intel_uart_reg_t *base = (intel_uart_reg_t *)inst->base_addr;
	intel_uart_reg_t *phy_addr = (intel_uart_reg_t *)inst->phy_addr;

	uart->regs = base;
	uart->phy_regs = phy_addr;
	volatile intel_uart_reg_t *regs = uart->regs;

	volatile uint32_t unused_lsr __attribute__((unused));
	ret = intel_uart_set_baud_rate(inst, cfg->baud_rate, cfg->clk_speed_hz);
	if (ret != INTEL_DRIVER_OK) {
		return INTEL_DRIVER_ERROR_PARAMETER;
	}

	/* Set line parameters. This also unsets the DLAB. */
	regs->lcr = cfg->line_control;
	/* Hardware automatic flow control. */
	regs->mcr = 0;
	if (true == cfg->hw_fc) {
		regs->mcr |= INTEL_UART_MCR_AFCE | INTEL_UART_MCR_RTS;
	}
	/* FIFO's enable and reset, set interrupt threshold. */
	regs->iir_fcr =
	    (INTEL_UART_FCR_FIFOE | INTEL_UART_FCR_RFIFOR |
	     INTEL_UART_FCR_XFIFOR | INTEL_UART_FCR_TX_0_RX_1_2_THRESHOLD);
	/* Clear interrupt settings set by bootloader uart init.*/
	regs->ier_dlh = 0;
	/* Enable the programmable fifo threshold interrupt.
	 * NOTE: This changes the interpretation of the THRE bit in LSR.
	 * It indicates FIFO Full status instead of THR Empty.*/
	regs->ier_dlh |= INTEL_UART_IER_PTIME;
	/* Clear LSR. */
	unused_lsr = regs->lsr;

	/* Enable both tx and rx in default configuration. */
	uart->uart_xfer_ctrl.tx_disable = false;
	uart->uart_xfer_ctrl.rx_disable = false;
	return INTEL_DRIVER_OK;
}

int intel_uart_get_status(IN intel_instance_t *inst, OUT uint32_t *const status)
{
	DBG_CHECK(status != NULL, INTEL_DRIVER_ERROR_PARAMETER);

	struct intel_uart_context *uart =
	    CONTAINER_OF(inst, struct intel_uart_context, instance);
	volatile intel_uart_reg_t *regs = uart->regs;
	uint32_t lsr = regs->lsr;

	*status = (lsr & (INTEL_UART_LSR_OE | INTEL_UART_LSR_PE |
			  INTEL_UART_LSR_FE | INTEL_UART_LSR_BI));

	/*
	 * Check as an IRQ TX completed, if so, the Shift register may still be
	 * busy.An IRQ TX might have completed after we read the lsr.
	 * This will be reflected in the scr.
	 */
	if (regs->scr & INTEL_UART_SCR_STATUS_UPDATE) {
		regs->scr &= ~INTEL_UART_SCR_STATUS_UPDATE;
	} else if (!(lsr & (INTEL_UART_LSR_TEMT))) {
		*status |= INTEL_UART_TX_BUSY;
	}

	if (lsr & INTEL_UART_LSR_DR) {
		*status |= INTEL_UART_RX_BUSY;
	}

	return INTEL_DRIVER_OK;
}

int intel_uart_write(IN intel_instance_t *inst, IN uint8_t data)
{

	struct intel_uart_context *uart =
	    CONTAINER_OF(inst, struct intel_uart_context, instance);
	if (is_tx_disabled(inst)) {
		return INTEL_DRIVER_ERROR_UNSUPPORTED;
	}

	volatile intel_uart_reg_t *regs = uart->regs;
	while (!(regs->lsr & INTEL_UART_LSR_TEMT)) {
	}

	regs->rbr_thr_dll = data;
	while (!(regs->lsr & INTEL_UART_LSR_TEMT)) {
	}

	return INTEL_DRIVER_OK;
}

int intel_uart_read(IN intel_instance_t *inst, OUT uint8_t *const data,
		    OUT uint32_t *status)
{
	DBG_CHECK(data != NULL, INTEL_DRIVER_ERROR_PARAMETER);
	DBG_CHECK(unsol_read_ctxt[inst].enable_unsol_rx == false,
		  INTEL_DRIVER_ERROR_UNSUPPORTED);

	if (is_rx_disabled(inst)) {
		return INTEL_DRIVER_ERROR_UNSUPPORTED;
	}
	struct intel_uart_context *uart =
	    CONTAINER_OF(inst, struct intel_uart_context, instance);
	volatile intel_uart_reg_t *regs = uart->regs;

	uint32_t lsr = regs->lsr;
	while (!(lsr & INTEL_UART_LSR_DR)) {
		lsr = regs->lsr;
	}
	/* Check if there are any errors on the line. */
	if (lsr & uart->status_report_mask) {
		if (status) {
			*status = (lsr & INTEL_UART_LSR_ERROR_BITS);
		}
		return INTEL_DRIVER_ERROR;
	}
	*data = regs->rbr_thr_dll;

	return INTEL_DRIVER_OK;
}

int intel_uart_write_non_block(IN intel_instance_t *inst, IN uint8_t data)
{
	DBG_CHECK(uart < INTEL_UART_NUM, INTEL_DRIVER_ERROR_PARAMETER);

	if (is_tx_disabled(inst)) {
		return INTEL_DRIVER_ERROR_UNSUPPORTED;
	}

	struct intel_uart_context *uart =
	    CONTAINER_OF(inst, struct intel_uart_context, instance);
	volatile intel_uart_reg_t *regs = uart->regs;

	regs->rbr_thr_dll = data;

	return INTEL_DRIVER_OK;
}

int intel_uart_read_non_block(IN intel_instance_t *inst,
			      OUT uint8_t *const data)
{
	DBG_CHECK(data != NULL, INTEL_DRIVER_ERROR_PARAMETER);

	if (is_rx_disabled(inst)) {
		return INTEL_DRIVER_ERROR_UNSUPPORTED;
	}

	struct intel_uart_context *uart =
	    CONTAINER_OF(inst, struct intel_uart_context, instance);
	volatile intel_uart_reg_t *regs = uart->regs;

	*data = regs->rbr_thr_dll;

	return INTEL_DRIVER_OK;
}

int intel_uart_write_buffer(IN intel_instance_t *inst, IN uint8_t *const data,
			    IN uint32_t len)
{
	DBG_CHECK(data != NULL, INTEL_DRIVER_ERROR_PARAMETER);

	if (is_tx_disabled(inst)) {
		return INTEL_DRIVER_ERROR_UNSUPPORTED;
	}

	uint32_t write_length = len;
	struct intel_uart_context *uart =
	    CONTAINER_OF(inst, struct intel_uart_context, instance);
	volatile intel_uart_reg_t *regs = uart->regs;

	uint8_t *d = (uint8_t *)data;

	while (write_length--) {
		/*
		 * Because FCR_FIFOE and IER_PTIME are enabled, LSR_THRE
		 * behaves as a TX FIFO full indicator.
		 */
		while ((regs->lsr & INTEL_UART_LSR_THRE)) {
		}
		regs->rbr_thr_dll = *d;
		d++;
	}
	/* Wait for transaction to complete. */
	while (!(regs->lsr & INTEL_UART_LSR_TEMT)) {
	}
	return INTEL_DRIVER_OK;
}

int intel_uart_read_buffer(IN intel_instance_t *inst, OUT uint8_t *const data,
			   IN uint32_t req_len, OUT uint32_t *comp_len,
			   OUT uint32_t *status)
{
	DBG_CHECK(data != NULL, INTEL_DRIVER_ERROR_PARAMETER);
	DBG_CHECK(status != NULL, INTEL_DRIVER_ERROR_PARAMETER);
	DBG_CHECK(comp_len != NULL, INTEL_DRIVER_ERROR_PARAMETER);
	DBG_CHECK(unsol_read_ctxt[uart].enable_unsol_rx == false,
		  INTEL_DRIVER_ERROR_UNSUPPORTED);

	if (is_rx_disabled(inst)) {
		return INTEL_DRIVER_ERROR_UNSUPPORTED;
	}

	struct intel_uart_context *uart =
	    CONTAINER_OF(inst, struct intel_uart_context, instance);
	volatile intel_uart_reg_t *regs = uart->regs;
	uint8_t *d = data;
	uint32_t read_len = req_len;
	uint32_t lsr = 0;
	*comp_len = 0;
	while (read_len--) {
		while (!(lsr & INTEL_UART_LSR_DR)) {
			lsr = regs->lsr;
		}

		*status = (lsr & uart->status_report_mask);

		if (*status) {
			return INTEL_DRIVER_ERROR;
		}
		*d = regs->rbr_thr_dll;
		(*comp_len)++;
		d++;
		lsr = 0;
	}
	return INTEL_DRIVER_OK;
}

int intel_uart_write_async(IN intel_instance_t *inst,
			   IN intel_uart_transfer_t *const xfer)
{
	DBG_CHECK(xfer != NULL, INTEL_DRIVER_ERROR_PARAMETER);
	DBG_CHECK(uart_write_transfer == 0, INTEL_DRIVER_ERROR_BUSY);

	if (is_tx_disabled(inst)) {
		return INTEL_DRIVER_ERROR_UNSUPPORTED;
	}

	struct intel_uart_context *uart =
	    CONTAINER_OF(inst, struct intel_uart_context, instance);
	volatile intel_uart_reg_t *regs = uart->regs;

	uart->write_pos = 0;
	uart->uart_write_transfer = xfer;

	/* Set threshold. */
	regs->iir_fcr =
	    (INTEL_UART_FCR_FIFOE | INTEL_UART_FCR_TX_0_RX_1_2_THRESHOLD);

	/* Enable TX holding reg empty interrupt. */
	regs->ier_dlh |= INTEL_UART_IER_ETBEI;
	return INTEL_DRIVER_OK;
}

int intel_uart_read_async(IN intel_instance_t *inst,
			  IN intel_uart_transfer_t *const xfer)
{
	DBG_CHECK(xfer != NULL, INTEL_DRIVER_ERROR_PARAMETER);
	DBG_CHECK(uart_read_transfer[uart] == 0, INTEL_DRIVER_ERROR_BUSY);

	DBG_CHECK(unsol_read_ctxt[uart].enable_unsol_rx == false,
		  INTEL_DRIVER_ERROR_UNSUPPORTED);

	if (is_rx_disabled(inst)) {
		return INTEL_DRIVER_ERROR_UNSUPPORTED;
	}

	struct intel_uart_context *uart =
	    CONTAINER_OF(inst, struct intel_uart_context, instance);
	volatile intel_uart_reg_t *regs = uart->regs;

	uart->read_pos = 0;
	uart->uart_read_transfer = xfer;

	/* Set threshold. */
	regs->iir_fcr =
	    (INTEL_UART_FCR_FIFOE | INTEL_UART_FCR_TX_0_RX_1_2_THRESHOLD);

	/*
	 * Enable both 'Receiver Data Available' and 'Receiver
	 * Line Status' interrupts.
	 */
	regs->ier_dlh |= INTEL_UART_IER_ERBFI | INTEL_UART_IER_ELSI;

	return INTEL_DRIVER_OK;
}

int intel_uart_async_write_terminate(IN intel_instance_t *inst)
{

	struct intel_uart_context *uart =
	    CONTAINER_OF(inst, struct intel_uart_context, instance);
	volatile intel_uart_reg_t *regs = uart->regs;
	const intel_uart_transfer_t *const transfer = uart->uart_write_transfer;

	/* No ongoing write transaction to be terminated. */
	if (transfer == NULL) {
		return INTEL_DRIVER_ERROR;
	}

	/* Disable TX holding reg empty interrupt. */
	regs->ier_dlh &= ~INTEL_UART_IER_ETBEI;
	if (transfer) {
		if (transfer->callback) {
			transfer->callback(transfer->callback_data,
					   INTEL_USART_ERROR_CANCELED,
					   INTEL_UART_IDLE, uart->write_pos);
		}
		uart->uart_write_transfer = NULL;
		uart->write_pos = 0;
	}

	return INTEL_DRIVER_OK;
}

int intel_uart_async_read_terminate(IN intel_instance_t *inst)
{

	struct intel_uart_context *uart =
	    CONTAINER_OF(inst, struct intel_uart_context, instance);
	volatile intel_uart_reg_t *regs = uart->regs;
	const intel_uart_transfer_t *const transfer = uart->uart_read_transfer;

	/* No ongoing read transaction to be terminated. */
	if (transfer == NULL) {
		return INTEL_DRIVER_ERROR;
	}

	/*
	 * Disable both 'Receiver Data Available' and 'Receiver Line Status'
	 * interrupts.
	 */
	regs->ier_dlh &= ~(INTEL_UART_IER_ERBFI | INTEL_UART_IER_ELSI);

	if (transfer) {
		if (transfer->callback) {
			transfer->callback(transfer->callback_data,
					   INTEL_USART_ERROR_CANCELED,
					   INTEL_UART_IDLE, uart->read_pos);
		}
		uart->uart_read_transfer = NULL;
		uart->read_pos = 0;
	}
	return INTEL_DRIVER_OK;
}

#if (HAS_UART_RS485_SUPPORT)

int intel_uart_rs485_set_config(IN intel_instance_t *inst,
				IN intel_uart_rs485_config_t *cfg)
{

	uint32_t de_assertion_cycles;
	uint32_t de_deassertion_cycles;
	uint32_t de_re_tat_cycles;
	uint32_t re_de_tat_cycles;

	struct intel_uart_context *uart =
	    CONTAINER_OF(inst, struct intel_uart_context, instance);
	volatile intel_uart_reg_t *regs = uart->regs;
	DBG_CHECK(cfg != NULL, INTEL_DRIVER_ERROR_PARAMETER);
	DBG_CHECK(cfg->de_assertion_time < INTEL_UART_DE_AT_DT_NS_MAX,
		  INTEL_DRIVER_ERROR_PARAMETER);
	DBG_CHECK(cfg->de_deassertion_time < INTEL_UART_DE_AT_DT_NS_MAX,
		  INTEL_DRIVER_ERROR_PARAMETER);
	DBG_CHECK(cfg->de_re_tat < INTEL_UART_TAT_NS_MAX,
		  INTEL_DRIVER_ERROR_PARAMETER);
	DBG_CHECK(cfg->re_de_tat < INTEL_UART_TAT_NS_MAX,
		  INTEL_DRIVER_ERROR_PARAMETER);

	/* Setting configuration for supporting RS-485 extension. */

	/* Setting the bit enable writes to RS485 registers. */
	regs->tcr |= INTEL_UART_TCR_RS485_EN;

	de_assertion_cycles =
	    (cfg->de_assertion_time / INTEL_UART_SERIAL_CLK_PERIOD_NS);
	de_deassertion_cycles =
	    (cfg->de_deassertion_time / INTEL_UART_SERIAL_CLK_PERIOD_NS);

	/* Set the values of assertion and de-assertion time. */
	regs->det = INTEL_UART_DET_AT_DT_PACK(de_assertion_cycles,
					      de_deassertion_cycles);

	/* Clearing previous values of transfer mode in TCR. */
	regs->tcr &= ~(INTEL_UART_TCR_TRANSFER_MODE_MASK);

	/* The TAT values are valid only in half duplex mode. */
	if (cfg->transfer_mode == INTEL_UART_RS485_XFER_MODE_HALF_DUPLEX) {
		/* Setting the transfer mode in TCR. */
		regs->tcr |=
		    (uint32_t)(INTEL_UART_XFER_MODE_HW_HALF_DUPLEX
			       << (INTEL_UART_TCR_TRANSFER_MODE_OFFSET));

		/* Set the values of de-re and re-de tat.*/
		de_re_tat_cycles =
		    cfg->de_re_tat / INTEL_UART_SERIAL_CLK_PERIOD_NS;
		re_de_tat_cycles =
		    cfg->re_de_tat / INTEL_UART_SERIAL_CLK_PERIOD_NS;
		regs->tat =
		    INTEL_UART_TAT_PACK(de_re_tat_cycles, re_de_tat_cycles);
	} else {
		regs->tcr |=
		    (uint32_t)(INTEL_UART_XFER_MODE_FULL_DUPLEX
			       << (INTEL_UART_TCR_TRANSFER_MODE_OFFSET));
	}

	/* Clearing previous values of DE & RE polarity in TCR. */
	regs->tcr &= ~(INTEL_UART_TCR_RE_POL | INTEL_UART_TCR_DE_POL);
	regs->tcr |= (((cfg->re_polarity) << INTEL_UART_TCR_RE_POL_OFFSET) |
		      ((cfg->de_polarity) << INTEL_UART_TCR_DE_POL_OFFSET));

	/* Enable or disable the driver based on config. */
	regs->de_en = cfg->de_en;

	/* Enable or disable the receiver based on config. */
	regs->re_en = cfg->re_en;

	regs->tcr &= ~(INTEL_UART_TCR_RS485_EN);
	return INTEL_DRIVER_OK;
}

int intel_uart_rs485_disable(IN intel_instance_t *inst)
{
	struct intel_uart_context *uart =
	    CONTAINER_OF(inst, struct intel_uart_context, instance);
	volatile intel_uart_reg_t *regs = uart->regs;
	regs->tcr &= ~(INTEL_UART_TCR_RS485_EN);
	return INTEL_DRIVER_OK;
}

int intel_uart_rs485_enable(IN intel_instance_t *inst)
{

	struct intel_uart_context *uart =
	    CONTAINER_OF(inst, struct intel_uart_context, instance);
	volatile intel_uart_reg_t *regs = uart->regs;
	regs->tcr |= (INTEL_UART_TCR_RS485_EN);

	/* Reset rx_fifo right after enable to clear any error conditions
	 * generated as rx line held low when rs485 is not enabled.
	 */
	regs->iir_fcr |= (INTEL_UART_FCR_RFIFOR);
	regs->lsr;
	return INTEL_DRIVER_OK;
}

int intel_uart_rs485_get_config(IN intel_instance_t *inst,
				intel_uart_rs485_config_t *cfg)
{

	DBG_CHECK(cfg != NULL, INTEL_DRIVER_ERROR_PARAMETER);
	struct intel_uart_context *uart =
	    CONTAINER_OF(inst, struct intel_uart_context, instance);
	volatile intel_uart_reg_t *regs = uart->regs;

	cfg->de_assertion_time = (uint32_t)(
	    ((regs->det & INTEL_UART_DET_AT_MASK) >> INTEL_UART_DET_AT_OFFSET) *
	    INTEL_UART_SERIAL_CLK_PERIOD_NS);

	cfg->de_deassertion_time = (uint32_t)(
	    ((regs->det & INTEL_UART_DET_DT_MASK) >> INTEL_UART_DET_DT_OFFSET) *
	    INTEL_UART_SERIAL_CLK_PERIOD_NS);
	cfg->de_re_tat = (uint32_t)(((regs->tat & INTEL_UART_TAT_DE_RE_MASK) >>
				     INTEL_UART_TAT_DE_RE_OFFSET) *
				    INTEL_UART_SERIAL_CLK_PERIOD_NS);

	cfg->re_de_tat = (uint32_t)(((regs->tat & INTEL_UART_TAT_RE_DE_MASK) >>
				     INTEL_UART_TAT_RE_DE_OFFSET) *
				    INTEL_UART_SERIAL_CLK_PERIOD_NS);

	cfg->transfer_mode = (regs->tcr & INTEL_UART_TCR_TRANSFER_MODE_MASK) >>
			     INTEL_UART_TCR_TRANSFER_MODE_OFFSET;

	if (regs->tcr & (INTEL_UART_TCR_DE_POL)) {
		cfg->de_polarity = INTEL_UART_RS485_POL_ACTIVE_HIGH;
	} else {
		cfg->de_polarity = INTEL_UART_RS485_POL_ACTIVE_LOW;
	}

	if (regs->tcr & (INTEL_UART_TCR_RE_POL)) {
		cfg->re_polarity = INTEL_UART_RS485_POL_ACTIVE_HIGH;
	} else {
		cfg->re_polarity = INTEL_UART_RS485_POL_ACTIVE_LOW;
	}

	cfg->de_en = regs->de_en;
	cfg->re_en = regs->re_en;
	return INTEL_DRIVER_OK;
}

/* Clear the RS485 configuration registers. */
int intel_uart_rs485_clear_config(IN intel_instance_t *inst)
{

	struct intel_uart_context *uart =
	    CONTAINER_OF(inst, struct intel_uart_context, instance);
	volatile intel_uart_reg_t *regs = uart->regs;
	regs->tcr |= INTEL_UART_TCR_RS485_EN;
	regs->det = 0;
	regs->tat = 0;
	regs->de_en = 0;
	regs->re_en = 0;
	regs->tcr = 0;
	return INTEL_DRIVER_OK;
}

#endif /* HAS_UART_RS485_SUPPORT */

#if (HAS_UART_9BIT_SUPPORT)
int intel_uart_9bit_set_config(IN intel_instance_t *inst,
			       IN intel_uart_9bit_config_t *cfg)
{

	DBG_CHECK(cfg != NULL, INTEL_DRIVER_ERROR_PARAMETER);
	struct intel_uart_context *uart =
	    CONTAINER_OF(inst, struct intel_uart_context, instance);
	volatile intel_uart_reg_t *regs = uart->regs;

	regs->lcr_ext = (INTEL_UART_LCR_EXT_ENABLE_9BIT_MODE);

	if (cfg->addr_ctrl == INTEL_UART_9BIT_HW_ADDR_CTRL) {
		regs->lcr_ext &= ~(INTEL_UART_LCR_EXT_SW_TRANSMIT_MODE);
		regs->rar = (cfg->receive_address &
			     (INTEL_UART_RAR_RECEIVE_ADDRESS_MASK));
		regs->lcr_ext |= (INTEL_UART_LCR_EXT_HW_RECV_ADDRESS_MATCH);
	} else {
		regs->lcr_ext |= (INTEL_UART_LCR_EXT_SW_TRANSMIT_MODE);
		regs->lcr_ext &= ~(INTEL_UART_LCR_EXT_HW_RECV_ADDRESS_MATCH);
	}

	regs->lcr_ext &= ~(INTEL_UART_LCR_EXT_ENABLE_9BIT_MODE);

	return INTEL_DRIVER_OK;
}

int intel_uart_9bit_disable(IN intel_instance_t *inst)
{

	struct intel_uart_context *uart =
	    CONTAINER_OF(inst, struct intel_uart_context, instance);
	volatile intel_uart_reg_t *regs = uart->regs;

	regs->lcr_ext &= ~(INTEL_UART_LCR_EXT_ENABLE_9BIT_MODE);
	return INTEL_DRIVER_OK;
}

int intel_uart_9bit_enable(IN intel_instance_t *inst)
{

	struct intel_uart_context *uart =
	    CONTAINER_OF(inst, struct intel_uart_context, instance);
	volatile intel_uart_reg_t *regs = uart->regs;

	regs->lcr_ext |= (INTEL_UART_LCR_EXT_ENABLE_9BIT_MODE);
	return INTEL_DRIVER_OK;
}

int intel_uart_9bit_send_address(IN intel_instance_t *inst, uint8_t address)
{

	struct intel_uart_context *uart =
	    CONTAINER_OF(inst, struct intel_uart_context, instance);
	volatile intel_uart_reg_t *regs = uart->regs;

	if (is_tx_disabled(inst)) {
		return INTEL_DRIVER_ERROR_UNSUPPORTED;
	}

	if (regs->lcr_ext & INTEL_UART_LCR_EXT_ENABLE_9BIT_MODE) {

		if ((regs->lcr_ext & INTEL_UART_LCR_EXT_SW_TRANSMIT_MODE)) {
			regs->rbr_thr_dll = (BIT(8) | (uint32_t)address);

			/* Wait for address to be sent. */
			while (!(regs->lsr & INTEL_UART_LSR_TEMT)) {
			}
		} else {
			regs->tar = address;
			/* Sending the address. */
			regs->lcr_ext |= INTEL_UART_LCR_EXT_SEND_ADDRESS;
			/* Wait for address to be sent. */
			while (regs->lcr_ext & INTEL_UART_LCR_EXT_SEND_ADDRESS)
				;
		}
	} else {
		/* UART not configured for 9 bit operation. */
		return INTEL_DRIVER_ERROR;
	}

	return INTEL_DRIVER_OK;
}

int intel_uart_9bit_get_config(IN intel_instance_t *inst,
			       intel_uart_9bit_config_t *cfg)
{
	DBG_CHECK(cfg != NULL, INTEL_DRIVER_ERROR_PARAMETER);

	struct intel_uart_context *uart =
	    CONTAINER_OF(inst, struct intel_uart_context, instance);
	volatile intel_uart_reg_t *regs = uart->regs;

	/* Address of this  node when hardware address match enabled. */
	cfg->receive_address = regs->rar & INTEL_UART_RAR_RECEIVE_ADDRESS_MASK;

	/* Transmit Addr Ctrl s/w or h/w  enabled address transmit. */
	if (regs->lcr_ext & INTEL_UART_LCR_EXT_SW_TRANSMIT_MODE) {
		cfg->addr_ctrl = INTEL_UART_9BIT_SW_ADDR_CTRL;
	} else {
		cfg->addr_ctrl = INTEL_UART_9BIT_HW_ADDR_CTRL;
	}

	return INTEL_DRIVER_OK;
}

int intel_uart_read_rx_fifo(IN intel_instance_t *inst, uint16_t *rx_buff,
			    uint16_t *length_read)
{
	DBG_CHECK(rx_buff != NULL, INTEL_DRIVER_ERROR_PARAMETER);
	DBG_CHECK(length_read != NULL, INTEL_DRIVER_ERROR_PARAMETER);

	if (is_rx_disabled(inst)) {
		return INTEL_DRIVER_ERROR_UNSUPPORTED;
	}

	uint16_t data;
	uint16_t i = 0;
	struct intel_uart_context *uart =
	    CONTAINER_OF(inst, struct intel_uart_context, instance);
	volatile intel_uart_reg_t *regs = uart->regs;
	uint32_t lsr = regs->lsr;
	*length_read = 0;

	if (!(lsr & INTEL_UART_LSR_DR)) {
		return INTEL_DRIVER_ERROR;
	} else {

		while ((lsr & INTEL_UART_LSR_DR)) {
			if (lsr & uart->status_report_mask) {
				return INTEL_DRIVER_ERROR;
			}
			data = regs->rbr_thr_dll;
			rx_buff[i] = data;
			lsr = regs->lsr;
			++i;
		}
	}
	*length_read = i;
	return INTEL_DRIVER_OK;
}

int intel_uart_9bit_clear_config(IN intel_instance_t *inst)
{

	struct intel_uart_context *uart =
	    CONTAINER_OF(inst, struct intel_uart_context, instance);
	volatile intel_uart_reg_t *regs = uart->regs;
	regs->rar = 0;
	regs->tar = 0;
	regs->lcr_ext = 0;
	return INTEL_DRIVER_OK;
}
#endif /* HAS_UART_9BIT_SUPPORT */
static bool is_tx_fifo_full(const intel_instance_t *inst)
{
	/* As fifos are enabled and ptime is enabled the thre bit
	 * acts as a fifo full inidicator.
	 */
	struct intel_uart_context *uart =
	    CONTAINER_OF(inst, struct intel_uart_context, instance);
	volatile intel_uart_reg_t *regs = uart->regs;

	return !!(regs->lsr & INTEL_UART_LSR_THRE);
}

bool intel_uart_irq_tx_ready(IN intel_instance_t *inst)
{
	uint32_t id;

	struct intel_uart_context *uart =
	    CONTAINER_OF(inst, struct intel_uart_context, instance);
	id = uart->iir_cache & INTEL_UART_IIR_IID_MASK;
	return id == INTEL_UART_IIR_THR_EMPTY;
}

static bool intel_is_rx_data_available(const intel_instance_t *inst)
{
	struct intel_uart_context *uart =
	    CONTAINER_OF(inst, struct intel_uart_context, instance);
	volatile intel_uart_reg_t *regs = uart->regs;

	return regs->lsr & INTEL_UART_LSR_DR;
}

bool intel_uart_is_irq_rx_ready(IN intel_instance_t *inst)
{
	struct intel_uart_context *uart =
	    CONTAINER_OF(inst, struct intel_uart_context, instance);
	uint32_t id = (uart->iir_cache & INTEL_UART_IIR_IID_MASK);

	return (id == INTEL_UART_IIR_RECV_DATA_AVAIL) ||
	       (id == INTEL_UART_IIR_CHAR_TIMEOUT);
}

bool intel_uart_is_irq_pending(IN intel_instance_t *inst)
{
	struct intel_uart_context *uart =
	    CONTAINER_OF(inst, struct intel_uart_context, instance);

	return !(uart->iir_cache & INTEL_UART_IIR_NO_INTERRUPT_PENDING);
}

int intel_uart_fifo_fill(IN intel_instance_t *inst, IN uint8_t *data,
			 IN uint32_t size)
{
	uint32_t i;
	struct intel_uart_context *uart =
	    CONTAINER_OF(inst, struct intel_uart_context, instance);
	volatile intel_uart_reg_t *regs = uart->regs;

	if (is_tx_disabled(inst))
		return 0;

	for (i = 0; ((i < size) && (!is_tx_fifo_full(inst))); i++) {
		regs->rbr_thr_dll = data[i];
	}
	return i;
}

int intel_uart_fifo_read(IN intel_instance_t *inst, OUT uint8_t *data,
			 IN uint32_t size)
{
	int i;
	struct intel_uart_context *uart =
	    CONTAINER_OF(inst, struct intel_uart_context, instance);
	volatile intel_uart_reg_t *regs = uart->regs;

	if (is_rx_disabled(inst))
		return 0;

	for (i = 0; i < size && intel_is_rx_data_available(inst); i++) {
		data[i] = regs->rbr_thr_dll;
	}
	return i;
}

int intel_uart_irq_tx_enable(IN intel_instance_t *inst)
{

	struct intel_uart_context *uart =
	    CONTAINER_OF(inst, struct intel_uart_context, instance);
	volatile intel_uart_reg_t *regs = uart->regs;

	regs->ier_dlh |= INTEL_UART_IER_ETBEI;
	return INTEL_DRIVER_OK;
}

int intel_uart_irq_tx_disable(IN intel_instance_t *inst)
{
	struct intel_uart_context *uart =
	    CONTAINER_OF(inst, struct intel_uart_context, instance);
	volatile intel_uart_reg_t *regs = uart->regs;

	regs->ier_dlh &= ~INTEL_UART_IER_ETBEI;
	return INTEL_DRIVER_OK;
}

bool intel_uart_is_tx_complete(IN intel_instance_t *inst)
{
	struct intel_uart_context *uart =
	    CONTAINER_OF(inst, struct intel_uart_context, instance);
	volatile intel_uart_reg_t *regs = uart->regs;

	return !!(regs->lsr & INTEL_UART_LSR_TEMT);
}

int intel_uart_irq_rx_enable(IN intel_instance_t *inst)
{

	struct intel_uart_context *uart =
	    CONTAINER_OF(inst, struct intel_uart_context, instance);
	volatile intel_uart_reg_t *regs = uart->regs;

	regs->ier_dlh |= INTEL_UART_IER_ERBFI;
	return INTEL_DRIVER_OK;
}

int intel_uart_irq_rx_disable(IN intel_instance_t *inst)
{

	struct intel_uart_context *uart =
	    CONTAINER_OF(inst, struct intel_uart_context, instance);
	volatile intel_uart_reg_t *regs = uart->regs;

	regs->ier_dlh &= ~INTEL_UART_IER_ERBFI;
	return INTEL_DRIVER_OK;
}

int intel_uart_update_irq_cache(IN intel_instance_t *inst)
{

	struct intel_uart_context *uart =
	    CONTAINER_OF(inst, struct intel_uart_context, instance);
	volatile intel_uart_reg_t *regs = uart->regs;

	uart->iir_cache = regs->iir_fcr;
	return INTEL_DRIVER_OK;
}

int intel_uart_irq_err_enable(IN intel_instance_t *inst)
{

	struct intel_uart_context *uart =
	    CONTAINER_OF(inst, struct intel_uart_context, instance);
	volatile intel_uart_reg_t *regs = uart->regs;

	regs->ier_dlh |= INTEL_UART_IER_ELSI;
	return INTEL_DRIVER_OK;
}

int intel_uart_irq_err_disable(IN intel_instance_t *inst)
{

	struct intel_uart_context *uart =
	    CONTAINER_OF(inst, struct intel_uart_context, instance);
	volatile intel_uart_reg_t *regs = uart->regs;

	regs->ier_dlh &= ~INTEL_UART_IER_ELSI;
	return INTEL_DRIVER_OK;
}

int intel_uart_set_baud_rate(IN intel_instance_t *inst, IN uint32_t baud_rate,
			     IN uint32_t clk_speed_hz)
{

	struct intel_uart_context *uart =
	    CONTAINER_OF(inst, struct intel_uart_context, instance);
	/* Divisor = clock_speed_hz /(16* baudrate) */
	uint32_t divisor = clk_speed_hz / (baud_rate << 4);
	DBG_CHECK(divisor <= (INTEL_UART_MAX_BAUD_DIVISOR),
		  INTEL_DRIVER_ERROR_PARAMETER);

	uint32_t dlf = (clk_speed_hz % (baud_rate << 4)) / baud_rate;
	uint32_t scaled_dlf =
	    ((clk_speed_hz % (baud_rate << 4)) * INTEL_UART_DLF_SCALAR) /
	    baud_rate;

	dlf = dlf + ((scaled_dlf % INTEL_UART_DLF_SCALAR) >=
		     (INTEL_UART_DLF_SCALAR / 2));
	volatile intel_uart_reg_t *regs = uart->regs;
	/* Store LCR before making changes. */
	uint32_t lcr_saved = regs->lcr;
	/* Set divisor latch registers (integer + fractional part). */
	regs->lcr = INTEL_UART_LCR_DLAB;
	regs->ier_dlh = INTEL_UART_GET_DLH(divisor);
	regs->rbr_thr_dll = INTEL_UART_GET_DLL(divisor);
	regs->dlf = dlf;
	/* Restore the lcr to its previous value. */
	regs->lcr = lcr_saved;
	uart->baud_rate_cache = baud_rate;
	uart->clk_speed_cache = clk_speed_hz;

	return INTEL_DRIVER_OK;
}

int intel_uart_get_config(IN intel_instance_t *inst,
			  OUT intel_uart_config_t *cfg)
{

	DBG_CHECK(cfg != NULL, INTEL_DRIVER_ERROR_PARAMETER);
	struct intel_uart_context *uart =
	    CONTAINER_OF(inst, struct intel_uart_context, instance);
	volatile intel_uart_reg_t *regs = uart->regs;
	uint32_t fc_setting;
	cfg->line_control = regs->lcr;
	fc_setting = (regs->mcr & (INTEL_UART_MCR_AFCE | INTEL_UART_MCR_RTS));
	cfg->hw_fc = (fc_setting == (INTEL_UART_MCR_AFCE | INTEL_UART_MCR_RTS));
	cfg->baud_rate = uart->baud_rate_cache;
	cfg->clk_speed_hz = uart->clk_speed_cache;
	return INTEL_DRIVER_OK;
}

/* Set the given UART port to loopback mode. */
int intel_uart_set_loopback_mode(IN intel_instance_t *inst)
{
	struct intel_uart_context *uart =
	    CONTAINER_OF(inst, struct intel_uart_context, instance);
	volatile intel_uart_reg_t *regs = uart->regs;

	/* Setting to loopback. */
	regs->mcr |= INTEL_UART_MCR_LOOPBACK;
	return INTEL_DRIVER_OK;
}

/* Clear loopback mode */
int intel_uart_clr_loopback_mode(IN intel_instance_t *inst)
{
	struct intel_uart_context *uart =
	    CONTAINER_OF(inst, struct intel_uart_context, instance);
	volatile intel_uart_reg_t *regs = uart->regs;

	/* Clearing loopback. */
	regs->mcr &= ~(INTEL_UART_MCR_LOOPBACK);
	return INTEL_DRIVER_OK;
}

int intel_uart_get_loopback_mode(IN intel_instance_t *inst, uint32_t *p_mode)
{
	struct intel_uart_context *uart =
	    CONTAINER_OF(inst, struct intel_uart_context, instance);
	volatile intel_uart_reg_t *regs = uart->regs;
	DBG_CHECK(p_mode != NULL, INTEL_DRIVER_ERROR_PARAMETER);

	*p_mode = !!(regs->mcr & (INTEL_UART_MCR_LOOPBACK));
	return INTEL_DRIVER_OK;
}

/* Generate Break condition */
int intel_uart_set_break_con(IN intel_instance_t *inst)
{
	struct intel_uart_context *uart =
	    CONTAINER_OF(inst, struct intel_uart_context, instance);
	volatile intel_uart_reg_t *regs = uart->regs;
	regs->lcr |= INTEL_UART_LCR_BREAK;
	return INTEL_DRIVER_OK;
}

/* Clear Break condition */
int intel_uart_clr_break_con(IN intel_instance_t *inst)
{
	struct intel_uart_context *uart =
	    CONTAINER_OF(inst, struct intel_uart_context, instance);
	volatile intel_uart_reg_t *regs = uart->regs;
	regs->lcr &= ~INTEL_UART_LCR_BREAK;
	return INTEL_DRIVER_OK;
}

/* Enable auto flow control */
int intel_uart_auto_fc_enable(IN intel_instance_t *inst)
{
	struct intel_uart_context *uart =
	    CONTAINER_OF(inst, struct intel_uart_context, instance);
	volatile intel_uart_reg_t *regs = uart->regs;
	regs->mcr |= (INTEL_UART_MCR_AFCE | INTEL_UART_MCR_RTS);
	return INTEL_DRIVER_OK;
}

/* Disable auto flow control */
int intel_uart_auto_fc_disable(IN intel_instance_t *inst)
{
	struct intel_uart_context *uart =
	    CONTAINER_OF(inst, struct intel_uart_context, instance);
	volatile intel_uart_reg_t *regs = uart->regs;

	regs->mcr &= ~(INTEL_UART_MCR_AFCE | INTEL_UART_MCR_RTS);
	return INTEL_DRIVER_OK;
}

int intel_set_ln_status_report_mask(IN intel_instance_t *inst, IN uint32_t mask)
{
	struct intel_uart_context *uart =
	    CONTAINER_OF(inst, struct intel_uart_context, instance);

	uart->status_report_mask = mask;
	return INTEL_DRIVER_OK;
}

int intel_uart_enable_unsol_rx(IN intel_instance_t *inst,
			       IN intel_uart_unsol_rx_t *const unsol_rx)
{

	DBG_CHECK(unsol_rx != NULL, INTEL_DRIVER_ERROR_PARAMETER);
	DBG_CHECK(unsol_rx->buffer != NULL, INTEL_DRIVER_ERROR_PARAMETER);
	DBG_CHECK(unsol_rx->size != 0, INTEL_DRIVER_ERROR_PARAMETER);
	DBG_CHECK(unsol_rx->unsol_rx_callback != NULL,
		  INTEL_DRIVER_ERROR_PARAMETER);

	struct intel_uart_context *uart =
	    CONTAINER_OF(inst, struct intel_uart_context, instance);
	volatile intel_uart_reg_t *regs = uart->regs;

	if (is_rx_disabled(inst)) {
		return INTEL_DRIVER_ERROR_UNSUPPORTED;
	}

	/* Report error if there is an ongoing async read. */
	if (uart->uart_read_transfer) {
		return INTEL_DRIVER_ERROR_UNSUPPORTED;
	}

	/* Setting initial conditions for read and write index */
	uart->unsol_read_ctxt.read_idx = -1;
	uart->unsol_read_ctxt.write_idx = -1;
	uart->unsol_read_ctxt.curr_len = 0;
	uart->unsol_read_ctxt.unsol_rx = unsol_rx;

	uart->unsol_read_ctxt.enable_unsol_rx = true;

	/* Enable receive data available interrupt */
	regs->ier_dlh |= (INTEL_UART_IER_ERBFI | INTEL_UART_IER_ELSI);
	return INTEL_DRIVER_OK;
}

int intel_uart_disable_unsol_rx(IN intel_instance_t *inst)
{

	struct intel_uart_context *uart =
	    CONTAINER_OF(inst, struct intel_uart_context, instance);
	volatile intel_uart_reg_t *regs = uart->regs;

	if (uart->uart_read_transfer) {
		return INTEL_DRIVER_ERROR_UNSUPPORTED;
	}

	if (!uart->unsol_read_ctxt.enable_unsol_rx) {
		return INTEL_DRIVER_ERROR_UNSUPPORTED;
	}

	regs->ier_dlh &= ~(INTEL_UART_IER_ERBFI | INTEL_UART_IER_ELSI);

	uart->unsol_read_ctxt.enable_unsol_rx = false;
	uart->unsol_read_ctxt.unsol_rx = NULL;
	return INTEL_DRIVER_OK;
}

int intel_uart_get_unsol_data(IN intel_instance_t *inst, uint8_t *buffer,
			      int len)
{

	DBG_CHECK(buffer != NULL, INTEL_DRIVER_ERROR_PARAMETER);
	int i, count = 0, start_idx, end_idx;
	int read_comp = 0;
	struct intel_uart_context *uart =
	    CONTAINER_OF(inst, struct intel_uart_context, instance);
	volatile intel_uart_reg_t *regs = uart->regs;

	if (!uart->unsol_read_ctxt.enable_unsol_rx) {
		return INTEL_DRIVER_ERROR_UNSUPPORTED;
	}
	const intel_uart_unsol_rx_t *const unsol_rx =
	    uart->unsol_read_ctxt.unsol_rx;

	if ((len == 0) || (len > uart->unsol_read_ctxt.curr_len + 1)) {
		return INTEL_DRIVER_ERROR_PARAMETER;
	}

	regs->ier_dlh &= ~(INTEL_UART_IER_ERBFI | INTEL_UART_IER_ELSI);

	/* read_idx is the last read location so adding 1 for next valid
	 * location, similarly write_idx is the last written location thus
	 * adding 1 for getting the end condition.
	 */
	start_idx = uart->unsol_read_ctxt.read_idx + 1;
	end_idx = uart->unsol_read_ctxt.write_idx + 1;
	if (start_idx < end_idx) {
		for (i = start_idx; i < end_idx; i++) {
			buffer[count++] = unsol_rx->buffer[i];
			if (count == len) {
				break;
			}
		}

	} else {
		for (i = start_idx; i < unsol_rx->size; i++) {
			buffer[count++] = unsol_rx->buffer[i];
			if (count == len) {
				read_comp = true;
				break;
			}
		}
		if (!read_comp) {
			for (i = 0; i < end_idx; i++) {
				buffer[count++] = unsol_rx->buffer[i];
				if (count == len) {
					break;
				}
			}
		}
	}

	/* Update the read idx to last read location. */
	uart->unsol_read_ctxt.read_idx =
	    (uart->unsol_read_ctxt.read_idx + len) % unsol_rx->size;
	uart->unsol_read_ctxt.curr_len = (uart->unsol_read_ctxt.curr_len - len);

	/* Enable receive data available interrupt */
	regs->ier_dlh |= (INTEL_UART_IER_ERBFI | INTEL_UART_IER_ELSI);

	return INTEL_DRIVER_OK;
}

int intel_uart_get_unsol_data_len(IN intel_instance_t *inst, int *p_len)
{
	DBG_CHECK(p_len != NULL, INTEL_DRIVER_ERROR_PARAMETER);

	struct intel_uart_context *uart =
	    CONTAINER_OF(inst, struct intel_uart_context, instance);

	if (!uart->unsol_read_ctxt.enable_unsol_rx) {
		return INTEL_DRIVER_ERROR_UNSUPPORTED;
	}

	*p_len = uart->unsol_read_ctxt.curr_len;
	return INTEL_DRIVER_OK;
}

int intel_get_ln_status_report_mask(IN intel_instance_t *inst,
				    OUT uint32_t *p_mask)
{
	DBG_CHECK(p_mask != NULL, INTEL_DRIVER_ERROR_PARAMETER);
	struct intel_uart_context *uart =
	    CONTAINER_OF(inst, struct intel_uart_context, instance);

	*p_mask = uart->status_report_mask;
	return INTEL_DRIVER_OK;
}

int intel_uart_assert_rts(IN intel_instance_t *inst)
{

	struct intel_uart_context *uart =
	    CONTAINER_OF(inst, struct intel_uart_context, instance);
	volatile intel_uart_reg_t *regs = uart->regs;
	if (regs->mcr & INTEL_UART_MCR_AFCE) {
		return INTEL_DRIVER_ERROR;
	}

	regs->mcr |= INTEL_UART_MCR_RTS;
	return INTEL_DRIVER_OK;
}
int intel_uart_de_assert_rts(IN intel_instance_t *inst)
{
	struct intel_uart_context *uart =
	    CONTAINER_OF(inst, struct intel_uart_context, instance);
	volatile intel_uart_reg_t *regs = uart->regs;
	if (regs->mcr & INTEL_UART_MCR_AFCE) {
		return INTEL_DRIVER_ERROR;
	}

	regs->mcr &= ~(INTEL_UART_MCR_RTS);
	return INTEL_DRIVER_OK;
}

int intel_uart_read_rts(IN intel_instance_t *inst, uint32_t *p_rts)
{
	struct intel_uart_context *uart =
	    CONTAINER_OF(inst, struct intel_uart_context, instance);
	volatile intel_uart_reg_t *regs = uart->regs;

	*p_rts = !!(regs->mcr & INTEL_UART_MCR_RTS);
	return INTEL_DRIVER_OK;
}

int intel_uart_read_cts(IN intel_instance_t *inst, OUT uint32_t *p_cts)
{
	struct intel_uart_context *uart =
	    CONTAINER_OF(inst, struct intel_uart_context, instance);
	volatile intel_uart_reg_t *regs = uart->regs;
	*p_cts = !!(regs->msr & INTEL_UART_MSR_CTS);
	return INTEL_DRIVER_OK;
}

int intel_uart_write_vec_async(IN intel_instance_t *inst,
			       IN intel_uart_io_vec_xfer_t *const vec_xfer)
{

	DBG_CHECK(vec_xfer != NULL, INTEL_DRIVER_ERROR_PARAMETER);
	DBG_CHECK(vec_xfer->count != 0, INTEL_DRIVER_ERROR_PARAMETER);

	if (is_tx_disabled(inst)) {
		return INTEL_DRIVER_ERROR_UNSUPPORTED;
	}

	struct intel_uart_context *uart =
	    CONTAINER_OF(inst, struct intel_uart_context, instance);
	volatile intel_uart_reg_t *regs = uart->regs;

	uart->vec_write_ctxt.vec = vec_xfer;
	uart->write_pos = 0;
	uart->vec_write_ctxt.curr_count = 0;
	uart->vec_write_ctxt.active = true;

	/* Initiate transfer with the first member of the vector. */
	uart->vec_write_ctxt.xfer.data = vec_xfer->vec[0].base;
	uart->vec_write_ctxt.xfer.data_len = vec_xfer->vec[0].len;
	uart->vec_write_ctxt.xfer.callback_data = (void *)inst;
	uart->vec_write_ctxt.xfer.callback = io_vec_write_callback;

	uart->uart_write_transfer = &uart->vec_write_ctxt.xfer;

	/* Set threshold. */
	regs->iir_fcr =
	    (INTEL_UART_FCR_FIFOE | INTEL_UART_FCR_TX_0_RX_1_2_THRESHOLD);

	/* Enable TX holding reg empty interrupt. */
	regs->ier_dlh |= INTEL_UART_IER_ETBEI;
	return INTEL_DRIVER_OK;
}

int intel_uart_read_vec_async(IN intel_instance_t *inst,
			      IN intel_uart_io_vec_xfer_t *const vec_xfer)
{

	DBG_CHECK(vec_xfer != NULL, INTEL_DRIVER_ERROR_PARAMETER);
	DBG_CHECK(vec_xfer->count != 0, INTEL_DRIVER_ERROR_PARAMETER);

	if (is_rx_disabled(inst)) {
		return INTEL_DRIVER_ERROR_UNSUPPORTED;
	}

	struct intel_uart_context *uart =
	    CONTAINER_OF(inst, struct intel_uart_context, instance);
	volatile intel_uart_reg_t *regs = uart->regs;

	uart->vec_read_ctxt.vec = vec_xfer;
	uart->read_pos = 0;
	uart->vec_read_ctxt.curr_count = 0;
	uart->vec_read_ctxt.active = true;

	/* Initiate transfer with the first member of the vector. */
	uart->vec_read_ctxt.xfer.data = vec_xfer->vec[0].base;
	uart->vec_read_ctxt.xfer.data_len = vec_xfer->vec[0].len;
	uart->vec_read_ctxt.xfer.callback_data = (void *)uart;
	uart->vec_read_ctxt.xfer.callback = io_vec_read_callback;
	uart->uart_read_transfer = &uart->vec_read_ctxt.xfer;

	/* Set threshold. */
	regs->iir_fcr =
	    (INTEL_UART_FCR_FIFOE | INTEL_UART_FCR_TX_0_RX_1_2_THRESHOLD);

	/*
	 * Enable both 'Receiver Data Available' and 'Receiver
	 * Line Status' interrupts.
	 */
	regs->ier_dlh |= INTEL_UART_IER_ERBFI | INTEL_UART_IER_ELSI;

	return INTEL_DRIVER_OK;
}

/* UART DMA functions. */
static int intel_uart_dma_config(IN intel_instance_t *dma, int32_t channel,
				 IN intel_dma_event_cb_t cb, void *param,
				 dma_operation_type_t op)
{
	int32_t ret = 0;
	ret = intel_dma_init(dma, channel, cb, param);
	DBG_CHECK(INTEL_DRIVER_OK == ret, INTEL_DRIVER_ERROR);

	DBG_CHECK(INTEL_DRIVER_OK == ret, INTEL_DRIVER_ERROR);

	if (op == READ) {
		ret = intel_dma_control(dma, channel,
					INTEL_CONFIG_DMA_BURST_LENGTH,
					DMA_BURST_TRANS_LENGTH_1);
	} else if (op == WRITE) {
		ret = intel_dma_control(dma, channel,
					INTEL_CONFIG_DMA_BURST_LENGTH,
					DMA_BURST_TRANS_LENGTH_32);
	} else {
		return INTEL_DRIVER_ERROR_PARAMETER;
	}

	DBG_CHECK(INTEL_DRIVER_OK == ret, INTEL_DRIVER_ERROR);

	ret = intel_dma_control(dma, channel, INTEL_CONFIG_DMA_SR_TRANS_WIDTH,
				DMA_TRANS_WIDTH_8);
	DBG_CHECK(INTEL_DRIVER_OK == ret, INTEL_DRIVER_ERROR);

	ret = intel_dma_control(dma, channel, INTEL_CONFIG_DMA_DT_TRANS_WIDTH,
				DMA_TRANS_WIDTH_8);
	DBG_CHECK(INTEL_DRIVER_OK == ret, INTEL_DRIVER_ERROR);

	ret = intel_dma_control(dma, channel, INTEL_CONFIG_DMA_HS_POLARITY,
				DMA_HS_POLARITY_HIGH);

	DBG_CHECK(0 == ret, INTEL_DRIVER_ERROR);

	return INTEL_DRIVER_OK;
}

static int intel_uart_dma_io_async(IN intel_instance_t *inst,
				   IN intel_uart_dma_xfer_t *const xfer,
				   dma_operation_type_t op)
{
	int32_t ret;
	struct intel_uart_context *uart =
	    CONTAINER_OF(inst, struct intel_uart_context, instance);
	volatile intel_uart_reg_t *regs = uart->regs;
	volatile intel_uart_reg_t *regs_phy = uart->phy_regs;
	dma_channel_direction_t dma_dir;
	dma_hs_per_rtx_t dma_hs_per;
	uint32_t src, dst;

	if (op == WRITE) {
		uart->dma_ctxt.dma_xfer = xfer;
		uart->dma_ctxt.operation = WRITE;
		ret = intel_uart_dma_config(
		    (const intel_instance_t *)xfer->dma_dev, xfer->channel,
		    intel_dma_event_cb, (void *)(inst), WRITE);
		if (ret != INTEL_DRIVER_OK) {
			return ret;
		}
		dma_dir = DMA_MEMORY_TO_PERIPHERAL;
		dma_hs_per = DMA_HS_PER_TX;
		src = (uintptr_t)xfer->data;
		dst = (uintptr_t)(&regs_phy->rbr_thr_dll);
	} else if (op == READ) {
		uart->dma_ctxt.dma_xfer = xfer;
		uart->dma_ctxt.operation = READ;
		ret = intel_uart_dma_config(
		    (const intel_instance_t *)xfer->dma_dev, xfer->channel,
		    intel_dma_event_cb, (void *)(inst), READ);
		if (ret != INTEL_DRIVER_OK) {
			return ret;
		}
		dma_dir = DMA_PERIPHERAL_TO_MEMORY;
		dma_hs_per = DMA_HS_PER_RX;
		dst = (uintptr_t)xfer->data;
		src = (uintptr_t)(&regs_phy->rbr_thr_dll);
		regs->dmasa |= INTEL_UART_DMASA;
	} else {
		return INTEL_DRIVER_ERROR;
	}

	ret = intel_dma_control((intel_instance_t *)xfer->dma_dev,
				xfer->channel, INTEL_CONFIG_DMA_HS_DEVICE_ID,
				uart->uart_dma_hs_id);
	DBG_CHECK(0 == ret, INTEL_DRIVER_ERROR);

	ret = intel_dma_control(
	    (intel_instance_t *)xfer->dma_dev, xfer->channel,
	    INTEL_CONFIG_DMA_HS_DEVICE_ID_PER_DIR, dma_hs_per);
	DBG_CHECK(0 == ret, INTEL_DRIVER_ERROR);

	ret =
	    intel_dma_control((intel_instance_t *)xfer->dma_dev, xfer->channel,
			      INTEL_CONFIG_DMA_DIRECTION, dma_dir);
	if (ret != INTEL_DRIVER_OK) {
		return ret;
	}

	regs->iir_fcr =
	    (INTEL_UART_FCR_FIFOE | INTEL_UART_FCR_DEFAULT_TX_RX_THRESHOLD);
	ret = intel_dma_start_transfer((intel_instance_t *)xfer->dma_dev,
				       xfer->channel, src, dst, xfer->len);
	return ret;
}

static int intel_uart_dma_io_polled(IN intel_instance_t *inst,
				    IN intel_instance_t *dma_dev,
				    uint32_t channel, const uint8_t *buff,
				    uint32_t length, dma_operation_type_t op)
{
	int ret;
	struct intel_uart_context *uart =
	    CONTAINER_OF(inst, struct intel_uart_context, instance);
	volatile intel_uart_reg_t *regs = uart->regs;
	volatile intel_uart_reg_t *regs_phy = uart->phy_regs;
	dma_channel_direction_t dma_dir;
	dma_hs_per_rtx_t dma_hs_per;
	uint32_t src, dst;
	if (op == WRITE) {
		dma_dir = DMA_MEMORY_TO_PERIPHERAL;
		dma_hs_per = DMA_HS_PER_TX;
		src = (uintptr_t)buff;
		dst = (uintptr_t)(&regs_phy->rbr_thr_dll);
	} else if (op == READ) {
		dma_dir = DMA_PERIPHERAL_TO_MEMORY;
		dma_hs_per = DMA_HS_PER_RX;
		dst = (uintptr_t)buff;
		src = (uintptr_t)(&regs_phy->rbr_thr_dll);
		regs->dmasa |= INTEL_UART_DMASA;
	} else {
		return INTEL_DRIVER_ERROR;
	}
	ret = intel_uart_dma_config(dma_dev, channel, intel_dma_poll_dummy_cb,
				    NULL, op);
	if (ret != INTEL_DRIVER_OK) {
		return ret;
	}
	ret = intel_dma_control(dma_dev, channel, INTEL_CONFIG_DMA_HS_DEVICE_ID,
				uart->uart_dma_hs_id);

	if (ret != INTEL_DRIVER_OK) {
		return ret;
	}
	DBG_CHECK(0 == ret, INTEL_DRIVER_ERROR);
	ret = intel_dma_control(dma_dev, channel,
				INTEL_CONFIG_DMA_HS_DEVICE_ID_PER_DIR,
				dma_hs_per);

	if (ret != INTEL_DRIVER_OK) {
		return ret;
	}
	DBG_CHECK(0 == ret, INTEL_DRIVER_ERROR);
	ret = intel_dma_control(dma_dev, channel, INTEL_CONFIG_DMA_DIRECTION,
				dma_dir);
	if (ret != INTEL_DRIVER_OK) {
		return ret;
	}
	if (ret != INTEL_DRIVER_OK) {
		return ret;
	}

	regs->iir_fcr =
	    (INTEL_UART_FCR_FIFOE | INTEL_UART_FCR_DEFAULT_TX_RX_THRESHOLD);

	ret = intel_dma_start_transfer_polling(dma_dev, channel, src, dst,
					       length);

	if (ret != INTEL_DRIVER_OK) {
		return ret;
	}
	/* wait for transfer to complete */
	if (op == WRITE) {
		while (!(regs->lsr & INTEL_UART_LSR_TEMT)) {
		}
	}

	return INTEL_DRIVER_OK;
}

int intel_uart_dma_write_async(IN intel_instance_t *inst,
			       IN intel_uart_dma_xfer_t *const xfer)
{

	DBG_CHECK(xfer != NULL, INTEL_DRIVER_ERROR_PARAMETER);
	DBG_CHECK(xfer->dma_dev < INTEL_DMA_NUM, INTEL_DRIVER_ERROR_PARAMETER);
	DBG_CHECK(xfer->data != NULL, INTEL_DRIVER_ERROR_PARAMETER);
	DBG_CHECK(xfer->callback != NULL, INTEL_DRIVER_ERROR_PARAMETER);
	if (is_tx_disabled(inst)) {
		return INTEL_DRIVER_ERROR_UNSUPPORTED;
	}

	int32_t ret;
	ret = intel_uart_dma_io_async(inst, xfer, WRITE);
	return ret;
}

int intel_uart_dma_write_terminate(IN intel_instance_t *inst)
{
	struct intel_uart_context *uart =
	    CONTAINER_OF(inst, struct intel_uart_context, instance);

	const intel_uart_dma_xfer_t *xfer = uart->dma_ctxt.dma_xfer;
	int ret;

	/* No ongoing write transaction to be terminated. */
	if (xfer == NULL) {
		return INTEL_DRIVER_ERROR;
	}

	volatile intel_uart_reg_t *regs = uart->regs;
	regs->dmasa |= INTEL_UART_DMASA;

	DBG_CHECK(INTEL_DRIVER_OK == ret, INTEL_DRIVER_ERROR);
	ret =
	    intel_dma_uninit((intel_instance_t *)xfer->dma_dev, xfer->channel);
	if (xfer->callback && (ret == INTEL_DRIVER_OK)) {
		xfer->callback(xfer->cb_param, INTEL_USART_ERROR_CANCELED, 0,
			       0);
		uart->dma_ctxt.dma_xfer = NULL;
	}

	return ret;
}

int intel_uart_dma_read_async(IN intel_instance_t *inst,
			      IN intel_uart_dma_xfer_t *const xfer)
{
	DBG_CHECK(xfer != NULL, INTEL_DRIVER_ERROR_PARAMETER);
	DBG_CHECK(xfer->dma_dev < INTEL_DMA_NUM, INTEL_DRIVER_ERROR_PARAMETER);
	DBG_CHECK(xfer->data != NULL, INTEL_DRIVER_ERROR_PARAMETER);
	DBG_CHECK(xfer->callback != NULL, INTEL_DRIVER_ERROR_PARAMETER);
	if (is_rx_disabled(inst)) {
		return INTEL_DRIVER_ERROR_UNSUPPORTED;
	}

	int32_t ret;
	ret = intel_uart_dma_io_async(inst, xfer, READ);
	return ret;
}

int intel_uart_dma_read_terminate(IN intel_instance_t *inst)
{
	struct intel_uart_context *uart =
	    CONTAINER_OF(inst, struct intel_uart_context, instance);
	const intel_uart_dma_xfer_t *xfer = uart->dma_ctxt.dma_xfer;
	int ret;

	/* No ongoing read transaction to be terminated. */
	if (xfer == NULL) {
		return INTEL_DRIVER_ERROR;
	}

	volatile intel_uart_reg_t *regs = uart->regs;
	regs->dmasa |= INTEL_UART_DMASA;

	DBG_CHECK(INTEL_DRIVER_OK == ret, INTEL_DRIVER_ERROR);
	ret = intel_dma_uninit(xfer->dma_dev, xfer->channel);
	if (xfer->callback && (ret == INTEL_DRIVER_OK)) {
		xfer->callback(xfer->cb_param, INTEL_USART_ERROR_CANCELED, 0,
			       0);
		uart->dma_ctxt.dma_xfer = NULL;
	}

	return ret;
}

int intel_uart_dma_write_polled(IN intel_instance_t *inst,
				IN intel_instance_t *dma_dev,
				IN uint32_t channel, IN uint8_t *buff,
				IN uint32_t length)
{
	int ret;

	DBG_CHECK(buff != NULL, INTEL_DRIVER_ERROR_PARAMETER);
	DBG_CHECK(length != 0, INTEL_DRIVER_ERROR_PARAMETER);
	if (is_tx_disabled(inst)) {
		return INTEL_DRIVER_ERROR_UNSUPPORTED;
	}

	ret = intel_uart_dma_io_polled(inst, dma_dev, channel, buff, length,
				       WRITE);
	return ret;
}

int intel_uart_dma_read_polled(IN intel_instance_t *inst,
			       IN intel_instance_t *dma_dev,
			       IN uint32_t channel, OUT uint8_t *buff,
			       IN uint32_t length, OUT uint32_t *status)
{
	int ret;

	DBG_CHECK(buff != NULL, INTEL_DRIVER_ERROR_PARAMETER);
	DBG_CHECK(length != 0, INTEL_DRIVER_ERROR_PARAMETER);
	DBG_CHECK(status != 0, INTEL_DRIVER_ERROR_PARAMETER);
	if (is_tx_disabled(inst)) {
		return INTEL_DRIVER_ERROR_UNSUPPORTED;
	}

	struct intel_uart_context *uart =
	    CONTAINER_OF(inst, struct intel_uart_context, instance);
	volatile intel_uart_reg_t *regs = uart->regs;
	ret = intel_uart_dma_io_polled(inst, dma_dev, channel, buff, length,
				       READ);
	*status = (regs->lsr & INTEL_UART_LSR_ERROR_BITS);
	return ret;
}

int intel_uart_set_tx_only_mode(IN intel_instance_t *inst, bool tx_only)
{

	struct intel_uart_context *uart =
	    CONTAINER_OF(inst, struct intel_uart_context, instance);
	volatile intel_uart_reg_t *regs = uart->regs;

	if (tx_only) {
		uart->uart_xfer_ctrl.tx_disable = false;
		uart->uart_xfer_ctrl.rx_disable = true;

#if (HAS_UART_RS485_SUPPORT)
		regs->de_en = true;
		regs->re_en = false;
#endif
	} else {
		uart->uart_xfer_ctrl.tx_disable = false;
		uart->uart_xfer_ctrl.rx_disable = false;

#if (HAS_UART_RS485_SUPPORT)
		regs->de_en = true;
		regs->re_en = true;
#endif
	}
	return INTEL_DRIVER_OK;
}

int intel_uart_set_rx_only_mode(IN intel_instance_t *inst, bool rx_only)
{

	struct intel_uart_context *uart =
	    CONTAINER_OF(inst, struct intel_uart_context, instance);
	volatile intel_uart_reg_t *regs = uart->regs;

	if (rx_only) {
		uart->uart_xfer_ctrl.tx_disable = true;
		uart->uart_xfer_ctrl.rx_disable = false;

#if (HAS_UART_RS485_SUPPORT)
		regs->de_en = false;
		regs->re_en = true;
#endif
	} else {
		uart->uart_xfer_ctrl.tx_disable = false;
		uart->uart_xfer_ctrl.rx_disable = false;

#if (HAS_UART_RS485_SUPPORT)
		regs->de_en = true;
		regs->re_en = true;
#endif
	}

	return INTEL_DRIVER_OK;
}
