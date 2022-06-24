/*
 * Copyright (c) 2021 Intel Corporation
 *
 * SPDX-License-Identifier:  BSD-3-Clause
 */

#ifndef _INTEL_HAL_UART_H_
#define _INTEL_HAL_UART_H_

#include <intel/hal_driver_common.h>
#include <intel/hal_dma.h>
#include <intel/hal_device.h>
#include <../drivers/uart/dw_uart_priv.h>

/*!
 * \brief Get driver context
 */
#define INTEL_UART_INSTANCE(n)                                                 \
	((intel_instance_t *)((struct intel_uart_context[]){{                  \
	    .status_report_mask = INTEL_UART_LSR_ERROR_BITS,                   \
	    .uart_dma_hs_id = 0x88 + n, }}))

/**
 * UART Status type.
 */
typedef enum {
	INTEL_UART_IDLE = 0,		  /**< IDLE. */
	INTEL_UART_RX_OE = BIT(1),	/**< Receiver overrun. */
	INTEL_UART_RX_PE = BIT(2),	/**< Parity error. */
	INTEL_UART_RX_FE = BIT(3),	/**< Framing error. */
	INTEL_UART_RX_BI = BIT(4),	/**< Break interrupt. */
	INTEL_UART_TX_BUSY = BIT(5),      /**< TX Busy flag. */
	INTEL_UART_RX_BUSY = BIT(6),      /**< RX Busy flag. */
	INTEL_UART_TX_NFULL = BIT(7),     /**< TX FIFO not full. */
	INTEL_UART_RX_NEMPTY = BIT(8),    /**< RX FIFO not empty. */
	INTEL_UART_UNHANDLED_INT = BIT(9) /**< Unhandled Interrupt. */
} intel_uart_status_t;

/**
 * UART Line control.
 */
typedef enum {
	/**< 5 data bits, no parity, 1 stop bit. */
	INTEL_UART_LC_5N1 = 0x00,
	/**< 5 data bits, no parity, 1.5 stop bits. */
	INTEL_UART_LC_5N1_5 = 0x04,
	/**< 5 data bits, even parity, 1 stop bit. */
	INTEL_UART_LC_5E1 = 0x18,
	/**< 5 data bits, even parity, 1.5 stop bits. */
	INTEL_UART_LC_5E1_5 = 0x1c,
	/**< 5 data bits, odd parity, 1 stop bit. */
	INTEL_UART_LC_5O1 = 0x08,
	/**<  5 data bits, odd parity, 1.5 stop bits.*/
	INTEL_UART_LC_5O1_5 = 0x0c,
	/**<  6 data bits, no parity, 1 stop bit. */
	INTEL_UART_LC_6N1 = 0x01,
	/**<  6 data bits, no parity, 2 stop bits. */
	INTEL_UART_LC_6N2 = 0x05,
	/**<  6 data bits, even parity, 1 stop bit. */
	INTEL_UART_LC_6E1 = 0x19,
	/**<  6 data bits, even parity, 2 stop bits. */
	INTEL_UART_LC_6E2 = 0x1d,
	/**<  6 data bits, odd parity, 1 stop bit. */
	INTEL_UART_LC_6O1 = 0x09,
	/**<  6 data bits, odd parity, 2 stop bits. */
	INTEL_UART_LC_6O2 = 0x0d,
	/**<  7 data bits, no parity, 1 stop bit. */
	INTEL_UART_LC_7N1 = 0x02,
	/**<  7 data bits, no parity, 2 stop bits. */
	INTEL_UART_LC_7N2 = 0x06,
	/**<  7 data bits, even parity, 1 stop bit. */
	INTEL_UART_LC_7E1 = 0x1a,
	/**<  7 data bits, even parity, 2 stop bits. */
	INTEL_UART_LC_7E2 = 0x1e,
	/**<  7 data bits, odd parity, 1 stop bit. */
	INTEL_UART_LC_7O1 = 0x0a,
	/**<  7 data bits, odd parity, 2 stop bits. */
	INTEL_UART_LC_7O2 = 0x0e,
	/**<  8 data bits, no parity, 1 stop bit. */
	INTEL_UART_LC_8N1 = 0x03,
	/**<  8 data bits, no parity, 2 stop bits. */
	INTEL_UART_LC_8N2 = 0x07,
	/**<  8 data bits, even parity, 1 stop bit. */
	INTEL_UART_LC_8E1 = 0x1b,
	/**<  8 data bits, even parity, 2 stop bits. */
	INTEL_UART_LC_8E2 = 0x1f,
	/**<  8 data bits, odd parity, 1 stop bit. */
	INTEL_UART_LC_8O1 = 0x0b,
	/**<  8 data bits, odd parity, 2 stop bits. */
	INTEL_UART_LC_8O2 = 0x0f
} intel_uart_lc_t;

/**
 * UART RS-485 transfer mode.
 */
typedef enum {
	/** Full Duplex mode. */
	INTEL_UART_RS485_XFER_MODE_FULL_DUPLEX,

	/** Software Controlled Half Duplex mode. */
	INTEL_UART_RS485_XFER_MODE_HALF_DUPLEX

} intel_uart_rs485_transfer_mode_t;

/**
 * UART RS-485 polarity for de and re signals.
 */
typedef enum {
	INTEL_UART_RS485_POL_ACTIVE_LOW = 0,
	INTEL_UART_RS485_POL_ACTIVE_HIGH

} intel_uart_rs485_polarity;

typedef struct {
	/** de to re turnaround time in nanoseconds. */
	uint32_t de_re_tat;

	/** re to de turnaround time in nanoseconds. */
	uint32_t re_de_tat;

	/** Assertion time for DE in nanoseconds. */
	uint32_t de_assertion_time;

	/** De-assertion time for DE in nanoseconds */
	uint32_t de_deassertion_time;

	/** Transfer mode.*/
	intel_uart_rs485_transfer_mode_t transfer_mode : 1;

	/** de polarity 0:active low , 1:active high. */
	intel_uart_rs485_polarity de_polarity : 1;

	/** re polarity 0:active low , 1:active high. */
	intel_uart_rs485_polarity re_polarity : 1;

	/** Driver enable. */
	uint32_t de_en : 1;

	/** Receiver enable. */
	uint32_t re_en : 1;
} intel_uart_rs485_config_t;

/* Hardware / Software based address transmit
 * and address match control.
 */

typedef enum {
	INTEL_UART_9BIT_HW_ADDR_CTRL = 0,
	INTEL_UART_9BIT_SW_ADDR_CTRL

} intel_uart_9bit_addr_mode_t;

typedef struct {

	/** Address of this node when hardware address match enabled. */
	uint8_t receive_address;

	/** Addr Ctrl s/w or h/w enabled receive address match. */
	intel_uart_9bit_addr_mode_t addr_ctrl;

} intel_uart_9bit_config_t;

/**
 * UART configuration structure type
 */
typedef struct {
	intel_uart_lc_t line_control; /**< Line control (enum). */
	uint32_t baud_rate;	   /**< Baud Rate. */
	bool hw_fc;		      /**< Hardware Automatic Flow Control. */
	uint32_t clk_speed_hz;	/**< Clock speed for uart in Hz. */
} intel_uart_config_t;

/**
 * UART asynchronous transfer structure.
 */
typedef struct {
	uint8_t *data;     /**< Pre-allocated write or read buffer. */
	uint32_t data_len; /**< Number of bytes to transfer. */

	/** Transfer callback
	 *
	 * @param[in] data Callback user data.
	 * @param[in] INTEL_DRIVER_OK on success,negative value for possible
	 * errors.
	 * @param[in] status UART module status.To be interpreted with
	 * intel_uart_status_t for different error bits.
	 * @param[in] len Length of the UART transfer if successful, 0
	 * otherwise.
	 */
	void (*callback)(void *data, int error, uint32_t status, uint32_t len);
	void *callback_data; /**< Callback identifier. */
} intel_uart_transfer_t;

typedef struct {
	uint8_t *base; /* Start address. */
	uint32_t len;  /* Length of transfer */
} intel_uart_io_vec_t;

typedef struct {
	intel_uart_io_vec_t *vec; /* Pointer to vector of transfers */
	uint32_t count;		  /* Number of transfers requested. */
	void *cb_data;		  /* Callback data */
	void (*callback)(void *data, int error, uint32_t status,
			 uint32_t completed_count);
} intel_uart_io_vec_xfer_t;

/**
 * UART unsolicited receive structure
 */
typedef struct {
	uint8_t *buffer; /* Buffer for unsolicited receive */
	int32_t size;    /* Size of the buffer */
	void *cb_data;   /* Callback data or context */
	void (*unsol_rx_callback)(void *usr_data, int err, uint32_t status,
				  int len);
} intel_uart_unsol_rx_t;

/**
 * UART DMA transfer structure
 */

typedef struct {
	const intel_instance_t *dma_dev; /**< Dma device to be used. */
	int32_t channel;		 /**< Dma channel number to be used. */
	uint8_t *data;			 /**< Data ptr. */
	void (*callback)(void *cb_param, int error, uint32_t status,
			 uint32_t len);
	void *cb_param; /** Callback param. */
	uint32_t len;   /**< Length of transfer. */
} intel_uart_dma_xfer_t;

/**
 * Set UART configuration.
 *
 * Change the configuration of a UART module. This includes line control,
 * baud rate and hardware flow control.
 *
 * @param[in] uart Which UART module to configure.
 * @param[in] cfg New configuration for UART. This must not be NULL.
 *
 * @return Standard return code for INTEL.
 * @retval INTEL_DRIVER_OK on success, negative value for possible errors.
 */
int intel_uart_set_config(IN intel_instance_t *inst,
			  IN intel_uart_config_t *const cfg);

/**
 * Get UART status.
 *
 * Retrieve UART interface status. Return INTEL_UART_BUSY if transmitting
 * data; INTEL_UART_IDLE if available for transfer; INTEL_UART_TX_ERROR if an
 * error has occurred in transmission.
 *
 * The user may call this function before performing an UART transfer in order
 * to guarantee that the UART interface is available.

 * @param[in] uart Which UART to read the status of.
 * @param[out] status Current UART status. This must not be NULL.
 * The returned status is to be interpreted using the intel_uart_status_t for
 * different error conditions.
 *
 * @return Standard return code for INTEL.
 * @retval INTEL_DRIVER_OK on success, negative value for possible errors.
 */
int intel_uart_get_status(IN intel_instance_t *inst,
			  OUT uint32_t *const status);

/**
 * UART character data write.
 *
 * Perform a single character write on the UART interface.
 * This is a blocking synchronous call.
 *
 * @param[in] uart UART identifer.
 * @param[in] data Data to write to UART.
 *
 * @return Standard driver return type for INTEL.
 * @retval INTEL_DRIVER_OK  on success.
 * @retval Negative @ref errno for possible error codes.
 */
int intel_uart_write(IN intel_instance_t *inst, IN uint8_t data);

/**
 * UART character data read.
 *
 * Perform a single character read from the UART interface.
 * This is a blocking synchronous call.
 *
 * @param[in] uart UART identifer.
 * @param[out] data Data to read from UART. This must not be NULL.
 * @param[out] status UART specific status. The returned status to be
 * interpreted using intel_uart_status_t for different error conditions.
 *
 * @return Standard return code for INTEL.
 * @retval INTEL_DRIVER_OK on success, negative value for possible errors.
 */
int intel_uart_read(IN intel_instance_t *inst, OUT uint8_t *const data,
		    OUT uint32_t *const status);

/**
 *  Uart read multi bytes.
 *
 * Perform a read on the UART interface. This is a blocking
 * synchronous call. The function will block until all data has
 * been read to the buffer or an error has occured.
 *
 * @param[in] uart UART controller identifier
 * @param[OUT] data buffer where data is to be read.This must not be NULL.
 * @param[in] req_len Requested length of data to be read.
 * @param[OUT] comp_len Length of data read completed by this call.
 * @param[OUT] status Line status in case of an error.
 *
 * @return Standard return code for INTEL.
 * @retval INTEL_DRIVER_OK on success, negative value for possible errors.
 */

int intel_uart_read_buffer(IN intel_instance_t *inst, OUT uint8_t *const data,
			   IN uint32_t req_len, OUT uint32_t *comp_len,
			   OUT uint32_t *status);

/**
 * UART character data write.
 *
 * Perform a single character write on the UART interface.
 * This is a non-blocking synchronous call.
 *
 * @param[in] uart UART identifier.
 * @param[in] data Data to write to UART.
 *
 * @return Standard return code for INTEL.
 * @retval INTEL_DRIVER_OK on success, negative value for possible errors.
 */
int intel_uart_write_non_block(IN intel_instance_t *inst, IN uint8_t data);

/**
 * UART character data read.
 *
 * Perform a single character read from the UART interface.
 * This is a non-blocking synchronous call.
 *
 * @param[in] uart UART identifer.
 * @param[out] data Character read. This must not be NULL.
 *
 * @return Standard return code for INTEL.
 * @retval INTEL_DRIVER_OK on success, negative value for possible errors.
 */
int intel_uart_read_non_block(IN intel_instance_t *inst,
			      OUT uint8_t *const data);

/**
 * UART multi-byte data write.
 *
 * Perform a write on the UART interface. This is a blocking
 * synchronous call. The function will block until all data has
 * been transferred.
 *
 * @param[in] uart UART controller identifier
 * @param[in] data Data to write to UART. This must not be NULL.
 * @param[in] len Length of data to write to UART.
 *
 * @return Standard return code for INTEL.
 * @retval INTEL_DRIVER_OK on success, negative value for possible errors.
 */
int intel_uart_write_buffer(IN intel_instance_t *inst, IN uint8_t *const data,
			    IN uint32_t len);

/**
 * Interrupt based TX on UART.
 *
 * Perform an asynchronous interrupt based TX transfer on the UART bus.
 * The function will replenish the TX FIFOs on UART empty interrupts.
 * This function should not be called when an asynchronous write operation
 * is in already in progress.API user should treminate any ongong async writes
 * before issuing a new asysnchronous write.
 *
 * @param[in] uart UART identifier.
 * @param[in] xfer Structure containing pre-allocated
 *                 write buffer and callback functions.
 *                 The structure must not be NULL and must be kept valid until
 *                 the transfer is complete.
 *
 * @return Standard return code for INTEL.
 * @retval INTEL_DRIVER_OK on success, negative value for possible errors.
 */
int intel_uart_write_async(IN intel_instance_t *inst,
			   IN intel_uart_transfer_t *const xfer);

/**
 * Interrupt based RX on UART.
 *
 * Perform an asynchronous interrupt based RX transfer on the UART bus.
 * The function will read back the RX FIFOs on UART empty interrupts.
 * This function should not be called when an asynchronous receive operation
 * is already in progress. API user should terminate any ongoing async
 * read transfers before issuing an new aysnchronous read.
 *
 * @param[in] uart UART identifier.
 * @param[in] xfer Structure containing pre-allocated read
 *                 buffer and callback functions.
 *                 The structure must not be NULL and must be kept valid until
 *                 the transfer is complete.
 *
 * @return Standard return code for INTEL.
 * @retval INTEL_DRIVER_OK on success, negative value for possible errors.
 */
int intel_uart_read_async(IN intel_instance_t *inst,
			  IN intel_uart_transfer_t *const xfer);

/**
 * Terminate UART asynchronous(IRQ)TX transfer.
 *
 * Terminate the current IRQ TX transfer on the UART bus.
 * This will cause the relevant callbacks to be called.
 *
 * @param[in] uart UART identifier.
 *
 * @return Standard return code for INTEL.
 * @retval INTEL_DRIVER_OK on success, negative value for possible errors.
 */
int intel_uart_async_write_terminate(IN intel_instance_t *inst);

/**
 * Terminate UART asynchronous(IRQ) RX transfer.
 *
 * Terminate the current IRQ RX transfer on the UART bus.
 * This will cause the relevant callbacks to be called.
 *
 * @param[in] uart UART identifier.
 *
 * @return Standard return code for INTEL.
 * @retval INTEL_DRIVER_OK on success, negative value for possible errors.
 */
int intel_uart_async_read_terminate(IN intel_instance_t *inst);

/* Additional functions defined for UART Zephyr only. */

/**
 * Is there a pending IRQ for uart.
 *
 * This function returns a boolean value indiciating
 * whether an IRQ is pending for the given UART instance.
 *
 * @param[in] uart UART port index.
 *
 * @return Boolean value indicating whether IQR is pending.
 * @retval false if no irq is pending.
 *         true if irq is pending.
 */
bool intel_uart_is_irq_pending(IN intel_instance_t *inst);

/**
 * Is transmit operation complete.
 *
 * This function returns a boolean value indiciating
 * whether a transmit operation is cmplete or not.
 *
 *
 * @param[in] uart UART port index.
 *
 * @return Boolean value indication whether transmit operation is
 * complete.
 * @retval true if both tansmit holding register and transmitter are empty.
 *         false otherwise.
 */

bool intel_uart_is_tx_complete(IN intel_instance_t *inst);

/**
 * Is receive data available in the interupt context.
 *
 * This function returns a boolean value indiciating
 * whether the interrupt is asserted due to data availability
 * in the rx path.This API should be called in interrupt context
 * after  calling the IRQ update function.
 *
 * @param[in] uart UART port index.
 *
 * @return Boolean value indicating whether IRQ is asserted due to rx
 * data.
 * @retval false if IRQ is not due to rx data.
 *         true if IRQ is due ot rx data..
 */

bool intel_uart_is_irq_rx_ready(IN intel_instance_t *inst);

/**
 * Is transmit ready.
 *
 * This function returns a boolean value indiciating
 * whether transmit path is ready to accept data for transmit operation.
 *
 * @param[in] uart UART port index.
 *
 * @return boolean value indicating whether tx .
 * @retval false if transmit holding regsiter is not empty.
 *         true if transmit holding register is empty
 */

bool intel_uart_irq_tx_ready(IN intel_instance_t *inst);

/**
 * Fill uart fifo.
 *
 * This function fills the uart fifo with data provided in the
 * data pointer based on specifed length , based on the space available
 * in the transmit fifo.
 *
 * @param[in] uart UART port index.
 * @param[in] data pointer to data to be written.
 * @param[in] size length of data to be filled.
 *
 * @return length of data written to the tansmit fifo.
 */

int intel_uart_fifo_fill(IN intel_instance_t *inst, IN uint8_t *data,
			 IN uint32_t size);

/**
 * Read uart fifo.
 *
 * Reads the uart fifo into provided data buffer for specified length.If
 * data for specified read length is not availble in the rx fifo,the
 * function returns.
 *
 * @param[in] uart UART port index.
 * @param[out] data pointer where data is to be read.
 * @param[in] size length of data to be read.
 *
 * @return length of data read from the rx fifo.
 */

int intel_uart_fifo_read(IN intel_instance_t *inst, OUT uint8_t *data,
			 IN uint32_t size);

/**
 * Enable interrupt generation for transmit event.
 *
 * Enable interrupt genreation when transmit fifo/transmit holding
 * register is empty.
 *
 * @param[in] uart UART port index.
 *
 * @return Standard return code for INTEL.
 * @retval INTEL_DRIVER_OK on success, negative value for possible errors.
 */

int intel_uart_irq_tx_enable(IN intel_instance_t *inst);

/**
 * Disable interrupt generation for transmit event.
 *
 * Disable interrupt genreation when transmit fifo/transmit holding
 * register is empty.
 *
 * @param[in] uart UART port index.
 *
 * @return Standard return code for INTEL.
 * @retval INTEL_DRIVER_OK on success, negative value for possible errors.
 */

int intel_uart_irq_tx_disable(IN intel_instance_t *inst);

/**
 * Enable interrupt generation for receive event.
 *
 * Enable interrupt generation when data available is receive buffer.
 *
 * @param[in] uart UART port index.
 *
 * @return Standard return code for INTEL.
 * @retval INTEL_DRIVER_OK on success, negative value for possible errors.
 */

int intel_uart_irq_rx_enable(IN intel_instance_t *inst);

/**
 * Disable interrupt generation for receive event.
 *
 * Disable interrupt generation when data available is receive buffer.
 *
 * @param[in] uart UART port index.
 *
 * @return Standard return code for INTEL.
 * @retval INTEL_DRIVER_OK on success, negative value for possible errors.
 */
int intel_uart_irq_rx_disable(IN intel_instance_t *inst);

/**
 * Enable interrupt generation for error conditions.
 *
 * Enable interrupt for error conditions - fifo overrun,
 * parity,framing error,break condition.
 *
 * @param[in] uart UART port index.
 *
 * @return Standard return code for INTEL.
 * @retval INTEL_DRIVER_OK on success, negative value for possible errors.
 */

int intel_uart_irq_err_enable(IN intel_instance_t *inst);
/**
 * Disable interrupt generation for error conditions.
 *
 * Disable interrupt for error conditions - fifo overrun,
 * parity,framing error,break condition.
 *
 * @param[in] uart UART port index.
 *
 * @return Standard return code for INTEL.
 * @retval INTEL_DRIVER_OK on success, negative value for possible errors.
 */

int intel_uart_irq_err_disable(IN intel_instance_t *inst);

/**
 * Update the cached values of the IRQ status.
 *
 * Updates the cached values of uart iid status register.
 * Caching is required as many interrupt status bits get cleared on
 * reading.
 *
 * @param[in] uart UART port index.
 *
 * @return Standard return code for INTEL.
 * @retval INTEL_DRIVER_OK on success, negative value for possible errors.
 */
int intel_uart_update_irq_cache(IN intel_instance_t *inst);

/**
 * Sets the baud rate for uart specifed uart port.
 *
 * @param[in] uart UART port index.
 * @param[in] baud_rate to set.
 * @param[in] clock speed suppled to UART IP ,in Hz.
 *
 * @return Standard return code for INTEL.
 * @retval INTEL_DRIVER_OK on success, negative value for possible errors.
 */

int intel_uart_set_baud_rate(IN intel_instance_t *inst, IN uint32_t baud_rate,
			     IN uint32_t clock_speed_hz);

/**
 * Gets the current configuration for the specified UART port.
 *
 * @param[in] uart UART port index.
 * @param[out] cfg pointer to configuration structure populated by this
 * call.
 *
 * @return Standard return code for INTEL.
 * @retval INTEL_DRIVER_OK on success, negative value for possible errors.
 */

int intel_uart_get_config(IN intel_instance_t *inst,
			  OUT intel_uart_config_t *cfg);

/**
 *  Set uart loopback mode.
 *
 * @param[in] uart UART port index.
 *
 * @return Standard return code for INTEL.
 * @retval INTEL_DRIVER_OK on success, negative value for possible errors.
 */

int intel_uart_set_loopback_mode(IN intel_instance_t *inst);

/**
 *  Clear uart loopback mode.
 *
 * @param[in] uart UART port index.
 *
 * @return Standard return code for INTEL.
 * @retval INTEL_DRIVER_OK on success, negative value for possible errors.
 */

int intel_uart_clr_loopback_mode(IN intel_instance_t *inst);

/**
 *  Set uart break condition.
 *
 * @param[in] uart UART port index.
 *
 * @return Standard return code for INTEL.
 * @retval INTEL_DRIVER_OK on success, negative value for possible errors.
 */

int intel_uart_set_break_con(IN intel_instance_t *inst);

/**
 *  Clear uart break condition.
 *
 * @param[in] uart UART port index.
 *
 * @return Standard return code for INTEL.
 * @retval INTEL_DRIVER_OK on success, negative value for possible errors.
 */

int intel_uart_clr_break_con(IN intel_instance_t *inst);

/**
 *  Enable uart auto flow control.
 *
 * @param[in] uart UART port index.
 *
 * @return Standard return code for INTEL.
 * @retval INTEL_DRIVER_OK on success, negative value for possible errors.
 */

int intel_uart_auto_fc_enable(IN intel_instance_t *inst);

/**
 *  Disable uart auto flow control.
 *
 * @param[in] uart UART port index.
 *
 * @return Standard return code for INTEL.
 * @retval INTEL_DRIVER_OK on success, negative value for possible errors.
 */

int intel_uart_auto_fc_disable(IN intel_instance_t *inst);

/**
 *  Set line status report mask.
 *
 * This function sets the line status errors that can be reported during
 * recevie operations.The errors include break condition , framing
 * errors,parity error and fifo overrun error.
 * The mask can be generated by OR ing the paramters of intel_uart_lc_t.
 *
 * @param[in] uart UART port index.
 * @param[in] mask : bit mask specifying the line staus errors to be reported.
 *
 * @return Standard return code for INTEL.
 * @retval INTEL_DRIVER_OK on success, negative value for possible errors.
 */

int intel_set_ln_status_report_mask(IN intel_instance_t *inst,
				    IN uint32_t mask);

/**
 *  UART interrupt handler function.
 *
 * @param[in] uart UART port index.
 *
 * @return void :No return value.
 */

void intel_uart_isr_handler(IN intel_instance_t *inst);
/**
 *  Enable unsolicited receive.
 *
 * This function enables unsolicited receive using the buffer provided.
 * The buffered data is maintained as a circular buffer and oldest unread data
 * is overwritten in the event of received data exceeding the buffer length.
 * On receving any data on the uart device , user callback is triggered
 * indicating the status and length of the accumulated data. User may call the
 * intel_uart_get_unsol_data API to read the accumulated data.Caller must set
 * the unsol_rx_callback field in the intel_uart_unsol_rx_t otherwise error
 * would be returned.
 *
 * @param[in] uart UART port index.
 * @param[in] unsol_rx pointer to unsolicited receive structure of type
 *            intel_unsol_rx_t.
 *
 * @return Standard return code for INTEL.
 * @retval INTEL_DRIVER_OK on success, negative value for possible errors.
 */

int intel_uart_enable_unsol_rx(IN intel_instance_t *inst,
			       IN intel_uart_unsol_rx_t *const unsol_rx);

/**
 * Disable unsolicited read.
 *
 * @param[in] uart UART port index.
 *
 * @return Standard return code for INTEL.
 * @retval INTEL_DRIVER_OK on success, negative value for possible errors.
 */

int intel_uart_disable_unsol_rx(IN intel_instance_t *inst);

/**
 *  Get unsolicited data received. This may be called for the unsolicited
 *  receive callback to read the data accumulated.
 *
 * @param[in] uart UART port index.
 * @param[in] buffer where data is to be transferred.
 * @param[in] len length of data to be copied.
 *
 * @return Standard return code for INTEL.
 * @retval INTEL_DRIVER_OK on success, negative value for possible errors.
 */

int intel_uart_get_unsol_data(IN intel_instance_t *inst, OUT uint8_t *buffer,
			      IN int len);

/**
 *  Get length of unsolicited data received.
 *
 * @param[in] uart UART port index.
 * @param[OUT] p_len  pointer to int which would be populated by length.
 *
 * @return Standard return code for INTEL.
 * @retval INTEL_DRIVER_OK on success, negative value for possible errors.
 */

int intel_uart_get_unsol_data_len(IN intel_instance_t *inst, OUT int *p_len);
/**
 *  Set RTS line to logical high when Auto Flow Control is disabled.
 *  If auto flow contorl is enabled, error is returned.
 *
 * @param[in] uart UART port index.
 *
 * @return Standard return code for INTEL.
 * @retval INTEL_DRIVER_OK on success, negative value for possible errors.
 */

int intel_uart_assert_rts(IN intel_instance_t *inst);

/**
 *  Set RTS line to logical low  when Auto Flow Control is disabled.
 *
 *  If auto flow contorl is enabled, error is returned.
 *
 * @param[in] uart UART port index.
 *
 * @return Standard return code for INTEL.
 * @retval INTEL_DRIVER_OK on success, negative value for possible errors.
 */

int intel_uart_de_assert_rts(IN intel_instance_t *inst);

/**
 *  Read the current state of CTS line.
 * @param[out] p_rts pointer to uint32_t to hold current CTS value.
 *
 * @param[in] uart UART port index.
 *
 * @return Standard return code for INTEL.
 * @retval INTEL_DRIVER_OK on success, negative value for possible errors.
 */

int intel_uart_read_cts(IN intel_instance_t *inst, OUT uint32_t *p_cts);

/**
 *  Read the current state of RTS line.
 *
 * @param[in] uart UART port index.
 * @param[out] p_rts pointer to uint32_t to hold current RTS value.
 *
 * @return Standard return code for INTEL.
 * @retval INTEL_DRIVER_OK on success, negative value for possible errors.
 */

int intel_uart_read_rts(IN intel_instance_t *inst, OUT uint32_t *p_rts);

/**
 *  Read the current state of loopback mode.
 *
 * @param[in] uart UART port index.
 * @param[out] p_mode pointer to uint32_t to hold current state of loopback
 * mode.
 *
 * @return Standard return code for INTEL.
 * @retval INTEL_DRIVER_OK on success, negative value for possible errors.
 */

int intel_uart_get_loopback_mode(IN intel_instance_t *inst,
				 OUT uint32_t *p_mode);

/**
 *  Get the current line stauts report mask.
 *
 * @param[in] uart UART port index.
 * @param[out] p_mask pointer to uint32_t to hold current line status mask
 * mode.
 *
 * @return Standard return code for INTEL.
 * @retval INTEL_DRIVER_OK on success, negative value for possible errors.
 */

int intel_get_ln_status_report_mask(IN intel_instance_t *inst,
				    OUT uint32_t *p_mask);

/**
 *  Perform non-contiguous buffer asynchronous writes.
 *
 * @param[in] uart UART port index.
 * @param[in] vec_xfer pointer to vector transfer of type
 * intel_uart_io_vec_xfer_t specifying  the number of transfers in the count
 * field and the io_vector in the vec field. User callback is called after all
 * the writes specifed in the vec fieald are completed or an error is detected.
 * Callback field may be null.This call is non-blocking.
 *
 * @return Standard return code for INTEL.
 * @retval INTEL_DRIVER_OK on success, negative value for possible errors.
 */

int intel_uart_write_vec_async(IN intel_instance_t *inst,
			       IN intel_uart_io_vec_xfer_t *const vec_xfer);

/**
 *  Perform non-contiguous buffer asynchronous reads.
 *
 * @param[in] uart UART port index.
 * @param[in] vec_xfer pointer to vector transfer of type
 * intel_uart_io_vec_xfer_t specifying  the number of transfers in the count
 * field and the io_vector in the vec field. User callback is called after all
 * the reads specified in the vec field are completed or an error is detected.
 * Callback field may be null.This call is non-blocking.
 *
 * @return Standard return code for INTEL.
 * @retval INTEL_DRIVER_OK on success, negative value for possible errors.
 */
int intel_uart_read_vec_async(IN intel_instance_t *inst,
			      IN intel_uart_io_vec_xfer_t *const vec_xfer);

/**
 * Asynchronous DMA write on the specified UART port.
 *
 * @param[in] uart UART port index.
 * * @param[out] dma transfer strucutre.
 * call.
 *
 * @return Standard return code for INTEL.
 * @retval INTEL_DRIVER_OK on success, negative value for possible errors
 */

int intel_uart_dma_write_async(IN intel_instance_t *inst,
			       IN intel_uart_dma_xfer_t *const xfer);

/**
 * Terminate UART asynchronous DMA write operation.
 *
 * Terminate an ongoing DMA write transfer.
 * This will cause the relevant callbacks to be called.
 *
 * @param[in] uart UART identifier.
 *
 * @return Standard return code for INTEL.
 * @retval INTEL_DRIVER_OK on success, negative value for possible errors.
 */
int intel_uart_dma_write_terminate(IN intel_instance_t *inst);

/**
 * Asynchronous DMA read on the specified UART port.
 *
 * @param[in] uart UART port index.
 * @param[out] dma transfer strucutre.
 * call.
 *
 * @return Standard return code for INTEL.
 * @retval INTEL_DRIVER_OK on success, negative value for possible errors
 */

int intel_uart_dma_read_async(IN intel_instance_t *inst,
			      IN intel_uart_dma_xfer_t *const xfer);

/**
 * Terminate UART asynchronous DMA read operation.
 *
 * Terminate an ongoing DMA read transfer.
 * This will cause the relevant callbacks to be called.
 *
 * @param[in] uart UART identifier.
 *
 * @return Standard return code for INTEL.
 * @retval INTEL_DRIVER_OK on success, negative value for possible errors.
 */

int intel_uart_dma_read_terminate(IN intel_instance_t *inst);

/**
 * Polled DMA write on the specified UART port.
 *
 * @param[in] uart UART port index.
 * @param[in] dma_dev  Dma controller to use for transaction.
 * @param[in] channel  Dma channel to be used for transaction.
 * @param[in] buff   Pointer to uint8_t, from which data would be written.
 * @param[in] length of data to be read,
 *
 * @return Standard return code for INTEL.
 * @retval INTEL_DRIVER_OK on success, negative value for possible errors
 */

int intel_uart_dma_write_polled(IN intel_instance_t *inst,
				IN intel_instance_t *dma_inst,
				IN uint32_t channel, IN uint8_t *buff,
				IN uint32_t length);

/**
 *  Polled DMA read on the specified UART port.
 *
 * @param[in] uart UART port index.
 * @param[in] dma_dev  Dma controller to use for transaction.
 * @param[in] channel  Dma channel to be used for transaction.
 * @param[out] buff   Pointer to uint8_t, where data would be read.
 * @param[in] length of data to be read,
 * @param[out] status Pointer to uint32_t to report line status.
 *
 * @return Standard return code for INTEL.
 * @retval INTEL_DRIVER_OK on success, negative value for possible erors
 */

int intel_uart_dma_read_polled(IN intel_instance_t *inst,
			       IN intel_instance_t *dma_inst,
			       IN uint32_t channel, OUT uint8_t *buff,
			       IN uint32_t length, OUT uint32_t *status);

/**
 *  Set RS-485 config.
 *
 * @param[in] uart UART port index.
 * @param[in] cfg  Pointer to intel_uart_rs485_config_t.
 *
 * @return Standard return code for INTEL.
 * @retval INTEL_DRIVER_OK on success, negative value for possible erors
 */

int intel_uart_rs485_set_config(IN intel_instance_t *inst,
				IN intel_uart_rs485_config_t *cfg);

/**
 *  Set or clear uart rx-only mode.
 *
 *  This APIs sets uart port to rx-only mode when rx_only paramter is true.
 *  When rx_only paramter is false, it clears rx-only mode and enables both
 *  tx and rx operation for the given port, irrespective of current mode.
 *
 *  On enabling rx-only mode all transmit APIs return error.
 *  If RS-485 mode is enabled, the driver enable signal(de) is disabled and
 *  only receiver enable (re) signal can be activated.
 *
 * @param[in] uart UART port index.
 * @param[in] rx_only when rx_only flag set to true rx-only mode is enabled and
 * disabled when set to false.
 *
 * @return Standard return code for INTEL.
 * @retval INTEL_DRIVER_OK on success, negative value for possible erors
 */

int intel_uart_set_rx_only_mode(IN intel_instance_t *inst, bool rx_only);

/**
 *  Set or clear uart tx-only mode.
 *
 *  This APIs sets uart port to tx-only mode when tx_only paramter is true.
 *  When tx_only paramter is false, it clears tx-only mode and enables both tx
 *  and rx operation for the given port, irrespective of the current mode.
 *
 *  On enabling tx-only mode all receive functions return error.
 *  If RS-485 mode is enabled, the recevier enable signal(re) is disabled  and
 *  only driver enable (de) signal can be activated.
 *
 * @param[in] uart UART port index.
 * @param[in] tx_only when flag set to true tx-only mode is enabled and disabled
 *            when set to false.
 *
 * @return Standard return code for INTEL.
 * @retval INTEL_DRIVER_OK on success, negative value for possible erors
 */

int intel_uart_set_tx_only_mode(IN intel_instance_t *inst, bool tx_only);

/**
 *  Enable rs-485 signals for selected uart.
 *  Should be enabled after configuring using intel_uart_set_config.
 *
 * @param[in] uart UART port index.
 *
 * @return Standard return code for INTEL.
 * @retval INTEL_DRIVER_OK on success, negative value for possible erors
 */

int intel_uart_rs485_enable(IN intel_instance_t *inst);

/**
 *  Disble rs-485 signals for selected uart.
 *
 * @param[in] uart UART port index.
 *
 * @return Standard return code for INTEL.
 * @retval INTEL_DRIVER_OK on success, negative value for possible erors
 */

int intel_uart_rs485_disable(IN intel_instance_t *inst);

/**
 *  Get rs485 configuration for selected uart.
 *
 * @param[in] uart UART port index.
 * @param[out] cfg pointer to iseis_uart_rs485_config_t populated by the call.
 *
 * @return Standard return code for INTEL.
 * @retval INTEL_DRIVER_OK on success, negative value for possible erors
 */

int intel_uart_rs485_get_config(IN intel_instance_t *inst,
				OUT intel_uart_rs485_config_t *cfg);

/**
 *  Set the configuration for 9-bit operation.
 *
 * @param[in] uart UART port index.
 * @param[in] cfg pointer to configuration for 9-bit operation
 *
 * @return Standard return code for INTEL.
 * @retval INTEL_DRIVER_OK on success, negative value for possible erors
 */

int intel_uart_9bit_set_config(IN intel_instance_t *inst,
			       IN intel_uart_9bit_config_t *cfg);

/**
 *  Disable 9-bit operation.
 *
 * @param[in] uart UART port index.
 *
 * @return Standard return code for INTEL.
 * @retval INTEL_DRIVER_OK on success, negative value for possible erors
 */

int intel_uart_9bit_disable(IN intel_instance_t *inst);

/**
 *  Enable 9-bit operation.
 *
 * @param[in] uart UART port index.
 *
 * @return Standard return code for INTEL.
 * @retval INTEL_DRIVER_OK on success, negative value for possible erors
 */

int intel_uart_9bit_enable(IN intel_instance_t *inst);

/**
 *  Transmit 9-bit destination address.
 *
 * @param[in] uart UART port index.
 * @param[in] address address to send.
 *
 * @return Standard return code for INTEL.
 * @retval INTEL_DRIVER_OK on success, negative value for possible erors
 */

int intel_uart_9bit_send_address(IN intel_instance_t *inst, uint8_t address);

/**
 *  Get current configuration for 9-bit mode.
 *
 * @param[in] uart UART port index.
 * @param[out] cfg pointer to intel_uart_9bit_config_t populated by this call.
 *
 * @return Standard return code for INTEL.
 * @retval INTEL_DRIVER_OK on success, negative value for possible erors
 */

int intel_uart_9bit_get_config(IN intel_instance_t *inst,
			       intel_uart_9bit_config_t *cfg);

/**
 * @brief  UART set power state.
 *
 * Set UART instance to specified power state
 *
 * @param[in] power state to be entered.
 *
 * @retval INTEL_DRIVER_OK if operation was successful.
 * @retval non zero error code otherwise.
 */

int32_t intel_uart_set_power(IN intel_instance_t *inst,
			     IN intel_power_state_t state);

#include <intel/hal_uart_priv.h>

#endif /* _INTEL_HAL_UART_H_ */
