/*
 * Copyright (c) 2021 Intel Corporation
 *
 * SPDX-License-Identifier:  BSD-3-Clause
 */

#ifndef _INTEL_HAL_SPI_H_
#define _INTEL_HAL_SPI_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <intel/hal_driver_common.h>
#include <intel/hal_dma.h>
#include <intel/hal_device.h>

/*!
 * \defgroup intel_driver_spi SPI
 * \ingroup intel_driver
 */

#define INTEL_SPI_API_VERSION INTEL_DRIVER_VERSION_MAJOR_MINOR(0, 1)

/*!
 * \brief Get driver context
 */
#define INTEL_SPI_INSTANCE(n) ((intel_instance_t *)((struct spi_context[]){{\
			.spi_dma_hs_id = 0x98 + n,}}))

/*!
 * \defgroup spi_control_mode  SPI Control
 * \ingroup intel_driver_spi
 * \{
 */

enum intel_spi_ioctl {

	/*!
	 * Clock Polarity 0, Clock Phase 0
	 */
	INTEL_SPI_IOCTL_CPOL0_CPHA0,

	/*!
	 * Clock Polarity 0, Clock Phase 1
	 */
	INTEL_SPI_IOCTL_CPOL0_CPHA1,

	/*!
	 * Clock Polarity 1, Clock Phase 0
	 */
	INTEL_SPI_IOCTL_CPOL1_CPHA0,

	/*!
	 * Clock Polarity 1, Clock Phase 1
	 */
	INTEL_SPI_IOCTL_CPOL1_CPHA1,

	/*!
	 * Texas Instruments Frame Format
	 * No need to input args
	 */
	INTEL_SPI_IOCTL_TI_SSI,

	/*!
	 * National Microwire Frame Format
	 * Need to input microwire configure pointer
	 */
	INTEL_SPI_IOCTL_MICROWIRE,

	/*!
	 * Set the number of bits per SPI frame
	 * The parameter range is [1,16]
	 * For 4 bits operation, users need to provide an expand buffer, which
	 * covert 4-bits data to 8-bits data. The transfer size shall also use
	 * the expanded buffer size while do transfer.
	 */
	INTEL_SPI_IOCTL_DATA_WIDTH,

	/*!
	 * MSB first send out, default
	 */
	INTEL_SPI_IOCTL_MSB,

	/*!
	 * LSB first send out, it should be supported by hardware
	 */
	INTEL_SPI_IOCTL_LSB,

	/*!
	 * The SPI will not drive or monitor the CS pin, user drives it with
	 * GPIO API before and after transfaction, or directly connect the
	 * SPI slave's CS pin to a fixed low level.
	 */
	INTEL_SPI_IOCTL_CS_UNUSED,

	/*!
	 * The SPI auto active the CS pin before transaction, and deactive it
	 * after complete or abort.
	 * The parameter is bitmask of CS pin been controlled by SPI
	 */
	INTEL_SPI_IOCTL_CS_HW,

	/*!
	 * Get SPI speed in Hz
	 */
	INTEL_SPI_IOCTL_SPEED_SET,

	/*!
	 * Set SPI speed in Hz
	 */
	INTEL_SPI_IOCTL_SPEED_GET,

	/*!
	 * Set the FIFO depth, set it large for bulk transfer, set it small
	 * for few data transfer (register access).
	 * For bulk transfer, the data length should be aligned with the FIFO
	 * depth.
	 */
	INTEL_SPI_IOCTL_TX_WATERMARK_SET,

	/*!
	 * Set the FIFO depth, set it large for bulk transfer, set it small
	 * for few data transfer (register access).
	 * For bulk transfer, the data length should be aligned with the FIFO
	 * depth.
	 */
	INTEL_SPI_IOCTL_RX_WATERMARK_SET,

	/*!
	 * Get the FIFO depth
	 */
	INTEL_SPI_IOCTL_TX_WATERMARK_GET,

	/*!
	 * Get the FIFO depth
	 */
	INTEL_SPI_IOCTL_RX_WATERMARK_GET,

	/*!
	 * Set the dummy data on the bus, for example, master wants to read
	 * from slave. It can be 0x00 or 0xFF, which means keep data line low
	 * level or high level.
	 */
	INTEL_SPI_IOCTL_DUMMY_DATA,

	/*!
	 * Abort the data transfer.
	 */
	INTEL_SPI_IOCTL_ABORT,

	/*!
	 * Loopback mode for test
	 */
	INTEL_SPI_IOCTL_LOOPBACK,

	/*!
	 * SPI transfer buffer sets, it means in a transfer, the buffer can be
	 * not physical continous. Users need to update buffer while callback
	 * event is INTEL_SPI_EVENT_TX_FINISHED or intel_SPI_EVENT_RX_FINISHED.
	 * Usrs need to use function intel_spi_update_tx_buf or
	 * intel_spi_update_rx_buf to update buffers.
	 * Input 1 means enable this mode, 0 means disable this mode.
	 * Notice: This mode only supported in irq transfer mode.
	 */
	INTEL_SPI_IOCTL_BUFFER_SETS,

	/*!
	 * Set SPI rx sample delay, the unit is a SPI clock cycle, this need to
	 * set in some high speed (higher than 25M) transfer to devices.
	 */
	INTEL_SPI_IOCTL_SET_RX_DELAY,

	/*!
	 * Indicates the end of ioctl code
	 */
	INTEL_SPI_IOCTL_NUM
};

/*!
 * \}
 */

/*!
 * \defgroup spi_event SPI Event
 * \ingroup intel_driver_spi
 * \{
 */

/*! transfer complete */
#define INTEL_SPI_EVENT_COMPLETE (1UL << 0)

/*! data lost happens, for example, tx underflow or rx overflow */
#define INTEL_SPI_EVENT_DATA_LOST (1UL << 1)

/*!
 * it will be detected by hardware, master detect the CS pin deactive while
 * it transfer data, see intel_spi_capabilities to get if hw support it.
 */
#define INTEL_SPI_EVENT_MODE_FAULT (1UL << 2)

#define INTEL_SPI_EVENT_TX_FINISHED (1UL << 3)

#define INTEL_SPI_EVENT_RX_FINISHED (1UL << 4)

/*!
 * \}
 */

/*!
 * \struct intel_spi_status_t
 * \brief SPI Status, be used if callback has not been registered
 * \ingroup intel_driver_spi
 */
typedef volatile struct {
	uint32_t busy : 1;
	uint32_t data_lost : 1;
	uint32_t mode_fault : 1;
	uint32_t is_available : 1; /** 1:available 0:used by host  **/
	uint32_t reserved : 28;
} intel_spi_status_t;

/*!
 * \struct intel_spi_capabilities_t
 * \brief SPI Capabilities, the hardware feature
 * \ingroup intel_driver_spi
 */
typedef struct {
	uint32_t ti_ssi : 1;
	uint32_t microwire : 1;
	uint32_t mode_fault : 1;
	uint32_t is_available : 1; /** 1:available 0:used by host  **/
	uint32_t reserved : 28;
} intel_spi_capabilities_t;

/*!
 * \struct intel_spi_microwire_config_t
 * \brief Microwire configure
 * \ingroup intel_driver_spi
 */
typedef struct {
	bool microwire_handshake; /*!< If enable microwire handshake  */
	bool data_direction_tx;   /*!< True means data to transmit, false means
				     data for receive */
	bool sequential_mode;     /*!< true means only need one control data for
				     data transfer, false means one data needs one
				     control data*/
} intel_spi_microwire_config_t;

/*!
 * \defgroup spi_event_handler SPI Event Handler Callback
 * \ingroup intel_driver_spi
 * \{
 */

/*!
 * \typedef intel_spi_event_cb_t
 * \brief Callback function type for signal spi event.
 * \param[in] event: event type. see \ref spi_event
 * \param[inout] param: user parameter
 * \return    void
 */
typedef void (*intel_spi_event_cb_t)(IN uint32_t event, INOUT void *param);

/*!
 * \}
 */

/*!
 * \defgroup spi_function_calls SPI Driver Function Calls
 * \ingroup intel_driver_spi
 * \{
 */

/*!
 * \brief Get the spi driver's API version.
 * \return the version of current spi driver's API
 */
intel_driver_version_t intel_spi_get_version(void);

/*!
 * \brief Get the device's capabilities.
 * \param[in] spi_device: spi device id
 * \param[out] cap: spi device capabilities
 * \return  \ref return_status
 */
int32_t intel_spi_get_capabilities(INOUT intel_instance_t *inst,
				   OUT intel_spi_capabilities_t *cap);

/*!
 * \brief Initialize the device
 * \param[in] spi_device: spi device id
 * \param[in] cb: the callback function which can receive device's events.
 * \return  \ref return_status
 */
int32_t intel_spi_init(INOUT intel_instance_t *inst, IN intel_spi_event_cb_t cb,
		       INOUT void *param);

/*!
 * \brief Uninitailize the device
 * \param[in] spi_device: spi device id
 * \return  \ref return_status
 */
int32_t intel_spi_uninit(INOUT intel_instance_t *inst);

/*!
 * \brief Set the device's power
 * \param[in] spi_device: spi device id
 * \param[in] state: the power state to be set to the device
 * \return  \ref return_status
 */
int32_t intel_spi_set_power(INOUT intel_instance_t *inst,
			    IN intel_power_state_t state);

/*!
 * \brief  Get transferred data count
 * \param[in] spi_device: spi device id
 * \return number of data bytes transferred;
 */
int32_t intel_spi_get_data_count(INOUT intel_instance_t *inst);

/*!
 * \brief  Control the spi device
 * \note Shall not use it in dma mode, as DMA HW cannot support get transferred
 * bytes. \param[in] spi_device: spi device id \param[in] control: control
 * operation code. see \ref spi_control_codes \param[in] arg: argument of
 * operation (optional) \return  \ref return_status
 */
int32_t intel_spi_control(INOUT intel_instance_t *inst, IN uint32_t control,
			  IN uintptr_t arg);

/*!
 * \brief  Get device's status
 * \param[in] spi_device: spi device id
 * \param[out] status: spi device status
 * \return  \ref return_status
 */
int32_t intel_spi_get_status(INOUT intel_instance_t *inst,
			     OUT intel_spi_status_t *status);

/*!
 * \brief Update tx buffer during a spi irq transfer
 * This function used to tell spi driver a new tx buffer during a irq transfer.
 * Typically used in callback function with event INTEL_SPI_EVENT_TX_FINISHED.
 * Notice, if not call this function in callback, will cause data not send
 * correct data.
 * \param[in] spi_device: spi device id
 * \param[in] *tx_buffer: pointer of the buffer with data
 *                   which need send to slave
 * \param[in] len: number of data bytes to transfer
 * \return  \ref return_status
 */
int32_t intel_spi_update_tx_buf(INOUT intel_instance_t *inst, IN uint8_t *tx_buf,
				IN uint32_t len);

/*!
 * \brief Update rx buffer during a spi irq transfer
 * This function used to tell spi driver a new rx buffer during a irq transfer.
 * Typically used in callback function with event INTEL_SPI_EVENT_RX_FINISHED.
 * Notice, if not call this function in callback, will cause data not receive
 * to correct location.
 * \param[in] spi_device: spi device id
 * \param[out] *rx_buffer: pointer of the buffer with data
 *                   which need read from the slave
 * \param[in] len: number of data bytes to transfer
 * \return  \ref return_status
 */
int32_t intel_spi_update_rx_buf(INOUT intel_instance_t *inst, OUT uint8_t *rx_buf,
				IN uint32_t len);

/*!
 * \brief Start transmitting data to spi slave device as master
 * \param[in] spi_device: spi device id
 * \param[in] *data_out: pointer of the buffer with data
 *                   which need write to the slave
 * \param[out] *data_in: pointer of the buffer with data
 *                   which need read from the slave
 * \param[in] num: number of data bytes to transfer
 * \return  \ref return_status
 */
int32_t intel_spi_transfer(INOUT intel_instance_t *inst, IN uint8_t *data_out,
			   OUT uint8_t *data_in, IN uint32_t num, IN bool poll_mode);

/*!
 * \brief Start transmitting data to spi slave device as master in polling mode
 * \param[in] spi_device: spi device id
 * \param[in] *data_out: pointer of the buffer with data
 *                   which need write to the slave
 * \param[out] *data_in: pointer of the buffer with data
 *                   which need read from the slave
 * \param[in] num: number of data bytes to transfer
 * \return  \ref return_status
 */
int32_t intel_spi_poll_transfer(INOUT intel_instance_t *inst,
				IN uint8_t *data_out, OUT uint8_t *data_in,
				IN uint32_t num);

/*!
 * \brief Start transmitting data to spi slave device as master in dma mode
 * \param[in] spi_device: spi device id
 * \param[in] tx_dma: dma instance
 * \param[in] tx_dma_chan: dma channel id
 * \param[in] *data_out: pointer of the buffer with data
 *                   which need write to the slave
 * \param[in] rx_dma: dma instance
 * \param[in] rx_dma_chan: dma channel id
 * \param[out] *data_in: pointer of the buffer with data
 *                   which need read from the slave
 * \param[in] num: number of data bytes to transfer
 * \return  \ref return_status
 */
int32_t intel_spi_dma_transfer(INOUT intel_instance_t *inst,
			       INOUT intel_instance_t *tx_dma,
			       IN uint32_t tx_dma_chan, IN uint8_t *data_out,
			       INOUT intel_instance_t *rx_dma,
			       IN uint32_t rx_dma_chan, OUT uint8_t *data_in,
			       IN uint32_t num);

void intel_spi_isr(INOUT intel_instance_t *inst);
/*!
 * \}
 */

#ifdef __cplusplus
}
#endif

#endif /* _INTEL_HAL_SPI_H_ */

#include "../../drivers/spi/dw_spi.h"
