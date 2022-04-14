/*
 * Copyright (c) 2021 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _SEDI_DRIVER_I2S_H_
#define _SEDI_DRIVER_I2S_H_

#include <sedi.h>
#include <string.h>
#include "sedi_driver_common.h"
#include "sedi_driver_dma.h"

/**
 * @name I2S
 * @{
 */

#define I2S_DEBUG (0)
#define I2S_DEBUG_TRACE_EN (0)
#define I2S_ISR_DEBUG_TRACE_EN (0)
#define LEADING_DUMMY_BIT (0)
#define LEADING_DUMMY_BIT_1 (1)
#define SEDI_MAX_I2S_INSTANCE (2)
#define SEDI_I2S_API_VERSION (1)

#define I2S0_TX_FIFO_ADDRESS (0x48300040)
#define I2S0_RX_FIFO_ADDRESS (0x48300040)
#define I2S1_TX_FIFO_ADDRESS (0x48302040)
#define I2S1_RX_FIFO_ADDRESS (0x48302040)

#define INTR_STAT_FIFO_EMPTY BIT(2)
#define INTR_STAT_FIFO_AEMPTY BIT(3)
#define INTR_STAT_FIFO_FULL BIT(4)
#define INTR_STAT_FIFO_AFULL BIT(5)
#define INTR_STAT_RFIFO_EMPTY BIT(16)
#define INTR_STAT_RFIFO_AEMPTY BIT(17)
#define INTR_STAT_RFIFO_FULL BIT(18)
#define INTR_STAT_RFIFO_AFULL BIT(19)

/** @} */

typedef enum {
	I2S_BITS_PER_SAMPLE_12BIT       = 12,   /*!< I2S bits per sample: 12-bits*/
	I2S_BITS_PER_SAMPLE_16BIT       = 16,   /*!< I2S bits per sample: 16-bits*/
	I2S_BITS_PER_SAMPLE_24BIT       = 24,   /*!< I2S bits per sample: 24-bits*/
	I2S_BITS_PER_SAMPLE_32BIT       = 32,   /*!< I2S bits per sample: 32-bits*/
} i2s_bits_per_sample_t;

/**
 * @brief I2S communication standard format
 *
 */
typedef enum {
	I2S_COMM_FORMAT_I2S_MSB = 0x00, /*!< I2S format MSB*/
	I2S_COMM_FORMAT_I2S_LSB,        /*!< I2S format LSB*/
} i2s_comm_format_t;
/**
 * @brief I2S channel.
 *
 */
typedef enum {
	I2S_MONO_CHANNEL_LEFT   = 0,    /*!< I2S 1 channel (mono)*/
	I2S_MONO_CHANNEL_RIGHT,         /*!< I2S 1 channel (mono)*/
	I2S_STEREO_CAHNNEL,             /*!< I2S 2 channel (stereo)*/
	I2S_6_CHANNEL           = 6,    /*!< I2S  channel (TDM)*/
	I2S_8_CHANNEL           = 8,    /*!< I2S 8 channel (TDM)*/
} i2s_channel_t;

/**
 * @brief I2S channel format type
 */
typedef enum {
	I2S_CHANNEL_FMT_LEFT = 1,
	I2S_CHANNEL_FMT_RIGHT,
	I2S_CHANNEL_FMT_STD,
	I2S_CHANNEL_FMT_TDM,
} i2s_channel_fmt_t;

/**
 * @brief I2S Polarity
 */
typedef enum {
	I2S_CLK_NF_NB   = 1,
	I2S_CLK_NF_IB   = 2,
	I2S_CLK_IF_NB   = 3,
	I2S_CLK_IF_IB   = 4,
} i2s_polarity_t;

/**
 * @brief I2S Mode
 */
typedef enum {
	I2S_MASTER_TX           = 1,
	I2S_MASTER_RX           = 2,
	I2S_SLAVE_TX            = 3,
	I2S_SLAVE_RX            = 4,
	I2S_MASTER_FULL_DUPLEX  = 5,
	I2S_SLAVE_FULL_DUPLEX   = 6,
} i2s_mode_t;

/**
 * @brief I2S transfer direction
 */
typedef enum {
	I2S_TX = 0x00,
	I2S_RX,
} i2s_dir_t;

/**
 * @brief I2S FIFO Status
 */
typedef enum {
	I2S_FIFO_EMPTY = 0,     /*!< I2S FIFO empty interrupt*/
	I2S_FIFO_AEMPTY,        /*!< I2S FIFO almost empty interrupt*/
	I2S_FIFO_FULL,          /*!< I2S FIFO full interrupt*/
	I2S_FIFO_AFULL,         /*!< I2S FIFO almost full interrupt*/
	I2S_RFIFO_EMPTY,        /*!< I2S receive FIFO empty interrupt*/
	I2S_RFIFO_AEMPTY,       /*!< I2S receive FIFO almost empty interrupt*/
	I2S_RFIFO_FULL,         /*!< I2S receive FIFO full interrupt*/
	I2S_RFIFO_AFULL,        /*!< I2S receive FIFO almost full interrupt*/
} i2s_interrupt;

/**
 * @brief I2S TX/RX active Channel
 */
typedef union {
	struct {
		char ch0 : 1;
		char ch1 : 1;
		char ch2 : 1;
		char ch3 : 1;
		char ch4 : 1;
		char ch5 : 1;
		char ch6 : 1;
		char ch7 : 1;
	};
	char bits;
} i2s_tdm_active_ch;

/**
 * @brief I2S TX active Channel in Full duplex mode
 */
typedef union {
	struct {
		char ch0 : 1;
		char ch1 : 1;
		char ch2 : 1;
		char ch3 : 1;
		char ch4 : 1;
		char ch5 : 1;
		char ch6 : 1;
		char ch7 : 1;
	};
	char bits;
} i2s_tdm_full_dx_tx;

/**
 * @brief I2S RX active Channel in Full duplex mode
 */
typedef union {
	struct {
		char ch0 : 1;
		char ch1 : 1;
		char ch2 : 1;
		char ch3 : 1;
		char ch4 : 1;
		char ch5 : 1;
		char ch6 : 1;
		char ch7 : 1;
	};
	char bits;
} i2s_tdm_full_dx_rx;

/**
 * @brief I2S configuration parameters
 *
 */

typedef struct {
	uint32_t mode : 3;                      /*!< I2S work mode*/
	uint32_t sample_rate : 18;              /*!< I2S sample rate*/
	uint32_t polariy : 3;                   /*!< I2S Signal polarity*/
	uint32_t msb_first : 1;                 /*!< I2S MSB/LSB config*/
	i2s_bits_per_sample_t bits;             /*!< I2S bits per sample for tx/rx*/
	i2s_bits_per_sample_t rx_bits;          /*!< I2S bits per sample for rx in full
						 *!< duplex mode
						 */
	i2s_channel_fmt_t channel_format;       /*!< I2S channel format */
	i2s_channel_t channel;                  /*!< I2S channels mono, stereo or TDM*/
	i2s_tdm_active_ch tdm_ch;               /*!< I2S TDM active channels*/
	i2s_tdm_full_dx_tx tdm_tx_ch;           /*!< I2S TDM active transmit channels*/
	i2s_tdm_full_dx_rx tdm_rx_ch;           /*!< I2S TDM active receive channels*/
	uint8_t tx_dn;                          /*!< I2S transmit done*/
	uint8_t rx_dn;                          /*!< I2S receive done*/
	uint8_t mn_enable;                      /*!< I2S MN Div reg enable*/
	uint32_t m_val;                         /*!< I2S MN Div reg, M value*/
	uint32_t n_val;                         /*!< I2S MN Div reg, N value*/
} i2s_config_t;

/**
 * @brief I2S event types
 *
 */
typedef enum {
	I2S_EVENT_DMA_ERROR,
	I2S_EVENT_TX_DONE,      /*!< I2S DMA finish sent 1 buffer*/
	I2S_EVENT_RX_DONE,      /*!< I2S DMA finish received 1 buffer*/
	I2S_EVENT_MAX,          /*!< I2S event max index*/
} i2s_event_type_t;

/**
 * @brief Event structure used in I2S event queue
 *
 */
typedef struct {
	i2s_event_type_t type;  /*!< I2S event type */
	uint32_t size;          /*!< I2S data size for I2S_DATA event*/
} i2s_event_t;

/**
 * @brief DMA configuration parameters
 *
 */
typedef struct {
	sedi_dma_t dma_dev;
	int32_t channel;
	uint8_t *data;
	void (*dma_callback)(sedi_dma_t dma_device, int channel_id, int event,
			     void *ctx);
	void *cb_param;
	uint32_t source_data_size;              /*!< width of source data (in bytes)*/
	uint32_t dest_data_size;                /*!< destination data width*/
	uint32_t source_burst_length;           /*!< number of source data units */
	uint32_t dest_burst_length;             /*!< number of destination data units */
	uint32_t complete_callback_en : 1;      /*!< Reserved field for future use */
	uint32_t error_callback_en : 1;         /*!< Reserved field for future use */
	uint32_t block_count : 16;              /*!< Reserved field for future use */
	uint32_t channel_direction : 2;         /*!< Channel direction */
	uint32_t dev_hwid : 12;                 /*!< DMA HW ID for I2S */
	uint32_t dev_hwid_dir;                  /*!< DMA_HS_PER_TX Or DMA_HS_PER_RX*/
} sedi_i2s_dma_config_t;

/**
 * @brief DMA Address parameters
 *
 */
typedef struct {
	uint32_t channel;               /*!< DMA Channel */
	uint32_t source_address;        /*!< Source Address */
	uint32_t dest_address;          /*!< Destination Address */
	uint32_t block_size;            /*!< Block Size in byte */
} sedi_i2s_dma_add_t;

/**
 * @brief Configure I2S.
 *
 * @param[in] I2S identifier.
 * @param i2s_config I2S configurations - see i2s_config_t struct
 *
 * @return
 *     - SEDI_DRIVER_OK   Success
 *     - SEDI_DRIVER_ERROR_PARAMETER Parameter error
 */
int sedi_i2s_init(IN sedi_i2s_t i2s_num, IN i2s_config_t *i2s_config);

/**
 * I2S Read in Polling mode.
 *
 * @brief Perform a poll read and is a blocking synchronous call.
 *
 * @param[in] I2S identifier.
 * @param[in] data Data to read from I2S.
 * @param[in] Size of data Data to read from I2S.
 *
 * @return Standard driver return type for SEDI.
 * @retval SEDI_DRIVER_OK  on success.
 * @retval Negative @ref errno for possible error codes.
 */
int sedi_i2s_poll_read(IN sedi_i2s_t num, INOUT uint32_t *dst,
		       IN uint32_t size);
/**
 * I2S Write in Polling mode.
 *
 * @brief Perform a poll write and is a blocking synchronous call.
 *
 * @param[in] I2S identifier.
 * @param[in] data Data to be written into I2S.
 * @param[in] Size of data Data to be write to I2S.
 *
 * @return Standard driver return type for SEDI.
 * @retval SEDI_DRIVER_OK  on success.
 * @retval Negative @ref errno for possible error codes.
 */
int sedi_i2s_poll_write(IN sedi_i2s_t num, IN uint32_t *src, IN uint32_t size);
/**
 * Enable I2S transceiver.
 *
 * @brief Perform Enabling I2S transceiver .
 *
 * @param[in] I2S identifier.
 *
 * @return Standard driver return type for SEDI.
 * @retval SEDI_DRIVER_OK  on success.
 * @retval Negative @ref errno for possible error codes.
 */
int sedi_enable_transceiver(IN sedi_i2s_t num);

/**
 * Disable I2S transceiver.
 *
 * @brief Perform Disabling I2S transceiver .
 *
 * @param[in] I2S identifier.
 *
 * @return Standard driver return type for SEDI.
 * @retval SEDI_DRIVER_OK  on success.
 * @retval Negative @ref errno for possible error codes.
 */
int sedi_disable_transceiver(IN sedi_i2s_t num);

/**
 * Get M/N divider register status for I2S.
 *
 * @brief Perform Enabling M/N divider clock for I2S transceiver.
 *
 * @param[in] I2S identifier.
 *
 * @return 1 if mn divider is enabled.
 * @retval 0 if mn divider is not enabled.
 */
int sedi_get_mn_divide(IN sedi_i2s_t num);

/**
 * Set M/N divider clock for I2S.
 *
 * @brief Perform Enabling M/N divider clock for I2S transceiver.
 *
 * @param[in] I2S identifier.
 *
 * @param[in] M divider value to be set.
 *
 * @param[in] N divider value to be set.
 *
 * @param[in] enable MN divider.
 *
 * @return Standard driver return type for SEDI.
 * @retval SEDI_DRIVER_OK  on success.
 * @retval Negative @ref errno for possible error codes.
 */
int sedi_set_mn_divide(IN sedi_i2s_t num, IN uint32_t mdiv_val,
		       IN uint32_t ndiv_val, IN uint8_t enable);

/**
 * Enable I2S Interrupt.
 *
 * @brief Perform Enabling I2S Interrupt .
 *
 * @param[in] I2S identifier.
 *
 * @return Standard driver return type for SEDI.
 * @retval SEDI_DRIVER_OK  on success.
 * @retval Negative @ref errno for possible error codes.
 */
int sedi_i2s_enable_interrupt(IN sedi_i2s_t num);

/**
 * Disable I2S Interrupt.
 *
 * @brief Perform Disabling I2S Interrupt .
 *
 * @param[in] I2S identifier.
 *
 * @return Standard driver return type for SEDI.
 * @retval SEDI_DRIVER_OK  on success.
 * @retval Negative @ref errno for possible error codes.
 */
int sedi_i2s_disable_interrupt(IN sedi_i2s_t num);

/**
 * Clear all I2S Interrupt.
 *
 * @brief Perform Clearing I2S Interrupt .
 *
 * @param[in] I2S identifier.
 *
 * @return Standard driver return type for SEDI.
 * @retval SEDI_DRIVER_OK  on success.
 * @retval Negative @ref errno for possible error codes.
 */
int sedi_interrupt_clear(IN sedi_i2s_t num);

/**
 * Get Interrupt Type.
 *
 * @brief This function will return interrupt type currently occurred.
 *
 * @param[in] I2S identifier.
 *
 * @return Interrupt Type.
 */
i2s_interrupt sedi_get_interrupt_type(IN sedi_i2s_t num);

/**
 * Reset RX FIFO in case of over flow.
 *
 * @brief This function will reset overflow condition so that RX will restart.
 *
 * @param[in] I2S identifier.
 *
 * @return NIL.
 */
int sedi_i2s_reset_rx_fifo(IN uint32_t num);

/**
 * Reset TX FIFO.
 *
 * @brief This function will reset TX will FIFO.
 *
 * @param[in] I2S identifier.
 *
 * @return NIL.
 */
int sedi_i2s_reset_tx_fifo(IN uint32_t num);

/**
 * Configure DMA Transfer.
 *
 * @brief Configure DMA before a each Transfer.
 *
 * @param[in] I2S identifier.
 * @param[in] DMA direction.
 * @param[in] DMA config info.
 *
 * @return Standard driver return type for SEDI.
 * @retval SEDI_DRIVER_OK  on success.
 * @retval Negative @ref errno for possible error codes.
 */

int sedi_i2s_config_dma(IN sedi_dma_t dma, IN uint32_t dir,
			IN sedi_i2s_dma_config_t *cfg);

/**
 * Start DMA Transfer.
 *
 * @brief initiate DMA Transfer.
 *
 * @param[in] I2S identifier.
 * @param[in] DMA channel and Address info.
 *
 * @return Standard driver return type for SEDI.
 * @retval SEDI_DRIVER_OK  on success.
 * @retval Negative @ref errno for possible error codes.
 */
int sedi_i2s_dma_start(IN sedi_dma_t dma, IN sedi_i2s_dma_add_t *dma_add);

/**
 * Stop DMA Transfer.
 *
 * @brief This function will Stop DMA Transfer..
 *
 * @param[in] I2S identifier.
 * @param[in] DMA channel number.
 *
 * @return NIL.
 */
int sedi_i2s_dma_stop(IN sedi_dma_t dma, IN uint32_t channel);

/**
 * Get I2S driver version number
 *
 * @brief This function will return current I2S Version number.
 *
 * @param[in] NIL.
 *
 * @return return I2S driver version number.
 */
sedi_driver_version_t sedi_i2s_get_version(void);

/**
 * @brief I2S set power state.
 *
 * Set I2S instance to specified power state
 *
 * @param[in] power state to be entered.
 *
 * @retval SEDI_DRIVER_OK if operation was successful.
 * @retval non zero error code otherwise.
 */

int32_t sedi_i2s_set_power(IN sedi_i2s_t uart, IN sedi_power_state_t state);

#endif
