/*
 * Copyright (c) 2021 Intel Corporation
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "spi_dw_apb.h"
#include "pm/pm_internal_if.h"

static const sedi_driver_version_t driver_version = {SEDI_SPI_API_VERSION,
						     SEDI_SPI_DRV_VERSION};

static sedi_spi_capabilities_t driver_capabilities[SEDI_SPI_NUM] = {0};

static const driver_id_t spi_pm_id[SEDI_SPI_NUM] = {
    DRIVER_ID_SPI0, DRIVER_ID_SPI1, DRIVER_ID_SPI2, DRIVER_ID_SPI3};

#define SPI_CONTEXT_INIT(x)                                                    \
	{                                                                      \
		.base = (spi_reg_t *)SEDI_SPI_##x##_REG_BASE,                  \
		.dma_handshake = DMA_HWID_SPI##x, .dummy_data = 0x00           \
	}

static struct spi_context contexts[SEDI_SPI_NUM] = {
    SPI_CONTEXT_INIT(0), SPI_CONTEXT_INIT(1), SPI_CONTEXT_INIT(2),
    SPI_CONTEXT_INIT(3)};

static inline void lld_spi_enable(spi_reg_t *spi, bool enable)
{
	uint32_t val = enable ? 1 : 0;

	if (spi->ssienr == val) {
		return;
	}
	/* prevent pending interrupt */
	spi->imr = 0;

	spi->ssienr = val;
	while (spi->ssienr != val)
		;
}

static inline void lld_spi_dma_enable(spi_reg_t *spi, bool enable)
{
	spi->dmacr = enable ? REG_DMA_ENABLE : 0;
}

static inline void lld_spi_config_interrupt(spi_reg_t *spi, uint32_t mask)
{
	spi->imr = mask;
}

static inline bool lld_spi_is_busy(spi_reg_t *spi)
{
	return ((spi->sr & REG_SR_BUSY) || (!(spi->sr & REG_SR_TX_EMPTY)))
		   ? true
		   : false;
}

static inline bool lld_spi_is_enabled(spi_reg_t *spi)
{
	return spi->ssienr ? true : false;
}

static inline uint32_t lld_spi_interrupt_clear(spi_reg_t *spi)
{
	uint32_t tmp;
	uint32_t isr;

	isr = spi->isr;
	tmp = spi->icr;

	/* Clear all error interrupt by read*/
	tmp = spi->txoicr;
	tmp = spi->rxoicr;
	tmp = spi->rxuicr;

	return isr;
}

static int lld_spi_default_config(sedi_spi_t spi_device)
{
	struct spi_context *context = &contexts[spi_device];
	spi_reg_t *spi = context->base;

	uint32_t ctrl0;
	uint32_t watermark = SPI_FIFO_DEPTH / 2 - 1;

	uint8_t loopback = false;
	uint8_t width = SPI_BITWIDTH_8BITS;
	uint32_t prescale = DEFAULT_PRESCALE;
	uint8_t cs_mask = 0x1;

	ctrl0 = width - 1;
	ctrl0 |= loopback ? REG_CTRL0_LOOPBACK : 0;
	ctrl0 |= (width - 1) << OFFSET_CTRL0_WIDTH;

	/* Set default SPI watermark */
	spi->txftlr = watermark;
	spi->rxftlr = watermark;
	spi->dmatdlr = watermark;
	spi->dmardlr = watermark;

	spi->ctrl0 = ctrl0;
	spi->baudr = prescale;
	spi->ser = cs_mask;

	/* Update context default settings */
	context->tx_watermark = watermark + 1U;
	context->rx_watermark = watermark + 1U;
	context->prescale = DEFAULT_PRESCALE;
	context->frame_size = 1U;
	context->is_lsb = false;

	return 0;
}

static inline void lld_spi_config_cpol_cpha(spi_reg_t *spi, int cpol, int cpha)
{
	spi->ctrl0 &= ~(REG_CTRL0_CPHA | REG_CTRL0_CPOL);
	spi->ctrl0 |= cpha ? REG_CTRL0_CPHA : 0;
	spi->ctrl0 |= cpol ? REG_CTRL0_CPOL : 0;
}

static inline void lld_spi_config_loopback(spi_reg_t *spi, int loopback)
{
	spi->ctrl0 &= ~REG_CTRL0_LOOPBACK;
	spi->ctrl0 |= loopback ? REG_CTRL0_LOOPBACK : 0;
}

static inline void lld_spi_config_prescale(spi_reg_t *spi, uint32_t prescale)
{
	spi->baudr = prescale;
}

static inline void lld_spi_config_width(spi_reg_t *spi, uint8_t width)
{
	spi->ctrl0 &= ~MASK_CTRL0_WIDTH;
	spi->ctrl0 |= (width - 1) << OFFSET_CTRL0_WIDTH;
}

static inline void lld_spi_set_tx_watermark(spi_reg_t *spi, uint32_t watermark)
{
	spi->txftlr = watermark - 1;
}

static inline void lld_spi_set_rx_watermark(spi_reg_t *spi, uint32_t watermark)
{
	spi->rxftlr = watermark - 1;
}

static inline void lld_spi_config_cs(spi_reg_t *spi, uint32_t cs_mask)
{
	spi->ser = cs_mask;
}

static inline void lld_spi_set_rx_delay(spi_reg_t *spi, uint8_t delay)
{
	spi->rx_sample_delay = delay;
}

static void lld_spi_set_transfer_mode(sedi_spi_t spi_device,
				      IN uint8_t *data_out,
				      OUT uint8_t *data_in)
{
	struct spi_context *context = &contexts[spi_device];
	spi_reg_t *spi = context->base;
	uint32_t ctrl0 = spi->ctrl0;

	ctrl0 &= ~SPI_CTRL0_TMOD_MASK;
	if (data_out == NULL) {
		/* Set to receive only mode */
		ctrl0 |= SPI_CTRL0_RECEIVE_MODE;
		context->transfer_mode = SPI_TRANSFER_MODE_RECEIVE;
	} else if (data_in == NULL) {
		/* Set to receive only mode */
		ctrl0 |= SPI_CTRL0_SEND_MODE;
		context->transfer_mode = SPI_TRANSFER_MODE_SEND;
	} else {
		ctrl0 |= SPI_CTRL0_BOTH_MODE;
		context->transfer_mode = SPI_TRANSFER_MODE_BOTH;
	}

	spi->ctrl0 = ctrl0;
}

static int lld_spi_fill_fifo(sedi_spi_t spi_device, uint8_t frame_size,
			     IN uint8_t *buff, uint32_t count)
{
	struct spi_context *context = &contexts[spi_device];
	spi_reg_t *spi = context->base;
	uint32_t size = (SPI_FIFO_DEPTH - spi->txflr) * frame_size;
	uint32_t data = 0;

	/* Get the number which can be filled to fifo */
	size = (count > size) ? size : count;
	/* Used to return the actual fill size in bytes */
	count = size;
	while (size) {
		/* Get the data in a FIFO entry */
		if (buff) {
			switch (frame_size) {
			case SPI_FRAME_SIZE_1_BYTE:
				data = (uint32_t)(*buff);
				break;
			case SPI_FRAME_SIZE_2_BYTES:
				data = (uint32_t)(*(uint16_t *)buff);
				break;
			default:
				break;
			}
			/* Update size */
			buff += frame_size;
		} else {
			data = context->dummy_data;
		}
		/* Write data */
		spi->dr = data;
		size -= frame_size;
	}

	return count;
}

static int lld_spi_receive_fifo(spi_reg_t *spi, uint8_t frame_size,
				OUT uint8_t *buff, uint32_t count)
{
	uint32_t size = spi->rxflr * frame_size;
	uint32_t data;

	/* Get the number which can be filled to fifo */
	size = (count > size) ? size : count;
	/* Used to return the actual fill size in bytes */
	count = size;
	while (size) {
		/* Get the data in a FIFO entry */
		data = spi->dr;
		if (buff) {
			switch (frame_size) {
			case SPI_FRAME_SIZE_1_BYTE:
				*buff = (data & 0xFF);
				break;
			case SPI_FRAME_SIZE_2_BYTES:
				buff[0] = (data & 0xFF);
				buff[1] = ((data >> 8U) & 0xFF);
				break;
			default:
				break;
			}
			/* Update size and buff */
			buff += frame_size;
		}
		size -= frame_size;
	}

	return count;
}

static inline uint32_t lld_spi_dr_address(spi_reg_t *spi)
{
	return (uint32_t)&spi->dr;
}

static void spi_bit_reverse(uint8_t *val, uint32_t len, uint8_t frame_size)
{
	if (frame_size == SPI_FRAME_SIZE_1_BYTE) {
		msb_lsb_convert_8bits(val, len);
	} else {
		msb_lsb_convert_16bits((uint16_t *)val, len / frame_size);
	}
}

static void lld_spi_set_ti_mode(spi_reg_t *spi)
{
	if (lld_spi_is_enabled(spi) == true) {
		lld_spi_enable(spi, false);
	}
	spi->ctrl0 &= ~REG_CTRL0_FRF_MASK;
	spi->ctrl0 |= REG_CTRL0_FRF_TI_SSP;
}

static void lld_spi_set_microwire_mode(spi_reg_t *spi,
				       sedi_spi_microwire_config_t *config)
{
	uint32_t mwcr;

	if (lld_spi_is_enabled(spi) == true) {
		lld_spi_enable(spi, false);
	}
	spi->ctrl0 &= ~REG_CTRL0_FRF_MASK;
	spi->ctrl0 |= REG_CTRL0_FRF_MICROWIRE;

	/* Configure microwire mode */
	mwcr = ((config->microwire_handshake << REG_MWCR_MHS_SHIFT) |
		(config->data_direction_tx << REG_MWCR_MDD_SHIFT) |
		(config->sequential_mode << REG_MWCR_MWMOD_SHIFT));
}

/******************************************************************************
 * SEDI interface
 *****************************************************************************/

sedi_driver_version_t sedi_spi_get_version(void)
{
	return driver_version;
}

int32_t sedi_spi_get_capabilities(IN sedi_spi_t spi_device,
				  sedi_spi_capabilities_t *cap)
{
	DBG_CHECK(spi_device < SEDI_SPI_NUM, SEDI_DRIVER_ERROR_PARAMETER);

	if (DEV_PSE_OWNED ==
	    sedi_get_dev_ownership(PSE_DEV_SPI0 + spi_device)) {
		driver_capabilities[spi_device].is_available = 1;
	} else {
		driver_capabilities[spi_device].is_available = 0;
	}

	driver_capabilities[spi_device].ti_ssi = 1;
	driver_capabilities[spi_device].microwire = 1;
	driver_capabilities[spi_device].mode_fault = 0;

	*cap = driver_capabilities[spi_device];

	return SEDI_DRIVER_OK;
}

int32_t sedi_spi_init(IN sedi_spi_t spi_device, IN sedi_spi_event_cb_t cb_event,
		      INOUT void *param)
{
	DBG_CHECK(spi_device < SEDI_SPI_NUM, SEDI_DRIVER_ERROR_PARAMETER);

	struct spi_context *context = &contexts[spi_device];

	if (DEV_PSE_OWNED !=
	    sedi_get_dev_ownership(PSE_DEV_SPI0 + spi_device)) {
		return SEDI_DRIVER_ERROR_NO_DEV;
	}

	context->cb_event = cb_event;
	context->cb_param = param;

	return SEDI_DRIVER_OK;
}

int32_t sedi_spi_uninit(IN sedi_spi_t spi_device)
{
	DBG_CHECK(spi_device < SEDI_SPI_NUM, SEDI_DRIVER_ERROR_PARAMETER);

	struct spi_context *context = &contexts[spi_device];

	context->cb_event = NULL;
	context->is_lsb = false;

	return SEDI_DRIVER_OK;
}

int32_t sedi_spi_set_power(IN sedi_spi_t spi_device,
			   IN sedi_power_state_t state)
{
	DBG_CHECK(spi_device < SEDI_SPI_NUM, SEDI_DRIVER_ERROR_PARAMETER);
	driver_id_t id = spi_pm_id[spi_device];
	struct spi_context *context = &contexts[spi_device];
	int32_t ret = SEDI_DRIVER_OK;

	switch (state) {
	case SEDI_POWER_FULL:
		pm_driver_start_trans(id);
		lld_spi_default_config(spi_device);
		break;
	case SEDI_POWER_SUSPEND:
		lld_spi_enable(context->base, false);
		pm_driver_end_trans(id);
		break;
	case SEDI_POWER_FORCE_SUSPEND:
		lld_spi_enable(context->base, false);
		pm_driver_end_trans(id);
		break;
	case SEDI_POWER_LOW:
		/* While in low power and shut down, clock gate */
		lld_spi_enable(context->base, false);
		pm_driver_end_trans(id);
		break;
	case SEDI_POWER_OFF:
		ret = SEDI_DRIVER_ERROR_UNSUPPORTED;
		break;
	}

	return ret;
}

int32_t sedi_spi_get_data_count(IN sedi_spi_t spi_device)
{
	DBG_CHECK(spi_device < SEDI_SPI_NUM, SEDI_DRIVER_ERROR_PARAMETER);

	struct spi_context *context = &contexts[spi_device];

	if (context->data_tx) {
		return context->data_tx_idx;
	} else {
		return context->data_rx_idx;
	}
}

int32_t sedi_spi_get_status(IN sedi_spi_t spi_device, sedi_spi_status_t *status)
{
	DBG_CHECK(spi_device < SEDI_SPI_NUM, SEDI_DRIVER_ERROR_PARAMETER);
	DBG_CHECK(NULL != status, SEDI_DRIVER_ERROR_PARAMETER);

	struct spi_context *context = &contexts[spi_device];

	status->busy = context->status.busy;
	status->data_lost = context->status.data_lost;
	status->mode_fault = context->status.mode_fault;

	return SEDI_DRIVER_OK;
}

int32_t sedi_spi_control(IN sedi_spi_t spi_device, IN uint32_t control,
			 IN uint32_t arg)
{
	DBG_CHECK(spi_device < SEDI_SPI_NUM, SEDI_DRIVER_ERROR_PARAMETER);
	DBG_CHECK(control < SEDI_SPI_IOCTL_NUM, SEDI_DRIVER_ERROR_PARAMETER);

	struct spi_context *context = &contexts[spi_device];

	switch (control) {
	case SEDI_SPI_IOCTL_CPOL0_CPHA0:
		lld_spi_config_cpol_cpha(context->base, 0, 0);
		break;
	case SEDI_SPI_IOCTL_CPOL0_CPHA1:
		lld_spi_config_cpol_cpha(context->base, 0, 1);
		break;
	case SEDI_SPI_IOCTL_CPOL1_CPHA0:
		lld_spi_config_cpol_cpha(context->base, 1, 0);
		break;
	case SEDI_SPI_IOCTL_CPOL1_CPHA1:
		lld_spi_config_cpol_cpha(context->base, 1, 1);
		break;
	case SEDI_SPI_IOCTL_TI_SSI:
		lld_spi_set_ti_mode(context->base);
		break;
	case SEDI_SPI_IOCTL_MICROWIRE:
		lld_spi_set_microwire_mode(context->base,
					   (sedi_spi_microwire_config_t *)arg);
		break;
	case SEDI_SPI_IOCTL_MSB:
		context->is_lsb = false;
		break;
	case SEDI_SPI_IOCTL_LSB:
		context->is_lsb = true;
		break;
	case SEDI_SPI_IOCTL_DATA_WIDTH:
		DBG_CHECK(((arg == SPI_BITWIDTH_4BITS) ||
			   (arg == SPI_BITWIDTH_8BITS) ||
			   (arg == SPI_BITWIDTH_16BITS)),
			  SEDI_DRIVER_ERROR_PARAMETER);
		context->frame_size = (uint8_t)arg / 8U;
		/* For 4 bits operation, operate like 8 bits */
		if (context->frame_size == 0) {
			context->frame_size = SPI_FRAME_SIZE_1_BYTE;
		}
		lld_spi_config_width(context->base, (uint8_t)arg);
		break;
	case SEDI_SPI_IOCTL_CS_UNUSED:
		lld_spi_config_cs(context->base, 0U);
		break;
	case SEDI_SPI_IOCTL_CS_HW:
		lld_spi_config_cs(context->base, (uint32_t)arg);
		break;
	case SEDI_SPI_IOCTL_SPEED_SET:
		context->prescale = SSI_IC_FREQ / (uint32_t)arg;
		if (context->prescale < SSI_PRESCALE_MIN) {
			context->prescale = SSI_PRESCALE_MIN;
		}
		/* Divider can only support even number */
		context->prescale &= 0xFFFFFFFE;
		lld_spi_config_prescale(context->base, context->prescale);
		break;
	case SEDI_SPI_IOCTL_TX_WATERMARK_SET:
		context->tx_watermark = (uint32_t)arg;
		lld_spi_set_tx_watermark(context->base, (uint32_t)arg);
		break;
	case SEDI_SPI_IOCTL_RX_WATERMARK_SET:
		context->rx_watermark = (uint32_t)arg;
		lld_spi_set_rx_watermark(context->base, (uint32_t)arg);
		break;
	case SEDI_SPI_IOCTL_DUMMY_DATA:
		context->dummy_data = (uint32_t)arg;
		break;
	case SEDI_SPI_IOCTL_LOOPBACK:
		lld_spi_config_loopback(context->base, (bool)arg);
		break;
	case SEDI_SPI_IOCTL_SPEED_GET:
		*((uint32_t *)arg) = SSI_IC_FREQ / context->prescale;
		break;
	case SEDI_SPI_IOCTL_TX_WATERMARK_GET:
		*((uint32_t *)arg) = context->tx_watermark;
		break;
	case SEDI_SPI_IOCTL_RX_WATERMARK_GET:
		*((uint32_t *)arg) = context->rx_watermark;
		break;
	case SEDI_SPI_IOCTL_ABORT:
		lld_spi_enable(context->base, false);
		context->status.busy = 0;
		break;
	case SEDI_SPI_IOCTL_BUFFER_SETS:
		context->is_cs_continuous = (bool)arg;
		break;
	case SEDI_SPI_IOCTL_SET_RX_DELAY:
		lld_spi_set_rx_delay(context->base, (uint8_t)arg);
		break;
	default:
		break;
	}

	return SEDI_DRIVER_OK;
}

static void callback_dma_transfer(const sedi_dma_t dma, const int chan,
				  const int event, void *param)
{
	sedi_spi_t spi_device = (sedi_spi_t)param;

	struct spi_context *context = &contexts[spi_device];

	/* release the dma resource */
	sedi_dma_set_power(dma, chan, SEDI_POWER_OFF);
	sedi_dma_uninit(dma, chan);

	/* See tx or rx finished */
	if (chan == context->tx_channel) {
		context->dma_tx_finished = true;
		context->data_tx_idx = context->tx_data_len;
		/* Recover LSB reverse, DMA mode tx buff pointer not changed */
		if (context->is_lsb == true) {
			spi_bit_reverse(context->data_tx, context->tx_data_len,
					context->frame_size);
			sedi_core_clean_dcache_by_addr(
			    (uint32_t *)(context->data_tx),
			    context->tx_data_len);
		}
		/* Waiting for TX FIFO empty */
		while (lld_spi_is_busy(context->base))
			;
	} else if (chan == context->rx_channel) {
		context->dma_rx_finished = true;
		context->data_rx_idx = context->rx_data_len;
		/* If finished Rx, and need to do bit convert */
		if (context->is_lsb == true) {
			/* Invalidate cache */
			sedi_core_inv_dcache_by_addr(
			    (uint32_t *)(context->data_rx),
			    context->rx_data_len);
			spi_bit_reverse(context->data_rx, context->rx_data_len,
					context->frame_size);
			sedi_core_clean_dcache_by_addr(
			    (uint32_t *)(context->data_rx),
			    context->rx_data_len);
		}
	}

	/* All tx and rx finished */
	if ((context->dma_tx_finished == true) &&
	    (context->dma_rx_finished == true)) {
		/* reset busy bit to enable user call async transfer in callback
		 */
		context->status.busy = 0;
		lld_spi_dma_enable(context->base, false);
		lld_spi_config_interrupt(context->base, REG_INT_NONE);
		lld_spi_enable(context->base, false);

		if (context->cb_event) {
			if (SEDI_DMA_EVENT_TRANSFER_DONE == event) {
				context->cb_event(SEDI_SPI_EVENT_COMPLETE,
						  context->cb_param);
			} else {
				context->cb_event(SEDI_SPI_EVENT_DATA_LOST,
						  context->cb_param);
			}
		}
	}
}

static int config_and_enable_dma_channel(sedi_spi_t spi_dev, int dma,
					 int handshake, int chan, int width,
					 int burst, uint32_t src, uint32_t dst,
					 uint32_t len, int is_tx)
{
	int ret;
	int dma_dir;
	int dma_per_dir;
	dma_transfer_width_t wid = DMA_TRANS_WIDTH_8;

	PARAM_UNUSED(
	    burst); /* Set burst to 1 to finish transfer all data size */

	if (is_tx) {
		dma_dir = DMA_MEMORY_TO_PERIPHERAL;
		dma_per_dir = DMA_HS_PER_TX;
	} else {
		dma_dir = DMA_PERIPHERAL_TO_MEMORY;
		dma_per_dir = DMA_HS_PER_RX;
	}

	switch (width) {
	case 1:
		wid = DMA_TRANS_WIDTH_8;
		break;
	case 2:
		wid = DMA_TRANS_WIDTH_16;
		break;
	case 4:
		wid = DMA_TRANS_WIDTH_32;
		break;
	default:
		break;
	}

	ret = sedi_dma_init(dma, chan, callback_dma_transfer, (void *)spi_dev);
	DBG_CHECK(0 == ret, SEDI_DRIVER_ERROR);

	ret = sedi_dma_set_power(dma, chan, SEDI_POWER_FULL);
	DBG_CHECK(0 == ret, SEDI_DRIVER_ERROR);

	ret = sedi_dma_control(dma, chan, SEDI_CONFIG_DMA_BURST_LENGTH,
			       DMA_BURST_TRANS_LENGTH_1);
	DBG_CHECK(0 == ret, SEDI_DRIVER_ERROR);

	ret = sedi_dma_control(dma, chan, SEDI_CONFIG_DMA_SR_TRANS_WIDTH, wid);
	DBG_CHECK(0 == ret, SEDI_DRIVER_ERROR);

	ret = sedi_dma_control(dma, chan, SEDI_CONFIG_DMA_DT_TRANS_WIDTH, wid);
	DBG_CHECK(0 == ret, SEDI_DRIVER_ERROR);

	ret = sedi_dma_control(dma, chan, SEDI_CONFIG_DMA_HS_DEVICE_ID,
			       handshake);
	DBG_CHECK(0 == ret, SEDI_DRIVER_ERROR);

	ret = sedi_dma_control(dma, chan, SEDI_CONFIG_DMA_HS_POLARITY,
			       DMA_HS_POLARITY_HIGH);
	DBG_CHECK(0 == ret, SEDI_DRIVER_ERROR);

	ret = sedi_dma_control(dma, chan, SEDI_CONFIG_DMA_DIRECTION, dma_dir);
	DBG_CHECK(0 == ret, SEDI_DRIVER_ERROR);

	ret = sedi_dma_control(dma, chan, SEDI_CONFIG_DMA_HS_DEVICE_ID_PER_DIR,
			       dma_per_dir);
	DBG_CHECK(0 == ret, SEDI_DRIVER_ERROR);

	ret = sedi_dma_start_transfer(dma, chan, src, dst, len);
	DBG_CHECK(0 == ret, SEDI_DRIVER_ERROR);

	return 0;
}

int32_t sedi_spi_dma_transfer(IN sedi_spi_t spi_device, IN uint32_t tx_dma,
			      IN uint32_t tx_dma_chan, IN uint8_t *data_out,
			      IN uint32_t rx_dma, IN uint32_t rx_dma_chan,
			      OUT uint8_t *data_in, IN uint32_t num)
{
	DBG_CHECK(spi_device < SEDI_SPI_NUM, SEDI_DRIVER_ERROR_PARAMETER);

	struct spi_context *context = &contexts[spi_device];
	int handshake = context->dma_handshake;
	int width = context->frame_size;
	int burst = 1;

	DBG_CHECK(((num % context->frame_size) == 0),
		  SEDI_DRIVER_ERROR_PARAMETER);

	if (context->status.busy) {
		return SEDI_DRIVER_ERROR_BUSY;
	}

	context->status.busy = 1U;

	context->base->dmatdlr = SPI_FIFO_DEPTH - 1;
	context->base->dmardlr = 0;
	context->tx_channel = tx_dma_chan;
	context->rx_channel = rx_dma_chan;
	context->dma_tx_finished = false;
	context->dma_rx_finished = false;
	context->tx_data_len = num;
	context->rx_data_len = num;
	context->data_tx = (uint8_t *)data_out;
	context->data_rx = data_in;

	/* Decide the transfer mode, send ,receive or both */
	lld_spi_set_transfer_mode(spi_device, data_out, data_in);

	/* If need to bit reverse tx buffer */
	if ((context->transfer_mode != SPI_TRANSFER_MODE_RECEIVE) &&
	    (context->is_lsb == true)) {
		spi_bit_reverse(context->data_tx, context->tx_data_len,
				context->frame_size);
		/* Clean the cache for DMA transfer */
		sedi_core_clean_dcache_by_addr((uint32_t *)(context->data_tx),
					       context->tx_data_len);
	}

	/* According to different transfer mode, do different fill or receive */
	if (context->transfer_mode == SPI_TRANSFER_MODE_SEND) {
		/* start dma first */
		config_and_enable_dma_channel(
		    spi_device, tx_dma, handshake, tx_dma_chan, width, burst,
		    (uint32_t)data_out, lld_spi_dr_address(context->base), num,
		    true);
		context->dma_rx_finished = true;
		context->rx_channel = 0xFF;
	} else if (context->transfer_mode == SPI_TRANSFER_MODE_RECEIVE) {
		/* Send dummy data first */
		lld_spi_fill_fifo(spi_device, context->frame_size, NULL,
				  context->frame_size);
		/* Configure rx channel */
		config_and_enable_dma_channel(spi_device, rx_dma, handshake,
					      rx_dma_chan, width, burst,
					      lld_spi_dr_address(context->base),
					      (uint32_t)data_in, num, false);
		/* Set NDF bits for receive only mode */
		DBG_CHECK((num <= SPI_RECEIVE_MODE_MAX_SIZE),
			  SEDI_DRIVER_ERROR_PARAMETER);
		context->base->ctrl1 = num / context->frame_size - 1;
		context->dma_tx_finished = true;
		context->tx_channel = 0xFF;
	} else {
		/* Enable both channel to do transfer */
		config_and_enable_dma_channel(
		    spi_device, tx_dma, handshake, tx_dma_chan, width, burst,
		    (uint32_t)data_out, lld_spi_dr_address(context->base), num,
		    true);
		config_and_enable_dma_channel(spi_device, rx_dma, handshake,
					      rx_dma_chan, width, burst,
					      lld_spi_dr_address(context->base),
					      (uint32_t)data_in, num, false);
	}

	lld_spi_enable(context->base, true);

	lld_spi_dma_enable(context->base, true);

	/* dma will clear tx/rx event, and error event will be cleared here */
	lld_spi_config_interrupt(context->base, REG_INT_ERROR);

	return SEDI_DRIVER_OK;
}

int32_t sedi_spi_poll_transfer(IN sedi_spi_t spi_device, IN uint8_t *data_out,
			       OUT uint8_t *data_in, IN uint32_t num)
{
	DBG_CHECK(spi_device < SEDI_SPI_NUM, SEDI_DRIVER_ERROR_PARAMETER);

	uint32_t tx_num = num, rx_num = num, fill_num = 0, receive_num = 0;
	struct spi_context *context = &contexts[spi_device];

	DBG_CHECK(((num % context->frame_size) == 0),
		  SEDI_DRIVER_ERROR_PARAMETER);

	if (context->status.busy) {
		return SEDI_DRIVER_ERROR_BUSY;
	}

	context->status.busy = 1U;
	context->data_tx = (void *)data_out;
	context->data_rx = (void *)data_in;
	context->tx_data_len = num;
	context->rx_data_len = num;
	context->data_tx_idx = 0;
	context->data_rx_idx = 0;

	/* Decide the transfer mode, send ,receive or both */
	lld_spi_set_transfer_mode(spi_device, data_out, data_in);

	/* First convert tx buffer */
	if ((context->transfer_mode != SPI_TRANSFER_MODE_RECEIVE) &&
	    (context->is_lsb == true)) {
		spi_bit_reverse(context->data_tx, context->tx_data_len,
				context->frame_size);
	}

	/* According to different transfer mode, do different fill or receive */
	if (context->transfer_mode == SPI_TRANSFER_MODE_SEND) {
		rx_num = 0;
	} else if (context->transfer_mode == SPI_TRANSFER_MODE_RECEIVE) {
		tx_num = context->frame_size; /* Shall send at least one data
						 for receive */
		DBG_CHECK((num <= SPI_RECEIVE_MODE_MAX_SIZE),
			  SEDI_DRIVER_ERROR_PARAMETER);
		context->base->ctrl1 = num / context->frame_size - 1;
	}

	lld_spi_enable(context->base, true);

	/* First send some data in both transfer mode */
	fill_num = lld_spi_fill_fifo(spi_device, context->frame_size, data_out,
				     tx_num);
	/* Update buffer and number */
	if (data_out) {
		data_out += fill_num;
	}
	tx_num -= fill_num;

	while ((tx_num > 0) || (rx_num > 0)) {
		if (tx_num > 0) {
			/* First send some data */
			fill_num = lld_spi_fill_fifo(
			    spi_device, context->frame_size, data_out, tx_num);
			/* Update buffer and number */
			data_out += fill_num;
			tx_num -= fill_num;
		}

		if (rx_num > 0) {
			/* Receive some data */
			receive_num = lld_spi_receive_fifo(context->base,
							   context->frame_size,
							   data_in, rx_num);
			data_in += receive_num;
			rx_num -= receive_num;
		}
	}

	/* Waiting for SPI idle */
	while (lld_spi_is_busy(context->base))
		;
	lld_spi_enable(context->base, false);

	context->status.busy = 0U;
	context->data_tx_idx = num;
	context->data_rx_idx = num;

	/* If has rx buffer and need bit reverse */
	if ((context->transfer_mode != SPI_TRANSFER_MODE_SEND) &&
	    (context->is_lsb == true)) {
		spi_bit_reverse(context->data_rx, context->rx_data_len,
				context->frame_size);
	}

	/* If need to recover tx buffer */
	if ((context->transfer_mode != SPI_TRANSFER_MODE_RECEIVE) &&
	    (context->is_lsb == true)) {
		spi_bit_reverse(context->data_tx, context->tx_data_len,
				context->frame_size);
	}
	return SEDI_DRIVER_OK;
}

int32_t sedi_spi_transfer(IN sedi_spi_t spi_device, IN uint8_t *data_out,
			  OUT uint8_t *data_in, IN uint32_t num)
{
	DBG_CHECK(spi_device < SEDI_SPI_NUM, SEDI_DRIVER_ERROR_PARAMETER);

	struct spi_context *context = &contexts[spi_device];
	spi_reg_t *spi = context->base;
	uint32_t send_count = num;

	DBG_CHECK(((num % context->frame_size) == 0),
		  SEDI_DRIVER_ERROR_PARAMETER);

	if (context->status.busy) {
		return SEDI_DRIVER_ERROR_BUSY;
	}

	/* For transfer size less than watermark */
	if (num < context->rx_watermark * context->frame_size) {
		/* Only shall reset the receive watermark to finish trigger
		 * interrupt */
		lld_spi_set_rx_watermark(context->base,
					 num / context->frame_size);
	} else {
		lld_spi_set_rx_watermark(context->base,
					 (SPI_FIFO_DEPTH / 2 - 1) *
					     context->frame_size);
	}

	lld_spi_set_transfer_mode(spi_device, data_out, data_in);

	/* For IRQ mode only, if use multiple buffers, cannot change mode in
	 * transfer */
	if (context->is_cs_continuous == true) {
		spi->ctrl0 &= ~SPI_CTRL0_TMOD_MASK;
		spi->ctrl0 |= SPI_CTRL0_BOTH_MODE;
		context->transfer_mode = SPI_TRANSFER_MODE_BOTH;
	}

	context->status.busy = 1U;

	context->data_tx = (void *)data_out;
	context->data_rx = (void *)data_in;
	context->tx_data_len = num;
	context->rx_data_len = num;
	context->data_tx_idx = 0;
	context->data_rx_idx = 0;

	/* First convert tx buffer */
	if ((context->transfer_mode != SPI_TRANSFER_MODE_RECEIVE) &&
	    (context->is_lsb == true)) {
		spi_bit_reverse(context->data_tx, context->tx_data_len,
				context->frame_size);
	}

	/* It is better to fill some data before enable interrupt to avoid fifo
	 * error */
	/* According to different transfer mode, do different fill or receive */
	if (context->transfer_mode == SPI_TRANSFER_MODE_SEND) {
		context->data_rx_idx = num;
	} else if (context->transfer_mode == SPI_TRANSFER_MODE_RECEIVE) {
		send_count = context->frame_size;
		DBG_CHECK((num <= SPI_RECEIVE_MODE_MAX_SIZE),
			  SEDI_DRIVER_ERROR_PARAMETER);
		context->base->ctrl1 = num / context->frame_size - 1;
		lld_spi_fill_fifo(spi_device, context->frame_size, data_out,
				  send_count);
		context->data_tx_idx = num;
	}

	lld_spi_enable(context->base, true);

	lld_spi_config_interrupt(context->base,
				 REG_INT_TX | REG_INT_RX | REG_INT_ERROR);

	return SEDI_DRIVER_OK;
}

int32_t sedi_spi_update_tx_buf(IN sedi_spi_t spi_device, IN uint8_t *tx_buf,
			       IN uint32_t len)
{
	DBG_CHECK(spi_device < SEDI_SPI_NUM, SEDI_DRIVER_ERROR_PARAMETER);

	struct spi_context *context = &contexts[spi_device];

	DBG_CHECK(((len % context->frame_size) == 0),
		  SEDI_DRIVER_ERROR_PARAMETER);

	/* This function can only used in continuous mode */
	DBG_CHECK((context->is_cs_continuous == true),
		  SEDI_DRIVER_ERROR_UNSUPPORTED);

	if (len == 0) {
		return SEDI_DRIVER_ERROR_PARAMETER;
	}
	/* As continuous mode all use both transfer mode, rx also need to update
	 * length */
	context->data_tx = (void *)tx_buf;
	context->tx_data_len += len;

	return SEDI_DRIVER_OK;
}

int32_t sedi_spi_update_rx_buf(IN sedi_spi_t spi_device, OUT uint8_t *rx_buf,
			       IN uint32_t len)
{
	DBG_CHECK(spi_device < SEDI_SPI_NUM, SEDI_DRIVER_ERROR_PARAMETER);

	struct spi_context *context = &contexts[spi_device];

	DBG_CHECK(((len % context->frame_size) == 0),
		  SEDI_DRIVER_ERROR_PARAMETER);

	/* This function can only used in continuous mode */
	DBG_CHECK((context->is_cs_continuous == true),
		  SEDI_DRIVER_ERROR_UNSUPPORTED);

	if (len == 0) {
		return SEDI_DRIVER_ERROR_PARAMETER;
	}

	/* As continuous mode all use both transfer mode, rx also need to update
	 * length */
	context->data_rx = (void *)rx_buf;
	context->rx_data_len += len;

	return SEDI_DRIVER_OK;
}

void spi_isr(IN sedi_spi_t spi_device)
{
	struct spi_context *context = &contexts[spi_device];
	uint32_t intr_stat;
	int error = false;
	int end = false;
	int event;
	int idx;
	uint32_t temp, rx_len;

	intr_stat = lld_spi_interrupt_clear(context->base);

	/* To reduce the interrupt times, send/receive as many as possible */
	if (intr_stat & (REG_INT_RX | REG_INT_TX)) {
		while (context->data_tx_idx < context->tx_data_len) {
			temp = context->tx_data_len - context->data_tx_idx;
			idx = lld_spi_fill_fifo(spi_device, context->frame_size,
						context->data_tx, temp);

			context->data_tx_idx += idx;
			if (context->data_tx != NULL) {
				context->data_tx += idx;
			}

			if ((context->data_tx_idx == context->tx_data_len) &&
			    (context->cb_event)) {
				context->cb_event(SEDI_SPI_EVENT_TX_FINISHED,
						  context->cb_param);
			}

			if (idx < temp) {
				/* If last transfer filled FIFO full, break */
				break;
			}
		}

		while (context->data_rx_idx < context->rx_data_len) {
			rx_len = context->rx_data_len - context->data_rx_idx;
			idx = lld_spi_receive_fifo(context->base,
						   context->frame_size,
						   context->data_rx, rx_len);

			context->data_rx_idx += idx;
			if (context->data_rx != NULL) {
				context->data_rx += idx;
			}

			/*Check if need to modify watermark for last transfer*/
			if ((context->rx_data_len - context->data_rx_idx <
			     context->frame_size * context->rx_watermark) &&
			    (context->rx_data_len != context->data_rx_idx)) {
				temp = (context->rx_data_len -
					context->data_rx_idx) /
				       context->frame_size;
				lld_spi_set_rx_watermark(context->base, temp);
				context->rx_watermark = temp;
			}

			if ((context->data_rx_idx == context->rx_data_len) &&
			    (context->cb_event)) {
				context->cb_event(SEDI_SPI_EVENT_RX_FINISHED,
						  context->cb_param);
			}

			if (idx < rx_len) {
				/* If last transfer received all data in FIFO,
				 * break */
				break;
			}
		}
	}

	if ((context->data_rx_idx == context->tx_data_len) &&
	    (context->data_tx_idx == context->rx_data_len)) {
		end = true;
		event = SEDI_SPI_EVENT_COMPLETE;
		/* Wait for Data in FIFO send out while not continuous */
		while (lld_spi_is_busy(context->base))
			;

		/* If need to reverse rx buffer */
		if ((context->transfer_mode != SPI_TRANSFER_MODE_SEND) &&
		    (context->is_lsb == true)) {
			context->data_rx -= context->data_rx_idx;
			spi_bit_reverse(context->data_rx, context->rx_data_len,
					context->frame_size);
		}
		/* If need to recover tx buffer */
		if ((context->transfer_mode != SPI_TRANSFER_MODE_RECEIVE) &&
		    (context->is_lsb == true)) {
			context->data_tx -= context->data_tx_idx;
			spi_bit_reverse(context->data_tx, context->tx_data_len,
					context->frame_size);
		}
	}

	if (intr_stat & REG_INT_ERROR) {
		error = true;
		event = SEDI_SPI_EVENT_DATA_LOST;
		context->status.data_lost = true;
	}

	if ((error || end) && (context->status.busy != 0)) {
		context->status.busy = 0;
		lld_spi_config_interrupt(context->base, REG_INT_NONE);
		lld_spi_enable(context->base, false);

		if (context->cb_event) {
			context->cb_event(event, context->cb_param);
		}
	}
}

SEDI_ISR_DECLARE(sedi_spi_0_isr)
{
	spi_isr(SEDI_SPI_0);
}

SEDI_ISR_DECLARE(sedi_spi_1_isr)
{
	spi_isr(SEDI_SPI_1);
}

SEDI_ISR_DECLARE(sedi_spi_2_isr)
{
	spi_isr(SEDI_SPI_2);
}

SEDI_ISR_DECLARE(sedi_spi_3_isr)
{
	spi_isr(SEDI_SPI_3);
}
