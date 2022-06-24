/*
 * Copyright (c) 2021 Intel Corporation
 *
 * SPDX-License-Identifier:  BSD-3-Clause
 */

#include <intel/hal_driver_common.h>
#include <intel/hal_spi.h>

#define SPI_REG(ctx) ((spi_reg_t *)((ctx)->inst.base_addr))
#define SPI_REG_PHY(ctx) ((spi_reg_t *)((ctx)->inst.phy_addr))

static const intel_driver_version_t driver_version = { 0, 1 };

static inline void lld_spi_enable(spi_reg_t *spi, bool enable)
{
	uint32_t val = enable ? 1 : 0;

	if (spi->ssienr == val) {
		return;
	}
	/* prevent pending interrupt */
	spi->imr = 0;

	spi->ssienr = val;
	while (spi->ssienr != val) {
		;
	}
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

static int lld_spi_default_config(struct spi_context *ctx)
{
	spi_reg_t *spi = SPI_REG(ctx);

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
#ifdef CONFIG_INTEL_DMA
	spi->dmatdlr = watermark;
	spi->dmardlr = watermark;
#endif

	spi->ctrl0 = ctrl0;
	spi->baudr = prescale;
	spi->ser = cs_mask;

	/* Update ctx default settings */
	ctx->tx_watermark = watermark + 1U;
	ctx->rx_watermark = watermark + 1U;
	ctx->prescale = DEFAULT_PRESCALE;
	ctx->frame_size = 1U;
	ctx->is_lsb = false;

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
	uint32_t ctrl0 = spi->ctrl0 & ~MASK_CTRL0_WIDTH;

	ctrl0 |= (width - 1) << OFFSET_CTRL0_WIDTH;
	spi->ctrl0 = ctrl0;
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

static void lld_spi_set_transfer_mode(struct spi_context *ctx,
				      IN uint8_t *data_out,
				      OUT uint8_t *data_in)
{
	spi_reg_t *spi = SPI_REG(ctx);
	uint32_t ctrl0 = spi->ctrl0;

	ctrl0 &= ~SPI_CTRL0_TMOD_MASK;
	if (data_out == NULL) {
		/* Set to receive only mode */
		ctrl0 |= SPI_CTRL0_RECEIVE_MODE;
		ctx->transfer_mode = SPI_TRANSFER_MODE_RECEIVE;
	} else if (data_in == NULL) {
		/* Set to receive only mode */
		ctrl0 |= SPI_CTRL0_SEND_MODE;
		ctx->transfer_mode = SPI_TRANSFER_MODE_SEND;
	} else {
		ctrl0 |= SPI_CTRL0_BOTH_MODE;
		ctx->transfer_mode = SPI_TRANSFER_MODE_BOTH;
	}

	spi->ctrl0 = ctrl0;
}

static int lld_spi_fill_fifo(struct spi_context *ctx, uint8_t frame_size,
			     IN uint8_t *buff, uint32_t count)
{
	spi_reg_t *spi = SPI_REG(ctx);
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
			data = ctx->dummy_data;
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

static inline void *lld_spi_dr_phy_addr(struct spi_context *ctx)
{
	spi_reg_t *spi = SPI_REG_PHY(ctx);
	return (void *)&spi->dr;
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
				       intel_spi_microwire_config_t *config)
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
 * intel interface
 *****************************************************************************/

intel_driver_version_t intel_spi_get_version(void)
{
	return driver_version;
}

int32_t intel_spi_get_capabilities(INOUT intel_instance_t *inst,
				   intel_spi_capabilities_t *cap)
{
	struct spi_context *ctx = CONTAINER_OF(inst, struct spi_context, inst);

#ifdef CONFIG_ARM
	if (DEV_PSE_OWNED == intel_get_dev_ownership(PSE_DEV_SPI0 + ctx->id)) {
		ctx->cap.is_available = 1;
	} else {
		ctx->cap.is_available = 0;
	}
#else
	ctx->cap.is_available = 0;
#endif

	ctx->cap.ti_ssi = 1;
	ctx->cap.microwire = 1;
	ctx->cap.mode_fault = 0;

	*cap = ctx->cap;

	return INTEL_DRIVER_OK;
}

int32_t intel_spi_init(INOUT intel_instance_t *inst, IN intel_spi_event_cb_t cb_event,
		       INOUT void *param)
{
	struct spi_context *ctx = CONTAINER_OF(inst, struct spi_context, inst);

#ifdef CONFIG_ARM
	if (DEV_PSE_OWNED != intel_get_dev_ownership(PSE_DEV_SPI0 + ctx->id)) {
		return INTEL_DRIVER_ERROR_NO_DEV;
	}
#endif
	lld_spi_default_config(ctx);

	ctx->cb_event = cb_event;
	ctx->cb_param = param;

	return INTEL_DRIVER_OK;
}

int32_t intel_spi_uninit(INOUT intel_instance_t *inst)
{
	struct spi_context *ctx = CONTAINER_OF(inst, struct spi_context, inst);

	ctx->cb_event = NULL;
	ctx->is_lsb = false;

	return INTEL_DRIVER_OK;
}

int32_t intel_spi_get_data_count(INOUT intel_instance_t *inst)
{
	struct spi_context *ctx = CONTAINER_OF(inst, struct spi_context, inst);

	if (ctx->data_tx) {
		return ctx->data_tx_idx;
	} else {
		return ctx->data_rx_idx;
	}
}

int32_t intel_spi_get_status(INOUT intel_instance_t *inst, intel_spi_status_t *status)
{
	DBG_CHECK(NULL != status, INTEL_DRIVER_ERROR_PARAMETER);

	struct spi_context *ctx = CONTAINER_OF(inst, struct spi_context, inst);

	status->busy = ctx->status.busy;
	status->data_lost = ctx->status.data_lost;
	status->mode_fault = ctx->status.mode_fault;

	return INTEL_DRIVER_OK;
}

int32_t intel_spi_control(INOUT intel_instance_t *inst, IN uint32_t control,
			  IN uintptr_t arg)
{
	DBG_CHECK(control < INTEL_SPI_IOCTL_NUM, INTEL_DRIVER_ERROR_PARAMETER);

	struct spi_context *ctx = CONTAINER_OF(inst, struct spi_context, inst);

	switch (control) {
	case INTEL_SPI_IOCTL_CPOL0_CPHA0:
		lld_spi_config_cpol_cpha(SPI_REG(ctx), 0, 0);
		break;
	case INTEL_SPI_IOCTL_CPOL0_CPHA1:
		lld_spi_config_cpol_cpha(SPI_REG(ctx), 0, 1);
		break;
	case INTEL_SPI_IOCTL_CPOL1_CPHA0:
		lld_spi_config_cpol_cpha(SPI_REG(ctx), 1, 0);
		break;
	case INTEL_SPI_IOCTL_CPOL1_CPHA1:
		lld_spi_config_cpol_cpha(SPI_REG(ctx), 1, 1);
		break;
	case INTEL_SPI_IOCTL_TI_SSI:
		lld_spi_set_ti_mode(SPI_REG(ctx));
		break;
	case INTEL_SPI_IOCTL_MICROWIRE:
		lld_spi_set_microwire_mode(SPI_REG(ctx),
					   (intel_spi_microwire_config_t *)arg);
		break;
	case INTEL_SPI_IOCTL_MSB:
		ctx->is_lsb = false;
		break;
	case INTEL_SPI_IOCTL_LSB:
		return INTEL_DRIVER_ERROR_PARAMETER;
	case INTEL_SPI_IOCTL_DATA_WIDTH:
		DBG_CHECK(((arg == SPI_BITWIDTH_4BITS) ||
			   (arg == SPI_BITWIDTH_8BITS) ||
			   (arg == SPI_BITWIDTH_16BITS)),
			  INTEL_DRIVER_ERROR_PARAMETER);
		ctx->frame_size = (uint8_t)arg / 8U;
		/* For 4 bits operation, operate like 8 bits */
		if (ctx->frame_size == 0) {
			ctx->frame_size = SPI_FRAME_SIZE_1_BYTE;
		}
		lld_spi_config_width(SPI_REG(ctx), (uint8_t)arg);
		break;
	case INTEL_SPI_IOCTL_CS_UNUSED:
		lld_spi_config_cs(SPI_REG(ctx), 0U);
		break;
	case INTEL_SPI_IOCTL_CS_HW:
		lld_spi_config_cs(SPI_REG(ctx), (uint32_t)arg);
		break;
	case INTEL_SPI_IOCTL_SPEED_SET:
		ctx->prescale = SSI_IC_FREQ / (uint32_t)arg;
		if (ctx->prescale < SSI_PRESCALE_MIN) {
			ctx->prescale = SSI_PRESCALE_MIN;
		}
		/* Divider can only support even number */
		ctx->prescale &= 0xFFFFFFFE;
		lld_spi_config_prescale(SPI_REG(ctx), ctx->prescale);
		break;
	case INTEL_SPI_IOCTL_TX_WATERMARK_SET:
		ctx->tx_watermark = (uint32_t)arg;
		lld_spi_set_tx_watermark(SPI_REG(ctx), (uint32_t)arg);
		break;
	case INTEL_SPI_IOCTL_RX_WATERMARK_SET:
		ctx->rx_watermark = (uint32_t)arg;
		lld_spi_set_rx_watermark(SPI_REG(ctx), (uint32_t)arg);
		break;
	case INTEL_SPI_IOCTL_DUMMY_DATA:
		ctx->dummy_data = (uint32_t)arg;
		break;
	case INTEL_SPI_IOCTL_LOOPBACK:
		lld_spi_config_loopback(SPI_REG(ctx), (bool)arg);
		break;
	case INTEL_SPI_IOCTL_SPEED_GET:
		*((uint32_t *)arg) = SSI_IC_FREQ / ctx->prescale;
		break;
	case INTEL_SPI_IOCTL_TX_WATERMARK_GET:
		*((uint32_t *)arg) = ctx->tx_watermark;
		break;
	case INTEL_SPI_IOCTL_RX_WATERMARK_GET:
		*((uint32_t *)arg) = ctx->rx_watermark;
		break;
	case INTEL_SPI_IOCTL_ABORT:
		lld_spi_enable(SPI_REG(ctx), false);
		ctx->status.busy = 0;
		break;
	case INTEL_SPI_IOCTL_BUFFER_SETS:
		ctx->is_cs_continuous = (bool)arg;
		break;
	case INTEL_SPI_IOCTL_SET_RX_DELAY:
		lld_spi_set_rx_delay(SPI_REG(ctx), (uint8_t)arg);
		break;
	default:
		break;
	}

	return INTEL_DRIVER_OK;
}

#ifdef CONFIG_INTEL_DMA
static inline void lld_spi_dma_enable(spi_reg_t *spi, bool enable)
{
	spi->dmacr = enable ? REG_DMA_ENABLE : 0;
}

static void callback_dma_transfer(const intel_instance_t *dma, const int chan,
				  const int event, void *param)
{
	struct spi_context *ctx = param;

	/* release the dma resource */
	intel_dma_uninit(dma, chan);

	/* See tx or rx finished */
	if (chan == ctx->tx_channel) {
		ctx->dma_tx_finished = true;
		ctx->data_tx_idx = ctx->tx_data_len;
		/* Waiting for TX FIFO empty */
		while (lld_spi_is_busy(SPI_REG(ctx))) {
			;
		}
	} else if (chan == ctx->rx_channel) {
		ctx->dma_rx_finished = true;
		ctx->data_rx_idx = ctx->rx_data_len;
	}

	/* All tx and rx finished */
	if ((ctx->dma_tx_finished == true) &&
	    (ctx->dma_rx_finished == true)) {
		/* reset busy bit to enable user call async transfer in callback
		 */
		ctx->status.busy = 0;
		lld_spi_dma_enable(SPI_REG(ctx), false);
		lld_spi_config_interrupt(SPI_REG(ctx), REG_INT_NONE);
		lld_spi_enable(SPI_REG(ctx), false);

		if (ctx->cb_event) {
			if (INTEL_DMA_EVENT_TRANSFER_DONE == event) {
				ctx->cb_event(INTEL_SPI_EVENT_COMPLETE,
					      ctx->cb_param);
			} else {
				ctx->cb_event(INTEL_SPI_EVENT_DATA_LOST,
					      ctx->cb_param);
			}
		}
	}
}

static int config_and_enable_dma_channel(struct spi_context *ctx,
					 const intel_instance_t *dma,
					 int handshake, int chan, int width,
					 int burst, const void *src, void *dst,
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

	ret = intel_dma_init(dma, chan, callback_dma_transfer, ctx);
	DBG_CHECK(0 == ret, INTEL_DRIVER_ERROR);

	ret = intel_dma_control(dma, chan, INTEL_CONFIG_DMA_BURST_LENGTH,
			       DMA_BURST_TRANS_LENGTH_1);
	DBG_CHECK(0 == ret, INTEL_DRIVER_ERROR);

	ret = intel_dma_control(dma, chan, INTEL_CONFIG_DMA_SR_TRANS_WIDTH, wid);
	DBG_CHECK(0 == ret, INTEL_DRIVER_ERROR);

	ret = intel_dma_control(dma, chan, INTEL_CONFIG_DMA_DT_TRANS_WIDTH, wid);
	DBG_CHECK(0 == ret, INTEL_DRIVER_ERROR);

	ret = intel_dma_control(dma, chan, INTEL_CONFIG_DMA_HS_DEVICE_ID,
			       handshake);
	DBG_CHECK(0 == ret, INTEL_DRIVER_ERROR);

	ret = intel_dma_control(dma, chan, INTEL_CONFIG_DMA_HS_POLARITY,
			       DMA_HS_POLARITY_HIGH);
	DBG_CHECK(0 == ret, INTEL_DRIVER_ERROR);

	ret = intel_dma_control(dma, chan, INTEL_CONFIG_DMA_DIRECTION, dma_dir);
	DBG_CHECK(0 == ret, INTEL_DRIVER_ERROR);

	ret = intel_dma_control(dma, chan, INTEL_CONFIG_DMA_HS_DEVICE_ID_PER_DIR,
			       dma_per_dir);
	DBG_CHECK(0 == ret, INTEL_DRIVER_ERROR);

	ret = intel_dma_start_transfer(dma, chan,
				       (uintptr_t)src, (uintptr_t)dst, len);
	DBG_CHECK(0 == ret, INTEL_DRIVER_ERROR);

	return 0;
}

int32_t intel_spi_dma_transfer(INOUT intel_instance_t *inst,
			       INOUT intel_instance_t *tx_dma,
			       IN uint32_t tx_dma_chan, IN uint8_t *data_out,
			       INOUT intel_instance_t *rx_dma,
			       IN uint32_t rx_dma_chan, OUT uint8_t *data_in,
			       IN uint32_t num)
{
	struct spi_context *ctx = CONTAINER_OF(inst, struct spi_context, inst);
	int handshake = ctx->spi_dma_hs_id;
	int width = ctx->frame_size;
	int burst = 1;

	DBG_CHECK(((num % ctx->frame_size) == 0),
		  INTEL_DRIVER_ERROR_PARAMETER);

	if (ctx->status.busy) {
		return INTEL_DRIVER_ERROR_BUSY;
	}

	ctx->status.busy = 1U;

	SPI_REG(ctx)->dmatdlr = SPI_FIFO_DEPTH - 1;
	SPI_REG(ctx)->dmardlr = 0;
	ctx->tx_channel = tx_dma_chan;
	ctx->rx_channel = rx_dma_chan;
	ctx->dma_tx_finished = false;
	ctx->dma_rx_finished = false;
	ctx->tx_data_len = num;
	ctx->rx_data_len = num;
	ctx->data_tx = (uint8_t *)data_out;
	ctx->data_rx = data_in;

	/* Decide the transfer mode, send ,recive or both */
	lld_spi_set_transfer_mode(ctx, data_out, data_in);

	/* According to different transfer mode, do different fill or receive */
	if (ctx->transfer_mode == SPI_TRANSFER_MODE_SEND) {
		/* start dma first */
		config_and_enable_dma_channel(
		    ctx, tx_dma, handshake, tx_dma_chan, width, burst,
		    data_out, lld_spi_dr_phy_addr(ctx), num,
		    true);
		ctx->dma_rx_finished = true;
		ctx->rx_channel = 0xFF;
	} else if (ctx->transfer_mode == SPI_TRANSFER_MODE_RECEIVE) {
		/* Send dummy data first */
		lld_spi_fill_fifo(ctx, ctx->frame_size, NULL,
				  ctx->frame_size);
		/* Configure rx channel */
		config_and_enable_dma_channel(ctx, rx_dma, handshake,
					      rx_dma_chan, width, burst,
					      lld_spi_dr_phy_addr(ctx),
					      data_in, num, false);
		/* Set NDF bits for receive only mode */
		DBG_CHECK((num <= SPI_RECEIVE_MODE_MAX_SIZE),
			  INTEL_DRIVER_ERROR_PARAMETER);
		SPI_REG(ctx)->ctrl1 = num / ctx->frame_size - 1;
		ctx->dma_tx_finished = true;
		ctx->tx_channel = 0xFF;
	} else {
		/* Enable both channel to do transfer */
		config_and_enable_dma_channel(ctx, tx_dma, handshake,
					      tx_dma_chan, width, burst,
					      data_out,
					      lld_spi_dr_phy_addr(ctx),
					      num, true);
		config_and_enable_dma_channel(ctx, rx_dma, handshake,
					      rx_dma_chan, width, burst,
					      lld_spi_dr_phy_addr(ctx),
					      data_in, num, false);
	}

	lld_spi_enable(SPI_REG(ctx), true);

	lld_spi_dma_enable(SPI_REG(ctx), true);

	/* dma will clear tx/rx event, and error event will be cleared here */
	lld_spi_config_interrupt(SPI_REG(ctx), REG_INT_ERROR);

	return INTEL_DRIVER_OK;
}
#endif /* CONFIG_INTEL_DMA */

int32_t intel_spi_poll_transfer(INOUT intel_instance_t *inst, IN uint8_t *data_out,
			       OUT uint8_t *data_in, IN uint32_t num)
{
	uint32_t tx_num = num, rx_num = num, fill_num = 0, receive_num = 0;
	struct spi_context *ctx = CONTAINER_OF(inst, struct spi_context, inst);

	DBG_CHECK(((num % ctx->frame_size) == 0),
		  INTEL_DRIVER_ERROR_PARAMETER);

	if (ctx->status.busy) {
		return INTEL_DRIVER_ERROR_BUSY;
	}

	ctx->status.busy = 1U;
	ctx->data_tx = (void *)data_out;
	ctx->data_rx = (void *)data_in;
	ctx->tx_data_len = num;
	ctx->rx_data_len = num;
	ctx->data_tx_idx = 0;
	ctx->data_rx_idx = 0;

	/* Decide the transfer mode, send ,recive or both */
	lld_spi_set_transfer_mode(ctx, data_out, data_in);

	/* According to different transfer mode, do different fill or receive */
	if (ctx->transfer_mode == SPI_TRANSFER_MODE_SEND) {
		rx_num = 0;
	} else if (ctx->transfer_mode == SPI_TRANSFER_MODE_RECEIVE) {
		tx_num = ctx->frame_size; /* Shall send at least one data
						 for receive */
		DBG_CHECK((num <= SPI_RECEIVE_MODE_MAX_SIZE),
			  INTEL_DRIVER_ERROR_PARAMETER);
		SPI_REG(ctx)->ctrl1 = num / ctx->frame_size - 1;
	}

	lld_spi_enable(SPI_REG(ctx), true);

	/* First send some data in both transfer mode */
	fill_num = lld_spi_fill_fifo(ctx, ctx->frame_size, data_out, tx_num);
	/* Update buffer and number */
	if (data_out) {
		data_out += fill_num;
	}
	tx_num -= fill_num;

	while ((tx_num > 0) || (rx_num > 0)) {
		if (tx_num > 0) {
			/* First send some data */
			fill_num = lld_spi_fill_fifo(ctx, ctx->frame_size,
						     data_out, tx_num);
			/* Update buffer and number */
			data_out += fill_num;
			tx_num -= fill_num;
		}

		if (rx_num > 0) {
			/* Receive some data */
			receive_num = lld_spi_receive_fifo(SPI_REG(ctx),
							   ctx->frame_size,
							   data_in, rx_num);
			data_in += receive_num;
			rx_num -= receive_num;
		}
	}

	/* Waiting for SPI idle */
	while (lld_spi_is_busy(SPI_REG(ctx)))
		;
	lld_spi_enable(SPI_REG(ctx), false);

	ctx->status.busy = 0U;
	ctx->data_tx_idx = num;
	ctx->data_rx_idx = num;

	return INTEL_DRIVER_OK;
}

static int xfr_polled_mode(struct spi_context *ctx)
{
	uint32_t temp, rx_len;
	int idx;

	do {
		while (ctx->data_tx_idx < ctx->tx_data_len) {
			temp = ctx->tx_data_len - ctx->data_tx_idx;
			idx = lld_spi_fill_fifo(ctx, ctx->frame_size,
						ctx->data_tx, temp);

			ctx->data_tx_idx += idx;
			if (ctx->data_tx != NULL) {
				ctx->data_tx += idx;
			}

			if ((ctx->data_tx_idx == ctx->tx_data_len) &&
				(ctx->cb_event)) {
				ctx->cb_event(INTEL_SPI_EVENT_TX_FINISHED,
						  ctx->cb_param);
			}

			if (idx < temp) {
				/* If last transfer filled FIFO full, break */
				break;
			}
		}

		while (ctx->data_rx_idx < ctx->rx_data_len) {
			rx_len = ctx->rx_data_len - ctx->data_rx_idx;
			idx = lld_spi_receive_fifo(SPI_REG(ctx),
						   ctx->frame_size,
						   ctx->data_rx, rx_len);

			ctx->data_rx_idx += idx;
			if (ctx->data_rx != NULL) {
				ctx->data_rx += idx;
			}

			/*Check if need to modify watermark for last transfer*/
			if ((ctx->rx_data_len - ctx->data_rx_idx <
				 ctx->frame_size * ctx->rx_watermark) &&
				(ctx->rx_data_len != ctx->data_rx_idx)) {
				temp = (ctx->rx_data_len -
					ctx->data_rx_idx) /
					   ctx->frame_size;
				lld_spi_set_rx_watermark(SPI_REG(ctx), temp);
				ctx->rx_watermark = temp;
			}

			if ((ctx->data_rx_idx == ctx->rx_data_len) &&
				(ctx->cb_event)) {
				ctx->cb_event(INTEL_SPI_EVENT_RX_FINISHED,
						  ctx->cb_param);
			}

			if (idx < rx_len) {
				break;
			}
		}
	} while ((ctx->data_rx_idx != ctx->tx_data_len) ||
	    (ctx->data_tx_idx != ctx->rx_data_len));

	while (lld_spi_is_busy(SPI_REG(ctx)))
		;
	if (ctx->cb_event) {
		ctx->cb_event(INTEL_SPI_EVENT_COMPLETE,
					  ctx->cb_param);
	}
	ctx->status.busy = 0;

	return INTEL_DRIVER_OK;
}

int32_t intel_spi_transfer(INOUT intel_instance_t *inst, IN uint8_t *data_out,
			  OUT uint8_t *data_in, IN uint32_t num, IN bool poll_mode)
{
	struct spi_context *ctx = CONTAINER_OF(inst, struct spi_context, inst);
	spi_reg_t *spi = SPI_REG(ctx);
	uint32_t send_count = num;

	DBG_CHECK(((num % ctx->frame_size) == 0), INTEL_DRIVER_ERROR_PARAMETER);

	if (ctx->status.busy) {
		return INTEL_DRIVER_ERROR_BUSY;
	}

	/* For transfer size less than watermark */
	if (num < ctx->rx_watermark * ctx->frame_size) {
		/* Only shall reset the revice watermark to finish trigger
		 * interrupt */
		lld_spi_set_rx_watermark(SPI_REG(ctx),
					 num / ctx->frame_size);
		lld_spi_set_tx_watermark(SPI_REG(ctx),
					 num / ctx->frame_size);
	} else {
		lld_spi_set_rx_watermark(SPI_REG(ctx),
					 (SPI_FIFO_DEPTH / 2 - 1) *
					     ctx->frame_size);
		lld_spi_set_tx_watermark(SPI_REG(ctx),
					 (SPI_FIFO_DEPTH / 2 - 1) *
					     ctx->frame_size);
	}

	lld_spi_set_transfer_mode(ctx, data_out, data_in);

	/* For IRQ mode only, if use multiple buffers, cannot change mode in
	 * transfer */
	if (ctx->is_cs_continuous == true) {
		spi->ctrl0 &= ~SPI_CTRL0_TMOD_MASK;
		spi->ctrl0 |= SPI_CTRL0_BOTH_MODE;
		ctx->transfer_mode = SPI_TRANSFER_MODE_BOTH;
	}

	ctx->status.busy = 1U;

	ctx->data_tx = (void *)data_out;
	ctx->data_rx = (void *)data_in;
	ctx->tx_data_len = num;
	ctx->rx_data_len = num;
	ctx->data_tx_idx = 0;
	ctx->data_rx_idx = 0;

	/* It is better to fill some data before enable interrupt to avoid fifo
	 * error */
	/* According to different transfer mode, do different fill or receive */
	if (ctx->transfer_mode == SPI_TRANSFER_MODE_SEND) {
		ctx->data_rx_idx = num;
	} else if (ctx->transfer_mode == SPI_TRANSFER_MODE_RECEIVE) {
		send_count = ctx->frame_size;
		DBG_CHECK((num <= SPI_RECEIVE_MODE_MAX_SIZE),
			  INTEL_DRIVER_ERROR_PARAMETER);
		SPI_REG(ctx)->ctrl1 = num / ctx->frame_size - 1;
		lld_spi_fill_fifo(ctx, ctx->frame_size, data_out, send_count);
		ctx->data_tx_idx = num;
	}

	lld_spi_enable(SPI_REG(ctx), true);

	if (poll_mode) {
		xfr_polled_mode(ctx);
		lld_spi_enable(SPI_REG(ctx), false);
	} else {
		lld_spi_config_interrupt(SPI_REG(ctx),
				 REG_INT_TX | REG_INT_RX | REG_INT_ERROR);
	}

	return INTEL_DRIVER_OK;
}

int32_t intel_spi_update_tx_buf(INOUT intel_instance_t *inst, IN uint8_t *tx_buf,
			       IN uint32_t len)
{
	struct spi_context *ctx = CONTAINER_OF(inst, struct spi_context, inst);

	DBG_CHECK(((len % ctx->frame_size) == 0), INTEL_DRIVER_ERROR_PARAMETER);

	/* This function can only used in continous mode */
	DBG_CHECK((ctx->is_cs_continuous == true), INTEL_DRIVER_ERROR_UNSUPPORTED);

	if (len == 0) {
		return INTEL_DRIVER_ERROR_PARAMETER;
	}
	/* As continous mode all use both transfer mode, rx also need to update
	 * length */
	ctx->data_tx = (void *)tx_buf;
	ctx->tx_data_len += len;

	return INTEL_DRIVER_OK;
}

int32_t intel_spi_update_rx_buf(INOUT intel_instance_t *inst, OUT uint8_t *rx_buf,
				IN uint32_t len)
{
	struct spi_context *ctx = CONTAINER_OF(inst, struct spi_context, inst);

	DBG_CHECK(((len % ctx->frame_size) == 0), INTEL_DRIVER_ERROR_PARAMETER);

	/* This function can only used in continous mode */
	DBG_CHECK((ctx->is_cs_continuous == true), INTEL_DRIVER_ERROR_UNSUPPORTED);

	if (len == 0) {
		return INTEL_DRIVER_ERROR_PARAMETER;
	}

	/* As continous mode all use both transfer mode, rx also need to update
	 * length */
	ctx->data_rx = (void *)rx_buf;
	ctx->rx_data_len += len;

	return INTEL_DRIVER_OK;
}

void intel_spi_isr(INOUT intel_instance_t *inst)
{
	struct spi_context *ctx = CONTAINER_OF(inst, struct spi_context, inst);
	uint32_t intr_stat;
	int error = false;
	int end = false;
	int event;
	int idx;
	uint32_t temp, rx_len;

	intr_stat = lld_spi_interrupt_clear(SPI_REG(ctx));
	lld_spi_config_interrupt(SPI_REG(ctx), REG_INT_NONE);

	/* To reduce the interrupt times, send/receive as many as possible */
	if (intr_stat & (REG_INT_RX | REG_INT_TX)) {
		while (ctx->data_tx_idx < ctx->tx_data_len) {
			temp = ctx->tx_data_len - ctx->data_tx_idx;
			idx = lld_spi_fill_fifo(ctx, ctx->frame_size,
						ctx->data_tx, temp);

			ctx->data_tx_idx += idx;
			if (ctx->data_tx != NULL) {
				ctx->data_tx += idx;
			}

			if ((ctx->data_tx_idx == ctx->tx_data_len) &&
			    (ctx->cb_event)) {
				lld_spi_set_tx_watermark(SPI_REG(ctx), 1);
				ctx->cb_event(INTEL_SPI_EVENT_TX_FINISHED,
						  ctx->cb_param);
			}

			if (idx < temp) {
				/* If last transfer filled FIFO full, break */
				break;
			}
		}

		while (ctx->data_rx_idx < ctx->rx_data_len) {
			rx_len = ctx->rx_data_len - ctx->data_rx_idx;
			idx = lld_spi_receive_fifo(SPI_REG(ctx),
						   ctx->frame_size,
						   ctx->data_rx, rx_len);

			ctx->data_rx_idx += idx;
			if (ctx->data_rx != NULL) {
				ctx->data_rx += idx;
			}

			/*Check if need to modify watermark for last transfer*/
			if ((ctx->rx_data_len - ctx->data_rx_idx <
			     ctx->frame_size * ctx->rx_watermark) &&
			    (ctx->rx_data_len != ctx->data_rx_idx)) {
				temp = (ctx->rx_data_len -
					ctx->data_rx_idx) /
				       ctx->frame_size;
				lld_spi_set_rx_watermark(SPI_REG(ctx), temp);
				ctx->rx_watermark = temp;
			}

			if ((ctx->data_rx_idx == ctx->rx_data_len) &&
			    (ctx->cb_event)) {
				ctx->cb_event(INTEL_SPI_EVENT_RX_FINISHED,
						  ctx->cb_param);
			}

			if (idx < rx_len) {
				/* If last transfer recevied all data in FIFO,
				 * break */
				break;
			}
		}
	}

	if ((ctx->data_rx_idx == ctx->tx_data_len) &&
	    (ctx->data_tx_idx == ctx->rx_data_len)) {
		end = true;
		event = INTEL_SPI_EVENT_COMPLETE;
		/* Wait for Data in FIFO send out while not continous */
		while (lld_spi_is_busy(SPI_REG(ctx)))
			;
	}

	if (intr_stat & REG_INT_ERROR) {
		error = true;
		event = INTEL_SPI_EVENT_DATA_LOST;
		ctx->status.data_lost = true;
	}

	if ((error || end) && (ctx->status.busy != 0)) {
		ctx->status.busy = 0;
		lld_spi_config_interrupt(SPI_REG(ctx), REG_INT_NONE);
		lld_spi_enable(SPI_REG(ctx), false);

		if (ctx->cb_event) {
			ctx->cb_event(event, ctx->cb_param);
		}
	} else {
		lld_spi_config_interrupt(SPI_REG(ctx),
				REG_INT_TX | REG_INT_RX | REG_INT_ERROR);
	}
}
