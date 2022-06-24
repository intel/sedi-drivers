/*
 * Copyright (c) 2021 Intel Corporation
 *
 * SPDX-License-Identifier:  BSD-3-Clause
 */

#include "dma_ann_1p0.h"
#include <intel/hal_dma.h>

#define DMA_RS0 0       /* root space for common memmory*/
#define DMA_RS1 1       /* root space for csme memmory*/
#define DMA_RS3 3       /* root space for IMR memory*/
#define DT_IS_IN_DRAM (1 << 0)
#define SR_IS_IN_DRAM (1 << 1)

#define DMA_WRITE_ENABLE(i) (BIT(i) | BIT(DMA_CHANNEL_NUM + i))
#define DMA_WRITE_DISABLE(i) BIT(DMA_CHANNEL_NUM + i)
#define GET_MSB(data64) ((uint32_t)(data64 >> 32))
#define GET_LSB(data64) ((uint32_t)(data64))

#undef SCATTER_GATHER_EN
#undef GET_STATUS_EN

/*driver version*/
static const intel_driver_version_t driver_version = { INTEL_DMA_API_VERSION,
						       INTEL_DMA_DRIVER_VERSION };

static intel_instance_t *dma_instance[INTEL_DMA_NUM];

typedef enum {
	INTEL_CONFIG_DMA_TRANS_TYPE = INTEL_CONFIG_DMA_CONTROL_ID_MAX,
	INTEL_CONFIG_DMA_LL_HEADER
} dma_inner_control_code;

/*!
 * DMA Transfer Type, inner usage
 */
typedef enum {
	DMA_TYPE_SINGLE,                /**< Single block mode. */
	DMA_TYPE_MULTI_CONT,            /**< Contiguous multiblock mode. */
	DMA_TYPE_MULTI_LL,              /**< Link list multiblock mode. */
	DMA_TYPE_MULTI_LL_CIRCULAR,     /**< Multiblock circular operation. */
	DMA_TYPE_MAX
} dma_transfer_type_t;

intel_driver_version_t intel_dma_get_version(void)
{
	return driver_version;
}

static void dma_set_default_channel_config(OUT channel_config_t *config)
{
	config->tf_mode = DMA_TYPE_MAX;
	config->sr_mem_type = DMA_DRAM_MEM;
	config->dt_mem_type = DMA_DRAM_MEM;
	config->sr_msb = 0;
	config->dt_msb = 0;
	config->burst_length = DMA_BURST_TRANS_LENGTH_MAX;
	config->sr_width = DMA_TRANS_WIDTH_MAX;
	config->dt_width = DMA_TRANS_WIDTH_MAX;
	config->direction = DMA_DIRECTION_MAX;
	config->handshake_device_id = 0;
	config->linked_list_header = 0;
	config->config_applied = 0;
}

/*  mask  channel interrupt */
static void mask_channel_interrupt(IN intel_instance_t *inst, IN int channel_id)
{
	struct intel_dma_context *dma =
		CONTAINER_OF(inst, struct intel_dma_context, instance);
	volatile dma_ann_1p0_regs_t *regs = dma->regs;

	regs->int_reg.mask_tfr_low = DMA_WRITE_DISABLE(channel_id);
	regs->int_reg.mask_block_low = DMA_WRITE_DISABLE(channel_id);
	regs->int_reg.mask_src_trans_low = DMA_WRITE_DISABLE(channel_id);
	regs->int_reg.mask_dst_trans_low = DMA_WRITE_DISABLE(channel_id);
	regs->int_reg.mask_err_low = DMA_WRITE_DISABLE(channel_id);
}

/* clear channel interrupt */
static void clear_channel_interrupt(IN intel_instance_t *inst, IN int channel_id)
{
	struct intel_dma_context *dma =
		CONTAINER_OF(inst, struct intel_dma_context, instance);
	volatile dma_ann_1p0_regs_t *regs = dma->regs;

	regs->int_reg.clear_tfr_low = BIT(channel_id);
	regs->int_reg.clear_block_low = BIT(channel_id);
	regs->int_reg.clear_src_trans_low = BIT(channel_id);
	regs->int_reg.clear_dst_trans_low = BIT(channel_id);
	regs->int_reg.clear_err_low = BIT(channel_id);
}

/* enable channel interrupt */
static void unmask_channel_interrupt(IN intel_instance_t *inst,
				     IN int channel_id)
{
	struct intel_dma_context *dma =
		CONTAINER_OF(inst, struct intel_dma_context, instance);
	volatile dma_ann_1p0_regs_t *regs = dma->regs;

	regs->int_reg.mask_tfr_low = DMA_WRITE_ENABLE(channel_id);
	regs->int_reg.mask_err_low = DMA_WRITE_ENABLE(channel_id);
}

intel_instance_t *intel_get_dma_instance(intel_dma_t id)
{
	if (id < 0 || id > INTEL_DMA_NUM) {
		return NULL;
	}

	return dma_instance[id];
}

void intel_dma_set_instance(intel_instance_t *inst, IN intel_dma_t id)
{
	dma_instance[id] = inst;
}

int32_t intel_dma_init(IN intel_instance_t *inst, IN int channel_id,
		       IN intel_dma_event_cb_t cb, INOUT void *param)
{
	DBG_CHECK(channel_id < DMA_CHANNEL_NUM, INTEL_DRIVER_ERROR_PARAMETER);

	struct intel_dma_context *dma =
		CONTAINER_OF(inst, struct intel_dma_context, instance);
	dma_ann_1p0_regs_t *base = (dma_ann_1p0_regs_t *)(inst->base_addr);

	dma->regs = base;

	/* init default config context */
	channel_config_t *config = &(dma->channel_config[channel_id]);

	dma_set_default_channel_config(config);
	/*add callback*/
	dma->cb_event[channel_id] = cb;
	dma->cb_param[channel_id] = param;

	mask_channel_interrupt(inst, channel_id);
	clear_channel_interrupt(inst, channel_id);
	dma->status[channel_id].busy = 0;
	dma->status[channel_id].bus_error = 0;

	return INTEL_DRIVER_OK;
}

int32_t intel_dma_uninit(IN intel_instance_t *inst, IN int channel_id)
{
	DBG_CHECK(channel_id < DMA_CHANNEL_NUM, INTEL_DRIVER_ERROR_PARAMETER);

	struct intel_dma_context *dma =
		CONTAINER_OF(inst, struct intel_dma_context, instance);

	dma->cb_event[channel_id] = NULL;

	mask_channel_interrupt(inst, channel_id);
	dma->status[channel_id].busy = 0;
	dma->status[channel_id].bus_error = 0;
	return INTEL_DRIVER_OK;
}

static int32_t intel_dma_control_aux(IN intel_instance_t *inst, IN int channel_id,
				     IN uint32_t control_id, IN uint32_t arg)
{
	DBG_CHECK(channel_id < DMA_CHANNEL_NUM, INTEL_DRIVER_ERROR_PARAMETER);

	struct intel_dma_context *dma =
		CONTAINER_OF(inst, struct intel_dma_context, instance);

	channel_config_t *config = &(dma->channel_config[channel_id]);

	switch (control_id) {
	case INTEL_CONFIG_DMA_TRANS_TYPE:
		DBG_CHECK(arg < DMA_TYPE_MAX, INTEL_DRIVER_ERROR_PARAMETER);
		config->tf_mode = arg;
		break;
	case INTEL_CONFIG_DMA_SR_MEM_TYPE:
		DBG_CHECK(arg < DMA_MEM_TYPE_MAX, INTEL_DRIVER_ERROR_PARAMETER);
		config->sr_mem_type = arg;
		break;
	case INTEL_CONFIG_DMA_DT_MEM_TYPE:
		DBG_CHECK(arg < DMA_MEM_TYPE_MAX, INTEL_DRIVER_ERROR_PARAMETER);
		config->dt_mem_type = arg;
		break;
	case INTEL_CONFIG_DMA_LL_SR_MSB:
		config->sr_msb = arg;
		break;
	case INTEL_CONFIG_DMA_LL_DT_MSB:
		config->dt_msb = arg;
		break;
	case INTEL_CONFIG_DMA_DIRECTION:
		DBG_CHECK(arg < DMA_DIRECTION_MAX, INTEL_DRIVER_ERROR_PARAMETER);
		config->direction = arg;
		break;
	case INTEL_CONFIG_DMA_BURST_LENGTH:
		DBG_CHECK(arg < DMA_BURST_TRANS_LENGTH_MAX,
			  INTEL_DRIVER_ERROR_PARAMETER);
		config->burst_length = arg;
		break;
	case INTEL_CONFIG_DMA_SR_TRANS_WIDTH:
		DBG_CHECK(arg < DMA_TRANS_WIDTH_MAX,
			  INTEL_DRIVER_ERROR_PARAMETER);
		config->sr_width = arg;
		break;
	case INTEL_CONFIG_DMA_DT_TRANS_WIDTH:
		DBG_CHECK(arg < DMA_TRANS_WIDTH_MAX,
			  INTEL_DRIVER_ERROR_PARAMETER);
		config->dt_width = arg;
		break;
	case INTEL_CONFIG_DMA_HS_DEVICE_ID:
		config->handshake_device_id = arg;
		break;
	case INTEL_CONFIG_DMA_HS_DEVICE_ID_PER_DIR:
		config->peripheral_direction = arg;
		break;
	case INTEL_CONFIG_DMA_HS_POLARITY:
		DBG_CHECK(arg < DMA_HS_PER_RTX_MAX,
			  INTEL_DRIVER_ERROR_PARAMETER);
		config->handshake_polarity = arg;
		break;
	case INTEL_CONFIG_DMA_LL_HEADER:
		DBG_CHECK((arg), INTEL_DRIVER_ERROR_PARAMETER);
		config->linked_list_header = arg;
		break;
	default:
		return INTEL_DRIVER_ERROR_PARAMETER;
	}
	config->config_applied = 0;
	return INTEL_DRIVER_OK;
}

int32_t intel_dma_control(IN intel_instance_t *inst, IN int channel_id,
			  IN uint32_t control_id, IN uint32_t arg)
{
	DBG_CHECK(channel_id < DMA_CHANNEL_NUM, INTEL_DRIVER_ERROR_PARAMETER);
	DBG_CHECK(control_id < INTEL_CONFIG_DMA_CONTROL_ID_MAX,
		  INTEL_DRIVER_ERROR_PARAMETER);

	return intel_dma_control_aux(inst, channel_id, control_id, arg);
}

int intel_dma_get_status(IN intel_instance_t *inst, IN int channel_id,
			 OUT intel_dma_status_t *status)
{
	DBG_CHECK(channel_id < DMA_CHANNEL_NUM, INTEL_DRIVER_ERROR_PARAMETER);

	struct intel_dma_context *dma =
		CONTAINER_OF(inst, struct intel_dma_context, instance);

	*status = dma->status[channel_id];
	return INTEL_DRIVER_OK;
}

/*config misc and other registers, as an adding for ctrl register*/
static int32_t dma_apply_other_regs(volatile dma_misc_regs_t *misc_regs,
				    volatile dma_chan_reg_t *chan_regs,
				    channel_config_t *config, int channel_id)
{
	uint8_t sr_rs = DMA_RS0;
	uint8_t dt_rs = DMA_RS0;
	uint8_t dma_mem_trans_mode = 0;

	/* peripheral related registers*/
	if (config->direction != DMA_MEMORY_TO_MEMORY) {
		/* config check */
		DBG_CHECK(config->peripheral_direction < DMA_HS_PER_RTX_MAX,
			  INTEL_DRIVER_ERROR_PARAMETER);
		DBG_CHECK(config->handshake_polarity < DMA_HS_POLARITY_MAX,
			  INTEL_DRIVER_ERROR_PARAMETER);
		/* hardware handshake only*/
		/* select peripheral rx/tx to link up with dma*/
		SET_BITS(misc_regs->dma_xbar_sel[channel_id], RX_TX_LOC,
			 RX_TX_LEN, config->peripheral_direction);
		/* select peripheral device to connect with dma req wire*/
		SET_BITS(misc_regs->dma_xbar_sel[channel_id], DEVID_LOC,
			 DEVID_LEN, config->handshake_device_id);
		/* set handshaking polarity */
		SET_BITS(chan_regs->cfg_low, DST_HS_POL_LOC, DST_HS_POL_LEN,
			 config->handshake_polarity);
		SET_BITS(chan_regs->cfg_low, SRC_HS_POL_LOC, SRC_HS_POL_LEN,
			 config->handshake_polarity);
		SET_BITS(chan_regs->cfg_low, HSHAKE_NP_WR_LOC, HSHAKE_NP_WR_LEN,
			 1U);
		/* fill channel id to DST/SRC_PER reg*/
		SET_BITS(chan_regs->cfg_high, SRC_PER_LOC, SRC_PER_LEN,
			 channel_id);
		SET_BITS(chan_regs->cfg_high, DST_PER_LOC, DST_PER_LEN,
			 channel_id);
	}
	/* memory type related registers config*/
	/* source is memory*/
	if ((config->direction == DMA_MEMORY_TO_PERIPHERAL) ||
	    (config->direction == DMA_MEMORY_TO_MEMORY)) {
		DBG_CHECK(config->sr_mem_type < DMA_MEM_TYPE_MAX,
			  INTEL_DRIVER_ERROR_PARAMETER);
		if (config->sr_mem_type == DMA_UMA_MEM) {
			sr_rs = DMA_RS3;
		}
		if (config->sr_mem_type != DMA_SRAM_MEM) {
			dma_mem_trans_mode |= SR_IS_IN_DRAM;
		}
	}
	/* destination is memory*/
	if ((config->direction == DMA_PERIPHERAL_TO_MEMORY) ||
	    (config->direction == DMA_MEMORY_TO_MEMORY)) {
		DBG_CHECK(config->dt_mem_type < DMA_MEM_TYPE_MAX,
			  INTEL_DRIVER_ERROR_PARAMETER);
		if (config->dt_mem_type == DMA_UMA_MEM) {
			dt_rs = DMA_RS3;
		}
		if (config->dt_mem_type != DMA_SRAM_MEM) {
			dma_mem_trans_mode |= DT_IS_IN_DRAM;
		}
	}
	/*set root space and memory type*/
	SET_BITS(misc_regs->dma_ctl_ch[channel_id], WR_RS_LOC, WR_RS_LEN,
		 dt_rs);
	SET_BITS(misc_regs->dma_ctl_ch[channel_id], RD_RS_LOC, RD_RS_LEN,
		 sr_rs);
	SET_BITS(misc_regs->dma_ctl_ch[channel_id], M2M_TYPE_LOC, M2M_TYPE_LEN,
		 dma_mem_trans_mode);

	/* fill higher 32bit of 64bit addr */
	misc_regs->iosf_addr_fillin_dma_ch[channel_id] = config->sr_msb;
	misc_regs->iosf_dest_addr_fillin_dma_ch[channel_id] = config->dt_msb;
	return INTEL_DRIVER_OK;
}

static int32_t dma_channel_apply_config(IN intel_instance_t *inst,
					IN int channel_id)
{
	int32_t ret;

	struct intel_dma_context *dma =
		CONTAINER_OF(inst, struct intel_dma_context, instance);

	channel_config_t *config = &(dma->channel_config[channel_id]);
	volatile dma_ann_1p0_regs_t *regs = dma->regs;
	volatile dma_chan_reg_t *chan_regs = &(regs->chan_reg[channel_id]);
	volatile dma_misc_regs_t *misc_regs = &(regs->pse_misc_reg);

	/*return if no need to config*/
	if (config->config_applied == 1) {
		return INTEL_DRIVER_OK;
	}

	DBG_CHECK(config->tf_mode < DMA_TYPE_MAX, INTEL_DRIVER_ERROR_PARAMETER);
	DBG_CHECK(config->direction < DMA_DIRECTION_MAX,
		  INTEL_DRIVER_ERROR_PARAMETER);
	misc_regs->dma_regaccess_chid = channel_id;
	if (config->tf_mode == DMA_TYPE_SINGLE) {
		/* single block mode config, mainly config ctrl_low reg*/
		DBG_CHECK(config->burst_length < DMA_BURST_TRANS_LENGTH_MAX,
			  INTEL_DRIVER_ERROR_PARAMETER);
		DBG_CHECK(config->sr_width < DMA_TRANS_WIDTH_MAX,
			  INTEL_DRIVER_ERROR_PARAMETER);
		DBG_CHECK(config->dt_width < DMA_TRANS_WIDTH_MAX,
			  INTEL_DRIVER_ERROR_PARAMETER);
		DBG_CHECK(config->peripheral_direction < DMA_HS_PER_RTX_MAX,
			  INTEL_DRIVER_ERROR_PARAMETER);
		/*set dest and src burst size */
		SET_BITS(chan_regs->ctrl_low, DEST_MSIZE_LOC, SRC_MSIZE_LEN,
			 config->burst_length);
		SET_BITS(chan_regs->ctrl_low, SRC_MSIZE_LOC, SRC_MSIZE_LEN,
			 config->burst_length);
		/*source and destination transfer width */
		SET_BITS(chan_regs->ctrl_low, DST_TR_WIDTH_LOC,
			 DST_TR_WIDTH_LEN, config->dt_width);
		SET_BITS(chan_regs->ctrl_low, SRC_TR_WIDTH_LOC,
			 SRC_TR_WIDTH_LEN, config->sr_width);

		/*transfer  direction */
		SET_BITS(chan_regs->ctrl_low, TT_FC_LOC, TT_FC_LEN,
			 config->direction);

		/* Set increment*/
		switch (config->direction) {
		case DMA_PERIPHERAL_TO_MEMORY:
			SET_BITS(chan_regs->ctrl_low, SINC_LOC, SINC_LEN,
				 DMA_INCREMENT_NO_CHANGE);
			SET_BITS(chan_regs->ctrl_low, DINC_LOC, DINC_LEN,
				 DMA_INCREMENT_INC);
			break;
		case DMA_MEMORY_TO_MEMORY:
			SET_BITS(chan_regs->ctrl_low, SINC_LOC, SINC_LEN,
				 DMA_INCREMENT_INC);
			SET_BITS(chan_regs->ctrl_low, DINC_LOC, DINC_LEN,
				 DMA_INCREMENT_INC);
			break;
		case DMA_MEMORY_TO_PERIPHERAL:
			SET_BITS(chan_regs->ctrl_low, DINC_LOC, DINC_LEN,
				 DMA_INCREMENT_NO_CHANGE);
			SET_BITS(chan_regs->ctrl_low, SINC_LOC, SINC_LEN,
				 DMA_INCREMENT_INC);
			break;
		case DMA_PERIPHERAL_TO_PERIPHERAL:
			SET_BITS(chan_regs->ctrl_low, DINC_LOC, DINC_LEN,
				 DMA_INCREMENT_NO_CHANGE);
			SET_BITS(chan_regs->ctrl_low, SINC_LOC, SINC_LEN,
				 DMA_INCREMENT_NO_CHANGE);
			break;
		}
		chan_regs->llp_low = 0;
	} else if (config->tf_mode == DMA_TYPE_MULTI_LL) {
		DBG_CHECK(config->linked_list_header != NULL,
			  INTEL_DRIVER_ERROR_PARAMETER);
		chan_regs->llp_low = config->linked_list_header;
	}

	/*config misc and other registers, as an adding for ctrl register*/
	ret = dma_apply_other_regs(misc_regs, chan_regs, config, channel_id);
	if (ret != INTEL_DRIVER_OK) {
		return ret;
	}

	config->config_applied = 1;
	return INTEL_DRIVER_OK;
}

int dma_fill_linkedlist(INOUT volatile dma_linked_list_item_t *ll_p,
			IN uint32_t src_addr, IN uint32_t dst_addr,
			IN uint32_t block_size, uint32_t ctrl_low,
			IN uint32_t ll_p_next)
{
	DBG_CHECK(ll_p != NULL, INTEL_DRIVER_ERROR_PARAMETER);
	DBG_CHECK(src_addr != 0, INTEL_DRIVER_ERROR_PARAMETER);
	DBG_CHECK(dst_addr != 0, INTEL_DRIVER_ERROR_PARAMETER);
	DBG_CHECK(block_size <= DMA_MAX_BLOCK_SIZE,
		  INTEL_DRIVER_ERROR_PARAMETER);

	ll_p->src_addr = src_addr;
	ll_p->dst_addr = dst_addr;
	ll_p->ctrl_low.raw = ctrl_low;
	SET_BITS(ll_p->ctrl_high.raw, BLOCK_TS_LOC, BLOCK_TS_LEN, block_size);
	ll_p->next_ll_p = ll_p_next;
	return INTEL_DRIVER_OK;
}

#ifdef SCATTER_GATHER_EN
int dma_fill_sc_linkedlist(INOUT volatile dma_linked_list_item_t *llp,
			   IN uint8_t count, IN uint32_t ctrl_reg_low,
			   IN sc_attr_t *attr)
{
	DBG_CHECK(llp != NULL, INTEL_DRIVER_ERROR_PARAMETER);
	DBG_CHECK(attr != NULL, INTEL_DRIVER_ERROR_PARAMETER);
	DBG_CHECK(count > 0, INTEL_DRIVER_ERROR_PARAMETER);

	int ret;
	dma_linked_list_item_t *ll_tmp = (dma_linked_list_item_t *)llp;
	uint32_t src = attr->src_addr;
	uint32_t dst = attr->dst_addr;

	for (int i = 0; i < count - 1; i++) {
		ret =
			dma_fill_linkedlist(ll_tmp + i, src, dst, attr->block_size,
					    ctrl_reg_low, ll_tmp + i + 1);
		if (ret != INTEL_DRIVER_OK) {
			return ret;
		}
		if (attr->is_scatter) {
			if (attr->need_reload == 0) {
				src += attr->block_size;
			}
			dst += (attr->interval);
		} else {
			if (attr->need_reload == 0) {
				dst += attr->block_size;
			}
			src += (attr->interval);
		}
	}
	ret = dma_fill_linkedlist(ll_tmp + count - 1, src, dst,
				  attr->block_size, ctrl_reg_low, NULL);
	return ret;
}
#endif

static int32_t intel_dma_start_transfer_aux(IN intel_instance_t *inst,
					    IN int channel_id,
					    IN uint32_t sr_addr,
					    IN uint32_t dest_addr,
					    IN uint32_t length)
{
	struct intel_dma_context *dma =
		CONTAINER_OF(inst, struct intel_dma_context, instance);
	int32_t ret;
	volatile dma_ann_1p0_regs_t *regs = dma->regs;
	volatile dma_chan_reg_t *chan_regs = &(regs->chan_reg[channel_id]);
	channel_config_t *config = &(dma->channel_config[channel_id]);

	if (regs->misc_reg.chan_en_low & BIT(channel_id)) {
		return INTEL_DRIVER_ERROR_BUSY;
	}
	/* channel config*/
	ret = dma_channel_apply_config(inst, channel_id);
	if (ret != INTEL_DRIVER_OK) {
		dma->status[channel_id].busy = 0;
		return ret;
	}
	if (config->tf_mode == DMA_TYPE_SINGLE) {
		chan_regs->sar_low = sr_addr;
		chan_regs->dar_low = dest_addr;
		SET_BITS(chan_regs->ctrl_high, BLOCK_TS_LOC, BLOCK_TS_LEN,
			 length);
		SET_BITS(chan_regs->ctrl_low, INT_EN_LOC, INT_EN_LEN, 1);
	} else if (config->tf_mode == DMA_TYPE_MULTI_LL) {
		SET_BITS(chan_regs->ctrl_low, LLP_DST_EN_LOC, LLP_DST_EN_LEN,
			 1);
		SET_BITS(chan_regs->ctrl_low, LLP_SRC_EN_LOC, LLP_SRC_EN_LEN,
			 1);
	}

	dma_set_default_channel_config(config);
	/* enable interrupt */
	unmask_channel_interrupt(inst, channel_id);

	/* enable channel*/
	regs->misc_reg.cfg_low = 1;
	SET_BITS(chan_regs->cfg_low, CH_DRAIN_LOC, CH_DRAIN_LEN, 0);
	SET_BITS(chan_regs->cfg_low, CH_SUSP_LOC, CH_SUSP_LEN, 0);
	regs->misc_reg.chan_en_low = DMA_WRITE_ENABLE(channel_id);
	return INTEL_DRIVER_OK;
}

int32_t intel_dma_start_transfer(IN intel_instance_t *inst, IN int channel_id,
				 IN uint64_t sr_addr, IN uint64_t dest_addr,
				 IN uint32_t length)
{
	DBG_CHECK(channel_id < DMA_CHANNEL_NUM, INTEL_DRIVER_ERROR_PARAMETER);
	DBG_CHECK(sr_addr > 0, INTEL_DRIVER_ERROR_PARAMETER);
	DBG_CHECK(dest_addr > 0, INTEL_DRIVER_ERROR_PARAMETER);
	DBG_CHECK((length <= DMA_MAX_BLOCK_SIZE) && (length > 0),
		  INTEL_DRIVER_ERROR_PARAMETER);

	struct intel_dma_context *dma =
		CONTAINER_OF(inst, struct intel_dma_context, instance);

	if (dma->status[channel_id].busy == 1) {
		return INTEL_DRIVER_ERROR_BUSY;
	}
	dma->status[channel_id].busy = 1;
	dma->status[channel_id].bus_error = 0;

	intel_dma_control_aux(inst, channel_id, INTEL_CONFIG_DMA_TRANS_TYPE,
			      DMA_TYPE_SINGLE);

	/* pass higher 32 bit of address*/
	intel_dma_control_aux(inst, channel_id, INTEL_CONFIG_DMA_LL_SR_MSB,
			      GET_MSB(sr_addr));

	intel_dma_control_aux(inst, channel_id, INTEL_CONFIG_DMA_LL_DT_MSB,
			      GET_MSB(dest_addr));

	return intel_dma_start_transfer_aux(inst, channel_id,
					    GET_LSB(sr_addr), GET_LSB(dest_addr),
					    length);
}

int32_t intel_dma_start_ll_transfer(IN intel_instance_t *inst, IN int channel_id,
				    IN dma_linked_list_item_t *linkedlist_header)
{
	DBG_CHECK(channel_id < DMA_CHANNEL_NUM, INTEL_DRIVER_ERROR_PARAMETER);
	DBG_CHECK(linkedlist_header != NULL, INTEL_DRIVER_ERROR_PARAMETER);

	int32_t ret;
	struct intel_dma_context *dma =
		CONTAINER_OF(inst, struct intel_dma_context, instance);

	if (dma->status[channel_id].busy == 1) {
		return INTEL_DRIVER_ERROR_BUSY;
	}
	dma->status[channel_id].busy = 1;
	dma->status[channel_id].bus_error = 0;
	/* channel config*/
	intel_dma_control_aux(inst, channel_id, INTEL_CONFIG_DMA_TRANS_TYPE,
			      DMA_TYPE_MULTI_LL);
	intel_dma_control_aux(inst, channel_id, INTEL_CONFIG_DMA_LL_HEADER,
			      (uintptr_t)linkedlist_header);

	ret = intel_dma_start_transfer_aux(inst, channel_id, 0, 0, 0);

	return ret;
}

static void dma_transfer_post(IN intel_instance_t *inst, IN int channel_id)
{
	struct intel_dma_context *dma =
		CONTAINER_OF(inst, struct intel_dma_context, instance);
	channel_config_t *config = &(dma->channel_config[channel_id]);
	volatile dma_ann_1p0_regs_t *regs = dma->regs;
	volatile dma_chan_reg_t *chan_regs = &(regs->chan_reg[channel_id]);

	/* get status*/
	dma->done_byte[channel_id] =
		GET_BITS(chan_regs->ctrl_high, BLOCK_TS_LOC, BLOCK_TS_LEN);
	dma->next_llp[channel_id] = chan_regs->llp_low;

	/* disable dma channel*/
	regs->misc_reg.chan_en_low = DMA_WRITE_DISABLE(channel_id);
	while (regs->misc_reg.chan_en_low & BIT(channel_id)) {
	}

	/* mask and clear interrupt*/
	clear_channel_interrupt(inst, channel_id);
	mask_channel_interrupt(inst, channel_id);

	dma_set_default_channel_config(config);
	dma->status[channel_id].busy = 0;
}

/* Polling mode is only used in single-block mode */
int32_t intel_dma_start_transfer_polling(IN intel_instance_t *inst,
					 IN int channel_id, IN uint64_t sr_addr,
					 IN uint64_t dest_addr,
					 IN uint32_t length)
{
	DBG_CHECK(channel_id < DMA_CHANNEL_NUM, INTEL_DRIVER_ERROR_PARAMETER);
	DBG_CHECK(length <= DMA_MAX_BLOCK_SIZE, INTEL_DRIVER_ERROR_PARAMETER);

	struct intel_dma_context *dma =
		CONTAINER_OF(inst, struct intel_dma_context, instance);
	uint32_t ret;
	volatile dma_ann_1p0_regs_t *regs = dma->regs;
	volatile dma_chan_reg_t *chan_regs = &(regs->chan_reg[channel_id]);
	intel_dma_event_cb_t cb = dma->cb_event[channel_id];
	void *usr_param = dma->cb_param[channel_id];

	if (dma->status[channel_id].busy == 1) {
		return INTEL_DRIVER_ERROR_BUSY;
	}
	dma->status[channel_id].busy = 1;
	dma->status[channel_id].bus_error = 0;

	intel_dma_control_aux(inst, channel_id, INTEL_CONFIG_DMA_TRANS_TYPE,
			      DMA_TYPE_SINGLE);

	/* pass higher 32 bit of address*/
	intel_dma_control_aux(inst, channel_id, INTEL_CONFIG_DMA_LL_SR_MSB,
			      GET_MSB(sr_addr));
	intel_dma_control_aux(inst, channel_id, INTEL_CONFIG_DMA_LL_DT_MSB,
			      GET_MSB(dest_addr));

	ret = dma_channel_apply_config(inst, channel_id);
	if (ret != INTEL_DRIVER_OK) {
		dma->status[channel_id].busy = 0;
		return ret;
	}

	chan_regs->sar_low = GET_LSB(sr_addr);
	chan_regs->dar_low = GET_LSB(dest_addr);
	SET_BITS(chan_regs->ctrl_high, BLOCK_TS_LOC, BLOCK_TS_LEN, length);

	/* disable and clear interrupt */
	mask_channel_interrupt(inst, channel_id);
	clear_channel_interrupt(inst, channel_id);

	/* enable channel*/
	regs->misc_reg.cfg_low = 1;
	regs->misc_reg.chan_en_low = DMA_WRITE_ENABLE(channel_id);

	while (1) {
		if (regs->misc_reg.chan_en_low & BIT(channel_id)) {
			continue;
		}
		if (regs->int_reg.raw_tfr_low & BIT(channel_id)) {
			dma_transfer_post(inst, channel_id);
			if (cb != NULL) {
				cb(inst, channel_id,
				   INTEL_DMA_EVENT_TRANSFER_DONE, usr_param);
			}
			return INTEL_DRIVER_OK;
		}
		if (regs->int_reg.raw_err_low & BIT(channel_id)) {
			dma_transfer_post(inst, channel_id);
			dma->status[channel_id].bus_error = 1;
			if (cb != NULL) {
				cb(inst, channel_id,
				   INTEL_DMA_EVENT_BUS_ERROR, usr_param);
			}
			return INTEL_DRIVER_ERROR_TRANSFER;
		}
	}
}

int32_t intel_dma_abort_transfer(IN intel_instance_t *inst, IN int channel_id)
{
	DBG_CHECK(channel_id < DMA_CHANNEL_NUM, INTEL_DRIVER_ERROR_PARAMETER);

	struct intel_dma_context *dma =
		CONTAINER_OF(inst, struct intel_dma_context, instance);
	volatile dma_ann_1p0_regs_t *regs = dma->regs;
	volatile dma_chan_reg_t *chan_regs = &(regs->chan_reg[channel_id]);

	if (dma->status[channel_id].busy == 0) {
		return INTEL_DRIVER_OK;
	}

	SET_BITS(chan_regs->cfg_low, CH_SUSP_LOC, CH_SUSP_LEN, 1);
	SET_BITS(chan_regs->cfg_low, CH_DRAIN_LOC, CH_DRAIN_LEN, 1);

	while ((chan_regs->cfg_low & BIT(FIFO_EMPTY_LOC)) == 0) {
	}

	dma_transfer_post(inst, channel_id);
	return INTEL_DRIVER_OK;
}

#ifdef GET_STATUS_EN
int32_t intel_dma_get_done_status(IN intel_instance_t *inst, IN int channel_id,
				  OUT uint32_t *done_bytes, OUT dma_linked_list_item_t **next_llp)
{
	DBG_CHECK(channel_id < DMA_CHANNEL_NUM, INTEL_DRIVER_ERROR_PARAMETER);

	struct intel_dma_context *dma =
		CONTAINER_OF(inst, struct intel_dma_context, instance);

	if (dma->status[channel_id].busy == 1) {
		return INTEL_DRIVER_ERROR_BUSY;
	}

	if (done_bytes) {
		*done_bytes = dma->done_byte[channel_id];
	}
	if (next_llp) {
		*next_llp = dma->next_llp[channel_id];
	}
	return INTEL_DRIVER_OK;
}
#endif

void intel_dma_isr_handler(IN intel_instance_t *inst)
{
	struct intel_dma_context *dma =
		CONTAINER_OF(inst, struct intel_dma_context, instance);
	volatile dma_ann_1p0_regs_t *regs = dma->regs;
	intel_dma_event_cb_t cb;
	void *usr_param;
	uint32_t tfr_status = regs->int_reg.status_tfr_low;
	uint32_t err_status = regs->int_reg.status_err_low;

	for (int channel_id = 0; channel_id < DMA_CHANNEL_NUM; channel_id++) {
		cb = dma->cb_event[channel_id];
		usr_param = dma->cb_param[channel_id];
		if (tfr_status & BIT(channel_id)) {
			dma_transfer_post(inst, channel_id);
			if (cb != NULL) {
				cb(inst, channel_id,
				   INTEL_DMA_EVENT_TRANSFER_DONE, usr_param);
			}
		}
		if (err_status & BIT(channel_id)) {
			dma_transfer_post(inst, channel_id);
			dma->status[channel_id].bus_error = 1;
			if (cb != NULL) {
				cb(inst, channel_id,
				   INTEL_DMA_EVENT_BUS_ERROR, usr_param);
			}
		}
	}
}
