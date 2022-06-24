/*
 * Copyright (c) 2021 Intel Corporation
 *
 * SPDX-License-Identifier:  BSD-3-Clause
 */

#include "emmc_5p1.h"
#include <intel/hal_emmc.h>

#define EMMC_CMD_WAIT_TIMEOUT_US 1000
#define EMMC_CMD_CMPLETE_TIMEOUT_US 3000
#define EMMC_SDMA_BOUNDARY 0x7
#define EMMC_RCA_ADDRESS 0x2

static void enable_interrupts(IN intel_instance_t *inst)
{
	struct intel_emmc_context *emmc =
	    CONTAINER_OF(inst, struct intel_emmc_context, instance);
	volatile emmc_5p1_regs_t *regs = emmc->regs;

	regs->normal_int_stat_en = EMMC_NORMAL_INTR_MASK;
	regs->err_int_stat_en = EMMC_ERROR_INTR_MASK;
	regs->normal_int_signal_en = EMMC_NORMAL_INTR_MASK;
	regs->err_int_signal_en = EMMC_ERROR_INTR_MASK;

	/* Configure Timeout Control register: TMCLK * 2^27 Set time out counter
	 * to maximum
	 */
	regs->timeout_ctrl = EMMC_HOST_MAX_TIMEOUT;
}

static void clear_interrupts(IN intel_instance_t *inst)
{
	struct intel_emmc_context *emmc =
	    CONTAINER_OF(inst, struct intel_emmc_context, instance);
	volatile emmc_5p1_regs_t *regs = emmc->regs;

	regs->normal_int_stat = EMMC_NORMAL_INTR_MASK_CLR;
	regs->err_int_stat = EMMC_ERROR_INTR_MASK;
}

static void emmc_set_power(IN intel_instance_t *inst)
{
	struct intel_emmc_context *emmc =
	    CONTAINER_OF(inst, struct intel_emmc_context, instance);
	volatile emmc_5p1_regs_t *regs = emmc->regs;

	/* 1.8v voltage select*/
	regs->power_ctrl = EMMC_POWER_CTRL_SD_BUS_VOLT_SEL << 1;
	/* Set Bus Power */
	regs->power_ctrl |= EMMC_POWER_CTRL_SD_BUS_POWER;
}

static bool emmc_disable_clock(IN intel_instance_t *inst)
{
	struct intel_emmc_context *emmc =
	    CONTAINER_OF(inst, struct intel_emmc_context, instance);
	volatile emmc_5p1_regs_t *regs = emmc->regs;

	if (regs->present_state & EMMC_HOST_PSTATE_REG_CMD_INHIBIT) {
		return false;
	}
	if (regs->present_state & EMMC_HOST_PSTATE_REG_DAT_INHIBIT) {
		return false;
	}

	regs->clock_ctrl &= ~EMMC_HOST_INTERNAL_CLOCK_EN;
	regs->clock_ctrl &= ~EMMC_HOST_SD_CLOCK_EN;

	while ((regs->clock_ctrl & EMMC_HOST_SD_CLOCK_EN) != 0)
		;

	return true;
}

static bool emmc_enable_clock(IN intel_instance_t *inst)
{
	struct intel_emmc_context *emmc =
	    CONTAINER_OF(inst, struct intel_emmc_context, instance);
	volatile emmc_5p1_regs_t *regs = emmc->regs;

	regs->clock_ctrl |= EMMC_HOST_INTERNAL_CLOCK_EN;
	/* Wait for the stable Internal Clock. */
	while ((regs->clock_ctrl & EMMC_HOST_INTERNAL_CLOCK_STABLE) == 0)
		;

	/* Enable SD Clock*/
	regs->clock_ctrl |= EMMC_HOST_SD_CLOCK_EN;
	while ((regs->clock_ctrl & EMMC_HOST_SD_CLOCK_EN) == 0)
		;

	return true;
}

static bool emmc_clock_set(IN intel_instance_t *inst, float freq)
{
	struct intel_emmc_context *emmc =
	    CONTAINER_OF(inst, struct intel_emmc_context, instance);
	volatile emmc_5p1_regs_t *regs = emmc->regs;
	uint8_t base_freq;
	int clock_divider;
	bool ret;

	ret = emmc_disable_clock(inst);
	if (!ret) {
		return false;
	}

	base_freq = regs->capabilities >> 8;

	clock_divider = (int)(base_freq / (freq * 2));

	SET_BITS16(regs->clock_ctrl, EMMC_CLOCK_CONTROL_SDCLCK_FREQ_SEL_LOC,
		   EMMC_CLOCK_CONTROL_SDCLCK_FREQ_SEL_LEN, clock_divider);
	SET_BITS16(
	    regs->clock_ctrl, EMMC_CLOCK_CONTROL_SDCLCK_FREQ_SEL_UPPER_LOC,
	    EMMC_CLOCK_CONTROL_SDCLCK_FREQ_SEL_UPPER_LEN, clock_divider >> 8);
	emmc_enable_clock(inst);
	return true;
}

static void set_uhs_mode(IN intel_instance_t *inst, uint16_t mode)
{
	struct intel_emmc_context *emmc =
	    CONTAINER_OF(inst, struct intel_emmc_context, instance);
	volatile emmc_5p1_regs_t *regs = emmc->regs;

	regs->host_ctrl2 |= EMMC_HOST_CTRL2_1P8V_SIG_EN
			    << EMMC_HOST_CTRL2_1P8V_SIG_LOC;
	SET_BITS16(regs->host_ctrl2, EMMC_HOST_CTRL2_UHS_MODE_SEL_LOC,
		EMMC_HOST_CTRL2_UHS_MODE_SEL_LEN, mode);

	emmc_clock_set(inst, EMMC_HOST_CLK_FREQ_50M);
}

static bool poll_cmd_complete(IN intel_instance_t *inst)
{
	struct intel_emmc_context *emmc =
	    CONTAINER_OF(inst, struct intel_emmc_context, instance);
	volatile emmc_5p1_regs_t *regs = emmc->regs;

	bool ret = false;
	int32_t retry = EMMC_CMD_COMPLETE_RETRY;

	while (retry > 0) {
		if (regs->normal_int_stat & EMMC_HOST_CMD_COMPLETE) {
			regs->normal_int_stat = EMMC_HOST_CMD_COMPLETE;
			ret = true;
			break;
		}

		DELAY(EMMC_CMD_CMPLETE_TIMEOUT_US);
		retry--;
	}
	return ret;
}

void emmc_host_sw_reset(IN intel_instance_t *inst, emmc_host_sw_reset_t reset)
{
	struct intel_emmc_context *emmc =
	    CONTAINER_OF(inst, struct intel_emmc_context, instance);
	volatile emmc_5p1_regs_t *regs = emmc->regs;

	if (reset == EMMC_SW_RESET_DATA_LINE) {
		regs->sw_reset = EMMC_SW_RESET_REG_DATA;
	} else if (reset == EMMC_SW_RESET_CMD_LINE) {
		regs->sw_reset = EMMC_SW_RESET_REG_CMD;
	} else if (reset == EMMC_SW_RESET_ALL) {
		regs->sw_reset = EMMC_SW_RESET_REG_ALL;
	}

	while (regs->sw_reset != 0)
		;
}

static int32_t emmc_host_send_cmd(IN intel_instance_t *inst,
				  IN emmc_host_cmd_config_t *config)
{
	struct intel_emmc_context *emmc =
	    CONTAINER_OF(inst, struct intel_emmc_context, instance);
	volatile emmc_5p1_regs_t *regs = emmc->regs;
	uint16_t cmd_reg;

	/* Check if CMD line is available */
	if (regs->present_state & EMMC_HOST_PSTATE_REG_CMD_INHIBIT) {
		return INTEL_DRIVER_ERROR_BUSY;
	}

	regs->argument = config->cmd_arg;

	cmd_reg = config->cmd_idx << EMMC_CMD_INDEX_LOC |
		  config->cmd_type << EMMC_CMD_TYPE_LOC |
		  config->data_present << EMMC_CMD_DATA_PRESENT_LOC |
		  config->idx_check_en << EMMC_CMD_IDX_CHECK_EN_LOC |
		  config->crc_check_en << EMMC_CMD_CRC_CHECK_EN_LOC |
		  config->resp_type << EMMC_CMD_RESP_TYPE_LOC;
	regs->cmd = cmd_reg;

	while (!regs->err_int_stat && !(poll_cmd_complete(inst)))
		;

	if (regs->err_int_stat) {
		if (regs->err_int_stat & EMMC_HOST_CMD_TIMEOUT_ERR) {
			regs->err_int_stat = EMMC_HOST_CMD_TIMEOUT_ERR;
		}
		if (regs->err_int_stat & EMMC_HOST_CMD_CRC_ERR) {
			regs->err_int_stat = EMMC_HOST_CMD_CRC_ERR;
		}
		if (regs->err_int_stat & EMMC_HOST_CMD_END_BIT_ERR) {
			regs->err_int_stat = EMMC_HOST_CMD_END_BIT_ERR;
		}
		if (regs->err_int_stat & EMMC_HOST_CMD_IDX_ERR) {
			regs->err_int_stat = EMMC_HOST_CMD_IDX_ERR;
		}
		return INTEL_DRIVER_ERROR;
	}

	return INTEL_DRIVER_OK;
}

static void emmc_host_get_response(IN intel_instance_t *inst,
				   uint32_t *response, bool large_resp)
{
	struct intel_emmc_context *emmc =
	    CONTAINER_OF(inst, struct intel_emmc_context, instance);
	volatile emmc_5p1_regs_t *regs = emmc->regs;
	volatile uint32_t const *resp_addr = &regs->resp_01;
	int8_t i;
	int8_t resp_len = (true == large_resp) ? EMMC_HOST_RESPONSE_SIZE : 1;

	/* Read the response registers values */
	for (i = 0; i < resp_len; i++) {
		*response = *resp_addr;
		response++;
		resp_addr++;
	}
}

static int32_t emmc_reset_card(IN intel_instance_t *inst)
{
	emmc_host_cmd_config_t cmd;
	int32_t ret;

	cmd.cmd_arg = 0u;
	cmd.cmd_idx = EMMC_HOST_CMD0;
	cmd.cmd_type = EMMC_HOST_CMD_NORMAL;
	cmd.data_present = false;
	cmd.idx_check_en = false;
	cmd.crc_check_en = false;
	cmd.resp_type = EMMC_HOST_RESP_NONE;

	ret = emmc_host_send_cmd(inst, &cmd);

	DELAY(EMMC_CMD_WAIT_TIMEOUT_US);
	return ret;
}

/** Get OCR Register Details CMD1. */
static int32_t emmc_get_ocr(IN intel_instance_t *inst, uint32_t *ocr,
			    uint32_t cmd_arg)
{
	emmc_host_cmd_config_t cmd;
	int32_t ret;
	uint32_t response = 0;

	cmd.cmd_arg = cmd_arg;
	cmd.cmd_idx = EMMC_HOST_CMD1;
	cmd.cmd_type = EMMC_HOST_CMD_NORMAL;
	cmd.data_present = false;
	cmd.idx_check_en = false;
	cmd.crc_check_en = false;
	cmd.resp_type = EMMC_HOST_RESP_LEN_48;

	ret = emmc_host_send_cmd(inst, &cmd);

	if (ret) {
		return ret;
	}

	DELAY(EMMC_CMD_WAIT_TIMEOUT_US);
	/* Get OCR from response register */
	emmc_host_get_response(inst, (uint32_t *)&response, false);

	*ocr = response;
	/** OCR busy bit is set to low, if the device is not finished
	 * power up routine.
	 */
	if (!(EMMC_HOST_OCR_BUSY_BIT & response)) {
		ret = INTEL_DRIVER_ERROR_BUSY;
	}
	return ret;
}

/** Get CID Register Details CMD2. */
static int32_t emmc_read_cid(IN intel_instance_t *inst)
{
	struct intel_emmc_context *emmc =
	    CONTAINER_OF(inst, struct intel_emmc_context, instance);
	emmc_host_cmd_config_t cmd;
	int32_t ret;
	uint32_t i;
	uint32_t response[EMMC_CID_SIZE] = {0u};

	cmd.cmd_arg = 0u;
	cmd.cmd_idx = EMMC_HOST_CMD2;
	cmd.cmd_type = EMMC_HOST_CMD_NORMAL;
	cmd.data_present = false;
	cmd.idx_check_en = false;
	cmd.crc_check_en = true;
	cmd.resp_type = EMMC_HOST_RESP_LEN_136;

	ret = emmc_host_send_cmd(inst, &cmd);

	if (ret) {
		return ret;
	}

	emmc_host_get_response(inst, (uint32_t *)response, true);

	/* Get CID from the response. */
	for (i = 0u; i < EMMC_CID_SIZE; i++) {
		emmc->cid[i] = *((uint32_t *)(response + i));
	}
	return ret;
}

static int32_t emmc_set_rca(IN intel_instance_t *inst)
{
	struct intel_emmc_context *emmc =
	    CONTAINER_OF(inst, struct intel_emmc_context, instance);
	emmc_host_cmd_config_t cmd;
	uint32_t response;
	uint32_t ret;

	cmd.cmd_arg = emmc->rca << EMMC_HOST_RCA_SHIFT;
	cmd.cmd_idx = EMMC_HOST_CMD3;
	cmd.cmd_type = EMMC_HOST_CMD_NORMAL;
	cmd.data_present = false;
	cmd.idx_check_en = true;
	cmd.crc_check_en = true;
	cmd.resp_type = EMMC_HOST_RESP_LEN_48;

	ret = emmc_host_send_cmd(inst, &cmd);

	DELAY(EMMC_CMD_WAIT_TIMEOUT_US);

	emmc_host_get_response(inst, (uint32_t *)&response, false);

	return ret;
}

static int32_t emmc_select_card(IN intel_instance_t *inst)
{
	struct intel_emmc_context *emmc =
	    CONTAINER_OF(inst, struct intel_emmc_context, instance);
	emmc_host_cmd_config_t cmd;
	int32_t ret;
	uint32_t response;

	cmd.cmd_arg = emmc->rca << EMMC_HOST_RCA_SHIFT;
	cmd.cmd_idx = EMMC_HOST_CMD7;
	cmd.cmd_type = EMMC_HOST_CMD_NORMAL;
	cmd.data_present = false;
	cmd.idx_check_en = true;
	cmd.crc_check_en = true;
	cmd.resp_type = EMMC_HOST_RESP_LEN_48;

	ret = emmc_host_send_cmd(inst, &cmd);

	DELAY(EMMC_CMD_WAIT_TIMEOUT_US);

	emmc_host_get_response(inst, (uint32_t *)&response, false);

	return ret;
}

static int32_t emmc_set_block_count(IN intel_instance_t *inst,
				    uint32_t num_of_block, bool reliable_write)
{
	emmc_host_cmd_config_t cmd;
	int32_t ret;

	cmd.cmd_arg = (num_of_block & 0xFFFF) |
		      (reliable_write << EMMC_RELIABLE_WRITE_CMD23_OFFSET);
	cmd.cmd_idx = EMMC_HOST_CMD23;
	cmd.cmd_type = EMMC_HOST_CMD_NORMAL;
	cmd.data_present = false;
	cmd.idx_check_en = true;
	cmd.crc_check_en = true;
	cmd.resp_type = EMMC_HOST_RESP_LEN_48;

	ret = emmc_host_send_cmd(inst, &cmd);

	DELAY(EMMC_CMD_WAIT_TIMEOUT_US);

	return ret;
}

static int32_t emmc_host_init_data_xfer(IN intel_instance_t *inst,
					IN emmc_host_data_config_t *data_cfg)
{
	struct intel_emmc_context *emmc =
	    CONTAINER_OF(inst, struct intel_emmc_context, instance);
	volatile emmc_5p1_regs_t *regs = emmc->regs;
	uint16_t multi_block = 0u;
	int32_t ret = 0;

	if (data_cfg->dma_en) {
		regs->sdma_sysaddr = (uintptr_t)data_cfg->data;
	}

	/* Set Block Size Register */
	SET_BITS16(regs->block_size, EMMC_SDMA_BUF_SIZE_LOC,
		   EMMC_SDMA_BUF_SIZE_LOC, EMMC_SDMA_BOUNDARY);
	SET_BITS16(regs->block_size, EMMC_BLOCK_SIZE_LOC, EMMC_BLOCK_SIZE_LEN,
		   data_cfg->block_size);

	if (data_cfg->num_of_block > 1) {
		multi_block = 1u;
		/* Enable block count in transfer register*/
		SET_BITS16(regs->transfer_mode, EMMC_XFER_BLOCK_CNT_EN_LOC,
			   EMMC_XFER_BLOCK_CNT_EN_LEN, 1u);
	}

	/* Set block count regitser*/
	regs->block_count = (uint16_t)data_cfg->num_of_block;

	SET_BITS16(regs->transfer_mode, EMMC_XFER_MULTI_BLOCK_SEL_LOC,
		   EMMC_XFER_MULTI_BLOCK_SEL_LEN, multi_block);

	/* Set data transfer direction, Read = 1, Write = 0 */
	SET_BITS16(regs->transfer_mode, EMMC_XFER_DATA_DIR_LOC,
		   EMMC_XFER_DATA_DIR_LEN, (true == data_cfg->read) ? 1u : 0u);

	/* Enable DMA or not */
	SET_BITS16(regs->transfer_mode, EMMC_XFER_DMA_EN_LOC,
		   EMMC_XFER_DMA_EN_LEN, (true == data_cfg->dma_en) ? 1u : 0u);

	/* Set an interrupt at the block gap */
	SET_BITS8(regs->block_gap_ctrl, EMMC_BLOCK_GAP_LOC, EMMC_BLOCK_GAP_LEN,
		  (true == data_cfg->intr_block_gap_en) ? 1u : 0u);

	/* Set data timeout time */
	regs->timeout_ctrl = data_cfg->data_timeout;

	/* Send CMD23 to set block count for multiblocks */
	if (multi_block) {
		ret = emmc_set_block_count(inst, data_cfg->num_of_block,
					   data_cfg->reliable_write_en);
	}
	return ret;
}

static void wait_xfer_complete(IN intel_instance_t *inst)
{
	struct intel_emmc_context *emmc =
	    CONTAINER_OF(inst, struct intel_emmc_context, instance);
	volatile emmc_5p1_regs_t *regs = emmc->regs;

	while (1) {
		if (regs->normal_int_stat & EMMC_HOST_XFER_COMPLETE) {
			regs->normal_int_stat = EMMC_HOST_XFER_COMPLETE;
			break;
		}
	}
}

static int32_t read_data_port(IN intel_instance_t *inst,
			      emmc_host_data_config_t *data_cfg)
{
	int32_t ret;
	struct intel_emmc_context *emmc =
	    CONTAINER_OF(inst, struct intel_emmc_context, instance);
	volatile emmc_5p1_regs_t *regs = emmc->regs;
	uint32_t block_size = data_cfg->block_size;
	uint32_t i;

	while ((regs->present_state & EMMC_HOST_PSTATE_REG_BUF_READ_EN) == 0)
		;

	while (1) {
		if (regs->normal_int_stat & EMMC_HOST_BUF_RD_READY) {
			/* Clear interrupt bit */
			regs->normal_int_stat = EMMC_HOST_BUF_RD_READY;
			ret = INTEL_DRIVER_OK;
			break;
		}
	}

	if (regs->present_state & EMMC_HOST_PSTATE_REG_DAT_INHIBIT) {
		for (i = block_size >> 2u; i != 0u; i--) {
			*data_cfg->data = regs->data_port;
			data_cfg->data++;
		}
	}

	wait_xfer_complete(inst);

	return ret;
}

static int32_t emmc_read_csd(IN intel_instance_t *inst)
{
	struct intel_emmc_context *emmc =
	    CONTAINER_OF(inst, struct intel_emmc_context, instance);
	emmc_host_cmd_config_t cmd;
	int32_t ret;
	uint32_t response[EMMC_CSD_SIZE] = {0u};

	cmd.cmd_arg = emmc->rca << EMMC_HOST_RCA_SHIFT;
	cmd.cmd_idx = EMMC_HOST_CMD9;
	cmd.cmd_type = EMMC_HOST_CMD_NORMAL;
	cmd.data_present = false;
	cmd.idx_check_en = false;
	cmd.crc_check_en = true;
	cmd.resp_type = EMMC_HOST_RESP_LEN_136;

	ret = emmc_host_send_cmd(inst, &cmd);
	if (ret) {
		return ret;
	}

	emmc_host_get_response(inst, (uint32_t *)response, true);
	/* Get CSD from the response. */
	for (int i = 0u; i < EMMC_CSD_SIZE; i++) {
		emmc->csd[i] = *((uint32_t *)(response + i));
	}

	return ret;
}

static int32_t emmc_read_ext_csd(IN intel_instance_t *inst)
{
	struct intel_emmc_context *emmc =
	    CONTAINER_OF(inst, struct intel_emmc_context, instance);
	volatile emmc_5p1_regs_t *regs = emmc->regs;
	emmc_host_cmd_config_t cmd;
	int32_t ret;
	emmc_host_data_config_t data_cfg;

	cmd.cmd_arg = emmc->rca << EMMC_HOST_RCA_SHIFT;
	cmd.cmd_idx = EMMC_HOST_CMD8;
	cmd.cmd_type = EMMC_HOST_CMD_NORMAL;
	cmd.data_present = true;
	cmd.idx_check_en = true;
	cmd.crc_check_en = true;
	cmd.resp_type = EMMC_HOST_RESP_LEN_48;

	data_cfg.block_size = EMMC_BLOCK_SIZE;
	data_cfg.num_of_block = 1u;
	data_cfg.read = true;
	data_cfg.data = emmc->ext_csd;
	data_cfg.data_timeout = EMMC_HOST_MAX_TIMEOUT;
	data_cfg.intr_block_gap_en = false;
	data_cfg.reliable_write_en = false;
	data_cfg.dma_en = false;

	SET_BITS8(regs->host_ctrl1, EMMC_HOST_CTRL1_DMA_SEL_LOC,
		  EMMC_HOST_CTRL1_DMA_SEL_LEN, 0u);

	emmc_host_init_data_xfer(inst, &data_cfg);

	ret = emmc_host_send_cmd(inst, &cmd);

	if (ret == INTEL_DRIVER_OK) {
		ret = read_data_port(inst, &data_cfg);
		if (ret == INTEL_DRIVER_OK) {
			emmc->max_sectors =
			    emmc->ext_csd[EMMC_HOST_EXTCSD_SEC_COUNT];
		}
	}

	return ret;
}

static int32_t emmc_host_switch_cmd(IN intel_instance_t *inst, uint32_t cmd_arg)
{
	emmc_host_cmd_config_t cmd;
	int32_t ret;

	cmd.cmd_arg = cmd_arg;
	cmd.cmd_idx = EMMC_HOST_CMD6;
	cmd.cmd_type = EMMC_HOST_CMD_NORMAL;
	cmd.data_present = false;
	cmd.idx_check_en = true;
	cmd.crc_check_en = true;
	cmd.resp_type = EMMC_HOST_RESP_LEN_48B;

	ret = emmc_host_send_cmd(inst, &cmd);

	DELAY(EMMC_CMD_WAIT_TIMEOUT_US);

	return ret;
}

static void emmc_set_host_bus_width(IN intel_instance_t *inst,
				    uint8_t ext_data_width, uint8_t data_width)
{
	struct intel_emmc_context *emmc =
	    CONTAINER_OF(inst, struct intel_emmc_context, instance);
	volatile emmc_5p1_regs_t *regs = emmc->regs;

	SET_BITS8(regs->host_ctrl1, EMMC_HOST_CTRL1_EXT_DAT_WIDTH_LOC,
			EMMC_HOST_CTRL1_EXT_DAT_WIDTH_LEN, ext_data_width);
	regs->host_ctrl1 |= data_width << EMMC_HOST_CTRL1_DAT_WIDTH_LOC;
}

int32_t emmc_set_bus_speed(IN intel_instance_t *inst, uint32_t speed_mode,
                           uint16_t uhs_mode)
{
        int32_t ret;
        uint32_t cmd_arg = 0u;

        cmd_arg = CMD_ARG_BUS_SPEED_EXT_CSD_REG(speed_mode);

        ret = emmc_host_switch_cmd(inst, cmd_arg);

        if (ret == INTEL_DRIVER_OK) {
                set_uhs_mode(inst, uhs_mode);
        }

        return ret;
}

int32_t intel_emmc_set_bus_width(IN intel_instance_t *inst, uint8_t bus_width,
				  bool dual_data_rate)
{
	int32_t ret;
	uint32_t cmd_arg = 0u;
	uint32_t speed_mode;
	uint16_t uhs_mode;
	uint8_t extended_data_width, data_width, bus_mode;

	if (bus_width == 1) {
		bus_mode = 0;
		extended_data_width = 0;
		data_width = 0;
		/* Single Data Line can support only SDR mode */
		speed_mode = EMMC_HOST_BUS_SPEED_HIGHSPEED;
		uhs_mode = EMMC_HOST_CTRL2_UHSMODE_SDR25;
		emmc_set_bus_speed(inst, speed_mode, uhs_mode);
	} else if (bus_width == 4) {
		bus_mode = 1;
		extended_data_width = 0;
		data_width = 1;
		if (dual_data_rate) {
			bus_mode = 5;
		}
	} else if (bus_width == 8) {
		bus_mode = 2;
		extended_data_width = 1;
		data_width = 0;
		if (dual_data_rate) {
			bus_mode = 6;
		}
	} else {
		return INTEL_DRIVER_ERROR_PARAMETER;
	}

	cmd_arg = CMD_ARG_BUS_WIDTH_EXT_CSD_REG(bus_mode);

	/* Send CMD6 Switch command to set bus width in EXT CSD register*/
	ret = emmc_host_switch_cmd(inst, cmd_arg);

	if (ret == INTEL_DRIVER_OK) {
		emmc_set_host_bus_width(inst, extended_data_width, data_width);
	}

	return ret;
}

int32_t intel_emmc_get_card_status(IN intel_instance_t *inst)
{
	emmc_host_cmd_config_t cmd;
	uint32_t response = 0u;
	int32_t ret;
	struct intel_emmc_context *emmc =
	    CONTAINER_OF(inst, struct intel_emmc_context, instance);

	cmd.cmd_arg = emmc->rca << EMMC_HOST_RCA_SHIFT;
	cmd.cmd_idx = EMMC_HOST_CMD13;
	cmd.cmd_type = EMMC_HOST_CMD_NORMAL;
	cmd.data_present = false;
	cmd.idx_check_en = true;
	cmd.crc_check_en = true;
	cmd.resp_type = EMMC_HOST_RESP_LEN_48;

	ret = emmc_host_send_cmd(inst, &cmd);
	if (ret) {
		return ret;
	}

	emmc_host_get_response(inst, (uint32_t *)&response, false);
	ret = EMMC_GET_STATE(response);

	return ret;
}

int32_t intel_emmc_init_card(IN intel_instance_t *inst)
{
	int32_t ret, cmd1_retry;
	uint32_t ocr = 0; /* Operation Condition register. */
	uint32_t speed_mode = EMMC_HOST_BUS_SPEED_HIGHSPEED;
	uint16_t uhs_mode = EMMC_HOST_CTRL2_UHSMODE_DDR50;
	uint8_t bus_width = EMMC_HOST_BUS_WIDTH_8_BIT;
	volatile emmc_5p1_regs_t *regs;
	uint32_t cmd6_timeout;

	struct intel_emmc_context *emmc =
	    CONTAINER_OF(inst, struct intel_emmc_context, instance);
	emmc_5p1_regs_t *base = (emmc_5p1_regs_t *)(inst->base_addr);

	emmc->regs = base;
	emmc->rca = EMMC_RCA_ADDRESS;
	regs = emmc->regs;

	/* Software Reset */
	emmc_host_sw_reset(inst, EMMC_SW_RESET_ALL);

	enable_interrupts(inst);

	clear_interrupts(inst);

	emmc_set_power(inst);

	ret = emmc_clock_set(inst, EMMC_HOST_CLK_FREQ_400K);
	if (!ret) {
		return INTEL_DRIVER_ERROR;
	}

	/* Reset device to idle state. */
	ret = emmc_reset_card(inst);

	cmd1_retry = EMMC_CMD1_RETRY_TIMEOUT;

	while (cmd1_retry > 0) {
		/* Get OCR (CMD1). */
		ret = emmc_get_ocr(inst, &ocr, EMMC_DUAL_VOLTAGE_RANGE);
		if (ret != INTEL_DRIVER_OK) {
			emmc_host_sw_reset(inst, EMMC_SW_RESET_CMD_LINE);
			DELAY(EMMC_CMD1_RETRY_TIMEOUT);
		} else {
			emmc->card_cap = EMMC_HOST_LESS_2G;

			/* eMMC capacity is greater than 2GB */
			if (ocr & EMMC_HOST_OCR_CAPACITY_MASK) {
				emmc->card_cap = EMMC_HOST_GREATER_2G;
			}
			break;
		}
		cmd1_retry--;
	}

	if (ret != INTEL_DRIVER_OK) {
		goto err;
	}

	/* Ready state */
	ret = emmc_read_cid(inst);

	if (ret) {
		goto err;
	}

	/* Identification State CMD3 RCA register*/
	ret = emmc_set_rca(inst);

	ret = emmc_read_csd(inst);
	if (ret) {
		goto err;
	}

	ret = emmc_select_card(inst);
	if (ret) {
		goto err;
	}

	regs->host_ctrl2 |= EMMC_HOST_CTRL2_1P8V_SIG_EN
			    << EMMC_HOST_CTRL2_1P8V_SIG_LOC;
	emmc_clock_set(inst, EMMC_HOST_CLK_FREQ_25M);

	/* Transfer State */
	ret = emmc_read_ext_csd(inst);
	if (ret) {
		goto err;
	}

	cmd6_timeout =
	    EMMC_HOST_CMD6_TIMEOUT_MULT *
	    (emmc->ext_csd[EMMC_HOST_EXTCSD_GENERIC_CMD6_TIME] & 0xFF);

	/* Set to High Speed and DDR mode */
	ret = emmc_set_bus_speed(inst, speed_mode, uhs_mode);
	DELAY(cmd6_timeout);

	if (uhs_mode == EMMC_HOST_CTRL2_UHSMODE_DDR50 &&
	    bus_width == EMMC_HOST_BUS_WIDTH_1_BIT) {
		return INTEL_DRIVER_ERROR;
	}

	/* Set Bus Width */
	ret = intel_emmc_set_bus_width(inst, bus_width, true);
	if (ret) {
		goto err;
	}

err:
	return ret;
}

int32_t intel_emmc_read(IN intel_instance_t *inst, uint8_t *data_buff,
			uint32_t start_sector, uint32_t num_of_blocks)
{
	DBG_CHECK(data_buff != NULL, INTEL_DRIVER_ERROR_PARAMETER);
	DBG_CHECK(start_sector == 0, INTEL_DRIVER_ERROR_PARAMETER);
	DBG_CHECK(num_of_blocks == 0, INTEL_DRIVER_ERROR_PARAMETER);

	emmc_host_cmd_config_t cmd;
	emmc_host_data_config_t data_cfg;
	int32_t ret;
	struct intel_emmc_context *emmc =
	    CONTAINER_OF(inst, struct intel_emmc_context, instance);
	uint32_t cmd_data_addr = start_sector;

	if ((start_sector + num_of_blocks) > emmc->max_sectors) {
		return INTEL_DRIVER_ERROR;
	}

	if (num_of_blocks > 1) {
		cmd.cmd_idx = EMMC_HOST_CMD18;
		data_cfg.intr_block_gap_en = true;
	} else {
		cmd.cmd_idx = EMMC_HOST_CMD17;
		data_cfg.intr_block_gap_en = false;
	}

	data_cfg.block_size = EMMC_BLOCK_SIZE;
	data_cfg.num_of_block = num_of_blocks;
	data_cfg.dma_en = true;
	data_cfg.read = true;
	data_cfg.data_timeout = EMMC_HOST_MAX_TIMEOUT;
	data_cfg.reliable_write_en = false;
	data_cfg.data = (uint32_t *)data_buff;

	ret = emmc_host_init_data_xfer(inst, &data_cfg);
	if (ret) {
		return INTEL_DRIVER_ERROR;
	}

	if (emmc->card_cap == EMMC_HOST_LESS_2G) {
		cmd_data_addr *= 512u;
	}

	cmd.cmd_arg = cmd_data_addr;
	cmd.cmd_type = EMMC_HOST_CMD_NORMAL;
	cmd.data_present = true;
	cmd.idx_check_en = true;
	cmd.crc_check_en = true;
	cmd.resp_type = EMMC_HOST_RESP_LEN_48;

	ret = emmc_host_send_cmd(inst, &cmd);

	wait_xfer_complete(inst);
	return ret;
}

int32_t intel_emmc_write(IN intel_instance_t *inst, uint8_t *data_buff,
			 uint32_t start_sector, uint32_t num_of_blocks)
{
	DBG_CHECK(data_buff != NULL, INTEL_DRIVER_ERROR_PARAMETER);
	DBG_CHECK(start_sector == 0, INTEL_DRIVER_ERROR_PARAMETER);
	DBG_CHECK(num_of_blocks == 0, INTEL_DRIVER_ERROR_PARAMETER);

	emmc_host_cmd_config_t cmd;
	emmc_host_data_config_t data_cfg;
	int32_t ret;
	struct intel_emmc_context *emmc =
	    CONTAINER_OF(inst, struct intel_emmc_context, instance);
	uint32_t cmd_data_addr = start_sector;

	if ((start_sector + num_of_blocks) > emmc->max_sectors) {
		return INTEL_DRIVER_ERROR;
	}

	if (num_of_blocks > 1) {
		cmd.cmd_idx = EMMC_HOST_CMD25;
		data_cfg.intr_block_gap_en = true;
	} else {
		cmd.cmd_idx = EMMC_HOST_CMD24;
		data_cfg.intr_block_gap_en = false;
	}

	data_cfg.block_size = EMMC_BLOCK_SIZE;
	data_cfg.num_of_block = num_of_blocks;
	data_cfg.dma_en = true;
	data_cfg.read = false;
	data_cfg.data_timeout = EMMC_HOST_MAX_TIMEOUT;
	data_cfg.reliable_write_en = true;
	data_cfg.data = (uint32_t *)data_buff;

	ret = emmc_host_init_data_xfer(inst, &data_cfg);
	if (ret) {
		return INTEL_DRIVER_ERROR;
	}

	if (emmc->card_cap == EMMC_HOST_LESS_2G) {
		cmd_data_addr *= 512u;
	}

	cmd.cmd_arg = cmd_data_addr;
	cmd.cmd_type = EMMC_HOST_CMD_NORMAL;
	cmd.data_present = true;
	cmd.idx_check_en = true;
	cmd.crc_check_en = true;
	cmd.resp_type = EMMC_HOST_RESP_LEN_48;

	ret = emmc_host_send_cmd(inst, &cmd);

	wait_xfer_complete(inst);

	return ret;
}

int32_t intel_emmc_erase(IN intel_instance_t *inst, uint32_t start_addr,
			 uint32_t end_addr)
{
	DBG_CHECK(start_addr == 0, INTEL_DRIVER_ERROR_PARAMETER);
	DBG_CHECK(end_addr == 0, INTEL_DRIVER_ERROR_PARAMETER);

	int32_t ret;
	emmc_host_cmd_config_t cmd;
	struct intel_emmc_context *emmc =
	    CONTAINER_OF(inst, struct intel_emmc_context, instance);

	if ((end_addr < start_addr) || (end_addr > emmc->max_sectors)) {
		return INTEL_DRIVER_ERROR;
	}

	if (emmc->card_cap == EMMC_HOST_LESS_2G) {
		start_addr *= 512u;
		end_addr *= 512u;
	}

	cmd.cmd_type = EMMC_HOST_CMD_NORMAL;
	cmd.data_present = false;
	cmd.idx_check_en = true;
	cmd.crc_check_en = true;
	cmd.resp_type = EMMC_HOST_RESP_LEN_48;

	/* Send start address group with CMD35 */
	cmd.cmd_idx = EMMC_HOST_CMD35;
	cmd.cmd_arg = start_addr;
	ret = emmc_host_send_cmd(inst, &cmd);

	/* Send end address group with CMD36 */
	cmd.cmd_idx = EMMC_HOST_CMD36;
	cmd.cmd_arg = end_addr;
	ret = emmc_host_send_cmd(inst, &cmd);

	/* Erase command CMD38 */
	cmd.cmd_idx = EMMC_HOST_CMD38;
	cmd.cmd_arg = 0u;
	ret = emmc_host_send_cmd(inst, &cmd);

	return ret;
}

uint32_t intel_emmc_get_card_info(IN intel_instance_t *inst,
				  emmc_card_info_t info)
{
	uint32_t ret;
	struct intel_emmc_context *emmc =
	    CONTAINER_OF(inst, struct intel_emmc_context, instance);

	switch (info) {
	case EMMC_CARD_SECTOR_COUNT:
		ret = emmc->max_sectors;
		break;
	case EMMC_CARD_BLOCK_SIZE:
		ret = EMMC_BLOCK_SIZE;
		break;
	default:
		ret = INTEL_DRIVER_ERROR_PARAMETER;
		break;
	}
	return ret;
}
