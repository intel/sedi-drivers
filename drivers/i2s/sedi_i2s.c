/*
 * Copyright (c) 2021 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "sedi_driver_i2s.h"
#include "sedi_i2s.h"
#include "pm/pm_internal_if.h"
#include "sedi_driver_pm.h"

#if I2S_DEBUG
#define I2S_DEBUG_PRINT PRINT
#else
#define I2S_DEBUG_PRINT(...)
#endif

#if I2S_DEBUG_TRACE_EN
#define I2S_DEBUG_TRACE PRINT
#else
#define I2S_DEBUG_TRACE(...)
#endif

#if I2S_ISR_DEBUG_TRACE_EN
#define I2S_ISR_DEBUG_TRACE PRINT
#else
#define I2S_ISR_DEBUG_TRACE(...)
#endif

/* driver version */

static const sedi_driver_version_t i2s_driver_version = {
	SEDI_I2S_API_VERSION, SEDI_I2S_DRIVER_VERSION
};

#ifndef __NOP
#define __NOP() __asm volatile ("nop")
#endif

#define I2S_NOP_DELAY()	\
	__NOP();	\
	__NOP();	\
	__NOP();	\
	__NOP();	\
	__NOP();

static i2s_resource_t i2s_inst[SEDI_MAX_I2S_INSTANCE] = {
	{
		.regs = (i2s_regs_t *)I2S_REGS_BASE_0,
	},
	{
		.regs = (i2s_regs_t *)I2S_REGS_BASE_1,
	},
};

static i2s_mn_div_resource_t i2s_mndiv_inst[SEDI_MAX_I2S_INSTANCE] = {
	{
		.regs = (i2s_mn_div_regs_t *)I2S_MN_DIV_REGS_BASE_0,

	},
	{
		.regs = (i2s_mn_div_regs_t *)I2S_MN_DIV_REGS_BASE_1,
	},
};

int sedi_i2s_reinit(sedi_i2s_t num);
static int sedi_i2s_config(IN sedi_i2s_t num, IN i2s_config_t *i2s_config,
			   IN bool pm_exit);

#if SAMPLE_FREQ_LOOKUP_TABLE
/* Note: This lookup table is for 100Mhz clock freq . Do not use this
 * if the clock frequency is different.
 */
const uint32_t sample_rate[BIT_SAMPLE_MAX][SFR_MAX][I2S_CHNL_MAX] = {
	{
		{
			0x412, 0x209, 0xAE, 0x82,
		},
		{
			0x2FA, 0x17A, 0x7E, 0x5E,
		},
		{
			0x209, 0x104, 0x57, 0x41,
		},
		{
			0x17A, 0xBD, 0x3F, 0x2F,
		},
		{
			0x104, 0x82, 0x2B, 0x21,
		},
		{
			0xBD, 0x5E, 0x1F, 0x18,
		},
		{
			0xAE, 0x57, 0x1D, 0x16,
		},
		{
			0x5E, 0x2F, 0x10, 0x0C,
		},
		{
			0x57, 0x2B, 0x0E, 0x0B,
		},
		{
			0x2F, 0x18, 0x08, 0x06,
		},
		{
			0x2B, 0x16, 0x07, 0x05,
		},
	},
	{
		{
			0x30D, 0x187, 0x82, 0x62,
		},
		{
			0x237, 0x11B, 0x5E, 0x47,
		},
		{
			0x187, 0xC3, 0x41, 0x31,
		},
		{
			0x11B, 0x8E, 0x2F, 0x23,
		},
		{
			0xC3, 0x62, 0x21, 0x18,
		},
		{
			0x8E, 0x47, 0x18, 0x12,
		},
		{
			0x82, 0x41, 0x16, 0x10,
		},
		{
			0x47, 0x23, 0x0C, 0x09,
		},
		{
			0x41, 0x21, 0x0B, 0x08,
		},
		{
			0x23, 0x12, 0x06, 0x04,
		},
		{
			0x21, 0x10, 0x05, 0x04,
		},
	},
	{
		{
			0x209, 0x104, 0x57, 0x41,
		},
		{
			0x17A, 0xBD, 0x3F, 0x2F,
		},
		{
			0x104, 0x82, 0x2B, 0x21,
		},
		{
			0xBD, 0x5E, 0x1F, 0x18,
		},
		{
			0x82, 0x41, 0x16, 0x10,
		},
		{
			0x5E, 0x2F, 0x10, 0x0C,
		},
		{
			0x57, 0x2B, 0x0E, 0x0B,
		},
		{
			0x2F, 0x18, 0x08, 0x06,
		},
		{
			0x2B, 0x16, 0x07, 0x05,
		},
		{
			0x18, 0x0C, 0x04, 0x03,
		},
		{
			0x16, 0x0B, 0x04, 0x03,
		},
	},
	{
		{
			0x187, 0xC3, 0x41, 0x31,
		},
		{
			0x11B, 0x8E, 0x2F, 0x23,
		},
		{
			0xC3, 0x62, 0x21, 0x18,
		},
		{
			0x8E, 0x47, 0x18, 0x12,
		},
		{
			0x62, 0x31, 0x10, 0x0C,
		},
		{
			0x47, 0x23, 0x0C, 0x09,
		},
		{
			0x41, 0x21, 0x0B, 0x08,
		},
		{
			0x23, 0x12, 0x06, 0x04,
		},
		{
			0x21, 0x10, 0x05, 0x04,
		},
		{
			0x12, 0x09, 0x03, 0x02,
		},
		{
			0x10, 0x08, 0x03, 0x02,
		},
	},
};

static uint32_t cal_sample_rate(uint32_t word_len, uint32_t sample_freq,
				uint32_t chnl)
{
	I2S_DEBUG_TRACE("%s\n", __func__);
	uint32_t wrd_ln_inx, srf_indx, chl_idx = chnl;

	switch (word_len) {
	case I2S_12_BIT_SAMPLE:
		wrd_ln_inx = 0;
		break;
	case I2S_16_BIT_SAMPLE:
		wrd_ln_inx = 1;
		break;
	case I2S_24_BIT_SAMPLE:
		wrd_ln_inx = 2;
		break;
	case I2S_32_BIT_SAMPLE:
		wrd_ln_inx = 3;
		break;
	default:
		I2S_DEBUG_PRINT("wrong word len\n");
		return SEDI_DRIVER_ERROR_PARAMETER;
	}

	switch (sample_freq) {
	case I2S_8K_SFR:
		srf_indx = 0;
		break;
	case I2S_11K_SFR:
		srf_indx = 1;
		break;
	case I2S_16K_SFR:
		srf_indx = 2;
		break;
	case I2S_22K_SFR:
		srf_indx = 3;
		break;
	case I2S_32K_SFR:
		srf_indx = 4;
		break;
	case I2S_44K_SFR:
		srf_indx = 5;
		break;
	case I2S_48K_SFR:
		srf_indx = 6;
		break;
	case I2S_88K_SFR:
		srf_indx = 7;
		break;
	case I2S_96K_SFR:
		srf_indx = 8;
		break;
	case I2S_176K_SFR:
		srf_indx = 9;
		break;
	case I2S_192K_SFR:
		srf_indx = 10;
		break;
	default:
		I2S_DEBUG_PRINT(" invalid sample freq\n");
		return SEDI_DRIVER_ERROR_PARAMETER;
	}
	switch (chnl) {
	case I2S_CHNL_MONO:
		chl_idx = 0;
		break;
	case I2S_CHNL_STERIO:
		chl_idx = 1;
		break;
	case I2S_CHNL_6:
		chl_idx = 2;
		break;
	case I2S_CHNL_8:
		chl_idx = 3;
		break;
	default:
		I2S_DEBUG_PRINT(" invalid channel number\n");
		return SEDI_DRIVER_ERROR_PARAMETER;
	}

	I2S_DEBUG_TRACE(" Sample rate Register value %x\n",
			sample_rate[wrd_ln_inx][srf_indx][chl_idx]);

	return sample_rate[wrd_ln_inx][srf_indx][chl_idx];
}

#else

static inline uint32_t round(float f)
{
	return (uint32_t)(f + 0.5f);
}

static uint32_t cal_sample_rate(sedi_i2s_t num, int chn_width, int asf,
				int chns)
{
	float sample_rate;
	float i, error, aux1, aux2, aux3, aux4, aux5;
	uint32_t lbw_clock;

	if (sedi_get_mn_divide(num)) {
		sample_rate = 0;
	} else {
		lbw_clock = sedi_pm_get_lbw_clock();

		aux1 = (lbw_clock / asf);
		I2S_DEBUG_PRINT("aux1:%f\n", aux1);
		sample_rate = 0;
		error = I2S_SET_BIT;

		for (i = 0; i < SAMPLE_RATE_CAL_ITRN; i++) {
			aux2 = (aux1 / (chns * chn_width)) + i;
			I2S_DEBUG_PRINT("aux2:%f\n", aux2);

			aux3 = aux2 * chns * chn_width;
			I2S_DEBUG_PRINT("aux3:%f\n", aux3);

			aux4 = I2S_SET_BIT / (aux3 / lbw_clock);
			I2S_DEBUG_PRINT("aux4:%f\n", aux4);

			aux5 = (asf - aux4) / asf;
			I2S_DEBUG_PRINT("aux5:%f\n", aux5);
			if (aux5 < error) {
				I2S_DEBUG_PRINT("sample rate:%f\n", aux2);
				sample_rate = aux2;
				error = aux5;
			}
		}
	}

	return round(sample_rate);
}

#endif

sedi_driver_version_t sedi_i2s_get_version(void)
{
	return i2s_driver_version;
}

static inline void reset_i2s_instance(sedi_i2s_t num)
{
	i2s_inst[num].regs->ctrl.sfr_rst = I2S_CLEAR_BIT;
	do {    /* Loop Until SFR Reset complete */
		I2S_NOP_DELAY();
	} while (!i2s_inst[num].regs->ctrl.sfr_rst);

	i2s_inst[num].ctrl.bits = I2S_RESET_VAL;
}

static inline void enable_transceiver(sedi_i2s_t num)
{
	I2S_DEBUG_TRACE("%s\n", __func__);
	i2s_inst[num].ctrl.i2s_en = I2S_SET_BIT;
	i2s_inst[num].regs->ctrl.bits = i2s_inst[num].ctrl.bits;
}

static inline void disable_transceiver(sedi_i2s_t num)
{
	I2S_DEBUG_TRACE("%s\n", __func__);
	i2s_inst[num].ctrl.i2s_en = I2S_CLEAR_BIT;
	i2s_inst[num].regs->ctrl.bits = i2s_inst[num].ctrl.bits;
}

int sedi_enable_transceiver(IN sedi_i2s_t num)
{
	I2S_DEBUG_TRACE("%s\n", __func__);
	DBG_CHECK(num < SEDI_I2S_MAX, SEDI_DRIVER_ERROR_PARAMETER);
	enable_transceiver(num);
	return SEDI_DRIVER_OK;
}

int sedi_get_mn_divide(IN sedi_i2s_t num)
{
	I2S_DEBUG_TRACE("%s\n", __func__);

	return i2s_inst[num].i2s_mn_div.mn_div_en.enable;
}

int sedi_set_mn_divide(IN sedi_i2s_t num, IN uint32_t mdiv_val,
		       IN uint32_t ndiv_val, uint8_t enable)
{
	I2S_DEBUG_TRACE("%s\n", __func__);
	DBG_CHECK(num < SEDI_I2S_MAX, SEDI_DRIVER_ERROR_PARAMETER);

	i2s_inst[num].i2s_mn_div.mval.val = mdiv_val;
	i2s_inst[num].i2s_mn_div.nval.val = ndiv_val;
	i2s_inst[num].i2s_mn_div.mn_div_en.enable = enable;

	return SEDI_DRIVER_OK;
}

int sedi_disable_transceiver(IN sedi_i2s_t num)
{
	I2S_DEBUG_TRACE("%s\n", __func__);
	DBG_CHECK(num < SEDI_I2S_MAX, SEDI_DRIVER_ERROR_PARAMETER);
	disable_transceiver(num);
	return SEDI_DRIVER_OK;
}

static inline void enable_transceiver_clk(sedi_i2s_t num)
{
	I2S_DEBUG_TRACE("%s\n", __func__);
	i2s_inst[num].ctrl.i2s_stb = I2S_CLEAR_BIT;
}

static inline void disable_transceiver_clk(sedi_i2s_t num)
{
	I2S_DEBUG_TRACE("%s\n", __func__);
	i2s_inst[num].ctrl.i2s_stb = I2S_SET_BIT;
}

int sedi_i2s_enable_interrupt(IN sedi_i2s_t num)
{
	I2S_ISR_DEBUG_TRACE("%s\n", __func__);
	DBG_CHECK(num < SEDI_I2S_MAX, SEDI_DRIVER_ERROR_PARAMETER);

	i2s_inst[num].ctrl.intreq_mask = ENABLE_I2S_INTR;
	i2s_inst[num].regs->ctrl.bits = i2s_inst[num].ctrl.bits;

	return SEDI_DRIVER_OK;
}

int sedi_i2s_disable_interrupt(IN sedi_i2s_t num)
{
	I2S_ISR_DEBUG_TRACE("%s\n", __func__);
	DBG_CHECK(num < SEDI_I2S_MAX, SEDI_DRIVER_ERROR_PARAMETER);

	i2s_inst[num].ctrl.intreq_mask = I2S_CLEAR_BIT;
	i2s_inst[num].regs->ctrl.bits = i2s_inst[num].ctrl.bits;

	return SEDI_DRIVER_OK;
}

static void sedi_i2s_set_fifos(sedi_i2s_t num)
{
	I2S_DEBUG_TRACE("%s\n", __func__);

	if ((i2s_inst[num].i2s_config.mode == I2S_MASTER_FULL_DUPLEX) ||
	    (i2s_inst[num].i2s_config.mode == I2S_SLAVE_FULL_DUPLEX)) {

		I2S_DEBUG_TRACE(
			"I2S_MASTER_FULL_DUPLEX or I2S_SLAVE_FULL_DUPLEX\n");

		i2s_inst[num].regs->fifo_aempty = TX_FIFO_AEMPTY;
		i2s_inst[num].regs->fifo_afull_fdr = RX_FIFO_AFULL;
		i2s_inst[num].regs->fifo_aempty_fdr = RX_FIFO_AEMPTY;
		i2s_inst[num].regs->fifo_afull = TX_FIFO_AFULL - 1;

	} else if ((i2s_inst[num].i2s_config.mode == I2S_MASTER_TX) ||
		   (i2s_inst[num].i2s_config.mode == I2S_SLAVE_TX)) {

		I2S_DEBUG_TRACE("I2S_MASTER_TX or I2S_SLAVE_TX\n");

		i2s_inst[num].regs->fifo_aempty = TX_FIFO_AEMPTY;

	} else {

		I2S_DEBUG_TRACE("I2S_MASTER_RX or I2S_SLAVE_RX\n");
		I2S_DEBUG_TRACE("RX Half duplex mode\n");

		i2s_inst[num].regs->fifo_aempty = RX_FIFO_AEMPTY;
		i2s_inst[num].regs->fifo_afull = RX_FIFO_AFULL;
	}
}

int sedi_i2s_reset_rx_fifo(IN uint32_t num)
{
	DBG_CHECK(num < SEDI_I2S_MAX, SEDI_DRIVER_ERROR_PARAMETER);

	if (i2s_inst[num].regs->stat.rdata_ovrerr == I2S_BIT_HIGH) {

		I2S_DEBUG_PRINT("FIFO Over Run\n");

		i2s_inst[num].regs->stat.rdata_ovrerr = I2S_CLEAR_BIT;
		i2s_inst[num].regs->stat.fifo_empty = I2S_CLEAR_BIT;
		i2s_inst[num].regs->stat.fifo_full = I2S_CLEAR_BIT;
		i2s_inst[num].regs->stat.fifo_afull = I2S_CLEAR_BIT;
		i2s_inst[num].regs->stat.rfifo_empty = I2S_CLEAR_BIT;
		i2s_inst[num].regs->stat.rfifo_aempty = I2S_CLEAR_BIT;
		i2s_inst[num].regs->stat.rfifo_full = I2S_CLEAR_BIT;
		i2s_inst[num].regs->stat.rfifo_afull = I2S_CLEAR_BIT;
	}
	i2s_inst[num].regs->ctrl_fdr.fifo_rst = I2S_CLEAR_BIT;
	I2S_NOP_DELAY();
	i2s_inst[num].ctrl_fdr.fifo_rst = I2S_SET_BIT;
	return SEDI_DRIVER_OK;
}

int sedi_i2s_reset_tx_fifo(IN uint32_t num)
{
	DBG_CHECK(num < SEDI_I2S_MAX, SEDI_DRIVER_ERROR_PARAMETER);
	i2s_inst[num].regs->ctrl.fifo_rst = I2S_CLEAR_BIT;
	I2S_NOP_DELAY();
	i2s_inst[num].ctrl.fifo_rst = I2S_SET_BIT;

	return SEDI_DRIVER_OK;
}

int sedi_i2s_poll_read(IN sedi_i2s_t num, INOUT uint32_t *dst, IN uint32_t size)
{
	I2S_DEBUG_TRACE("%s\n", __func__);
	DBG_CHECK(num < SEDI_I2S_MAX, SEDI_DRIVER_ERROR_PARAMETER);

	__IO_RW uint32_t *src;
	__IO_RW uint32_t *fifo_lvl_reg;
	uint16_t *dst_16 = (uint16_t *)dst;
	uint32_t ret = SEDI_DRIVER_OK;
	uint32_t idx = 0x0UL;

	DBG_CHECK(num < SEDI_I2S_MAX, SEDI_DRIVER_ERROR_PARAMETER);
	DBG_CHECK(dst != NULL, SEDI_DRIVER_ERROR_PARAMETER);

	src = (uint32_t *)(((num == SEDI_I2S_NUM_0) ? I2S_REGS_BASE_0
			    : I2S_REGS_BASE_1) +
			   I2S_RX_FIFO);
	DBG_CHECK(src != NULL, SEDI_DRIVER_ERROR_PARAMETER);

	if ((i2s_inst[num].i2s_config.mode == I2S_MASTER_FULL_DUPLEX) ||
	    (i2s_inst[num].i2s_config.mode == I2S_SLAVE_FULL_DUPLEX)) {
		fifo_lvl_reg =
			(__IO_RW uint32_t *)&i2s_inst[num].regs->fifo_level_fdr;
	} else {
		fifo_lvl_reg =
			(__IO_RW uint32_t *)&i2s_inst[num].regs->fifo_level;
	}

	if (i2s_inst[num].i2s_config.bits <= I2S_BITS_PER_SAMPLE_16BIT) {
		/* To keep minimal condition check before reading
		 * Rx FIFO, otherwise Rx FIFO overflow may occur
		 * while loop is repeated in both if and else code blocks.
		 */
		while (idx < size) {
			if (*fifo_lvl_reg) {
				dst_16[idx++] = (uint16_t)*src;
			}
		}

	} else {
		while (idx < size) {
			if (*fifo_lvl_reg) {
				dst[idx++] = *src;
			}
		}
	}

	return ret;
}

int sedi_i2s_poll_write(IN sedi_i2s_t num, IN uint32_t *src, IN uint32_t size)
{
	I2S_DEBUG_TRACE("%s\n", __func__);

	DBG_CHECK(num < SEDI_I2S_MAX, SEDI_DRIVER_ERROR_PARAMETER);
	DBG_CHECK(src != NULL, SEDI_DRIVER_ERROR_PARAMETER);

	__IO_RW uint32_t *dst;
	int ret = SEDI_DRIVER_OK;
	uint32_t idx = 0;
	uint16_t *src_16bit;

	dst = (__IO_RW uint32_t *)(((num == SEDI_I2S_NUM_0) ? I2S_REGS_BASE_0
				    : I2S_REGS_BASE_1) +
				   (I2S_TX_FIFO));

	DBG_CHECK(dst != NULL, SEDI_DRIVER_ERROR_PARAMETER);

	if (i2s_inst[num].i2s_config.bits <= I2S_BITS_PER_SAMPLE_16BIT) {
		src_16bit = (uint16_t *)src;
		while (idx < size) {
			if (i2s_inst[num].regs->fifo_level <
			    KEEP_FIFO_LEVEL_EMPTY) {
				/* Check FIFO Level before pushing data into
				 * FIFO. FIFO Level Register is read only
				 * register.
				 */
				*dst = (__IO_RW uint32_t)src_16bit[idx++];
			}
		}

	} else {
		while (idx < size) {
			if (i2s_inst[num].regs->fifo_level <
			    KEEP_FIFO_LEVEL_EMPTY) {
				/* Check FIFO Level before pushing data into
				 * FIFO. FIFO Level Register is read only
				 * register.
				 */
				*dst = (__IO_RW uint32_t)src[idx++];
			}
		}
	}
	while (!i2s_inst[num].regs->stat.tdata_underr) {
		/* tdata_underr flag is updated by hardware.
		 * tdata_underr is set when FIFO is empty and
		 * cleared when data is pushed to FIFO.
		 */
	}

	return ret;
}

static void i2s_enable_interrupt(IN sedi_i2s_t num)
{
	I2S_DEBUG_TRACE("%s\n", __func__);

	i2s_inst[num].ctrl.i2s_mask = I2S_SET_BIT;
	i2s_inst[num].ctrl.fifo_empty_mask = I2S_CLEAR_BIT;
	i2s_inst[num].ctrl.fifo_aempty_mask = I2S_CLEAR_BIT;
	i2s_inst[num].ctrl.fifo_full_mask = I2S_CLEAR_BIT;
	i2s_inst[num].ctrl.fifo_afull_mask = I2S_CLEAR_BIT;
}

int i2s_disable_interrupt(IN sedi_i2s_t num, IN i2s_interrupt inter)
{
	I2S_ISR_DEBUG_TRACE("%s\n", __func__);
	DBG_CHECK(num < SEDI_I2S_MAX, SEDI_DRIVER_ERROR_PARAMETER);
	switch (inter) {
	case I2S_FIFO_EMPTY:
		i2s_inst[num].regs->ctrl.fifo_empty_mask = I2S_CLEAR_BIT;
		break;
	case I2S_FIFO_AEMPTY:
		i2s_inst[num].regs->ctrl.fifo_aempty_mask = I2S_CLEAR_BIT;
		break;
	case I2S_FIFO_FULL:
		i2s_inst[num].regs->ctrl.fifo_full_mask = I2S_CLEAR_BIT;
		break;
	case I2S_FIFO_AFULL:
		i2s_inst[num].regs->ctrl.fifo_afull_mask = I2S_CLEAR_BIT;
		break;
	case I2S_RFIFO_EMPTY:
		i2s_inst[num].regs->ctrl_fdr.rfifo_empty_mask = I2S_CLEAR_BIT;
		break;
	case I2S_RFIFO_AEMPTY:
		i2s_inst[num].regs->ctrl_fdr.rfifo_aempty_mask = I2S_CLEAR_BIT;
		break;
	case I2S_RFIFO_FULL:
		i2s_inst[num].regs->ctrl_fdr.rfifo_full_mask = I2S_CLEAR_BIT;
		break;
	case I2S_RFIFO_AFULL:
		i2s_inst[num].regs->ctrl_fdr.rfifo_afull_mask = I2S_CLEAR_BIT;
		break;
	default:
		I2S_DEBUG_PRINT("Invalid interrupt disable request for %d\n",
				num);
	}
	return SEDI_DRIVER_OK;
}

i2s_interrupt sedi_get_interrupt_type(IN sedi_i2s_t num)
{
	I2S_ISR_DEBUG_TRACE("%s\n", __func__);
	DBG_CHECK(num < SEDI_I2S_MAX, SEDI_DRIVER_ERROR_PARAMETER);
	return i2s_inst[num].regs->stat.bits;
}

int sedi_interrupt_clear(IN sedi_i2s_t num)
{
	I2S_ISR_DEBUG_TRACE("%s\n", __func__);
	DBG_CHECK(num < SEDI_I2S_MAX, SEDI_DRIVER_ERROR_PARAMETER);
#if I2S_ISR_DEBUG_TRACE_EN
	__IO_RW uint32_t *stat = &i2s_inst[num].regs->stat.bits;
#endif

	if (i2s_inst[num].regs->stat.rdata_ovrerr == I2S_BIT_HIGH) {
		i2s_inst[num].regs->stat.rdata_ovrerr = I2S_CLEAR_BIT;
		I2S_DEBUG_PRINT("FIFO Over Run\n");
	}

	i2s_inst[num].regs->stat.fifo_empty = I2S_CLEAR_BIT;
	i2s_inst[num].regs->stat.fifo_full = I2S_CLEAR_BIT;
	i2s_inst[num].regs->stat.fifo_afull = I2S_CLEAR_BIT;

	I2S_ISR_DEBUG_TRACE("FIFO_LEVEL reg: %x\n",
			    i2s_inst[num].regs->fifo_level);
	I2S_ISR_DEBUG_TRACE("stat reg: %x\n", *stat);
	return SEDI_DRIVER_OK;
}

static int sedi_i2s_set_mode(sedi_i2s_t num)
{
	I2S_DEBUG_TRACE("%s\n", __func__);
	switch (i2s_inst[num].i2s_config.mode) {
	case I2S_MASTER_TX:
		I2S_DEBUG_TRACE("I2S_MASTER_TX\n");
		i2s_inst[num].ctrl.full_duplex = I2S_CLEAR_BIT;
		i2s_inst[num].ctrl.dir_cfg = I2S_SET_BIT;
		i2s_inst[num].ctrl.ms_cfg = I2S_SET_BIT;
		break;
	case I2S_MASTER_RX:
		I2S_DEBUG_TRACE("I2S_MASTER_RX\n");
		i2s_inst[num].ctrl.full_duplex = I2S_CLEAR_BIT;
		i2s_inst[num].ctrl.dir_cfg = I2S_CLEAR_BIT;
		i2s_inst[num].ctrl.ms_cfg = I2S_SET_BIT;
		break;
	case I2S_SLAVE_TX:
		I2S_DEBUG_TRACE("I2S_SLAVE_TX\n");
		i2s_inst[num].ctrl.full_duplex = I2S_CLEAR_BIT;
		i2s_inst[num].ctrl.dir_cfg = I2S_SET_BIT;
		i2s_inst[num].ctrl.ms_cfg = I2S_CLEAR_BIT;
		break;
	case I2S_SLAVE_RX:
		I2S_DEBUG_TRACE("I2S_SLAVE_RX\n");
		i2s_inst[num].ctrl.full_duplex = I2S_CLEAR_BIT;
		i2s_inst[num].ctrl.dir_cfg = I2S_CLEAR_BIT;
		i2s_inst[num].ctrl.ms_cfg = I2S_CLEAR_BIT;
		break;
	case I2S_MASTER_FULL_DUPLEX:
		I2S_DEBUG_TRACE("I2S_MASTER_FULL_DUPLEX\n");
		i2s_inst[num].ctrl.full_duplex = I2S_SET_BIT;
		i2s_inst[num].ctrl.ms_cfg = I2S_SET_BIT;
		break;
	case I2S_SLAVE_FULL_DUPLEX:
		I2S_DEBUG_TRACE("I2S_SLAVE_FULL_DUPLEX\n");
		i2s_inst[num].ctrl.full_duplex = I2S_SET_BIT;
		i2s_inst[num].ctrl.ms_cfg = I2S_CLEAR_BIT;
		break;
	default:
		I2S_DEBUG_PRINT("[i2s]: Unknown operating mode\n");
		return SEDI_DRIVER_ERROR_PARAMETER;
	}
	return SEDI_DRIVER_OK;
}

static int sedi_i2s_set_channel_fmt(sedi_i2s_t num)
{
	I2S_DEBUG_TRACE("%s\n", __func__);

	I2S_DEBUG_PRINT("i2s_port:%d, channel_format: %d\n", num,
			i2s_inst[num].i2s_config.channel_format);

	i2s_inst[num].tdm_ctrl.tdm_en = I2S_CLEAR_BIT;
	i2s_inst[num].ctrl.data_ws_del = LEADING_DUMMY_BIT;

	switch (i2s_inst[num].i2s_config.channel_format) {
	case I2S_CHANNEL_FMT_RIGHT:
		I2S_DEBUG_TRACE("I2S_CHANNEL_FMT_RIGHT\n");
		i2s_inst[num].ctrl.ws_polar = I2S_CLEAR_BIT;
		i2s_inst[num].ctrl.sck_polar = I2S_CLEAR_BIT;
		i2s_inst[num].ctrl.data_align = I2S_SET_BIT;
		break;
	case I2S_CHANNEL_FMT_LEFT:
		I2S_DEBUG_TRACE("I2S_CHANNEL_FMT_LEFT\n");
		i2s_inst[num].ctrl.data_align = I2S_CLEAR_BIT;
		i2s_inst[num].ctrl.ws_polar = I2S_SET_BIT;
		i2s_inst[num].ctrl.sck_polar = I2S_CLEAR_BIT;
		break;
	case I2S_CHANNEL_FMT_STD:
		I2S_DEBUG_TRACE("I2S_CHANNEL_FMT_STD\n");
		i2s_inst[num].ctrl.ws_polar = I2S_CLEAR_BIT;
		i2s_inst[num].ctrl.sck_polar = I2S_CLEAR_BIT;
		i2s_inst[num].ctrl.data_align = I2S_SET_BIT;
		i2s_inst[num].ctrl.data_ws_del = LEADING_DUMMY_BIT_1;
		break;
	case I2S_CHANNEL_FMT_TDM:
		I2S_DEBUG_TRACE("I2S_CHANNEL_FMT_TDM\n");
		i2s_inst[num].ctrl.ws_polar = I2S_SET_BIT;
		i2s_inst[num].ctrl.sck_polar = I2S_SET_BIT;
		i2s_inst[num].tdm_ctrl.tdm_en = I2S_SET_BIT;
		i2s_inst[num].ctrl.data_align = I2S_SET_BIT;
		i2s_inst[num].ctrl.data_ws_del = LEADING_DUMMY_BIT_1;
		break;
	default:
		I2S_DEBUG_PRINT("[i2s]: Unknown operating mode\n");
		return SEDI_DRIVER_ERROR_PARAMETER;
	}

	/* PSE I2S Driver will not support DSP mode. */
	if (i2s_inst[num].i2s_config.channel_format == I2S_CHANNEL_FMT_TDM) {
		i2s_inst[num].ctrl.ws_mode = I2S_CLEAR_BIT;
	} else {
		i2s_inst[num].ctrl.ws_mode = I2S_SET_BIT;
	}

	if (i2s_inst[num].i2s_config.msb_first == I2S_BIT_HIGH) {
		i2s_inst[num].ctrl.data_order = I2S_CLEAR_BIT;
	} else {
		i2s_inst[num].ctrl.data_order = I2S_SET_BIT;
	}

	return SEDI_DRIVER_OK;
}

static int sedi_set_sample_resolution(sedi_i2s_t num)
{
	I2S_DEBUG_TRACE("%s\n", __func__);
	uint32_t max_res;
	uint32_t mode = i2s_inst[num].i2s_config.mode;

	if ((i2s_inst[num].i2s_config.bits >= I2S_12_BIT_SAMPLE) &&
	    (i2s_inst[num].i2s_config.bits <= I2S_32_BIT_SAMPLE)) {
		i2s_inst[num].regs->sres.resolution =
			(i2s_inst[num].i2s_config.bits - 1);
	} else {
		I2S_DEBUG_PRINT(" Invalid bits_per_sample.\n");
		return SEDI_DRIVER_ERROR_PARAMETER;
	}

	if ((mode == I2S_MASTER_FULL_DUPLEX) ||
	    (mode == I2S_SLAVE_FULL_DUPLEX)) {
		I2S_DEBUG_TRACE(
			"I2S_MASTER_FULL_DUPLEX or I2S_SLAVE_FULL_DUPLEX");
		if ((i2s_inst[num].i2s_config.rx_bits >= I2S_12_BIT_SAMPLE) ||
		    (i2s_inst[num].i2s_config.rx_bits <= I2S_32_BIT_SAMPLE)) {
			i2s_inst[num].regs->sres_fdr.rresolution =
				(i2s_inst[num].i2s_config.rx_bits - 1);
		} else {
			I2S_DEBUG_PRINT(" Invalid bits_per_sample.\n");
			return SEDI_DRIVER_ERROR_PARAMETER;
		}
	}

	max_res =
		i2s_inst[num].i2s_config.bits > i2s_inst[num].i2s_config.rx_bits
		? i2s_inst[num].i2s_config.bits
		: i2s_inst[num].i2s_config.rx_bits;

	I2S_DEBUG_PRINT("max_res %x\n", max_res);

	if (i2s_inst[num].i2s_config.channel_format == I2S_CHANNEL_FMT_LEFT ||
	    i2s_inst[num].i2s_config.channel_format == I2S_CHANNEL_FMT_RIGHT) {

		if (max_res <= I2S_16_BIT_SAMPLE) {
			max_res = I2S_16_BIT_SAMPLE;
		} else if (max_res <= I2S_32_BIT_SAMPLE) {
			max_res = I2S_32_BIT_SAMPLE;
		}
	}

	switch (max_res) {
	case I2S_12_BIT_SAMPLE:
		i2s_inst[num].ctrl.chn_width = I2S_CHNL_WIDTH_12;
		break;
	case I2S_16_BIT_SAMPLE:
		i2s_inst[num].ctrl.chn_width = I2S_CHNL_WIDTH_16;
		break;
	case I2S_24_BIT_SAMPLE:
		i2s_inst[num].ctrl.chn_width = I2S_CHNL_WIDTH_24;
		break;
	case I2S_32_BIT_SAMPLE:
		i2s_inst[num].ctrl.chn_width = I2S_CHNL_WIDTH_32;
		break;
	default:
		I2S_DEBUG_PRINT("wrong Bit per sample\n");
		return SEDI_DRIVER_ERROR_PARAMETER;
	}

	return SEDI_DRIVER_OK;
}

static int sedi_i2s_set_channel(sedi_i2s_t num)
{
	I2S_DEBUG_TRACE("%s\n", __func__);

	switch (i2s_inst[num].i2s_config.channel) {
	case I2S_MONO_CHANNEL_RIGHT:
		i2s_inst[num].ctrl.audio_mode = I2S_SET_BIT;
		i2s_inst[num].ctrl.mono_mode = I2S_SET_BIT;
		break;
	case I2S_MONO_CHANNEL_LEFT:
		i2s_inst[num].ctrl.audio_mode = I2S_SET_BIT;
		i2s_inst[num].ctrl.mono_mode = I2S_CLEAR_BIT;
		break;
	case I2S_STEREO_CAHNNEL:
		i2s_inst[num].ctrl.audio_mode = I2S_CLEAR_BIT;
		break;
	case I2S_6_CHANNEL:
		i2s_inst[num].tdm_ctrl.chn_no = (I2S_6_CHANNEL - 1);
		i2s_inst[num].tdm_ctrl.ch_slot = TDM_ACTIVE_TX_6_CHNL_SLOT;
		i2s_inst[num].tdm_fd_dir.ch_tx_slot = TDM_ACTIVE_TX_6_CHNL_SLOT;
		i2s_inst[num].tdm_fd_dir.ch_rx_slot = TDM_ACTIVE_RX_6_CHNL_SLOT;
		break;
	case I2S_8_CHANNEL:
		i2s_inst[num].tdm_ctrl.chn_no = (I2S_8_CHANNEL - 1);
		i2s_inst[num].tdm_ctrl.ch_slot = TDM_ACTIVE_TX_8_CHNL_SLOT;
		i2s_inst[num].tdm_fd_dir.ch_tx_slot = TDM_ACTIVE_TX_8_CHNL_SLOT;
		i2s_inst[num].tdm_fd_dir.ch_rx_slot = TDM_ACTIVE_RX_8_CHNL_SLOT;
		break;

	default:
		return SEDI_DRIVER_ERROR_PARAMETER;
	}
	return SEDI_DRIVER_OK;
}

static int sedi_i2s_set_clock(sedi_i2s_t num, uint32_t rate)
{
	I2S_DEBUG_TRACE("%s\n", __func__);
	uint32_t ch_width, ch;

	switch (i2s_inst[num].i2s_config.channel) {
	case I2S_MONO_CHANNEL_LEFT:
	case I2S_MONO_CHANNEL_RIGHT:
		ch = I2S_SET_BIT;
		break;
	default:
		ch = i2s_inst[num].i2s_config.channel;
	}

	switch (i2s_inst[num].ctrl.chn_width) {
	case I2S_CHNL_WIDTH_12:
		ch_width = I2S_12_BIT_SAMPLE;
		break;
	case I2S_CHNL_WIDTH_16:
		ch_width = I2S_16_BIT_SAMPLE;
		break;
	case I2S_CHNL_WIDTH_24:
		ch_width = I2S_24_BIT_SAMPLE;
		break;
	case I2S_CHNL_WIDTH_32:
		ch_width = I2S_32_BIT_SAMPLE;
		break;
	default:
		I2S_DEBUG_PRINT(" Unknown channel width %x for %d\n",
				i2s_inst[num].ctrl.chn_width, num);
		return SEDI_DRIVER_ERROR_PARAMETER;
	}

	i2s_inst[num].regs->srate.sample_rate =
		cal_sample_rate(num, ch_width, rate, ch);
	I2S_DEBUG_TRACE(" sample rate Register value %x\n",
			i2s_inst[num].regs->srate.sample_rate);

	return SEDI_DRIVER_OK;
}

static int sedi_i2s_set_sample_rate(sedi_i2s_t num, uint32_t rate)
{
	I2S_DEBUG_TRACE("%s\n", __func__);

	int ret = SEDI_DRIVER_OK;

	DBG_CHECK(num < SEDI_I2S_MAX, SEDI_DRIVER_ERROR_PARAMETER);

	ret = sedi_i2s_set_clock(num, rate);

	return ret;
}

static int sedi_i2s_set_polarity(sedi_i2s_t num)
{
	I2S_DEBUG_TRACE("%s\n", __func__);
	switch (i2s_inst[num].i2s_config.polariy) {
	case I2S_CLK_NF_NB:
		I2S_DEBUG_TRACE("I2S_CLK_NF_NB\n");
		i2s_inst[num].ctrl.sck_polar = I2S_SET_BIT;
		i2s_inst[num].ctrl.ws_polar = I2S_CLEAR_BIT;
		break;

	case I2S_CLK_NF_IB:
		I2S_DEBUG_TRACE("I2S_CLK_NF_IB\n");
		i2s_inst[num].ctrl.sck_polar = I2S_CLEAR_BIT;
		i2s_inst[num].ctrl.ws_polar = I2S_CLEAR_BIT;
		break;

	case I2S_CLK_IF_NB:
		I2S_DEBUG_TRACE("I2S_CLK_IF_NB\n");
		i2s_inst[num].ctrl.sck_polar = I2S_SET_BIT;
		i2s_inst[num].ctrl.ws_polar = I2S_SET_BIT;
		break;

	case I2S_CLK_IF_IB:
		I2S_DEBUG_TRACE("I2S_CLK_IF_IB\n");
		i2s_inst[num].ctrl.sck_polar = I2S_CLEAR_BIT;
		i2s_inst[num].ctrl.ws_polar = I2S_SET_BIT;
		break;
	}

	return SEDI_DRIVER_OK;
}

int sedi_i2s_config_dma(IN sedi_dma_t dma, IN uint32_t dir,
			IN sedi_i2s_dma_config_t *cfg)
{
	int ret;

	PARAM_UNUSED(dir);
	DBG_CHECK(cfg != NULL, SEDI_DRIVER_ERROR_PARAMETER);

	ret = sedi_dma_init(dma, cfg->channel, cfg->dma_callback, cfg->data);
	DBG_CHECK(ret == SEDI_DRIVER_OK, SEDI_DRIVER_ERROR);

	ret = sedi_dma_set_power(dma, cfg->channel, SEDI_POWER_FULL);
	DBG_CHECK(ret == SEDI_DRIVER_OK, SEDI_DRIVER_ERROR);

	ret = sedi_dma_control(dma, cfg->channel, SEDI_CONFIG_DMA_BURST_LENGTH,
			       DMA_BURST_TRANS_LENGTH_32);
	DBG_CHECK(ret == SEDI_DRIVER_OK, SEDI_DRIVER_ERROR);

	ret =
		sedi_dma_control(dma,
				 cfg->channel,
				 SEDI_CONFIG_DMA_SR_TRANS_WIDTH,
				 cfg->source_data_size);
	DBG_CHECK(ret == SEDI_DRIVER_OK, SEDI_DRIVER_ERROR);

	ret =
		sedi_dma_control(dma,
				 cfg->channel,
				 SEDI_CONFIG_DMA_DT_TRANS_WIDTH,
				 cfg->dest_data_size);
	DBG_CHECK(ret == SEDI_DRIVER_OK, SEDI_DRIVER_ERROR);

	ret = sedi_dma_control(dma, cfg->channel, SEDI_CONFIG_DMA_HS_DEVICE_ID,
			       cfg->dev_hwid);
	DBG_CHECK(ret == SEDI_DRIVER_OK, SEDI_DRIVER_ERROR);

	ret = sedi_dma_control(dma, cfg->channel, SEDI_CONFIG_DMA_HS_POLARITY,
			       DMA_HS_POLARITY_HIGH);
	DBG_CHECK(ret == SEDI_DRIVER_OK, SEDI_DRIVER_ERROR);

	ret = sedi_dma_control(dma, cfg->channel, SEDI_CONFIG_DMA_DIRECTION,
			       cfg->channel_direction);
	DBG_CHECK(ret == SEDI_DRIVER_OK, SEDI_DRIVER_ERROR);

	ret = sedi_dma_control(dma, cfg->channel,
			       SEDI_CONFIG_DMA_HS_DEVICE_ID_PER_DIR,
			       cfg->dev_hwid_dir);
	DBG_CHECK(ret == SEDI_DRIVER_OK, SEDI_DRIVER_ERROR);

	return SEDI_DRIVER_OK;
}

int sedi_i2s_dma_start(IN sedi_dma_t dma, IN sedi_i2s_dma_add_t *dma_add)
{
	int ret;

	DBG_CHECK(dma_add != NULL, SEDI_DRIVER_ERROR_PARAMETER);

	ret = sedi_dma_start_transfer(
		dma, dma_add->channel, dma_add->source_address,
		dma_add->dest_address, dma_add->block_size);
	if (ret != I2S_CLEAR_BIT) {
		I2S_DEBUG_PRINT("dma transfer failed\n");
		return ret;
	}
	return ret;
}

int sedi_i2s_dma_stop(IN sedi_dma_t dma, IN uint32_t channel)
{
	int ret;

	ret = sedi_dma_set_power(dma, channel, SEDI_POWER_OFF);
	DBG_CHECK(ret == SEDI_DRIVER_OK, SEDI_DRIVER_ERROR);
	return sedi_dma_uninit(dma, channel);
}

int32_t sedi_i2s_set_power(IN sedi_i2s_t i2s, IN sedi_power_state_t state)
{
	DBG_CHECK(i2s < SEDI_I2S_MAX, SEDI_DRIVER_ERROR_PARAMETER);

	driver_id_t id = DRIVER_ID_I2S0 + i2s;

	int32_t ret = SEDI_DRIVER_OK;

	switch (state) {
	case SEDI_POWER_FULL:
		pm_driver_start_trans(id);
		ret = sedi_i2s_config(i2s, NULL, true);
		break;

	case SEDI_POWER_SUSPEND:
		pm_driver_end_trans(id);
		break;

	case SEDI_POWER_FORCE_SUSPEND:
		pm_driver_end_trans(id);
		break;

	case SEDI_POWER_LOW:
		pm_driver_end_trans(id);
		break;
	case SEDI_POWER_OFF:
		ret = SEDI_DRIVER_ERROR_UNSUPPORTED;
		break;
	}
	return ret;
}

int sedi_i2s_init(IN sedi_i2s_t num, IN i2s_config_t *i2s_config)
{
	I2S_DEBUG_TRACE("%s port:%x\n", __func__, num);
	return sedi_i2s_config(num, i2s_config, false);
}

static int sedi_i2s_config(IN sedi_i2s_t num, IN i2s_config_t *i2s_config,
			   IN bool pm_exit)
{
	I2S_DEBUG_TRACE("%s port:%x\n", __func__, num);

	int ret;

	DBG_CHECK(num < SEDI_I2S_MAX, SEDI_DRIVER_ERROR_PARAMETER);

	if (!pm_exit) {
		i2s_inst[num].i2s_config = *i2s_config;
	}

	disable_transceiver(num);
	reset_i2s_instance(num);

	if (sedi_get_mn_divide(num)) {
		i2s_mndiv_inst[num].regs->mn_div_en.enable =
			i2s_inst[num].i2s_mn_div.mn_div_en.enable;
		i2s_mndiv_inst[num].regs->mval.val =
			i2s_inst[num].i2s_mn_div.mval.val;
		i2s_mndiv_inst[num].regs->nval.val =
			i2s_inst[num].i2s_mn_div.nval.val;
	}

	ret = sedi_i2s_set_mode(num);
	if (ret != SEDI_DRIVER_OK) {
		I2S_DEBUG_PRINT("Error in config\n");
		return ret;
	}

	ret = sedi_i2s_set_channel(num);
	if (ret != SEDI_DRIVER_OK) {
		I2S_DEBUG_PRINT("Error in config\n");
		return ret;
	}

	ret = sedi_i2s_set_channel_fmt(num);
	if (ret != SEDI_DRIVER_OK) {
		I2S_DEBUG_PRINT("Error in config\n");
		return ret;
	}

	sedi_i2s_set_polarity(num);

	ret = sedi_set_sample_resolution(num);
	if (ret != SEDI_DRIVER_OK) {
		I2S_DEBUG_PRINT("Error in sedi_set_sample_resolution\n");
		return ret;
	}

	i2s_inst[num].ctrl.i2s_en = I2S_CLEAR_BIT;

	i2s_inst[num].regs->stat.tdata_underr = I2S_CLEAR_BIT;
	i2s_inst[num].regs->stat.fifo_empty = I2S_CLEAR_BIT;
	i2s_inst[num].regs->stat.bits = I2S_CLEAR_BIT;
	i2s_inst[num].regs->tdm_ctrl = i2s_inst[num].tdm_ctrl;
	i2s_inst[num].regs->tdm_fd_dir = i2s_inst[num].tdm_fd_dir;

	I2S_DEBUG_PRINT(" sample_rate %d\n",
			i2s_inst[num].i2s_config.sample_rate);
	sedi_i2s_set_sample_rate(num, i2s_inst[num].i2s_config.sample_rate);

	sedi_i2s_set_fifos(num);
	i2s_enable_interrupt(num);

	return SEDI_DRIVER_OK;
}
