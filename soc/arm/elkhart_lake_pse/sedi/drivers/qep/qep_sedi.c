/*
 * Copyright (c) 2021 Intel Corporation
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "stdio.h"
#include <sedi_driver_common.h>
#include <sedi_driver_qep.h>
#include "qep_priv.h"
#include "pm/pm_internal_if.h"
#include "sedi_driver_ipc.h"

static sedi_qep_reg_t *const sedi_qep[SEDI_QEP_MAX] = {
	(sedi_qep_reg_t *)SEDI_QEP_0_BASE, (sedi_qep_reg_t *)SEDI_QEP_1_BASE,
	(sedi_qep_reg_t *)SEDI_QEP_2_BASE, (sedi_qep_reg_t *)SEDI_QEP_3_BASE
};

static bool wdt_enable[SEDI_QEP_MAX];

static sedi_qep_ctxt_t sedi_qep_context[SEDI_QEP_MAX];

static const sedi_qep_ctxt_t init_state = { .last_known_dir =
						    SEDI_QEP_DIR_UNKNOWN };

static void sedi_qep_reset_context(sedi_qep_t qep);

int sedi_qep_filter_config(IN sedi_qep_t qep, IN uint32_t glitch_width_ns);

static void qep_soft_rst(sedi_qep_t qep)
{
	volatile uint32_t value = read32(SEDI_QEP_SFT_RST_REG);

	write32(SEDI_QEP_SFT_RST_REG, (value | (1 << qep)));
	write32(SEDI_QEP_SFT_RST_REG, (value & (~(1 << qep))));
}

static inline void sedi_qep_clr_cap_fifo(sedi_qep_t qep)
{
	uint32_t time_stamp;
	sedi_qep_reg_t *regs = sedi_qep[qep];

	while (!(regs->qep_con & SEDI_QEP_CFG_FIFO_EMPTY)) {
		time_stamp = regs->qep_cap_buf;
	}
}

static inline int sedi_qep_is_enabled(sedi_qep_t qep)
{
	return (sedi_qep[qep]->qep_con & SEDI_QEP_CFG_EN);
}

static inline int sedi_qep_is_decode_selected(sedi_qep_t qep)
{
	return !(sedi_qep[qep]->qep_con & (SEDI_QEP_CFG_OP_MODE));
}

int sedi_qep_init(IN sedi_qep_t qep)
{
	DBG_CHECK(qep < SEDI_QEP_MAX, SEDI_DRIVER_ERROR_PARAMETER);

	if (sedi_get_dev_ownership(PSE_DEV_QEP0 + qep) != DEV_PSE_OWNED) {
		return SEDI_DRIVER_ERROR_NO_DEV;
	}

	qep_soft_rst(qep);
	return SEDI_DRIVER_OK;
}

int sedi_qep_config(IN sedi_qep_t qep, IN sedi_qep_config_t *config)
{
	DBG_CHECK(qep < SEDI_QEP_MAX, SEDI_DRIVER_ERROR_PARAMETER);
	DBG_CHECK(config != NULL, SEDI_DRIVER_ERROR_PARAMETER);

	int ret;

	sedi_qep_reg_t *qep_regs = sedi_qep[qep];

	/*Disable all interrupts*/
	sedi_qep_int_disable(qep, SEDI_QEP_INT_ALL);

	/* Disable QEP bofore making configuration changes.*/
	qep_regs->qep_con &= ~(SEDI_QEP_CFG_EN);

	/* Reset context to initial state.*/
	sedi_qep_reset_context(qep);

	if (config->swap_a_b == true) {
		qep_regs->qep_con |= SEDI_QEP_CFG_SWAP_A_B;
	} else {
		qep_regs->qep_con &= ~(SEDI_QEP_CFG_SWAP_A_B);
	}

	if (config->filter_en == true) {
		ret =
			sedi_qep_filter_config(qep, config->noise_filter_width_ns);
		if (ret != SEDI_DRIVER_OK) {
			return SEDI_DRIVER_ERROR_PARAMETER;
		}
		qep_regs->qep_con |= SEDI_QEP_CFG_FLT_EN;
	} else {
		qep_regs->qep_con &= ~(SEDI_QEP_CFG_FLT_EN);
		qep_regs->qep_flt = 0;
	}

	if (config->edge_phase_a == SEDI_QEP_FALLING_EDGE) {
		qep_regs->qep_con &= ~(SEDI_QEP_CFG_EDGE_SEL_A);
	} else {
		qep_regs->qep_con |= (SEDI_QEP_CFG_EDGE_SEL_A);
	}

	if (config->edge_phase_b == SEDI_QEP_FALLING_EDGE) {
		qep_regs->qep_con &= ~(SEDI_QEP_CFG_EDGE_SEL_B);
	} else {
		qep_regs->qep_con |= (SEDI_QEP_CFG_EDGE_SEL_B);
	}

	if (config->mode == SEDI_QEP_MODE_QEP_DECODER) {
		qep_regs->qep_con &= ~(SEDI_QEP_CFG_OP_MODE);

		if (config->edge_index == SEDI_QEP_FALLING_EDGE) {
			qep_regs->qep_con &= ~(SEDI_QEP_CFG_EDGE_SEL_IDX);
		} else {
			qep_regs->qep_con |= (SEDI_QEP_CFG_EDGE_SEL_IDX);
		}

		/* Set qep_max based on pulses per rotation. */
		qep_regs->qep_max =
			config->pulses_per_rev * SEDI_QEP_COUNTS_PER_PULSE;

		/* Set Reset mode. */
		if (config->ctr_rst_mode == SEDI_QEP_COUNTER_RESET_ON_INDEX) {
			qep_regs->qep_con &= ~(SEDI_QEP_CFG_CNT_RST_MODE);
			/* Index gating Select. */
			qep_regs->qep_con &=
				~(SEDI_QEP_CFG_IDX_GATE_SEL_MASK
					<< SEDI_QEP_CFG_IDX_GATE_SEL_OFFSET);
			qep_regs->qep_con |=
				(config->gating_index
					<< SEDI_QEP_CFG_IDX_GATE_SEL_OFFSET);

		} else {
			qep_regs->qep_con |= (SEDI_QEP_CFG_CNT_RST_MODE);
		}

		wdt_enable[qep] = config->wdt_en;

		/* Set WDT count value if provided. */
		qep_regs->qep_wdt =
			SEDI_QEP_USEC_TO_PCLK_CYCLES(config->wdt_usec);
	} else if (config->mode == SEDI_QEP_MODE_CAPTURE) {
		qep_regs->qep_con |= (SEDI_QEP_CFG_OP_MODE);
		qep_regs->qep_con &= ~(SEDI_QEP_CFG_CAP_FIFO_TH_MASK
				       << SEDI_QEP_CFG_CAP_FIFO_TH_OFFSET);
		qep_regs->qep_con |= (SEDI_QEP_CAP_FIFO_THRESHOLD
				      << SEDI_QEP_CFG_CAP_FIFO_TH_OFFSET);
		qep_regs->qep_cap_div = SEDI_QEP_CAP_CLK_DIV;

		if (config->capture_edge == SEDI_QEP_CAP_SINGLE_EDGE) {
			qep_regs->qep_con &= ~(SEDI_QEP_CFG_CAP_MODE);
		} else {
			qep_regs->qep_con |= (SEDI_QEP_CFG_CAP_MODE);
		}
	}

	return SEDI_DRIVER_OK;
}

int sedi_qep_int_enable(sedi_qep_t qep, uint32_t flag)
{
	DBG_CHECK(qep < SEDI_QEP_MAX, SEDI_DRIVER_ERROR_PARAMETER);
	sedi_qep[qep]->qep_int_mask &= ~(flag);
	return SEDI_DRIVER_OK;
}

int sedi_qep_int_disable(sedi_qep_t qep, uint32_t flag)
{
	DBG_CHECK(qep < SEDI_QEP_MAX, SEDI_DRIVER_ERROR_PARAMETER);
	sedi_qep[qep]->qep_int_mask |= flag;
	return SEDI_DRIVER_OK;
}

static inline void sedi_qep_int_clr(sedi_qep_t qep, uint32_t flag)
{
	sedi_qep[qep]->qep_int_stat &= flag;
}

/* Start QEP decoding */
int sedi_qep_start_decoding(sedi_qep_t qep)
{
	DBG_CHECK(qep < SEDI_QEP_MAX, SEDI_DRIVER_ERROR_PARAMETER);

	/* Check if configured for qep decode. */
	if (sedi_qep[qep]->qep_con & SEDI_QEP_CFG_OP_MODE) {
		return SEDI_DRIVER_ERROR_UNSUPPORTED;
	}

	/* Clear any pending interrupts. */
	sedi_qep_int_clr(qep, SEDI_QEP_INT_ALL);

	sedi_qep_int_enable(qep, SEDI_QEP_INT_RST_DN | SEDI_QEP_INT_RST_UP |
			    SEDI_QEP_INT_QEP_DIR |
			    SEDI_QEP_INT_FIFO_CRIT_PH_ERR);

	/* Enable wdt interrupts if it was enabled during config.*/
	if (wdt_enable[qep] == true) {
		sedi_qep_int_enable(qep, SEDI_QEP_INT_WDT);
	}

	/* Enable the qep module.*/
	sedi_qep[qep]->qep_con |= SEDI_QEP_CFG_EN;
	return SEDI_DRIVER_OK;
}

/* Stop QEP decoding.*/
int sedi_qep_stop_decoding(sedi_qep_t qep)
{
	DBG_CHECK(qep < SEDI_QEP_MAX, SEDI_DRIVER_ERROR_PARAMETER);

	/* Check if configured for qep decode. */
	if (sedi_qep[qep]->qep_con & SEDI_QEP_CFG_OP_MODE) {
		return SEDI_DRIVER_ERROR_UNSUPPORTED;
	}

	/* Stop only if QEP was running. */
	if (!sedi_qep_is_enabled(qep)) {
		return SEDI_DRIVER_ERROR_UNSUPPORTED;
	}

	sedi_qep[qep]->qep_con &= ~(SEDI_QEP_CFG_EN);
	sedi_qep_reset_context(qep);

	/* Disable all interrupts.*/
	sedi_qep_int_disable(qep, SEDI_QEP_INT_ALL);
	return SEDI_DRIVER_OK;
}

inline int sedi_qep_enable(sedi_qep_t qep)
{
	DBG_CHECK(qep < SEDI_QEP_MAX, SEDI_DRIVER_ERROR_PARAMETER);
	sedi_qep[qep]->qep_con |= SEDI_QEP_CFG_EN;
	return SEDI_DRIVER_OK;
}

inline int sedi_qep_disable(sedi_qep_t qep)
{
	DBG_CHECK(qep < SEDI_QEP_MAX, SEDI_DRIVER_ERROR_PARAMETER);
	sedi_qep[qep]->qep_con &= ~SEDI_QEP_CFG_EN;
	return SEDI_DRIVER_OK;
}

int sedi_qep_filter_config(IN sedi_qep_t qep, IN uint32_t glitch_width_ns)
{
	DBG_CHECK(qep < SEDI_QEP_MAX, SEDI_DRIVER_ERROR_PARAMETER);
	DBG_CHECK(glitch_width_ns < SEDI_QEP_MAX_FILTER_WIDTH_NS,
		  SEDI_DRIVER_ERROR_PARAMETER);

	uint32_t cycles = SEDI_QEP_NS_TO_PCLK_CYCLES(glitch_width_ns);

	if (cycles < SEDI_QEP_MIN_FILTER_WIDTH_CYCLES) {
		/* Minimum filter width is 2 cycles. */
		return SEDI_DRIVER_ERROR_PARAMETER;
	}

	sedi_qep[qep]->qep_flt = cycles - SEDI_QEP_MIN_FILTER_WIDTH_CYCLES;
	return SEDI_DRIVER_OK;
}

static void sedi_qep_reset_context(sedi_qep_t qep)
{
	/* Reset the context. */
	sedi_qep_context[qep] = init_state;
}

int sedi_qep_start_cap(sedi_qep_t qep, OUT uint64_t *buff,
		       IN uint32_t count)
{

	DBG_CHECK(qep < SEDI_QEP_MAX, SEDI_DRIVER_ERROR_PARAMETER);
	DBG_CHECK(buff != NULL, SEDI_DRIVER_ERROR_PARAMETER);
	/* Check if configured for capture mode. */
	if (!(sedi_qep[qep]->qep_con & SEDI_QEP_CFG_OP_MODE)) {
		return SEDI_DRIVER_ERROR_UNSUPPORTED;
	}

	/* If the controller is running, the active operation has to be
	 * stopped and controller disabled before capture can begin.
	 */
	if (sedi_qep_is_enabled(qep)) {
		return SEDI_DRIVER_ERROR_UNSUPPORTED;
	}

	sedi_qep_context[qep].cap_buff = buff;
	sedi_qep_context[qep].cap_term_count = count;
	sedi_qep_context[qep].cap_current_count = 0;
	sedi_qep_context[qep].cap_done_flag = 0;

	/* Clear any pending inetrrupts.*/
	sedi_qep_int_clr(qep, SEDI_QEP_INT_ALL);
	sedi_qep_int_enable(qep, SEDI_QEP_INT_FIFO_ENTRY |
			    SEDI_QEP_INT_FIFO_CRIT_PH_ERR);
	sedi_qep_enable(qep);
	return SEDI_DRIVER_OK;
}

void sedi_qep_int_handler(sedi_qep_t qep)
{
	sedi_qep_ctxt_t *qep_ctxt = &sedi_qep_context[qep];
	sedi_qep_reg_t *regs = sedi_qep[qep];
	uint32_t valid_ints = regs->qep_int_stat & (~regs->qep_int_mask);
	uint64_t time_ns;

	/* Unexpected interrupt generated, clear interrupt and disable qep */
	if (!sedi_qep_is_enabled(qep)) {
		sedi_qep_int_clr(qep, SEDI_QEP_INT_ALL);
		sedi_qep_int_disable(qep, SEDI_QEP_INT_ALL);
		return;
	}

	if (!valid_ints) {
		sedi_qep_int_clr(qep, SEDI_QEP_INT_ALL);
		return;
	}

	/* If int due to qep mode. */
	if (!(regs->qep_con & (SEDI_QEP_CFG_OP_MODE))) {
		if (valid_ints & SEDI_QEP_INT_WDT) {
			qep_ctxt->status_flag |= SEDI_QEP_INT_WDT;
			if (qep_ctxt->callback) {
				qep_ctxt->callback(qep_ctxt->usr_param,
						   SEDI_QEP_WDT_DETECTED, 0);
			}
		}
		if (valid_ints & SEDI_QEP_INT_RST_UP) {
			qep_ctxt->last_known_dir = SEDI_QEP_DIR_CW;
			if (qep_ctxt->callback) {
				qep_ctxt->callback(qep_ctxt->usr_param,
						   SEDI_QEP_RST_UP, 0);
			}
		}
		if (valid_ints & SEDI_QEP_INT_RST_DN) {
			qep_ctxt->last_known_dir = SEDI_QEP_DIR_CCW;
			if (qep_ctxt->callback) {
				qep_ctxt->callback(qep_ctxt->usr_param,
						   SEDI_QEP_RST_DN, 0);
			}
		}
		if (valid_ints & SEDI_QEP_INT_QEP_DIR) {
			if (qep_ctxt->callback) {
				qep_ctxt->callback(qep_ctxt->usr_param,
						   SEDI_QEP_DIR_CHANGE, 0);
			}
			if (qep_ctxt->last_known_dir == SEDI_QEP_DIR_CW) {
				qep_ctxt->last_known_dir = SEDI_QEP_DIR_CCW;
			} else if (qep_ctxt->last_known_dir ==
				   SEDI_QEP_DIR_CCW) {
				qep_ctxt->last_known_dir = SEDI_QEP_DIR_CW;
			}
			qep_ctxt->status_flag |= SEDI_QEP_INT_QEP_DIR;
		}
		if (valid_ints & SEDI_QEP_INT_FIFO_CRIT_PH_ERR) {
			if (qep_ctxt->callback) {
				qep_ctxt->callback(qep_ctxt->usr_param,
						   SEDI_QEP_PH_ERR, 0);
			}
		}
		sedi_qep_int_clr(qep, SEDI_QEP_DECODE_INTS);
	} else {
		/* There is an entry in the FIFO or the FIOF threshold
		 * has been reached.In both cases we read the FIFO
		 * till it becomes empty and transfer contents to user
		 * FIFO.
		 */

		while (!(regs->qep_con & SEDI_QEP_CFG_FIFO_EMPTY)) {
			if (qep_ctxt->cap_current_count ==
			    qep_ctxt->cap_term_count) {
				/* Disable the interrupts for cap. */
				sedi_qep_int_disable(qep,
						     SEDI_QEP_INT_FIFO_CRIT_PH_ERR |
						     SEDI_QEP_INT_FIFO_ENTRY);

				/* Clear the interrupts for CAP. */
				sedi_qep_int_clr(qep,
						 SEDI_QEP_INT_FIFO_CRIT_PH_ERR |
						 SEDI_QEP_INT_FIFO_ENTRY);

				/* Disabling QEP so that no more interrupts are
				 * generated.
				 */
				sedi_qep_disable(qep);
				qep_ctxt->cap_done_flag = true;
				if (qep_ctxt->callback) {
					qep_ctxt->callback(qep_ctxt->usr_param,
							   SEDI_QEP_CAP_DONE,
							   qep_ctxt->cap_current_count);
				}
				return;
			}
			time_ns = regs->qep_cap_buf * SEDI_QEP_FRC_CLK_RES_NS;
			qep_ctxt->cap_buff[qep_ctxt->cap_current_count++] =
				time_ns;
		}
		/* Clear the interrupts for CAP. */
		sedi_qep_int_clr(qep, SEDI_QEP_INT_FIFO_CRIT_PH_ERR |
				 SEDI_QEP_INT_FIFO_ENTRY);
	}
}

int sedi_qep_wdt_config(IN sedi_qep_t qep, IN uint32_t timeout_usec)
{
	DBG_CHECK(qep < SEDI_QEP_MAX, SEDI_DRIVER_ERROR_PARAMETER);
	if (sedi_qep_is_enabled(qep)) {
		return SEDI_DRIVER_ERROR_UNSUPPORTED;
	}
	sedi_qep[qep]->qep_wdt = SEDI_QEP_USEC_TO_PCLK_CYCLES(timeout_usec);
	return SEDI_DRIVER_OK;
}

int sedi_qep_get_position(IN sedi_qep_t qep, OUT uint32_t *p_position)
{
	DBG_CHECK(qep < SEDI_QEP_MAX, SEDI_DRIVER_ERROR_PARAMETER);
	DBG_CHECK(p_position != NULL, SEDI_DRIVER_ERROR_PARAMETER);

	if (sedi_qep_is_decode_selected(qep) == true) {
		*p_position = sedi_qep[qep]->qep_count;
		return SEDI_DRIVER_OK;
	} else {
		return SEDI_DRIVER_ERROR_UNSUPPORTED;
	}
}

int sedi_qep_get_direction(IN sedi_qep_t qep,
			   OUT sedi_qep_dir_t *p_direction)
{
	DBG_CHECK(qep < SEDI_QEP_MAX, SEDI_DRIVER_ERROR_PARAMETER);
	DBG_CHECK(p_direction != NULL, SEDI_DRIVER_ERROR_PARAMETER);
	if (sedi_qep_is_decode_selected(qep) == true) {
		*p_direction = sedi_qep_context[qep].last_known_dir;
		return SEDI_DRIVER_OK;
	} else {
		return SEDI_DRIVER_ERROR_UNSUPPORTED;
	}
}

int sedi_qep_register_callback(IN sedi_qep_t qep,
			       IN sedi_qep_callback_t callback,
			       IN void *usr_param)
{
	DBG_CHECK(qep < SEDI_QEP_MAX, SEDI_DRIVER_ERROR_PARAMETER);
	sedi_qep_context[qep].callback = callback;
	sedi_qep_context[qep].usr_param = usr_param;
	return SEDI_DRIVER_OK;
}

int sedi_qep_wdt_enable(IN sedi_qep_t qep)
{

	DBG_CHECK(qep < SEDI_QEP_MAX, SEDI_DRIVER_ERROR_PARAMETER);
	sedi_qep_int_enable(qep, SEDI_QEP_INT_WDT);
	wdt_enable[qep] = true;
	return SEDI_DRIVER_OK;
}

int sedi_qep_get_phase_err(IN sedi_qep_t qep, OUT uint32_t *p_ph_err)
{
	DBG_CHECK(qep < SEDI_QEP_MAX, SEDI_DRIVER_ERROR_PARAMETER);
	DBG_CHECK(p_ph_err != NULL, SEDI_DRIVER_ERROR_PARAMETER);

	if (sedi_qep_is_decode_selected(qep) == true) {
		*p_ph_err = !!(sedi_qep[qep]->qep_int_stat &
			       SEDI_QEP_INT_FIFO_CRIT_PH_ERR);
		return SEDI_DRIVER_OK;
	} else {
		return SEDI_DRIVER_ERROR_UNSUPPORTED;
	}
}

int sedi_qep_stop_cap(IN sedi_qep_t qep)
{
	DBG_CHECK(qep < SEDI_QEP_MAX, SEDI_DRIVER_ERROR_PARAMETER);
	sedi_qep_ctxt_t *qep_ctxt = &sedi_qep_context[qep];

	if (!sedi_qep_is_enabled(qep)) {
		return SEDI_DRIVER_ERROR_UNSUPPORTED;
	}

	sedi_qep_disable(qep);
	sedi_qep_int_disable(qep, SEDI_QEP_INT_ALL);

	if (qep_ctxt->callback) {
		qep_ctxt->callback(qep_ctxt->usr_param, SEDI_QEP_CAP_CANCELLED,
				   qep_ctxt->cap_current_count);
	}
	sedi_qep_reset_context(qep);
	return SEDI_DRIVER_OK;
}

int sedi_qep_wdt_disable(IN sedi_qep_t qep)
{
	DBG_CHECK(qep < SEDI_QEP_MAX, SEDI_DRIVER_ERROR_PARAMETER);
	sedi_qep_int_disable(qep, SEDI_QEP_INT_WDT);
	wdt_enable[qep] = false;
	return SEDI_DRIVER_OK;
}

int sedi_qep_filter_enable(IN sedi_qep_t qep)
{
	DBG_CHECK(qep < SEDI_QEP_MAX, SEDI_DRIVER_ERROR_PARAMETER);
	sedi_qep_reg_t *qep_regs = sedi_qep[qep];

	if (sedi_qep_is_enabled(qep)) {
		return SEDI_DRIVER_ERROR_UNSUPPORTED;
	}
	qep_regs->qep_con |= SEDI_QEP_CFG_FLT_EN;
	return SEDI_DRIVER_OK;
}

int sedi_qep_filter_disable(IN sedi_qep_t qep)
{
	DBG_CHECK(qep < SEDI_QEP_MAX, SEDI_DRIVER_ERROR_PARAMETER);
	sedi_qep_reg_t *qep_regs = sedi_qep[qep];

	if (sedi_qep_is_enabled(qep)) {
		return SEDI_DRIVER_ERROR_UNSUPPORTED;
	}

	qep_regs->qep_con &= (~SEDI_QEP_CFG_FLT_EN);
	return SEDI_DRIVER_OK;
}

int sedi_qep_disable_event(sedi_qep_t qep, sedi_qep_event_t event)
{
	DBG_CHECK(qep < SEDI_QEP_MAX, SEDI_DRIVER_ERROR_PARAMETER);

	if (sedi_qep_is_decode_selected(qep) == false) {
		return SEDI_DRIVER_ERROR_UNSUPPORTED;
	}
	switch (event) {
	case SEDI_QEP_WDT_DETECTED:
		wdt_enable[qep] = false;
		sedi_qep_int_disable(qep, SEDI_QEP_INT_WDT);
		break;

	case SEDI_QEP_DIR_CHANGE:
		sedi_qep_int_disable(qep, SEDI_QEP_INT_QEP_DIR);
		break;

	case SEDI_QEP_RST_UP:
		sedi_qep_int_disable(qep, SEDI_QEP_INT_RST_UP);
		break;

	case SEDI_QEP_RST_DN:
		sedi_qep_int_disable(qep, SEDI_QEP_INT_RST_DN);
		break;

	case SEDI_QEP_PH_ERR:
		sedi_qep_int_disable(qep, SEDI_QEP_INT_FIFO_CRIT_PH_ERR);
		break;

	default:
		return SEDI_DRIVER_ERROR_PARAMETER;
	}

	return SEDI_DRIVER_OK;
}

int sedi_qep_enable_event(sedi_qep_t qep, sedi_qep_event_t event)
{
	DBG_CHECK(qep < SEDI_QEP_MAX, SEDI_DRIVER_ERROR_PARAMETER);

	if (sedi_qep_is_decode_selected(qep) == false) {
		return SEDI_DRIVER_ERROR_UNSUPPORTED;
	}
	switch (event) {
	case SEDI_QEP_WDT_DETECTED:
		wdt_enable[qep] = true;
		sedi_qep_int_enable(qep, SEDI_QEP_INT_WDT);
		break;

	case SEDI_QEP_DIR_CHANGE:
		sedi_qep_int_enable(qep, SEDI_QEP_INT_QEP_DIR);
		break;

	case SEDI_QEP_RST_UP:
		sedi_qep_int_enable(qep, SEDI_QEP_INT_RST_UP);
		break;

	case SEDI_QEP_RST_DN:
		sedi_qep_int_enable(qep, SEDI_QEP_INT_RST_DN);
		break;

	case SEDI_QEP_PH_ERR:
		sedi_qep_int_enable(qep, SEDI_QEP_INT_FIFO_CRIT_PH_ERR);
		break;

	default:
		return SEDI_DRIVER_ERROR_PARAMETER;
	}

	return SEDI_DRIVER_OK;
}

static void sedi_qep_save_regs(sedi_qep_t qep)
{
	sedi_qep_reg_t *regs = sedi_qep[qep];

	sedi_qep_context[qep].saved_regs.qep_con = regs->qep_con;
	sedi_qep_context[qep].saved_regs.qep_flt = regs->qep_flt;
	sedi_qep_context[qep].saved_regs.qep_max = regs->qep_max;
	sedi_qep_context[qep].saved_regs.qep_wdt = regs->qep_wdt;
	sedi_qep_context[qep].saved_regs.qep_cap_div = regs->qep_cap_div;
	sedi_qep_context[qep].saved_regs.qep_int_mask = regs->qep_int_mask;
	sedi_qep_context[qep].regs_valid = true;
}

static void sedi_qep_restore_regs(sedi_qep_t qep)
{
	/* Restore context if there was any */
	if (sedi_qep_context[qep].regs_valid == true) {
		sedi_qep_reg_t *regs = sedi_qep[qep];

		/* Disable qep to make registers writable */
		regs->qep_con &= ~SEDI_QEP_CFG_EN;
		regs->qep_flt = sedi_qep_context[qep].saved_regs.qep_flt;
		regs->qep_max = sedi_qep_context[qep].saved_regs.qep_max;
		regs->qep_wdt = sedi_qep_context[qep].saved_regs.qep_wdt;
		regs->qep_cap_div =
			sedi_qep_context[qep].saved_regs.qep_cap_div;
		regs->qep_int_mask =
			sedi_qep_context[qep].saved_regs.qep_int_mask;
		regs->qep_con = sedi_qep_context[qep].saved_regs.qep_con;
		sedi_qep_context[qep].regs_valid = false;
	}
}

int sedi_qep_set_power(IN sedi_qep_t qep, IN sedi_power_state_t state)
{
	DBG_CHECK(qep < SEDI_QEP_MAX, SEDI_DRIVER_ERROR_PARAMETER);

	driver_id_t id = DRIVER_ID_QEP0 + qep;
	int32_t ret = SEDI_DRIVER_OK;

	switch (state) {
	case SEDI_POWER_FULL:

		/* Enable clock gates */
		pm_driver_start_trans(id);

		/* Restore regsiters if there was a saved context */
		sedi_qep_restore_regs(qep);
		break;

	case SEDI_POWER_SUSPEND:
		sedi_qep_save_regs(qep);
		pm_driver_end_trans(id);
		break;

	case SEDI_POWER_FORCE_SUSPEND:
		pm_driver_end_trans(id);
		break;

	case SEDI_POWER_LOW:

		/* Clock gating for qep */
		pm_driver_end_trans(id);
		break;
	case SEDI_POWER_OFF:
		ret = SEDI_DRIVER_ERROR_UNSUPPORTED;
		break;
	}
	return ret;
}
