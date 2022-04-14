/*
 * Copyright (c) 2021 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <sedi_driver_tgpio.h>
#include "sedi_tgpio.h"
#include "pm/pm_internal_if.h"
#include "sedi_driver_ipc.h"

/* Global resources */
static tgpio_resource_t tgpio_resource[SEDI_TGPIO_NUM] = {
    {.regs = (tgpio_regs_t *)SEDI_TGPIO_0_BASE},
    {.regs = (tgpio_regs_t *)SEDI_TGPIO_1_BASE},
};

/* Global runtime */
static tgpio_runtime_t tgpio_runtime[SEDI_TGPIO_NUM];
static inline void art_cycles_to_timespec(IN uint64_t art_cycles,
					  INOUT sedi_tgpio_time_t *tgpio_time)
{
	uint64_t art_cycles_khz = art_cycles / KHZ;
	uint32_t art_cycles_khz_rem = art_cycles % KHZ;
	uint64_t total_nsec = art_cycles_khz * ART_PERIOD;
	total_nsec = total_nsec + (art_cycles_khz_rem * ART_PERIOD) / KHZ;

	tgpio_time->sec = total_nsec / NSEC_PER_SEC;
	tgpio_time->nsec = total_nsec % NSEC_PER_SEC;
}

static inline void timespec_to_art_cycles(IN sedi_tgpio_time_t tgpio_time,
					  INOUT uint64_t *art_cycles)
{
	uint64_t total_nsec =
	    ((uint64_t)NSEC_PER_SEC * tgpio_time.sec) + tgpio_time.nsec;

	*art_cycles = (total_nsec / ART_PERIOD) * KHZ;
}

static void sedi_tgpio_save_regs(sedi_tgpio_t instance)
{
	int i;
	uint64_t art_cycles;
	sedi_tgpio_time_t local_time, reference_time;

	tgpio_regs_t *src = tgpio_resource[instance].regs;
	tgpio_regs_t *dst = &tgpio_runtime[instance].regs_saved;

	/* Save IRQ registers */
	dst->tgpio_intr_regs.tgpio_intr_ctl =
	    src->tgpio_intr_regs.tgpio_intr_ctl;

	/* Save other registers */
	dst->tgpio_clk_sel = src->tgpio_clk_sel;
	dst->tgpio_xtal_clk_gate = src->tgpio_xtal_clk_gate;
	dst->tgpio_ptp_clk_gate = src->tgpio_ptp_clk_gate;
	dst->tgpio_ts_sel[0] = src->tgpio_ts_sel[0];
	dst->tgpio_ts_sel[1] = src->tgpio_ts_sel[1];
	dst->tgpio_tmt_clk_sel = src->tgpio_tmt_clk_sel;
	dst->tgpio_cts_enable = src->tgpio_cts_enable;
	dst->tgpio_cts_valid = src->tgpio_cts_valid;

	/* Save TMT registers */
	for (i = 0; i < TGPIO_MAX_TIMERS; i++) {
		sedi_tgpio_get_cross_timestamp(instance, i, SEDI_TGPIO_ART,
					       &local_time, &reference_time);
		art_cycles = src->tgpio_tmt[i].tmt_art_high;
		art_cycles =
		    (art_cycles << REG_WIDTH) + src->tgpio_tmt[i].tmt_art_low;
		art_cycles_to_timespec(art_cycles, &reference_time);
		dst->tgpio_tmt[i].tmt_r = src->tgpio_tmt[i].tmt_r;
		if (reference_time.nsec < src->tgpio_tmt[i].tmt_l) {
			reference_time.sec--;
		}
		dst->tgpio_tmt[i].tmt_l =
		    reference_time.nsec - src->tgpio_tmt[i].tmt_l;
		dst->tgpio_tmt[i].tmt_h =
		    reference_time.sec - src->tgpio_tmt[i].tmt_h;
		dst->tgpio_tmt[i].tmt_timinca = src->tgpio_tmt[i].tmt_timinca;
		dst->tgpio_tmt[i].tmt_timadj = src->tgpio_tmt[i].tmt_timadj;
	}

	tgpio_runtime[instance].regs_valid = true;
}

static void sedi_tgpio_restore_regs(sedi_tgpio_t instance)
{
	int i;
	sedi_tgpio_time_t local_time, reference_time;
	tgpio_regs_t *src = &tgpio_runtime[instance].regs_saved;
	tgpio_regs_t *dst = tgpio_resource[instance].regs;

	if (tgpio_runtime[instance].regs_valid == false)
		return;

	/* Restore other registers */
	dst->tgpio_clk_sel = src->tgpio_clk_sel;
	dst->tgpio_xtal_clk_gate = src->tgpio_xtal_clk_gate;
	dst->tgpio_ptp_clk_gate = src->tgpio_ptp_clk_gate;
	dst->tgpio_ts_sel[0] = src->tgpio_ts_sel[0];
	dst->tgpio_ts_sel[1] = src->tgpio_ts_sel[1];
	dst->tgpio_tmt_clk_sel = src->tgpio_tmt_clk_sel;
	dst->tgpio_cts_enable = src->tgpio_cts_enable;
	dst->tgpio_cts_valid = src->tgpio_cts_valid;

	/* Restore TMT registers */
	for (i = 0; i < TGPIO_MAX_TIMERS; i++) {
		dst->tgpio_tmt[i].tmt_ctl = src->tgpio_tmt[i].tmt_ctl;
		sedi_tgpio_get_cross_timestamp(instance, i, SEDI_TGPIO_ART,
					       &local_time, &reference_time);
		dst->tgpio_tmt[i].tmt_r = src->tgpio_tmt[i].tmt_r;
		if (reference_time.nsec < src->tgpio_tmt[i].tmt_l) {
			reference_time.sec--;
		}
		dst->tgpio_tmt[i].tmt_l =
		    reference_time.nsec - src->tgpio_tmt[i].tmt_l;
		dst->tgpio_tmt[i].tmt_h =
		    reference_time.sec - src->tgpio_tmt[i].tmt_h;
		dst->tgpio_tmt[i].tmt_timinca = src->tgpio_tmt[i].tmt_timinca;
		dst->tgpio_tmt[i].tmt_timadj = src->tgpio_tmt[i].tmt_timadj;
		dst->tgpio_tmt[i].tmt_ctl = src->tgpio_tmt[i].tmt_ctl;
	}

	/* Restore IRQ registers */
	dst->tgpio_intr_regs.tgpio_intr_icr = 0xffffffff;
	dst->tgpio_intr_regs.tgpio_intr_imsc = 0;
	dst->tgpio_intr_regs.tgpio_intr_ctl =
	    src->tgpio_intr_regs.tgpio_intr_ctl;

	tgpio_runtime[instance].regs_valid = false;
}

int32_t sedi_tgpio_set_power(IN sedi_tgpio_t instance,
			     IN sedi_power_state_t state)
{
	SEDI_ASSERT(instance < SEDI_TGPIO_NUM);

	int32_t ret = SEDI_DRIVER_OK;
	driver_id_t id = DRIVER_ID_TGPIO0 + instance;

	switch (state) {
	case SEDI_POWER_FULL:
		/* Enable clock gates */
		pm_driver_start_trans(id);

		/* Restore registers if there was a saved context */
		sedi_tgpio_restore_regs(instance);
		break;
	case SEDI_POWER_FORCE_SUSPEND:
	case SEDI_POWER_SUSPEND:
		sedi_tgpio_save_regs(instance);
		pm_driver_end_trans(id);
		break;
	case SEDI_POWER_LOW:
		/* Clock gating for tgpio */
		pm_driver_end_trans(id);
		break;
	case SEDI_POWER_OFF:
		ret = SEDI_DRIVER_ERROR_UNSUPPORTED;
		break;
	}
	return ret;
}

static inline uint32_t gpio_status(IN uint32_t status_reg, IN uint32_t pinmux)
{
	uint32_t status = status_reg;
	/*
	 * Map TGPIO status bits to GPIO status bits
	 * Use will see only GPIO pins
	 */
	switch (pinmux) {
	case SEDI_TGPIO_PINMUX_1:
		status = (status << MUX_SHIFT_SIZE);
		break;
	case SEDI_TGPIO_PINMUX_2:
		status = (status & TGPIO_PINMUX_2_MASK);
		break;
	case SEDI_TGPIO_PINMUX_3:
		status = (status & LOWER_10_BITS_MASK) |
			 ((status << MUX_SHIFT_SIZE) & UPPER_10_BITS_MASK);
		break;
	default:
		status = FALSE;
	}
	return status;
}

static inline int32_t tgpio_find_pin(IN sedi_tgpio_t instance,
				     IN sedi_gpio_pin_t gpio_pin)
{
	SEDI_ASSERT(instance < SEDI_TGPIO_NUM);
	SEDI_ASSERT(gpio_pin < SEDI_GPIO_PIN_NUM);

	int32_t tgpio_pin, pmx;
	pmx = read32((instance) ? TGPIO_MUX_SEL_REG_1_LH2OSE
				: TGPIO_MUX_SEL_REG_0_LH2OSE);

	/* Check if the pin is a valid tgpio pin */
	switch (pmx) {
	case SEDI_TGPIO_PINMUX_1:
		tgpio_pin = (BIT(gpio_pin) & ~TGPIO_PINMUX_1_MASK)
				? SEDI_DRIVER_ERROR_PARAMETER
				: (gpio_pin - MUX_SHIFT_SIZE);
		break;
	case SEDI_TGPIO_PINMUX_2:
		tgpio_pin = (BIT(gpio_pin) & ~TGPIO_PINMUX_2_MASK)
				? SEDI_DRIVER_ERROR_PARAMETER
				: gpio_pin;
		break;
	case SEDI_TGPIO_PINMUX_3:
		tgpio_pin = (BIT(gpio_pin) & ~TGPIO_PINMUX_3_MASK)
				? SEDI_DRIVER_ERROR_PARAMETER
				: (gpio_pin < MUX_SHIFT_SIZE)
				      ? gpio_pin
				      : (gpio_pin - MUX_SHIFT_SIZE);
		break;
	default:
		tgpio_pin = SEDI_DRIVER_ERROR_PARAMETER;
	}

	if (tgpio_pin >= TGPIO_MAX_PIN_NUM) {
		tgpio_pin = SEDI_DRIVER_ERROR_PARAMETER;
	}

	return tgpio_pin;
}

static inline int32_t config_ts_sel(OUT tgpio_regs_t *base, IN uint32_t pin,
				    IN sedi_tgpio_timer_t timer)
{
	SEDI_ASSERT(base != NULL);

	uint32_t tgpio_pin = (pin % TS_SEL_REG_LIMIT) * BITS_PER_PIN;
	uint32_t index = pin / TS_SEL_REG_LIMIT;
	switch (timer) {
	case SEDI_TGPIO_TMT_0:
		base->tgpio_ts_sel[index] &=
		    ~(BIT(tgpio_pin) | BIT(tgpio_pin + 1));
		break;
	case SEDI_TGPIO_TMT_1:
		base->tgpio_ts_sel[index] |= BIT(tgpio_pin);
		base->tgpio_ts_sel[index] &= ~BIT(tgpio_pin + 1);
		break;
	case SEDI_TGPIO_TMT_2:
		base->tgpio_ts_sel[index] &= ~BIT(tgpio_pin);
		base->tgpio_ts_sel[index] |= BIT(tgpio_pin + 1);
		break;
	case SEDI_TGPIO_ART:
		base->tgpio_ts_sel[index] |=
		    (BIT(tgpio_pin) | BIT(tgpio_pin + 1));
		break;
	default:
		return SEDI_DRIVER_ERROR_PARAMETER;
	}
	return SEDI_DRIVER_OK;
}

void sedi_tgpio_isr_handler(IN sedi_tgpio_t instance)
{
	SEDI_ASSERT(instance < SEDI_TGPIO_NUM);

	uint32_t imsc, status, pmx;
	tgpio_regs_t *base = tgpio_resource[instance].regs;

	/* Save and mask all interrupts */
	imsc = base->tgpio_intr_regs.tgpio_intr_imsc;
	base->tgpio_intr_regs.tgpio_intr_imsc = 0;

	pmx = read32((instance) ? TGPIO_MUX_SEL_REG_1_LH2OSE
				: TGPIO_MUX_SEL_REG_0_LH2OSE);

	status = base->tgpio_intr_regs.tgpio_intr_mis;

	if (tgpio_runtime[instance].cb) {
		tgpio_runtime[instance].cb(tgpio_runtime[instance].cb_data,
					   gpio_status(status, pmx));
	}

	/* Clear and unmask interrupts */
	base->tgpio_intr_regs.tgpio_intr_icr = status;
	base->tgpio_intr_regs.tgpio_intr_imsc = imsc;
}

SEDI_ISR_DECLARE(sedi_tgpio_0_isr)
{
	sedi_tgpio_isr_handler(SEDI_TGPIO_0);
}

SEDI_ISR_DECLARE(sedi_tgpio_1_isr)
{
	sedi_tgpio_isr_handler(SEDI_TGPIO_1);
}

int32_t sedi_tgpio_pin_enable(IN sedi_tgpio_t instance,
			      IN sedi_gpio_pin_t gpio_pin)
{
	SEDI_ASSERT(instance < SEDI_TGPIO_NUM);
	SEDI_ASSERT(gpio_pin < SEDI_GPIO_PIN_NUM);

	/* Check if the pin belongs to TGPIO */
	int32_t tgpio_pin = tgpio_find_pin(instance, gpio_pin);
	if (tgpio_pin < 0) {
		return SEDI_DRIVER_ERROR_PARAMETER;
	}

	tgpio_resource[instance].regs->tgpio_pin[tgpio_pin].tgpio_ctl |=
	    (TGPIO_PIN_ENABLE);

	return SEDI_DRIVER_OK;
}

int32_t sedi_tgpio_pin_disable(IN sedi_tgpio_t instance,
			       IN sedi_gpio_pin_t gpio_pin)
{
	SEDI_ASSERT(instance < SEDI_TGPIO_NUM);
	SEDI_ASSERT(gpio_pin < SEDI_GPIO_PIN_NUM);

	/* Check if the pin belongs to TGPIO */
	int32_t tgpio_pin = tgpio_find_pin(instance, gpio_pin);
	if (tgpio_pin < 0) {
		return SEDI_DRIVER_ERROR_PARAMETER;
	}

	tgpio_resource[instance].regs->tgpio_pin[tgpio_pin].tgpio_ctl &=
	    ~(TGPIO_PIN_ENABLE);

	return SEDI_DRIVER_OK;
}

static inline int32_t tgpio_config_input(INOUT uint32_t *ctrl_reg,
					 INOUT tgpio_pin_regs_t *pin,
					 IN sedi_tgpio_pin_config_t cfg)
{

	uint32_t control_reg = 0;
	uint64_t art_cycles;

	if (cfg.events_ceiling) {
		/* Mode: Input, Periodic, Count based interrupt */
		/* Input Counter Select. COMPV and EC will match */
		control_reg |= TGPIO_PIN_INPUT_CNTR_SLCT;
		/* Enable event counting */
		control_reg |= TGPIO_PIN_INPUT_REPEAT;
		/* Enable reapeated interrupt generation */
		control_reg |= TGPIO_PIN_PERIODIC;
		pin->tgpio_compv_31_0 = cfg.events_ceiling;
		pin->tgpio_compv_63_32 = cfg.events_ceiling >> REG_WIDTH;

		pin->tgpio_piv_31_0 = cfg.events_ceiling;
		pin->tgpio_piv_63_32 = cfg.events_ceiling >> REG_WIDTH;
	} else if (cfg.start_time.nsec || cfg.start_time.sec) {
		if ((cfg.timer > TGPIO_MAX_TIMERS) ||
		    (cfg.repeat_interval.nsec > TGPIO_MAX_NS) ||
		    (cfg.start_time.nsec > TGPIO_MAX_NS)) {
			return SEDI_DRIVER_ERROR_PARAMETER;
		}

		/* Enable event counting */
		control_reg |= TGPIO_PIN_INPUT_REPEAT;
		/* Enable reapeated interrupt generation */
		control_reg |= TGPIO_PIN_PERIODIC;

		if (cfg.timer == SEDI_TGPIO_ART) {
			timespec_to_art_cycles(cfg.start_time, &art_cycles);
			pin->tgpio_compv_31_0 = art_cycles;
			pin->tgpio_compv_63_32 = art_cycles >> REG_WIDTH;

			timespec_to_art_cycles(cfg.repeat_interval,
					       &art_cycles);
			pin->tgpio_piv_31_0 = art_cycles;
			pin->tgpio_piv_63_32 = art_cycles >> REG_WIDTH;
		} else {
			pin->tgpio_compv_31_0 = cfg.start_time.nsec;
			pin->tgpio_compv_63_32 = cfg.start_time.sec;
			pin->tgpio_piv_31_0 = cfg.repeat_interval.nsec;
			pin->tgpio_piv_63_32 = cfg.repeat_interval.sec;
		}
	} else {

		return SEDI_DRIVER_ERROR_PARAMETER;
	}

	/* Periodic sequence length bit - 17-24. */
	control_reg |= (REPEAT_COUNT_MAX < cfg.repeat_count)
			   ? (REPEAT_COUNT_MAX << TGPIO_N_REPEATS_BIT)
			   : (cfg.repeat_count << TGPIO_N_REPEATS_BIT);

	pin->tgpio_mask_31_0 = 0;
	control_reg |= TGPIO_PIN_DIRECTION;

	*ctrl_reg = control_reg;

	return SEDI_DRIVER_OK;
}

static inline int32_t tgpio_config_output(INOUT uint32_t *ctrl_reg,
					  INOUT tgpio_pin_regs_t *pin,
					  IN sedi_tgpio_pin_config_t cfg)
{
	uint32_t control_reg = 0;
	uint64_t art_cycles;

	if (cfg.start_time.nsec || cfg.start_time.sec) {
		if ((cfg.timer > TGPIO_MAX_TIMERS) ||
		    (cfg.repeat_interval.nsec > TGPIO_MAX_NS) ||
		    (cfg.start_time.nsec > TGPIO_MAX_NS)) {
			return SEDI_DRIVER_ERROR_PARAMETER;
		}

		if (cfg.timer == SEDI_TGPIO_ART) {
			timespec_to_art_cycles(cfg.start_time, &art_cycles);
			pin->tgpio_compv_31_0 = art_cycles & ~0;
			pin->tgpio_compv_63_32 = art_cycles >> REG_WIDTH;

			timespec_to_art_cycles(cfg.repeat_interval,
					       &art_cycles);
			pin->tgpio_piv_31_0 = art_cycles & ~0;
			pin->tgpio_piv_63_32 = art_cycles >> REG_WIDTH;
		} else {

			pin->tgpio_compv_31_0 = cfg.start_time.nsec;
			pin->tgpio_compv_63_32 = cfg.start_time.sec;

			pin->tgpio_piv_31_0 = cfg.repeat_interval.nsec;
			pin->tgpio_piv_63_32 = cfg.repeat_interval.sec;
		}

		/* Mode: Scheduled, Repeated, Output */
		control_reg |= TGPIO_PIN_PERIODIC;

		/* Periodic sequence length bit - 17-24. */
		control_reg |= (REPEAT_COUNT_MAX < cfg.repeat_count)
				   ? (REPEAT_COUNT_MAX << TGPIO_N_REPEATS_BIT)
				   : (cfg.repeat_count << TGPIO_N_REPEATS_BIT);
	} else {
		/*
		 * Mode: Unscheduled, Output (simple output),
		 * Not supported in RTL, May be in future.
		 */
		control_reg |= TGPIO_PIN_OUTPUT_SKIP_MATCH;
	}
	/* Pulse width stretch bit - 5-8. range: 2-15, reg range: 0-15 */
	if ((cfg.pulse_width < PULSE_WIDTH_MIN) ||
	    (cfg.pulse_width > PULSE_WIDTH_MAX)) {
		return SEDI_DRIVER_ERROR_PARAMETER;
	}
	control_reg |= (cfg.pulse_width << TGPIO_PWS_BIT);

	if (cfg.timer == SEDI_TGPIO_ART) {
		pin->tgpio_mask_31_0 = DEFAULT_ART_MATCH_MASK;
	} else {
		pin->tgpio_mask_31_0 = DEFAULT_TMT_MATCH_MASK;
	}

	/* Event Counter bit - 16. */
	control_reg |= TGPIO_PIN_ENABLE_EC;

	*ctrl_reg = control_reg;

	return SEDI_DRIVER_OK;
}

int32_t sedi_tgpio_pin_config(IN sedi_tgpio_t instance,
			      IN sedi_gpio_pin_t gpio_pin,
			      IN sedi_tgpio_pin_config_t cfg)
{
	SEDI_ASSERT(instance < SEDI_TGPIO_NUM);
	SEDI_ASSERT(gpio_pin < SEDI_GPIO_PIN_NUM);

	uint32_t control_reg = 0, ret;
	int32_t tgpio_pin;
	tgpio_pin_regs_t *pin;
	tgpio_regs_t *base = tgpio_resource[instance].regs;
	tgpio_intr_regs_t *intr = &(base->tgpio_intr_regs);

	/* Check if the pin belongs to TGPIO based on pinmux */
	tgpio_pin = tgpio_find_pin(instance, gpio_pin);
	if (tgpio_pin < 0) {
		return SEDI_DRIVER_ERROR_PARAMETER;
	}

	pin = &(base->tgpio_pin[tgpio_pin]);

	/* Disable pin*/
	sedi_tgpio_pin_disable(instance, gpio_pin);

	if (cfg.direction == SEDI_INPUT) {
		/* Mode: Input */
		ret = tgpio_config_input(&control_reg, pin, cfg);
	} else {
		/* Mode: Output */
		ret = tgpio_config_output(&control_reg, pin, cfg);
	}
	if (ret != SEDI_DRIVER_OK) {
		return ret;
	}

	/* Event Polarity bit 3:2 */
	control_reg |= (cfg.ev_polarity << TGPIO_EVENT_POLARITY_BIT);

	/* Clock selection for each pin. 1=xtal clock, 0=ptp clock*/
	if (cfg.timer == SEDI_TGPIO_ART) {
		base->tgpio_clk_sel |= BIT(tgpio_pin);
	} else {
		base->tgpio_clk_sel &= ~BIT(tgpio_pin);
	}
	/* Timestamp counter select for each pin. 00,01 & 10 - TMT, 11 - ART */
	config_ts_sel(base, tgpio_pin, cfg.timer);

	/* Disable & clear off any pending interrupt for the pin */
	intr->tgpio_intr_imsc &= ~BIT(tgpio_pin);
	intr->tgpio_intr_icr |= BIT(tgpio_pin);

	/* Enable interrupt for this pin*/
	if (cfg.intr_enable) {
		intr->tgpio_intr_imsc |= BIT(tgpio_pin);
	}

	/* Write control register except enable bit*/
	base->tgpio_pin[tgpio_pin].tgpio_ctl = control_reg;
	base->tgpio_pin[tgpio_pin].tgpio_tcv_31_0;
	base->tgpio_pin[tgpio_pin].tgpio_tcv_63_32;

	return SEDI_DRIVER_OK;
}

int32_t sedi_tgpio_get_cross_timestamp(IN sedi_tgpio_t instance,
				       IN sedi_tgpio_timer_t local_clock,
				       IN sedi_tgpio_timer_t reference_clock,
				       OUT sedi_tgpio_time_t *local_time,
				       OUT sedi_tgpio_time_t *reference_time)
{
	SEDI_ASSERT(instance < SEDI_TGPIO_NUM);
	SEDI_ASSERT(local_clock < TGPIO_MAX_TIMERS);
	SEDI_ASSERT(local_time != NULL);
	SEDI_ASSERT(reference_time != NULL);

	uint32_t timer_mask, art_low;
	uint64_t art_cycles;
	tgpio_regs_t *base = tgpio_resource[instance].regs;
	tgpio_tmt_regs_t *tmt = &(base->tgpio_tmt[local_clock]);

	if (reference_clock != SEDI_TGPIO_ART) {
		/* Only ART is supported for now; TMT0/PTM in future may be. */
		return SEDI_DRIVER_ERROR_PARAMETER;
	}

	timer_mask = BIT(local_clock * BITS_PER_PIN);
	base->tgpio_cts_enable |= timer_mask;

	/* Wait till TS becomes Valid */
	while (!(base->tgpio_cts_valid & timer_mask))
		;

	local_time->nsec = tmt->tmt_local_low;
	local_time->sec = tmt->tmt_local_high;

	art_low = tmt->tmt_art_low;
	art_cycles = tmt->tmt_art_high;
	art_cycles = (art_cycles << REG_WIDTH) + art_low;

	art_cycles_to_timespec(art_cycles, reference_time);

	return SEDI_DRIVER_OK;
}

int32_t sedi_tgpio_set_time(IN sedi_tgpio_t instance,
			    IN sedi_tgpio_timer_t tgpio_timer, IN uint32_t sec,
			    IN uint32_t ns)
{
	SEDI_ASSERT(instance < SEDI_TGPIO_NUM);
	SEDI_ASSERT(tgpio_timer < TGPIO_MAX_TIMERS);
	SEDI_ASSERT(ns < TGPIO_MAX_NS);

	tgpio_regs_t *base = tgpio_resource[instance].regs;
	tgpio_tmt_regs_t *tmt = &(base->tgpio_tmt[tgpio_timer]);

	/* Disable timer */
	tmt->tmt_ctl &= ~TGPIO_TIMER_ENABLE;

	/* H/W Mandates to set L and then H. */
	tmt->tmt_l = ns;
	tmt->tmt_h = sec;
	tmt->tmt_ctl |= TGPIO_TIMER_ENABLE;

	return SEDI_DRIVER_OK;
}

uint32_t sedi_tgpio_get_time(IN sedi_tgpio_t instance,
			     IN sedi_tgpio_timer_t tgpio_timer,
			     OUT uint32_t *sec, OUT uint32_t *ns)
{
	SEDI_ASSERT(instance < SEDI_TGPIO_NUM);
	SEDI_ASSERT(tgpio_timer < TGPIO_MAX_TIMERS);

	tgpio_regs_t *base = tgpio_resource[instance].regs;
	tgpio_tmt_regs_t *tmt = &(base->tgpio_tmt[tgpio_timer]);

	/* H/W Mandates to read R, L and then H. */
	tmt->tmt_r;
	*ns = tmt->tmt_l;
	*sec = tmt->tmt_h;

	return SEDI_DRIVER_OK;
}

int32_t sedi_tgpio_adjust_time(IN sedi_tgpio_t instance,
			       IN sedi_tgpio_timer_t tgpio_timer,
			       IN int32_t time_ns)
{
	SEDI_ASSERT(instance < SEDI_TGPIO_NUM);
	SEDI_ASSERT(tgpio_timer < TGPIO_MAX_TIMERS);
	SEDI_ASSERT(time_ns <= TGPIO_MAX_ADJUST);

	tgpio_regs_t *base = tgpio_resource[instance].regs;

	/* Check if timer enabled or not */
	if (!(base->tgpio_tmt[tgpio_timer].tmt_ctl & TGPIO_TIMER_ENABLE)) {
		return SEDI_DRIVER_ERROR;
	}

	/* set time adjust completion interrupt */
	base->tgpio_tmt[tgpio_timer].tmt_timadj =
	    (time_ns < 0) ? ((-time_ns) | TGPIO_TMT_TIME_ADJUST_SIGN) : time_ns;

	/* wait for time adjust completion interrupt */
	while (!(base->tgpio_intr_regs.tgpio_intr_ris &
		 BIT(TGPIO_TADJ_TMT_BIT + tgpio_timer)))
		;

	/* Done adjusting, Clear interrupt */
	base->tgpio_intr_regs.tgpio_intr_icr |=
	    BIT(TGPIO_TADJ_TMT_BIT + tgpio_timer);

	return SEDI_DRIVER_OK;
}

int32_t sedi_tgpio_adjust_frequency(IN sedi_tgpio_t instance,
				    IN sedi_tgpio_timer_t tgpio_timer,
				    IN int32_t ppb)
{
	SEDI_ASSERT(instance < SEDI_TGPIO_NUM);
	SEDI_ASSERT(tgpio_timer < TGPIO_MAX_TIMERS);

	/* Check if timer enabled or not */
	if (!(tgpio_resource[instance].regs->tgpio_tmt[tgpio_timer].tmt_ctl &
	      TGPIO_TIMER_ENABLE)) {
		return SEDI_DRIVER_ERROR;
	}

	if (ppb < MIN_PPB || ppb > MAX_PPB) {
		return SEDI_DRIVER_ERROR_PARAMETER;
	}

	uint32_t timinca;
	float total_timinca;

	/* Adjustments are in terms of 2^-32 ns.
	 * Adjustments are increments of 2^32/1e9 = 4.29.
	 * Making the increment register = ~(4.3) would result in an approximate
	 * 1 PPB change - sign bit determines faster or slower. The maximum
	 * adjustment is 31 bits wide or ~500K ppb.
	 */

	/* Prepare new value*/
	if (ppb < 0) {
		total_timinca = -ppb * PPB_TO_TIMINCA;
		timinca = (uint32_t)total_timinca;
		timinca = (timinca | TGPIO_TMT_TIME_ADJUST_SIGN);
	} else {
		total_timinca = ppb * PPB_TO_TIMINCA;
		timinca = (uint32_t)total_timinca;
	}

	/* Configure new rate */
	tgpio_resource[instance].regs->tgpio_tmt[tgpio_timer].tmt_timinca =
	    timinca;

	return SEDI_DRIVER_OK;
}

int32_t sedi_tgpio_init(IN sedi_tgpio_t instance, IN sedi_tgpio_cb_t cb,
			OUT void *cb_data)
{
	SEDI_ASSERT(instance < SEDI_TGPIO_NUM);

	tgpio_regs_t *base = tgpio_resource[instance].regs;

	if (DEV_PSE_OWNED != sedi_get_dev_ownership(PSE_DEV_GPIO0 + instance)) {
		return SEDI_DRIVER_ERROR_NO_DEV;
	}

	/* SFT_RST for the instance */
	write32(SEDI_TGPIO_SFT_RST_REG,
		read32(SEDI_TGPIO_SFT_RST_REG) | (1 << instance));
	write32(SEDI_TGPIO_SFT_RST_REG,
		read32(SEDI_TGPIO_SFT_RST_REG) & ~(1 << instance));

	/* Disable nano second counter wrap. */
	base->tgpio_intr_regs.tgpio_intr_ctl &=
	    ~(NANO_SEC_WRAP_BITS_0_2 << TGPIO_NANO_SEC_WRAP_BIT);

	/*EOI - clear interrupts*/
	base->tgpio_intr_regs.tgpio_intr_icr = ~0;
	base->tgpio_intr_regs.tgpio_intr_imsc = 0;

	tgpio_runtime[instance].cb = cb;
	tgpio_runtime[instance].cb_data = cb_data;

	return SEDI_DRIVER_OK;
}

int32_t sedi_tgpio_get_pindata(IN sedi_tgpio_t instance,
			       IN sedi_gpio_pin_t gpio_pin,
			       OUT sedi_tgpio_time_t *timestamp,
			       OUT uint64_t *event_counter)
{
	SEDI_ASSERT(instance < SEDI_TGPIO_NUM);
	SEDI_ASSERT(gpio_pin < SEDI_GPIO_PIN_NUM);

	uint64_t art_cycles;
	tgpio_regs_t *base = tgpio_resource[instance].regs;
	/* Check if the pin belongs to TGPIO */
	int32_t tgpio_pin = tgpio_find_pin(instance, gpio_pin);
	if (tgpio_pin < 0) {
		return SEDI_DRIVER_ERROR_PARAMETER;
	}
	/* If the pin is driven by ART, convert art cycles to timespec */
	if (base->tgpio_clk_sel & BIT(tgpio_pin)) {
		art_cycles = tgpio_resource[instance]
				 .regs->tgpio_pin[tgpio_pin]
				 .tgpio_tcv_31_0;
		art_cycles += (uint64_t)(tgpio_resource[instance]
					     .regs->tgpio_pin[tgpio_pin]
					     .tgpio_tcv_63_32)
			      << REG_WIDTH;
		art_cycles_to_timespec(art_cycles, timestamp);
	} else {
		timestamp->nsec = tgpio_resource[instance]
				      .regs->tgpio_pin[tgpio_pin]
				      .tgpio_tcv_31_0;
		timestamp->sec = tgpio_resource[instance]
				     .regs->tgpio_pin[tgpio_pin]
				     .tgpio_tcv_63_32;
	}

	*event_counter = (uint64_t)(tgpio_resource[instance]
					.regs->tgpio_pin[tgpio_pin]
					.tgpio_eccv_31_0);

	*event_counter += (uint64_t)(tgpio_resource[instance]
					 .regs->tgpio_pin[tgpio_pin]
					 .tgpio_eccv_63_32)
			  << REG_WIDTH;

	return SEDI_DRIVER_OK;
}

int32_t sedi_tgpio_uninit(IN sedi_tgpio_t instance)
{
	SEDI_ASSERT(instance < SEDI_TGPIO_NUM);

	tgpio_runtime[instance].cb = NULL;
	tgpio_runtime[instance].cb_data = NULL;

	return SEDI_DRIVER_OK;
}

sedi_driver_version_t sedi_tgpio_get_version(void)
{
	return tgpio_driver_version;
}
