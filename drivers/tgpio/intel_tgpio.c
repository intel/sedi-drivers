/*
 * Copyright (c) 2021 Intel Corporation
 *
 * SPDX-License-Identifier:  BSD-3-Clause
 */

#include <intel/hal_tgpio.h>
#include "intel_tgpio_regs.h"

static inline void art_cycles_to_timespec(IN uint64_t art_cycles,
					  INOUT intel_tgpio_time_t *tgpio_time)
{
	uint64_t art_cycles_khz = art_cycles / KHz;
	uint32_t art_cycles_khz_rem = art_cycles % KHz;
	uint64_t total_nsec = art_cycles_khz * ART_PERIOD;

	total_nsec = total_nsec + (art_cycles_khz_rem * ART_PERIOD) / KHz;

	tgpio_time->sec = total_nsec / NSEC_PER_SECOND;
	tgpio_time->nsec = total_nsec % NSEC_PER_SECOND;
}

static inline void timespec_to_art_cycles(IN intel_tgpio_time_t tgpio_time,
					  INOUT uint64_t *art_cycles)
{
	uint64_t total_nsec =
		((uint64_t)NSEC_PER_SECOND * tgpio_time.sec) + tgpio_time.nsec;

	*art_cycles = (total_nsec / ART_PERIOD) * KHz;
}

static inline uint32_t gpio_status(IN uint32_t status_reg, IN uint32_t pinmux)
{
	uint32_t status = status_reg;

	/*
	 * Map TGPIO status bits to GPIO status bits
	 * Use will see only GPIO pins
	 */
	switch (pinmux) {
	case INTEL_TGPIO_PINMUX_1:
		status = (status << MUX_SHIFT_SIZE);
		break;
	case INTEL_TGPIO_PINMUX_2:
		status = (status & TGPIO_PINMUX_2_MASK);
		break;
	case INTEL_TGPIO_PINMUX_3:
		status = (status & LOWER_10_BITS_MASK) |
			 ((status << MUX_SHIFT_SIZE) & UPPER_10_BITS_MASK);
		break;
	default:
		status = FALSE;
	}
	return status;
}

static inline int32_t tgpio_find_pin(IN intel_instance_t *inst,
				     IN intel_gpio_pin_t gpio_pin)
{
	INTEL_ASSERT(gpio_pin < INTEL_GPIO_PIN_NUM);

	int32_t tgpio_pin;
	int pmx;

	struct intel_tgpio_context *tgpio =
		CONTAINER_OF(inst, struct intel_tgpio_context, instance);
	pmx = tgpio->pmx;

	/* Check if the pin is a valid tgpio pin */
	switch (pmx) {
	case INTEL_TGPIO_PINMUX_1:
		tgpio_pin = (BIT(gpio_pin) & ~TGPIO_PINMUX_1_MASK)
			    ? INTEL_DRIVER_ERROR_PARAMETER
			    : (gpio_pin - MUX_SHIFT_SIZE);
		break;
	case INTEL_TGPIO_PINMUX_2:
		tgpio_pin = (BIT(gpio_pin) & ~TGPIO_PINMUX_2_MASK)
			    ? INTEL_DRIVER_ERROR_PARAMETER
			    : gpio_pin;
		break;
	case INTEL_TGPIO_PINMUX_3:
		tgpio_pin = (BIT(gpio_pin) & ~TGPIO_PINMUX_3_MASK)
			    ? INTEL_DRIVER_ERROR_PARAMETER
			    : (gpio_pin < MUX_SHIFT_SIZE)
			    ? gpio_pin
			    : (gpio_pin - MUX_SHIFT_SIZE);
		break;
	default:
		tgpio_pin = INTEL_DRIVER_ERROR_PARAMETER;
	}

	if (tgpio_pin >= TGPIO_MAX_PIN_NUM) {
		tgpio_pin = INTEL_DRIVER_ERROR_PARAMETER;
	}

	return tgpio_pin;
}

static inline int32_t config_ts_sel(OUT tgpio_regs_t *base, IN uint32_t pin,
				    IN intel_tgpio_timer_t timer)
{
	INTEL_ASSERT(base != NULL);

	uint32_t tgpio_pin = (pin % TS_SEL_REG_LIMIT) * BITS_PER_PIN;
	uint32_t index = pin / TS_SEL_REG_LIMIT;

	switch (timer) {
	case INTEL_TGPIO_TMT_0:
		base->tgpio_ts_sel[index] &=
			~(BIT(tgpio_pin) | BIT(tgpio_pin + 1));
		break;
	case INTEL_TGPIO_TMT_1:
		base->tgpio_ts_sel[index] |= BIT(tgpio_pin);
		base->tgpio_ts_sel[index] &= ~BIT(tgpio_pin + 1);
		break;
	case INTEL_TGPIO_TMT_2:
		base->tgpio_ts_sel[index] &= ~BIT(tgpio_pin);
		base->tgpio_ts_sel[index] |= BIT(tgpio_pin + 1);
		break;
	case INTEL_TGPIO_ART:
		base->tgpio_ts_sel[index] |=
			(BIT(tgpio_pin) | BIT(tgpio_pin + 1));
		break;
	default:
		return INTEL_DRIVER_ERROR_PARAMETER;
	}
	return INTEL_DRIVER_OK;
}

void intel_tgpio_isr_handler(IN intel_instance_t *inst)
{
	uint32_t imsc, status;
	int pmx;
	struct intel_tgpio_context *tgpio =
		CONTAINER_OF(inst, struct intel_tgpio_context, instance);
	tgpio_regs_t *base = tgpio->regs;

	pmx = tgpio->pmx;
	/* Save and mask all interrupts */
	imsc = base->tgpio_intr_regs.tgpio_intr_imsc;
	base->tgpio_intr_regs.tgpio_intr_imsc = 0;

	status = base->tgpio_intr_regs.tgpio_intr_mis;

	if (tgpio->cb) {
		tgpio->cb(tgpio->cb_data, gpio_status(status, pmx));
	}

	/* Clear and unmask interrupts */
	base->tgpio_intr_regs.tgpio_intr_icr = status;
	base->tgpio_intr_regs.tgpio_intr_imsc = imsc;
}

int32_t intel_tgpio_pin_enable(IN intel_instance_t *inst,
			       IN intel_gpio_pin_t gpio_pin)
{
	INTEL_ASSERT(gpio_pin < INTEL_GPIO_PIN_NUM);

	struct intel_tgpio_context *tgpio =
		CONTAINER_OF(inst, struct intel_tgpio_context, instance);
	/* Check if the pin belongs to TGPIO */
	int32_t tgpio_pin = tgpio_find_pin(inst, gpio_pin);
	tgpio_regs_t *base = tgpio->regs;

	if (tgpio_pin < 0) {
		return INTEL_DRIVER_ERROR_PARAMETER;
	}

	base->tgpio_pin[tgpio_pin].tgpio_ctl |= (TGPIO_PIN_ENABLE);

	return INTEL_DRIVER_OK;
}

int32_t intel_tgpio_pin_disable(IN intel_instance_t *inst,
				IN intel_gpio_pin_t gpio_pin)
{
	INTEL_ASSERT(gpio_pin < INTEL_GPIO_PIN_NUM);

	struct intel_tgpio_context *tgpio =
		CONTAINER_OF(inst, struct intel_tgpio_context, instance);
	/* Check if the pin belongs to TGPIO */
	int32_t tgpio_pin = tgpio_find_pin(inst, gpio_pin);
	tgpio_regs_t *base = tgpio->regs;

	if (tgpio_pin < 0) {
		return INTEL_DRIVER_ERROR_PARAMETER;
	}

	base->tgpio_pin[tgpio_pin].tgpio_ctl &= ~(TGPIO_PIN_ENABLE);

	return INTEL_DRIVER_OK;
}

static inline int32_t tgpio_config_input(INOUT uint32_t *ctrl_reg,
					 INOUT tgpio_pin_regs_t *pin,
					 IN intel_tgpio_pin_config_t cfg)
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
			return INTEL_DRIVER_ERROR_PARAMETER;
		}

		/* Enable event counting */
		control_reg |= TGPIO_PIN_INPUT_REPEAT;
		/* Enable reapeated interrupt generation */
		control_reg |= TGPIO_PIN_PERIODIC;

		if (cfg.timer == INTEL_TGPIO_ART) {
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

		return INTEL_DRIVER_ERROR_PARAMETER;
	}

	/* Periodic sequence length bit - 17-24. */
	control_reg |= (cfg.repeat_count > REPEAT_COUNT_MAX)
		       ? (REPEAT_COUNT_MAX << TGPIO_N_REPEATS_BIT)
		       : (cfg.repeat_count << TGPIO_N_REPEATS_BIT);

	pin->tgpio_mask_31_0 = 0;
	control_reg |= TGPIO_PIN_DIRECTION;

	*ctrl_reg = control_reg;

	return INTEL_DRIVER_OK;
}

static inline int32_t tgpio_config_output(INOUT uint32_t *ctrl_reg,
					  INOUT tgpio_pin_regs_t *pin,
					  IN intel_tgpio_pin_config_t cfg)
{
	uint32_t control_reg = 0;
	uint64_t art_cycles;

	if (cfg.start_time.nsec || cfg.start_time.sec) {
		if ((cfg.timer > TGPIO_MAX_TIMERS) ||
		    (cfg.repeat_interval.nsec > TGPIO_MAX_NS) ||
		    (cfg.start_time.nsec > TGPIO_MAX_NS)) {
			return INTEL_DRIVER_ERROR_PARAMETER;
		}

		if (cfg.timer == INTEL_TGPIO_ART) {
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
		control_reg |= (cfg.repeat_count > REPEAT_COUNT_MAX)
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
		return INTEL_DRIVER_ERROR_PARAMETER;
	}
	control_reg |= (cfg.pulse_width << TGPIO_PWS_BIT);

	if (cfg.timer == INTEL_TGPIO_ART) {
		pin->tgpio_mask_31_0 = DEFAULT_ART_MATCH_MASK;
	} else {
		pin->tgpio_mask_31_0 = DEFAULT_TMT_MATCH_MASK;
	}

	/* Event Counter bit - 16. */
	control_reg |= TGPIO_PIN_ENABLE_EC;

	*ctrl_reg = control_reg;

	return INTEL_DRIVER_OK;
}

int32_t intel_tgpio_pin_config(IN intel_instance_t *inst,
			       IN intel_gpio_pin_t gpio_pin,
			       IN intel_tgpio_pin_config_t cfg)
{
	INTEL_ASSERT(gpio_pin < INTEL_GPIO_PIN_NUM);

	uint32_t control_reg = 0, ret;
	int32_t tgpio_pin;
	tgpio_pin_regs_t *pin;
	struct intel_tgpio_context *tgpio =
		CONTAINER_OF(inst, struct intel_tgpio_context, instance);
	tgpio_regs_t *base = tgpio->regs;
	tgpio_intr_regs_t *intr = &(base->tgpio_intr_regs);

	/* Check if the pin belongs to TGPIO based on pinmux */
	tgpio_pin = tgpio_find_pin(inst, gpio_pin);
	if (tgpio_pin < 0) {
		return INTEL_DRIVER_ERROR_PARAMETER;
	}

	pin = &(base->tgpio_pin[tgpio_pin]);

	/* Disable pin*/
	intel_tgpio_pin_disable(inst, gpio_pin);

	if (cfg.direction == INTEL_INPUT) {
		/* Mode: Input */
		ret = tgpio_config_input(&control_reg, pin, cfg);
	} else {
		/* Mode: Output */
		ret = tgpio_config_output(&control_reg, pin, cfg);
	}
	if (ret != INTEL_DRIVER_OK) {
		return ret;
	}

	/* Event Polarity bit 3:2 */
	control_reg |= (cfg.ev_polarity << TGPIO_EVENT_POLARITY_BIT);

	/* Clock selection for each pin. 1=xtal clock, 0=ptp clock*/
	if (cfg.timer == INTEL_TGPIO_ART) {
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

	return INTEL_DRIVER_OK;
}

int32_t intel_tgpio_get_cross_timestamp(IN intel_instance_t *inst,
					IN intel_tgpio_timer_t local_clock,
					IN intel_tgpio_timer_t reference_clock,
					OUT intel_tgpio_time_t *local_time,
					OUT intel_tgpio_time_t *reference_time)
{
	INTEL_ASSERT(local_clock < TGPIO_MAX_TIMERS);
	INTEL_ASSERT(local_time != NULL);
	INTEL_ASSERT(reference_time != NULL);

	uint32_t timer_mask, art_low;
	uint64_t art_cycles;
	struct intel_tgpio_context *tgpio =
		CONTAINER_OF(inst, struct intel_tgpio_context, instance);
	tgpio_regs_t *base = tgpio->regs;
	tgpio_tmt_regs_t *tmt = &(base->tgpio_tmt[local_clock]);

	if (reference_clock != INTEL_TGPIO_ART) {
		/* Only ART is supported for now; TMT0/PTM in future may be. */
		return INTEL_DRIVER_ERROR_PARAMETER;
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

	return INTEL_DRIVER_OK;
}

int32_t intel_tgpio_set_time(IN intel_instance_t *inst,
			     IN intel_tgpio_timer_t tgpio_timer, IN uint32_t sec,
			     IN uint32_t ns)
{
	INTEL_ASSERT(tgpio_timer < TGPIO_MAX_TIMERS);
	INTEL_ASSERT(ns < TGPIO_MAX_NS);

	struct intel_tgpio_context *tgpio =
		CONTAINER_OF(inst, struct intel_tgpio_context, instance);
	tgpio_regs_t *base = tgpio->regs;
	tgpio_tmt_regs_t *tmt = &(base->tgpio_tmt[tgpio_timer]);

	/* Disable timer */
	tmt->tmt_ctl &= ~TGPIO_TIMER_ENABLE;

	/* H/W Mandates to set L and then H. */
	tmt->tmt_l = ns;
	tmt->tmt_h = sec;
	tmt->tmt_ctl |= TGPIO_TIMER_ENABLE;

	return INTEL_DRIVER_OK;
}

uint32_t intel_tgpio_get_time(IN intel_instance_t *inst,
			      IN intel_tgpio_timer_t tgpio_timer,
			      OUT uint32_t *sec, OUT uint32_t *ns)
{
	INTEL_ASSERT(tgpio_timer < TGPIO_MAX_TIMERS);

	struct intel_tgpio_context *tgpio =
		CONTAINER_OF(inst, struct intel_tgpio_context, instance);
	tgpio_regs_t *base = tgpio->regs;
	tgpio_tmt_regs_t *tmt = &(base->tgpio_tmt[tgpio_timer]);

	/* H/W Mandates to read R, L and then H. */
	tmt->tmt_r;
	*ns = tmt->tmt_l;
	*sec = tmt->tmt_h;

	return INTEL_DRIVER_OK;
}

int32_t intel_tgpio_adjust_time(IN intel_instance_t *inst,
				IN intel_tgpio_timer_t tgpio_timer,
				IN int32_t time_ns)
{
	INTEL_ASSERT(tgpio_timer < TGPIO_MAX_TIMERS);
	INTEL_ASSERT(time_ns <= TGPIO_MAX_ADJUST);

	struct intel_tgpio_context *tgpio =
		CONTAINER_OF(inst, struct intel_tgpio_context, instance);
	tgpio_regs_t *base = tgpio->regs;

	/* Check if timer enabled or not */
	if (!(base->tgpio_tmt[tgpio_timer].tmt_ctl & TGPIO_TIMER_ENABLE)) {
		return INTEL_DRIVER_ERROR;
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

	return INTEL_DRIVER_OK;
}

int32_t intel_tgpio_adjust_frequency(IN intel_instance_t *inst,
				     IN intel_tgpio_timer_t tgpio_timer,
				     IN int32_t ppb)
{
	INTEL_ASSERT(tgpio_timer < TGPIO_MAX_TIMERS);

	struct intel_tgpio_context *tgpio =
		CONTAINER_OF(inst, struct intel_tgpio_context, instance);
	tgpio_regs_t *base = tgpio->regs;

	/* Check if timer enabled or not */
	if (!(base->tgpio_tmt[tgpio_timer].tmt_ctl & TGPIO_TIMER_ENABLE)) {
		return INTEL_DRIVER_ERROR;
	}

	if (ppb < MIN_PPB || ppb > MAX_PPB) {
		return INTEL_DRIVER_ERROR_PARAMETER;
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
	base->tgpio_tmt[tgpio_timer].tmt_timinca = timinca;

	return INTEL_DRIVER_OK;
}

void intel_set_pin_mux(IN intel_instance_t *inst, IN int pin_mux)
{
	struct intel_tgpio_context *tgpio =
		CONTAINER_OF(inst, struct intel_tgpio_context, instance);

	tgpio->pmx = pin_mux;
}

int32_t intel_tgpio_init(IN intel_instance_t *inst, IN intel_tgpio_cb_t cb,
			 OUT void *cb_data)
{
	struct intel_tgpio_context *tgpio =
		CONTAINER_OF(inst, struct intel_tgpio_context, instance);
	tgpio_regs_t *base = (tgpio_regs_t *)(inst->base_addr + BASE_OFFSET);

	tgpio->regs = base;

	/* Disable nano second counter wrap. */
	base->tgpio_intr_regs.tgpio_intr_ctl &=
		~(NANO_SEC_WRAP_BITS_0_2 << TGPIO_NANO_SEC_WRAP_BIT);

	/*EOI - clear interrupts*/
	base->tgpio_intr_regs.tgpio_intr_icr = ~0;
	base->tgpio_intr_regs.tgpio_intr_imsc = 0;

	tgpio->cb = cb;
	tgpio->cb_data = cb_data;

	return INTEL_DRIVER_OK;
}

int32_t intel_tgpio_get_pindata(IN intel_instance_t *inst,
				IN intel_gpio_pin_t gpio_pin,
				OUT intel_tgpio_time_t *timestamp,
				OUT uint64_t *event_counter)
{
	INTEL_ASSERT(gpio_pin < INTEL_GPIO_PIN_NUM);

	uint64_t art_cycles;
	struct intel_tgpio_context *tgpio =
		CONTAINER_OF(inst, struct intel_tgpio_context, instance);
	tgpio_regs_t *base = tgpio->regs;

	/* Check if the pin belongs to TGPIO */
	int32_t tgpio_pin = tgpio_find_pin(inst, gpio_pin);

	if (tgpio_pin < 0) {
		return INTEL_DRIVER_ERROR_PARAMETER;
	}
	/* If the pin is driven by ART, convert art cycles to timespec */
	if (base->tgpio_clk_sel & BIT(tgpio_pin)) {
		art_cycles = base->tgpio_pin[tgpio_pin].tgpio_tcv_31_0;
		art_cycles += (uint64_t)(base->tgpio_pin[tgpio_pin].tgpio_tcv_63_32) << REG_WIDTH;
		art_cycles_to_timespec(art_cycles, timestamp);
	} else {
		timestamp->nsec = base->tgpio_pin[tgpio_pin].tgpio_tcv_31_0;
		timestamp->sec = base->tgpio_pin[tgpio_pin].tgpio_tcv_63_32;
	}

	*event_counter = (uint64_t)(base->tgpio_pin[tgpio_pin].tgpio_eccv_31_0);
	*event_counter += (uint64_t)(base->tgpio_pin[tgpio_pin].tgpio_eccv_63_32) << REG_WIDTH;

	return INTEL_DRIVER_OK;
}

int32_t intel_tgpio_uninit(IN intel_instance_t *inst)
{
	struct intel_tgpio_context *tgpio =
		CONTAINER_OF(inst, struct intel_tgpio_context, instance);

	tgpio->cb = NULL;
	tgpio->cb_data = NULL;

	return INTEL_DRIVER_OK;
}

intel_driver_version_t intel_tgpio_get_version(void)
{
	return tgpio_driver_version;
}
