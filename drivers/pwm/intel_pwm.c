/*
 * Copyright (c) 2021 Intel Corporation
 *
 * SPDX-License-Identifier:  BSD-3-Clause
 */

#include <intel/hal_pwm.h>
#include "intel_pwm.h"

#define NSECS_PER_SEC (1000000000)
#define NSECS_PER_CYCLE (NSECS_PER_SEC / (100 * 1000000UL))
#define NSECS_TO_CYCLES(nsec) (nsec / NSECS_PER_CYCLE)
#define CYCLES_MAX ((uint32_t)(~0))

/* driver version */
#define INTEL_PWM_DRIVER_VERSION INTEL_DRIVER_VERSION_MAJOR_MINOR(0, 1)

static const intel_driver_version_t pwm_driver_version = { INTEL_PWM_API_VERSION,
						INTEL_PWM_DRIVER_VERSION};

void intel_pwm_isr_handler(IN intel_instance_t *inst)
{
	intel_pwm_id_t pwm = 0;
	struct intel_pwm_context *pwm_inst =
	    CONTAINER_OF(inst, struct intel_pwm_context, instance);
	volatile intel_pwm_reg_t *base = pwm_inst->regs;

	uint32_t status = base->timersintstatus;

	while (pwm < INTEL_PWM_ID_NUM) {
		if (status & BIT(pwm)) {
			base->timer[pwm].controlreg |= PWM_INTR_DISABLE;
			if (pwm_inst->cb) {
				pwm_inst->cb(pwm_inst->cb_data, status);
			}
			base->timer[pwm].controlreg &= ~PWM_INTR_DISABLE;
			base->timer[pwm].eoi;

			break;
		}
		pwm++;
	}
}

intel_driver_version_t intel_pwm_get_version(void)
{
	return pwm_driver_version;
}

int32_t intel_pwm_start(IN intel_instance_t *inst, IN intel_pwm_id_t pwm)
{
	struct intel_pwm_context *pwm_inst =
	    CONTAINER_OF(inst, struct intel_pwm_context, instance);
	volatile intel_pwm_reg_t *base = pwm_inst->regs;

	base->timer[pwm].controlreg |= PWM_START;

	return INTEL_DRIVER_OK;
}

int32_t intel_pwm_stop(IN intel_instance_t *inst, IN intel_pwm_id_t pwm)
{
	struct intel_pwm_context *pwm_inst =
	    CONTAINER_OF(inst, struct intel_pwm_context, instance);
	volatile intel_pwm_reg_t *base = pwm_inst->regs;

	base->timer[pwm].controlreg &= ~PWM_START;

	return INTEL_DRIVER_OK;
}

int32_t intel_pwm_set_config(IN intel_instance_t *inst, IN intel_pwm_id_t pwm,
			     IN intel_pwm_config_t cfg)
{
	struct intel_pwm_context *pwm_inst =
	    CONTAINER_OF(inst, struct intel_pwm_context, instance);
	volatile intel_pwm_reg_t *base = pwm_inst->regs;

	switch (cfg.mode) {
	case INTEL_PWM_MODE_TIMER_FREERUNNING:
		base->timer[pwm].controlreg = PWM_TIMER_FREERUNNING;
		break;
	case INTEL_PWM_MODE_TIMER_COUNT:
		base->timer[pwm].loadcount = cfg.lo_count - 1;
		base->timer[pwm].controlreg = PWM_TIMER_MODE;
		break;
	case INTEL_PWM_MODE_PWM:
		/**
		 * HIGH period =
		 *	(TimerNLoadCount2 + 1) * clock period
		 *
		 * LOW period =
		 *	(TimerNLoadCount + 1) * clock period
		 */
		base->timer[pwm].loadcount = cfg.lo_count - 1;
		base->timer_loadcount2[pwm] = cfg.hi_count - 1;
		base->timer[pwm].controlreg =
		    PWM_MODE | PWM_TIMER_MODE | PWM_INTR_DISABLE;
		break;
	default:
		return INTEL_DRIVER_ERROR_PARAMETER;
	}

	if (!cfg.intr_enable) {
		base->timer[pwm].controlreg |= PWM_INTR_DISABLE;
	}

	return INTEL_DRIVER_OK;
}

int32_t intel_pwm_get_count(IN intel_instance_t *inst, IN intel_pwm_id_t pwm,
			    OUT uint32_t *current_count)
{
	struct intel_pwm_context *pwm_inst =
	    CONTAINER_OF(inst, struct intel_pwm_context, instance);
	volatile intel_pwm_reg_t *base = pwm_inst->regs;

	*current_count = base->timer[pwm].currentvalue;

	return INTEL_DRIVER_OK;
}

int32_t intel_pwm_pin_set_nsec(IN intel_instance_t *inst, IN intel_pwm_id_t pwm,
			       IN uint32_t ns_period, IN uint32_t ns_pulse)
{
	uint32_t period_cycles, pulse_cycles, high, low;
	intel_pwm_config_t cfg;

	period_cycles = NSECS_TO_CYCLES(ns_period);
	pulse_cycles = NSECS_TO_CYCLES(ns_pulse);

	if ((period_cycles == 0) || (pulse_cycles == 0) ||
	    (pulse_cycles > period_cycles) || (period_cycles > CYCLES_MAX) ||
	    (pulse_cycles > CYCLES_MAX)) {
		return INTEL_DRIVER_ERROR;
	}

	high = pulse_cycles;
	low = period_cycles - pulse_cycles;

	/*
	 * low must be more than zero. Otherwise, the PWM pin will be
	 * turned off. Let's make sure low is always more than zero.
	 */
	if (low == 0) {
		high--;
		low = 1;
	}

	intel_pwm_stop(inst, pwm);

	cfg.mode = INTEL_PWM_MODE_PWM;
	cfg.intr_enable = 0;
	cfg.hi_count = high;
	cfg.lo_count = low;

	intel_pwm_set_config(inst, pwm, cfg);

	intel_pwm_start(inst, pwm);

	return INTEL_DRIVER_OK;
}

int32_t intel_pwm_init(IN intel_instance_t *inst, IN intel_pwm_cb_t cb,
		       INOUT void *cb_data)
{
	struct intel_pwm_context *pwm =
	    CONTAINER_OF(inst, struct intel_pwm_context, instance);
	volatile intel_pwm_reg_t *base = (intel_pwm_reg_t *)inst->base_addr;

	pwm->regs = base;
	pwm->cb = cb;
	pwm->cb_data = cb_data;

	return INTEL_DRIVER_OK;
}
