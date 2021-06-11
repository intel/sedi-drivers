/*
 * Copyright (c) 2021 Intel Corporation
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <sedi_driver_pwm.h>
#include "sedi_pwm.h"
#include "pm/pm_internal_if.h"
#include "sedi_driver_ipc.h"
#include "sedi_driver_pm.h"

#define NSECS_PER_SEC (1000000000)
#define NSECS_PER_CYCLE (NSECS_PER_SEC / sedi_pm_get_lbw_clock())
#define NSECS_TO_CYCLES(nsec) (nsec / NSECS_PER_CYCLE)
#define CYCLES_MAX ((uint32_t)(~0))

/* driver version */
#define SEDI_PWM_DRIVER_VERSION SEDI_DRIVER_VERSION_MAJOR_MINOR(0, 1)

static const sedi_driver_version_t pwm_driver_version = {
	SEDI_PWM_API_VERSION, SEDI_PWM_DRIVER_VERSION
};

/* Runtime structure */
typedef struct runtime {
	sedi_pwm_cb_t cb;
	void *cb_data;
	sedi_pwm_reg_t saved_regs;
	bool regs_valid;
} pwm_runtime_t;

/* PWM register block. */
static sedi_pwm_reg_t *sedi_pwm[PWM_TOTAL_NUM] = {
	(sedi_pwm_reg_t *)SEDI_PWM_0_BASE, (sedi_pwm_reg_t *)SEDI_PWM_1_BASE
};

/* Runtime driver data */
static pwm_runtime_t pwm_runtime[SEDI_PWM_NUM];

static void sedi_pwm_save_regs(sedi_pwm_t instance)
{
	int i;

	sedi_pwm_reg_t *src = sedi_pwm[instance];
	sedi_pwm_reg_t *dst = &pwm_runtime[instance].saved_regs;

	/* Save timer registers */
	for (i = 0; i < SEDI_PWM_ID_NUM; i++) {
		dst->timer[i].loadcount = src->timer[i].loadcount;
		dst->timer[i].currentvalue = src->timer[i].loadcount;
		dst->timer[i].controlreg = src->timer[i].controlreg;
	}

	/* Save load count registers */
	for (i = 0; i < SEDI_PWM_ID_NUM; i++) {
		dst->timer_loadcount2[i] = src->timer_loadcount2[i];
	}

	pwm_runtime[instance].regs_valid = true;
}

static void sedi_pwm_restore_regs(sedi_pwm_t instance)
{
	int i;

	sedi_pwm_reg_t *src = &pwm_runtime[instance].saved_regs;
	sedi_pwm_reg_t *dst = sedi_pwm[instance];

	if (pwm_runtime[instance].regs_valid == false) {
		return;
	}

	/* Save timer registers */
	for (i = 0; i < SEDI_PWM_ID_NUM; i++) {
		dst->timer[i].loadcount = src->timer[i].loadcount;
		dst->timer[i].currentvalue = src->timer[i].loadcount;
		dst->timer[i].controlreg = src->timer[i].controlreg;
	}

	/* Save load count registers */
	for (i = 0; i < SEDI_PWM_ID_NUM; i++) {
		dst->timer_loadcount2[i] = src->timer_loadcount2[i];
	}

	pwm_runtime[instance].regs_valid = false;
}

int32_t sedi_pwm_set_power(IN sedi_pwm_t instance, IN sedi_power_state_t state)
{
	SEDI_ASSERT(instance < PWM_TOTAL_NUM);

	int32_t ret = SEDI_DRIVER_OK;
	driver_id_t id = DRIVER_ID_PWM0 + instance;

	switch (state) {
	case SEDI_POWER_FULL:
		/* Enable clock gates */
		pm_driver_start_trans(id);
		/* Restore registers if there is a saved context */
		sedi_pwm_restore_regs(instance);
		break;
	case SEDI_POWER_FORCE_SUSPEND:
	case SEDI_POWER_SUSPEND:
		/* Save registers */
		sedi_pwm_save_regs(instance);
		pm_driver_end_trans(instance);
		break;
	case SEDI_POWER_LOW:
		pm_driver_end_trans(instance);
		break;
	case SEDI_POWER_OFF:
		ret = SEDI_DRIVER_ERROR_UNSUPPORTED;
		break;
	}
	return ret;
}
void sedi_pwm_isr_handler(IN sedi_pwm_t instance)
{
	__SEDI_FTRACE__
	SEDI_ASSERT(instance < SEDI_PWM_NUM);
	sedi_pwm_id_t pwm = 0;
	uint32_t status = sedi_pwm[instance]->timersintstatus;

	while (pwm < SEDI_PWM_ID_NUM) {
		if (status & BIT(pwm)) {
			sedi_pwm[instance]->timer[pwm].controlreg |=
				PWM_INTR_DISABLE;
			if (pwm_runtime[instance].cb) {
				pwm_runtime[instance].cb(
					pwm_runtime[instance].cb_data, status);
			}
			sedi_pwm[instance]->timer[pwm].controlreg &=
				~PWM_INTR_DISABLE;
			sedi_pwm[instance]->timer[pwm].eoi;

			break;
		}
		pwm++;
	}
}

SEDI_ISR_DECLARE(sedi_pwm_0_isr) {
	sedi_pwm_isr_handler(SEDI_PWM_0);
}

SEDI_ISR_DECLARE(sedi_pwm_1_isr) {
	sedi_pwm_isr_handler(SEDI_PWM_1);
}

sedi_driver_version_t sedi_pwm_get_version(void)
{
	__SEDI_FTRACE__
	return pwm_driver_version;
}

int32_t sedi_pwm_start(IN sedi_pwm_t instance, IN sedi_pwm_id_t pwm)
{
	__SEDI_FTRACE__
	SEDI_ASSERT(instance < SEDI_PWM_NUM);
	SEDI_ASSERT(pwm < SEDI_PWM_ID_NUM);

	sedi_pwm[instance]->timer[pwm].controlreg |= PWM_START;

	return SEDI_DRIVER_OK;
}

int32_t sedi_pwm_stop(IN sedi_pwm_t instance, IN sedi_pwm_id_t pwm)
{
	__SEDI_FTRACE__
	SEDI_ASSERT(instance < SEDI_PWM_NUM);
	SEDI_ASSERT(pwm < SEDI_PWM_ID_NUM);

	sedi_pwm[instance]->timer[pwm].controlreg &= ~PWM_START;

	return SEDI_DRIVER_OK;
}

int32_t sedi_pwm_set_config(IN sedi_pwm_t instance, IN sedi_pwm_id_t pwm,
			    IN sedi_pwm_config_t cfg)
{
	__SEDI_FTRACE__
	SEDI_ASSERT(instance < SEDI_PWM_NUM);
	SEDI_ASSERT(pwm < SEDI_PWM_ID_NUM);

	switch (cfg.mode) {
	case SEDI_PWM_MODE_TIMER_FREERUNNING:
		sedi_pwm[instance]->timer[pwm].controlreg =
			PWM_TIMER_FREERUNNING;
		break;
	case SEDI_PWM_MODE_TIMER_COUNT:
		sedi_pwm[instance]->timer[pwm].loadcount = cfg.lo_count - 1;
		sedi_pwm[instance]->timer[pwm].controlreg = PWM_TIMER_MODE;
		break;
	case SEDI_PWM_MODE_PWM:
		/**
		 * HIGH period =
		 *	(TimerNLoadCount2 + 1) * clock period
		 *
		 * LOW period =
		 *	(TimerNLoadCount + 1) * clock period
		 */
		sedi_pwm[instance]->timer[pwm].loadcount = cfg.lo_count - 1;
		sedi_pwm[instance]->timer_loadcount2[pwm] = cfg.hi_count - 1;
		sedi_pwm[instance]->timer[pwm].controlreg =
			PWM_MODE | PWM_TIMER_MODE | PWM_INTR_DISABLE;
		break;
	default:
		return SEDI_DRIVER_ERROR_PARAMETER;
	}
	if (!cfg.intr_enable) {
		sedi_pwm[instance]->timer[pwm].controlreg |= PWM_INTR_DISABLE;
	}
	return SEDI_DRIVER_OK;
}

int32_t sedi_pwm_get_count(IN sedi_pwm_t instance, IN sedi_pwm_id_t pwm,
			   OUT uint32_t *current_count)
{
	__SEDI_FTRACE__
	SEDI_ASSERT(instance < SEDI_PWM_NUM);
	SEDI_ASSERT(pwm < SEDI_PWM_ID_NUM);
	SEDI_ASSERT(current_count != NULL);

	*current_count = sedi_pwm[instance]->timer[pwm].currentvalue;

	return SEDI_DRIVER_OK;
}

int32_t sedi_pwm_pin_set_nsec(IN sedi_pwm_t instance, IN sedi_pwm_id_t pwm,
			      IN uint32_t ns_period, IN uint32_t ns_pulse)
{
	__SEDI_FTRACE__
	uint32_t period_cycles, pulse_cycles, high, low;
	sedi_pwm_config_t cfg;

	period_cycles = NSECS_TO_CYCLES(ns_period);
	pulse_cycles = NSECS_TO_CYCLES(ns_pulse);

	if ((period_cycles == 0) || (pulse_cycles == 0) ||
	    (pulse_cycles > period_cycles) || (period_cycles > CYCLES_MAX) ||
	    (pulse_cycles > CYCLES_MAX)) {
		return SEDI_DRIVER_ERROR;
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

	sedi_pwm_stop(instance, pwm);

	cfg.mode = SEDI_PWM_MODE_PWM;
	cfg.intr_enable = 0;
	cfg.hi_count = high;
	cfg.lo_count = low;

	sedi_pwm_set_config(instance, pwm, cfg);

	sedi_pwm_start(instance, pwm);

	return SEDI_DRIVER_OK;
}

int32_t sedi_pwm_init(IN sedi_pwm_t instance, IN sedi_pwm_cb_t cb,
		      INOUT void *cb_data)
{
	__SEDI_FTRACE__
	SEDI_ASSERT(instance < SEDI_PWM_NUM);
	if (sedi_get_dev_ownership(PSE_DEV_PWM) != DEV_PSE_OWNED) {
		return SEDI_DRIVER_ERROR_NO_DEV;
	}

	/* SFT_RST for the instance */
	write32(SEDI_PWM_SFT_RST_REG,
		read32(SEDI_PWM_SFT_RST_REG) | (1 << instance));
	write32(SEDI_PWM_SFT_RST_REG,
		read32(SEDI_PWM_SFT_RST_REG) & ~(1 << instance));

	pwm_runtime[instance].cb = cb;
	pwm_runtime[instance].cb_data = cb_data;

	return SEDI_DRIVER_OK;
}
