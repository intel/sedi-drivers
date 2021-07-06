/*
 * Copyright (c) 2019-2021 Intel Corporation
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "pm/pm_internal_if.h"
#include <sedi.h>
#include <sedi_driver_watchdog.h>
#include <stdint.h>

#define SEDI_WATCHDOG_DRV_VERSION SEDI_DRIVER_VERSION_MAJOR_MINOR(1, 0)

#define CCU_WDT_DIVIDER_VALUE (sedi_pm_get_lbw_clock() / 255)

#define REG_CTRL_ENABLE BIT(17)
#define REG_CTRL_T1 (0)
#define REG_CTRL_T2 (8)
#define REG_RELOAD BIT(0)
#define REG_T1_T2_MAX (0xFF)
#define MAX_DIV_VAL (0xFFFFFFFF)
#define US_PER_SEC (1000000)
#define US_PER_MS (1000)
struct watchdog_regs {
	volatile uint32_t ctrl;
	volatile uint32_t reload;
	volatile uint32_t value;
};

struct watchdog_context {
	uint32_t regbase;
	sedi_watchdog_status_t status;
	sedi_watchdog_capabilities_t cap;
	sedi_watchdog_event_cb_t callback;
	const void *arg;
	sedi_power_state_t pm_state;
	bool is_clock_configured;
	uint32_t time_per_count;
	uint32_t max_time;
};

static struct watchdog_context context[SEDI_WATCHDOG_NUM] = {
	{.regbase = SEDI_WATCHDOG_0_REG_BASE, .max_time = MS_PER_SEC}};

static const sedi_driver_version_t driver_version = {SEDI_WATCHDOG_API_VERSION,
						     SEDI_WATCHDOG_DRV_VERSION};

sedi_driver_version_t sedi_watchdog_get_version(void)
{
	return driver_version;
}

sedi_watchdog_capabilities_t
sedi_watchdog_get_capabilities(IN sedi_watchdog_t device)
{
	return context[device].cap;
}

int32_t sedi_watchdog_set_power(IN sedi_watchdog_t device,
				IN sedi_power_state_t state)
{
	DBG_CHECK(device < SEDI_WATCHDOG_NUM, SEDI_DRIVER_ERROR_PARAMETER);
	DBG_CHECK(((state == SEDI_POWER_OFF) || (state == SEDI_POWER_LOW) ||
		   (state == SEDI_POWER_FULL)),
		  SEDI_DRIVER_ERROR_PARAMETER);

	context[device].pm_state = state;

	return SEDI_DRIVER_OK;
}

int32_t sedi_watchdog_init(IN sedi_watchdog_t device,
			   IN sedi_watchdog_event_cb_t cb, IN void *arg)
{
	DBG_CHECK(device < SEDI_WATCHDOG_NUM, SEDI_DRIVER_ERROR_PARAMETER);

	if (context[device].is_clock_configured == false) {
		PM_SET_CLOCK_DIV_WDT(CCU_WDT_DIVIDER_VALUE);
		context[device].time_per_count = US_PER_SEC / REG_T1_T2_MAX;
		context[device].max_time = MS_PER_SEC;
	}

	context[device].callback = cb;
	context[device].arg = arg;

	return SEDI_DRIVER_OK;
}

int32_t sedi_watchdog_uninit(IN sedi_watchdog_t device)
{
	DBG_CHECK(device < SEDI_WATCHDOG_NUM, SEDI_DRIVER_ERROR_PARAMETER);

	struct watchdog_regs *regs = (void *)context[device].regbase;

	regs->ctrl &= ~REG_CTRL_ENABLE;

	context[device].callback = NULL;
	context[device].is_clock_configured = false;

	return SEDI_DRIVER_OK;
}

int32_t sedi_watchdog_config(IN sedi_watchdog_t device,
			     IN sedi_watchdog_config_t *config)
{
	DBG_CHECK(device < SEDI_WATCHDOG_NUM, SEDI_DRIVER_ERROR_PARAMETER);
	DBG_CHECK(config != NULL, SEDI_DRIVER_ERROR_PARAMETER);

	uint32_t ctrl;
	uint32_t cycles_low;
	uint32_t cycles_high;
	uint32_t time_per_cycle = context[device].time_per_count;
	uint32_t max_time_ms = context[device].max_time;
	struct watchdog_regs *regs = (void *)context[device].regbase;

	/* can't config if watchdog running */
	if (regs->ctrl & REG_CTRL_ENABLE)
		return SEDI_DRIVER_ERROR;

	/* If config timeout time bigger than max time or less than one cycle
	 * return error.
	 */
	if ((config->ms_high > max_time_ms) || (config->ms_low > max_time_ms) ||
	    (config->ms_high * US_PER_MS < time_per_cycle) ||
	    (config->ms_low * US_PER_MS < time_per_cycle)) {
		return SEDI_DRIVER_ERROR_PARAMETER;
	}

	/* Clear the control register */
	regs->ctrl = 0;

	cycles_low = config->ms_low * US_PER_MS / time_per_cycle;
	cycles_high = config->ms_high * US_PER_MS / time_per_cycle;

	if (cycles_low > REG_T1_T2_MAX) {
		cycles_low = REG_T1_T2_MAX;
	}

	if (cycles_high > REG_T1_T2_MAX) {
		cycles_high = REG_T1_T2_MAX;
	}

	ctrl = (cycles_high << REG_CTRL_T2) | (cycles_low << REG_CTRL_T1);

	regs->ctrl |= ctrl;

	return SEDI_DRIVER_OK;
}

int32_t sedi_watchdog_feed(IN sedi_watchdog_t device)
{
	DBG_CHECK(device < SEDI_WATCHDOG_NUM, SEDI_DRIVER_ERROR_PARAMETER);

	struct watchdog_regs *regs = (void *)context[device].regbase;
	volatile uint32_t wdtv;
	uint32_t timeout_count;

	regs->reload |= REG_RELOAD;

	/* Waiting for data reloaded */
	timeout_count = (regs->ctrl & 0xFF);
	wdtv = regs->value;
	while ((wdtv & 0xFF) < timeout_count) {
		wdtv = regs->value;
	}

	return SEDI_DRIVER_OK;
}

int32_t sedi_watchdog_enable(IN sedi_watchdog_t device)
{
	DBG_CHECK(device < SEDI_WATCHDOG_NUM, SEDI_DRIVER_ERROR_PARAMETER);

	struct watchdog_regs *regs = (void *)context[device].regbase;

	regs->ctrl |= REG_CTRL_ENABLE;

	return SEDI_DRIVER_OK;
}

int32_t sedi_watchdog_disable(IN sedi_watchdog_t device)
{
	DBG_CHECK(device < SEDI_WATCHDOG_NUM, SEDI_DRIVER_ERROR_PARAMETER);

	struct watchdog_regs *regs = (void *)context[device].regbase;

	regs->ctrl &= ~REG_CTRL_ENABLE;

	return SEDI_DRIVER_OK;
}

void sedi_watchdog_0_isr(void)
{
	if (context[SEDI_WATCHDOG_0].callback) {
		context[SEDI_WATCHDOG_0].callback(SEDI_WATCHDOG_EVENT_TIMEOUT,
						  context[SEDI_WATCHDOG_0].arg);
	}
}

void sedi_watchdog_pause_in_sleep(IN uint32_t enable)
{
	pm_set_wdt_cg(enable);
}

int32_t sedi_watchdog_set_max_timeout(IN sedi_watchdog_t device,
				      IN uint32_t time)
{
	DBG_CHECK(device < SEDI_WATCHDOG_NUM, SEDI_DRIVER_ERROR_PARAMETER);

	uint32_t div = sedi_pm_get_lbw_clock() / REG_T1_T2_MAX * time;

	if ((time == 0) ||
	    (time >= (MAX_DIV_VAL / sedi_pm_get_lbw_clock() * REG_T1_T2_MAX))) {
		return SEDI_DRIVER_ERROR_PARAMETER;
	}

	PM_SET_CLOCK_DIV_WDT(div);

	context[device].is_clock_configured = true;
	/* How many microseconds per count */
	context[device].time_per_count = US_PER_SEC * time / REG_T1_T2_MAX;
	context[device].max_time = MS_PER_SEC * time;

	return SEDI_DRIVER_OK;
}

int32_t sedi_watchdog_get_max_timeout(IN sedi_watchdog_t device)
{
	DBG_CHECK(device < SEDI_WATCHDOG_NUM, SEDI_DRIVER_ERROR_PARAMETER);

	return (context[device].max_time / MS_PER_SEC);
}
