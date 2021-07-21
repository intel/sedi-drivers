/*
 * Copyright (c) 2021 Intel Corporation
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <sedi_driver_rtc.h>
#include <sedi_driver_sideband.h>

/* Remote RTC register offset */
#define REMOTE_RTC_PORT (0xc3)
#define REMOTE_RTC_WRITE_ATTR (0x000103c3)
#define REMOTE_RTC_READ_ATTR (0x000202c3)
#define REMOTE_RTC_WRITE_CMD (0x1)
#define REMOTE_RTC_READ_CMD (0x3)
#define REMOTE_RTC_WRITE_ADDR (0x70)
#define REMOTE_RTC_READ_ADDR (0x71)
#define RTC_SECONDS (0)
#define RTC_MINUTES (2)
#define RTC_HOURS (4)
#define RTC_DAY_OF_WEEK (6)
#define RTC_DAY_OF_MONTH (7)
#define RTC_MONTH (8)
#define RTC_YEAR (9)
#define RTC_REG_B (11)

/* driver version */
#define SEDI_RTC_DRIVER_VERSION SEDI_DRIVER_VERSION_MAJOR_MINOR(0, 1)

/* driver version */
static const sedi_driver_version_t driver_version = {SEDI_RTC_API_VERSION,
						     SEDI_RTC_DRIVER_VERSION};

/* driver capabilities */
static const sedi_rtc_capabilities_t driver_capabilities = {
	.support_alarm = 0, .support_callback = 0, .reserved = 0};

static uint64_t sedi_rtc_to_us(uint64_t rtc_orig)
{
	uint64_t us_rtc = (rtc_orig * USECS_PER_SEC) / RTC_TICKS_IN_SECOND;
	return us_rtc;
}

static uint8_t read_remote_rtc_reg(uint32_t reg)
{
	uint32_t data = 0;
	/* First write to REMOTE_RTC_WRITE_ADDR */
	sedi_sideband_upstream_write_raw(
	    SEDI_SIDEBAND_0, REMOTE_RTC_WRITE_ADDR, 0x0, REMOTE_RTC_WRITE_ATTR,
	    reg, REMOTE_RTC_WRITE_CMD, 0, SEDI_SIDEBAND_ACTION_WRITE);
	sedi_sideband_wait_ack(SEDI_SIDEBAND_0, REMOTE_RTC_PORT,
			       SEDI_SIDEBAND_ACTION_WRITE, &data);
	/* Then read REMOTE_RTC_READ_ADDR */
	sedi_sideband_upstream_write_raw(
	    SEDI_SIDEBAND_0, REMOTE_RTC_READ_ADDR, 0x0, REMOTE_RTC_READ_ATTR,
	    reg, REMOTE_RTC_READ_CMD, 0, SEDI_SIDEBAND_ACTION_READ);
	sedi_sideband_wait_ack(SEDI_SIDEBAND_0, REMOTE_RTC_PORT,
			       SEDI_SIDEBAND_ACTION_WRITE, &data);

	return (uint8_t)data;
}

/* Convert BCD format to binary */
static uint8_t bcd2binary(uint8_t val)
{
	return (((val >> 4) & 0xF) * 10 + (val & 0xF));
}

sedi_driver_version_t sedi_rtc_get_version(void)
{
	return driver_version;
}

int sedi_rtc_get_capabilities(sedi_rtc_capabilities_t *cap)
{
	SEDI_ASSERT(cap);

	cap->support_alarm = driver_capabilities.support_alarm;
	cap->support_callback = driver_capabilities.support_callback;

	return SEDI_DRIVER_OK;
}

int sedi_rtc_init(void)
{
	/* PSE RTC have no alarm and interrupt feature, callback not used */
	return SEDI_DRIVER_OK;
}

int sedi_rtc_uninit(void)
{
	return SEDI_DRIVER_OK;
}

int sedi_rtc_set_power(sedi_power_state_t state)
{
	PARAM_UNUSED(state);

	return SEDI_DRIVER_OK;
}

uint64_t sedi_rtc_get(void)
{
	uint32_t lower;
	uint32_t upper;

	do {
		upper = read32(SEDI_RTC_COUNTER1);
		lower = read32(SEDI_RTC_COUNTER0);
	} while (upper != read32(SEDI_RTC_COUNTER1));

	return ((uint64_t)upper << 32U) | lower;
}

void sedi_rtc_get_us(INOUT uint64_t *us)
{
	/* returns the current time in micro seconds. */
	uint64_t result;
	uint64_t rtc = sedi_rtc_get();

	result = sedi_rtc_to_us(rtc);
	*us = result;
}

void sedi_rtc_get_soc_time(INOUT sedi_rtc_time_t *rtc_time)
{
	SEDI_ASSERT(rtc_time);

	/* Use sideband to get the time */
	rtc_time->seconds = bcd2binary(read_remote_rtc_reg(RTC_SECONDS));
	rtc_time->minute = bcd2binary(read_remote_rtc_reg(RTC_MINUTES));
	rtc_time->hour = bcd2binary(read_remote_rtc_reg(RTC_HOURS));
	rtc_time->day = bcd2binary(read_remote_rtc_reg(RTC_DAY_OF_MONTH));
	rtc_time->month = bcd2binary(read_remote_rtc_reg(RTC_MONTH));
	rtc_time->year = bcd2binary(read_remote_rtc_reg(RTC_YEAR));
}
