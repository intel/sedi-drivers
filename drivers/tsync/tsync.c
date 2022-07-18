/*
 * Copyright (c) 2021 Intel Corporation
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */


#include "tsync.h"
#include "pm/pm_internal_if.h"
#include <sedi.h>

#define SYNC_START_CMD_ATTR (0x000f50cc)
#define SYNC_START_CMD_DATA (0x9)
#define SYNC_START_CMD (0x5)
#define LOCAL_SYNC_OPCODE (0x51)
#define SYNC_COMP_OPCODE (0x92)
#define TSYNC_POLL_TIMEOUT 1000
#define TSYNC_SYNC_TRY_MAX 5

#define SEDI_TSYNC_DRIVER_VERSION SEDI_DRIVER_VERSION_MAJOR_MINOR(0, 1)

static const sedi_driver_version_t driver_version = {SEDI_TSYNC_API_VERSION,
						     SEDI_TSYNC_DRIVER_VERSION};

static const sedi_tsync_capabilities_t driver_cap = {0};

sedi_tsync_ctx_t tsync_ctx = {0};

static int safe_poll(uint32_t addr, uint32_t expected, uint32_t mask,
		     uint32_t cycles_timeout)
{
	int retval = SEDI_DRIVER_OK;
	uint64_t end_time = sedi_rtc_get() + ((uint64_t)cycles_timeout);

	while (1) {
		if ((read32(addr) & mask) == expected) {
			retval = SEDI_DRIVER_OK;
			break;
		}
		if (((uint64_t)end_time) < sedi_rtc_get()) {
			retval = SEDI_DRIVER_ERROR_TIMEOUT;
			break;
		}
	}
	return retval;
}

sedi_driver_version_t sedi_tsync_get_version(void)
{
	return driver_version;
}

sedi_tsync_capabilities_t sedi_tsync_get_capabilities(void)
{
	return driver_cap;
}

int32_t sedi_tsync_init(void)
{
	int32_t sts;
	/* Read TSYNC freq from Host, this could be 0 if driver is not
	 * loaded or if driver doesn't know the frequency in this case use a
	 * default value.
	 */
	tsync_ctx.tsync_freq = 0;
	if (tsync_ctx.tsync_freq == 0) {
		tsync_ctx.tsync_freq = TSYNC_DEFAULT_FREQ;
	}

	/* Register sideband client, use sync mode, no callback */
	sedi_sideband_register_client(SEDI_SIDEBAND_0, SB_DOWN_TSYNC,
				      LOCAL_SYNC_OPCODE, NULL, NULL);

	sts = sedi_tsync_sync();
	return sts;
}

int32_t sedi_tsync_uninit(void)
{
	tsync_ctx.sync = 0;
	sedi_sideband_unregister_client(SEDI_SIDEBAND_0, SB_DOWN_TSYNC);
	return SEDI_DRIVER_OK;
}

int32_t sedi_tsync_set_power(IN sedi_power_state_t state)
{
	PARAM_UNUSED(state);
	return SEDI_DRIVER_OK;
}

int32_t sedi_tsync_sync(void)
{
	int32_t sts = SEDI_DRIVER_ERROR;
	int32_t count = 0;
	uint64_t addr, comp_data, comp;
	bool sync_completed = false;

	/* If received Reset prepare or in Sx state, shall not sync */
	if (PM_IS_IN_SX()) {
		return sts;
	}

	/* Set sync flag to true */
	tsync_ctx.sync = true;
	tsync_ctx.sync_succeed = false;

	/* Disable sideband interrupt */
	sedi_sideband_set_interrupt(0, false);

	while (count++ < TSYNC_SYNC_TRY_MAX) {
		/* Arm the ART Timer Capture Logic */
		write32(MISC_TSYNC_ART_TS_CTRL,
			read32(MISC_TSYNC_ART_TS_CTRL) | TSYNC_TIMESTAMP_EN);

		/* send StartSyncCmd to ARU via sideband */
		sedi_sideband_upstream_write_raw(
		    SEDI_SIDEBAND_0, 0, 0, SYNC_START_CMD_ATTR,
		    SYNC_START_CMD_DATA, SYNC_START_CMD, 0,
		    SEDI_SIDEBAND_ACTION_WRITE);

		/* wait for LocalSync, ARU is responsible for setting the VNN */
		sedi_sideband_recv(SEDI_SIDEBAND_0, SB_PMC, LOCAL_SYNC_OPCODE,
				   &addr, &comp_data);

		sedi_sideband_send_ack(SEDI_SIDEBAND_0, SB_PMC,
				       SEDI_SIDEBAND_ACTION_WRITE,
				       SEDI_SIDEBAND_RESPONSE_COMPLETE, 0);

		/* Wait for SyncComp */
		sedi_sideband_recv(SEDI_SIDEBAND_0, SB_PMC, SYNC_COMP_OPCODE,
				   &addr, &comp);

		/* Update local ART timer */
		write64(MISC_TSYNC_ART_TB_STAMP_LOW,
			comp_data / TSYNC_PMC_LOCAL_MULTI);

		write32(MISC_TSYNC_ART_TS_CTRL,
			read32(MISC_TSYNC_ART_TS_CTRL) | TSYNC_FW_UPDATE);
		sts = safe_poll(MISC_TSYNC_ART_TS_CTRL, 0, TSYNC_FW_UPDATE,
				TSYNC_POLL_TIMEOUT);
		if (sts != SEDI_DRIVER_OK) {
			sts = SEDI_DRIVER_ERROR_TIMEOUT;
			break;
		}
		/* If saturation event happens, sync again */
		if (read32(MISC_TSYNC_SATURATION) & TSYNC_SAT_STAT) {
			/* Clear saturation status bit */
			write32(MISC_TSYNC_SATURATION, TSYNC_SAT_STAT);
			continue;
		}
		sts = SEDI_DRIVER_OK;
		sync_completed = true;
		break;
	}

	if (sync_completed) {
		/* Enable reading current free running ART value */
		write32(MISC_TSYNC_ART_TS_CTRL,
			read32(MISC_TSYNC_ART_TS_CTRL) | TSYNC_ART_TS_RD_EN);
		/* No any RTC is needed in tsync now */
		sts = SEDI_DRIVER_OK;
		tsync_ctx.sync_succeed = true;
	}

	/* Set sync flag to false */
	tsync_ctx.sync = false;

	/* Re-enable sideband interrupt */
	sedi_sideband_set_interrupt(0, true);

	return sts;
}

int32_t sedi_tsync_get_time(INOUT uint64_t *tsync)
{
	int sts;

	if (tsync_ctx.sync) {
		/* TSYNC is syncing, shall wait after sync finished */
		sts = SEDI_DRIVER_ERROR_BUSY;

	} else if (tsync_ctx.sync_succeed != true) {
		/* Last sync failed */
		sts = SEDI_DRIVER_ERROR;
	} else {
		write32(MISC_TSYNC_ART_TS_CTRL,
			read32(MISC_TSYNC_ART_TS_CTRL) | TSYNC_ART_TS_RD_EN);
		*tsync = read32(MISC_TSYNC_ART_RD_TS_SNAPSHOT_LOW) |
			 ((uint64_t)read32(MISC_TSYNC_ART_RD_TS_SNAPSHOT_HIGH)
			  << 32);
		sts = SEDI_DRIVER_OK;
	}

	return sts;
}

bool sedi_tsync_is_enabled(void)
{
	return (bool)(read32(MISC_TSYNC_ART_TS_CTRL) & TSYNC_TIMESTAMP_EN);
}
