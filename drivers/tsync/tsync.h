/*
 * Copyright (c) 2021 Intel Corporation
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */


#ifndef _TSYNC_H_
#define _TSYNC_H_

#include <sedi_driver.h>

#define MISC_ART_SNAPSHOT_LO (MISC_BASE + 0xa0)
#define MISC_ART_SNAPSHOT_HI (MISC_BASE + 0xa4)
#define MISC_ART_SNAPSHOT_CTRL (MISC_BASE + 0xa8)
#define MISC_ART_SNAPSHOT_EN (1 << 0)
#define MISC_ART_SNAPSHOT_VALID (1 << 1)
#define MISC_TSYNC_ART_TS_CTRL (MISC_BASE + 0xC0)
#define TSYNC_TIMESTAMP_EN (1 << 0)
#define TSYNC_FW_UPDATE (1 << 1)
#define TSYNC_ART_TS_RD_EN (1 << 2)
#define MISC_TSYNC_ART_TB_STAMP_LOW (MISC_BASE + 0xC4)
#define MISC_TSYNC_ART_TB_STAMP_HIGH (MISC_BASE + 0xC8)
#define MISC_TSYNC_SATURATION (MISC_BASE + 0xCC)
#define TSYNC_SAT_STAT (1 << 0)
#define MISC_TSYNC_ART_RD_TS_SNAPSHOT_LOW (MISC_BASE + 0xD0)
#define MISC_TSYNC_ART_RD_TS_SNAPSHOT_HIGH (MISC_BASE + 0xD4)

/* tsync internal context */
typedef struct sedi_tsync_ctx {
	uint32_t tsync_freq; /* TSYNC frequency */
	bool sync;	   /* If TSYNC is syncing */
	bool sync_succeed;   /* If last sync succeed */
} sedi_tsync_ctx_t;

#endif
