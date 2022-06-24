/*
 * Copyright (c) 2021 Intel Corporation
 *
 * SPDX-License-Identifier:  BSD-3-Clause
 */

#ifndef _INTEL_HAL_EMMC_PRIV_H_
#define _INTEL_HAL_EMMC_PRIV_H_

/* eMMC context data structure */
struct intel_emmc_context {
	intel_instance_t instance;
	__IO_RW void *regs;
	emmc_host_card_capacity_t card_cap;
	uint32_t rca;
	uint8_t bus_width;
	uint32_t max_sectors;
	uint32_t cid[EMMC_CID_SIZE];
	uint32_t ext_csd[EMMC_EXTCSD_SIZE];
	uint32_t csd[EMMC_CSD_SIZE];
};

#endif /* _INTEL_HAL_EMMC_PRIV_H_ */
