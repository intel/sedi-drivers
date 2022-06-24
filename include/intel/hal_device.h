/*
 * Copyright (c) 2021 Intel Corporation
 *
 * SPDX-License-Identifier:  BSD-3-Clause
 */

#ifndef _INTEL_HAL_DEVICE_H_
#define _INTEL_HAL_DEVICE_H_

#include <stddef.h>

typedef struct {
	uintptr_t base_addr;
	uintptr_t phy_addr;
} intel_instance_t;

/*!
 * \brief Set the device base address
 *
 * \param[in] inst: Device instance
 * \param[in] base_addr: Device instance base address
 */
static inline void intel_set_base_addr(INOUT intel_instance_t *inst,
				       IN uintptr_t base_addr)
{
	inst->base_addr = base_addr;
}

/*!
 * \brief Set the device physical address
 *
 * \param[in] inst: Device instance
 * \param[in] base_addr: Device instance phy address
 */
static inline void intel_set_phy_addr(INOUT intel_instance_t *inst,
                                       IN uintptr_t phy_addr)
{
        inst->phy_addr = phy_addr;
}
#endif /* _INTEL_HAL_DEVICE_H_ */
