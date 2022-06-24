/*
 * Copyright (c) 2021 Intel Corporation
 *
 * SPDX-License-Identifier:  BSD-3-Clause
 */

#ifndef _INTEL_HAL_PWM_PRIV_H_
#define _INTEL_HAL_PWM_PRIV_H_

/* PWM context data structure */
struct intel_pwm_context {
	intel_instance_t instance;
	__IO_RW void *regs;
	intel_pwm_cb_t cb;
	void *cb_data;
	bool regs_valid;
};

#endif /*_INTEL_HAL_PWM_PRIV_H_*/
