/*
 * Copyright (c) 2021 Intel Corporation
 *
 * SPDX-License-Identifier:  BSD-3-Clause
 */

#ifndef _INTEL_HAL_TGPIO_PRIV_H_
#define _INTEL_HAL_TGPIO_PRIV_H_

/**
 * TGPIO pinmux selection for an instance - * Internal use *
     pin_mux_sel	Owner for GPIO Pins
			[29:20]	[19:10]	[9:0]
	2'b00		GPIO	GPIO	GPIO
	2'b01		TGPIO	TGPIO	GPIO
	2'b10		GPIO	TGPIO	TGPIO
	2'b11		TGPIO	GPIO	TGPIO
 */
typedef enum pinmux {
	INTEL_TGPIO_PINMUX_1 = 1, /**< Pinmux setting 1 */
	INTEL_TGPIO_PINMUX_2,     /**< Pinmux setting 2 */
	INTEL_TGPIO_PINMUX_3      /**< Pinmux setting 3 */
} intel_tgpio_pinmux_t;

/* TGPIO context data structure */
struct intel_tgpio_context {
	intel_instance_t instance;
	__IO_RW void *regs;
	intel_tgpio_cb_t cb;
	void *cb_data;
	bool regs_valid;
	int pmx;
};

/* Driver Version */
#define INTEL_TGPIO_DRIVER_VERSION INTEL_DRIVER_VERSION_MAJOR_MINOR(0, 1)

static const intel_driver_version_t tgpio_driver_version = {
	INTEL_TGPIO_API_VERSION, INTEL_TGPIO_DRIVER_VERSION};

#endif /* _INTEL_TGPIO_H_ */
