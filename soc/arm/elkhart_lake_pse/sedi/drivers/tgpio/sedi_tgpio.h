/*
 * Copyright (c) 2021 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _SEDI_TGPIO_H_
#define _SEDI_TGPIO_H_

#include "sedi_tgpio_regs.h"

/* Runtime structure */
typedef struct runtime {
	sedi_tgpio_cb_t cb;
	void *cb_data;
	tgpio_regs_t regs_saved;
	bool regs_valid;
} tgpio_runtime_t;

/* Resource data structure */
typedef const struct tgpio_resource {
	tgpio_regs_t *regs;
} tgpio_resource_t;

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
	SEDI_TGPIO_PINMUX_1 = 1, /**< Pinmux setting 1 */
	SEDI_TGPIO_PINMUX_2,     /**< Pinmux setting 2 */
	SEDI_TGPIO_PINMUX_3      /**< Pinmux setting 3 */
} sedi_tgpio_pinmux_t;

/* Driver Version */
#define SEDI_TGPIO_DRIVER_VERSION SEDI_DRIVER_VERSION_MAJOR_MINOR(0, 1)

static const sedi_driver_version_t tgpio_driver_version = {
    SEDI_TGPIO_API_VERSION, SEDI_TGPIO_DRIVER_VERSION};

#endif /* _SEDI_TGPIO_H_ */
