/*
 * Copyright (c) 2021 Intel Corporation
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include "sedi.h"

#ifndef _SEDI_PWM_H_
#define _SEDI_PWM_H_

#define PWM_NS_PER_TICK (50)
#define PWM_TOTAL_CYCLES (100)
#define PWM_TOTAL_NUM (2)

#define PWM_START BIT(0)
#define PWM_TIMER_MODE BIT(1)
#define PWM_INTR_DISABLE BIT(2)
#define PWM_MODE BIT(3)
#define PWM_TIMER_FREERUNNING (0)

/* Timer registers */
typedef struct {
	__IO_RW uint32_t loadcount;     /* Load Count */
	__IO_RW uint32_t currentvalue;  /* Current Value */
					/* Timer control register
					 *  BIT(31-4): Reserved
					 *  BIT(3)   : Mode
					 *		     1 - PWM Mode
					 *		     0 - Timer Mode
					 *  BIT(2)   : Interrupt Mask
					 *		     1 - Intr Masked
					 *		     0 - Intr not masked
					 *  BIT(1)   : Timer Mode
					 *		     1 - Timer User-defined count
					 *		     0 - Timer Free running counter
					 *  BIT(0)   : Timer Enable
					 *		     1 - Enable
					 *		     0 - Disable
					 */
	__IO_RW uint32_t controlreg;
	__IO_RW uint32_t eoi;           /* End Of Interrupt */
	__IO_RW uint32_t intstatus;     /* Interrupt Status */
} sedi_pwm_channel_t;

/* IP Block Register map. */
typedef struct {
	sedi_pwm_channel_t timer[SEDI_PWM_ID_NUM];
	__IO_RW uint32_t timersintstatus;               /* Timers Interrupt Status */
	__IO_RW uint32_t timerseoi;                     /* Timers End Of Intr */
	__IO_RW uint32_t timersrawintstatus;            /* Timers Raw Intr Status */
	__IO_RW uint32_t timerscompversion;             /* Timers Component Version */

	__IO_RW uint32_t
		timer_loadcount2[SEDI_PWM_ID_NUM];      /* Timer Load Count 2 */
} sedi_pwm_reg_t;

#endif /*_SEDI_PWM_H_ */
