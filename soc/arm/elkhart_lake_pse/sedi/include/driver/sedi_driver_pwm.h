/*
 * Copyright (c) 2021 Intel Corporation
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef __SEDI_DRIVER_PWM_H__
#define __SEDI_DRIVER_PWM_H__

#ifdef __cplusplus
extern "C" {
#endif

/*!
 * \defgroup sedi_driver_pwm PWM
 * \ingroup sedi_driver
 */

#include "sedi_driver_common.h"

#define SEDI_PWM_API_VERSION SEDI_DRIVER_VERSION_MAJOR_MINOR(0, 1)

/*!
 * \defgroup pwm_config PWM configurations
 * \ingroup sedi_driver_pwm
 * \{
 */

/**< Number of Controllers. */
typedef enum {
	SEDI_PWM_0 = 0, /**< Controller instance 0 */
	SEDI_PWM_1,     /**< Controller instance 1 */
	SEDI_PWM_NUM
} sedi_pwm_t;

/*!
 * \brief Channel ID
 */
typedef enum {
	SEDI_PWM_ID_0 = 0, /**< Channel 0 */
	SEDI_PWM_ID_1,     /**< Channel 1 */
	SEDI_PWM_ID_2,     /**< Channel 2 */
	SEDI_PWM_ID_3,     /**< Channel 3 */
	SEDI_PWM_ID_4,     /**< Channel 4 */
	SEDI_PWM_ID_5,     /**< Channel 5 */
	SEDI_PWM_ID_6,     /**< Channel 6 */
	SEDI_PWM_ID_7,     /**< Channel 7 */
	SEDI_PWM_ID_NUM
} sedi_pwm_id_t;

/*!
 * \brief PWM Mode
 */
typedef enum {
	SEDI_PWM_MODE_TIMER_FREERUNNING = 0, /**< Free-running timer mode */
	SEDI_PWM_MODE_TIMER_COUNT,	   /**< Count-based timer mode */
	SEDI_PWM_MODE_PWM		     /**< PWM mode */
} sedi_pwm_mode_t;

/*!
 * \strcut sedi_pwm_config_
 * \brief Structure representing Single PWM channel
 */
typedef struct {
	/*!
	 * Number of cycles the PWM output is driven low.
	 * In timer mode, this is the timer load count. Must be > 0.
	 */
	uint32_t lo_count;
	/*!
	 * Number of cycles the PWM output is driven high.
	 * Not applicable in timer mode. Must be > 0.
	 */
	uint32_t hi_count;
	/**< Enabled interrupt */
	bool intr_enable;
	/**< Operating mode */
	sedi_pwm_mode_t mode;

} sedi_pwm_config_t;

/*!
 * \}
 */

/*!
 * \defgroup pwm_driver_apis PWM Driver APIs
 * \ingroup sedi_driver_pwm
 * \brief PWM APIs help the user to program a PWM channel in a PWM mode or in
	a timer mode. In timer mode, a channel can be configured either as a
	free-running timer or as a user-defined count timer.
 * \{
 */
typedef void (*sedi_pwm_cb_t)(OUT void *cb_data, IN uint32_t status);

/*!
 * \brief PWM ISR handler. Can be registered by an OS or firmware.
 *
 * \param[in] instance Instance of the PWM block
 */
void sedi_pwm_isr_handler(IN sedi_pwm_t instance);

/*!
 * \brief Set PWM channel configuration.
 *
 * \param[in] instance Which PWM block to configure.
 * \param[in] pwm PWM channel in the block to configure.
 * \param[in] cfg New configuration for PWM. This must not be NULL.
 *
 * \return Standard errno return type for SEDI.
 * \retval SEDI_DRIVER_OK on success.
 * \retval Negative errno for possible error codes.
 */
int32_t sedi_pwm_set_config(IN sedi_pwm_t instance, IN sedi_pwm_id_t pwm,
			IN sedi_pwm_config_t cfg);

/*!
 * \brief Get current value of a timer
 *
 * \param[in] instance Which PWM block to configure.
 * \param[in] pwm PWM channel in the block to configure.
 * \param[in] current_count current value of the timer
 *
 * \return Standard errno return type for SEDI.
 * \retval Negative errno for possible error codes.
 */
int32_t sedi_pwm_get_count(IN sedi_pwm_t instance, IN sedi_pwm_id_t pwm,
		       OUT uint32_t *current_count);

/*!
 * Start a PWM/timer channel.
 *
 * \param[in] instance Which PWM block to configure.
 * \param[in] pwm PWM channel in the block to configure.
 *
 * \return Standard errno return type for SEDI.
 * \retval SEDI_DRIVER_OK on success.
 * \retval Negative errno for possible error codes.
 */
int32_t sedi_pwm_start(IN sedi_pwm_t instance, IN sedi_pwm_id_t pwm);

/*!
 * Stop a PWM/timer channel.
 *
 * \param[in] instance Which PWM block to configure.
 * \param[in] pwm PWM channel in the block to configure.
 *
 * \return Standard errno return type for SEDI.
 * \retval SEDI_DRIVER_OK on success.
 * \retval Negative errno for possible error codes.
 */
int32_t sedi_pwm_stop(IN sedi_pwm_t instance, IN sedi_pwm_id_t pwm);

/*!
 * \brief configures a pwm channel with period and pulse width in nanosecs
 *
 * \param[in] instance Which PWM block to configure.
 * \param[in] pwm PWM channel in the block to configure.
 * \param[in] ns_period period is nanoseconds
 * \param[in] ns_pulse pulse width in nanosecons
 *
 * \retval SEDI_DRIVER_OK on success.
 * \retval Negative errno for possible error codes.
 */
int32_t sedi_pwm_pin_set_nsec(IN sedi_pwm_t instance, IN sedi_pwm_id_t pwm,
			      IN uint32_t ns_period, IN uint32_t ns_pulse);

/*!
 * \brief initialize a pwm block
 *
 * \param[in] instance Which PWM block to configure.
 * \param[in] cb callback for the instance
 * \param[in] cb_data callback data to be passed if any
 *
 * \return Standard errno return type for SEDI.
 * \retval SEDI_DRIVER_OK on success.
 * \retval Negative errno for possible error codes.
 */
int32_t sedi_pwm_init(IN sedi_pwm_t instance, IN sedi_pwm_cb_t cb,
		      INOUT void *cb_data);

/*!
 * \brief Set power states for PWM device
 *
 * \param[in] instance Which PWM block to configure.
 * \param[in] state Power state
 *
 * \return SEDI_DRIVER_OK on success
 */
int32_t sedi_pwm_set_power(IN sedi_pwm_t instance, IN sedi_power_state_t state);

/*!
 * \brief get pwm driver API version
 *
 * \return driver API version
 */
sedi_driver_version_t sedi_pwm_get_version(void);

/*!
 * \}
 */

#ifdef __cplusplus
}
#endif

#endif /* _SEDI_DRIVER_PWM_H_ */
