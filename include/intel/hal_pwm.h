/*
 * Copyright (c) 2021 Intel Corporation
 *
 * SPDX-License-Identifier:  BSD-3-Clause
 */

#ifndef __INTEL_DRIVER_PWM_H__
#define __INTEL_DRIVER_PWM_H__

#ifdef __cplusplus
extern "C" {
#endif

/*!
 * \defgroup intel_driver_pwm PWM
 * \ingroup intel_driver
 */

#include <intel/hal_driver_common.h>
#include <intel/hal_device.h>

#define INTEL_PWM_API_VERSION INTEL_DRIVER_VERSION_MAJOR_MINOR(0, 1)
#define S0IX_CLK (100 * 1000000UL)

/*!
 * \brief Get driver context
 */
#define INTEL_PWM_INSTANCE(n)                                                  \
	((intel_instance_t *)((struct intel_pwm_context[]){{}}))

/*!
 * \defgroup pwm_config PWM configurations
 * \ingroup intel_driver_pwm
 * \{
 */

/**< Number of Controllers. */
typedef enum {
	INTEL_PWM_0 = 0, /**< Controller instance 0 */
	INTEL_PWM_1,     /**< Controller instance 1 */
	INTEL_PWM_NUM
} intel_pwm_t;

/*!
 * \brief Channel ID
 */
typedef enum {
	INTEL_PWM_ID_0 = 0, /**< Channel 0 */
	INTEL_PWM_ID_1,     /**< Channel 1 */
	INTEL_PWM_ID_2,     /**< Channel 2 */
	INTEL_PWM_ID_3,     /**< Channel 3 */
	INTEL_PWM_ID_4,     /**< Channel 4 */
	INTEL_PWM_ID_5,     /**< Channel 5 */
	INTEL_PWM_ID_6,     /**< Channel 6 */
	INTEL_PWM_ID_7,     /**< Channel 7 */
	INTEL_PWM_ID_NUM
} intel_pwm_id_t;

/*!
 * \brief PWM Mode
 */
typedef enum {
	INTEL_PWM_MODE_TIMER_FREERUNNING = 0, /**< Free-running timer mode */
	INTEL_PWM_MODE_TIMER_COUNT,	   /**< Count-based timer mode */
	INTEL_PWM_MODE_PWM		      /**< PWM mode */
} intel_pwm_mode_t;

/*!
 * \strcut intel_pwm_config_
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
	intel_pwm_mode_t mode;

} intel_pwm_config_t;

/*!
 * \}
 */

/*!
 * \defgroup pwm_driver_apis PWM Driver APIs
 * \ingroup intel_driver_pwm
 * \brief PWM APIs help the user to program a PWM channel in a PWM mode or in
	a timer mode. In timer mode, a channel can be configured either as a
	free-running timer or as a user-defined count timer.
 * \{
 */
typedef void (*intel_pwm_cb_t)(OUT void *cb_data, IN uint32_t status);

/*!
 * \brief PWM ISR handler. Can be registered by an OS or firmware.
 *
 * \param[in] instance Instance of the PWM block
 */
void intel_pwm_isr_handler(IN intel_instance_t *inst);

/*!
 * \brief Set PWM channel configuration.
 *
 * \param[in] instance Which PWM block to configure.
 * \param[in] pwm PWM channel in the block to configure.
 * \param[in] cfg New configuration for PWM. This must not be NULL.
 *
 * \return Standard errno return type for INTEL.
 * \retval INTEL_DRIVER_OK on success.
 * \retval Negative errno for possible error codes.
 */
int32_t intel_pwm_set_config(IN intel_instance_t *inst, IN intel_pwm_id_t pwm,
			     IN intel_pwm_config_t cfg);

/*!
 * \brief Get current value of a timer
 *
 * \param[in] instance Which PWM block to configure.
 * \param[in] pwm PWM channel in the block to configure.
 * \param[in] current_count current value of the timer
 *
 * \return Standard errno return type for INTEL.
 * \retval Negative errno for possible error codes.
 */
int32_t intel_pwm_get_count(IN intel_instance_t *inst, IN intel_pwm_id_t pwm,
			    OUT uint32_t *current_count);

/*!
 * Start a PWM/timer channel.
 *
 * \param[in] instance Which PWM block to configure.
 * \param[in] pwm PWM channel in the block to configure.
 *
 * \return Standard errno return type for INTEL.
 * \retval INTEL_DRIVER_OK on success.
 * \retval Negative errno for possible error codes.
 */
int32_t intel_pwm_start(IN intel_instance_t *inst, IN intel_pwm_id_t pwm);

/*!
 * Stop a PWM/timer channel.
 *
 * \param[in] instance Which PWM block to configure.
 * \param[in] pwm PWM channel in the block to configure.
 *
 * \return Standard errno return type for INTEL.
 * \retval INTEL_DRIVER_OK on success.
 * \retval Negative errno for possible error codes.
 */
int32_t intel_pwm_stop(IN intel_instance_t *inst, IN intel_pwm_id_t pwm);

/*!
 * \brief configures a pwm channel with period and pulse width in nanosecs
 *
 * \param[in] instance Which PWM block to configure.
 * \param[in] pwm PWM channel in the block to configure.
 * \param[in] ns_period period is nanoseconds
 * \param[in] ns_pulse pulse width in nanosecons
 *
 * \retval INTEL_DRIVER_OK on success.
 * \retval Negative errno for possible error codes.
 */
int32_t intel_pwm_pin_set_nsec(IN intel_instance_t *inst, IN intel_pwm_id_t pwm,
			       IN uint32_t ns_period, IN uint32_t ns_pulse);

/*!
 * \brief initialize a pwm block
 *
 * \param[in] instance Which PWM block to configure.
 * \param[in] cb callback for the instance
 * \param[in] cb_data callback data to be passed if any
 *
 * \return Standard errno return type for INTEL.
 * \retval INTEL_DRIVER_OK on success.
 * \retval Negative errno for possible error codes.
 */
int32_t intel_pwm_init(IN intel_instance_t *inst, IN intel_pwm_cb_t cb,
		       INOUT void *cb_data);

/*!
 * \brief get pwm driver API version
 *
 * \return driver API version
 */
intel_driver_version_t intel_pwm_get_version(void);

/*!
 * \}
 */

#ifdef __cplusplus
}
#endif

#include <intel/hal_pwm_priv.h>
#endif /* _INTEL_DRIVER_PWM_H_ */
