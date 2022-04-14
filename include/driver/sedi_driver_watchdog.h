/*
 * Copyright (c) 2019-2021 Intel Corporation
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _SEDI_DRIVER_WATCHDOG_H_
#define _SEDI_DRIVER_WATCHDOG_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "sedi_driver_common.h"

/*!
 * \defgroup sedi_driver_watchdog WATCHDOG
 * \ingroup sedi_driver
 */

#define SEDI_WATCHDOG_API_VERSION SEDI_DRIVER_VERSION_MAJOR_MINOR(0, 1)

/*!
 * \defgroup watchdog_event WATCHDOG Event Types
 * \ingroup sedi_driver_watchdog
 * \{
 */

/*!
 * \def SEDI_WATCHDOG_EVENT_TRANSFER_DONE
 * \brief Watchdog first timeout warning
 */
#define SEDI_WATCHDOG_EVENT_TIMEOUT (1UL << 0)

/*!
 * \}
 */

/*!
 * \struct sedi_watchdog_status_t
 * \brief Watchdog Status
 * \ingroup sedi_driver_watchdog
 */
typedef volatile struct {
	uint32_t busy;
} sedi_watchdog_status_t;

/*!
 * \struct sedi_watchdog_capabilities_t
 * \brief Watchdog Driver Capabilities.
 * \ingroup sedi_driver_watchdog
 */
typedef struct {
	uint32_t reserved; /**< Reserved (must be zero) */
} sedi_watchdog_capabilities_t;

/*!
 * \struct sedi_watchdog_config_t
 * \brief Watchdog Config
 * \ingroup sedi_driver_watchdog
 */
typedef struct {
	uint32_t ms_low;  /* Time in milliseconds to trigger first timeout */
	uint32_t ms_high; /* Time in milliseconds to trigger second timeout */
} sedi_watchdog_config_t;

/*!
 * \defgroup watchdog_event_handler Watchdog Event Handler Callback
 * \ingroup sedi_driver_watchdog
 * \{
 */

/*!
 * \typedef sedi_watchdog_event_cb_t
 * \brief Callback function type for signal watchdog event.
 * \param[in] event: event type. see \ref watchdog_event
 * \param[in] arg: user parameter.
 * \return void
 */
typedef void (*sedi_watchdog_event_cb_t)(IN uint32_t event, IN void *arg);

/*!
 * \}
 */

/*!
 * \defgroup watchdog_function_calls WATCHDOG Driver Function Calls
 * \ingroup sedi_driver_watchdog
 * \{
 */

/*!
 * \brief Get the watchdog driver's API version.
 * \return the version of current watchdog driver's API
 */
sedi_driver_version_t sedi_watchdog_get_version(void);

/*!
 * \brief Get the device's capabilities.
 * \param[in] watchdog_device: watchdog device id
 * \return  the capabilities of specific watchdog device
 */
sedi_watchdog_capabilities_t
sedi_watchdog_get_capabilities(IN sedi_watchdog_t watchdog_device);

/*!
 * \brief Set the device's power
 * \param[in] device: watchdog device id
 * \param[in] state: the power state to be set to the device
 * \return  \ref return_status
 */
int32_t sedi_watchdog_set_power(IN sedi_watchdog_t device,
				IN sedi_power_state_t state);

/*!
 * \brief initialize the device
 * \param[in] device: watchdog device id
 * \param[in] cb: watchdog user callback
 * \param[in] arg: watchdog user parameter
 * \return \ref return_status
 */
int32_t sedi_watchdog_init(IN sedi_watchdog_t device,
			   IN sedi_watchdog_event_cb_t cb, IN void *arg);

/*!
 * \brief initialize the device
 * \param[in] device: watchdog device id
 * \return \ref return_status
 */
int32_t sedi_watchdog_uninit(IN sedi_watchdog_t device);

/*!
 * \brief reload the device
 * \param[in] device: watchdog device id
 * \return \ref return_status
 */
int32_t sedi_watchdog_feed(IN sedi_watchdog_t device);

/*!
 * \brief enable the device
 * \param[in] device: watchdog device id
 * \return \ref return_status
 */
int32_t sedi_watchdog_enable(IN sedi_watchdog_t device);

/*!
 * \brief disable the device
 * \param[in] device: watchdog device id
 * \return \ref return_status
 */
int32_t sedi_watchdog_disable(IN sedi_watchdog_t device);

/*!
 * \brief config the device
 * \param[in] device: watchdog device id
 * \param[in] config: watchdog configuration
 * \return \ref return_status
 */
int32_t sedi_watchdog_config(IN sedi_watchdog_t device,
			     IN sedi_watchdog_config_t *config);

/*!
 * \brief config the WDT behavior in sleep
 * \param[in] enable: if the wdt should be paused in sleep
 * \return \ref none
 */
void sedi_watchdog_pause_in_sleep(IN uint32_t enable);

/*!
 * \brief Set the max timeout time in seconds, default is 1s if not change.
 * \param[in] device: watchdog device id
 * \param[in] time The max WDT timeout seconds
 * \return \ref return_status
 */
int32_t sedi_watchdog_set_max_timeout(IN sedi_watchdog_t device,
				      IN uint32_t time);

/*!
 * \brief Get the max timeout time in seconds, default is 1s if not change.
 * \param[in] device: watchdog device id
 * \return \ref Max timeout in seconds
 */
int32_t sedi_watchdog_get_max_timeout(IN sedi_watchdog_t device);

/*!
 * \}
 */

#ifdef __cplusplus
}
#endif

#endif /* _SEDI_DRIVER_WATCHDOG_H_*/
