/*
 * Copyright (c) 2021 Intel Corporation
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef __TSYNC_H
#define __TSYNC_H

#ifdef __cplusplus
extern "C" {
#endif

#include "sedi_driver_common.h"

/*!
 * \defgroup sedi_driver_tsync TSYNC
 * \ingroup sedi_driver
 */

#define SEDI_TSYNC_API_VERSION SEDI_DRIVER_VERSION_MAJOR_MINOR(0, 1)

/*!
 * \struct sedi_tsync_capabilities_t
 * \brief TSYNC Driver Capabilities.
 * \ingroup sedi_driver_tsync
 */
typedef struct sedi_tsync_capabilities {
	uint32_t reserved;
} sedi_tsync_capabilities_t;

/*!
 * \defgroup tsync_function_calls TSYNC Driver Function Calls
 * \ingroup sedi_driver_tsync
 * \{
 */

/*!
 * \brief Get the tsync driver's API version.
 * \return the version of current tsync driver API
 */
sedi_driver_version_t sedi_tsync_get_version(void);

/*!
 * \brief Get the device's capabilities.
 * \return the capabilities of tsync device
 */
sedi_tsync_capabilities_t sedi_tsync_get_capabilities(void);

/*!
 * \brief Initialize the device
 * \return \ref return_status
 */
int32_t sedi_tsync_init(void);

/*!
 * \brief Uninitialize the device
 * \return \ref return_status
 */
int32_t sedi_tsync_uninit(void);

/*!
 * \brief Set the device's power
 * \param[in] state: the power state to be set to the device
 * \return \ref return_status
 */
int32_t sedi_tsync_set_power(IN sedi_power_state_t state);

/*!
 * \brief Sync the time-synchronization
 * \return \ref return_status
 */
int32_t sedi_tsync_sync(void);

/*!
 * \brief Get tsync time
 * \param[INOUT] tsync: tsync value got
 * \return \ref return_status
 */
int32_t sedi_tsync_get_time(INOUT uint64_t *tsync);

/*!
 * \brief Get tsync enable state
 * \return True if tsync enabled, false if not
 */
bool sedi_tsync_is_enabled(void);

/*!
 * \}
 */

#ifdef __cplusplus
}
#endif

#endif /*__TSYNC_H*/
