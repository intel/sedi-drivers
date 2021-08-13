/*
 * Copyright (c) 2021 Intel Corporation
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _SEDI_DRIVER_DTF_H_
#define _SEDI_DRIVER_DTF_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "sedi_driver_common.h"

/*!
 * \defgroup sedi_driver_dtf DTF
 * \ingroup sedi_driver
 */

#define SEDI_DTF_API_VERSION SEDI_DRIVER_VERSION_MAJOR_MINOR(0, 1)

typedef union {
	uint32_t data;
	struct {
		/**< SVEN event type ID             */
		uint32_t et_type : 4;
		/**< severity level of message      */
		uint32_t et_severity : 3;
		/**< indicate 32bit sequence number */
		uint32_t et_sequence : 1;
		/**< indicate location information  */
		uint32_t et_location : 1;
		/**< indicate variable length event */
		uint32_t et_length : 1;
		/**< indicate 32bit CRC             */
		uint32_t et_chksum : 1;
		/**< indicate header extension      */
		uint32_t et_ext : 1;
		/**< module instance ID value       */
		uint32_t et_unit : 4;
		uint32_t et_module : 8;
		uint32_t et_subtype : 8;
	};
} sven_hdr_t;

/**
 * Message severity level enumeration
 */
typedef enum e_sven_severity {
	SVEN_SEVERITY_NONE = 0,    /**< undefined severity         */
	SVEN_SEVERITY_FATAL = 1,   /**< critical error level       */
	SVEN_SEVERITY_ERROR = 2,   /**< error message level        */
	SVEN_SEVERITY_WARNING = 3, /**< warning message level      */
	SVEN_SEVERITY_NORMAL = 4,  /**< normal message level       */
	SVEN_SEVERITY_USER1 = 5,   /**< user defined level 5       */
	SVEN_SEVERITY_USER2 = 6,   /**< user defined level 6       */
	SVEN_SEVERITY_USER3 = 7    /**< user defined level 7       */
} sven_severity_t;

/*!
 * * SVEN Major event Types
 */
enum e_sven_eventtype_t {
	SVEN_EVENT_TYPE_INVALID = 0,      /**< NO ZEROES ALLOWED         */
	SVEN_EVENT_TYPE_DEBUG_STRING = 2, /**< text message output       */
	SVEN_EVENT_TYPE_MAX
};

/*!
 ** SVEN_event_type_debug_string Sub-Types
 */
typedef enum e_sven_eventtype_debugstr {
	SVEN_DEBUGSTR_INVALID = 0, /**< no zeroes allowed            */
	SVEN_DEBUGSTR_GENERIC = 1, /**< String generic debug         */
	SVEN_DEBUGSTR_MAX
} sven_eventtype_debugstr_t;

/*!
 * \struct sedi_dtf_capabilities_t
 * \brief DTF Driver Capabilities.
 * \ingroup sedi_driver_dtf
 */
typedef struct sedi_dtf_capabilities {
	uint32_t reserved;
} sedi_dtf_capabilities_t;

/*!
 * \defgroup dtf_function_calls DTF Driver Function Calls
 * \ingroup sedi_driver_dtf
 * \{
 */

/*!
 * \brief Get the dtf driver's API version.
 * \return the version of current dtf driver's API
 */
sedi_driver_version_t sedi_dtf_get_version(void);

/*!
 * \brief Get the device's capabilities.
 * \param[inout] the capabilities of dtf device
 * \return return_status
 */
int sedi_dtf_get_capabilities(INOUT sedi_dtf_capabilities_t *cap);

/*!
 * \brief Initialize the device
 * \return  \ref return_status
 */
int32_t sedi_dtf_init(void);

/*!
 * \brief Uninitailize the device
 * \return  \ref return_status
 */
int32_t sedi_dtf_uninit(void);

/*!
 * \brief Set the device's power
 * \param[in] state: the power state to be set to the device
 * \return  \ref return_status
 */
int32_t sedi_dtf_set_power(IN sedi_power_state_t state);

/*!
 * \brief Send the log in buffer to DTF
 * \param[in] buf: buffer contains the log
 * \param[in] len: buffer length
 * \param[in] sven_hdr: SVEN header of the NPK message
 * \param[in] ts: timestamp of the log
 * \return void
 */
int32_t sedi_dtf_send(IN char *buf, IN uint32_t len, IN sven_hdr_t sven_hdr);

/*!
 * \}
 */

#ifdef __cplusplus
}
#endif

#endif /* _SEDI_DRIVER_DTF_H_*/
