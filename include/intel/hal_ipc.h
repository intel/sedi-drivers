/*
 * Copyright (c) 2019 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _INTEL_DRIVER_IPC_H_
#define _INTEL_DRIVER_IPC_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <intel/hal_driver_common.h>
#include <intel/hal_device.h>

/****** IPC Interface type *****/

/*
 * struct intel_ipc
 * define IPC interface ID
 * \ingroup intel_driver_ipc
 */
enum intel_ipc_t {
	INTEL_IPC_HOST = 0,
	INTEL_IPC_CSME,
	INTEL_IPC_PMC,
	INTEL_IPC_NUM
};

/*!
 * \brief Get driver context
 */
#define INTEL_IPC_INSTANCE(n)                                                  \
	((intel_instance_t *)((struct ipc_context_t[]){{}}))

/****** IPC Event *****/

/*
 * INTEL_IPC_EVENT_MSG_IN
 * Received an coming ipc message
 * \ingroup intel_driver_ipc
 */
#define INTEL_IPC_EVENT_MSG_IN (1UL << 0)

/*
 * INTEL_IPC_EVENT_MSG_PEER_ACKED
 * An ipc busy bit is cleared by peer
 * \ingroup intel_driver_ipc
 */
#define INTEL_IPC_EVENT_MSG_PEER_ACKED (1UL << 1)

/*
 * INTEL_IPC_EVENT_MSG_OUT
 * An ipc message is received by peer
 * \ingroup intel_driver_ipc
 */
#define INTEL_IPC_EVENT_MSG_OUT (1UL << 2)

/*
 * INTEL_IPC_EVENT_CSR_ACK
 * Receive a CSR ack from peer after writing CSR
 * \ingroup intel_driver_ipc
 */
#define INTEL_IPC_EVENT_CSR_ACK (1UL << 3)

/****** IPC Capbility *****/

/*
 * struct intel_ipc_capabilities_t
 * define IPC Driver Capabilities.
 * \ingroup intel_driver_ipc
 */
struct intel_ipc_capabilities_t {
	uint32_t is_available : 1;
	uint32_t reserved : 31;
};

/****** IPC Driver helper definitions *****/

#define IPC_PROTOCOL_BOOT 0
#define IPC_PROTOCOL_HECI 1
#define IPC_PROTOCOL_MCTP 2
#define IPC_PROTOCOL_MNG 3

#define IPC_DATA_LEN_MAX (128)

#define IPC_HEADER_LENGTH_MASK (0x03FF)
#define IPC_HEADER_PROTOCOL_MASK (0x0F)
#define IPC_HEADER_LENGTH_OFFSET 0
#define IPC_HEADER_PROTOCOL_OFFSET 10
#define IPC_HEADER_GET_LENGTH(drbl_reg)                                        \
	(((drbl_reg) >> IPC_HEADER_LENGTH_OFFSET) & IPC_HEADER_LENGTH_MASK)
#define IPC_HEADER_GET_PROTOCOL(drbl_reg)                                      \
	(((drbl_reg) >> IPC_HEADER_PROTOCOL_OFFSET) & IPC_HEADER_PROTOCOL_MASK)
#define IPC_IS_BUSY(drbl_reg)                                                  \
	(((drbl_reg)&IPC_DRBL_BUSY_BIT) == ((uint32_t)IPC_DRBL_BUSY_BIT))
#define IPC_BUILD_HEADER(length, protocol, busy)                               \
	(((busy) << IPC_DRBL_BUSY_OFFS) |                                      \
	 ((protocol) << IPC_HEADER_PROTOCOL_OFFSET) |                          \
	 ((length) << IPC_HEADER_LENGTH_OFFSET))
#define IPC_SET_BUSY(drbl_reg) ((drbl_reg) | (IPC_DRBL_BUSY_BIT))

/****** IPC Driver API *****/

/*
 * ipc_event_handler I2C Event Handler Callback
 * typedef intel_i2c_event_cb_t
 * Callback function type for signal i2c event.
 * param[in] event: event type.
 * return    void
 */
typedef void (*intel_ipc_event_cb_t)(IN intel_instance_t *device,
				     IN uint32_t event, INOUT void *params);

/*
 * Get the ipc driver's API version.
 * return the version of current ipc driver's API
 */
intel_driver_version_t intel_ipc_get_version(void);

/*
 * Get the device's capabilities.
 * param[in] ipc_device: ipc device id
 * param[inout]  the capabilities of specific ipc device
 * return return status
 */
int32_t intel_ipc_get_capabilities(IN intel_instance_t *inst,
				   INOUT struct intel_ipc_capabilities_t *cap);

/*
 * Initialize the device
 * param[in] ipc_device: ipc device id
 * param[in] cb: the callback function which can receive device's events.
 * param[in] user_params: user params, will be the last input of callback
 * return  return_status
 */
int32_t intel_ipc_init(IN intel_instance_t *inst, IN intel_ipc_event_cb_t cb,
		       INOUT void *user_params);

/*
 * Uninitailize the device
 * param[in] ipc_device: ipc device id
 * return  return_status
 */
int32_t intel_ipc_uninit(IN intel_instance_t *inst);

/*
 * Write data to IPC message fifo
 * param[in] ipc_device: ipc device id
 * param[in] msg: point to memory area where data is stored
 * param[in] size: the length of data buffer
 * return  return_status
 */
int32_t intel_ipc_write_msg(IN intel_instance_t *inst, IN uint8_t *msg,
			    IN int32_t size);

/*
 * Write IPC doorbell register
 * param[in] ipc_device: ipc device id
 * param[in] doorbell: the value of doorbell
 * return  return_status
 */
int32_t intel_ipc_write_dbl(IN intel_instance_t *inst, IN uint32_t doorbell);

/*
 * Read data from IPC message fifo
 * param[in] ipc_device: ipc device id
 * param[out] msg: point to memory area where data will be stored
 * param[in] size: the length of data buffer
 * return  return_status
 */
int32_t intel_ipc_read_msg(IN intel_instance_t *inst, OUT uint8_t *msg,
			   IN int32_t size);

/*
 * Read IPC doorbell register
 * param[in] ipc_device: ipc device id
 * param[out] doorbel: point to the value of doorbell
 * return  return_status
 */
int32_t intel_ipc_read_dbl(IN intel_instance_t *inst, OUT uint32_t *doorbell);

/*
 * send ack IPC message
 * param[in] ipc_device: ipc device id
 * param[in] msg: the ack msg
 * param[in] size: the length of data buffer
 * return  return_status
 */
int32_t intel_ipc_send_ack_msg(IN intel_instance_t *inst, IN uint8_t *msg,
			       IN int32_t size);

/*
 * send ack IPC doorbell register
 * param[in] ipc_device: ipc device id
 * param[in] doorbell: the ack doorbell
 * return  return_status
 */
int32_t intel_ipc_send_ack_drbl(IN intel_instance_t *inst, IN uint32_t ack);

/*
 * read peer ack IPC message
 * param[in] ipc_device: ipc device id
 * param[out] doorbell: the buffer to store ack msg
 * param[in] size: the length of data buffer
 * return  return_status
 */
int32_t intel_ipc_read_ack_msg(IN intel_instance_t *inst, OUT uint8_t *msg,
			       IN int32_t size);

/*
 * read peer ack IPC doorbell register
 * param[in] ipc_device: ipc device id
 * param[out] doorbell: the ack doorbell
 * return  return_status
 */
int32_t intel_ipc_read_ack_drbl(IN intel_instance_t *inst, OUT uint32_t *ack);

/*
 * Set Host status to PSE FW
 * param[in] ipc_device: ipc device instance
 * param[out] doorbell: the ack doorbell
 * return  return_status
 */
int32_t intel_ipc_set_host_ready(IN intel_instance_t *inst, bool ready);

/*!
 * \enum intel_fwst_type_t
 * \brief Firmware status types
 * \ingroup intel_driver_ipc
 */
enum intel_fwst_type_t {
	ILUP_HOST,
	ILUP_SEC,
	HECI_READY,
	FAILURE_REASON, /**< Failure reason, check below for details. */
	RESET_COUNT,
	FW_STATE,      /**< Firware status, check below for details. */
	DMA_PROGRESS0, /**< DMA0 is in progress. */
	DMA_PROGRESS1,
	DMA_PROGRESS2,
	DMA_PROGRESS3,
	PWR_STATE,       /** < PSE power state, check below for details. */
	AON_CHECKPOINTS, /**< Checkpoints for AON task. */
	FWST_TYPES_LAST
};

/*!
 * \enum intel_fwst_t
 * \brief Firmware status
 * \ingroup intel_driver_ipc
 */
enum intel_fwst_t {
	AFTER_RESET = 0,
	WAITING_FOR_PATCH = 1,
	BUP_IN_PROGRESS = 2,
	LOAD_REQ_SENT = 3,
	WAIT_FOR_HOST = 4,
	START_KERNEL_DMA = 5,
	JUMP_TO_KERNEL = 6,
	FW_IS_RUNNING = 7, /**< From now on, kernel used. */
	SENSORS_APP_LOADED = 8,
	SENSORS_TABLES_LOADED = 9,
	SENSORS_APP_RUNNING = 0xf,
};

/*!
 * \enum intel_pm_fail_reason_t
 * \brief PSE PM failure reason
 * \ingroup intel_driver_ipc
 */
enum intel_pm_fail_reason_t {
	PSE_DISABLED = 0,
	AUTH_FAILURE,
	EXCEPTION_OVER_LIMIT,
	INVALID_UMA_REGS,
	PLATFORM_RESET_WAIT,
	WRONG_SEC_MSG,
	DMA_FAILURE,
	OTHER_FAILURE,
	AON_STACK_OVERFLOW = 9, /**< AON used, stack overflow. */
	NO_FAILURE = 0xe,
	FAIL_REASON_MAX = 0xf
};

/*!
 * \enum intel_pse_pwr_state_t
 * \brief PSE power state
 * \ingroup intel_driver_ipc
 */
enum intel_pse_pwr_state_t {
	PSE_D0 = 0,
	PSE_D0i0,
	PSE_D0i1,
	PSE_D0i2,
	PSE_D0i3 = 5,
	PSE_D3_WAITING = 7,
	PSE_PD,
	PSE_PWR_STATE_MAX = 0xf
};

/*!
 * \struct intel_aon_checkpoints_t
 * \brief Checkpoints for AON task
 * \ingroup intel_driver_ipc
 */
enum intel_aon_checkpoints_t {
	NOT_IN_AON = 0,
	AON_ENTRY,
	AON_HALT,
	AON_WAKE,
	AON_EXIT,
	AON_CHECKPOINT_MAX = 0x7
};

void intel_ipc_isr(IN intel_instance_t *inst);
#ifdef __cplusplus
}
#endif

#endif /* _INTEL_DRIVER_IPC_H_*/

#include "../../drivers/ipc/ipc_bsp.h"
