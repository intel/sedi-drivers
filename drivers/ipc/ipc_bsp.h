/*
 * Copyright (c) 2021 Intel Corporation
 *
 * SPDX-License-Identifier:  BSD-3-Clause
 */

#ifndef _IPC_BSP_H_
#define _IPC_BSP_H_
#include <intel/hal_driver_common.h>
#include <intel/hal_device.h>
#include <intel/hal_ipc.h>

#define SEDI_IPC_API_VERSION 0
/* driver version */
#define SEDI_IPC_DRIVER_VERSION SEDI_DRIVER_VERSION_MAJOR_MINOR(0, 1)

#define HOST_COMM_REG_OFFSET 0x0E
#define HOST_RDY_BIT 7
#define SET_HOST_UP(base)                                                      \
	(*((uint32_t *)base + HOST_COMM_REG_OFFSET) |= BIT(HOST_RDY_BIT))
#define CLEAR_HOST_UP(base)                                                    \
	(*((uint32_t *)base + HOST_COMM_REG_OFFSET) ^= BIT(HOST_RDY_BIT))

/* IPC register description */
struct ipc_reg_t {
	__IO_RW uint32_t pisr_in_2pse;    /**<PISR_*2OSE */
	__IO_RW uint32_t pimr_in_2pse;    /**<PIMR_*2OSE */
	__IO_RW uint32_t pimr_out_2host;  /**<PIMR_*OSE2 */
	__IO_RW uint32_t pisr_out_2host;  /**<PISR_*OSE2 */
	__IO_RW uint32_t channel_intr_en; /**<CIM_PSE */
	__IO_RW uint32_t channel_intr_st; /**<CIS_PSE */
	__IO_R uint32_t reserved0[(0x48 - 0x18) >> 2];
	__IO_RW uint32_t drbl_in_2pse; /**< *2OSE DOORBELL */
	__IO_R uint32_t reserved1[2];
	__IO_RW uint32_t drbl_out_2host; /**< OSE2* DOORBELL */
	__IO_R uint32_t reserved2[2];
	__IO_RW uint8_t msgs_out[IPC_DATA_LEN_MAX]; /**< OSE2* MSG */
	__IO_RW uint8_t msgs_in[IPC_DATA_LEN_MAX];  /**< *2OSE MSG */
	__IO_R uint32_t reserved3[(0x378 - 0x160) >> 2];
};

/*ipc runtime context information */
struct ipc_context_t {
	intel_instance_t inst;
	uint32_t id;
	struct intel_ipc_capabilities_t driver_capabilities;
	bool initialized;
	intel_ipc_event_cb_t cb_event; /*event callback*/
	uint32_t in_msg_count;	 /* for debug usage*/
	uint32_t out_msg_count;	/* for debug usage*/
	void *usr_params;	      /* user parameters for event callback*/
				       /*other private runtime data*/
};

#define IPC_OSE2PEER_FWSTS 0x34
#define IPC_D0I3C_REG 0x6D0
#define IPC_D0I3C_INT_BIT 0
#define IPC_D0I3C_STATUS_BIT 2

#define IPC_OWNERSHIP_REG_OFFSET 0x900
#define IPC_OWNERSHIP_BIT_FILED_LEN 4
#define IPC_OWNERSHIP_INTR_MASK (1 << (IPC_OWNERSHIP_BIT_FILED_LEN - 1))
#define IPC_OWNERSHIP_OWNER_MASK (IPC_OWNERSHIP_INTR_MASK - 1)
#define IPC_OWNERSHIP_DEV_OFF 1
#define IPC_OWNERSHIP_DEV_MSK 1
#define IPC_DEV_OWNED_BY_PSE_BITS 0x0
#define IPC_DEV_OWNED_BY_HOST_BITS 0x1
#define IPC_HOST_DEV_MSI_INT 0x0
#define IPC_HOST_DEV_SB_INT 0x1

#define IPC_BUSY_BIT 31
#define IPC_INT_STAT_BIT 0
#define IPC_INT_MASK_IN_BIT 0
#define IPC_INT_MASK_BC_BIT 8
#define IPC_INT_STAT_BC_BIT 8
#define IPC_INT_MASK_OUT_BIT 0

#define IPC_REG_SB_OSE2PMC_DRBL 0x1804
#define IPC_REG_SB_OSE2PMC_MSG 0x1808
#define IPC_REG_SB_PMC2OSE_DRBL_MIRROR 0x1888

#define SINGLE_BIT 1

#define IPC_INT_MASK 0
#define IPC_INT_UNMASK 1
#endif
