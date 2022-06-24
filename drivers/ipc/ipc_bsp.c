/*
 * Copyright (c) 2021 Intel Corporation
 *
 * SPDX-License-Identifier:  BSD-3-Clause
 */

#include "ipc_bsp.h"

#define INTEL_SIDEBAND_PMC INTEL_SIDEBAND_0

#define IPC_REG(ctx) ((struct ipc_reg_t *)((ctx)->inst.base_addr))

/*driver version*/
static const intel_driver_version_t driver_version;

static int32_t check_ipc_available(IN intel_instance_t *inst)
{
	struct ipc_context_t *ipc_contexts =
	    CONTAINER_OF(inst, struct ipc_context_t, inst);

	if (!ipc_contexts->initialized) {
		return INTEL_DRIVER_ERROR;
	}
	return INTEL_DRIVER_OK;
}

/********** IPC driver API implementation ***********/

intel_driver_version_t intel_ipc_get_version(void)
{
	return driver_version;
}

int32_t intel_ipc_get_capabilities(IN intel_instance_t *inst,
				   INOUT struct intel_ipc_capabilities_t *cap)
{
	DBG_CHECK(inst, INTEL_DRIVER_ERROR_PARAMETER);
	struct ipc_context_t *ipc_contexts =
	    CONTAINER_OF(inst, struct ipc_context_t, inst);
	/* IPC is always available to PSE*/
	ipc_contexts->driver_capabilities.is_available = 1;
	*cap = ipc_contexts->driver_capabilities;
	return INTEL_DRIVER_OK;
}
int32_t intel_ipc_init(IN intel_instance_t *inst, IN intel_ipc_event_cb_t cb,
		       INOUT void *param)
{
	DBG_CHECK(inst, INTEL_DRIVER_ERROR_PARAMETER);
	struct ipc_context_t *ipc_contexts =
	    CONTAINER_OF(inst, struct ipc_context_t, inst);

	volatile struct ipc_reg_t *regs = IPC_REG(ipc_contexts);

	ipc_contexts->initialized = false;
	ipc_contexts->cb_event = cb;
	ipc_contexts->usr_params = param;
	ipc_contexts->in_msg_count = 0;
	ipc_contexts->out_msg_count = 0;

	if ((regs->drbl_in_2pse & BIT(IPC_BUSY_BIT)) != 0) {
		regs->drbl_in_2pse = 0;
	}
	ipc_contexts->initialized = true;

	if (regs->pisr_out_2host) {
		/*set 1 to clear this interrupt*/
		regs->pisr_out_2host |= 1 << IPC_INT_MASK_BC_BIT;
	}
	/* Enable Msg in, busyClean and out Interrupt */
	regs->pimr_out_2host |=
	    BIT(IPC_INT_MASK_IN_BIT) | BIT(IPC_INT_MASK_BC_BIT);

	return INTEL_DRIVER_OK;
}

int32_t intel_ipc_uninit(IN intel_instance_t *inst)
{
	DBG_CHECK(inst, INTEL_DRIVER_ERROR_PARAMETER);
	struct ipc_context_t *ipc_contexts =
	    CONTAINER_OF(inst, struct ipc_context_t, inst);
	volatile struct ipc_reg_t *regs = IPC_REG(ipc_contexts);

	/* disable Msg in, busyClean and out Interrupt  */
	regs->pimr_out_2host &=
	    ~(BIT(IPC_INT_MASK_IN_BIT) | BIT(IPC_INT_MASK_BC_BIT));

	ipc_contexts->cb_event = NULL;
	ipc_contexts->initialized = false;

	return INTEL_DRIVER_OK;
}

int32_t intel_ipc_write_msg(IN intel_instance_t *inst, IN uint8_t *msg,
			    IN int32_t size)
{
	int ret, i;

	DBG_CHECK(inst, INTEL_DRIVER_ERROR_PARAMETER);
	struct ipc_context_t *ipc_contexts =
	    CONTAINER_OF(inst, struct ipc_context_t, inst);
	volatile struct ipc_reg_t *regs = IPC_REG(ipc_contexts);

	ret = check_ipc_available(inst);
	DBG_CHECK(ret == INTEL_DRIVER_OK, ret);

	DBG_CHECK((size <= IPC_DATA_LEN_MAX) && (size >= 0),
		  INTEL_DRIVER_ERROR_PARAMETER);
	DBG_CHECK(msg != NULL, INTEL_DRIVER_ERROR_PARAMETER);

	/* write data in 32-bit*/
	for (i = 0; i < (size >> 2); i++) {
		*((uint32_t *)regs->msgs_in + i) = *((uint32_t *)msg + i);
	}
	/* write data in 8-bit for the rest*/
	for (i = ((size >> 2) << 2); i < size; i++) {
		regs->msgs_in[i] = msg[i];
	}

	return INTEL_DRIVER_OK;
}

int32_t intel_ipc_write_dbl(IN intel_instance_t *inst, IN uint32_t doorbell)
{
	DBG_CHECK(inst, INTEL_DRIVER_ERROR_PARAMETER);
	int ret;
	struct ipc_context_t *ipc_contexts =
	    CONTAINER_OF(inst, struct ipc_context_t, inst);
	volatile struct ipc_reg_t *regs = IPC_REG(ipc_contexts);

	ret = check_ipc_available(inst);
	DBG_CHECK(ret == INTEL_DRIVER_OK, ret);

	regs->drbl_in_2pse = doorbell;
	if (!(doorbell & BIT(IPC_BUSY_BIT))) {
	}

	return INTEL_DRIVER_OK;
}

int32_t intel_ipc_read_msg(IN intel_instance_t *inst, OUT uint8_t *msg,
			   IN int32_t size)
{
	int ret, i;

	DBG_CHECK(inst, INTEL_DRIVER_ERROR_PARAMETER);
	struct ipc_context_t *ipc_contexts =
	    CONTAINER_OF(inst, struct ipc_context_t, inst);
	volatile struct ipc_reg_t *regs = IPC_REG(ipc_contexts);

	ret = check_ipc_available(inst);
	DBG_CHECK(ret == INTEL_DRIVER_OK, ret);

	DBG_CHECK((size <= IPC_DATA_LEN_MAX) && (size >= 0),
		  INTEL_DRIVER_ERROR_PARAMETER);
	DBG_CHECK(msg != NULL, INTEL_DRIVER_ERROR_PARAMETER);

	/* read data in 32-bit*/
	for (i = 0; i < (size >> 2); i++) {
		*((uint32_t *)msg + i) = *((uint32_t *)regs->msgs_out + i);
	}
	/* read data in 8-bit for the rest*/
	for (i = ((size >> 2) << 2); i < size; i++) {
		msg[i] = regs->msgs_out[i];
	}
	return INTEL_DRIVER_OK;
}

int32_t intel_ipc_read_dbl(IN intel_instance_t *inst, OUT uint32_t *doorbell)
{
	int ret;

	DBG_CHECK(inst, INTEL_DRIVER_ERROR_PARAMETER);
	struct ipc_context_t *ipc_contexts =
	    CONTAINER_OF(inst, struct ipc_context_t, inst);
	volatile struct ipc_reg_t *regs = IPC_REG(ipc_contexts);

	ret = check_ipc_available(inst);
	DBG_CHECK(ret == INTEL_DRIVER_OK, ret);

	*doorbell = regs->drbl_out_2host;
	return INTEL_DRIVER_OK;
}

int32_t intel_ipc_send_ack_drbl(IN intel_instance_t *inst, IN uint32_t ack)
{
	int ret;

	DBG_CHECK(inst, INTEL_DRIVER_ERROR_PARAMETER);
	struct ipc_context_t *ipc_contexts =
	    CONTAINER_OF(inst, struct ipc_context_t, inst);
	volatile struct ipc_reg_t *regs = IPC_REG(ipc_contexts);

	ret = check_ipc_available(inst);
	DBG_CHECK(ret == INTEL_DRIVER_OK, ret);

	if (ack & BIT(IPC_BUSY_BIT)) {
		return INTEL_DRIVER_ERROR_PARAMETER;
	}

	regs->drbl_out_2host = ack;

	regs->pimr_out_2host |= BIT(IPC_INT_MASK_IN_BIT);
	return INTEL_DRIVER_OK;
}

int32_t intel_ipc_read_ack_drbl(IN intel_instance_t *inst, OUT uint32_t *ack)
{
	int ret;

	DBG_CHECK(inst, INTEL_DRIVER_ERROR_PARAMETER);
	struct ipc_context_t *ipc_contexts =
	    CONTAINER_OF(inst, struct ipc_context_t, inst);
	volatile struct ipc_reg_t *regs = IPC_REG(ipc_contexts);

	ret = check_ipc_available(inst);
	DBG_CHECK(ret == INTEL_DRIVER_OK, ret);
	DBG_CHECK(ack != NULL, ret);

	*ack = regs->drbl_in_2pse;

	return INTEL_DRIVER_OK;
}

int32_t intel_ipc_send_ack_msg(IN intel_instance_t *inst, IN uint8_t *msg,
			       IN int32_t size)
{
	int ret, i;

	DBG_CHECK(inst, INTEL_DRIVER_ERROR_PARAMETER);
	struct ipc_context_t *ipc_contexts =
	    CONTAINER_OF(inst, struct ipc_context_t, inst);
	volatile struct ipc_reg_t *regs = IPC_REG(ipc_contexts);

	ret = check_ipc_available(inst);
	DBG_CHECK(ret == INTEL_DRIVER_OK, ret);

	DBG_CHECK((size <= IPC_DATA_LEN_MAX) && (size >= 0),
		  INTEL_DRIVER_ERROR_PARAMETER);
	DBG_CHECK(msg != NULL, INTEL_DRIVER_ERROR_PARAMETER);

	/* write data in 32-bit*/
	for (i = 0; i < (size >> 2); i++) {
		*((uint32_t *)regs->msgs_in + i) = *((uint32_t *)msg + i);
	}
	/* write data in 8-bit for the rest*/
	for (i = ((size >> 2) << 2); i < size; i++) {
		regs->msgs_in[i] = msg[i];
	}
	return INTEL_DRIVER_OK;
}

int32_t intel_ipc_read_ack_msg(IN intel_instance_t *inst, OUT uint8_t *msg,
			       IN int32_t size)
{
	int ret, i;

	DBG_CHECK(inst, INTEL_DRIVER_ERROR_PARAMETER);
	struct ipc_context_t *ipc_contexts =
	    CONTAINER_OF(inst, struct ipc_context_t, inst);
	volatile struct ipc_reg_t *regs = IPC_REG(ipc_contexts);

	ret = check_ipc_available(inst);
	DBG_CHECK(ret == INTEL_DRIVER_OK, ret);

	DBG_CHECK((size <= IPC_DATA_LEN_MAX) && (size >= 0),
		  INTEL_DRIVER_ERROR_PARAMETER);
	DBG_CHECK(msg != NULL, INTEL_DRIVER_ERROR_PARAMETER);

	/* read data in 32-bit*/
	for (i = 0; i < (size >> 2); i++) {
		*((uint32_t *)msg + i) = *((uint32_t *)regs->msgs_out + i);
	}
	/* read data in 8-bit for the rest*/
	for (i = ((size >> 2) << 2); i < size; i++) {
		msg[i] = regs->msgs_out[i];
	}

	return INTEL_DRIVER_OK;
}
/* IPC ISR process function */

void intel_ipc_isr(IN intel_instance_t *inst)
{
	DBG_CHECK(inst, INTEL_DRIVER_ERROR_PARAMETER);
	int ret;
	int retry_cnt = 2;
	struct ipc_context_t *ipc_contexts =
	    CONTAINER_OF(inst, struct ipc_context_t, inst);
	volatile struct ipc_reg_t *regs = IPC_REG(ipc_contexts);

	ret = check_ipc_available(inst);
	if (ret != INTEL_DRIVER_OK) {
		return;
	}

	do {
		if ((regs->pisr_out_2host & BIT(IPC_INT_STAT_BIT)) &&
		    (regs->pimr_out_2host & BIT(IPC_INT_MASK_IN_BIT))) {
			regs->pimr_out_2host &= ~BIT(IPC_INT_MASK_IN_BIT);

			if (ipc_contexts->cb_event) {
				ipc_contexts->in_msg_count++;
				ipc_contexts->cb_event(
				    inst, INTEL_IPC_EVENT_MSG_IN,
				    ipc_contexts->usr_params);
			}
		}

		if (regs->pisr_out_2host & BIT(IPC_INT_STAT_BC_BIT)) {
			/*set 1 to clear this interrupt*/
			regs->pisr_out_2host |= 1 << IPC_INT_MASK_BC_BIT;
			if ((ipc_contexts->initialized == true) &&
			    (ipc_contexts->cb_event)) {
				ipc_contexts->out_msg_count++;
				ipc_contexts->cb_event(
				    inst, INTEL_IPC_EVENT_MSG_PEER_ACKED,
				    ipc_contexts->usr_params);
			}
		}

	} while (regs->pisr_out_2host && retry_cnt--);
}

int32_t intel_ipc_set_host_ready(IN intel_instance_t *inst, bool ready)
{
	DBG_CHECK(inst, INTEL_DRIVER_ERROR_PARAMETER);
	struct ipc_context_t *ipc_contexts =
	    CONTAINER_OF(inst, struct ipc_context_t, inst);
	volatile struct ipc_reg_t *regs = IPC_REG(ipc_contexts);

	if (ready) {
		SET_HOST_UP(regs);
	} else {
		CLEAR_HOST_UP(regs);
	}

	return 0;
}
