/*
 * Copyright (c) 2021 Intel Corporation
 *
 * SPDX-License-Identifier:  BSD-3-Clause
 */

#ifndef _INTEL_HAL_DMA_PRIV_H_
#define _INTEL_HAL_DMA_PRIV_H_

/* channel config*/
typedef struct {
	uint8_t config_applied;
	uint8_t tf_mode;
	uint8_t sr_mem_type;
	uint8_t dt_mem_type;
	uint8_t sr_msb;
	uint8_t dt_msb;
	uint8_t burst_length;
	uint8_t sr_width;
	uint8_t dt_width;
	uint8_t direction;
	uint8_t handshake_polarity;
	uint8_t peripheral_direction;
	uint16_t handshake_device_id;
	uint32_t linked_list_header;
} channel_config_t;

/* DMA context data structure */
struct intel_dma_context {

	intel_instance_t instance;

	__IO_RW void *regs;
	channel_config_t channel_config[DMA_CHANNEL_NUM];
	intel_dma_event_cb_t cb_event[DMA_CHANNEL_NUM]; /*event callback*/
	void *cb_param[DMA_CHANNEL_NUM];                /*event callback*/
	intel_dma_status_t status[DMA_CHANNEL_NUM];     /*status flags*/
	uint32_t done_byte[DMA_CHANNEL_NUM];            /*the transferred byte*/
	uint32_t next_llp[DMA_CHANNEL_NUM];
	uint32_t flags[DMA_CHANNEL_NUM];                /*control and state flags*/
	uint8_t vnn_status;
	uint8_t power_status;
	/*other private runtime data*/
};

#endif /* _INTEL_HAL_DMA_PRIV_H_ */
