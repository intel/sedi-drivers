/*
 * Copyright (c) 2021 Intel Corporation
 *
 * SPDX-License-Identifier:  BSD-3-Clause
 */

#ifndef _INTEL_HAL_EMMC_H_
#define _INTEL_HAL_EMMC_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <intel/hal_driver_common.h>
#include <intel/hal_device.h>

/** Set the Bit Mode */
#define DISK_IOCTL_LINE_CTRL		6

/* EMMC States */
#define EMMC_GET_STATE(x)		(((x) >> 9) & 0xf)
#define EMMC_STATE_IDLE			0
#define EMMC_STATE_READY		1
#define EMMC_STATE_IDENTIFICATION	2
#define EMMC_STATE_STANDBY		3
#define EMMC_STATE_TRANSFER		4
#define EMMC_STATE_DATA			5
#define EMMC_STATE_RCV_DATA		6
#define EMMC_STATE_PROGRAMMING		7
#define EMMC_STATE_DISCONNECT		8
#define EMMC_STATE_BUS_TST		9
#define EMMC_STATE_SLEEP		10

/*!
 * EMMC CSD & CID Reg Size
 */
#define EMMC_EXTCSD_SIZE	128
#define EMMC_CSD_SIZE		4
#define EMMC_CID_SIZE		4

/*!
 * EMMC Bus Width
 */
#define EMMC_HOST_BUS_WIDTH_1_BIT	1
#define EMMC_HOST_BUS_WIDTH_4_BIT	4
#define EMMC_HOST_BUS_WIDTH_8_BIT	8

/*!
 * EMMC SW Reset
 */
typedef enum {
	EMMC_SW_RESET_DATA_LINE = 0,
	EMMC_SW_RESET_CMD_LINE,
	EMMC_SW_RESET_ALL
} emmc_host_sw_reset_t;

/*!
 * The card capacity type
 */
typedef enum {
	EMMC_HOST_LESS_2G       = 0,    /* eMMC block addressing < 2GB. */
	EMMC_HOST_GREATER_2G    = 1,    /* eMMC block addressing > 2GB. */
	EMMC_HOST_UNSUPPORTED   = 4     /* Not supported. */
} emmc_host_card_capacity_t;

/*!
 * The eMMC card info
 */
typedef enum {
	EMMC_CARD_SECTOR_COUNT = 0,
	EMMC_CARD_BLOCK_SIZE
} emmc_card_info_t;

/*!
 * \brief Get driver context
 */
#define INTEL_EMMC_INSTANCE()				       \
	((intel_instance_t *)((struct intel_emmc_context[]) {{ \
							     } }))

/*!
 * \defgroup emmc_function_calls eMMC Driver Function Calls
 * \ingroup intel_driver_emmc
 * \{
 */

/*!
 * \brief Initialize the device
 * \param[in] inst: eMMC instance pointer
 * \param[in] config: eMMC host configuration
 * \return  \ref return_status
 */
int32_t intel_emmc_init_card(IN intel_instance_t *inst);

/*!
 * \brief eMMC Read
 * \param[in] inst: eMMC instance pointer
 * \param[in] data_buff: Pointer to buffer data
 * \param[in] start_sector: Start sector
 * \param[in] num_of_blocks: Number of blocks to be read
 * \return  \ref return_status
 */
int32_t intel_emmc_read(IN intel_instance_t *inst, uint8_t *data_buff,
			uint32_t start_sector, uint32_t num_of_blocks);

/*!
 * \brief eMMC Write
 * \param[in] inst: eMMC instance pointer
 * \param[in] data_buff: Pointer to buffer data
 * \param[in] start_sector: Start sector
 * \param[in] num_of_blocks: Number of blocks to be written
 * \return  \ref return_status
 */
int32_t intel_emmc_write(IN intel_instance_t *inst, uint8_t *data_buff,
			uint32_t start_sector, uint32_t num_of_blocks);

/*!
 * \brief eMMC Get Card Information
 * \param[in] inst: eMMC instance pointer
 * \param[in] info: Type of information to be retrieved
 * \return  \ref returns card info
 */
uint32_t intel_emmc_get_card_info(IN intel_instance_t *inst,
					emmc_card_info_t info);

/*!
 * \brief eMMC Get Card Status
 * \param[in] inst: eMMC instance pointer
 * \return  \ref returns card status
 */
int32_t intel_emmc_get_card_status(IN intel_instance_t *inst);

/*!
 * \brief eMMC Erase card
 * \param[in] inst: eMMC instance pointer
 * \param[in] start_addr: starting address to be erased
 * \param[in] end_addr: ending address to be erased
 * \return  \ref return_status
 */
int32_t intel_emmc_erase(IN intel_instance_t *inst, uint32_t start_addr,
			uint32_t end_addr);
/*!
 * \brief eMMC Set Bus Width
 * \param[in] inst: eMMC instance pointer
 * \param[in] bus_width: data line width
 * \param[in] dual_data_rate: enable dual or single data rate
 * \return  \ref return_status
 */
int32_t intel_emmc_set_bus_width(IN intel_instance_t *inst, uint8_t bus_width,
                                  bool dual_data_rate);

#ifdef __cplusplus
}
#endif

#include <intel/hal_emmc_priv.h>
#endif /* _INTEL_HAL_EMMC_H_*/

