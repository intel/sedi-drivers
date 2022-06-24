/*
 * Copyright (c) 2021 Intel Corporation
 *
 * SPDX-License-Identifier:  BSD-3-Clause
 */

#ifndef _INTEL_HAL_DMA_H_
#define _INTEL_HAL_DMA_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <intel/hal_driver_common.h>
#include <intel/hal_device.h>

/*!
 * \defgroup intel_driver_dma DMA
 * \ingroup intel_driver
 */

#define INTEL_DMA_API_VERSION INTEL_DRIVER_VERSION_MAJOR_MINOR(0, 1)

#define DMA_CHANNEL_NUM 8

/*!
 * \struct intel_dma
 * \brief  DMA device bus ID
 */
typedef enum {
	INTEL_DMA_0 = 0,
	INTEL_DMA_1,
	INTEL_DMA_2,
	INTEL_DMA_NUM
} intel_dma_t;

/*!
 * \}
 */

/****** DMA Control Codes *****/

/*!
 * \defgroup dma_control_codes  DMA Control Codes
 * \ingroup intel_driver_dma
 * \{
 */

typedef enum {
	INTEL_CONFIG_DMA_DIRECTION = 0,
	INTEL_CONFIG_DMA_SR_MEM_TYPE,
	INTEL_CONFIG_DMA_DT_MEM_TYPE,
	INTEL_CONFIG_DMA_LL_SR_MSB, /* MSB bits of source addr in LL mode*/
	INTEL_CONFIG_DMA_LL_DT_MSB, /* MSB bits of destination addr in LL mode*/
	INTEL_CONFIG_DMA_BURST_LENGTH,
	INTEL_CONFIG_DMA_SR_TRANS_WIDTH,
	INTEL_CONFIG_DMA_DT_TRANS_WIDTH,
	INTEL_CONFIG_DMA_HS_DEVICE_ID,
	INTEL_CONFIG_DMA_HS_DEVICE_ID_PER_DIR,
	INTEL_CONFIG_DMA_HS_POLARITY,
	INTEL_CONFIG_DMA_CONTROL_ID_MAX
} dma_control_code;

/*!
 * DMA Burst Transfer Length
 */
typedef enum {
	DMA_BURST_TRANS_LENGTH_1 = 0x0,   /**< Burst length 1 data item. */
	DMA_BURST_TRANS_LENGTH_4 = 0x1,   /**< Burst length 4 data items. */
	DMA_BURST_TRANS_LENGTH_8 = 0x2,   /**< Burst length 8 data items. */
	DMA_BURST_TRANS_LENGTH_16 = 0x3,  /**< Burst length 16 data items. */
	DMA_BURST_TRANS_LENGTH_32 = 0x4,  /**< Burst length 32 data items. */
	DMA_BURST_TRANS_LENGTH_64 = 0x5,  /**< Burst length 64 data items. */
	DMA_BURST_TRANS_LENGTH_128 = 0x6, /**< Burst length 128 data items. */
	DMA_BURST_TRANS_LENGTH_256 = 0x7, /**< Burst length 256 data items. */
	DMA_BURST_TRANS_LENGTH_MAX
} dma_burst_length_t;

/*!
 * DMA Transfer Width
 */
typedef enum {
	DMA_TRANS_WIDTH_8 = 0x0,   /**< Transfer width of 8 bits. */
	DMA_TRANS_WIDTH_16 = 0x1,  /**< Transfer width of 16 bits. */
	DMA_TRANS_WIDTH_32 = 0x2,  /**< Transfer width of 32 bits. */
	DMA_TRANS_WIDTH_64 = 0x3,  /**< Transfer width of 64 bits. */
	DMA_TRANS_WIDTH_128 = 0x4, /**< Transfer width of 128 bits. */
	DMA_TRANS_WIDTH_256 = 0x5, /**< Transfer width of 256 bits. */
	DMA_TRANS_WIDTH_MAX
} dma_transfer_width_t;

/*!
 * DMA channel direction.
 */
typedef enum {
	DMA_MEMORY_TO_MEMORY = 0x0,     /**< Memory to memory transfer. */
	DMA_MEMORY_TO_PERIPHERAL = 0x1, /**< Memory to peripheral transfer.*/
	DMA_PERIPHERAL_TO_MEMORY = 0x2, /**< Peripheral to memory transfer. */
	DMA_PERIPHERAL_TO_PERIPHERAL = 0x3, /**< Peripheral to peripheral. */
	DMA_DIRECTION_MAX
} dma_channel_direction_t;

typedef enum {
	DMA_SRAM_MEM = 0x0,
	DMA_DRAM_MEM = 0x1,
	DMA_UMA_MEM = 0x2,
	DMA_MEM_TYPE_MAX
} dma_memory_type_t;
/*!
 * handshake polarity
 */
typedef enum {
	DMA_HS_POLARITY_HIGH = 0x0, /* handshake polarity high */
	DMA_HS_POLARITY_LOW = 0x1,  /* handshake polarity low*/
	DMA_HS_POLARITY_MAX
} dma_hs_polarity_t;

/*!
 * dma handshake rx/tx
 */
typedef enum {
	DMA_HS_PER_RX = 0x0, /* handshake peripheral rx */
	DMA_HS_PER_TX = 0x1, /* handshake peripheral tx */
	DMA_HS_PER_RTX_MAX
} dma_hs_per_rtx_t;

typedef enum { DMA_INCREMENT_INC, DMA_INCREMENT_NO_CHANGE } dma_increment_t;

/*!
 * peripheral device id for handshake
 */
typedef enum {
	DMA_HWID_I2C0 = 0xD8,
	DMA_HWID_I2C1 = 0xD9,
	DMA_HWID_I2C2 = 0xDA,
	DMA_HWID_I2C3 = 0xDB,
	DMA_HWID_I2C4 = 0xDC,
	DMA_HWID_I2C5 = 0xDD,
	DMA_HWID_I2C6 = 0xDE,
	DMA_HWID_I2C7 = 0xC0,
	DMA_HWID_UART0 = 0x88,
	DMA_HWID_UART1 = 0x89,
	DMA_HWID_UART2 = 0x8A,
	DMA_HWID_UART3 = 0x8B,
	DMA_HWID_UART4 = 0x8C,
	DMA_HWID_UART5 = 0x8D,
	DMA_HWID_SPI0 = 0x98,
	DMA_HWID_SPI1 = 0x99,
	DMA_HWID_SPI2 = 0x9A,
	DMA_HWID_SPI3 = 0x9B,
	DMA_HWID_I2S0 = 0x8E,
	DMA_HWID_I2S1 = 0x8F,
	DMA_HWID_ADC0 = 0xEF
} dma_hs_per_dev_id_t;

/* dma channel control register bit map details*/
#define LLP_SRC_EN_LOC 28
#define LLP_DST_EN_LOC 27
#define TT_FC_LOC 20
#define SRC_MSIZE_LOC 14
#define DEST_MSIZE_LOC 11
#define SINC_LOC 10
#define DINC_LOC 8
#define SRC_TR_WIDTH_LOC 4
#define DST_TR_WIDTH_LOC 1
#define INT_EN_LOC 0

#define BUILD_LL_CTRL_REG(direction, src_burst, dst_burst, src_width,          \
			  dst_width, sinc, dinc)                               \
	((direction << TT_FC_LOC) | (sinc << SINC_LOC) | (dinc << DINC_LOC) |  \
	 (src_burst << SRC_MSIZE_LOC) | (dst_burst << DEST_MSIZE_LOC) |        \
	 (src_width << SRC_TR_WIDTH_LOC) | (dst_width << DST_TR_WIDTH_LOC) |   \
	 (1 << LLP_DST_EN_LOC) | (1 << LLP_SRC_EN_LOC) | (1 << INT_EN_LOC))

#define BUILD_LL_M2M_CTRL_REG(src_burst, dst_burst, src_width, dst_width)      \
	BUILD_LL_CTRL_REG(DMA_MEMORY_TO_MEMORY, src_burst, dst_burst,          \
			  src_width, dst_width, DMA_INCREMENT_INC,             \
			  DMA_INCREMENT_INC)

#define BUILD_LL_M2P_CTRL_REG(src_burst, dst_burst, src_width, dst_width)      \
	BUILD_LL_CTRL_REG(DMA_MEMORY_TO_PERIPHERAL, src_burst, dst_burst,      \
			  src_width, dst_width, DMA_INCREMENT_INC,             \
			  DMA_INCREMENT_NO_CHANGE)

#define BUILD_LL_P2M_CTRL_REG(src_burst, dst_burst, src_width, dst_width)      \
	BUILD_LL_CTRL_REG(DMA_PERIPHERAL_TO_MEMORY, src_burst, dst_burst,      \
			  src_width, dst_width, DMA_INCREMENT_NO_CHANGE,       \
			  DMA_INCREMENT_INC)

/*!
 * \brief Get driver context
 */
#define INTEL_DMA_INSTANCE() \
	((intel_instance_t *)((struct intel_dma_context[]) {{ \
	}}))

/*!
 * list item for DMA linker list mode.
 */
typedef struct dma_linked_list_item {
	uint32_t src_addr; /**< Block source address. */
	uint32_t dst_addr; /**< Block destination address. */
	/**< Pointer to next LLI. */
	uint32_t next_ll_p;
	/**< Bottom half Ctrl register. */
	union {
		uint32_t raw;
		struct {
			uint32_t int_en : 1;
			uint32_t dst_tr_width : 3;
			uint32_t src_tr_width : 3;
			uint32_t resvd0 : 1;
			uint32_t dinc : 1;
			uint32_t rsevd1 : 1;
			uint32_t sinc : 1;
			uint32_t dest_burst_len : 3;
			uint32_t src_burst_len : 3;
			uint32_t resvd2 : 1;
			uint32_t resvd3 : 1;
			uint32_t resvd4 : 1;
			uint32_t direction : 2;
			uint32_t resvd5 : 5;
			uint32_t llp_dst_en : 1;
			uint32_t llp_src_en : 1;
			uint32_t resvd6 : 3;
		} bits;
	} ctrl_low;
	/**< Top half Ctrl register. */
	union {
		uint32_t raw;
		struct {
			uint32_t resvd0 : 15;
			uint32_t block_size : 17;
		};
	} ctrl_high;
	/* write backs for D/S STAT*/
	uint32_t dstat;
	uint32_t sstat;
} dma_linked_list_item_t;

/*!
 * \}
 */

/****** DMA Event *****/

/*!
 * \defgroup dma_event DMA Event Types
 * \ingroup intel_driver_dma
 * \{
 */

/*!
 * \def INTEL_DMA_EVENT_TRANSFER_DONE
 * \brief DMA transfer finished
 */
#define INTEL_DMA_EVENT_TRANSFER_DONE (1UL << 0)

/*!
 * \def INTEL_DMA_EVENT_TRANSFER_INCOMPLETE
 * \brief DMA incomplete transfer
 */
#define INTEL_DMA_EVENT_TRANSFER_INCOMPLETE (1UL << 1)

/*!
 * \def INTEL_DMA_EVENT_BUSY
 * \brief DMA is busy
 */
#define INTEL_DMA_EVENT_BUSY (1UL << 2)

/*!
 * \def INTEL_DMA_EVENT_BUS_ERROR
 * \brief Bus error detected
 */
#define INTEL_DMA_EVENT_BUS_ERROR (1UL << 3)

/*!
 * \}
 */

/*!
 * \struct intel_dma_status_t
 * \brief DMA Status
 * \ingroup intel_driver_dma
 */
typedef volatile struct {
	/**< Busy flag */
	uint32_t busy : 1;
	uint32_t bus_error : 1;
	uint32_t reserved : 30;
} intel_dma_status_t;

/*!
 * \struct dma_sc_attr_t
 * \brief DMA scatter gather attributions
 * \ingroup intel_driver_dma
 */
typedef struct {
	uint32_t src_addr;
	uint32_t dst_addr;
	uint32_t block_size;
	uint32_t interval;
	uint8_t is_scatter;
	uint8_t need_reload;
} sc_attr_t;

/*!
 * \defgroup dma_event_handler DMA Event Handler Callback
 * \ingroup intel_driver_dma
 * \{
 */

/*!
 * \typedef intel_dma_event_cb_t
 * \brief Callback function type for signal dma event.
 * \param[in] event: event type. see \ref dma_event
 * \param[in] dma_device: dma device id
 * \param[in] channel_id: dma channel id
 * \param[inout] param: other params
 * \return    void
 */
typedef void (*intel_dma_event_cb_t)(IN intel_instance_t *inst,
				     IN int channel_id,
				     IN int event, INOUT void *param);

/*!
 * \}
 */

/*!
 * \brief dma ISR handler. Can be registered by an OS.
 * \param[in] instance Instance of the DMA block
 */
void intel_dma_isr_handler(IN intel_instance_t *inst);


/*!
 * \defgroup dma_function_calls DMA Driver Function Calls
 * \ingroup intel_driver_dma
 * \{
 */

/*!
 * \brief Get the dma driver's API version.
 * \return the version of current dma driver's API
 */
intel_driver_version_t intel_dma_get_version(void);

/*!
 * \brief Get DMA instance pointer
 * \param id: dma peripheral id
 * \return  \ref return dma instance pointer
 */
intel_instance_t *intel_get_dma_instance(intel_dma_t id);

/*!
 * \brief Set DMA instance pointer
 * \param inst: dma instance pointer
 * \param id: dma peripheral id
 * \return
 */
void intel_dma_set_instance(intel_instance_t *inst, intel_dma_t id);

/*!
 * \brief Initialize the device
 * \param[in] dma_device: dma device id
 * \param[in] channel_id: dma channel id
 * \param[in] cb: the callback function which can receive device's events.
 * \param[inout] param: the user definded callback param, like controller point.
 * \return  \ref return_status
 */
int32_t intel_dma_init(IN intel_instance_t *inst, IN int channel_id,
		      IN intel_dma_event_cb_t cb, INOUT void *param);

/*!
 * \brief Uninitailize the device
 * \param[in] dma_device: dma device id
 * \return  \ref return_status
 */
int32_t intel_dma_uninit(IN intel_instance_t *inst, IN int channel_id);

/*!
 * \brief  Control the dma device
 * \param[in] dma_device: dma device id
 * \param[in] channel_id: dma channel id
 * \param[in] control: control operation code. see \ref dma_control_codes
 * \param[in] arg: argument of operation (optional)
 * \return  \ref return_status
 */
int32_t intel_dma_control(IN intel_instance_t *inst, IN int channel_id,
			 IN uint32_t control_id, IN uint32_t arg);

/*!
 * \brief  Get device's status
 * \param[in] dma_device: dma device id
 * \param[in] channel_id: dma channel id
 * \param[out] status: the memory pointer for dma status
 * \return  \ref return_status
 */
int intel_dma_get_status(IN intel_instance_t *inst, IN int channel_id,
			OUT intel_dma_status_t *status);

/*!
 * \brief  fill up linked-list node with given configuration
 * \param[inout] ll_p: linked list pointer to fill up
 * \param[in] src_addr: source addr
 * \param[in] dst_addr: destination addr
 * \param[in] block_size: tranfer data length
 * \param[in] ctrl_reg_low: control register lower 32-bitcontent
 * \param[in] ll_p_next: the pointer to next node, set NULL for the lastest one
 * \return \ref return_status
 */
int dma_fill_linkedlist(INOUT volatile dma_linked_list_item_t *ll_p,
			IN uint32_t src_addr, IN uint32_t dst_addr,
			IN uint32_t block_size, IN uint32_t ctrl_reg_low,
			IN uint32_t ll_p_next);

/*!
 * \brief  fill up linked-list node with given scatter gather configuration
 * \param[inout] ll_p: linked list pointer to fill up
 * \param[in] count: linked-list node count
 * \param[in] ctrl_reg_low: control register lower 32-bitcontent
 * \param[in] attr: scatter gather attributions
 * \return \ref return_status
 */
int dma_fill_sc_linkedlist(INOUT volatile dma_linked_list_item_t *llp,
			   IN uint8_t count, IN uint32_t ctrl_reg_low,
			   IN sc_attr_t *attr);

/*!
 * \brief  tranfer data with dma in linked-list mode
 * \param[in] dma_device: dma device id
 * \param[in] channel_id: dma channel id
 * \param[in] linkedlist_header: linked list header indicating the transferring
 * \return \ref return_status
 */
int32_t
intel_dma_start_ll_transfer(IN intel_instance_t *inst, IN int channel_id,
			   IN dma_linked_list_item_t *linkedlist_header);

/*!
 * \brief  tranfer data with dma
 * \param[in] dma_device: dma device id
 * \param[in] channel_id: dma channel id
 * \param[in] sr_addr: source addr
 * \param[in] dest_addr: destination addr
 * \param[in] length: tranfer data length
 * \return \ref return_status
 */
int32_t intel_dma_start_transfer(IN intel_instance_t *inst, IN int channel_id,
				IN uint64_t sr_addr, IN uint64_t dest_addr,
				IN uint32_t length);

/*!
 * \brief  tranfer data with dma in pulling mode
 * \param[in] dma_device: dma device id
 * \param[in] channel_id: dma channel id
 * \param[in] sr_addr: source addr
 * \param[in] dest_addr: destination addr
 * \param[in] length: tranfer data length
 * \return \ref return_status
 */
int32_t intel_dma_start_transfer_polling(IN intel_instance_t *inst,
					IN int channel_id, IN uint64_t sr_addr,
					IN uint64_t dest_addr,
					IN uint32_t length);
/*!
 * \brief  abort dma tranfer in process
 * \param[in] dma_device: dma device id
 * \param[in] channel_id: dma channel id
 * \return \ref return_status
 */
int32_t intel_dma_abort_transfer(IN intel_instance_t *inst, IN int channel_id);

int32_t intel_dma_get_done_status(IN intel_instance_t *inst, IN int channel_id,
	OUT uint32_t *done_bytes, OUT dma_linked_list_item_t **next_llp);
/*!
 * \}
 */

#ifdef __cplusplus
}
#endif

#include <intel/hal_dma_priv.h>
#endif /* _INTEL_HAL_DMA_H_*/
