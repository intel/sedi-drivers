/*
 * Copyright (c) 2021 Intel Corporation
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _SEDI_DRIVER_CAN_H_
#define _SEDI_DRIVER_CAN_H_

#include <stddef.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include "sedi_driver_common.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CAN_INVALID_BUFNUM 255
#define CAN_MAX_INSTANCE 2
#define CAN_INTERRUPT_LINE_0 53
#define CAN_INTERRUPT_LINE_1 54

#define CAN_MAX_DATA_LEN 64

#define CAN_BUFFER_ESI_FLAG (0x8)
#define CAN_BUFFER_RTR_FLAG (0x4)
#define CAN_BUFFER_ANMF_FLAG (0x2)
#define CAN_BUFFER_BRS_FLAG (0x1)
#define CAN_BUFFER_MM_FLAG (0x10)
#define CAN_BUFFER_EFC_FLAG (0x20)
#define CAN_BUFFER_FDF_FLAG (0x40)

/**
 * Avoid redefinition error if included in rtos
 * RTOS may have its own define for other vendors
 *.This allows RTOS to have own definition,
 * without affect bsp functionality if this header
 * is included in RTOS drivers.
 */
#define CAN_DLC_MAX_LEN (64)
#ifndef CAN_MAX_DLC
#define CAN_MAX_DLC (0xF)
#endif
/**
 * dlc to message length mapping array
 */
static const uint8_t dlc_to_msg_len[] = { 0, 1,  2,  3,  4,  5,  6,  7,
					  8, 12, 16, 20, 24, 32, 48, 64 };

/**
 * message length to dlc mapping array
 */
static const uint8_t msg_len_to_dlc[] = {
	0,  1,  2,  3,  4,  5,  6,  7,  8,  9,  9,  9,  9,  10, 10, 10, 10,
	11, 11, 11, 11, 12, 12, 12, 12, 13, 13, 13, 13, 13, 13, 13, 13, 14,
	14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 15, 15,
	15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15
};

/**
 * @brief : Map message length into dlc(data length code)
 *
 * @param[in] len: length of message
 *
 * Map message length into dlc size.
 *
 * @return: mapped dlc value corresponding to message length
 */
static inline uint8_t sedi_can_msg_len_to_dlc(IN uint32_t len)
{
	if (len > CAN_DLC_MAX_LEN)
		return (CAN_MAX_DLC);

	return msg_len_to_dlc[len];
}

/**
 * @brief : Map message dlc into message length
 *
 * @param[in] dlc: dlc value of message
 *
 * Map dlc size into message length.
 *
 * @return: mapped message length corresponding to dlc.
 */
static inline uint8_t sedi_can_dlc_to_msg_len(IN uint32_t dlc)
{
	return dlc_to_msg_len[dlc & CAN_MAX_DLC];
}

/**
 * CAN instance Id types.
 */
enum can_id {
	CAN_0           = 0,    /**< CAN instance 0 */
	CAN_1           = 1,    /**< CAN instance 1 */
	CAN_ID_MAX      = 2,    /**< MAX CAN instance */
};

/**
 * CAN message Id types
 */
enum can_msg_id {
	CAN_STD_ID      = 0,    /**< Standard message type */
	CAN_EXT_ID      = 1,    /**< Extended message type */
};

/**
 * CAN RX FIFO Id types
 */
enum can_fifo {
	CAN_FIFO_0      = 0,    /**< RX FIFO 0 */
	CAN_FIFO_1      = 1,    /**< RX FIFO 1 */
	CAN_FIFO_INVALID        /**< Invalid FIFO Id */
};

/**
 * CAN interrupt line types
 */
enum can_intr_line {
	CAN_INT_LINE_0  = 0,    /**< CAN interrupt line 0 */
	CAN_INT_LINE_1  = 1     /**< CAN interrupt line 1 */
};

/**
 * CAN protocol states
 */
enum can_states {
	CAN_STATE_BUS_OFF               = 0,    /**< BUS OFF state */
	CAN_STATE_ERR_PASSIVE           = 1,    /**< Error passive state */
	CAN_STATE_BUS_MONITORING        = 2,    /**< BUS monitoring state */
	CAN_STATE_ERR_ACTIVE            = 3,    /**< Error active state */
	CAN_STATE_ERR_WARNING           = 4,    /**< Error warning state */
	CAN_STATE_STOP                  = 5     /**< CAN stop state */
};

/**
 * CAN bus error counters
 */
struct can_err_cnt {
	uint8_t tx_err_cnt;
	uint8_t rx_err_cnt;
};

/**
 * Loopback mode
 */
enum can_loopback_mode {
	CAN_LOOPBACK_INTERNAL   = 0,    /**< Internal loopback mode */
	CAN_LOOPBACK_EXTERNAL   = 1     /**< External loopback mode */
};

/**
 * CAN filter types
 */
enum can_filter_type {
	/**< Range filters- id1 <= accepted id <= id2 */
	CAN_FILTER_RANGE        = 0,
	/**< Dual Id filters accepted id = id1 || id2 */
	CAN_FILTER_DUAL_ID      = 1,
	/**< Classic filters id1 = filter id & id2 = mask*/
	CAN_FILTER_CLASSIC      = 2
};

/**
 * Filter operation type
 */
enum can_filter_op {
	/**< Disbale the filter */
	CAN_FILTER_OP_DISABLE           = 0,
	/**< accepted message will stored into FIFO 0 */
	CAN_FILTER_OP_FIFO0             = 1,
	/**< accepted message will stored into FIFO 1 */
	CAN_FILTER_OP_FIFO1             = 2,
	/**< reject matching id messages */
	CAN_FILTER_OP_REJECT            = 3,
	/**< matched Id message has high priority*/
	CAN_FILTER_OP_PRIO              = 4,
	/**< matched Id message has high priority and stored into FIFO 0*/
	CAN_FILTER_OP_FIFO0_PRIO        = 5,
	/**< matched Id message has high priority and stored into FIFO 1*/
	CAN_FILTER_OP_FIFO1_PRIO        = 6,
	/**< stored accpeted messaged into rx buf*/
	CAN_FILTER_OP_RXBUF             = 7
};

/**
 * FIFO operating mode
 */
enum can_fiflo_mode {
	CAN_FIFO_BLOCKING       = 0,    /**< Blocking mode */
	CAN_FIFO_OVERWRITE      = 1     /**< Overwrite mode */
};

/**
 * Baud rate for CAN FD mode transmission
 */
enum can_fd_bps {
	CAN_FD_BPS_500KBPS      = 0,    /**< 500 KBPS */
	CAN_FD_BPS_1MBPS        = 1,    /**< 1 MBPS */
	CAN_FD_BPS_2MBPS        = 2     /**< 2 MBPS */
};

/**
 * Timestamp apply mode
 */
enum can_timestamp_type {
	/**< Time stamp is always 0 */
	CAN_TIMESTAMP_ZERO      = 0,
	/**< Time stamp incremented by TCP value */
	CAN_TIMESTAMP_TCP       = 1,
	/**< External Time stamp- not supported by HW*/
	CAN_TIMESTAMP_EXT,
};

/**
 * CAN controller operation modes
 */
enum can_op_mode {
	/**< Initialization */
	CAN_MODE_INITIALIZATION         = 0,
	/**< Nomral mode according to ISO11898-1 */
	CAN_MODE_NORMAL                 = 1,
	/**< Restricted mode */
	CAN_MODE_RESTRICTED             = 2,
	/**< CAN bus monitor mode */
	CAN_MODE_MONITOR                = 3,
	/**< Internal loopback mode */
	CAN_MODE_LOOPBACK_INTERNAL      = 4,
	/**< External loopback mode */
	CAN_MODE_LOOPBACK_EXTERNAL      = 5,
	/**< CAN FD mode */
	CAN_MODE_FD                     = 6,
	/**< CAN FD with bit rate switch mode */
	CAN_MODE_FDBS                   = 7,
	/**< Clock stop mode */
	CAN_MODE_CLK_STOP_REQ           = 8,
	/**< Auto retransmission mode */
	CAN_MODE_DAR                    = 9,
	/**< Transmission pause */
	CAN_MODE_TX_PAUSE               = 10,
	/**< Loppback disable */
	CAN_MODE_LOOPBACK_DISABLE       = 11,
	/**< Restricted mode disable */
	CAN_MODE_RESTRICTED_DISABLE     = 12,
	/**< Clock stop disable */
	CAN_MODE_CLK_STOP_DISABLE       = 13,
	/**< Automatic retransmission disable */
	CAN_MODE_DAR_DISABLE            = 14,
	/**< Transmission pause disable */
	CAN_MODE_TX_PAUSE_DISABLE       = 15,
	/**< Idle */
	CAN_MODE_NONE                   = 16
};

/**
 * Protocol error type.
 */

enum can_proto_err_type {
	/**< No protocol error */
	CAN_PROTO_ERR_NONE,

	/**< Bit stuff error */
	CAN_PROTO_ERR_STUFF,

	/**< Frame format error */
	CAN_PROTO_ERR_FORMAT,

	/**< Frame Ack error */
	CAN_PROTO_ERR_ACK,

	/**< Bit1 error */
	CAN_PROTO_ERR_BIT_1,

	/**< Bit0 error */
	CAN_PROTO_ERR_BIT_0,

	/**< CRC error */
	CAN_PROTO_ERR_CRC,

	/**< Unknown error */
	CAN_PROTO_ERR_UNKNOWN
};

/**
 * User callback notification
 */
enum can_callback_evt_type {
	/**< Standard message received */
	CAN_STD_MSG_RECEIVE             = 0,
	/**< Extended message received */
	CAN_EXT_MSG_RECEIVE             = 1,
	/**< Received tx event */
	CAN_TX_EVT_OCCURRED             = 2,
	/**< Transmission canceled */
	CAN_TX_CANCELED                 = 3,
	/**< Transmission occurred */
	CAN_TX_OCCURRED                 = 4,
	/**< TX event fifo water mark reached */
	CAN_TX_EVT_WM                   = 5,
	/**< TX event fifo is full */
	CAN_TX_EVT_FULL                 = 6,
	/**< TX event lost */
	CAN_TX_EVT_LOST                 = 7,
	/**< RX fifo message lost */
	CAN_RX_FIFO_LOST                = 8,
	/**< Error occurred in accessing reserved area in message ram */
	CAN_ERR_ACCESS_TO_RESERVED      = 9,
	/**< Protocol error occurred */
	CAN_ERR_PROT_DATA               = 10,
	/**< Arbitration bit error occurred */
	CAN_ERR_PROT_ARBITRATION        = 11,
	/**< Bit error occurred */
	CAN_ERR_BIT                     = 12,
	/**< CRC error occurred */
	CAN_ERR_CRC                     = 13,
	/**< Error log overflow occurred */
	CAN_ERR_LOG_OVERFLOW            = 14,
	/**< Access to message RAM failed */
	CAN_ERR_MSG_RAM_FAILURE         = 15,
	/**< Error passive occurred */
	CAN_ERR_PASSIVE                 = 16,
	/**< Time out error occurred */
	CAN_ERR_TIMEOUT                 = 17,
	/**< Bit stuff error occurred */
	CAN_ERR_STUFF_BIT               = 18,
	/**< parity error in upper word byte occurred */
	CAN_PARITY_ERR_OCCURRED_UPPER   = 19,
	/**< parity error in lower word byte occurred */
	CAN_PARITY_ERR_OCCURRED_LOWER   = 20,
	/**< can bus off event */
	CAN_ERR_BUS_OFF                 = 21,
	/**< Error Warning Status occurred */
	CAN_ERR_WARNING                 = 22,
	/**< Can bus err ack */
	CAN_ERR_ACK                     = 23,
	/**< Can bus err format */
	CAN_ERR_FORMAT                  = 24,

};

/**
 * parity error injection mode
 */
enum can_err_inject_mode {
	CAN_PARITY_EINJ_ONE_TIME        = 0,    /* One time error injection */
	CAN_PARITY_EINJ_CONTINEOUS      = 1,    /* Contoineous error injection */
};

/**
 * Normal bit rate setting structure type
 */
struct can_bittiming_t {
	uint8_t phase_seg1;     /**< Time segment before sample */
	uint8_t phase_seg2;     /**< Time segment after sample */
	uint8_t sjw;            /**< Sync jump width */
	uint32_t brp;           /**< Baud rate pre-scaler */
};

/**
 * Fast bit rate setting structure type
 */
struct can_fast_bittiming_t {
	uint8_t fast_phase_seg1;        /**< Time segment before sample */
	uint8_t fast_phase_seg2;        /**< Time segment after sample */
	uint8_t fast_sjw;               /**< Sync jump width */
	uint32_t fast_brp;              /**< Baud rate pre-scaler */
};

/**
 * Filter configuration structure
 */
struct can_filter_t {
	/**< Filter type */
	enum can_filter_type filter_type;
	/**< Filter index */
	uint32_t filter_num;
	/**< Id1 */
	uint32_t id1;
	/**< Id2 */
	uint32_t id2;
	/**< RX buffer index incase message need to stored into rx buf,
	 * otherwise CAN_INVALID_BUFNUM
	 */
	uint32_t buf_num;
	/**< FIFO id incase message need to stored into rx fifo,
	 * otherwise CAN_FIFO_INVALID
	 */
	enum can_fifo fifo_id;
	/**< Filter operation type */
	enum can_filter_op op;
};

/**
 * can buffer info structure
 */
struct can_buf_info_t {
	/**< Set true if message need to be send through TX fifo
	 * else set false
	 */
	uint8_t isfifo;
	/**< Message marker index */
	uint16_t mm;
	/**< Buffer num if message is sent throguh TX buffer otherwise set
	 * CAN_INVALID_BUFNUM
	 */
	uint16_t buff_num;
	/**< Message id - 11 bit for std message and 29 bits for ext messages*/
	uint32_t id;
	/**< Message length */
	uint32_t length;
	/**< Message time stamp will be set on received messages*/
	uint32_t timestamp;
	/**< Message flags */
	uint32_t flags;
	/**< filter index for received messages*/
	uint32_t filter_index;
};

/**
 * CAN standard message frame structure
 */
struct can_frame_t {
	/**< Message info */
	struct can_buf_info_t info;
	/**< Message data */
	uint8_t data[CAN_MAX_DATA_LEN];
};

/**
 * Parity error configuration structure
 */
struct can_parity_err_t {
	/**< parity error injection mask */
	uint8_t einj_parity_mask;
	/**< offset */
	uint16_t einj_offset;
	/**< parity error injection mode */
	enum can_err_inject_mode einj_mode;
	/**< parity error injection data mask */
	uint32_t einj_data_mask;
};

/**
 * CAN parameters required during initialization of driver
 */
struct can_params_t {
	/**< CAN normal bit rate parameters */
	struct can_bittiming_t bit_timing;
	/**< CAN fast bit rate parameters */
	struct can_fast_bittiming_t fast_bit_timing;
	/**< Standard filter count MAX 128 for CAN 0 and 32 for CAN 1 */
	uint32_t std_filts_cnt;
	/**< Extended filter count MAX 64 for CAN 0 and 32 for CAN 1 */
	uint32_t ext_filts_cnt;
	/**< RX FIFO 0 elements count MAX 64 for CAN 0 and 32 for CAN 1 */
	uint32_t rx_fifo0_cnt;
	/**< RX FIFO 1 elements count MAX 64 for CAN 0 and 32 for CAN 1 */
	uint32_t rx_fifo1_cnt;
	/**< RX Buf elements count MAX 64 for CAN 0 and 32 for CAN 1 */
	uint32_t rx_buf_cnt;
	/**< TXEvent FIFO elements count MAX 32 for CAN 0 and 32 for CAN 1 */
	uint32_t tx_evt_fifo_cnt;
	/**< TX Buf elements count (TX Buf + TX FIFO) count
	 * MAX 32 for CAN 0 and 32 for CAN 1
	 */
	uint32_t tx_buf_cnt;
	/**< TX Buf elements count (TX Buf + TX FIFO) count
	 * MAX 32 for CAN 0 and 32 for CAN 1
	 */
	uint32_t tx_fifo_cnt;
	/**< RX FIFO 0 word size 8 bit is 1 word*/
	uint32_t rx_fifo0_word_size;
	/**< RX FIFO 1 word size 8 bit is 1 word*/
	uint32_t rx_fifo1_word_size;
	/**< RX Buf word size 8 bit is 1 word*/
	uint32_t rx_buf_word_size;
	/**< TX buf word size 8 bit is 1 word*/
	uint32_t tx_buf_word_size;
	/**< RX FIFO 0 water mark count*/
	uint32_t rx_fifo0_wm;
	/**< RX FIFO 1 water mark count*/
	uint32_t rx_fifo1_wm;
	/**< TX event FIFO water mark count*/
	uint32_t tx_evt_fifo_wm;
	/**< Enable/Disable global filter for data messages*/
	uint8_t gfc_reject;
	/**< Enable/Disable global filter for remote messages*/
	uint8_t gfc_remote_reject;
	/**< TCP value for increasing time stamp*/
	uint8_t time_counter;
	/**< Enable/Disable fd mode*/
	uint8_t fd_enable;
	/**< Baud rate for CAN FD mode*/
	enum can_fd_bps fd_bps;
	/* RX FIFO mode */
	enum can_fiflo_mode rx_fifo_mode;
	/**< Disable auto-retransmit*/
	uint8_t disable_auto_retransmit;
};

/**
 * @brief : Callback to notify driver application
 *
 * @param[in] id: Can instance id.
 * @param[in] val: callback event type
 * @param[in] data: data passed to application
 *
 * User application register this callback during init of driver and driver
 * will notify the application for all the CAN events/errors.
 *
 * @return: None
 */

typedef void (*can_callback_t)(IN enum can_id id, IN uint32_t val,
			       IN void *data);

/**
 * @brief : Initailization of CAN driver.
 * @param[in] id: Id for CAN instance.
 * @param[in] line: address of callback function for notifying application for
 * CAN events.
 *
 * Initialize CAN driver with default mode.
 *
 * @return: returns error code on failure and SEDI_DRIVER_OK on success.
 */
int32_t sedi_can_init(IN enum can_id id, IN can_callback_t cb,
		      INOUT struct can_params_t *canParams);

/**
 * @brief : Set operation mode CAN driver
 * @param[in] id: Id for CAN instance
 * @param[in] mode: mode of operation
 *
 * Set operation mode CAN driver.
 *
 * @return: None
 */
int32_t sedi_can_set_mode(IN enum can_id id, IN enum can_op_mode mode);

/**
 * @brief : Set bit rates for CAN IP.
 * @param[in] id: Id for CAN instance.
 * @param[in] bt: can_bittiming structure.
 *
 * Set bit timing settings for CAN IP.
 *
 * @return: None
 */
void sedi_can_set_bitrate(IN enum can_id id, IN struct can_bittiming_t *bt);

/**
 * @brief : Set fast bit rates for CAN IP.
 * @param[in] id: Id for CAN instance.
 * @param[in] bt: can_fast_bittiming structure.
 *
 * Set fast bit timing settings for CAN IP.
 *
 * @return: None
 */
void sedi_can_set_fast_bitrate(IN enum can_id id,
			       IN struct can_fast_bittiming_t *fbt);

/**
 * @brief : Transmit standard/extended message.
 * @param[in] id: Id for CAN instance.
 * @param[in] msg_id: Id for CAN message as CAN_STD_ID or CAN_EXT_ID.
 * @param[in] frame: message data
 *
 * Transmit extended message by configuring dedicated or fifo tx memory.
 *
 * @return: return SEDI_DRIVER_OK on success.
 */
int32_t sedi_can_send_message(IN enum can_id id, IN enum can_msg_id msgID,
			      IN void *frame);

/**
 * @brief : Receive standard/extended message from rx fifo 0/1 and rx buffer.
 * @param[in] id: Id for CAN instance
 * @param[in] fifo_id: fifo Id in case of rx buf it is set as
 * CAN_FIFO_INVALID.
 * @param[in] buf_num: buffer index for rx buf in case message to be retrieved
 * from rx fifo, it is set as CAN_INVALID_BUFNUM.
 *
 * Receive standard/extended message from rx fifo and notifies vai registered
 * callback.
 *
 * @return: return SEDI_DRIVER_OK on success and error code on failure.
 */
int32_t sedi_can_receive_message(IN enum can_id id, IN enum can_fifo fifo_id,
				 IN int buf_num);

/**
 * @brief : Configure CAN filter to recevie accepted messages into
 * rx fifo/buffer.
 * @param[in] id: Id for CAN instance
 * @param[in] filter: address of can_filter_t structure, contains informantion
 * about filter number and type of filter and mode of opertion.
 * @param[in] msg_id: Id for CAN message as CAN_STD_ID or CAN_EXT_ID.
 *
 * Configure CAN filter to recevie accepted messages into rx buffer/fifo.
 *
 * @return: return SEDI_DRIVER_OK on success.
 */
int32_t sedi_can_config_filter(IN enum can_id id,
			       IN struct can_filter_t *filter,
			       IN enum can_msg_id msg_id);

/**
 * @brief : Cancel a message transmission.
 * @param[in] id: Id for CAN instance
 * @param[in] buffer: buffer numer for which transmission to be canceled.
 *
 * Cancel a message transmission.
 *
 * @return: return SEDI_DRIVER_OK on success and error code on failure.
 */
int32_t sedi_can_cancel_tx(IN enum can_id id, IN uint16_t buffer);

/**
 * @brief : Reset CAN IP.
 * @param[in] id: Id for CAN instance
 *
 * Reset CAN IP to come out of BUS_OFF state.
 *
 * @return: return SEDI_DRIVER_OK on success and error code on failure.
 */
int32_t sedi_can_reset(IN enum can_id id);

/**
 * @brief Recover from bus-off state
 * @param[in] id: Id for CAN instance
 *
 * Recover the CAN controller from bus-off state to error-active state.
 *
 * @return: return SEDI_DRIVER_OK on success and error code on failure.
 */
int32_t sedi_can_recover(IN enum can_id id);

/**
 * @brief : Configure time stamp for tx/rx messages.
 * @param[in] id: Id for CAN instance.
 * @param[in] counter: counter by which timestamp increases.
 * @param[in] type: type of time stamping.
 *
 * Configure time stamp for tx/rx messages.
 *
 * @return: returns error code on failure and SEDI_DRIVER_OK on success.
 */
int32_t sedi_can_configure_time_stamp(IN enum can_id id, IN uint8_t counter,
				      IN enum can_timestamp_type type);

/**
 * @brief : Control CAN IP by enabling and disbaling CAN internal clock.
 * @param[in] id: Id for CAN instance
 * @param[in] enable: 1 or 0 for enabling and disabling CAN operations.
 *
 * Control CAN IP by enabling and disbaling CAN internal clock.
 *
 * @return: None
 */
void sedi_can_power_control(IN enum can_id id, IN uint8_t enable);

/**
 * @brief : Get current state for CAN protocol.
 *
 * Control  Get current value of clock freqency.
 *
 * @return: current state of CAN protocol.
 */
uint32_t sedi_can_get_status(IN enum can_id id);

/**
 * @brief Get current state
 * @param[in] id: Id for CAN instance
 * @param[in] err_cnt: Pointer to the error count structure
 *
 * Updates error count and returns the actual state of the CAN controller.
 *
 * @return: current state of CAN controller.
 */
uint32_t sedi_can_get_state_err_cnt(IN enum can_id id,
				    struct can_err_cnt *err_cnt);

/**
 * @brief : Get current value of clock freqency.
 *
 * @param[in] id: Id for CAN instance
 *
 * Get current value of clock freqency.
 *
 * @return: value of clock freqency.
 */
uint32_t sedi_can_get_clock(void);

/**
 * @brief : configure for injections of parity in message ram
 * @param[in]: id: Id for CAN instance
 * @param[in]: err: defines the mode of error and offset of message ram
 * @param[in]: enable: enable or disable parity injection
 *
 * configure for injections of parity in message ram
 *
 * @return: return SEDI_DRIVER_OK on success and error code on failure.
 */
int32_t sedi_can_inject_parity_err(IN enum can_id id,
				   IN struct can_parity_err_t *err,
				   IN uint8_t enable);

/**
 * @brief : set power state for can device
 * @param[in]: id: Id for CAN instance
 * @param[in]: state: Power management state to be set for device
 *
 * @return: return SEDI_DRIVER_OK on success and error code on failure.
 */

int32_t sedi_can_set_power(IN enum can_id id, IN sedi_power_state_t state);

#ifdef __cplusplus
}
#endif

#endif /* #ifndef SEDI_DRIVER_CAN_H_ */
