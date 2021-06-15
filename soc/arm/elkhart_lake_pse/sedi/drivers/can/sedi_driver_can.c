/*
 * Copyright (c) 2021 Intel Corporation
 *
 * SPDX-License-Identifier:  BSD-3-Clause
 */

#include "can_bsp.h"
#include "pm/pm_internal_if.h"
#include "sedi_driver_ipc.h"

static void sedi_can_set_interrupts(IN enum can_id id,
				    IN enum can_intr_line line);

static int32_t sedi_can_msg_ram_init(IN enum can_id id,
				     INOUT struct can_config_t *mcanConfig);

static void sedi_can_set_global_filter(IN struct can_config_t *mcanConfig);

void sedi_can_isr(IN enum can_id id);

static void sedi_can_handle_error(IN enum can_id id, IN uint32_t val);

static void sedi_can_handle_tx_event(IN enum can_id id, IN uint32_t val);

static void sedi_can_handle_state_change(IN enum can_id id,
					 IN enum can_states state);

static uint8_t sedi_can_send_std_message(IN enum can_id id, IN void *frame);

static uint8_t sedi_can_send_ext_message(IN enum can_id id, IN void *frame);

static __IO_RW uint32_t *sedi_can_receive_from_fifo(IN enum can_id,
						    IN enum can_fifo fifo_id);

static void sedi_can_config_rx_buf_filter(IN enum can_id id,
					  IN struct can_filter_t *filter,
					  IN enum can_msg_id msg_id);

static void sedi_can_config_std_filter(IN enum can_id id,
				       IN struct can_filter_t *filter);

static void sedi_can_config_ext_filter(IN enum can_id id,
				       IN struct can_filter_t *filter);

static int sedi_can_get_proto_err(uint32_t psr);

/* CAN configuration structure  for both CAN instances */

static struct can_msg_ram_t msg_ram[CAN_MAX_INSTANCE] = {
	{
		.std_filters = (uint32_t *)CAN0_MSG_RAM_START,
	},
	{
		.std_filters = (uint32_t *)CAN1_MSG_RAM_START,
	}
};
static struct can_config_t can_config[CAN_MAX_INSTANCE] = {
	{
		/* CAN 0 configuration */
		(struct can_regs_t *)CAN0_REG_START,
		(struct parity_stat_regs_t *)PARITY0_STAT_REG_START,
		(struct parity_ctrl_regs_t *)PARITY0_CTRL_REG_START, NULL, &msg_ram[CAN_0],
		CAN_STATE_STOP, CAN_MODE_NONE, NULL,
	},
	{
		/* CAN 1 configuration */
		(struct can_regs_t *)CAN1_REG_START,
		(struct parity_stat_regs_t *)PARITY1_STAT_REG_START,
		(struct parity_ctrl_regs_t *)PARITY1_CTRL_REG_START, NULL, &msg_ram[CAN_1],
		CAN_STATE_STOP, CAN_MODE_NONE, NULL,
	}
};

static can_pm_regs_t can_pm_context[CAN_MAX_INSTANCE];

/**
 * @brief : Convert element size into CAN HW specific size
 *
 * @param[in] size: Size of element
 *
 * Convert element size into CAN HW specific size
 *
 * @return: converted size of element
 */

static inline int8_t sedi_can_element_size(INOUT uint8_t size)
{
	int8_t ret;

	if (size > 0 && size < 8) {
		size = 8;
	}
	switch (size) {
	case 8:
		ret = 0;
		break;
	case 12:
		ret = 1;
		break;
	case 16:
		ret = 2;
		break;
	case 20:
		ret = 3;
		break;
	case 24:
		ret = 4;
		break;
	case 32:
		ret = 5;
		break;
	case 48:
		ret = 6;
		break;
	case 64:
		ret = 7;
		break;
	default:
		ret = -1;
	}
	return ret;
}

/**
 * @brief : Configure CAN IP for programming
 *
 * @param[inout] can_regs: pointer to CAN IP registers
 *
 * Configure CAN IP for programming
 *
 * @return: SEDI_DRIVER_OK or SEDI_DRIVER_ERROR_TIMEOUT in case of error.
 */
static inline int32_t
sedi_can_config_init_start(INOUT struct can_regs_t *can_regs)
{
	int32_t ret = SEDI_DRIVER_OK;
	uint32_t count = TIMER_LIMIT;

	can_regs->cccr |= CAN_CCCR_CCE_CONFIGURABLE | CAN_CCCR_INIT_ENABLED;
	do {
		count--;
	} while (!(can_regs->cccr & CAN_CCCR_INIT_ENABLED) && count);

	if (!count) {
		ret = SEDI_DRIVER_ERROR_TIMEOUT;
	}
	return ret;
}
/**
 * @brief : Configure CAN IP for operation
 *
 * @param[inout] can_regs: pointer to CAN IP registers
 *
 * Configure CAN IP for operation
 *
 * @return: SEDI_DRIVER_OK or SEDI_DRIVER_ERROR_TIMEOUT in case of error.
 */
static inline int32_t
sedi_can_config_init_finished(INOUT struct can_regs_t *can_regs)
{
	int32_t ret = SEDI_DRIVER_OK;
	uint32_t count = TIMER_LIMIT, reg_val;

	reg_val = can_regs->cccr;
	can_regs->cccr = reg_val & ~(CAN_CCCR_INIT_ENABLED);
	do {
		count--;
	} while ((can_regs->cccr & CAN_CCCR_INIT_ENABLED) && count);
	if (!count) {
		ret = SEDI_DRIVER_ERROR_TIMEOUT;
	}
	return ret;
}

/**
 * @brief : Internal function to configure interrupts for CAN IP.
 * @param[in] id: Id for CAN instance.
 * @param[in] line: Interrupt line for receiving CAN interrupts.
 *
 * Set interrupt for RX and TX operations as well as water mark reached
 * interrupts.
 *
 * @return: None
 */
static void sedi_can_set_interrupts(IN enum can_id id,
				    IN enum can_intr_line line)
{

	uint32_t val;
	struct can_config_t *mcan_config = &can_config[id];
	struct can_params_t *params = mcan_config->params;
	struct parity_stat_regs_t *parity_stat;
	struct can_regs_t *can_regs = mcan_config->regs;

	parity_stat = mcan_config->parity_stat_regs;

	if (line == CAN_INT_LINE_0) {
		can_regs->ile = CAN_ILE_EINT0;
		can_regs->ils = CAN_SELECT_INTR_LINE_0;
	} else {
		can_regs->ile = CAN_ILE_EINT1;
		can_regs->ils = CAN_SELECT_INTR_LINE_1;
	}
	/* Disable timeout, time stamp overwrite and tx fifo empty interrupts */
	val = CAN_IE_ALL & ~(CAN_IE_TOOE | CAN_IE_TSWE | CAN_IE_TFEE);

	if (!(params->rx_fifo0_wm)) {
		val &= ~CAN_IE_RF0WE;
	}

	if (!(params->rx_fifo1_wm)) {
		val &= ~CAN_IE_RF1WE;
	}

	if (!(params->tx_evt_fifo_wm)) {
		val &= ~CAN_IE_TEFWE;
	}

	can_regs->ie = val;

	/* Parity interrupt enable */
	parity_stat->int_ctl = PARITY_INT_CTL_PRIMARY_INTR_ENABLE;
}

/**
 * @brief : Check input param values against max supported by HW
 * @param[in] id: Id for CAN instance.
 * @param[in] params: pointer to struct can_params_t
 *
 * Check input param values against max supported by HW.
 *
 * @return: return SEDI_DRIVER_OK on success, error code in case error.
 */
static int32_t sedi_can_param_max_limit(IN enum can_id id,
					IN struct can_params_t *params)
{
	int32_t ret = SEDI_DRIVER_OK;

	DBG_CHECK((CAN_0 == id || CAN_1 == id), SEDI_DRIVER_ERROR_PARAMETER);
	if (id == CAN_0) {
		if (params->std_filts_cnt > CAN0_MAX_STD_FILTER_CNT) {
			ret = SEDI_DRIVER_ERROR_PARAMETER;
			goto err;
		}

		if (params->ext_filts_cnt > CAN0_MAX_XTD_FILTER_CNT) {
			ret = SEDI_DRIVER_ERROR_PARAMETER;
			goto err;
		}
		if (params->rx_fifo0_cnt > CAN0_MAX_RX_FIFO0_CNT) {
			ret = SEDI_DRIVER_ERROR_PARAMETER;
			goto err;
		}
		if (params->rx_fifo1_cnt > CAN0_MAX_RX_FIFO1_CNT) {
			ret = SEDI_DRIVER_ERROR_PARAMETER;
			goto err;
		}
		if (params->rx_buf_cnt > CAN0_MAX_RX_BUF_CNT) {
			ret = SEDI_DRIVER_ERROR_PARAMETER;
			goto err;
		}
		if (params->tx_evt_fifo_cnt > CAN0_MAX_TX_EVTFIFO_CNT) {
			ret = SEDI_DRIVER_ERROR_PARAMETER;
			goto err;
		}
		if ((params->tx_buf_cnt + params->tx_fifo_cnt) >
		    CAN0_MAX_TX_BUF_CNT) {
			ret = SEDI_DRIVER_ERROR_PARAMETER;
			goto err;
		}
	} else if (id == CAN_1) {
		if (params->std_filts_cnt > CAN1_MAX_STD_FILTER_CNT) {
			ret = SEDI_DRIVER_ERROR_PARAMETER;
			goto err;
		}

		if (params->ext_filts_cnt > CAN1_MAX_XTD_FILTER_CNT) {
			ret = SEDI_DRIVER_ERROR_PARAMETER;
			goto err;
		}
		if (params->rx_fifo0_cnt > CAN1_MAX_RX_FIFO0_CNT) {
			ret = SEDI_DRIVER_ERROR_PARAMETER;
			goto err;
		}
		if (params->rx_fifo1_cnt > CAN1_MAX_RX_FIFO1_CNT) {
			ret = SEDI_DRIVER_ERROR_PARAMETER;
			goto err;
		}
		if (params->rx_buf_cnt > CAN1_MAX_RX_BUF_CNT) {
			ret = SEDI_DRIVER_ERROR_PARAMETER;
			goto err;
		}
		if (params->tx_evt_fifo_cnt > CAN1_MAX_TX_EVTFIFO_CNT) {
			ret = SEDI_DRIVER_ERROR_PARAMETER;
			goto err;
		}
		if ((params->tx_buf_cnt + params->tx_fifo_cnt) >
		    CAN1_MAX_TX_BUF_CNT) {
			ret = SEDI_DRIVER_ERROR_PARAMETER;
			goto err;
		}
	}
err:
	return ret;
}
/**
 * @brief : Initialize message RAM for CAN operations.
 * @param[in] id: CAN instance id.
 * @param[inout] mcan_config: CAN configuration structure.
 *
 * Initialize message RAM memory for std/ext filters, RX/TX
 * buffers and FIFOs and  triggered memory.
 *
 * @return: error code on errorneous conditions and SEDI_DRIVER_OK on success.
 */

static int32_t sedi_can_msg_ram_init(IN enum can_id id,
				     INOUT struct can_config_t *mcan_config)
{
	uint32_t cnt;
	struct can_regs_t *can_regs = mcan_config->regs;
	struct can_params_t *params = mcan_config->params;
	struct can_msg_ram_t *msg_ram = mcan_config->msg_ram;
	uint32_t *start_addr;
	int32_t ret = SEDI_DRIVER_OK;

	DBG_CHECK((CAN_0 == id || CAN_1 == id), SEDI_DRIVER_ERROR_PARAMETER);
	/* Check param against max supported values */
	ret = sedi_can_param_max_limit(id, params);
	if (ret != SEDI_DRIVER_OK) {
		goto err;
	}

	/* Configure standard and extended filters */
	can_regs->sidfc = CAN_MSGRAM_ADDR((uint32_t)msg_ram->std_filters) |
			  CAN_SIDFC_LSS(params->std_filts_cnt);

	msg_ram->ext_filters =
		msg_ram->std_filters +
		(params->std_filts_cnt * (CAN_SIDF_ELEM_SIZE / CAN_MEM_WORD_SIZE));

	can_regs->xidfc = CAN_MSGRAM_ADDR((uint32_t)msg_ram->ext_filters) |
			  CAN_XIDFC_LSE(params->ext_filts_cnt);

	/* Configure RX FIFO 0 & 1 */
	msg_ram->rx_fifo_0 =
		msg_ram->ext_filters +
		(params->ext_filts_cnt * (CAN_XIDF_ELEM_SIZE / CAN_MEM_WORD_SIZE));

	can_regs->rxf0c = CAN_MSGRAM_ADDR((uint32_t)msg_ram->rx_fifo_0) |
			  CAN_RXF0C_F0S(params->rx_fifo0_cnt) |
			  CAN_RXF0C_F0WM(params->rx_fifo0_wm) |
			  CAN_RXF0C_F0OM(params->rx_fifo_mode);

	msg_ram->rx_fifo_1 =
		msg_ram->rx_fifo_0 +
		(params->rx_fifo0_cnt *
		 ((params->rx_fifo0_word_size / CAN_MEM_WORD_SIZE) +
		  CAN_HEADER_SIZE));

	can_regs->rxf1c = CAN_MSGRAM_ADDR((uint32_t)msg_ram->rx_fifo_1) |
			  CAN_RXF1C_F1S(params->rx_fifo1_cnt) |
			  CAN_RXF1C_F1WM(params->rx_fifo1_wm) |
			  CAN_RXF1C_F1OM(params->rx_fifo_mode);
	;

	/* Configure dedicated RX buffer */
	msg_ram->rx_buf = msg_ram->rx_fifo_1 +
			  (params->rx_fifo1_cnt *
			   ((params->rx_fifo1_word_size / CAN_MEM_WORD_SIZE) +
			    CAN_HEADER_SIZE));

	can_regs->rxbc = CAN_MSGRAM_ADDR((uint32_t)msg_ram->rx_buf);

	/* Configure TX Event FIFO */
	msg_ram->tx_evt_fifo =
		msg_ram->rx_buf + (params->rx_buf_cnt *
				   ((params->rx_buf_word_size / CAN_MEM_WORD_SIZE) +
				    CAN_HEADER_SIZE));

	can_regs->txefc = CAN_MSGRAM_ADDR((uint32_t)msg_ram->tx_evt_fifo) |
			  CAN_TXEFC_EFS(params->tx_evt_fifo_cnt) |
			  CAN_TXEFC_EFWM(params->tx_evt_fifo_wm);

	/* Configure TX buffers */
	msg_ram->tx_buf =
		msg_ram->tx_evt_fifo + (params->tx_evt_fifo_cnt *
					CAN_TX_EVT_ELEM_SIZE / CAN_MEM_WORD_SIZE);

	can_regs->txbc = CAN_MSGRAM_ADDR((uint32_t)msg_ram->tx_buf) |
			 CAN_TXBC_NDTB(params->tx_buf_cnt) |
			 CAN_TXBC_TFQS(params->tx_fifo_cnt);

	/* Set RX/TX buffer/FIFO element size */
	can_regs->rxesc =
		CAN_RXESC_RBDS(sedi_can_element_size(params->rx_buf_word_size)) |
		CAN_RXESC_F0DS(sedi_can_element_size(params->rx_fifo0_word_size)) |
		CAN_RXESC_F1DS(sedi_can_element_size(params->rx_fifo1_word_size));

	can_regs->txesc =
		CAN_TXESC_TBDS(sedi_can_element_size(params->tx_buf_word_size));

	/* Init standard filters */
	start_addr = (uint32_t *)msg_ram->std_filters;

	cnt = params->std_filts_cnt;

	while (cnt-- > 0) {
		*start_addr++ = CAN_STD_FILTER_SFEC_DISABLE;
	}

	/* Init extended filters */

	start_addr = (uint32_t *)msg_ram->ext_filters;

	cnt = params->ext_filts_cnt;

	while (cnt-- > 0) {
		*start_addr = CAN_EXT_FILTER_EFEC_DISABLE;
		start_addr = start_addr + 2;
	}
err:
	return ret;
}
/**
 * @brief : Set global filters for messages which are not matched by any filter.
 * @param[in] mcan_config: CAN configuration structures.
 *
 * Set the behaviour of global filters for messages which are not matched by
 * any filter, user can configure to store such messages in RX FIFO or reject.
 *
 * @return: None
 */

static void sedi_can_set_global_filter(IN struct can_config_t *mcan_config)
{
	struct can_regs_t *can_regs = mcan_config->regs;
	struct can_params_t *params = mcan_config->params;

	if (params->gfc_reject) {
		can_regs->gfc |= CAN_GFC_ANFE(2) | CAN_GFC_ANFS(2);
	} else {
		can_regs->gfc &= ~(CAN_GFC_ANFE(3) | CAN_GFC_ANFS(3));
	}
	if (params->gfc_remote_reject) {
		can_regs->gfc |= CAN_GFC_RRFE_REJECT | CAN_GFC_RRFS_REJECT;
	} else {
		can_regs->gfc &= ~(CAN_GFC_RRFE_REJECT | CAN_GFC_RRFS_REJECT);
	}
}

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
				      IN enum can_timestamp_type type)
{
	DBG_CHECK((CAN_0 == id || CAN_1 == id), SEDI_DRIVER_ERROR_PARAMETER);
	struct can_config_t *mcan_config = &can_config[id];
	struct can_regs_t *can_regs = mcan_config->regs;
	int32_t ret = SEDI_DRIVER_OK;

	switch (type) {
	case CAN_TIMESTAMP_ZERO:
		can_regs->tscc |= CAN_TSCC_TSS_ALWAYS_0;
		break;
	case CAN_TIMESTAMP_TCP:
		can_regs->tscc |= CAN_TSCC_TSS_TCP_INC | CAN_TSCC_TCP(counter);
		break;
	case CAN_TIMESTAMP_EXT:
		can_regs->tscc |= CAN_TSCC_TSS_EXT_TIMESTAMP;
		break;
	default:
		ret = SEDI_DRIVER_ERROR_PARAMETER;
	}
	return ret;
}

static void can_soft_rst(void)
{
	static bool rst_done;
	volatile uint32_t *rst_reg = (uint32_t *)(CAN_SFT_RST_REG);

	if (!rst_done) {
		*rst_reg = CAN_SFT_RST_MASK;
		*rst_reg = 0;
		rst_done = true;
	}
}

/**
 * @brief : Initialization of CAN driver.
 * @param[in] id: Id for CAN instance.
 * @param[in] line: address of callback function for notifying application for
 * CAN events.
 *
 * Initialize CAN driver with default mode.
 *
 * @return: returns error code on failure and SEDI_DRIVER_OK on success.
 */

int32_t sedi_can_init(IN enum can_id id, IN can_callback_t cb,
		      INOUT struct can_params_t *can_params)
{
	struct can_config_t *mcan_config = NULL;
	struct can_params_t *params = NULL;
	struct parity_ctrl_regs_t *parity_ctrl = NULL;
	struct parity_stat_regs_t *parity_stat = NULL;
	struct can_regs_t *can_regs = NULL;
	uint32_t can_intr_line;
	int32_t ret = SEDI_DRIVER_OK;

	if (sedi_get_dev_ownership(PSE_DEV_CAN0 + id) != DEV_PSE_OWNED) {
		return SEDI_DRIVER_ERROR_NO_DEV;
	}

	if (!(id < CAN_ID_MAX)) {
		ret = SEDI_DRIVER_ERROR_PARAMETER;
		goto err;
	}

	can_soft_rst();
	mcan_config = &can_config[id];
	mcan_config->mode = CAN_MODE_INITIALIZATION;
	can_intr_line = CAN_INT_LINE_0;
	can_regs = mcan_config->regs;
	parity_ctrl = mcan_config->parity_ctrl_regs;
	parity_stat = mcan_config->parity_stat_regs;

	/* Configure CAN for register setting */
	ret = sedi_can_config_init_start(can_regs);

	if (ret != SEDI_DRIVER_OK) {
		goto err;
	}

	/* Enable parity check */
	parity_ctrl->csr = PARITY_CTL_PAR_ENABLE;

	/* Init message RAM */
	if ((can_params != NULL) && (mcan_config->msg_ram != NULL)) {

		mcan_config->params = can_params;

		ret = sedi_can_msg_ram_init(id, mcan_config);
		if (ret < 0) {
			goto err;
		}
	}

	params = mcan_config->params;

	if (params->disable_auto_retransmit) {
		can_regs->cccr |= CAN_CCCR_DAR_NO_AUTO_RETX;
	}

	/* Set Callback */
	mcan_config->cb = cb;

	/* Reset extended ID and mask */
	can_regs->xidam = CAN_EXTENDED_ID_MASK;

	/* Disable all interrupt */
	can_regs->ie = CAN_DISABLE_ALL_INTR;
	can_regs->txbtie = CAN_DISABLE_ALL_INTR;
	can_regs->ils = CAN_SELECT_INTR_LINE_0;
	can_regs->ile = CAN_DISABLE_INTR_LINE;

	/* Reset interrupt status */
	can_regs->ir = CAN_RESET_INTR_ALL;

	/* Configure bit timing */
	sedi_can_set_bitrate(id, &(params->bit_timing));
	sedi_can_set_fast_bitrate(id, &(params->fast_bit_timing));

	/* Reset new data entries */

	can_regs->ndata1 = CAN_RESET_NEW_DATA_REG;
	can_regs->ndata2 = CAN_RESET_NEW_DATA_REG;

	/* Configure global filter */
	sedi_can_set_global_filter(mcan_config);

	/* Configure time stamp */
	sedi_can_configure_time_stamp(id, params->time_counter,
				      CAN_TIMESTAMP_TCP);

	sedi_can_set_interrupts(id, can_intr_line);

	ret = sedi_can_config_init_finished(can_regs);
	if (ret != SEDI_DRIVER_OK) {
		goto err;
	}
	mcan_config->state = CAN_STATE_ERR_ACTIVE;
err:
	return ret;
}

/**
 * @brief : Set bit rates for CAN IP.
 * @param[in] id: Id for CAN instance.
 * @param[in] bt: can_bittiming structure.
 *
 * Set bit timing settings for CAN IP.
 *
 * @return: None
 */
void sedi_can_set_bitrate(IN enum can_id id, IN struct can_bittiming_t *bt)
{
	struct can_config_t *mcan_config = &can_config[id];
	struct can_regs_t *can_regs = mcan_config->regs;

	if (mcan_config->mode != CAN_MODE_INITIALIZATION) {
		sedi_can_config_init_start(can_regs);
	}
	if (bt != NULL) {
		can_regs->cccr |= CAN_CCCR_CCE_CONFIGURABLE;
		can_regs->nbtp =
			CAN_NBTP_NBRP(bt->brp) | CAN_NBTP_NTSEG1(bt->phase_seg1) |
			CAN_NBTP_NTSEG2(bt->phase_seg2) | CAN_NBTP_NSJW(bt->sjw);
	}
	if (mcan_config->mode != CAN_MODE_INITIALIZATION) {
		sedi_can_config_init_finished(can_regs);
	}
}

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
			       IN struct can_fast_bittiming_t *fbt)
{
	struct can_config_t *mcan_config = &can_config[id];
	struct can_regs_t *can_regs = mcan_config->regs;

	if (mcan_config->mode != CAN_MODE_INITIALIZATION) {
		sedi_can_config_init_start(can_regs);
	}
	if (fbt != NULL) {
		can_regs->cccr |= CAN_CCCR_CCE_CONFIGURABLE;
		can_regs->dbtp = CAN_DBTP_DBRP(fbt->fast_brp) |
				 CAN_DBTP_DTSEG1(fbt->fast_phase_seg1) |
				 CAN_DBTP_DTSEG2(fbt->fast_phase_seg2) |
				 CAN_DBTP_DSJW(fbt->fast_sjw);
	}
	if (mcan_config->mode != CAN_MODE_INITIALIZATION) {
		sedi_can_config_init_finished(can_regs);
	}
}

/**
 * @brief : Set operation mode CAN driver
 * @param[in] id: Id for CAN instance
 * @param[in] mode: mode of operation
 *
 * Set operation mode CAN driver.
 *
 * @return: None
 */

int32_t sedi_can_set_mode(IN enum can_id id, IN enum can_op_mode mode)
{
	int32_t ret = SEDI_DRIVER_OK;

	DBG_CHECK((CAN_0 == id || CAN_1 == id), SEDI_DRIVER_ERROR_PARAMETER);
	struct can_config_t *mcan_config = &can_config[id];
	struct can_regs_t *can_regs = mcan_config->regs;

	ret = sedi_can_config_init_start(can_regs);
	if (ret != SEDI_DRIVER_OK) {
		goto err;
	}
	can_regs->cccr |= CAN_CCCR_CCE_CONFIGURABLE;
	switch (mode) {
	case CAN_MODE_INITIALIZATION:
		sedi_can_set_interrupts(id, CAN_INT_LINE_0);
		mcan_config->mode = CAN_MODE_INITIALIZATION;
		break;
	case CAN_MODE_NORMAL:
		can_regs->cccr &=
			~(CAN_CCCR_TEST_ENABLED | CAN_CCCR_MON_ENABLED |
			  CAN_TEST_LBCK_ENABLED | CAN_CCCR_ASM_RESTRICTED);
		can_regs->cccr &=
			~(CAN_CCCR_FDOE_ENABLE | CAN_CCCR_BRSE_ENABLE);
		mcan_config->mode = CAN_MODE_NORMAL;
		break;
	case CAN_MODE_FD:
		can_regs->cccr &=
			~(CAN_CCCR_TEST_ENABLED | CAN_CCCR_MON_ENABLED |
			  CAN_TEST_LBCK_ENABLED | CAN_CCCR_ASM_RESTRICTED);

		can_regs->cccr |= CAN_CCCR_FDOE_ENABLE;
		can_regs->cccr &= ~(CAN_CCCR_BRSE_ENABLE);
		mcan_config->mode = CAN_MODE_FD;
		break;
	case CAN_MODE_FDBS:
		can_regs->cccr &=
			~(CAN_CCCR_TEST_ENABLED | CAN_CCCR_MON_ENABLED |
			  CAN_TEST_LBCK_ENABLED | CAN_CCCR_ASM_RESTRICTED);

		can_regs->cccr |= CAN_CCCR_FDOE_ENABLE | CAN_CCCR_BRSE_ENABLE;
		mcan_config->mode = CAN_MODE_FDBS;
		break;
	case CAN_MODE_LOOPBACK_INTERNAL:
		can_regs->cccr |= CAN_CCCR_TEST_ENABLED;
		can_regs->cccr |= CAN_CCCR_MON_ENABLED;
		can_regs->test |= CAN_TEST_LBCK_ENABLED;
		mcan_config->mode = CAN_MODE_LOOPBACK_INTERNAL;
		break;
	case CAN_MODE_LOOPBACK_EXTERNAL:
		can_regs->cccr &= ~(CAN_CCCR_MON_ENABLED);

		can_regs->cccr |= CAN_CCCR_TEST_ENABLED;
		can_regs->test |= CAN_TEST_LBCK_ENABLED;
		mcan_config->mode = CAN_MODE_LOOPBACK_EXTERNAL;
		break;
	case CAN_MODE_MONITOR:
		can_regs->cccr &=
			~(CAN_CCCR_TEST_ENABLED | CAN_TEST_LBCK_ENABLED);

		can_regs->cccr |= CAN_CCCR_MON_ENABLED;
		mcan_config->mode = CAN_MODE_MONITOR;
		break;
	case CAN_MODE_RESTRICTED:
		can_regs->cccr |= CAN_CCCR_ASM_RESTRICTED;
		mcan_config->mode = CAN_MODE_RESTRICTED;
		break;
	case CAN_MODE_CLK_STOP_REQ:
		can_regs->cccr |= CAN_CCCR_CSR_CLOCK_STOP;
		mcan_config->mode = CAN_MODE_CLK_STOP_REQ;
		break;
	case CAN_MODE_DAR:
		can_regs->cccr |= CAN_CCCR_DAR_NO_AUTO_RETX;
		mcan_config->mode = CAN_MODE_DAR;
		break;
	case CAN_MODE_TX_PAUSE:
		can_regs->cccr |= CAN_CCCR_TXP_ENABLE;
		mcan_config->mode = CAN_MODE_TX_PAUSE;
		break;
	case CAN_MODE_TX_PAUSE_DISABLE:
		if (mcan_config->mode == CAN_MODE_TX_PAUSE) {
			can_regs->cccr &= CAN_CCCR_TXP_DISABLE;
			mcan_config->mode = CAN_MODE_TX_PAUSE_DISABLE;
		}
		break;
	case CAN_MODE_LOOPBACK_DISABLE:
		if ((mcan_config->mode == CAN_MODE_LOOPBACK_INTERNAL) ||
		    (mcan_config->mode == CAN_MODE_LOOPBACK_EXTERNAL)) {
			can_regs->test &= ~CAN_TEST_LBCK_ENABLED;
			can_regs->cccr &= ~CAN_CCCR_TEST_ENABLED;
			if (mcan_config->mode == CAN_MODE_LOOPBACK_INTERNAL) {
				can_regs->cccr &= ~CAN_CCCR_MON_ENABLED;
			}
			mcan_config->mode = CAN_MODE_LOOPBACK_DISABLE;
		}
		break;
	case CAN_MODE_RESTRICTED_DISABLE:
		if (mcan_config->mode == CAN_MODE_RESTRICTED) {
			can_regs->cccr &= ~CAN_CCCR_ASM_RESTRICTED;
			mcan_config->mode = CAN_MODE_RESTRICTED_DISABLE;
		}
		break;
	case CAN_MODE_CLK_STOP_DISABLE:
		if (mcan_config->mode == CAN_MODE_CLK_STOP_REQ) {
			can_regs->cccr &= ~CAN_CCCR_CSR_CLOCK_STOP;
			mcan_config->mode = CAN_MODE_CLK_STOP_DISABLE;
		}
		break;
	case CAN_MODE_DAR_DISABLE:
		if (mcan_config->mode == CAN_MODE_DAR) {
			can_regs->cccr &= ~CAN_CCCR_DAR;
			mcan_config->mode = CAN_MODE_DAR_DISABLE;
		}
		break;
	case CAN_MODE_NONE:
		/* INVALID MODE */
		ret = SEDI_DRIVER_ERROR_PARAMETER;
		goto err;
	}
	ret = sedi_can_config_init_finished(can_regs);
	if (ret != SEDI_DRIVER_OK) {
		goto err;
	}
err:
	return ret;
}

/**
 * @brief : Transmit standard message.
 * @param[in] id: Id for CAN instance.
 * @param[in] frame: standard message data
 *
 * Transmit standard message by configuring dedicated or fifo tx memory.
 *
 * @return: buffer number which is configured to transmit.
 */

static uint8_t sedi_can_send_std_message(IN enum can_id id, IN void *frame)
{
	struct can_frame_t *sd_frame;
	uint32_t mask1 = 0, mask2;
	uint8_t buf_num = CAN_INVALID_BUFNUM, put_index = CAN_INVALID_BUFNUM;
	__IO_RW uint32_t *buffer = NULL;
	uint32_t *src_data = NULL;
	uint32_t cnt, length = 0;

	DBG_CHECK((CAN_0 == id || CAN_1 == id), CAN_INVALID_BUFNUM);
	struct can_config_t *mcan_config = &can_config[id];
	struct can_regs_t *can_regs = mcan_config->regs;
	struct can_params_t *params = mcan_config->params;
	struct can_msg_ram_t *msg_ram = mcan_config->msg_ram;

	sd_frame = (struct can_frame_t *)frame;

	if (sd_frame->info.flags & CAN_BUFFER_RTR_FLAG) {
		mask1 |= CAN_BUFFER_RTR_MASK;
	}

	mask2 = (sd_frame->info.mm << CAN_MSG_MM_POS);
	if (mcan_config->mode == CAN_MODE_FD) {
		mask2 |= CAN_BUFFER_FDF_MASK;
	}
	if (mcan_config->mode == CAN_MODE_FDBS) {
		mask2 |= CAN_BUFFER_BRS_MASK | CAN_BUFFER_FDF_MASK;
	}

	if (sd_frame->info.flags & CAN_BUFFER_EFC_FLAG) {
		mask2 |= CAN_BUFFER_EFC_MASK;
	}

	if (sd_frame->info.isfifo) {
		if ((params->tx_fifo_cnt > 0) &&
		    ((can_regs->txfqs & CAN_TXFQS_TFQF) == 0)) {
			put_index =
				(((can_regs->txfqs & CAN_TXFQS_TFQPI_MASK) >>
				  CAN_TXFQS_TFQPI_POS));

			buffer =
				msg_ram->tx_buf +
				(put_index *
				 ((params->tx_buf_word_size / CAN_MEM_WORD_SIZE) +
				  CAN_HEADER_SIZE));
			*buffer++ =
				mask1 | ((sd_frame->info.id << CAN_STD_ID_POS) &
					 (CAN_11_BIT_ID_MASK << CAN_STD_ID_POS));

			*buffer++ =
				mask2 |
				(sedi_can_msg_len_to_dlc(sd_frame->info.length))
					<< CAN_MSG_LEN_POS;

			src_data = (uint32_t *)&(sd_frame->data);
			length = sd_frame->info.length;
			buf_num = put_index;
		}
	} else {
		if (sd_frame->info.buff_num < params->tx_buf_cnt) {

			buf_num = sd_frame->info.buff_num;

			buffer =
				msg_ram->tx_buf +
				(buf_num *
				 ((params->tx_buf_word_size / CAN_MEM_WORD_SIZE) +
				  CAN_HEADER_SIZE));
			*buffer++ =
				mask1 | ((sd_frame->info.id << CAN_STD_ID_POS) &
					 (CAN_11_BIT_ID_MASK << CAN_STD_ID_POS));

			*buffer++ =
				mask2 |
				(sedi_can_msg_len_to_dlc(sd_frame->info.length))
					<< CAN_MSG_LEN_POS;

			src_data = (uint32_t *)&(sd_frame->data);

			length = sd_frame->info.length;
		}
	}

	if (!(mask1 & CAN_BUFFER_RTR_MASK)) {
		for (cnt = 0; cnt < length; cnt += CAN_MEM_WORD_SIZE) {
			*buffer++ = *src_data++;
		}
	}
	return buf_num;
}
/**
 * @brief : Transmit extended message.
 * @param[in] id: Id for CAN instance.
 * @param[in] frame: extended message data
 *
 * Transmit extended message by configuring dedicated or fifo tx memory.
 *
 * @return: buffer number which is configured to transmit.
 */

static uint8_t sedi_can_send_ext_message(IN enum can_id id, IN void *data)
{
	struct can_frame_t *ext_frame;
	uint32_t mask1, mask2;
	uint8_t buf_num = CAN_INVALID_BUFNUM, put_index = CAN_INVALID_BUFNUM;
	__IO_RW uint32_t *buffer = NULL, *temp = NULL;
	uint32_t *src_data = NULL;
	uint32_t cnt, length = 0;

	DBG_CHECK((CAN_0 == id || CAN_1 == id), CAN_INVALID_BUFNUM);
	struct can_config_t *mcan_config = &can_config[id];
	struct can_regs_t *can_regs = mcan_config->regs;
	struct can_params_t *params = mcan_config->params;
	struct can_msg_ram_t *msg_ram = mcan_config->msg_ram;

	ext_frame = (struct can_frame_t *)data;

	mask1 = CAN_BUFFER_XTD_MASK;

	if (ext_frame->info.flags & CAN_BUFFER_RTR_FLAG) {
		mask1 |= CAN_BUFFER_RTR_MASK;
	}

	mask2 = (ext_frame->info.mm << CAN_MSG_MM_POS);

	if (ext_frame->info.flags & CAN_BUFFER_EFC_FLAG) {
		mask2 |= CAN_BUFFER_EFC_MASK;
	}
	if (mcan_config->mode == CAN_MODE_FD) {
		mask2 |= CAN_BUFFER_FDF_MASK;
	}
	if (mcan_config->mode == CAN_MODE_FDBS) {
		mask2 |= CAN_BUFFER_BRS_MASK | CAN_BUFFER_FDF_MASK;
	}
	if (ext_frame->info.isfifo) {

		if ((params->tx_fifo_cnt > 0) &&
		    ((can_regs->txfqs & CAN_TXFQS_TFQF) == 0)) {

			put_index =
				(((can_regs->txfqs & CAN_TXFQS_TFQPI_MASK) >>
				  CAN_TXFQS_TFQPI_POS));
			buffer =
				msg_ram->tx_buf +
				(put_index *
				 ((params->tx_buf_word_size / CAN_MEM_WORD_SIZE) +
				  CAN_HEADER_SIZE));

			temp = buffer;
			*buffer++ = mask1 | CAN_BUFFER_XTD_MASK |
				    (ext_frame->info.id & CAN_29_BIT_ID_MASK);

			*buffer++ =
				mask2 |
				(sedi_can_msg_len_to_dlc(ext_frame->info.length))
					<< CAN_MSG_LEN_POS;

			src_data = (uint32_t *)&(ext_frame->data);
			length = ext_frame->info.length;
			buf_num = put_index;
		}
	} else {
		if (ext_frame->info.buff_num < params->tx_buf_cnt) {
			buf_num = ext_frame->info.buff_num;

			buffer =
				msg_ram->tx_buf +
				(buf_num *
				 ((params->tx_buf_word_size / CAN_MEM_WORD_SIZE) +
				  CAN_HEADER_SIZE));
			temp = buffer;
			*buffer++ = CAN_BUFFER_XTD_MASK |
				    (ext_frame->info.id & CAN_29_BIT_ID_MASK);

			*buffer++ =
				(sedi_can_msg_len_to_dlc(ext_frame->info.length))
					<< CAN_MSG_LEN_POS;

			src_data = (uint32_t *)&(ext_frame->data);
			length = ext_frame->info.length;
		}
	}
	if (!(mask1 & CAN_BUFFER_RTR_MASK)) {
		for (cnt = 0; cnt < length; cnt += CAN_MEM_WORD_SIZE) {
			*buffer++ = *src_data++;
		}
	}
	return buf_num;
}

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

int32_t sedi_can_send_message(IN enum can_id id, IN enum can_msg_id msg_id,
			      IN void *frame)
{

	DBG_CHECK((CAN_0 == id || CAN_1 == id), SEDI_DRIVER_ERROR_PARAMETER);
	struct can_config_t *mcan_config = &can_config[id];
	struct can_regs_t *can_regs = mcan_config->regs;
	uint8_t buf_num = CAN_INVALID_BUFNUM;
	int32_t ret = SEDI_DRIVER_OK;

	if (frame == NULL) {
		ret = SEDI_DRIVER_ERROR_PARAMETER;
		goto err;
	}
	if (msg_id == CAN_STD_ID) {
		buf_num = sedi_can_send_std_message(id, frame);
	} else {
		buf_num = sedi_can_send_ext_message(id, frame);
	}
	if (buf_num != CAN_INVALID_BUFNUM) {
		can_regs->txbtie |= BIT(buf_num);
		can_regs->txbar |= BIT(buf_num);
	} else {
		ret = SEDI_DRIVER_ERROR;
	}
err:
	return ret;
}

/**
 * @brief : Receive standard/extended message from rx fifo 0/1.
 * @param[in] id: Id for CAN instance.
 * @param[in] fifo_id: fifo id for rx fifo as CAN_FIFO_0 or CAN_FIFO_1.
 *
 * Receive standard/extended message from rx fifo.
 *
 * @return: buffer pointer to message data.
 */

static __IO_RW uint32_t *sedi_can_receive_from_fifo(IN enum can_id id,
						    IN enum can_fifo fifo_id)
{
	DBG_CHECK((CAN_0 == id || CAN_1 == id), NULL);
	struct can_config_t *mcan_config = &can_config[id];
	struct can_regs_t *can_regs = mcan_config->regs;
	struct can_params_t *params = mcan_config->params;
	struct can_msg_ram_t *msg_ram = mcan_config->msg_ram;
	uint32_t status = can_regs->ir;
	uint32_t get_index, put_index, fill_level;
	__IO_RW uint32_t *buffer = NULL;

	if (fifo_id == CAN_FIFO_0) {
		if (status & CAN_IR_RF0L) {
			return NULL;
		} else if ((status & CAN_IR_RF0F) &&
			   (params->rx_fifo_mode == CAN_FIFO_BLOCKING)) {
			return NULL;
		} else if ((status & CAN_IR_RF0W) &&
			   (params->rx_fifo_mode == CAN_FIFO_BLOCKING)) {
			return NULL;
		}
		get_index = ((can_regs->rxf0s & CAN_RXF0S_F0GI_MASK) >>
			     CAN_RXF0S_F0GI_POS);
		if (params->rx_fifo_mode == CAN_FIFO_OVERWRITE) {
			put_index = (can_regs->rxf0s &
				     CAN_RXF0S_F0PI_MASK >> CAN_RXF0S_F0PI_POS);

			if ((get_index > put_index) &&
			    (can_regs->rxf0s & CAN_RXF0S_F0F)) {
				get_index = 0;
			}
		}
		fill_level = (can_regs->rxf0s & CAN_RXF0S_F0FL_MASK) >>
			     CAN_RXF0S_F0FL_POS;
		if (fill_level > 0) {
			buffer =
				msg_ram->rx_fifo_0 +
				(get_index *
				 ((params->rx_fifo0_word_size / CAN_MEM_WORD_SIZE) +
				  CAN_HEADER_SIZE));
		}
		can_regs->ir |= CAN_IR_RF0N;
		can_regs->rxf0a = CAN_RXF0A_F0AI(get_index);

	} else {
		if (status & CAN_IR_RF1L) {
			return NULL;
		} else if ((status & CAN_IR_RF1F) &&
			   (params->rx_fifo_mode == CAN_FIFO_BLOCKING)) {
			return NULL;
		} else if ((status & CAN_IR_RF1W) &&
			   (params->rx_fifo_mode == CAN_FIFO_BLOCKING)) {
			return NULL;
		}
		get_index = ((can_regs->rxf1s & CAN_RXF1S_F1GI_MASK) >>
			     CAN_RXF1S_F1GI_POS);
		if (params->rx_fifo_mode == CAN_FIFO_OVERWRITE) {
			put_index = (can_regs->rxf1s &
				     CAN_RXF1S_F1PI_MASK >> CAN_RXF1S_F1PI_POS);

			if ((get_index > put_index) &&
			    (can_regs->rxf1s & CAN_RXF1S_F1F)) {
				get_index = 0;
			}
		}
		fill_level = (can_regs->rxf1s & CAN_RXF1S_F1FL_MASK) >>
			     CAN_RXF1S_F1FL_POS;
		if (fill_level > 0) {
			buffer =
				msg_ram->rx_fifo_1 +
				(get_index *
				 ((params->rx_fifo1_word_size / CAN_MEM_WORD_SIZE) +
				  CAN_HEADER_SIZE));
		}
		can_regs->ir |= CAN_IR_RF1N;
		can_regs->rxf1a = CAN_RXF1A_F1AI(get_index);
	}

	return buffer;
}

/**
 * @brief : Fill can_ext_frame structure from received message.
 * @param[in] buffer: buffer pointer received from can bus.
 * @param[inout] ext_frame: structure for ext frame
 * @param[in] mask: mask for received message flags
 *
 * Fill can_frame structure from received message.
 *
 * @return: None
 */

static void sedi_fill_xtd_message(IN __IO_RW uint32_t *buffer,
				  INOUT struct can_frame_t *ext_frame,
				  IN uint32_t mask)
{
	int dlc;

	ext_frame->info.flags = 0;
	ext_frame->info.id = *buffer & CAN_BUFFER_EXT_ID_MASK;
	ext_frame->info.flags |= mask;
	buffer++;

	if (*buffer & CAN_BUFFER_ANMF_MASK) {
		ext_frame->info.flags |= CAN_BUFFER_ANMF_FLAG;
	}
	ext_frame->info.filter_index =
		((*buffer & CAN_BUFFER_FIDX_MASK) >> CAN_MSG_FIDX_POS);

	if (*buffer & CAN_BUFFER_FDF_MASK) {
		ext_frame->info.flags |= CAN_BUFFER_FDF_FLAG;
	}

	if (*buffer & CAN_BUFFER_BRS_MASK) {
		ext_frame->info.flags |= CAN_BUFFER_BRS_FLAG;
	}

	ext_frame->info.timestamp = *buffer & CAN_BUFFER_TIMESTAMP_MASK;

	dlc = (*buffer & CAN_BUFFER_DLC_MASK) >> CAN_MSG_LEN_POS;

	ext_frame->info.length = sedi_can_dlc_to_msg_len(dlc);
}

/**
 * @brief : Fill can_sd_frame structure from received message.
 * @param[in] buffer: buffer pointer received from can bus.
 * @param[inout] sd_frame: structure for frame
 * @param[in] mask: mask for received message flags
 *
 * Fill can_sd_frame structure from received message.
 *
 * @return: None
 */

static void sedi_fill_std_message(IN __IO_RW uint32_t *buffer,
				  INOUT struct can_frame_t *sd_frame,
				  IN uint32_t mask)
{
	int dlc;

	sd_frame->info.flags = 0;
	sd_frame->info.id =
		(*buffer & CAN_BUFFER_STD_ID_MASK) >> CAN_STD_ID_POS;
	sd_frame->info.flags |= mask;
	buffer++;

	if (*buffer & CAN_BUFFER_ANMF_MASK) {
		sd_frame->info.flags |= CAN_BUFFER_ANMF_FLAG;
	}

	sd_frame->info.filter_index =
		((*buffer & CAN_BUFFER_FIDX_MASK) >> CAN_MSG_FIDX_POS);

	if (*buffer & CAN_BUFFER_FDF_MASK) {
		sd_frame->info.flags |= CAN_BUFFER_FDF_FLAG;
	}

	if (*buffer & CAN_BUFFER_BRS_MASK) {
		sd_frame->info.flags |= CAN_BUFFER_BRS_FLAG;
	}

	sd_frame->info.timestamp = *buffer & CAN_BUFFER_TIMESTAMP_MASK;

	dlc = (*buffer & CAN_BUFFER_DLC_MASK) >> CAN_MSG_LEN_POS;

	sd_frame->info.length = sedi_can_dlc_to_msg_len(dlc);
}

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
				 IN int buf_num)
{
	DBG_CHECK((CAN_0 == id || CAN_1 == id), SEDI_DRIVER_ERROR_PARAMETER);
	struct can_config_t *mcan_config = &can_config[id];
	struct can_regs_t *can_regs = mcan_config->regs;
	struct can_params_t *params = mcan_config->params;
	struct can_msg_ram_t *msg_ram = mcan_config->msg_ram;
	uint32_t mask1 = 0;
	uint8_t *src_buf = NULL, *dst_buf = NULL;
	struct can_frame_t frame;
	uint32_t length = 0, cnt = 0;
	uint8_t extended = 0, rtr = 0;
	__IO_RW uint32_t *buffer = NULL;
	int32_t ret = SEDI_DRIVER_ERROR;

	if (buf_num == CAN_INVALID_BUFNUM && CAN_FIFO_INVALID != fifo_id) {
		buffer = sedi_can_receive_from_fifo(id, fifo_id);
	} else {
		if (CAN_INVALID_BUFNUM != buf_num &&
		    buf_num < params->rx_buf_cnt) {

			buffer =
				msg_ram->rx_buf +
				(buf_num *
				 ((params->rx_buf_word_size / CAN_MEM_WORD_SIZE) +
				  CAN_HEADER_SIZE));
		}
	}
	if (buffer == NULL) {
		goto err;
	} else {
		if (*buffer & CAN_BUFFER_ESI_MASK) {
			mask1 |= CAN_BUFFER_ESI_FLAG;
		}

		if (*buffer & CAN_BUFFER_RTR_MASK) {
			mask1 |= CAN_BUFFER_RTR_FLAG;
			rtr = 1;
		}
		if (*buffer & CAN_BUFFER_XTD_MASK) {
			extended = 1;
			sedi_fill_xtd_message(buffer, (void *)&frame, mask1);
		} else {
			sedi_fill_std_message(buffer, (void *)&frame, mask1);
		}
		length = frame.info.length;
		dst_buf = (uint8_t *)&(frame.data);

		buffer += 2;
		src_buf = (uint8_t *)buffer;

		if (!rtr) {
			for (cnt = 0; cnt < length; cnt++) {
				*dst_buf++ = *src_buf++;
			}
		}
		ret = SEDI_DRIVER_OK;
	}
	if (buf_num >= 0) {
		if (buf_num < CAN_MAX_NEW_DATA_NUM_1) {
			can_regs->ndata1 = (1 << buf_num);
		} else if (buf_num > CAN_MAX_NEW_DATA_NUM_1) {
			can_regs->ndata2 =
				(1 << (buf_num - CAN_MAX_NEW_DATA_NUM_1));
		}
		can_regs->ir = CAN_IR_DRX;
	}

	if (extended) {
		mcan_config->cb(id, CAN_EXT_MSG_RECEIVE, (void *)&frame);
	} else {
		mcan_config->cb(id, CAN_STD_MSG_RECEIVE, (void *)&frame);
	}
err:
	return ret;
}

/**
 * @brief : Configure CAN filter to receive accepted messages into rx buffer.
 * @param[in] id: Id for CAN instance
 * @param[in] filter: address of can_filter_t structure, contains information
 * about filter number and type of filter and mode of operation.
 * @param[in] msg_id: Id for CAN message as CAN_STD_ID or CAN_EXT_ID.
 *
 * Configure CAN filter to receive accepted messages into rx buffer.
 *
 * @return: None
 */

static void sedi_can_config_rx_buf_filter(IN enum can_id id,
					  IN struct can_filter_t *filter,
					  IN enum can_msg_id msg_id)
{
	__IO_RW uint32_t *curr_filter;
	struct can_config_t *mcan_config = &can_config[id];
	struct can_params_t *params = mcan_config->params;
	struct can_msg_ram_t *msg_ram = mcan_config->msg_ram;
	uint32_t mask = 0;

	switch (filter->filter_type) {
	case CAN_FILTER_DUAL_ID:
		mask |= CAN_STD_FILTER_SFT_DUAL_FILTER;
		break;
	case CAN_FILTER_CLASSIC:
		mask |= CAN_STD_FILTER_SFT_CLASSIC_FILTER;
		break;
	case CAN_FILTER_RANGE:
		mask |= CAN_STD_FILTER_SFT_RANGE;
		break;
	}

	if (msg_id == CAN_STD_ID) {
		if (filter->filter_num < params->std_filts_cnt) {

			curr_filter = msg_ram->std_filters + filter->filter_num;

			*curr_filter = mask | CAN_STD_FILTER_SFEC_BUFFER |
				       (filter->id1 << CAN_FILTER_ID1_POS) |
				       CAN_STD_FILTER_SFID2_RX_BUFFER |
				       filter->buf_num;
		}
	} else {
		if (filter->filter_num < params->ext_filts_cnt) {
			curr_filter =
				msg_ram->ext_filters + (2 * filter->filter_num);
			*curr_filter =
				CAN_EXT_FILTER_EFEC_BUFFER | (filter->id1);

			curr_filter++;
			*curr_filter = mask | CAN_EXT_FILTER_EFID2_RX_BUFFER |
				       filter->buf_num;
		}
	}
}

/**
 * @brief : Configure CAN filter to receive accepted std messages into rx fifo.
 * @param[in] id: Id for CAN instance
 * @param[in] filter: address of can_filter_t structure, contains information
 * about filter number and type of filter and mode of operation.
 *
   Configure CAN filter to receive accepted std messages into rx fifo.
 *
 * @return: None
 */

static void sedi_can_config_std_filter(IN enum can_id id,
				       IN struct can_filter_t *filter)
{
	__IO_RW uint32_t *curr_filter;
	struct can_config_t *mcan_config = &can_config[id];
	struct can_regs_t *can_regs = mcan_config->regs;
	struct can_params_t *params = mcan_config->params;
	struct can_msg_ram_t *msg_ram = mcan_config->msg_ram;
	uint32_t mask = 0;

	switch (filter->filter_type) {
	case CAN_FILTER_DUAL_ID:
		mask |= CAN_STD_FILTER_SFT_DUAL_FILTER;
		break;
	case CAN_FILTER_CLASSIC:
		mask |= CAN_STD_FILTER_SFT_CLASSIC_FILTER;
		break;
	case CAN_FILTER_RANGE:
		mask |= CAN_STD_FILTER_SFT_RANGE;
		break;
	}
	if (filter->filter_num < params->std_filts_cnt) {
		curr_filter = msg_ram->std_filters + filter->filter_num;
		switch (filter->op) {
		case CAN_FILTER_OP_FIFO0:
			mask |= CAN_STD_FILTER_SFEC_FIFO0;
			break;
		case CAN_FILTER_OP_FIFO0_PRIO:
			mask |= CAN_STD_FILTER_SFEC_PRIORITY_FIFO0;
			can_regs->ie = CAN_IE_HPME;
			break;
		case CAN_FILTER_OP_FIFO1:
			mask |= CAN_STD_FILTER_SFEC_FIFO1;
			break;
		case CAN_FILTER_OP_FIFO1_PRIO:
			mask |= CAN_STD_FILTER_SFEC_PRIORITY_FIFO1;
			can_regs->ie = CAN_IE_HPME;
			break;
		case CAN_FILTER_OP_PRIO:
			mask |= CAN_STD_FILTER_SFEC_PRIORITY;
			can_regs->ie = CAN_IE_HPME;
			break;
		case CAN_FILTER_OP_DISABLE:
		case CAN_FILTER_OP_RXBUF:
		case CAN_FILTER_OP_REJECT:
			/* To Avoid -Werror */
			break;
		}
		*curr_filter =
			mask | (filter->id1 << CAN_FILTER_ID1_POS) | filter->id2;
	}
}

/**
 * @brief : Configure CAN filter to receive accepted ext messages into rx fifo.
 * @param[in] id: Id for CAN instance
 * @param[in] filter: address of can_filter_t structure, contains information
 * about filter number and type of filter and mode of operation.
 *
   Configure CAN filter to receive accepted std messages into rx fifo.
 *
 * @return: None
 */

static void sedi_can_config_ext_filter(IN enum can_id id,
				       IN struct can_filter_t *filter)
{
	__IO_RW uint32_t *curr_filter;
	struct can_config_t *mcan_config = &can_config[id];
	struct can_regs_t *can_regs = mcan_config->regs;
	struct can_params_t *params = mcan_config->params;
	struct can_msg_ram_t *msg_ram = mcan_config->msg_ram;
	uint32_t mask = 0, mask2 = 0;

	switch (filter->filter_type) {
	case CAN_FILTER_DUAL_ID:
		mask2 |= CAN_EXT_FILTER_EFT_DUAL_FILTER;
		break;
	case CAN_FILTER_CLASSIC:
		mask2 |= CAN_EXT_FILTER_EFT_CLASSIC_FILTER;
		break;
	case CAN_FILTER_RANGE:
		mask2 |= CAN_EXT_FILTER_EFT_RANGE_FILTER;
		break;
	}
	if (filter->filter_num < params->ext_filts_cnt) {
		curr_filter = msg_ram->ext_filters + (2 * filter->filter_num);
		switch (filter->op) {
		case CAN_FILTER_OP_FIFO0:
			mask |= CAN_EXT_FILTER_EFEC_FIFO0;
			break;
		case CAN_FILTER_OP_FIFO0_PRIO:
			mask |= CAN_EXT_FILTER_EFEC_PRIORITY_FIFO0;
			can_regs->ie = CAN_IE_HPME;
			break;
		case CAN_FILTER_OP_FIFO1:
			mask |= CAN_EXT_FILTER_EFEC_FIFO1;
			break;
		case CAN_FILTER_OP_FIFO1_PRIO:
			mask |= CAN_EXT_FILTER_EFEC_PRIORITY_FIFO1;
			can_regs->ie = CAN_IE_HPME;
			break;
		case CAN_FILTER_OP_PRIO:
			mask |= CAN_EXT_FILTER_EFEC_PRIORITY;
			can_regs->ie = CAN_IE_HPME;
			break;
		case CAN_FILTER_OP_DISABLE:
		case CAN_FILTER_OP_RXBUF:
		case CAN_FILTER_OP_REJECT:
			/* To Avoid -Werror */
			break;
		}
		*curr_filter = mask | filter->id1;
		curr_filter++;
		*curr_filter = mask2 | filter->id2;
	}
}

/**
 * @brief : Configure CAN filter to receive accepted messages into
 * rx fifo/buffer.
 * @param[in] id: Id for CAN instance
 * @param[in] filter: address of can_filter_t structure, contains information
 * about filter number and type of filter and mode of operation.
 * @param[in] msg_id: Id for CAN message as CAN_STD_ID or CAN_EXT_ID.
 *
 * Configure CAN filter to receive accepted messages into rx buffer/fifo.
 *
 * @return: return SEDI_DRIVER_OK on success.
 */
int32_t sedi_can_config_filter(IN enum can_id id,
			       IN struct can_filter_t *filter,
			       IN enum can_msg_id msg_id)
{
	__IO_RW uint32_t *curr_filter;

	if (!(id < CAN_ID_MAX)) {
		return SEDI_DRIVER_ERROR_PARAMETER;
	}

	struct can_config_t *mcan_config = &can_config[id];
	struct can_params_t *params = mcan_config->params;
	struct can_msg_ram_t *msg_ram = mcan_config->msg_ram;
	uint8_t filter_cnt = CAN_INVALID_FILTER_NUM;
	int32_t ret = SEDI_DRIVER_OK;

	if (msg_id == CAN_STD_ID) {
		filter_cnt = params->std_filts_cnt;
	} else if (msg_id == CAN_EXT_ID) {
		filter_cnt = params->ext_filts_cnt;
	}
	if (filter->filter_num >= filter_cnt) {
		ret = SEDI_DRIVER_ERROR_PARAMETER;
		goto err;
	}
	if (filter->op == CAN_FILTER_OP_RXBUF) {
		sedi_can_config_rx_buf_filter(id, filter, msg_id);
	} else if ((filter->op != CAN_FILTER_OP_DISABLE) &&
		   (filter->op != CAN_FILTER_OP_REJECT)) {

		if (msg_id == CAN_STD_ID) {
			sedi_can_config_std_filter(id, filter);
		} else {

			sedi_can_config_ext_filter(id, filter);
		}
	} else if (filter->op == CAN_FILTER_OP_DISABLE) {

		if (CAN_STD_ID == msg_id &&
		    filter->filter_num < params->std_filts_cnt) {

			curr_filter = msg_ram->std_filters + filter->filter_num;
			*curr_filter &= CAN_STD_FILTER_SFEC_DISABLE;
		} else if (CAN_EXT_ID == msg_id &&
			   filter->filter_num < params->std_filts_cnt) {

			curr_filter =
				msg_ram->std_filters + (2 * filter->filter_num);
			*curr_filter &= CAN_EXT_FILTER_EFEC_DISABLE;
		}
	} else if (filter->op == CAN_FILTER_OP_REJECT) {

		if (CAN_STD_ID == msg_id &&
		    filter->filter_num < params->std_filts_cnt) {

			curr_filter = msg_ram->std_filters + filter->filter_num;

			*curr_filter = CAN_STD_FILTER_SFEC_REJECT;
		} else if (CAN_EXT_ID == msg_id &&
			   filter->filter_num < params->std_filts_cnt) {

			curr_filter =
				msg_ram->ext_filters + (2 * filter->filter_num);

			*curr_filter = CAN_EXT_FILTER_EFEC_REJECT;
		}
	}
err:
	return ret;
}

/**
 * @brief : Cancel a message transmission.
 * @param[in] id: Id for CAN instance
 * @param[in] buffer: buffer number for which transmission to be canceled.
 *
 * Cancel a message transmission.
 *
 * @return: return SEDI_DRIVER_OK on success and error code on failure.
 */

int32_t sedi_can_cancel_tx(IN enum can_id id, IN uint16_t buffer)
{
	DBG_CHECK((CAN_0 == id || CAN_1 == id), SEDI_DRIVER_ERROR_PARAMETER);
	struct can_config_t *mcan_config = &can_config[id];
	struct can_regs_t *can_regs = mcan_config->regs;
	struct can_params_t *params = mcan_config->params;
	int32_t ret = SEDI_DRIVER_OK;

	if (!(can_regs->txbc & CAN_TXBC_TFQM) &&
	    ((buffer < params->tx_buf_cnt) || buffer < params->tx_fifo_cnt)) {

		can_regs->txbcr = BIT(buffer);
		can_regs->txbcie = BIT(buffer);
	} else {
		ret = SEDI_DRIVER_ERROR;
	}
	return ret;
}

/**
 * @brief : Reset CAN IP.
 * @param[in] id: Id for CAN instance
 *
 * Reset CAN IP to come out of BUS_OFF state.
 *
 * @return: return SEDI_DRIVER_OK on success and error code on failure.
 */

int32_t sedi_can_reset(IN enum can_id id)
{
	uint32_t val;
	int count = TIMER_LIMIT;

	DBG_CHECK((CAN_0 == id || CAN_1 == id), SEDI_DRIVER_ERROR_PARAMETER);
	struct can_config_t *mcan_config = &can_config[id];
	struct can_regs_t *can_regs = mcan_config->regs;
	int32_t ret = SEDI_DRIVER_OK;

	/* Recover from Bus Off state */
	if (mcan_config->state == CAN_STATE_BUS_OFF) {

		can_regs->cccr &= ~CAN_CCCR_INIT_ENABLED;
		do {
			val = can_regs->ecr;
		} while (!(((val & CAN_ECR_REC_MASK) >> CAN_ECR_REC_POS) <
			   CAN_MAX_RESET_CNT));
	} else {
		can_regs->cccr &= ~CAN_CCCR_INIT_ENABLED;
	}
	count = TIMER_LIMIT;
	while ((can_regs->cccr & CAN_CCCR_INIT) && count) {
		count--;
	}
	if (!count) {

		ret = SEDI_DRIVER_ERROR_TIMEOUT;
		goto err;
	}

	if (mcan_config->state != CAN_STATE_STOP) {
		mcan_config->state = CAN_STATE_STOP;
	}
err:
	return ret;
}

/**
 * @brief Recover from bus-off state
 * @param[in] id: Id for CAN instance
 *
 * Recover the CAN controller from bus-off state to error-active state.
 *
 * @return: return SEDI_DRIVER_OK on success and error code on failure.
 */

int32_t sedi_can_recover(IN enum can_id id)
{
	DBG_CHECK((CAN_0 == id || CAN_1 == id), SEDI_DRIVER_ERROR_PARAMETER);
	struct can_config_t *mcan_config = &can_config[id];
	int32_t ret = SEDI_DRIVER_OK;

	ret = sedi_can_reset(id);
	if (ret != SEDI_DRIVER_OK) {
		goto err;
	}
	mcan_config->state = CAN_STATE_ERR_ACTIVE;
err:
	return ret;
}

/**
 * @brief : Handle parity error interrupt from ISR.
 * @param[in] id: Id for CAN instance
 * @param[in] parity_ctrl: parity control register value to identity the error.
 *
 * Handle parity error interrupt from ISR.
 *
 * @return: None
 */

static void sedi_can_handle_parity_err(IN enum can_id id,
				       INOUT struct parity_ctrl_regs_t *parity_ctrl)
{
	int offset;
	struct can_config_t *mcan_config = &can_config[id];

	offset = (parity_ctrl->err_offset & PARITY_CTL_PAR_ERR_OFFSET_MASK);
	if (parity_ctrl->err_offset & PARITY_CTL_PAR_ERR_IN_LOWER_16_BITS) {
		mcan_config->cb(id, CAN_PARITY_ERR_OCCURRED_LOWER, &offset);
	}
	if (parity_ctrl->err_offset & PARITY_CTL_PAR_ERR_IN_UPPER_16_BITS) {
		mcan_config->cb(id, CAN_PARITY_ERR_OCCURRED_UPPER, &offset);
	}
	parity_ctrl->csr |= PARITY_CTL_PAR_ERR_OCCURED;
	mcan_config->state = CAN_STATE_STOP;
	sedi_can_reset(id);
	sedi_can_set_interrupts(id, (enum can_intr_line)id);
}

static void sedi_can_handle_hight_priority_message(enum can_id id,
						   uint32_t status)
{
	uint32_t hpms;
	enum can_fifo fifo_id = CAN_FIFO_INVALID;
	struct can_config_t *mcan_config = &can_config[id];
	struct can_regs_t *can_regs = mcan_config->regs;

	hpms = (status & CAN_HPMS_MSI_MASK) >> CAN_HPMS_MSI_POS;

	/*Check mode of working */
	if (status & CAN_HPMS_MSI_FIFO_0) {
		fifo_id = CAN_FIFO_0;
	} else if (status & CAN_HPMS_MSI_FIFO_1) {
		fifo_id = CAN_FIFO_1;
	} else if (status & CAN_HPMS_MSI_LOST) {
		/* Message lost */
		mcan_config->cb(id, CAN_RX_FIFO_LOST, NULL);
		can_regs->ir |= CAN_IR_HPM;
		return;
	}
	if (sedi_can_receive_message(id, fifo_id, CAN_INVALID_BUFNUM)) {
		return;
	}
	can_regs->ir |= CAN_IR_HPM;
}
/**
 * @brief : get message for RX buffer.
 * @param[in] id: Id for CAN instance
 * @param[in] val: value of ndata 1 or ndata2 register
 * get buffer index for RX buffer.
 *
 * @return: none
 */
static void sedi_get_msg_from_rx_buf(IN enum can_id id, INOUT uint8_t val)
{
	uint8_t buf_num = 0;

	if (val) {
		while (val) {
			buf_num++;
			val = val >> 1;
		}
		sedi_can_receive_message(id, CAN_FIFO_INVALID, buf_num - 1);
	}
}
/**
 * @brief : Handle RX event from ISR.
 * @param[in] id: Id for CAN instance
 * @param[in] val: value to identify the event type
 * Handle RX event from ISR.
 *
 * @return: None
 */

static void sedi_can_handle_rx_event(IN enum can_id id, IN uint32_t val)
{
	struct can_config_t *mcan_config = &can_config[id];
	struct can_regs_t *can_regs = mcan_config->regs;

	if (val & CAN_IR_RF0N) {
		/* RX FIFO 0 */
		sedi_can_receive_message(id, CAN_FIFO_0, CAN_INVALID_BUFNUM);
	}
	if (val & CAN_IR_RF1N) {
		/* RX FIFO 1 */
		sedi_can_receive_message(id, CAN_FIFO_1, CAN_INVALID_BUFNUM);
	}
	if (val & CAN_IR_DRX) {
		/* RX dedicated buffer */
		sedi_get_msg_from_rx_buf(id, can_regs->ndata1);
		sedi_get_msg_from_rx_buf(id, can_regs->ndata2);
	}
	if ((val & CAN_IR_RF0L) || (val & CAN_IR_RF1L)) {
		/* FIFO MESSAGE LOST */
		mcan_config->cb(id, CAN_RX_FIFO_LOST, 0);
	}
	can_regs->ir &= (val & CAN_IR_RX_EVT);
}

/**
 * @brief : Interrupt service routine for errors/tx/rx/state change events.
 * @param[in] id: Id for CAN instance
 *
 * Interrupt service routine for errors/tx/rx/state change events and notify
 *  event vai callback.
 *
 * @return: None
 */

void sedi_can_isr(IN enum can_id id)
{
	uint32_t interrupt_reg, mask, status, masked_int;
	uint32_t parity_status;

	if (id >= CAN_MAX_INSTANCE) {
		return;
	}
	struct can_config_t *mcan_config = &can_config[id];
	struct can_regs_t *can_regs = mcan_config->regs;
	struct parity_ctrl_regs_t *parity_ctrl;
	struct parity_stat_regs_t *parity_stat;

	parity_stat = mcan_config->parity_stat_regs;
	parity_ctrl = mcan_config->parity_ctrl_regs;
	interrupt_reg = can_regs->ir;
	mask = can_regs->ie;
	masked_int = interrupt_reg & mask;
	parity_status = parity_stat->int_stat;
	can_regs->ile &= ~(BIT(id));

	/* Parity error interrupt */
	if (parity_status & PARITY_INT_STAT_ERR) {
		sedi_can_handle_parity_err(id, parity_ctrl);
	}

	/*Event triggered interrupts */

	/* Check for error interrupts */
	if (masked_int & CAN_IR_ERR) {
		sedi_can_handle_error(id, masked_int);
	}

	/* High priority message interrupt */
	if (masked_int & CAN_IR_HPM) {
		status = can_regs->hpms;
		sedi_can_handle_hight_priority_message(id, status);
	}
	/* RX interrupt */
	if (masked_int & CAN_IR_RX_EVT) {
		sedi_can_handle_rx_event(id, masked_int);
	}
	/*TX interrupt */
	if (masked_int & CAN_IR_TX_EVT) {
		sedi_can_handle_tx_event(id, masked_int);
	}
	can_regs->ile |= (BIT(id));
}

static int sedi_can_get_proto_err(uint32_t lec)
{
	uint32_t err;

	switch (lec) {
	case CAN_PSR_LEC_NO_ERROR:
		err = CAN_PROTO_ERR_NONE;
	case CAN_PSR_LEC_STUFF_ERROR:
		err = CAN_PROTO_ERR_STUFF;
		break;
	case CAN_PSR_LEC_FORM_ERROR:
		err = CAN_PROTO_ERR_FORMAT;
		break;
	case CAN_PSR_LEC_ACK_ERROR:
		err = CAN_PROTO_ERR_ACK;
		break;
	case CAN_PSR_LEC_BIT1_ERROR:
		err = CAN_PROTO_ERR_BIT_1;
		break;
	case CAN_PSR_LEC_BIT0_ERROR:
		err = CAN_PROTO_ERR_BIT_0;
		break;
	case CAN_PSR_LEC_CRC_ERROR:
		err = CAN_PROTO_ERR_CRC;
		break;
	default:
		err = CAN_PROTO_ERR_UNKNOWN;
	}
	return err;
}

/**
 * @brief : Handles error in CAN operations and protocol.
 * @param[in] id: Id for CAN instance
 * @param[in] val: status value received from interrupt status register.
 *
 * Handles error in CAN operations and protocol.
 *
 * @return: None
 */

static void sedi_can_handle_error(IN enum can_id id, IN uint32_t val)
{
	struct can_config_t *mcan_config = &can_config[id];
	struct can_regs_t *can_regs = mcan_config->regs;
	uint32_t psr = can_regs->psr;
	uint32_t proto_err;

	if (val & CAN_IR_ERR) {
		if (val & CAN_IR_TOO) {
			mcan_config->cb(id, CAN_ERR_TIMEOUT, 0);
		}

		if (val & CAN_IR_ARA) {
			mcan_config->cb(id, CAN_ERR_ACCESS_TO_RESERVED, 0);
		}
		if (val & CAN_IR_PED) {
			uint32_t dlec =
				(psr & CAN_PSR_DLEC_MASK) >> CAN_PSR_DLEC_POS;
			proto_err = sedi_can_get_proto_err(dlec);
			mcan_config->cb(id, CAN_ERR_PROT_DATA,
					(void *)&proto_err);
		}
		if (val & CAN_IR_PEA) {
			uint32_t lec =
				(psr & CAN_PSR_LEC_MASK) >> CAN_PSR_LEC_POS;
			proto_err = sedi_can_get_proto_err(lec);
			mcan_config->cb(id, CAN_ERR_PROT_ARBITRATION,
					(void *)&proto_err);
		}
		if (val & CAN_IR_BEU) {
			mcan_config->cb(id, CAN_ERR_BIT, 0);
		}
		if (val & CAN_IR_BO) {
			mcan_config->cb(id, CAN_ERR_BUS_OFF, 0);
			sedi_can_handle_state_change(id, CAN_STATE_BUS_OFF);
		}
		if (val & CAN_IR_ELO) {
			mcan_config->cb(id, CAN_ERR_LOG_OVERFLOW, 0);
		}

		if (val & CAN_IR_MRAF) {
			/* Reset if restricted mode activated */
			if (can_regs->cccr & CAN_CCCR_ASM_RESTRICTED) {
				can_regs->cccr &= ~CAN_CCCR_ASM_RESTRICTED;
			}

			mcan_config->cb(id, CAN_ERR_MSG_RAM_FAILURE, 0);
		}

		if (val & CAN_IR_EW) {
			mcan_config->cb(id, CAN_ERR_WARNING, 0);
			if (psr & CAN_PSR_EW) {
				sedi_can_handle_state_change(
					id, CAN_STATE_ERR_WARNING);
			} else {
				sedi_can_handle_state_change(
					id, CAN_STATE_ERR_ACTIVE);
			}
		}

		if (val & CAN_IR_EP) {
			mcan_config->cb(id, CAN_ERR_PASSIVE, 0);
			if (psr & CAN_PSR_EP) {
				sedi_can_handle_state_change(
					id, CAN_STATE_ERR_PASSIVE);
				if (psr & CAN_PSR_ACT_TRANSMITTER) {
					can_regs->txbcr |= can_regs->txbrp;
					while (can_regs->txbrp != 0) {
					}
					can_regs->ir &=
						(CAN_IR_PEA | CAN_IR_PED);
				}
			} else {
				sedi_can_handle_state_change(
					id, CAN_STATE_ERR_WARNING);
			}
		}
		can_regs->ir &= (val & CAN_IR_ERR);
	}
}

/**
 * @brief : Handles transmission events.
 * @param[in] id: Id for CAN instance
 * @param[in] val: status value received from interrupt status register.
 *
 * Handles tx events in CAN operations and protocol.
 *
 * @return: None
 */

static void sedi_can_handle_tx_event(IN enum can_id id, IN uint32_t val)
{
	struct can_config_t *mcan_config = &can_config[id];
	struct can_regs_t *can_regs = mcan_config->regs;
	struct can_msg_ram_t *msg_ram = mcan_config->msg_ram;
	uint32_t get_index, buf_num;
	__IO_RW uint32_t *evtbuffer;

	if (val & CAN_IR_TEFL) {
		/* TX Event message lost */
		mcan_config->cb(id, CAN_TX_EVT_LOST, 0);
	}
	if (val & CAN_IR_TEFF) {
		/* TX Event full */
		mcan_config->cb(id, CAN_TX_EVT_FULL, 0);
	}
	if (val & CAN_IR_TEFW) {
		/* TX Event FIFO Water Mark reached */
		mcan_config->cb(id, CAN_TX_EVT_WM, 0);
	}
	if (val & CAN_IR_TEFN) {
		/* new element in TX Event FIFO */
		get_index = (can_regs->txefs & CAN_TXEFS_EFGI_MASK) >>
			    CAN_TXEFS_EFGI_POS;
		evtbuffer =
			msg_ram->tx_evt_fifo +
			(get_index * (CAN_TX_EVT_ELEM_SIZE / CAN_MEM_WORD_SIZE));

		mcan_config->cb(id, CAN_TX_EVT_OCCURRED, (uint32_t *)evtbuffer);
		can_regs->txefa |= get_index;
	}
	if (val & CAN_IR_TCF) {
		/* TX cancellation completed */
		buf_num = can_regs->txbcf;
		mcan_config->cb(id, CAN_TX_CANCELED, &buf_num);
	}
	if (val & CAN_IR_TC) {
		/* TX completed */
		buf_num = can_regs->txbto;
		mcan_config->cb(id, CAN_TX_OCCURRED, &buf_num);
	}
	can_regs->ir &= (val & CAN_IR_TX_EVT);
}

/**
 * @brief : Handles state changes in CAN operations and protocol.
 * @param[in] id: Id for CAN instance
 * @param[in] state: new state value for CAN protocol.
 *
 * Handles state changes in CAN operations and protocol.
 *
 * @return: None
 */

static void sedi_can_handle_state_change(IN enum can_id id,
					 IN enum can_states state)
{
	struct can_config_t *mcan_config = &can_config[id];

	switch (state) {
	case CAN_STATE_ERR_WARNING:
		mcan_config->state = CAN_STATE_ERR_WARNING;
		break;
	case CAN_STATE_BUS_MONITORING:
		mcan_config->state = CAN_STATE_BUS_MONITORING;
		break;
	case CAN_STATE_BUS_OFF:
		mcan_config->state = CAN_STATE_BUS_OFF;
		sedi_can_reset(id);
		sedi_can_set_interrupts(id, (enum can_intr_line)id);
		break;
	case CAN_STATE_ERR_ACTIVE:
		mcan_config->state = CAN_STATE_ERR_ACTIVE;
		break;
	case CAN_STATE_ERR_PASSIVE:
		mcan_config->state = CAN_STATE_ERR_PASSIVE;
		break;
	case CAN_STATE_STOP:
		mcan_config->state = CAN_STATE_STOP;
		break;
	}
}

/**
 * @brief : Control CAN IP by enabling and disabling CAN internal clock.
 * @param[in] id: Id for CAN instance
 * @param[in] enable: 1 or 0 for enabling and disabling CAN operations.
 *
 * Control CAN IP by enabling and disabling CAN internal clock.
 *
 * @return: None
 */

void sedi_can_power_control(IN enum can_id id, IN uint8_t enable)
{
	struct can_config_t *mcan_config = &can_config[id];
	struct can_regs_t *can_regs = mcan_config->regs;
	uint32_t val, count = TIMER_LIMIT;

	if (!enable) {
		can_regs->cccr |= CAN_CCCR_CSR;
		do {
			val = can_regs->cccr;
			count--;
		} while (!(val & CAN_CCCR_CSA) && count);
	} else {
		can_regs->cccr &= ~CAN_CCCR_CSR;
		do {
			val = can_regs->cccr;
			count--;
		} while ((val & CAN_CCCR_CSA) && count);
		sedi_can_reset(id);
	}
}

/**
 * @brief : Get current value of clock frequency.
 *
 * Get current value of clock frequency.
 *
 * @return: value of clock frequency.
 */

uint32_t sedi_can_get_clock(void)
{
	return CAN_CLK_FREQ_HZ;
}

/**
 * @brief : Get current state for CAN protocol.
 *
 * Control  Get current value of clock freqency.
 *
 * @return: current state of CAN protocol.
 */

uint32_t sedi_can_get_status(IN enum can_id id)
{
	return can_config[id].state;
}

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
				    struct can_err_cnt *err_cnt)
{
	DBG_CHECK((CAN_0 == id || CAN_1 == id), SEDI_DRIVER_ERROR_PARAMETER);
	struct can_config_t *mcan_config = &can_config[id];
	struct can_regs_t *can_regs = mcan_config->regs;
	uint32_t val = can_regs->ecr;

	if (err_cnt != NULL) {
		err_cnt->rx_err_cnt = (uint8_t)((val & CAN_ECR_REC_MASK) >>
						CAN_ECR_REC_POS);
		err_cnt->tx_err_cnt = (uint8_t)((val & CAN_ECR_TEC_MASK) >>
						CAN_ECR_TEC_POS);
	}
	return can_config[id].state;
}

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
				   IN uint8_t enable)
{
	DBG_CHECK((CAN_0 == id || CAN_1 == id), SEDI_DRIVER_ERROR_PARAMETER);
	struct can_config_t *mcan_config = &can_config[id];
	struct can_regs_t *can_regs = mcan_config->regs;
	struct parity_ctrl_regs_t *parity_ctrl = mcan_config->parity_ctrl_regs;
	int32_t ret = SEDI_DRIVER_OK;

	sedi_can_config_init_start(can_regs);
	if (!enable) {
		parity_ctrl->einj_ctl_stat &= ~(PARITY_CTL_ERR_INJ_ENABLE);
		goto err;
	}
	if (err == NULL) {
		ret = SEDI_DRIVER_ERROR_PARAMETER;
		goto err;
	}

	parity_ctrl->einj_offset = PARITY_CTL_INJ_OFFSET(err->einj_offset);

	parity_ctrl->einj_data_mask = err->einj_data_mask;

	parity_ctrl->einj_parity_mask =
		PARITY_CTL_INJ_PAR(err->einj_parity_mask);

	if (err->einj_mode == CAN_PARITY_EINJ_ONE_TIME) {
		parity_ctrl->einj_ctl_stat = (PARITY_CTL_ERR_INJ_ENABLE |
					      PARITY_CTL_ERR_INJ_MODE_ONETIME);
	} else {
		parity_ctrl->einj_ctl_stat =
			(PARITY_CTL_ERR_INJ_ENABLE |
			 PARITY_CTL_ERR_INJ_MODE_CONTINIOUS);
	}

err:
	sedi_can_config_init_finished(can_regs);
	return ret;
}

static void sedi_can_save_regs(enum can_id id)
{
	can_pm_regs_t *pm_ctxt = &can_pm_context[id];
	struct can_config_t *mcan_config = &can_config[id];
	struct can_regs_t *can_regs = mcan_config->regs;
	struct can_params_t *params = mcan_config->params;
	struct can_msg_ram_t *msg_ram = mcan_config->msg_ram;
	struct parity_ctrl_regs_t *parity_ctrl = mcan_config->parity_ctrl_regs;
	struct parity_stat_regs_t *parity_stat = mcan_config->parity_stat_regs;

	pm_ctxt->cust = can_regs->cust;
	pm_ctxt->dbtp = can_regs->dbtp;
	pm_ctxt->test = can_regs->test;
	pm_ctxt->rwd = can_regs->rwd;
	pm_ctxt->cccr = can_regs->cccr;
	pm_ctxt->nbtp = can_regs->nbtp;
	pm_ctxt->tscc = can_regs->tscc;
	pm_ctxt->tscv = can_regs->tscv;
	pm_ctxt->tocc = can_regs->tocc;
	pm_ctxt->tocv = can_regs->tocv;
	pm_ctxt->ie = can_regs->ie;
	pm_ctxt->ils = can_regs->ils;
	pm_ctxt->ile = can_regs->ile;
	pm_ctxt->gfc = can_regs->gfc;
	pm_ctxt->sidfc = can_regs->sidfc;
	pm_ctxt->xidfc = can_regs->xidfc;
	pm_ctxt->xidam = can_regs->xidam;
	pm_ctxt->rxf0c = can_regs->rxf0c;
	pm_ctxt->rxbc = can_regs->rxbc;
	pm_ctxt->rxf1c = can_regs->rxf1c;
	pm_ctxt->rxesc = can_regs->rxesc;
	pm_ctxt->txbc = can_regs->txbc;
	pm_ctxt->txesc = can_regs->txesc;
	pm_ctxt->txbtie = can_regs->txbtie;
	pm_ctxt->txbcie = can_regs->txbcie;
	pm_ctxt->txefc = can_regs->txefc;

	/* Save filter */
	for (int i = 0; i < params->std_filts_cnt; i++) {
		pm_ctxt->std_filter[i] = msg_ram->std_filters[i];
	}

	/* Ext filter */
	for (int i = 0; i < params->ext_filts_cnt * 2; i++) {
		pm_ctxt->ext_filter[i] = msg_ram->ext_filters[i];
	}

	/**
	 * Parity control registers
	 */
	pm_ctxt->csr = parity_ctrl->csr;
	pm_ctxt->einj_ctl_stat = parity_ctrl->einj_ctl_stat;
	pm_ctxt->einj_offset = parity_ctrl->einj_offset;
	pm_ctxt->einj_data_mask = parity_ctrl->einj_data_mask;
	pm_ctxt->einj_parity_mask = parity_ctrl->einj_parity_mask;

	/**
	 * Parity stat registers
	 */
	pm_ctxt->ctl = parity_stat->ctl;
	pm_ctxt->int_ctl = parity_stat->int_ctl;

	pm_ctxt->ctxt_valid = true;
}

static void sedi_can_restore_regs(enum can_id id)
{
	can_pm_regs_t *pm_ctxt = &can_pm_context[id];
	struct can_config_t *mcan_config = &can_config[id];
	struct can_regs_t *can_regs = mcan_config->regs;
	struct can_params_t *params = mcan_config->params;
	struct can_msg_ram_t *msg_ram = mcan_config->msg_ram;
	struct parity_ctrl_regs_t *parity_ctrl = mcan_config->parity_ctrl_regs;
	struct parity_stat_regs_t *parity_stat = mcan_config->parity_stat_regs;

	if (pm_ctxt->ctxt_valid == true) {
		sedi_can_config_init_start(can_regs);
		can_regs->cust = pm_ctxt->cust;
		can_regs->dbtp = pm_ctxt->dbtp;
		can_regs->test = pm_ctxt->test;
		can_regs->rwd = pm_ctxt->rwd;
		can_regs->nbtp = pm_ctxt->nbtp;
		can_regs->tscc = pm_ctxt->tscc;
		can_regs->tscv = pm_ctxt->tscv;
		can_regs->tocc = pm_ctxt->tocc;
		can_regs->tocv = pm_ctxt->tocv;
		can_regs->ie = pm_ctxt->ie;
		can_regs->ils = pm_ctxt->ils;
		can_regs->ile = pm_ctxt->ile;
		can_regs->gfc = pm_ctxt->gfc;
		can_regs->sidfc = pm_ctxt->sidfc;
		can_regs->xidfc = pm_ctxt->xidfc;
		can_regs->xidam = pm_ctxt->xidam;
		can_regs->rxf0c = pm_ctxt->rxf0c;
		can_regs->rxbc = pm_ctxt->rxbc;
		can_regs->rxf1c = pm_ctxt->rxf1c;
		can_regs->rxesc = pm_ctxt->rxesc;
		can_regs->txbc = pm_ctxt->txbc;
		can_regs->txesc = pm_ctxt->txesc;
		can_regs->txbtie = pm_ctxt->txbtie;
		can_regs->txbcie = pm_ctxt->txbcie;
		can_regs->txefc = pm_ctxt->txefc;

		if (pm_ctxt->cccr & CAN_CCCR_TEST_ENABLED) {
			can_regs->cccr |= CAN_CCCR_TEST_ENABLED;
			can_regs->test = pm_ctxt->test;
		}

		/* Restore std filter */
		for (int i = 0; i < params->std_filts_cnt; i++) {
			msg_ram->std_filters[i] = pm_ctxt->std_filter[i];
		}

		/* Restore ext filter */
		int size = (params->ext_filts_cnt * CAN_WORDS_PER_EXT_FITER);

		for (int i = 0; i < size; i++) {
			msg_ram->ext_filters[i] = pm_ctxt->ext_filter[i];
		}

		/**
		 * Parity control registers
		 */
		parity_ctrl->csr = pm_ctxt->csr;
		parity_ctrl->einj_ctl_stat = pm_ctxt->einj_ctl_stat;
		parity_ctrl->einj_offset = pm_ctxt->einj_offset;
		parity_ctrl->einj_data_mask = pm_ctxt->einj_data_mask;
		parity_ctrl->einj_parity_mask = pm_ctxt->einj_parity_mask;

		/**
		 * Parity stat registers
		 */
		parity_stat->ctl = pm_ctxt->ctl;
		parity_stat->int_ctl = pm_ctxt->int_ctl;

		pm_ctxt->ctxt_valid = false;
		can_regs->cccr = pm_ctxt->cccr;

		sedi_can_config_init_finished(can_regs);
	}
}

int32_t sedi_can_set_power(IN enum can_id id, IN sedi_power_state_t state)
{

	DBG_CHECK((id < CAN_ID_MAX), SEDI_DRIVER_ERROR_PARAMETER);

	driver_id_t driver_id = DRIVER_ID_CAN0 + id;
	int32_t ret = SEDI_DRIVER_OK;

	switch (state) {
	case SEDI_POWER_FULL:
		/* Enable clocks*/
		pm_driver_start_trans(driver_id);
		sedi_can_restore_regs(id);
		break;
	case SEDI_POWER_SUSPEND:
		sedi_can_save_regs(id);
		pm_driver_end_trans(driver_id);
		break;

	case SEDI_POWER_FORCE_SUSPEND:
		pm_driver_end_trans(driver_id);
		break;

	case SEDI_POWER_LOW:
		/* Clock gating for CAN */
		pm_driver_end_trans(driver_id);
		break;
	case SEDI_POWER_OFF:
		ret = SEDI_DRIVER_ERROR_UNSUPPORTED;
		break;
	}
	return ret;
}
