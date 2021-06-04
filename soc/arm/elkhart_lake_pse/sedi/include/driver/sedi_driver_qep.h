/*
 * Copyright (c) 2021 Intel Corporation
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _SEDI_DRIVER_QEP_H_
#define _SEDI_DRIVER_QEP_H_

#include "sedi_driver_common.h"

/**
 * Error Codes for QEP
 */
typedef enum {
	SEDI_QEP_OK             = 0,
	SEDI_QEP_INVALID_MODE   = BIT(0),
	SEDI_QEP_WDT_DETECTED   = BIT(1),
	SEDI_QEP_DIR_CHANGE     = BIT(2),
	SEDI_QEP_RST_UP         = BIT(3),
	SEDI_QEP_RST_DN         = BIT(4),
	SEDI_QEP_CAP_DONE       = BIT(5),
	SEDI_QEP_CAP_CANCELLED  = BIT(6),
	SEDI_QEP_PH_ERR         = BIT(7)

} sedi_qep_event_t;

/**
 * Edge selection for input signal.
 */
typedef enum {
	SEDI_QEP_FALLING_EDGE,  /**< Sample at falling edge for input. */
	SEDI_QEP_RISING_EDGE    /**< Sample at rising edge for input. */
} sedi_qep_edge_select_t;

/**
 * Direction of rotation.
 */
typedef enum {
	SEDI_QEP_DIR_CW,   /**< Clockwise direction of rotation. */
	SEDI_QEP_DIR_CCW,  /**< Counter clockwise direction of rotation. */
	SEDI_QEP_DIR_UNKNOWN /**<  Direction of rotation is unknown */
} sedi_qep_dir_t;

/**
 * Operating mode for QEP
 */
typedef enum {
	SEDI_QEP_MODE_QEP_DECODER,      /**< QEP decoding mode. */
	SEDI_QEP_MODE_CAPTURE           /**< Capture mode. */
} sedi_qep_mode_t;

/**
 * Counter reset mode for QEP
 */
typedef enum {
	SEDI_QEP_COUNTER_RESET_ON_INDEX,        /**< Reset on index pulse. */
	SEDI_QEP_COUNTER_RESET_ON_MAX_COUNT     /**< Reset on max count. */
} sedi_qep_ctr_rst_mode_t;

/**
 * Index gating selection
 */

typedef enum {
	SEDI_QEP_PHASE_A_LOW_PHASE_B_LOW,       /**< Phase A low and Phase B low. */
	SEDI_QEP_PHASE_A_LOW_PHASE_B_HIGH,      /**< Phase A low and Phase B high. */
	SEDI_QEP_PHASE_A_HIGH_PHASE_B_LOW,      /**< Phase A high and Phase B low. */
	SEDI_QEP_PHASE_A_HIGH_PHASE_B_HIGH      /**< Phase A high and Phase B high. */
} sedi_qep_idx_gate_sel_t;

/**
 *  QEP Capture Mode.
 */
typedef enum {
	SEDI_QEP_CAP_SINGLE_EDGE,       /**< Capture on single edge. */
	SEDI_QEP_CAP_BOTH_EDGE          /**< Capture on both edge. */
} sedi_qep_cap_config_t;

typedef void (*sedi_qep_callback_t)(IN void *param, uint32_t event,
				    uint32_t cap_len);

/**
 *  QEP Configuration Struct.
 */

typedef struct {
	/* Common controls for QEP and Capture Mode. */
	sedi_qep_mode_t mode;                   /* Mode of operation. */
	uint32_t swap_a_b;                      /* Swap inuts A and B. */
	sedi_qep_edge_select_t edge_phase_a;    /* Edge selection for Phase A. */
	sedi_qep_edge_select_t edge_phase_b;    /* Edge selection for Phase B. */
	uint32_t noise_filter_width_ns;         /* Width of noise filter in
						 * nanoseconds.
						 */
	uint32_t filter_en;                      /* Flag to enable or disable filter. */

	/* Controls for QEP mode only. */
	sedi_qep_edge_select_t edge_index;      /* Edge selection for Index. */
	sedi_qep_ctr_rst_mode_t ctr_rst_mode;   /* Count reset mode. */
	sedi_qep_idx_gate_sel_t gating_index;   /* Index gating. */
	uint32_t pulses_per_rev;                /*Pulses per rev. */
	uint32_t wdt_usec;                      /* Watchdog timeout in usec to detect stalls in QPE
						 * mode.
						 */
	bool wdt_en;                            /* Enable Watchdog timer. */

	/* Controls for Capture mode only. */
	sedi_qep_cap_config_t capture_edge; /* Capture mode - single of both
					     * edges.
					     */
} sedi_qep_config_t;

/**
 * @brief Configure qep
 *
 * This function configures a QEP instance with the inputs provided
 * in the configureation structure.
 *
 * @param[in] qep instance to be configured.
 * @param[in] config structure
 *
 * @retval SEDI_DRIVER_OK if operation was successful.
 * @retval non zero error code otherwise.
 */

int sedi_qep_config(IN sedi_qep_t qep, IN sedi_qep_config_t *config);

/**
 * @brief Configure qep noise filter
 *
 * This function configures the maximum width of the input
 * signals that is to be filtered by the noise filtering unit.
 *
 * @param[in] qep instance.
 * @param[in] maximum width of the noise to be filtered in nanoseconds.
 *
 * @retval SEDI_DRIVER_OK if operation was successful.
 * @retval non zero error code otherwise.
 */

int sedi_qep_filter_config(IN sedi_qep_t qep,
			   IN uint32_t max_glitch_width_ns);

/**
 * @brief Enable qep noise filter
 *
 * This function enables input noise filtering.
 *
 * @param[in]  qep instance.
 *
 * @retval SEDI_DRIVER_OK if operation was successful.
 * @retval non zero error code otherwise.
 */

int sedi_qep_filter_enable(IN sedi_qep_t qep);

/**
 * @brief Disable qep noise filter
 *
 * This function disables input noise filtering.
 *
 * @param[in]  qep instance.
 *
 * @retval SEDI_DRIVER_OK if operation was successful.
 * @retval non zero error code otherwise.
 */

int sedi_qep_filter_disable(IN sedi_qep_t qep);

/**
 * @brief Enable QEP
 *
 * This function enables QEP IP.
 *
 * @param[in]  qep instance.
 *
 * @retval SEDI_DRIVER_OK if operation was successful.
 * @retval non zero error code otherwise.
 */

int sedi_qep_enable(IN sedi_qep_t qep);

/**
 * @brief Disable QEP
 *
 * This function disables QEP IP.
 *
 * @param[in]  qep instance.
 *
 * @retval SEDI_DRIVER_OK if operation was successful.
 * @retval non zero error code otherwise.
 */

int sedi_qep_disable(IN sedi_qep_t qep);

/**
 * @brief Configure Watchdog Timer
 *
 * This function configures the watchdog timer with the specified timeout.
 *
 * @param[in]  qep instance.
 * @param[in]  timeout in microseconds.
 *
 * @retval SEDI_DRIVER_OK if operation was successful.
 * @retval non zero error code otherwise.
 */

int sedi_qep_wdt_config(IN sedi_qep_t qep, IN uint32_t timeout_usec);

/**
 * @brief Enable WDT
 *
 * This function enables QEP watchdog timer.
 *
 * @param[in]  qep instance.
 *
 * @retval SEDI_DRIVER_OK if operation was successful.
 * @retval non zero error code otherwise.
 */
int sedi_qep_wdt_enable(IN sedi_qep_t qep);

/**
 * @brief Disable WDT
 *
 * This function disables QEP watchdog timer.
 *
 * @param[in]  qep instance.
 *
 * @retval SEDI_DRIVER_OK if operation was successful.
 * @retval non zero error code otherwise.
 */

int sedi_qep_wdt_disable(IN sedi_qep_t qep);

/**
 * @brief Swap QEP A B signals
 *
 * This function swaps PhaseA and Phase B input signals of QEP
 *
 * @param[in]  qep instance.
 *
 * @retval SEDI_DRIVER_OK if operation was successful.
 * @retval non zero error code otherwise.
 */

int sedi_qep_swap_a_b(IN sedi_qep_t qep);

/**
 * @brief Get position count register value
 *
 * This function returns the current position count register value
 *
 * @param[in]  qep instance.
 * @param[inout] p_pos_count vairable to return the current position count.
 *
 * @retval SEDI_DRIVER_OK if operation was successful.
 * @retval non zero error code otherwise.
 */

int sedi_qep_get_position(IN sedi_qep_t qep, INOUT uint32_t *p_pos_count);

/**
 * @brief Get Direction
 *
 * This function returns the current position count register value
 *
 * @param[in]  qep instance.
 * @param[inout] Direction of rotation.
 *
 * @retval SEDI_DRIVER_OK if operation was successful.
 * @retval non zero error code otherwise.
 */

int sedi_qep_get_direction(IN sedi_qep_t qep,
			   INOUT sedi_qep_dir_t *direction);

/**
 * @brief Get Phase Error.
 *
 * This function checks if a phase error is detected between
 * phase A and Phase B input signals.
 *
 * @param[in]  qep instance.
 * @parma[out] p_ph_err pointer to uint32_t to hold err status.
 * ph_err is set to true if a phase error is detected otherwise flase.
 *
 * @retval SEDI_DRIVER_OK if operation was successful.
 * @retval non zero error code otherwise.
 */

int sedi_qep_get_phase_err(IN sedi_qep_t qep, OUT uint32_t *p_ph_err);

/**
 * @brief  QEP interrupt enable
 *
 * This function enables the interrupt for QEP
 * according to interrupt flags passed.
 *
 * @param[in] qep instance.
 * @param[in] flag  bitmask of interrupt flags to enable.
 *
 * @retval SEDI_DRIVER_OK if operation was successful.
 * @retval non zero error code otherwise.
 */
int sedi_qep_int_enable(IN sedi_qep_t qep, IN uint32_t flag);

/**
 * @brief  QEP interrupt disable
 *
 * This function disables the interrupt for QEP
 * according to interrupt flags passed.
 *
 * @param[in]  qep instance.
 * @param[in] flag  bitmask of interrupt flags to disable.
 *
 * @retval SEDI_DRIVER_OK if operation was successful.
 * @retval non zero error code otherwise.
 */

int sedi_qep_int_disable(IN sedi_qep_t qep, IN uint32_t flag);

/**
 * @brief  QEP clock enable
 *
 * This function enables clock for QEP.
 *
 * @param[in]  qep instance.
 *
 * @retval SEDI_DRIVER_OK if operation was successful.
 * @retval non zero error code otherwise.
 */

int sedi_qep_clk_enable(IN sedi_qep_t qep);

/**
 * @brief  QEP clock disable
 *
 * This function disables clock for QEP.
 *
 * @param[in]  qep instance.
 *
 * @retval SEDI_DRIVER_OK if operation was successful.
 * @retval non zero error code otherwise.
 */

int sedi_qep_clk_disable(IN sedi_qep_t qep);

/**
 * @brief  QEP start capture
 *
 * When QEP moduled is configured for capture mode, this function captures
 * the timestamps for the configured edge transition events in the capture
 * buffer. The capture timestamps are in nanoseconds.
 * This is an asynchrounous call and calls the user regsiterd callback
 * after completion of the capture.
 *
 * @param[in] qep instance.
 * @param[out] buff buffer to be populated with captured entries.
 * @param[in] count number of elements to capture.
 *
 * @retval SEDI_DRIVER_OK if operation was successful.
 * @retval non zero error code otherwise.
 */

int sedi_qep_start_cap(IN sedi_qep_t qep, OUT uint64_t *buff,
		       IN uint32_t count);

/**
 * @brief  QEP stop capture
 *
 * Stops an ongoing capture.
 *
 * @param[in] qep instance.
 *
 * @retval SEDI_DRIVER_OK if operation was successful.
 * @retval non zero error code otherwise.
 */

int sedi_qep_stop_cap(IN sedi_qep_t qep);

/**
 * @brief  QEP save context.
 *
 * Save context of QEP for power management support.
 *
 * @retval SEDI_DRIVER_OK if operation was successful.
 * @retval non zero error code otherwise.
 */

int sedi_qep_save_context(IN sedi_qep_t qep);

/**
 * @brief  QEP restore context.
 *
 * Restore context of QEP from saved context.
 *
 * @retval SEDI_DRIVER_OK if operation was successful.
 * @retval non zero error code otherwise.
 */

int sedi_qep_restore_context(IN sedi_qep_t qep);

/**
 * @brief  QEP register callback.
 *
 * @param[in] QEP instance.
 * @param[in] QEP callback function of type sedi_qep_callback_t.
 *
 * @retval SEDI_DRIVER_OK if operation was successful.
 * @retval non zero error code otherwise.
 */

int sedi_qep_register_callback(IN sedi_qep_t qep,
			       IN sedi_qep_callback_t callback,
			       IN void *usr_param);

/**
 * @brief  QEP start decoding
 *
 * When QEP is configured in qep decoding mode , this function enables the
 * qep module in qep decoding mode. Once decode is started, user notified
 * of events(watchdog timeout, direction change, counter reset)  through
 * registered callack.
 *
 * @param[in] qep instance.
 *
 * @retval SEDI_DRIVER_OK if operation was successful.
 * @retval non zero error code otherwise.
 */

int sedi_qep_start_decoding(sedi_qep_t qep);
/**
 * @brief  QEP stop decoding
 *
 * Stops any ongoing decoding operation for given qep instance and disables
 * QEP interrupts..
 *
 * @param[in] qep instance.
 *
 * @retval SEDI_DRIVER_OK if operation was successful.
 * @retval non zero error code otherwise.
 */
int sedi_qep_stop_decoding(sedi_qep_t qep);

/**
 * @brief  QEP disable qep decode event.
 *
 * Disables generation of events for QEP as specified by the
 * sedi_qep_event_t enum.
 * This api should be used only when QEP decoding mode is selected.
 * Otherwise error is returned.
 *
 * @param[in] qep instance.
 * @parma[in] event of type sedi_qep_event_t to be disabled.
 *
 * @retval SEDI_DRIVER_OK if operation was successful.
 * @retval non zero error code otherwise.
 */

int sedi_qep_disable_event(sedi_qep_t qep, sedi_qep_event_t event);

/**
 * @brief  QEP enable qep decode event.
 *
 * Enables generation of events for QEP decodig as specified by the
 * sedi_qep_event_t enum.
 *
 * This api should be used only when QEP decoding mode is selected.
 * Otherwise error is returned.
 *
 * @param[in] qep instance.
 * @parma[in] event of type sedi_qep_event_t to be enabled.
 *
 * @retval SEDI_DRIVER_OK if operation was successful.
 * @retval non zero error code otherwise.
 */

int sedi_qep_enable_event(sedi_qep_t qep, sedi_qep_event_t event);

/**
 *  QEP interrupt handler function.
 *
 * @param[in] QEP instance.
 *
 * @return void :No return value.
 */

void sedi_qep_int_handler(sedi_qep_t qep);

/**
 * @brief  QEP set power state.
 *
 * Set specified power state for QEP
 *
 * @param[in] qep instance.
 * @parma[in] power state to be entered.
 *
 * @retval SEDI_DRIVER_OK if operation was successful.
 * @retval non zero error code otherwise.
 */

int sedi_qep_set_power(IN sedi_qep_t qep, IN sedi_power_state_t state);

/**
 * @brief  QEP init.
 *
 * Initialize specified qep instance
 *
 * @param[in] qep instance.
 *
 * @retval SEDI_DRIVER_OK if operation was successful.
 * @retval non zero error code otherwise.
 */
int sedi_qep_init(IN sedi_qep_t qep);
#endif
