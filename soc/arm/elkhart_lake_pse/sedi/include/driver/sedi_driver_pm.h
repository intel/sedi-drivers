/*
 * Copyright (c) 2021 Intel Corporation
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _SEDI_DRIVER_PM_H_
#define _SEDI_DRIVER_PM_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "sedi_driver_common.h"
#include "sedi_driver_ipc.h"

/*!
 * \defgroup sedi_driver_pm PM
 * \ingroup sedi_driver
 */

#define SEDI_PM_API_VERSION SEDI_DRIVER_VERSION_MAJOR_MINOR(0, 1)

/*!
 * \enum sedi_pm_ioctl_t
 * \brief control option for PM driver
 * \ingroup sedi_driver_pm
 */
typedef enum {
	/*!
	 * Enable CCU_TCG_GBE1_RGMII_PHY_RX and
	 * CCU_TCG_GBE2_RGMII_PHY_RX trunk clock
	 */
	SEDI_PM_IOCTL_TCG_GBE_RGMII_PHY_RX,
	/*!
	 * OOB support, to keep PSE alive when
	 * host reboots or shutdowns
	 * arg: zero to disable, none-zero to enable
	 */
	SEDI_PM_IOCTL_OOB_SUPPORT,
	/*!
	 * WOL support, D0i3 would not be available
	 * arg: zero to disable, none-zero to enable
	 */
	SEDI_PM_IOCTL_WOL_SUPPORT,
	SEDI_PM_IOCTL_MAX
} sedi_pm_ioctl_t;

/*!
 * \enum sedi_clock_frequency_t
 * \brief Clock frequency for ARM core and HBW
 * \ingroup sedi_driver_pm
 */
typedef enum {
	CLOCK_FREQ_100M,        /**< 100MHz S0i3 clock */
	CLOCK_FREQ_200M,
	CLOCK_FREQ_400M,        /**< 400MHz main clock */
	CLOCK_FREQ_500M,
} sedi_clock_frequency_t;

/*!
 * \enum vnn_id_t
 * \brief VNN ID bit for different device
 * \ingroup sedi_driver_pm
 */
typedef enum {
	FIRST_VNN_ID            = 0,
	VNN_ID_AON_TASK         = FIRST_VNN_ID,
	VNN_ID_IPC_PMC_W        = 1,    /**< DO NOT CHANGE. USED BY ROM. */
	VNN_ID_IPC_CSE_W        = 2,    /**< DO NOT CHANGE. USED BY ROM. */
	VNN_ID_IPC_HOST_W       = 3,    /**< DO NOT CHANGE. USED BY ROM. */
	VNN_ID_DMA0             = 4,
	VNN_ID_DMA1             = 5,
	VNN_ID_DMA2             = 6,
	VNN_ID_AON              = 8, /**< DO NOT CHANGE. USED BY ROM. */
	VNN_ID_IPC_PMC_R        = 9,
	VNN_ID_IPC_CSE_R        = 10,
	VNN_ID_IPC_HOST_R       = 11,
	VNN_ID_EXC_LOG          = 17,
	VNN_ID_SRAM             = 19,
	VNN_ID_HIGH_FREQUENCY   = 20,
	VNN_ID_SIDEBAND         = 21,
	VNN_ID_ROM_DEBUG        = 22, /**< DO NOT CHANGE. USED BY ROM. */
	VNN_ID_GBE0             = 23,
	VNN_ID_GBE1             = 24,
	VNN_ID_PM               = 25, /**< for host owned devices */
	VNN_ID_BRIDGE           = 26,
	LAST_VNN_ID,
} vnn_id_t;

/*!
 * \enum sedi_wake_event_type_t
 * \brief Wake event types
 * \ingroup sedi_driver_pm
 */
typedef enum {
	/**< UART event. */
	PM_WAKE_EVENT_UART,
	/**< I2C event. */
	PM_WAKE_EVENT_I2C,
	/**< SPI event. */
	PM_WAKE_EVENT_SPI,
	/**< SRAM event. */
	PM_WAKE_EVENT_SRAM_ERASE,
	/**< QEP event. */
	PM_WAKE_EVENT_QEP,
	/**< PWM event. */
	PM_WAKE_EVENT_PWM,
	/**< ADC event. */
	PM_WAKE_EVENT_ADC,
	/**< CAN event. */
	PM_WAKE_EVENT_CAN,
	/**< ETHR event. */
	PM_WAKE_EVENT_ETHR,
	/**< DMA event. */
	PM_WAKE_EVENT_DMA,
	/**< HPET event. */
	PM_WAKE_EVENT_HPET,
	/**< VNN ACK RISE event. */
	PM_WAKE_EVENT_VNN_ACK_RISE,
	/**< VNN ACK FALL event. */
	PM_WAKE_EVENT_VNN_ACK_FALL,
	/**< SBEP event. */
	PM_WAKE_EVENT_SBEP,
	/**< HOST IPC event. */
	PM_WAKE_EVENT_HOSTIPC,
	/**< PME status clear event */
	PM_WAKE_EVENT_PME_STATUS_CLEAR,
	/**< PCE change detect event */
	PM_WAKE_EVENT_PCE_CHG_DETECT,
	/**< Host reset rise event */
	PM_WAKE_EVENT_HOST_RESET_RISE,
	/**< Host reset fall event */
	PM_WAKE_EVENT_HOST_RESET_FALL,
	/**< Bridge isolation ack fall event */
	PM_WAKE_EVENT_BRIDGE_ISOL_ACK_FALL,
	/**< Bridge isolation ack rise event */
	PM_WAKE_EVENT_BRIDGE_ISOL_ACK_RISE,
	/**< PMC PG wake fall event */
	PM_WAKE_EVENT_PMC_PG_WAKE_FALL,
	/**< PMC PG wake rise event */
	PM_WAKE_EVENT_PMC_PG_WAKE_RISE,
	/**< PMC PG REQ fall event */
	PM_WAKE_EVENT_PMC_PG_REQ_FALL,
	/**< PMC PG REQ rise event */
	PM_WAKE_EVENT_PMC_PG_REQ_RISE,
	/**< PMC reset prepare available event */
	PM_WAKE_EVENT_RESET_PREPARE_AVAIL,
	/**< PCE D3 Rise event. */
	PM_WAKE_EVENT_PCI_D3_RISE,
	/**< PCE D3 fall event. */
	PM_WAKE_EVENT_PCI_D3_FALL,
	/**< PCE BME Rise event. */
	PM_WAKE_EVENT_PCI_BME_RISE,
	/**< PCE BME fall event. */
	PM_WAKE_EVENT_PCI_BME_FALL,
	/**< PCE D3 event. */
	PM_WAKE_EVENT_PCI_D3,
	/**< GPIO event. */
	PM_WAKE_EVENT_GPIO,
	PM_WAKE_EVENT_MAX,
} sedi_wake_event_type_t;

/*!
 * \enum sedi_global_reset_type_t
 * \brief Global reset types
 * \ingroup sedi_driver_pm
 */
typedef enum {
	SW_RESET,
	MIA_SHUTDOWN,
	FABRIC_INTR,
	FABRIC_UR
} sedi_global_reset_type_t;

/*!
 * \enum sedi_wake_event_instance_t
 * \brief Wake event instance number, represents controller ID
 * for I2C, SPI, QEP, CAN and DMA controller, GPIO pin number or
 * PCI function number. Be ignored for other wake event types
 * \ingroup sedi_driver_pm
 */
typedef union {
	uint32_t per_inst;      /**< Instance number for some peripheral */
	uint32_t gpio_pin;      /**< GPIO Pin number. */
	uint32_t pci_func;      /**< PCI Function number. */
} sedi_wake_event_instance_t;

/*!
 * \enum sedi_callback_priority_t
 * \brief Priority of callback function
 * \ingroup sedi_driver_pm
 */
typedef enum {
	CALLBACK_PRI_LOW        = 1,
	CALLBACK_PRI_NORMAL,
	CALLBACK_PRI_HIGH,
	CALLBACK_PRI_MAX        = 0x4
} sedi_pm_callback_priority_t;

/*!
 * \enum sedi_pm_callback_type_t
 * \brief Type of callback function
 * \ingroup sedi_driver_pm
 */
typedef enum {
	CALLBACK_TYPE_RESET_PREP = 1,
	CALLBACK_TYPE_CLOCK_CHANGE,
	CALLBACK_TYPE_MAX
} sedi_pm_callback_type_t;

/*!
 * \enum sedi_pm_sx_event_t
 * \brief Type of host Sx event
 * \ingroup sedi_driver_pm
 */
typedef enum {
	PM_EVENT_HOST_SX_ENTRY, /* Host entered Sx state. */
	PM_EVENT_HOST_SX_EXIT,  /* Host exit Sx state. */
} sedi_pm_sx_event_t;

/*!
 * \enum sedi_pm_d3_event_t
 * \brief Type of host D3 event
 * \ingroup sedi_driver_pm
 */
typedef enum {
	PM_EVENT_HOST_D3_ENTRY, /* Host entered D3 state. */
	PM_EVENT_HOST_D3_EXIT,  /* Host exit D3 state. */
} sedi_pm_d3_event_t;

/*!
 * \enum sedi_pm_s0ix_event_t
 * \brief Type of host S0ix event
 * \ingroup sedi_driver_pm
 */
typedef enum {
	PM_EVENT_HOST_S0IX_ENTRY,       /* Host entered S0ix state. */
	PM_EVENT_HOST_S0IX_EXIT,        /* Host exit S0ix state. */
} sedi_pm_s0ix_event_t;

/*!
 * \enum sedi_pm_trunk_clock_t
 * \brief Type of trunk clock
 * \ingroup sedi_driver_pm
 */
typedef enum {
	PM_TRUNK_CLOCK_MIN              = 4,
	PM_TRUNK_CLOCK_ADC_HIP          = PM_TRUNK_CLOCK_MIN,
	PM_TRUNK_CLOCK_ADC_CTRL,
	PM_TRUNK_CLOCK_PLL_PTP,
	PM_TRUNK_CLOCK_GBE1_RGMII_TX    = 8,
	PM_TRUNK_CLOCK_GBE2_RGMII_TX,
	PM_TRUNK_CLOCK_GBE1_SGMII_REF,
	PM_TRUNK_CLOCK_GBE2_SGMII_REF,
	PM_TRUNK_CLOCK_GBE1_SGMII_PHY_RX,
	PM_TRUNK_CLOCK_GBE2_SGMII_PHY_RX,
	PM_TRUNK_CLOCK_GBE1_RGMII_PHY_RX,
	PM_TRUNK_CLOCK_GBE2_RGMII_PHY_RX,
	PM_TRUNK_CLOCK_MAX,
} sedi_pm_trunk_clock_t;

/*!
 * \enum sedi_pm_reset_type_t
 * \brief Type of reset type
 * \ingroup sedi_driver_pm
 */
typedef enum {
	PM_RESET_TYPE_S0                = 0x0,  /* S0 */
	PM_RESET_TYPE_S3                = 0x3,  /* S3 */
	PM_RESET_TYPE_S4,                       /* S4 */
	PM_RESET_TYPE_S5,                       /* S5 */
	PM_RESET_TYPE_WARM_RESET        = 0x10, /* Warm reset */
} sedi_pm_reset_type_t;

/*!
 * \enum sedi_pm_prep_type_t
 * \brief Type of reset prep type
 * \ingroup sedi_driver_pm
 */
typedef enum {
	PM_PREP_TYPE_GENERAL = 0x0, /* General Prep */
} sedi_pm_prep_type_t;

/*!
 * \def PM_RSTPREP_TYPE_BIOS_SETTING_CHANGED
 * \brief used as special prep_type/reset_type to notify sedi_pm_rstprep_cb
 *	that BIOS setting was just changed and FW will reset
 */
#define PM_RSTPREP_TYPE_BIOS_SETTING_CHANGED ((uint32_t)-1)

/*!
 * \fn sedi_pm_rstprep_cb
 * \brief Callback function after reset prepare message received.
 * \ingroup sedi_driver_pm
 */
typedef void (*sedi_pm_rstprep_cb)(uint32_t prep_type, uint32_t reset_type,
				   void *ctx);

/*!
 * \fn sedi_pm_clkchange_cb
 * \brief Callback function after clock changed.
 * \ingroup sedi_driver_pm
 */
typedef void (*sedi_pm_clkchange_cb)(uint32_t core_clk_freq,
				     uint32_t hbw_clk_freq, uint32_t is_before,
				     void *ctx);

/*!
 * \fn sedi_pm_sx_cb
 * \brief Callback function of host sx entry/exit notification.
 * \ingroup sedi_driver_pm
 */
typedef void (*sedi_pm_sx_cb)(sedi_pm_sx_event_t sx_event, void *ctx);

/*!
 * \fn sedi_pm_d3_cb
 * \brief Callback function of host device D3 entry/exit notification.
 * \ingroup sedi_driver_pm
 */
typedef void (*sedi_pm_d3_cb)(sedi_pm_d3_event_t d3_event, void *ctx);

/*!
 * \fn sedi_pm_s0ix_cb
 * \brief Callback function of host s0ix entry/exit notifcation.
 * \ingroup sedi_driver_pm
 */
typedef void (*sedi_pm_s0ix_cb)(sedi_pm_s0ix_event_t s0ix_event, void *ctx);

/*!
 * \union sedi_pm_callback_fn_t
 * \brief Union for PM callback function
 * Caution: don't call blocked function in rstprep_cb, or PM reset flow might
 * be delayed and miss its time window to acknowledge PMC
 * \ingroup sedi_driver_pm
 */
typedef union {
	sedi_pm_rstprep_cb rstprep_cb;
	sedi_pm_clkchange_cb clk_change_cb;
} sedi_pm_callback_fn_t;

/*!
 * \struct sedi_pm_callback_config_t
 * \brief Structure for setting PM callback
 * \ingroup sedi_driver_pm
 */
typedef struct {
	sedi_pm_callback_type_t type;           /**< Callback type */
	sedi_pm_callback_fn_t func;             /**< Callback function */
	sedi_pm_callback_priority_t pri;        /**< Callback function priority */
	void *ctx;                              /**< Context for callback */
} sedi_pm_callback_config_t;

/*!
 * \struct sedi_pm_client_config_t
 * \brief PM timing threshold
 * \ingroup sedi_driver_pm
 */
typedef struct {
	uint16_t ltr_ms;        /**< Latest timer */
	uint16_t ttne_ms;       /**< Time to next event */
	uint16_t id;
} sedi_pm_client_config_t;

/*!
 * \struct sedi_pm_capabilities_t
 * \brief PM Driver Capabilities.
 * \ingroup sedi_driver_pm
 */
typedef struct sedi_pm_capabilities {
	uint32_t reserved;
} sedi_pm_capabilities_t;

/*!
 * \struct sedi_pm_reset_prep_t
 * \brief Structure for reset_prep info.
 * \ingroup sedi_driver_pm
 */
typedef struct sedi_pm_reset_prep {
	sedi_pm_reset_type_t reset_type;        /* Reset type */
	sedi_pm_prep_type_t prep_type;          /* Prep type */
} sedi_pm_reset_prep_t;

/*!
 * \defgroup pm_function_calls PM Driver Function Calls
 * \ingroup sedi_driver_pm
 * \{
 */

/*!
 * \fn sedi_driver_version_t sedi_pm_get_version(void)
 * \brief Get PM driver's API version.
 * \return the version of current PM driver's API
 */
sedi_driver_version_t sedi_pm_get_version(void);

/*!
 * \fn int32_t sedi_pm_get_capabilities(OUT sedi_pm_capabilities_t *cap)
 * \brief Get the driver's capabilities.
 * \param[out] cap: the capabilities of PM driver
 * \return the status codes
 */
int32_t sedi_pm_get_capabilities(OUT sedi_pm_capabilities_t *cap);

/*!
 * \fn int32_t sedi_pm_init(void)
 * \brief Initialize the PM, at very beginning
 * \return result of PM initialization
 */
int32_t sedi_pm_init(void);

/*!
 * \fn int32_t sedi_pm_late_init(void)
 * \brief Some operations should be performed after the initialization of
 * peripheral drivers. That's why this late init API is needed.
 * \return result of PM late initialization
 */
int32_t sedi_pm_late_init(void);

/*!
 * \fn int32_t sedi_pm_uninit(void)
 * \brief Uninitailize the PM
 * \return result of PM un-initialization
 */
int32_t sedi_pm_uninit(void);

/*!
 * \fn void sedi_pm_get_cur_config(OUT sedi_pm_client_config_t *config)
 * \brief Get current active PM configuration.
 * \param[out] config: pointer which contains current active PM configuration.
 * Currently this means the minimum LTR and minimum TTNE that were set.
 * config->id is ignored.
 * \return void
 */
void sedi_pm_get_cur_config(OUT sedi_pm_client_config_t *config);

/*!
 * \fn int32_t sedi_pm_set_client_config(INOUT sedi_pm_client_config_t *config)
 * \brief Sets a PM configuration (currently latency tolerance and time to next
 * event). This will prevent going into D0i3 if the latency tolerance/TTNE is
 * smaller than a set threshold. If this is a first setting of config - invoke
 * with config.id = 0 and a non-zero values
 * \param[in,out] config: When returning with success, config.id will contain
 * the assigned ID. If you wish to update the config - invoke with your assigned
 * ID, and with a new non-zero value in one of the config params. If you wish to
 * cancel the config - invoke with your assigned ID, and with zero in all config
 * params.
 * \return result of setting client configuration SEDI_DRIVER_OK - setting of
 * config succeeded. config.id will contain a non-zero value if its value was
 * larger than 0. If config params values are 0, all params are cleared and
 * config.id = 0.
 * SEDI_DRIVER_ERROR_PARAMETER - parameters are incorrect. For instance -
 * config.id has a non-zero value which wasn't returned by the function
 * previously.
 * SEDI_PM_DRIVER_ERROR_NOMEM - the maximal number of configurations
 * has been set already.
 */
int32_t sedi_pm_set_client_config(INOUT sedi_pm_client_config_t *config);

/*!
 * \fn uint32_t sedi_pm_get_lbw_clock(void)
 * \brief Get current LBW clock frequency
 * \return uint32_t current LBW clock frequency
 */
uint32_t sedi_pm_get_lbw_clock(void);

/*!
 * \fn uint32_t sedi_pm_get_hbw_clock(void)
 * \brief Get current HBW clock frequency
 * \return uint32_t current HBW clock frequency
 */
uint32_t sedi_pm_get_hbw_clock(void);

/*!
 * \fn uint32_t sedi_pm_get_core_clock(void)
 * \brief Get current core clock frequency
 * \return uint32_t current core clock frequency
 */
uint32_t sedi_pm_get_core_clock(void);

/*!
 * \fn void sedi_pm_switch_core_clock(IN sedi_clock_frequency_t freq)
 * \brief Switch ARM core clock frequency with HBW clock frequency
 * \param[in] freq: the core clock frequency
 * \return void
 */
void sedi_pm_switch_core_clock(IN sedi_clock_frequency_t freq);

/*!
 * \fn int32_t sedi_pm_check_host_device_status(void)
 * \brief Check host owned device status
 * \return status codes
 */
int32_t sedi_pm_check_host_device_status(void);

/*!
 * \fn void sedi_pm_check_pending_interrupt(void)
 * \brief Check if any interrupts are pending
 * \return SEDI_DRIVER_OK for no pending interrupts, otherwise return
 * SEDI_DRIVER_ERROR
 */
int32_t sedi_pm_check_pending_interrupt(void);

/*!
 * \fn void sedi_pm_set_power_state(IN sedi_pse_pwr_state_t state)
 * \brief Set PSE low power state
 * \param[in] state: the low power state PSE would enter
 * \return 0 or error codes
 */
int32_t sedi_pm_set_power_state(IN sedi_pse_pwr_state_t state);

/*!
 * \fn int32_t sedi_pm_set_control(IN uint32_t control, IN uint32_t arg)
 * \brief Set control flag for pm driver
 * \param[in] control: control operation code.
 * \param[in] arg: argument of control operation, it's optional
 * \return 0 or error codes
 */
int32_t sedi_pm_set_control(IN uint32_t control, IN uint32_t arg);

/*!
 * \fn int32_t sedi_pm_configure_wake_source(IN sedi_wake_event_type_t type,
 * IN sedi_wake_event_instance_t inst, IN int32_t enable)
 * \brief Configure wake source
 * \param[in] type: the wake event type
 * \param[in] inst: the instance includes controller ID for I2C, SPI, QEP,
 * CAN and DMA controller, pin number for GPIO or function number for PCI
 * \param[in] enable: non-zero to enable and 0 to disable
 * \return 0 or error codes
 */
int32_t sedi_pm_configure_wake_source(IN sedi_wake_event_type_t type,
				      IN sedi_wake_event_instance_t inst,
				      IN int32_t enable);

/*!
 * \fn void sedi_pm_handle_events(void)
 * \brief Handle D3/BME or reset events. It would be called in a thread loop
 * with high priority. A hook function __pm_reset_prep_callback() which is
 * provided by PMA service would be invoked to notify the thread when D3 or
 * reset event received.
 * \return void
 */
void sedi_pm_handle_events(void);

/*!
 * \fn int32_t sedi_pm_register_callback(sedi_pm_callback_config_t
 * *callback_config)
 * \brief register callback functions for apps and PMA
 * \param[in] callback_config: the pointer to sedi_pm_callback_config_t, include
 * callback type, callback function pointer and priority
 * \return 0 or error codes
 */
int32_t sedi_pm_register_callback(sedi_pm_callback_config_t *callback_config);

/*!
 * \fn int32_t sedi_pm_register_sx_notification(sedi_pm_sx_cb
 * cb, void *ctx)
 * \brief register notification for Sx notification
 * \param[in] cb: client callback
 * \param[in] ctx: client context
 * \return 0 or error codes
 */
int sedi_pm_register_sx_notification(sedi_pm_sx_cb cb, void *ctx);

/*!
 * \fn int32_t sedi_pm_register_d3_notification(int pci_func, sedi_pm_sx_cb
 * cb, void *ctx)
 * \brief register notification for d3 notification
 * \param[in] pci_func: host device function number/index
 * \param[in] cb: client callback
 * \param[in] ctx: client context
 * \return 0 or error codes
 */
int sedi_pm_register_d3_notification(int pci_func, sedi_pm_d3_cb cb, void *ctx);

/*!
 * \fn int32_t sedi_pm_enable_global_reset(sedi_global_reset_type_t reset_type,
 * bool enable)
 * \brief enable global reset
 * \param[in] reset_type: global reset type
 * \param[in] enable: enable/disable reset
 * \return 0 or error codes
 */
int sedi_pm_enable_global_reset(sedi_global_reset_type_t reset_type,
				bool enable);

/*!
 * \fn int32_t sedi_pm_register_s0ix_notification(sedi_pm_s0ix_cb
 * cb, void *ctx)
 * \brief register callback function for S0ix entry/exit event
 * \param[in] cb: client callback
 * \param[in] ctx: client context
 * \return 0 or error codes
 */
int sedi_pm_register_s0ix_notification(sedi_pm_s0ix_cb cb, void *ctx);

/*!
 * \fn int32_t sedi_pm_trigger_pme(int32_t pci_func)
 * \brief Trigger PME flow to wake up host
 * \return 0 or error codes
 */
int32_t sedi_pm_trigger_pme(int32_t pci_func);

/*!
 * \fn int32_t sedi_pm_vnn_request(IN vnn_id_t id, IN int32_t enable)
 * \brief request/release SoC VNN resource
 * \param[in] id: vnn_id_t for device
 * \param[in] enable: 1 for request, 0 for release
 * \return 0 or error codes
 */
int32_t sedi_pm_vnn_request(IN vnn_id_t id, IN int32_t enable);

/*!
 * \fn int32_t sedi_pm_trunk_clock_control(IN sedi_pm_trunk_clock_t cid, IN bool enable)
 * \brief Clock gating/un-gating individual trunk clock
 * \param[in] cid: sedi_pm_trunk_clock_t for individual trunk clock
 * \param[in] enable: true for un-gating, false for gating
 * \return 0 or error codes
 */
int32_t sedi_pm_trunk_clock_control(IN sedi_pm_trunk_clock_t cid, IN bool enable);

/*!
 * \fn int32_t sedi_pm_get_reset_prep_info(INOUT sedi_pm_reset_prep_t *reset_prep)
 * \brief Get reset_prep info
 * \param[in,out] reset_prep: pointer to reset_prep info
 * \return 0 or error codes
 */
int32_t sedi_pm_get_reset_prep_info(INOUT sedi_pm_reset_prep_t *reset_prep);

/*!
 * \fn sedi_hw_rev_t sedi_pm_get_hw_rev(void)
 * \brief Get HW Rev
 * \return HW Rev or SEDI_HW_REV_INVALID
 */
static inline sedi_hw_rev_t sedi_pm_get_hw_rev(void)
{
	uint32_t val;
	sedi_hw_rev_t ret;

	val = read32(SEDI_HW_REVID);
	switch (val) {
	case SEDI_HW_REVID_A0:
		ret = SEDI_HW_REV_A0;
		break;
	case SEDI_HW_REVID_B0:
		ret = SEDI_HW_REV_B0;
		break;
	default:
		ret = SEDI_HW_REV_INVALID;
		break;
	}

	return ret;
}

/*!
 * \}
 */

#ifdef __cplusplus
}
#endif

#endif /* _SEDI_DRIVER_PM_H_*/
