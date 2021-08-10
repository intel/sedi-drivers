/*
 * Copyright (c) 2021 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _SEDI_DRIVER_TGPIO_H_
#define _SEDI_DRIVER_TGPIO_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "sedi_driver_common.h"

/*!
 * \defgroup sedi_driver_tgpio TGPIO
 * \ingroup sedi_driver
 */

/*!
 * \defgroup tgpio_pin_config TGPIO Pin Configurations
 * \ingroup sedi_driver_tgpio
 * \{
 */

/*!
 * \brief TGPIO instance number
 */
typedef enum instance {
	SEDI_TGPIO_0 = 0, /**< TGPIO Instance 0 */
	SEDI_TGPIO_1,     /**< TGPIO Instance 1 */
	SEDI_TGPIO_NUM
} sedi_tgpio_t;

/*!
 * \brief TGPIO timer selection
 */
typedef enum timer {
	SEDI_TGPIO_TMT_0 = 0, /**< TMT timer 0 */
	SEDI_TGPIO_TMT_1,     /**< TMT timer 1 */
	SEDI_TGPIO_TMT_2,     /**< TMT timer 2 */
	SEDI_TGPIO_ART,       /**< ART timer */
	SEDI_TGPIO_PTM,       /**< PTM timer */
	SEDI_TGPIO_TIMER_NUM,
} sedi_tgpio_timer_t;

/*!
 * \brief GPIO pins per instance
 */
typedef enum pin {
	SEDI_GPIO_PIN_0 = 0, /**< GPIO pin number 0 */
	SEDI_GPIO_PIN_1,
	SEDI_GPIO_PIN_2,
	SEDI_GPIO_PIN_3,
	SEDI_GPIO_PIN_4,
	SEDI_GPIO_PIN_5,
	SEDI_GPIO_PIN_6,
	SEDI_GPIO_PIN_7,
	SEDI_GPIO_PIN_8,
	SEDI_GPIO_PIN_9,
	SEDI_GPIO_PIN_10,
	SEDI_GPIO_PIN_11,
	SEDI_GPIO_PIN_12,
	SEDI_GPIO_PIN_13,
	SEDI_GPIO_PIN_14,
	SEDI_GPIO_PIN_15,
	SEDI_GPIO_PIN_16,
	SEDI_GPIO_PIN_17,
	SEDI_GPIO_PIN_18,
	SEDI_GPIO_PIN_19,
	SEDI_GPIO_PIN_20,
	SEDI_GPIO_PIN_21,
	SEDI_GPIO_PIN_22,
	SEDI_GPIO_PIN_23,
	SEDI_GPIO_PIN_24,
	SEDI_GPIO_PIN_25,
	SEDI_GPIO_PIN_26,
	SEDI_GPIO_PIN_27,
	SEDI_GPIO_PIN_28,
	SEDI_GPIO_PIN_29,
	SEDI_GPIO_PIN_NUM /**< Total number of GPIO pins per instance */
} sedi_gpio_pin_t;

/*!
 * \brief Direction of a TGPIO pin
 */
typedef enum direction {
	SEDI_OUTPUT = 0, /**< Output mode */
	SEDI_INPUT       /**< Input mode */
} sedi_tgpio_direction_t;

/*!
 * \brief TGPIO pin event polarity
 */
typedef enum event_polarity {
	SEDI_EVENT_RISING_EDGE = 0, /**< Generate event on rising edge */
	SEDI_EVENT_FALLING_EDGE,    /**< Generate event on falling edge */
	SEDI_EVENT_TOGGLE	   /**< Generate event on both edges */

} sedi_tgpio_event_t;

/*!
 * \brief Structure representing time
 */
typedef struct time {
	uint32_t nsec; /**< Nanosecond */
	uint32_t sec;  /**< Seconds */
} sedi_tgpio_time_t;

/*!
 * \strcut sedi_tgpio_pin_config_
 * \brief TGPIO single pin configuration
 *
 * The below structure can be used by an appication in Following modes:
 * 1. Simple Output scheduled
 *		In this mode, the controller generates out signal only once.
 * 		The following are the necessary values to be set:
 *		a. Pulse Width Stretch.
 *		b. Start_ime - when this event should occur relative to seleted
 *		   timer.
 *		c. Timer - Select reference timer
 *		d. Interrupt enable; If Yes, then set Int. polarity.
 *		e. Direction - Set to Output
 *
 * 2. Repeated Output (infinite/limited) -
 *		In this mode, the controller generates out signal multiple
 *		times. The following are the necessary values to be set:
 *		a. repeat_enable - enable repeatation
 *		b. repeat_count - number of times this event should repeat.
			set 0 for infinite.
 *		c. repeat_interval - absolute time between two repeats
 *		d. Pulse Width Stretch.
 *		e. Timer - Select an available timer
 *		f. Start_time - when the first event will occur relative to
 *		   selected timer.
 *		g. Interrupt enable;
 *		h. Direction - Set to Output
 *
 * 3. Input - Event count based
 *		In this mode, the controller is programmed to receive input
 *		signals and raise interrupt every time it receives an input
 *		signal. The following are the necessary values to be set:
 *		a. Interrupt enabled;
 *		b. Input event polarity
 *		c. direction - set to input
 *
 * 4. Input - Time Based -
 *		In this mode, the controller is programmed to receive input
 *		signals and raise interrupt in periodic time interval
 *		with number of events happened in that time window.
 *		The following are the necessary values to be set:
 *		a. Timer
 *		b. Interrupt Enabled;
 *		c. Input event polarity
 *		d. direction - set to input
 *		e. repeat_enable
 *		e. repeat_interval - Int. after every  ns time
 *		f. Start_time - Absolute time matching to timer selected
 */
typedef struct tgpio_pin_config_t {
	/**< Select timer to be used for timestamping */
	sedi_tgpio_timer_t timer;
	/**< Select event polarity */
	sedi_tgpio_event_t ev_polarity;
	/**< Interrupt enable for the pin */
	uint32_t intr_enable;
	/**< GPIO direction, 0b: Input, 1b: Output. */
	uint32_t direction;
	/**< Absolute time to start an operation */
	sedi_tgpio_time_t start_time;
	/**< Periodic repeatation time */
	sedi_tgpio_time_t repeat_interval;
	/**< Number of repeatation, 0: unlimited, range: 0-255 */
	uint32_t repeat_count;
	/**< Mandatory values for both input and output modes */
	union {
		/**< Pulse Width in cycles, range: 2-17 */
		uint32_t pulse_width;
		/**< Ceiling to raise an interrupt */
		uint64_t events_ceiling;
	};

} sedi_tgpio_pin_config_t;

/*!
 * \}
 */
#define SEDI_TGPIO_API_VERSION SEDI_DRIVER_VERSION_MAJOR_MINOR(0, 1)

/*!
 * \defgroup tgpio_driver_apis TGPIO Driver APIs
 * \ingroup sedi_driver_tgpio
 * \brief Timed GPIO APIs. The API set is logically diveded into two parts
 *	1. Pin configuration APIs - helps to configure a pin to be operated as
	   timed gpio pin with flexible configuration options.
	2. Timer configuration APIs - helps to program a TMT based on given
	   configuration values
 * \{
 */

/*!
 * \brief tgpio ISR handler. Can be registered by an OS.
 *
 * \param[in] instance Instance of the TPGIO block
 */
void sedi_tgpio_isr_handler(IN sedi_tgpio_t instance);

/*!
 * \brief Callback function to be registered for an event
 *
 * \param[in] cb_data Callback data
 * \param[in] status Status of the pins. 0b-event not present, 1b-event present
 *
 * \return
 */
typedef void (*sedi_tgpio_cb_t)(IN void *cb_data, IN uint32_t status);

/*!
 * \brief TGPIO initialization function. Registers passed callback.
 *
 * \param[in] instance Instance of the TPGIO block
 * \param[in] cb pointer to callback
 * \param[in] cb_data pointer to callback data
 *
 * \return SEDI_DRIVER_OK on success
 */
int32_t sedi_tgpio_init(IN sedi_tgpio_t instance, IN sedi_tgpio_cb_t cb,
			void *cb_data);

/*!
 * \brief Configure a single pin on a TGPIO block
 *
 * \param[in] instance Instance of the TPGIO block
 * \param[in] gpio_pin GPIO pin number
 * \param[in] cfg configuration structure
 *
 * \return SEDI_DRIVER_OK on success
 */
int32_t sedi_tgpio_pin_config(IN sedi_tgpio_t instance,
			      IN sedi_gpio_pin_t gpio_pin,
			      IN sedi_tgpio_pin_config_t cfg);

/*!
 * \brief  Enables a pin
 *
 * \param[in] instance Instance of the TPGIO block
 * \param[in] gpio_pin Pin number to be enabled
 *
 * \return SEDI_DRIVER_OK on success
 */
int32_t sedi_tgpio_pin_enable(IN sedi_tgpio_t instance,
			      IN sedi_gpio_pin_t gpio_pin);

/*!
 * \brief Disables a pin
 *
 * \param[in] instance Instance of the TPGIO block
 * \param[in] gpio_pin Pin number to be enabled
 *
 * \return SEDI_DRIVER_OK on success
 */
int32_t sedi_tgpio_pin_disable(IN sedi_tgpio_t instance,
			       IN sedi_gpio_pin_t gpio_pin);

/*!
 * \brief Get crosstimestamp
 *
 * \param[in] instance Instance of the TPGIO block
 * \param[in] local_clock Select a local clock
 * \param[in] reference_clock Select a reference clock
 * \param[in] local_time Capture local time
 * \param[in] reference_time Capture reference time
 *
 * \return SEDI_DRIVER_OK on success
 */
int32_t sedi_tgpio_get_cross_timestamp(IN sedi_tgpio_t instance,
				       IN sedi_tgpio_timer_t local_clock,
				       IN sedi_tgpio_timer_t reference_clock,
				       OUT sedi_tgpio_time_t *local_time,
				       OUT sedi_tgpio_time_t *reference_time);

/*!
 * \brief Set time in a timer
 *
 * \param[in] instance Instance of the TPGIO block
 * \param[in] tgpio_timer timer to be set
 * \param[in] sec timevalue in seconds
 * \param[in] ns timevalue in nanoseconds
 *
 * \return SEDI_DRIVER_OK on success
 */
int32_t sedi_tgpio_set_time(IN sedi_tgpio_t instance,
			    IN sedi_tgpio_timer_t tgpio_timer, IN uint32_t sec,
			    IN uint32_t ns);

/*!
 * \brief Get time from a timer
 *
 * \param[in] instance Instance of the TPGIO block
 * \param[in] tgpio_timer timer from which time to be read
 * \param[in] sec capture timevalue in seconds
 * \param[in] ns capture timevalue in nanoseconds
 *
 * \return SEDI_DRIVER_OK on success
 */
uint32_t sedi_tgpio_get_time(IN sedi_tgpio_t instance,
			     IN sedi_tgpio_timer_t tgpio_timer,
			     OUT uint32_t *sec, OUT uint32_t *ns);

/*!
 * \brief  Adjust a timer value dynamically
 *
 * \param[in] instance Instance of the TPGIO block
 * \param[in] tgpio_timer timer to be adjusted
 * \param[in] time_ns timevalue to be added/subtracted
 *
 * \return SEDI_DRIVER_OK on success
 */
int32_t sedi_tgpio_adjust_time(IN sedi_tgpio_t instance,
			       IN sedi_tgpio_timer_t tgpio_timer,
			       IN int32_t time_ns);

/*!
 * \brief  Adjuest running rate of a timer
 *
 * \param[in] instance Instance of the TPGIO block
 * \param[in] tgpio_timer timer of which the frequency to be adjusted
 * \param[in] ppb particles per billion adjuest value
 *
 * \return SEDI_DRIVER_OK on success
 */
int32_t sedi_tgpio_adjust_frequency(IN sedi_tgpio_t instance,
				    IN sedi_tgpio_timer_t tgpio_timer,
				    IN int32_t ppb);

/*!
 * \brief  get timestamp and/or event count of last event occured
 *
 * \param[in] instance Instance of the TPGIO block
 * \param[in] gpio_pin GPIO pin number
 * \param[in] timestamp timestamp of the event occured
 * \param[in] event_counter
 *
 * \return SEDI_DRIVER_OK on success
 */
int32_t sedi_tgpio_get_pindata(IN sedi_tgpio_t instance,
			       IN sedi_gpio_pin_t gpio_pin,
			       OUT sedi_tgpio_time_t *timestamp,
			       OUT uint64_t *event_counter);

/*!
 * \brief Uninitialize an instance
 *
 * \param[in] instance Instance of the TPGIO block
 *
 * \return SEDI_DRIVER_OK on success
 */
int32_t sedi_tgpio_uninit(IN sedi_tgpio_t instance);

/*!
 * \brief Set power states for a TGPIO device
 *
 * \param[in] instance Instance of the TPGIO block
 * \param[in] state Power state
 *
 * \return SEDI_DRIVER_OK on success
 */
int32_t sedi_tgpio_set_power(IN sedi_tgpio_t instance,
			     IN sedi_power_state_t state);
/*!
 * \brief get driver version
 *
 * \return structure of driver version
 */
sedi_driver_version_t sedi_tgpio_get_version(void);

/*!
 * \}
 */

#ifdef __cplusplus
}
#endif

#endif /* _SEDI_DRIVER_TGPIO_H_*/
