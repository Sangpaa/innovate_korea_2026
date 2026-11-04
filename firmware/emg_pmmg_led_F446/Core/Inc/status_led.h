/**
  ******************************************************************************
  * @file    status_led.h
  * @brief   Nonblocking LD2 application-status indicator interface.
  *
  * Initialize after MX_GPIO_Init() (LD2 = PA5), then update from the main loop
  * with the desired mode and HAL_GetTick(). No delay or timer interrupt is used.
  * The source file documents the pattern timing. This interface has no user
  * configuration settings; sensor state selects the mode in main.c.
  ******************************************************************************
  */
#ifndef STATUS_LED_H
#define STATUS_LED_H

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/* Exported types ------------------------------------------------------------*/
/** Repeating patterns within a one-second period, or continuously on. */
typedef enum {
    STATUS_LED_IDLE,      /**< 500 ms ON, 500 ms OFF: application heartbeat. */
    STATUS_LED_ACQUIRING, /**< 50 ms ON, 950 ms OFF: healthy sensor acquisition. */
    STATUS_LED_FAULT,     /**< 100 ON, 100 OFF, 100 ON, 700 OFF (all in ms). */
    STATUS_LED_REINIT     /**< Continuously ON: B1 sensor re-init in progress, at least 500 ms. */
} StatusLedMode;

/* Exported functions --------------------------------------------------------*/
/**
 * @brief Start the idle heartbeat with LD2 initially ON.
 * @note Foreground only, after MX_GPIO_Init().
 */
void status_led_init(void);

/**
 * @brief Apply the current pattern without blocking the main loop.
 * @param mode Desired status; changing it restarts the pattern's first pulse.
 *        STATUS_LED_REINIT stays at least 500 ms after it was entered.
 * @param now_ms Current HAL millisecond tick; normal counter wrap is supported.
 * @note Call regularly from foreground code, normally once per changed tick.
 */
void status_led_update(StatusLedMode mode, uint32_t now_ms);

#endif
