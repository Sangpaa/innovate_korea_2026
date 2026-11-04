/**
  ******************************************************************************
  * @file    status_led.c
  * @brief   LD2 heartbeat and acquisition/fault patterns using the HAL tick.
  *
  * Each pattern repeats every second. Idle: 500 ms ON / 500 ms OFF.
  * Acquiring: 50 ms ON / 950 ms OFF. Fault: two 100 ms pulses separated
  * by 100 ms OFF, followed by 700 ms OFF. Re-init (B1): continuously ON, for
  * at least REINIT_MIN_ON_MS, so a very short re-init still acknowledges the
  * press. A mode change restarts the pattern.
  *
  * status_led_update() computes the required state from elapsed time and writes
  * the BSP LED only when its state changes. No HAL_Delay is used. Because the
  * main loop services the LED, a stalled application also stops its pattern;
  * this is an application indicator, not an independent watchdog.
  *
  * There are no user-adjustable settings in this module. Agreed pattern durations
  * are documented in StatusLedMode; modify that documentation too if they change.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "status_led.h"
#include "main.h"
#include <stdbool.h>

/* Private constants ---------------------------------------------------------*/
#define REINIT_MIN_ON_MS 500U /**< Minimum STATUS_LED_REINIT time (agreed, spec v4). */

/* Private state -------------------------------------------------------------*/
static StatusLedMode current_mode; /**< Pattern selected by the last update. */
static uint32_t pattern_start_ms;  /**< HAL tick at the current pattern's start. */
static uint32_t reinit_start_ms;   /**< HAL tick at which STATUS_LED_REINIT was entered. */
static bool led_on;               /**< Last state written to the BSP LED. */

/* Public functions ----------------------------------------------------------*/
/**
 * @brief Initialize the idle pattern and turn LD2 ON for its first half-second.
 * @note The caller must first initialize LED_GREEN through the board BSP.
 */
void status_led_init(void)
{
    current_mode = STATUS_LED_IDLE;
    pattern_start_ms = HAL_GetTick();
    led_on = true;
    BSP_LED_On(LED_GREEN);
}

/**
 * @brief Select the required LED state from elapsed time without waiting.
 * @param mode Desired application-status pattern.
 * @param now_ms Current HAL tick in milliseconds.
 * @note Call from the main loop. Unsigned subtraction handles normal tick wrap;
 *       long gaps advance directly to the current pattern position. Once
 *       entered, STATUS_LED_REINIT is kept for at least REINIT_MIN_ON_MS even if
 *       another mode is requested earlier; a longer re-init ends when it does.
 */
void status_led_update(StatusLedMode mode, uint32_t now_ms)
{
    if (current_mode == STATUS_LED_REINIT && mode != STATUS_LED_REINIT &&
        now_ms - reinit_start_ms < REINIT_MIN_ON_MS) {
        mode = STATUS_LED_REINIT; /* Keep a short B1 acknowledgement visible. */
    }
    if (mode != current_mode) {
        current_mode = mode;
        pattern_start_ms = now_ms;
        if (mode == STATUS_LED_REINIT) {
            reinit_start_ms = now_ms;
        }
    }

    uint32_t elapsed = now_ms - pattern_start_ms; /* Time into pattern; then reduced to 0-999 ms. */
    if (elapsed >= 1000U) {
        pattern_start_ms += (elapsed / 1000U) * 1000U;
        elapsed %= 1000U;
    }

    bool on; /* Required LED state at this position within the selected pattern. */
    switch (mode) {
    case STATUS_LED_ACQUIRING:
        on = elapsed < 50U;
        break;
    case STATUS_LED_FAULT:
        on = elapsed < 100U || (elapsed >= 200U && elapsed < 300U);
        break;
    case STATUS_LED_REINIT:
        on = true;
        break;
    default:
        on = elapsed < 500U;
        break;
    }
    if (on != led_on) {
        led_on = on;
        if (on) {
            BSP_LED_On(LED_GREEN);
        } else {
            BSP_LED_Off(LED_GREEN);
        }
    }
}
