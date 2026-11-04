/**
  ******************************************************************************
  * @file    timebase.h
  * @brief   Shared free-running 1 MHz TIM2 clock for sensor and sync timestamps.
  *
  * TIM2 counts microseconds with auto-reload 0xFFFFFFFF and wraps about every
  * 71.6 minutes. Pressure conversion timestamps (ms5607.c) and synchronization
  * markers (sync_input.c) share this one clock, so they can be related on the
  * PC. The counter is started once and never reset afterwards: B1 re-init, SPI
  * or UART recovery and sync faults leave it running.
  *
  * This module owns the TIM2 time base (PSC, ARR, CNT, CR1). sync_input.c owns
  * capture channels 3/4 and the TIM2 interrupt. Captures latch the same
  * counter, so do not change PSC/ARR or reset CNT anywhere else.
  * See synchronization_spec.md step 2 and synchroizationXMtoNucelo.md 4.2.
  ******************************************************************************
  */
#ifndef TIMEBASE_H
#define TIMEBASE_H

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

/* Exported functions --------------------------------------------------------*/
/**
 * @brief Start TIM2 as the shared 1 MHz counter, once per boot.
 * @return true if the timer runs at exactly 1 MHz; false if the APB1 timer
 *         clock is not a whole multiple of 1 MHz (permanent configuration error).
 * @note Foreground only, after SystemClock_Config(). Later calls return the
 *       first result without touching the timer, so callers can rely on it.
 */
bool timebase_init(void);

/** @brief Report whether timebase_init() succeeded. */
bool timebase_ready(void);

/**
 * @brief Read the shared microsecond counter.
 * @return TIM2 count, wrapping modulo 2^32 (about 71.6 minutes).
 * @pre timebase_ready(). Safe in interrupt and foreground context.
 */
uint32_t timebase_now_us(void);

#endif
