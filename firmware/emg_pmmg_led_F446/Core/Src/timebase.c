/**
  ******************************************************************************
  * @file    timebase.c
  * @brief   One-time start of the shared 1 MHz TIM2 counter.
  *
  * Extracted from the MS5607 driver so synchronization pulses and pressure
  * timestamps use the same clock and so the timer also runs without sensors.
  * The first successful call resets and starts TIM2; it is never reset again.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "timebase.h"

/* Private state -------------------------------------------------------------*/
static bool timebase_initialized; /**< timebase_init() has run; started holds its result. */
static bool timebase_started;     /**< TIM2 counts at 1 MHz. */

/* Public functions ----------------------------------------------------------*/
/**
 * @brief Reserve and start TIM2 as a 1 MHz counter without timer interrupts.
 * @return true if the APB1 timer clock supports an exact 1 us tick.
 * @note APB1 timer clocks double when the APB1 prescaler is not unity. The RCC
 *       reset runs only on the first call, before any timestamp exists.
 */
bool timebase_init(void)
{
    if (timebase_initialized) {
        return timebase_started;
    }
    timebase_initialized = true;
    uint32_t clock_hz = HAL_RCC_GetPCLK1Freq(); /* Peripheral clock, then timer input Hz. */
    if ((RCC->CFGR & RCC_CFGR_PPRE1_2) != 0U) { /* PPRE1 = 0xx means not divided. */
        clock_hz *= 2U;
    }
    if (clock_hz < 1000000U || clock_hz % 1000000U != 0U) {
        return false;
    }
    __HAL_RCC_TIM2_CLK_ENABLE();
    __HAL_RCC_TIM2_FORCE_RESET();
    __HAL_RCC_TIM2_RELEASE_RESET();
    TIM2->PSC = clock_hz / 1000000U - 1U;
    TIM2->ARR = UINT32_MAX;
    TIM2->EGR = TIM_EGR_UG;
    TIM2->SR = 0U;
    TIM2->CNT = 0U;
    TIM2->CR1 = TIM_CR1_CEN;
    timebase_started = true;
    return true;
}

/** @brief Report whether the shared counter is running at 1 MHz. */
bool timebase_ready(void)
{
    return timebase_started;
}

/**
 * @brief Read the shared free-running microsecond clock.
 * @return TIM2 count, wrapping modulo 2^32 (about 71.6 minutes).
 */
uint32_t timebase_now_us(void)
{
    return TIM2->CNT;
}
