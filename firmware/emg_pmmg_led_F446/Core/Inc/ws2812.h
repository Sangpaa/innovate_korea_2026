/**
  ******************************************************************************
  * @file    ws2812.h
  * @brief   WS2812/WS2812B driver: three data lines on TIM4 CH1/CH2/CH3 + DMA.
  *
  * Outputs (one data line each, independent content):
  *  - WS2812_LEFT_ARM:  PB6 (D10),     TIM4_CH1, DMA1 stream 0 channel 2.
  *  - WS2812_RIGHT_ARM: PB7 (CN7-21),  TIM4_CH2, DMA1 stream 3 channel 2.
  *  - WS2812_RING:      PB8 (D15),     TIM4_CH3, DMA1 stream 7 channel 2
  *                      (arc reactor: concentric rings, chained outer to inner).
  * On each arm the strips are daisy-chained (DOUT of strip k to DIN of strip
  * k+1), so pixel index = strip * WS2812_LEDS_PER_STRIP + position.
  *
  * Each bit is one 1.25 us PWM period. TIM4 runs continuously; its CCx DMA
  * requests come on the update event (CCDS = 1), and every DMA write lands
  * in the preloaded compare register, so it takes effect at the next period.
  * The last slots of each buffer are zero, so a line stays low between
  * frames. No interrupts are used: ws2812_busy() reads the stream enable
  * bits, which the DMA clears when a transfer completes.
  *
  * Colour values passed to ws2812_set() are perceptual (0-255). ws2812_show()
  * applies a gamma 2.2 curve, the WS2812_MAX_BRIGHTNESS limit and the
  * current caps: each arm is held at WS2812_ARM_LIMIT_MA and the arc reactor
  * at WS2812_RING_LIMIT_MA (so one fuse per output never sees more), then
  * all outputs together at WS2812_POWER_LIMIT_MA (supply). The caps count
  * the colour current only; add about 1 mA per LED of idle current.
  *
  * PB6-PB8 are 5 V tolerant. A 3.3 V high level is below the 0.7 x VDD input
  * threshold of a 5 V WS2812, so use a 74HCT125 buffer at 5 V, or set
  * WS2812_OPEN_DRAIN 1 with a 1 kOhm pull-up from each pin to the LEDs' 5 V.
  ******************************************************************************
  */
#ifndef WS2812_H
#define WS2812_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

/* USER CONFIGURATION BEGIN --------------------------------------------------*/
#ifndef WS2812_ARM_STRIPS
#define WS2812_ARM_STRIPS      2U   /**< Strips on each arm's chain. */
#endif
#ifndef WS2812_LEDS_PER_STRIP
#define WS2812_LEDS_PER_STRIP  144U /**< LEDs per arm strip (1 m of 144 LED/m). */
#endif
#ifndef WS2812_RING_LEDS
#define WS2812_RING_LEDS       60U  /**< LEDs on the arc reactor chain (rings 24+16+12+8, see fx.h). */
#endif
/** Brightness limit 0-255, applied after gamma to every colour channel. */
#ifndef WS2812_MAX_BRIGHTNESS
#define WS2812_MAX_BRIGHTNESS  128U
#endif
/** Estimated colour-current caps in mA. An output above its cap is dimmed
 *  uniformly; then, if all outputs together are above WS2812_POWER_LIMIT_MA,
 *  every output is dimmed by the same factor.
 *  ARM: keep cap + idle (about 0.3 A for 288 LEDs) below 80 % of the arm fuse
 *  (5 A fuse -> 4 A). POWER: keep it below about 70 % of the weakest supply
 *  stage (20 V trigger 3 A ~ 10 A at 5 V) minus the idle current. */
#ifndef WS2812_ARM_LIMIT_MA
#define WS2812_ARM_LIMIT_MA    3500U  /**< Each arm (left, right). */
#endif
#ifndef WS2812_RING_LIMIT_MA
#define WS2812_RING_LIMIT_MA   1800U  /**< Arc reactor. */
#endif
#ifndef WS2812_POWER_LIMIT_MA
#define WS2812_POWER_LIMIT_MA  8000U  /**< All outputs together. */
#endif
/** 0 = push-pull pins (use a 5 V buffer); 1 = open-drain with external 5 V pull-ups. */
#ifndef WS2812_OPEN_DRAIN
#define WS2812_OPEN_DRAIN      0U
#endif
/* USER CONFIGURATION END ----------------------------------------------------*/

#define WS2812_ARM_LEDS (WS2812_ARM_STRIPS * WS2812_LEDS_PER_STRIP) /**< Pixels per arm. */

#if WS2812_ARM_LEDS == 0U || WS2812_ARM_LEDS > 600U
#error "Each arm chain must have 1-600 LEDs"
#endif
#if WS2812_RING_LEDS == 0U || WS2812_RING_LEDS > 600U
#error "The ring must have 1-600 LEDs"
#endif
#if WS2812_MAX_BRIGHTNESS > 255U
#error "WS2812_MAX_BRIGHTNESS must be 0-255"
#endif

/** One data line. */
typedef enum {
    WS2812_LEFT_ARM,
    WS2812_RIGHT_ARM,
    WS2812_RING,
    WS2812_OUTPUT_COUNT
} Ws2812Output;

/* Exported functions --------------------------------------------------------*/
/**
 * @brief Configure PB6-PB8, TIM4 and the three DMA streams, then send one
 *        all-off frame.
 * @return false if the timer clock cannot form the bit timing or HAL init failed.
 * @note Foreground, once, after SystemClock_Config().
 */
bool ws2812_init(void);

/** @brief Number of LEDs on one output (0 for an invalid output). */
uint16_t ws2812_length(Ws2812Output output);

/** @brief Set one pixel in the frame buffer (not sent until ws2812_show()).
 *  Out-of-range outputs or indices are ignored. */
void ws2812_set(Ws2812Output output, uint16_t index, uint8_t red, uint8_t green, uint8_t blue);

/** @brief Clear all frame buffers to black. */
void ws2812_clear(void);

/**
 * @brief Encode all frame buffers and start the three DMA transfers.
 * @return false while the previous frame is still being sent (nothing changes).
 * @note Foreground only. An arm frame takes ARM_LEDS x 30 us + 300 us.
 */
bool ws2812_show(void);

/** @brief true while any output is still sending. */
bool ws2812_busy(void);

/** @brief Estimated colour current of the last frame in mA, after the caps. */
uint32_t ws2812_estimated_ma(void);

/** @brief Same for one output (0 for an invalid output). */
uint32_t ws2812_output_ma(Ws2812Output output);

#endif
