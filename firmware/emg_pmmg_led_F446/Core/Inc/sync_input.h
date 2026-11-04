/**
  ******************************************************************************
  * @file    sync_input.h
  * @brief   Hardware-captured synchronization pulses from the XM thigh controller.
  *
  * XM DIO4 drives J12 pin 6 (CS12, PB10, AF1 TIM2_CH3); the carrier's R12
  * pulls the line up. XM idles the line high and pulls it low for nominally
  * 10 ms; the falling edge is the timing marker and the rising edge validates
  * the pulse width. The Nucleo never drives PB10 while this module owns it.
  *
  * TIM2 channel 3 captures falling edges of TI3 directly; channel 4 captures
  * rising edges of the same TI3 input (indirect selection), so no second pin is
  * used. Both capture registers latch the shared 1 MHz counter (timebase.h), so
  * marker times do not depend on interrupt latency and share the clock of the
  * pressure conversion timestamps.
  *
  * The interrupt only queues raw edges. sync_input_process() pairs them in the
  * foreground, applies the width and timeout rules and publishes one marker per
  * falling edge after its pulse is complete, valid or not. Marker sequences are
  * local to this board (first falling edge after boot = 1); they are NOT XM's
  * sequence numbers. Only the PC can match the two boards' markers.
  *
  * Call sync_input_init() after timebase_init() and before ms5607_init(), and
  * sync_input_process() on every main-loop pass before stream_process().
  * Only sync_input_irq() runs in interrupt context (TIM2_IRQHandler).
  * Specification: synchroizationXMtoNucelo.md section 4.
  ******************************************************************************
  */
#ifndef SYNC_INPUT_H
#define SYNC_INPUT_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "ms5607.h"
#include <stdbool.h>
#include <stdint.h>

/* USER CONFIGURATION BEGIN --------------------------------------------------*/
/** 1 reserves J12 for the XM sync input; 0 returns J12 to sensor use (then the
 *  sync cable must be disconnected). The ifndef permits a compiler definition
 *  for software tests. */
#ifndef SYNC_INPUT_ENABLED
#define SYNC_INPUT_ENABLED 1U
#endif
/* USER CONFIGURATION END ----------------------------------------------------*/

/* Board mapping: J12 pin 6 = CS12 = PB10, AF1 TIM2_CH3. The register code in
 * sync_input.c is specific to TIM2 channels 3/4; change both together. */
#define SYNC_INPUT_CHANNEL   11U                  /**< Reserved connector index (J12). */
#define SYNC_INPUT_GPIO_Port CS12_J12_GPIO_Port   /**< PB10 port from main.h. */
#define SYNC_INPUT_Pin       CS12_J12_Pin         /**< PB10 pin mask from main.h. */
#define SYNC_INPUT_AF        GPIO_AF1_TIM2        /**< PB10 alternate function TIM2_CH3. */

/** Connector mask that the sensor driver must never drive while sync is enabled. */
#define SYNC_INPUT_RESERVED_MASK (SYNC_INPUT_ENABLED != 0U ? (1U << SYNC_INPUT_CHANNEL) : 0U)

#if SYNC_INPUT_ENABLED > 1U
#error "SYNC_INPUT_ENABLED must be 0 or 1"
#endif
#if SYNC_INPUT_ENABLED != 0U && (MS5607_ENABLED_MASK & (1U << SYNC_INPUT_CHANNEL)) != 0U
#error "J12 cannot be both an enabled sensor and the sync input; clear bit 11 of MS5607_ENABLED_MASK or set SYNC_INPUT_ENABLED to 0"
#endif

/* Pulse validation, fixed by the specification; the PC relies on these values. */
#define SYNC_INPUT_MIN_LOW_US  7000U    /**< Shortest valid low time (inclusive). */
#define SYNC_INPUT_MAX_LOW_US  13000U   /**< Longest valid low time (inclusive). */
#define SYNC_INPUT_TIMEOUT_US  50000U   /**< Low this long without a rising edge: incomplete. */
#define SYNC_INPUT_STALE_US    2500000U /**< No pulse event this long: stale. */

/* Marker flags (wire uint16). PATTERN (0x01) and FIRST_NORMAL (0x02) are XM
 * source flags; the receiver leaves them clear and the PC recognizes the pattern. */
#define SYNC_FLAG_INVALID     0x0004U /**< Not usable for clock fitting. */
#define SYNC_FLAG_OVERCAPTURE 0x0008U /**< An edge was lost in hardware or the edge queue. */
#define SYNC_FLAG_INCOMPLETE  0x0010U /**< No matching rising edge; width unknown. */
#define SYNC_WIDTH_UNKNOWN    UINT32_MAX /**< Width of an incomplete pulse. */

/* Exported types ------------------------------------------------------------*/
/** Receiver state; the numeric value is sent in the stream (version 5). */
typedef enum {
    SYNC_DISABLED,     /**< SYNC_INPUT_ENABLED is 0; J12 belongs to the sensor driver. */
    SYNC_CONFIG_ERROR, /**< Timebase unavailable; PB10 stays a plain GPIO input. */
    SYNC_WAITING,      /**< Capturing; no complete pulse observed yet. */
    SYNC_RECEIVING,    /**< The latest pulse was valid. Not proof of cross-board alignment. */
    SYNC_STALE,        /**< No pulse event for SYNC_INPUT_STALE_US after earlier activity. */
    SYNC_FAULT         /**< The latest pulse event was invalid, incomplete or lost edges. */
} SyncState;

/** One marker (falling edge), published once its pulse is complete or timed out. */
typedef struct {
    uint32_t sequence;   /**< Board-local falling-edge count, first = 1; includes lost edges. */
    uint32_t falling_us; /**< TIM2 capture time of the falling edge. */
    uint32_t width_us;   /**< Low time in us, or SYNC_WIDTH_UNKNOWN. */
    uint16_t generation; /**< Capture generation at this edge; +1 when edges were lost. */
    uint16_t flags;      /**< SYNC_FLAG_* bits. */
} SyncMarker;

/** Foreground snapshot of the receiver state and counters. */
typedef struct {
    SyncState state;      /**< Current receiver state. */
    uint16_t generation;  /**< Current capture generation. */
    uint32_t fault_count; /**< Invalid pulses, timeouts, unmatched edges and losses since boot. */
    uint32_t events_lost; /**< Edges or marker records known to be lost (lower bound). */
} SyncStatus;

/* Exported functions --------------------------------------------------------*/
/**
 * @brief Hand PB10 to TIM2 input capture and start receiving.
 * @note Startup only, after MX_GPIO_Init() and timebase_init(). Both channels
 *       are configured as inputs before PB10 becomes an alternate-function pin,
 *       so the pin is never an output. Without a timebase, reports
 *       SYNC_CONFIG_ERROR and leaves PB10 a GPIO input. With SYNC_INPUT_ENABLED
 *       0, restores PB10 as an idle-high chip-select output for the sensor driver.
 */
void sync_input_init(void);

/**
 * @brief Queue captured edges from TIM2 channels 3 and 4.
 * @note Call only from TIM2_IRQHandler. Bounded work: no HAL, SPI or UART calls.
 */
void sync_input_irq(void);

/**
 * @brief Pair queued edges into markers and update timeouts and state.
 * @note Foreground only, on every main-loop pass before stream_process().
 */
void sync_input_process(void);

/**
 * @brief Remove the oldest published marker; foreground only.
 * @param out Destination; NULL returns false without removing anything.
 * @return true if a record was copied.
 */
bool sync_input_pop(SyncMarker *out);

/**
 * @brief Copy the current state and counters; foreground only.
 * @param out Destination; NULL is ignored.
 */
void sync_input_get_status(SyncStatus *out);

#endif
