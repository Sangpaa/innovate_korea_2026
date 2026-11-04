/**
  ******************************************************************************
  * @file    ms5607.h
  * @brief   Public interface and measurement types for the MS5607 acquisition driver.
  *
  * Call ms5607_init() once after generated GPIO/SPI/UART initialization, then
  * ms5607_process() on every main-loop pass. Twelve fixed output slots represent
  * J1-J12; disabled/failed channels contain NaN and an explanatory status.
  * ms5607_copy_frame() provides a stable foreground snapshot for future streaming.
  * ms5607_status() provides the state used by the LD2 application indicator.
  *
  * USER CONFIGURATION lives at the top of ms5607.c, except the connector mask
  * and sampling rate, which are here so the streaming and sync modules can check
  * them. The default enabled mask selects J1 only (0x001). J12 cannot be enabled
  * while the sync input reserves it (sync_input.h). The active driver owns its
  * SPI handles and reads the shared TIM2 timebase (timebase.h); it never resets
  * that timer. Only the two SPI callback hooks and ms5607_request_reinit()
  * may run in interrupts; all other public functions must be called from
  * foreground code.
  *
  * Faults are not latched: a failed sample invalidates only that cycle's slot,
  * repeatedly failing sensors are reset and re-checked, and absent sensors are
  * re-probed periodically. See change_spec_v3.md / sensor_acquisition_spec.md.
  * The user button B1 requests a re-init of all enabled sensors through
  * ms5607_request_reinit(); finish the current cycle, pause all acquisition,
  * attempt each channel once, then resume successful channels together.
  * Status notifications remain distinct from measurement DATA (change_spec_v5.md).
  ******************************************************************************
  */
#ifndef MS5607_H
#define MS5607_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

/* Exported constants --------------------------------------------------------*/
#define MS5607_CHANNEL_COUNT 12U /**< Fixed carrier connector count; not an enabled count. */

/* ===== USER SETTINGS: pMMG connectors (1 = enabled, 0 = disabled) ============
 * J12 is the sync input (SYNC_INPUT_ENABLED in sync_input.h) and cannot be
 * enabled here. An enabled connector without a sensor is re-probed periodically
 * and streams status != 2 with NaN values. Rebuild and flash after a change. */
#define PMMG_ENABLE_J1   1U
#define PMMG_ENABLE_J2   1U
#define PMMG_ENABLE_J3   1U
#define PMMG_ENABLE_J4   1U
#define PMMG_ENABLE_J5   1U
#define PMMG_ENABLE_J6   1U
#define PMMG_ENABLE_J7   1U
#define PMMG_ENABLE_J8   1U
#define PMMG_ENABLE_J9   0U
#define PMMG_ENABLE_J10  0U
#define PMMG_ENABLE_J11  0U
/* ============================================================================ */

/** Enabled connectors: bit 0 = J1, bit 11 = J12, built from the switches above.
 *  Whole bus masks: SPI1 = 0x207, SPI2 = 0x438, SPI3 = 0x9C0 (0x1C0 while the
 *  sync input reserves J12). All twelve (0xFFF) requires SYNC_INPUT_ENABLED 0.
 *  The ifndef also permits a compiler definition for software tests. */
#ifndef MS5607_ENABLED_MASK
#define MS5607_ENABLED_MASK   ((PMMG_ENABLE_J1 << 0) | (PMMG_ENABLE_J2 << 1) | \
                               (PMMG_ENABLE_J3 << 2) | (PMMG_ENABLE_J4 << 3) | \
                               (PMMG_ENABLE_J5 << 4) | (PMMG_ENABLE_J6 << 5) | \
                               (PMMG_ENABLE_J7 << 6) | (PMMG_ENABLE_J8 << 7) | \
                               (PMMG_ENABLE_J9 << 8) | (PMMG_ENABLE_J10 << 9) | \
                               (PMMG_ENABLE_J11 << 10))
#endif

/** Common pressure sampling rate in Hz. Use 1-100 Hz and a whole-us period.
 *  Temperature is acquired once per pressure sample for compensation.
 *  The ifndef permits a compiler definition for software tests. */
#ifndef MS5607_SAMPLE_HZ
#define MS5607_SAMPLE_HZ      100U
#endif

/* One-shot channel flags, set in the first valid sample after the event. */
#define MS5607_FLAG_RECOVERED    0x01U /**< First valid sample after reset, recovery or hot-plug. */
#define MS5607_FLAG_PROM_CHANGED 0x02U /**< A different valid calibration PROM was adopted. */

/* Exported types ------------------------------------------------------------*/
/** Validity or failure reason for one connector's measurement slot. */
typedef enum {
    MS5607_DISABLED,          /**< Connector is not selected in the enabled mask. */
    MS5607_READY,             /**< Calibration passed; no complete measurement yet. */
    MS5607_VALID,             /**< Pressure and temperature belong to a successful cycle. */
    MS5607_CALIBRATION_ERROR, /**< PROM empty or CRC mismatch; retried periodically. */
    MS5607_TRANSFER_ERROR,    /**< SPI transfer failed in this cycle or recovery attempt. */
    MS5607_ADC_ERROR,         /**< Invalid raw ADC value or out-of-range result this cycle. */
    MS5607_CONFIG_ERROR,      /**< Required HAL handles or timer clock invalid; permanent. */
    MS5607_RECOVERING         /**< Sensor reset or PROM re-check in progress. */
} MS5607_ChannelStatus;

/** Application status; a fault is reported while present and for a hold time after.
 *  The numeric value is sent as stream header byte 24. */
typedef enum {
    MS5607_IDLE,           /**< Enabled mask is zero; board heartbeat mode. */
    MS5607_ACQUIRING,      /**< Acquisition active with no current or recent fault. */
    MS5607_FAULT,          /**< Current or recent fault; healthy channels may still be running. */
    MS5607_REINITIALIZING  /**< User-requested (B1) re-init of all enabled sensors in progress. */
} MS5607_Status;

/** One connector's compensated result and the timing of its conversion commands. */
typedef struct {
    float pressure_pa;              /**< Absolute pressure in Pa; NaN when unavailable. */
    float temperature_c;            /**< Compensation temperature in degrees C, or NaN. */
    uint32_t pressure_start_us;      /**< Software timestamp just before D1 start call. */
    uint32_t temperature_start_us;   /**< Software timestamp just before D2 start call. */
    uint32_t sequence;               /**< Cycle number of last successful measurement. */
    MS5607_ChannelStatus status;     /**< Interpret validity before using value/timestamps. */
    uint16_t error_count;            /**< Saturating count of this channel's error events. */
    uint8_t flags;                   /**< One-shot MS5607_FLAG_* bits for this sample. */
} MS5607_Measurement;

/** Complete acquisition snapshot. Timestamps wrap modulo 2^32 microseconds.
 *  Command timestamps are software observations, not synchronized sample instants.
 *  Data in invalid slots must not be treated as a new measurement. */
typedef struct {
    uint32_t sequence;     /**< Common cycle number, including gaps for skipped cycles. */
    uint32_t scheduled_us; /**< Intended cycle deadline, distinct from actual start. */
    MS5607_Measurement channel[MS5607_CHANNEL_COUNT]; /**< J1 at index 0 through J12 at 11. */
} MS5607_Frame;

/** Foreground diagnostics accumulated since initialization; counters can wrap. */
typedef struct {
    uint32_t calibration_errors;     /**< Failed calibration attempts (startup and recovery). */
    uint32_t transfer_errors;        /**< Failed SPI transfers, one per event. */
    uint32_t adc_errors;             /**< Invalid ADC results plus out-of-range results. */
    uint32_t missed_cycles;          /**< Skipped common sampling deadlines. */
    uint32_t overwritten_frames;     /**< Unread snapshots replaced by a newer frame. */
    uint32_t max_cycle_us;           /**< Longest actual start-to-publication duration. */
    uint32_t max_start_lateness_us;  /**< Largest lateness among cycles that were started. */
    uint32_t last_hal_error;          /**< Saved HAL error bits; may be zero for a timeout/BUSY. */
    uint32_t sensor_resets;           /**< Reset commands sent during recovery (debugger only). */
    uint32_t prom_changes;            /**< Different valid PROMs adopted (debugger only). */
    uint32_t bus_resets;              /**< Fault-path SPI reinitializations; B1 re-init not counted (debugger only). */
    uint32_t recovery_postponed;      /**< Due recovery steps deferred by the deadline guard (debugger only). */
    uint32_t user_reinits;            /**< Accepted and started B1 re-init requests (debugger only). */
} MS5607_Diagnostics;

/* Exported functions --------------------------------------------------------*/
/**
 * @brief Initialize calibration and acquisition state once at startup.
 * @param spi1 Generated SPI1 handle; assigned to J1/J2/J3/J10.
 * @param spi2 Generated SPI2 handle; assigned to J4/J5/J6/J11.
 * @param spi3 Generated SPI3 handle; assigned to J7/J8/J9/J12.
 * @note Call after GPIO/SPI/UART initialization and timebase_init(). Mask zero
 *       performs no SPI traffic and enables no SPI IRQs. Enabled startup uses short blocking
 *       reset/PROM operations. Do not call it again during operation; use
 *       ms5607_request_reinit() to re-initialize the sensors.
 */
void ms5607_init(SPI_HandleTypeDef *spi1, SPI_HandleTypeDef *spi2,
                 SPI_HandleTypeDef *spi3);

/**
 * @brief Service all buses and advance acquisition from the main loop.
 * @note Call every pass without a millisecond gate. No work when disabled;
 *       conversion waits return to the caller. Fault handling may wait in HAL.
 */
void ms5607_process(void);

/**
 * @brief Request a re-init of all enabled sensors (user button B1).
 * @note Interrupt-safe (B1 EXTI callback) and foreground-safe: only reads the
 *       driver state and sets a volatile flag, with no SPI, GPIO or HAL call.
 *       Ignored for a zero enabled
 *       mask, a configuration error, or before ms5607_init() completed. The
 *       re-init starts in the idle gap after the current cycle; a request while
 *       one is running is ignored. All acquisition pauses during recovery.
 */
void ms5607_request_reinit(void);

/**
 * @brief Obtain the application state used to select an LD2 pattern.
 * @return MS5607_IDLE for a zero enabled mask; MS5607_FAULT for a configuration
 *         error; MS5607_REINITIALIZING while a B1 re-init runs; otherwise
 *         MS5607_FAULT during a fault or within MS5607_FAULT_HOLD_MS after it,
 *         else MS5607_ACQUIRING.
 */
MS5607_Status ms5607_status(void);

/**
 * @brief Copy and acknowledge the latest complete frame from foreground code.
 * @param out Destination; NULL returns false without acknowledging anything.
 * @return true for a previously unread frame; false if there is no new frame.
 * @note A non-NULL destination is always filled, even if false is returned.
 *       Before the first frame, measurements are NaN with initialization status.
 */
bool ms5607_copy_frame(MS5607_Frame *out);

/** @brief Check for a new complete frame without copying or acknowledging it.
 *  @return true if a frame is waiting for a foreground consumer.
 *  @note Enables streaming to avoid copying the same twelve slots every loop. */
bool ms5607_frame_pending(void);

/** Register an active foreground DATA consumer. Disabled/stopped streams must
 * unregister so recovery never waits for a consumer that cannot run. */
void ms5607_set_frame_consumer(bool enabled);
/** Current health without acknowledging DATA; foreground only. */
void ms5607_copy_status(MS5607_Frame *out, MS5607_Status *state);
/** Pop one captured transition (frame and matching header state). A four-entry
 * FIFO drops newest on overflow; NULL outputs do not acknowledge. */
bool ms5607_pop_status(MS5607_Frame *out, MS5607_Status *state);
/** Claim and clear transition overflow count for stream loss accounting. */
uint32_t ms5607_status_drops(void);

/** Copy one slot of the latest frame without acknowledging DATA (second
 * foreground reader, e.g. the pMMG light). Foreground only. */
bool ms5607_peek_channel(uint8_t index, MS5607_Measurement *out, uint32_t *frame_sequence);

/**
 * @brief Copy diagnostic counters without clearing them; foreground only.
 * @param out Destination; NULL is ignored.
 */
void ms5607_get_diagnostics(MS5607_Diagnostics *out);

/**
 * @brief Handle successful SPI completion in interrupt context.
 * @param spi Handle forwarded by main.c's HAL_SPI_TxRxCpltCallback.
 * @note Releases CS and notifies foreground code; ignores foreign or idle buses.
 */
void ms5607_spi_complete(SPI_HandleTypeDef *spi);

/**
 * @brief Record a SPI error in interrupt context for foreground recovery.
 * @param spi Handle forwarded by main.c's HAL_SPI_ErrorCallback.
 * @note No retry or blocking recovery is performed inside this hook.
 */
void ms5607_spi_error(SPI_HandleTypeDef *spi);

#endif
