/**
  ******************************************************************************
  * @file    ms5607.c
  * @brief   MS5607 sensor configuration, SPI driver and acquisition scheduler.
  *
  * This module manages twelve connector positions on three independent SPI buses.
  * At startup it resets enabled sensors, reads their factory calibration PROM and
  * validates its CRC. The main loop then calls ms5607_process() continuously:
  * temperature conversion -> wait -> read -> pressure conversion -> wait -> read.
  * Each phase services all three buses using interrupt-driven HAL transfers.
  * Completed measurements are compensated and published as one fixed-size frame.
  *
  * Faults are not latched. A failed transfer, invalid ADC value or out-of-range
  * result invalidates only that channel's slot in that cycle. A sensor that fails
  * MS5607_RESET_AFTER_FAILURES cycles in a row is reset and its PROM re-checked;
  * sensors missing at boot or failing recovery are re-probed every
  * MS5607_RETRY_PERIOD_MS (hot-plug). Recovery uses short blocking transfers in
  * the idle gap after publication, at most one step per cycle. Repeated transfer
  * failures on one bus reinitialize that SPI peripheral; nothing is disabled
  * until reboot except a configuration error. ms5607_status() reports a fault
  * while it exists and for MS5607_FAULT_HOLD_MS after the last fault event.
  *
  * B1 finishes the current cycle and hands its DATA to the stream, then pauses
  * all acquisition. Foreground steps reset buses and sensors, wait for PROM
  * reload, and validate each enabled channel once. Successful channels resume
  * together on a fresh cycle grid after all attempts finish. Streaming/LD2 keep
  * running between steps. Ordinary automatic recovery still uses the idle guard.
  * Recovery-start and resume health notifications have independent ownership
  * from DATA. A disabled stream never blocks recovery's clean-frame boundary.
  *
  * USER CONFIGURATION below selects the fault-recovery policy; the connector
  * mask is in ms5607.h (default J1 only). A zero mask disables sensor
  * communication and SPI IRQs for board-only heartbeat use. Timestamps come from
  * the shared 1 MHz TIM2 timebase (timebase.c), started once and never reset
  * here. With sensors enabled, this module owns the three SPI handles. While the
  * sync input is enabled (sync_input.h), J12/PB10 is never driven as a chip
  * select (synchroizationXMtoNucelo.md section 4.1).
  * Public operations run in the main loop; only completion/error hooks and the
  * re-init request run in interrupt context. Initialization is startup-only.
  *
  * Formula sources:
  * - TE MS5607-02BA03 datasheet, March 2024, pp. 8-9: pressure/temperature
  *   compensation and second-order low-temperature correction.
  * - MEAS AN520, revision 004, pp. 4-5: calibration PROM CRC-4 procedure.
  * Source links and hardware validation status: sensor_acquisition_spec.md.
  * Fault model and recovery: change_spec_v3.md section 3. B1 re-init:
  * change_spec_v5.md.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "ms5607.h"
#include "main.h"
#include "sync_input.h"
#include "timebase.h"
#include <math.h>
#include <string.h>

/* USER CONFIGURATION BEGIN --------------------------------------------------*/
/* MS5607_ENABLED_MASK (connector selection) and MS5607_SAMPLE_HZ (common
 * sampling rate) are configured in ms5607.h. */

/** Consecutive bad cycles before a sensor is reset and its PROM re-checked.
 *  Range 1-100. At 100 Hz the default of 3 means a 30 ms fault burst. */
#ifndef MS5607_RESET_AFTER_FAILURES
#define MS5607_RESET_AFTER_FAILURES 3U
#endif

/** Interval in ms between recovery attempts for an absent sensor, or one whose
 *  recovery failed. Range 1-60000. Also the hot-plug probe interval. */
#ifndef MS5607_RETRY_PERIOD_MS
#define MS5607_RETRY_PERIOD_MS 1000U
#endif

/** Consecutive failed transfers on one bus before its SPI peripheral is
 *  reinitialized. Range 1-100. Any successful transfer clears the count. */
#ifndef MS5607_BUS_FAILURE_LIMIT
#define MS5607_BUS_FAILURE_LIMIT 3U
#endif

/** Time in ms that MS5607_FAULT stays reported after the last fault event.
 *  Range 1-60000. Keeps short faults visible on LD2 and in the stream header. */
#ifndef MS5607_FAULT_HOLD_MS
#define MS5607_FAULT_HOLD_MS 2000U
#endif
/* USER CONFIGURATION END ----------------------------------------------------*/

/* Private constants ---------------------------------------------------------*/
/* Protocol/timing constants are coupled, not independent user settings.
 * OSR 1024 is the agreed starting point. A different OSR requires changing the
 * command bytes, maximum conversion wait and validating the complete cycle. */
#define CMD_PRESSURE          0x44U  /**< Start D1 pressure conversion, OSR 1024. */
#define CMD_TEMPERATURE       0x54U  /**< Start D2 temperature conversion, OSR 1024. */
#define CMD_RESET             0x1EU  /**< Sensor reset; reloads the PROM into its registers. */
#define CMD_PROM_READ         0xA0U  /**< PROM word 0 read; word n uses 0xA0 + 2n. */
#define CONVERSION_WAIT_US    2300U  /**< 2280 us datasheet maximum plus guard. */
#define TRANSFER_TIMEOUT_US   1000U  /**< Foreground SPI transaction deadline. */
#define STARTUP_TIMEOUT_MS    5U     /**< HAL blocking reset/PROM transfer timeout at startup. */
#define RESET_WAIT_US         3000U  /**< Datasheet PROM reload time of 2.8 ms, plus guard. */
#define RECOVERY_GUARD_US     3000U  /**< Minimum time before the next deadline to start a recovery
                                          transfer group (reset, or one 8-word PROM read). Worst case
                                          per group: 7 x ~85 us + one 2 ms HAL timeout = ~2.6 ms. */
#define RECOVERY_TIMEOUT_MS   2U     /**< HAL timeout for one blocking recovery transfer. */
#define RX_FIFO_BYTES         4U     /**< SPI RX FIFO depth in 8-bit frames; drain bound. */
#define PRESSURE_MIN_PA       1000   /**< Sensor range 10 mbar; lower results are rejected. */
#define PRESSURE_MAX_PA       130000 /**< Sensor range 1200 mbar plus margin. */
#define TEMPERATURE_MIN_C     (-40)  /**< Sensor operating range, lower limit. */
#define TEMPERATURE_MAX_C     85     /**< Sensor operating range, upper limit. */
#define BUS_COUNT             3U    /**< Number of independent SPI peripherals. */
#define NO_CHANNEL            MS5607_CHANNEL_COUNT /**< End-of-channel-list marker. */
#define SAMPLE_PERIOD_US      (1000000U / MS5607_SAMPLE_HZ) /**< Common cycle period. */

/* Preprocessor checks work without C11 static-assert support in the IDE. */
#if MS5607_SAMPLE_HZ == 0U || MS5607_SAMPLE_HZ > 100U
#error "MS5607_SAMPLE_HZ must be between 1 and 100 Hz"
#elif (1000000U % MS5607_SAMPLE_HZ) != 0U
#error "MS5607_SAMPLE_HZ must produce a whole-microsecond period"
#endif
#if (MS5607_ENABLED_MASK & ~0xFFFU) != 0U
#error "MS5607_ENABLED_MASK may only select J1 through J12"
#endif
#if MS5607_RESET_AFTER_FAILURES < 1U || MS5607_RESET_AFTER_FAILURES > 100U
#error "MS5607_RESET_AFTER_FAILURES must be between 1 and 100"
#endif
#if MS5607_RETRY_PERIOD_MS < 1U || MS5607_RETRY_PERIOD_MS > 60000U
#error "MS5607_RETRY_PERIOD_MS must be between 1 and 60000 ms"
#endif
#if MS5607_BUS_FAILURE_LIMIT < 1U || MS5607_BUS_FAILURE_LIMIT > 100U
#error "MS5607_BUS_FAILURE_LIMIT must be between 1 and 100"
#endif
#if MS5607_FAULT_HOLD_MS < 1U || MS5607_FAULT_HOLD_MS > 60000U
#error "MS5607_FAULT_HOLD_MS must be between 1 and 60000 ms"
#endif

/* Private types and board mapping -------------------------------------------*/
/** Physical wiring for one connector; its array index is connector number - 1. */
typedef struct {
    GPIO_TypeDef *port; /**< Active-low chip-select GPIO port. */
    uint16_t pin;      /**< Chip-select pin mask within port. */
    uint8_t bus;       /**< Index into buses[]: 0 = SPI1, 1 = SPI2, 2 = SPI3. */
} ChannelConfig;

/* BOARD MAPPING: matches the carrier schematic and generated main.h labels.
 * Change only for a hardware revision; select occupied sensors with the mask. */
static const ChannelConfig channels[MS5607_CHANNEL_COUNT] = {
    {CS1_J1_GPIO_Port,   CS1_J1_Pin,   0},
    {CS2_J2_GPIO_Port,   CS2_J2_Pin,   0},
    {CS3_J3_GPIO_Port,   CS3_J3_Pin,   0},
    {CS4_J4_GPIO_Port,   CS4_J4_Pin,   1},
    {CS5_J5_GPIO_Port,   CS5_J5_Pin,   1},
    {CS6_J6_GPIO_Port,   CS6_J6_Pin,   1},
    {CS7_J7_GPIO_Port,   CS7_J7_Pin,   2},
    {CS8_J8_GPIO_Port,   CS8_J8_Pin,   2},
    {CS9_J9_GPIO_Port,   CS9_J9_Pin,   2},
    {CS10_J10_GPIO_Port, CS10_J10_Pin, 0},
    {CS11_J11_GPIO_Port, CS11_J11_Pin, 1},
    {CS12_J12_GPIO_Port, CS12_J12_Pin, 2}
};

/** Lifecycle of one connector position; only ACTIVE sensors join cycles. */
typedef enum {
    SENSOR_OFF,       /**< Not selected in the enabled mask (or configuration error). */
    SENSOR_ACTIVE,    /**< Calibrated; participates in every acquisition cycle. */
    SENSOR_RESETTING, /**< Reset sent; waiting for the PROM reload, then PROM re-check. */
    SENSOR_ABSENT     /**< Uncalibrated or recovery due; next reset at next_retry_ms. */
} SensorState;

/** Calibration, raw ADC values and fault bookkeeping of one sensor, never shared. */
typedef struct {
    uint16_t prom[8];             /**< Factory PROM: coefficients C1-C6 at indices 1-6. */
    uint32_t raw_pressure;        /**< Most recent 24-bit D1 pressure ADC result. */
    uint32_t raw_temperature;     /**< Most recent 24-bit D2 temperature ADC result. */
    SensorState state;            /**< Participation and recovery state. */
    uint8_t consecutive_failures; /**< Bad cycles in a row; cleared by a valid sample. */
    bool cycle_failed;            /**< Already failed in the current cycle; skip later phases. */
    uint8_t flags;                /**< Pending one-shot MS5607_FLAG_* bits for the next valid sample. */
    uint16_t error_count;         /**< Saturating count of all error events; streamed. */
    uint32_t reset_sent_us;       /**< TIM2 time the recovery reset command completed. */
    uint32_t next_retry_ms;       /**< HAL tick of the next reset attempt while absent. */
    bool reinit_pending;          /**< In a B1 re-init; cleared when its recovery succeeds or fails. */
} Sensor;

/** Transfer ownership shared between foreground code and HAL callbacks. */
typedef enum {
    XFER_IDLE,   /**< Bus context is free to start another transfer. */
    XFER_BUSY,   /**< HAL interrupt transfer is in progress. */
    XFER_DONE,   /**< Callback completed; foreground has not consumed the data. */
    XFER_FAILED  /**< Start/error callback failed; foreground must abort the transfer. */
} TransferState;

/** Persistent transfer context; buffers must outlive asynchronous HAL calls. */
typedef struct {
    SPI_HandleTypeDef *spi;          /**< Generated HAL handle owned by this bus. */
    IRQn_Type irq;                  /**< NVIC interrupt associated with the handle. */
    uint8_t tx[4];                  /**< Command plus up to three dummy bytes. */
    uint8_t rx[4];                  /**< Received command response and result bytes. */
    uint8_t cursor;                 /**< Next channel index to examine this phase. */
    uint8_t channel;                /**< Channel owning the current/finished transfer. */
    uint8_t consecutive_failures;   /**< Failed transfers in a row; cleared by any success. */
    bool needs_init;                /**< HAL_SPI_Init failed; reinitialize before reuse. */
    volatile TransferState transfer; /**< Current ISR/foreground handoff state. */
    volatile uint32_t hal_error;     /**< Saved HAL error bits before abort clears them. */
    uint32_t started_us;             /**< Software start time for transfer timeout. */
} Bus;

/** Global acquisition phase; every participating bus finishes before advancing. */
typedef enum {
    IDLE,              /**< Wait for the next common sampling deadline. */
    START_TEMPERATURE, /**< Send D2 conversion commands to enabled sensors. */
    WAIT_TEMPERATURE,  /**< Allow the last commanded sensor to finish D2. */
    READ_TEMPERATURE,  /**< Collect each sensor's D2 ADC result. */
    START_PRESSURE,    /**< Send closely spaced D1 conversion commands. */
    WAIT_PRESSURE,     /**< Allow the last commanded sensor to finish D1. */
    READ_PRESSURE      /**< Collect D1, then compensate and publish the frame. */
} Phase;

/* Private state -------------------------------------------------------------*/
static Sensor sensors[MS5607_CHANNEL_COUNT]; /**< Per-connector calibration/raw data. */
static Bus buses[BUS_COUNT];                /**< Independent SPI1/SPI2/SPI3 contexts. */
static MS5607_Frame working;                /**< Measurements being assembled. */
static MS5607_Frame published;              /**< Last complete, foreground-readable frame. */
static MS5607_Diagnostics diagnostics;      /**< Accumulated error events and timing maxima. */
static Phase phase;                        /**< Current acquisition phase. */
static bool active;                        /**< Scheduler runs: channels enabled, config valid. */
static bool config_error;                  /**< Missing handle or invalid timer clock; permanent. */
static bool fault_recent;                  /**< last_fault_ms is valid and may be within the hold. */
static uint32_t last_fault_ms;             /**< HAL tick of the most recent fault event. */
static bool frame_unread;                  /**< Published frame not yet copied by consumer. */
static uint8_t recovery_cursor;            /**< First connector examined by the next recovery step. */
static uint32_t next_cycle_us;             /**< Next intended common sampling deadline. */
static uint32_t ready_us;                  /**< Earliest safe ADC read time for this phase. */
static uint32_t cycle_start_us;            /**< Actual start used to measure cycle duration. */
static uint32_t sequence;                  /**< Cycle counter, including skipped cycles. */
static volatile bool reinit_requested;     /**< Set by ms5607_request_reinit() (EXTI); consumed in IDLE. */

/* Foreground owns the phase/cursor; ISR only observes the volatile operation
 * latch. It is set before consuming the request and cleared after resume. */
typedef enum { REINIT_NONE, REINIT_BUSES, REINIT_RESETS, REINIT_PROM,
               REINIT_RESUME } ReinitPhase;
static volatile bool reinit_active;
static ReinitPhase reinit_phase;
static uint8_t reinit_cursor;
static bool frame_consumer;
#define STATUS_NOTIFICATIONS 4U
typedef struct { MS5607_Frame frame; MS5607_Status state; } StatusNotification;
static StatusNotification status_notifications[STATUS_NOTIFICATIONS];
static uint8_t status_head, status_tail, status_count;
static uint32_t status_lost;
static bool health_snapshot; /* Keep READY/NaN after resume until real DATA. */

/* Private timing and GPIO functions -----------------------------------------*/
/**
 * @brief Read the shared free-running microsecond clock.
 * @return TIM2 count, wrapping modulo 2^32 (about 71.6 minutes).
 * @pre timebase_init() succeeded; not used by the disabled driver.
 */
static uint32_t now_us(void)
{
    return timebase_now_us();
}

/**
 * @brief Test whether a deadline has arrived, including across counter wrap.
 * @param now Current timestamp (TIM2 microseconds or HAL milliseconds).
 * @param deadline Timestamp in the same unit to compare with now.
 * @return true if now is at or after deadline.
 * @note Compared intervals must be shorter than half the 32-bit counter range.
 */
static bool due(uint32_t now, uint32_t deadline)
{
    return (int32_t)(now - deadline) >= 0;
}

/**
 * @brief Set one sensor's active-low chip select without changing other CS pins.
 * @param channel Connector index, 0-11 for J1-J12.
 * @param level GPIO_PIN_RESET selects the sensor; GPIO_PIN_SET releases it.
 * @note A connector reserved by the sync input (J12) is never written, so no
 *       init, transfer, recovery or B1 path can drive PB10.
 */
static void select_channel(uint8_t channel, GPIO_PinState level)
{
    if ((SYNC_INPUT_RESERVED_MASK & (1U << channel)) != 0U) {
        return;
    }
    HAL_GPIO_WritePin(channels[channel].port, channels[channel].pin, level);
}

/* Private calibration and conversion functions ------------------------------*/
/**
 * @brief Calculate the factory PROM CRC-4 using the MEAS AN520 procedure.
 * @param prom Eight 16-bit PROM words in sensor address order.
 * @return Calculated four-bit CRC, to compare with the low nibble of prom[7].
 * @note The low byte of word 7 is excluded without modifying the saved PROM.
 * @see https://www.amsys-sensor.com/downloads/notes/MS5XXX-C-code-example-for-MS56xx-MS57xx-MS58xx-AMSYS-an520e.pdf
 */
static uint8_t prom_crc4(const uint16_t prom[8])
{
    uint16_t remainder = 0; /* Polynomial division register. */
    /* byte selects the high then low byte of each of the eight PROM words. */
    for (uint8_t byte = 0; byte < 16U; ++byte) {
        uint16_t word = prom[byte / 2U]; /* Local copy allows masking the CRC byte. */
        if (byte / 2U == 7U) {
            word &= 0xFF00U;
        }
        remainder ^= (byte & 1U) ? (word & 0xFFU) : (word >> 8);
        for (uint8_t bit = 0; bit < 8U; ++bit) { /* Eight polynomial steps per byte. */
            remainder = (uint16_t)((remainder << 1) ^
                         ((remainder & 0x8000U) ? 0x3000U : 0U));
        }
    }
    return (uint8_t)((remainder >> 12) & 0xFU);
}

/**
 * @brief Reject empty calibration and check its stored CRC.
 * @param prom Eight PROM words read from a single sensor.
 * @return true if coefficients are not all zero/all ones and CRC matches.
 * @note SPI transfer success alone does not establish that a sensor is attached.
 */
static bool valid_prom(const uint16_t prom[8])
{
    bool all_zero = true; /* Detect an empty or permanently low data line. */
    bool all_ones = true; /* Detect an empty or permanently high data line. */
    for (uint8_t i = 1; i <= 6U; ++i) { /* Only factory coefficients C1-C6. */
        all_zero &= prom[i] == 0U;
        all_ones &= prom[i] == 0xFFFFU;
    }
    return !all_zero && !all_ones && prom_crc4(prom) == (prom[7] & 0xFU);
}

/**
 * @brief Convert raw D1/D2 values to compensated pressure and temperature.
 * @param sensor Sensor-specific factory coefficients and completed ADC readings.
 * @param measurement Destination; only pressure_pa and temperature_c are changed.
 * @note TE MS5607-02BA03 datasheet (03/2024), pp. 8-9, Figures 2 and 3.
 *       These are MS5607 equations; MS5611 uses different scale factors.
 *       Wide signed intermediates prevent overflow. Integer divisions truncate
 *       toward zero; output pressure is in Pa and temperature in degrees C.
 * @see https://www.tti.com/content/dam/tti-commons/supplier/te-connectivity/doc/te-connectivity-ms5607-barometric-pressure-sensor-datasheet-and-specifications.pdf
 */
static void compensate(const Sensor *sensor, MS5607_Measurement *measurement)
{
    const uint16_t *c = sensor->prom; /* c[1]..c[6] are the factory C1..C6 values. */
    /* First order: dT = D2 - C5*2^8; TEMP = 2000 + dT*C6/2^23. */
    int64_t dt = (int64_t)sensor->raw_temperature - (int64_t)c[5] * 256; /* Raw T deviation. */
    int64_t temperature = 2000 + dt * c[6] / 8388608; /* Temperature in 0.01 degrees C. */
    int64_t offset = (int64_t)c[2] * 131072 + dt * c[4] / 64; /* OFF = C2*2^17 + dT*C4/2^6. */
    int64_t sensitivity = (int64_t)c[1] * 65536 + dt * c[3] / 128; /* SENS = C1*2^16 + dT*C3/2^7. */
    /* Second order applies below 20 degrees C, with extra terms below -15 C. */
    if (temperature < 2000) {
        int64_t square = (temperature - 2000) * (temperature - 2000); /* Squared temperature deviation. */
        int64_t off2 = 61 * square / 16; /* Second-order offset correction OFF2. */
        int64_t sens2 = 2 * square;     /* Second-order sensitivity correction SENS2. */
        if (temperature < -1500) {
            square = (temperature + 1500) * (temperature + 1500);
            off2 += 15 * square;
            sens2 += 8 * square;
        }
        temperature -= dt * dt / 2147483648LL; /* T2 = dT^2 / 2^31. */
        offset -= off2;
        sensitivity -= sens2;
    }
    /* P = (D1*SENS/2^21 - OFF)/2^15; 0.01 mbar equals exactly 1 Pa. */
    int64_t pressure = ((int64_t)sensor->raw_pressure * sensitivity / 2097152
                        - offset) / 32768;
    measurement->pressure_pa = (float)pressure;
    measurement->temperature_c = (float)temperature / 100.0f;
}

/**
 * @brief Check a compensated result against the sensor's physical range.
 * @param measurement Slot just written by compensate().
 * @return true if pressure and temperature are both inside their limits.
 * @note Results outside the range indicate a corrupted read or a damaged
 *       sensor and are treated as ADC errors (change_spec_v3.md, C2).
 */
static bool in_range(const MS5607_Measurement *measurement)
{
    return measurement->pressure_pa >= (float)PRESSURE_MIN_PA &&
           measurement->pressure_pa <= (float)PRESSURE_MAX_PA &&
           measurement->temperature_c >= (float)TEMPERATURE_MIN_C &&
           measurement->temperature_c <= (float)TEMPERATURE_MAX_C;
}

/* Private fault handling functions ------------------------------------------*/
/**
 * @brief Record a fault event for the status hold time.
 * @note Foreground only. Events: transfer, ADC/range and calibration errors,
 *       bus reinitialization, missed cycles and configuration errors.
 */
static void note_fault(void)
{
    last_fault_ms = HAL_GetTick();
    fault_recent = true;
}

/**
 * @brief Write an invalid result into one working slot.
 * @param channel Connector index, 0-11.
 * @param status Reason shown in the next published frame.
 */
static void set_slot(uint8_t channel, MS5607_ChannelStatus status)
{
    working.channel[channel].status = status;
    working.channel[channel].pressure_pa = NAN;
    working.channel[channel].temperature_c = NAN;
}

/**
 * @brief Count one error event for a channel and record the fault time.
 * @param channel Connector index, 0-11.
 * @note The per-channel counter saturates at 65535 instead of wrapping.
 */
static void count_error(uint8_t channel)
{
    if (sensors[channel].error_count < UINT16_MAX) {
        sensors[channel].error_count++;
    }
    note_fault();
}

/**
 * @brief Invalidate one channel for the rest of the current cycle only.
 * @param channel Connector index, 0-11.
 * @param status MS5607_TRANSFER_ERROR or MS5607_ADC_ERROR.
 * @note The caller increments the matching diagnostics counter. The sensor
 *       stays ACTIVE; publish_frame() decides whether recovery is needed.
 */
static void fail_sample(uint8_t channel, MS5607_ChannelStatus status)
{
    sensors[channel].cycle_failed = true;
    set_slot(channel, status);
    count_error(channel);
}

/**
 * @brief Remove a sensor from acquisition and request a reset in the next step.
 * @param channel Connector index, 0-11.
 * @note An ABSENT sensor with a retry time of now is served by the next
 *       recovery step; its slot keeps the last failure status until the reset.
 */
static void enter_recovery(uint8_t channel)
{
    sensors[channel].state = SENSOR_ABSENT;
    sensors[channel].consecutive_failures = 0U;
    sensors[channel].next_retry_ms = HAL_GetTick();
}

/**
 * @brief Mark a calibration/recovery attempt as failed and schedule a retry.
 * @param channel Connector index, 0-11.
 * @param status MS5607_TRANSFER_ERROR or MS5607_CALIBRATION_ERROR.
 * @note Also ends this sensor's part of a B1 re-init, as a normal fault.
 */
static void retry_later(uint8_t channel, MS5607_ChannelStatus status)
{
    sensors[channel].state = SENSOR_ABSENT;
    sensors[channel].consecutive_failures = 0U;
    sensors[channel].next_retry_ms = HAL_GetTick() + MS5607_RETRY_PERIOD_MS;
    sensors[channel].reinit_pending = false;
    set_slot(channel, status);
}

/**
 * @brief Pulse the RCC reset of one SPI peripheral, clearing its FIFOs and flags.
 * @param index Bus index: 0 = SPI1, 1 = SPI2, 2 = SPI3 (fixed by ms5607_init()).
 * @note Call between HAL_SPI_DeInit() and HAL_SPI_Init(); Init rewrites all
 *       configuration registers from the handle.
 */
static void force_reset_spi(uint8_t index)
{
    if (index == 0U) {
        __HAL_RCC_SPI1_FORCE_RESET();
        __HAL_RCC_SPI1_RELEASE_RESET();
    } else if (index == 1U) {
        __HAL_RCC_SPI2_FORCE_RESET();
        __HAL_RCC_SPI2_RELEASE_RESET();
    } else {
        __HAL_RCC_SPI3_FORCE_RESET();
        __HAL_RCC_SPI3_RELEASE_RESET();
    }
}

/**
 * @brief Discard bytes left in the RX FIFO after an aborted transfer.
 * @param spi Handle of the aborted bus.
 * @note HAL_SPI_Abort() flushes the FIFO only while TXEIE/RXNEIE are still
 *       set; after an error callback or rejected start they are already clear.
 *       A stale byte would otherwise shift the next result. Bounded by the
 *       FIFO depth, so it cannot hang.
 */
static void drain_rx_fifo(SPI_HandleTypeDef *spi)
{
    for (uint8_t n = 0; n < RX_FIFO_BYTES &&
         (spi->Instance->SR & SPI_SR_FRLVL) != 0U; ++n) { /* Bytes discarded. */
        (void)*(volatile uint8_t *)&spi->Instance->DR;
    }
}

/**
 * @brief Reset and reinitialize one SPI peripheral (hardware part only).
 * @param index Bus index: 0 = SPI1, 1 = SPI2, 2 = SPI3.
 * @return true if HAL_SPI_Init succeeded.
 * @note Foreground only, with no interrupt transfer active on this bus. Uses
 *       the Init settings stored in the generated handle; an RCC reset between
 *       DeInit and Init clears the FIFOs. Clears the bus failure count. No
 *       fault bookkeeping: callers decide what the reset means for sensors.
 */
static bool restart_spi(uint8_t index)
{
    Bus *bus = &buses[index]; /* Transport context being reinitialized. */
    bus->consecutive_failures = 0U;
    bus->transfer = XFER_IDLE;
    (void)HAL_SPI_DeInit(bus->spi);
    force_reset_spi(index);
    bool ok = HAL_SPI_Init(bus->spi) == HAL_OK; /* Peripheral usable again. */
    bus->needs_init = !ok;
    HAL_NVIC_ClearPendingIRQ(bus->irq);
    return ok;
}

/**
 * @brief Reinitialize one SPI peripheral after repeated transfer failures.
 * @param index Bus index: 0 = SPI1, 1 = SPI2, 2 = SPI3.
 * @return true if HAL_SPI_Init succeeded.
 * @note Foreground only, with no interrupt transfer active on this bus. Counts
 *       a bus reset fault event. Active sensors on the
 *       bus enter recovery, because their state after a bus fault is unknown.
 *       If Init fails, every enabled sensor on the bus becomes ABSENT and the
 *       next recovery attempt for one of them reinitializes the bus first.
 */
static bool reset_bus(uint8_t index)
{
    diagnostics.bus_resets++;
    note_fault();
    bool ok = restart_spi(index); /* Peripheral usable again. */
    for (uint8_t i = 0; i < MS5607_CHANNEL_COUNT; ++i) { /* Connectors on this bus. */
        Sensor *sensor = &sensors[i]; /* Candidate sensor on the reset bus. */
        if (channels[i].bus != index || sensor->state == SENSOR_OFF) {
            continue;
        }
        if (!ok) {
            sensor->cycle_failed = true;
            retry_later(i, MS5607_TRANSFER_ERROR);
        } else if (sensor->state == SENSOR_ACTIVE) {
            if (!sensor->cycle_failed) {
                set_slot(i, MS5607_TRANSFER_ERROR); /* This cycle's sample is lost. */
            }
            sensor->cycle_failed = true;
            enter_recovery(i);
        }
    }
    return ok;
}

/**
 * @brief Terminate a failed transfer, invalidate its channel and track bus health.
 * @param index Bus index: 0 = SPI1, 1 = SPI2, 2 = SPI3.
 * @note Foreground only. bus->channel and bus->hal_error must describe the
 *       failed transfer. IRQs stay enabled during HAL abort so it can progress;
 *       leftover RX bytes are drained afterwards.
 *       HAL's internal abort waits can exceed the foreground transfer deadline.
 *       After MS5607_BUS_FAILURE_LIMIT failures in a row the bus is reset.
 */
static void fail_transfer(uint8_t index)
{
    Bus *bus = &buses[index];        /* Transport context of the failed transfer. */
    uint8_t channel = bus->channel;  /* Connector that owned the transfer. */
    diagnostics.transfer_errors++;
    diagnostics.last_hal_error = bus->hal_error;
    (void)HAL_SPI_Abort(bus->spi);
    drain_rx_fifo(bus->spi);
    select_channel(channel, GPIO_PIN_SET);
    bus->transfer = XFER_IDLE;
    fail_sample(channel, MS5607_TRANSFER_ERROR);
    if (++bus->consecutive_failures >= MS5607_BUS_FAILURE_LIMIT && !reinit_active) {
        (void)reset_bus(index);
    }
}

/* Private blocking reset/PROM functions -------------------------------------*/
/**
 * @brief Perform one blocking reset or PROM transaction.
 * @param channel Connector index, 0-11.
 * @param command Sensor command placed in the first transmitted byte.
 * @param length Total byte count, including command and result clocks (1-4).
 * @param timeout_ms HAL timeout for the whole transaction.
 * @return true on success; result bytes remain in the bus RX buffer.
 * @note Startup, or the idle gap with no interrupt transfer active. CS is
 *       released on either path; a failure is handled by fail_transfer().
 */
static bool blocking_transfer(uint8_t channel, uint8_t command, uint8_t length,
                              uint32_t timeout_ms)
{
    uint8_t index = channels[channel].bus; /* SPI bus assigned to this connector. */
    Bus *bus = &buses[index];              /* Persistent transfer buffers. */
    memset(bus->tx, 0, sizeof(bus->tx));
    bus->tx[0] = command;
    bus->channel = channel;
    select_channel(channel, GPIO_PIN_RESET);
    /* result distinguishes successful byte exchange from HAL timeout/error. */
    HAL_StatusTypeDef result = HAL_SPI_TransmitReceive(bus->spi, bus->tx,
                                                     bus->rx, length,
                                                     timeout_ms);
    select_channel(channel, GPIO_PIN_SET);
    if (result != HAL_OK) {
        bus->hal_error = HAL_SPI_GetError(bus->spi);
        fail_transfer(index);
        return false;
    }
    bus->consecutive_failures = 0U;
    return true;
}

/**
 * @brief Test whether a blocking recovery transfer group may start now.
 * @return true if at least RECOVERY_GUARD_US remain before the next deadline.
 */
static bool guard_left(void)
{
    return (int32_t)(next_cycle_us - now_us()) >= (int32_t)RECOVERY_GUARD_US;
}

/**
 * @brief Read the eight PROM words of one sensor.
 * @param channel Connector index, 0-11.
 * @param timeout_ms HAL timeout for each word transfer.
 * @param prom Destination for the words in address order.
 * @return true if all eight transfers succeeded (content not yet checked).
 * @note A transfer failure is handled by fail_transfer(); the caller schedules
 *       the retry.
 */
static bool read_prom(uint8_t channel, uint32_t timeout_ms, uint16_t prom[8])
{
    const Bus *bus = &buses[channels[channel].bus]; /* Holds each PROM response. */
    for (uint8_t word = 0; word < 8U; ++word) { /* PROM word address, 0-7. */
        if (!blocking_transfer(channel, (uint8_t)(CMD_PROM_READ + 2U * word), 3U,
                               timeout_ms)) {
            return false;
        }
        prom[word] = (uint16_t)((uint16_t)bus->rx[1] << 8) | bus->rx[2];
    }
    return true;
}

/**
 * @brief Compare two PROM images word by word, including the CRC word.
 * @param a First image.
 * @param b Second image.
 * @return true if all eight words are equal.
 */
static bool same_prom(const uint16_t a[8], const uint16_t b[8])
{
    bool same = true; /* Cleared by the first differing word. */
    for (uint8_t word = 0; word < 8U; ++word) { /* Compare every word. */
        same &= a[word] == b[word];
    }
    return same;
}

/**
 * @brief Count a failed calibration attempt and schedule a retry.
 * @param channel Connector index, 0-11.
 */
static void calibration_failed(uint8_t channel)
{
    diagnostics.calibration_errors++;
    count_error(channel);
    retry_later(channel, MS5607_CALIBRATION_ERROR);
}

/**
 * @brief Read and check the PROM, then activate the sensor.
 * @param channel Connector index, 0-11.
 * @param timeout_ms HAL timeout for each word transfer.
 * @param recovery false at startup; true for a re-check after a reset.
 * @note The PROM is read into local copies, so an invalid read never replaces
 *       a valid stored calibration. A read equal to the stored valid PROM is
 *       accepted at once. Any other valid read (startup, hot-plug, exchanged
 *       sensor) is adopted only if a second read in the same step returns
 *       identical words: CRC-4 alone passes about 1 in 16 garbage reads, for
 *       example from a floating MISO line. Disagreeing reads count as a
 *       calibration failure. In recovery the second read starts only if the
 *       deadline guard still allows it; otherwise the whole PROM step is
 *       repeated in a later cycle and the sensor stays RESETTING. Recovery sets
 *       RECOVERED; replacing a valid stored PROM also sets PROM_CHANGED. Any
 *       failure leaves the sensor ABSENT with a retry after the period.
 *       Success or failure ends this sensor's part of a B1 re-init.
 */
static void read_calibration(uint8_t channel, uint32_t timeout_ms, bool recovery)
{
    Sensor *sensor = &sensors[channel]; /* Sensor being calibrated. */
    uint16_t prom[8];                   /* First candidate calibration read. */
    uint16_t confirm[8];                /* Second read; must match the first. */
    if (!read_prom(channel, timeout_ms, prom)) {
        retry_later(channel, MS5607_TRANSFER_ERROR);
        return;
    }
    if (!valid_prom(prom)) {
        calibration_failed(channel);
        return;
    }
    bool stored_valid = valid_prom(sensor->prom); /* A calibration is already known. */
    if (!stored_valid || !same_prom(prom, sensor->prom)) {
        if (recovery && !reinit_active && !guard_left()) {
            diagnostics.recovery_postponed++; /* Repeat the PROM step next cycle. */
            return;
        }
        if (!read_prom(channel, timeout_ms, confirm)) {
            retry_later(channel, MS5607_TRANSFER_ERROR);
            return;
        }
        if (!same_prom(prom, confirm)) {
            calibration_failed(channel);
            return;
        }
        if (recovery && stored_valid) {
            sensor->flags |= MS5607_FLAG_PROM_CHANGED;
            diagnostics.prom_changes++;
        }
        memcpy(sensor->prom, prom, sizeof(sensor->prom));
    }
    if (recovery) {
        sensor->flags |= MS5607_FLAG_RECOVERED;
    } else {
        set_slot(channel, MS5607_READY);
    }
    sensor->consecutive_failures = 0U;
    sensor->reinit_pending = false;
    sensor->state = SENSOR_ACTIVE;
}

/* Public startup function ---------------------------------------------------*/
/**
 * @brief Initialize driver state and calibrate the enabled sensor channels.
 * @param spi1 Generated handle for SPI1 (J1, J2, J3, J10).
 * @param spi2 Generated handle for SPI2 (J4, J5, J6, J11).
 * @param spi3 Generated handle for SPI3 (J7, J8, J9, J12).
 * @note Call once after generated GPIO/SPI and BSP setup. A zero mask returns
 *       without sensor traffic or SPI IRQ enable. Otherwise it ensures the
 *       shared timebase runs (normally already started by main.c); reset and
 *       PROM reads can block briefly. Recurring acquisition does not wait
 *       in place for sensor conversions. Sensors failing here are re-probed
 *       every MS5607_RETRY_PERIOD_MS. Read ms5607_status() for failures.
 */
void ms5607_init(SPI_HandleTypeDef *spi1, SPI_HandleTypeDef *spi2,
                 SPI_HandleTypeDef *spi3)
{
    /* Startup-only API: do not call again while transfers are active. */
    memset(sensors, 0, sizeof(sensors));
    memset(buses, 0, sizeof(buses));
    memset(&working, 0, sizeof(working));
    memset(&diagnostics, 0, sizeof(diagnostics));
    buses[0].spi = spi1; buses[0].irq = SPI1_IRQn;
    buses[1].spi = spi2; buses[1].irq = SPI2_IRQn;
    buses[2].spi = spi3; buses[2].irq = SPI3_IRQn;
    active = false; /* Before the flag reset: requests are ignored until init ends. */
    reinit_requested = false;
    reinit_active = false;
    reinit_phase = REINIT_NONE;
    frame_consumer = false;
    status_head = status_tail = status_count = 0U;
    status_lost = 0U;
    health_snapshot = false;
    config_error = false;
    fault_recent = false;
    last_fault_ms = 0U;
    frame_unread = false;
    recovery_cursor = 0U;
    sequence = 0U;
    phase = IDLE;
    for (uint8_t i = 0; i < MS5607_CHANNEL_COUNT; ++i) { /* Initialize every output slot. */
        select_channel(i, GPIO_PIN_SET);
        set_slot(i, MS5607_DISABLED);
    }
    published = working;
    if (MS5607_ENABLED_MASK == 0U) {
        return;
    }
    if (spi1 == NULL || spi2 == NULL || spi3 == NULL || !timebase_init()) {
        config_error = true; /* Permanent: retrying cannot fix the build. */
        note_fault();
        for (uint8_t i = 0; i < MS5607_CHANNEL_COUNT; ++i) { /* Flag enabled connectors. */
            if ((MS5607_ENABLED_MASK & (1U << i)) != 0U) {
                set_slot(i, MS5607_CONFIG_ERROR);
            }
        }
        published = working;
        return;
    }

    for (uint8_t i = 0; i < MS5607_CHANNEL_COUNT; ++i) { /* Enabled, not yet calibrated. */
        if ((MS5607_ENABLED_MASK & (1U << i)) != 0U) {
            sensors[i].state = SENSOR_ABSENT;
        }
    }
    for (uint8_t i = 0; i < MS5607_CHANNEL_COUNT; ++i) { /* Probe enabled connectors. */
        if (sensors[i].state == SENSOR_OFF || buses[channels[i].bus].needs_init) {
            continue; /* A failed bus reset already scheduled a retry. */
        }
        if (!blocking_transfer(i, CMD_RESET, 1U, STARTUP_TIMEOUT_MS)) {
            retry_later(i, MS5607_TRANSFER_ERROR);
            continue;
        }
        HAL_Delay(4U); /* At least the 2.8 ms PROM reload time, startup only. */
        read_calibration(i, STARTUP_TIMEOUT_MS, false);
    }
    for (uint8_t b = 0; b < BUS_COUNT; ++b) { /* Enable IRQs of buses with enabled channels. */
        for (uint8_t i = 0; i < MS5607_CHANNEL_COUNT; ++i) { /* Find one enabled connector. */
            if (channels[i].bus == b && sensors[i].state != SENSOR_OFF) {
                HAL_NVIC_SetPriority(buses[b].irq, 1U, 0U); /* SysTick remains priority 0. */
                HAL_NVIC_ClearPendingIRQ(buses[b].irq);
                HAL_NVIC_EnableIRQ(buses[b].irq);
                break;
            }
        }
    }
    for (uint8_t i = 0; i < MS5607_CHANNEL_COUNT; ++i) { /* Show startup error counts. */
        working.channel[i].error_count = sensors[i].error_count;
    }
    active = true; /* Scheduler runs even if no sensor calibrated (hot-plug). */
    published = working;
    next_cycle_us = now_us();
}

/* Private acquisition scheduling functions ----------------------------------*/
/**
 * @brief Enter a transfer phase and restart each bus's channel scan.
 * @param next New START or READ phase; previous transfers must be retired.
 */
static void begin_phase(Phase next)
{
    phase = next;
    for (uint8_t b = 0; b < BUS_COUNT; ++b) { /* SPI1/SPI2/SPI3 context index. */
        buses[b].cursor = 0U;
    }
}

/**
 * @brief Retire finished transfers and start the next sensor on each free bus.
 * @return true when every bus has finished or failed all its participating channels.
 * @note At most one new transaction per bus per call. Start calls are issued
 *       back-to-back without awaiting another bus's completion or another tick.
 *       ADC bytes are consumed only after the successful completion callback.
 *       A failed transfer invalidates only its channel; the bus moves on to
 *       its next channel in the same call. Only ACTIVE channels that have not
 *       failed in this cycle participate.
 */
static bool service_transfers(void)
{
    bool finished = true; /* Cleared if any bus still has work in this phase. */
    for (uint8_t b = 0; b < BUS_COUNT; ++b) { /* Independent bus index. */
        Bus *bus = &buses[b]; /* Current bus's persistent transfer state. */
        TransferState state = bus->transfer; /* Snapshot of IRQ/foreground handoff. */
        if (state == XFER_FAILED ||
            (state == XFER_BUSY && now_us() - bus->started_us >= TRANSFER_TIMEOUT_US)) {
            fail_transfer(b);
        } else if (state == XFER_BUSY) {
            finished = false;
            continue;
        } else if (state == XFER_DONE) {
            __DMB(); /* Pairs with the ISR barrier: read rx[] only after DONE. */
            bus->consecutive_failures = 0U;
            if (phase == READ_TEMPERATURE || phase == READ_PRESSURE) {
                /* raw is the 24-bit, MSB-first ADC value; rx[0] is discarded. */
                uint32_t raw = ((uint32_t)bus->rx[1] << 16) |
                               ((uint32_t)bus->rx[2] << 8) | bus->rx[3];
                if (raw == 0U || raw == 0xFFFFFFU) {
                    diagnostics.adc_errors++;
                    fail_sample(bus->channel, MS5607_ADC_ERROR);
                } else if (phase == READ_TEMPERATURE) {
                    sensors[bus->channel].raw_temperature = raw;
                } else {
                    sensors[bus->channel].raw_pressure = raw;
                }
            }
            bus->transfer = XFER_IDLE;
        }
        uint8_t channel = bus->cursor; /* Next eligible connector on this bus. */
        while (channel < MS5607_CHANNEL_COUNT &&
               (channels[channel].bus != b ||
                sensors[channel].state != SENSOR_ACTIVE ||
                sensors[channel].cycle_failed)) {
            ++channel;
        }
        bus->cursor = channel;
        if (channel == NO_CHANNEL) {
            continue;
        }
        bus->cursor++;
        bus->channel = channel;
        memset(bus->tx, 0, sizeof(bus->tx));
        uint8_t length = 1U; /* Command length; ADC reads require four bytes. */
        if (phase == START_TEMPERATURE) {
            bus->tx[0] = CMD_TEMPERATURE;
        } else if (phase == START_PRESSURE) {
            bus->tx[0] = CMD_PRESSURE;
        } else {
            length = 4U; /* ADC read (0x00) plus 24 result clocks. */
        }
        bus->started_us = now_us();
        if (phase == START_TEMPERATURE) {
            working.channel[channel].temperature_start_us = bus->started_us;
        } else if (phase == START_PRESSURE) {
            working.channel[channel].pressure_start_us = bus->started_us;
        }
        bus->hal_error = HAL_SPI_ERROR_NONE;
        __DMB();
        bus->transfer = XFER_BUSY; /* Set ownership before interrupts can fire. */
        select_channel(channel, GPIO_PIN_RESET);
        /* result reports whether HAL accepted the asynchronous start request. */
        HAL_StatusTypeDef result = HAL_SPI_TransmitReceive_IT(bus->spi, bus->tx,
                                                            bus->rx, length);
        if (result != HAL_OK) {
            bus->hal_error = HAL_SPI_GetError(bus->spi);
            bus->transfer = XFER_FAILED;
            /* Recover on the next pass, after other buses have been started. */
        }
        finished = false;
    }
    return finished;
}

/**
 * @brief Compensate successful readings and publish one complete twelve-slot frame.
 * @note Foreground only, after the pressure-read phase is complete. Failed
 *       slots hold NaN and the cycle's error status. Out-of-range results are
 *       ADC errors. A sensor failing MS5607_RESET_AFTER_FAILURES cycles in a
 *       row enters recovery. Pending one-shot flags go into the first valid
 *       sample and are cleared in the following frame. Record overwritten
 *       snapshots and late cycles; missed deadlines are skipped so acquisition
 *       never performs burst catch-up.
 */
static void publish_frame(void)
{
    for (uint8_t i = 0; i < MS5607_CHANNEL_COUNT; ++i) { /* Output connector index. */
        Sensor *sensor = &sensors[i];            /* Channel state and raw results. */
        MS5607_Measurement *slot = &working.channel[i]; /* Channel output slot. */
        slot->flags = 0U;
        if (sensor->state == SENSOR_ACTIVE) {
            if (!sensor->cycle_failed) {
                compensate(sensor, slot);
                if (!in_range(slot)) {
                    diagnostics.adc_errors++;
                    fail_sample(i, MS5607_ADC_ERROR);
                }
            }
            if (sensor->cycle_failed) {
                if (++sensor->consecutive_failures >= MS5607_RESET_AFTER_FAILURES) {
                    enter_recovery(i);
                }
            } else {
                slot->sequence = working.sequence;
                slot->status = MS5607_VALID;
                slot->flags = sensor->flags;
                sensor->flags = 0U;
                sensor->consecutive_failures = 0U;
            }
        }
        slot->error_count = sensor->error_count;
    }
    if (frame_unread) {
        diagnostics.overwritten_frames++;
        /* Latest-value storage is intentional until a streamer consumes it.
         * This count alone does not turn the LED into a sensor fault. */
    }
    published = working;
    health_snapshot = false;
    frame_unread = true;
    uint32_t now = now_us();                 /* Actual publication timestamp. */
    uint32_t duration = now - cycle_start_us; /* Full cycle execution time in us. */
    if (duration > diagnostics.max_cycle_us) {
        diagnostics.max_cycle_us = duration;
    }
    if (due(now, next_cycle_us)) {
        uint32_t skipped = (now - next_cycle_us) / SAMPLE_PERIOD_US + 1U; /* Elapsed cycle deadlines. */
        diagnostics.missed_cycles += skipped;
        sequence += skipped;
        next_cycle_us += skipped * SAMPLE_PERIOD_US;
        note_fault();
    }
    phase = IDLE;
}

/**
 * @brief Run at most one blocking recovery step in the idle gap after publication.
 * @note Foreground only, in IDLE with no interrupt transfer active. Skipped if
 *       less than RECOVERY_GUARD_US remain before the next cycle deadline; the
 *       step is then postponed to the next cycle and counted in
 *       diagnostics.recovery_postponed. The PROM step re-checks the guard
 *       before a confirming second read. Candidates are served
 *       round-robin by connector index: a RESETTING sensor whose PROM reload
 *       time has passed gets its PROM step; an ABSENT sensor whose retry time
 *       has passed gets a reset step (after reinitializing its bus if needed).
 */
static void recover_one(void)
{
    uint32_t now = now_us(); /* Step start time for the reload checks. */
    uint32_t tick = HAL_GetTick(); /* Millisecond time for retry deadlines. */
    for (uint8_t k = 0; k < MS5607_CHANNEL_COUNT; ++k) { /* Round-robin offset. */
        uint8_t i = (uint8_t)((recovery_cursor + k) % MS5607_CHANNEL_COUNT); /* Candidate connector. */
        Sensor *sensor = &sensors[i]; /* Candidate sensor state. */
        bool prom_due = sensor->state == SENSOR_RESETTING &&
                        due(now, sensor->reset_sent_us + RESET_WAIT_US); /* PROM reloaded. */
        bool reset_due = sensor->state == SENSOR_ABSENT &&
                         due(tick, sensor->next_retry_ms); /* Retry time reached. */
        if (!prom_due && !reset_due) {
            continue;
        }
        if (!guard_left()) {
            diagnostics.recovery_postponed++; /* Same candidate first next cycle. */
            return;
        }
        recovery_cursor = (uint8_t)((i + 1U) % MS5607_CHANNEL_COUNT);
        if (prom_due) {
            read_calibration(i, RECOVERY_TIMEOUT_MS, true);
            return;
        }
        if (buses[channels[i].bus].needs_init && !reset_bus(channels[i].bus)) {
            return; /* Every sensor on the bus was rescheduled by reset_bus(). */
        }
        if (!blocking_transfer(i, CMD_RESET, 1U, RECOVERY_TIMEOUT_MS)) {
            retry_later(i, MS5607_TRANSFER_ERROR);
            return;
        }
        diagnostics.sensor_resets++;
        sensor->state = SENSOR_RESETTING;
        sensor->reset_sent_us = now_us();
        set_slot(i, MS5607_RECOVERING);
        return;
    }
}

/** The operation latch also covers the final calibration/resume window. */
static bool reinit_running(void) { return reinit_active; }

void ms5607_copy_status(MS5607_Frame *out, MS5607_Status *state)
{
    if (out != NULL) {
        *out = health_snapshot ? working : published;
        if (health_snapshot) {
            for (uint8_t i = 0; i < MS5607_CHANNEL_COUNT; ++i) {
                out->channel[i].error_count = sensors[i].error_count;
                out->channel[i].flags = 0U;
            }
        }
    }
    if (state != NULL) { *state = ms5607_status(); }
}

/* FIFO preserves a short start/end pair. Overflow drops the new notification;
 * the stream claims status_lost and applies its existing visible loss policy. */
static void notify_status(void)
{
    if (status_count == STATUS_NOTIFICATIONS) { status_lost++; return; }
    StatusNotification *entry = &status_notifications[status_head];
    ms5607_copy_status(&entry->frame, &entry->state);
    status_head = (uint8_t)((status_head + 1U) % STATUS_NOTIFICATIONS);
    status_count++;
}

bool ms5607_pop_status(MS5607_Frame *out, MS5607_Status *state)
{
    if (status_count == 0U || out == NULL || state == NULL) { return false; }
    *out = status_notifications[status_tail].frame;
    *state = status_notifications[status_tail].state;
    status_tail = (uint8_t)((status_tail + 1U) % STATUS_NOTIFICATIONS);
    status_count--;
    return true;
}

uint32_t ms5607_status_drops(void)
{
    uint32_t lost = status_lost;
    status_lost = 0U;
    return lost;
}

void ms5607_set_frame_consumer(bool enabled) { frame_consumer = enabled; }

/** Enter only after final DATA handoff (or without a registered consumer).
 * No SPI here: each later pass performs one bus or sensor step. */
static void start_reinit(void)
{
    reinit_active = true; /* Close ISR acceptance window before clearing request. */
    reinit_requested = false;
    diagnostics.user_reinits++;
    reinit_phase = REINIT_BUSES;
    reinit_cursor = 0U;
    ready_us = 0U;
    health_snapshot = true;
    for (uint8_t i = 0; i < MS5607_CHANNEL_COUNT; ++i) {
        select_channel(i, GPIO_PIN_SET);
        if (sensors[i].state == SENSOR_OFF) { continue; }
        enter_recovery(i);
        sensors[i].reinit_pending = true;
        sensors[i].cycle_failed = false;
        set_slot(i, MS5607_RECOVERING);
        working.channel[i].flags = 0U;
    }
    notify_status();
}

/** Bounded work independent of next_cycle_us; no conversions in this operation.
 * Successful calibration is internally ACTIVE, but the global gate prevents
 * sampling until every attempt is terminal. Failed channels are never retried
 * here and SPI errors cannot invoke nested bus resets. */
static void service_reinit(void)
{
    if (reinit_phase == REINIT_BUSES) {
        if (reinit_cursor < BUS_COUNT) {
            uint8_t b = reinit_cursor++;
            bool participating = false;
            for (uint8_t i = 0; i < MS5607_CHANNEL_COUNT; ++i) {
                participating |= channels[i].bus == b && sensors[i].reinit_pending;
            }
            if (participating && !restart_spi(b)) {
                for (uint8_t i = 0; i < MS5607_CHANNEL_COUNT; ++i) {
                    if (channels[i].bus == b && sensors[i].reinit_pending) {
                        count_error(i);
                        retry_later(i, MS5607_TRANSFER_ERROR);
                    }
                }
            }
            return;
        }
        reinit_cursor = 0U;
        reinit_phase = REINIT_RESETS;
    }
    if (reinit_phase == REINIT_RESETS) {
        while (reinit_cursor < MS5607_CHANNEL_COUNT) {
            uint8_t i = reinit_cursor++;
            if (!sensors[i].reinit_pending) { continue; }
            if (!blocking_transfer(i, CMD_RESET, 1U, RECOVERY_TIMEOUT_MS)) {
                retry_later(i, MS5607_TRANSFER_ERROR);
            } else {
                diagnostics.sensor_resets++;
                sensors[i].state = SENSOR_RESETTING;
                sensors[i].reset_sent_us = now_us();
            }
            return;
        }
        reinit_phase = REINIT_PROM;
    }
    if (reinit_phase == REINIT_PROM) {
        bool pending_attempt = false;
        for (uint8_t i = 0; i < MS5607_CHANNEL_COUNT; ++i) {
            if (!sensors[i].reinit_pending) { continue; }
            pending_attempt = true;
            if (due(now_us(), sensors[i].reset_sent_us + RESET_WAIT_US)) {
                read_calibration(i, RECOVERY_TIMEOUT_MS, true);
                return;
            }
        }
        if (pending_attempt) { return; } /* PROM reload wait, service main loop. */
        reinit_phase = REINIT_RESUME;
        return; /* Keep latch asserted across the last calibration boundary. */
    }
    if (reinit_phase == REINIT_RESUME) {
        for (uint8_t i = 0; i < MS5607_CHANNEL_COUNT; ++i) {
            if (sensors[i].state == SENSOR_ACTIVE) { set_slot(i, MS5607_READY); }
            else if (sensors[i].state == SENSOR_ABSENT) {
                sensors[i].next_retry_ms = HAL_GetTick() + MS5607_RETRY_PERIOD_MS;
            }
        }
        for (uint8_t b = 0; b < BUS_COUNT; ++b) {
            buses[b].cursor = 0U;
            buses[b].transfer = XFER_IDLE;
        }
        phase = IDLE;
        next_cycle_us = now_us() + SAMPLE_PERIOD_US;
        reinit_phase = REINIT_NONE;
        reinit_active = false; /* Everything needed for resume is now installed. */
        notify_status();
    }
}

/* Public main-loop interface ------------------------------------------------*/
/**
 * @brief Advance the acquisition state machine without waiting for conversions.
 * @note Call on every main-loop pass, not only once per millisecond. Returns
 *       immediately when disabled or misconfigured. Conversion waits are
 *       measured from completion of the last start command in the phase, so no
 *       ADC is read early. One short blocking recovery step may run after
 *       publication; HAL fault handling may also incur its own timeout waits.
 *       A B1 re-init request is accepted only in IDLE, after publication.
 */
void ms5607_process(void)
{
    if (!active) {
        return;
    }
    if (phase == IDLE && reinit_requested) {
        if (frame_consumer && frame_unread) { return; }
        start_reinit();
        return;
    }
    if (reinit_active) {
        service_reinit();
        return;
    }
    uint32_t now = now_us(); /* Current time used for this scheduler decision. */
    switch (phase) {
    case IDLE:
        if (!due(now, next_cycle_us)) {
            return;
        }
        if (now - next_cycle_us >= SAMPLE_PERIOD_US) {
            uint32_t skipped = (now - next_cycle_us) / SAMPLE_PERIOD_US + 1U; /* Expired deadlines. */
            diagnostics.missed_cycles += skipped;
            sequence += skipped;
            next_cycle_us += skipped * SAMPLE_PERIOD_US;
            note_fault();
            return; /* Resume at a future deadline, never burst catch-up. */
        }
        if (now - next_cycle_us > diagnostics.max_start_lateness_us) {
            diagnostics.max_start_lateness_us = now - next_cycle_us;
        }
        working.sequence = ++sequence;
        working.scheduled_us = next_cycle_us;
        next_cycle_us += SAMPLE_PERIOD_US;
        cycle_start_us = now;
        for (uint8_t i = 0; i < MS5607_CHANNEL_COUNT; ++i) { /* New cycle, fresh chance. */
            if (sensors[i].state == SENSOR_ACTIVE) {
                sensors[i].cycle_failed = false;
            }
        }
        begin_phase(START_TEMPERATURE);
        (void)service_transfers();
        break;
    case START_TEMPERATURE:
    case START_PRESSURE:
        if (service_transfers()) {
            ready_us = now_us() + CONVERSION_WAIT_US;
            phase = phase == START_TEMPERATURE ? WAIT_TEMPERATURE : WAIT_PRESSURE;
        }
        break;
    case WAIT_TEMPERATURE:
    case WAIT_PRESSURE:
        if (due(now, ready_us)) {
            begin_phase(phase == WAIT_TEMPERATURE ? READ_TEMPERATURE : READ_PRESSURE);
            (void)service_transfers();
        }
        break;
    case READ_TEMPERATURE:
        if (service_transfers()) {
            begin_phase(START_PRESSURE);
            (void)service_transfers();
        }
        break;
    case READ_PRESSURE:
        if (service_transfers()) {
            publish_frame();
            if (!reinit_requested) { recover_one(); }
        }
        break;
    }
}

/**
 * @brief Request a re-init of all enabled sensors; the B1 handler calls this.
 * @note Interrupt context (BSP EXTI, NVIC priority 15) or foreground. Only sets
 *       a volatile flag: no SPI, GPIO or HAL call. active is false for a zero
 *       mask, a configuration error and until ms5607_init() completes, so those
 *       requests are ignored. A press while a re-init runs is dropped here, in
 *       any phase, so it cannot start a second re-init once the first ends
 *       using the volatile operation latch. The latch is set before the request
 *       is consumed and remains set through the final calibration/resume step.
 *       ms5607_process() consumes the flag at a clean IDLE boundary.
 */
void ms5607_request_reinit(void)
{
    if (active && !reinit_running()) {
        reinit_requested = true;
    }
}

/**
 * @brief Report application-level sensor status for the LED and stream header.
 * @return In priority order: MS5607_IDLE for a zero mask; MS5607_FAULT for a
 *         configuration error; MS5607_REINITIALIZING while a B1 re-init runs;
 *         MS5607_FAULT for an enabled sensor that is not ACTIVE, or a fault
 *         event within the last MS5607_FAULT_HOLD_MS; otherwise
 *         MS5607_ACQUIRING.
 * @note Foreground only. Once the hold has expired, it is forgotten so that
 *       HAL tick wrap (about 49.7 days) cannot make an old fault recent again.
 *       Sensors pending re-init do not count as not ACTIVE, and the deliberate
 *       re-init itself records no fault event.
 */
MS5607_Status ms5607_status(void)
{
    if (MS5607_ENABLED_MASK == 0U) {
        return MS5607_IDLE;
    }
    bool fault = false;  /* Any current or recent sensor fault condition. */
    bool reinit = false; /* At least one sensor is still pending re-init. */
    for (uint8_t i = 0; i < MS5607_CHANNEL_COUNT; ++i) { /* Current sensor faults. */
        if (sensors[i].reinit_pending) {
            reinit = true;
        } else if ((MS5607_ENABLED_MASK & (1U << i)) != 0U &&
                   sensors[i].state != SENSOR_ACTIVE) {
            fault = true;
        }
    }
    if (fault_recent) {
        if (HAL_GetTick() - last_fault_ms < MS5607_FAULT_HOLD_MS) {
            fault = true;
        } else {
            fault_recent = false;
        }
    }
    if (config_error) {
        return MS5607_FAULT;
    }
    if (reinit_active || reinit) {
        return MS5607_REINITIALIZING;
    }
    return fault ? MS5607_FAULT : MS5607_ACQUIRING;
}

/**
 * @brief Copy the latest complete frame and acknowledge it as consumed.
 * @param out Destination frame; NULL is allowed and returns false.
 * @return true if the copied frame was previously unread; false otherwise.
 * @note Foreground only. Even when false, a non-NULL destination receives the
 *       latest snapshot (all NaN/status slots before the first measurement).
 */
bool ms5607_copy_frame(MS5607_Frame *out)
{
    if (out == NULL) {
        return false;
    }
    bool fresh = frame_unread; /* Preserve freshness before acknowledging the copy. */
    *out = published;
    frame_unread = false;
    return fresh;
}

/**
 * @brief Copy accumulated diagnostic counters without clearing them.
 * @param out Destination structure; NULL is ignored.
 * @note Foreground only; counters reset at startup.
 */
void ms5607_get_diagnostics(MS5607_Diagnostics *out)
{
    if (out != NULL) {
        *out = diagnostics;
    }
}

/**
 * @brief Test for an unread frame without copying measurement storage.
 * @return true if publication has occurred since the last copy.
 * @note Foreground-only; the streaming module consumes each publication once.
 */
bool ms5607_frame_pending(void)
{
    return frame_unread;
}

/* Public HAL callback hooks -------------------------------------------------*/
/**
 * @brief Release CS and notify the scheduler that a HAL transfer completed.
 * @param spi HAL handle supplied to HAL_SPI_TxRxCpltCallback.
 * @note Interrupt context. HAL has checked wire completion before this hook.
 *       No compensation, waiting or next-transfer launch is performed here.
 *       Unknown handles and callbacks without a busy transfer (for example
 *       after a foreground abort) are ignored.
 */
void ms5607_spi_complete(SPI_HandleTypeDef *spi)
{
    for (uint8_t b = 0; b < BUS_COUNT; ++b) { /* Find the callback's bus handle. */
        Bus *bus = &buses[b]; /* Candidate bus context. */
        if (bus->spi == spi && bus->transfer == XFER_BUSY) {
            select_channel(bus->channel, GPIO_PIN_SET);
            __DMB();
            bus->transfer = XFER_DONE;
            return;
        }
    }
}

/**
 * @brief Save a HAL SPI error for handling on the next foreground service pass.
 * @param spi HAL handle supplied to HAL_SPI_ErrorCallback.
 * @note Interrupt context; does not block, abort, or attempt a retry here.
 */
void ms5607_spi_error(SPI_HandleTypeDef *spi)
{
    for (uint8_t b = 0; b < BUS_COUNT; ++b) { /* Find the callback's bus handle. */
        Bus *bus = &buses[b]; /* Candidate bus context. */
        if (bus->spi == spi && bus->transfer == XFER_BUSY) {
            bus->hal_error = HAL_SPI_GetError(spi);
            bus->transfer = XFER_FAILED;
            return;
        }
    }
}
