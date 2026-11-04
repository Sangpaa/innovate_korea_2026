/**
  ******************************************************************************
  * @file    pmmg_light.h
  * @brief   pMMG pressure on one connector -> heartbeat (beats, BPM) and a
  *          0..1 pressure level for the arc reactor.
  *
  * The arc reactor effect (fx.c) reads the state. This module owns no
  * peripheral.
  *
  * Each new ms5607 frame (100 Hz) is read through ms5607_peek_channel(), so
  * the binary stream still receives every frame. Rest pressure (baseline) is
  * averaged for PMMG_BASELINE_MS at start, after the sensor recovers and
  * after pmmg_light_rebaseline() (B1). While the pressure stays below the dead
  * band, the baseline slowly follows temperature drift. The rise above the
  * dead band maps to 0..1 and is low-pass filtered.
  * The level is 0 while calibrating, when the channel is invalid, and when
  * no new frame arrived for PMMG_STALE_MS (for example during a B1 re-init);
  * the state flags tell the ring which of these applies.
  *
  * Heartbeat: the pulse is the small pressure wave on top of the contact
  * pressure. Each sample is filtered (high-pass PMMG_PULSE_HP_S removes
  * contact pressure and drift, two low-pass stages of PMMG_PULSE_LP_MS remove
  * sensor noise). The detector watches the rise over PMMG_SLOPE_MS, because
  * the systolic upstroke is the steepest part of a beat while breathing and
  * drift change slowly. A beat is an upward crossing of PMMG_PULSE_THRESHOLD
  * x the recent peak rise (but at least PMMG_PULSE_MIN_PA), and not
  * sooner than 60000 / PMMG_BPM_MAX ms (or 60 % of the current beat interval)
  * after the previous beat. BPM is the median of the last PMMG_BPM_AVERAGE
  * intervals. An interval more than PMMG_INTERVAL_TOLERANCE from the median
  * is ignored, unless PMMG_PULSE_VALID_BEATS such intervals come in a row
  * (the rate really changed). The pulse is valid after
  * PMMG_PULSE_VALID_BEATS counted intervals and is lost when no beat comes
  * for PMMG_PULSE_LOST_MS.
  * Tune PMMG_PULSE_MIN_PA with slope_pa / slope_peak_pa in app_monitor: the
  * MS5607 at OSR 1024 has a few Pa of noise.
  ******************************************************************************
  */
#ifndef PMMG_LIGHT_H
#define PMMG_LIGHT_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

/* USER CONFIGURATION BEGIN --------------------------------------------------*/
#define PMMG_LIGHT_CHANNEL   0U       /**< Connector index: 0 = J1 (SPI1, CS1 = PC8). */
#define PMMG_PRESSURE_SIGN   1.0f     /**< +1: contraction raises pressure; -1 if it falls. */
#define PMMG_BASELINE_MS     1000U    /**< Rest-pressure averaging time. */
#define PMMG_DEADBAND_PA     30.0f    /**< Rise above baseline that still reads as 0. */
#define PMMG_FULL_SCALE_PA   1000.0f  /**< Rise above baseline that reads as level 1. */
#define PMMG_DRIFT_TAU_S     20.0f    /**< Baseline drift-follow time constant at rest. */
#define PMMG_SMOOTH_MS       30U      /**< Low-pass time constant of the level. */
#define PMMG_STALE_MS        200U     /**< No new frame for this long -> level 0, stale. */
#define PMMG_PULSE_HP_S      1.0f     /**< High-pass time constant of the pulse filter. */
#define PMMG_PULSE_LP_MS     30U      /**< Time constant of each of the two low-pass stages. */
#define PMMG_SLOPE_MS        50U      /**< Window of the upstroke rise. */
#define PMMG_PULSE_PEAK_S    2.0f     /**< Decay time constant of the recent peak rise. */
#define PMMG_PULSE_THRESHOLD 0.5f     /**< Beat threshold as a fraction of the recent peak rise. */
#define PMMG_PULSE_MIN_PA    3.0f     /**< Smallest rise (Pa per PMMG_SLOPE_MS) that can count as a beat. */
#define PMMG_INTERVAL_TOLERANCE 0.3f  /**< Allowed deviation of one interval from the median. */
#define PMMG_BPM_MIN         35U      /**< Slowest accepted heart rate. */
#define PMMG_BPM_MAX         180U     /**< Fastest accepted heart rate (also the refractory time). */
#define PMMG_BPM_AVERAGE     5U       /**< Intervals in the BPM median. */
#define PMMG_PULSE_VALID_BEATS 3U     /**< Consecutive in-range intervals before the pulse is valid. */
#define PMMG_PULSE_LOST_MS   2500U    /**< No beat for this long -> pulse lost. */
/* USER CONFIGURATION END ----------------------------------------------------*/

/** Snapshot for live-expression monitoring and tests. */
typedef struct {
    float pressure_pa;  /**< Last valid pressure of the channel (NaN before the first). */
    float baseline_pa;  /**< Rest pressure being subtracted. */
    float delta_pa;     /**< Signed rise above the baseline. */
    float level;        /**< Smoothed 0..1 level. */
    bool calibrating;   /**< Baseline is being measured; level 0. */
    bool channel_valid; /**< Last frame slot had MS5607_VALID. */
    bool stale;         /**< No frame yet, or none for PMMG_STALE_MS. */
    float pulse_pa;     /**< Filtered pulse signal (sign as PMMG_PRESSURE_SIGN). */
    float slope_pa;     /**< Rise of pulse_pa over PMMG_SLOPE_MS. */
    float slope_peak_pa;/**< Recent peak of slope_pa. */
    float bpm;          /**< Median heart rate, 0 while the pulse is not valid. */
    uint32_t beat_count;/**< Beats detected since start (wraps). */
    uint32_t last_beat_ms; /**< Tick of the last beat. */
    bool pulse_valid;   /**< Enough regular beats; bpm is meaningful. */
} PmmgLightState;

/* Exported functions --------------------------------------------------------*/
/** @brief Reset the state and start the first baseline measurement. */
void pmmg_light_init(void);

/** @brief Read the channel when a new frame exists and update the level.
 *  @note Foreground, every main-loop pass, after ms5607_process(). */
void pmmg_light_process(uint32_t now_ms);

/** @brief Request a new baseline measurement. Interrupt-safe (sets a flag). */
void pmmg_light_rebaseline(void);

/** @brief Copy the current state. */
void pmmg_light_get_state(PmmgLightState *out);

#endif
