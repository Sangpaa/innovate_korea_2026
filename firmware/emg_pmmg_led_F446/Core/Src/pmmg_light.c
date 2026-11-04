/**
  ******************************************************************************
  * @file    pmmg_light.c
  * @brief   Baseline tracking, level mapping and heartbeat detection for the arc reactor.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "pmmg_light.h"
#include "main.h"
#include "ms5607.h"
#include <math.h>

/* Private constants ---------------------------------------------------------*/
#define BASELINE_SAMPLES ((PMMG_BASELINE_MS * MS5607_SAMPLE_HZ) / 1000U)
#define DRIFT_SAMPLES    (PMMG_DRIFT_TAU_S * (float)MS5607_SAMPLE_HZ)
#define SAMPLE_MS        (1000U / MS5607_SAMPLE_HZ)
#define REFRACTORY_MS    (60000U / PMMG_BPM_MAX)
#define INTERVAL_MAX_MS  (60000U / PMMG_BPM_MIN)
#define SLOPE_SAMPLES    (PMMG_SLOPE_MS / SAMPLE_MS)

#if PMMG_LIGHT_CHANNEL >= MS5607_CHANNEL_COUNT
#error "PMMG_LIGHT_CHANNEL must select J1-J12"
#endif
#if (MS5607_ENABLED_MASK & (1U << PMMG_LIGHT_CHANNEL)) == 0U
#error "PMMG_LIGHT_CHANNEL must be enabled in ms5607.h"
#endif
#if BASELINE_SAMPLES == 0U
#error "PMMG_BASELINE_MS is shorter than one sample period"
#endif

/* Private state -------------------------------------------------------------*/
static PmmgLightState state;
static bool have_frame;           /**< last_sequence is meaningful. */
static uint32_t last_sequence;    /**< Frame cycle number last processed. */
static uint32_t last_frame_ms;    /**< Tick of the last new frame. */
static float baseline_sum;
static uint32_t baseline_count;
static volatile bool rebaseline_request;
static float pulse_mean;          /**< High-pass reference (slow mean). */
static float pulse_lp[2];         /**< Two low-pass stages, before the sign. */
static float slope_history[SLOPE_SAMPLES]; /**< pulse_pa of the last PMMG_SLOPE_MS. */
static uint32_t slope_next;
static uint32_t outliers;         /**< Consecutive intervals far from the median. */
static bool armed;                /**< The pressure fell since the last beat. */
static bool pulse_primed;         /**< pulse_mean holds a real sample. */
static bool have_beat;            /**< last_beat_ms is meaningful. */
static uint32_t intervals[PMMG_BPM_AVERAGE];
static uint32_t interval_count;   /**< Stored intervals, up to PMMG_BPM_AVERAGE. */
static uint32_t interval_next;    /**< Ring index of the next interval. */
static uint32_t regular_beats;    /**< Consecutive in-range intervals. */

/* Private functions ---------------------------------------------------------*/
static void reset_pulse(bool keep_filter);

/** @brief Set the level to 0 and restart the baseline measurement. */
static void start_baseline(void)
{
    state.calibrating = true;
    state.level = 0.0f;
    baseline_sum = 0.0f;
    baseline_count = 0U;
    reset_pulse(false);
}

/** @brief Forget the pulse history (new baseline, sensor reset or pulse lost). */
static void reset_pulse(bool keep_filter)
{
    if (!keep_filter) {
        pulse_primed = false;
        state.pulse_pa = 0.0f;
        state.slope_pa = 0.0f;
        state.slope_peak_pa = 0.0f;
    }
    have_beat = false;
    interval_count = 0U;
    interval_next = 0U;
    regular_beats = 0U;
    outliers = 0U;
    state.bpm = 0.0f;
    state.pulse_valid = false;
}

/** @brief Median of the stored intervals in ms (interval_count > 0). */
static uint32_t median_interval(void)
{
    uint32_t sorted[PMMG_BPM_AVERAGE];
    for (uint32_t i = 0U; i < interval_count; ++i) {
        uint32_t v = intervals[i];
        uint32_t j = i;
        for (; j > 0U && sorted[j - 1U] > v; --j) {
            sorted[j] = sorted[j - 1U];
        }
        sorted[j] = v;
    }
    return sorted[interval_count / 2U];
}

/** @brief Record one detected beat at now_ms. */
static void on_beat(uint32_t now_ms)
{
    if (have_beat) {
        uint32_t interval = now_ms - state.last_beat_ms;
        bool outlier = false;
        if (interval_count >= PMMG_PULSE_VALID_BEATS) {
            float ratio = (float)interval / (float)median_interval();
            outlier = ratio < 1.0f - PMMG_INTERVAL_TOLERANCE || ratio > 1.0f + PMMG_INTERVAL_TOLERANCE;
        }
        if (interval > INTERVAL_MAX_MS) {
            reset_pulse(true); /* Gap too long: start counting again from this beat. */
        } else if (outlier && ++outliers < PMMG_PULSE_VALID_BEATS) {
            /* Missed or extra beat: keep the history, do not count it. */
        } else {
            if (outliers >= PMMG_PULSE_VALID_BEATS) {
                interval_count = 0U;  /* The rate really changed: start a new history. */
                interval_next = 0U;
                regular_beats = 0U;
            }
            outliers = 0U;
            intervals[interval_next] = interval;
            interval_next = (interval_next + 1U) % PMMG_BPM_AVERAGE;
            if (interval_count < PMMG_BPM_AVERAGE) {
                interval_count++;
            }
            regular_beats++;
        }
    }
    have_beat = true;
    state.last_beat_ms = now_ms;
    state.beat_count++;
    state.pulse_valid = regular_beats >= PMMG_PULSE_VALID_BEATS;
    state.bpm = state.pulse_valid ? 60000.0f / (float)median_interval() : 0.0f;
}

/** @brief Filter one sample, track the upstroke slope and look for a beat. */
static void detect_pulse(float pressure_pa, uint32_t now_ms)
{
    if (!pulse_primed) {
        pulse_mean = pressure_pa;
        pulse_lp[0] = 0.0f;
        pulse_lp[1] = 0.0f;
        for (uint32_t i = 0U; i < SLOPE_SAMPLES; ++i) {
            slope_history[i] = 0.0f;
        }
        pulse_primed = true;
    }
    pulse_mean += (pressure_pa - pulse_mean) * ((float)SAMPLE_MS / (PMMG_PULSE_HP_S * 1000.0f));
    float alpha = (float)SAMPLE_MS / (float)(PMMG_PULSE_LP_MS + SAMPLE_MS);
    pulse_lp[0] += ((pressure_pa - pulse_mean) - pulse_lp[0]) * alpha;
    pulse_lp[1] += (pulse_lp[0] - pulse_lp[1]) * alpha;
    state.pulse_pa = PMMG_PRESSURE_SIGN * pulse_lp[1];

    /* Rise over the last PMMG_SLOPE_MS: the systolic upstroke is the steepest
     * part of each beat, while breathing and drift change slowly. */
    float previous = state.slope_pa;
    state.slope_pa = state.pulse_pa - slope_history[slope_next];
    slope_history[slope_next] = state.pulse_pa;
    slope_next = (slope_next + 1U) % SLOPE_SAMPLES;

    float decay = 1.0f - (float)SAMPLE_MS / (PMMG_PULSE_PEAK_S * 1000.0f);
    state.slope_peak_pa = fmaxf(state.slope_pa, state.slope_peak_pa * decay);

    uint32_t refractory = REFRACTORY_MS;
    if (state.pulse_valid) {
        uint32_t expected = (uint32_t)(0.6f * 60000.0f / state.bpm); /* Skips the dicrotic notch. */
        refractory = expected > refractory ? expected : refractory;
    }
    float threshold = fmaxf(PMMG_PULSE_THRESHOLD * state.slope_peak_pa, PMMG_PULSE_MIN_PA);
    if (state.slope_pa < 0.0f) {
        armed = true; /* The upstroke is over; the next rise may be a beat. */
    }
    bool crossed = previous < threshold && state.slope_pa >= threshold;
    bool blocked = have_beat && now_ms - state.last_beat_ms < refractory;
    if (armed && crossed && !blocked) {
        armed = false;
        on_beat(now_ms);
    }
    if (have_beat && now_ms - state.last_beat_ms >= PMMG_PULSE_LOST_MS) {
        reset_pulse(true);
    }
}

/** @brief Apply one valid pressure sample. */
static void apply_sample(float pressure_pa, uint32_t now_ms)
{
    state.pressure_pa = pressure_pa;
    detect_pulse(pressure_pa, now_ms);
    if (state.calibrating) {
        baseline_sum += pressure_pa;
        if (++baseline_count >= BASELINE_SAMPLES) {
            state.baseline_pa = baseline_sum / (float)baseline_count;
            state.calibrating = false;
        }
        return;
    }
    state.delta_pa = PMMG_PRESSURE_SIGN * (pressure_pa - state.baseline_pa);
    if (state.delta_pa < PMMG_DEADBAND_PA) {
        state.baseline_pa += (pressure_pa - state.baseline_pa) / DRIFT_SAMPLES;
    }
    float target = (state.delta_pa - PMMG_DEADBAND_PA) / (PMMG_FULL_SCALE_PA - PMMG_DEADBAND_PA);
    target = target < 0.0f ? 0.0f : (target > 1.0f ? 1.0f : target);
    float alpha = (float)SAMPLE_MS / (float)(PMMG_SMOOTH_MS + SAMPLE_MS);
    state.level += alpha * (target - state.level);
}

/* Public functions ----------------------------------------------------------*/
void pmmg_light_init(void)
{
    state.pressure_pa = NAN;
    state.stale = true;
    start_baseline();
}

void pmmg_light_process(uint32_t now_ms)
{
    if (rebaseline_request) {
        rebaseline_request = false;
        start_baseline();
    }
    MS5607_Measurement slot;
    uint32_t sequence;
    if (!ms5607_peek_channel(PMMG_LIGHT_CHANNEL, &slot, &sequence)) {
        return;
    }
    if (have_frame && sequence == last_sequence) {
        if (now_ms - last_frame_ms >= PMMG_STALE_MS) {
            state.level = 0.0f; /* No acquisition (B1 re-init, stalled bus). */
            state.stale = true;
        }
        return;
    }
    have_frame = true;
    last_sequence = sequence;
    last_frame_ms = now_ms;
    state.stale = false;

    state.channel_valid = slot.status == MS5607_VALID;
    if (!state.channel_valid) {
        state.level = 0.0f;
        return;
    }
    if ((slot.flags & (MS5607_FLAG_RECOVERED | MS5607_FLAG_PROM_CHANGED)) != 0U) {
        start_baseline(); /* Sensor was reset or replaced; old baseline is invalid. */
    }
    apply_sample(slot.pressure_pa, now_ms);
}

void pmmg_light_rebaseline(void)
{
    rebaseline_request = true;
}

void pmmg_light_get_state(PmmgLightState *out)
{
    if (out != NULL) {
        *out = state;
    }
}
