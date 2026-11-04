/**
  ******************************************************************************
  * @file    fx.c
  * @brief   Arm charge gauges and the pMMG ring effect (see fx.h).
  *
  * All colours are computed as floats 0..1 in perceptual units; ws2812.c
  * applies gamma, the brightness limit and the power cap.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "fx.h"
#include "emg_input.h"
#include "pmmg_light.h"
#include "ws2812.h"
#include <math.h>
#include <stdbool.h>

/* Private constants ---------------------------------------------------------*/
#define TWO_PI          6.2831853f
#define ARM_COUNT       2U
#define PATH_LEN        (FX_ARM_SEQUENTIAL != 0U ? WS2812_ARM_LEDS : WS2812_LEDS_PER_STRIP)
#define HEAD_WIDTH      0.08f   /**< White-hot zone behind the head (fraction of the path). */
#define EMBER_LEVEL     0.16f   /**< Glow of the empty part. */
#define FLASH_MS        400U    /**< White flash when the gauge reaches 100 %. */
#define SPARKLE_DECAY   200U    /**< Per frame, out of 256. */
#define RING_TAIL       0.25f   /**< Comet length in turns of its ring. */

/* Private types -------------------------------------------------------------*/
typedef struct { float r, g, b; } Rgb;

typedef struct {
    float gauge;                       /**< Charge 0..1. */
    bool active;                       /**< Contraction detected (with hysteresis). */
    bool full;                         /**< Reached 100 % and not yet below 98 %. */
    uint32_t last_active_ms;
    uint32_t full_ms;                  /**< Tick when full was entered. */
    uint8_t sparkle[WS2812_ARM_LEDS];  /**< Decaying white sparkle per LED. */
} Arm;

/* Private state -------------------------------------------------------------*/
static Arm arms[ARM_COUNT];
static const uint8_t ring_sizes[FX_RING_COUNT] = FX_RING_SIZES;
static float ring_phase;
static uint32_t seen_beats;          /**< pm.beat_count at the last flash. */
static uint32_t beat_ms;             /**< Tick of the last flash. */
static float dub_ms = 250.0f;        /**< Second flash delay. */
static bool have_beat;
static uint32_t last_ms;
static bool started;
static uint32_t rng = 0x12345678U;
static volatile bool reset_request;

/* Private functions ---------------------------------------------------------*/
/** @brief xorshift32 pseudo random number. */
static uint32_t rand_u32(void)
{
    rng ^= rng << 13;
    rng ^= rng >> 17;
    rng ^= rng << 5;
    return rng;
}

/** @brief Uniform 0..1. */
static float rand_unit(void)
{
    return (float)(rand_u32() & 0xFFFFU) / 65535.0f;
}

static float clamp01(float x)
{
    return x < 0.0f ? 0.0f : (x > 1.0f ? 1.0f : x);
}

/** @brief Hue in degrees (any value), saturation and value 0..1. */
static Rgb hsv(float hue, float sat, float val)
{
    hue = fmodf(hue, 360.0f);
    if (hue < 0.0f) {
        hue += 360.0f;
    }
    float h = hue / 60.0f;
    int sector = (int)h;
    float f = h - (float)sector;
    float p = val * (1.0f - sat);
    float q = val * (1.0f - sat * f);
    float t = val * (1.0f - sat * (1.0f - f));
    switch (sector) {
    case 0:  return (Rgb){val, t, p};
    case 1:  return (Rgb){q, val, p};
    case 2:  return (Rgb){p, val, t};
    case 3:  return (Rgb){p, q, val};
    case 4:  return (Rgb){t, p, val};
    default: return (Rgb){val, p, q};
    }
}

/** @brief a + (white - a) * amount: blend toward white. */
static Rgb whiten(Rgb a, float amount)
{
    amount = clamp01(amount);
    return (Rgb){a.r + (1.0f - a.r) * amount, a.g + (1.0f - a.g) * amount,
                 a.b + (1.0f - a.b) * amount};
}

/** @brief Per-channel maximum. */
static Rgb brighter(Rgb a, Rgb b)
{
    return (Rgb){fmaxf(a.r, b.r), fmaxf(a.g, b.g), fmaxf(a.b, b.b)};
}

static void put(Ws2812Output output, uint16_t index, Rgb c)
{
    ws2812_set(output, index, (uint8_t)(clamp01(c.r) * 255.0f + 0.5f),
               (uint8_t)(clamp01(c.g) * 255.0f + 0.5f),
               (uint8_t)(clamp01(c.b) * 255.0f + 0.5f));
}

/** @brief Position of chain LED i along the gauge path, 0..PATH_LEN-1. */
static uint16_t path_index(uint16_t i)
{
    uint16_t strip = i / WS2812_LEDS_PER_STRIP;
    uint16_t k = i % WS2812_LEDS_PER_STRIP;
    if (FX_ARM_SERPENTINE != 0U && (strip & 1U) != 0U) {
        k = (uint16_t)(WS2812_LEDS_PER_STRIP - 1U - k);
    }
    return FX_ARM_SEQUENTIAL != 0U ? (uint16_t)(strip * WS2812_LEDS_PER_STRIP + k) : k;
}

/** @brief Charge or drain one gauge from its EMG state. */
static void update_arm(Arm *arm, const EmgState *emg, uint32_t now_ms, float dt)
{
    if (emg->calibrating) {
        arm->gauge = 0.0f;
        arm->active = false;
        arm->full = false;
        return;
    }
    float threshold = arm->active ? FX_ACTIVE_LEVEL * 0.7f : FX_ACTIVE_LEVEL;
    arm->active = emg->level >= threshold;
    if (arm->active) {
        arm->last_active_ms = now_ms;
        arm->gauge += dt * (0.5f + 0.5f * emg->level) / FX_FILL_S;
    } else if (now_ms - arm->last_active_ms >= FX_HOLD_MS) {
        arm->gauge -= dt / FX_DRAIN_S;
    }
    arm->gauge = clamp01(arm->gauge);
    if (!arm->full && arm->gauge >= 1.0f) {
        arm->full = true;
        arm->full_ms = now_ms;
    } else if (arm->full && arm->gauge < 0.98f) {
        arm->full = false;
    }
}

/** @brief Draw one arm. */
static void render_arm(Ws2812Output output, Arm *arm, bool calibrating, uint32_t now_ms)
{
    float t = (float)now_ms * 0.001f;
    float head = arm->gauge * (float)PATH_LEN;   /* Lit LEDs along the path, fractional. */
    float flash = 0.0f;
    if (arm->full && now_ms - arm->full_ms < FLASH_MS) {
        flash = 1.0f - (float)(now_ms - arm->full_ms) / (float)FLASH_MS;
    }
    float ember = EMBER_LEVEL * (0.75f + 0.25f * sinf(TWO_PI * t / 3.0f));
    float scan = fmodf(t * 0.6f, 1.0f);           /* Calibration dot position. */
    float sparkle_chance = 0.004f + 0.03f * arm->gauge * arm->gauge;

    for (uint16_t i = 0U; i < WS2812_ARM_LEDS; ++i) {
        uint16_t k = path_index(i);
        float p = ((float)k + 0.5f) / (float)PATH_LEN; /* 0..1 along the path. */

        if (calibrating) {
            float d = fabsf(p - scan);
            put(output, i, hsv(220.0f, 1.0f, 0.6f * clamp01(1.0f - d * 10.0f)));
            continue;
        }

        Rgb c = hsv(FX_HUE_START, 1.0f, ember);
        float cover = clamp01(head - (float)k);
        if (arm->full) {
            Rgb rainbow = hsv(p * 360.0f - t * 240.0f, 1.0f, 0.9f);
            c = whiten(rainbow, flash);
        } else if (cover > 0.0f) {
            float wave = 0.6f + 0.4f * (0.5f + 0.5f * sinf(TWO_PI * (p * 2.5f - t * 1.2f)));
            Rgb fill = hsv(FX_HUE_START + FX_HUE_SPAN * p, 1.0f, wave * cover);
            float behind = arm->gauge - p;  /* Distance behind the head. */
            if (behind < HEAD_WIDTH) {
                fill = whiten(fill, 0.85f * cover * (1.0f - behind / HEAD_WIDTH));
            }
            c = brighter(c, fill);
        }

        /* Sparkles: born at random in the lit part, then decay. */
        if ((cover > 0.0f || arm->full) && rand_unit() < sparkle_chance) {
            arm->sparkle[i] = 255U;
        } else {
            arm->sparkle[i] = (uint8_t)((arm->sparkle[i] * SPARKLE_DECAY) >> 8);
        }
        if (arm->sparkle[i] != 0U) {
            c = whiten(c, (float)arm->sparkle[i] / 255.0f);
        }
        put(output, i, c);
    }
}

/** @brief Flash brightness ms after its start (0 before the start). */
static float flash(float ms)
{
    return ms < 0.0f ? 0.0f : expf(-ms / FX_BEAT_DECAY_MS);
}

/** @brief Draw the pMMG arc reactor: concentric rings, outermost first in the chain. */
static void render_ring(float dt, uint32_t now_ms)
{
    PmmgLightState pm;
    pmmg_light_get_state(&pm);
    float t = (float)now_ms * 0.001f;

    if (pm.calibrating && !pm.stale) {
        float v = 0.18f + 0.1f * sinf(TWO_PI * t / 1.5f);
        for (uint16_t i = 0U; i < WS2812_RING_LEDS; ++i) {
            put(WS2812_RING, i, hsv(220.0f, 1.0f, v));
        }
        return;
    }
    if (pm.stale || !pm.channel_valid) {
        float v = 0.5f * (0.5f + 0.5f * sinf(TWO_PI * t));
        for (uint16_t i = 0U; i < WS2812_RING_LEDS; ++i) {
            put(WS2812_RING, i, (Rgb){i == 0U ? v : 0.0f, 0.0f, 0.0f});
        }
        return;
    }

    /* A new beat restarts the flash; the dub follows at 30 % of the interval. */
    if (pm.beat_count != seen_beats) {
        seen_beats = pm.beat_count;
        beat_ms = now_ms;
        have_beat = true;
        if (pm.bpm > 0.0f) {
            dub_ms = fminf(0.3f * 60000.0f / pm.bpm, FX_DUB_MAX_MS);
        }
    }
    float since = have_beat ? (float)(now_ms - beat_ms) : 1.0e9f;
    float strength = pm.pulse_valid ? 1.0f : 0.35f;
    float rate = clamp01((pm.bpm - FX_BPM_CALM) / (FX_BPM_FAST - FX_BPM_CALM));
    float hue = 195.0f + 165.0f * rate;            /* Cyan -> blue -> purple -> red. */
    float idle = pm.pulse_valid ? 0.06f : 0.04f + 0.03f * (0.5f + 0.5f * sinf(TWO_PI * t / 3.0f));
    float turns = pm.pulse_valid ? 0.5f * pm.bpm / 60.0f : 0.1f;
    ring_phase = fmodf(ring_phase + dt * turns, 1.0f);
    Rgb comet_colour = hsv(hue, 1.0f, 0.22f * strength);

    uint16_t index = 0U;
    for (uint32_t r = 0U; r < FX_RING_COUNT; ++r) {
        uint16_t n = ring_sizes[r];
        float depth = (float)r / (float)(FX_RING_COUNT - 1U); /* 0 outer .. 1 core. */
        float delay = (1.0f - depth) * FX_BEAT_SPREAD_MS;
        float e = flash(since - delay) + FX_DUB_GAIN * flash(since - dub_ms - delay);
        e = clamp01(e) * strength;
        Rgb glow = hsv(hue, 1.0f - 0.7f * depth * e, idle + e * (0.55f + 0.45f * depth));
        /* Neighbouring rings turn in opposite directions; inner rings turn faster. */
        float head = fmodf(ring_phase * (1.0f + 0.3f * (float)r), 1.0f);
        bool reverse = (r & 1U) != 0U;

        for (uint16_t k = 0U; k < n && index < WS2812_RING_LEDS; ++k, ++index) {
            float angle = (float)k / (float)n;
            float d = reverse ? angle + head : head - angle; /* Turns behind the head. */
            d = fmodf(d + 2.0f, 1.0f);
            float tail = d < RING_TAIL ? 1.0f - d / RING_TAIL : 0.0f;
            Rgb comet = {comet_colour.r * tail, comet_colour.g * tail, comet_colour.b * tail};
            put(WS2812_RING, index, brighter(glow, comet));
        }
    }
}

/* Public functions ----------------------------------------------------------*/
void fx_render(uint32_t now_ms)
{
    if (!started) {
        started = true;
        last_ms = now_ms;
    }
    float dt = (float)(now_ms - last_ms) * 0.001f;
    last_ms = now_ms;
    if (reset_request) {
        reset_request = false;
        for (uint32_t a = 0U; a < ARM_COUNT; ++a) {
            arms[a].gauge = 0.0f;
            arms[a].full = false;
        }
    }

    static const Ws2812Output arm_outputs[ARM_COUNT] = {WS2812_LEFT_ARM, WS2812_RIGHT_ARM};
    for (uint32_t a = 0U; a < ARM_COUNT; ++a) {
        EmgState emg;
        emg_input_get_state((EmgChannel)a, &emg);
        update_arm(&arms[a], &emg, now_ms, dt);
        render_arm(arm_outputs[a], &arms[a], emg.calibrating, now_ms);
    }
    render_ring(dt, now_ms);
}

void fx_reset(void)
{
    reset_request = true;
}

void fx_get_state(FxState *out)
{
    if (out != NULL) {
        out->gauge[0] = arms[0].gauge;
        out->gauge[1] = arms[1].gauge;
        out->ring_phase = ring_phase;
    }
}
