/**
  ******************************************************************************
  * @file    fx.h
  * @brief   LED effects: EMG charge gauge on each arm, pMMG ring light.
  *
  * Arm gauge (one per arm, from that arm's EMG level):
  *  - While the level is at or above FX_ACTIVE_LEVEL the gauge charges. A
  *    stronger contraction charges faster; at level 1 it fills in
  *    FX_FILL_S seconds, at the threshold in about twice that time.
  *  - After the arm relaxes, the gauge holds for FX_HOLD_MS, then drains
  *    from full to empty in FX_DRAIN_S seconds.
  *  - Look: the filled part runs through a colour gradient with energy
  *    waves moving toward the head, a white-hot head and random sparkles.
  *    The empty part glows dimly. At 100 % the arm flashes white, then runs
  *    a rainbow while the contraction continues.
  *  - While the EMG rest level is measured, a dim blue dot scans the arm.
  *
  * Arc reactor (from the pMMG heartbeat): concentric rings on one chain.
  * Every detected beat flashes the core and the flash travels outward ring
  * by ring ("lub"), followed by a weaker second flash ("dub"). The core turns
  * white at the peak of each flash. The colour follows the heart rate: cyan
  * at FX_BPM_CALM, through blue and purple, to red at FX_BPM_FAST. Dim
  * comets turn between beats, neighbouring rings in opposite directions, at
  * a speed that follows the heart rate. Before the pulse is valid each beat
  * still flashes, at about a third of the strength. Without beats the reactor breathes
  * slowly in dim cyan (searching). A dim blue breath means the baseline is
  * being measured; a red blink on the first outer LED means no valid sensor
  * data.
  *
  * Call fx_render() once per frame, then ws2812_show().
  ******************************************************************************
  */
#ifndef FX_H
#define FX_H

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/* USER CONFIGURATION BEGIN --------------------------------------------------*/
/** Gauge path on an arm:
 *  0 = PARALLEL: all strips fill together along their length (LED 0 first).
 *  1 = SEQUENTIAL: strip 1 fills completely, then strip 2, and so on. */
#define FX_ARM_SEQUENTIAL   0U
/** 1 if every second strip on an arm is mounted in the reverse direction
 *  (zig-zag wiring), so the gauge still moves the same way on the body. */
#define FX_ARM_SERPENTINE   0U
#define FX_ACTIVE_LEVEL     0.10f  /**< EMG level that counts as "contracting". */
#define FX_FILL_S           3.0f   /**< Empty -> full at EMG level 1. */
#define FX_HOLD_MS          400U   /**< Hold after relaxing before draining. */
#define FX_DRAIN_S          1.5f   /**< Full -> empty while relaxed. */
#define FX_HUE_START        140.0f /**< Gauge colour at the start (degrees, 140 = green). */
#define FX_HUE_SPAN         -180.0f/**< Hue change to the end (-180: via yellow, red to magenta). */
/** Arc reactor rings in chain order (outermost first, DOUT -> DIN inward).
 *  The sum must equal WS2812_RING_LEDS. */
#define FX_RING_SIZES       {24U, 16U, 12U, 8U}
#define FX_RING_COUNT       4U
#define FX_BEAT_DECAY_MS    140.0f /**< Fade time constant of one flash. */
#define FX_BEAT_SPREAD_MS   160.0f /**< Delay of the flash from the core to the outer ring. */
#define FX_DUB_GAIN         0.55f  /**< Strength of the second flash. */
#define FX_DUB_MAX_MS       280.0f /**< Latest second flash; it comes at 30 % of the beat interval. */
#define FX_BPM_CALM         60.0f  /**< Heart rate shown in cyan. */
#define FX_BPM_FAST         120.0f /**< Heart rate shown in red. */
/* USER CONFIGURATION END ----------------------------------------------------*/

/** Snapshot for live-expression monitoring. */
typedef struct {
    float gauge[2];   /**< Left, right arm charge 0..1. */
    float ring_phase; /**< Ring comet position in turns. */
} FxState;

/* Exported functions --------------------------------------------------------*/
/** @brief Advance the gauges and draw all three outputs into the frame buffers.
 *  @note Foreground; reads emg_input and pmmg_light states. */
void fx_render(uint32_t now_ms);

/** @brief Empty both gauges (for example after B1). */
void fx_reset(void);

/** @brief Copy the current state. */
void fx_get_state(FxState *out);

#endif
