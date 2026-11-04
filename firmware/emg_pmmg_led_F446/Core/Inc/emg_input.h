/**
  ******************************************************************************
  * @file    emg_input.h
  * @brief   EMG envelope acquisition for two arms: PA0 (A0) left, PA1 (A1) right.
  *
  * Each external PSL-iEMG2 board delivers a rectified, smoothed envelope
  * voltage on its Ch2 output. ADC1 scans IN0 and IN1 continuously (480-cycle
  * sampling) into one interleaved circular DMA2 buffer without interrupts.
  * emg_input_process() runs once per millisecond: per channel it averages the
  * buffer, low-pass filters the value and maps it above the rest level to 0..1.
  * The LED effects (fx.c) read the levels.
  *
  * Rest level: measured during EMG_REST_CAL_MS after boot and after each
  * emg_input_recalibrate() (B1). Keep both arms relaxed during that time.
  * Inputs must stay within 0-3.3 V.
  * Resources owned here: ADC1, DMA2 stream 0 channel 0 (no IRQ).
  ******************************************************************************
  */
#ifndef EMG_INPUT_H
#define EMG_INPUT_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

/* USER CONFIGURATION BEGIN --------------------------------------------------*/
#define EMG_SMOOTH_MS       40U     /**< Low-pass time constant applied to the envelope. */
#define EMG_REST_CAL_MS     2000U   /**< Rest-level averaging time after boot or B1. */
#define EMG_DEADBAND_MV     30.0f   /**< Envelope rise above rest that still reads as 0. */
#define EMG_FULL_SCALE_MV   1500.0f /**< Rise above rest that reads as level 1. */
#define EMG_VREF_MV         3300.0f /**< ADC reference (VDDA) in mV. */
/* USER CONFIGURATION END ----------------------------------------------------*/

/** Input channel; also the ADC scan rank order. */
typedef enum {
    EMG_LEFT,   /**< A0, PA0, ADC1_IN0. */
    EMG_RIGHT,  /**< A1, PA1, ADC1_IN1. */
    EMG_CHANNEL_COUNT
} EmgChannel;

/** Snapshot for live-expression monitoring and tests. */
typedef struct {
    float envelope_mv;  /**< Filtered input voltage. */
    float rest_mv;      /**< Rest level subtracted before mapping. */
    float level;        /**< 0..1 contraction level. */
    bool calibrating;   /**< Rest level is being measured; level is held at 0. */
} EmgState;

/* Exported functions --------------------------------------------------------*/
/** @brief Configure PA0/PA1, ADC1 and DMA and start continuous conversion.
 *  @return false if HAL initialization failed. */
bool emg_input_init(void);

/** @brief Average, filter and map the newest samples; call once per ms tick. */
void emg_input_process(uint32_t now_ms);

/** @brief Restart the rest-level measurement. Interrupt-safe (sets a flag). */
void emg_input_recalibrate(void);

/** @brief Copy one channel's state (zeroed for an invalid channel). */
void emg_input_get_state(EmgChannel channel, EmgState *out);

#endif
