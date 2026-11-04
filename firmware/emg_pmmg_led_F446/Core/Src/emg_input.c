/**
  ******************************************************************************
  * @file    emg_input.c
  * @brief   ADC1/DMA envelope sampling and rest calibration for two channels.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "emg_input.h"
#include "main.h"
#include <string.h>

/* Private constants ---------------------------------------------------------*/
#define SAMPLE_COUNT 64U       /**< Samples per channel; about 2.8 ms of data. */
#define ADC_FULL     4095.0f   /**< 12-bit full-scale code. */

/* Private state -------------------------------------------------------------*/
static ADC_HandleTypeDef hadc1;
static DMA_HandleTypeDef hdma_adc1;
/** Written by DMA continuously, interleaved in scan order (left, right, ...). */
static volatile uint16_t samples[SAMPLE_COUNT * EMG_CHANNEL_COUNT];
static EmgState state[EMG_CHANNEL_COUNT];
static float cal_sum[EMG_CHANNEL_COUNT]; /**< Sum of filtered values during calibration. */
static uint32_t last_ms;          /**< Tick of the previous process call. */
static uint32_t cal_start_ms;     /**< Start of the rest measurement. */
static uint32_t cal_count;        /**< Number of values in each cal_sum. */
static bool filter_primed;        /**< First value initializes the low-pass filters. */
static bool calibrating;
static volatile bool recal_request;

/* Public functions ----------------------------------------------------------*/
bool emg_input_init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_ADC1_CLK_ENABLE();
    __HAL_RCC_DMA2_CLK_ENABLE();

    GPIO_InitTypeDef pin = {0};
    pin.Pin = EMG_LEFT_Pin | EMG_RIGHT_Pin;
    pin.Mode = GPIO_MODE_ANALOG;
    pin.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(EMG_GPIO_Port, &pin);

    hdma_adc1.Instance = DMA2_Stream0;
    hdma_adc1.Init.Channel = DMA_CHANNEL_0;
    hdma_adc1.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_adc1.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_adc1.Init.MemInc = DMA_MINC_ENABLE;
    hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_adc1.Init.Mode = DMA_CIRCULAR;
    hdma_adc1.Init.Priority = DMA_PRIORITY_LOW;
    hdma_adc1.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_adc1) != HAL_OK) {
        return false;
    }
    __HAL_LINKDMA(&hadc1, DMA_Handle, hdma_adc1);

    hadc1.Instance = ADC1;
    hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4; /* 90 MHz / 4 = 22.5 MHz (max 36). */
    hadc1.Init.Resolution = ADC_RESOLUTION_12B;
    hadc1.Init.ScanConvMode = ENABLE;
    hadc1.Init.ContinuousConvMode = ENABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion = EMG_CHANNEL_COUNT;
    hadc1.Init.DMAContinuousRequests = ENABLE;
    hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
    if (HAL_ADC_Init(&hadc1) != HAL_OK) {
        return false;
    }
    static const uint32_t adc_channels[EMG_CHANNEL_COUNT] = {ADC_CHANNEL_0, ADC_CHANNEL_1};
    ADC_ChannelConfTypeDef channel = {0};
    channel.SamplingTime = ADC_SAMPLETIME_480CYCLES; /* High source impedance tolerated. */
    for (uint32_t ch = 0U; ch < EMG_CHANNEL_COUNT; ++ch) {
        channel.Channel = adc_channels[ch];
        channel.Rank = ch + 1U;
        if (HAL_ADC_ConfigChannel(&hadc1, &channel) != HAL_OK) {
            return false;
        }
    }
    /* DMA and ADC IRQs stay disabled in the NVIC: the buffer is polled. The
     * buffer length is a multiple of the scan length, so slots keep their channel. */
    if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)samples, SAMPLE_COUNT * EMG_CHANNEL_COUNT) != HAL_OK) {
        return false;
    }
    last_ms = cal_start_ms = HAL_GetTick();
    calibrating = true;
    for (uint32_t ch = 0U; ch < EMG_CHANNEL_COUNT; ++ch) {
        state[ch].calibrating = true;
    }
    return true;
}

void emg_input_process(uint32_t now_ms)
{
    uint32_t sum[EMG_CHANNEL_COUNT] = {0U};
    for (uint32_t i = 0U; i < SAMPLE_COUNT * EMG_CHANNEL_COUNT; ++i) {
        sum[i % EMG_CHANNEL_COUNT] += samples[i];
    }
    uint32_t dt_ms = now_ms - last_ms; /* Normally 1; larger after a blocking step. */
    last_ms = now_ms;
    float alpha = (float)dt_ms / (float)(EMG_SMOOTH_MS + dt_ms);
    for (uint32_t ch = 0U; ch < EMG_CHANNEL_COUNT; ++ch) {
        float mv = (float)sum[ch] / (float)SAMPLE_COUNT * EMG_VREF_MV / ADC_FULL;
        if (!filter_primed) {
            state[ch].envelope_mv = mv;
        } else {
            state[ch].envelope_mv += alpha * (mv - state[ch].envelope_mv);
        }
    }
    filter_primed = true;

    if (recal_request) {
        recal_request = false;
        calibrating = true;
        cal_start_ms = now_ms;
        cal_count = 0U;
        memset(cal_sum, 0, sizeof(cal_sum));
    }
    if (calibrating) {
        cal_count++;
        bool done = now_ms - cal_start_ms >= EMG_REST_CAL_MS;
        for (uint32_t ch = 0U; ch < EMG_CHANNEL_COUNT; ++ch) {
            cal_sum[ch] += state[ch].envelope_mv;
            state[ch].level = 0.0f;
            state[ch].calibrating = !done;
            if (done) {
                state[ch].rest_mv = cal_sum[ch] / (float)cal_count;
            }
        }
        calibrating = !done;
        return;
    }

    for (uint32_t ch = 0U; ch < EMG_CHANNEL_COUNT; ++ch) {
        float rise = state[ch].envelope_mv - state[ch].rest_mv - EMG_DEADBAND_MV;
        float level = rise / (EMG_FULL_SCALE_MV - EMG_DEADBAND_MV);
        state[ch].level = level < 0.0f ? 0.0f : (level > 1.0f ? 1.0f : level);
    }
}

void emg_input_recalibrate(void)
{
    recal_request = true;
}

void emg_input_get_state(EmgChannel channel, EmgState *out)
{
    if (out == NULL) {
        return;
    }
    if (channel < EMG_CHANNEL_COUNT) {
        *out = state[channel];
    } else {
        memset(out, 0, sizeof(*out));
    }
}
