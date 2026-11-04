/**
  ******************************************************************************
  * @file    ws2812.c
  * @brief   WS2812 frame encoding and TIM4 CH1-CH3 PWM/DMA transmission.
  *
  * Timing (WS2812B datasheet): 800 kHz bit period, T0H 0.40 us, T1H 0.80 us,
  * tolerance +-150 ns; reset low > 280 us (newer WS2812B parts). The compare
  * values are computed from the actual TIM4 clock at init. Each DMA buffer
  * holds one leading low slot, 24 slots per LED (GRB, MSB first) and
  * RESET_SLOTS trailing low slots. The last slot written is zero, so the line
  * stays low after a transfer while TIM4 keeps running.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "ws2812.h"
#include "main.h"
#include <math.h>
#include <string.h>

/* Private constants ---------------------------------------------------------*/
#define BIT_HZ          800000U /**< WS2812 data rate. */
#define T0H_NS          400U    /**< High time of a 0 bit. */
#define T1H_NS          800U    /**< High time of a 1 bit. */
#define LEAD_SLOTS      1U      /**< Low slot before data; absorbs the preload delay. */
#define RESET_SLOTS     240U    /**< 240 x 1.25 us = 300 us low, above 280 us. */
#define BITS_PER_LED    24U
#define ARM_SLOTS       (LEAD_SLOTS + WS2812_ARM_LEDS * BITS_PER_LED + RESET_SLOTS)
#define RING_SLOTS      (LEAD_SLOTS + WS2812_RING_LEDS * BITS_PER_LED + RESET_SLOTS)
#define MA_PER_CHANNEL  20U     /**< Typical current of one colour channel at 255. */
#define GAMMA           2.2f
#define DMA_FLAGS       0x3DU   /**< FEIF, DMEIF, TEIF, HTIF, TCIF of one stream. */

/* Private types -------------------------------------------------------------*/
/** One data line: pin, compare register, DMA stream and buffers. */
typedef struct {
    uint16_t pin;                /**< GPIOB pin. */
    uint32_t channel;            /**< HAL TIM channel. */
    uint32_t dma_enable;         /**< TIM_DIER_CCxDE bit. */
    volatile uint32_t *ccr;      /**< Compare register written by the DMA. */
    DMA_Stream_TypeDef *stream;  /**< DMA1 stream (channel 2 = TIM4). */
    uint8_t stream_number;       /**< For the flag-clear register and shift. */
    uint16_t length;             /**< LEDs on the line. */
    uint32_t slot_count;         /**< DMA transfers per frame. */
    uint16_t *slots;             /**< One compare value per bit period. */
    uint8_t (*pixels)[3];        /**< Frame buffer in R, G, B order. */
} Output;

/* Private state -------------------------------------------------------------*/
static TIM_HandleTypeDef htim4;
static uint16_t left_slots[ARM_SLOTS];
static uint16_t right_slots[ARM_SLOTS];
static uint16_t ring_slots[RING_SLOTS];
static uint8_t left_pixels[WS2812_ARM_LEDS][3];
static uint8_t right_pixels[WS2812_ARM_LEDS][3];
static uint8_t ring_pixels[WS2812_RING_LEDS][3];
static Output outputs[WS2812_OUTPUT_COUNT] = {
    {WS2812_LEFT_Pin,  TIM_CHANNEL_1, TIM_DIER_CC1DE, NULL, DMA1_Stream0, 0U,
     WS2812_ARM_LEDS,  ARM_SLOTS,  left_slots,  left_pixels},
    {WS2812_RIGHT_Pin, TIM_CHANNEL_2, TIM_DIER_CC2DE, NULL, DMA1_Stream3, 3U,
     WS2812_ARM_LEDS,  ARM_SLOTS,  right_slots, right_pixels},
    {WS2812_RING_Pin,  TIM_CHANNEL_3, TIM_DIER_CC3DE, NULL, DMA1_Stream7, 7U,
     WS2812_RING_LEDS, RING_SLOTS, ring_slots,  ring_pixels},
};
static uint8_t level_lut[256];  /**< Perceptual value -> gamma and brightness limit. */
static uint16_t ccr_zero;       /**< Compare value of a 0 bit. */
static uint16_t ccr_one;        /**< Compare value of a 1 bit. */
static uint32_t estimated_ma;
static uint32_t output_ma[WS2812_OUTPUT_COUNT]; /**< Per-output estimate of the last frame. */
static bool ready;              /**< Init succeeded. */

/* Private functions ---------------------------------------------------------*/
/** @brief TIM4 input clock: APB1 timer clocks double when APB1 is divided. */
static uint32_t tim4_clock_hz(void)
{
    uint32_t hz = HAL_RCC_GetPCLK1Freq();
    if ((RCC->CFGR & RCC_CFGR_PPRE1_2) != 0U) {
        hz *= 2U;
    }
    return hz;
}

/** @brief Write the 8 bit slots of one colour byte, MSB first. */
static uint16_t *encode_byte(uint16_t *out, uint8_t value)
{
    for (uint8_t mask = 0x80U; mask != 0U; mask >>= 1) {
        *out++ = (value & mask) != 0U ? ccr_one : ccr_zero;
    }
    return out;
}

/** @brief Clear the event flags of one DMA1 stream. */
static void clear_stream_flags(uint8_t stream_number)
{
    static const uint8_t shift[4] = {0U, 6U, 16U, 22U};
    uint32_t mask = (uint32_t)DMA_FLAGS << shift[stream_number & 3U];
    if (stream_number < 4U) {
        DMA1->LIFCR = mask;
    } else {
        DMA1->HIFCR = mask;
    }
}

/** @brief Point one stream at its buffer and start it; TIM4 requests follow. */
static void start_output(Output *out)
{
    TIM4->DIER &= ~out->dma_enable; /* Drop any request latched since the last frame. */
    out->stream->CR &= ~DMA_SxCR_EN;
    while ((out->stream->CR & DMA_SxCR_EN) != 0U) {
    }
    clear_stream_flags(out->stream_number);
    out->stream->M0AR = (uint32_t)out->slots;
    out->stream->NDTR = out->slot_count;
    out->stream->CR |= DMA_SxCR_EN;
    TIM4->DIER |= out->dma_enable;
}

/* Public functions ----------------------------------------------------------*/
bool ws2812_init(void)
{
    uint32_t clock_hz = tim4_clock_hz();
    uint32_t period = clock_hz / BIT_HZ; /* Ticks per bit; 112 at 90 MHz (1.244 us). */
    if (period < 20U || period > 0xFFFFU) {
        return false;
    }
    ccr_zero = (uint16_t)((uint64_t)clock_hz * T0H_NS / 1000000000ULL);
    ccr_one = (uint16_t)((uint64_t)clock_hz * T1H_NS / 1000000000ULL);
    for (uint32_t i = 0U; i < 256U; ++i) {
        float linear = powf((float)i / 255.0f, GAMMA);
        level_lut[i] = (uint8_t)(linear * (float)WS2812_MAX_BRIGHTNESS + 0.5f);
    }

    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_TIM4_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();

    htim4.Instance = TIM4;
    htim4.Init.Prescaler = 0U;
    htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim4.Init.Period = period - 1U;
    htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_PWM_Init(&htim4) != HAL_OK) {
        return false;
    }
    TIM_OC_InitTypeDef oc = {0};
    oc.OCMode = TIM_OCMODE_PWM1;
    oc.Pulse = 0U; /* Output low until a frame is sent. */
    oc.OCPolarity = TIM_OCPOLARITY_HIGH;
    oc.OCFastMode = TIM_OCFAST_DISABLE;
    outputs[0].ccr = &TIM4->CCR1;
    outputs[1].ccr = &TIM4->CCR2;
    outputs[2].ccr = &TIM4->CCR3;

    GPIO_InitTypeDef pin = {0};
    pin.Mode = WS2812_OPEN_DRAIN != 0U ? GPIO_MODE_AF_OD : GPIO_MODE_AF_PP;
    pin.Pull = GPIO_NOPULL;
    pin.Speed = GPIO_SPEED_FREQ_HIGH;
    pin.Alternate = GPIO_AF2_TIM4;
    for (uint32_t i = 0U; i < WS2812_OUTPUT_COUNT; ++i) {
        Output *out = &outputs[i];
        if (HAL_TIM_PWM_ConfigChannel(&htim4, &oc, out->channel) != HAL_OK) {
            return false;
        }
        /* Memory to compare register, 16 bit, increment memory, high priority,
         * direct mode (FIFO off). The stream is enabled per frame. */
        out->stream->CR = 0U;
        out->stream->CR = (2U << DMA_SxCR_CHSEL_Pos) | DMA_SxCR_PL_1 |
                          DMA_SxCR_MSIZE_0 | DMA_SxCR_PSIZE_0 |
                          DMA_SxCR_MINC | DMA_SxCR_DIR_0;
        out->stream->FCR = 0U;
        out->stream->PAR = (uint32_t)out->ccr;
        memset(out->slots, 0, out->slot_count * sizeof(uint16_t)); /* Lead/reset stay 0. */
        pin.Pin = out->pin;
        HAL_GPIO_Init(WS2812_GPIO_Port, &pin);
    }
    TIM4->CR2 |= TIM_CR2_CCDS; /* CCx DMA requests on the update event. */
    for (uint32_t i = 0U; i < WS2812_OUTPUT_COUNT; ++i) {
        if (HAL_TIM_PWM_Start(&htim4, outputs[i].channel) != HAL_OK) {
            return false;
        }
    }

    ready = true;
    ws2812_clear();
    (void)ws2812_show();
    return true;
}

uint16_t ws2812_length(Ws2812Output output)
{
    return output < WS2812_OUTPUT_COUNT ? outputs[output].length : 0U;
}

void ws2812_set(Ws2812Output output, uint16_t index, uint8_t red, uint8_t green, uint8_t blue)
{
    if (output < WS2812_OUTPUT_COUNT && index < outputs[output].length) {
        uint8_t *px = outputs[output].pixels[index];
        px[0] = red;
        px[1] = green;
        px[2] = blue;
    }
}

void ws2812_clear(void)
{
    for (uint32_t i = 0U; i < WS2812_OUTPUT_COUNT; ++i) {
        memset(outputs[i].pixels, 0, (size_t)outputs[i].length * 3U);
    }
}

bool ws2812_show(void)
{
    if (!ready || ws2812_busy()) {
        return false;
    }
    /* Current caps: estimate each output after gamma/brightness and dim it to
     * its own cap, then dim all of them together to the supply cap. Factors
     * are 8.8 fixed point. */
    static const uint32_t limit_ma[WS2812_OUTPUT_COUNT] = {
        WS2812_ARM_LIMIT_MA, WS2812_ARM_LIMIT_MA, WS2812_RING_LIMIT_MA};
    uint32_t factors[WS2812_OUTPUT_COUNT];
    uint32_t total_ma = 0U;
    for (uint32_t o = 0U; o < WS2812_OUTPUT_COUNT; ++o) {
        const uint8_t *px = outputs[o].pixels[0];
        uint32_t sum = 0U;
        for (uint32_t i = 0U; i < (uint32_t)outputs[o].length * 3U; ++i) {
            sum += level_lut[px[i]];
        }
        uint32_t ma = sum * MA_PER_CHANNEL / 255U;
        factors[o] = 256U;
        if (ma > limit_ma[o]) {
            factors[o] = limit_ma[o] * 256U / ma;
            ma = limit_ma[o];
        }
        output_ma[o] = ma;
        total_ma += ma;
    }
    if (total_ma > WS2812_POWER_LIMIT_MA) {
        uint32_t factor = WS2812_POWER_LIMIT_MA * 256U / total_ma;
        total_ma = 0U;
        for (uint32_t o = 0U; o < WS2812_OUTPUT_COUNT; ++o) {
            factors[o] = factors[o] * factor >> 8;
            output_ma[o] = output_ma[o] * factor >> 8;
            total_ma += output_ma[o];
        }
    }
    estimated_ma = total_ma;

    for (uint32_t o = 0U; o < WS2812_OUTPUT_COUNT; ++o) {
        Output *out = &outputs[o];
        uint32_t factor = factors[o];
        uint16_t *slot = &out->slots[LEAD_SLOTS];
        for (uint16_t i = 0U; i < out->length; ++i) {
            const uint8_t *px = out->pixels[i];
            /* WS2812 wire order is G, R, B. */
            slot = encode_byte(slot, (uint8_t)((level_lut[px[1]] * factor) >> 8));
            slot = encode_byte(slot, (uint8_t)((level_lut[px[0]] * factor) >> 8));
            slot = encode_byte(slot, (uint8_t)((level_lut[px[2]] * factor) >> 8));
        }
    }
    for (uint32_t o = 0U; o < WS2812_OUTPUT_COUNT; ++o) {
        start_output(&outputs[o]);
    }
    return true;
}

bool ws2812_busy(void)
{
    for (uint32_t o = 0U; o < WS2812_OUTPUT_COUNT; ++o) {
        if ((outputs[o].stream->CR & DMA_SxCR_EN) != 0U) {
            return true;
        }
    }
    return false;
}

uint32_t ws2812_estimated_ma(void)
{
    return estimated_ma;
}

uint32_t ws2812_output_ma(Ws2812Output output)
{
    return output < WS2812_OUTPUT_COUNT ? output_ma[output] : 0U;
}
