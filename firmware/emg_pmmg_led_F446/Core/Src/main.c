/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : NUCLEO-F446RE port of pmmg_HND_STM plus EMG/pMMG light control.
  *
  * Application overview:
  * The pMMG part is the G474 reference application unchanged in behaviour:
  * MS5607 acquisition on J1-J12 (ms5607.h), shared 1 MHz TIM2 timebase, J12 XM
  * sync capture, binary stream over the ST-LINK VCP (now USART2, 921600 8N1)
  * and the LD2 status patterns. Two outputs are added:
  *  - Left/right arm EMG envelopes on A0/A1 -> charge gauge on each arm's
  *    WS2812 strips (PB6 left, PB7 right).
  *  - pMMG J1 pressure rise -> WS2812 ring light (PB8).
  *    Effects are in fx.c; the three data lines share TIM4 (ws2812.c).
  * B1 still requests the sensor re-init and is marked in the stream; it also
  * restarts the EMG rest and pMMG baseline measurements, so press it while
  * relaxed. app_monitor holds live values for the debugger.
  *
  * Clock: 8 MHz ST-LINK MCO (HSE bypass) -> PLL 180 MHz; APB1 45 MHz (timers
  * 90 MHz), APB2 90 MHz (timers 180 MHz). HSI 16 MHz is the fallback when the
  * MCO is not connected (solder bridge SB50 open).
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "emg_input.h"
#include "fx.h"
#include "ms5607.h"
#include "pmmg_light.h"
#include "status_led.h"
#include "stream.h"
#include "sync_input.h"
#include "timebase.h"
#include "ws2812.h"

/* Private define ------------------------------------------------------------*/
#define WS2812_FRAME_MS 10U /**< LED effect refresh period (100 Hz). */

/* Private types -------------------------------------------------------------*/
/** Live values for the debugger (Live Expressions) and SWD memory reads. */
typedef struct {
    uint32_t magic;          /**< 0x4C454431 ("LED1") once the main loop runs. */
    uint32_t tick_ms;        /**< HAL tick at the last update. */
    uint32_t init_errors;    /**< Bit 0 WS2812, bit 1 EMG ADC, bit 3 HSE fallback. */
    float emg_envelope_mv[2];/**< Left, right. */
    float emg_rest_mv[2];
    float emg_level[2];
    float gauge[2];
    float pmmg_pressure_pa;
    float pmmg_baseline_pa;
    float pmmg_delta_pa;
    float pmmg_level;
    float pmmg_pulse_pa;     /**< Band-passed pulse signal. */
    float pmmg_slope_peak_pa;/**< Recent peak upstroke rise; compare with PMMG_PULSE_MIN_PA. */
    float pmmg_bpm;          /**< 0 until the pulse is valid. */
    uint32_t pmmg_beats;     /**< Beats detected. */
    uint32_t led_current_ma; /**< Estimated WS2812 colour current after the caps. */
    uint32_t led_output_ma[3];/**< Same per output: left arm, right arm, arc reactor. */
    uint32_t sensor_status;  /**< MS5607_Status value. */
    uint32_t ws2812_frames;  /**< Frames started. */
} AppMonitor;

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi3;
UART_HandleTypeDef huart2;
volatile AppMonitor app_monitor;

/* Private function prototypes -----------------------------------------------*/
static bool SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI_Init(SPI_HandleTypeDef *spi, SPI_TypeDef *instance, uint32_t prescaler);
static void MX_USART2_UART_Init(void);
static void update_monitor(uint32_t now_ms);

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  HAL_Init();
  uint32_t init_errors = SystemClock_Config() ? 0U : 0x08U; /* HSE missing: HSI fallback. */

  MX_GPIO_Init();
  if (!ws2812_init()) { /* Early, so the LED data lines are driven low soon after reset. */
    init_errors |= 0x01U;
  }
  pmmg_light_init();
  /* SPI1 on APB2 90 MHz / 256, SPI2/3 on APB1 45 MHz / 128: 352 kHz on all buses
   * (375 kHz on the G474 reference). */
  MX_SPI_Init(&hspi1, SPI1, SPI_BAUDRATEPRESCALER_256);
  MX_SPI_Init(&hspi2, SPI2, SPI_BAUDRATEPRESCALER_128);
  MX_SPI_Init(&hspi3, SPI3, SPI_BAUDRATEPRESCALER_128);
  MX_USART2_UART_Init();

  /* Same order as the reference: the sync input before the sensor driver,
   * which must not drive PB10; stream.c sets the streaming baud rate. */
  status_led_init();
  (void)timebase_init(); /* Once per boot; a failure is reported by each enabled user below. */
  sync_input_init();
  ms5607_init(&hspi1, &hspi2, &hspi3);
  (void)stream_init(&huart2); /* Failure is reported through the LED/diagnostics. */
  if (!emg_input_init()) {
    init_errors |= 0x02U;
  }
  app_monitor.init_errors = init_errors;
  app_monitor.magic = 0x4C454431UL;

  uint32_t last_ms = HAL_GetTick();     /* Last tick serviced by the 1 ms tasks. */
  uint32_t last_frame_ms = last_ms;     /* Last WS2812 frame start. */
  while (1)
  {
    ms5607_process();       /* No bus traffic while the driver enabled mask is zero. */
    sync_input_process();   /* Pair captured XM edges before the stream takes markers. */
    pmmg_light_process(HAL_GetTick()); /* Peeks the newest frame; does not consume it. */
    stream_process();       /* Consume new frames and service queued UART DMA output. */

    uint32_t now_ms = HAL_GetTick();
    if (now_ms == last_ms) {
      continue;
    }
    last_ms = now_ms;
    emg_input_process(now_ms);
    if (now_ms - last_frame_ms >= WS2812_FRAME_MS) {
      if (!ws2812_busy()) {
        fx_render(now_ms);
        (void)ws2812_show();
        last_frame_ms = now_ms;
        app_monitor.ws2812_frames++;
      }
    }

    MS5607_Status status = ms5607_status(); /* Sensor activity, re-init or current/recent fault. */
    StatusLedMode led_mode = STATUS_LED_IDLE;
    if (status == MS5607_REINITIALIZING)    /* B1 acknowledgement; status_led keeps it >= 500 ms. */
      led_mode = STATUS_LED_REINIT;
    else if (status == MS5607_FAULT || stream_faulted() || (init_errors & 0x03U) != 0U)
      led_mode = STATUS_LED_FAULT;
    else if (status == MS5607_ACQUIRING)
      led_mode = STATUS_LED_ACQUIRING;
    status_led_update(led_mode, now_ms);
    update_monitor(now_ms);
  }
}

/** @brief Copy module states into app_monitor (1 ms). */
static void update_monitor(uint32_t now_ms)
{
  PmmgLightState light;
  FxState fx;
  pmmg_light_get_state(&light);
  fx_get_state(&fx);
  app_monitor.tick_ms = now_ms;
  for (uint32_t ch = 0U; ch < EMG_CHANNEL_COUNT; ++ch) {
    EmgState emg;
    emg_input_get_state((EmgChannel)ch, &emg);
    app_monitor.emg_envelope_mv[ch] = emg.envelope_mv;
    app_monitor.emg_rest_mv[ch] = emg.rest_mv;
    app_monitor.emg_level[ch] = emg.level;
    app_monitor.gauge[ch] = fx.gauge[ch];
  }
  app_monitor.pmmg_pressure_pa = light.pressure_pa;
  app_monitor.pmmg_baseline_pa = light.baseline_pa;
  app_monitor.pmmg_delta_pa = light.delta_pa;
  app_monitor.pmmg_level = light.level;
  app_monitor.pmmg_pulse_pa = light.pulse_pa;
  app_monitor.pmmg_slope_peak_pa = light.slope_peak_pa;
  app_monitor.pmmg_bpm = light.bpm;
  app_monitor.pmmg_beats = light.beat_count;
  app_monitor.led_current_ma = ws2812_estimated_ma();
  for (uint32_t o = 0U; o < WS2812_OUTPUT_COUNT; ++o) {
    app_monitor.led_output_ma[o] = ws2812_output_ma((Ws2812Output)o);
  }
  app_monitor.sensor_status = (uint32_t)ms5607_status();
}

/**
  * @brief  System clock: 180 MHz from the 8 MHz ST-LINK MCO, or from HSI.
  * @retval true with HSE, false when the HSI fallback is used.
  */
static bool SystemClock_Config(void)
{
  RCC_OscInitTypeDef osc = {0};
  RCC_ClkInitTypeDef clk = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  bool hse = true;
  osc.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  osc.HSEState = RCC_HSE_BYPASS;
  osc.PLL.PLLState = RCC_PLL_ON;
  osc.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  osc.PLL.PLLM = 8U;    /* 1 MHz VCO input. */
  osc.PLL.PLLN = 360U;  /* 360 MHz VCO. */
  osc.PLL.PLLP = RCC_PLLP_DIV2; /* 180 MHz SYSCLK. */
  osc.PLL.PLLQ = 8U;
  osc.PLL.PLLR = 2U;
  if (HAL_RCC_OscConfig(&osc) != HAL_OK) {
    hse = false;
    osc.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    osc.HSEState = RCC_HSE_OFF;
    osc.HSIState = RCC_HSI_ON;
    osc.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    osc.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    osc.PLL.PLLM = 16U;
    if (HAL_RCC_OscConfig(&osc) != HAL_OK) {
      Error_Handler();
    }
  }
  if (HAL_PWREx_EnableOverDrive() != HAL_OK) {
    Error_Handler();
  }
  clk.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                  RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  clk.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  clk.AHBCLKDivider = RCC_SYSCLK_DIV1;
  clk.APB1CLKDivider = RCC_HCLK_DIV4; /* 45 MHz (maximum). */
  clk.APB2CLKDivider = RCC_HCLK_DIV2; /* 90 MHz (maximum). */
  if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_5) != HAL_OK) {
    Error_Handler();
  }
  return hse;
}

/**
  * @brief  One SPI master, mode 0, 8 bit, software NSS (as in the reference).
  */
static void MX_SPI_Init(SPI_HandleTypeDef *spi, SPI_TypeDef *instance, uint32_t prescaler)
{
  spi->Instance = instance;
  spi->Init.Mode = SPI_MODE_MASTER;
  spi->Init.Direction = SPI_DIRECTION_2LINES;
  spi->Init.DataSize = SPI_DATASIZE_8BIT;
  spi->Init.CLKPolarity = SPI_POLARITY_LOW;
  spi->Init.CLKPhase = SPI_PHASE_1EDGE;
  spi->Init.NSS = SPI_NSS_SOFT;
  spi->Init.BaudRatePrescaler = prescaler;
  spi->Init.FirstBit = SPI_FIRSTBIT_MSB;
  spi->Init.TIMode = SPI_TIMODE_DISABLE;
  spi->Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  spi->Init.CRCPolynomial = 7U;
  if (HAL_SPI_Init(spi) != HAL_OK) {
    Error_Handler();
  }
}

/**
  * @brief  USART2 (ST-LINK VCP) at 115200 8N1; stream_init() sets 921600.
  */
static void MX_USART2_UART_Init(void)
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200U;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK) {
    Error_Handler();
  }
}

/**
  * @brief  GPIO: chip selects idle high, J12/PB10 input, LD2 output, B1 EXTI.
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOC, CS3_J3_Pin|CS2_J2_Pin|CS1_J1_Pin|CS7_J7_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, CS9_J9_Pin|CS8_J8_Pin|CS6_J6_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOA, CS11_J11_Pin|CS10_J10_Pin|CS5_J5_Pin|CS4_J4_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = CS3_J3_Pin|CS2_J2_Pin|CS1_J1_Pin|CS7_J7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = CS9_J9_Pin|CS8_J8_Pin|CS6_J6_Pin;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = CS11_J11_Pin|CS10_J10_Pin|CS5_J5_Pin|CS4_J4_Pin;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LD2_Pin;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = CS12_J12_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(CS12_J12_GPIO_Port, &GPIO_InitStruct);

  /* B1 is active low on the F446RE (external pull-up): press = falling edge. */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 15U, 0U); /* Same as the G474 BSP button. */
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

/**
  * @brief  B1 press: stream marker, sensor re-init and new rest references.
  * @note   EXTI context. All hooks only update flags/counters.
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == B1_Pin && stream_button_pressed()) {
    ms5607_request_reinit();
    pmmg_light_rebaseline();
    emg_input_recalibrate();
    fx_reset();
  }
}

/** @brief Forward a completed SPI transfer to the MS5607 driver (IRQ context). */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
  ms5607_spi_complete(hspi);
}

/** @brief Forward a SPI error to the MS5607 driver's deferred recovery path. */
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
  ms5607_spi_error(hspi);
}

/** @brief Forward final UART wire completion to the streaming queue. */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  stream_uart_complete(huart);
}

/** @brief Record UART/DMA errors for streaming recovery in the main loop. */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  stream_uart_error(huart);
}

/**
  * @brief  This function is executed in case of error occurrence.
  */
void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  (void)file;
  (void)line;
}
#endif /* USE_FULL_ASSERT */
