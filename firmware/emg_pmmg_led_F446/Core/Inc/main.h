/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Board pin map and shared handles for the NUCLEO-F446RE port.
  *
  * The pMMG mapping (CS1..CS12, SPI1/2/3, J12 sync on PB10) is the same as the
  * NUCLEO-G474RE reference, except CS7: PB11 does not exist on the F446RE
  * package, so CS7 (J7) is on PC9 (CN10 pin 1). Pins added for the LED
  * application are listed last.
  ******************************************************************************
  */
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Exported variables --------------------------------------------------------*/
extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi2;
extern SPI_HandleTypeDef hspi3;
extern UART_HandleTypeDef huart2;

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* Board pins: NUCLEO-F446RE ---------------------------------------------------*/
#define B1_Pin GPIO_PIN_13          /* User button, active low (pressed = 0). */
#define B1_GPIO_Port GPIOC
#define LD2_Pin GPIO_PIN_5          /* Green user LED, active high. */
#define LD2_GPIO_Port GPIOA
#define VCP_TX_Pin GPIO_PIN_2       /* USART2_TX -> ST-LINK VCP. */
#define VCP_RX_Pin GPIO_PIN_3       /* USART2_RX <- ST-LINK VCP. */
#define VCP_GPIO_Port GPIOA

/* pMMG chip selects (G474 reference pins; CS7 moved to PC9) ----------------*/
#define CS3_J3_Pin GPIO_PIN_5
#define CS3_J3_GPIO_Port GPIOC
#define CS9_J9_Pin GPIO_PIN_1
#define CS9_J9_GPIO_Port GPIOB
#define CS8_J8_Pin GPIO_PIN_2
#define CS8_J8_GPIO_Port GPIOB
#define CS12_J12_Pin GPIO_PIN_10
#define CS12_J12_GPIO_Port GPIOB
#define CS7_J7_Pin GPIO_PIN_9         /* Moved: PB11 (G474 CS7) is not bonded on the F446RE LQFP64. */
#define CS7_J7_GPIO_Port GPIOC
#define CS6_J6_Pin GPIO_PIN_12
#define CS6_J6_GPIO_Port GPIOB
#define CS2_J2_Pin GPIO_PIN_6
#define CS2_J2_GPIO_Port GPIOC
#define CS1_J1_Pin GPIO_PIN_8
#define CS1_J1_GPIO_Port GPIOC
#define CS11_J11_Pin GPIO_PIN_8
#define CS11_J11_GPIO_Port GPIOA
#define CS10_J10_Pin GPIO_PIN_9
#define CS10_J10_GPIO_Port GPIOA
#define CS5_J5_Pin GPIO_PIN_11
#define CS5_J5_GPIO_Port GPIOA
#define CS4_J4_Pin GPIO_PIN_12
#define CS4_J4_GPIO_Port GPIOA

/* LED application pins (new in the F446 port) ------------------------------*/
#define EMG_LEFT_Pin GPIO_PIN_0     /* A0, ADC1_IN0: left-arm PSL-iEMG2 Ch2 envelope, 0-3.3 V. */
#define EMG_RIGHT_Pin GPIO_PIN_1    /* A1, ADC1_IN1: right-arm PSL-iEMG2 Ch2 envelope, 0-3.3 V. */
#define EMG_GPIO_Port GPIOA
#define WS2812_LEFT_Pin GPIO_PIN_6  /* D10, TIM4_CH1 (AF2): left-arm strips via 5 V buffer. */
#define WS2812_RIGHT_Pin GPIO_PIN_7 /* CN7-21, TIM4_CH2 (AF2): right-arm strips via 5 V buffer. */
#define WS2812_RING_Pin GPIO_PIN_8  /* D15, TIM4_CH3 (AF2): pMMG ring via 5 V buffer. */
#define WS2812_GPIO_Port GPIOB

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
