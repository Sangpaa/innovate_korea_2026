/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  *
  * Core and SysTick handlers, B1 EXTI, the SPI1/2/3 sensor transfers, the VCP
  * stream (DMA1 stream 6 + USART2) and the J12 sync captures (TIM2). Each
  * module enables its own IRQs. The WS2812 DMA streams run without IRQs.
  ******************************************************************************
  */
#include "main.h"
#include "stream.h"
#include "sync_input.h"

void NMI_Handler(void)
{
  while (1)
  {
  }
}

void HardFault_Handler(void)
{
  while (1)
  {
  }
}

void MemManage_Handler(void)
{
  while (1)
  {
  }
}

void BusFault_Handler(void)
{
  while (1)
  {
  }
}

void UsageFault_Handler(void)
{
  while (1)
  {
  }
}

void SVC_Handler(void)
{
}

void DebugMon_Handler(void)
{
}

void PendSV_Handler(void)
{
}

void SysTick_Handler(void)
{
  HAL_IncTick();
}

/** @brief B1 (PC13) press. */
void EXTI15_10_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(B1_Pin);
}

/** @brief SPI1 byte/error interrupts for J1/J2/J3/J10 sensor transfers. */
void SPI1_IRQHandler(void)
{
  HAL_SPI_IRQHandler(&hspi1);
}

/** @brief SPI2 byte/error interrupts for J4/J5/J6/J11 sensor transfers. */
void SPI2_IRQHandler(void)
{
  HAL_SPI_IRQHandler(&hspi2);
}

/** @brief SPI3 byte/error interrupts for J7/J8/J9/J12 sensor transfers. */
void SPI3_IRQHandler(void)
{
  HAL_SPI_IRQHandler(&hspi3);
}

/** @brief Streaming TX DMA; stream.c owns DMA1 stream 6. */
void DMA1_Stream6_IRQHandler(void)
{
  stream_dma_irq();
}

/** @brief Final UART wire completion for the ST-LINK Virtual COM Port. */
void USART2_IRQHandler(void)
{
  stream_uart_irq();
}

/** @brief TIM2 channel 3/4 captures of the J12 sync pulses. */
void TIM2_IRQHandler(void)
{
  sync_input_irq();
}
