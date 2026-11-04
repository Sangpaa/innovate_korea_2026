/**
  ******************************************************************************
  * @file    stream.h
  * @brief   Board-side buffered binary streaming through the ST-LINK USB VCP.
  *
  * The STM32 transmits through USART2 (PA2/PA3); ST-LINK bridges UART to USB.
  * STM32F4 port of the G474 module, which used BSP COM1/LPUART1.
  * Call stream_init() after MX_USART2_UART_Init() and the MS5607 driver, then stream_process()
  * on every main-loop pass. The module copies new sensor frames into a bounded
  * packet queue and sends them by DMA without waiting for serial transmission.
  * With no new sensor frames it sends one status snapshot per second, or one
  * per received sync marker while XM pulses arrive on J12.
  * B1 pauses acquisition: captured start/end STATUS notifications are sent
  * promptly and separately from DATA. Timers and packet counters keep running.
  * B1 presses add a one-packet event flag and persistent count; idle status is
  * sent promptly for this event so reception can be tested without sensors.
  * An accepted press also starts the sensor re-init (requested by main.c).
  * Every packet carries the sync-input state and the last four J12 marker
  * records; a new marker triggers a prompt STATUS packet only when no DATA flows.
  *
  * User settings are at the top of stream.c. The module owns USART2 transmit,
  * DMA1 stream 6 and their IRQs. No printf logging may share this port.
  * USB receipt is not acknowledged: UART completion is not proof of PC receipt.
  * A runtime transmit fault drops the in-flight packet and pauses output for a
  * short retry delay; only initialization failures stop streaming until reboot.
  * Wire format (version 5) and loss behavior are specified in streaming_spec.md.
  ******************************************************************************
  */
#ifndef STREAM_H
#define STREAM_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

/* Exported constants --------------------------------------------------------*/
#define STREAM_PACKET_BYTES 444U /**< Version-5 packet length including sync block and CRC. */

/* Exported types ------------------------------------------------------------*/
/** Reason for the last transport failure; also encoded as a uint16 in packets.
 *  Initialization errors stop streaming until reboot; TX errors are recovered.
 *  The value is kept after recovery as the most recent fault reason. */
typedef enum {
    STREAM_ERROR_NONE = 0,    /**< No UART/DMA transport fault since startup. */
    STREAM_ERROR_ARGUMENT,    /**< Missing handle or incompatible UART instance/format; stops. */
    STREAM_ERROR_UART_INIT,   /**< Baud-rate configuration failed; stops. */
    STREAM_ERROR_DMA_INIT,    /**< DMA resource initialization failed; stops. */
    STREAM_ERROR_TX_START,    /**< HAL rejected a DMA transmission request; recovered. */
    STREAM_ERROR_TX_CALLBACK, /**< HAL reported an asynchronous UART/DMA error; recovered. */
    STREAM_ERROR_TX_TIMEOUT   /**< No wire-complete callback before the deadline; recovered. */
} StreamError;

/** Foreground diagnostics since startup; uint32 counters wrap naturally. */
typedef struct {
    uint32_t packets_queued;  /**< Successfully copied into owned packet storage. */
    uint32_t packets_sent;    /**< UART wire-complete events consumed by foreground. */
    uint32_t packets_dropped; /**< Queue/notification overflow or in-flight packet lost to a fault. */
    uint32_t tx_errors;       /**< Initialization/start/callback/timeout faults. */
    uint32_t max_queue_depth; /**< Highest occupancy, including the in-flight packet. */
    uint32_t queue_depth;     /**< Current occupancy; filled by the getter. */
    uint32_t last_hal_error;  /**< UART HAL error bits at failure; may be zero. */
    StreamError last_error;   /**< Context for the last failure; not cleared on recovery. */
} StreamDiagnostics;

/* Exported functions --------------------------------------------------------*/
/**
 * @brief Configure queued DMA output on the already initialized USART2 (main.c).
 * @param uart &huart2 from main.c; USART2, 8N1, no flow control.
 * @return true if ready, or intentionally disabled in stream.c; false on failure.
 * @note Startup only. Sets the configured streaming baud rate after MX_USART2_UART_Init().
 */
bool stream_init(UART_HandleTypeDef *uart);

/**
 * @brief Consume new measurements, queue packets and service transmit completion.
 * @note Foreground only, every main-loop pass after ms5607_process(). No blocking
 *       UART output. A runtime transport fault no longer quarantines streaming
 *       until reboot: the in-flight packet is dropped, new packets keep queueing,
 *       and output resumes with the oldest queued packet after a retry delay.
 */
void stream_process(void);

/** @brief Record a debounced B1 press for the next successfully queued packet.
 *  @return true if the press was accepted; false for contact bounce.
 *  @note Call from the B1 EXTI callback (main.c). No UART work or delay
 *        occurs in this hook. Presses within 200 ms are treated as contact bounce.
 *        A counter preserves multiple presses while a packet is pending. The
 *        debounce also runs with streaming disabled. main.c requests the
 *        sensor re-init (ms5607_request_reinit()) for an accepted press. */
bool stream_button_pressed(void);

/** @brief Report a current or recent queue loss or transport error for the fault LED.
 *  @return true while stopped by an initialization failure or recovering from a
 *          transmit fault, and for a 2 s hold (STREAM_FAULT_HOLD_MS) after the
 *          latest runtime fault or dropped packet; false at boot without faults. */
bool stream_faulted(void);

/** @brief Copy stream diagnostics without resetting them; foreground only.
 *  @param out Destination; NULL is ignored. */
void stream_get_diagnostics(StreamDiagnostics *out);

/** @brief Record final UART wire completion; called from HAL's TX callback.
 *  @param uart Callback's UART handle; unrelated handles are ignored. */
void stream_uart_complete(UART_HandleTypeDef *uart);

/** @brief Record an interrupt-time UART/DMA error for foreground handling.
 *  @param uart Callback's UART handle; unrelated handles are ignored. */
void stream_uart_error(UART_HandleTypeDef *uart);

/** @brief Forward DMA1 stream 6 interrupts to the module's HAL DMA handle. */
void stream_dma_irq(void);

/** @brief Forward USART2 interrupts, including final stop-bit completion, to HAL. */
void stream_uart_irq(void);

#endif
