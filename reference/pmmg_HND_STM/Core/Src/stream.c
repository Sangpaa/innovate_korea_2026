/**
  ******************************************************************************
  * @file    stream.c
  * @brief   Measurement packet serialization, bounded buffering and USB VCP output.
  *
  * New sensor frames are consumed once, serialized in connector order and copied
  * into static queue storage. One queue slot remains owned by UART DMA until
  * HAL_UART_TxCpltCallback confirms wire completion. Foreground alone changes
  * queue indices; IRQ callbacks publish events only. An overflow drops the new
  * packet and increments a counter, never overwriting an in-flight transmission.
  *
  * A runtime transmit fault (rejected start, HAL error callback or timeout) aborts
  * the transfer, drops only the in-flight packet and pauses output with both
  * stream IRQs disabled for STREAM_RETRY_MS. Queued packets are kept and sent
  * afterward. Only initialization failures stop streaming until reboot.
  * stream_faulted() reports a current fault plus a STREAM_FAULT_HOLD_MS hold.
  *
  * The transport is LPUART1 -> onboard ST-LINK -> USB Virtual COM Port. It is not
  * native USB CDC firmware. The PythonDecoder application receives this stream.
  * When sensors are disabled, a 1 Hz STATUS packet contains twelve
  * disabled/NaN slots so the serial connection can still be exercised.
  * B1 recovery pauses acquisition; its captured start/end STATUS notifications
  * are drained independently of DATA, preserving the pause without fake samples.
  *
  * Version 5: 64-byte header + twelve 24-byte slots + 8-byte events + 80-byte
  * sync block + CRC = 444 bytes. Bytes 0-359 keep the version-3 layout.
  * Each slot carries the channel status, one-shot flags and a saturating error
  * count. B1 adds a one-packet test flag and cumulative press count, including at
  * idle. The sync block uses the layout shared with XM thigh_pmmg v2: role 2
  * (Nucleo receiver), receiver state, record count, tick unit 2 (microseconds),
  * capture generation, fault/loss counters and the last four received markers
  * (newest first, with measured low time), repeated in every packet so one lost
  * packet loses no marker. While DATA flows, the next DATA packet carries a new
  * marker. Without DATA (no sensors, B1 pause) a new marker queues a prompt
  * STATUS packet, as a B1 press does, and these marker packets replace the
  * one-second STATUS heartbeat.
  * All integers and binary32 values are explicitly little-endian; C structure
  * padding is never sent. See streaming_spec.md for offsets and CRC parameters.
  * Default load: 44400 bytes/s at 100 Hz; 921600 baud 8N1 provides 92160 bytes/s.
  *
  * Resources owned here: LPUART1 TX, DMA1 channel 1 (LPUART1_TX), both IRQs.
  * IRQ priority 2 is below sensor SPI (1) and SysTick (0). The existing BSP
  * configures PA2/PA3. Do not send printf/BSP log text on this binary stream.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stream.h"
#include "ms5607.h"
#include "sync_input.h"
#include <float.h>
#include <string.h>

/* USER CONFIGURATION BEGIN --------------------------------------------------*/
#ifndef STREAM_ENABLED
#define STREAM_ENABLED          1U      /**< 1 enables output; 0 preserves the BSP UART. */
#endif
#ifndef STREAM_BAUD_RATE
#define STREAM_BAUD_RATE        921600U /**< PC serial port must use the same 8N1 rate. */
#endif
#ifndef STREAM_QUEUE_PACKETS
#define STREAM_QUEUE_PACKETS    8U      /**< Static slots, including one in-flight packet. */
#endif
#define STREAM_STATUS_PERIOD_MS 1000U   /**< Status interval when no fresh frame exists. */
#define STREAM_TX_TIMEOUT_MS    20U     /**< Maximum foreground wait for TX completion. */
#define STREAM_BUTTON_DEBOUNCE_MS 200U  /**< Ignore repeated B1 edges within this interval. */
/* Output pause after a runtime TX fault. Frames keep queueing meanwhile, so the
 * worst case (TX timeout + pause) must stay below the queue capacity time of
 * STREAM_QUEUE_PACKETS sample periods: 20 + 50 ms < 8 x 10 ms at 100 Hz. Then
 * only the in-flight packet is lost. A longer pause also drops queued frames. */
#ifndef STREAM_RETRY_MS
#define STREAM_RETRY_MS         50U     /**< Output pause after a runtime TX fault before resuming. */
#endif
#ifndef STREAM_FAULT_HOLD_MS
#define STREAM_FAULT_HOLD_MS    2000U   /**< Fault indication time after the last fault or drop. */
#endif
/* While sync markers arrive, the periodic STATUS waits this much longer, so it
 * does not precede a marker that is about due: the XM period equals
 * STREAM_STATUS_PERIOD_MS but runs on the XM clock with millisecond loop jitter.
 * Also covers main-loop delays of blocking B1 steps. */
#define STREAM_SYNC_GRACE_MS    50U     /**< Extra heartbeat wait while markers act as heartbeat. */
/* USER CONFIGURATION END ----------------------------------------------------*/

/* Private constants ---------------------------------------------------------*/
#define HEADER_BYTES           64U       /**< Offset of the first channel slot. */
#define CHANNEL_BYTES          24U       /**< Bytes per fixed connector slot. */
#define EVENT_OFFSET           352U      /**< uint32 event flags followed by uint32 press count. */
#define SYNC_OFFSET            360U      /**< Role, state, counters and marker history (80 bytes). */
#define SYNC_RECORDS           4U        /**< Marker records repeated in every packet. */
#define SYNC_RECORD_OFFSET     376U      /**< First (newest) 16-byte marker record. */
#define SYNC_RECORD_BYTES      16U       /**< Sequence, falling time, low time, generation, flags. */
#define SYNC_ROLE_RECEIVER     2U        /**< Shared sync block role: 1 XM source, 2 Nucleo receiver. */
#define SYNC_TICK_US           2U        /**< Shared sync block tick unit: 1 ms, 2 us. */
#define CRC_OFFSET             440U      /**< CRC covers all bytes preceding this offset. */
#define PACKET_DATA            1U        /**< New acquisition result. */
#define PACKET_STATUS          2U        /**< Latest snapshot; not a new sensor sample. */
#define CANONICAL_NAN          0x7FC00000UL /**< IEEE-754 quiet NaN on the wire. */
#define DATA_FLOW_MS           (2000U / MS5607_SAMPLE_HZ) /**< DATA this recent carries pending markers. */

#if STREAM_ENABLED > 1U || STREAM_QUEUE_PACKETS < 2U || STREAM_QUEUE_PACKETS > 64U
#error "Use STREAM_ENABLED 0/1 and a queue size between 2 and 64 packets"
#endif
#if STREAM_BAUD_RATE < (STREAM_PACKET_BYTES * MS5607_SAMPLE_HZ * 10U)
#error "Streaming baud rate cannot carry one packet per acquisition cycle"
#endif
#if (STREAM_TX_TIMEOUT_MS * STREAM_BAUD_RATE) <= (STREAM_PACKET_BYTES * 10000UL)
#error "TX timeout must exceed the packet wire time"
#endif
#if STREAM_STATUS_PERIOD_MS == 0U || STREAM_STATUS_PERIOD_MS > 60000U
#error "Use a status period between 1 and 60000 milliseconds"
#endif
#if STREAM_RETRY_MS == 0U || STREAM_RETRY_MS > 60000U
#error "Use a stream retry delay between 1 and 60000 milliseconds"
#endif
#if (STREAM_TX_TIMEOUT_MS + STREAM_RETRY_MS) * MS5607_SAMPLE_HZ >= STREAM_QUEUE_PACKETS * 1000U
#error "TX timeout plus retry delay must be shorter than the queue's sample periods"
#endif
#if STREAM_FAULT_HOLD_MS == 0U || STREAM_FAULT_HOLD_MS > 60000U
#error "Use a stream fault hold between 1 and 60000 milliseconds"
#endif
#if FLT_RADIX != 2 || FLT_MANT_DIG != 24 || FLT_MAX_EXP != 128
#error "Streaming requires IEEE-754 binary32 float"
#endif
#if HEADER_BYTES + MS5607_CHANNEL_COUNT * CHANNEL_BYTES != EVENT_OFFSET || EVENT_OFFSET + 8U != SYNC_OFFSET
#error "Packet offsets and channel count must agree with the protocol length"
#endif
#if SYNC_OFFSET + 16U != SYNC_RECORD_OFFSET || SYNC_RECORD_OFFSET + SYNC_RECORDS * SYNC_RECORD_BYTES != CRC_OFFSET || CRC_OFFSET + 4U != STREAM_PACKET_BYTES
#error "Sync block offsets must agree with the protocol length"
#endif

/* Private types -------------------------------------------------------------*/
/** Transmit state shared with callbacks; only foreground edits the queue. */
typedef enum {
    TX_OFF,        /**< Intentionally disabled or not initialized. */
    TX_IDLE,       /**< UART available for the oldest queued packet. */
    TX_BUSY,       /**< DMA/UART own the tail packet's bytes. */
    TX_DONE,       /**< UART wire completion awaits foreground retirement. */
    TX_ERROR,      /**< Asynchronous HAL error awaits foreground recovery. */
    TX_RECOVERING, /**< Runtime fault; IRQs off, output paused, packets still queued. */
    TX_STOPPED     /**< Initialization failed; no output or queueing until reboot. */
} TxState;

/* Private state -------------------------------------------------------------*/
static UART_HandleTypeDef *stream_uart; /**< BSP handle used exclusively for output. */
static DMA_HandleTypeDef tx_dma;        /**< Persistent DMA1 channel 1 HAL context. */
static uint8_t packets[STREAM_QUEUE_PACKETS][STREAM_PACKET_BYTES]; /**< Owned TX bytes. */
static uint32_t head;                  /**< Next free packet queue slot. */
static uint32_t tail;                  /**< Oldest queued slot, including active DMA. */
static uint32_t count;                 /**< Number of occupied packet slots. */
static bool fault_held;                /**< last_fault_ms is valid and its hold has not expired. */
static uint32_t last_fault_ms;         /**< HAL tick of the latest runtime fault or queue drop. */
static uint32_t packet_sequence;       /**< Incremented even when a new packet is dropped. */
static uint32_t last_source_ms;        /**< Last data/status attempt, for idle reporting. */
static uint32_t tx_start_ms;           /**< HAL tick at the current DMA start request. */
static uint32_t recovery_start_ms;     /**< HAL tick at which TX_RECOVERING was entered. */
static volatile TxState tx_state;      /**< Callback-to-foreground completion handoff. */
static volatile uint32_t callback_error; /**< Saved UART error bits from interrupt context. */
static StreamDiagnostics statistics;  /**< Foreground-owned counters and fault reason. */
static MS5607_Frame snapshot;          /**< Temporary complete sensor snapshot for encoding. */
static MS5607_Status snapshot_state;   /**< State captured with the snapshot, including queued transitions. */
static volatile uint32_t button_count; /**< ISR-owned accepted presses; read atomically by foreground. */
static uint32_t queued_button_count;   /**< Last count included in a successfully queued packet. */
static uint32_t last_button_ms;        /**< ISR-owned HAL tick for debounce, including timer wrap. */
static bool button_seen;               /**< ISR-owned marker allowing the first press immediately. */
static SyncMarker sync_history[SYNC_RECORDS]; /**< Latest marker records, newest at index 0. */
static bool sync_queued[SYNC_RECORDS];  /**< Record was in at least one successfully queued packet. */
static uint32_t sync_history_count;     /**< Valid entries in sync_history. */
static uint32_t sync_evicted;           /**< Records pushed out of the history before any packet carried them. */
static bool sync_pending;               /**< A record has not yet been queued in any packet. */
static bool data_seen;                  /**< last_data_ms is valid. */
static uint32_t last_data_ms;           /**< HAL tick of the latest queued DATA attempt. */
static bool marker_seen;                /**< last_marker_ms is valid. */
static uint32_t last_marker_ms;         /**< HAL tick at which the latest marker was ingested. */

/* Private serialization helpers --------------------------------------------*/
/** @brief Store a uint16 in wire byte order without alignment assumptions.
 *  @param out Address of two output bytes.
 *  @param value Integer to encode, least significant byte first. */
static void put_u16(uint8_t *out, uint16_t value)
{
    out[0] = (uint8_t)value;
    out[1] = (uint8_t)(value >> 8);
}

/** @brief Store a uint32 in wire byte order without sending compiler padding.
 *  @param out Address of four output bytes.
 *  @param value Integer or binary32 bit pattern to encode. */
static void put_u32(uint8_t *out, uint32_t value)
{
    for (uint8_t byte = 0; byte < 4U; ++byte) { /* Wire byte position. */
        out[byte] = (uint8_t)(value >> (8U * byte));
    }
}

/** @brief Encode a valid binary32 measurement or a canonical invalid NaN.
 *  @param out Address of four output bytes.
 *  @param value Measurement value; ignored when valid is false.
 *  @param valid Whether the source channel has MS5607_VALID status. */
static void put_measurement(uint8_t *out, float value, bool valid)
{
    uint32_t bits = CANONICAL_NAN; /* Explicit wire representation, independent of C NaN. */
    if (valid) {
        memcpy(&bits, &value, sizeof(bits));
    }
    put_u32(out, bits);
}

/** @brief Calculate CRC-32/ISO-HDLC using two nibble steps per byte.
 *  @param data Bytes covered by the checksum.
 *  @param length Number of bytes to process.
 *  @return Reflected CRC (poly 0xEDB88320, init/xorout 0xFFFFFFFF).
 *  @note The standard check vector "123456789" produces 0xCBF43926. */
static uint32_t packet_crc32(const uint8_t *data, uint32_t length)
{
    static const uint32_t table[16] = { /* Four reflected polynomial steps for each nibble. */
        0x00000000UL, 0x1DB71064UL, 0x3B6E20C8UL, 0x26D930ACUL,
        0x76DC4190UL, 0x6B6B51F4UL, 0x4DB26158UL, 0x5005713CUL,
        0xEDB88320UL, 0xF00F9344UL, 0xD6D6A3E8UL, 0xCB61B38CUL,
        0x9B64C2B0UL, 0x86D3D2D4UL, 0xA00AE278UL, 0xBDBDF21CUL
    };
    uint32_t crc = 0xFFFFFFFFUL; /* Running reflected remainder. */
    for (uint32_t i = 0; i < length; ++i) { /* Input byte index. */
        crc ^= data[i];
        crc = (crc >> 4) ^ table[crc & 0xFU];
        crc = (crc >> 4) ^ table[crc & 0xFU];
    }
    return crc ^ 0xFFFFFFFFUL;
}

/** @brief Serialize one complete packet into a free queue slot.
 *  @param out Destination with STREAM_PACKET_BYTES capacity.
 *  @param kind PACKET_DATA for a new frame, PACKET_STATUS for a repeated snapshot.
 *  @param now_ms Board uptime at enqueue, not the serial transmission time.
 *  @note Source snapshot is owned by foreground; diagnostics are sampled here. */
static void encode_packet(uint8_t *out, uint8_t kind, uint32_t now_ms)
{
    MS5607_Diagnostics sensor_diag; /* Sensor fault/cadence counters at enqueue. */
    ms5607_get_diagnostics(&sensor_diag);
    memset(out, 0, STREAM_PACKET_BYTES); /* Reserved bytes have a deterministic value. */
    out[0] = 'P'; out[1] = 'M'; out[2] = 'M'; out[3] = 'G';
    out[4] = 5U; /* Protocol version; version 5 carries the shared 80-byte sync block. */
    out[5] = kind;
    put_u16(out + 6, STREAM_PACKET_BYTES);
    put_u32(out + 8, packet_sequence);
    put_u32(out + 12, now_ms);
    put_u32(out + 16, snapshot.sequence);
    put_u32(out + 20, snapshot.scheduled_us);
    out[24] = (uint8_t)snapshot_state;
    out[25] = MS5607_CHANNEL_COUNT;
    put_u16(out + 26, (uint16_t)statistics.last_error);
    put_u32(out + 28, sensor_diag.missed_cycles);
    put_u32(out + 32, sensor_diag.overwritten_frames);
    put_u32(out + 36, statistics.packets_dropped);
    put_u32(out + 40, statistics.tx_errors);
    put_u32(out + 44, sensor_diag.calibration_errors);
    put_u32(out + 48, sensor_diag.transfer_errors);
    put_u32(out + 52, sensor_diag.adc_errors);
    put_u32(out + 56, sensor_diag.last_hal_error);
    put_u32(out + 60, statistics.last_hal_error);
    for (uint8_t i = 0; i < MS5607_CHANNEL_COUNT; ++i) { /* J1-J12 slot index. */
        uint8_t *slot = out + HEADER_BYTES + (uint32_t)i * CHANNEL_BYTES; /* Wire slot. */
        const MS5607_Measurement *value = &snapshot.channel[i]; /* Source connector. */
        bool valid = value->status == MS5607_VALID; /* Never encode invalid numbers as real data. */
        put_measurement(slot, value->pressure_pa, valid);
        put_measurement(slot + 4, value->temperature_c, valid);
        put_u32(slot + 8, value->pressure_start_us);
        put_u32(slot + 12, value->temperature_start_us);
        put_u32(slot + 16, value->sequence);
        slot[20] = (uint8_t)value->status;
        slot[21] = value->flags; /* One-shot flags are sent for every status. */
        put_u16(slot + 22, value->error_count);
    }
    uint32_t presses = button_count; /* Capture once; later interrupts remain pending. */
    put_u32(out + EVENT_OFFSET, presses != queued_button_count ? 1U : 0U);
    put_u32(out + EVENT_OFFSET + 4U, presses);
    SyncStatus sync; /* Receiver state and counters at enqueue. */
    sync_input_get_status(&sync);
    out[SYNC_OFFSET] = SYNC_ROLE_RECEIVER;
    out[SYNC_OFFSET + 1U] = (uint8_t)sync.state;
    out[SYNC_OFFSET + 2U] = (uint8_t)sync_history_count;
    out[SYNC_OFFSET + 3U] = SYNC_TICK_US;
    put_u16(out + SYNC_OFFSET + 4U, sync.generation); /* Offset 6: reserved, zero. */
    put_u32(out + SYNC_OFFSET + 8U, sync.fault_count);
    put_u32(out + SYNC_OFFSET + 12U, sync.events_lost + sync_evicted);
    for (uint32_t i = 0; i < sync_history_count; ++i) { /* Newest first; unused records stay zero. */
        uint8_t *record = out + SYNC_RECORD_OFFSET + i * SYNC_RECORD_BYTES; /* Wire record. */
        put_u32(record, sync_history[i].sequence);
        put_u32(record + 4, sync_history[i].falling_us);
        put_u32(record + 8, sync_history[i].width_us);
        put_u16(record + 12, sync_history[i].generation);
        put_u16(record + 14, sync_history[i].flags);
        sync_queued[i] = true; /* This encoder runs only after a free slot was found. */
    }
    sync_pending = false;
    put_u32(out + CRC_OFFSET, packet_crc32(out, CRC_OFFSET));
    queued_button_count = presses; /* This encoder runs only after a free slot was found. */
}

/** @brief Move new marker records from the sync input into the packet history.
 *  @param now_ms HAL tick of this pass, kept as the latest marker time.
 *  @note Foreground only. A record pushed out before any queued packet carried
 *        it counts as lost; the host also sees the gap in marker sequences. */
static void ingest_sync_markers(uint32_t now_ms)
{
    SyncMarker marker; /* Oldest record not yet taken from the sync input. */
    while (sync_input_pop(&marker)) {
        marker_seen = true;
        last_marker_ms = now_ms;
        if (sync_history_count == SYNC_RECORDS && !sync_queued[SYNC_RECORDS - 1U]) {
            sync_evicted++;
        }
        for (uint32_t i = SYNC_RECORDS - 1U; i > 0U; --i) { /* Shift older records back. */
            sync_history[i] = sync_history[i - 1U];
            sync_queued[i] = sync_queued[i - 1U];
        }
        sync_history[0] = marker;
        sync_queued[0] = false;
        if (sync_history_count < SYNC_RECORDS) {
            sync_history_count++;
        }
        sync_pending = true;
    }
}

/** @brief Decide whether an idle STATUS packet is due in this pass.
 *  @param now_ms Current HAL tick.
 *  @return true for a pending B1 event, a pending marker without flowing DATA,
 *          or an expired heartbeat period.
 *  @note Only called when no DATA frame is pending. Events wait for a free slot,
 *        so they never cause a drop. While markers arrive, each marker STATUS
 *        is the heartbeat; the periodic deadline gets STREAM_SYNC_GRACE_MS more.
 *        Unsigned tick differences handle HAL tick wrap. */
static bool status_due(uint32_t now_ms)
{
    bool free_slot = count < STREAM_QUEUE_PACKETS; /* An event packet cannot be dropped. */
    if (button_count != queued_button_count && free_slot) {
        return true;
    }
    bool data_flowing = data_seen && now_ms - last_data_ms < DATA_FLOW_MS; /* Next DATA carries markers. */
    if (sync_pending && !data_flowing && free_slot) {
        return true;
    }
    uint32_t period = STREAM_STATUS_PERIOD_MS; /* Heartbeat interval for this pass. */
    if (marker_seen && now_ms - last_marker_ms <= STREAM_STATUS_PERIOD_MS + STREAM_SYNC_GRACE_MS) {
        period += STREAM_SYNC_GRACE_MS;
    }
    return now_ms - last_source_ms >= period;
}

/* Private queue and failure handling ----------------------------------------*/
/** @brief Start or restart the fault indication hold.
 *  @param now_ms HAL tick of the runtime fault or queue drop.
 *  @note Foreground only. fault_held keeps a never-faulted stream from reporting
 *        a fault at boot, when HAL tick and last_fault_ms are both near zero. */
static void record_fault(uint32_t now_ms)
{
    last_fault_ms = now_ms;
    fault_held = true;
}

/** @brief Enqueue the current snapshot or count a drop if all slots are owned.
 *  @param kind Packet purpose (new measurement or idle status).
 *  @param now_ms Current HAL tick for the packet header and status timer.
 *  @note Only foreground calls this; never overwrite the in-flight tail slot.
 *        Also used while TX_RECOVERING, so packets wait for resumed output. */
static void enqueue_snapshot(uint8_t kind, uint32_t now_ms)
{
    packet_sequence++;
    last_source_ms = now_ms;
    if (count == STREAM_QUEUE_PACKETS) {
        statistics.packets_dropped++;
        record_fault(now_ms);
        return;
    }
    encode_packet(packets[head], kind, now_ms);
    head = (head + 1U) % STREAM_QUEUE_PACKETS;
    count++;
    statistics.packets_queued++;
    if (count > statistics.max_queue_depth) {
        statistics.max_queue_depth = count;
    }
}

/** @brief Stop output permanently after an initialization failure.
 *  @param reason STREAM_ERROR_ARGUMENT, _UART_INIT or _DMA_INIT.
 *  @param hal_error Saved UART HAL error bits (zero if none were reported).
 *  @note stream_init() only; retrying cannot fix these faults. No packet has been
 *        queued yet. Sensor acquisition and LD2 servicing remain active. */
static void stop_stream(StreamError reason, uint32_t hal_error)
{
    /* Disable owned IRQs before STOPPED makes their forwarding hooks inert.
     * SysTick and sensor IRQs remain enabled so their work can still progress. */
    if (stream_uart != NULL) {
        HAL_NVIC_DisableIRQ(DMA1_Channel1_IRQn);
        HAL_NVIC_DisableIRQ(LPUART1_IRQn);
    }
    tx_state = TX_STOPPED;
    ms5607_set_frame_consumer(false);
    statistics.tx_errors++;
    statistics.last_error = reason;
    statistics.last_hal_error = hal_error;
    if (stream_uart != NULL) {
        (void)HAL_UART_AbortTransmit(stream_uart);
        HAL_NVIC_ClearPendingIRQ(DMA1_Channel1_IRQn);
        HAL_NVIC_ClearPendingIRQ(LPUART1_IRQn);
    }
}

/** @brief Abort the in-flight packet after a runtime fault and pause output.
 *  @param reason STREAM_ERROR_TX_START, _TX_CALLBACK or _TX_TIMEOUT.
 *  @param hal_error Saved UART HAL error bits (zero if none were reported).
 *  @param now_ms HAL tick of the fault; starts the STREAM_RETRY_MS pause.
 *  @note Foreground only, while the tail slot is owned by a started or rejected
 *        transfer (TX_BUSY or TX_ERROR, so count is at least one). Only that slot
 *        is dropped and freed; the remaining queue is kept for resume_stream().
 *        HAL abort may wait for its own timeout in exceptional faults. */
static void recover_stream(StreamError reason, uint32_t hal_error, uint32_t now_ms)
{
    /* Disable owned IRQs before RECOVERING makes their forwarding hooks inert.
     * HAL_UART_AbortTransmit uses a polling DMA abort; it needs no TX callback.
     * SysTick and sensor IRQs remain enabled so their work can still progress. */
    HAL_NVIC_DisableIRQ(DMA1_Channel1_IRQn);
    HAL_NVIC_DisableIRQ(LPUART1_IRQn);
    tx_state = TX_RECOVERING; /* Ignore completion events during abort and pause. */
    statistics.tx_errors++;
    statistics.last_error = reason;
    statistics.last_hal_error = hal_error;
    record_fault(now_ms);
    recovery_start_ms = now_ms;
    (void)HAL_UART_AbortTransmit(stream_uart);
    HAL_NVIC_ClearPendingIRQ(DMA1_Channel1_IRQn);
    HAL_NVIC_ClearPendingIRQ(LPUART1_IRQn);
    if (count != 0U) { /* Defensive; a faulted transfer always owns the tail. */
        tail = (tail + 1U) % STREAM_QUEUE_PACKETS;
        count--;
        statistics.packets_dropped++;
    }
}

/** @brief Re-arm the transport after the recovery pause.
 *  @note Foreground only, in TX_RECOVERING. Stale IRQs raised before or during
 *        the abort are discarded. last_error is kept as the latest fault reason.
 *        The caller starts the oldest queued packet in the same pass. */
static void resume_stream(void)
{
    HAL_NVIC_ClearPendingIRQ(DMA1_Channel1_IRQn);
    HAL_NVIC_ClearPendingIRQ(LPUART1_IRQn);
    tx_state = TX_IDLE; /* Hooks may forward to HAL once the IRQs are enabled. */
    HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
    HAL_NVIC_EnableIRQ(LPUART1_IRQn);
}

/* Public initialization and foreground service ------------------------------*/
/** @brief Initialize the binary output queue and its UART DMA resources.
 *  @param uart Already initialized BSP LPUART1 handle.
 *  @return true when ready or disabled, false if configuration failed.
 *  @note Call once at startup. The first status snapshot is scheduled immediately. */
bool stream_init(UART_HandleTypeDef *uart)
{
    ms5607_set_frame_consumer(false); /* Disabled/failed output cannot hold B1 at a frame handoff. */
    memset(&statistics, 0, sizeof(statistics));
    memset(&tx_dma, 0, sizeof(tx_dma));
    stream_uart = NULL;
    head = tail = count = packet_sequence = 0U;
    last_fault_ms = recovery_start_ms = 0U;
    fault_held = false;
    tx_state = TX_OFF;
    callback_error = HAL_UART_ERROR_NONE;
    button_count = queued_button_count = last_button_ms = 0U;
    button_seen = false;
    memset(sync_history, 0, sizeof(sync_history));
    memset(sync_queued, 0, sizeof(sync_queued));
    sync_history_count = sync_evicted = 0U;
    sync_pending = data_seen = marker_seen = false;
    last_data_ms = last_marker_ms = 0U;
    if (STREAM_ENABLED == 0U) {
        return true;
    }
    if (uart == NULL || uart->Instance != LPUART1 ||
        uart->Init.WordLength != UART_WORDLENGTH_8B ||
        uart->Init.Parity != UART_PARITY_NONE || uart->Init.StopBits != UART_STOPBITS_1 ||
        uart->Init.HwFlowCtl != UART_HWCONTROL_NONE || uart->hdmatx != NULL) {
        stop_stream(STREAM_ERROR_ARGUMENT, 0U);
        return false;
    }
    stream_uart = uart;
    uart->Init.BaudRate = STREAM_BAUD_RATE;
    if (HAL_UART_Init(uart) != HAL_OK) {
        stop_stream(STREAM_ERROR_UART_INIT, HAL_UART_GetError(uart));
        return false;
    }
    __HAL_RCC_DMA1_CLK_ENABLE();
    __HAL_RCC_DMAMUX1_CLK_ENABLE();
    tx_dma.Instance = DMA1_Channel1;
    tx_dma.Init.Request = DMA_REQUEST_LPUART1_TX;
    tx_dma.Init.Direction = DMA_MEMORY_TO_PERIPH;
    tx_dma.Init.PeriphInc = DMA_PINC_DISABLE;
    tx_dma.Init.MemInc = DMA_MINC_ENABLE;
    tx_dma.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    tx_dma.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    tx_dma.Init.Mode = DMA_NORMAL;
    tx_dma.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&tx_dma) != HAL_OK) {
        stop_stream(STREAM_ERROR_DMA_INIT, HAL_UART_GetError(uart));
        return false;
    }
    __HAL_LINKDMA(uart, hdmatx, tx_dma);
    HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 2U, 0U);
    HAL_NVIC_SetPriority(LPUART1_IRQn, 2U, 0U);
    HAL_NVIC_ClearPendingIRQ(DMA1_Channel1_IRQn);
    HAL_NVIC_ClearPendingIRQ(LPUART1_IRQn);
    tx_state = TX_IDLE;
    HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
    HAL_NVIC_EnableIRQ(LPUART1_IRQn);
    last_source_ms = HAL_GetTick() - STREAM_STATUS_PERIOD_MS;
    ms5607_set_frame_consumer(true);
    return true;
}

/** @brief Service completion, ingest status transitions/new DATA and start output.
 *  @note Foreground after ms5607_process(). A new frame suppresses idle status
 *        output, so STATUS packets are never treated as extra sensor samples.
 *        A pending sync marker queues STATUS when no DATA flows and a slot is free.
 *        Frames are still queued while TX_RECOVERING; output resumes with the
 *        oldest queued packet once STREAM_RETRY_MS has elapsed. */
void stream_process(void)
{
    if (tx_state == TX_OFF || tx_state == TX_STOPPED) {
        return;
    }
    uint32_t now_ms = HAL_GetTick(); /* Timer for status reports and transport deadlines. */
    if (fault_held && now_ms - last_fault_ms >= STREAM_FAULT_HOLD_MS) {
        fault_held = false; /* Expire before the uint32 tick difference can wrap. */
    }
    if (tx_state == TX_ERROR) {
        recover_stream(STREAM_ERROR_TX_CALLBACK, callback_error, now_ms);
    } else if (tx_state == TX_DONE) {
        tail = (tail + 1U) % STREAM_QUEUE_PACKETS;
        count--;
        statistics.packets_sent++;
        tx_state = TX_IDLE;
    } else if (tx_state == TX_BUSY && now_ms - tx_start_ms >= STREAM_TX_TIMEOUT_MS) {
        recover_stream(STREAM_ERROR_TX_TIMEOUT, HAL_UART_GetError(stream_uart), now_ms);
    } else if (tx_state == TX_RECOVERING && now_ms - recovery_start_ms >= STREAM_RETRY_MS) {
        resume_stream();
    }

    /* The driver preserves the final pre-pause DATA until handoff. Its queued
     * transition snapshots therefore precede any currently pending resumed DATA.
     * Drain the bounded notification FIFO even under UART backpressure: normal
     * enqueue/drop accounting must never keep sensor recovery paused. */
    ingest_sync_markers(now_ms); /* Before encoding, so this pass's packets carry new markers. */
    bool transition_seen = false; /* Suppress redundant periodic STATUS this pass. */
    while (ms5607_pop_status(&snapshot, &snapshot_state)) {
        enqueue_snapshot(PACKET_STATUS, now_ms);
        transition_seen = true;
    }
    uint32_t status_drops = ms5607_status_drops(); /* Notifications lost before stream ingestion. */
    if (status_drops != 0U) {
        packet_sequence += status_drops;
        statistics.packets_dropped += status_drops;
        record_fault(now_ms);
    }
    if (ms5607_frame_pending()) {
        (void)ms5607_copy_frame(&snapshot);
        snapshot_state = ms5607_status();
        enqueue_snapshot(PACKET_DATA, now_ms);
        data_seen = true;
        last_data_ms = now_ms;
    } else if (!transition_seen && status_due(now_ms)) {
        ms5607_copy_status(&snapshot, &snapshot_state);
        enqueue_snapshot(PACKET_STATUS, now_ms);
    }

    if (tx_state == TX_IDLE && count != 0U) {
        tx_start_ms = now_ms;
        callback_error = HAL_UART_ERROR_NONE;
        __DMB();
        tx_state = TX_BUSY; /* Set ownership before DMA/IRQ callbacks can occur. */
        HAL_StatusTypeDef result = HAL_UART_Transmit_DMA(stream_uart, packets[tail],
                                                       STREAM_PACKET_BYTES); /* Start result. */
        if (result != HAL_OK) {
            recover_stream(STREAM_ERROR_TX_START, HAL_UART_GetError(stream_uart), now_ms);
        }
    }
}

/** @brief Report a current or recent transport fault or queue overflow.
 *  @return true while stopped or recovering, and for STREAM_FAULT_HOLD_MS after
 *          the latest runtime fault or dropped packet; false otherwise and at boot.
 *  @note Foreground only. Unsigned tick subtraction handles HAL tick wrap. */
bool stream_faulted(void)
{
    if (tx_state == TX_STOPPED || tx_state == TX_RECOVERING) {
        return true;
    }
    return fault_held && HAL_GetTick() - last_fault_ms < STREAM_FAULT_HOLD_MS;
}

/** @brief Copy stream counters and present queue occupancy without clearing them.
 *  @param out Foreground destination; NULL is ignored. */
void stream_get_diagnostics(StreamDiagnostics *out)
{
    if (out != NULL) {
        *out = statistics;
        out->queue_depth = count;
    }
}

/* Interrupt hooks -----------------------------------------------------------*/
/** @brief Debounce the B1 rising edge and publish an event counter to foreground.
 *  @return true if the press was accepted, false inside the debounce window.
 *  @note BSP EXTI context only. Unsigned subtraction handles HAL tick wrap.
 *        No sensor state, queue storage or UART resources are changed here; the
 *        caller starts the sensor re-init for an accepted press. Also works
 *        with STREAM_ENABLED 0, which only skips the transport setup. */
bool stream_button_pressed(void)
{
    uint32_t now_ms = HAL_GetTick(); /* Edge time for debounce, not a sensor timestamp. */
    if (button_seen && now_ms - last_button_ms < STREAM_BUTTON_DEBOUNCE_MS) {
        return false;
    }
    button_seen = true;
    last_button_ms = now_ms;
    button_count++;
    return true;
}

/** @brief Publish a UART wire-complete event; do not release queue storage here.
 *  @param uart HAL callback handle; only the active module-owned handle is used.
 *  @note Ignored unless TX_BUSY, so a late event after an abort cannot retire
 *        the wrong slot while recovering or stopped. */
void stream_uart_complete(UART_HandleTypeDef *uart)
{
    if (uart == stream_uart && tx_state == TX_BUSY) {
        tx_state = TX_DONE;
    }
}

/** @brief Preserve an asynchronous transport error for foreground recovery.
 *  @param uart HAL callback handle; unrelated, recovering or stopped transfers
 *         are ignored (only TX_BUSY is accepted). */
void stream_uart_error(UART_HandleTypeDef *uart)
{
    if (uart == stream_uart && tx_state == TX_BUSY) {
        callback_error = HAL_UART_GetError(uart);
        tx_state = TX_ERROR;
    }
}

/** @brief Check whether the stream IRQs are meant to be serviced by HAL.
 *  @return false while off, stopped or recovering, when both IRQs are disabled.
 *  @note A spurious hook call in those states must not touch HAL state. */
static bool irq_forwarding(void)
{
    TxState state = tx_state; /* One volatile read for a consistent decision. */
    return stream_uart != NULL && state != TX_OFF && state != TX_RECOVERING &&
           state != TX_STOPPED;
}

/** @brief Handle DMA1 channel 1 events through its persistent HAL handle. */
void stream_dma_irq(void)
{
    if (irq_forwarding()) {
        HAL_DMA_IRQHandler(&tx_dma);
    }
}

/** @brief Handle LPUART1 final transmission completion and UART error events. */
void stream_uart_irq(void)
{
    if (irq_forwarding()) {
        HAL_UART_IRQHandler(stream_uart);
    }
}
