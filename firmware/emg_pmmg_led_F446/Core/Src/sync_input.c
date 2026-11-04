/**
  ******************************************************************************
  * @file    sync_input.c
  * @brief   TIM2 input capture of XM synchronization pulses on J12 (PB10).
  *
  * Channel 3 (CC3S = 01, direct TI3, CC3P = 1) latches falling edges in CCR3;
  * channel 4 (CC4S = 10, indirect TI3, CC4P = 0) latches rising edges of the
  * same input in CCR4. Separate registers keep edge identity even when both
  * edges occur before the interrupt runs; the edge kind is never guessed from
  * the current pin level or the interrupt-entry time. The TI3 digital filter
  * (IC3F) needs 8 stable timer-clock samples (0.33 us at 24 MHz); it delays both
  * edges equally and rejects short cable glitches. The time base (PSC, ARR,
  * CNT) belongs to timebase.c and is never touched here.
  *
  * Interrupt: reading CCRx clears CCxIF in input-capture mode, so no flag is
  * cleared by a status write that could erase a newer capture. Overcapture
  * (CCxOF) is checked after the reads, so a capture that overwrote a value just
  * read is also seen; it marks at least one lost edge. Both pending edges are
  * queued in timestamp order. Falling edges are numbered here, including lost
  * ones that are known, so queue overflow shows up as a sequence gap.
  *
  * Foreground: sync_input_process() reads the clock first and then drains the
  * edge queue. Because pending interrupts preempt the main loop, every edge
  * captured before that clock read is already queued, so a delayed main loop
  * never times out a pulse that actually completed. Pairing, validation,
  * timeouts and the state all live in this one context.
  *
  * Rules: a valid pulse is a falling edge followed by a rising edge
  * SYNC_INPUT_MIN_LOW_US..SYNC_INPUT_MAX_LOW_US later. A falling edge without a
  * rising edge within SYNC_INPUT_TIMEOUT_US, or followed by another falling
  * edge, is published INVALID | INCOMPLETE. A rising edge while the line was
  * already low (at startup or after a timeout) is discarded silently; any other
  * unmatched rising edge counts as a fault. Lost edges raise the generation and
  * invalidate the affected pulses. The receiver never drives the line.
  * Specification: synchroizationXMtoNucelo.md section 4.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "sync_input.h"
#include "main.h"
#include "timebase.h"

/* Private constants ---------------------------------------------------------*/
#define EDGE_QUEUE        16U /**< Raw edges for the foreground: 2 s of pattern pulses. */
#define MARKER_QUEUE      8U  /**< Completed markers awaiting the stream. */
#define TIM2_IRQ_PRIORITY 2U  /**< Below sensor SPI (1): capture latching makes latency harmless. */
#define IC3_FILTER        3U  /**< IC3F = 0011: fCK_INT, N = 8. */

/** CCMR2 input setup: CH3 = TI3, CH4 = TI3 (indirect), no prescaler, TI3 filter. */
#define CCMR2_CAPTURE ((1UL << TIM_CCMR2_CC3S_Pos) | ((uint32_t)IC3_FILTER << TIM_CCMR2_IC3F_Pos) | \
                       (2UL << TIM_CCMR2_CC4S_Pos))
#define CAPTURE_FLAGS (TIM_SR_CC3IF | TIM_SR_CC4IF)
#define CCER_BITS     (TIM_CCER_CC3E | TIM_CCER_CC3P | TIM_CCER_CC3NP | \
                       TIM_CCER_CC4E | TIM_CCER_CC4P | TIM_CCER_CC4NP)

/* Raw edge flags. */
#define EDGE_RISING      0x01U /**< Rising edge (CCR4); otherwise falling (CCR3). */
#define EDGE_OVERCAPTURE 0x02U /**< This channel overwrote at least one earlier capture. */
#define EDGE_AFTER_LOSS  0x04U /**< At least one edge before this one was dropped (queue full). */

/* Private types -------------------------------------------------------------*/
/** One captured edge, queued by the interrupt for the foreground. */
typedef struct {
    uint32_t time_us;  /**< Latched capture value. */
    uint32_t sequence; /**< Falling-edge number; unused for rising edges. */
    uint8_t flags;     /**< EDGE_* bits. */
} Edge;

/* Private state -------------------------------------------------------------*/
/* Interrupt-owned after sync_input_init(); the foreground reads the queue head
 * and the drop counter only. */
static Edge edge_queue[EDGE_QUEUE];     /**< Edges awaiting the foreground. */
static volatile uint32_t edge_head;     /**< Free-running write index, interrupt only. */
static volatile uint32_t edge_tail;     /**< Free-running read index, foreground only. */
static uint32_t falling_sequence;       /**< Falling edges captured or known lost since boot. */
static bool edge_dropped;               /**< The next queued edge follows a dropped one. */
static volatile uint32_t edges_dropped; /**< Edges dropped because the queue was full. */

/* Foreground-owned. */
static SyncState sync_state;           /**< Published receiver state. */
static uint16_t sync_generation;       /**< Capture generation. */
static uint32_t sync_faults;           /**< Invalid markers and unmatched rising edges. */
static uint32_t sync_lost;             /**< Overcaptures and markers dropped on queue overflow. */
static uint32_t state_since_us;        /**< Time of the event that set the current state. */
static bool awaiting_release;          /**< The line may already be low: discard one rising edge. */
static bool loss_open;                 /**< The previous edge reported a loss (one generation step). */
static bool pulse_open;               /**< pulse holds a falling edge without its end. */
static SyncMarker pulse;               /**< Pulse being measured. */
static SyncMarker marker_queue[MARKER_QUEUE]; /**< Markers awaiting the stream. */
static uint32_t marker_head;           /**< Free-running write index. */
static uint32_t marker_tail;           /**< Free-running read index. */

/* Private functions: interrupt side -----------------------------------------*/
/**
 * @brief Queue one edge, or count it as dropped when the queue is full.
 * @param time_us Capture value.
 * @param sequence Falling-edge number (zero for rising edges).
 * @param flags EDGE_* bits.
 */
static void queue_edge(uint32_t time_us, uint32_t sequence, uint8_t flags)
{
    uint32_t head = edge_head; /* Only the interrupt writes the head. */
    if (head - edge_tail >= EDGE_QUEUE) {
        edges_dropped++;
        edge_dropped = true;
        return;
    }
    Edge *edge = &edge_queue[head % EDGE_QUEUE]; /* Free slot. */
    edge->time_us = time_us;
    edge->sequence = sequence;
    edge->flags = (uint8_t)(flags | (edge_dropped ? EDGE_AFTER_LOSS : 0U));
    edge_dropped = false;
    __DMB(); /* Record complete before the foreground can see the new head. */
    edge_head = head + 1U;
}

/* Private functions: foreground side ----------------------------------------*/
/** @brief Take the oldest queued edge; false when the queue is empty. */
static bool pop_edge(Edge *out)
{
    uint32_t tail = edge_tail; /* Only the foreground writes the tail. */
    if (tail == edge_head) {
        return false;
    }
    __DMB(); /* Head observed before reading the record it publishes. */
    *out = edge_queue[tail % EDGE_QUEUE];
    __DMB(); /* Record copied before its slot is released to the interrupt. */
    edge_tail = tail + 1U;
    return true;
}

/** @brief Enter a state at the time of the event that caused it. */
static void set_state(SyncState state, uint32_t when_us)
{
    sync_state = state;
    state_since_us = when_us;
}

/**
 * @brief Publish the measured pulse and update the state from its validity.
 * @param width_us Measured low time or SYNC_WIDTH_UNKNOWN.
 * @param flags SYNC_FLAG_* bits added to those of the falling edge.
 * @param when_us Time of the event that completed the pulse.
 */
static void finish_pulse(uint32_t width_us, uint16_t flags, uint32_t when_us)
{
    pulse_open = false;
    pulse.width_us = width_us;
    pulse.flags |= flags;
    if (marker_head - marker_tail >= MARKER_QUEUE) {
        sync_lost++;
    } else {
        marker_queue[marker_head % MARKER_QUEUE] = pulse;
        marker_head++;
    }
    if ((pulse.flags & SYNC_FLAG_INVALID) != 0U) {
        sync_faults++;
        set_state(SYNC_FAULT, when_us);
    } else {
        set_state(SYNC_RECEIVING, when_us);
    }
}

/** @brief Apply one captured edge to the pulse being measured. */
static void handle_edge(const Edge *edge)
{
    uint16_t loss = 0U; /* SYNC_FLAG_OVERCAPTURE if edges were lost before this one. */
    if ((edge->flags & (EDGE_OVERCAPTURE | EDGE_AFTER_LOSS)) != 0U) {
        loss = SYNC_FLAG_OVERCAPTURE;
        if ((edge->flags & EDGE_OVERCAPTURE) != 0U) {
            sync_lost++; /* Dropped edges are counted by the interrupt. */
        }
        if (!loss_open) { /* Consecutive lossy edges are one continuity break. */
            sync_generation++;
        }
    }
    loss_open = loss != 0U;
    if ((edge->flags & EDGE_RISING) != 0U) {
        if (!pulse_open) {
            if (loss != 0U || !awaiting_release) { /* Not explained by a line already low. */
                sync_faults++;
                set_state(SYNC_FAULT, edge->time_us);
            }
            awaiting_release = false;
            return;
        }
        uint32_t width = edge->time_us - pulse.falling_us; /* Wrap-safe. */
        bool valid = loss == 0U && (pulse.flags & SYNC_FLAG_INVALID) == 0U &&
                     width >= SYNC_INPUT_MIN_LOW_US && width <= SYNC_INPUT_MAX_LOW_US;
        finish_pulse(width, (uint16_t)(loss | (valid ? 0U : SYNC_FLAG_INVALID)), edge->time_us);
        return;
    }
    if (pulse_open) { /* Second falling edge: the first pulse never ended. */
        finish_pulse(SYNC_WIDTH_UNKNOWN,
                       (uint16_t)(SYNC_FLAG_INVALID | SYNC_FLAG_INCOMPLETE | loss), edge->time_us);
    }
    pulse.sequence = edge->sequence;
    pulse.falling_us = edge->time_us;
    pulse.width_us = SYNC_WIDTH_UNKNOWN;
    pulse.generation = sync_generation;
    pulse.flags = loss != 0U ? (uint16_t)(SYNC_FLAG_INVALID | loss) : 0U;
    pulse_open = true;
    awaiting_release = false;
}

/* Public functions ----------------------------------------------------------*/
/**
 * @brief Hand PB10 to TIM2 input capture, or back to the sensor driver.
 * @note Startup only. Input selection is written while both channels are off
 *       and before PB10 becomes an alternate-function pin, so the timer can
 *       never drive it. Flags raised by the pin handover are discarded.
 */
void sync_input_init(void)
{
    edge_head = edge_tail = 0U;
    falling_sequence = edges_dropped = 0U;
    edge_dropped = false;
    sync_state = SYNC_DISABLED;
    sync_generation = 0U;
    sync_faults = sync_lost = state_since_us = 0U;
    awaiting_release = true; /* The line may be low already: XM unpowered or mid-pulse. */
    loss_open = pulse_open = false;
    marker_head = marker_tail = 0U;
    if (SYNC_INPUT_ENABLED == 0U) {
        /* J12 is a sensor chip select: idle high before it becomes an output. */
        HAL_GPIO_WritePin(SYNC_INPUT_GPIO_Port, SYNC_INPUT_Pin, GPIO_PIN_SET);
        GPIO_InitTypeDef cs = {0};
        cs.Pin = SYNC_INPUT_Pin;
        cs.Mode = GPIO_MODE_OUTPUT_PP;
        cs.Pull = GPIO_NOPULL;
        cs.Speed = GPIO_SPEED_FREQ_LOW;
        HAL_GPIO_Init(SYNC_INPUT_GPIO_Port, &cs);
        return;
    }
    if (!timebase_ready()) {
        sync_state = SYNC_CONFIG_ERROR; /* PB10 stays the generated GPIO input. */
        return;
    }
    HAL_NVIC_DisableIRQ(TIM2_IRQn);
    TIM2->DIER &= ~(TIM_DIER_CC3IE | TIM_DIER_CC4IE);
    TIM2->CCER &= ~CCER_BITS;      /* CCxS is writable only while the channel is off. */
    TIM2->CCMR2 = CCMR2_CAPTURE;   /* Whole register: channels 3 and 4 belong to this module. */
    TIM2->CCER |= TIM_CCER_CC3P | TIM_CCER_CC3E | TIM_CCER_CC4E; /* CH3 falling, CH4 rising. */

    GPIO_InitTypeDef pin = {0}; /* PB10 as TIM2_CH3 input; never driven by the timer. */
    pin.Pin = SYNC_INPUT_Pin;
    pin.Mode = GPIO_MODE_AF_PP;
    pin.Pull = GPIO_NOPULL; /* R12 on the carrier is the pull-up. */
    pin.Speed = GPIO_SPEED_FREQ_LOW;
    pin.Alternate = SYNC_INPUT_AF;
    HAL_GPIO_Init(SYNC_INPUT_GPIO_Port, &pin);

    /* rc_w0: discard captures caused by connecting the pin; other flags unchanged. */
    TIM2->SR = ~(TIM_SR_CC3IF | TIM_SR_CC4IF | TIM_SR_CC3OF | TIM_SR_CC4OF);
    set_state(SYNC_WAITING, timebase_now_us());
    TIM2->DIER |= TIM_DIER_CC3IE | TIM_DIER_CC4IE;
    HAL_NVIC_SetPriority(TIM2_IRQn, TIM2_IRQ_PRIORITY, 0U);
    HAL_NVIC_ClearPendingIRQ(TIM2_IRQn);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);
}

/**
 * @brief Queue the edges latched in CCR3 (falling) and CCR4 (rising).
 * @note TIM2 interrupt context. The capture registers hold the edge times; the
 *       interrupt-entry time is never used.
 */
void sync_input_irq(void)
{
    uint32_t pending_flags = TIM2->SR & CAPTURE_FLAGS; /* Captures to service now. */
    uint32_t enabled = TIM2->DIER;
    if ((enabled & TIM_DIER_CC3IE) == 0U) pending_flags &= ~TIM_SR_CC3IF;
    if ((enabled & TIM_DIER_CC4IE) == 0U) pending_flags &= ~TIM_SR_CC4IF;
    if (pending_flags == 0U) {
        return;
    }
    bool fall = (pending_flags & TIM_SR_CC3IF) != 0U;
    bool rise = (pending_flags & TIM_SR_CC4IF) != 0U;
    uint32_t fall_us = fall ? TIM2->CCR3 : 0U; /* Each read clears its CCxIF. */
    uint32_t rise_us = rise ? TIM2->CCR4 : 0U;
    uint32_t status = TIM2->SR; /* After the reads, so an overwrite during them is seen. */
    uint8_t fall_flags = 0U;
    uint8_t rise_flags = EDGE_RISING;
    if (fall && (status & TIM_SR_CC3OF) != 0U) {
        fall_flags |= EDGE_OVERCAPTURE;
        TIM2->SR = ~TIM_SR_CC3OF; /* rc_w0: clears only this flag. */
    }
    if (rise && (status & TIM_SR_CC4OF) != 0U) {
        rise_flags |= EDGE_OVERCAPTURE;
        TIM2->SR = ~TIM_SR_CC4OF;
    }
    if (fall) {
        falling_sequence += (fall_flags & EDGE_OVERCAPTURE) != 0U ? 2U : 1U; /* Lost edge counted. */
    }
    if (fall && rise && (int32_t)(rise_us - fall_us) < 0) { /* Rising edge of the previous pulse first. */
        queue_edge(rise_us, 0U, rise_flags);
        queue_edge(fall_us, falling_sequence, fall_flags);
    } else {
        if (fall) queue_edge(fall_us, falling_sequence, fall_flags);
        if (rise) queue_edge(rise_us, 0U, rise_flags);
    }
}

/**
 * @brief Pair edges, apply the timeout and update the receiver state.
 * @note Foreground only. Signed differences tolerate edges captured after the
 *       clock read; the state is held until the next event, so a counter wrap
 *       during a long silence cannot make a stale receiver look fresh.
 */
void sync_input_process(void)
{
    if (sync_state == SYNC_DISABLED || sync_state == SYNC_CONFIG_ERROR) {
        return;
    }
    uint32_t now = timebase_now_us(); /* Before draining; see the file header. */
    Edge edge;
    while (pop_edge(&edge)) {
        handle_edge(&edge);
    }
    if (pulse_open && (int32_t)(now - pulse.falling_us) >= (int32_t)SYNC_INPUT_TIMEOUT_US) {
        finish_pulse(SYNC_WIDTH_UNKNOWN, SYNC_FLAG_INVALID | SYNC_FLAG_INCOMPLETE, now);
        awaiting_release = true; /* The line is still low; its release is not a new fault. */
    }
    if ((sync_state == SYNC_RECEIVING || sync_state == SYNC_FAULT) &&
        (int32_t)(now - state_since_us) >= (int32_t)SYNC_INPUT_STALE_US) {
        sync_state = SYNC_STALE;
    }
}

/** @brief Copy and remove the oldest published marker; foreground only. */
bool sync_input_pop(SyncMarker *out)
{
    if (out == NULL || marker_tail == marker_head) {
        return false;
    }
    *out = marker_queue[marker_tail % MARKER_QUEUE];
    marker_tail++;
    return true;
}

/** @brief Copy state and counters; foreground only. */
void sync_input_get_status(SyncStatus *out)
{
    if (out == NULL) {
        return;
    }
    out->state = sync_state;
    out->generation = sync_generation;
    out->fault_count = sync_faults;
    out->events_lost = sync_lost + edges_dropped; /* One aligned read of the interrupt counter. */
}
