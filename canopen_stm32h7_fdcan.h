#ifndef CANOPEN_STM32H7_FDCAN_H
#define CANOPEN_STM32H7_FDCAN_H

#include "canopen.h"
#include "stm32h7xx_hal.h"

/* Maximum frames processed per co_stm32_poll_rx() call (legacy polling path).
 * Caps the worst-case execution time of a single poll under bus overload. */
#define CO_STM32_RX_MAX_PER_POLL 32U

/* Ring buffer capacity for the ISR→main-loop frame queue.  Must be a power of
 * two.  At 1 ms SYNC with up to ~10 frames/ms, 64 slots gives ~6 ms headroom
 * before overflow under sustained load. */
#define CO_STM32_RX_QUEUE_SIZE   64U

/* SPSC ring buffer populated by co_stm32_rx_isr() and drained by
 * co_stm32_drain_rx().  head is written only by the ISR; tail only by the
 * main loop — no mutex required on single-core Cortex-M. */
typedef struct {
    co_can_frame_t      buf[CO_STM32_RX_QUEUE_SIZE];
    volatile uint32_t   head;   /* next write slot  (ISR)       */
    volatile uint32_t   tail;   /* next read  slot  (main loop) */
} co_stm32_rx_queue_t;

typedef struct {
    FDCAN_HandleTypeDef  *hfdcan;
    co_stm32_rx_queue_t   rx_queue;
} co_stm32_ctx_t;

co_error_t co_stm32_send(void *user, const co_can_frame_t *frame);
uint32_t   co_stm32_millis(void *user);

/* ISR entry point — call from HAL_FDCAN_RxFifo0MsgPendingCallback().
 * Drains the FDCAN RX FIFO into the ring buffer.  For heartbeat frames the
 * consumer timestamp is updated immediately so co_process() never fires a
 * false timeout due to main-loop latency. */
void co_stm32_rx_isr(co_node_t *node, co_stm32_ctx_t *ctx);

/* Main-loop drain — replaces co_stm32_poll_rx().  Pops every queued frame and
 * delivers it to co_on_can_rx() so the full stack (SDO, PDO, NMT callbacks)
 * runs in task context, not ISR context. */
void co_stm32_drain_rx(co_node_t *node, co_stm32_ctx_t *ctx);

/* Legacy polled path — kept for non-ISR setups. */
void co_stm32_poll_rx(co_node_t *node, FDCAN_HandleTypeDef *hfdcan);

void co_stm32_attach(co_node_t *node,
                     FDCAN_HandleTypeDef *hfdcan,
                     co_stm32_ctx_t *ctx,
                     uint8_t node_id,
                     uint16_t heartbeat_ms);

#endif
