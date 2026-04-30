#ifndef CANOPEN_STM32G4_FDCAN_H
#define CANOPEN_STM32G4_FDCAN_H

#include "canopen.h"
#include "stm32g4xx_hal.h"

/* Maximum frames drained per co_stm32g4_drain_rx() call on the legacy poll
 * path.  Caps worst-case execution time under bus overload. */
#ifndef CO_STM32G4_RX_MAX_PER_POLL
#  define CO_STM32G4_RX_MAX_PER_POLL 32U
#endif

/* ISR→main-loop frame ring buffer capacity.  Must be a power of two.
 * At 1 ms SYNC with ~10 frames/ms, 64 slots gives ~6 ms headroom before
 * overflow under sustained load. */
#ifndef CO_STM32G4_RX_QUEUE_SIZE
#  define CO_STM32G4_RX_QUEUE_SIZE   64U
#endif

/* SPSC ring buffer populated by co_stm32g4_rx_isr() and drained by
 * co_stm32g4_drain_rx().  head is written only by the ISR; tail only by the
 * main loop — no mutex required on single-core Cortex-M4. */
typedef struct {
    co_can_frame_t      buf[CO_STM32G4_RX_QUEUE_SIZE];
    volatile uint32_t   head;   /* next write slot  (ISR)       */
    volatile uint32_t   tail;   /* next read  slot  (main loop) */
} co_stm32g4_rx_queue_t;

typedef struct {
    FDCAN_HandleTypeDef   *hfdcan;
    co_stm32g4_rx_queue_t  rx_queue;
} co_stm32g4_ctx_t;

co_error_t co_stm32g4_send(void *user, const co_can_frame_t *frame);
uint32_t   co_stm32g4_millis(void *user);

/* ISR entry point — call from HAL_FDCAN_RxFifo0Callback().
 * Drains the FDCAN RX FIFO0 into the ring buffer.  For heartbeat frames the
 * consumer timestamp is updated immediately so co_process() never fires a
 * false timeout due to main-loop latency. */
void co_stm32g4_rx_isr(co_node_t *node, co_stm32g4_ctx_t *ctx);

/* Main-loop drain — pops every queued frame and delivers it to co_on_can_rx()
 * so the full stack (SDO, PDO, NMT callbacks) runs in task context. */
void co_stm32g4_drain_rx(co_node_t *node, co_stm32g4_ctx_t *ctx);

/* Legacy polled path — kept for non-ISR setups. */
void co_stm32g4_poll_rx(co_node_t *node, FDCAN_HandleTypeDef *hfdcan);

/* Initialises the context, starts the FDCAN peripheral, activates the RX
 * FIFO0 new-message interrupt, and calls co_init(). */
void co_stm32g4_attach(co_node_t *node,
                       FDCAN_HandleTypeDef *hfdcan,
                       co_stm32g4_ctx_t *ctx,
                       uint8_t node_id,
                       uint16_t heartbeat_ms);

#endif /* CANOPEN_STM32G4_FDCAN_H */
