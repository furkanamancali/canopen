#ifndef CANOPEN_STM32H7_FDCAN_H
#define CANOPEN_STM32H7_FDCAN_H

#include "canopen.h"
#include "stm32h7xx_hal.h"

/* Maximum frames processed per co_stm32_poll_rx() call.
 * Caps the worst-case execution time of a single poll under bus overload. */
#define CO_STM32_RX_MAX_PER_POLL 32U

typedef struct {
    FDCAN_HandleTypeDef *hfdcan;
} co_stm32_ctx_t;

co_error_t co_stm32_send(void *user, const co_can_frame_t *frame);
uint32_t co_stm32_millis(void *user);
void co_stm32_poll_rx(co_node_t *node, FDCAN_HandleTypeDef *hfdcan);
void co_stm32_attach(co_node_t *node,
                     FDCAN_HandleTypeDef *hfdcan,
                     co_stm32_ctx_t *ctx,
                     uint8_t node_id,
                     uint16_t heartbeat_ms);

#endif
