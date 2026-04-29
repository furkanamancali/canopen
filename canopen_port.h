#ifndef CANOPEN_PORT_H
#define CANOPEN_PORT_H

/*
 * canopen_port.h — MCU abstraction layer for the CANopen CAN driver.
 *
 * Centralises every MCU-specific choice so that example_embedded.c (and any
 * future application that uses the same stack) contains zero peripheral
 * #ifdefs.
 *
 * Supported targets (select by defining the MCU symbol before including):
 *   STM32H723xx  — FDCAN peripheral via canopen_stm32h7_fdcan
 *   STM32F423xx  — bxCAN peripheral via canopen_stm32f4_can
 *
 * Public surface exposed to the application:
 *   co_port_ctx_t                       — opaque driver context type
 *   co_port_attach(node, ctx, id, hb)   — init CAN peripheral + CANopen stack
 *   co_port_drain_rx(node, ctx)         — main-loop RX drain
 *   co_port_rx_isr(node, ctx)           — ISR entry point
 *   CO_PORT_DEFINE_RX_ISR(node_, ctx_)  — emit the HAL RX interrupt callback
 */

#if defined(STM32H723xx)
/* ── STM32H7 — FDCAN ────────────────────────────────────────────────────── */
#  include "canopen_stm32h7_fdcan.h"
   extern FDCAN_HandleTypeDef hfdcan1;

   typedef co_stm32_ctx_t co_port_ctx_t;

   static inline void co_port_attach(co_node_t *node, co_port_ctx_t *ctx,
                                     uint8_t node_id, uint16_t heartbeat_ms)
   {
       co_stm32_attach(node, &hfdcan1, ctx, node_id, heartbeat_ms);
   }

   static inline void co_port_drain_rx(co_node_t *node, co_port_ctx_t *ctx)
   {
       co_stm32_drain_rx(node, ctx);
   }

   static inline void co_port_rx_isr(co_node_t *node, co_port_ctx_t *ctx)
   {
       co_stm32_rx_isr(node, ctx);
   }

   /* Emits the FDCAN RX FIFO0 HAL callback.
    * Usage:  CO_PORT_DEFINE_RX_ISR(canopen_node, can_ctx) */
#  define CO_PORT_DEFINE_RX_ISR(node_, ctx_)                                \
   void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan,              \
                                   uint32_t RxFifo0ITs)                     \
   {                                                                         \
       (void)RxFifo0ITs;                                                     \
       if (hfdcan == &hfdcan1) { co_port_rx_isr(&(node_), &(ctx_)); }       \
   }

#elif defined(STM32F423xx)
/* ── STM32F4 — bxCAN ───────────────────────────────────────────────────── */
#  include "canopen_stm32f4_can.h"
   extern CAN_HandleTypeDef hcan1;

   typedef co_stm32f4_ctx_t co_port_ctx_t;

   static inline void co_port_attach(co_node_t *node, co_port_ctx_t *ctx,
                                     uint8_t node_id, uint16_t heartbeat_ms)
   {
       co_stm32f4_attach(node, &hcan1, ctx, node_id, heartbeat_ms);
   }

   static inline void co_port_drain_rx(co_node_t *node, co_port_ctx_t *ctx)
   {
       co_stm32f4_drain_rx(node, ctx);
   }

   static inline void co_port_rx_isr(co_node_t *node, co_port_ctx_t *ctx)
   {
       co_stm32f4_rx_isr(node, ctx);
   }

   /* Emits the bxCAN RX FIFO0 HAL callback.
    * Usage:  CO_PORT_DEFINE_RX_ISR(canopen_node, can_ctx) */
#  define CO_PORT_DEFINE_RX_ISR(node_, ctx_)                                \
   void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)          \
   {                                                                         \
       if (hcan == &hcan1) { co_port_rx_isr(&(node_), &(ctx_)); }           \
   }

#else
#  error "canopen_port.h: unsupported MCU — define STM32H723xx or STM32F423xx"
#endif

#endif /* CANOPEN_PORT_H */
