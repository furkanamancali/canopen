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
 *   co_port_handle_t                            — HAL peripheral handle pointer type
 *   co_port_ctx_t                               — opaque driver context type
 *   co_port_attach(node, handle, ctx, id, hb)   — init CAN peripheral + CANopen stack
 *   co_port_drain_rx(node, ctx)                 — main-loop RX drain
 *   co_port_rx_isr(node, ctx)                   — ISR entry point
 *   CO_PORT_DEFINE_RX_ISR(buses_, count_)        — emit HAL RX callback dispatching
 *                                                  over an array of bus contexts
 *
 * Multi-bus usage
 * ───────────────
 * Each entry in cia402_nodes[] carries a co_port_handle_t that identifies the
 * CAN peripheral the slave is wired to.  The application creates one co_bus_t
 * per unique handle; CO_PORT_DEFINE_RX_ISR iterates that array so a single HAL
 * callback covers all active peripherals.
 *
 * Example — two FDCAN peripherals (STM32H7):
 *   extern FDCAN_HandleTypeDef hfdcan1, hfdcan2;
 *   const cia402_cfg_t cia402_nodes[] = {
 *       { 0x01U, &hfdcan1, ... },   // slave 1 on FDCAN1
 *       { 0x02U, &hfdcan2, ... },   // slave 2 on FDCAN2
 *   };
 *   CO_PORT_DEFINE_RX_ISR(s_buses, s_bus_count)   // in example_embedded.c
 */

#if defined(STM32H723xx)
/* ── STM32H7 — FDCAN ────────────────────────────────────────────────────── */
#  include "canopen_stm32h7_fdcan.h"

   typedef FDCAN_HandleTypeDef *co_port_handle_t;
   typedef co_stm32_ctx_t       co_port_ctx_t;

   static inline void co_port_attach(co_node_t *node, co_port_handle_t handle,
                                     co_port_ctx_t *ctx,
                                     uint8_t node_id, uint16_t heartbeat_ms)
   {
       co_stm32_attach(node, handle, ctx, node_id, heartbeat_ms);
   }

   static inline void co_port_drain_rx(co_node_t *node, co_port_ctx_t *ctx)
   {
       co_stm32_drain_rx(node, ctx);
   }

   static inline void co_port_rx_isr(co_node_t *node, co_port_ctx_t *ctx)
   {
       co_stm32_rx_isr(node, ctx);
   }

   /* Emits the FDCAN RX FIFO0 HAL callback.  Iterates buses_[0..count_-1] and
    * dispatches to the context whose handle matches the interrupting peripheral.
    * buses_ must be an array of a struct that has .handle, .node, .ctx members
    * (i.e. co_bus_t defined in example_embedded.c). */
#  define CO_PORT_DEFINE_RX_ISR(buses_, count_)                              \
   void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan,              \
                                   uint32_t RxFifo0ITs)                     \
   {                                                                         \
       (void)RxFifo0ITs;                                                     \
       for (uint8_t _b = 0U; _b < (count_); ++_b) {                        \
           if (hfdcan == (buses_)[_b].handle) {                             \
               co_port_rx_isr(&(buses_)[_b].node, &(buses_)[_b].ctx);      \
               break;                                                        \
           }                                                                 \
       }                                                                     \
   }

#elif defined(STM32F423xx)
/* ── STM32F4 — bxCAN ───────────────────────────────────────────────────── */
#  include "canopen_stm32f4_can.h"

   typedef CAN_HandleTypeDef  *co_port_handle_t;
   typedef co_stm32f4_ctx_t    co_port_ctx_t;

   static inline void co_port_attach(co_node_t *node, co_port_handle_t handle,
                                     co_port_ctx_t *ctx,
                                     uint8_t node_id, uint16_t heartbeat_ms)
   {
       co_stm32f4_attach(node, handle, ctx, node_id, heartbeat_ms);
   }

   static inline void co_port_drain_rx(co_node_t *node, co_port_ctx_t *ctx)
   {
       co_stm32f4_drain_rx(node, ctx);
   }

   static inline void co_port_rx_isr(co_node_t *node, co_port_ctx_t *ctx)
   {
       co_stm32f4_rx_isr(node, ctx);
   }

   /* Emits the bxCAN RX FIFO0 HAL callback.  Same multi-bus dispatch pattern
    * as the FDCAN variant above. */
#  define CO_PORT_DEFINE_RX_ISR(buses_, count_)                              \
   void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)           \
   {                                                                          \
       for (uint8_t _b = 0U; _b < (count_); ++_b) {                         \
           if (hcan == (buses_)[_b].handle) {                                \
               co_port_rx_isr(&(buses_)[_b].node, &(buses_)[_b].ctx);       \
               break;                                                         \
           }                                                                  \
       }                                                                      \
   }

#else
#  error "canopen_port.h: unsupported MCU — define STM32H723xx or STM32F423xx"
#endif

#endif /* CANOPEN_PORT_H */
