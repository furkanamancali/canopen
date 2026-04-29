#ifndef CANOPEN_STM32F4_CAN_H
#define CANOPEN_STM32F4_CAN_H

#include "canopen.h"
#include "stm32f4xx_hal.h"

/* ISR basina islenen maksimum cerceve sayisi (yoklama yolu). */
#define CO_STM32F4_RX_MAX_PER_POLL 32U

/* ISR→ana-dongu cerceve kuyrugu kapasitesi. Iki'nin kuvveti olmalidir.
 * 1 ms SYNC'te yaklasik ~10 cerceve/ms ile 64 yuva, asiri yuklemede
 * ~6 ms bosluk saglar. */
#define CO_STM32F4_RX_QUEUE_SIZE   64U

/* ISR tarafindan doldurulan, ana dongu tarafindan bosaltilan SPSC halka tamponu.
 * head yalnizca ISR tarafindan; tail yalnizca ana dongu tarafindan yazilir —
 * tek cekirdekli Cortex-M'de mutex gerekmez. */
typedef struct {
    co_can_frame_t    buf[CO_STM32F4_RX_QUEUE_SIZE];
    volatile uint32_t head;   /* sonraki yazma yuvasi (ISR)      */
    volatile uint32_t tail;   /* sonraki okuma yuvasi (ana dongu) */
} co_stm32f4_rx_queue_t;

typedef struct {
    CAN_HandleTypeDef      *hcan;
    co_stm32f4_rx_queue_t   rx_queue;
} co_stm32f4_ctx_t;

co_error_t co_stm32f4_send(void *user, const co_can_frame_t *frame);
uint32_t   co_stm32f4_millis(void *user);

/* ISR giris noktasi — HAL_CAN_RxFifo0MsgPendingCallback() icerisinden cagrilir.
 * bxCAN RX FIFO0'i halka tamponuna bosaltir. Kalp atisi cerceleri icin
 * tuketici zaman damgasi aninda guncellenir; boylece co_process() ana dongu
 * gecikmesi nedeniyle yanlis zaman asimi bildirmez. */
void co_stm32f4_rx_isr(co_node_t *node, co_stm32f4_ctx_t *ctx);

/* Ana dongu bosaltma — kuyruklanmis her cerceve co_on_can_rx()'e iletilir;
 * tam CANopen stack islemi (SDO, PDO, NMT geri cagirmalari) gorev baglaminda calisir. */
void co_stm32f4_drain_rx(co_node_t *node, co_stm32f4_ctx_t *ctx);

/* CAN cevrebirimini baslatir, RX filtresini tum standart cerceveleri kabul edecek
 * sekilde yapilandirir ve CANopen stack'i baslatir. */
void co_stm32f4_attach(co_node_t *node,
                        CAN_HandleTypeDef *hcan,
                        co_stm32f4_ctx_t *ctx,
                        uint8_t node_id,
                        uint16_t heartbeat_ms);

#endif /* CANOPEN_STM32F4_CAN_H */
