#include "canopen_stm32f4_can.h"

#define CO_STM32F4_RX_FIFO CAN_RX_FIFO0

co_error_t co_stm32f4_send(void *user, const co_can_frame_t *frame)
{
    co_stm32f4_ctx_t *ctx = (co_stm32f4_ctx_t *)user;
    CAN_TxHeaderTypeDef tx_header;
    uint32_t tx_mailbox;

    tx_header.StdId              = frame->cob_id;
    tx_header.ExtId              = 0U;
    tx_header.IDE                = CAN_ID_STD;
    tx_header.RTR                = CAN_RTR_DATA;
    tx_header.DLC                = frame->len;
    tx_header.TransmitGlobalTime = DISABLE;

    if (HAL_CAN_AddTxMessage(ctx->hcan, &tx_header,
                              (uint8_t *)frame->data, &tx_mailbox) != HAL_OK) {
        return CO_ERROR_HW;
    }
    return CO_ERROR_NONE;
}

uint32_t co_stm32f4_millis(void *user)
{
    (void)user;
    return HAL_GetTick();
}

/* ── SPSC halka tamponu yardimcilari ─────────────────────────────────────────
 * co_stm32f4_rx_isr() yalnizca head'i; co_stm32f4_drain_rx() yalnizca tail'i
 * yazar. __DMB() engelleri, uretici ve tuketici tarafinda indeks guncellemesinden
 * once cerceve verisinin gorununur olmesini saglar (Cortex-M4 sirali islem hattinda
 * gereklidir). */

static bool co_stm32f4_rx_enqueue(co_stm32f4_rx_queue_t *q, const co_can_frame_t *f)
{
    uint32_t h    = q->head;
    uint32_t next = (h + 1U) & (CO_STM32F4_RX_QUEUE_SIZE - 1U);
    if (next == q->tail) {
        return false;   /* dolu — cerceve dusuruldu */
    }
    q->buf[h] = *f;
    __DMB();
    q->head = next;
    return true;
}

static bool co_stm32f4_rx_dequeue(co_stm32f4_rx_queue_t *q, co_can_frame_t *f)
{
    uint32_t t = q->tail;
    if (t == q->head) {
        return false;   /* bos */
    }
    *f = q->buf[t];
    __DMB();
    q->tail = (t + 1U) & (CO_STM32F4_RX_QUEUE_SIZE - 1U);
    return true;
}

/* ── ISR giris noktasi ───────────────────────────────────────────────────────
 * HAL_CAN_RxFifo0MsgPendingCallback() icerisinden cagrilir.
 *
 * Kalp atisi cerceveleri (COB-ID 0x701–0x77F) icin eslesен tuketicinin
 * last_rx_ms degeri kuyruga alinmadan once guncellenir. Bu, ana dongudeki
 * islem gecikmesi nedeniyle co_process()'in yanlis zaman asimi bildirmesini onler.
 * Diger tum stack islemleri co_stm32f4_drain_rx() araciligiyla ana dongu
 * baglamina ertelenir. */
void co_stm32f4_rx_isr(co_node_t *node, co_stm32f4_ctx_t *ctx)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t data[8];

    while (HAL_CAN_GetRxFifoFillLevel(ctx->hcan, CO_STM32F4_RX_FIFO) > 0U) {
        if (HAL_CAN_GetRxMessage(ctx->hcan, CO_STM32F4_RX_FIFO,
                                  &rx_header, data) != HAL_OK) {
            break;
        }
        if (rx_header.IDE != CAN_ID_STD || rx_header.RTR != CAN_RTR_DATA) {
            continue;
        }
        if (rx_header.DLC > 8U) {
            continue;
        }

        co_can_frame_t frame;
        frame.cob_id = rx_header.StdId;
        frame.len    = (uint8_t)rx_header.DLC;
        for (uint8_t i = 0U; i < frame.len; ++i) {
            frame.data[i] = data[i];
        }

        /* Kalp atisi hizli yolu: tuketici zaman damgasini guncelle. */
        if (frame.cob_id >= 0x701U && frame.cob_id <= 0x77FU && frame.len >= 1U) {
            const uint8_t hb_node = (uint8_t)(frame.cob_id - 0x700U);
            const uint32_t now    = HAL_GetTick();
            for (uint8_t i = 0U; i < CO_HEARTBEAT_CONSUMERS_MAX; ++i) {
                if (node->hb_consumers[i].enabled &&
                    node->hb_consumers[i].node_id == hb_node) {
                    node->hb_runtime[i].last_rx_ms = now;
                    break;
                }
            }
        }

        (void)co_stm32f4_rx_enqueue(&ctx->rx_queue, &frame);
    }
}

/* ── Ana dongu bosaltma ───────────────────────────────────────────────────── */
void co_stm32f4_drain_rx(co_node_t *node, co_stm32f4_ctx_t *ctx)
{
    co_can_frame_t frame;
    while (co_stm32f4_rx_dequeue(&ctx->rx_queue, &frame)) {
        (void)co_on_can_rx(node, &frame);
    }
}

/* ── Baglama ─────────────────────────────────────────────────────────────── */
void co_stm32f4_attach(co_node_t *node,
                        CAN_HandleTypeDef *hcan,
                        co_stm32f4_ctx_t *ctx,
                        uint8_t node_id,
                        uint16_t heartbeat_ms)
{
    co_if_t iface = {0};
    ctx->hcan = hcan;

    iface.user   = ctx;
    iface.send   = co_stm32f4_send;
    iface.millis = co_stm32f4_millis;

    /* Tum standart cerceveleri FIFO0'a yonlendiren gecirmez filtre.
     * FilterIdHigh/Low = 0, FilterMaskIdHigh/Low = 0 → ID maskesi her bitle eslenir. */
    CAN_FilterTypeDef filter = {0};
    filter.FilterBank           = 0U;
    filter.FilterMode           = CAN_FILTERMODE_IDMASK;
    filter.FilterScale          = CAN_FILTERSCALE_32BIT;
    filter.FilterIdHigh         = 0x0000U;
    filter.FilterIdLow          = 0x0000U;
    filter.FilterMaskIdHigh     = 0x0000U;
    filter.FilterMaskIdLow      = 0x0000U;
    filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    filter.FilterActivation     = ENABLE;
    (void)HAL_CAN_ConfigFilter(hcan, &filter);

    /* Cevrebirimi baslat ve FIFO0 bekleyen mesaj kesmesini etkinlestir. */
    (void)HAL_CAN_Start(hcan);
    (void)HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);

    co_init(node, &iface, node_id, heartbeat_ms);
}
