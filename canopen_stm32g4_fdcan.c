#include "canopen_stm32g4_fdcan.h"

#define CO_STM32G4_RX_FIFO FDCAN_RX_FIFO0

/*
 * STM32G4 FDCAN HAL API is identical to STM32H7 FDCAN — same function names,
 * same header structs (FDCAN_TxHeaderTypeDef / FDCAN_RxHeaderTypeDef), same
 * DLC constants, same ISR callback name (HAL_FDCAN_RxFifo0Callback).
 * The only build-time difference is the device HAL include (stm32g4xx_hal.h).
 *
 * G4 FDCAN message RAM is smaller than H7 (up to 10 KB on G474 vs 10 KB on
 * H723), but the classic CAN 2.0 frame path used by CANopen fits comfortably
 * in even the smallest G4 message RAM configuration.  Filter setup is left to
 * CubeMX-generated code; co_stm32g4_attach() only starts the peripheral and
 * activates the FIFO0 new-message notification.
 */

/* ── DLC encode/decode ───────────────────────────────────────────────────── */

static uint32_t co_stm32g4_encode_dlc(uint8_t len)
{
    switch (len) {
        case 0:  return FDCAN_DLC_BYTES_0;
        case 1:  return FDCAN_DLC_BYTES_1;
        case 2:  return FDCAN_DLC_BYTES_2;
        case 3:  return FDCAN_DLC_BYTES_3;
        case 4:  return FDCAN_DLC_BYTES_4;
        case 5:  return FDCAN_DLC_BYTES_5;
        case 6:  return FDCAN_DLC_BYTES_6;
        case 7:  return FDCAN_DLC_BYTES_7;
        case 8:
        default: return FDCAN_DLC_BYTES_8;
    }
}

static uint8_t co_stm32g4_decode_dlc(uint32_t dlc)
{
    switch (dlc) {
        case FDCAN_DLC_BYTES_0: return 0U;
        case FDCAN_DLC_BYTES_1: return 1U;
        case FDCAN_DLC_BYTES_2: return 2U;
        case FDCAN_DLC_BYTES_3: return 3U;
        case FDCAN_DLC_BYTES_4: return 4U;
        case FDCAN_DLC_BYTES_5: return 5U;
        case FDCAN_DLC_BYTES_6: return 6U;
        case FDCAN_DLC_BYTES_7: return 7U;
        case FDCAN_DLC_BYTES_8:
        default:                return 8U;
    }
}

/* ── TX ─────────────────────────────────────────────────────────────────── */

co_error_t co_stm32g4_send(void *user, const co_can_frame_t *frame)
{
    co_stm32g4_ctx_t *ctx = (co_stm32g4_ctx_t *)user;
    FDCAN_TxHeaderTypeDef tx_header;

    tx_header.Identifier          = frame->cob_id;
    tx_header.IdType              = FDCAN_STANDARD_ID;
    tx_header.TxFrameType         = FDCAN_DATA_FRAME;
    tx_header.DataLength          = co_stm32g4_encode_dlc(frame->len);
    tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx_header.BitRateSwitch       = FDCAN_BRS_OFF;
    tx_header.FDFormat            = FDCAN_CLASSIC_CAN;
    tx_header.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
    tx_header.MessageMarker       = 0U;

    if (HAL_FDCAN_AddMessageToTxFifoQ(ctx->hfdcan, &tx_header,
                                       (uint8_t *)frame->data) != HAL_OK) {
        return CO_ERROR_HW;
    }
    return CO_ERROR_NONE;
}

uint32_t co_stm32g4_millis(void *user)
{
    (void)user;
    return HAL_GetTick();
}

/* ── SPSC ring buffer helpers ────────────────────────────────────────────── */
/* Only co_stm32g4_rx_isr() writes head; only co_stm32g4_drain_rx() writes
 * tail.  __DMB() ensures frame data is visible before the index update on
 * both producer and consumer sides. */

static bool co_stm32g4_rx_enqueue(co_stm32g4_rx_queue_t *q,
                                   const co_can_frame_t  *f)
{
    uint32_t h    = q->head;
    uint32_t next = (h + 1U) & (CO_STM32G4_RX_QUEUE_SIZE - 1U);
    if (next == q->tail) {
        return false;   /* full — drop frame */
    }
    q->buf[h] = *f;
    __DMB();
    q->head = next;
    return true;
}

static bool co_stm32g4_rx_dequeue(co_stm32g4_rx_queue_t *q,
                                   co_can_frame_t        *f)
{
    uint32_t t = q->tail;
    if (t == q->head) {
        return false;   /* empty */
    }
    *f = q->buf[t];
    __DMB();
    q->tail = (t + 1U) & (CO_STM32G4_RX_QUEUE_SIZE - 1U);
    return true;
}

/* ── ISR entry point ─────────────────────────────────────────────────────── */
/* Call from HAL_FDCAN_RxFifo0Callback() (the G4 callback name is identical
 * to H7).  Drains FDCAN RX FIFO0 into the ring buffer.
 *
 * Heartbeat frames (COB-ID 0x701–0x77F) are handled in the fast path: the
 * matching consumer's last_rx_ms is updated before the frame is queued.
 * This prevents co_process() from reporting a false timeout when the main
 * loop is momentarily delayed.  All other stack processing is deferred to
 * co_stm32g4_drain_rx() running in task context. */
void co_stm32g4_rx_isr(co_node_t *node, co_stm32g4_ctx_t *ctx)
{
    FDCAN_RxHeaderTypeDef rx_header;
    uint8_t data[8];

    while (HAL_FDCAN_GetRxFifoFillLevel(ctx->hfdcan, CO_STM32G4_RX_FIFO) > 0U) {
        if (HAL_FDCAN_GetRxMessage(ctx->hfdcan, CO_STM32G4_RX_FIFO,
                                    &rx_header, data) != HAL_OK) {
            break;
        }
        if (rx_header.IdType      != FDCAN_STANDARD_ID ||
            rx_header.RxFrameType != FDCAN_DATA_FRAME) {
            continue;
        }
        const uint8_t len = co_stm32g4_decode_dlc(rx_header.DataLength);
        if (len > 8U) {
            continue;
        }

        co_can_frame_t frame;
        frame.cob_id = rx_header.Identifier;
        frame.len    = len;
        for (uint8_t i = 0U; i < len; ++i) {
            frame.data[i] = data[i];
        }

        /* Fast-path heartbeat: keep consumer timestamp current. */
        if (frame.cob_id >= 0x701U && frame.cob_id <= 0x77FU && len >= 1U) {
            const uint8_t  hb_node = (uint8_t)(frame.cob_id - 0x700U);
            const uint32_t now     = HAL_GetTick();
            for (uint8_t i = 0U; i < CO_HEARTBEAT_CONSUMERS_MAX; ++i) {
                if (node->hb_consumers[i].enabled &&
                    node->hb_consumers[i].node_id == hb_node) {
                    node->hb_runtime[i].last_rx_ms = now;
                    break;
                }
            }
        }

        (void)co_stm32g4_rx_enqueue(&ctx->rx_queue, &frame);
    }
}

/* ── Main-loop drain ─────────────────────────────────────────────────────── */

void co_stm32g4_drain_rx(co_node_t *node, co_stm32g4_ctx_t *ctx)
{
    co_can_frame_t frame;
    while (co_stm32g4_rx_dequeue(&ctx->rx_queue, &frame)) {
        (void)co_on_can_rx(node, &frame);
    }
}

/* ── Legacy polled path ──────────────────────────────────────────────────── */

void co_stm32g4_poll_rx(co_node_t *node, FDCAN_HandleTypeDef *hfdcan)
{
    FDCAN_RxHeaderTypeDef rx_header;
    uint8_t data[8];
    uint32_t count = 0U;

    while (count < CO_STM32G4_RX_MAX_PER_POLL &&
           HAL_FDCAN_GetRxFifoFillLevel(hfdcan, CO_STM32G4_RX_FIFO) > 0U) {

        if (HAL_FDCAN_GetRxMessage(hfdcan, CO_STM32G4_RX_FIFO,
                                    &rx_header, data) != HAL_OK) {
            break;
        }
        if (rx_header.IdType      != FDCAN_STANDARD_ID ||
            rx_header.RxFrameType != FDCAN_DATA_FRAME) {
            ++count;
            continue;
        }
        const uint8_t len = co_stm32g4_decode_dlc(rx_header.DataLength);
        if (len > 8U) {
            ++count;
            continue;
        }

        co_can_frame_t frame;
        frame.cob_id = rx_header.Identifier;
        frame.len    = len;
        for (uint8_t i = 0U; i < len; ++i) {
            frame.data[i] = data[i];
        }

        (void)co_on_can_rx(node, &frame);
        ++count;
    }
}

/* ── Attach ──────────────────────────────────────────────────────────────── */
/* Filter configuration is left to CubeMX-generated code.  The attach step
 * only starts the peripheral (idempotent if already started) and activates
 * the FIFO0 new-message notification so co_stm32g4_rx_isr() is called by
 * HAL_FDCAN_RxFifo0Callback(). */
void co_stm32g4_attach(co_node_t *node,
                       FDCAN_HandleTypeDef *hfdcan,
                       co_stm32g4_ctx_t *ctx,
                       uint8_t node_id,
                       uint16_t heartbeat_ms)
{
    co_if_t iface = {0};
    ctx->hfdcan = hfdcan;

    iface.user   = ctx;
    iface.send   = co_stm32g4_send;
    iface.millis = co_stm32g4_millis;

    (void)HAL_FDCAN_Start(hfdcan);
    (void)HAL_FDCAN_ActivateNotification(hfdcan,
                                          FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0U);

    co_init(node, &iface, node_id, heartbeat_ms);
}
