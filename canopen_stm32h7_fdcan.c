#include "canopen_stm32h7_fdcan.h"

#define CO_STM32_RX_FIFO FDCAN_RX_FIFO0

co_error_t co_stm32_send(void *user, const co_can_frame_t *frame)
{
    co_stm32_ctx_t *ctx = (co_stm32_ctx_t *)user;
    FDCAN_TxHeaderTypeDef tx_header;

    tx_header.Identifier = frame->cob_id;
    tx_header.IdType = FDCAN_STANDARD_ID;
    tx_header.TxFrameType = FDCAN_DATA_FRAME;
    tx_header.DataLength = (uint32_t)(frame->len << 16U);
    tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx_header.BitRateSwitch = FDCAN_BRS_OFF;
    tx_header.FDFormat = FDCAN_CLASSIC_CAN;
    tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    tx_header.MessageMarker = 0;

    if (HAL_FDCAN_AddMessageToTxFifoQ(ctx->hfdcan, &tx_header, (uint8_t *)frame->data) != HAL_OK) {
        return CO_ERROR_HW;
    }

    return CO_ERROR_NONE;
}

uint32_t co_stm32_millis(void *user)
{
    (void)user;
    return HAL_GetTick();
}

void co_stm32_poll_rx(co_node_t *node, FDCAN_HandleTypeDef *hfdcan)
{
    FDCAN_RxHeaderTypeDef rx_header;
    uint8_t data[8];

    while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, CO_STM32_RX_FIFO) > 0U) {
        if (HAL_FDCAN_GetRxMessage(hfdcan, CO_STM32_RX_FIFO, &rx_header, data) != HAL_OK) {
            break;
        }

        if (rx_header.IdType != FDCAN_STANDARD_ID || rx_header.RxFrameType != FDCAN_DATA_FRAME) {
            continue;
        }

        co_can_frame_t frame;
        frame.cob_id = rx_header.Identifier;
        frame.len = (uint8_t)(rx_header.DataLength >> 16U);
        for (uint8_t i = 0; i < frame.len; ++i) {
            frame.data[i] = data[i];
        }

        (void)co_on_can_rx(node, &frame);
    }
}

void co_stm32_attach(co_node_t *node,
                     FDCAN_HandleTypeDef *hfdcan,
                     co_stm32_ctx_t *ctx,
                     uint8_t node_id,
                     uint16_t heartbeat_ms)
{
    co_if_t iface = {0};
    ctx->hfdcan = hfdcan;

    iface.user = ctx;
    iface.send = co_stm32_send;
    iface.millis = co_stm32_millis;

    co_init(node, &iface, node_id, heartbeat_ms);
}
