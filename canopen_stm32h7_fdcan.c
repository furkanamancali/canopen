#include "canopen_stm32h7_fdcan.h"

#define CO_STM32_RX_FIFO FDCAN_RX_FIFO0

/*
 * Encode a byte count (0–8) to the HAL FDCAN DataLength constant.
 * The HAL defines FDCAN_DLC_BYTES_N = (N << 16) for classic CAN frames,
 * but using the named constants makes the intent explicit and guards against
 * future HAL encoding changes.
 */
static uint32_t co_stm32_encode_dlc(uint8_t len)
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

/*
 * Decode the HAL FDCAN DataLength constant to a byte count.
 * For classic CAN (DLC 0–8) the mapping is linear, but decoding via the
 * named constants is correct for all DLC values including FD extensions.
 */
static uint8_t co_stm32_decode_dlc(uint32_t dlc)
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

co_error_t co_stm32_send(void *user, const co_can_frame_t *frame)
{
    co_stm32_ctx_t *ctx = (co_stm32_ctx_t *)user;
    FDCAN_TxHeaderTypeDef tx_header;

    tx_header.Identifier            = frame->cob_id;
    tx_header.IdType                = FDCAN_STANDARD_ID;
    tx_header.TxFrameType           = FDCAN_DATA_FRAME;
    tx_header.DataLength            = co_stm32_encode_dlc(frame->len);
    tx_header.ErrorStateIndicator   = FDCAN_ESI_ACTIVE;
    tx_header.BitRateSwitch         = FDCAN_BRS_OFF;
    tx_header.FDFormat              = FDCAN_CLASSIC_CAN;
    tx_header.TxEventFifoControl    = FDCAN_NO_TX_EVENTS;
    tx_header.MessageMarker         = 0;

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
    uint32_t count = 0U;

    while (count < CO_STM32_RX_MAX_PER_POLL &&
           HAL_FDCAN_GetRxFifoFillLevel(hfdcan, CO_STM32_RX_FIFO) > 0U) {

        if (HAL_FDCAN_GetRxMessage(hfdcan, CO_STM32_RX_FIFO, &rx_header, data) != HAL_OK) {
            break;
        }

        if (rx_header.IdType != FDCAN_STANDARD_ID || rx_header.RxFrameType != FDCAN_DATA_FRAME) {
            ++count;
            continue;
        }

        const uint8_t len = co_stm32_decode_dlc(rx_header.DataLength);
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

void co_stm32_attach(co_node_t *node,
                     FDCAN_HandleTypeDef *hfdcan,
                     co_stm32_ctx_t *ctx,
                     uint8_t node_id,
                     uint16_t heartbeat_ms)
{
    co_if_t iface = {0};
    ctx->hfdcan = hfdcan;

    iface.user   = ctx;
    iface.send   = co_stm32_send;
    iface.millis = co_stm32_millis;

    /* Move the FDCAN peripheral from configuration mode to normal operation.
     * HAL_FDCAN_AddMessageToTxFifoQ returns HAL_ERROR if the peripheral has
     * not been started, which surfaces as CO_ERROR_HW on the first send.   */
    (void)HAL_FDCAN_Start(hfdcan);

    /* Activate RX FIFO0 so HAL_FDCAN_GetRxFifoFillLevel works correctly.  */
    (void)HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0U);

    co_init(node, &iface, node_id, heartbeat_ms);
}
