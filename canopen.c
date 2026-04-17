#include "canopen.h"

#include <string.h>

#define CO_COB_NMT 0x000U
#define CO_COB_SYNC 0x080U
#define CO_COB_EMCY_BASE 0x080U
#define CO_COB_TPDO1_BASE 0x180U
#define CO_COB_RPDO1_BASE 0x200U
#define CO_COB_TPDO2_BASE 0x280U
#define CO_COB_RPDO2_BASE 0x300U
#define CO_COB_TPDO3_BASE 0x380U
#define CO_COB_RPDO3_BASE 0x400U
#define CO_COB_TPDO4_BASE 0x480U
#define CO_COB_RPDO4_BASE 0x500U
#define CO_COB_SDO_TX_BASE 0x580U
#define CO_COB_SDO_RX_BASE 0x600U
#define CO_COB_HEARTBEAT_BASE 0x700U

static co_od_entry_t *co_od_find(co_node_t *node, uint16_t index, uint8_t subindex)
{
    for (size_t i = 0; i < node->od_count; ++i) {
        if (node->od[i].index == index && node->od[i].subindex == subindex) {
            return &node->od[i];
        }
    }
    return NULL;
}

static co_error_t co_send_heartbeat(co_node_t *node)
{
    co_can_frame_t frame = {
        .cob_id = CO_COB_HEARTBEAT_BASE + node->node_id,
        .len = 1U,
        .data = { (uint8_t)node->nmt_state }
    };
    return node->iface.send(node->iface.user, &frame);
}

static co_error_t co_send_sdo_abort(co_node_t *node, uint16_t index, uint8_t subindex, uint32_t abort_code)
{
    co_can_frame_t tx = {0};
    tx.cob_id = CO_COB_SDO_TX_BASE + node->node_id;
    tx.len = 8;
    tx.data[0] = 0x80;
    tx.data[1] = (uint8_t)(index & 0xFFU);
    tx.data[2] = (uint8_t)(index >> 8U);
    tx.data[3] = subindex;
    tx.data[4] = (uint8_t)(abort_code & 0xFFU);
    tx.data[5] = (uint8_t)((abort_code >> 8U) & 0xFFU);
    tx.data[6] = (uint8_t)((abort_code >> 16U) & 0xFFU);
    tx.data[7] = (uint8_t)((abort_code >> 24U) & 0xFFU);
    return node->iface.send(node->iface.user, &tx);
}

static co_error_t co_process_sdo(co_node_t *node, const co_can_frame_t *frame)
{
    if (frame->len != 8U) {
        return co_send_sdo_abort(node, 0, 0, 0x05040001UL);
    }

    const uint8_t cmd = frame->data[0] & 0xE0U;
    const uint16_t index = (uint16_t)frame->data[1] | ((uint16_t)frame->data[2] << 8U);
    const uint8_t subindex = frame->data[3];
    co_od_entry_t *entry = co_od_find(node, index, subindex);

    if (!entry) {
        return co_send_sdo_abort(node, index, subindex, 0x06020000UL);
    }

    co_can_frame_t tx = {0};
    tx.cob_id = CO_COB_SDO_TX_BASE + node->node_id;
    tx.len = 8;

    if (cmd == 0x40U) {
        if (!entry->readable) {
            return co_send_sdo_abort(node, index, subindex, 0x06010001UL);
        }

        const uint8_t n = (uint8_t)(4U - entry->size);
        tx.data[0] = (uint8_t)(0x43U | (n << 2U));
        tx.data[1] = frame->data[1];
        tx.data[2] = frame->data[2];
        tx.data[3] = frame->data[3];
        memset(&tx.data[4], 0, 4);
        memcpy(&tx.data[4], entry->data, entry->size);
        return node->iface.send(node->iface.user, &tx);
    }

    if (cmd == 0x20U) {
        if (!entry->writable) {
            return co_send_sdo_abort(node, index, subindex, 0x06010002UL);
        }

        const uint8_t n = (frame->data[0] >> 2U) & 0x03U;
        const uint8_t payload_size = (uint8_t)(4U - n);

        if (!(frame->data[0] & 0x02U) || !(frame->data[0] & 0x01U)) {
            return co_send_sdo_abort(node, index, subindex, 0x06070010UL);
        }

        if (payload_size != entry->size) {
            return co_send_sdo_abort(node, index, subindex, 0x06070010UL);
        }

        memcpy(entry->data, &frame->data[4], entry->size);

        tx.data[0] = 0x60U;
        tx.data[1] = frame->data[1];
        tx.data[2] = frame->data[2];
        tx.data[3] = frame->data[3];
        return node->iface.send(node->iface.user, &tx);
    }

    return co_send_sdo_abort(node, index, subindex, 0x05040001UL);
}

static co_error_t co_process_nmt(co_node_t *node, const co_can_frame_t *frame)
{
    if (frame->len < 2U) {
        return CO_ERROR_INVALID_ARGS;
    }

    if (frame->data[1] != 0U && frame->data[1] != node->node_id) {
        return CO_ERROR_NONE;
    }

    switch (frame->data[0]) {
        case 0x01:
            node->nmt_state = CO_NMT_OPERATIONAL;
            break;
        case 0x02:
            node->nmt_state = CO_NMT_STOPPED;
            break;
        case 0x80:
            node->nmt_state = CO_NMT_PRE_OPERATIONAL;
            break;
        case 0x81:
        case 0x82:
            node->nmt_state = CO_NMT_INITIALIZING;
            break;
        default:
            return CO_ERROR_INVALID_ARGS;
    }
    return CO_ERROR_NONE;
}

void co_init(co_node_t *node, const co_if_t *iface, uint8_t node_id, uint16_t heartbeat_ms)
{
    memset(node, 0, sizeof(*node));
    node->iface = *iface;
    node->node_id = node_id;
    node->heartbeat_ms = heartbeat_ms;
    node->nmt_state = CO_NMT_PRE_OPERATIONAL;

    node->rpdo_map[0] = CO_COB_RPDO1_BASE + node_id;
    node->rpdo_map[1] = CO_COB_RPDO2_BASE + node_id;
    node->rpdo_map[2] = CO_COB_RPDO3_BASE + node_id;
    node->rpdo_map[3] = CO_COB_RPDO4_BASE + node_id;
    node->tpdo_map[0] = CO_COB_TPDO1_BASE + node_id;
    node->tpdo_map[1] = CO_COB_TPDO2_BASE + node_id;
    node->tpdo_map[2] = CO_COB_TPDO3_BASE + node_id;
    node->tpdo_map[3] = CO_COB_TPDO4_BASE + node_id;
}

co_error_t co_od_add(co_node_t *node,
                     uint16_t index,
                     uint8_t subindex,
                     uint8_t *data,
                     uint8_t size,
                     bool readable,
                     bool writable)
{
    if (!node || !data || size == 0U || size > 4U) {
        return CO_ERROR_INVALID_ARGS;
    }

    if (node->od_count >= CO_MAX_OD_ENTRIES) {
        return CO_ERROR_OD_FULL;
    }

    co_od_entry_t *e = &node->od[node->od_count++];
    e->index = index;
    e->subindex = subindex;
    e->data = data;
    e->size = size;
    e->readable = readable;
    e->writable = writable;
    return CO_ERROR_NONE;
}

co_error_t co_process(co_node_t *node)
{
    if (!node || !node->iface.millis || !node->iface.send) {
        return CO_ERROR_INVALID_ARGS;
    }

    if (node->heartbeat_ms == 0U) {
        return CO_ERROR_NONE;
    }

    const uint32_t now = node->iface.millis(node->iface.user);
    if ((now - node->last_heartbeat_ms) >= node->heartbeat_ms) {
        node->last_heartbeat_ms = now;
        return co_send_heartbeat(node);
    }
    return CO_ERROR_NONE;
}

co_error_t co_on_can_rx(co_node_t *node, const co_can_frame_t *frame)
{
    if (!node || !frame) {
        return CO_ERROR_INVALID_ARGS;
    }

    if (frame->cob_id == CO_COB_NMT) {
        return co_process_nmt(node, frame);
    }

    if (frame->cob_id == (CO_COB_SDO_RX_BASE + node->node_id)) {
        return co_process_sdo(node, frame);
    }

    for (uint8_t i = 0; i < CO_MAX_RPDO; ++i) {
        if (frame->cob_id == node->rpdo_map[i]) {
            node->rpdo_data[i].len = frame->len;
            memcpy(node->rpdo_data[i].data, frame->data, frame->len);
            return CO_ERROR_NONE;
        }
    }

    return CO_ERROR_NONE;
}

co_error_t co_send_tpdo(co_node_t *node, uint8_t tpdo_num)
{
    if (!node || tpdo_num >= CO_MAX_TPDO) {
        return CO_ERROR_INVALID_ARGS;
    }

    co_can_frame_t frame = {
        .cob_id = node->tpdo_map[tpdo_num],
        .len = node->tpdo_data[tpdo_num].len
    };

    memcpy(frame.data, node->tpdo_data[tpdo_num].data, frame.len);
    return node->iface.send(node->iface.user, &frame);
}
