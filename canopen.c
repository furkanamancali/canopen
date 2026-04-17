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

static bool co_od_has_index(co_node_t *node, uint16_t index)
{
    for (size_t i = 0; i < node->od_count; ++i) {
        if (node->od[i].index == index) {
            return true;
        }
    }
    return false;
}

static co_error_t co_od_register_internal(co_node_t *node,
                                          uint16_t index,
                                          uint8_t subindex,
                                          uint8_t *data,
                                          size_t size,
                                          uint8_t access,
                                          co_od_read_cb_t on_read,
                                          co_od_write_cb_t on_write,
                                          void *user)
{
    if (!node || !data || size == 0U || (access & (CO_OD_ACCESS_READ | CO_OD_ACCESS_WRITE)) == 0U) {
        return CO_ERROR_INVALID_ARGS;
    }

    if (node->od_count >= CO_MAX_OD_ENTRIES) {
        return CO_ERROR_OD_FULL;
    }
    if (co_od_find(node, index, subindex) != NULL) {
        return CO_ERROR_INVALID_ARGS;
    }

    co_od_entry_t *e = &node->od[node->od_count++];
    e->index = index;
    e->subindex = subindex;
    e->size = size;
    e->data = data;
    e->access = access;
    e->on_read = on_read;
    e->on_write = on_write;
    e->user = user;
    return CO_ERROR_NONE;
}

static uint32_t co_od_abort_from_length(size_t provided, size_t expected)
{
    if (provided == expected) {
        return 0;
    }
    return (provided > expected) ? CO_SDO_ABORT_PARAM_TOO_HIGH : CO_SDO_ABORT_PARAM_TOO_LOW;
}

uint32_t co_od_read(co_node_t *node,
                    uint16_t index,
                    uint8_t subindex,
                    uint8_t *dst,
                    size_t dst_size,
                    size_t *out_size)
{
    if (!node || !dst || !out_size) {
        return CO_SDO_ABORT_COMMAND;
    }

    co_od_entry_t *entry = co_od_find(node, index, subindex);
    if (!entry) {
        return co_od_has_index(node, index) ? CO_SDO_ABORT_NO_SUBINDEX : CO_SDO_ABORT_NO_OBJECT;
    }

    if ((entry->access & CO_OD_ACCESS_READ) == 0U) {
        return CO_SDO_ABORT_WRITEONLY;
    }

    if (dst_size < entry->size) {
        return co_od_abort_from_length(dst_size, entry->size);
    }

    if (entry->on_read) {
        const uint32_t abort_code = entry->on_read(node, entry, entry->user);
        if (abort_code != 0U) {
            return abort_code;
        }
    }

    memcpy(dst, entry->data, entry->size);
    *out_size = entry->size;
    return 0;
}

uint32_t co_od_write(co_node_t *node,
                     uint16_t index,
                     uint8_t subindex,
                     const uint8_t *src,
                     size_t src_size)
{
    if (!node || !src) {
        return CO_SDO_ABORT_COMMAND;
    }

    co_od_entry_t *entry = co_od_find(node, index, subindex);
    if (!entry) {
        return co_od_has_index(node, index) ? CO_SDO_ABORT_NO_SUBINDEX : CO_SDO_ABORT_NO_OBJECT;
    }

    if ((entry->access & CO_OD_ACCESS_WRITE) == 0U) {
        return CO_SDO_ABORT_READONLY;
    }

    const uint32_t len_abort = co_od_abort_from_length(src_size, entry->size);
    if (len_abort != 0U) {
        return len_abort;
    }

    if (entry->on_write) {
        const uint32_t abort_code = entry->on_write(node, entry, src, src_size, entry->user);
        if (abort_code != 0U) {
            return abort_code;
        }
    }

    memcpy(entry->data, src, entry->size);
    return 0;
}

static uint32_t co_on_hb_write(co_node_t *node,
                               const co_od_entry_t *entry,
                               const uint8_t *data,
                               size_t size,
                               void *user)
{
    (void)entry;
    (void)user;
    if (size != sizeof(uint16_t)) {
        return CO_SDO_ABORT_PARAM_LENGTH;
    }

    uint16_t hb_ms = 0U;
    memcpy(&hb_ms, data, sizeof(hb_ms));
    node->heartbeat_ms = hb_ms;
    return 0;
}

static co_error_t co_register_builtin_od(co_node_t *node)
{
    co_error_t err = CO_ERROR_NONE;

    node->device_type = 0U;
    node->error_register = 0U;
    node->identity_sub_count = 4U;
    node->identity[0] = 0U;
    node->identity[1] = 0U;
    node->identity[2] = 0U;
    node->identity[3] = 0U;

    node->sdo_server_sub_count = 2U;
    node->sdo_server_cob_rx = CO_COB_SDO_RX_BASE + node->node_id;
    node->sdo_server_cob_tx = CO_COB_SDO_TX_BASE + node->node_id;

    for (uint8_t i = 0; i < CO_MAX_RPDO; ++i) {
        node->pdo_comm_sub_count[i] = 2U;
        node->pdo_map_count[i] = 0U;
        node->rpdo_transmission_type[i] = 255U;
    }

    for (uint8_t i = 0; i < CO_MAX_TPDO; ++i) {
        node->pdo_comm_sub_count[CO_MAX_RPDO + i] = 2U;
        node->pdo_map_count[CO_MAX_RPDO + i] = 0U;
        node->tpdo_transmission_type[i] = 255U;
    }

    err = co_od_register_internal(node,
                                  0x1000,
                                  0x00,
                                  (uint8_t *)&node->device_type,
                                  sizeof(node->device_type),
                                  CO_OD_ACCESS_READ,
                                  NULL,
                                  NULL,
                                  NULL);
    if (err != CO_ERROR_NONE) {
        return err;
    }

    err = co_od_register_internal(node,
                                  0x1001,
                                  0x00,
                                  &node->error_register,
                                  sizeof(node->error_register),
                                  CO_OD_ACCESS_READ,
                                  NULL,
                                  NULL,
                                  NULL);
    if (err != CO_ERROR_NONE) {
        return err;
    }

    err = co_od_register_internal(node,
                                  0x1017,
                                  0x00,
                                  (uint8_t *)&node->heartbeat_ms,
                                  sizeof(node->heartbeat_ms),
                                  CO_OD_ACCESS_READ | CO_OD_ACCESS_WRITE,
                                  NULL,
                                  co_on_hb_write,
                                  NULL);
    if (err != CO_ERROR_NONE) {
        return err;
    }

    err = co_od_register_internal(node,
                                  0x1018,
                                  0x00,
                                  &node->identity_sub_count,
                                  sizeof(node->identity_sub_count),
                                  CO_OD_ACCESS_READ,
                                  NULL,
                                  NULL,
                                  NULL);
    if (err != CO_ERROR_NONE) {
        return err;
    }

    for (uint8_t sub = 1; sub <= 4U; ++sub) {
        err = co_od_register_internal(node,
                                      0x1018,
                                      sub,
                                      (uint8_t *)&node->identity[sub - 1U],
                                      sizeof(uint32_t),
                                      CO_OD_ACCESS_READ,
                                      NULL,
                                      NULL,
                                      NULL);
        if (err != CO_ERROR_NONE) {
            return err;
        }
    }

    err = co_od_register_internal(node,
                                  0x1200,
                                  0x00,
                                  &node->sdo_server_sub_count,
                                  sizeof(node->sdo_server_sub_count),
                                  CO_OD_ACCESS_READ,
                                  NULL,
                                  NULL,
                                  NULL);
    if (err != CO_ERROR_NONE) {
        return err;
    }
    err = co_od_register_internal(node,
                                  0x1200,
                                  0x01,
                                  (uint8_t *)&node->sdo_server_cob_rx,
                                  sizeof(node->sdo_server_cob_rx),
                                  CO_OD_ACCESS_READ | CO_OD_ACCESS_WRITE,
                                  NULL,
                                  NULL,
                                  NULL);
    if (err != CO_ERROR_NONE) {
        return err;
    }
    err = co_od_register_internal(node,
                                  0x1200,
                                  0x02,
                                  (uint8_t *)&node->sdo_server_cob_tx,
                                  sizeof(node->sdo_server_cob_tx),
                                  CO_OD_ACCESS_READ | CO_OD_ACCESS_WRITE,
                                  NULL,
                                  NULL,
                                  NULL);
    if (err != CO_ERROR_NONE) {
        return err;
    }

    for (uint8_t i = 0; i < CO_MAX_RPDO; ++i) {
        const uint16_t comm_idx = (uint16_t)(0x1400U + i);
        const uint16_t map_idx = (uint16_t)(0x1600U + i);

        err = co_od_register_internal(node,
                                      comm_idx,
                                      0x00,
                                      &node->pdo_comm_sub_count[i],
                                      sizeof(node->pdo_comm_sub_count[i]),
                                      CO_OD_ACCESS_READ,
                                      NULL,
                                      NULL,
                                      NULL);
        if (err != CO_ERROR_NONE) {
            return err;
        }

        err = co_od_register_internal(node,
                                      comm_idx,
                                      0x01,
                                      (uint8_t *)&node->rpdo_map[i],
                                      sizeof(node->rpdo_map[i]),
                                      CO_OD_ACCESS_READ | CO_OD_ACCESS_WRITE,
                                      NULL,
                                      NULL,
                                      NULL);
        if (err != CO_ERROR_NONE) {
            return err;
        }

        err = co_od_register_internal(node,
                                      comm_idx,
                                      0x02,
                                      (uint8_t *)&node->rpdo_transmission_type[i],
                                      sizeof(node->rpdo_transmission_type[i]),
                                      CO_OD_ACCESS_READ | CO_OD_ACCESS_WRITE,
                                      NULL,
                                      NULL,
                                      NULL);
        if (err != CO_ERROR_NONE) {
            return err;
        }

        err = co_od_register_internal(node,
                                      map_idx,
                                      0x00,
                                      &node->pdo_map_count[i],
                                      sizeof(node->pdo_map_count[i]),
                                      CO_OD_ACCESS_READ | CO_OD_ACCESS_WRITE,
                                      NULL,
                                      NULL,
                                      NULL);
        if (err != CO_ERROR_NONE) {
            return err;
        }

        for (uint8_t sub = 1; sub <= CO_PDO_MAP_ENTRIES; ++sub) {
            err = co_od_register_internal(node,
                                          map_idx,
                                          sub,
                                          (uint8_t *)&node->rpdo_mapping[i][sub - 1U],
                                          sizeof(uint32_t),
                                          CO_OD_ACCESS_READ | CO_OD_ACCESS_WRITE,
                                          NULL,
                                          NULL,
                                          NULL);
            if (err != CO_ERROR_NONE) {
                return err;
            }
        }
    }

    for (uint8_t i = 0; i < CO_MAX_TPDO; ++i) {
        const uint16_t comm_idx = (uint16_t)(0x1800U + i);
        const uint16_t map_idx = (uint16_t)(0x1A00U + i);
        const uint8_t slot = (uint8_t)(CO_MAX_RPDO + i);

        err = co_od_register_internal(node,
                                      comm_idx,
                                      0x00,
                                      &node->pdo_comm_sub_count[slot],
                                      sizeof(node->pdo_comm_sub_count[slot]),
                                      CO_OD_ACCESS_READ,
                                      NULL,
                                      NULL,
                                      NULL);
        if (err != CO_ERROR_NONE) {
            return err;
        }

        err = co_od_register_internal(node,
                                      comm_idx,
                                      0x01,
                                      (uint8_t *)&node->tpdo_map[i],
                                      sizeof(node->tpdo_map[i]),
                                      CO_OD_ACCESS_READ | CO_OD_ACCESS_WRITE,
                                      NULL,
                                      NULL,
                                      NULL);
        if (err != CO_ERROR_NONE) {
            return err;
        }

        err = co_od_register_internal(node,
                                      comm_idx,
                                      0x02,
                                      (uint8_t *)&node->tpdo_transmission_type[i],
                                      sizeof(node->tpdo_transmission_type[i]),
                                      CO_OD_ACCESS_READ | CO_OD_ACCESS_WRITE,
                                      NULL,
                                      NULL,
                                      NULL);
        if (err != CO_ERROR_NONE) {
            return err;
        }

        err = co_od_register_internal(node,
                                      map_idx,
                                      0x00,
                                      &node->pdo_map_count[slot],
                                      sizeof(node->pdo_map_count[slot]),
                                      CO_OD_ACCESS_READ | CO_OD_ACCESS_WRITE,
                                      NULL,
                                      NULL,
                                      NULL);
        if (err != CO_ERROR_NONE) {
            return err;
        }

        for (uint8_t sub = 1; sub <= CO_PDO_MAP_ENTRIES; ++sub) {
            err = co_od_register_internal(node,
                                          map_idx,
                                          sub,
                                          (uint8_t *)&node->tpdo_mapping[i][sub - 1U],
                                          sizeof(uint32_t),
                                          CO_OD_ACCESS_READ | CO_OD_ACCESS_WRITE,
                                          NULL,
                                          NULL,
                                          NULL);
            if (err != CO_ERROR_NONE) {
                return err;
            }
        }
    }

    return CO_ERROR_NONE;
}

static co_error_t co_send_heartbeat(co_node_t *node)
{
    co_can_frame_t frame = {
        .cob_id = CO_COB_HEARTBEAT_BASE + node->node_id,
        .len = 1U,
        .data = {(uint8_t)node->nmt_state}
    };
    return node->iface.send(node->iface.user, &frame);
}

static co_error_t co_send_bootup(co_node_t *node)
{
    co_can_frame_t frame = {
        .cob_id = CO_COB_HEARTBEAT_BASE + node->node_id,
        .len = 1U,
        .data = {0x00U}
    };
    return node->iface.send(node->iface.user, &frame);
}

static void co_on_reset(co_node_t *node, co_reset_type_t type)
{
    node->nmt_state = CO_NMT_INITIALIZING;
    node->bootup_pending = true;
    node->last_heartbeat_ms = 0U;

    if (node->iface.on_reset) {
        node->iface.on_reset(node->iface.user, type);
    }
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
        return co_send_sdo_abort(node, 0, 0, CO_SDO_ABORT_COMMAND);
    }

    const uint8_t cmd = frame->data[0] & 0xE0U;
    const uint16_t index = (uint16_t)frame->data[1] | ((uint16_t)frame->data[2] << 8U);
    const uint8_t subindex = frame->data[3];

    co_can_frame_t tx = {0};
    tx.cob_id = CO_COB_SDO_TX_BASE + node->node_id;
    tx.len = 8;

    if (cmd == 0x40U) {
        uint8_t payload[4] = {0};
        size_t payload_size = 0U;
        const uint32_t abort_code = co_od_read(node, index, subindex, payload, sizeof(payload), &payload_size);
        if (abort_code != 0U) {
            return co_send_sdo_abort(node, index, subindex, abort_code);
        }

        const uint8_t n = (uint8_t)(4U - payload_size);
        tx.data[0] = (uint8_t)(0x43U | (n << 2U));
        tx.data[1] = frame->data[1];
        tx.data[2] = frame->data[2];
        tx.data[3] = frame->data[3];
        memcpy(&tx.data[4], payload, payload_size);
        return node->iface.send(node->iface.user, &tx);
    }

    if (cmd == 0x20U) {
        const bool expedited = (frame->data[0] & 0x02U) != 0U;
        const bool size_indicated = (frame->data[0] & 0x01U) != 0U;
        if (!expedited || !size_indicated) {
            return co_send_sdo_abort(node, index, subindex, CO_SDO_ABORT_PARAM_LENGTH);
        }

        const uint8_t n = (frame->data[0] >> 2U) & 0x03U;
        const size_t payload_size = (size_t)(4U - n);
        const uint32_t abort_code = co_od_write(node, index, subindex, &frame->data[4], payload_size);
        if (abort_code != 0U) {
            return co_send_sdo_abort(node, index, subindex, abort_code);
        }

        tx.data[0] = 0x60U;
        tx.data[1] = frame->data[1];
        tx.data[2] = frame->data[2];
        tx.data[3] = frame->data[3];
        return node->iface.send(node->iface.user, &tx);
    }

    return co_send_sdo_abort(node, index, subindex, CO_SDO_ABORT_COMMAND);
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
            co_on_reset(node, CO_RESET_COMMUNICATION);
            break;
        case 0x82:
            co_on_reset(node, CO_RESET_APPLICATION);
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
    node->nmt_state = CO_NMT_INITIALIZING;
    node->bootup_pending = true;

    node->rpdo_map[0] = CO_COB_RPDO1_BASE + node_id;
    node->rpdo_map[1] = CO_COB_RPDO2_BASE + node_id;
    node->rpdo_map[2] = CO_COB_RPDO3_BASE + node_id;
    node->rpdo_map[3] = CO_COB_RPDO4_BASE + node_id;
    node->tpdo_map[0] = CO_COB_TPDO1_BASE + node_id;
    node->tpdo_map[1] = CO_COB_TPDO2_BASE + node_id;
    node->tpdo_map[2] = CO_COB_TPDO3_BASE + node_id;
    node->tpdo_map[3] = CO_COB_TPDO4_BASE + node_id;

    (void)co_register_builtin_od(node);
}

co_error_t co_od_add(co_node_t *node,
                     uint16_t index,
                     uint8_t subindex,
                     uint8_t *data,
                     uint8_t size,
                     bool readable,
                     bool writable)
{
    uint8_t access = 0U;
    if (readable) {
        access |= CO_OD_ACCESS_READ;
    }
    if (writable) {
        access |= CO_OD_ACCESS_WRITE;
    }

    return co_od_register_internal(node, index, subindex, data, size, access, NULL, NULL, NULL);
}

co_error_t co_od_add_ex(co_node_t *node,
                        uint16_t index,
                        uint8_t subindex,
                        uint8_t *data,
                        size_t size,
                        uint8_t access,
                        co_od_read_cb_t on_read,
                        co_od_write_cb_t on_write,
                        void *user)
{
    return co_od_register_internal(node, index, subindex, data, size, access, on_read, on_write, user);
}

co_error_t co_process(co_node_t *node)
{
    if (!node || !node->iface.millis || !node->iface.send) {
        return CO_ERROR_INVALID_ARGS;
    }

    if (node->nmt_state == CO_NMT_INITIALIZING && node->bootup_pending) {
        const co_error_t err = co_send_bootup(node);
        if (err != CO_ERROR_NONE) {
            return err;
        }
        node->nmt_state = CO_NMT_PRE_OPERATIONAL;
        node->bootup_pending = false;
        node->last_heartbeat_ms = node->iface.millis(node->iface.user);
        return CO_ERROR_NONE;
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
