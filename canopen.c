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
#define CO_SDO_TIMEOUT_MS 1000U

typedef enum {
    CO_SDO_STATE_IDLE = 0,
    CO_SDO_STATE_SEG_DOWNLOAD,
    CO_SDO_STATE_SEG_UPLOAD,
    CO_SDO_STATE_BLK_DOWNLOAD,
    CO_SDO_STATE_BLK_DOWNLOAD_END,
    CO_SDO_STATE_BLK_UPLOAD,
    CO_SDO_STATE_BLK_UPLOAD_WAIT_ACK,
    CO_SDO_STATE_BLK_UPLOAD_END
} co_sdo_state_t;

typedef enum {
    CO_SDO_MODE_NONE = 0,
    CO_SDO_MODE_DOWNLOAD = 1,
    CO_SDO_MODE_UPLOAD = 2
} co_sdo_mode_t;

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

static void co_restore_default_comm_objects(co_node_t *node)
{
    node->sdo_server_cob_rx = CO_COB_SDO_RX_BASE + node->node_id;
    node->sdo_server_cob_tx = CO_COB_SDO_TX_BASE + node->node_id;

    node->rpdo_map[0] = CO_COB_RPDO1_BASE + node->node_id;
    node->rpdo_map[1] = CO_COB_RPDO2_BASE + node->node_id;
    node->rpdo_map[2] = CO_COB_RPDO3_BASE + node->node_id;
    node->rpdo_map[3] = CO_COB_RPDO4_BASE + node->node_id;
    node->tpdo_map[0] = CO_COB_TPDO1_BASE + node->node_id;
    node->tpdo_map[1] = CO_COB_TPDO2_BASE + node->node_id;
    node->tpdo_map[2] = CO_COB_TPDO3_BASE + node->node_id;
    node->tpdo_map[3] = CO_COB_TPDO4_BASE + node->node_id;
}

static void co_schedule_bootup(co_node_t *node)
{
    node->nmt_state = CO_NMT_INITIALIZING;
    node->bootup_pending = true;
    node->last_heartbeat_ms = 0U;
}

static void co_on_reset_communication(co_node_t *node)
{
    co_restore_default_comm_objects(node);
    co_schedule_bootup(node);

    if (node->iface.on_reset_communication) {
        node->iface.on_reset_communication(node->iface.user);
    }
}

static void co_on_reset_application(co_node_t *node)
{
    co_on_reset_communication(node);

    if (node->iface.on_reset_application) {
        node->iface.on_reset_application(node->iface.user);
    }

    if (node->iface.on_reset) {
        node->iface.on_reset(node->iface.user, CO_RESET_APPLICATION);
    }
}

static void co_on_reset(co_node_t *node, co_reset_type_t type)
{
    if (type == CO_RESET_APPLICATION) {
        co_on_reset_application(node);
        return;
    }

    co_on_reset_communication(node);
    if (node->iface.on_reset) {
        node->iface.on_reset(node->iface.user, CO_RESET_COMMUNICATION);
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

static void co_sdo_channel_reset(co_node_t *node, size_t ch)
{
    node->sdo_channels[ch].state = CO_SDO_STATE_IDLE;
    node->sdo_channels[ch].mode = CO_SDO_MODE_NONE;
    node->sdo_channels[ch].toggle = 0U;
    node->sdo_channels[ch].block_seq = 0U;
    node->sdo_channels[ch].block_last_sent_seq = 0U;
    node->sdo_channels[ch].block_blksize = 0U;
    node->sdo_channels[ch].block_finished = 0U;
    node->sdo_channels[ch].offset = 0U;
    node->sdo_channels[ch].size = 0U;
}

static uint32_t co_sdo_validate_request(co_node_t *node,
                                        uint16_t index,
                                        uint8_t subindex,
                                        bool is_write,
                                        size_t size_hint,
                                        co_od_entry_t **out_entry)
{
    co_od_entry_t *entry = co_od_find(node, index, subindex);
    if (!entry) {
        return co_od_has_index(node, index) ? CO_SDO_ABORT_NO_SUBINDEX : CO_SDO_ABORT_NO_OBJECT;
    }

    if (is_write && ((entry->access & CO_OD_ACCESS_WRITE) == 0U)) {
        return CO_SDO_ABORT_READONLY;
    }
    if (!is_write && ((entry->access & CO_OD_ACCESS_READ) == 0U)) {
        return CO_SDO_ABORT_WRITEONLY;
    }

    if (size_hint > 0U && size_hint != entry->size) {
        return co_od_abort_from_length(size_hint, entry->size);
    }

    if (out_entry) {
        *out_entry = entry;
    }
    return 0U;
}

static co_error_t co_sdo_abort_and_reset(co_node_t *node,
                                         size_t ch,
                                         uint16_t index,
                                         uint8_t subindex,
                                         uint32_t abort_code)
{
    co_sdo_channel_reset(node, ch);
    return co_send_sdo_abort(node, index, subindex, abort_code);
}

static co_error_t co_sdo_send_upload_segment(co_node_t *node, size_t ch)
{
    co_can_frame_t tx = {0};
    tx.cob_id = CO_COB_SDO_TX_BASE + node->node_id;
    tx.len = 8U;

    const size_t remaining = node->sdo_channels[ch].size - node->sdo_channels[ch].offset;
    const size_t send_size = (remaining > 7U) ? 7U : remaining;
    const bool is_last = (node->sdo_channels[ch].offset + send_size) >= node->sdo_channels[ch].size;
    const uint8_t n = (uint8_t)(7U - send_size);

    tx.data[0] = (uint8_t)((node->sdo_channels[ch].toggle << 4U) | (n << 1U) | (is_last ? 0x01U : 0x00U));
    memcpy(&tx.data[1], &node->sdo_channels[ch].buffer[node->sdo_channels[ch].offset], send_size);
    node->sdo_channels[ch].offset += send_size;
    node->sdo_channels[ch].toggle ^= 1U;
    if (is_last) {
        co_sdo_channel_reset(node, ch);
    }
    return node->iface.send(node->iface.user, &tx);
}

static co_error_t co_sdo_send_block_upload_data(co_node_t *node, size_t ch)
{
    uint8_t seq = (uint8_t)(node->sdo_channels[ch].block_last_sent_seq + 1U);
    for (; seq <= node->sdo_channels[ch].block_blksize; ++seq) {
        co_can_frame_t tx = {0};
        tx.cob_id = CO_COB_SDO_TX_BASE + node->node_id;
        tx.len = 8U;

        const size_t remaining = node->sdo_channels[ch].size - node->sdo_channels[ch].offset;
        const size_t send_size = (remaining > 7U) ? 7U : remaining;
        const bool last = (node->sdo_channels[ch].offset + send_size) >= node->sdo_channels[ch].size;

        tx.data[0] = (uint8_t)(seq | (last ? 0x80U : 0x00U));
        memcpy(&tx.data[1], &node->sdo_channels[ch].buffer[node->sdo_channels[ch].offset], send_size);
        node->sdo_channels[ch].offset += send_size;
        node->sdo_channels[ch].block_last_sent_seq = seq;
        if (last) {
            node->sdo_channels[ch].block_finished = 1U;
        }

        const co_error_t err = node->iface.send(node->iface.user, &tx);
        if (err != CO_ERROR_NONE) {
            return err;
        }
        if (last) {
            node->sdo_channels[ch].state = CO_SDO_STATE_BLK_UPLOAD_WAIT_ACK;
            return CO_ERROR_NONE;
        }
    }

    node->sdo_channels[ch].state = CO_SDO_STATE_BLK_UPLOAD_WAIT_ACK;
    return CO_ERROR_NONE;
}

static co_error_t co_process_sdo(co_node_t *node, const co_can_frame_t *frame)
{
    const size_t ch = 0U;
    if (frame->len != 8U) {
        return co_send_sdo_abort(node, 0, 0, CO_SDO_ABORT_COMMAND);
    }

    const uint8_t cmd = frame->data[0] & 0xE0U;
    const uint16_t index = (uint16_t)frame->data[1] | ((uint16_t)frame->data[2] << 8U);
    const uint8_t subindex = frame->data[3];

    co_can_frame_t tx = {0};
    tx.cob_id = CO_COB_SDO_TX_BASE + node->node_id;
    tx.len = 8;

    node->sdo_channels[ch].last_activity_ms = node->iface.millis ? node->iface.millis(node->iface.user) : 0U;

    if (frame->data[0] == 0x80U) {
        co_sdo_channel_reset(node, ch);
        return CO_ERROR_NONE;
    }

    if (cmd == 0x40U) {
        co_od_entry_t *entry = NULL;
        const uint32_t abort_code = co_sdo_validate_request(node, index, subindex, false, 0U, &entry);
        if (abort_code != 0U) {
            return co_sdo_abort_and_reset(node, ch, index, subindex, abort_code);
        }

        if (entry->on_read) {
            const uint32_t cb_abort = entry->on_read(node, entry, entry->user);
            if (cb_abort != 0U) {
                return co_sdo_abort_and_reset(node, ch, index, subindex, cb_abort);
            }
        }

        if (entry->size <= 4U) {
            const uint8_t n = (uint8_t)(4U - entry->size);
            tx.data[0] = (uint8_t)(0x43U | (n << 2U));
            tx.data[1] = frame->data[1];
            tx.data[2] = frame->data[2];
            tx.data[3] = frame->data[3];
            memcpy(&tx.data[4], entry->data, entry->size);
            co_sdo_channel_reset(node, ch);
            return node->iface.send(node->iface.user, &tx);
        }

        if (entry->size > CO_SDO_TRANSFER_BUF_SIZE) {
            return co_sdo_abort_and_reset(node, ch, index, subindex, CO_SDO_ABORT_OUT_OF_MEMORY);
        }

        memcpy(node->sdo_channels[ch].buffer, entry->data, entry->size);
        node->sdo_channels[ch].state = CO_SDO_STATE_SEG_UPLOAD;
        node->sdo_channels[ch].mode = CO_SDO_MODE_UPLOAD;
        node->sdo_channels[ch].toggle = 0U;
        node->sdo_channels[ch].offset = 0U;
        node->sdo_channels[ch].size = entry->size;
        node->sdo_channels[ch].index = index;
        node->sdo_channels[ch].subindex = subindex;

        tx.data[0] = 0x41U;
        tx.data[1] = frame->data[1];
        tx.data[2] = frame->data[2];
        tx.data[3] = frame->data[3];
        tx.data[4] = (uint8_t)(entry->size & 0xFFU);
        tx.data[5] = (uint8_t)((entry->size >> 8U) & 0xFFU);
        tx.data[6] = (uint8_t)((entry->size >> 16U) & 0xFFU);
        tx.data[7] = (uint8_t)((entry->size >> 24U) & 0xFFU);
        return node->iface.send(node->iface.user, &tx);
    }

    if (cmd == 0x20U) {
        const bool expedited = (frame->data[0] & 0x02U) != 0U;
        const bool size_indicated = (frame->data[0] & 0x01U) != 0U;
        co_od_entry_t *entry = NULL;
        size_t payload_size = 0U;
        if (expedited && size_indicated) {
            const uint8_t n = (frame->data[0] >> 2U) & 0x03U;
            payload_size = (size_t)(4U - n);
        } else if (!expedited && size_indicated) {
            payload_size = (size_t)frame->data[4] | ((size_t)frame->data[5] << 8U) |
                           ((size_t)frame->data[6] << 16U) | ((size_t)frame->data[7] << 24U);
        }
        const uint32_t abort_code = co_sdo_validate_request(node, index, subindex, true, payload_size, &entry);
        if (abort_code != 0U) {
            return co_sdo_abort_and_reset(node, ch, index, subindex, abort_code);
        }

        if (expedited && size_indicated) {
            if (entry->on_write) {
                const uint32_t cb_abort = entry->on_write(node, entry, &frame->data[4], payload_size, entry->user);
                if (cb_abort != 0U) {
                    return co_sdo_abort_and_reset(node, ch, index, subindex, cb_abort);
                }
            }
            memcpy(entry->data, &frame->data[4], payload_size);
            tx.data[0] = 0x60U;
            tx.data[1] = frame->data[1];
            tx.data[2] = frame->data[2];
            tx.data[3] = frame->data[3];
            co_sdo_channel_reset(node, ch);
            return node->iface.send(node->iface.user, &tx);
        }

        if (!expedited && size_indicated) {
            if (payload_size > CO_SDO_TRANSFER_BUF_SIZE) {
                return co_sdo_abort_and_reset(node, ch, index, subindex, CO_SDO_ABORT_OUT_OF_MEMORY);
            }
            node->sdo_channels[ch].state = CO_SDO_STATE_SEG_DOWNLOAD;
            node->sdo_channels[ch].mode = CO_SDO_MODE_DOWNLOAD;
            node->sdo_channels[ch].toggle = 0U;
            node->sdo_channels[ch].offset = 0U;
            node->sdo_channels[ch].size = payload_size;
            node->sdo_channels[ch].index = index;
            node->sdo_channels[ch].subindex = subindex;
            tx.data[0] = 0x60U;
            tx.data[1] = frame->data[1];
            tx.data[2] = frame->data[2];
            tx.data[3] = frame->data[3];
            return node->iface.send(node->iface.user, &tx);
        }

        return co_sdo_abort_and_reset(node, ch, index, subindex, CO_SDO_ABORT_PARAM_LENGTH);
    }

    if (cmd == 0x00U && node->sdo_channels[ch].state == CO_SDO_STATE_SEG_DOWNLOAD) {
        if (((frame->data[0] >> 4U) & 0x01U) != node->sdo_channels[ch].toggle) {
            return co_sdo_abort_and_reset(node,
                                          ch,
                                          node->sdo_channels[ch].index,
                                          node->sdo_channels[ch].subindex,
                                          CO_SDO_ABORT_TOGGLE);
        }
        const uint8_t n = (frame->data[0] >> 1U) & 0x07U;
        const bool last = (frame->data[0] & 0x01U) != 0U;
        const size_t chunk = 7U - n;
        if ((node->sdo_channels[ch].offset + chunk) > node->sdo_channels[ch].size) {
            return co_sdo_abort_and_reset(node,
                                          ch,
                                          node->sdo_channels[ch].index,
                                          node->sdo_channels[ch].subindex,
                                          CO_SDO_ABORT_PARAM_TOO_HIGH);
        }
        memcpy(&node->sdo_channels[ch].buffer[node->sdo_channels[ch].offset], &frame->data[1], chunk);
        node->sdo_channels[ch].offset += chunk;
        node->sdo_channels[ch].toggle ^= 1U;

        tx.data[0] = (uint8_t)(0x20U | ((node->sdo_channels[ch].toggle & 0x01U) << 4U));
        if (last) {
            co_od_entry_t *entry = NULL;
            const uint32_t abort_code = co_sdo_validate_request(node,
                                                                node->sdo_channels[ch].index,
                                                                node->sdo_channels[ch].subindex,
                                                                true,
                                                                node->sdo_channels[ch].offset,
                                                                &entry);
            if (abort_code != 0U) {
                return co_sdo_abort_and_reset(node,
                                              ch,
                                              node->sdo_channels[ch].index,
                                              node->sdo_channels[ch].subindex,
                                              abort_code);
            }
            if (entry->on_write) {
                const uint32_t cb_abort = entry->on_write(node,
                                                          entry,
                                                          node->sdo_channels[ch].buffer,
                                                          node->sdo_channels[ch].offset,
                                                          entry->user);
                if (cb_abort != 0U) {
                    return co_sdo_abort_and_reset(node,
                                                  ch,
                                                  node->sdo_channels[ch].index,
                                                  node->sdo_channels[ch].subindex,
                                                  cb_abort);
                }
            }
            memcpy(entry->data, node->sdo_channels[ch].buffer, node->sdo_channels[ch].offset);
            tx.data[0] = 0x20U;
            co_sdo_channel_reset(node, ch);
        }
        return node->iface.send(node->iface.user, &tx);
    }

    if (cmd == 0x60U && node->sdo_channels[ch].state == CO_SDO_STATE_SEG_UPLOAD) {
        if (((frame->data[0] >> 4U) & 0x01U) != node->sdo_channels[ch].toggle) {
            return co_sdo_abort_and_reset(node,
                                          ch,
                                          node->sdo_channels[ch].index,
                                          node->sdo_channels[ch].subindex,
                                          CO_SDO_ABORT_TOGGLE);
        }
        return co_sdo_send_upload_segment(node, ch);
    }

    if (cmd == 0xC0U) {
        const uint8_t subcmd = frame->data[0] & 0x03U;
        if (subcmd == 0x00U) {
            co_od_entry_t *entry = NULL;
            const uint32_t abort_code = co_sdo_validate_request(node, index, subindex, true, 0U, &entry);
            if (abort_code != 0U) {
                return co_sdo_abort_and_reset(node, ch, index, subindex, abort_code);
            }
            node->sdo_channels[ch].state = CO_SDO_STATE_BLK_DOWNLOAD;
            node->sdo_channels[ch].mode = CO_SDO_MODE_DOWNLOAD;
            node->sdo_channels[ch].index = index;
            node->sdo_channels[ch].subindex = subindex;
            node->sdo_channels[ch].offset = 0U;
            node->sdo_channels[ch].size = entry->size;
            node->sdo_channels[ch].block_blksize = frame->data[4] ? frame->data[4] : 127U;
            node->sdo_channels[ch].block_seq = 0U;
            tx.data[0] = 0xA4U;
            tx.data[1] = frame->data[1];
            tx.data[2] = frame->data[2];
            tx.data[3] = frame->data[3];
            tx.data[4] = node->sdo_channels[ch].block_blksize;
            return node->iface.send(node->iface.user, &tx);
        }
    }

    if (cmd == 0xA0U) {
        const uint8_t subcmd = frame->data[0] & 0x03U;
        if (subcmd == 0x00U) {
            co_od_entry_t *entry = NULL;
            const uint32_t abort_code = co_sdo_validate_request(node, index, subindex, false, 0U, &entry);
            if (abort_code != 0U) {
                return co_sdo_abort_and_reset(node, ch, index, subindex, abort_code);
            }
            if (entry->on_read) {
                const uint32_t cb_abort = entry->on_read(node, entry, entry->user);
                if (cb_abort != 0U) {
                    return co_sdo_abort_and_reset(node, ch, index, subindex, cb_abort);
                }
            }
            if (entry->size > CO_SDO_TRANSFER_BUF_SIZE) {
                return co_sdo_abort_and_reset(node, ch, index, subindex, CO_SDO_ABORT_OUT_OF_MEMORY);
            }
            memcpy(node->sdo_channels[ch].buffer, entry->data, entry->size);
            node->sdo_channels[ch].state = CO_SDO_STATE_BLK_UPLOAD;
            node->sdo_channels[ch].mode = CO_SDO_MODE_UPLOAD;
            node->sdo_channels[ch].index = index;
            node->sdo_channels[ch].subindex = subindex;
            node->sdo_channels[ch].size = entry->size;
            node->sdo_channels[ch].offset = 0U;
            node->sdo_channels[ch].block_blksize = frame->data[4] ? frame->data[4] : 127U;
            node->sdo_channels[ch].block_last_sent_seq = 0U;
            node->sdo_channels[ch].block_finished = 0U;

            tx.data[0] = 0xC6U;
            tx.data[1] = frame->data[1];
            tx.data[2] = frame->data[2];
            tx.data[3] = frame->data[3];
            tx.data[4] = node->sdo_channels[ch].block_blksize;
            tx.data[5] = 0x01U;
            tx.data[6] = (uint8_t)(entry->size & 0xFFU);
            tx.data[7] = (uint8_t)((entry->size >> 8U) & 0xFFU);
            const co_error_t err = node->iface.send(node->iface.user, &tx);
            if (err != CO_ERROR_NONE) {
                return err;
            }
            return co_sdo_send_block_upload_data(node, ch);
        }
    }

    if (node->sdo_channels[ch].state == CO_SDO_STATE_BLK_DOWNLOAD && (frame->data[0] & 0x7FU) > 0U) {
        const uint8_t seq = frame->data[0] & 0x7FU;
        if (seq != (uint8_t)(node->sdo_channels[ch].block_seq + 1U)) {
            return co_sdo_abort_and_reset(node,
                                          ch,
                                          node->sdo_channels[ch].index,
                                          node->sdo_channels[ch].subindex,
                                          CO_SDO_ABORT_TOGGLE);
        }
        const bool last = (frame->data[0] & 0x80U) != 0U;
        node->sdo_channels[ch].block_seq = seq;
        if ((node->sdo_channels[ch].offset + 7U) > CO_SDO_TRANSFER_BUF_SIZE) {
            return co_sdo_abort_and_reset(node,
                                          ch,
                                          node->sdo_channels[ch].index,
                                          node->sdo_channels[ch].subindex,
                                          CO_SDO_ABORT_PARAM_TOO_HIGH);
        }
        memcpy(&node->sdo_channels[ch].buffer[node->sdo_channels[ch].offset], &frame->data[1], 7U);
        node->sdo_channels[ch].offset += 7U;

        if (seq >= node->sdo_channels[ch].block_blksize || last) {
            tx.data[0] = 0xA2U;
            tx.data[1] = node->sdo_channels[ch].block_seq;
            tx.data[2] = node->sdo_channels[ch].block_blksize;
            node->sdo_channels[ch].block_seq = 0U;
            const co_error_t err = node->iface.send(node->iface.user, &tx);
            if (err != CO_ERROR_NONE) {
                return err;
            }
            if (last) {
                node->sdo_channels[ch].state = CO_SDO_STATE_BLK_DOWNLOAD_END;
            }
        }
        return CO_ERROR_NONE;
    }

    if (node->sdo_channels[ch].state == CO_SDO_STATE_BLK_DOWNLOAD_END && (frame->data[0] & 0xE1U) == 0xC1U) {
        const uint8_t n = (frame->data[0] >> 2U) & 0x07U;
        if (n > 7U || node->sdo_channels[ch].offset < n) {
            return co_sdo_abort_and_reset(node,
                                          ch,
                                          node->sdo_channels[ch].index,
                                          node->sdo_channels[ch].subindex,
                                          CO_SDO_ABORT_PARAM_LENGTH);
        }
        node->sdo_channels[ch].offset -= n;

        co_od_entry_t *entry = NULL;
        const uint32_t abort_code = co_sdo_validate_request(node,
                                                            node->sdo_channels[ch].index,
                                                            node->sdo_channels[ch].subindex,
                                                            true,
                                                            node->sdo_channels[ch].offset,
                                                            &entry);
        if (abort_code != 0U) {
            return co_sdo_abort_and_reset(node,
                                          ch,
                                          node->sdo_channels[ch].index,
                                          node->sdo_channels[ch].subindex,
                                          abort_code);
        }
        if (entry->on_write) {
            const uint32_t cb_abort = entry->on_write(node,
                                                      entry,
                                                      node->sdo_channels[ch].buffer,
                                                      node->sdo_channels[ch].offset,
                                                      entry->user);
            if (cb_abort != 0U) {
                return co_sdo_abort_and_reset(node,
                                              ch,
                                              node->sdo_channels[ch].index,
                                              node->sdo_channels[ch].subindex,
                                              cb_abort);
            }
        }
        memcpy(entry->data, node->sdo_channels[ch].buffer, node->sdo_channels[ch].offset);
        tx.data[0] = 0xA1U;
        co_sdo_channel_reset(node, ch);
        return node->iface.send(node->iface.user, &tx);
    }

    if (node->sdo_channels[ch].state == CO_SDO_STATE_BLK_UPLOAD_WAIT_ACK && (frame->data[0] & 0xE0U) == 0xA0U) {
        const uint8_t ack_seq = frame->data[1];
        const uint8_t req_blksize = frame->data[2];
        if (ack_seq < node->sdo_channels[ch].block_last_sent_seq && !node->sdo_channels[ch].block_finished) {
            const size_t rollback = (size_t)(node->sdo_channels[ch].block_last_sent_seq - ack_seq) * 7U;
            if (rollback <= node->sdo_channels[ch].offset) {
                node->sdo_channels[ch].offset -= rollback;
            }
            node->sdo_channels[ch].block_last_sent_seq = ack_seq;
        }
        if (!node->sdo_channels[ch].block_finished) {
            node->sdo_channels[ch].state = CO_SDO_STATE_BLK_UPLOAD;
            node->sdo_channels[ch].block_blksize = req_blksize ? req_blksize : node->sdo_channels[ch].block_blksize;
            return co_sdo_send_block_upload_data(node, ch);
        }
        const uint8_t rem = (uint8_t)(node->sdo_channels[ch].size % 7U);
        const uint8_t n = (rem == 0U) ? 0U : (uint8_t)(7U - rem);
        tx.data[0] = (uint8_t)(0xC1U | (n << 2U));
        co_sdo_channel_reset(node, ch);
        return node->iface.send(node->iface.user, &tx);
    }

    return co_sdo_abort_and_reset(node, ch, index, subindex, CO_SDO_ABORT_COMMAND);
}

static co_error_t co_process_nmt(co_node_t *node, const co_can_frame_t *frame)
{
    if (frame->len != 2U) {
        return CO_ERROR_INVALID_ARGS;
    }

    if (frame->data[1] != 0U && frame->data[1] != node->node_id) {
        return CO_ERROR_NONE;
    }

    switch (frame->data[0]) {
        case 0x01:
            if (node->nmt_state == CO_NMT_INITIALIZING) {
                return CO_ERROR_NONE;
            }
            node->nmt_state = CO_NMT_OPERATIONAL;
            break;
        case 0x02:
            if (node->nmt_state == CO_NMT_INITIALIZING) {
                return CO_ERROR_NONE;
            }
            node->nmt_state = CO_NMT_STOPPED;
            break;
        case 0x80:
            if (node->nmt_state == CO_NMT_INITIALIZING) {
                return CO_ERROR_NONE;
            }
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
    co_restore_default_comm_objects(node);
    co_schedule_bootup(node);

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

    const uint32_t now = node->iface.millis(node->iface.user);
    for (size_t ch = 0; ch < CO_SDO_CHANNELS; ++ch) {
        if (node->sdo_channels[ch].state == CO_SDO_STATE_IDLE) {
            continue;
        }
        if ((now - node->sdo_channels[ch].last_activity_ms) >= CO_SDO_TIMEOUT_MS) {
            const co_error_t err = co_send_sdo_abort(node,
                                                     node->sdo_channels[ch].index,
                                                     node->sdo_channels[ch].subindex,
                                                     CO_SDO_ABORT_TIMEOUT);
            co_sdo_channel_reset(node, ch);
            if (err != CO_ERROR_NONE) {
                return err;
            }
        }
    }

    if (node->nmt_state == CO_NMT_INITIALIZING && node->bootup_pending) {
        const co_error_t err = co_send_bootup(node);
        if (err != CO_ERROR_NONE) {
            return err;
        }
        node->nmt_state = CO_NMT_PRE_OPERATIONAL;
        node->bootup_pending = false;
        node->last_heartbeat_ms = now;
        return CO_ERROR_NONE;
    }

    if (node->heartbeat_ms == 0U) {
        return CO_ERROR_NONE;
    }

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
