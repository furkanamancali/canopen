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
#define CO_PDO_COMM_SUB_COUNT 5U
#define CO_PDO_MAX_BITS 64U
#define CO_SYNC_COB_ID_PRODUCER_BIT 0x40000000UL
#define CO_SYNC_COB_ID_INVALID_BIT 0x80000000UL
#define CO_EMCY_COB_ID_INVALID_BIT 0x80000000UL

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

static bool co_decode_pdo_mapping(uint32_t raw, co_pdo_map_entry_t *decoded)
{
    if (!decoded) {
        return false;
    }

    decoded->index = (uint16_t)((raw >> 16U) & 0xFFFFU);
    decoded->subindex = (uint8_t)((raw >> 8U) & 0xFFU);
    decoded->bit_length = (uint8_t)(raw & 0xFFU);
    return true;
}

static bool co_decode_pdo_cob_id(uint32_t raw, co_pdo_cob_id_cfg_t *decoded)
{
    if (!decoded) {
        return false;
    }

    decoded->valid = (raw & 0x80000000UL) == 0U;
    decoded->rtr_allowed = (raw & 0x40000000UL) == 0U;
    decoded->extended = (raw & 0x20000000UL) != 0U;
    decoded->can_id = decoded->extended ? (raw & 0x1FFFFFFFUL) : (raw & 0x7FFUL);
    return true;
}

static uint32_t co_encode_pdo_cob_id(const co_pdo_cob_id_cfg_t *cfg)
{
    uint32_t raw = cfg->can_id & (cfg->extended ? 0x1FFFFFFFUL : 0x7FFUL);
    if (!cfg->valid) {
        raw |= 0x80000000UL;
    }
    if (!cfg->rtr_allowed) {
        raw |= 0x40000000UL;
    }
    if (cfg->extended) {
        raw |= 0x20000000UL;
    }
    return raw;
}

static bool co_copy_bits(uint8_t *dst,
                         size_t dst_bits,
                         size_t dst_offset_bits,
                         const uint8_t *src,
                         size_t src_bits,
                         size_t src_offset_bits,
                         size_t bit_count)
{
    if (!dst || !src) {
        return false;
    }
    if ((dst_offset_bits + bit_count) > dst_bits || (src_offset_bits + bit_count) > src_bits) {
        return false;
    }

    for (size_t bit = 0; bit < bit_count; ++bit) {
        const size_t src_bit = src_offset_bits + bit;
        const uint8_t src_val = (uint8_t)((src[src_bit / 8U] >> (src_bit % 8U)) & 0x01U);
        const size_t dst_bit = dst_offset_bits + bit;
        uint8_t *dst_byte = &dst[dst_bit / 8U];
        const uint8_t dst_mask = (uint8_t)(1U << (dst_bit % 8U));
        if (src_val != 0U) {
            *dst_byte |= dst_mask;
        } else {
            *dst_byte = (uint8_t)(*dst_byte & (uint8_t)~dst_mask);
        }
    }
    return true;
}

static bool co_pdo_mapping_is_valid(co_node_t *node, const co_pdo_cfg_t *cfg, uint32_t *total_bits_out)
{
    if (!node || !cfg) {
        return false;
    }

    uint32_t total_bits = 0U;
    for (uint8_t i = 0; i < cfg->map_count; ++i) {
        co_pdo_map_entry_t mapped = {0};
        if (!co_decode_pdo_mapping(cfg->mapping_raw[i], &mapped)) {
            return false;
        }
        if (mapped.index == 0U || mapped.bit_length == 0U) {
            return false;
        }
        co_od_entry_t *entry = co_od_find(node, mapped.index, mapped.subindex);
        if (!entry) {
            return false;
        }
        if (mapped.bit_length > (entry->size * 8U)) {
            return false;
        }
        total_bits += mapped.bit_length;
        if (total_bits > CO_PDO_MAX_BITS) {
            return false;
        }
    }

    if (total_bits_out) {
        *total_bits_out = total_bits;
    }
    return true;
}

static void co_error_register_recompute(co_node_t *node)
{
    uint8_t reg = 0U;
    for (uint8_t i = 0U; i < CO_EMCY_FAULT_SLOTS; ++i) {
        if (node->faults[i].active) {
            reg = (uint8_t)(reg | node->faults[i].reg_bits);
        }
    }
    node->error_register = reg;
}

static void co_predef_error_push(co_node_t *node, uint16_t emcy_code, uint8_t msef)
{
    for (uint8_t i = CO_EMCY_ERROR_HISTORY_LEN - 1U; i > 0U; --i) {
        node->predef_error_field[i] = node->predef_error_field[i - 1U];
    }

    node->predef_error_field[0] = (uint32_t)emcy_code | ((uint32_t)node->error_register << 16U) |
                                  ((uint32_t)msef << 24U);

    if (node->predef_error_count < CO_EMCY_ERROR_HISTORY_LEN) {
        node->predef_error_count++;
    }
}

static co_error_t co_send_emcy(co_node_t *node, uint16_t emcy_code, uint8_t msef, const uint8_t mfg_data[5])
{
    if (!node || !node->iface.send || !node->emcy_valid) {
        return CO_ERROR_NONE;
    }

    co_can_frame_t frame = {0};
    frame.cob_id = node->emcy_cob_id;
    frame.len = 8U;
    frame.data[0] = (uint8_t)(emcy_code & 0xFFU);
    frame.data[1] = (uint8_t)(emcy_code >> 8U);
    frame.data[2] = node->error_register;
    frame.data[3] = msef;
    if (mfg_data) {
        memcpy(&frame.data[4], mfg_data, 4U);
    }
    return node->iface.send(node->iface.user, &frame);
}

static co_error_t co_send_frame(co_node_t *node, const co_can_frame_t *frame)
{
    if (!node || !frame || !node->iface.send) {
        return CO_ERROR_INVALID_ARGS;
    }

    const co_error_t err = node->iface.send(node->iface.user, frame);
    if (err != CO_ERROR_NONE) {
        (void)co_fault_raise(node,
                             CO_FAULT_CAN_TX,
                             CO_EMCY_ERR_CAN_TX,
                             CO_ERROR_REG_COMMUNICATION,
                             0U,
                             NULL);
    } else {
        (void)co_fault_clear(node, CO_FAULT_CAN_TX, 0U, NULL);
    }
    return err;
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

static uint32_t co_on_predef_error_count_write(co_node_t *node,
                                               const co_od_entry_t *entry,
                                               const uint8_t *data,
                                               size_t size,
                                               void *user)
{
    (void)entry;
    (void)user;
    if (size != sizeof(uint8_t)) {
        return CO_SDO_ABORT_PARAM_LENGTH;
    }
    if (data[0] != 0U) {
        return CO_SDO_ABORT_PARAM_TOO_HIGH;
    }

    node->predef_error_count = 0U;
    memset(node->predef_error_field, 0, sizeof(node->predef_error_field));
    return 0U;
}

static uint32_t co_on_emcy_cob_id_write(co_node_t *node,
                                        const co_od_entry_t *entry,
                                        const uint8_t *data,
                                        size_t size,
                                        void *user)
{
    (void)entry;
    (void)user;
    if (size != sizeof(uint32_t)) {
        return CO_SDO_ABORT_PARAM_LENGTH;
    }

    uint32_t raw = 0U;
    memcpy(&raw, data, sizeof(raw));
    if ((raw & 0x3FFFF800UL) != 0U) {
        return CO_SDO_ABORT_PARAM_TOO_HIGH;
    }

    node->emcy_cob_id = raw & 0x7FFU;
    node->emcy_valid = (raw & CO_EMCY_COB_ID_INVALID_BIT) == 0U;
    return 0U;
}

static uint32_t co_on_sync_cob_id_write(co_node_t *node,
                                        const co_od_entry_t *entry,
                                        const uint8_t *data,
                                        size_t size,
                                        void *user)
{
    (void)entry;
    (void)user;
    if (size != sizeof(uint32_t)) {
        return CO_SDO_ABORT_PARAM_LENGTH;
    }

    uint32_t raw = 0U;
    memcpy(&raw, data, sizeof(raw));
    if ((raw & 0x3FFFF800UL) != 0U) {
        return CO_SDO_ABORT_PARAM_TOO_HIGH;
    }

    node->sync_cob_id_raw = raw;
    node->sync_cob_id = (uint16_t)(raw & 0x7FFU);
    node->sync_valid = (raw & CO_SYNC_COB_ID_INVALID_BIT) == 0U;
    node->sync_producer = (raw & CO_SYNC_COB_ID_PRODUCER_BIT) != 0U;
    return 0U;
}

static uint32_t co_on_sync_overflow_write(co_node_t *node,
                                          const co_od_entry_t *entry,
                                          const uint8_t *data,
                                          size_t size,
                                          void *user)
{
    (void)entry;
    (void)user;
    if (size != sizeof(uint8_t)) {
        return CO_SDO_ABORT_PARAM_LENGTH;
    }
    node->sync_overflow = data[0];
    if (node->sync_overflow > 0U && node->sync_counter > node->sync_overflow) {
        node->sync_counter = 1U;
    }
    return 0U;
}

static co_pdo_cfg_t *co_get_pdo_cfg(co_node_t *node, uint16_t index)
{
    if (index >= 0x1400U && index < (0x1400U + CO_MAX_RPDO)) {
        return &node->rpdo_cfg[index - 0x1400U];
    }
    if (index >= 0x1600U && index < (0x1600U + CO_MAX_RPDO)) {
        return &node->rpdo_cfg[index - 0x1600U];
    }
    if (index >= 0x1800U && index < (0x1800U + CO_MAX_TPDO)) {
        return &node->tpdo_cfg[index - 0x1800U];
    }
    if (index >= 0x1A00U && index < (0x1A00U + CO_MAX_TPDO)) {
        return &node->tpdo_cfg[index - 0x1A00U];
    }
    return NULL;
}

static uint32_t co_on_pdo_comm_cob_id_write(co_node_t *node,
                                            const co_od_entry_t *entry,
                                            const uint8_t *data,
                                            size_t size,
                                            void *user)
{
    (void)user;
    if (size != sizeof(uint32_t)) {
        return CO_SDO_ABORT_PARAM_LENGTH;
    }

    co_pdo_cfg_t *cfg = co_get_pdo_cfg(node, entry->index);
    if (!cfg) {
        return CO_SDO_ABORT_NO_OBJECT;
    }

    uint32_t raw = 0U;
    memcpy(&raw, data, sizeof(raw));
    co_pdo_cob_id_cfg_t parsed = {0};
    if (!co_decode_pdo_cob_id(raw, &parsed)) {
        return CO_SDO_ABORT_PARAM_LENGTH;
    }
    if ((!parsed.extended && parsed.can_id > 0x7FFU) || (parsed.extended && parsed.can_id > 0x1FFFFFFFUL)) {
        return CO_SDO_ABORT_PARAM_TOO_HIGH;
    }

    cfg->cob_id_raw = raw;
    cfg->cob_id = parsed;
    return 0U;
}

static uint32_t co_on_pdo_mapping_entry_write(co_node_t *node,
                                              const co_od_entry_t *entry,
                                              const uint8_t *data,
                                              size_t size,
                                              void *user)
{
    (void)user;
    if (size != sizeof(uint32_t) || entry->subindex == 0U || entry->subindex > CO_PDO_MAP_ENTRIES) {
        return CO_SDO_ABORT_PARAM_LENGTH;
    }
    co_pdo_cfg_t *cfg = co_get_pdo_cfg(node, entry->index);
    if (!cfg) {
        return CO_SDO_ABORT_NO_OBJECT;
    }

    uint32_t raw = 0U;
    memcpy(&raw, data, sizeof(raw));
    co_pdo_map_entry_t mapped = {0};
    if (!co_decode_pdo_mapping(raw, &mapped)) {
        return CO_SDO_ABORT_PARAM_LENGTH;
    }

    if (mapped.bit_length > 0U) {
        co_od_entry_t *mapped_entry = co_od_find(node, mapped.index, mapped.subindex);
        if (!mapped_entry) {
            return CO_SDO_ABORT_NO_OBJECT;
        }
        if ((entry->index >= 0x1600U && entry->index < (0x1600U + CO_MAX_RPDO) &&
             (mapped_entry->access & CO_OD_ACCESS_WRITE) == 0U) ||
            (entry->index >= 0x1A00U && entry->index < (0x1A00U + CO_MAX_TPDO) &&
             (mapped_entry->access & CO_OD_ACCESS_READ) == 0U)) {
            return CO_SDO_ABORT_READONLY;
        }
        if (mapped.bit_length > (mapped_entry->size * 8U)) {
            return CO_SDO_ABORT_PARAM_TOO_HIGH;
        }
    }

    const uint8_t map_slot = (uint8_t)(entry->subindex - 1U);
    const uint32_t prev_raw = cfg->mapping_raw[map_slot];
    cfg->mapping_raw[map_slot] = raw;
    uint32_t total_bits = 0U;
    if (!co_pdo_mapping_is_valid(node, cfg, &total_bits)) {
        cfg->mapping_raw[map_slot] = prev_raw;
        return CO_SDO_ABORT_PARAM_LENGTH;
    }
    cfg->mapping[map_slot] = mapped;

    return 0U;
}

static uint32_t co_on_pdo_mapping_count_write(co_node_t *node,
                                              const co_od_entry_t *entry,
                                              const uint8_t *data,
                                              size_t size,
                                              void *user)
{
    (void)user;
    if (size != sizeof(uint8_t)) {
        return CO_SDO_ABORT_PARAM_LENGTH;
    }

    const uint8_t count = data[0];
    if (count > CO_PDO_MAP_ENTRIES) {
        return CO_SDO_ABORT_PARAM_TOO_HIGH;
    }

    co_pdo_cfg_t *cfg = co_get_pdo_cfg(node, entry->index);
    if (!cfg) {
        return CO_SDO_ABORT_NO_OBJECT;
    }

    const uint8_t prev_count = cfg->map_count;
    cfg->map_count = count;
    uint32_t total_bits = 0U;
    if (!co_pdo_mapping_is_valid(node, cfg, &total_bits)) {
        cfg->map_count = prev_count;
        return CO_SDO_ABORT_PARAM_LENGTH;
    }

    return 0U;
}

static void co_hb_consumer_decode_cfg(uint32_t raw, co_hb_consumer_cfg_t *cfg)
{
    if (!cfg) {
        return;
    }

    cfg->node_id = (uint8_t)((raw >> 16U) & 0x7FU);
    cfg->timeout_ms = (uint16_t)(raw & 0xFFFFU);
    cfg->enabled = (cfg->node_id != 0U) && (cfg->timeout_ms != 0U);
}

static void co_hb_consumer_sync_from_raw(co_node_t *node, uint8_t idx)
{
    if (!node || idx >= CO_HEARTBEAT_CONSUMERS_MAX) {
        return;
    }

    co_hb_consumer_decode_cfg(node->hb_consumer_cfg_raw[idx], &node->hb_consumers[idx]);
    if (!node->hb_consumers[idx].enabled) {
        node->hb_runtime[idx].state = CO_HB_CONSUMER_DISABLED;
        node->hb_runtime[idx].remote_state = CO_NMT_INITIALIZING;
        node->hb_runtime[idx].last_rx_ms = 0U;
    } else if (node->hb_runtime[idx].state == CO_HB_CONSUMER_DISABLED) {
        node->hb_runtime[idx].state = CO_HB_CONSUMER_UNKNOWN;
        node->hb_runtime[idx].remote_state = CO_NMT_INITIALIZING;
        node->hb_runtime[idx].last_rx_ms = 0U;
    }
}

static co_error_t co_hb_consumer_recompute_fault(co_node_t *node)
{
    bool timed_out = false;
    uint8_t timed_out_node = 0U;
    for (uint8_t i = 0U; i < node->hb_consumer_sub_count; ++i) {
        if (node->hb_runtime[i].state == CO_HB_CONSUMER_TIMEOUT) {
            timed_out = true;
            timed_out_node = node->hb_consumers[i].node_id;
            break;
        }
    }

    if (timed_out) {
        return co_fault_raise(node,
                              CO_FAULT_HEARTBEAT_CONSUMER,
                              CO_EMCY_ERR_HEARTBEAT_CONSUMER,
                              CO_ERROR_REG_COMMUNICATION,
                              timed_out_node,
                              NULL);
    }
    return co_fault_clear(node, CO_FAULT_HEARTBEAT_CONSUMER, 0U, NULL);
}

static uint32_t co_on_hb_consumer_subcount_write(co_node_t *node,
                                                 const co_od_entry_t *entry,
                                                 const uint8_t *data,
                                                 size_t size,
                                                 void *user)
{
    (void)entry;
    (void)user;
    if (!node || !data || size != sizeof(uint8_t)) {
        return CO_SDO_ABORT_PARAM_LENGTH;
    }

    if (data[0] > CO_HEARTBEAT_CONSUMERS_MAX) {
        return CO_SDO_ABORT_PARAM_TOO_HIGH;
    }

    node->hb_consumer_sub_count = data[0];
    for (uint8_t i = node->hb_consumer_sub_count; i < CO_HEARTBEAT_CONSUMERS_MAX; ++i) {
        node->hb_consumer_cfg_raw[i] = 0U;
        co_hb_consumer_sync_from_raw(node, i);
    }
    return 0U;
}

static uint32_t co_on_hb_consumer_entry_write(co_node_t *node,
                                              const co_od_entry_t *entry,
                                              const uint8_t *data,
                                              size_t size,
                                              void *user)
{
    (void)user;
    if (!node || !entry || !data || size != sizeof(uint32_t)) {
        return CO_SDO_ABORT_PARAM_LENGTH;
    }
    if (entry->subindex == 0U || entry->subindex > CO_HEARTBEAT_CONSUMERS_MAX) {
        return CO_SDO_ABORT_NO_SUBINDEX;
    }

    const uint8_t idx = (uint8_t)(entry->subindex - 1U);
    if (idx >= node->hb_consumer_sub_count) {
        return CO_SDO_ABORT_NO_SUBINDEX;
    }

    uint32_t raw = 0U;
    memcpy(&raw, data, sizeof(raw));
    co_hb_consumer_cfg_t cfg = {0};
    co_hb_consumer_decode_cfg(raw, &cfg);

    if (!cfg.enabled) {
        raw = 0U;
    }

    node->hb_consumer_cfg_raw[idx] = raw;
    co_hb_consumer_sync_from_raw(node, idx);
    return 0U;
}

static co_error_t co_register_builtin_od(co_node_t *node)
{
    co_error_t err = CO_ERROR_NONE;

    node->device_type = 0U;
    node->error_register = 0U;
    node->predef_error_count = 0U;
    memset(node->predef_error_field, 0, sizeof(node->predef_error_field));
    node->emcy_cob_id = CO_COB_EMCY_BASE + node->node_id;
    node->emcy_valid = true;
    memset(node->faults, 0, sizeof(node->faults));
    node->identity_sub_count = 4U;
    node->identity[0] = 0U;
    node->identity[1] = 0U;
    node->identity[2] = 0U;
    node->identity[3] = 0U;

    node->sdo_server_sub_count = 2U;
    node->sdo_server_cob_rx = CO_COB_SDO_RX_BASE + node->node_id;
    node->sdo_server_cob_tx = CO_COB_SDO_TX_BASE + node->node_id;
    node->sync_cob_id = CO_COB_SYNC;
    node->sync_cob_id_raw = node->sync_cob_id;
    node->sync_valid = true;
    node->sync_producer = false;
    node->sync_cycle_period_us = 0U;
    node->sync_overflow = 0U;
    node->sync_counter = 0U;
    node->hb_consumer_sub_count = CO_HEARTBEAT_CONSUMERS_MAX;
    memset(node->hb_consumer_cfg_raw, 0, sizeof(node->hb_consumer_cfg_raw));
    memset(node->hb_consumers, 0, sizeof(node->hb_consumers));
    memset(node->hb_runtime, 0, sizeof(node->hb_runtime));
    for (uint8_t i = 0U; i < CO_HEARTBEAT_CONSUMERS_MAX; ++i) {
        node->hb_runtime[i].state = CO_HB_CONSUMER_DISABLED;
    }

    for (uint8_t i = 0; i < CO_MAX_RPDO; ++i) {
        node->pdo_comm_sub_count[i] = CO_PDO_COMM_SUB_COUNT;
        node->rpdo_cfg[i].cob_id.valid = true;
        node->rpdo_cfg[i].cob_id.rtr_allowed = false;
        node->rpdo_cfg[i].cob_id.extended = false;
        node->rpdo_cfg[i].transmission_type = 255U;
        node->rpdo_cfg[i].inhibit_time_100us = 0U;
        node->rpdo_cfg[i].event_timer_ms = 0U;
        node->rpdo_cfg[i].map_count = 0U;
        memset(node->rpdo_cfg[i].mapping_raw, 0, sizeof(node->rpdo_cfg[i].mapping_raw));
        memset(node->rpdo_cfg[i].mapping, 0, sizeof(node->rpdo_cfg[i].mapping));
        node->rpdo_cfg[i].cob_id_raw = co_encode_pdo_cob_id(&node->rpdo_cfg[i].cob_id);
    }

    for (uint8_t i = 0; i < CO_MAX_TPDO; ++i) {
        node->pdo_comm_sub_count[CO_MAX_RPDO + i] = CO_PDO_COMM_SUB_COUNT;
        node->tpdo_cfg[i].cob_id.valid = true;
        node->tpdo_cfg[i].cob_id.rtr_allowed = false;
        node->tpdo_cfg[i].cob_id.extended = false;
        node->tpdo_cfg[i].transmission_type = 255U;
        node->tpdo_cfg[i].inhibit_time_100us = 0U;
        node->tpdo_cfg[i].event_timer_ms = 0U;
        node->tpdo_cfg[i].map_count = 0U;
        memset(node->tpdo_cfg[i].mapping_raw, 0, sizeof(node->tpdo_cfg[i].mapping_raw));
        memset(node->tpdo_cfg[i].mapping, 0, sizeof(node->tpdo_cfg[i].mapping));
        node->tpdo_cfg[i].cob_id_raw = co_encode_pdo_cob_id(&node->tpdo_cfg[i].cob_id);
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
                                  0x1003,
                                  0x00,
                                  &node->predef_error_count,
                                  sizeof(node->predef_error_count),
                                  CO_OD_ACCESS_READ | CO_OD_ACCESS_WRITE,
                                  NULL,
                                  co_on_predef_error_count_write,
                                  NULL);
    if (err != CO_ERROR_NONE) {
        return err;
    }

    for (uint8_t sub = 1U; sub <= CO_EMCY_ERROR_HISTORY_LEN; ++sub) {
        err = co_od_register_internal(node,
                                      0x1003,
                                      sub,
                                      (uint8_t *)&node->predef_error_field[sub - 1U],
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
                                  0x1014,
                                  0x00,
                                  (uint8_t *)&node->emcy_cob_id,
                                  sizeof(node->emcy_cob_id),
                                  CO_OD_ACCESS_READ | CO_OD_ACCESS_WRITE,
                                  NULL,
                                  co_on_emcy_cob_id_write,
                                  NULL);
    if (err != CO_ERROR_NONE) {
        return err;
    }

    err = co_od_register_internal(node,
                                  0x1005,
                                  0x00,
                                  (uint8_t *)&node->sync_cob_id_raw,
                                  sizeof(node->sync_cob_id_raw),
                                  CO_OD_ACCESS_READ | CO_OD_ACCESS_WRITE,
                                  NULL,
                                  co_on_sync_cob_id_write,
                                  NULL);
    if (err != CO_ERROR_NONE) {
        return err;
    }

    err = co_od_register_internal(node,
                                  0x1006,
                                  0x00,
                                  (uint8_t *)&node->sync_cycle_period_us,
                                  sizeof(node->sync_cycle_period_us),
                                  CO_OD_ACCESS_READ | CO_OD_ACCESS_WRITE,
                                  NULL,
                                  NULL,
                                  NULL);
    if (err != CO_ERROR_NONE) {
        return err;
    }

    err = co_od_register_internal(node,
                                  0x1019,
                                  0x00,
                                  &node->sync_overflow,
                                  sizeof(node->sync_overflow),
                                  CO_OD_ACCESS_READ | CO_OD_ACCESS_WRITE,
                                  NULL,
                                  co_on_sync_overflow_write,
                                  NULL);
    if (err != CO_ERROR_NONE) {
        return err;
    }

    err = co_od_register_internal(node,
                                  0x1016,
                                  0x00,
                                  &node->hb_consumer_sub_count,
                                  sizeof(node->hb_consumer_sub_count),
                                  CO_OD_ACCESS_READ | CO_OD_ACCESS_WRITE,
                                  NULL,
                                  co_on_hb_consumer_subcount_write,
                                  NULL);
    if (err != CO_ERROR_NONE) {
        return err;
    }

    for (uint8_t sub = 1U; sub <= CO_HEARTBEAT_CONSUMERS_MAX; ++sub) {
        err = co_od_register_internal(node,
                                      0x1016,
                                      sub,
                                      (uint8_t *)&node->hb_consumer_cfg_raw[sub - 1U],
                                      sizeof(uint32_t),
                                      CO_OD_ACCESS_READ | CO_OD_ACCESS_WRITE,
                                      NULL,
                                      co_on_hb_consumer_entry_write,
                                      NULL);
        if (err != CO_ERROR_NONE) {
            return err;
        }
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
                                      (uint8_t *)&node->rpdo_cfg[i].cob_id_raw,
                                      sizeof(uint32_t),
                                      CO_OD_ACCESS_READ | CO_OD_ACCESS_WRITE,
                                      NULL,
                                      co_on_pdo_comm_cob_id_write,
                                      NULL);
        if (err != CO_ERROR_NONE) {
            return err;
        }

        err = co_od_register_internal(node,
                                      comm_idx,
                                      0x02,
                                      &node->rpdo_cfg[i].transmission_type,
                                      sizeof(node->rpdo_cfg[i].transmission_type),
                                      CO_OD_ACCESS_READ | CO_OD_ACCESS_WRITE,
                                      NULL,
                                      NULL,
                                      NULL);
        if (err != CO_ERROR_NONE) {
            return err;
        }

        err = co_od_register_internal(node,
                                      comm_idx,
                                      0x03,
                                      (uint8_t *)&node->rpdo_cfg[i].inhibit_time_100us,
                                      sizeof(node->rpdo_cfg[i].inhibit_time_100us),
                                      CO_OD_ACCESS_READ | CO_OD_ACCESS_WRITE,
                                      NULL,
                                      NULL,
                                      NULL);
        if (err != CO_ERROR_NONE) {
            return err;
        }

        err = co_od_register_internal(node,
                                      comm_idx,
                                      0x05,
                                      (uint8_t *)&node->rpdo_cfg[i].event_timer_ms,
                                      sizeof(node->rpdo_cfg[i].event_timer_ms),
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
                                      &node->rpdo_cfg[i].map_count,
                                      sizeof(node->rpdo_cfg[i].map_count),
                                      CO_OD_ACCESS_READ | CO_OD_ACCESS_WRITE,
                                      NULL,
                                      co_on_pdo_mapping_count_write,
                                      NULL);
        if (err != CO_ERROR_NONE) {
            return err;
        }

        for (uint8_t sub = 1; sub <= CO_PDO_MAP_ENTRIES; ++sub) {
            err = co_od_register_internal(node,
                                          map_idx,
                                          sub,
                                          (uint8_t *)&node->rpdo_cfg[i].mapping_raw[sub - 1U],
                                          sizeof(uint32_t),
                                          CO_OD_ACCESS_READ | CO_OD_ACCESS_WRITE,
                                          NULL,
                                          co_on_pdo_mapping_entry_write,
                                          NULL);
            if (err != CO_ERROR_NONE) {
                return err;
            }
        }
    }

    for (uint8_t i = 0; i < CO_MAX_TPDO; ++i) {
        const uint16_t comm_idx = (uint16_t)(0x1800U + i);
        const uint16_t map_idx = (uint16_t)(0x1A00U + i);

        err = co_od_register_internal(node,
                                      comm_idx,
                                      0x00,
                                      &node->pdo_comm_sub_count[CO_MAX_RPDO + i],
                                      sizeof(node->pdo_comm_sub_count[CO_MAX_RPDO + i]),
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
                                      (uint8_t *)&node->tpdo_cfg[i].cob_id_raw,
                                      sizeof(uint32_t),
                                      CO_OD_ACCESS_READ | CO_OD_ACCESS_WRITE,
                                      NULL,
                                      co_on_pdo_comm_cob_id_write,
                                      NULL);
        if (err != CO_ERROR_NONE) {
            return err;
        }

        err = co_od_register_internal(node,
                                      comm_idx,
                                      0x02,
                                      &node->tpdo_cfg[i].transmission_type,
                                      sizeof(node->tpdo_cfg[i].transmission_type),
                                      CO_OD_ACCESS_READ | CO_OD_ACCESS_WRITE,
                                      NULL,
                                      NULL,
                                      NULL);
        if (err != CO_ERROR_NONE) {
            return err;
        }

        err = co_od_register_internal(node,
                                      comm_idx,
                                      0x03,
                                      (uint8_t *)&node->tpdo_cfg[i].inhibit_time_100us,
                                      sizeof(node->tpdo_cfg[i].inhibit_time_100us),
                                      CO_OD_ACCESS_READ | CO_OD_ACCESS_WRITE,
                                      NULL,
                                      NULL,
                                      NULL);
        if (err != CO_ERROR_NONE) {
            return err;
        }

        err = co_od_register_internal(node,
                                      comm_idx,
                                      0x05,
                                      (uint8_t *)&node->tpdo_cfg[i].event_timer_ms,
                                      sizeof(node->tpdo_cfg[i].event_timer_ms),
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
                                      &node->tpdo_cfg[i].map_count,
                                      sizeof(node->tpdo_cfg[i].map_count),
                                      CO_OD_ACCESS_READ | CO_OD_ACCESS_WRITE,
                                      NULL,
                                      co_on_pdo_mapping_count_write,
                                      NULL);
        if (err != CO_ERROR_NONE) {
            return err;
        }

        for (uint8_t sub = 1; sub <= CO_PDO_MAP_ENTRIES; ++sub) {
            err = co_od_register_internal(node,
                                          map_idx,
                                          sub,
                                          (uint8_t *)&node->tpdo_cfg[i].mapping_raw[sub - 1U],
                                          sizeof(uint32_t),
                                          CO_OD_ACCESS_READ | CO_OD_ACCESS_WRITE,
                                          NULL,
                                          co_on_pdo_mapping_entry_write,
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
    return co_send_frame(node, &frame);
}

static co_error_t co_send_bootup(co_node_t *node)
{
    co_can_frame_t frame = {
        .cob_id = CO_COB_HEARTBEAT_BASE + node->node_id,
        .len = 1U,
        .data = {0x00U}
    };
    return co_send_frame(node, &frame);
}

static void co_restore_default_comm_objects(co_node_t *node)
{
    node->sdo_server_cob_rx = CO_COB_SDO_RX_BASE + node->node_id;
    node->sdo_server_cob_tx = CO_COB_SDO_TX_BASE + node->node_id;
    node->emcy_cob_id = CO_COB_EMCY_BASE + node->node_id;
    node->emcy_valid = true;

    for (uint8_t i = 0; i < CO_MAX_RPDO; ++i) {
        node->rpdo_cfg[i].cob_id.can_id = (CO_COB_RPDO1_BASE + ((uint32_t)i * 0x100U) + node->node_id);
        node->rpdo_cfg[i].cob_id.valid = true;
        node->rpdo_cfg[i].cob_id.rtr_allowed = false;
        node->rpdo_cfg[i].cob_id.extended = false;
        node->rpdo_cfg[i].cob_id_raw = co_encode_pdo_cob_id(&node->rpdo_cfg[i].cob_id);
    }
    for (uint8_t i = 0; i < CO_MAX_TPDO; ++i) {
        node->tpdo_cfg[i].cob_id.can_id = (CO_COB_TPDO1_BASE + ((uint32_t)i * 0x100U) + node->node_id);
        node->tpdo_cfg[i].cob_id.valid = true;
        node->tpdo_cfg[i].cob_id.rtr_allowed = false;
        node->tpdo_cfg[i].cob_id.extended = false;
        node->tpdo_cfg[i].cob_id_raw = co_encode_pdo_cob_id(&node->tpdo_cfg[i].cob_id);
    }
}

static void co_schedule_bootup(co_node_t *node)
{
    node->nmt_state = CO_NMT_INITIALIZING;
    node->bootup_pending = true;
    node->last_heartbeat_ms = 0U;
    node->sync_event_pending = false;
    node->sync_last_timestamp_ms = 0U;
    node->sync_timebase_ms = 0U;
    node->sync_last_produced_ms = 0U;
    node->sync_counter = 0U;
    memset(node->tpdo_sync_tick, 0, sizeof(node->tpdo_sync_tick));
    memset(node->tpdo_last_tx_ms, 0, sizeof(node->tpdo_last_tx_ms));
    memset(node->tpdo_last_event_ms, 0, sizeof(node->tpdo_last_event_ms));
    for (uint8_t i = 0U; i < CO_HEARTBEAT_CONSUMERS_MAX; ++i) {
        co_hb_consumer_sync_from_raw(node, i);
    }
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
    return co_send_frame(node, &tx);
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
    return co_send_frame(node, &tx);
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

        const co_error_t err = co_send_frame(node, &tx);
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
            return co_send_frame(node, &tx);
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
        return co_send_frame(node, &tx);
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
            return co_send_frame(node, &tx);
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
            return co_send_frame(node, &tx);
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
        return co_send_frame(node, &tx);
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
            return co_send_frame(node, &tx);
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
            const co_error_t err = co_send_frame(node, &tx);
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
            const co_error_t err = co_send_frame(node, &tx);
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
        return co_send_frame(node, &tx);
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
        return co_send_frame(node, &tx);
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

co_error_t co_fault_raise(co_node_t *node,
                          uint8_t fault_id,
                          uint16_t emcy_code,
                          uint8_t error_register_bits,
                          uint8_t msef,
                          const uint8_t mfg_data[5])
{
    if (!node || fault_id >= CO_EMCY_FAULT_SLOTS || emcy_code == 0U) {
        return CO_ERROR_INVALID_ARGS;
    }

    const bool was_active = node->faults[fault_id].active;
    node->faults[fault_id].active = true;
    node->faults[fault_id].reg_bits = error_register_bits;
    node->faults[fault_id].emcy_code = emcy_code;
    node->faults[fault_id].msef = msef;
    memset(node->faults[fault_id].mfg_data, 0, sizeof(node->faults[fault_id].mfg_data));
    if (mfg_data) {
        memcpy(node->faults[fault_id].mfg_data, mfg_data, sizeof(node->faults[fault_id].mfg_data));
    }

    co_error_register_recompute(node);
    if (!was_active) {
        co_predef_error_push(node, emcy_code, msef);
        return co_send_emcy(node, emcy_code, msef, node->faults[fault_id].mfg_data);
    }

    return CO_ERROR_NONE;
}

co_error_t co_fault_clear(co_node_t *node,
                          uint8_t fault_id,
                          uint8_t msef,
                          const uint8_t mfg_data[5])
{
    if (!node || fault_id >= CO_EMCY_FAULT_SLOTS) {
        return CO_ERROR_INVALID_ARGS;
    }

    if (!node->faults[fault_id].active) {
        return CO_ERROR_NONE;
    }

    node->faults[fault_id].active = false;
    co_error_register_recompute(node);
    return co_send_emcy(node, 0x0000U, msef, mfg_data);
}

void co_fault_clear_all(co_node_t *node)
{
    if (!node) {
        return;
    }

    for (uint8_t i = 0U; i < CO_EMCY_FAULT_SLOTS; ++i) {
        node->faults[i].active = false;
    }
    co_error_register_recompute(node);
    (void)co_send_emcy(node, 0x0000U, 0U, NULL);
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
    bool sync_event = node->sync_event_pending;
    node->sync_event_pending = false;

    if (node->sync_valid && node->sync_producer && node->sync_cycle_period_us > 0U) {
        const uint32_t sync_period_ms = (node->sync_cycle_period_us + 999U) / 1000U;
        if ((now - node->sync_last_produced_ms) >= sync_period_ms) {
            co_can_frame_t sync_frame = {.cob_id = node->sync_cob_id, .len = 0U};
            if (node->sync_overflow > 0U) {
                node->sync_counter = (uint8_t)(node->sync_counter + 1U);
                if (node->sync_counter == 0U || node->sync_counter > node->sync_overflow) {
                    node->sync_counter = 1U;
                }
                sync_frame.len = 1U;
                sync_frame.data[0] = node->sync_counter;
            }
            const co_error_t err = co_send_frame(node, &sync_frame);
            if (err != CO_ERROR_NONE) {
                return err;
            }
            if (node->sync_last_timestamp_ms != 0U) {
                node->sync_timebase_ms = now - node->sync_last_timestamp_ms;
            }
            node->sync_last_timestamp_ms = now;
            node->sync_last_produced_ms = now;
            sync_event = true;
        }
    }
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

    for (uint8_t i = 0U; i < node->hb_consumer_sub_count; ++i) {
        const co_hb_consumer_cfg_t *cfg = &node->hb_consumers[i];
        co_hb_consumer_runtime_t *rt = &node->hb_runtime[i];
        if (!cfg->enabled || rt->last_rx_ms == 0U) {
            continue;
        }
        if ((now - rt->last_rx_ms) > cfg->timeout_ms) {
            rt->state = CO_HB_CONSUMER_TIMEOUT;
        }
    }

    const co_error_t hb_fault_err = co_hb_consumer_recompute_fault(node);
    if (hb_fault_err != CO_ERROR_NONE) {
        return hb_fault_err;
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

    if (node->nmt_state == CO_NMT_OPERATIONAL) {
        for (uint8_t i = 0; i < CO_MAX_TPDO; ++i) {
            co_pdo_cfg_t *cfg = &node->tpdo_cfg[i];
            if (!cfg->cob_id.valid || cfg->cob_id.extended || cfg->map_count == 0U) {
                continue;
            }

            const uint32_t inhibit_ms = (uint32_t)((cfg->inhibit_time_100us + 9U) / 10U);
            const bool inhibit_elapsed = (cfg->inhibit_time_100us == 0U) ||
                                         ((now - node->tpdo_last_tx_ms[i]) >= inhibit_ms);
            bool send_tpdo = false;

            if (sync_event && cfg->transmission_type > 0U && cfg->transmission_type <= 240U) {
                node->tpdo_sync_tick[i] = (uint16_t)(node->tpdo_sync_tick[i] + 1U);
                if (node->tpdo_sync_tick[i] >= cfg->transmission_type) {
                    node->tpdo_sync_tick[i] = 0U;
                    send_tpdo = true;
                }
            } else if (cfg->transmission_type >= 254U && cfg->event_timer_ms > 0U) {
                if ((now - node->tpdo_last_event_ms[i]) >= cfg->event_timer_ms) {
                    node->tpdo_last_event_ms[i] = now;
                    send_tpdo = true;
                }
            }

            if (send_tpdo && inhibit_elapsed) {
                const co_error_t err = co_send_tpdo(node, i);
                if (err != CO_ERROR_NONE) {
                    return err;
                }
                node->tpdo_last_tx_ms[i] = now;
            }
        }
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

static uint32_t co_rpdo_unpack_to_od(co_node_t *node, uint8_t rpdo_num, const co_can_frame_t *frame)
{
    if (!node || !frame || rpdo_num >= CO_MAX_RPDO) {
        return CO_SDO_ABORT_COMMAND;
    }

    co_pdo_cfg_t *cfg = &node->rpdo_cfg[rpdo_num];
    uint32_t bit_offset = 0U;
    for (uint8_t i = 0; i < cfg->map_count; ++i) {
        const co_pdo_map_entry_t *map = &cfg->mapping[i];
        if (map->bit_length == 0U) {
            continue;
        }

        co_od_entry_t *entry = co_od_find(node, map->index, map->subindex);
        if (!entry || (entry->access & CO_OD_ACCESS_WRITE) == 0U) {
            return CO_SDO_ABORT_NO_OBJECT;
        }

        uint8_t temp[CO_SDO_TRANSFER_BUF_SIZE] = {0};
        if (entry->size > sizeof(temp)) {
            return CO_SDO_ABORT_OUT_OF_MEMORY;
        }
        memcpy(temp, entry->data, entry->size);

        if (!co_copy_bits(temp,
                          entry->size * 8U,
                          0U,
                          frame->data,
                          frame->len * 8U,
                          bit_offset,
                          map->bit_length)) {
            return CO_SDO_ABORT_PARAM_LENGTH;
        }

        const uint32_t write_abort = co_od_write(node, map->index, map->subindex, temp, entry->size);
        if (write_abort != 0U) {
            return write_abort;
        }
        if (node->hooks.on_rpdo_map_written) {
            node->hooks.on_rpdo_map_written(node, rpdo_num, map->index, map->subindex, node->hooks.user);
        }
        bit_offset += map->bit_length;
    }
    return 0U;
}

static uint32_t co_tpdo_pack_from_od(co_node_t *node, uint8_t tpdo_num, co_can_frame_t *frame)
{
    if (!node || !frame || tpdo_num >= CO_MAX_TPDO) {
        return CO_SDO_ABORT_COMMAND;
    }

    co_pdo_cfg_t *cfg = &node->tpdo_cfg[tpdo_num];
    memset(frame->data, 0, sizeof(frame->data));

    uint32_t bit_offset = 0U;
    for (uint8_t i = 0; i < cfg->map_count; ++i) {
        const co_pdo_map_entry_t *map = &cfg->mapping[i];
        if (map->bit_length == 0U) {
            continue;
        }

        uint8_t temp[CO_SDO_TRANSFER_BUF_SIZE] = {0};
        size_t temp_size = 0U;
        const uint32_t read_abort =
            co_od_read(node, map->index, map->subindex, temp, sizeof(temp), &temp_size);
        if (read_abort != 0U) {
            return read_abort;
        }

        if (!co_copy_bits(frame->data,
                          sizeof(frame->data) * 8U,
                          bit_offset,
                          temp,
                          temp_size * 8U,
                          0U,
                          map->bit_length)) {
            return CO_SDO_ABORT_PARAM_LENGTH;
        }
        bit_offset += map->bit_length;
    }

    frame->len = (uint8_t)((bit_offset + 7U) / 8U);
    return 0U;
}

co_error_t co_on_can_rx(co_node_t *node, const co_can_frame_t *frame)
{
    if (!node || !frame) {
        return CO_ERROR_INVALID_ARGS;
    }
    if (frame->len > 8U) {
        return CO_ERROR_INVALID_ARGS;
    }

    if (frame->cob_id == CO_COB_NMT) {
        return co_process_nmt(node, frame);
    }

    if (frame->cob_id == (CO_COB_SDO_RX_BASE + node->node_id)) {
        return co_process_sdo(node, frame);
    }

    if (node->sync_valid && !node->sync_producer && frame->cob_id == node->sync_cob_id) {
        const uint32_t now = node->iface.millis ? node->iface.millis(node->iface.user) : 0U;
        if (frame->len > 1U) {
            return CO_ERROR_INVALID_ARGS;
        }

        if (frame->len == 1U) {
            node->sync_counter = frame->data[0];
        } else {
            node->sync_counter = (uint8_t)(node->sync_counter + 1U);
            if (node->sync_overflow > 0U && (node->sync_counter == 0U || node->sync_counter > node->sync_overflow)) {
                node->sync_counter = 1U;
            }
        }

        if (node->sync_last_timestamp_ms != 0U) {
            node->sync_timebase_ms = now - node->sync_last_timestamp_ms;
        }
        node->sync_last_timestamp_ms = now;
        node->sync_event_pending = true;
        return CO_ERROR_NONE;
    }

    if (frame->cob_id >= CO_COB_HEARTBEAT_BASE && frame->cob_id < (CO_COB_HEARTBEAT_BASE + 0x80U) && frame->len == 1U) {
        const uint8_t remote_node_id = (uint8_t)(frame->cob_id - CO_COB_HEARTBEAT_BASE);
        const uint8_t state_raw = frame->data[0];
        const co_nmt_state_t remote_state = (co_nmt_state_t)(state_raw & 0x7FU);
        const uint32_t now = node->iface.millis ? node->iface.millis(node->iface.user) : 0U;

        for (uint8_t i = 0U; i < node->hb_consumer_sub_count; ++i) {
            if (!node->hb_consumers[i].enabled || node->hb_consumers[i].node_id != remote_node_id) {
                continue;
            }
            node->hb_runtime[i].last_rx_ms = now;
            node->hb_runtime[i].remote_state = remote_state;
            node->hb_runtime[i].state = CO_HB_CONSUMER_ACTIVE;
        }
        return co_hb_consumer_recompute_fault(node);
    }

    for (uint8_t i = 0; i < CO_MAX_RPDO; ++i) {
        const co_pdo_cfg_t *cfg = &node->rpdo_cfg[i];
        if (cfg->cob_id.valid && !cfg->cob_id.extended && frame->cob_id == cfg->cob_id.can_id) {
            node->rpdo_data[i].len = frame->len;
            memcpy(node->rpdo_data[i].data, frame->data, frame->len);
            const uint32_t abort_code = co_rpdo_unpack_to_od(node, i, frame);
            return (abort_code == 0U) ? CO_ERROR_NONE : CO_ERROR_SDO_ABORT;
        }
    }

    return CO_ERROR_NONE;
}

co_error_t co_send_tpdo(co_node_t *node, uint8_t tpdo_num)
{
    if (!node || tpdo_num >= CO_MAX_TPDO) {
        return CO_ERROR_INVALID_ARGS;
    }

    if (!node->tpdo_cfg[tpdo_num].cob_id.valid || node->tpdo_cfg[tpdo_num].cob_id.extended) {
        return CO_ERROR_INVALID_ARGS;
    }

    if (node->hooks.on_tpdo_pre_tx) {
        node->hooks.on_tpdo_pre_tx(node, tpdo_num, node->hooks.user);
    }

    co_can_frame_t frame = {.cob_id = node->tpdo_cfg[tpdo_num].cob_id.can_id, .len = 0U};
    const uint32_t abort_code = co_tpdo_pack_from_od(node, tpdo_num, &frame);
    if (abort_code != 0U) {
        return CO_ERROR_SDO_ABORT;
    }
    node->tpdo_data[tpdo_num].len = frame.len;
    memcpy(node->tpdo_data[tpdo_num].data, frame.data, frame.len);
    return co_send_frame(node, &frame);
}

void co_set_hooks(co_node_t *node,
                  co_rpdo_map_written_cb_t on_rpdo_map_written,
                  co_tpdo_pre_tx_cb_t on_tpdo_pre_tx,
                  void *user)
{
    if (!node) {
        return;
    }

    node->hooks.on_rpdo_map_written = on_rpdo_map_written;
    node->hooks.on_tpdo_pre_tx = on_tpdo_pre_tx;
    node->hooks.user = user;
}
