#ifndef CANOPEN_H
#define CANOPEN_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CO_MAX_OD_ENTRIES 128U
#define CO_MAX_RPDO 4U
#define CO_MAX_TPDO 4U

typedef enum {
    CO_NMT_INITIALIZING = 0,
    CO_NMT_PRE_OPERATIONAL = 127,
    CO_NMT_OPERATIONAL = 5,
    CO_NMT_STOPPED = 4
} co_nmt_state_t;

typedef enum {
    CO_ERROR_NONE = 0,
    CO_ERROR_INVALID_ARGS,
    CO_ERROR_OD_FULL,
    CO_ERROR_OD_NOT_FOUND,
    CO_ERROR_SDO_ABORT,
    CO_ERROR_HW
} co_error_t;

typedef struct {
    uint16_t index;
    uint8_t subindex;
    uint8_t size;
    uint8_t *data;
    bool readable;
    bool writable;
} co_od_entry_t;

typedef struct {
    uint8_t len;
    uint8_t data[8];
} co_pdo_data_t;

typedef struct {
    uint8_t command;
    uint16_t index;
    uint8_t subindex;
    uint8_t payload[4];
} co_sdo_req_t;

typedef struct {
    uint32_t cob_id;
    uint8_t len;
    uint8_t data[8];
} co_can_frame_t;

typedef struct {
    void *user;
    co_error_t (*send)(void *user, const co_can_frame_t *frame);
    uint32_t (*millis)(void *user);
} co_if_t;

typedef struct {
    uint8_t node_id;
    uint16_t heartbeat_ms;
    co_if_t iface;

    co_nmt_state_t nmt_state;
    uint32_t last_heartbeat_ms;

    co_od_entry_t od[CO_MAX_OD_ENTRIES];
    size_t od_count;

    uint32_t rpdo_map[CO_MAX_RPDO];
    uint32_t tpdo_map[CO_MAX_TPDO];
    co_pdo_data_t rpdo_data[CO_MAX_RPDO];
    co_pdo_data_t tpdo_data[CO_MAX_TPDO];
} co_node_t;

void co_init(co_node_t *node, const co_if_t *iface, uint8_t node_id, uint16_t heartbeat_ms);
co_error_t co_od_add(co_node_t *node,
                     uint16_t index,
                     uint8_t subindex,
                     uint8_t *data,
                     uint8_t size,
                     bool readable,
                     bool writable);
co_error_t co_process(co_node_t *node);
co_error_t co_on_can_rx(co_node_t *node, const co_can_frame_t *frame);
co_error_t co_send_tpdo(co_node_t *node, uint8_t tpdo_num);

#ifdef __cplusplus
}
#endif

#endif
