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
#define CO_PDO_MAP_ENTRIES 8U

#define CO_SDO_ABORT_TOGGLE 0x05030000UL
#define CO_SDO_ABORT_COMMAND 0x05040001UL
#define CO_SDO_ABORT_READONLY 0x06010002UL
#define CO_SDO_ABORT_WRITEONLY 0x06010001UL
#define CO_SDO_ABORT_NO_OBJECT 0x06020000UL
#define CO_SDO_ABORT_NO_SUBINDEX 0x06090011UL
#define CO_SDO_ABORT_PARAM_LENGTH 0x06070010UL
#define CO_SDO_ABORT_PARAM_TOO_HIGH 0x06070012UL
#define CO_SDO_ABORT_PARAM_TOO_LOW 0x06070013UL

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

typedef enum {
    CO_OD_ACCESS_READ = 1U << 0,
    CO_OD_ACCESS_WRITE = 1U << 1
} co_od_access_t;

struct co_node;
typedef struct co_node co_node_t;
typedef struct co_od_entry co_od_entry_t;

typedef uint32_t (*co_od_read_cb_t)(co_node_t *node, const co_od_entry_t *entry, void *user);
typedef uint32_t (*co_od_write_cb_t)(co_node_t *node,
                                     const co_od_entry_t *entry,
                                     const uint8_t *data,
                                     size_t size,
                                     void *user);

struct co_od_entry {
    uint16_t index;
    uint8_t subindex;
    size_t size;
    uint8_t *data;
    uint8_t access;
    co_od_read_cb_t on_read;
    co_od_write_cb_t on_write;
    void *user;
};

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

typedef enum {
    CO_RESET_COMMUNICATION = 0,
    CO_RESET_APPLICATION = 1
} co_reset_type_t;

typedef struct {
    void *user;
    co_error_t (*send)(void *user, const co_can_frame_t *frame);
    uint32_t (*millis)(void *user);
    void (*on_reset)(void *user, co_reset_type_t type);
} co_if_t;

struct co_node {
    uint8_t node_id;
    uint16_t heartbeat_ms;
    co_if_t iface;

    co_nmt_state_t nmt_state;
    uint32_t last_heartbeat_ms;
    bool bootup_pending;

    co_od_entry_t od[CO_MAX_OD_ENTRIES];
    size_t od_count;

    uint32_t rpdo_map[CO_MAX_RPDO];
    uint32_t tpdo_map[CO_MAX_TPDO];
    co_pdo_data_t rpdo_data[CO_MAX_RPDO];
    co_pdo_data_t tpdo_data[CO_MAX_TPDO];

    uint32_t device_type;
    uint8_t error_register;
    uint8_t identity_sub_count;
    uint32_t identity[4];
    uint32_t sdo_server_cob_rx;
    uint32_t sdo_server_cob_tx;
    uint8_t sdo_server_sub_count;
    uint8_t pdo_comm_sub_count[CO_MAX_RPDO + CO_MAX_TPDO];
    uint8_t pdo_map_count[CO_MAX_RPDO + CO_MAX_TPDO];
    uint32_t rpdo_transmission_type[CO_MAX_RPDO];
    uint32_t tpdo_transmission_type[CO_MAX_TPDO];
    uint32_t rpdo_mapping[CO_MAX_RPDO][CO_PDO_MAP_ENTRIES];
    uint32_t tpdo_mapping[CO_MAX_TPDO][CO_PDO_MAP_ENTRIES];
};

void co_init(co_node_t *node, const co_if_t *iface, uint8_t node_id, uint16_t heartbeat_ms);
co_error_t co_od_add(co_node_t *node,
                     uint16_t index,
                     uint8_t subindex,
                     uint8_t *data,
                     uint8_t size,
                     bool readable,
                     bool writable);
co_error_t co_od_add_ex(co_node_t *node,
                        uint16_t index,
                        uint8_t subindex,
                        uint8_t *data,
                        size_t size,
                        uint8_t access,
                        co_od_read_cb_t on_read,
                        co_od_write_cb_t on_write,
                        void *user);
uint32_t co_od_read(co_node_t *node,
                    uint16_t index,
                    uint8_t subindex,
                    uint8_t *dst,
                    size_t dst_size,
                    size_t *out_size);
uint32_t co_od_write(co_node_t *node,
                     uint16_t index,
                     uint8_t subindex,
                     const uint8_t *src,
                     size_t src_size);
co_error_t co_process(co_node_t *node);
co_error_t co_on_can_rx(co_node_t *node, const co_can_frame_t *frame);
co_error_t co_send_tpdo(co_node_t *node, uint8_t tpdo_num);

#ifdef __cplusplus
}
#endif

#endif
