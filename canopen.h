#ifndef CANOPEN_H
#define CANOPEN_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CO_MAX_OD_ENTRIES 192U
#define CO_MAX_RPDO 4U
#define CO_MAX_TPDO 4U
#define CO_PDO_MAP_ENTRIES 8U
#define CO_SDO_CHANNELS 2U
#define CO_SDO_TRANSFER_BUF_SIZE 1024U
#define CO_EMCY_ERROR_HISTORY_LEN 8U
#define CO_EMCY_FAULT_SLOTS 16U
#define CO_HEARTBEAT_CONSUMERS_MAX 16U
#define CO_EMCY_ERR_CAN_TX 0x8110U
#define CO_EMCY_ERR_HEARTBEAT_CONSUMER 0x8130U
#define CO_EMCY_ERR_PROFILE 0xFF10U

#define CO_SDO_ABORT_TOGGLE 0x05030000UL
#define CO_SDO_ABORT_COMMAND 0x05040001UL
#define CO_SDO_ABORT_TIMEOUT 0x05040000UL
#define CO_SDO_ABORT_OUT_OF_MEMORY 0x05040005UL
#define CO_SDO_ABORT_READONLY 0x06010002UL
#define CO_SDO_ABORT_WRITEONLY 0x06010001UL
#define CO_SDO_ABORT_NO_OBJECT 0x06020000UL
#define CO_SDO_ABORT_NO_SUBINDEX 0x06090011UL
/* Data-type / length mismatch codes (0x0607xxxx) */
#define CO_SDO_ABORT_PARAM_LENGTH 0x06070010UL
#define CO_SDO_ABORT_PARAM_TOO_HIGH 0x06070012UL
#define CO_SDO_ABORT_PARAM_TOO_LOW 0x06070013UL
/* Value-range codes (0x0609xxxx) — use these for out-of-range parameter values */
#define CO_SDO_ABORT_VALUE_RANGE    0x06090030UL
#define CO_SDO_ABORT_VALUE_TOO_HIGH 0x06090031UL
#define CO_SDO_ABORT_VALUE_TOO_LOW  0x06090032UL

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
    CO_ERROR_REG_GENERIC = 0x01U,
    CO_ERROR_REG_CURRENT = 0x02U,
    CO_ERROR_REG_VOLTAGE = 0x04U,
    CO_ERROR_REG_TEMPERATURE = 0x08U,
    CO_ERROR_REG_COMMUNICATION = 0x10U,
    CO_ERROR_REG_DEVICE_PROFILE = 0x20U,
    CO_ERROR_REG_RESERVED = 0x40U,
    CO_ERROR_REG_MANUFACTURER = 0x80U
} co_error_reg_bits_t;

typedef enum {
    CO_FAULT_CAN_TX = 0U,
    CO_FAULT_CIA402_PROFILE = 1U,
    CO_FAULT_HEARTBEAT_CONSUMER = 2U
} co_fault_id_t;

typedef enum {
    CO_HB_CONSUMER_DISABLED = 0U,
    CO_HB_CONSUMER_UNKNOWN = 1U,
    CO_HB_CONSUMER_ACTIVE = 2U,
    CO_HB_CONSUMER_TIMEOUT = 3U
} co_hb_consumer_state_t;

typedef struct {
    uint8_t node_id;
    uint16_t timeout_ms;
    bool enabled;
} co_hb_consumer_cfg_t;

typedef struct {
    co_hb_consumer_state_t state;
    co_nmt_state_t remote_state;
    uint32_t last_rx_ms;
} co_hb_consumer_runtime_t;

typedef enum {
    CO_OD_ACCESS_READ = 1U << 0,
    CO_OD_ACCESS_WRITE = 1U << 1
} co_od_access_t;

struct co_node;
typedef struct co_node co_node_t;
typedef struct co_od_entry co_od_entry_t;
typedef void (*co_rpdo_map_written_cb_t)(co_node_t *node, uint8_t rpdo_num, uint16_t index, uint8_t subindex, void *user);
typedef void (*co_rpdo_frame_written_cb_t)(co_node_t *node, uint8_t rpdo_num, void *user);
typedef void (*co_tpdo_pre_tx_cb_t)(co_node_t *node, uint8_t tpdo_num, void *user);

typedef enum {
    CO_HB_EVENT_TIMEOUT = 0,
    CO_HB_EVENT_RECOVERED = 1
} co_hb_event_t;

typedef void (*co_hb_consumer_event_cb_t)(co_node_t *node, uint8_t slave_node_id,
                                          co_hb_event_t event, void *user);

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
    uint16_t index;
    uint8_t subindex;
    uint8_t bit_length;
} co_pdo_map_entry_t;

typedef struct {
    uint32_t can_id;
    bool valid;
    bool rtr_allowed;
    bool extended;
} co_pdo_cob_id_cfg_t;

typedef struct {
    uint32_t cob_id_raw;
    co_pdo_cob_id_cfg_t cob_id;
    uint8_t transmission_type;
    uint16_t inhibit_time_100us;
    uint16_t event_timer_ms;
    uint8_t map_count;
    uint32_t mapping_raw[CO_PDO_MAP_ENTRIES];
    co_pdo_map_entry_t mapping[CO_PDO_MAP_ENTRIES];
} co_pdo_cfg_t;

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
    void (*on_reset_communication)(void *user);
    void (*on_reset_application)(void *user);
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

    co_pdo_data_t rpdo_data[CO_MAX_RPDO];
    co_pdo_data_t tpdo_data[CO_MAX_TPDO];
    co_pdo_cfg_t rpdo_cfg[CO_MAX_RPDO];
    co_pdo_cfg_t tpdo_cfg[CO_MAX_TPDO];
    uint32_t tpdo_last_tx_ms[CO_MAX_TPDO];
    uint32_t tpdo_last_event_ms[CO_MAX_TPDO];
    uint16_t tpdo_sync_tick[CO_MAX_TPDO];

    uint32_t sync_cob_id_raw;
    uint16_t sync_cob_id;
    uint32_t sync_cycle_period_us;
    uint32_t sync_last_timestamp_ms;
    uint32_t sync_timebase_ms;
    uint32_t sync_last_produced_ms;
    uint8_t sync_counter;
    uint8_t sync_overflow;
    bool sync_valid;
    bool sync_producer;
    bool sync_event_pending;   /* sticky flag for application — cleared by caller */
    bool sync_tpdo_pending;    /* internal: cleared by co_process() after TPDO scheduling */

    uint32_t device_type;
    uint8_t error_register;
    uint8_t predef_error_count;
    uint32_t predef_error_field[CO_EMCY_ERROR_HISTORY_LEN];
    uint32_t emcy_cob_id;
    bool emcy_valid;
    struct {
        bool active;
        uint8_t reg_bits;
        uint16_t emcy_code;
        uint8_t msef;
        uint8_t mfg_data[4];   /* bytes 4-7 of EMCY frame; msef occupies byte 3 */
    } faults[CO_EMCY_FAULT_SLOTS];
    uint8_t identity_sub_count;
    uint32_t identity[4];
    uint32_t sdo_server_cob_rx[CO_SDO_CHANNELS];
    uint32_t sdo_server_cob_tx[CO_SDO_CHANNELS];
    uint8_t sdo_server_sub_count[CO_SDO_CHANNELS];
    uint8_t pdo_comm_sub_count[CO_MAX_RPDO + CO_MAX_TPDO];
    uint8_t hb_consumer_sub_count;
    uint32_t hb_consumer_cfg_raw[CO_HEARTBEAT_CONSUMERS_MAX];
    co_hb_consumer_cfg_t hb_consumers[CO_HEARTBEAT_CONSUMERS_MAX];
    co_hb_consumer_runtime_t hb_runtime[CO_HEARTBEAT_CONSUMERS_MAX];
    struct {
        uint8_t state;
        uint8_t mode;
        uint8_t toggle;
        uint8_t block_seq;
        uint8_t block_blksize;
        uint8_t block_last_sent_seq;
        uint8_t block_finished;
        uint16_t index;
        uint8_t subindex;
        size_t offset;
        size_t size;
        uint32_t last_activity_ms;
        uint8_t buffer[CO_SDO_TRANSFER_BUF_SIZE];
    } sdo_channels[CO_SDO_CHANNELS];

    struct {
        co_rpdo_map_written_cb_t  on_rpdo_map_written;   /* per-object (legacy) */
        co_rpdo_frame_written_cb_t on_rpdo_frame_written; /* per-frame (preferred) */
        co_tpdo_pre_tx_cb_t       on_tpdo_pre_tx;
        co_hb_consumer_event_cb_t on_hb_event;
        void *user;
    } hooks;
};

void co_init(co_node_t *node, const co_if_t *iface, uint8_t node_id, uint16_t heartbeat_ms);
co_error_t co_od_add(co_node_t *node,
                     uint16_t index,
                     uint8_t subindex,
                     uint8_t *data,
                     size_t size,
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
void co_set_hooks(co_node_t *node,
                  co_rpdo_map_written_cb_t on_rpdo_map_written,
                  co_tpdo_pre_tx_cb_t on_tpdo_pre_tx,
                  void *user);
void co_set_rpdo_frame_hook(co_node_t *node, co_rpdo_frame_written_cb_t on_rpdo_frame_written, void *user);
void co_set_hb_event_hook(co_node_t *node, co_hb_consumer_event_cb_t on_hb_event, void *user);
void co_nmt_set_state(co_node_t *node, co_nmt_state_t state);
co_error_t co_nmt_master_send(co_node_t *node, uint8_t command, uint8_t target_node_id);
co_error_t co_fault_raise(co_node_t *node,
                          uint8_t fault_id,
                          uint16_t emcy_code,
                          uint8_t error_register_bits,
                          uint8_t msef,
                          const uint8_t mfg_data[4]);
co_error_t co_fault_clear(co_node_t *node,
                          uint8_t fault_id,
                          uint8_t msef,
                          const uint8_t mfg_data[4]);
void co_fault_clear_all(co_node_t *node);

#ifdef __cplusplus
}
#endif

#endif
