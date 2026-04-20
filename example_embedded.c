#include "canopen.h"
#include "cia402.h"
#include "canopen_stm32h7_fdcan.h"

extern FDCAN_HandleTypeDef hfdcan1;

static co_node_t canopen_node;
static co_stm32_ctx_t can_ctx;
static cia402_axis_t axis;

static volatile int32_t velocity_feedback_rpm = 0;
static volatile bool inverter_fault_active = false;
static volatile uint8_t sync_tick = 0;

static void on_rpdo_map_written(co_node_t *node, uint8_t rpdo_num, uint16_t index, uint8_t subindex, void *user)
{
    (void)node;
    (void)user;

    /* Optional map-audit hook for runtime remapping over SDO. */
    if (rpdo_num == 1U && index == 0x1600U && subindex == 0x00U) {
        /* RPDO1 map count changed. */
    }
}

static void on_tpdo_pre_tx(co_node_t *node, uint8_t tpdo_num, void *user)
{
    (void)node;
    (void)user;

    if (tpdo_num == 1U) {
        /* Keep status/feedback coherent exactly at TPDO1 packing time. */
        cia402_set_feedback(&axis,
                            axis.position_actual,
                            axis.velocity_actual,
                            axis.torque_actual);
    }
}

static void app_fault_monitor_step(void)
{
    if (inverter_fault_active) {
        (void)co_fault_raise(&canopen_node,
                             CO_FAULT_CIA402_PROFILE,
                             CO_EMCY_ERR_PROFILE,
                             CO_ERROR_REG_DEVICE_PROFILE,
                             0x01U,
                             NULL);
    } else {
        (void)co_fault_clear(&canopen_node, CO_FAULT_CIA402_PROFILE, 0x00U, NULL);
    }
}

static void app_drive_hw_step(cia402_axis_t *a)
{
    /* Example: convert target velocity command to inverter units and apply it. */
    if (a->state == CIA402_OPERATION_ENABLED &&
        a->mode_of_operation == CIA402_MODE_PROFILE_VELOCITY) {
        const int32_t cmd_rpm = a->target_velocity;
        (void)cmd_rpm;
        /* TODO: set motor/inverter target speed from cmd_rpm. */
    } else {
        /* TODO: command safe torque-off / zero output. */
    }
}

void app_canopen_init(void)
{
    /* 1) Attach CANopen stack to STM32H7 FDCAN. */
    co_stm32_attach(&canopen_node, &hfdcan1, &can_ctx, 0x01, 100);

    /* 2) Initialize and bind CiA 402 axis OD entries (0x6040/41, 0x6060/61, etc). */
    cia402_init(&axis);
    cia402_attach_node(&axis, &canopen_node);
    cia402_bind_od(&axis, &canopen_node);
    co_set_hooks(&canopen_node, on_rpdo_map_written, on_tpdo_pre_tx, NULL);

    /* 3) Optional defaults before master takes control via RPDO/SDO. */
    axis.mode_of_operation = CIA402_MODE_PROFILE_VELOCITY;
    axis.profile_acceleration = 1000;
    axis.profile_deceleration = 1000;

    /* 4) Local PDO mapping defaults (can be overwritten later by SDO). */
    const uint32_t rpdo1_map1 = (0x6040UL << 16) | (0x00UL << 8) | 16UL;
    const uint32_t rpdo1_map2 = (0x6060UL << 16) | (0x00UL << 8) | 8UL;
    const uint32_t rpdo1_map3 = (0x60FFUL << 16) | (0x00UL << 8) | 32UL;
    const uint32_t tpdo1_map1 = (0x6041UL << 16) | (0x00UL << 8) | 16UL;
    const uint32_t tpdo1_map2 = (0x606CUL << 16) | (0x00UL << 8) | 32UL;

    (void)co_od_write(&canopen_node, 0x1600, 0x00, (const uint8_t[]){0}, 1);
    (void)co_od_write(&canopen_node, 0x1600, 0x01, (const uint8_t *)&rpdo1_map1, sizeof(rpdo1_map1));
    (void)co_od_write(&canopen_node, 0x1600, 0x02, (const uint8_t *)&rpdo1_map2, sizeof(rpdo1_map2));
    (void)co_od_write(&canopen_node, 0x1600, 0x03, (const uint8_t *)&rpdo1_map3, sizeof(rpdo1_map3));
    (void)co_od_write(&canopen_node, 0x1600, 0x00, (const uint8_t[]){3}, 1);

    (void)co_od_write(&canopen_node, 0x1A00, 0x00, (const uint8_t[]){0}, 1);
    (void)co_od_write(&canopen_node, 0x1A00, 0x01, (const uint8_t *)&tpdo1_map1, sizeof(tpdo1_map1));
    (void)co_od_write(&canopen_node, 0x1A00, 0x02, (const uint8_t *)&tpdo1_map2, sizeof(tpdo1_map2));
    (void)co_od_write(&canopen_node, 0x1A00, 0x00, (const uint8_t[]){2}, 1);

    /* 5) SYNC-driven timing: consume 0x80 SYNC with 1ms cycle. */
    const uint32_t sync_cob_id = 0x00000080UL;
    const uint32_t sync_cycle_us = 1000UL;
    const uint8_t tpdo1_sync_type = 1U;

    (void)co_od_write(&canopen_node, 0x1005, 0x00, (const uint8_t *)&sync_cob_id, sizeof(sync_cob_id));
    (void)co_od_write(&canopen_node, 0x1006, 0x00, (const uint8_t *)&sync_cycle_us, sizeof(sync_cycle_us));
    (void)co_od_write(&canopen_node, 0x1800, 0x02, &tpdo1_sync_type, sizeof(tpdo1_sync_type));

    /* 6) Node guarding equivalent: heartbeat consumer for master 0x7E, timeout 500 ms. */
    const uint8_t hb_consumer_count = 1U;
    const uint32_t hb_master = (0x7EUL << 16) | 500UL;

    (void)co_od_write(&canopen_node, 0x1016, 0x00, &hb_consumer_count, sizeof(hb_consumer_count));
    (void)co_od_write(&canopen_node, 0x1016, 0x01, (const uint8_t *)&hb_master, sizeof(hb_master));
}

void app_main_loop(void)
{
    for (;;) {
        /* Fast path: keep SDO/PDO/NMT/heartbeat responsive. */
        co_stm32_poll_rx(&canopen_node, &hfdcan1);
        co_process(&canopen_node);

        /* Drive control loop off each SYNC edge. */
        if (canopen_node.sync_event_pending) {
            canopen_node.sync_event_pending = false;
            ++sync_tick;

            axis.velocity_actual = velocity_feedback_rpm;
            axis.position_actual += axis.velocity_actual / 60;

            cia402_apply_controlword(&axis, axis.controlword);
            cia402_step(&axis);

            app_drive_hw_step(&axis);
            app_fault_monitor_step();

            if ((sync_tick % 10U) == 0U) {
                (void)co_send_tpdo(&canopen_node, 1U);
            }
        }
    }
}
