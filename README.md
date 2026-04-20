# CANopen (CiA 301 + CiA 402) starter for STM32H7

This repository contains a compact C implementation intended as a **starting point** for CANopen nodes on STM32H7:

- CiA 301 essentials:
  - NMT state handling
  - NMT boot-up/reset lifecycle (including communication vs application reset hooks)
  - Heartbeat production
  - Expedited SDO server (upload/download)
  - RPDO receive / TPDO transmit buffers
- Structured object dictionary registration (built-ins + app extensions)
- CiA 402 essentials:
  - Basic drive state machine transitions
  - Controlword / statusword handling
  - Mode and feedback data containers
- STM32H7 integration:
  - FDCAN send/poll wrappers based on STM32 HAL

## Files

- `canopen.h`, `canopen.c`: CiA 301 core node stack.
- `cia402.h`, `cia402.c`: CiA 402 drive profile state logic.
- `canopen_stm32h7_fdcan.h`, `canopen_stm32h7_fdcan.c`: STM32H7 HAL FDCAN bindings.

## Quick integration steps

1. Add these files to your STM32CubeIDE project.
2. Ensure FDCAN is configured for classic CAN 2.0 frames (11-bit ID) and started.
3. Initialize a node once:

```c
co_node_t canopen_node;
co_stm32_ctx_t can_ctx;
co_stm32_attach(&canopen_node, &hfdcan1, &can_ctx, 0x01, 100);
```

4. Register OD variables (application extensions on top of built-in CiA 301 objects):

```c
static uint16_t statusword = 0;
co_od_add(&canopen_node, 0x6041, 0x00, (uint8_t *)&statusword, sizeof(statusword), true, false);
```

Use `co_od_add_ex` for variable-length entries, access flags, and read/write callbacks.

5. In your main loop:

```c
co_stm32_poll_rx(&canopen_node, &hfdcan1);
co_process(&canopen_node);
```

6. Update CiA 402 axis state from received controlword values and expose statusword via OD.

## Detailed STM32H7 main loop example (PDO mapping + EMCY + SYNC + guarding)

The snippet below shows a practical pattern for a **single-axis drive node** on STM32H7:

- RPDO1 receives command objects (controlword, mode, target velocity).
- TPDO1 publishes feedback (statusword, actual velocity).
- Runtime verifies that expected PDO mapping is present.
- EMCY is raised/cleared from application-detected hardware faults.
- Control/update loop is SYNC-driven (1 ms SYNC period example).
- Master supervision uses heartbeat consumer (practical replacement for legacy node guarding).
- CiA 402 is run in **profile velocity mode** (`0x6060 = 3`).
- The loop is split into fast CANopen servicing and slower control updates.

```c
#include "canopen.h"
#include "cia402.h"
#include "canopen_stm32h7_fdcan.h"

extern FDCAN_HandleTypeDef hfdcan1;

static co_node_t canopen_node;
static co_stm32_ctx_t can_ctx;
static cia402_axis_t axis;

static volatile bool inverter_fault_active = false;

static void on_rpdo_map_written(co_node_t *node, uint8_t rpdo_num, uint16_t index, uint8_t subindex, void *user)
{
    (void)node;
    (void)user;
    /* Optional: log/map-audit hook when 0x1600.. map entries change at runtime. */
    if (rpdo_num == 1U && index == 0x1600U && subindex == 0x00U) {
        /* RPDO1 map count changed. */
    }
}

static void on_tpdo_pre_tx(co_node_t *node, uint8_t tpdo_num, void *user)
{
    (void)user;
    if (tpdo_num == 1U) {
        /* Keep status/feedback coherent exactly at TPDO1 packing time. */
        cia402_set_feedback(&axis,
                            axis.position_actual,
                            axis.velocity_actual,
                            axis.torque_actual);
    }
    (void)node;
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
    /* Apply current CiA 402 command objects to your motor/inverter driver. */
    if (a->state == CIA402_OPERATION_ENABLED &&
        a->mode_of_operation == CIA402_MODE_PROFILE_VELOCITY) {
        const int32_t cmd_rpm = a->target_velocity;
        motor_set_target_velocity_rpm(cmd_rpm);
    } else {
        motor_disable_output();
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
    axis.profile_acceleration = 1000; /* rpm/s */
    axis.profile_deceleration = 1000; /* rpm/s */

    /* 4) PDO mapping is configured by the master through standard objects:
       - RPDO1 comm/map: 0x1400 / 0x1600
       - TPDO1 comm/map: 0x1800 / 0x1A00
       Typical map for profile velocity drive:
       RPDO1: 0x6040:00 (16), 0x6060:00 (8), 0x60FF:00 (32)
       TPDO1: 0x6041:00 (16), 0x606C:00 (32)
    */

    /* 5) Optional local default mapping (before/without external SDO config). */
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

    /* 6) SYNC-driven timing: consumer on COB-ID 0x80 with 1 ms cycle. */
    const uint32_t sync_cob_id = 0x00000080UL; /* valid, consumer, std-id */
    const uint32_t sync_cycle_us = 1000UL;
    (void)co_od_write(&canopen_node, 0x1005, 0x00, (const uint8_t *)&sync_cob_id, sizeof(sync_cob_id));
    (void)co_od_write(&canopen_node, 0x1006, 0x00, (const uint8_t *)&sync_cycle_us, sizeof(sync_cycle_us));
    const uint8_t tpdo1_sync_type = 1U; /* every SYNC */
    (void)co_od_write(&canopen_node, 0x1800, 0x02, &tpdo1_sync_type, sizeof(tpdo1_sync_type));

    /* 7) Guarding/supervision: configure heartbeat consumer for master node 0x7E, 500 ms timeout. */
    const uint8_t hb_consumer_count = 1U;
    const uint32_t hb_master = (0x7EUL << 16) | 500UL; /* 0x1016:01 */
    (void)co_od_write(&canopen_node, 0x1016, 0x00, &hb_consumer_count, sizeof(hb_consumer_count));
    (void)co_od_write(&canopen_node, 0x1016, 0x01, (const uint8_t *)&hb_master, sizeof(hb_master));
}

void app_main_loop(void)
{
    for (;;) {
        /* Fast path: drain RX FIFO and process protocol state machine. */
        co_stm32_poll_rx(&canopen_node, &hfdcan1);
        co_process(&canopen_node);

        /* Run control update on every SYNC event (1 ms from master in this example). */
        if (canopen_node.sync_event_pending) {
            canopen_node.sync_event_pending = false;

            /* Feedback objects are written by your application/ISR:
               axis.position_actual, axis.velocity_actual, axis.torque_actual. */

            /* Apply latest RPDO-written controlword and mode commands. */
            cia402_apply_controlword(&axis, axis.controlword);
            cia402_step(&axis);

            /* Run hardware command side according to CiA 402 state/mode. */
            app_drive_hw_step(&axis);

            /* Raise/clear EMCY faults from application hardware diagnostics. */
            app_fault_monitor_step();
        }

        /* Optional: low-power idle until next interrupt/event. */
    }
}
```

### Why this loop structure works

- `co_stm32_poll_rx()` + `co_process()` runs every iteration to keep SDO/PDO/NMT/heartbeat timing responsive.
- SYNC edges gate the control law, so command/feedback exchange is aligned to network time.
- Explicit local PDO mapping defaults make bring-up deterministic before external SDO tooling is added.
- Heartbeat consumer (`0x1016`) provides master supervision and drives communication EMCY on timeout.
- EMCY fault hooks (`co_fault_raise` / `co_fault_clear`) cleanly connect hardware diagnostics to CANopen diagnostics.

## Notes

- This is intentionally minimal and deterministic for embedded use.
- For production, add:
  - segmented/block SDO,
  - PDO mapping objects (0x1600/0x1A00),
  - EMCY/error register and fault logging,
  - SYNC-driven PDO timing,
  - node guarding / heartbeat consumer,
  - conformance test coverage.


## Host-side deterministic test harness

A host-side harness is provided to exercise `canopen.c` and `cia402.c` without MCU dependencies:

- `tests_host.c` uses:
  - deterministic fake CAN send callback (captures all transmitted frames in-order),
  - deterministic fake clock (`millis`) controlled by test code,
  - reset hook counters for communication/application reset coverage.
- Build and run:

```sh
make test
```

This compiles `tests_host` from `tests_host.c`, `canopen.c`, and `cia402.c`, then executes all tests.

## Minimal conformance checklist (implemented vs gaps)

### Implemented and covered by host tests

- **NMT command handling and node lifecycle**
  - NMT start/stop/pre-operational command routing (broadcast and node-targeted).
  - Boot-up frame emission and reset-triggered re-initializing/boot-up flow.
  - Communication reset vs application reset callback behavior.
- **SDO server transfer paths**
  - Expedited upload/download success.
  - Segmented upload/download success.
  - Block upload/download success.
  - Abort paths: unknown object, toggle/sequence mismatch, invalid command handling.
- **PDO mapping and runtime data exchange**
  - Mapping validation failures (e.g., unmapped OD object).
  - RPDO runtime unpack into OD variables.
  - TPDO runtime pack from OD variables.
- **Heartbeat + EMCY behavior**
  - Heartbeat periodic timing against fake clock.
  - EMCY emit/clear behavior via fault raise/clear.
- **CiA 402 state and statusword**
  - Core transition matrix coverage (shutdown/switch on/enable op/quick stop/fault reset).
  - Statusword low-bit correctness for key states.
  - Profile-fault to EMCY propagation on unsupported-mode fault path.

### Known gaps (not yet covered or intentionally minimal)

- Full CiA 301 certification profile coverage is not claimed (this is a compact starter stack).
- SDO block transfer edge cases (CRC variants, retransmit corner cases, malformed-frame fuzzing) are not exhaustively tested.
- TPDO timing matrix is only partially covered in host tests (no exhaustive SYNC type/inhibit/event combinations).
- Heartbeat consumer is implemented, but legacy RTR-based node guarding protocol is not implemented.
- Advanced CiA 402 operation-mode semantics (trajectory/profile constraints, hardware interlocks, mode-specific diagnostics) remain application-specific.
- No automated hardware-in-loop conformance run is included.
