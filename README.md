# CANopen CiA 301 + CiA 402 Master Stack

A compact, production-oriented CANopen **master** implementation in C for STM32 microcontrollers. The master controls one or more CiA 402 Profile Velocity servo drives (ZeroErr eRob, Delta ASDA) across one or more independent CAN buses. The MCU abstraction layer keeps application code free of `#ifdef` guards for different STM32 families.

---

## What is implemented

### CiA 301 Core (`canopen.h` / `canopen.c`)

- NMT state machine (Initializing → Pre-Operational → Operational → Stopped) with master send
- Boot-up frame emission; communication reset and application reset hooks
- Heartbeat producer (configurable period) and heartbeat consumer (up to `CO_HEARTBEAT_CONSUMERS_MAX` slaves)
- Expedited, segmented, and block SDO server (upload and download)
- RPDO receive with OD unpack; TPDO transmit with OD pack
- SYNC producer/consumer with configurable COB-ID and cycle period
- Emergency (EMCY) producer with fault raise / clear / clear-all API
- Object dictionary with up to `CO_MAX_OD_ENTRIES` entries, optional read/write callbacks

### CiA 402 Drive Profile (`cia402.h` / `cia402.c`)

- Full CiA 402 state machine (Not Ready → Switch-On Disabled → Ready-To-Switch-On → Switched-On → Operation Enabled → Quick Stop → Fault Reaction → Fault)
- Controlword / statusword handling; mode-of-operation and mode-display
- Profile velocity, profile position, profile torque, homing, CSP/CSV/CST containers
- Velocity window monitoring (0x606D / 0x606E) for target-reached detection
- Fault → EMCY propagation

### Master Application (`example_embedded.c` / `example_embedded.h`)

- Multi-slave, multi-bus CANopen master
- Automatic SDO configuration sequence for each slave on first connect
- Per-slave CiA 402 state machine driven by RPDO feedback
- Runtime SDO write queue (`cia402_set_accel`) with timeout and abort detection
- RPDO watchdog: Quick Stop if no RPDO received within `CIA402_RPDO_WATCHDOG_MS`
- PRE-OP debounce: transient heartbeat loss does not trigger full re-init
- Structured diagnostic callback covering all fault/state-change events
- DEBUG-mode per-slave CiA 402 transition ring buffer

### MCU Port Layer (`canopen_port.h`)

- Unified API wrapping STM32H7 FDCAN and STM32F4 bxCAN
- ISR→main-loop SPSC ring buffer; no mutex required on single-core Cortex-M
- `CO_PORT_DEFINE_RX_ISR` macro generates the HAL callback for any number of buses

---

## File map

| File | Purpose |
|------|---------|
| `canopen.h` / `canopen.c` | CiA 301 protocol core |
| `cia402.h` / `cia402.c` | CiA 402 drive profile state machine |
| `canopen_port.h` | MCU abstraction — selects H7, G4, or F4 driver |
| `canopen_stm32h7_fdcan.h` / `.c` | STM32H7 FDCAN HAL bindings |
| `canopen_stm32g4_fdcan.h` / `.c` | STM32G4 FDCAN HAL bindings |
| `canopen_stm32f4_can.h` / `.c` | STM32F4 bxCAN HAL bindings |
| `example_embedded.h` | Public master API — node config table, diagnostic types |
| `example_embedded.c` | Master implementation — SDO sequencer, CiA 402 FSM, loop |
| `tests_host.c` | Host-side deterministic test harness |

---

## MCU target selection

Define exactly one of the following symbols before including any project header, or add it as a compiler flag (`-DSTM32H723xx`):

| Symbol | Peripheral | HAL type | HAL include |
|--------|-----------|---------|-------------|
| `STM32H723xx` | FDCAN | `FDCAN_HandleTypeDef` | `stm32h7xx_hal.h` |
| `STM32G474xx` (or any G4 variant — see below) | FDCAN | `FDCAN_HandleTypeDef` | `stm32g4xx_hal.h` |
| `STM32F423xx` | bxCAN | `CAN_HandleTypeDef` | `stm32f4xx_hal.h` |

Supported STM32G4 variants: `STM32G431xx`, `STM32G441xx`, `STM32G471xx`, `STM32G473xx`, `STM32G474xx`, `STM32G483xx`, `STM32G484xx`, `STM32G491xx`, `STM32G4A1xx`.

`canopen_port.h` reads this symbol and transparently maps `co_port_handle_t`, `co_port_ctx_t`, `co_port_attach()`, `co_port_drain_rx()`, `co_port_rx_isr()`, and `CO_PORT_DEFINE_RX_ISR()` to the correct HAL calls. No `#ifdef` is needed anywhere else in the project.

---

## Integration — step by step

### 1. Add files to your project

Add all `.c` and `.h` files to your STM32CubeIDE (or CMake) project. Ensure the MCU symbol is defined (see above).

### 2. Define `CIA402_MAX_NODES` in `app_config.h`

```c
// app_config.h
#define CIA402_MAX_NODES  4U   // upper bound on total slave count across all buses
```

This value sizes all static arrays inside `example_embedded.c`. Raise it together with `CO_MAX_RPDO` / `CO_MAX_TPDO` in `canopen.h` when adding more slaves (each slave consumes 2 RPDOs and 2 TPDOs on its bus).

### 3. Declare the node configuration table

Provide `cia402_nodes[]` and `cia402_node_count` in any translation unit:

```c
#include "example_embedded.h"

// STM32H7 example — two buses, three slaves
extern FDCAN_HandleTypeDef hfdcan1, hfdcan2;

const cia402_cfg_t cia402_nodes[] = {
    //  node_id  handle    accel(c/s²)  decel(c/s²)  CPR        ratio  driver
    { 0x01U, &hfdcan1,  50000U,      200000U,      524288U,   1U,    DRIVER_ZEROERR },
    { 0x02U, &hfdcan1,  50000U,      200000U,      524288U,   5U,    DRIVER_ZEROERR },
    { 0x05U, &hfdcan2,  30000U,      100000U,      16777216U, 1U,    DRIVER_DELTA   },
};
const uint8_t cia402_node_count =
    (uint8_t)(sizeof(cia402_nodes) / sizeof(cia402_nodes[0]));
```

Nodes sharing the same `handle` share one CAN bus and one master `co_node_t`. Nodes with different handles get independent buses, independent SYNC generators, and independent heartbeat consumers. The bus pool is built automatically at init time — no extra setup is needed.

#### `cia402_cfg_t` fields

| Field | Type | Description |
|-------|------|-------------|
| `node_id` | `uint8_t` | CANopen node ID of the slave (1–127) |
| `can_handle` | `co_port_handle_t` | Pointer to the HAL peripheral handle the slave is wired to (`&hfdcan1`, `&hcan1`, …) |
| `default_accel` | `uint32_t` | Acceleration ramp written to 0x6083 once at startup [count/s²] |
| `default_decel` | `uint32_t` | Deceleration ramp written to 0x6084 once at startup [count/s²] |
| `encoder_counts_per_rev` | `uint32_t` | Raw encoder CPR (counts per motor revolution). Examples: 524 288 for ZeroErr 19-bit absolute, 16 777 216 for Delta ASDA-B3 24-bit optical |
| `reductor_ratio` | `uint16_t` | Gearbox ratio (output-shaft revolutions per motor revolution). Use `1` for direct drive. Multiplied into the internal counts-per-RPM constant so all RPM values in the API refer to the **output shaft** |
| `driver_type` | `driver_type_t` | Selects the startup recipe — see below |
| `extra_sdo` | `const sdo_entry_t *` | Pointer to an application-defined table of additional SDO writes executed **after** the driver's built-in init sequence. `NULL` = no extras |
| `extra_sdo_count` | `uint8_t` | Number of entries in `extra_sdo`. `0` when `extra_sdo` is `NULL` |

#### Driver types

| Value | Drive | Startup recipe |
|-------|-------|---------------|
| `DRIVER_ZEROERR` | ZeroErr eRob (19-bit absolute encoder) | Full PDO mapping + profile ramp SDO + heartbeat. Velocity unit: count/s |
| `DRIVER_DELTA` | Delta ASDA-B3 servo | Same PDO layout; velocity unit: 0.1 RPM (driver native). `counts_per_rpm_s` adjusted accordingly |
| `DRIVER_WAT` | Non-CiA-402 drive sharing the same bus | Minimal startup: heartbeat period + consumer only. PDO mapping and CiA 402 FSM are skipped. Extend `build_sdo_table()` when the slave's OD is known |

### 4. Call init and loop functions

```c
#include "example_embedded.h"

// In your RTOS task or main():
app_canopen_init();   // called once before any interrupts are enabled

// Optional — register diagnostic callback before or after init:
app_canopen_set_diag_cb(my_diag_handler, NULL);

// High-priority timer ISR or highest-priority RTOS task (1 ms period):
void TIM_PeriodElapsedCallback(void) {
    co_transmit_process();   // produces SYNC, locks targets, sends TPDOs
}

// Main loop or lower-priority task:
void main_loop(void) {
    for (;;) {
        app_canopen_loop();   // drains RX, runs SDO sequencer, CiA 402 FSM
    }
}
```

### 5. Wire the RX interrupt

Place this macro **once** in any `.c` file (typically `example_embedded.c` already contains it):

```c
CO_PORT_DEFINE_RX_ISR(s_buses, s_bus_count)
```

This emits `HAL_FDCAN_RxFifo0Callback` (H7) or `HAL_CAN_RxFifo0MsgPendingCallback` (F4) and dispatches to the correct bus context by matching the peripheral handle.

---

## Public API

```c
// Initialise all buses and slaves. Call once before interrupts.
co_error_t app_canopen_init(void);

// Drain RX, run SDO sequencer, advance CiA 402 state machines.
// Call from main loop or low-priority task.
void app_canopen_loop(void);

// Produce SYNC, commit target RPMs to OD, send TPDOs via co_process().
// Must be called every SYNC_PERIOD_US (default 1 ms) from high-priority context.
void co_transmit_process(void);

// Set velocity target for one slave. Thread-safe (IRQ-disabled critical section).
// target_rpm refers to the output shaft (reductor_ratio already applied).
void cia402_set_target_rpm(uint16_t node_id, float target_rpm);

// Queue acceleration and deceleration updates via runtime SDO write.
// Thread-safe. Enqueued into the per-slave SDO runtime queue (depth: SDO_RT_QUEUE_SIZE).
void cia402_set_accel(uint16_t node_id, uint32_t accel_counts_s2, uint32_t decel_counts_s2);

// Read back latest position, velocity, and torque from RPDO feedback.
// pos_deg  : output-shaft angle in degrees [0, 360)
// vel_rpm  : output-shaft velocity in RPM (signed)
// torque_pct: torque as percentage of rated torque (0.1 % resolution from 0x6077)
void cia402_get_pos_vel(uint16_t node_id, float *pos_deg, float *vel_rpm, float *torque_pct);

// Register the structured diagnostic callback.
// Call any time — safe before app_canopen_init().
void app_canopen_set_diag_cb(co_diag_cb_t cb, void *user);
```

### Observable globals (read-only from application)

```c
volatile uint16_t g_emcy_code[CIA402_MAX_NODES];       // last EMCY code per slave
volatile uint8_t  g_emcy_reg[CIA402_MAX_NODES];        // last EMCY error register

volatile uint32_t g_sdo_rt_abort_code[CIA402_MAX_NODES];    // last runtime SDO abort code
volatile uint16_t g_sdo_rt_abort_index[CIA402_MAX_NODES];   // object index that was rejected
volatile uint8_t  g_sdo_rt_abort_subindex[CIA402_MAX_NODES];
```

---

## Structured diagnostic callback

Register a single callback to receive all fault and state-change events:

```c
void my_diag(const co_diag_info_t *info, void *user)
{
    switch (info->event) {
    case CO_DIAG_HB_TIMEOUT:
        // info->node_id — which slave lost its heartbeat
        break;
    case CO_DIAG_HB_RECOVERED:
        // slave heartbeat is back; re-init sequence will start automatically
        break;
    case CO_DIAG_EMCY_RECEIVED:
        // info->detail.emcy.emcy_code  — CiA 301 emergency error code
        // info->detail.emcy.error_reg  — error register (0x1001)
        // info->detail.emcy.msef       — manufacturer sub-error code (byte 3)
        break;
    case CO_DIAG_SDO_ABORT:
        // info->detail.sdo_abort.index      — OD index of rejected write
        // info->detail.sdo_abort.subindex
        // info->detail.sdo_abort.abort_code — CiA 301 abort code (e.g. 0x06090030)
        break;
    case CO_DIAG_SDO_TIMEOUT:
        // info->detail.sdo_timeout.index
        // info->detail.sdo_timeout.subindex
        // info->detail.sdo_timeout.is_init  — true: init sequence, false: runtime queue
        break;
    case CO_DIAG_DRIVE_STATE_CHANGE:
        // info->detail.drive.from  — master_cia402_state_t before transition
        // info->detail.drive.to    — master_cia402_state_t after  transition
        break;
    case CO_DIAG_NMT_STATE_CHANGE:
        // info->detail.nmt.state   — new co_nmt_state_t of the slave
        break;
    }
}

app_canopen_set_diag_cb(my_diag, NULL);
```

**Context warning:** the callback may fire from different execution contexts depending on the event:

| Event | Typical context |
|-------|----------------|
| `CO_DIAG_EMCY_RECEIVED`, `CO_DIAG_SDO_ABORT`, `CO_DIAG_SDO_TIMEOUT` (init) | `app_canopen_loop()` (main loop) |
| `CO_DIAG_SDO_TIMEOUT` (runtime) | `app_canopen_loop()` (main loop) |
| `CO_DIAG_DRIVE_STATE_CHANGE`, `CO_DIAG_NMT_STATE_CHANGE` | `app_canopen_loop()` or RPDO callback |
| `CO_DIAG_HB_TIMEOUT`, `CO_DIAG_HB_RECOVERED` | `co_transmit_process()` (high-priority) |

Keep the callback short and non-blocking. A ring buffer log is a safe pattern.

---

## Configuration reference

Every compile-time constant in the driver is wrapped with an `#ifndef` guard so it can be overridden **without touching any driver source file**. This is the intended usage when the driver is added as a git submodule.

### How to override

**Option A — `app_config.h`** (recommended for submodule use)

The driver includes `app_config.h` (via `example_embedded.h` → `canopen.h`). Define any constant there before the driver headers are processed:

```c
// app_config.h  (your project, not the submodule)
#define CIA402_MAX_NODES     6U    // required — no default
#define CO_MAX_RPDO         12U    // must be >= CIA402_MAX_NODES * 2
#define CO_MAX_TPDO         12U    // must be >= CIA402_MAX_NODES * 2
#define SYNC_PERIOD_US    2000UL   // 2 ms SYNC
#define CIA402_MAX_VEL_RPM 1500    // tighter velocity clamp
```

**Option B — compiler `-D` flags** (CMake / Makefile)

```cmake
target_compile_definitions(myapp PRIVATE
    CIA402_MAX_NODES=6
    CO_MAX_RPDO=12
    CO_MAX_TPDO=12
    SYNC_PERIOD_US=2000
)
```

Both options work simultaneously — `app_config.h` is evaluated first because it is `#include`d inside the guard, so `-D` flags from the compiler still take precedence over any `#define` inside it.

`CIA402_MAX_NODES` is the **only required** constant — omitting it is a hard `#error` at compile time. All others have safe defaults.

### Slave count and bus count

| Constant | Default | Description |
|----------|---------|-------------|
| `CIA402_MAX_NODES` | **required** | Maximum total slave count across all buses. Sizes `m_node[]`, `m_sdo_tbl[]`, `g_*[]`. A compile-time `_Static_assert` enforces `CIA402_MAX_NODES * 2 ≤ CO_MAX_RPDO` |
| `CO_PORT_MAX_BUSES` | `4` | Maximum distinct CAN peripherals (unique `can_handle` values). Sizes `s_buses[]` |

### Timing — `example_embedded.c`

| Constant | Default | Description |
|----------|---------|-------------|
| `MASTER_NODE_ID` | `0x7F` | CANopen node ID of the master on every bus |
| `MASTER_HEARTBEAT_MS` | `50` | Master heartbeat period sent to slaves [ms] |
| `SLAVE_HB_TIMEOUT_MS` | `150` | Heartbeat consumer timeout per slave [ms]. Must be > `MASTER_HEARTBEAT_MS` with enough margin to absorb one missed frame |
| `SYNC_PERIOD_US` | `1000` | SYNC frame period [µs]. Each bus produces its own SYNC. Determines `co_transmit_process()` call rate |
| `NMT_RESET_DELAY_MS` | `500` | Wait after sending NMT Reset Communication (0x82) before sending Pre-Op (0x80) [ms] |
| `NMT_PREOP_DELAY_MS` | `100` | Wait in Pre-Operational before sending the first init SDO [ms] |
| `NMT_PREOP_DEBOUNCE_MS` | `150` | Minimum time a slave must stay in PRE-OP after commissioning before re-init is triggered. Prevents a single corrupt/missed HB from restarting the 28-step SDO sequence [ms] |
| `SDO_TIMEOUT_MS` | `500` | SDO response timeout for both init sequence and runtime queue [ms] |
| `SDO_RT_QUEUE_SIZE` | `4` | Runtime SDO write queue depth per slave. `cia402_set_accel()` enqueues 2 entries; full queue returns false silently |

### CiA 402 FSM — `example_embedded.c`

| Constant | Default | Description |
|----------|---------|-------------|
| `CIA402_MAX_VEL_RPM` | `3000` | Software velocity clamp applied inside `co_transmit_process()` before writing to OD [RPM, output shaft] |
| `CIA402_STATE_TIMEOUT_MS` | `2000` | Timeout for each CiA 402 state transition (Shutdown→Ready, Switch-On→Switched-On, etc.). Expiry triggers a fault-reset cycle [ms] |
| `CIA402_FAULT_RESET_MS` | `10` | Duration the fault-reset controlword (0x0080) is held before clearing [ms] |
| `CIA402_RPDO_WATCHDOG_MS` | `100` | If no RPDO1 (statusword + velocity) is received within this window while RUNNING, the master sends Quick Stop and drops to IDLE [ms] |

### CAN ring buffer — per-driver headers

| Constant | Default | Driver | Description |
|----------|---------|--------|-------------|
| `CO_STM32_RX_QUEUE_SIZE` | `64` | H7 FDCAN | ISR→main-loop ring buffer size. Must be a power of two |
| `CO_STM32_RX_MAX_PER_POLL` | `32` | H7 FDCAN | Max frames drained per legacy poll call |
| `CO_STM32G4_RX_QUEUE_SIZE` | `64` | G4 FDCAN | Same semantics as H7 variant |
| `CO_STM32G4_RX_MAX_PER_POLL` | `32` | G4 FDCAN | Same semantics as H7 variant |
| `CO_STM32F4_RX_QUEUE_SIZE` | `64` | F4 bxCAN | Same semantics |
| `CO_STM32F4_RX_MAX_PER_POLL` | `32` | F4 bxCAN | Same semantics |

### Core stack — `canopen.h`

| Constant | Default | Description |
|----------|---------|-------------|
| `CO_MAX_OD_ENTRIES` | `192` | Maximum object dictionary entries per node. Each slave uses ~8 vendor OD entries for PDO backing; raise if adding many custom objects |
| `CO_MAX_RPDO` | `4` | Maximum RPDOs per node. **Must be ≥ `CIA402_MAX_NODES * 2`** — each slave consumes 2 RPDOs on its bus. Enforced by `_Static_assert` at compile time |
| `CO_MAX_TPDO` | `4` | Maximum TPDOs per node. **Must be ≥ `CIA402_MAX_NODES * 2`** — same constraint as `CO_MAX_RPDO` |
| `CO_PDO_MAP_ENTRIES` | `8` | Maximum mapped objects per PDO |
| `CO_SDO_CHANNELS` | `2` | Concurrent SDO server channels |
| `CO_SDO_TRANSFER_BUF_SIZE` | `1024` | Buffer for segmented/block SDO transfers [bytes] |
| `CO_EMCY_FAULT_SLOTS` | `16` | Number of simultaneous active faults the EMCY producer can track |
| `CO_HEARTBEAT_CONSUMERS_MAX` | `16` | Maximum slaves the HB consumer can supervise per node |

---

## Multi-bus topology

Each entry in `cia402_nodes[]` carries a `can_handle`. At `app_canopen_init()` time the master scans the table, groups entries by unique handle, and allocates one `co_node_t` + `co_port_ctx_t` pair per distinct peripheral:

```
cia402_nodes[0] — node 0x01, &hfdcan1  ─┐
cia402_nodes[1] — node 0x02, &hfdcan1  ─┤→ s_buses[0]  (FDCAN1, node_id 0x7F)
                                          │   SYNC, HB consumer for 0x01 + 0x02
cia402_nodes[2] — node 0x05, &hfdcan2  ──→ s_buses[1]  (FDCAN2, node_id 0x7F)
                                              SYNC, HB consumer for 0x05 only
```

- Each bus produces its own SYNC and supervises only its own slaves.
- RPDO/TPDO slot numbers restart at 0 within each bus (`local_index * 2`).
- The ISR macro dispatches by matching the peripheral handle — a single HAL callback covers all buses.
- `on_reset_communication` fires independently per bus; only that bus's slaves are reset.

---

## Threading model

Two execution contexts are used; they must **not** preempt each other without a mutex unless noted:

| Context | Functions | Touches |
|---------|----------|---------|
| High-priority (1 ms timer ISR or top-priority task) | `co_transmit_process()` | `s_buses[].node.sync_event_pending`, OD target velocity fields, `co_process()` |
| Main loop / low-priority task | `app_canopen_loop()` | `m_node[]`, `s_buses[]`, SDO state machines |
| Any context (ISR-safe) | `cia402_set_target_rpm()`, `cia402_set_accel()` | `g_target_rpm[]` via IRQ-disabled critical section |
| RX FIFO ISR only | `CO_PORT_DEFINE_RX_ISR` → `co_port_rx_isr()` | ISR ring buffer head — SPSC, no mutex needed |

If `co_transmit_process()` and `app_canopen_loop()` run at different priority levels, protect `m_node[]` and `s_buses[]` with a mutex or run both from the same context.

---

## PDO layout per CiA 402 slave

Each DRIVER_ZEROERR / DRIVER_DELTA slave is configured with two TPDO/RPDO pairs (from the master's perspective):

| Direction | PDO | COB-ID | Mapped objects |
|-----------|-----|--------|---------------|
| Master→Slave (TPDO) | TPDO0 of bus | `0x200 + node_id` | 0x6040:00 controlword (16 bit), 0x6060:00 mode (8 bit), 0x60FF:00 target velocity (32 bit) |
| Slave→Master (RPDO) | RPDO0 of bus | `0x180 + node_id` | 0x6041:00 statusword (16 bit), 0x6061:00 mode display (8 bit), 0x606C:00 velocity actual (32 bit) |
| Slave→Master (RPDO) | RPDO1 of bus | `0x280 + node_id` | 0x6064:00 position actual (32 bit), 0x6077:00 torque actual (16 bit) |

Transmission type for both TPDO and RPDO is `0x01` (every SYNC). The init SDO sequence (`build_sdo_table()`) writes these mappings to the slave's OD; no external SDO tooling is required.

---

## Slave init SDO sequence

When a slave is first detected or after heartbeat recovery, `sdo_cfg_run()` executes the following sequence automatically, with a `SDO_TIMEOUT_MS` timeout per step:

1. NMT Reset Communication (0x82) → wait `NMT_RESET_DELAY_MS`
2. NMT Pre-Operational (0x80) → wait `NMT_PREOP_DELAY_MS`
3. Configure slave RPDO1 communication + mapping (COB-ID, transmission type, 3 mapped objects)
4. Configure slave TPDO1 communication + mapping
5. Configure slave TPDO2 communication + mapping
6. Write acceleration ramp (0x6083) = `default_accel`
7. Write deceleration ramp (0x6084) = `default_decel`
8. Write slave heartbeat period (0x1017) = 50 ms
9. Write slave heartbeat consumer (0x1016:01) = master node ID + 150 ms timeout
10. NMT Start (0x01)

Any SDO abort or timeout fires `CO_DIAG_SDO_ABORT` / `CO_DIAG_SDO_TIMEOUT` and restarts the sequence from step 1.

### Adding node-specific parameters via `extra_sdo`

Provide a `const sdo_entry_t[]` table in your application and pass it through `cia402_cfg_t`. The driver appends these writes to its built-in sequence automatically — no driver source files need to be modified:

```c
#include "example_embedded.h"   // sdo_entry_t is declared here

// Additional parameters for node 0x01 (ZeroErr) — homing + following error limit
static const sdo_entry_t node01_params[] = {
    { 0x6065U, 0x00U, 100000U, 4U },  // following error window
    { 0x6066U, 0x00U,    500U, 2U },  // following error timeout [ms]
    { 0x6098U, 0x00U,     35U, 1U },  // homing method
    { 0x6099U, 0x01U,   5000U, 4U },  // homing fast approach speed
    { 0x6099U, 0x02U,    500U, 4U },  // homing slow approach speed
};
#define NODE01_PARAMS_COUNT  ((uint8_t)(sizeof(node01_params) / sizeof(node01_params[0])))

// Node with no extras (use NULL / 0)
const cia402_cfg_t cia402_nodes[] = {
    { 0x01U, &hfdcan1, 50000U, 200000U, 524288U, 1U, DRIVER_ZEROERR,
      node01_params, NODE01_PARAMS_COUNT },
    { 0x02U, &hfdcan1, 50000U, 200000U, 524288U, 1U, DRIVER_ZEROERR,
      NULL, 0U },
};
```

`sdo_entry_t.size` is the byte width of the value: `1` for `uint8_t`, `2` for `uint16_t`, `4` for `uint32_t`. Each entry is sent as an expedited SDO download with the same `SDO_TIMEOUT_MS` timeout as the built-in sequence.

---

## DEBUG build

Define `DEBUG` (`-DDEBUG`) to enable the per-slave transition ring buffer:

```c
#ifdef DEBUG
    cia402_transition_t transition_log[CIA402_TRANSITION_LOG_SIZE];  // 16 entries
    uint8_t             transition_log_head;
    uint8_t             transition_log_count;
#endif
```

Each entry records `ts_ms`, `statusword`, `from`, and `to` state. Inspect via debugger watch on `m_node[i].transition_log`. The ring does not wrap in `transition_log_count` — when full, count saturates at 16 so the debugger can distinguish "not yet wrapped" from "full".

`CIA402_TRANSITION_LOG_SIZE` defaults to 16 and must be a power of two.

---

## Host-side test harness

Build and run without any MCU hardware:

```sh
make test
```

`tests_host.c` exercises `canopen.c` and `cia402.c` with:
- Deterministic fake CAN transmit callback (captures frames in order)
- Fake clock (`millis`) advanced by test code
- Reset hook counters

### What is covered

- NMT command routing, boot-up sequence, communication/application reset hooks
- SDO expedited, segmented, and block upload/download; abort paths
- RPDO unpack and TPDO pack against OD variables
- Heartbeat producer timing
- EMCY raise/clear behavior
- CiA 402 core transition matrix (shutdown → switch-on → enable → quick-stop → fault-reset)

### Known gaps

- SDO block transfer edge cases (CRC variants, retransmit corner cases) not exhaustively tested
- TPDO SYNC-type / inhibit-time / event-timer combinations partially covered
- Heartbeat consumer full timeout recovery path not in host tests
- No hardware-in-loop conformance run
- Full CiA 301 certification profile not claimed — this is a compact production starter, not a certified stack
