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
- Heartbeat consumer / node guarding behavior is not implemented.
- Advanced CiA 402 operation-mode semantics (trajectory/profile constraints, hardware interlocks, mode-specific diagnostics) remain application-specific.
- No automated hardware-in-loop conformance run is included.
