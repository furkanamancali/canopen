/*
 * eRob CANopen Master — STM32H7 FDCAN
 *
 * This file implements a CANopen master that controls a single eRob integrated
 * joint actuator (CiA 402, Profile Velocity mode).
 *
 * Topology
 * ────────
 *   STM32H7 (this node, master)  ←──CAN──→  eRob actuator (slave, node 0x01)
 *
 *   Master node ID : 0x7F
 *   eRob  node ID  : 0x01  (match hardware DIP/EEP)
 *   SYNC producer  : master, 1 ms period (COB-ID 0x080)
 *
 * PDO wiring (master perspective)
 * ────────────────────────────────
 *   Master TPDO1  0x201  → eRob RPDO1  (56 bits, every SYNC)
 *     Controlword  0x6040  16 bits
 *     Mode of Op   0x6060   8 bits   [fixed: Profile Velocity = 3]
 *     Target Vel   0x60FF  32 bits   [RPM, signed]
 *
 *   Master TPDO2  0x301  → eRob RPDO2  (64 bits, every SYNC)
 *     Profile Accel  0x6083  32 bits  [RPM/s]
 *     Profile Decel  0x6084  32 bits  [RPM/s]
 *
 *   eRob TPDO1  0x181  → Master RPDO1  (56 bits, every SYNC)
 *     Statusword     0x6041  16 bits
 *     Mode Display   0x6061   8 bits
 *     Velocity Act   0x606C  32 bits  [RPM, signed]
 *
 *   eRob TPDO2  0x281  → Master RPDO2  (48 bits, every SYNC)
 *     Position Act   0x6064  32 bits  [encoder counts]
 *     Torque Act     0x6077  16 bits  [0.1 % rated]
 *
 * CiA 402 state sequencing
 * ─────────────────────────
 *   The master drives the eRob through the CiA 402 state machine by writing
 *   specific controlword patterns into Master TPDO1:
 *
 *     SWITCH_ON_DISABLED → READY_TO_SWITCH_ON : controlword 0x0006
 *     READY_TO_SWITCH_ON → SWITCHED_ON        : controlword 0x0007
 *     SWITCHED_ON        → OPERATION_ENABLED  : controlword 0x000F
 *     OPERATION_ENABLED  → QUICK_STOP         : controlword 0x000B
 *     any fault          → clear fault        : controlword 0x0080 (rising edge)
 *
 * Runtime velocity change
 * ────────────────────────
 *   Call erob_set_target_velocity(rpm) at any time.  The new value is packed
 *   into Master TPDO1 and transmitted on the next SYNC tick.
 */

#include "canopen.h"
#include "canopen_stm32h7_fdcan.h"

extern FDCAN_HandleTypeDef hfdcan1;

/* ── Master / eRob configuration ─────────────────────────────────────────── */
#define MASTER_NODE_ID           0x7FU
#define MASTER_HEARTBEAT_MS      100U

#define EROB_NODE_ID             0x01U
#define EROB_HB_TIMEOUT_MS       1000U   /* declare eRob lost after 1 s       */

#define SYNC_PERIOD_US           1000UL  /* 1 ms SYNC produced by master      */

#define EROB_DEFAULT_ACCEL       500U    /* [RPM/s]  initial profile accel     */
#define EROB_DEFAULT_DECEL       500U    /* [RPM/s]  initial profile decel     */
#define EROB_MAX_VEL_RPM         3000    /* [RPM]    clamped before write      */

/* CiA 402 controlword patterns (DS402 §8.1.3) */
#define CW_SHUTDOWN              0x0006U /* → READY_TO_SWITCH_ON              */
#define CW_SWITCH_ON             0x0007U /* → SWITCHED_ON                     */
#define CW_ENABLE_OPERATION      0x000FU /* → OPERATION_ENABLED               */
#define CW_QUICK_STOP            0x000BU /* → QUICK_STOP                      */
#define CW_FAULT_RESET           0x0080U /* rising edge clears fault           */
#define CW_FAULT_RESET_CLEAR     0x0000U /* lower the fault-reset bit          */

/* Profile Velocity mode identifier (CiA 402 §6.1.1) */
#define EROB_MODE_PROFILE_VEL    ((int8_t)3)

/* ── CANopen stack instance ───────────────────────────────────────────────── */
static co_node_t      canopen_node;
static co_stm32_ctx_t can_ctx;

/* ── OD backing storage for master PDO objects ───────────────────────────── */
/* TPDO1: command frame sent to eRob every SYNC */
static uint16_t  m_controlword    = CW_SHUTDOWN;
static int8_t    m_mode_of_op     = EROB_MODE_PROFILE_VEL;
static int32_t   m_target_vel     = 0;

/* TPDO2: ramp parameters sent to eRob every SYNC */
static uint32_t  m_profile_accel  = EROB_DEFAULT_ACCEL;
static uint32_t  m_profile_decel  = EROB_DEFAULT_DECEL;

/* RPDO1: feedback received from eRob every SYNC */
static uint16_t  m_statusword     = 0;
static int8_t    m_mode_display   = 0;
static int32_t   m_velocity_act   = 0;

/* RPDO2: feedback received from eRob every SYNC */
static int32_t   m_position_act   = 0;
static int16_t   m_torque_act     = 0;

/* ── CiA 402 state machine (master-side, drives controlword) ─────────────── */
typedef enum {
    MASTER_CIA402_IDLE = 0,     /* not yet started                          */
    MASTER_CIA402_FAULT_RESET,  /* sending fault-reset pulse                */
    MASTER_CIA402_SHUTDOWN,     /* sending Shutdown  → READY_TO_SWITCH_ON   */
    MASTER_CIA402_SWITCH_ON,    /* sending Switch On → SWITCHED_ON          */
    MASTER_CIA402_ENABLE,       /* sending Enable Op → OPERATION_ENABLED    */
    MASTER_CIA402_RUNNING,      /* OPERATION_ENABLED, applying velocity      */
    MASTER_CIA402_FAULT,        /* eRob reported a fault                     */
} master_cia402_state_t;

static master_cia402_state_t m_drive_state = MASTER_CIA402_IDLE;
static uint32_t              m_state_entry_ms = 0;

/* Bit masks for interpreting statusword (CiA 402 §8.2.1) */
#define SW_RTSO    0x0001U  /* Ready to Switch On       */
#define SW_SO      0x0002U  /* Switched On              */
#define SW_OE      0x0004U  /* Operation Enabled        */
#define SW_FAULT   0x0008U  /* Fault                    */
#define SW_QS      0x0020U  /* Quick Stop               */
#define SW_SOD     0x0040U  /* Switch On Disabled       */

static bool erob_in_state_switch_on_disabled(uint16_t sw)
{
    return (sw & (SW_RTSO | SW_SO | SW_OE | SW_FAULT | SW_QS | SW_SOD))
            == SW_SOD;
}
static bool erob_in_state_ready_to_switch_on(uint16_t sw)
{
    return (sw & (SW_RTSO | SW_SO | SW_OE | SW_FAULT | SW_QS | SW_SOD))
            == (SW_RTSO | SW_QS);
}
static bool erob_in_state_switched_on(uint16_t sw)
{
    return (sw & (SW_RTSO | SW_SO | SW_OE | SW_FAULT | SW_QS | SW_SOD))
            == (SW_RTSO | SW_SO | SW_QS);
}
static bool erob_in_state_operation_enabled(uint16_t sw)
{
    return (sw & (SW_RTSO | SW_SO | SW_OE | SW_FAULT | SW_QS | SW_SOD))
            == (SW_RTSO | SW_SO | SW_OE | SW_QS);
}
static bool erob_in_state_fault(uint16_t sw)
{
    return (sw & SW_FAULT) != 0U;
}

/*
 * Drive the eRob CiA 402 state machine from the master side.
 * Called once per SYNC tick after RPDO data has been unpacked.
 */
static void erob_state_machine_step(void)
{
    uint32_t now_ms = HAL_GetTick();
    uint16_t sw     = m_statusword;

    switch (m_drive_state) {

    case MASTER_CIA402_IDLE:
        /* Begin sequencing: attempt fault reset first, then enable. */
        m_controlword   = CW_FAULT_RESET;
        m_drive_state   = MASTER_CIA402_FAULT_RESET;
        m_state_entry_ms = now_ms;
        break;

    case MASTER_CIA402_FAULT_RESET:
        /* Hold fault-reset bit for ≥1 SYNC tick, then lower it. */
        if ((now_ms - m_state_entry_ms) >= 5U) {
            m_controlword  = CW_FAULT_RESET_CLEAR;
            m_drive_state  = MASTER_CIA402_SHUTDOWN;
            m_state_entry_ms = now_ms;
        }
        break;

    case MASTER_CIA402_SHUTDOWN:
        m_controlword = CW_SHUTDOWN;
        if (erob_in_state_ready_to_switch_on(sw)) {
            m_drive_state  = MASTER_CIA402_SWITCH_ON;
            m_state_entry_ms = now_ms;
        } else if (erob_in_state_fault(sw)) {
            m_drive_state  = MASTER_CIA402_FAULT;
            m_state_entry_ms = now_ms;
        }
        break;

    case MASTER_CIA402_SWITCH_ON:
        m_controlword = CW_SWITCH_ON;
        if (erob_in_state_switched_on(sw)) {
            m_drive_state  = MASTER_CIA402_ENABLE;
            m_state_entry_ms = now_ms;
        } else if (erob_in_state_fault(sw)) {
            m_drive_state  = MASTER_CIA402_FAULT;
            m_state_entry_ms = now_ms;
        }
        break;

    case MASTER_CIA402_ENABLE:
        m_controlword = CW_ENABLE_OPERATION;
        if (erob_in_state_operation_enabled(sw)) {
            m_drive_state  = MASTER_CIA402_RUNNING;
            m_state_entry_ms = now_ms;
        } else if (erob_in_state_fault(sw)) {
            m_drive_state  = MASTER_CIA402_FAULT;
            m_state_entry_ms = now_ms;
        }
        break;

    case MASTER_CIA402_RUNNING:
        m_controlword = CW_ENABLE_OPERATION;
        if (erob_in_state_fault(sw)) {
            m_drive_state  = MASTER_CIA402_FAULT;
            m_state_entry_ms = now_ms;
            m_target_vel   = 0;
        }
        break;

    case MASTER_CIA402_FAULT:
        /* Zero velocity, hold fault-reset high, then restart sequencing. */
        m_target_vel   = 0;
        m_controlword  = CW_FAULT_RESET;
        if ((now_ms - m_state_entry_ms) >= 200U && !erob_in_state_fault(sw)) {
            m_controlword  = CW_FAULT_RESET_CLEAR;
            m_drive_state  = MASTER_CIA402_SHUTDOWN;
            m_state_entry_ms = now_ms;
        }
        break;
    }
}

/* ── Runtime velocity setter (public API) ────────────────────────────────── */

/*
 * erob_set_target_velocity() — change the eRob target velocity at any time.
 *
 * Clamped to ±EROB_MAX_VEL_RPM.  The value is written into the master's
 * TPDO1 OD entry (0x60FF:00) and transmitted on the next SYNC tick.
 * Only effective when the drive is in OPERATION_ENABLED (MASTER_CIA402_RUNNING).
 *
 * Not ISR-safe; call from main-task context only.
 */
void erob_set_target_velocity(int32_t rpm)
{
    if (rpm >  EROB_MAX_VEL_RPM) { rpm =  EROB_MAX_VEL_RPM; }
    if (rpm < -EROB_MAX_VEL_RPM) { rpm = -EROB_MAX_VEL_RPM; }
    (void)co_od_write(&canopen_node, 0x60FFU, 0x00U,
                      (const uint8_t *)&rpm, sizeof(rpm));
}

/* ── RPDO hook: called after eRob feedback PDOs are unpacked ─────────────── */
static void on_erob_rpdo_written(co_node_t *node, uint8_t rpdo_num,
                                 uint16_t index, uint8_t subindex, void *user)
{
    (void)node; (void)index; (void)subindex; (void)user;

    /*
     * Both RPDOs are fully unpacked into the OD backing variables
     * (m_statusword, m_velocity_act, …) by the stack before this callback
     * fires.  No explicit read needed — just act on the fresh values.
     *
     * RPDO2 carries position and torque; no extra action needed here.
     * RPDO1 carries the statusword — advance the drive state machine.
     */
    if (rpdo_num == 0U) {
        erob_state_machine_step();
    }
}

/* ── TPDO hook: refresh OD before transmission ───────────────────────────── */
static void on_tpdo_pre_tx(co_node_t *node, uint8_t tpdo_num, void *user)
{
    (void)node; (void)tpdo_num; (void)user;
    /*
     * m_controlword and m_target_vel are already in the OD backing storage
     * (pointed to directly by the OD entries).  Nothing extra to do here;
     * the hook exists as an extension point for future use.
     */
}

/* ── NMT Start helper ────────────────────────────────────────────────────── */
/*
 * Send NMT "Start Remote Node" to the eRob.
 * The CANopen NMT frame is: COB-ID 0x000, byte[0]=0x01, byte[1]=node_id.
 * The stack does not expose an NMT-master API, so we build it via the
 * hardware send callback directly.
 */
static void nmt_start_node(uint8_t node_id)
{
    co_can_frame_t f;
    f.cob_id  = 0x000U;
    f.len     = 2U;
    f.data[0] = 0x01U;   /* Start Remote Node */
    f.data[1] = node_id;
    (void)canopen_node.iface.send(canopen_node.iface.user, &f);
}

/* ── Initialization ──────────────────────────────────────────────────────── */
void app_canopen_init(void)
{
    /* 1. Attach to FDCAN1 as master (node 0x7F). */
    co_stm32_attach(&canopen_node, &hfdcan1, &can_ctx,
                    MASTER_NODE_ID, MASTER_HEARTBEAT_MS);

    /* 2. Register OD entries for the master-side PDO objects.
     *
     *    The OD objects (0x6040, 0x6060, 0x60FF, 0x6083, 0x6084, 0x6041,
     *    0x6061, 0x606C, 0x6064, 0x6077) live in this master node's OD solely
     *    so that the PDO mapping mechanism can pack/unpack them.  They are NOT
     *    the real eRob objects — they are the master's local mirrors.
     */

    /* TPDO1 objects (master transmits → eRob receives) */
    (void)co_od_add(&canopen_node, 0x6040U, 0x00U,
                    (uint8_t *)&m_controlword,   sizeof(m_controlword),
                    true, true);
    (void)co_od_add(&canopen_node, 0x6060U, 0x00U,
                    (uint8_t *)&m_mode_of_op,    sizeof(m_mode_of_op),
                    true, true);
    (void)co_od_add(&canopen_node, 0x60FFU, 0x00U,
                    (uint8_t *)&m_target_vel,    sizeof(m_target_vel),
                    true, true);

    /* TPDO2 objects */
    (void)co_od_add(&canopen_node, 0x6083U, 0x00U,
                    (uint8_t *)&m_profile_accel, sizeof(m_profile_accel),
                    true, true);
    (void)co_od_add(&canopen_node, 0x6084U, 0x00U,
                    (uint8_t *)&m_profile_decel, sizeof(m_profile_decel),
                    true, true);

    /* RPDO1 objects (eRob transmits → master receives) */
    (void)co_od_add(&canopen_node, 0x6041U, 0x00U,
                    (uint8_t *)&m_statusword,    sizeof(m_statusword),
                    true, true);
    (void)co_od_add(&canopen_node, 0x6061U, 0x00U,
                    (uint8_t *)&m_mode_display,  sizeof(m_mode_display),
                    true, true);
    (void)co_od_add(&canopen_node, 0x606CU, 0x00U,
                    (uint8_t *)&m_velocity_act,  sizeof(m_velocity_act),
                    true, true);

    /* RPDO2 objects */
    (void)co_od_add(&canopen_node, 0x6064U, 0x00U,
                    (uint8_t *)&m_position_act,  sizeof(m_position_act),
                    true, true);
    (void)co_od_add(&canopen_node, 0x6077U, 0x00U,
                    (uint8_t *)&m_torque_act,    sizeof(m_torque_act),
                    true, true);

    /* 3. Register RPDO/TPDO hooks. */
    co_set_hooks(&canopen_node, on_erob_rpdo_written, on_tpdo_pre_tx, NULL);

    /* 4. RPDO communication parameters (master receives from eRob).
     *
     *    RPDO1 listens on 0x181 (eRob's TPDO1).
     *    RPDO2 listens on 0x281 (eRob's TPDO2).
     *    Transmission type 0x01: accept data on SYNC-triggered frames.
     */
    const uint32_t rpdo1_cob = 0x180UL + EROB_NODE_ID;   /* 0x181 */
    const uint8_t  rpdo_sync = 0x01U;
    (void)co_od_write(&canopen_node, 0x1400U, 0x01U,
                      (const uint8_t *)&rpdo1_cob, sizeof(rpdo1_cob));
    (void)co_od_write(&canopen_node, 0x1400U, 0x02U, &rpdo_sync, 1U);

    const uint32_t rpdo2_cob = 0x280UL + EROB_NODE_ID;   /* 0x281 */
    (void)co_od_write(&canopen_node, 0x1401U, 0x01U,
                      (const uint8_t *)&rpdo2_cob, sizeof(rpdo2_cob));
    (void)co_od_write(&canopen_node, 0x1401U, 0x02U, &rpdo_sync, 1U);

    /* 5. RPDO mapping. */
    const uint32_t r1m1 = (0x6041UL << 16) | (0x00UL << 8) | 16UL;
    const uint32_t r1m2 = (0x6061UL << 16) | (0x00UL << 8) |  8UL;
    const uint32_t r1m3 = (0x606CUL << 16) | (0x00UL << 8) | 32UL;
    (void)co_od_write(&canopen_node, 0x1600U, 0x00U, (const uint8_t[]){0U}, 1U);
    (void)co_od_write(&canopen_node, 0x1600U, 0x01U, (const uint8_t *)&r1m1, 4U);
    (void)co_od_write(&canopen_node, 0x1600U, 0x02U, (const uint8_t *)&r1m2, 4U);
    (void)co_od_write(&canopen_node, 0x1600U, 0x03U, (const uint8_t *)&r1m3, 4U);
    (void)co_od_write(&canopen_node, 0x1600U, 0x00U, (const uint8_t[]){3U}, 1U);

    const uint32_t r2m1 = (0x6064UL << 16) | (0x00UL << 8) | 32UL;
    const uint32_t r2m2 = (0x6077UL << 16) | (0x00UL << 8) | 16UL;
    (void)co_od_write(&canopen_node, 0x1601U, 0x00U, (const uint8_t[]){0U}, 1U);
    (void)co_od_write(&canopen_node, 0x1601U, 0x01U, (const uint8_t *)&r2m1, 4U);
    (void)co_od_write(&canopen_node, 0x1601U, 0x02U, (const uint8_t *)&r2m2, 4U);
    (void)co_od_write(&canopen_node, 0x1601U, 0x00U, (const uint8_t[]){2U}, 1U);

    /* 6. TPDO communication parameters (master sends to eRob).
     *
     *    TPDO1 transmits on 0x201 (eRob's RPDO1 COB-ID).
     *    TPDO2 transmits on 0x301 (eRob's RPDO2 COB-ID).
     *    Transmission type 0x01: send on every SYNC.
     */
    const uint32_t tpdo1_cob = 0x200UL + EROB_NODE_ID;   /* 0x201 */
    const uint8_t  tpdo_sync = 0x01U;
    (void)co_od_write(&canopen_node, 0x1800U, 0x01U,
                      (const uint8_t *)&tpdo1_cob, sizeof(tpdo1_cob));
    (void)co_od_write(&canopen_node, 0x1800U, 0x02U, &tpdo_sync, 1U);

    const uint32_t tpdo2_cob = 0x300UL + EROB_NODE_ID;   /* 0x301 */
    (void)co_od_write(&canopen_node, 0x1801U, 0x01U,
                      (const uint8_t *)&tpdo2_cob, sizeof(tpdo2_cob));
    (void)co_od_write(&canopen_node, 0x1801U, 0x02U, &tpdo_sync, 1U);

    /* 7. TPDO mapping. */
    const uint32_t t1m1 = (0x6040UL << 16) | (0x00UL << 8) | 16UL;
    const uint32_t t1m2 = (0x6060UL << 16) | (0x00UL << 8) |  8UL;
    const uint32_t t1m3 = (0x60FFUL << 16) | (0x00UL << 8) | 32UL;
    (void)co_od_write(&canopen_node, 0x1A00U, 0x00U, (const uint8_t[]){0U}, 1U);
    (void)co_od_write(&canopen_node, 0x1A00U, 0x01U, (const uint8_t *)&t1m1, 4U);
    (void)co_od_write(&canopen_node, 0x1A00U, 0x02U, (const uint8_t *)&t1m2, 4U);
    (void)co_od_write(&canopen_node, 0x1A00U, 0x03U, (const uint8_t *)&t1m3, 4U);
    (void)co_od_write(&canopen_node, 0x1A00U, 0x00U, (const uint8_t[]){3U}, 1U);

    const uint32_t t2m1 = (0x6083UL << 16) | (0x00UL << 8) | 32UL;
    const uint32_t t2m2 = (0x6084UL << 16) | (0x00UL << 8) | 32UL;
    (void)co_od_write(&canopen_node, 0x1A01U, 0x00U, (const uint8_t[]){0U}, 1U);
    (void)co_od_write(&canopen_node, 0x1A01U, 0x01U, (const uint8_t *)&t2m1, 4U);
    (void)co_od_write(&canopen_node, 0x1A01U, 0x02U, (const uint8_t *)&t2m2, 4U);
    (void)co_od_write(&canopen_node, 0x1A01U, 0x00U, (const uint8_t[]){2U}, 1U);

    /* 8. SYNC producer: master generates 0x080 every 1 ms.
     *    Bit 30 of COB-ID set → this node is the SYNC producer.
     */
    const uint32_t sync_cob_id   = 0x40000080UL;  /* bit30=1: producer      */
    const uint32_t sync_cycle_us = SYNC_PERIOD_US;
    (void)co_od_write(&canopen_node, 0x1005U, 0x00U,
                      (const uint8_t *)&sync_cob_id,   sizeof(sync_cob_id));
    (void)co_od_write(&canopen_node, 0x1006U, 0x00U,
                      (const uint8_t *)&sync_cycle_us, sizeof(sync_cycle_us));

    /* 9. Heartbeat consumer: detect eRob silence within EROB_HB_TIMEOUT_MS. */
    const uint8_t  hb_count = 1U;
    const uint32_t hb_erob  = ((uint32_t)EROB_NODE_ID << 16)
                              | (uint32_t)EROB_HB_TIMEOUT_MS;
    (void)co_od_write(&canopen_node, 0x1016U, 0x00U, &hb_count, 1U);
    (void)co_od_write(&canopen_node, 0x1016U, 0x01U,
                      (const uint8_t *)&hb_erob, sizeof(hb_erob));
}

/* ── Main loop ───────────────────────────────────────────────────────────── */

/*
 * g_target_rpm — set from anywhere (motion planner, UART parser, RTOS task)
 * to change the eRob velocity at runtime.  Effective only while the drive
 * is in OPERATION_ENABLED.
 */
volatile int32_t g_target_rpm = 0;

void app_main_loop(void)
{
    /* Send NMT Start to the eRob so it enters OPERATIONAL state. */
    nmt_start_node(EROB_NODE_ID);

    for (;;) {
        /* 1. Drain FDCAN RX FIFO — unpacks incoming eRob TPDOs into OD,
         *    fires on_erob_rpdo_written → erob_state_machine_step().       */
        co_stm32_poll_rx(&canopen_node, &hfdcan1);

        /* 2. On each SYNC tick produced by co_process(), forward the
         *    application target velocity into the TPDO1 OD entry.           */
        if (canopen_node.sync_event_pending) {
            canopen_node.sync_event_pending = false;
            erob_set_target_velocity(g_target_rpm);
        }

        /* 3. Run the CANopen protocol stack:
         *      - produces SYNC every 1 ms (triggers TPDO auto-transmit)
         *      - transmits TPDO1 (command) and TPDO2 (ramp params) on SYNC
         *      - sends master heartbeat every 100 ms
         *      - monitors eRob heartbeat timeout                             */
        co_process(&canopen_node);
    }
}
