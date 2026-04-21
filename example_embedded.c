/*
 * eRob CANopen Master — STM32H7 FDCAN
 *
 * Controls a single eRob integrated joint actuator (CiA 402 Profile Velocity).
 *
 * Topology
 * ────────
 *   STM32H7  (master, node 0x7F)  ←──CAN──→  eRob (slave, node 0x01)
 *
 *   SYNC producer : master, 1 ms period, COB-ID 0x080
 *
 * PDO wiring (master perspective)
 * ────────────────────────────────
 *   Master TPDO1  0x201 → eRob RPDO1   (every SYNC, 0x1800/0x1A00)
 *     Controlword  0x6040  16 bits
 *     Mode of Op   0x6060   8 bits   [fixed: Profile Velocity = 3]
 *     Target Vel   0x60FF  32 bits   [RPM, signed]
 *
 *   Master TPDO2  0x301 → eRob RPDO2   (every SYNC, 0x1801/0x1A01)
 *     Profile Accel  0x6083  32 bits  [RPM/s]
 *     Profile Decel  0x6084  32 bits  [RPM/s]
 *
 *   eRob TPDO1  0x181 → Master RPDO1   (every SYNC, 0x1400/0x1600)
 *     Statusword     0x6041  16 bits
 *     Mode Display   0x6061   8 bits
 *     Velocity Act   0x606C  32 bits  [RPM, signed]
 *
 *   eRob TPDO2  0x281 → Master RPDO2   (every SYNC, 0x1401/0x1601)
 *     Position Act   0x6064  32 bits  [encoder counts]
 *     Torque Act     0x6077  16 bits  [0.1 % rated]
 *
 * CiA 402 sequencing (master drives controlword)
 * ───────────────────────────────────────────────
 *   The master implements a supervisory state machine that writes specific
 *   controlword patterns into TPDO1 to move the eRob through CiA 402 states:
 *
 *     SWITCH_ON_DISABLED → READY_TO_SWITCH_ON : 0x0006  (Shutdown)
 *     READY_TO_SWITCH_ON → SWITCHED_ON        : 0x0007  (Switch On)
 *     SWITCHED_ON        → OPERATION_ENABLED  : 0x000F  (Enable Operation)
 *     any fault / reset                        : 0x0080  (Fault Reset, rising edge)
 *
 * Runtime velocity change
 * ────────────────────────
 *   Set g_target_rpm from any task/context.  On each SYNC tick the value is
 *   clamped and written directly to m_target_vel (the OD backing variable for
 *   0x60FF:00), which is packed into TPDO1 on the next co_process() call.
 */

#include "canopen.h"
#include "canopen_stm32h7_fdcan.h"

extern FDCAN_HandleTypeDef hfdcan1;

/* ── Master / eRob configuration ─────────────────────────────────────────── */
#define MASTER_NODE_ID          0x7FU
#define MASTER_HEARTBEAT_MS     100U

#define EROB_NODE_ID            0x01U
#define EROB_HB_TIMEOUT_MS      1000U    /* declare eRob lost after 1 s       */

#define SYNC_PERIOD_US          1000UL   /* 1 ms SYNC produced by master      */

#define EROB_DEFAULT_ACCEL      500U     /* [RPM/s]                           */
#define EROB_DEFAULT_DECEL      500U     /* [RPM/s]                           */
#define EROB_MAX_VEL_RPM        3000     /* [RPM]  clamped before write       */

/* Sequencing state timeouts */
#define EROB_STATE_TIMEOUT_MS   2000U    /* max time to wait in any seq state */
#define EROB_FAULT_RESET_MS     10U      /* duration to hold fault-reset bit  */

/* CiA 402 controlword patterns (DS402 §8.1.3) */
#define CW_SHUTDOWN             0x0006U
#define CW_SWITCH_ON            0x0007U
#define CW_ENABLE_OPERATION     0x000FU
#define CW_QUICK_STOP           0x000BU
#define CW_FAULT_RESET          0x0080U
#define CW_FAULT_RESET_CLEAR    0x0000U

/* Profile Velocity mode (CiA 402 §6.1.1) */
#define EROB_MODE_PROFILE_VEL   ((int8_t)3)

/* ── CANopen stack instance ───────────────────────────────────────────────── */
static co_node_t      canopen_node;
static co_stm32_ctx_t can_ctx;

/* ── OD backing storage ──────────────────────────────────────────────────── */
/* TPDO1: command frame sent to eRob every SYNC */
static uint16_t  m_controlword   = CW_SHUTDOWN;
static int8_t    m_mode_of_op    = EROB_MODE_PROFILE_VEL;
static int32_t   m_target_vel    = 0;

/* TPDO2: ramp parameters sent to eRob every SYNC */
static uint32_t  m_profile_accel = EROB_DEFAULT_ACCEL;
static uint32_t  m_profile_decel = EROB_DEFAULT_DECEL;

/* RPDO1: feedback received from eRob every SYNC */
static uint16_t  m_statusword    = 0;
static int8_t    m_mode_display  = 0;
static int32_t   m_velocity_act  = 0;

/* RPDO2: feedback received from eRob every SYNC */
static int32_t   m_position_act  = 0;
static int16_t   m_torque_act    = 0;

/* ── CiA 402 master state machine ────────────────────────────────────────── */
typedef enum {
    MASTER_CIA402_IDLE = 0,      /* waiting for first statusword               */
    MASTER_CIA402_FAULT_RESET,   /* holding fault-reset bit high               */
    MASTER_CIA402_WAIT_FAULT_CLEAR, /* waiting for SW_FAULT to drop            */
    MASTER_CIA402_SHUTDOWN,      /* sending Shutdown → READY_TO_SWITCH_ON      */
    MASTER_CIA402_SWITCH_ON,     /* sending Switch On → SWITCHED_ON            */
    MASTER_CIA402_ENABLE,        /* sending Enable Op → OPERATION_ENABLED      */
    MASTER_CIA402_RUNNING,       /* OPERATION_ENABLED, applying velocity        */
    MASTER_CIA402_FAULT,         /* eRob reported a fault — recovery pending    */
} master_cia402_state_t;

static master_cia402_state_t m_drive_state    = MASTER_CIA402_IDLE;
static uint32_t              m_state_entry_ms = 0;

/* Whether the master has self-promoted to OPERATIONAL and started the eRob. */
static bool m_master_started = false;

/* ── CiA 402 statusword bit helpers (DS402 Table 14) ─────────────────────── */
#define SW_RTSO  0x0001U  /* Ready to Switch On       */
#define SW_SO    0x0002U  /* Switched On              */
#define SW_OE    0x0004U  /* Operation Enabled        */
#define SW_FAULT 0x0008U  /* Fault                    */
#define SW_QS    0x0020U  /* Quick Stop               */
#define SW_SOD   0x0040U  /* Switch On Disabled       */
#define SW_MASK  (SW_RTSO | SW_SO | SW_OE | SW_FAULT | SW_QS | SW_SOD)

static inline bool erob_sw_switch_on_disabled(uint16_t sw)
{
    return (sw & SW_MASK) == SW_SOD;
}
static inline bool erob_sw_ready_to_switch_on(uint16_t sw)
{
    return (sw & SW_MASK) == (SW_RTSO | SW_QS);
}
static inline bool erob_sw_switched_on(uint16_t sw)
{
    return (sw & SW_MASK) == (SW_RTSO | SW_SO | SW_QS);
}
static inline bool erob_sw_operation_enabled(uint16_t sw)
{
    return (sw & SW_MASK) == (SW_RTSO | SW_SO | SW_OE | SW_QS);
}
static inline bool erob_sw_fault_reaction_active(uint16_t sw)
{
    /* DS402 Table 14: Fault Reaction Active = RTSO+SO+OE+Fault bits all set */
    return (sw & SW_MASK) == (SW_FAULT | SW_OE | SW_SO | SW_RTSO);
}
static inline bool erob_sw_fault(uint16_t sw)
{
    return (sw & SW_FAULT) != 0U;
}

/* ── Inline millis helper ────────────────────────────────────────────────── */
static inline uint32_t now_ms(void)
{
    return canopen_node.iface.millis(canopen_node.iface.user);
}

/*
 * Drive the eRob CiA 402 state machine from the master side.
 * Called once per SYNC tick from on_erob_rpdo_frame (after full RPDO1 unpack).
 */
static void erob_state_machine_step(void)
{
    const uint32_t t   = now_ms();
    const uint16_t sw  = m_statusword;

    switch (m_drive_state) {

    case MASTER_CIA402_IDLE:
        /* Wait for a stable, known state before attempting sequencing.
         * - SW=0 ("Not Ready to Switch On"): transient power-on, wait.
         * - Fault Reaction Active: drive is still stopping; fault-reset
         *   requires a 0→1 rising edge *after* the drive reaches stable
         *   Fault state — issuing it earlier is a spec violation and most
         *   drives silently ignore it, leaving the fault uncleared.
         * - Fault: raise the reset pulse.
         * - Any non-fault state: proceed directly to Shutdown.            */
        if (erob_sw_fault_reaction_active(sw)) {
            break;  /* wait for drive to reach stable Fault state */
        } else if (erob_sw_fault(sw)) {
            m_controlword    = CW_FAULT_RESET;
            m_drive_state    = MASTER_CIA402_FAULT_RESET;
            m_state_entry_ms = t;
        } else if (erob_sw_switch_on_disabled(sw) ||
                   erob_sw_ready_to_switch_on(sw)  ||
                   erob_sw_switched_on(sw)          ||
                   erob_sw_operation_enabled(sw)) {
            m_drive_state    = MASTER_CIA402_SHUTDOWN;
            m_state_entry_ms = t;
        }
        break;

    case MASTER_CIA402_FAULT_RESET:
        /* Hold fault-reset high for EROB_FAULT_RESET_MS, then release. */
        m_controlword = CW_FAULT_RESET;
        if ((t - m_state_entry_ms) >= EROB_FAULT_RESET_MS) {
            m_controlword    = CW_FAULT_RESET_CLEAR;
            m_drive_state    = MASTER_CIA402_WAIT_FAULT_CLEAR;
            m_state_entry_ms = t;
        }
        break;

    case MASTER_CIA402_WAIT_FAULT_CLEAR:
        m_controlword = CW_FAULT_RESET_CLEAR;
        if (!erob_sw_fault(sw)) {
            m_drive_state    = MASTER_CIA402_SHUTDOWN;
            m_state_entry_ms = t;
        } else if ((t - m_state_entry_ms) >= EROB_STATE_TIMEOUT_MS) {
            /* eRob is stuck in fault — retry the reset pulse. */
            m_drive_state    = MASTER_CIA402_FAULT_RESET;
            m_state_entry_ms = t;
        }
        break;

    case MASTER_CIA402_SHUTDOWN:
        m_controlword = CW_SHUTDOWN;
        if (erob_sw_ready_to_switch_on(sw)) {
            m_drive_state    = MASTER_CIA402_SWITCH_ON;
            m_state_entry_ms = t;
        } else if (erob_sw_fault(sw)) {
            m_drive_state    = MASTER_CIA402_FAULT;
            m_state_entry_ms = t;
        } else if ((t - m_state_entry_ms) >= EROB_STATE_TIMEOUT_MS) {
            /* Stuck — re-attempt via fault-reset. */
            m_drive_state    = MASTER_CIA402_FAULT_RESET;
            m_state_entry_ms = t;
        }
        break;

    case MASTER_CIA402_SWITCH_ON:
        m_controlword = CW_SWITCH_ON;
        if (erob_sw_switched_on(sw)) {
            m_drive_state    = MASTER_CIA402_ENABLE;
            m_state_entry_ms = t;
        } else if (erob_sw_fault(sw)) {
            m_drive_state    = MASTER_CIA402_FAULT;
            m_state_entry_ms = t;
        } else if ((t - m_state_entry_ms) >= EROB_STATE_TIMEOUT_MS) {
            m_drive_state    = MASTER_CIA402_FAULT_RESET;
            m_state_entry_ms = t;
        }
        break;

    case MASTER_CIA402_ENABLE:
        m_controlword = CW_ENABLE_OPERATION;
        if (erob_sw_operation_enabled(sw)) {
            m_drive_state    = MASTER_CIA402_RUNNING;
            m_state_entry_ms = t;
        } else if (erob_sw_fault(sw)) {
            m_drive_state    = MASTER_CIA402_FAULT;
            m_state_entry_ms = t;
        } else if ((t - m_state_entry_ms) >= EROB_STATE_TIMEOUT_MS) {
            m_drive_state    = MASTER_CIA402_FAULT_RESET;
            m_state_entry_ms = t;
        }
        break;

    case MASTER_CIA402_RUNNING:
        m_controlword = CW_ENABLE_OPERATION;
        if (erob_sw_fault(sw)) {
            m_target_vel     = 0;
            m_drive_state    = MASTER_CIA402_FAULT;
            m_state_entry_ms = t;
        }
        break;

    case MASTER_CIA402_FAULT:
        m_target_vel  = 0;

        if (erob_sw_fault_reaction_active(sw)) {
            /* Drive is still reacting — do not touch controlword. */
            break;
        }

        /* Phase A: raise fault-reset bit. */
        if ((t - m_state_entry_ms) < EROB_FAULT_RESET_MS) {
            m_controlword = CW_FAULT_RESET;
        } else {
            /* Phase B: lower the bit; wait for SW_FAULT to clear. */
            m_controlword = CW_FAULT_RESET_CLEAR;
            if (!erob_sw_fault(sw)) {
                m_drive_state    = MASTER_CIA402_SHUTDOWN;
                m_state_entry_ms = t;
            } else if ((t - m_state_entry_ms) >= EROB_STATE_TIMEOUT_MS) {
                /* eRob is still faulted after timeout — retry reset pulse. */
                m_state_entry_ms = t;  /* restart the FAULT phase timer */
            }
        }
        break;
    }
}

/* ── Hook: per-frame RPDO callback ──────────────────────────────────────────
 * Fires after ALL objects in the RPDO have been unpacked into OD backing
 * storage.  Safe to read m_statusword, m_mode_display, m_velocity_act here.
 */
static void on_erob_rpdo_frame(co_node_t *node, uint8_t rpdo_num, void *user)
{
    (void)node; (void)user;

    if (rpdo_num == 0U) {
        /* RPDO1 complete: statusword + mode_display + velocity_act are fresh. */
        erob_state_machine_step();
    }
    /* RPDO2 (position_act + torque_act) needs no immediate action. */
}

/* ── Hook: heartbeat consumer event ─────────────────────────────────────────
 * Fires when the eRob heartbeat transitions between ACTIVE and TIMEOUT.
 */
static void on_erob_hb_event(co_node_t *node, uint8_t slave_node_id,
                              co_hb_event_t event, void *user)
{
    (void)node; (void)user;

    if (slave_node_id != EROB_NODE_ID) {
        return;
    }

    if (event == CO_HB_EVENT_TIMEOUT) {
        /* eRob went silent — zero velocity command and return to IDLE.
         * On reconnect, the sequencing restarts from scratch.               */
        m_target_vel  = 0;
        m_controlword = CW_SHUTDOWN;
        m_drive_state = MASTER_CIA402_IDLE;
    } else {
        /* eRob recovered — restart sequencing. */
        m_drive_state    = MASTER_CIA402_IDLE;
        m_state_entry_ms = now_ms();
    }
}

/* ── NMT Reset Communication callback ───────────────────────────────────────
 * Called by the stack when this node receives an NMT Reset Communication
 * command.  Resets application-level state so sequencing restarts cleanly.
 */
static void on_reset_communication(void *user)
{
    (void)user;
    m_target_vel     = 0;
    m_controlword    = CW_SHUTDOWN;
    m_drive_state    = MASTER_CIA402_IDLE;
    m_master_started = false;
}

/* ── Runtime velocity setter (public API) ────────────────────────────────── */

/*
 * erob_set_target_velocity() — change the eRob target velocity at any time.
 *
 * Writes directly to m_target_vel, the OD backing variable for 0x60FF:00.
 * The value is packed into TPDO1 and sent to the eRob on the next SYNC tick.
 * Only takes effect while the drive is in OPERATION_ENABLED (RUNNING state).
 *
 * Not ISR-safe; call from main-task context only.
 */
void erob_set_target_velocity(int32_t rpm)
{
    if (rpm >  EROB_MAX_VEL_RPM) { rpm =  EROB_MAX_VEL_RPM; }
    if (rpm < -EROB_MAX_VEL_RPM) { rpm = -EROB_MAX_VEL_RPM; }
    m_target_vel = rpm;
}

/* ── Initialization ──────────────────────────────────────────────────────── */

/*
 * app_canopen_init() — configure the CANopen master for eRob control.
 *
 * Returns CO_ERROR_NONE on success.  Any other value means an OD registration
 * or configuration call failed — treat as a fatal initialisation error.
 */
co_error_t app_canopen_init(void)
{
    co_error_t err;

    /* 1. Attach to FDCAN1 as the CANopen master (node 0x7F). */
    co_stm32_attach(&canopen_node, &hfdcan1, &can_ctx,
                    MASTER_NODE_ID, MASTER_HEARTBEAT_MS);

    /* Wire the NMT reset callback so application state is cleaned up. */
    canopen_node.iface.on_reset_communication = on_reset_communication;

    /* 2. Register OD entries for PDO-mapped objects.
     *
     *    These exist in the master's own OD solely as backing storage for
     *    the PDO pack/unpack engine.  They are the master's local mirrors of
     *    the eRob objects — not the real drive objects.
     */

    /* TPDO1 objects (master → eRob commands) */
    err = co_od_add(&canopen_node, 0x6040U, 0x00U,
                    (uint8_t *)&m_controlword,   sizeof(m_controlword),   true, true);
    if (err != CO_ERROR_NONE) { return err; }

    err = co_od_add(&canopen_node, 0x6060U, 0x00U,
                    (uint8_t *)&m_mode_of_op,    sizeof(m_mode_of_op),    true, true);
    if (err != CO_ERROR_NONE) { return err; }

    err = co_od_add(&canopen_node, 0x60FFU, 0x00U,
                    (uint8_t *)&m_target_vel,    sizeof(m_target_vel),    true, true);
    if (err != CO_ERROR_NONE) { return err; }

    /* TPDO2 objects (master → eRob ramp parameters) */
    err = co_od_add(&canopen_node, 0x6083U, 0x00U,
                    (uint8_t *)&m_profile_accel, sizeof(m_profile_accel), true, true);
    if (err != CO_ERROR_NONE) { return err; }

    err = co_od_add(&canopen_node, 0x6084U, 0x00U,
                    (uint8_t *)&m_profile_decel, sizeof(m_profile_decel), true, true);
    if (err != CO_ERROR_NONE) { return err; }

    /* RPDO1 objects (eRob → master feedback) */
    err = co_od_add(&canopen_node, 0x6041U, 0x00U,
                    (uint8_t *)&m_statusword,    sizeof(m_statusword),    true, true);
    if (err != CO_ERROR_NONE) { return err; }

    err = co_od_add(&canopen_node, 0x6061U, 0x00U,
                    (uint8_t *)&m_mode_display,  sizeof(m_mode_display),  true, true);
    if (err != CO_ERROR_NONE) { return err; }

    err = co_od_add(&canopen_node, 0x606CU, 0x00U,
                    (uint8_t *)&m_velocity_act,  sizeof(m_velocity_act),  true, true);
    if (err != CO_ERROR_NONE) { return err; }

    /* RPDO2 objects */
    err = co_od_add(&canopen_node, 0x6064U, 0x00U,
                    (uint8_t *)&m_position_act,  sizeof(m_position_act),  true, true);
    if (err != CO_ERROR_NONE) { return err; }

    err = co_od_add(&canopen_node, 0x6077U, 0x00U,
                    (uint8_t *)&m_torque_act,    sizeof(m_torque_act),    true, true);
    if (err != CO_ERROR_NONE) { return err; }

    /* 3. Register hooks.
     *    on_rpdo_frame_written fires once per RPDO after all objects are
     *    unpacked — all m_* feedback variables are coherent at that point.
     *    on_hb_event fires when the eRob heartbeat times out or recovers.
     */
    co_set_rpdo_frame_hook(&canopen_node, on_erob_rpdo_frame, NULL);
    co_set_hb_event_hook(&canopen_node,  on_erob_hb_event,   NULL);

    /* 4. RPDO communication parameters (master receives from eRob).
     *
     *    0x1400: RPDO1 listens on 0x181 (eRob's TPDO1 COB-ID).
     *    0x1401: RPDO2 listens on 0x281 (eRob's TPDO2 COB-ID).
     *
     *    COB-ID encoding (CiA 301 §7.3.3): bit 31=0 → valid, bits 0-10 = ID.
     *    Transmission type 0x01: process on SYNC (matches eRob TPDO type).
     */
    const uint32_t rpdo1_cob = 0x180UL + EROB_NODE_ID;   /* 0x181 */
    const uint8_t  rpdo_sync = 0x01U;
    if (co_od_write(&canopen_node, 0x1400U, 0x01U,
                    (const uint8_t *)&rpdo1_cob, sizeof(rpdo1_cob)) != 0U) { return CO_ERROR_INVALID_ARGS; }
    if (co_od_write(&canopen_node, 0x1400U, 0x02U, &rpdo_sync, 1U) != 0U)  { return CO_ERROR_INVALID_ARGS; }

    const uint32_t rpdo2_cob = 0x280UL + EROB_NODE_ID;   /* 0x281 */
    if (co_od_write(&canopen_node, 0x1401U, 0x01U,
                    (const uint8_t *)&rpdo2_cob, sizeof(rpdo2_cob)) != 0U) { return CO_ERROR_INVALID_ARGS; }
    if (co_od_write(&canopen_node, 0x1401U, 0x02U, &rpdo_sync, 1U) != 0U)  { return CO_ERROR_INVALID_ARGS; }

    /* 5. RPDO mapping (eRob TPDO → master OD objects).
     *
     *    Encoding: (index << 16) | (subindex << 8) | bit_length
     *    CiA 301 re-mapping sequence: write sub 0 = 0 (lock), set entries,
     *    write sub 0 = count (unlock).
     */
    const uint32_t r1m1 = (0x6041UL << 16) | (0x00UL << 8) | 16UL;
    const uint32_t r1m2 = (0x6061UL << 16) | (0x00UL << 8) |  8UL;
    const uint32_t r1m3 = (0x606CUL << 16) | (0x00UL << 8) | 32UL;
    if (co_od_write(&canopen_node, 0x1600U, 0x00U, (const uint8_t[]){0U}, 1U) != 0U) { return CO_ERROR_INVALID_ARGS; }
    if (co_od_write(&canopen_node, 0x1600U, 0x01U, (const uint8_t *)&r1m1, 4U) != 0U) { return CO_ERROR_INVALID_ARGS; }
    if (co_od_write(&canopen_node, 0x1600U, 0x02U, (const uint8_t *)&r1m2, 4U) != 0U) { return CO_ERROR_INVALID_ARGS; }
    if (co_od_write(&canopen_node, 0x1600U, 0x03U, (const uint8_t *)&r1m3, 4U) != 0U) { return CO_ERROR_INVALID_ARGS; }
    if (co_od_write(&canopen_node, 0x1600U, 0x00U, (const uint8_t[]){3U}, 1U) != 0U) { return CO_ERROR_INVALID_ARGS; }

    const uint32_t r2m1 = (0x6064UL << 16) | (0x00UL << 8) | 32UL;
    const uint32_t r2m2 = (0x6077UL << 16) | (0x00UL << 8) | 16UL;
    if (co_od_write(&canopen_node, 0x1601U, 0x00U, (const uint8_t[]){0U}, 1U) != 0U) { return CO_ERROR_INVALID_ARGS; }
    if (co_od_write(&canopen_node, 0x1601U, 0x01U, (const uint8_t *)&r2m1, 4U) != 0U) { return CO_ERROR_INVALID_ARGS; }
    if (co_od_write(&canopen_node, 0x1601U, 0x02U, (const uint8_t *)&r2m2, 4U) != 0U) { return CO_ERROR_INVALID_ARGS; }
    if (co_od_write(&canopen_node, 0x1601U, 0x00U, (const uint8_t[]){2U}, 1U) != 0U) { return CO_ERROR_INVALID_ARGS; }

    /* 6. TPDO communication parameters (master sends to eRob).
     *
     *    0x1800: TPDO1 transmits on 0x201 (eRob's RPDO1 COB-ID).
     *    0x1801: TPDO2 transmits on 0x301 (eRob's RPDO2 COB-ID).
     */
    const uint32_t tpdo1_cob = 0x200UL + EROB_NODE_ID;   /* 0x201 */
    const uint8_t  tpdo_sync = 0x01U;
    if (co_od_write(&canopen_node, 0x1800U, 0x01U,
                    (const uint8_t *)&tpdo1_cob, sizeof(tpdo1_cob)) != 0U) { return CO_ERROR_INVALID_ARGS; }
    if (co_od_write(&canopen_node, 0x1800U, 0x02U, &tpdo_sync, 1U) != 0U)  { return CO_ERROR_INVALID_ARGS; }

    const uint32_t tpdo2_cob = 0x300UL + EROB_NODE_ID;   /* 0x301 */
    if (co_od_write(&canopen_node, 0x1801U, 0x01U,
                    (const uint8_t *)&tpdo2_cob, sizeof(tpdo2_cob)) != 0U) { return CO_ERROR_INVALID_ARGS; }
    if (co_od_write(&canopen_node, 0x1801U, 0x02U, &tpdo_sync, 1U) != 0U)  { return CO_ERROR_INVALID_ARGS; }

    /* 7. TPDO mapping (master OD objects → eRob RPDO). */
    const uint32_t t1m1 = (0x6040UL << 16) | (0x00UL << 8) | 16UL;
    const uint32_t t1m2 = (0x6060UL << 16) | (0x00UL << 8) |  8UL;
    const uint32_t t1m3 = (0x60FFUL << 16) | (0x00UL << 8) | 32UL;
    if (co_od_write(&canopen_node, 0x1A00U, 0x00U, (const uint8_t[]){0U}, 1U) != 0U) { return CO_ERROR_INVALID_ARGS; }
    if (co_od_write(&canopen_node, 0x1A00U, 0x01U, (const uint8_t *)&t1m1, 4U) != 0U) { return CO_ERROR_INVALID_ARGS; }
    if (co_od_write(&canopen_node, 0x1A00U, 0x02U, (const uint8_t *)&t1m2, 4U) != 0U) { return CO_ERROR_INVALID_ARGS; }
    if (co_od_write(&canopen_node, 0x1A00U, 0x03U, (const uint8_t *)&t1m3, 4U) != 0U) { return CO_ERROR_INVALID_ARGS; }
    if (co_od_write(&canopen_node, 0x1A00U, 0x00U, (const uint8_t[]){3U}, 1U) != 0U) { return CO_ERROR_INVALID_ARGS; }

    const uint32_t t2m1 = (0x6083UL << 16) | (0x00UL << 8) | 32UL;
    const uint32_t t2m2 = (0x6084UL << 16) | (0x00UL << 8) | 32UL;
    if (co_od_write(&canopen_node, 0x1A01U, 0x00U, (const uint8_t[]){0U}, 1U) != 0U) { return CO_ERROR_INVALID_ARGS; }
    if (co_od_write(&canopen_node, 0x1A01U, 0x01U, (const uint8_t *)&t2m1, 4U) != 0U) { return CO_ERROR_INVALID_ARGS; }
    if (co_od_write(&canopen_node, 0x1A01U, 0x02U, (const uint8_t *)&t2m2, 4U) != 0U) { return CO_ERROR_INVALID_ARGS; }
    if (co_od_write(&canopen_node, 0x1A01U, 0x00U, (const uint8_t[]){2U}, 1U) != 0U) { return CO_ERROR_INVALID_ARGS; }

    /* 8. SYNC producer: master generates COB-ID 0x080 every 1 ms.
     *    Bit 30 of the COB-ID raw value = 1 → this node is the producer.
     */
    const uint32_t sync_cob_id   = 0x40000080UL;  /* bit30=1: producer      */
    const uint32_t sync_cycle_us = SYNC_PERIOD_US;
    if (co_od_write(&canopen_node, 0x1005U, 0x00U,
                    (const uint8_t *)&sync_cob_id,   sizeof(sync_cob_id))   != 0U) { return CO_ERROR_INVALID_ARGS; }
    if (co_od_write(&canopen_node, 0x1006U, 0x00U,
                    (const uint8_t *)&sync_cycle_us, sizeof(sync_cycle_us)) != 0U) { return CO_ERROR_INVALID_ARGS; }

    /* 9. Heartbeat consumer: detect eRob silence within EROB_HB_TIMEOUT_MS.
     *    Encoding: (node_id << 16) | timeout_ms
     */
    const uint8_t  hb_count = 1U;
    const uint32_t hb_erob  = ((uint32_t)EROB_NODE_ID << 16)
                              | (uint32_t)EROB_HB_TIMEOUT_MS;
    if (co_od_write(&canopen_node, 0x1016U, 0x00U, &hb_count, 1U) != 0U) { return CO_ERROR_INVALID_ARGS; }
    if (co_od_write(&canopen_node, 0x1016U, 0x01U,
                    (const uint8_t *)&hb_erob, sizeof(hb_erob)) != 0U) { return CO_ERROR_INVALID_ARGS; }

    return CO_ERROR_NONE;
}

/* ── Main loop ───────────────────────────────────────────────────────────── */

/*
 * g_target_rpm — set from anywhere (motion planner, UART parser, RTOS task)
 * to change the eRob velocity at runtime.  Effective only while the drive is
 * in OPERATION_ENABLED (MASTER_CIA402_RUNNING).
 *
 * Aligned 32-bit accesses on Cortex-M7 are atomic at the hardware level.
 * For multi-core or DMA scenarios use __LDREXW/__STREXW or a mutex instead.
 */
volatile int32_t g_target_rpm = 0;

void app_main_loop(void)
{
    for (;;) {
        /* 1. Drain the FDCAN RX FIFO (max CO_STM32_RX_MAX_PER_POLL frames).
         *    Incoming eRob TPDOs are unpacked into OD backing storage and
         *    on_erob_rpdo_frame → erob_state_machine_step() is called.      */
        co_stm32_poll_rx(&canopen_node, &hfdcan1);

        /* 2. After co_process() sends bootup and promotes the master to
         *    PRE_OPERATIONAL, self-promote to OPERATIONAL and send NMT Start
         *    to the eRob so it enters OPERATIONAL and begins exchanging PDOs.
         *
         *    co_nmt_set_state() bypasses the NMT frame (self-start is allowed
         *    per CiA 301 for master nodes).  co_nmt_master_send() sends the
         *    NMT Start command to the eRob via the stack's send path (so TX
         *    errors are tracked through the existing fault mechanism).       */
        if (!m_master_started &&
            canopen_node.nmt_state == CO_NMT_PRE_OPERATIONAL) {
            co_nmt_set_state(&canopen_node, CO_NMT_OPERATIONAL);
            (void)co_nmt_master_send(&canopen_node, 0x01U, EROB_NODE_ID);
            m_master_started = true;
        }

        /* 3. On each SYNC tick: forward the application target velocity into
         *    the TPDO1 backing variable.  co_process() will pack it and send
         *    it to the eRob on the same SYNC event.                          */
        if (canopen_node.sync_event_pending) {
            canopen_node.sync_event_pending = false;
            erob_set_target_velocity(g_target_rpm);
        }

        /* 4. Run the CANopen stack:
         *      - produces SYNC every 1 ms and sets sync_event_pending
         *      - auto-transmits TPDO1 and TPDO2 on every SYNC
         *      - sends master heartbeat every 100 ms
         *      - checks eRob heartbeat consumer timeout                      */
        co_process(&canopen_node);
    }
}
