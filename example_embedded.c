/*
 * eRob CANopen Master — STM32H7 FDCAN
 *
 * Controls a single eRob integrated joint actuator (CiA 402 Profile Velocity).
 *
 * Topology
 * ────────
 *   STM32H7  (master, node 0x7F)  ←──CAN──→  eRob (slave, node 0x05)
 *
 *   SYNC producer : master, 1 ms period, COB-ID 0x080
 *
 * PDO wiring (master perspective)
 * ────────────────────────────────
 *   Master TPDO1  0x205 → eRob RPDO1   (every SYNC, 0x1800/0x1A00)
 *     Controlword  0x6040  16 bits
 *     Mode of Op   0x6060   8 bits   [fixed: Profile Velocity = 3]
 *     Target Vel   0x60FF  32 bits   [RPM, signed]
 *
 *   Master TPDO2  0x305 → eRob RPDO2   (every SYNC, 0x1801/0x1A01)
 *     Profile Accel  0x6083  32 bits  [RPM/s]
 *     Profile Decel  0x6084  32 bits  [RPM/s]
 *
 *   eRob TPDO1  0x185 → Master RPDO1   (every SYNC, 0x1400/0x1600)
 *     Statusword     0x6041  16 bits
 *     Mode Display   0x6061   8 bits
 *     Velocity Act   0x606C  32 bits  [RPM, signed]
 *
 *   eRob TPDO2  0x285 → Master RPDO2   (every SYNC, 0x1401/0x1601)
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

#define EROB_NODE_ID            0x05U
#define EROB_HB_TIMEOUT_MS      1000U    /* declare eRob lost after 1 s       */

#define SYNC_PERIOD_US          1000UL   /* 1 ms SYNC produced by master      */

#define EROB_DEFAULT_ACCEL            (5000)     /* [plus/s] */
#define EROB_DEFAULT_DECEL            (5000)     /* [plus/s] */ 
#define EROB_MAX_VEL_RPM              (3000)     /* [RPM]  clamped before unit convert */
#define EROB_ENCODER_MAX_VAL          (524288U)  /* [plus]*/
#define EROB_ENCODER_REDUCTOR_RATE    (0.6F)
#define EROB_ENCODER_RPM_TO_PLUS_RATE (8738)     /* EROB_ENCODER_MAX_VAL * EROB_ENCODER_REDUCTOR_RATE */
/* Sequencing state timeouts */
#define EROB_STATE_TIMEOUT_MS     2000U  /* max time to wait in any seq state */
#define EROB_FAULT_RESET_MS       10U    /* duration to hold fault-reset bit  */
#define EROB_RPDO_WATCHDOG_MS     500U   /* master-side: stop if no RPDO for this long */

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

/* ── SDO client: remote eRob PDO configuration ───────────────────────────── */
#define EROB_SDO_COB_TX  (0x600U + EROB_NODE_ID)   /* master→eRob */
#define EROB_SDO_COB_RX  (0x580U + EROB_NODE_ID)   /* eRob→master */
#define EROB_SDO_TIMEOUT_MS 500U

typedef struct {
    uint16_t index;
    uint8_t  subindex;
    uint32_t value;
    uint8_t  size;
} erob_sdo_write_t;

#define EROB_RPDO1_COB  (0x80000000UL | (0x200UL + EROB_NODE_ID))  /* disabled */
#define EROB_RPDO1_COBEN             (0x200UL + EROB_NODE_ID)       /* enabled  */
#define EROB_RPDO2_COB  (0x80000000UL | (0x300UL + EROB_NODE_ID))
#define EROB_RPDO2_COBEN             (0x300UL + EROB_NODE_ID)
#define EROB_TPDO1_COB  (0x80000000UL | (0x180UL + EROB_NODE_ID))
#define EROB_TPDO1_COBEN             (0x180UL + EROB_NODE_ID)
#define EROB_TPDO2_COB  (0x80000000UL | (0x280UL + EROB_NODE_ID))
#define EROB_TPDO2_COBEN             (0x280UL + EROB_NODE_ID)

static const erob_sdo_write_t EROB_PDO_CFG[] = {
    /* ── eRob RPDO1 (receives master TPDO1) ── */
    { 0x1400U, 0x01U, EROB_RPDO1_COB,   4U },  /* disable while reconfiguring  */
    { 0x1400U, 0x02U, 0xFFU,             1U },  /* event-driven transmission    */
    { 0x1600U, 0x00U, 0U,                1U },  /* lock mapping                 */
    { 0x1600U, 0x01U, 0x60400010UL,     4U },  /* controlword 16-bit           */
    { 0x1600U, 0x02U, 0x60600008UL,     4U },  /* modes of operation 8-bit     */
    { 0x1600U, 0x03U, 0x60FF0020UL,     4U },  /* target velocity 32-bit       */
    { 0x1600U, 0x00U, 3U,                1U },  /* unlock: 3 entries            */
    { 0x1400U, 0x01U, EROB_RPDO1_COBEN, 4U },  /* enable                       */
    { 0x1400U, 0x05U, 500U,             2U },  /* comm watchdog: fault if no PDO for 500 ms */
    /* ── eRob RPDO2 (receives master TPDO2) ── */
    { 0x1401U, 0x01U, EROB_RPDO2_COB,   4U },
    { 0x1401U, 0x02U, 0xFFU,             1U },
    { 0x1601U, 0x00U, 0U,                1U },
    { 0x1601U, 0x01U, 0x60830020UL,     4U },  /* profile acceleration 32-bit  */
    { 0x1601U, 0x02U, 0x60840020UL,     4U },  /* profile deceleration 32-bit  */
    { 0x1601U, 0x00U, 2U,                1U },
    { 0x1401U, 0x01U, EROB_RPDO2_COBEN, 4U },
    { 0x1401U, 0x05U, 500U,             2U },  /* comm watchdog: fault if no PDO for 500 ms */
    /* ── eRob TPDO1 (sends to master RPDO1) ── */
    { 0x1800U, 0x01U, EROB_TPDO1_COB,   4U },
    { 0x1800U, 0x02U, 0x01U,             1U },  /* synchronous (SYNC-triggered) */
    { 0x1A00U, 0x00U, 0U,                1U },
    { 0x1A00U, 0x01U, 0x60410010UL,     4U },  /* statusword 16-bit            */
    { 0x1A00U, 0x02U, 0x60610008UL,     4U },  /* modes of operation display   */
    { 0x1A00U, 0x03U, 0x606C0020UL,     4U },  /* velocity actual value 32-bit */
    { 0x1A00U, 0x00U, 3U,                1U },
    { 0x1800U, 0x01U, EROB_TPDO1_COBEN, 4U },
    /* ── eRob TPDO2 (sends to master RPDO2) ── */
    { 0x1801U, 0x01U, EROB_TPDO2_COB,   4U },
    { 0x1801U, 0x02U, 0x01U,             1U },
    { 0x1A01U, 0x00U, 0U,                1U },
    { 0x1A01U, 0x01U, 0x60640020UL,     4U },  /* position actual value 32-bit */
    { 0x1A01U, 0x02U, 0x60770010UL,     4U },  /* torque actual value 16-bit   */
    { 0x1A01U, 0x00U, 2U,                1U },
    { 0x1801U, 0x01U, EROB_TPDO2_COBEN, 4U },
};
#define EROB_PDO_CFG_COUNT ((uint32_t)(sizeof(EROB_PDO_CFG) / sizeof(EROB_PDO_CFG[0])))

typedef enum {
    EROB_SDO_CFG_IDLE = 0,
    EROB_SDO_CFG_NMT_DELAY,      /* wait after NMT Pre-Op before first SDO write */
    EROB_SDO_CFG_WAIT_RESPONSE,
    EROB_SDO_CFG_DONE,
} erob_sdo_cfg_state_t;

#define EROB_NMT_PREOP_DELAY_MS  100U  /* guard time: eRob processes NMT Pre-Op */

static erob_sdo_cfg_state_t m_sdo_cfg_state   = EROB_SDO_CFG_IDLE;
static uint32_t             m_sdo_cfg_step     = 0U;
static uint32_t             m_sdo_cfg_step_ms  = 0U;
static bool                 m_sdo_cfg_resp_ok  = false;

/* Timestamp of the last received RPDO1 from the eRob (ms). */
static uint32_t m_last_rpdo1_ms = 0U;

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

/* Last observed NMT state of the eRob (from its heartbeat frames). */
static co_nmt_state_t m_erob_nmt_state = CO_NMT_INITIALIZING;

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
    /* FRA: Fault set while drive still appears enabled (RTSO+SO+OE all set).
     * QS is excluded from the mask — drives differ on whether it is set or
     * clear during fault reaction, so matching it would miss half the cases. */
    return (sw & (SW_FAULT | SW_OE | SW_SO | SW_RTSO))
               == (SW_FAULT | SW_OE | SW_SO | SW_RTSO);
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

/* Last EMCY frame received from eRob — inspect in debugger to identify fault. */
volatile uint16_t g_erob_last_emcy_code = 0U;   /* e.g. 0x8611 = following error */
volatile uint8_t  g_erob_last_emcy_reg  = 0U;   /* error register bits           */

static void erob_sdo_send_write(uint16_t index, uint8_t subindex, uint32_t value, uint8_t size)
{
    uint8_t cmd;
    switch (size) {
        case 1U: cmd = 0x2FU; break;
        case 2U: cmd = 0x2BU; break;
        case 3U: cmd = 0x27U; break;
        default: cmd = 0x23U; break;
    }
    co_can_frame_t f;
    f.cob_id  = EROB_SDO_COB_TX;
    f.len     = 8U;
    f.data[0] = cmd;
    f.data[1] = (uint8_t)(index & 0xFFU);
    f.data[2] = (uint8_t)(index >> 8U);
    f.data[3] = subindex;
    f.data[4] = (uint8_t)(value        & 0xFFU);
    f.data[5] = (uint8_t)((value >> 8U)  & 0xFFU);
    f.data[6] = (uint8_t)((value >> 16U) & 0xFFU);
    f.data[7] = (uint8_t)((value >> 24U) & 0xFFU);
    (void)canopen_node.iface.send(canopen_node.iface.user, &f);
}

static void on_erob_nmt_state_change(co_nmt_state_t new_state)
{
    switch (new_state) {

    case CO_NMT_INITIALIZING:
        /* Bootup message — full reconnect: re-run SDO config + NMT Start. */
        m_target_vel     = 0;
        m_controlword    = CW_SHUTDOWN;
        m_drive_state    = MASTER_CIA402_IDLE;
        m_sdo_cfg_state  = EROB_SDO_CFG_IDLE;
        m_sdo_cfg_step   = 0U;
        m_master_started = false;
        break;

    case CO_NMT_PRE_OPERATIONAL:
        /* eRob dropped to pre-op (e.g. received Reset Communication or
         * external NMT command) — PDOs are inactive.  Re-configure and
         * send NMT Start so PDO exchange resumes. */
        m_target_vel     = 0;
        m_controlword    = CW_SHUTDOWN;
        m_drive_state    = MASTER_CIA402_IDLE;
        m_sdo_cfg_state  = EROB_SDO_CFG_IDLE;
        m_sdo_cfg_step   = 0U;
        m_master_started = false;
        break;

    case CO_NMT_STOPPED:
        /* eRob in Stopped state — no PDOs, no SDO.  Zero velocity and
         * stall the drive state machine; send NMT Start to recover. */
        m_target_vel     = 0;
        m_controlword    = CW_SHUTDOWN;
        m_drive_state    = MASTER_CIA402_IDLE;
        /* Do NOT reset m_master_started — we just need NMT Start, not
         * a full SDO re-config.  The main loop detects !m_master_started
         * only when nmt_state == OPERATIONAL; here we send Start directly. */
        (void)co_nmt_master_send(&canopen_node, 0x01U, EROB_NODE_ID);
        break;

    case CO_NMT_OPERATIONAL:
        /* eRob is (back) in Operational — CiA 402 sequencing resumes
         * automatically via the RPDO frame callback. */
        break;
    }
}

/* on_rx_frame hook — intercepts SDO responses and EMCY frames from the eRob. */
static void on_rx_frame(void *user, const co_can_frame_t *frame)
{
    (void)user;

    /* Track eRob NMT state from its heartbeat (COB-ID 0x700 + node_id). */
    if (frame->cob_id == (0x700U + EROB_NODE_ID) && frame->len >= 1U) {
        const co_nmt_state_t new_state = (co_nmt_state_t)(frame->data[0] & 0x7FU);
        if (new_state != m_erob_nmt_state) {
            m_erob_nmt_state = new_state;
            on_erob_nmt_state_change(new_state);
        }
    }

    /* Capture EMCY frames (COB-ID 0x80 + node_id) to expose the fault code. */
    if (frame->cob_id == (0x80U + EROB_NODE_ID) && frame->len >= 3U) {
        g_erob_last_emcy_code = (uint16_t)frame->data[0]
                              | ((uint16_t)frame->data[1] << 8);
        g_erob_last_emcy_reg  = frame->data[2];
    }

    if (m_sdo_cfg_state != EROB_SDO_CFG_WAIT_RESPONSE) {
        return;
    }
    if (frame->cob_id != EROB_SDO_COB_RX || frame->len < 1U) {
        return;
    }
    /* Download response: scs=3 (0x60), any n/e/s bits */
    if ((frame->data[0] & 0xE0U) == 0x60U) {
        m_sdo_cfg_resp_ok = true;
    }
    /* Abort (0x80): reset and retry whole sequence from NMT Pre-Op */
    if (frame->data[0] == 0x80U) {
        m_sdo_cfg_state   = EROB_SDO_CFG_IDLE;
        m_sdo_cfg_step    = 0U;
        m_sdo_cfg_resp_ok = false;
    }
}

/*
 * erob_sdo_cfg_run() — advance the SDO configuration state machine one step.
 * Call from the main loop while eRob is in PRE_OPERATIONAL.
 * Returns true when all PDO mapping writes have been acknowledged.
 */
static bool erob_sdo_cfg_run(void)
{
    if (m_sdo_cfg_state == EROB_SDO_CFG_DONE) {
        return true;
    }

    const uint32_t t = now_ms();

    if (m_sdo_cfg_state == EROB_SDO_CFG_IDLE) {
        /* Send NMT Pre-Op to eRob on every attempt so PDO mapping writes are
         * accepted even if the eRob was left in OPERATIONAL from a prior run. */
        (void)co_nmt_master_send(&canopen_node, 0x80U, EROB_NODE_ID);
        m_sdo_cfg_step    = 0U;
        m_sdo_cfg_resp_ok = false;
        m_sdo_cfg_step_ms = t;
        m_sdo_cfg_state   = EROB_SDO_CFG_NMT_DELAY;
        return false;
    }

    if (m_sdo_cfg_state == EROB_SDO_CFG_NMT_DELAY) {
        if ((t - m_sdo_cfg_step_ms) < EROB_NMT_PREOP_DELAY_MS) {
            return false;  /* still waiting for eRob to process NMT Pre-Op */
        }
        m_sdo_cfg_step_ms = t;
        m_sdo_cfg_state   = EROB_SDO_CFG_WAIT_RESPONSE;
        const erob_sdo_write_t *w = &EROB_PDO_CFG[0];
        erob_sdo_send_write(w->index, w->subindex, w->value, w->size);
        return false;
    }

    /* EROB_SDO_CFG_WAIT_RESPONSE */
    if (!m_sdo_cfg_resp_ok) {
        if ((t - m_sdo_cfg_step_ms) >= EROB_SDO_TIMEOUT_MS) {
            /* Timed out — retry from beginning */
            m_sdo_cfg_state = EROB_SDO_CFG_IDLE;
            m_sdo_cfg_step  = 0U;
        }
        return false;
    }

    /* Response received — move to next step */
    m_sdo_cfg_resp_ok = false;
    m_sdo_cfg_step++;

    if (m_sdo_cfg_step >= EROB_PDO_CFG_COUNT) {
        m_sdo_cfg_state = EROB_SDO_CFG_DONE;
        return true;
    }

    m_sdo_cfg_step_ms = t;
    const erob_sdo_write_t *w = &EROB_PDO_CFG[m_sdo_cfg_step];
    erob_sdo_send_write(w->index, w->subindex, w->value, w->size);
    return false;
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
        if (erob_sw_fault(sw)) {
            m_target_vel     = 0;
            m_controlword    = CW_FAULT_RESET_CLEAR;  /* stop enabling immediately */
            m_drive_state    = MASTER_CIA402_FAULT;
            m_state_entry_ms = t;
        } else {
            m_controlword = CW_ENABLE_OPERATION;
        }
        break;

    case MASTER_CIA402_FAULT:
        m_target_vel  = 0;

        if (erob_sw_fault_reaction_active(sw)) {
            /* Drive is decelerating — keep controlword neutral and hold the
             * timer so the fault-reset pulse starts only after FRA exits. */
            m_controlword    = CW_FAULT_RESET_CLEAR;
            m_state_entry_ms = t;
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
                m_state_entry_ms = t;
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
        m_last_rpdo1_ms = now_ms();
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
        /* Attempt a Quick Stop — if the bus is still up the drive will
         * decelerate gracefully; if not, the drive's own RPDO event timer
         * (0x1400:05 = 500 ms) will have already triggered a self-stop. */
        m_target_vel  = 0;
        m_controlword = CW_QUICK_STOP;
        (void)co_send_tpdo(&canopen_node, 0U);  /* force immediate send */

        m_controlword    = CW_SHUTDOWN;
        m_drive_state    = MASTER_CIA402_IDLE;
        m_sdo_cfg_state  = EROB_SDO_CFG_IDLE;
        m_sdo_cfg_step   = 0U;
        m_master_started = false;
    } else {
        /* eRob recovered — re-run SDO config, then restart CiA 402 sequencing. */
        m_sdo_cfg_state  = EROB_SDO_CFG_IDLE;
        m_sdo_cfg_step   = 0U;
        m_master_started = false;
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
    m_sdo_cfg_state  = EROB_SDO_CFG_IDLE;
    m_sdo_cfg_step   = 0U;
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
    m_target_vel = rpm * EROB_ENCODER_RPM_TO_PLUS_RATE;
}

/*
 * erob_set_accel() — change profile acceleration/deceleration at runtime.
 * Values are in plus/s.  Takes effect on the next TPDO2 transmission.
 */
void erob_set_accel(uint32_t accel_plus_s, uint32_t decel_plus_s)
{
    m_profile_accel = accel_plus_s;
    m_profile_decel = decel_plus_s;
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

    /* Wire callbacks. */
    canopen_node.iface.on_reset_communication = on_reset_communication;
    canopen_node.iface.on_rx_frame            = on_rx_frame;

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
     *    0x1400: RPDO1 listens on 0x185 (eRob TPDO1 COB-ID, node 5).
     *    0x1401: RPDO2 listens on 0x285 (eRob TPDO2 COB-ID, node 5).
     *
     *    COB-ID encoding (CiA 301 §7.3.3): bit 31=0 → valid, bits 0-10 = ID.
     *    Transmission type 0x01: process on SYNC (matches eRob TPDO type).
     *    Note: eRob's TPDO COB-IDs are corrected to node-5 offsets by the SDO
     *    configuration sequence that runs before NMT Start is sent.
     */
    const uint32_t rpdo1_cob = 0x180UL + EROB_NODE_ID;   /* 0x185 */
    const uint8_t  rpdo_sync = 0x01U;
    if (co_od_write(&canopen_node, 0x1400U, 0x01U,
                    (const uint8_t *)&rpdo1_cob, sizeof(rpdo1_cob)) != 0U) { return CO_ERROR_INVALID_ARGS; }
    if (co_od_write(&canopen_node, 0x1400U, 0x02U, &rpdo_sync, 1U) != 0U)  { return CO_ERROR_INVALID_ARGS; }

    const uint32_t rpdo2_cob = 0x280UL + EROB_NODE_ID;   /* 0x285 */
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
     *    0x1800: TPDO1 transmits on 0x205 (eRob's RPDO1 COB-ID, node 5).
     *    0x1801: TPDO2 transmits on 0x305 (eRob's RPDO2 COB-ID, node 5).
     *
     *    Transmission type 0xFF (event-driven) + event timer = SYNC_PERIOD_US/1000
     *    ensures TPDOs fire on their own timer independent of SYNC delivery.
     *    Using sync transmission type (0x01) caused TPDOs to silently stall if
     *    the SYNC send failed even once (sync_last_produced_ms never updated →
     *    sync_event stays false → send_tpdo never becomes true).
     */
    const uint32_t tpdo1_cob  = 0x200UL + EROB_NODE_ID;   /* 0x205 */
    const uint8_t  tpdo_event = 0xFFU;
    const uint16_t tpdo_timer_ms = (uint16_t)(SYNC_PERIOD_US / 1000U);  /* 1 ms */
    if (co_od_write(&canopen_node, 0x1800U, 0x01U,
                    (const uint8_t *)&tpdo1_cob,   sizeof(tpdo1_cob))   != 0U) { return CO_ERROR_INVALID_ARGS; }
    if (co_od_write(&canopen_node, 0x1800U, 0x02U, &tpdo_event, 1U)     != 0U) { return CO_ERROR_INVALID_ARGS; }
    if (co_od_write(&canopen_node, 0x1800U, 0x05U,
                    (const uint8_t *)&tpdo_timer_ms, sizeof(tpdo_timer_ms)) != 0U) { return CO_ERROR_INVALID_ARGS; }

    const uint32_t tpdo2_cob = 0x300UL + EROB_NODE_ID;   /* 0x305 */
    if (co_od_write(&canopen_node, 0x1801U, 0x01U,
                    (const uint8_t *)&tpdo2_cob,   sizeof(tpdo2_cob))   != 0U) { return CO_ERROR_INVALID_ARGS; }
    if (co_od_write(&canopen_node, 0x1801U, 0x02U, &tpdo_event, 1U)     != 0U) { return CO_ERROR_INVALID_ARGS; }
    if (co_od_write(&canopen_node, 0x1801U, 0x05U,
                    (const uint8_t *)&tpdo_timer_ms, sizeof(tpdo_timer_ms)) != 0U) { return CO_ERROR_INVALID_ARGS; }

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

        /* 2. Startup sequence once the stack sends bootup and enters PRE_OPERATIONAL:
         *
         *    Step A — self-promote to OPERATIONAL and send NMT Pre-Operational to
         *             the eRob so it is in a known state for SDO access.
         *
         *    Step B — run the SDO configuration state machine to write all eRob
         *             PDO mapping entries one by one (waits for each ACK).  The
         *             eRob must be in PRE_OPERATIONAL for PDO mapping writes; most
         *             CiA 402 drives reject them in OPERATIONAL.
         *
         *    Step C — once every SDO write is acknowledged, send NMT Start so the
         *             eRob enters OPERATIONAL and PDO exchange begins.             */
        if (!m_master_started &&
            canopen_node.nmt_state == CO_NMT_PRE_OPERATIONAL) {
            co_nmt_set_state(&canopen_node, CO_NMT_OPERATIONAL);
        }

        if (!m_master_started &&
            canopen_node.nmt_state == CO_NMT_OPERATIONAL) {
            /* erob_sdo_cfg_run() sends NMT Pre-Op itself on each attempt,
             * waits 100 ms, then writes all eRob PDO mapping entries via SDO.
             * Only after all writes are ACK'd do we send NMT Start.          */
            if (erob_sdo_cfg_run()) {
                (void)co_nmt_master_send(&canopen_node, 0x01U, EROB_NODE_ID);
                m_master_started = true;
            }
        }

        /* 3. RPDO watchdog — if the drive is running but no RPDO1 has arrived
         *    within EROB_RPDO_WATCHDOG_MS, the bus is probably broken.  Send a
         *    Quick Stop immediately; the drive's own RPDO event timer (500 ms)
         *    provides the drive-side redundant safety stop.                   */
        if (m_master_started &&
            m_drive_state == MASTER_CIA402_RUNNING &&
            (now_ms() - m_last_rpdo1_ms) >= EROB_RPDO_WATCHDOG_MS) {
            m_target_vel  = 0;
            m_controlword = CW_QUICK_STOP;
            (void)co_send_tpdo(&canopen_node, 0U);
            m_drive_state = MASTER_CIA402_IDLE;
        }

        /* 4. On each SYNC tick: forward the application target velocity into
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
