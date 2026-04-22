/*
 * eRob CANopen Master — STM32H7 FDCAN (multi-instance)
 *
 * Controls one or more eRob CiA 402 Profile Velocity actuators.
 *
 * Topology
 * ────────
 *   STM32H7 (master, node 0x7F) ←──CAN──→ eRob[0] (node 0x05)
 *                                          eRob[1] (node 0x06)  ← optional
 *                                          ...
 *
 * To add or remove eRob modules edit only the EROB_NODES[] table below.
 * Each instance uses two TPDOs and two RPDOs on the master:
 *   instance i → tpdo_num = i*2    (controlword, mode, target_vel → eRob RPDO1)
 *              → rpdo_num = i*2    (statusword, mode_display, velocity_act ← eRob TPDO1)
 *              → rpdo_num = i*2+1  (position_act, torque_act ← eRob TPDO2)
 * Accel/decel are written once at startup via SDO (0x6083 / 0x6084).
 *
 * OD backing storage uses vendor-specific indices so instances never collide:
 *   TPDO data for instance i: OD index 0x4000 + i  (sub 1–5)
 *   RPDO data for instance i: OD index 0x4100 + i  (sub 1–5)
 */

#include "canopen.h"
#include "canopen_stm32h7_fdcan.h"

extern FDCAN_HandleTypeDef hfdcan1;

/* ── Master configuration ────────────────────────────────────────────────── */
#define MASTER_NODE_ID              0x7FU
#define MASTER_HEARTBEAT_MS         50U
#define EROB_HB_TIMEOUT_MS          150U
#define SYNC_PERIOD_US              1000UL
#define EROB_MAX_VEL_RPM            3000
#define EROB_COUNTS_PER_REV         540000UL    /* counts per output revolution (empirical) */
#define EROB_ENCODER_RPM_TO_PLUS    9000        /* counts/s per RPM = 540000/60 */
#define EROB_STATE_TIMEOUT_MS       2000U
#define EROB_FAULT_RESET_MS         10U
#define EROB_RPDO_WATCHDOG_MS       100U
#define EROB_NMT_RESET_DELAY_MS     500U
#define EROB_NMT_PREOP_DELAY_MS     100U
#define EROB_SDO_TIMEOUT_MS         500U

#define CW_SHUTDOWN             0x0006U
#define CW_SWITCH_ON            0x0007U
#define CW_ENABLE_OPERATION     0x000FU
#define CW_QUICK_STOP           0x000BU
#define CW_FAULT_RESET          0x0080U
#define CW_FAULT_RESET_CLEAR    0x0000U
#define EROB_MODE_PROFILE_VEL   ((int8_t)3)

/* ═══════════════════════════════════════════════════════════════════════════
 * INSTANCE CONFIGURATION — add or remove rows to control more eRob modules.
 * CO_MAX_RPDO / CO_MAX_TPDO in canopen.h must each be >= count * 2.
 * ═══════════════════════════════════════════════════════════════════════════ */
typedef struct {
    uint8_t  node_id;
    uint32_t default_accel;   /* [plus/s] */
    uint32_t default_decel;   /* [plus/s] */
} erob_cfg_t;

static const erob_cfg_t EROB_NODES[] = {
    { 0x05U, 50000U, 200000U },
//    { 0x01U, 5000U, 5000U },
    /* { 0x06U, 5000U, 5000U }, */
};
/* ══════════════════════════════════════════════════════════════════════════ */

#define EROB_N  (sizeof(EROB_NODES) / sizeof(EROB_NODES[0]))

_Static_assert(EROB_N * 2U <= CO_MAX_RPDO,
    "Too many eRob instances — raise CO_MAX_RPDO and CO_MAX_TPDO in canopen.h");

/* ── CANopen stack instance ───────────────────────────────────────────────── */
static co_node_t      canopen_node;
static co_stm32_ctx_t can_ctx;

/* ── Per-instance SDO remote config tables ───────────────────────────────── */
typedef struct {
    uint16_t index;
    uint8_t  subindex;
    uint32_t value;
    uint8_t  size;
} erob_sdo_entry_t;

#define EROB_SDO_STEPS 28U
static erob_sdo_entry_t m_sdo_tbl[EROB_N][EROB_SDO_STEPS];

static void erob_build_sdo_table(uint8_t i)
{
    const uint8_t  nid = EROB_NODES[i].node_id;
    const uint32_t r1d = 0x80000000UL | (0x200UL + nid);
    const uint32_t r1e =                 0x200UL + nid;
    const uint32_t t1d = 0x80000000UL | (0x180UL + nid);
    const uint32_t t1e =                 0x180UL + nid;
    const uint32_t t2d = 0x80000000UL | (0x280UL + nid);
    const uint32_t t2e =                 0x280UL + nid;
    erob_sdo_entry_t *t = m_sdo_tbl[i];
    uint32_t n = 0U;

    /* eRob RPDO1 */
    t[n++] = (erob_sdo_entry_t){ 0x1400U, 0x01U, r1d,          4U };
    t[n++] = (erob_sdo_entry_t){ 0x1400U, 0x02U, 0xFFU,         1U };
    t[n++] = (erob_sdo_entry_t){ 0x1600U, 0x00U, 0U,            1U };
    t[n++] = (erob_sdo_entry_t){ 0x1600U, 0x01U, 0x60400010UL, 4U };
    t[n++] = (erob_sdo_entry_t){ 0x1600U, 0x02U, 0x60600008UL, 4U };
    t[n++] = (erob_sdo_entry_t){ 0x1600U, 0x03U, 0x60FF0020UL, 4U };
    t[n++] = (erob_sdo_entry_t){ 0x1600U, 0x00U, 3U,            1U };
    t[n++] = (erob_sdo_entry_t){ 0x1400U, 0x01U, r1e,           4U };
    t[n++] = (erob_sdo_entry_t){ 0x1400U, 0x05U, 500U,          2U };
    /* eRob TPDO1 */
    t[n++] = (erob_sdo_entry_t){ 0x1800U, 0x01U, t1d,          4U };
    t[n++] = (erob_sdo_entry_t){ 0x1800U, 0x02U, 0x01U,         1U };
    t[n++] = (erob_sdo_entry_t){ 0x1A00U, 0x00U, 0U,            1U };
    t[n++] = (erob_sdo_entry_t){ 0x1A00U, 0x01U, 0x60410010UL, 4U };
    t[n++] = (erob_sdo_entry_t){ 0x1A00U, 0x02U, 0x60610008UL, 4U };
    t[n++] = (erob_sdo_entry_t){ 0x1A00U, 0x03U, 0x606C0020UL, 4U };
    t[n++] = (erob_sdo_entry_t){ 0x1A00U, 0x00U, 3U,            1U };
    t[n++] = (erob_sdo_entry_t){ 0x1800U, 0x01U, t1e,           4U };
    /* eRob TPDO2 */
    t[n++] = (erob_sdo_entry_t){ 0x1801U, 0x01U, t2d,          4U };
    t[n++] = (erob_sdo_entry_t){ 0x1801U, 0x02U, 0x01U,         1U };
    t[n++] = (erob_sdo_entry_t){ 0x1A01U, 0x00U, 0U,            1U };
    t[n++] = (erob_sdo_entry_t){ 0x1A01U, 0x01U, 0x60640020UL, 4U };
    t[n++] = (erob_sdo_entry_t){ 0x1A01U, 0x02U, 0x60770010UL, 4U };
    t[n++] = (erob_sdo_entry_t){ 0x1A01U, 0x00U, 2U,            1U };
    t[n++] = (erob_sdo_entry_t){ 0x1801U, 0x01U, t2e,           4U };
    /* Profile ramp — written once; accel/decel are no longer PDO-mapped */
    t[n++] = (erob_sdo_entry_t){ 0x6083U, 0x00U, EROB_NODES[i].default_accel, 4U };
    t[n++] = (erob_sdo_entry_t){ 0x6084U, 0x00U, EROB_NODES[i].default_decel, 4U };
    /* Heartbeat */
    t[n++] = (erob_sdo_entry_t){ 0x1017U, 0x00U, 50U,           2U };
    t[n++] = (erob_sdo_entry_t){ 0x1016U, 0x01U,
                                  ((uint32_t)MASTER_NODE_ID << 16) | 150U, 4U };
}

/* ── Per-instance runtime state ──────────────────────────────────────────── */
typedef enum {
    EROB_SDO_CFG_IDLE = 0,
    EROB_SDO_CFG_RESET_DELAY,
    EROB_SDO_CFG_NMT_DELAY,
    EROB_SDO_CFG_WAIT_RESPONSE,
    EROB_SDO_CFG_DONE,
} erob_sdo_cfg_state_t;

typedef enum {
    MASTER_CIA402_IDLE = 0,
    MASTER_CIA402_FAULT_RESET,
    MASTER_CIA402_WAIT_FAULT_CLEAR,
    MASTER_CIA402_SHUTDOWN,
    MASTER_CIA402_SWITCH_ON,
    MASTER_CIA402_ENABLE,
    MASTER_CIA402_RUNNING,
    MASTER_CIA402_FAULT,
} master_cia402_state_t;

typedef struct {
    /* OD backing: TPDO1 — master → eRob commands */
    uint16_t controlword;
    int8_t   mode_of_op;
    int32_t  target_vel;
    /* OD backing: RPDO1 — eRob → master feedback */
    uint16_t statusword;
    int8_t   mode_display;
    int32_t  velocity_act;
    /* OD backing: RPDO2 — eRob → master position/torque */
    int32_t  position_act;
    int16_t  torque_act;

    erob_sdo_cfg_state_t sdo_cfg_state;
    uint32_t             sdo_cfg_step;
    uint32_t             sdo_cfg_step_ms;
    bool                 sdo_cfg_resp_ok;

    master_cia402_state_t drive_state;
    uint32_t              state_entry_ms;
    uint32_t              last_rpdo1_ms;

    bool           master_started;
    bool           hb_lost;
    co_nmt_state_t nmt_state;
} erob_state_t;

static erob_state_t m_erob[EROB_N];

/* Last EMCY per instance — inspect in debugger to identify faults. */
volatile uint16_t g_erob_emcy_code[EROB_N];
volatile uint8_t  g_erob_emcy_reg[EROB_N];

/* Per-instance target RPM set from any task context. */
volatile int32_t g_target_rpm[EROB_N];

/* ── CiA 402 statusword helpers ──────────────────────────────────────────── */
#define SW_RTSO  0x0001U
#define SW_SO    0x0002U
#define SW_OE    0x0004U
#define SW_FAULT 0x0008U
#define SW_QS    0x0020U
#define SW_SOD   0x0040U
#define SW_MASK  (SW_RTSO | SW_SO | SW_OE | SW_FAULT | SW_QS | SW_SOD)

static inline bool erob_sw_fault_reaction_active(uint16_t sw)
{
    return (sw & (SW_FAULT | SW_OE | SW_SO | SW_RTSO))
               == (SW_FAULT | SW_OE | SW_SO | SW_RTSO);
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
static inline bool erob_sw_fault(uint16_t sw) { return (sw & SW_FAULT) != 0U; }

static inline uint32_t now_ms(void)
{
    return canopen_node.iface.millis(canopen_node.iface.user);
}

/* ── Instance lookup ─────────────────────────────────────────────────────── */
static int8_t erob_find(uint8_t node_id)
{
    for (uint8_t i = 0U; i < (uint8_t)EROB_N; ++i) {
        if (EROB_NODES[i].node_id == node_id) { return (int8_t)i; }
    }
    return -1;
}

/* ── Instance reset ──────────────────────────────────────────────────────── */
static void erob_reset(uint8_t i, bool full)
{
    m_erob[i].target_vel     = 0;
    m_erob[i].controlword    = CW_SHUTDOWN;
    m_erob[i].drive_state    = MASTER_CIA402_IDLE;
    m_erob[i].master_started = false;
    if (full) {
        m_erob[i].sdo_cfg_state = EROB_SDO_CFG_IDLE;
        m_erob[i].sdo_cfg_step  = 0U;
        m_erob[i].hb_lost       = false;
    }
}

void erob_set_target_rpm(uint16_t node_id_u16, int32_t target_rpm_i32)
{
	g_target_rpm[erob_find(node_id_u16)] = target_rpm_i32;
}

static void erob_set_velocity(uint8_t instance, int32_t rpm)
{
    if (instance >= (uint8_t)EROB_N) { return; }
    if (rpm >  EROB_MAX_VEL_RPM) { rpm =  EROB_MAX_VEL_RPM; }
    if (rpm < -EROB_MAX_VEL_RPM) { rpm = -EROB_MAX_VEL_RPM; }
    m_erob[instance].target_vel = rpm * EROB_ENCODER_RPM_TO_PLUS;
}

/* ── SDO send helper ─────────────────────────────────────────────────────── */
static void erob_sdo_send(uint8_t node_id, uint16_t index, uint8_t subindex,
                          uint32_t value, uint8_t size)
{
    uint8_t cmd;
    switch (size) {
        case 1U: cmd = 0x2FU; break;
        case 2U: cmd = 0x2BU; break;
        case 3U: cmd = 0x27U; break;
        default: cmd = 0x23U; break;
    }
    co_can_frame_t f;
    f.cob_id  = 0x600U + node_id;
    f.len     = 8U;
    f.data[0] = cmd;
    f.data[1] = (uint8_t)(index & 0xFFU);
    f.data[2] = (uint8_t)(index >> 8U);
    f.data[3] = subindex;
    f.data[4] = (uint8_t)(value         & 0xFFU);
    f.data[5] = (uint8_t)((value >>  8U) & 0xFFU);
    f.data[6] = (uint8_t)((value >> 16U) & 0xFFU);
    f.data[7] = (uint8_t)((value >> 24U) & 0xFFU);
    (void)canopen_node.iface.send(canopen_node.iface.user, &f);
}

/* ── NMT state change ────────────────────────────────────────────────────── */
static void on_erob_nmt_change(uint8_t i, co_nmt_state_t st)
{
    switch (st) {
    case CO_NMT_INITIALIZING:
        if (m_erob[i].sdo_cfg_state == EROB_SDO_CFG_RESET_DELAY) { break; }
        erob_reset(i, true);
        break;
    case CO_NMT_PRE_OPERATIONAL:
        erob_reset(i, false);
        if (m_erob[i].sdo_cfg_state != EROB_SDO_CFG_NMT_DELAY &&
            m_erob[i].sdo_cfg_state != EROB_SDO_CFG_RESET_DELAY) {
            m_erob[i].sdo_cfg_state = EROB_SDO_CFG_IDLE;
            m_erob[i].sdo_cfg_step  = 0U;
        }
        break;
    case CO_NMT_STOPPED:
        erob_reset(i, true);
        break;
    case CO_NMT_OPERATIONAL:
        break;
    }
}

/* ── on_rx_frame hook ────────────────────────────────────────────────────── */
static void on_rx_frame(void *user, const co_can_frame_t *frame)
{
    (void)user;

    /* Heartbeat: track per-instance NMT state */
    if (frame->cob_id >= 0x701U && frame->cob_id <= 0x77FU && frame->len >= 1U) {
        const uint8_t nid = (uint8_t)(frame->cob_id - 0x700U);
        const int8_t  idx = erob_find(nid);
        if (idx >= 0) {
            const co_nmt_state_t st = (co_nmt_state_t)(frame->data[0] & 0x7FU);
            if (st != m_erob[idx].nmt_state) {
                m_erob[idx].nmt_state = st;
                on_erob_nmt_change((uint8_t)idx, st);
            }
        }
        return;
    }

    /* EMCY (COB-ID = 0x080 + node_id): capture fault code per instance */
    if (frame->cob_id >= 0x081U && frame->cob_id <= 0x0FFU && frame->len >= 3U) {
        const uint8_t nid = (uint8_t)(frame->cob_id - 0x080U);
        const int8_t  idx = erob_find(nid);
        if (idx >= 0) {
            g_erob_emcy_code[idx] = (uint16_t)frame->data[0]
                                  | ((uint16_t)frame->data[1] << 8);
            g_erob_emcy_reg[idx]  = frame->data[2];
        }
        return;
    }

    /* SDO response (COB-ID = 0x580 + node_id): signal waiting instance */
    if (frame->cob_id >= 0x581U && frame->cob_id <= 0x5FFU && frame->len >= 1U) {
        const uint8_t nid = (uint8_t)(frame->cob_id - 0x580U);
        const int8_t  idx = erob_find(nid);
        if (idx >= 0 &&
            m_erob[idx].sdo_cfg_state == EROB_SDO_CFG_WAIT_RESPONSE) {
            if ((frame->data[0] & 0xE0U) == 0x60U || frame->data[0] == 0x80U) {
                m_erob[idx].sdo_cfg_resp_ok = true;
            }
        }
    }
}

/* ── SDO config state machine (per instance) ─────────────────────────────── */
static bool erob_sdo_cfg_run(uint8_t i)
{
    erob_state_t *e  = &m_erob[i];
    if (e->sdo_cfg_state == EROB_SDO_CFG_DONE) { return true; }

    const uint32_t t   = now_ms();
    const uint8_t  nid = EROB_NODES[i].node_id;

    if (e->sdo_cfg_state == EROB_SDO_CFG_IDLE) {
        (void)co_nmt_master_send(&canopen_node, 0x82U, nid);
        e->sdo_cfg_step    = 0U;
        e->sdo_cfg_resp_ok = false;
        e->sdo_cfg_step_ms = t;
        e->sdo_cfg_state   = EROB_SDO_CFG_RESET_DELAY;
        return false;
    }

    if (e->sdo_cfg_state == EROB_SDO_CFG_RESET_DELAY) {
        if ((t - e->sdo_cfg_step_ms) < EROB_NMT_RESET_DELAY_MS) { return false; }
        (void)co_nmt_master_send(&canopen_node, 0x80U, nid);
        e->sdo_cfg_step_ms = t;
        e->sdo_cfg_state   = EROB_SDO_CFG_NMT_DELAY;
        return false;
    }

    if (e->sdo_cfg_state == EROB_SDO_CFG_NMT_DELAY) {
        if ((t - e->sdo_cfg_step_ms) < EROB_NMT_PREOP_DELAY_MS) { return false; }
        e->sdo_cfg_step_ms = t;
        e->sdo_cfg_state   = EROB_SDO_CFG_WAIT_RESPONSE;
        const erob_sdo_entry_t *w = &m_sdo_tbl[i][0];
        erob_sdo_send(nid, w->index, w->subindex, w->value, w->size);
        return false;
    }

    /* EROB_SDO_CFG_WAIT_RESPONSE */
    if (!e->sdo_cfg_resp_ok) {
        if ((t - e->sdo_cfg_step_ms) >= EROB_SDO_TIMEOUT_MS) {
            e->sdo_cfg_state = EROB_SDO_CFG_IDLE;
            e->sdo_cfg_step  = 0U;
        }
        return false;
    }

    e->sdo_cfg_resp_ok = false;
    e->sdo_cfg_step++;
    if (e->sdo_cfg_step >= EROB_SDO_STEPS) {
        e->sdo_cfg_state = EROB_SDO_CFG_DONE;
        return true;
    }
    e->sdo_cfg_step_ms = t;
    const erob_sdo_entry_t *w = &m_sdo_tbl[i][e->sdo_cfg_step];
    erob_sdo_send(nid, w->index, w->subindex, w->value, w->size);
    return false;
}

/* ── CiA 402 state machine (per instance) ────────────────────────────────── */
static void erob_cia402_step(uint8_t i)
{
    erob_state_t *e   = &m_erob[i];
    const uint32_t t  = now_ms();
    const uint16_t sw = e->statusword;

    switch (e->drive_state) {

    case MASTER_CIA402_IDLE:
        if (erob_sw_fault_reaction_active(sw)) {
            break;
        } else if (erob_sw_fault(sw)) {
            e->controlword    = CW_FAULT_RESET;
            e->drive_state    = MASTER_CIA402_FAULT_RESET;
            e->state_entry_ms = t;
        } else if (sw != 0U) {
            /* Any non-zero, non-fault statusword including Quick Stop Active
             * (0x0003) and Switch On Disabled with QS bit set (0x0060).
             * CiA 402 Table 7 marks QS "don't care" in several states so
             * exact-mask checks miss valid words with unexpected bits set. */
            e->drive_state    = MASTER_CIA402_SHUTDOWN;
            e->state_entry_ms = t;
        }
        break;

    case MASTER_CIA402_FAULT_RESET:
        e->controlword = CW_FAULT_RESET;
        if ((t - e->state_entry_ms) >= EROB_FAULT_RESET_MS) {
            e->controlword    = CW_FAULT_RESET_CLEAR;
            e->drive_state    = MASTER_CIA402_WAIT_FAULT_CLEAR;
            e->state_entry_ms = t;
        }
        break;

    case MASTER_CIA402_WAIT_FAULT_CLEAR:
        e->controlword = CW_FAULT_RESET_CLEAR;
        if (!erob_sw_fault(sw)) {
            e->drive_state    = MASTER_CIA402_SHUTDOWN;
            e->state_entry_ms = t;
        } else if ((t - e->state_entry_ms) >= EROB_STATE_TIMEOUT_MS) {
            e->drive_state    = MASTER_CIA402_FAULT_RESET;
            e->state_entry_ms = t;
        }
        break;

    case MASTER_CIA402_SHUTDOWN:
        e->controlword = CW_SHUTDOWN;
        if (erob_sw_ready_to_switch_on(sw)) {
            e->drive_state    = MASTER_CIA402_SWITCH_ON;
            e->state_entry_ms = t;
        } else if (erob_sw_fault(sw)) {
            e->drive_state    = MASTER_CIA402_FAULT;
            e->state_entry_ms = t;
        } else if ((t - e->state_entry_ms) >= EROB_STATE_TIMEOUT_MS) {
            e->drive_state    = MASTER_CIA402_FAULT_RESET;
            e->state_entry_ms = t;
        }
        break;

    case MASTER_CIA402_SWITCH_ON:
        e->controlword = CW_SWITCH_ON;
        if (erob_sw_switched_on(sw)) {
            e->drive_state    = MASTER_CIA402_ENABLE;
            e->state_entry_ms = t;
        } else if (erob_sw_fault(sw)) {
            e->drive_state    = MASTER_CIA402_FAULT;
            e->state_entry_ms = t;
        } else if ((t - e->state_entry_ms) >= EROB_STATE_TIMEOUT_MS) {
            e->drive_state    = MASTER_CIA402_FAULT_RESET;
            e->state_entry_ms = t;
        }
        break;

    case MASTER_CIA402_ENABLE:
        e->controlword = CW_ENABLE_OPERATION;
        if (erob_sw_operation_enabled(sw)) {
            e->drive_state    = MASTER_CIA402_RUNNING;
            e->state_entry_ms = t;
        } else if (erob_sw_fault(sw)) {
            e->drive_state    = MASTER_CIA402_FAULT;
            e->state_entry_ms = t;
        } else if ((t - e->state_entry_ms) >= EROB_STATE_TIMEOUT_MS) {
            e->drive_state    = MASTER_CIA402_FAULT_RESET;
            e->state_entry_ms = t;
        }
        break;

    case MASTER_CIA402_RUNNING:
        if (erob_sw_fault(sw)) {
            e->target_vel     = 0;
            e->controlword    = CW_FAULT_RESET_CLEAR;
            e->drive_state    = MASTER_CIA402_FAULT;
            e->state_entry_ms = t;
        } else if (!erob_sw_operation_enabled(sw)) {
            /* Drive dropped out of Operation Enabled without asserting Fault
             * (e.g. Quick Stop Active, Switch On Disabled after cable-replug,
             * or an inhibit input).  Zero velocity and fall back to IDLE so
             * the state machine re-reads the live statusword on the next
             * RPDO1 and runs the correct CiA 402 re-enable sequence. */
            e->target_vel     = 0;
            e->controlword    = CW_SHUTDOWN;
            e->drive_state    = MASTER_CIA402_IDLE;
            e->state_entry_ms = t;
        } else {
            e->controlword = CW_ENABLE_OPERATION;
        }
        break;

    case MASTER_CIA402_FAULT:
        e->target_vel = 0;
        if (erob_sw_fault_reaction_active(sw)) {
            e->controlword    = CW_FAULT_RESET_CLEAR;
            e->state_entry_ms = t;
            break;
        }
        if ((t - e->state_entry_ms) < EROB_FAULT_RESET_MS) {
            e->controlword = CW_FAULT_RESET;
        } else {
            e->controlword = CW_FAULT_RESET_CLEAR;
            if (!erob_sw_fault(sw)) {
                e->drive_state    = MASTER_CIA402_SHUTDOWN;
                e->state_entry_ms = t;
            } else if ((t - e->state_entry_ms) >= EROB_STATE_TIMEOUT_MS) {
                e->state_entry_ms = t;
            }
        }
        break;
    }
}

/* ── RPDO frame callback ─────────────────────────────────────────────────── */
static void on_rpdo_frame(co_node_t *node, uint8_t rpdo_num, void *user)
{
    (void)node; (void)user;
    const uint8_t i = rpdo_num / 2U;
    if (i >= (uint8_t)EROB_N) { return; }
    if (rpdo_num % 2U == 0U) {   /* RPDO1: statusword + mode + velocity */
        m_erob[i].last_rpdo1_ms = now_ms();
        erob_cia402_step(i);
    }
    /* RPDO2 (position + torque) needs no immediate action */
}

/* ── Heartbeat consumer callback ─────────────────────────────────────────── */
static void on_hb_event(co_node_t *node, uint8_t slave_node_id,
                        co_hb_event_t event, void *user)
{
    (void)node; (void)user;
    const int8_t idx = erob_find(slave_node_id);
    if (idx < 0) { return; }
    const uint8_t i = (uint8_t)idx;

    if (event == CO_HB_EVENT_TIMEOUT) {
        /* Ignore timeouts that fire before the drive has been started.
         * erob_sdo_cfg_run() issues an NMT reset (0x82) as its first step,
         * which takes the drive offline for ~500 ms — longer than
         * EROB_HB_TIMEOUT_MS.  That expected gap must not disrupt the SDO
         * config sequence.  Recovery for drives that never completed startup
         * is handled by on_erob_nmt_change() via the NMT heartbeat callbacks,
         * not through this path. */
        if (!m_erob[i].master_started) { return; }
        m_erob[i].target_vel  = 0;
        m_erob[i].controlword = CW_QUICK_STOP;
        (void)co_send_tpdo(&canopen_node, (uint8_t)(i * 2U));
        m_erob[i].controlword    = CW_SHUTDOWN;
        m_erob[i].drive_state    = MASTER_CIA402_IDLE;
        m_erob[i].sdo_cfg_state  = EROB_SDO_CFG_IDLE;
        m_erob[i].sdo_cfg_step   = 0U;
        m_erob[i].master_started = false;
        m_erob[i].hb_lost        = true;
    } else {
        if (!m_erob[i].hb_lost) { return; }
        m_erob[i].hb_lost        = false;
        m_erob[i].sdo_cfg_state  = EROB_SDO_CFG_IDLE;
        m_erob[i].sdo_cfg_step   = 0U;
        m_erob[i].master_started = false;
        m_erob[i].drive_state    = MASTER_CIA402_IDLE;
        m_erob[i].state_entry_ms = now_ms();
    }
}

/* ── NMT Reset Communication callback ───────────────────────────────────────*/
static void on_reset_communication(void *user)
{
    (void)user;
    for (uint8_t i = 0U; i < (uint8_t)EROB_N; ++i) {
        erob_reset(i, true);
    }
}


/* ── Public API ──────────────────────────────────────────────────────────── */

void erob_get_pos_vel(uint16_t node_id_u16, float *const p_pos_deg_f32,
                      float *const p_vel_rpm_f32, float *const p_torque_pct_f32)
{
    const int8_t idx = erob_find((uint8_t)node_id_u16);
    if (idx < 0) { return; }
    /* position: counts × (360 / 524288) */
    *p_pos_deg_f32    = (float)m_erob[idx].position_act
                        * (360.0f / (float)EROB_COUNTS_PER_REV);
    /* velocity: counts/s ÷ (524288/60) = RPM */
    *p_vel_rpm_f32    = (float)m_erob[idx].velocity_act
                        / (float)EROB_ENCODER_RPM_TO_PLUS;
    /* torque: CiA 402 0x6077 is in 0.1 % of rated torque */
    *p_torque_pct_f32 = (float)m_erob[idx].torque_act * 0.1f;
}

/* ── Initialization ──────────────────────────────────────────────────────── */
co_error_t app_canopen_init(void)
{
    co_error_t err;

    co_stm32_attach(&canopen_node, &hfdcan1, &can_ctx,
                    MASTER_NODE_ID, MASTER_HEARTBEAT_MS);
    canopen_node.iface.on_reset_communication = on_reset_communication;
    canopen_node.iface.on_rx_frame            = on_rx_frame;

    for (uint8_t i = 0U; i < (uint8_t)EROB_N; ++i) {
        erob_build_sdo_table(i);

        m_erob[i].controlword = CW_SHUTDOWN;
        m_erob[i].mode_of_op  = EROB_MODE_PROFILE_VEL;
        m_erob[i].drive_state = MASTER_CIA402_IDLE;
        m_erob[i].nmt_state     = CO_NMT_INITIALIZING;

        /* Vendor-specific OD indices avoid collision between instances.
         * Instance i TPDO data: 0x4000+i,  RPDO data: 0x4100+i  (sub 1–5) */
        const uint16_t ti = (uint16_t)(0x4000U + i);
        const uint16_t ri = (uint16_t)(0x4100U + i);

        err = co_od_add(&canopen_node, ti, 0x01U,
                        (uint8_t *)&m_erob[i].controlword,   2U, true, true);
        if (err != CO_ERROR_NONE) { return err; }
        err = co_od_add(&canopen_node, ti, 0x02U,
                        (uint8_t *)&m_erob[i].mode_of_op,    1U, true, true);
        if (err != CO_ERROR_NONE) { return err; }
        err = co_od_add(&canopen_node, ti, 0x03U,
                        (uint8_t *)&m_erob[i].target_vel,    4U, true, true);
        if (err != CO_ERROR_NONE) { return err; }

        err = co_od_add(&canopen_node, ri, 0x01U,
                        (uint8_t *)&m_erob[i].statusword,    2U, true, true);
        if (err != CO_ERROR_NONE) { return err; }
        err = co_od_add(&canopen_node, ri, 0x02U,
                        (uint8_t *)&m_erob[i].mode_display,  1U, true, true);
        if (err != CO_ERROR_NONE) { return err; }
        err = co_od_add(&canopen_node, ri, 0x03U,
                        (uint8_t *)&m_erob[i].velocity_act,  4U, true, true);
        if (err != CO_ERROR_NONE) { return err; }
        err = co_od_add(&canopen_node, ri, 0x04U,
                        (uint8_t *)&m_erob[i].position_act,  4U, true, true);
        if (err != CO_ERROR_NONE) { return err; }
        err = co_od_add(&canopen_node, ri, 0x05U,
                        (uint8_t *)&m_erob[i].torque_act,    2U, true, true);
        if (err != CO_ERROR_NONE) { return err; }

        /* RPDO comm + mapping.  Instance i uses rpdo_num = i*2 and i*2+1. */
        const uint8_t  rn0      = (uint8_t)(i * 2U);
        const uint8_t  rn1      = (uint8_t)(i * 2U + 1U);
        const uint32_t rcob0    = 0x180UL + EROB_NODES[i].node_id;
        const uint32_t rcob1    = 0x280UL + EROB_NODES[i].node_id;
        const uint8_t  rpdo_syn = 0x01U;
        const uint8_t  z = 0U, two = 2U, three = 3U;

        if (co_od_write(&canopen_node, 0x1400U + rn0, 0x01U,
                        (const uint8_t *)&rcob0, 4U) != 0U) { return CO_ERROR_INVALID_ARGS; }
        if (co_od_write(&canopen_node, 0x1400U + rn0, 0x02U,
                        &rpdo_syn, 1U) != 0U) { return CO_ERROR_INVALID_ARGS; }

        const uint32_t r1m1 = ((uint32_t)ri << 16) | (0x01UL << 8) | 16UL;
        const uint32_t r1m2 = ((uint32_t)ri << 16) | (0x02UL << 8) |  8UL;
        const uint32_t r1m3 = ((uint32_t)ri << 16) | (0x03UL << 8) | 32UL;
        if (co_od_write(&canopen_node, 0x1600U + rn0, 0x00U, &z,    1U) != 0U) { return CO_ERROR_INVALID_ARGS; }
        if (co_od_write(&canopen_node, 0x1600U + rn0, 0x01U, (const uint8_t *)&r1m1, 4U) != 0U) { return CO_ERROR_INVALID_ARGS; }
        if (co_od_write(&canopen_node, 0x1600U + rn0, 0x02U, (const uint8_t *)&r1m2, 4U) != 0U) { return CO_ERROR_INVALID_ARGS; }
        if (co_od_write(&canopen_node, 0x1600U + rn0, 0x03U, (const uint8_t *)&r1m3, 4U) != 0U) { return CO_ERROR_INVALID_ARGS; }
        if (co_od_write(&canopen_node, 0x1600U + rn0, 0x00U, &three, 1U) != 0U) { return CO_ERROR_INVALID_ARGS; }

        if (co_od_write(&canopen_node, 0x1400U + rn1, 0x01U,
                        (const uint8_t *)&rcob1, 4U) != 0U) { return CO_ERROR_INVALID_ARGS; }
        if (co_od_write(&canopen_node, 0x1400U + rn1, 0x02U,
                        &rpdo_syn, 1U) != 0U) { return CO_ERROR_INVALID_ARGS; }

        const uint32_t r2m1 = ((uint32_t)ri << 16) | (0x04UL << 8) | 32UL;
        const uint32_t r2m2 = ((uint32_t)ri << 16) | (0x05UL << 8) | 16UL;
        if (co_od_write(&canopen_node, 0x1600U + rn1, 0x00U, &z,   1U) != 0U) { return CO_ERROR_INVALID_ARGS; }
        if (co_od_write(&canopen_node, 0x1600U + rn1, 0x01U, (const uint8_t *)&r2m1, 4U) != 0U) { return CO_ERROR_INVALID_ARGS; }
        if (co_od_write(&canopen_node, 0x1600U + rn1, 0x02U, (const uint8_t *)&r2m2, 4U) != 0U) { return CO_ERROR_INVALID_ARGS; }
        if (co_od_write(&canopen_node, 0x1600U + rn1, 0x00U, &two,  1U) != 0U) { return CO_ERROR_INVALID_ARGS; }

        /* TPDO comm + mapping.  Instance i uses tpdo_num = i*2 only. */
        const uint8_t  tn0   = (uint8_t)(i * 2U);
        const uint32_t tcob0 = 0x200UL + EROB_NODES[i].node_id;
        const uint8_t  tev   = 0xFFU;
        const uint16_t tms   = 1U;

        if (co_od_write(&canopen_node, 0x1800U + tn0, 0x01U,
                        (const uint8_t *)&tcob0, 4U) != 0U) { return CO_ERROR_INVALID_ARGS; }
        if (co_od_write(&canopen_node, 0x1800U + tn0, 0x02U, &tev, 1U) != 0U) { return CO_ERROR_INVALID_ARGS; }
        if (co_od_write(&canopen_node, 0x1800U + tn0, 0x05U,
                        (const uint8_t *)&tms, 2U) != 0U) { return CO_ERROR_INVALID_ARGS; }

        const uint32_t t1m1 = ((uint32_t)ti << 16) | (0x01UL << 8) | 16UL;
        const uint32_t t1m2 = ((uint32_t)ti << 16) | (0x02UL << 8) |  8UL;
        const uint32_t t1m3 = ((uint32_t)ti << 16) | (0x03UL << 8) | 32UL;
        if (co_od_write(&canopen_node, 0x1A00U + tn0, 0x00U, &z,    1U) != 0U) { return CO_ERROR_INVALID_ARGS; }
        if (co_od_write(&canopen_node, 0x1A00U + tn0, 0x01U, (const uint8_t *)&t1m1, 4U) != 0U) { return CO_ERROR_INVALID_ARGS; }
        if (co_od_write(&canopen_node, 0x1A00U + tn0, 0x02U, (const uint8_t *)&t1m2, 4U) != 0U) { return CO_ERROR_INVALID_ARGS; }
        if (co_od_write(&canopen_node, 0x1A00U + tn0, 0x03U, (const uint8_t *)&t1m3, 4U) != 0U) { return CO_ERROR_INVALID_ARGS; }
        if (co_od_write(&canopen_node, 0x1A00U + tn0, 0x00U, &three, 1U) != 0U) { return CO_ERROR_INVALID_ARGS; }
    }

    co_set_rpdo_frame_hook(&canopen_node, on_rpdo_frame, NULL);
    co_set_hb_event_hook(&canopen_node,  on_hb_event,   NULL);

    /* SYNC producer: master generates COB-ID 0x080 every 1 ms */
    const uint32_t sync_cob_id   = 0x40000080UL;
    const uint32_t sync_cycle_us = SYNC_PERIOD_US;
    if (co_od_write(&canopen_node, 0x1005U, 0x00U,
                    (const uint8_t *)&sync_cob_id,   4U) != 0U) { return CO_ERROR_INVALID_ARGS; }
    if (co_od_write(&canopen_node, 0x1006U, 0x00U,
                    (const uint8_t *)&sync_cycle_us, 4U) != 0U) { return CO_ERROR_INVALID_ARGS; }

    /* Heartbeat consumer: one sub-index per eRob instance */
    const uint8_t hb_count = (uint8_t)EROB_N;
    if (co_od_write(&canopen_node, 0x1016U, 0x00U,
                    &hb_count, 1U) != 0U) { return CO_ERROR_INVALID_ARGS; }
    for (uint8_t i = 0U; i < (uint8_t)EROB_N; ++i) {
        const uint32_t hb = ((uint32_t)EROB_NODES[i].node_id << 16)
                          | (uint32_t)EROB_HB_TIMEOUT_MS;
        if (co_od_write(&canopen_node, 0x1016U, (uint8_t)(i + 1U),
                        (const uint8_t *)&hb, 4U) != 0U) { return CO_ERROR_INVALID_ARGS; }
    }

    return CO_ERROR_NONE;
}

/* ── FDCAN RX interrupt ──────────────────────────────────────────────────── */

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    (void)RxFifo0ITs;
    if (hfdcan == &hfdcan1) {
        co_stm32_rx_isr(&canopen_node, &can_ctx);
    }
}

/* ── Main loop ───────────────────────────────────────────────────────────── */
void app_canopen_loop(void)
{
	/* 1. Deliver queued frames (heartbeat timestamps already updated by ISR). */
	co_stm32_drain_rx(&canopen_node, &can_ctx);

	/* 2. Self-promote master to OPERATIONAL once it reaches PRE_OPERATIONAL. */
	if (canopen_node.nmt_state == CO_NMT_PRE_OPERATIONAL) {
		co_nmt_set_state(&canopen_node, CO_NMT_OPERATIONAL);
	}

	/* 3. Per-instance startup: SDO config → NMT Start. */
	if (canopen_node.nmt_state == CO_NMT_OPERATIONAL) {
		for (uint8_t i = 0U; i < (uint8_t)EROB_N; ++i) {
			if (!m_erob[i].master_started && erob_sdo_cfg_run(i)) {
				(void)co_nmt_master_send(&canopen_node, 0x01U,
										EROB_NODES[i].node_id);
				m_erob[i].master_started = true;
			}
		}
	}

	/* 4. RPDO watchdog: if RUNNING but no RPDO1 for EROB_RPDO_WATCHDOG_MS,
	 *    send Quick Stop; drive's own event timer provides redundant stop. */
	const uint32_t now = now_ms();
	for (uint8_t i = 0U; i < (uint8_t)EROB_N; ++i) {
		if (m_erob[i].master_started &&
			m_erob[i].drive_state == MASTER_CIA402_RUNNING &&
			(now - m_erob[i].last_rpdo1_ms) >= EROB_RPDO_WATCHDOG_MS) {
			m_erob[i].target_vel  = 0;
			m_erob[i].controlword = CW_QUICK_STOP;
			(void)co_send_tpdo(&canopen_node, (uint8_t)(i * 2U));
			m_erob[i].drive_state = MASTER_CIA402_IDLE;
		}
	}
}

void co_transmit_process(void)
{
    /* 5. Forward application target velocities on each SYNC tick. */
    if (canopen_node.sync_event_pending) {
        canopen_node.sync_event_pending = false;
        for (uint8_t i = 0U; i < (uint8_t)EROB_N; ++i) {
            erob_set_velocity(i, (int32_t)g_target_rpm[i]);
        }
    }

    /* Run CANopen stack: SYNC, TPDOs, heartbeat, HB consumer timeout. */
	co_process(&canopen_node);
}
