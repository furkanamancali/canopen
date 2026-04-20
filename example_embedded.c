/*
 * eRob Actuator CANopen Node — STM32H7 FDCAN
 *
 * This file is the top-level application glue for a single-axis eRob
 * integrated joint actuator driven over CiA 301 / CiA 402 (Profile Velocity).
 *
 * PDO layout (eRob profile):
 *
 *   RPDO1  COB-ID 0x200+node_id  (56 bits, SYNC-driven)
 *     Bytes 0-1  : Controlword          0x6040:00  16 bits
 *     Byte  2    : Modes of Operation   0x6060:00   8 bits
 *     Bytes 3-6  : Target Velocity      0x60FF:00  32 bits  [RPM, signed]
 *
 *   RPDO2  COB-ID 0x300+node_id  (64 bits, SYNC-driven)
 *     Bytes 0-3  : Profile Acceleration 0x6083:00  32 bits  [RPM/s]
 *     Bytes 4-7  : Profile Deceleration 0x6084:00  32 bits  [RPM/s]
 *
 *   TPDO1  COB-ID 0x180+node_id  (56 bits, every SYNC)
 *     Bytes 0-1  : Statusword           0x6041:00  16 bits
 *     Byte  2    : Mode Display         0x6061:00   8 bits
 *     Bytes 3-6  : Velocity Actual      0x606C:00  32 bits  [RPM, signed]
 *
 *   TPDO2  COB-ID 0x280+node_id  (48 bits, every SYNC)
 *     Bytes 0-3  : Position Actual      0x6064:00  32 bits  [encoder counts]
 *     Bytes 4-5  : Torque Actual        0x6077:00  16 bits  [0.1 % rated]
 *
 * SYNC period : 1 ms  (producer COB-ID 0x080)
 * Heartbeat   : 100 ms producer; consumer monitors master at node 0x7F, 1 s timeout
 * Node ID     : 0x01  (change EROB_NODE_ID below to match hardware DIP/EEP setting)
 */

#include "canopen.h"
#include "cia402.h"
#include "canopen_stm32h7_fdcan.h"

extern FDCAN_HandleTypeDef hfdcan1;

/* ── Node configuration ───────────────────────────────────────────────────── */
#define EROB_NODE_ID             0x01U
#define EROB_HEARTBEAT_MS        100U
#define EROB_MASTER_NODE_ID      0x7FU
#define EROB_MASTER_HB_TIMEOUT   1000U   /* ms – NMT lost-comms detection     */
#define EROB_SYNC_PERIOD_US      1000UL  /* 1 ms SYNC                         */

/* ── eRob physical limits (adapt to your actuator variant) ───────────────── */
#define EROB_MAX_VEL_RPM         3000U   /* rated max speed [RPM]             */
#define EROB_DEFAULT_ACCEL       500U    /* default ramp accel [RPM/s]        */
#define EROB_DEFAULT_DECEL       500U    /* default ramp decel [RPM/s]        */
#define EROB_DEFAULT_QS_DECEL    2000U   /* quick-stop decel  [RPM/s]         */
#define EROB_DEFAULT_VEL_WINDOW  50U     /* velocity window   [RPM]           */
#define EROB_DEFAULT_VEL_WIN_MS  50U     /* window settle time [ms]           */

/* ── CANopen stack instances ──────────────────────────────────────────────── */
static co_node_t    canopen_node;
static co_stm32_ctx_t can_ctx;
static cia402_axis_t  axis;

/* ── Hardware interface (written by motor ISR / DMA completion) ───────────── */
static volatile int32_t  g_vel_feedback_rpm    = 0;   /* [RPM, signed]        */
static volatile int32_t  g_pos_feedback_counts = 0;   /* [encoder counts]     */
static volatile int16_t  g_trq_feedback_pct    = 0;   /* [0.1 % rated torque] */
static volatile bool     g_inverter_fault      = false;

/* ── Software trapezoidal velocity ramp ──────────────────────────────────── */
static struct {
    int32_t  commanded_rpm;   /* current output to motor inverter [RPM]  */
    uint32_t last_tick_ms;    /* HAL_GetTick() at previous ramp step     */
} g_ramp;

/*
 * Step the trapezoidal ramp from `current` toward `target` in `dt_ms` ms,
 * bounded by accel_rpm_s (acceleration rate) or decel_rpm_s (deceleration).
 * Returns the new commanded value.
 */
static int32_t ramp_step(int32_t current, int32_t target,
                         uint32_t accel_rpm_s, uint32_t decel_rpm_s,
                         uint32_t dt_ms)
{
    int32_t delta = target - current;
    int32_t limit;

    if (delta > 0) {
        limit = (int32_t)((accel_rpm_s * dt_ms + 500U) / 1000U);
        if (delta > limit) {
            delta = limit;
        }
    } else if (delta < 0) {
        limit = (int32_t)((decel_rpm_s * dt_ms + 500U) / 1000U);
        if (-delta > limit) {
            delta = -limit;
        }
    }

    return current + delta;
}

/* ── CiA 402 application callbacks ───────────────────────────────────────── */

/*
 * on_feedback: called at the start of every cia402_step().
 * Snapshot hardware feedback into the axis struct so statusword and
 * velocity-window checks operate on fresh data.
 */
static void erob_on_feedback(cia402_axis_t *a, int8_t mode, void *user)
{
    (void)mode;
    (void)user;
    a->position_actual = g_pos_feedback_counts;
    a->velocity_actual  = g_vel_feedback_rpm;
    a->torque_actual    = g_trq_feedback_pct;
}

/*
 * on_profile_velocity: called each cia402_step() while OPERATION_ENABLED
 * and mode == Profile Velocity (3).
 * Clamps target to the physical limit and advances the trapezoidal ramp.
 */
static bool erob_on_profile_velocity(cia402_axis_t *a, void *user)
{
    (void)user;

    uint32_t now_ms = HAL_GetTick();
    uint32_t dt_ms  = now_ms - g_ramp.last_tick_ms;
    if (dt_ms == 0U) {
        dt_ms = 1U;
    }
    g_ramp.last_tick_ms = now_ms;

    /* Clamp target to actuator's rated limit. */
    int32_t target = a->target_velocity;
    if (target >  (int32_t)EROB_MAX_VEL_RPM) {
        target =  (int32_t)EROB_MAX_VEL_RPM;
    } else if (target < -(int32_t)EROB_MAX_VEL_RPM) {
        target = -(int32_t)EROB_MAX_VEL_RPM;
    }

    g_ramp.commanded_rpm = ramp_step(g_ramp.commanded_rpm, target,
                                     a->profile_acceleration,
                                     a->profile_deceleration,
                                     dt_ms);

    /* TODO: write g_ramp.commanded_rpm to motor inverter
     *       e.g. via SPI, UART, PWM compare register, or shared memory.    */
    return true;
}

/*
 * on_quick_stop_reaction_check: called each cia402_step() while in QUICK_STOP.
 * Ramps commanded velocity to zero at the quick-stop deceleration rate.
 * Returns true (reaction complete) when the drive has reached standstill.
 */
static bool erob_on_quick_stop_reaction_check(cia402_axis_t *a, void *user)
{
    (void)user;

    uint32_t now_ms = HAL_GetTick();
    uint32_t dt_ms  = now_ms - g_ramp.last_tick_ms;
    if (dt_ms == 0U) {
        dt_ms = 1U;
    }
    g_ramp.last_tick_ms = now_ms;

    g_ramp.commanded_rpm = ramp_step(g_ramp.commanded_rpm, 0,
                                     a->quick_stop_deceleration,
                                     a->quick_stop_deceleration,
                                     dt_ms);

    /* TODO: write g_ramp.commanded_rpm to motor inverter. */
    return (g_ramp.commanded_rpm == 0);
}

/*
 * on_fault_reaction_check: called each cia402_step() while FAULT_REACTION_ACTIVE.
 * Immediately command safe torque off; stack transitions to FAULT after this
 * returns true.
 */
static bool erob_on_fault_reaction_check(cia402_axis_t *a, void *user)
{
    (void)a;
    (void)user;
    g_ramp.commanded_rpm = 0;
    /* TODO: assert motor STO (safe torque off) via hardware enable line. */
    return true;
}

/* Application interface registered with the CiA 402 stack. */
static const cia402_app_if_t g_erob_app = {
    .user                         = NULL,
    .supported_modes              = CIA402_MODE_BIT(CIA402_MODE_PROFILE_VELOCITY),
    .on_feedback                  = erob_on_feedback,
    .on_profile_position          = NULL,
    .on_velocity                  = NULL,
    .on_profile_velocity          = erob_on_profile_velocity,
    .on_profile_torque            = NULL,
    .on_homing                    = NULL,
    .on_interpolated_position     = NULL,
    .on_cyclic_sync_position      = NULL,
    .on_cyclic_sync_velocity      = NULL,
    .on_cyclic_sync_torque        = NULL,
    .on_quick_stop_reaction_check = erob_on_quick_stop_reaction_check,
    .on_fault_reaction_check      = erob_on_fault_reaction_check,
};

/* ── Hardware fault monitor ──────────────────────────────────────────────── */
static void fault_monitor_step(void)
{
    if (g_inverter_fault) {
        (void)co_fault_raise(&canopen_node,
                             CO_FAULT_CIA402_PROFILE,
                             CO_EMCY_ERR_PROFILE,
                             CO_ERROR_REG_DEVICE_PROFILE,
                             0x01U,  /* MSEF: drive hardware fault */
                             NULL);
    } else {
        (void)co_fault_clear(&canopen_node, CO_FAULT_CIA402_PROFILE, 0x00U, NULL);
    }
}

/* ── Initialization ──────────────────────────────────────────────────────── */
void app_canopen_init(void)
{
    /* 1. Bind the CANopen stack to STM32H7 FDCAN1. */
    co_stm32_attach(&canopen_node, &hfdcan1, &can_ctx,
                    EROB_NODE_ID, EROB_HEARTBEAT_MS);

    /* 2. Initialise the CiA 402 axis, register all OD entries (0x6040–0x606E),
     *    then wire the eRob application callbacks. */
    cia402_init(&axis);
    cia402_bind_od(&axis, &canopen_node);
    cia402_set_callbacks(&axis, &g_erob_app);

    /* 3. Set default velocity profile parameters.
     *    The master may override any of these at runtime via SDO or RPDO2. */
    axis.mode_of_operation            = CIA402_MODE_PROFILE_VELOCITY;
    axis.profile_velocity             = EROB_MAX_VEL_RPM;
    axis.profile_velocity_valid       = true;
    axis.profile_acceleration         = EROB_DEFAULT_ACCEL;
    axis.profile_acceleration_valid   = true;
    axis.profile_deceleration         = EROB_DEFAULT_DECEL;
    axis.profile_deceleration_valid   = true;
    axis.quick_stop_deceleration      = EROB_DEFAULT_QS_DECEL;
    axis.quick_stop_deceleration_valid = true;
    axis.velocity_window              = EROB_DEFAULT_VEL_WINDOW;
    axis.velocity_window_time         = EROB_DEFAULT_VEL_WIN_MS;
    axis.velocity_window_valid        = true;

    /* 4. PDO mapping — eRob profile layout.
     *
     * Encoding: (index << 16) | (subindex << 8) | bit_length
     *
     * RPDO1 (0x1600): Controlword[16] | ModeOfOp[8] | TargetVelocity[32]
     * RPDO2 (0x1601): ProfileAccel[32] | ProfileDecel[32]
     * TPDO1 (0x1A00): Statusword[16]  | ModeDisplay[8] | VelocityActual[32]
     * TPDO2 (0x1A01): PositionActual[32] | TorqueActual[16]
     */

    /* RPDO1 */
    const uint32_t r1m1 = (0x6040UL << 16) | (0x00UL << 8) | 16UL;
    const uint32_t r1m2 = (0x6060UL << 16) | (0x00UL << 8) |  8UL;
    const uint32_t r1m3 = (0x60FFUL << 16) | (0x00UL << 8) | 32UL;
    (void)co_od_write(&canopen_node, 0x1600, 0x00, (const uint8_t[]){0U}, 1U);
    (void)co_od_write(&canopen_node, 0x1600, 0x01, (const uint8_t *)&r1m1, 4U);
    (void)co_od_write(&canopen_node, 0x1600, 0x02, (const uint8_t *)&r1m2, 4U);
    (void)co_od_write(&canopen_node, 0x1600, 0x03, (const uint8_t *)&r1m3, 4U);
    (void)co_od_write(&canopen_node, 0x1600, 0x00, (const uint8_t[]){3U}, 1U);

    /* RPDO2 — ramp parameters writable at runtime from master */
    const uint32_t r2m1 = (0x6083UL << 16) | (0x00UL << 8) | 32UL;
    const uint32_t r2m2 = (0x6084UL << 16) | (0x00UL << 8) | 32UL;
    (void)co_od_write(&canopen_node, 0x1601, 0x00, (const uint8_t[]){0U}, 1U);
    (void)co_od_write(&canopen_node, 0x1601, 0x01, (const uint8_t *)&r2m1, 4U);
    (void)co_od_write(&canopen_node, 0x1601, 0x02, (const uint8_t *)&r2m2, 4U);
    (void)co_od_write(&canopen_node, 0x1601, 0x00, (const uint8_t[]){2U}, 1U);

    /* TPDO1 */
    const uint32_t t1m1 = (0x6041UL << 16) | (0x00UL << 8) | 16UL;
    const uint32_t t1m2 = (0x6061UL << 16) | (0x00UL << 8) |  8UL;
    const uint32_t t1m3 = (0x606CUL << 16) | (0x00UL << 8) | 32UL;
    (void)co_od_write(&canopen_node, 0x1A00, 0x00, (const uint8_t[]){0U}, 1U);
    (void)co_od_write(&canopen_node, 0x1A00, 0x01, (const uint8_t *)&t1m1, 4U);
    (void)co_od_write(&canopen_node, 0x1A00, 0x02, (const uint8_t *)&t1m2, 4U);
    (void)co_od_write(&canopen_node, 0x1A00, 0x03, (const uint8_t *)&t1m3, 4U);
    (void)co_od_write(&canopen_node, 0x1A00, 0x00, (const uint8_t[]){3U}, 1U);

    /* TPDO2 */
    const uint32_t t2m1 = (0x6064UL << 16) | (0x00UL << 8) | 32UL;
    const uint32_t t2m2 = (0x6077UL << 16) | (0x00UL << 8) | 16UL;
    (void)co_od_write(&canopen_node, 0x1A01, 0x00, (const uint8_t[]){0U}, 1U);
    (void)co_od_write(&canopen_node, 0x1A01, 0x01, (const uint8_t *)&t2m1, 4U);
    (void)co_od_write(&canopen_node, 0x1A01, 0x02, (const uint8_t *)&t2m2, 4U);
    (void)co_od_write(&canopen_node, 0x1A01, 0x00, (const uint8_t[]){2U}, 1U);

    /* 5. SYNC consumer: receive 0x080 at 1 ms; both TPDOs transmit on every SYNC. */
    const uint32_t sync_cob_id   = 0x00000080UL;
    const uint32_t sync_cycle_us = EROB_SYNC_PERIOD_US;
    const uint8_t  sync_type     = 1U;  /* transmit on every SYNC event */
    (void)co_od_write(&canopen_node, 0x1005, 0x00,
                      (const uint8_t *)&sync_cob_id,   sizeof(sync_cob_id));
    (void)co_od_write(&canopen_node, 0x1006, 0x00,
                      (const uint8_t *)&sync_cycle_us, sizeof(sync_cycle_us));
    (void)co_od_write(&canopen_node, 0x1800, 0x02, &sync_type, 1U);  /* TPDO1 */
    (void)co_od_write(&canopen_node, 0x1801, 0x02, &sync_type, 1U);  /* TPDO2 */

    /* 6. Heartbeat consumer: detect master (0x7F) silence within 1 s. */
    const uint8_t  hb_count  = 1U;
    const uint32_t hb_master = ((uint32_t)EROB_MASTER_NODE_ID << 16)
                               | (uint32_t)EROB_MASTER_HB_TIMEOUT;
    (void)co_od_write(&canopen_node, 0x1016, 0x00, &hb_count,  1U);
    (void)co_od_write(&canopen_node, 0x1016, 0x01,
                      (const uint8_t *)&hb_master, sizeof(hb_master));
}

/* ── Main loop ───────────────────────────────────────────────────────────── */
void app_main_loop(void)
{
    for (;;) {
        /* Keep SDO / NMT / heartbeat protocol responsive on every iteration. */
        co_stm32_poll_rx(&canopen_node, &hfdcan1);
        co_process(&canopen_node);

        /* Control update is locked to the 1 ms SYNC edge. */
        if (!canopen_node.sync_event_pending) {
            continue;
        }
        canopen_node.sync_event_pending = false;

        /*
         * cia402_step() internally calls:
         *   1. erob_on_feedback   — snapshot position/velocity/torque from hw
         *   2. cia402_dispatch_mode_handler → erob_on_profile_velocity (ramp step)
         *   3. cia402_update_velocity_target_reached — set target_reached bit
         *   4. cia402_update_statusword + cia402_sync_fault_to_canopen
         *
         * Controlword transitions are handled automatically via RPDO hook
         * (cia402_on_rpdo_mapped_write) when RPDO1 arrives, so no explicit
         * cia402_apply_controlword() call is needed in the loop.
         */
        cia402_step(&axis);

        /* Propagate hardware inverter faults to the CANopen EMCY service. */
        fault_monitor_step();

        /* When not enabled, ensure the inverter output is zeroed. */
        if (axis.state != CIA402_OPERATION_ENABLED &&
            axis.state != CIA402_QUICK_STOP) {
            g_ramp.commanded_rpm = 0;
            /* TODO: de-assert motor inverter enable / STO line. */
        }
    }
}
