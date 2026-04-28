#ifndef __CIA402_APP_H__
#define __CIA402_APP_H__

#include "canopen.h"
#include "app_config.h"

/* ── Driver type ─────────────────────────────────────────────────────────── */
/* Selects which init recipe app_canopen_init() applies to a given node:
 *   DRIVER_ZEROERR — CiA 402 Profile Velocity (ZeroErr eRob actuator).
 *                    Full PDO mapping + profile-ramp SDO + heartbeat.
 *   DRIVER_DELTA   — CiA 402 Profile Velocity (Delta ASDA servo).
 *                    Same PDO layout; counts_per_rpm_s calculated differently.
 *   DRIVER_WAT     — non-CiA-402 drive sharing the same bus.  Master writes
 *                    only the bare-minimum heartbeat config; PDO mapping and
 *                    the CiA 402 state machine are skipped. */
typedef enum {
    DRIVER_ZEROERR,
    DRIVER_DELTA,
	DRIVER_WAT
} driver_type_t;

/* ── Per-node configuration ──────────────────────────────────────────────── */
typedef struct {
    uint8_t       node_id;             /* CANopen node id (1..127) */
    uint32_t      default_accel;       /* [counts/s²] written once to 0x6083 at startup;
                                          updated at runtime via cia402_set_accel(). */
    uint32_t      default_decel;       /* [counts/s²] written once to 0x6084 at startup;
                                          updated at runtime via cia402_set_accel(). */
    uint32_t      encoder_counts_per_rev; /* encoder counts per motor revolution.
                                             Supply the raw CPR value, e.g.:
                                               524288  — ZeroErr 19-bit absolute
                                               1280000 — Delta ASDA-A3 20-bit
                                               10000   — 2500-PPR incremental ×4
                                             Used for RPM↔counts conversion in
                                             cia402_set_target_rpm() and
                                             cia402_get_pos_vel(). */
    uint16_t      reductor_ratio;      /* gearbox ratio: motor revolutions per
                                          one output revolution.  Use 1 for a
                                          direct-drive actuator.  Folded into the
                                          cached counts_per_rpm_s so target/RPM
                                          values supplied to the public API are
                                          interpreted on the OUTPUT shaft, while
                                          the encoder counts on the motor side. */
    driver_type_t driver_type;         /* selects the init recipe (see above) */
} cia402_cfg_t;

/* Compile-time upper bound on the number of slave instances.  Storage in
 * example_embedded.c (m_node[], m_sdo_tbl[], g_*[], etc.) is sized to this
 * cap; cia402_node_count <= CIA402_MAX_NODES is checked at startup.  Bump
 * this and CO_MAX_RPDO / CO_MAX_TPDO (canopen.h) together if more drives
 * are needed. */
#ifndef CIA402_MAX_NODES
#warning "CIA402_MAX_NODES is not defined"
#endif

/* Configuration table — define once in your application, e.g.:
 *
 *   const cia402_cfg_t cia402_nodes[] = {
 *       { 0x05U, 50000U, 200000U, 524288U, 1U, DRIVER_ZEROERR },
 *   };
 *   const uint8_t cia402_node_count =
 *       (uint8_t)(sizeof(cia402_nodes) / sizeof(cia402_nodes[0]));
 *
 * example_embedded.c only references these by extern, so the application
 * can supply the table from any translation unit without modifying the
 * stack source. */
extern const cia402_cfg_t cia402_nodes[];
extern const uint8_t      cia402_node_count;

/* ── Public API ──────────────────────────────────────────────────────────── */
co_error_t app_canopen_init(void);
void       app_canopen_loop(void);
void       co_transmit_process(void);

void cia402_set_target_rpm(uint16_t node_id, float target_rpm);
void cia402_set_accel(uint16_t node_id, uint32_t accel_counts_s2, uint32_t decel_counts_s2);
void cia402_get_pos_vel(uint16_t node_id, float *pos_deg, float *vel_rpm, float *torque_pct);

#endif /* __CIA402_APP_H__ */
