#ifndef CIA402_H
#define CIA402_H

#include <stdbool.h>
#include <stdint.h>

struct co_node;
typedef struct co_node co_node_t;

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    CIA402_NOT_READY = 0,
    CIA402_SWITCH_ON_DISABLED,
    CIA402_READY_TO_SWITCH_ON,
    CIA402_SWITCHED_ON,
    CIA402_OPERATION_ENABLED,
    CIA402_QUICK_STOP,
    CIA402_FAULT_REACTION_ACTIVE,
    CIA402_FAULT
} cia402_state_t;

typedef enum {
    CIA402_MODE_NONE = 0,
    CIA402_MODE_PROFILE_POSITION = 1,
    CIA402_MODE_VELOCITY = 2,
    CIA402_MODE_PROFILE_VELOCITY = 3,
    CIA402_MODE_PROFILE_TORQUE = 4,
    CIA402_MODE_HOMING = 6,
    CIA402_MODE_INTERPOLATED_POSITION = 7,
    CIA402_MODE_CYCLIC_SYNC_POSITION = 8,
    CIA402_MODE_CYCLIC_SYNC_VELOCITY = 9,
    CIA402_MODE_CYCLIC_SYNC_TORQUE = 10
} cia402_mode_t;

struct cia402_axis;
typedef struct cia402_axis cia402_axis_t;

typedef void (*cia402_feedback_cb_t)(cia402_axis_t *axis, int8_t mode, void *user);
typedef bool (*cia402_mode_command_cb_t)(cia402_axis_t *axis, void *user);

typedef struct {
    void *user;
    uint32_t supported_modes;
    cia402_feedback_cb_t on_feedback;
    cia402_mode_command_cb_t on_profile_position;
    cia402_mode_command_cb_t on_velocity;
    cia402_mode_command_cb_t on_profile_velocity;
    cia402_mode_command_cb_t on_profile_torque;
    cia402_mode_command_cb_t on_homing;
    cia402_mode_command_cb_t on_interpolated_position;
    cia402_mode_command_cb_t on_cyclic_sync_position;
    cia402_mode_command_cb_t on_cyclic_sync_velocity;
    cia402_mode_command_cb_t on_cyclic_sync_torque;
} cia402_app_if_t;

#define CIA402_MODE_BIT(mode) (1UL << (uint32_t)(mode))

struct cia402_axis {
    cia402_state_t state;
    uint16_t controlword;
    uint16_t statusword;
    int8_t mode_of_operation;
    int8_t mode_display;
    int32_t target_position;
    int32_t target_velocity;
    int16_t target_torque;
    uint32_t profile_velocity;
    uint32_t profile_acceleration;
    uint32_t profile_deceleration;
    uint32_t quick_stop_deceleration;
    int32_t position_actual;
    int32_t velocity_actual;
    int16_t torque_actual;
    uint16_t quick_stop_option_code;
    bool warning;
    bool remote;
    bool target_reached;
    bool internal_limit_active;
    bool fault_reaction_complete;
    bool target_position_valid;
    bool target_velocity_valid;
    bool target_torque_valid;
    bool profile_velocity_valid;
    bool profile_acceleration_valid;
    bool profile_deceleration_valid;
    bool quick_stop_deceleration_valid;
    co_node_t *node;
    uint8_t fault_msef;
    uint16_t fault_code;
    int8_t diag_last_mode_checked;
    uint32_t diag_precondition_mask;
    const cia402_app_if_t *app;
};

void cia402_init(cia402_axis_t *axis);
void cia402_attach_node(cia402_axis_t *axis, co_node_t *node);
void cia402_bind_od(cia402_axis_t *axis, co_node_t *node);
void cia402_set_callbacks(cia402_axis_t *axis, const cia402_app_if_t *app);
void cia402_set_feedback(cia402_axis_t *axis, int32_t pos, int32_t vel, int16_t tq);
void cia402_apply_controlword(cia402_axis_t *axis, uint16_t controlword);
void cia402_step(cia402_axis_t *axis);

#ifdef __cplusplus
}
#endif

#endif
