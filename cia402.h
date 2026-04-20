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

typedef struct {
    cia402_state_t state;
    uint16_t controlword;
    uint16_t statusword;
    int8_t mode_of_operation;
    int8_t mode_display;
    int32_t target_position;
    int32_t target_velocity;
    int16_t target_torque;
    int32_t position_actual;
    int32_t velocity_actual;
    int16_t torque_actual;
    uint16_t quick_stop_option_code;
    bool warning;
    bool remote;
    bool target_reached;
    bool internal_limit_active;
    bool fault_reaction_complete;
    co_node_t *node;
} cia402_axis_t;

void cia402_init(cia402_axis_t *axis);
void cia402_attach_node(cia402_axis_t *axis, co_node_t *node);
void cia402_set_feedback(cia402_axis_t *axis, int32_t pos, int32_t vel, int16_t tq);
void cia402_apply_controlword(cia402_axis_t *axis, uint16_t controlword);
void cia402_step(cia402_axis_t *axis);

#ifdef __cplusplus
}
#endif

#endif
