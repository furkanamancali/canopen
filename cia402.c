#include "cia402.h"

#include "canopen.h"

#include <stdbool.h>
#include <string.h>

typedef enum {
    CIA402_CMD_NONE = 0,
    CIA402_CMD_DISABLE_VOLTAGE,
    CIA402_CMD_SHUTDOWN,
    CIA402_CMD_SWITCH_ON,
    CIA402_CMD_DISABLE_OPERATION,
    CIA402_CMD_ENABLE_OPERATION,
    CIA402_CMD_QUICK_STOP,
    CIA402_CMD_FAULT_RESET
} cia402_command_t;

typedef struct {
    cia402_state_t from;
    cia402_command_t command;
    cia402_state_t to;
} cia402_transition_t;

static const cia402_transition_t g_cia402_transitions[] = {
    { CIA402_SWITCH_ON_DISABLED, CIA402_CMD_SHUTDOWN, CIA402_READY_TO_SWITCH_ON },
    { CIA402_READY_TO_SWITCH_ON, CIA402_CMD_DISABLE_VOLTAGE, CIA402_SWITCH_ON_DISABLED },
    { CIA402_READY_TO_SWITCH_ON, CIA402_CMD_SWITCH_ON, CIA402_SWITCHED_ON },
    { CIA402_READY_TO_SWITCH_ON, CIA402_CMD_QUICK_STOP, CIA402_SWITCH_ON_DISABLED },
    { CIA402_SWITCHED_ON, CIA402_CMD_SHUTDOWN, CIA402_READY_TO_SWITCH_ON },
    { CIA402_SWITCHED_ON, CIA402_CMD_DISABLE_VOLTAGE, CIA402_SWITCH_ON_DISABLED },
    { CIA402_SWITCHED_ON, CIA402_CMD_ENABLE_OPERATION, CIA402_OPERATION_ENABLED },
    { CIA402_SWITCHED_ON, CIA402_CMD_QUICK_STOP, CIA402_SWITCH_ON_DISABLED },
    { CIA402_OPERATION_ENABLED, CIA402_CMD_DISABLE_OPERATION, CIA402_SWITCHED_ON },
    { CIA402_OPERATION_ENABLED, CIA402_CMD_SHUTDOWN, CIA402_READY_TO_SWITCH_ON },
    { CIA402_OPERATION_ENABLED, CIA402_CMD_DISABLE_VOLTAGE, CIA402_SWITCH_ON_DISABLED },
    { CIA402_OPERATION_ENABLED, CIA402_CMD_QUICK_STOP, CIA402_QUICK_STOP },
    { CIA402_QUICK_STOP, CIA402_CMD_DISABLE_VOLTAGE, CIA402_SWITCH_ON_DISABLED },
    { CIA402_QUICK_STOP, CIA402_CMD_ENABLE_OPERATION, CIA402_OPERATION_ENABLED },
    { CIA402_FAULT, CIA402_CMD_FAULT_RESET, CIA402_SWITCH_ON_DISABLED },
};

static cia402_command_t cia402_decode_command(uint16_t controlword)
{
    if ((controlword & 0x0080U) != 0U) {
        return CIA402_CMD_FAULT_RESET;
    }

    if ((controlword & 0x008FU) == 0x0000U) {
        return CIA402_CMD_DISABLE_VOLTAGE;
    }

    if ((controlword & 0x008FU) == 0x0006U) {
        return CIA402_CMD_SHUTDOWN;
    }

    if ((controlword & 0x008FU) == 0x0007U) {
        return CIA402_CMD_SWITCH_ON;
    }

    if ((controlword & 0x008FU) == 0x000FU) {
        return CIA402_CMD_ENABLE_OPERATION;
    }

    if ((controlword & 0x008FU) == 0x0002U) {
        return CIA402_CMD_QUICK_STOP;
    }

    return CIA402_CMD_NONE;
}

static bool cia402_quick_stop_complete_to_switch_on_disabled(const cia402_axis_t *axis)
{
    switch (axis->quick_stop_option_code) {
        case 0U:
        case 1U:
        case 2U:
            return true;
        case 5U:
        case 6U:
        default:
            return false;
    }
}

static void cia402_apply_quick_stop_rules(cia402_axis_t *axis, cia402_command_t command)
{
    if (axis->state != CIA402_QUICK_STOP) {
        return;
    }

    if (cia402_quick_stop_complete_to_switch_on_disabled(axis)) {
        axis->state = CIA402_SWITCH_ON_DISABLED;
        return;
    }

    if (command == CIA402_CMD_ENABLE_OPERATION) {
        axis->state = CIA402_OPERATION_ENABLED;
    }
}

static void cia402_update_statusword(cia402_axis_t *axis)
{
    const bool ready_to_switch_on =
        (axis->state == CIA402_READY_TO_SWITCH_ON) ||
        (axis->state == CIA402_SWITCHED_ON) ||
        (axis->state == CIA402_OPERATION_ENABLED) ||
        (axis->state == CIA402_QUICK_STOP);

    const bool switched_on =
        (axis->state == CIA402_SWITCHED_ON) ||
        (axis->state == CIA402_OPERATION_ENABLED) ||
        (axis->state == CIA402_QUICK_STOP);

    const bool operation_enabled = (axis->state == CIA402_OPERATION_ENABLED);
    const bool fault = (axis->state == CIA402_FAULT) || (axis->state == CIA402_FAULT_REACTION_ACTIVE);
    const bool voltage_enabled =
        (axis->state == CIA402_READY_TO_SWITCH_ON) ||
        (axis->state == CIA402_SWITCHED_ON) ||
        (axis->state == CIA402_OPERATION_ENABLED) ||
        (axis->state == CIA402_QUICK_STOP) ||
        (axis->state == CIA402_FAULT_REACTION_ACTIVE);

    const bool quick_stop =
        (axis->state == CIA402_OPERATION_ENABLED) ||
        (axis->state == CIA402_SWITCHED_ON) ||
        (axis->state == CIA402_READY_TO_SWITCH_ON);

    const bool switch_on_disabled = (axis->state == CIA402_SWITCH_ON_DISABLED);

    uint16_t sw = 0;
    if (ready_to_switch_on) {
        sw |= (1U << 0);
    }
    if (switched_on) {
        sw |= (1U << 1);
    }
    if (operation_enabled) {
        sw |= (1U << 2);
    }
    if (fault) {
        sw |= (1U << 3);
    }
    if (voltage_enabled) {
        sw |= (1U << 4);
    }
    if (quick_stop) {
        sw |= (1U << 5);
    }
    if (switch_on_disabled) {
        sw |= (1U << 6);
    }

    if (axis->warning) {
        sw |= (1U << 7);
    }
    if (axis->remote) {
        sw |= (1U << 9);
    }
    if (axis->target_reached) {
        sw |= (1U << 10);
    }
    if (axis->internal_limit_active) {
        sw |= (1U << 11);
    }

    axis->statusword = sw;
    axis->mode_display = axis->mode_of_operation;
}

static void cia402_sync_fault_to_canopen(cia402_axis_t *axis)
{
    if (!axis || !axis->node) {
        return;
    }

    if (axis->state == CIA402_FAULT || axis->state == CIA402_FAULT_REACTION_ACTIVE) {
        (void)co_fault_raise(axis->node,
                             CO_FAULT_CIA402_PROFILE,
                             CO_EMCY_ERR_PROFILE,
                             CO_ERROR_REG_DEVICE_PROFILE,
                             0U,
                             NULL);
    } else {
        (void)co_fault_clear(axis->node, CO_FAULT_CIA402_PROFILE, 0U, NULL);
    }
}

void cia402_init(cia402_axis_t *axis)
{
    memset(axis, 0, sizeof(*axis));
    axis->state = CIA402_SWITCH_ON_DISABLED;
    axis->quick_stop_option_code = 2U;
    axis->remote = true;
    cia402_update_statusword(axis);
}

void cia402_attach_node(cia402_axis_t *axis, co_node_t *node)
{
    if (!axis) {
        return;
    }
    axis->node = node;
    cia402_sync_fault_to_canopen(axis);
}

void cia402_set_feedback(cia402_axis_t *axis, int32_t pos, int32_t vel, int16_t tq)
{
    axis->position_actual = pos;
    axis->velocity_actual = vel;
    axis->torque_actual = tq;
}

void cia402_apply_controlword(cia402_axis_t *axis, uint16_t controlword)
{
    axis->controlword = controlword;

    const cia402_command_t command = cia402_decode_command(controlword);

    for (size_t i = 0; i < (sizeof(g_cia402_transitions) / sizeof(g_cia402_transitions[0])); ++i) {
        if ((g_cia402_transitions[i].from == axis->state) &&
            (g_cia402_transitions[i].command == command)) {
            axis->state = g_cia402_transitions[i].to;
            break;
        }
    }

    cia402_apply_quick_stop_rules(axis, command);

    cia402_update_statusword(axis);
    cia402_sync_fault_to_canopen(axis);
}

void cia402_step(cia402_axis_t *axis)
{
    if (axis->state == CIA402_FAULT_REACTION_ACTIVE) {
        if (axis->fault_reaction_complete) {
            axis->state = CIA402_FAULT;
        }
        cia402_update_statusword(axis);
        cia402_sync_fault_to_canopen(axis);
    }
}
