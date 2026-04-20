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

enum {
    CIA402_MSEF_UNSUPPORTED_MODE = 1U,
    CIA402_MSEF_MODE_HANDLER = 2U
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
                             axis->fault_msef,
                             NULL);
    } else {
        (void)co_fault_clear(axis->node, CO_FAULT_CIA402_PROFILE, 0U, NULL);
    }
}

static bool cia402_mode_is_supported(const cia402_axis_t *axis, int8_t mode)
{
    if (!axis || !axis->app || mode < 0 || mode >= 32) {
        return false;
    }

    return (axis->app->supported_modes & CIA402_MODE_BIT(mode)) != 0U;
}

static void cia402_enter_mode_fault(cia402_axis_t *axis, uint8_t msef)
{
    axis->warning = true;
    axis->target_reached = false;
    axis->internal_limit_active = true;
    axis->fault_msef = msef;
    axis->state = CIA402_FAULT_REACTION_ACTIVE;
    axis->fault_reaction_complete = true;
}

static bool cia402_dispatch_mode_handler(cia402_axis_t *axis)
{
    cia402_mode_command_cb_t handler = NULL;

    if (axis->state != CIA402_OPERATION_ENABLED) {
        return true;
    }

    if (!cia402_mode_is_supported(axis, axis->mode_of_operation)) {
        cia402_enter_mode_fault(axis, CIA402_MSEF_UNSUPPORTED_MODE);
        return false;
    }

    switch ((cia402_mode_t)axis->mode_of_operation) {
        case CIA402_MODE_PROFILE_POSITION:
            handler = axis->app ? axis->app->on_profile_position : NULL;
            if (!handler) {
                axis->target_reached = true;
            }
            break;
        case CIA402_MODE_VELOCITY:
            handler = axis->app ? axis->app->on_velocity : NULL;
            break;
        case CIA402_MODE_PROFILE_VELOCITY:
            handler = axis->app ? axis->app->on_profile_velocity : NULL;
            break;
        case CIA402_MODE_PROFILE_TORQUE:
            handler = axis->app ? axis->app->on_profile_torque : NULL;
            break;
        case CIA402_MODE_HOMING:
            handler = axis->app ? axis->app->on_homing : NULL;
            if (!handler) {
                axis->target_reached = true;
            }
            break;
        case CIA402_MODE_INTERPOLATED_POSITION:
            handler = axis->app ? axis->app->on_interpolated_position : NULL;
            break;
        case CIA402_MODE_CYCLIC_SYNC_POSITION:
            handler = axis->app ? axis->app->on_cyclic_sync_position : NULL;
            break;
        case CIA402_MODE_CYCLIC_SYNC_VELOCITY:
            handler = axis->app ? axis->app->on_cyclic_sync_velocity : NULL;
            break;
        case CIA402_MODE_CYCLIC_SYNC_TORQUE:
            handler = axis->app ? axis->app->on_cyclic_sync_torque : NULL;
            break;
        case CIA402_MODE_NONE:
        default:
            cia402_enter_mode_fault(axis, CIA402_MSEF_UNSUPPORTED_MODE);
            return false;
    }

    if (!handler) {
        return true;
    }

    if (!handler(axis, axis->app->user)) {
        cia402_enter_mode_fault(axis, CIA402_MSEF_MODE_HANDLER);
        return false;
    }

    return true;
}

static uint32_t cia402_on_write_controlword(co_node_t *node,
                                            const co_od_entry_t *entry,
                                            const uint8_t *data,
                                            size_t size,
                                            void *user)
{
    (void)node;
    (void)entry;
    if (!user || !data || size != sizeof(uint16_t)) {
        return CO_SDO_ABORT_PARAM_LENGTH;
    }

    cia402_axis_t *axis = (cia402_axis_t *)user;
    uint16_t controlword = 0U;
    memcpy(&controlword, data, sizeof(controlword));
    cia402_apply_controlword(axis, controlword);
    return 0U;
}

static uint32_t cia402_on_write_mode_of_operation(co_node_t *node,
                                                  const co_od_entry_t *entry,
                                                  const uint8_t *data,
                                                  size_t size,
                                                  void *user)
{
    (void)node;
    (void)entry;
    if (!user || !data || size != sizeof(int8_t)) {
        return CO_SDO_ABORT_PARAM_LENGTH;
    }

    cia402_axis_t *axis = (cia402_axis_t *)user;
    axis->mode_of_operation = (int8_t)data[0];
    return 0U;
}

static bool cia402_tpdo_maps_feedback(const co_node_t *node, uint8_t tpdo_num)
{
    const co_pdo_cfg_t *cfg = &node->tpdo_cfg[tpdo_num];
    for (uint8_t i = 0; i < cfg->map_count; ++i) {
        const uint16_t index = cfg->mapping[i].index;
        if (index == 0x6041U || index == 0x6061U || index == 0x6064U || index == 0x606CU ||
            index == 0x6077U) {
            return true;
        }
    }
    return false;
}

static void cia402_on_rpdo_mapped_write(co_node_t *node,
                                        uint8_t rpdo_num,
                                        uint16_t index,
                                        uint8_t subindex,
                                        void *user)
{
    (void)node;
    (void)rpdo_num;
    if (!user || subindex != 0U) {
        return;
    }

    cia402_axis_t *axis = (cia402_axis_t *)user;
    if (index == 0x6040U) {
        cia402_apply_controlword(axis, axis->controlword);
    }
}

static void cia402_on_tpdo_pre_tx(co_node_t *node, uint8_t tpdo_num, void *user)
{
    if (!node || !user || tpdo_num >= CO_MAX_TPDO) {
        return;
    }

    if (!cia402_tpdo_maps_feedback(node, tpdo_num)) {
        return;
    }

    cia402_axis_t *axis = (cia402_axis_t *)user;
    cia402_step(axis);
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

void cia402_bind_od(cia402_axis_t *axis, co_node_t *node)
{
    if (!axis || !node) {
        return;
    }

    cia402_attach_node(axis, node);
    (void)co_od_add_ex(node,
                       0x6040U,
                       0x00U,
                       (uint8_t *)&axis->controlword,
                       sizeof(axis->controlword),
                       (uint8_t)(CO_OD_ACCESS_READ | CO_OD_ACCESS_WRITE),
                       NULL,
                       cia402_on_write_controlword,
                       axis);
    (void)co_od_add_ex(node,
                       0x6041U,
                       0x00U,
                       (uint8_t *)&axis->statusword,
                       sizeof(axis->statusword),
                       CO_OD_ACCESS_READ,
                       NULL,
                       NULL,
                       axis);
    (void)co_od_add_ex(node,
                       0x6060U,
                       0x00U,
                       (uint8_t *)&axis->mode_of_operation,
                       sizeof(axis->mode_of_operation),
                       (uint8_t)(CO_OD_ACCESS_READ | CO_OD_ACCESS_WRITE),
                       NULL,
                       cia402_on_write_mode_of_operation,
                       axis);
    (void)co_od_add_ex(node,
                       0x6061U,
                       0x00U,
                       (uint8_t *)&axis->mode_display,
                       sizeof(axis->mode_display),
                       CO_OD_ACCESS_READ,
                       NULL,
                       NULL,
                       axis);
    (void)co_od_add_ex(node,
                       0x6064U,
                       0x00U,
                       (uint8_t *)&axis->position_actual,
                       sizeof(axis->position_actual),
                       CO_OD_ACCESS_READ,
                       NULL,
                       NULL,
                       axis);
    (void)co_od_add_ex(node,
                       0x606CU,
                       0x00U,
                       (uint8_t *)&axis->velocity_actual,
                       sizeof(axis->velocity_actual),
                       CO_OD_ACCESS_READ,
                       NULL,
                       NULL,
                       axis);
    (void)co_od_add_ex(node,
                       0x6077U,
                       0x00U,
                       (uint8_t *)&axis->torque_actual,
                       sizeof(axis->torque_actual),
                       CO_OD_ACCESS_READ,
                       NULL,
                       NULL,
                       axis);

    co_set_hooks(node, cia402_on_rpdo_mapped_write, cia402_on_tpdo_pre_tx, axis);
}

void cia402_set_callbacks(cia402_axis_t *axis, const cia402_app_if_t *app)
{
    if (!axis) {
        return;
    }

    axis->app = app;
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
    if ((command == CIA402_CMD_FAULT_RESET) && (axis->state == CIA402_SWITCH_ON_DISABLED)) {
        axis->fault_msef = 0U;
        axis->warning = false;
        axis->internal_limit_active = false;
    }

    cia402_update_statusword(axis);
    cia402_sync_fault_to_canopen(axis);
}

void cia402_step(cia402_axis_t *axis)
{
    if (axis->app && axis->app->on_feedback) {
        axis->app->on_feedback(axis, axis->mode_of_operation, axis->app->user);
    }

    (void)cia402_dispatch_mode_handler(axis);

    if (axis->state == CIA402_FAULT_REACTION_ACTIVE) {
        if (axis->fault_reaction_complete) {
            axis->state = CIA402_FAULT;
        }
        cia402_update_statusword(axis);
        cia402_sync_fault_to_canopen(axis);
        return;
    }

    cia402_update_statusword(axis);
    cia402_sync_fault_to_canopen(axis);
}
