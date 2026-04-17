#include "cia402.h"

#include <string.h>

static void cia402_update_statusword(cia402_axis_t *axis)
{
    uint16_t sw = 0;

    switch (axis->state) {
        case CIA402_NOT_READY:
            sw = 0x0000;
            break;
        case CIA402_SWITCH_ON_DISABLED:
            sw = 0x0040;
            break;
        case CIA402_READY_TO_SWITCH_ON:
            sw = 0x0021;
            break;
        case CIA402_SWITCHED_ON:
            sw = 0x0023;
            break;
        case CIA402_OPERATION_ENABLED:
            sw = 0x0027;
            break;
        case CIA402_QUICK_STOP:
            sw = 0x0007;
            break;
        case CIA402_FAULT_REACTION_ACTIVE:
            sw = 0x000F;
            break;
        case CIA402_FAULT:
            sw = 0x0008;
            break;
    }

    axis->statusword = sw;
    axis->mode_display = axis->mode_of_operation;
}

void cia402_init(cia402_axis_t *axis)
{
    memset(axis, 0, sizeof(*axis));
    axis->state = CIA402_SWITCH_ON_DISABLED;
    cia402_update_statusword(axis);
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

    if (axis->state == CIA402_FAULT && (controlword & 0x0080U)) {
        axis->state = CIA402_SWITCH_ON_DISABLED;
        cia402_update_statusword(axis);
        return;
    }

    switch (axis->state) {
        case CIA402_SWITCH_ON_DISABLED:
            if ((controlword & 0x008FU) == 0x0006U) {
                axis->state = CIA402_READY_TO_SWITCH_ON;
            }
            break;
        case CIA402_READY_TO_SWITCH_ON:
            if ((controlword & 0x008FU) == 0x0007U) {
                axis->state = CIA402_SWITCHED_ON;
            } else if ((controlword & 0x008FU) == 0x0000U) {
                axis->state = CIA402_SWITCH_ON_DISABLED;
            }
            break;
        case CIA402_SWITCHED_ON:
            if ((controlword & 0x008FU) == 0x000FU) {
                axis->state = CIA402_OPERATION_ENABLED;
            } else if ((controlword & 0x008FU) == 0x0006U) {
                axis->state = CIA402_READY_TO_SWITCH_ON;
            }
            break;
        case CIA402_OPERATION_ENABLED:
            if ((controlword & 0x0002U) == 0U) {
                axis->state = CIA402_QUICK_STOP;
            } else if ((controlword & 0x008FU) == 0x0007U) {
                axis->state = CIA402_SWITCHED_ON;
            }
            break;
        case CIA402_QUICK_STOP:
            if ((controlword & 0x0002U) != 0U) {
                axis->state = CIA402_OPERATION_ENABLED;
            }
            break;
        case CIA402_NOT_READY:
        case CIA402_FAULT_REACTION_ACTIVE:
        case CIA402_FAULT:
            break;
    }

    cia402_update_statusword(axis);
}

void cia402_step(cia402_axis_t *axis)
{
    if (axis->state == CIA402_FAULT_REACTION_ACTIVE) {
        axis->state = CIA402_FAULT;
        cia402_update_statusword(axis);
    }
}
