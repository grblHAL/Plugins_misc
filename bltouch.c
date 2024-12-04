/*

  bltouch.c - auto deploy & stove of bltouch probe

  Implements Marlin style M401 and M402 commands

  Part of grblHAL misc. plugins

  Based on original code by @wakass. Public domain.
  https://github.com/wakass/grlbhal_servo

  https://marlinfw.org/docs/gcode/M401.html
  https://marlinfw.org/docs/gcode/M402.html
*/

#include "driver.h"

#if BLTOUCH_ENABLE == 1

#include <math.h>
#include <string.h>
#include <stdio.h>

#include "grbl/hal.h"
#include "grbl/nuts_bolts.h"
#include "grbl/protocol.h"

#define STOW_ALARM true

// Safety: The probe needs time to recognize the command.
//         Minimum command delay (ms). Increase if needed.

#ifndef BLTOUCH_MIN_DELAY
#define BLTOUCH_MIN_DELAY 500
#endif

// SIGNAL AND DELAY DEFINITIONS
// This is from Marlin firmware, seems reasonable.
// BLTouch commands are sent as servo angles

/**
 * The following commands require different minimum delays.
 *
 * 500ms required for a reliable Reset.
 *
 * 750ms required for Deploy/Stow, otherwise the alarm state
 *       will not be seen until the following move command.
 */

#ifndef BLTOUCH_SET5V_DELAY
#define BLTOUCH_SET5V_DELAY         150
#endif
#ifndef BLTOUCH_SETOD_DELAY
#define BLTOUCH_SETOD_DELAY         150
#endif
#ifndef BLTOUCH_MODE_STORE_DELAY
#define BLTOUCH_MODE_STORE_DELAY    150
#endif
#ifndef BLTOUCH_DEPLOY_DELAY
#define BLTOUCH_DEPLOY_DELAY        750
#endif
#ifndef BLTOUCH_STOW_DELAY
#define BLTOUCH_STOW_DELAY          750
#endif
#ifndef BLTOUCH_RESET_DELAY
#define BLTOUCH_RESET_DELAY         500
#endif
#ifndef BLTOUCH_SELFTEST_TIME
#define BLTOUCH_SELFTEST_TIME     12000
#endif

typedef enum {
    BLTouch_Deploy    = 10,
    BLTouch_Stow      = 90,
    BLTouch_SwMode    = 60,
    BLTouch_Selftest  = 120,
    BLTouch_ModeStore = 130,
    BLTouch_5vMode    = 140,
    BLTouch_OdMode    = 150,
    BLTouch_Reset     = 160
} BLTCommand_t;

static xbar_t servo = {0};
static uint8_t servo_port = 0xFF;
static on_probe_start_ptr on_probe_start;
static on_probe_completed_ptr on_probe_completed;
static on_report_options_ptr on_report_options;
static user_mcode_ptrs_t user_mcode;
static bool high_speed = false, selftest = false;

static bool bltouch_cmd (BLTCommand_t cmd, uint16_t ms);

static void selftest_done (void *data)
{
    bltouch_cmd(BLTouch_Stow, BLTOUCH_STOW_DELAY);
}

static bool bltouch_cmd (BLTCommand_t cmd, uint16_t ms)
{
    static float current_angle = -1.0f;

    // If the new command (angle) is the same, skip it (and the delay).
    // The previous write should've already delayed to detect the alarm.

#ifdef DEBUGOUT
    debug_print("Command bltouch: {%d}", cmd);
#endif

    if(selftest)
        task_delete(selftest_done, NULL);

    selftest = cmd == BLTouch_Selftest;

    if((float)cmd != (servo.get_value ? servo.get_value(&servo) : current_angle)) {

        hal.port.analog_out(servo_port, current_angle = (float)cmd);
        if(ms)
            delay_sec(max((float)ms / 1e3f, (float)BLTOUCH_MIN_DELAY / 1e3f), DelayMode_SysSuspend);
    }

    return true;
}

static status_code_t bltouch_selftest (sys_state_t state, char *args)
{
    if(bltouch_cmd(BLTouch_Selftest, 0))
        task_add_delayed(selftest_done, NULL, BLTOUCH_SELFTEST_TIME);

    return Status_OK;
}

static user_mcode_type_t mcode_check (user_mcode_t mcode)
{
    return mcode == Probe_Deploy || mcode == Probe_Stow
                     ? UserMCode_NoValueWords
                     : (user_mcode.check ? user_mcode.check(mcode) : UserMCode_Unsupported);
}

static status_code_t mcode_validate (parser_block_t *gc_block)
{
    status_code_t state = Status_OK;

    switch(gc_block->user_mcode) {

        case Probe_Deploy:
            if(gc_block->words.s) {
                if(!isintf(gc_block->values.s))
                    state = Status_BadNumberFormat;
                else if(gc_block->values.s < -0.0f || gc_block->values.s > 1.0f)
                    state = Status_GcodeValueOutOfRange;
            }
            if(state == Status_OK && gc_block->words.r) {
                if(!isintf(gc_block->values.r))
                    state = Status_BadNumberFormat;
                else if(gc_block->values.r < -0.0f || gc_block->values.r > 1.0f)
                    state = Status_GcodeValueOutOfRange;
            }
            gc_block->words.h = gc_block->words.r = gc_block->words.s = Off;
            break;

        case Probe_Stow:
            if(gc_block->words.r) {
                if(!isintf(gc_block->values.r))
                    state = Status_BadNumberFormat;
                else if(gc_block->values.r < -0.0f || gc_block->values.r > 1.0f)
                    state = Status_GcodeValueOutOfRange;
            }
            gc_block->words.r = Off;
            break;

        default:
            state = Status_Unhandled;
            break;
    }

    return state == Status_Unhandled && user_mcode.validate ? user_mcode.validate(gc_block) : state;
}

static void mcode_execute (uint_fast16_t state, parser_block_t *gc_block)
{
    bool handled = true;

    switch(gc_block->user_mcode) {

         case Probe_Deploy:
             if(gc_block->words.s)
                 high_speed = gc_block->values.s != 0.0f;
             if(gc_block->words.h) {
                 hal.stream.write("[PROBE HS:");
                 hal.stream.write(uitoa(high_speed));
                 hal.stream.write("]" ASCII_EOL);
             }
             if(!(gc_block->words.s || gc_block->words.h))
                 bltouch_cmd(BLTouch_Deploy, BLTOUCH_DEPLOY_DELAY);
             break;

         case Probe_Stow:
             bltouch_cmd(BLTouch_Stow, BLTOUCH_STOW_DELAY);
             break;

         default:
            handled = false;
            break;
    }

    if(!handled && user_mcode.execute)
        user_mcode.execute(state, gc_block);
}

static bool onProbeStart (axes_signals_t axes, float *target, plan_line_data_t *pl_data)
{
    bool ok = on_probe_start == NULL || on_probe_start(axes, target, pl_data);

    if(!high_speed && ok)
        bltouch_cmd(BLTouch_Deploy, BLTOUCH_DEPLOY_DELAY);

    return ok;
}

static void onProbeCompleted (void)
{
    if(!high_speed)
        bltouch_cmd(BLTouch_Stow, BLTOUCH_STOW_DELAY);

    if(on_probe_completed)
        on_probe_completed();
}

const sys_command_t bltouch_command_list[] = {
    {"BLTEST", bltouch_selftest, {}, { .str = "perform BLTouch probe self-test" } },
};

static sys_commands_t bltouch_commands = {
    .n_commands = sizeof(bltouch_command_list) / sizeof(sys_command_t),
    .commands = bltouch_command_list
};

static void onReportOptions (bool newopt)
{
    on_report_options(newopt);

    if(!newopt)
        report_plugin(servo_port == 0xFF ? "BLTouch (N/A)" : "BLTouch", "0.04");
}

static bool claim_servo (xbar_t *servo_pwm, uint8_t port, void *data)
{
    servo_port = port;

    if(ioport_claim(Port_Analog, Port_Output, &servo_port, "BLTouch probe")) {

        if(servo_pwm->get_value)
            memcpy(&servo, servo_pwm, sizeof(xbar_t));

        return true;
    } else
        servo_port = 0xFF;

    return false;
}

static void bltouch_stow (void *data)
{
    bltouch_cmd(BLTouch_Stow, BLTOUCH_STOW_DELAY);
}

void bltouch_init (void)
{
    on_report_options = grbl.on_report_options;
    grbl.on_report_options = onReportOptions;

    if(ioports_enumerate(Port_Analog, Port_Output, (pin_cap_t){ .servo_pwm = On, .claimable = On }, claim_servo, NULL)) {

        memcpy(&user_mcode, &grbl.user_mcode, sizeof(user_mcode_ptrs_t));

        grbl.user_mcode.check = mcode_check;
        grbl.user_mcode.validate = mcode_validate;
        grbl.user_mcode.execute = mcode_execute;

        on_probe_start = grbl.on_probe_start;
        grbl.on_probe_start = onProbeStart;

        on_probe_completed = grbl.on_probe_completed;
        grbl.on_probe_completed = onProbeCompleted;

        system_register_commands(&bltouch_commands);
        protocol_enqueue_foreground_task(bltouch_stow, NULL);
    } else
        protocol_enqueue_foreground_task(report_warning, "No servo PWM output available for BLTouch!");
}

#endif // BLTOUCH_ENABLE
