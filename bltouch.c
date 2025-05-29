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
#include "grbl/task.h"

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
static driver_reset_ptr driver_reset = NULL;
static on_report_options_ptr on_report_options;
static user_mcode_ptrs_t user_mcode;
static bool high_speed = false, auto_deploy = true;

static bool bltouch_cmd (BLTCommand_t cmd, uint16_t ms);

static void bltouch_stow (void *data)
{
    bltouch_cmd(BLTouch_Stow, BLTOUCH_STOW_DELAY);
}

static bool bltouch_cmd (BLTCommand_t cmd, uint16_t ms)
{
    static float current_angle = -1.0f;
    static bool selftest = false;

    // If the new command (angle) is the same, skip it (and the delay).
    // The previous write should've already delayed to detect the alarm.

#ifdef DEBUGOUT
    debug_print("Command bltouch: {%d}", cmd);
#endif

    if(selftest)
        task_delete(bltouch_stow, NULL);

    selftest = cmd == BLTouch_Selftest;

    if((float)cmd != (servo.get_value ? servo.get_value(&servo) : current_angle)) {

        ioport_analog_out(servo_port, current_angle = (float)cmd);
        if(ms)
            delay_sec(max((float)ms / 1e3f, (float)BLTOUCH_MIN_DELAY / 1e3f), DelayMode_SysSuspend);
    }

    return true;
}

static status_code_t bltouch_selftest (sys_state_t state, char *args)
{
    if(bltouch_cmd(BLTouch_Selftest, 0))
        task_add_delayed(bltouch_stow, NULL, BLTOUCH_SELFTEST_TIME);

    return Status_OK;
}

static status_code_t bltouch_reset (sys_state_t state, char *args)
{
    if(bltouch_cmd(BLTouch_Reset, BLTOUCH_RESET_DELAY))
        task_add_delayed(bltouch_stow, NULL, 10);

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
            if(gc_block->words.d) {
                if(!isintf(gc_block->values.s))
                    state = Status_BadNumberFormat;
                else if(gc_block->values.d < -0.0f || gc_block->values.d > 1.0f)
                    state = Status_GcodeValueOutOfRange;
            }
            if(state == Status_OK && gc_block->words.r) {
                if(!isintf(gc_block->values.r))
                    state = Status_BadNumberFormat;
                else if(gc_block->values.r < -0.0f || gc_block->values.r > 1.0f)
                    state = Status_GcodeValueOutOfRange;
            }
            gc_block->words.d = gc_block->words.h = gc_block->words.r = gc_block->words.s = Off;
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
             if(gc_block->words.d)
                 auto_deploy = gc_block->values.d != 0.0f;
             if(!(gc_block->words.s || gc_block->words.h || gc_block->words.d))
                 bltouch_cmd(BLTouch_Deploy, BLTOUCH_DEPLOY_DELAY);
             break;

         case Probe_Stow:
             bltouch_stow(NULL);
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

    if(ok && auto_deploy && !high_speed) {

        bltouch_cmd(BLTouch_Deploy, BLTOUCH_DEPLOY_DELAY);

        if(!pl_data->condition.probing_toolsetter && settings.probe.probe2_auto_select)
            hal.probe.select(Probe_2);
    }

    return ok;
}

static void onProbeCompleted (void)
{
    if(auto_deploy && !high_speed) {

        bltouch_stow(NULL);

        if(settings.probe.probe2_auto_select)
            hal.probe.select(Probe_Default);
    }

    if(on_probe_completed)
        on_probe_completed();
}

static void onDriverReset (void)
{
    driver_reset();

    task_add_immediate(bltouch_stow, NULL);
}

static void onReportOptions (bool newopt)
{
    on_report_options(newopt);

    if(!newopt)
        report_plugin(servo_port == 0xFF ? "BLTouch (N/A)" : "BLTouch", "0.06");
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

void bltouch_start (void *data)
{
    auto_deploy = !(hal.driver_cap.probe2 || hal.driver_cap.toolsetter);

    bltouch_stow(NULL);
}

void bltouch_init (void)
{
    static const sys_command_t bltouch_command_list[] = {
        { "BLRESET", bltouch_reset, {}, { .str = "perform BLTouch probe reset" } },
        { "BLTEST", bltouch_selftest, {}, { .str = "perform BLTouch probe self-test" } }
     };

    static sys_commands_t bltouch_commands = {
        .n_commands = sizeof(bltouch_command_list) / sizeof(sys_command_t),
        .commands = bltouch_command_list
    };

    on_report_options = grbl.on_report_options;
    grbl.on_report_options = onReportOptions;

    if((hal.driver_cap.bltouch_probe = ioports_enumerate(Port_Analog, Port_Output, (pin_cap_t){ .servo_pwm = On, .claimable = On }, claim_servo, NULL))) {

        memcpy(&user_mcode, &grbl.user_mcode, sizeof(user_mcode_ptrs_t));

        grbl.user_mcode.check = mcode_check;
        grbl.user_mcode.validate = mcode_validate;
        grbl.user_mcode.execute = mcode_execute;

        driver_reset = hal.driver_reset;
        hal.driver_reset = onDriverReset;

        on_probe_start = grbl.on_probe_start;
        grbl.on_probe_start = onProbeStart;

        on_probe_completed = grbl.on_probe_completed;
        grbl.on_probe_completed = onProbeCompleted;

        system_register_commands(&bltouch_commands);
        task_run_on_startup(bltouch_start, NULL);
    } else
        task_run_on_startup(report_warning, "No servo PWM output available for BLTouch!");
}

#endif // BLTOUCH_ENABLE
