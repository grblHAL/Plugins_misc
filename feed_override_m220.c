/*

  feed_override_m220.c - set feed or rapid rate override

  Implements Marlin style M220 command

  Part of grblHAL misc. plugins

  M220 [B] [R] [S<percent>]

  B - backup current values

  R - restore values from backup

  S<percent> - percentage of current feedrate

  NOTE: M220RS<percentage> can be used to override the rapids rate, if R is not specified the feed rate will be overridden.

  https://marlinfw.org/docs/gcode/M220.html
*/

#include "driver.h"

#if FEED_OVERRIDE_ENABLE == 1

#include <math.h>

#include "grbl/hal.h"

static override_t feed_rate = 0, rapid_rate = 0;
static user_mcode_ptrs_t user_mcode;
static on_report_options_ptr on_report_options;

static user_mcode_type_t mcode_check (user_mcode_t mcode)
{
    return mcode == SetFeedOverrides
                     ? UserMCode_NoValueWords
                     : (user_mcode.check ? user_mcode.check(mcode) : UserMCode_Unsupported);
}

static status_code_t mcode_validate (parser_block_t *gc_block)
{
    status_code_t state = Status_OK;

    if((state = gc_block->user_mcode == SetFeedOverrides ? Status_OK : Status_Unhandled) == Status_OK) {

        if(gc_block->words.s) {
            if(!isintf(gc_block->values.s))
                state = Status_BadNumberFormat;
            else if(gc_block->values.s < (gc_block->words.r ? 5.0f : (float)MIN_FEED_RATE_OVERRIDE) ||
                     gc_block->values.s > (gc_block->words.r ? 100.0f : (float)MAX_FEED_RATE_OVERRIDE))
                state = Status_GcodeValueOutOfRange;
        }
        if(state == Status_OK && gc_block->words.b) {
            if(!isnan(gc_block->values.b))
                state = Status_BadNumberFormat;
        }
        if(state == Status_OK && gc_block->words.r) {
            if(!isnan(gc_block->values.r))
                state = Status_BadNumberFormat;
        }
        gc_block->words.b = gc_block->words.r = gc_block->words.s = Off;

    }

    return state == Status_Unhandled && user_mcode.validate ? user_mcode.validate(gc_block) : state;
}

static void mcode_execute (uint_fast16_t state, parser_block_t *gc_block)
{
    bool handled;

    if((handled = (gc_block->user_mcode == SetFeedOverrides))) {

        if(gc_block->words.b) {
            feed_rate = sys.override.feed_rate;
            rapid_rate = sys.override.rapid_rate;
        }

        if(gc_block->words.s) {
            if(gc_block->words.r)
                plan_feed_override(sys.override.feed_rate, (override_t)gc_block->values.s);
            else
                plan_feed_override((override_t)gc_block->values.s, sys.override.rapid_rate);
        } else if(gc_block->words.r && feed_rate != 0)
            plan_feed_override(feed_rate, rapid_rate);
    }

    if(!handled && user_mcode.execute)
        user_mcode.execute(state, gc_block);
}

static void onReportOptions (bool newopt)
{
    on_report_options(newopt);

    if(!newopt)
        report_plugin("Feed override", "0.01");
}

void feed_override_init (void)
{
    memcpy(&user_mcode, &grbl.user_mcode, sizeof(user_mcode_ptrs_t));

    grbl.user_mcode.check = mcode_check;
    grbl.user_mcode.validate = mcode_validate;
    grbl.user_mcode.execute = mcode_execute;

    on_report_options = grbl.on_report_options;
    grbl.on_report_options = onReportOptions;
}

#endif // FEED_OVERRIDE_ENABLE
