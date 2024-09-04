/*

  rgb_led_m150.c - plugin for M150, Marlin style LED strip control

  Part of grblHAL misc. plugins

  Public domain.

  Usage: M150 [B<intensity>] [I<pixel>] [K] [P<intensity>] [R<intensity>] [S0] [U<intensity>] [W<intensity>]

    B<intensity> - blue component, 0 - 255
    I<pixel>     - NeoPixel index, available if number of pixels > 1
    K            - keep unspecified values
    P<intensity> - brightness, 0 - 255
    S0           - write values to all LEDs in strip
    R<intensity> - red component, 0 - 255
    U<intensity> - red component, 0 - 255

  https://marlinfw.org/docs/gcode/M150.html

  $536 - length of strip 1.
  $537 - length of strip 2.

*/

#include "driver.h"

#if RGB_LED_ENABLE == 2

#if N_AXIS > 4 || AXIS_REMAP_ABC2UVW || LATHE_UVW_OPTION
#error "RGB LED plugin is not supported in this configuration!"
#endif

#include <math.h>
#include <string.h>

#include "rgb_led_strips.c"

static bool is_neopixels;
static user_mcode_ptrs_t user_mcode;
static on_report_options_ptr on_report_options;

static user_mcode_t mcode_check (user_mcode_t mcode)
{
    return mcode == RGB_WriteLEDs
                     ? mcode
                     : (user_mcode.check ? user_mcode.check(mcode) : UserMCode_Ignore);
}

static status_code_t parameter_validate (float *value)
{
    status_code_t state = Status_OK;

    if(isnanf(*value) || !isintf(*value))
        state = Status_BadNumberFormat;

    if(*value < -0.0f || *value > 255.0f)
        state = Status_GcodeValueOutOfRange;

    return state;
}

static status_code_t mcode_validate (parser_block_t *gc_block, parameter_words_t *deprecated)
{
    status_code_t state = Status_OK;

    switch(gc_block->user_mcode) {

        case RGB_WriteLEDs:

            if(gc_block->words.b && (state = parameter_validate(&gc_block->values.b)) != Status_OK)
                return state;

            if(gc_block->words.r && (state = parameter_validate(&gc_block->values.r)) != Status_OK)
                return state;

            if(gc_block->words.u && (state = parameter_validate(&gc_block->values.u)) != Status_OK)
                return state;

            if(gc_block->words.w && (state = parameter_validate(&gc_block->values.w)) != Status_OK)
                return state;

            if(gc_block->words.p && is_neopixels && (state = parameter_validate(&gc_block->values.p)) != Status_OK)
                return state;

            if(!(gc_block->words.r || gc_block->words.u || gc_block->words.b || gc_block->words.w || gc_block->words.p))
                return Status_GcodeValueWordMissing;

            if(gc_block->words.s && !(gc_block->values.s == 0.0f || (gc_block->values.s == 1.0f && !!hal.rgb1.out)))
                return Status_GcodeValueOutOfRange;

            rgb_ptr_t *strip = gc_block->words.s && gc_block->values.s == 1.0f ? &hal.rgb1 : &hal.rgb0;

            if(gc_block->words.i && strip->num_devices > 1) {
                if(gc_block->values.ijk[0] < -0.0f || gc_block->values.ijk[0] > (float)(strip->num_devices - 1))
                    state = Status_GcodeValueOutOfRange;
                else
                    gc_block->words.i = Off;
            }

            if(gc_block->words.p && is_neopixels)
                gc_block->words.p = Off;

            gc_block->words.k = gc_block->words.b = gc_block->words.r = gc_block->words.u = gc_block->words.w = gc_block->words.s = Off;
            break;

        default:
            state = Status_Unhandled;
            break;
    }

    return state == Status_Unhandled && user_mcode.validate ? user_mcode.validate(gc_block, deprecated) : state;
}

static void mcode_execute (uint_fast16_t state, parser_block_t *gc_block)
{
    static rgb_color_t color = {0}; // TODO: allocate for all leds?

    bool handled = true;

    if(state != STATE_CHECK_MODE) {

        rgb_ptr_t *strip = gc_block->words.s && gc_block->values.s == 1.0f ? &hal.rgb1 : &hal.rgb0;

        switch(gc_block->user_mcode) {

             case RGB_WriteLEDs:;

                 bool set_colors;
                 uint16_t device = gc_block->words.i ? (uint16_t)gc_block->values.ijk[0] : 0;
                 rgb_color_mask_t mask = { .value = 0xFF };
                 rgb_color_t new_color;

                 if((set_colors = gc_block->words.r || gc_block->words.u || gc_block->words.b || gc_block->words.w)) {
                     if(!gc_block->words.k)
                         color.value = 0;
                     else {
                         mask.R = gc_block->words.r;
                         mask.G = gc_block->words.u;
                         mask.B = gc_block->words.b;
                         mask.W = gc_block->words.w;
                     }
                 }

                 if(gc_block->words.w) {
                     if(strip->cap.W)
                         color.W = (uint8_t)(gc_block->words.p ? gc_block->values.p : gc_block->values.w);
                     else
                         color.R = color.G = color.B = gc_block->values.w;
                 }

                 if(!gc_block->words.w || strip->cap.W) {
                     if(gc_block->words.r)
                         color.R = (uint8_t)gc_block->values.r;
                     if(gc_block->words.u)
                         color.G = (uint8_t)gc_block->values.u;
                     if(gc_block->words.b)
                         color.B = (uint8_t)gc_block->values.b;
                 }

                 new_color.value = color.value;

                 if(gc_block->words.p) {

                     if(strip->set_intensity)
                         strip->set_intensity((uint8_t)gc_block->values.p);
                     else
                         new_color = rgb_set_intensity(color, (uint8_t)gc_block->values.p);
                 }

                 if(set_colors || (gc_block->words.p && strip->set_intensity == NULL)) {
                     if(!gc_block->words.i && strip->num_devices > 1) {
                         for(device = 0; device < strip->num_devices; device++) {
                             if(strip->out_masked)
                                 strip->out_masked(device, new_color, mask);
                             else
                                 strip->out(device, new_color);
                         }
                     } else {
                         if(strip->out_masked)
                             strip->out_masked(device, new_color, mask);
                         else
                             strip->out(device, new_color);
                     }
                 }

                 if(set_colors && strip->num_devices > 1 && strip->write)
                     strip->write();

                 break;

            default:
                handled = false;
                break;
        }
    }

    if(!handled && user_mcode.execute)
        user_mcode.execute(state, gc_block);
}

static void onReportOptions (bool newopt)
{
    on_report_options(newopt);

    if(!newopt)
        report_plugin(hal.rgb0.out
                       ? "RGB LED strips (M150)"
                       : "RGB LED strips (N/A)", "0.03");
}

void rgb_led_init (void)
{
    if(hal.rgb0.out) {

        memcpy(&user_mcode, &hal.user_mcode, sizeof(user_mcode_ptrs_t));

        hal.user_mcode.check = mcode_check;
        hal.user_mcode.validate = mcode_validate;
        hal.user_mcode.execute = mcode_execute;

        if((is_neopixels = rgb_is_neopixels(&hal.rgb0)))
            rgb_led_settings_register();
    }

    on_report_options = grbl.on_report_options;
    grbl.on_report_options = onReportOptions;
}

#endif // RGB_LED_ENABLE
