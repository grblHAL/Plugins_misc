/*

  rgb_led_strips.c - plugin for configuring number of LEDs in up to two strips

  Part of grblHAL misc. plugins

  Public domain.

  $536 - length of strip 1.
  $537 - length of strip 2.

*/

#include "driver.h"

#if RGB_LED_ENABLE

static bool is_setting_available (const setting_detail_t *setting)
{
    bool available = false;

    switch(setting->id) {

        case Setting_RGB_StripLengt0:
            available = hal.rgb0.flags.is_strip;
            break;

        case Setting_RGB_StripLengt1:
            available = hal.rgb1.flags.is_strip;
            break;

        default:
            break;
    }

    return available;
}

static const setting_group_detail_t rgb_groups[] = {
    { Group_Root, Group_AuxPorts, "Aux ports"}
};

static const setting_detail_t rgb_settings[] = {
    { Setting_RGB_StripLengt0, Group_AuxPorts, "LED strip 1 length", NULL, Format_Int8, "##0", NULL, "255", Setting_NonCore, &settings.rgb_strip0_length, NULL, is_setting_available },
    { Setting_RGB_StripLengt1, Group_AuxPorts, "LED strip 2 length", NULL, Format_Int8, "##0", NULL, "255", Setting_NonCore, &settings.rgb_strip1_length, NULL, is_setting_available }
};

#ifndef NO_SETTINGS_DESCRIPTIONS

static const setting_descr_t rgb_settings_descr[] = {
    { Setting_RGB_StripLengt0, "Number of LEDS in strip 1." },
    { Setting_RGB_StripLengt1, "Number of LEDS in strip 2." }
};

#endif

static void rgb_setting_changed (settings_t *settings, settings_changed_flags_t changed)
{
    hal.settings_changed(settings, changed);
}

static setting_details_t setting_details = {
    .groups = rgb_groups,
    .n_groups = sizeof(rgb_groups) / sizeof(setting_group_detail_t),
    .settings = rgb_settings,
    .n_settings = sizeof(rgb_settings) / sizeof(setting_detail_t),
#ifndef NO_SETTINGS_DESCRIPTIONS
    .descriptions = rgb_settings_descr,
    .n_descriptions = sizeof(rgb_settings_descr) / sizeof(setting_descr_t),
#endif
    .on_changed = rgb_setting_changed,
    .save = settings_write_global
};

#if RGB_LED_ENABLE == 1

static bool led_enabled;
static on_report_options_ptr on_report_options;

static void onReportOptions (bool newopt)
{
    on_report_options(newopt);

    if(!newopt)
        report_plugin(led_enabled
                       ? "RGB LED strips"
                       : "RGB LED strips (N/A)", "0.02");
}

void rgb_led_init (void)
{
    if((led_enabled = hal.rgb0.flags.is_strip || hal.rgb1.flags.is_strip))
        settings_register(&setting_details);

    on_report_options = grbl.on_report_options;
    grbl.on_report_options = onReportOptions;
}

#else

static inline bool rgb_led_settings_register (void)
{
	bool ok;

	if((ok = hal.rgb0.flags.is_strip || hal.rgb1.flags.is_strip))
		settings_register(&setting_details);

    return ok;
}

#endif

#endif // RGB_LED_ENABLE
