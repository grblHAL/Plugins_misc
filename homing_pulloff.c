/*
  homing_pulloff.c - adds per axis settings for homing pulloff distance

  Part of grblHAL misc. plugins

  Copyright (c) 2024 Terje Io

  grblHAL is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  grblHAL is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with grblHAL. If not, see <http://www.gnu.org/licenses/>.
*/

#include "driver.h"

#if HOMING_PULLOFF_ENABLE == 1

#include "grbl/nvs_buffer.h"

typedef struct {
    coord_data_t pulloff;
} plugin_settings_t;

static nvs_address_t nvs_address;
static plugin_settings_t homing;
static on_report_options_ptr on_report_options;
static settings_changed_ptr settings_changed;

static status_code_t set_axis_setting (setting_id_t setting, float value)
{
    uint_fast8_t idx;
    status_code_t status = Status_OK;

    switch(settings_get_axis_base(setting, &idx)) {

        case Setting_AxisExtended9:
            homing.pulloff.values[idx] = value;
            limits_homing_pulloff(&homing.pulloff);
            break;

        default:
            status = Status_SettingDisabled;
            break;
    }

    return status;
}

static float get_float (setting_id_t setting)
{
    uint_fast8_t idx;
    float value = 0.0f;

    switch(settings_get_axis_base(setting, &idx)) {

        case Setting_AxisExtended9:
            value = homing.pulloff.values[idx];
            break;

        default:
            break;
    }

    return value;
}

static const setting_detail_t plugin_settings[] = {
    { Setting_AxisExtended9 , Group_Axis0, "-axis homing switch pull-off distance", "mm", Format_Decimal, "#0.000", NULL, NULL, Setting_IsLegacyFn, set_axis_setting, get_float, NULL, { .subgroups = On, .increment = 1 } }
};

#ifndef NO_SETTINGS_DESCRIPTIONS

static const setting_descr_t plugin_settings_descr[] = {
    { Setting_AxisExtended9, "Maximum axis travel distance from homing switch. Determines valid machine space for soft-limits and homing search distances." }
};

#endif

static void plugin_settings_save (void)
{
    hal.nvs.memcpy_to_nvs(nvs_address, (uint8_t *)&homing, sizeof(plugin_settings_t), true);
}

static void plugin_settings_restore (void)
{
    uint_fast8_t idx = N_AXIS;

    do {
        homing.pulloff.values[--idx] = settings.homing.pulloff;
    } while(idx);

    limits_homing_pulloff(&homing.pulloff);

    hal.nvs.memcpy_to_nvs(nvs_address, (uint8_t *)&homing, sizeof(plugin_settings_t), true);
}

static void plugin_settings_load (void)
{
    if(hal.nvs.memcpy_from_nvs((uint8_t *)&homing, nvs_address, sizeof(plugin_settings_t), true) != NVS_TransferResult_OK)
        plugin_settings_restore();

    limits_homing_pulloff(&homing.pulloff);
}

static void onSettingsChanged (settings_t *settings, settings_changed_flags_t changed)
{
    static float pulloff = -1.0f;

    if(pulloff != settings->homing.pulloff) {

        if(pulloff != -1 && pulloff != settings->homing.pulloff) {

            uint_fast8_t idx = N_AXIS;

            do {
                if(homing.pulloff.values[--idx] == pulloff)
                    homing.pulloff.values[idx] = settings->homing.pulloff;
            } while(idx);

            plugin_settings_save();
            limits_homing_pulloff(&homing.pulloff);
        }

        pulloff = settings->homing.pulloff;
    }

    settings_changed(settings, changed);
}

static void on_report_my_options (bool newopt)
{
    on_report_options(newopt);

    if(!newopt)
        report_plugin("Homing pulloff", "0.01");
}

void homing_pulloff_init (void)
{
    static setting_details_t setting_details = {
        .settings = plugin_settings,
        .n_settings = sizeof(plugin_settings) / sizeof(setting_detail_t),
#ifndef NO_SETTINGS_DESCRIPTIONS
        .descriptions = plugin_settings_descr,
        .n_descriptions = sizeof(plugin_settings_descr) / sizeof(setting_descr_t),
#endif
        .save = plugin_settings_save,
        .load = plugin_settings_load,
        .restore = plugin_settings_restore
    };

    if((nvs_address = nvs_alloc(sizeof(plugin_settings_t)))) {

        settings_changed = hal.settings_changed;
        hal.settings_changed = onSettingsChanged;

        on_report_options = grbl.on_report_options;
        grbl.on_report_options = on_report_my_options;

        settings_register(&setting_details);
    }
}

#endif // HOMING_PULLOFF_ENABLE
