/*
  eventout.c - plugin for binding some events to aux output pins

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

#ifdef ARDUINO
#include "../driver.h"
#else
#include "driver.h"
#endif

#if EVENTOUT_ENABLE

#include <string.h>

#include "grbl/nvs_buffer.h"
#include "grbl/protocol.h"

#ifndef N_EVENTS
#define N_EVENTS 4
#endif

// Sanity check
#if N_EVENTS > 10
#undef N_EVENTS
#define N_EVENTS 10
#endif

#define EVENT_OPTS { .subgroups = Off, .increment = 1 }
#define EVENT_TRIGGERS "None,Spindle enable (M3/M4),Laser enable (M3/M4),Mist enable (M7),Flood enable (M8),Feed hold"

typedef enum {
    Event_Ignore = 0,
    Event_Spindle,
    Event_Laser,
    Event_Mist,
    Event_Flood,
    Event_FeedHold
} event_trigger_t;

typedef struct {
    uint8_t port;
    event_trigger_t trigger;
} event_setting_t;

typedef struct {
    event_setting_t event[N_EVENTS];
} event_settings_t;

static uint8_t n_ports, n_events;
static uint8_t port[N_EVENTS];
static char max_port[4];
static nvs_address_t nvs_address;
static event_settings_t plugin_settings;
static on_report_options_ptr on_report_options;
static driver_reset_ptr driver_reset;

static coolant_set_state_ptr coolant_set_state_ = NULL;
static on_spindle_programmed_ptr on_spindle_programmed;
static on_state_change_ptr on_state_change;
static bool on_spindle_programmed_attached = false;
static bool on_state_change_attached = false;

static void onReset (void)
{
    uint_fast16_t idx = n_events;

    do {
        if(plugin_settings.event[--idx].trigger)
            hal.port.digital_out(port[idx], 0);
    } while(idx);

    driver_reset();
}

static void onSpindleProgrammed (spindle_ptrs_t *spindle, spindle_state_t state, float rpm, spindle_rpm_mode_t mode)
{
    uint_fast16_t idx = n_events;

    if(on_spindle_programmed)
        on_spindle_programmed(spindle, state, rpm, mode);

    do {
        if(plugin_settings.event[--idx].trigger == (spindle->cap.laser ? Event_Laser : Event_Spindle))
            hal.port.digital_out(port[idx], state.on);
    } while(idx);
}

static void onCoolantSetState (coolant_state_t state)
{
    uint_fast16_t idx = n_events;

    coolant_set_state_(state);

    do {
        switch(plugin_settings.event[--idx].trigger) {

            case Event_Mist:
                hal.port.digital_out(port[idx], state.mist);
                break;

            case Event_Flood:
                hal.port.digital_out(port[idx], state.flood);
                break;

            default:
                break;
        }
    } while(idx);
}

static void onStateChanged (sys_state_t state)
{
    static sys_state_t last_state = STATE_IDLE;

    if(state != last_state) {

        uint_fast16_t idx = n_events;

        last_state = state;

        do {
            if(plugin_settings.event[--idx].trigger == Event_FeedHold)
                hal.port.digital_out(port[idx], state == STATE_HOLD);
        } while(idx);
    }

    if(on_state_change)
        on_state_change(state);
}

static void register_handlers (void)
{
    uint_fast16_t idx = n_events;

    do {
        switch(plugin_settings.event[--idx].trigger) {

            case Event_Spindle:
                if(!on_spindle_programmed_attached) {
                    on_spindle_programmed_attached = true;
                    on_spindle_programmed = grbl.on_spindle_programmed;
                    grbl.on_spindle_programmed = onSpindleProgrammed;
                }
                break;

            case Event_Mist:
            case Event_Flood:
                if(coolant_set_state_ == NULL) {
                    coolant_set_state_ = hal.coolant.set_state;
                    hal.coolant.set_state = onCoolantSetState;
                }
                break;

            case Event_FeedHold:
                if(!on_state_change_attached) {
                    on_state_change_attached = true;
                    on_state_change = grbl.on_state_change;
                    grbl.on_state_change = onStateChanged;
                }
                break;

            default:
                break;
        }
    } while(idx);
}

static setting_id_t normalize_id (setting_id_t setting, uint_fast16_t *idx)
{
    *idx = setting % 10;

    return (setting_id_t)(setting - *idx);
}

static status_code_t set_int (setting_id_t setting, uint_fast16_t value)
{
    uint_fast16_t idx;
    status_code_t status = Status_OK;

    setting = normalize_id(setting, &idx);

    if(idx < n_events) switch(setting) {

        case Setting_ActionBase:
            plugin_settings.event[idx].trigger = value;
            register_handlers();
            break;

        case Setting_ActionPortBase:
            plugin_settings.event[idx].port = value;
            break;

        default:
            status = Status_Unhandled;
            break;
    }

    return status;
}

static uint_fast16_t get_int (setting_id_t setting)
{
    uint_fast16_t idx, value = 0;

    setting = normalize_id(setting, &idx);

    if(idx < n_events) switch(setting) {

        case Setting_ActionBase:
            value = plugin_settings.event[idx].trigger;
            break;

        case Setting_ActionPortBase:
            value = plugin_settings.event[idx].port;
            break;

        default:
            break;
    }

    return value;
}

static bool is_setting_available (const setting_detail_t *setting)
{
    uint_fast16_t idx;

    normalize_id(setting->id, &idx);

    return idx < n_events;
}

static const setting_group_detail_t macro_groups [] = {
    { Group_Root, Group_UserSettings, "Macros"}
};

static const setting_detail_t event_settings[] = {
    { Setting_ActionBase, Group_AuxPorts, "Event ? trigger", NULL, Format_RadioButtons, EVENT_TRIGGERS, NULL, NULL, Setting_NonCoreFn, set_int, get_int, is_setting_available, EVENT_OPTS },
    { Setting_ActionPortBase, Group_AuxPorts, "Event ? port", NULL, Format_Int8, "#0", "0", max_port, Setting_NonCoreFn, set_int, get_int, is_setting_available, EVENT_OPTS }
};

#ifndef NO_SETTINGS_DESCRIPTIONS

static const setting_descr_t event_settings_descr[] = {
    { Setting_ActionBase, "Event triggering output port change.\\n\\n"
                          "NOTE: the port can still be controlled by M62-M65 commands even when bound to an event."},
    { Setting_ActionPortBase, "Aux output port number to bind to the associated event trigger." SETTINGS_HARD_RESET_REQUIRED }
};

#endif

static void event_settings_save (void)
{
    hal.nvs.memcpy_to_nvs(nvs_address, (uint8_t *)&plugin_settings, sizeof(event_settings_t), true);
}

static void event_settings_restore (void)
{
    uint_fast8_t idx = n_ports;

    memset(&plugin_settings, 0xFF, sizeof(event_settings_t));

    for(idx = 0; idx < n_events; idx++) {
        plugin_settings.event[idx].trigger = Event_Ignore;
        plugin_settings.event[idx].port = hal.port.num_digital_out < (idx + 1) ? 0xFF : idx;
    }

    hal.nvs.memcpy_to_nvs(nvs_address, (uint8_t *)&plugin_settings, sizeof(event_settings_t), true);
}

static void event_settings_load (void)
{
    if(hal.nvs.memcpy_from_nvs((uint8_t *)&plugin_settings, nvs_address, sizeof(event_settings_t), true) != NVS_TransferResult_OK)
        event_settings_restore();
}

static bool event_settings_iterator (const setting_detail_t *setting, setting_output_ptr callback, void *data)
{
    uint_fast16_t idx;

    for(idx = 0; idx < n_events; idx++)
        callback(setting, idx, data);

    return true;
}

static setting_details_t setting_details = {
    .groups = macro_groups,
    .n_groups = sizeof(macro_groups) / sizeof(setting_group_detail_t),
    .settings = event_settings,
    .n_settings = sizeof(event_settings) / sizeof(setting_detail_t),
#ifndef NO_SETTINGS_DESCRIPTIONS
    .descriptions = event_settings_descr,
    .n_descriptions = sizeof(event_settings_descr) / sizeof(setting_descr_t),
#endif
    .save = event_settings_save,
    .load = event_settings_load,
    .restore = event_settings_restore,
    .iterator = event_settings_iterator
};

static void report_options (bool newopt)
{
    on_report_options(newopt);

    if(!newopt)
        report_plugin("Events plugin", "0.01");
}

static void event_out_cfg (void *data)
{
    if((n_ports = hal.port.num_digital_out)) {

        n_events = min(n_ports, N_EVENTS);
        strcpy(max_port, uitoa(n_ports - 1));

        uint_fast16_t idx;
        for(idx = 0; idx < n_events; idx++) {
            port[idx] = plugin_settings.event[idx].port;
        }

        register_handlers();
    }
}

void event_out_init (void)
{
    if((nvs_address = nvs_alloc(sizeof(event_settings_t)))) {

        settings_register(&setting_details);

        on_report_options = grbl.on_report_options;
        grbl.on_report_options = report_options;

        driver_reset = hal.driver_reset;
        hal.driver_reset = onReset;

        protocol_enqueue_foreground_task(event_out_cfg, NULL);
    } else
        protocol_enqueue_foreground_task(report_warning, "Events plugin failed to initialize!");
}

#endif // EVENTOUT_ENABLE