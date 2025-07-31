/*
  eventout.c - plugin for binding some events to aux output pins

  Part of grblHAL misc. plugins

  Copyright (c) 2024-2025 Terje Io

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

#if EVENTOUT_ENABLE == 1

#include <stdio.h>
#include <string.h>

#include "grbl/nvs_buffer.h"
#include "grbl/protocol.h"
#include "grbl/task.h"

#ifndef N_EVENTS
#define N_EVENTS 4
#endif

// Sanity check
#if N_EVENTS > 10
#undef N_EVENTS
#define N_EVENTS 10
#endif

#define EVENT_OPTS { .subgroups = Off, .increment = 1 }
#define EVENT_OPTS_REBOOT { .subgroups = Off, .increment = 1, .reboot_required = On }
#define EVENT_TRIGGERS "None,Spindle enable (M3/M4),Laser enable (M3/M4),Mist enable (M7),Flood enable (M8),Feed hold,Alarm,Spindle at speed"

typedef enum {
    Event_Ignore = 0,
    Event_Spindle,
    Event_Laser,
    Event_Mist,
    Event_Flood,
    Event_FeedHold,
    Event_Alarm,
    Event_SpindleAtSpeed
} event_trigger_t;

typedef struct {
    uint8_t port;
    event_trigger_t trigger;
} event_setting_t;

typedef struct {
    event_setting_t event[N_EVENTS];
} event_settings_t;

static uint8_t max_port, n_events;
static uint8_t port[N_EVENTS];
static char max_ports[4];
static nvs_address_t nvs_address;
static event_settings_t plugin_settings;
static on_report_options_ptr on_report_options;
static driver_reset_ptr driver_reset;

static coolant_set_state_ptr coolant_set_state_ = NULL;
static on_spindle_programmed_ptr on_spindle_programmed;
static on_spindle_at_speed_ptr on_spindle_at_speed;
static on_state_change_ptr on_state_change;
static bool on_spindle_programmed_attached = false;
static bool on_spindle_at_speed_attached = false;
static bool on_state_change_attached = false;

static void onReset (void)
{
    uint_fast16_t idx = n_events;

    do {
        if(port[--idx] != IOPORT_UNASSIGNED && plugin_settings.event[idx].trigger && plugin_settings.event[idx].trigger != Event_Alarm)
            ioport_digital_out(port[idx], 0);
    } while(idx);

    driver_reset();
}

static void onSpindleProgrammed (spindle_ptrs_t *spindle, spindle_state_t state, float rpm, spindle_rpm_mode_t mode)
{
    uint_fast16_t idx = n_events;

    if(on_spindle_programmed)
        on_spindle_programmed(spindle, state, rpm, mode);

    do {
        if(port[--idx] != IOPORT_UNASSIGNED && plugin_settings.event[idx].trigger == Event_SpindleAtSpeed)
            ioport_digital_out(port[idx], state.on);
    } while(idx);
}

static void onSpindleAtSpeed (spindle_ptrs_t *spindle, spindle_state_t state)
{
    uint_fast16_t idx = n_events;

    if(on_spindle_at_speed)
        on_spindle_at_speed(spindle, state);

    do {
        if(port[--idx] != IOPORT_UNASSIGNED && plugin_settings.event[idx].trigger == (spindle->cap.laser ? Event_Laser : Event_Spindle))
            ioport_digital_out(port[idx], state.on);
    } while(idx);
}

static void onCoolantSetState (coolant_state_t state)
{
    uint_fast16_t idx = n_events;

    coolant_set_state_(state);

    do {
        if(port[--idx] != IOPORT_UNASSIGNED)
          switch(plugin_settings.event[idx].trigger) {

            case Event_Mist:
                ioport_digital_out(port[idx], state.mist);
                break;

            case Event_Flood:
                ioport_digital_out(port[idx], state.flood);
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
            if(port[--idx] != IOPORT_UNASSIGNED)
              switch(plugin_settings.event[idx].trigger) {

                case Event_FeedHold:
                    ioport_digital_out(port[idx], state == STATE_HOLD);
                    break;

                case Event_Alarm:
                    ioport_digital_out(port[idx], state == STATE_ALARM);
                    break;

                default: break;
            }
        } while(idx);
    }

    if(on_state_change)
        on_state_change(state);
}

static void register_handlers (void)
{
    static char descr[N_EVENTS][25] = {0};

    uint_fast16_t idx = n_events;

    do {
        if(port[--idx] != IOPORT_UNASSIGNED)
          switch(plugin_settings.event[idx].trigger) {

            case Event_Laser:
            case Event_Spindle:
                sprintf(descr[idx], "P%d <- %s", port[idx], plugin_settings.event[idx].trigger == Event_Spindle ? "Spindle enable" : "Laser enable");
                if(!on_spindle_programmed_attached) {
                    on_spindle_programmed_attached = true;
                    on_spindle_programmed = grbl.on_spindle_programmed;
                    grbl.on_spindle_programmed = onSpindleProgrammed;
                }
                break;

            case Event_SpindleAtSpeed:
                sprintf(descr[idx], "P%d <- %s", port[idx], "Spindle at speed");
                if(!on_spindle_at_speed_attached) {
                    on_spindle_at_speed_attached = true;
                    on_spindle_at_speed = grbl.on_spindle_at_speed;
                    grbl.on_spindle_at_speed = onSpindleAtSpeed;
                }
                break;

            case Event_Mist:
            case Event_Flood:
                sprintf(descr[idx], "P%d <- %s", port[idx], plugin_settings.event[idx].trigger == Event_Mist ? "Mist enable" : "Flood enable");
                if(coolant_set_state_ == NULL) {
                    coolant_set_state_ = hal.coolant.set_state;
                    hal.coolant.set_state = onCoolantSetState;
                }
                break;

            case Event_Alarm:
            case Event_FeedHold:
                sprintf(descr[idx], "P%d <- %s", port[idx], plugin_settings.event[idx].trigger == Event_Alarm ? "Alarm": "Feed hold");
                if(!on_state_change_attached) {
                    on_state_change_attached = true;
                    on_state_change = grbl.on_state_change;
                    grbl.on_state_change = onStateChanged;
                }
                break;

            default:
                sprintf(descr[idx], "P%d", port[idx]);
                break;
        }

        ioport_set_description(Port_Digital, Port_Output, port[idx], descr[idx]);

    } while(idx);
}

static status_code_t set_int (setting_id_t id, uint_fast16_t value)
{
    plugin_settings.event[id - Setting_ActionBase].trigger = (event_trigger_t)value;

    return Status_OK;
}

static uint_fast16_t get_int (setting_id_t id)
{
    return plugin_settings.event[id - Setting_ActionBase].trigger;
}

static status_code_t set_port (setting_id_t id, float value)
{
    if(!isintf(value))
        return Status_BadNumberFormat;

    if(!ioport_claimable(Port_Digital, Port_Output, (uint8_t)value))
        return Status_AuxiliaryPortUnavailable;

    plugin_settings.event[id - Setting_ActionPortBase].port = value < 0.0f ? IOPORT_UNASSIGNED : (uint8_t)value;

    return Status_OK;
}

static float get_port (setting_id_t id)
{
    return plugin_settings.event[id - Setting_ActionPortBase].port > max_port ? -1.0f : (float)plugin_settings.event[id - Setting_ActionPortBase].port;
}

static bool is_setting_available (const setting_detail_t *setting, uint_fast16_t offset)
{
    return offset < n_events;
}

static const setting_detail_t event_settings[] = {
    { Setting_ActionBase, Group_AuxPorts, "Event ? trigger", NULL, Format_RadioButtons, EVENT_TRIGGERS, NULL, NULL, Setting_NonCoreFn, set_int, get_int, is_setting_available, EVENT_OPTS },
    { Setting_ActionPortBase, Group_AuxPorts, "Event ? port", NULL, Format_Decimal, "-#0", "-1", max_ports, Setting_NonCoreFn, set_port, get_port, is_setting_available, EVENT_OPTS_REBOOT }
};

static const setting_descr_t event_settings_descr[] = {
    { Setting_ActionBase, "Event triggering output port change.\\n\\n"
                          "NOTE: the port can still be controlled by M62-M65 commands even when bound to an event."},
    { Setting_ActionPortBase, "Aux output port number to bind to the associated event trigger. Set to -1 to disable." }
};

static void event_settings_save (void)
{
    hal.nvs.memcpy_to_nvs(nvs_address, (uint8_t *)&plugin_settings, sizeof(event_settings_t), true);
}

static void event_settings_restore (void)
{
    uint_fast8_t idx;

    if(n_events == 0 && (n_events = ioports_unclaimed(Port_Digital, Port_Output)))
        n_events = min(n_events, N_EVENTS);

    idx = n_events;

    memset(&plugin_settings, 0xFF, sizeof(event_settings_t));

    plugin_settings.event[idx - 1].port = ioport_find_free(Port_Digital, Port_Output, (pin_cap_t){ .claimable = On }, NULL);

    do {

        switch(--idx) {

#ifdef EVENTOUT_1_ACTION
            case 0:
                plugin_settings.event[idx].trigger = (event_trigger_t)EVENTOUT_1_ACTION;
                break;
#endif
#ifdef EVENTOUT_2_ACTION
            case 1:
                plugin_settings.event[idx].trigger = (event_trigger_t)EVENTOUT_2_ACTION;
                break;
#endif
#ifdef EVENTOUT_3_ACTION
            case 2:
                plugin_settings.event[idx].trigger = (event_trigger_t)EVENTOUT_3_ACTION;
                break;
#endif
#ifdef EVENTOUT_4_ACTION
            case 3:
                plugin_settings.event[idx].trigger = (event_trigger_t)EVENTOUT_4_ACTION;
                break;
#endif

            default:
                plugin_settings.event[idx].trigger = Event_Ignore;
                break;
        }

        if(idx < n_events - 1)
            plugin_settings.event[idx].port = ioport_find_free(Port_Digital, Port_Output, (pin_cap_t){ .claimable = On }, uitoa(plugin_settings.event[idx + 1].port));

    } while(idx);

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

static setting_id_t event_settings_normalize (setting_id_t id)
{
    return (id > Setting_ActionBase && id <= Setting_Action9) ||
            (id > Setting_ActionPortBase && id <= Setting_ActionPort9)
              ? (setting_id_t)(id - (id % 10))
              : id;
}

static void onReportOptions (bool newopt)
{
    on_report_options(newopt);

    if(!newopt)
        report_plugin("Events plugin", "0.09");
}

static void event_out_cfg (void *data)
{
    n_events = ioports_unclaimed(Port_Digital, Port_Output);

    if(min(n_events, N_EVENTS)) {

        strcpy(max_ports, uitoa((max_port = ioport_find_free(Port_Digital, Port_Output, (pin_cap_t){ .claimable = On }, NULL))));

        uint_fast16_t idx;
        for(idx = 0; idx < n_events; idx++) {
            if(plugin_settings.event[idx].port == IOPORT_UNASSIGNED)
                port[idx] = IOPORT_UNASSIGNED;
            else
                port[idx] = min(plugin_settings.event[idx].port, max_port);
        }

        register_handlers();
    }
}

void event_out_init (void)
{
    static setting_details_t setting_details = {
        .settings = event_settings,
        .n_settings = sizeof(event_settings) / sizeof(setting_detail_t),
        .descriptions = event_settings_descr,
        .n_descriptions = sizeof(event_settings_descr) / sizeof(setting_descr_t),
        .save = event_settings_save,
        .load = event_settings_load,
        .restore = event_settings_restore,
        .iterator = event_settings_iterator,
        .normalize = event_settings_normalize
    };

    if((nvs_address = nvs_alloc(sizeof(event_settings_t)))) {

        settings_register(&setting_details);

        on_report_options = grbl.on_report_options;
        grbl.on_report_options = onReportOptions;

        driver_reset = hal.driver_reset;
        hal.driver_reset = onReset;

        task_run_on_startup(event_out_cfg, NULL);
    } else
        task_run_on_startup(report_warning, "Events plugin failed to initialize!");
}

#endif // EVENTOUT_ENABLE
