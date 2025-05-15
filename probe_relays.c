/*

  probe_relays.c - controls relay(s) for switching between probes using a single probe input

  Use G65P5Q<n> to select probe where <n> = 0 is for direct input, <n> = 1 is for toolsetter and <n> = 2 is for second spindle probe.

  Part of grblHAL

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

#if PROBE_ENABLE == 2 && NGC_PARAMETERS_ENABLE

#include "grbl/hal.h"
#include "grbl/task.h"
#include "grbl/nvs_buffer.h"

#ifndef PROBE_RELAY_DEBOUNCE
#define PROBE_RELAY_DEBOUNCE 50 // ms - increase if relay is slow and/or bouncy
#endif

typedef struct {
    uint8_t port[2];
} relay_settings_t;

typedef struct {
    uint8_t port;
    bool enabled;
    bool on;
    bool probe_ok;
} probe_relay_t;

static uint8_t n_ports;
static char max_port[4];
static probe_relay_t relay[2];
static relay_settings_t relay_settings;
static nvs_address_t nvs_address;
static on_report_options_ptr on_report_options;
static relay_settings_t relay_settings;
static probe_select_ptr hal_probe_select;

bool onProbeSelect (probe_id_t probe_id)
{
    bool ok;

    if((ok = probe_id == Probe_Default || (hal_probe_select && hal_probe_select(probe_id))))
        relay[0].on = relay[1].on = false;

    else switch(probe_id) {

        case Probe_Toolsetter:
            if((ok = relay[0].enabled)) {
                relay[0].on = true;
                relay[1].on = false;
            }
            break;

        case Probe_2:
            if((ok = relay[1].enabled)) {
                relay[0].on = false;
                relay[1].on = true;
            }
            break;

        default: break;
    }

    if(ok) {

        if(relay[0].enabled)
            hal.port.digital_out(relay[0].port, relay[0].on);

        if(relay[1].enabled)
            hal.port.digital_out(relay[1].port, relay[1].on);

        hal.delay_ms(PROBE_RELAY_DEBOUNCE, NULL); // Delay a bit to let any contact bounce settle.
    }

    return ok;
}

static status_code_t set_port (setting_id_t setting, float value)
{
    if(!isintf(value))
        return Status_BadNumberFormat;

    if(setting == Setting_RelayPortToolsetter)
        relay_settings.port[0] = value < 0.0f ? IOPORT_UNASSIGNED : (uint8_t)value;
    else
        relay_settings.port[1] = value < 0.0f ? IOPORT_UNASSIGNED : (uint8_t)value;

    return Status_OK;
}

static float get_port (setting_id_t setting)
{
    return setting == Setting_RelayPortToolsetter
            ? (relay_settings.port[0] >= n_ports ? -1.0f : (float)relay_settings.port[0])
            : (relay_settings.port[1] >= n_ports ? -1.0f : (float)relay_settings.port[1]);
}

static bool is_setting_available (const setting_detail_t *setting, uint_fast16_t offset)
{
    return setting->id == Setting_RelayPortToolsetter
            ? !relay[0].probe_ok
            : !relay[1].probe_ok;
}

static const setting_detail_t user_settings[] = {
    { Setting_RelayPortToolsetter, Group_AuxPorts, "Toolsetter relay port", NULL, Format_Decimal, "-#0", "-1", max_port, Setting_NonCoreFn, set_port, get_port, is_setting_available, { .reboot_required = On } },
    { Setting_RelayPortProbe2, Group_AuxPorts, "Probe 2 relay port", NULL, Format_Decimal, "-#0", "-1", max_port, Setting_NonCoreFn, set_port, get_port, is_setting_available, { .reboot_required = On } }
};

#ifndef NO_SETTINGS_DESCRIPTIONS

static const setting_descr_t relay_settings_descr[] = {
    { Setting_RelayPortToolsetter, "Aux port number to use for toolsetter relay control. Set to -1 to disable." },
    { Setting_RelayPortProbe2, "Aux port number to use for probe 2 relay control. Set to -1 to disable." }
};

#endif

// Write settings to non volatile storage (NVS).
static void plugin_settings_save (void)
{
    hal.nvs.memcpy_to_nvs(nvs_address, (uint8_t *)&relay_settings, sizeof(relay_settings_t), true);
}

static void plugin_settings_restore (void)
{
    relay_settings.port[0] = relay[0].probe_ok
                              ? IOPORT_UNASSIGNED
                              : ioport_find_free(Port_Digital, Port_Output, (pin_cap_t){ .claimable = On }, "Toolsetter relay");
    relay_settings.port[1] = relay[0].probe_ok
                              ? IOPORT_UNASSIGNED
                              : ioport_find_free(Port_Digital, Port_Output, (pin_cap_t){ .claimable = On }, "Probe2 relay");

    if(relay_settings.port[0] == relay_settings.port[1] && relay_settings.port[1] != IOPORT_UNASSIGNED)
        relay_settings.port[1] -= 1;

    hal.nvs.memcpy_to_nvs(nvs_address, (uint8_t *)&relay_settings, sizeof(relay_settings_t), true);
}

static void plugin_settings_load (void)
{
    bool ok = true;

    if(hal.nvs.memcpy_from_nvs((uint8_t *)&relay_settings, nvs_address, sizeof(relay_settings_t), true) != NVS_TransferResult_OK)
        plugin_settings_restore();

    if(relay[0].probe_ok)
        relay_settings.port[0] = IOPORT_UNASSIGNED;
    else if((relay[0].port = relay_settings.port[0]) != IOPORT_UNASSIGNED &&
             ioport_claim(Port_Digital, Port_Output, &relay[0].port, "Toolsetter relay"))
        relay[0].enabled = (hal.driver_cap.toolsetter = On);
    else
        ok = false;

    if(relay[1].probe_ok)
        relay_settings.port[1] = IOPORT_UNASSIGNED;
    else if((relay[1].port = relay_settings.port[1]) != IOPORT_UNASSIGNED &&
          ioport_claim(Port_Digital, Port_Output, &relay[1].port, "Probe2 relay"))
        relay[1].enabled = (hal.driver_cap.probe2 = On);
    else
        ok = false;

    if(relay[0].enabled || relay[1].enabled) {
        hal_probe_select = hal.probe.select;
        hal.probe.select = onProbeSelect;
    } else if(!ok)
        task_run_on_startup(report_warning, "Probe relay plugin: configured port number(s) not available");
}

static void onReportOptions (bool newopt)
{
    on_report_options(newopt);

    if(!newopt)
        report_plugin("Probe relay", "0.01");
}

void probe_select_init (void)
{
    static setting_details_t setting_details = {
        .settings = user_settings,
        .n_settings = sizeof(user_settings) / sizeof(setting_detail_t),
    #ifndef NO_SETTINGS_DESCRIPTIONS
        .descriptions = relay_settings_descr,
        .n_descriptions = sizeof(relay_settings_descr) / sizeof(setting_descr_t),
    #endif
        .save = plugin_settings_save,
        .load = plugin_settings_load,
        .restore = plugin_settings_restore
    };

    if(hal.probe.get_state == NULL || (hal.driver_cap.probe2 && hal.driver_cap.toolsetter))
        return;

    relay[0].probe_ok = hal.driver_cap.toolsetter;
    relay[1].probe_ok = hal.driver_cap.probe2;

    if(ioport_can_claim_explicit() &&
       (n_ports = ioports_available(Port_Digital, Port_Output)) &&
        (nvs_address = nvs_alloc(sizeof(relay_settings_t)))) {

        strcpy(max_port, uitoa(n_ports - 1));

        on_report_options = grbl.on_report_options;
        grbl.on_report_options = onReportOptions;

        settings_register(&setting_details);
    } else
        task_run_on_startup(report_warning, "Probe relay plugin failed to initialize!");
}

#endif // PROBE_ENABLE == 2 && NGC_PARAMETERS_ENABLE
