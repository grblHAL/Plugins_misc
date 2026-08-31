/*
  ina219.c - INA219 I2C digital power monitor

  Part of grblHAL

  Copyright (c) 2026 Ooznest Ltd.

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

#ifdef INA219_ENABLE

#include "grbl/plugins.h"
#include <string.h>

#ifndef INA219_ADDRESS
#define INA219_ADDRESS 0x40
#endif

#ifndef INA219_POLL_INTERVAL
#define INA219_POLL_INTERVAL 500
#endif

static float last_vbus_v = 0.0f;
static float last_current_a = 0.0f;
static on_realtime_report_ptr prev_realtime_report;

static void ina219_task (void *pvParameters)
{
    static uint32_t last_poll = 0;

    if (hal.get_elapsed_ticks() - last_poll < INA219_POLL_INTERVAL) return;
    last_poll = hal.get_elapsed_ticks();

    uint8_t reg;
    uint8_t data[2];

    // Read Bus Voltage Register (0x02)
    reg = 0x02;
    if (i2c_send(INA219_ADDRESS, &reg, 1, true) && i2c_receive(INA219_ADDRESS, data, 2, true)) {
        uint16_t bus_reg = (data[0] << 8) | data[1];
        last_vbus_v = (bus_reg >> 3) * 0.004f;
    }

    // Read Shunt Voltage Register (0x01)
    reg = 0x01;
    if (i2c_send(INA219_ADDRESS, &reg, 1, true) && i2c_receive(INA219_ADDRESS, data, 2, true)) {
        int16_t shunt_reg = (data[0] << 8) | data[1];
        float shunt_mv = shunt_reg * 0.01f;
        last_current_a = (shunt_mv / 1000.0f) / 0.03f; // R = 0.03 ohm
    }
}

static void ina219_on_realtime_report (stream_write_ptr stream_write, report_tracking_flags_t report)
{
    if (prev_realtime_report)
        prev_realtime_report(stream_write, report);

    stream_write("|INA219:");
    stream_write(ftoa(last_vbus_v, 2));
    stream_write(",");
    stream_write(ftoa(last_current_a, 2));
}

void ina219_init (void)
{
    if (i2c_start().ok && i2c_probe(INA219_ADDRESS)) {
        prev_realtime_report = grbl.on_realtime_report;
        grbl.on_realtime_report = ina219_on_realtime_report;
        task_add_systick(ina219_task, NULL);
    }
}

#endif // INA219_ENABLE
