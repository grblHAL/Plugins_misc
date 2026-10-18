/*
  mcp4728.c - analog output to a MCP4728 I2C DAC (4 channels, 12 bit)

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

#ifdef MCP4728_ENABLE

#include "grbl/plugins.h"
#include "grbl/ioports.h"
#include <string.h>
#include <stdio.h>

#ifndef MCP4728_ADDRESS
#define MCP4728_ADDRESS 0x60
#endif

#ifndef max
#define max(a,b) (((a) > (b)) ? (a) : (b))
#endif

static float a_out[4];
static enumerate_pins_ptr on_enumerate_pins;
static io_ports_data_t analog = {};

static xbar_t mcp4728[4] = {
    {
        .id = 0,
        .group = PinGroup_AuxOutputAnalog,
        .port = &a_out[0],
        .cap = { .output = On, .analog = On, .resolution = Resolution_12bit, .external = On, .claimable = On },
        .mode = { .output = On, .analog = On }
    },
    {
        .id = 1,
        .group = PinGroup_AuxOutputAnalog,
        .port = &a_out[1],
        .cap = { .output = On, .analog = On, .resolution = Resolution_12bit, .external = On, .claimable = On },
        .mode = { .output = On, .analog = On }
    },
    {
        .id = 2,
        .group = PinGroup_AuxOutputAnalog,
        .port = &a_out[2],
        .cap = { .output = On, .analog = On, .resolution = Resolution_12bit, .external = On, .claimable = On },
        .mode = { .output = On, .analog = On }
    },
    {
        .id = 3,
        .group = PinGroup_AuxOutputAnalog,
        .port = &a_out[3],
        .cap = { .output = On, .analog = On, .resolution = Resolution_12bit, .external = On, .claimable = On },
        .mode = { .output = On, .analog = On }
    }
};

static float mcp4728_out_state (xbar_t *output)
{
    return output->id < 4 ? a_out[output->id] : -1.0f;
}

static bool mcp4728_analog_out (uint8_t port, float value)
{
    if (port < analog.out.n_ports) {
        uint8_t data[3];
        uint16_t v = (uint16_t)value;
        if (v > 4095) v = 4095;

        // Single Write Command for MCP4728
        // Byte 1: 0 1 0 0 0 DAC1 DAC0 UDAC (=1 for immediate update)
        data[0] = 0x40 | ((port & 0x03) << 1) | 0x01;
        // Byte 2: VREF(=0) PD1(=0) PD0(=0) Gx(=0) D11 D10 D9 D8
        data[1] = (v >> 8) & 0x0F;
        // Byte 3: D7 D6 D5 D4 D3 D2 D1 D0
        data[2] = v & 0xFF;

        if (i2c_send(MCP4728_ADDRESS, data, 3, true)) {
            a_out[port] = value;
            return true;
        }
    }
    return false;
}

static void mcp4728_set_value (xbar_t *output, float value)
{
    if (output->id < 4)
        mcp4728_analog_out(output->id, value);
}

static bool set_pin_function (xbar_t *output, pin_function_t function)
{
    if (output->id < 4)
        mcp4728[output->id].function = function;

    return output->id < 4;
}

static xbar_t *mcp4728_get_pin_info (io_port_direction_t dir, uint8_t port)
{
    static xbar_t pin;
    xbar_t *info = NULL;

    if (dir == Port_Output && port < analog.out.n_ports) {
        memcpy(&pin, &mcp4728[port], sizeof(xbar_t));
        pin.get_value = mcp4728_out_state;
        pin.set_value = mcp4728_set_value;
        pin.set_function = set_pin_function;
        info = &pin;
    }

    return info;
}

static void mcp4728_set_pin_description (io_port_direction_t dir, uint8_t port, const char *description)
{
    if (dir == Port_Output && port < analog.out.n_ports)
        mcp4728[port].description = description;
}

static void onEnumeratePins (bool low_level, pin_info_ptr pin_info, void *data)
{
    on_enumerate_pins(low_level, pin_info, data);

    for (uint8_t i = 0; i < analog.out.n_ports; i++) {
        xbar_t pin;
        memcpy(&pin, &mcp4728[i], sizeof(xbar_t));
        if (!low_level) {
            static char port_name[16];
            snprintf(port_name, 16, "MCP4728:%u", i);
            pin.port = port_name;
        }
        pin_info(&pin, data);
    }
}

static void get_next_port (xbar_t *pin, void *fn)
{
    if (pin->group == PinGroup_AuxOutputAnalog)
        *(pin_function_t *)fn = max(*(pin_function_t *)fn, pin->function + 1);
}

void mcp4728_init (void)
{
    if (i2c_start().ok && i2c_probe(MCP4728_ADDRESS)) {
        io_analog_t ports = {
            .ports = &analog,
            .get_pin_info = mcp4728_get_pin_info,
            .analog_out = mcp4728_analog_out,
            .set_pin_description = mcp4728_set_pin_description
        };

        pin_function_t next_fn = Output_Analog_Aux0;
        hal.enumerate_pins(false, get_next_port, &next_fn);

        for (uint8_t i = 0; i < 4; i++)
            mcp4728[i].function = next_fn + i;

        analog.out.n_ports = 4;
        ioports_add_analog(&ports);
    }

    on_enumerate_pins = hal.enumerate_pins;
    hal.enumerate_pins = onEnumeratePins;
}

#endif // MCP4728_ENABLE
