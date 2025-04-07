/*
  mcp3221.c - analog input from a MCP3221 I2C ADC

  Part of grblHAL

  Copyright (c) 2021-2025 Terje Io

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

#ifdef MCP3221_ENABLE_NEW

#include "grbl/plugins.h"
#include "grbl/ioports.h"

static float value;
static enumerate_pins_ptr on_enumerate_pins;
static io_ports_data_t analog = {};

static xbar_t mcp3221 = {
    .id = 0,
    .function = Input_Analog_Aux0,
    .group = PinGroup_AuxInputAnalog,
    .port = &value,
    .cap = {
        .output = On,
        .analog = On,
        .external = On,
        .claimable = On
    },
    .mode = {
        .output = On,
        .analog = On
    }
};

static float mcp3221_in_state (xbar_t *input)
{
    value = -1;

    if(input->id == mcp3221.id) {

        uint8_t result[2];

        i2c_receive(MCP3221_ENABLE, result, 2, true);

        value = (float)((result[0] << 8) | result[1]);
    }

    return value;
}

static int32_t mcp3221_wait_on_input (uint8_t port, wait_mode_t wait_mode, float timeout)
{
    return port < analog.in.n_ports ? (int32_t)mcp3221_in_state(&mcp3221) : -1;
}

static bool set_pin_function (xbar_t *input, pin_function_t function)
{
    if(input->id == mcp3221.id)
        mcp3221.function = function;

    return input->id == mcp3221.id;
}

static xbar_t *mcp3221_get_pin_info (io_port_direction_t dir, uint8_t port)
{
    static xbar_t pin;

    xbar_t *info = NULL;

    memcpy(&pin, &mcp3221, sizeof(xbar_t));

    if(dir == Port_Input && port < analog.in.n_ports) {
        pin.get_value = mcp3221_in_state;
        pin.set_function = set_pin_function;
        info = &pin;
    }

    return info;
}

static void mcp3221_set_pin_description (io_port_direction_t dir, uint8_t port, const char *description)
{
    if(dir == Port_Input && port < analog.in.n_ports)
        mcp3221.description = description;
}

static void onEnumeratePins (bool low_level, pin_info_ptr pin_info, void *data)
{
    static xbar_t pin = {};

    on_enumerate_pins(low_level, pin_info, data);

    memcpy(&pin, &mcp3221, sizeof(xbar_t));

    if(!low_level)
        pin.port = "MCP3221:";

    if(!analog.in.n_ports)
        mcp3221.description = "No power";

    pin_info(&pin, data);
}

static void get_next_port (xbar_t *pin, void *fn)
{
    if(pin->group == PinGroup_AuxInputAnalog)
        *(pin_function_t *)fn = max(*(pin_function_t *)fn, pin->function + 1);
}

void mcp3221_init (void)
{
    if(i2c_start().ok && i2c_probe(MCP3221_ENABLE)) {

        io_analog_t ports = {
            .ports = &analog,
            .get_pin_info = mcp3221_get_pin_info,
            .wait_on_input = mcp3221_wait_on_input,
            .set_pin_description = mcp3221_set_pin_description
        };

        hal.enumerate_pins(false, get_next_port, &mcp3221.function);

        analog.in.n_ports = 1;
        ioports_add_analog(&ports);
    }

    on_enumerate_pins = hal.enumerate_pins;
    hal.enumerate_pins = onEnumeratePins;
}

#endif // MCP3221_ENABLE
