/*
  mcp4725.c - analog output to a MCP4725 I2C DAC (12 bit)

  Part of grblHAL

  Copyright (c) 2025 Terje Io

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

#ifdef MCP4725_ENABLE

#include "grbl/plugins.h"
#include "grbl/ioports.h"

#ifndef MCP4725_ADDRESS
#define MCP4725_ADDRESS (0xC0 >> 1)
#endif

static float a_out;
static enumerate_pins_ptr on_enumerate_pins;
static io_ports_data_t analog = {};

static xbar_t mcp4725 = {
    .id = 0,
    .function = Output_Analog_Aux0,
    .group = PinGroup_AuxOutputAnalog,
    .port = &a_out,
    .cap = {
        .output = On,
        .analog = On,
        .resolution = Resolution_12bit,
        .external = On,
        .claimable = On
    },
    .mode = {
        .output = On,
        .analog = On
    }
};

static float mcp4725_out_state (xbar_t *output)
{
    return output->id == mcp4725.id ? a_out : -1.0f;
}

static bool mcp4725_analog_out (uint8_t port, float value)
{
    bool ok;

    if((ok = port < analog.out.n_ports)) {

        uint8_t data[3];

        data[0] = 0x40; // Output data
        data[1] = ((uint16_t)value & 0xFF0) >> 4;   // MSB data
        data[2] = ((uint16_t)value & 0x00F) << 4;   // LSB data

        if((ok = i2c_send(MCP4725_ADDRESS, data, 3, true)))
            a_out = value;
    }

    return port < analog.out.n_ports;
}

static void mcp4725_set_value (xbar_t *output, float value)
{
    if(output->id == mcp4725.id)
        mcp4725_analog_out(mcp4725.id, value);
}

static bool set_pin_function (xbar_t *output, pin_function_t function)
{
    if(output->id == mcp4725.id)
        mcp4725.function = function;

    return output->id == mcp4725.id;
}

static xbar_t *mcp4725_get_pin_info (io_port_direction_t dir, uint8_t port)
{
    static xbar_t pin;

    xbar_t *info = NULL;

    memcpy(&pin, &mcp4725, sizeof(xbar_t));

    if(dir == Port_Output && port < analog.out.n_ports) {
        pin.get_value = mcp4725_out_state;
        pin.set_value = mcp4725_set_value;
        pin.set_function = set_pin_function;
        info = &pin;
    }

    return info;
}

static void mcp4725_set_pin_description (io_port_direction_t dir, uint8_t port, const char *description)
{
    if(dir == Port_Output && port < analog.out.n_ports)
        mcp4725.description = description;
}

static void onEnumeratePins (bool low_level, pin_info_ptr pin_info, void *data)
{
    static xbar_t pin = {};

    on_enumerate_pins(low_level, pin_info, data);

    memcpy(&pin, &mcp4725, sizeof(xbar_t));

    if(!low_level)
        pin.port = "MCP4725:";

    if(!analog.out.n_ports)
        mcp4725.description = "No power";

    pin_info(&pin, data);
}

static void get_next_port (xbar_t *pin, void *fn)
{
    if(pin->group == PinGroup_AuxOutputAnalog)
        *(pin_function_t *)fn = max(*(pin_function_t *)fn, pin->function + 1);
}

void mcp4725_init (void)
{
    if(i2c_start().ok && i2c_probe(MCP4725_ADDRESS)) {

        io_analog_t ports = {
            .ports = &analog,
            .get_pin_info = mcp4725_get_pin_info,
            .analog_out = mcp4725_analog_out,
            .set_pin_description = mcp4725_set_pin_description
        };

        uint8_t data[5] = { 0x06, 0x09 };

        i2c_send(0, &data[0], 1, true); // General command
        i2c_send(0, &data[1], 1, true); // WakeUp DAC

        hal.delay_ms(2, NULL);

        i2c_receive(MCP4725_ADDRESS, data, 5, true);

        a_out = (float)(((data[1] << 8) | (data[2])) >> 4); // Current DAC value set from EEPROM
/*
        uint16_t eeprom = ((data[3] & 0x0F) << 8) | data[4];

        data[0] = 0x60; // Write EEPROM
        data[1] = 0;    // MSB data
        data[2] = 0;    // LSB data
        i2c_send(MCP4725_ADDRESS, data, 3, true);
*/
        hal.enumerate_pins(false, get_next_port, &mcp4725.function);

        analog.out.n_ports = 1;
        ioports_add_analog(&ports);
    }

    on_enumerate_pins = hal.enumerate_pins;
    hal.enumerate_pins = onEnumeratePins;
}

#endif // MCP4725_ENABLE
