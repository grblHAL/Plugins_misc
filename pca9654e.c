/*

  pca9654e.c - driver code for PCA9654E I2C expander (output only)

  Part of grblHAL

  Copyright (c) 2018-2025 Terje Io

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

#if PCA9654E_ENABLE == 1

#include "grbl/plugins.h"

#ifndef PCA9654E_ADDRESS
#define PCA9654E_ADDRESS (0x40 >> 1)
#endif

#define READ_INPUT   0
#define RW_OUTPUT    1
#define RW_INVERSION 2
#define RW_CONFIG    3

static uint8_t pca9654_out;
static io_ports_data_t digital;
static xbar_t aux_out[8] = {};
static enumerate_pins_ptr on_enumerate_pins;

/*
ioexpand_t ioexpand_in (void)
{
    ioexpand_t pins = {0};
    uint8_t cmd[2];

    cmd[0] = READ_INPUT;
    cmd[1] = 0;
    i2c_receive(PCA9654E_ADDRESS, cmd, 1, true);

    return (ioexpand_t)cmd[0];
}
*/

static void digital_out_ll (xbar_t *output, float value)
{
    static uint8_t last_out = 0;

    if(value != 0.0f)
        pca9654_out |= (1 << output->pin);
    else
        pca9654_out &= ~(1 << output->pin);

    if(last_out != pca9654_out) {
 
        uint8_t cmd[2];

        cmd[0] = RW_OUTPUT;
        cmd[1] = pca9654_out;
        last_out = pca9654_out;

        i2c_send(PCA9654E_ADDRESS, cmd, 2, true);
    }
}

static void digital_out (uint8_t port, bool on)
{
    if(port < digital.out.n_ports)
        digital_out_ll(&aux_out[port], (float)on);
}

static float digital_out_state (xbar_t *output)
{
    float value = -1.0f;

    if(output->id < digital.out.n_ports)
        value = (float)((pca9654_out & (1 << output->pin)) != 0);

    return value;
}

static bool set_pin_function (xbar_t *output, pin_function_t function)
{
    if(output->id < digital.out.n_ports)
        aux_out[output->id].function = function;

    return output->id < digital.out.n_ports;
}

static xbar_t *get_pin_info (io_port_direction_t dir, uint8_t port)
{
    static xbar_t pin;

    xbar_t *info = NULL;

    if(dir == Port_Output && port < digital.out.n_ports) {
        memcpy(&pin, &aux_out[port], sizeof(xbar_t));
        pin.get_value = digital_out_state;
        pin.set_value = digital_out_ll;
        pin.set_function = set_pin_function;
        info = &pin;
    }

    return info;
}

static void set_pin_description (io_port_direction_t dir, uint8_t port, const char *description)
{
    if(dir == Port_Output && port < digital.out.n_ports)
        aux_out[port].description = description;
}

static void onEnumeratePins (bool low_level, pin_info_ptr pin_info, void *data)
{
    static xbar_t pin = {};

    on_enumerate_pins(low_level, pin_info, data);

    uint_fast8_t idx;

    for(idx = 0; idx < digital.out.n_ports; idx ++) {

        memcpy(&pin, &aux_out[idx], sizeof(xbar_t));

        if(!low_level)
            pin.port = "PCA9654E:";

        pin_info(&pin, data);
    };
}

static void get_aux_max (xbar_t *pin, void *fn)
{
    if(pin->group == PinGroup_AuxOutput)
        *(pin_function_t *)fn = max(*(pin_function_t *)fn, pin->function + 1);
}

void pca9654e_init (void)
{
    if(i2c_start().ok && i2c_probe(PCA9654E_ADDRESS)) {

        uint_fast8_t idx;
        uint8_t cmd[2];
        pin_function_t aux_out_base = Output_Aux0;

        io_digital_t dports = {
            .ports = &digital,
            .digital_out = digital_out,
            .get_pin_info = get_pin_info,
            .set_pin_description = set_pin_description,
        };

        cmd[0] = RW_CONFIG;
        cmd[1] = 0;    // 0 = output, 1 = input
        i2c_send(PCA9654E_ADDRESS, cmd, 2, true);

        cmd[0] = RW_INVERSION;
        cmd[1] = 0; // cfg
        i2c_send(PCA9654E_ADDRESS, cmd, 2, true);

        cmd[0] = RW_OUTPUT;
        cmd[1] = 0;
        i2c_send(PCA9654E_ADDRESS, cmd, 2, true);

        hal.enumerate_pins(false, get_aux_max, &aux_out_base);

        digital.out.n_ports = sizeof(aux_out) / sizeof(xbar_t);

        for(idx = 0; idx < digital.out.n_ports; idx ++) {
            aux_out[idx].id = idx;
            aux_out[idx].pin = idx;
            aux_out[idx].port = &pca9654_out;
            aux_out[idx].function = aux_out_base + idx;
            aux_out[idx].group = PinGroup_AuxOutput;
            aux_out[idx].cap.output = On;
            aux_out[idx].cap.external = On;
            aux_out[idx].cap.claimable = On;
            aux_out[idx].mode.output = On;
        }

        ioports_add_digital(&dports);

        on_enumerate_pins = hal.enumerate_pins;
        hal.enumerate_pins = onEnumeratePins;
    }
}

#endif // PCA9654E_ENABLE
