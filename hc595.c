/*

  hc595.c - driver code for 74HC595 shift register(s) (output only)

  Part of grblHAL

  Copyright (c) 2026 Terje Io

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

#if HC595_ENABLE

#ifndef HC595_CS_PIN
#error "Chip select pin must be defined for 74HC595 shift register(s)"
#endif

#include "grbl/plugins.h"

#define HC595_BITS (HC595_ENABLE << 3)

static union {
    uint32_t value;
    uint8_t n[4];
} reg = {0};

static io_ports_data_t digital;
static xbar_t aux_out[HC595_BITS] = {};
static enumerate_pins_ptr on_enumerate_pins;

static spi_slave_t dev = {
    .cs_pin = HC595_CS_PIN,
#ifdef HC595_CS_PORT
    .cs_port = HC595_CS_PORT,
#endif
#ifdef HC595_FCLK
    .f_clock = HC595_FCLK
#else
    .f_clock = 4000000
#endif
};

static void digital_out_ll (xbar_t *output, float value)
{
    static uint32_t last_out = 0;

    bool on = value != 0.0f;

    if(aux_out[output->id].mode.inverted)
        on = !on;

    if(on)
        reg.value |= (1 << output->pin);
    else
        reg.value &= ~(1 << output->pin);

    if(last_out != reg.value && spi_select(&dev)) {
        last_out = reg.value;
#if HC595_ENABLE == 1
        spi_put_byte(reg.n[0]);
#else
        uint8_t data[HC595_ENABLE], *s = &reg.n[HC595_ENABLE - 1], *d = data;

        *d++ = *s--;
#if HC595_ENABLE > 2
        *d++ = *s--;
#endif
#if HC595_ENABLE > 3
        *d++ = *s--;
#endif
        *d = *s;

        spi_write(data, HC595_ENABLE);
#endif
        spi_deselect(&dev);
    }
}

static bool digital_out_cfg (xbar_t *output, gpio_out_config_t *config, bool persistent)
{
    if(output->id < digital.out.n_ports) {

        if(config->inverted != aux_out[output->id].mode.inverted) {
            aux_out[output->id].mode.inverted = config->inverted;
            digital_out_ll(&aux_out[output->id], (float)(!(reg.value & (1 << output->pin)) ^ config->inverted));
        }

        // Open drain not supported

        if(persistent)
            ioport_save_output_settings(output, config);
    }

    return output->id < digital.out.n_ports;
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
        value = (float)((reg.value & (1 << output->pin)) != 0);

    return value;
}

static bool set_function (xbar_t *output, pin_function_t function)
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
        pin.ports_id = &digital;
        pin.get_value = digital_out_state;
        pin.set_value = digital_out_ll;
        pin.set_function = set_function;
        pin.config = digital_out_cfg;
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

    for(idx = 0; idx < digital.out.n_ports; idx++) {

        memcpy(&pin, &aux_out[idx], sizeof(xbar_t));

        if(!low_level)
            pin.port = "HC595.";

        pin_info(&pin, data);
    };
}

static void get_aux_max (xbar_t *pin, void *fn)
{
    if(pin->group == PinGroup_AuxOutput)
        *(pin_function_t *)fn = max(*(pin_function_t *)fn, pin->function + 1);
}

void hc595_init (void)
{
    uint_fast8_t idx;
    pin_function_t aux_out_base = Output_Aux0;

    io_digital_t dports = {
        .ports = &digital,
        .digital_out = digital_out,
        .get_pin_info = get_pin_info,
        .set_pin_description = set_pin_description,
    };

    if(!spi_start(&dev).ok)
        return;

    hal.enumerate_pins(false, get_aux_max, &aux_out_base);

    digital.out.n_ports = sizeof(aux_out) / sizeof(xbar_t);

    for(idx = 0; idx < digital.out.n_ports; idx++) {
        aux_out[idx].pin = idx;
        aux_out[idx].port = &reg.value;
        aux_out[idx].id = idx;
        aux_out[idx].function = aux_out_base + idx;
        aux_out[idx].group = PinGroup_AuxOutput;
        aux_out[idx].cap.output = On;
        aux_out[idx].cap.external = On;
        aux_out[idx].cap.claimable = On;
        aux_out[idx].mode.output = On;
    }

    if(ioports_add_digital(&dports)) {
        on_enumerate_pins = hal.enumerate_pins;
        hal.enumerate_pins = onEnumeratePins;
        // TODO: set CS pin description
    }
}

#endif // HC595_ENABLE
