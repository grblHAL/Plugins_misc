/*

  mcp23017.c - driver code for MCP23017 I2C expander

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

#if MCP23017_ENABLE

#include <math.h>

#include "grbl/plugins.h"
#include "grbl/protocol.h"
#include "grbl/task.h"

#ifndef MCP23017_ADDRESS
#define MCP23017_ADDRESS (0x40 >> 1)
#endif

#if MCP23017_ENABLE == 1
#define MCP_INPUTS 8
#define MCP_OUTPUTS 8
#define MCP_REG_OFFSET 1
typedef uint8_t mcp_reg_t;
#else
typedef uint16_t mcp_reg_t;
#if MCP23017_ENABLE == 2
#define MCP_INPUTS 0
#define MCP_OUTPUTS 16
#else
#define MCP_INPUTS 16
#define MCP_OUTPUTS 0
#define MCP_REG_OFFSET 0
#endif
#endif

// IOCON.BANK = 0

#define MCP_IODIRA      0x00
#define MCP_IODIRB      0x01
#define MCP_IPOLA       0x02
#define MCP_IPOLB       0x03
#define MCP_GPINTENA    0x04
#define MCP_GPINTENB    0x05
#define MCP_DEFVALA     0x06
#define MCP_DEFVALB     0x07
#define MCP_INTCONA     0x08
#define MCP_INTCONB     0x09
#define IOCON           0x0A
//#define IOCON           0x0B
#define MCP_GPPUA       0x0C
#define MCP_GPPUB       0x0D
#define MCP_INTFA       0x0E
#define MCP_INTFB       0x0F
#define MCP_INTCAPA     0x10
#define MCP_INTCAPB     0x11
#define MCP_GPIOA       0x12
#define MCP_GPIOB       0x13
#define MCP_OLATA       0x14
#define MCP_OLATB       0x15

#pragma pack(push, 1)

typedef union {
    struct {
        uint8_t addr;
        mcp_reg_t value;
    };
#if MCP_INPUTS && MCP_OUTPUTS
    uint8_t data[2];
#else
    uint8_t data[3];
#endif
} mcp_cmd_t;

#pragma pack(pop)

/*

// IOCON.BANK = 1

#define MCP_IODIRA      0x00
#define MCP_IODIRB      0x10
#define MCP_IPOLA       0x01
#define MCP_IPOLB       0x11
#define MCP_GPINTENA    0x02
#define MCP_GPINTENB    0x12
#define MCP_DEFVALA     0x03
#define MCP_DEFVALB     0x13
#define MCP_INTCONA     0x04
#define MCP_INTCONB     0x14
#define IOCON           0x05
//#define IOCON           0x15
#define MCP_GPPUA       0x06
#define MCP_GPPUB       0x16
#define MCP_INTFA       0x07
#define MCP_INTFB       0x17
#define MCP_INTCAPA     0x08
#define MCP_INTCAPB     0x18
#define MCP_GPIOA       0x09
#define MCP_GPIOB       0x19
#define MCP_OLATA       0x0A
#define MCP_OLATB       0x1A

*/

static io_ports_data_t digital = { .external = true };

static enumerate_pins_ptr on_enumerate_pins;

typedef union {
    uint8_t value;
    struct {
        uint8_t unused :1,
                intpol :1,
                odr    :1,
                haen   :1,
                disslw :1,
                seqop  :1,
                mirror :1,
                bank   :1;
    };
} iocon_t;

#if MCP_OUTPUTS

static mcp_reg_t d_out = 0;
static xbar_t aux_out[MCP_OUTPUTS] = {};

static void digital_out_ll (xbar_t *output, float value)
{
    static mcp_reg_t last_out = 0;

    bool on = value != 0.0f;

    if(aux_out[output->id].mode.inverted)
        on = !on;

    if(on)
        d_out |= (1 << output->id);
    else
        d_out &= ~(1 << output->id);

    if(last_out != d_out) {

        mcp_cmd_t cmd;

        cmd.addr = MCP_GPIOA;
        cmd.value = last_out = d_out;

        i2c_send(MCP23017_ADDRESS, cmd.data, sizeof(cmd.data), true);
    }
}

static bool digital_out_cfg (xbar_t *output, gpio_out_config_t *config, bool persistent)
{
    if(output->id < digital.out.n_ports) {

        if(config->inverted != aux_out[output->id].mode.inverted) {
            aux_out[output->id].mode.inverted = config->inverted;
            digital_out_ll(output, (float)(!!(d_out & (1 << output->id))));
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
        value = (float)((d_out & (1 << output->id)) != 0);

    return value;
}

static void get_aux_out_max (xbar_t *pin, void *fn)
{
    if(pin->group == PinGroup_AuxOutput)
        *(pin_function_t *)fn = max(*(pin_function_t *)fn, pin->function + 1);
}

#endif

#if MCP_INPUTS

static xbar_t aux_in[MCP_INPUTS] = {};
static uint32_t d_in = 0;
static struct {
    mcp_reg_t volatile event_bits;
    mcp_reg_t inverted;       // MCP_IPOL
    mcp_reg_t pullup;         // MCP_GPPU
    mcp_reg_t irq_enabled;    // MCP_GPINTEN
    mcp_reg_t irq_change;     // MCP_INTCON
    mcp_reg_t irq_level;      // MCP_DEFVAL
    mcp_reg_t irq_flag;       // MCP_INTF
    mcp_reg_t irq_val;        // MCP_INTCAP
    struct {
        pin_irq_mode_t mode;
        uint8_t user_port;
        ioport_interrupt_callback_ptr callback;
    } irq[MCP_INPUTS];
} mcp = {};

static bool digital_in_cfg (xbar_t *input, gpio_in_config_t *config, bool persistent)
{
    bool ok;

    if((ok = input->id < digital.in.n_ports)) {

        mcp_cmd_t cmd;

        if(!xbar_is_probe_in(input->function)) {

            if(config->inverted)
                mcp.inverted |= (1 << input->id);
            else
                mcp.inverted &= ~(1 << input->id);

            cmd.addr = MCP_IPOLA + MCP_REG_OFFSET;
            cmd.value = mcp.inverted;
            i2c_send(MCP23017_ADDRESS, cmd.data, sizeof(cmd), true);

            aux_in[input->id].mode.inverted = config->inverted;
        }

        if((ok = !(config->pull_mode == PullMode_Down || config->pull_mode == PullMode_UpDown)) && aux_in[input->id].mode.pull_mode != config->pull_mode) {

            if(config->pull_mode)
                mcp.pullup |= (1 << input->id);
            else
                mcp.pullup &= ~(1 << input->id);

            cmd.addr = MCP_GPPUA + MCP_REG_OFFSET;
            cmd.value = mcp.pullup;
            i2c_send(MCP23017_ADDRESS, cmd.data, sizeof(cmd), true);

            aux_in[input->id].mode.pull_mode = config->pull_mode;
        }

        if(persistent && ok)
            ioport_save_input_settings(input, config);
    }

    return ok;
}

static float digital_in_state (xbar_t *input)
{
    if(input->id < digital.in.n_ports) {

        mcp_reg_t gpio;

        i2c_transfer_t t = {
            .address = MCP23017_ADDRESS,
            .cmd = MCP_GPIOA + MCP_REG_OFFSET,
            .cmd_bytes = 1,
            .count = sizeof(mcp_reg_t),
            .data = (uint8_t *)&gpio
        };
        i2c_transfer(&t, true);

        return gpio & (1 << input->id) ? 1.0f : 0.0f;

    } else
        return -1.0f;
}

static pin_irq_mode_t irq_enable (mcp_reg_t mask, pin_irq_mode_t mode)
{
    mcp_cmd_t cmd;

    if(mode == IRQ_Mode_None)
        mcp.irq_enabled &= ~mask;

    else {

        mcp.irq_enabled |= mask;

        switch(mode) {

            case IRQ_Mode_Rising:
            case IRQ_Mode_Falling:
            case IRQ_Mode_Change:
                mcp.irq_change &= ~mask;
                mcp.irq_level &= ~mask;
                break;

            case IRQ_Mode_High:
                mcp.irq_change |= mask;
                mcp.irq_level &= ~mask;
                break;

            case IRQ_Mode_Low:
                mcp.irq_change |= mask;
                mcp.irq_level |= mask;
                break;

            default:
                break;
        }

        if(mcp.irq_change & mask) {
            cmd.addr = MCP_DEFVALA +  + MCP_REG_OFFSET;
            cmd.value = mcp.irq_level;
            i2c_send(MCP23017_ADDRESS, cmd.data, sizeof(cmd), true);
        }

        cmd.addr = MCP_INTCONA + MCP_REG_OFFSET;
        cmd.value = mcp.irq_change;
        i2c_send(MCP23017_ADDRESS, cmd.data, sizeof(cmd), true);
    }

    cmd.addr = MCP_GPINTENA + MCP_REG_OFFSET;
    cmd.value = mcp.irq_enabled;
    i2c_send(MCP23017_ADDRESS, cmd.data, sizeof(cmd), true);

    return mode;
}

static void mcp23017_irq (void *data)
{
    i2c_transfer_t t = {
        .address = MCP23017_ADDRESS,
        .cmd = MCP_INTFA + MCP_REG_OFFSET,
        .cmd_bytes = 1,
        .count = sizeof(mcp_reg_t),
        .data = (uint8_t *)&mcp.irq_flag
    };
    i2c_transfer(&t, true);

    t.cmd = MCP_INTCAPA + MCP_REG_OFFSET;
    t.data = (uint8_t *)&mcp.irq_val;
    i2c_transfer(&t, true);

    mcp_reg_t mask = 1, idx = 0, irq_flag = mcp.irq_flag;

    while(irq_flag) {

        if(mcp.irq_flag & mask) {

            switch(mcp.irq[idx].mode) {

                case IRQ_Mode_Rising:
                    if(!(mcp.irq_val & mask))
                        mcp.irq_flag &= ~mask;
                    break;

                case IRQ_Mode_Falling:
                    if(mcp.irq_val & mask)
                        mcp.irq_flag &= ~mask;
                    break;

                default:
                    break;
            }

            if(mcp.irq[idx].callback)
                mcp.irq[idx].callback(mcp.irq[idx].user_port, !!(mcp.irq_val & (1 << idx)));
        }

        idx++;
        mask <<= 1;
        irq_flag >>= 1;
    }

    mcp.event_bits |= mcp.irq_flag;
}

inline static __attribute__((always_inline)) int32_t get_input (const xbar_t *input, wait_mode_t wait_mode, float timeout)
{
    if(wait_mode == WaitMode_Immediate)
        return (int32_t)digital_in_state((xbar_t *)input);

    int32_t value = -1;
    mcp_reg_t mask = 1 << input->id;
    uint_fast16_t delay = (uint_fast16_t)ceilf((1000.0f / 50.0f) * timeout) + 1;

    if(wait_mode == WaitMode_Rise || wait_mode == WaitMode_Fall) {

        pin_irq_mode_t mode = wait_mode == WaitMode_Rise ? IRQ_Mode_Rising : IRQ_Mode_Falling;

        if(input->cap.irq_mode & mode) {

            mcp.event_bits &= ~mask;
            mcp.irq[input->id].mode = irq_enable(mask, mode);

            do {
                if(mcp.event_bits & mask) {
                    value = !!(mcp.irq_val & mask);
                    break;
                }
                if(delay) {
                    protocol_execute_realtime();
                    hal.delay_ms(min(delay, 50), NULL);
                } else
                    break;
            } while(--delay && !sys.abort);

            mcp.irq[input->id].mode = irq_enable(mask, input->mode.irq_mode); // Restore pin interrupt status
        }

    } else {

        bool wait_for = wait_mode != WaitMode_Low;

        do {
            if((digital_in_state((xbar_t *)input) == 1.0f) == wait_for) {
                value = wait_for;
                break;
            }
            if(delay) {
                protocol_execute_realtime();
                hal.delay_ms(50, NULL);
            } else
                break;
        } while(--delay && !sys.abort);
    }

    return value;
}

static int32_t wait_on_input (uint8_t port, wait_mode_t wait_mode, float timeout)
{
    int32_t value = -1;

    if(port < digital.in.n_ports)
        value = get_input(&aux_in[port], wait_mode, timeout);

    return value;
}

static bool register_interrupt_handler (uint8_t port, uint8_t user_port, pin_irq_mode_t irq_mode, ioport_interrupt_callback_ptr interrupt_callback)
{
    bool ok;

    if((ok = port < digital.in.n_ports && aux_in[port].cap.irq_mode != IRQ_Mode_None)) {

        xbar_t *input = &aux_in[port];

        if((ok = (irq_mode & input->cap.irq_mode) == irq_mode && interrupt_callback != NULL)) {
            mcp.irq[input->id].user_port = user_port;
            mcp.irq[input->id].callback = interrupt_callback;
            mcp.irq[input->id].mode = input->mode.irq_mode = irq_enable(1 << input->id, irq_mode);
        }

        if(irq_mode == IRQ_Mode_None || !ok) {
            mcp.irq[input->id].callback = NULL;
            mcp.irq[input->id].mode = input->mode.irq_mode = irq_enable(1 << input->id, irq_mode);
        }
    }

    return ok;
}

static void get_aux_in_max (xbar_t *pin, void *fn)
{
    if(pin->group == PinGroup_AuxInput)
        *(pin_function_t *)fn = max(*(pin_function_t *)fn, pin->function + 1);
}

ISR_CODE bool ISR_FUNC(mcp23017_irq_handler)(uint_fast8_t id, bool level)
{
    task_add_immediate(mcp23017_irq, NULL);

    return true;
}

#endif

// Common

static bool set_pin_function (xbar_t *port, pin_function_t function)
{
#if MCP_INPUTS && MCP_OUTPUTS
    if(port->mode.input)
        aux_in[port->id].function = function;
    else
        aux_out[port->id].function = function;
#elif MCP_INPUTS
    aux_in[port->id].function = function;
#else
    aux_out[port->id].function = function;
#endif
    return true;
}

static void set_pin_description (io_port_direction_t dir, uint8_t port, const char *description)
{
#if MCP_INPUTS && MCP_OUTPUTS
     if(dir == Port_Input && port < digital.in.n_ports)
        aux_in[port].description = description;
     else if(dir == Port_Output && port < digital.out.n_ports)
        aux_out[port].description = description;
#elif MCP_INPUTS
     if(port < digital.in.n_ports)
        aux_in[port].description = description;
#else
     if(port < digital.out.n_ports)
         aux_out[port].description = description;
#endif
}

static xbar_t *get_pin_info (io_port_direction_t dir, uint8_t port)
{
    static xbar_t pin;

    xbar_t *info = NULL;

#if MCP_INPUTS

    if(dir == Port_Input && port < digital.in.n_ports) {
        memcpy(&pin, &aux_in[port], sizeof(xbar_t));
        pin.pin += digital.in.pin_base;
        pin.get_value = digital_in_state;
        pin.set_function = set_pin_function;
        pin.config = digital_in_cfg;
        info = &pin;
    }

#endif

#if MCP_OUTPUTS

    if(dir == Port_Output && port < digital.out.n_ports) {
        memcpy(&pin, &aux_out[port], sizeof(xbar_t));
        pin.pin += digital.out.pin_base;
        pin.get_value = digital_out_state;
        pin.set_value = digital_out_ll;
        pin.set_function = set_pin_function;
        pin.config = digital_out_cfg;
        info = &pin;
    }

#endif

    return info;
}

static void onEnumeratePins (bool low_level, pin_info_ptr pin_info, void *data)
{
    static xbar_t pin = {};

    on_enumerate_pins(low_level, pin_info, data);

    uint_fast8_t idx;

#if MCP_INPUTS

    for(idx = 0; idx < digital.in.n_ports; idx++) {

        memcpy(&pin, &aux_in[idx], sizeof(xbar_t));

        if(!low_level) {
#if MCP_INPUTS == 16
            if(pin.id >= 8) {
                pin.port = "MCP23017:B";
                pin.pin -= 8;
            } else
                pin.port = "MCP23017:A";
#else
            pin.port = "MCP23017:B";
#endif
        }

        pin_info(&pin, data);
    }

#endif

#if MCP_OUTPUTS

    for(idx = 0; idx < digital.out.n_ports; idx++) {

        memcpy(&pin, &aux_out[idx], sizeof(xbar_t));

        if(!low_level) {
#if MCP_OUTPUTS == 16
            if(pin.id >= 8) {
                pin.port = "MCP23017:B";
                pin.pin -= 8;
            } else
                pin.port = "MCP23017:A";
#else
            pin.port = "MCP23017:A";
#endif
        }

        pin_info(&pin, data);
    };

#endif
}

void mcp23017_init (void)
{
    static bool ok = false;

    if(!ok && i2c_start().ok && i2c_probe(MCP23017_ADDRESS)) {

        uint_fast8_t idx;
        iocon_t iocon = {
            .odr   = 1,
#if MCP23017_ENABLE == 3
            .mirror = 1
#endif
#if MCP23017_ENABLE == 1
            .seqop = 1
#endif
        };
        io_digital_t dports = {
            .ports = &digital,
#if MCP_INPUTS
            .wait_on_input = wait_on_input,
            .register_interrupt_handler = register_interrupt_handler,
#endif
#if MCP_OUTPUTS
            .digital_out = digital_out,
#endif
            .get_pin_info = get_pin_info,
            .set_pin_description = set_pin_description
        };

        uint8_t bcmd[] = { IOCON, iocon.value };
        i2c_send(MCP23017_ADDRESS, bcmd, 2, true);

        mcp_cmd_t cmd;

#if MCP_INPUTS && MCP_OUTPUTS

        // disable interrupts
        cmd.addr = MCP_GPINTENA + MCP_REG_OFFSET;
        cmd.value = 0;
        i2c_send(MCP23017_ADDRESS, cmd.data, sizeof(cmd.data), true);

        // configure GPIO B as inputs
        cmd.addr = MCP_IODIRA + MCP_REG_OFFSET;
        cmd.value = 0xFF;
        i2c_send(MCP23017_ADDRESS, cmd.data, sizeof(cmd.data), true);

        // configure GPIO A as outputs
        cmd.addr = MCP_IODIRA;
        cmd.value = 0x00;
        i2c_send(MCP23017_ADDRESS, cmd.data, sizeof(cmd.data), true);

        // set GPIO A outputs to 0
        cmd.addr = MCP_GPIOA;
        cmd.value = d_out;
        i2c_send(MCP23017_ADDRESS, cmd.data, sizeof(cmd.data), true);

#elif MCP_INPUTS == 16

        // configure GPIO A and B as inputs
        cmd.addr = MCP_IODIRA;
        cmd.value = 0xFFFF;
        i2c_send(MCP23017_ADDRESS, cmd.data, sizeof(cmd.data), true);

        // disable interrupts
        cmd.addr = MCP_GPINTENA;
        cmd.value = 0;
        i2c_send(MCP23017_ADDRESS, cmd.data, sizeof(cmd.data), true);

#else
        // configure GPIO A and B as outputs
        cmd.addr = MCP_IODIRA;
        cmd.value = 0x00; // 0 = output
        i2c_send(MCP23017_ADDRESS, cmd.data, sizeof(cmd.data), true);

        // set GPIO A and B outputs to 0
        cmd.addr = MCP_GPIOA;
        cmd.value = d_out;
        i2c_send(MCP23017_ADDRESS, cmd.data, sizeof(cmd.data), true);

#endif

#if MCP_INPUTS

        pin_function_t aux_in_base = Input_Aux0;
        bool irq_ok = hal.irq_claim(IRQ_I2C_Strobe, 0, mcp23017_irq_handler);

        hal.enumerate_pins(false, get_aux_in_max, &aux_in_base);

        digital.in.n_ports = sizeof(aux_in) / sizeof(xbar_t);

        for(idx = 0; idx < digital.in.n_ports; idx++) {
            aux_in[idx].id = idx;
            aux_in[idx].pin = idx;
            aux_in[idx].port = &d_in;
            aux_in[idx].function = aux_in_base + idx;
            aux_in[idx].group = PinGroup_AuxInput;
            aux_in[idx].cap.input = On;
            aux_in[idx].cap.irq_mode = irq_ok ? IRQ_Mode_Edges : IRQ_Mode_None;
            aux_in[idx].cap.pull_mode = PullMode_Up;
            aux_in[idx].cap.external = On;
            aux_in[idx].cap.claimable = On;
            aux_in[idx].mode.input = On;
        }

        digital_in_state(&aux_in[0]); // dummy read to clear any pending interrupt

#endif

#if MCP_OUTPUTS

        pin_function_t aux_out_base = Output_Aux0;

        hal.enumerate_pins(false, get_aux_out_max, &aux_out_base);

        digital.out.n_ports = sizeof(aux_out) / sizeof(xbar_t);

        for(idx = 0; idx < digital.out.n_ports; idx++) {
            aux_out[idx].id = idx;
            aux_out[idx].pin = idx;
            aux_out[idx].port = &d_out;
            aux_out[idx].function = aux_out_base + idx;
            aux_out[idx].group = PinGroup_AuxOutput;
            aux_out[idx].cap.output = On;
            aux_out[idx].cap.external = On;
            aux_out[idx].cap.claimable = On;
            aux_out[idx].mode.output = On;
        }

#endif

        ok = ioports_add_digital(&dports);

        on_enumerate_pins = hal.enumerate_pins;
        hal.enumerate_pins = onEnumeratePins;
    }
}

#endif // MCP23017_ENABLE
