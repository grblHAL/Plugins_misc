/*

  fnc_expander.c - driver code for Airedale FluidNC I/O Expander

  *** EXPERIMENTAL ***

  Part of grblHAL

  Copyright (c) 2025-2026 Terje Io

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

// http://wiki.fluidnc.com/en/hardware/official/airedale

#include "driver.h"

#if FNC_EXPANDER_ENABLE

#include <stdio.h>
#include <math.h>

#include "grbl/task.h"
#include "grbl/protocol.h"
#include "grbl/utf8.h"

#define FNC_ACK     0xB2  	// Command accepted character
#define FNC_NAK     0xB3  	// Command rejected character
#define FNC_RST     0xB4  	// Expander restarted character
#define FNC_LOW     0xC4  	// Start of two-character sequence; second is port number
#define FNC_HIGH    0xC5  	// Start of two-character sequence; second is port number
#define FNC_PINBASE 0x80  	// Port number offset in second character
#define FNC_PWMBASE 0x10000	// Offset base for PWM ports

#ifndef FNC_N_AOUT
#define FNC_N_AOUT 4
#endif
#ifndef FNC_N_DIN
#define FNC_N_DIN 8
#endif
#ifndef FNC_N_DOUT
#define FNC_N_DOUT (10 - FNC_N_AOUT)
#endif

#ifndef FNC_STREAM
#ifdef MPG_STREAM
#define FNC_STREAM MPG_STREAM
#else
#define FNC_STREAM 255
#endif
#endif

#ifndef FNC_BAUD
#define FNC_BAUD 1000000
//#define FNC_BAUD 115200
#endif

static struct {
    pin_irq_mode_t mode;
    ioport_interrupt_callback_ptr callback;
} irq[FNC_N_DIN] = {};
static struct {
    stream_write_ptr write;
    stream_write_char_ptr write_char;
    stream_write_n_ptr write_n;
} expander = {};
static xbar_t aux_in[FNC_N_DIN] = {};
static xbar_t aux_out[FNC_N_DOUT] = {};
static io_ports_data_t digital;
static uint32_t d_out = 0, d_in = 0;
static bool reset_pending = false;
static volatile uint32_t event_bits = 0;
static char expander_id[15];
static uint8_t ledcmd[] = { FNC_LOW, FNC_PINBASE + 18, FNC_LOW, FNC_PINBASE + 19, FNC_LOW, FNC_PINBASE + 20 };
#if FNC_N_AOUT
static uint32_t aout_pin_base;
static xbar_t aux_aout[FNC_N_AOUT] = {};
static struct {
	float value;
	float min_value;
	float max_value;
} pwm[FNC_N_AOUT] = {};
static io_ports_data_t analog;
#endif

static driver_reset_ptr driver_reset;
static enumerate_pins_ptr on_enumerate_pins;
static on_report_options_ptr on_report_options;

#if MPG_ENABLE && FNC_STREAM == MPG_STREAM

static on_mpg_registered_ptr on_mpg_registered;
static set_enqueue_rt_handler_ptr set_enqueue_rt_handler;
static enqueue_realtime_command_ptr org_handler;

static bool fnc_response (uint8_t c);

static bool disableRX (bool disable)
{
    return true;
}

static enqueue_realtime_command_ptr setRtHandler (enqueue_realtime_command_ptr handler)
{
//    enqueue_realtime_command_ptr fnc_handler = set_enqueue_rt_handler(NULL);

    set_enqueue_rt_handler(handler);
    set_enqueue_rt_handler(fnc_response);

    return (org_handler = handler);
}

static void onMpgRegistered (io_stream_t *stream, bool rx_only)
{
    org_handler = stream->set_enqueue_rt_handler(fnc_response);

    set_enqueue_rt_handler = stream->set_enqueue_rt_handler;
    stream->set_enqueue_rt_handler = setRtHandler;

    stream->disable_rx(false);
    stream->set_baud_rate(FNC_BAUD);

    stream->disable_rx = disableRX;
    expander.write = stream->write;
    expander.write_char = stream->write_char;
    expander.write_n = stream->write_n;

    stream_set_description(stream, "MPG + FNC Expander");
}

#endif // MPG_ENABLE

static void led_out_masked (uint16_t device, rgb_color_t color, rgb_color_mask_t mask)
{
    if(mask.R)
        ledcmd[0] = color.R ? FNC_HIGH : FNC_LOW;
    if(mask.G)
        ledcmd[2] = color.G ? FNC_HIGH : FNC_LOW;
    if(mask.B)
        ledcmd[4] = color.B ? FNC_HIGH : FNC_LOW;

    expander.write_n(ledcmd, 6);
}

static void led_out (uint16_t device, rgb_color_t color)
{
    ledcmd[0] = color.R ? FNC_HIGH : FNC_LOW;
    ledcmd[2] = color.G ? FNC_HIGH : FNC_LOW;
    ledcmd[4] = color.B ? FNC_HIGH : FNC_LOW;

    expander.write_n(ledcmd, 6);
}

static void digital_out_ll (xbar_t *output, float value)
{
    static uint32_t last_out = 0;

    bool on = value != 0.0f;

    if(aux_out[output->id].mode.inverted)
        on = !on;

    if(on)
        *(uint32_t *)output->port |= (1 << output->id);
    else
        *(uint32_t *)output->port &= ~(1 << output->id);

    if(last_out != *(uint32_t *)output->port) {
        last_out = *(uint32_t *)output->port;
        expander.write_char(on ? FNC_HIGH : FNC_LOW);
        expander.write_char(FNC_PINBASE + output->id + 8);
    }
}

static bool digital_out_cfg (xbar_t *output, gpio_out_config_t *config, bool persistent)
{
    if(output->id == 1) {

        if(config->inverted != aux_out[output->id].mode.inverted) {
            aux_out[output->id].mode.inverted = config->inverted;
            digital_out_ll(output, (float)(!(*(uint32_t *)output->port & (1 << output->id)) ^ config->inverted));
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
        value = (float)((*(uint32_t *)output->port & (1 << output->id)) != 0);

    return value;
}

static bool digital_in_cfg (xbar_t *input, gpio_in_config_t *config, bool persistent)
{
    if(input->id < digital.in.n_ports && config->pull_mode != PullMode_UpDown) {

        if(!xbar_is_probe_in(input->function))
            aux_in[input->id].mode.inverted = config->inverted;

        if(aux_in[input->id].mode.pull_mode != config->pull_mode) {

            char buf[40];

            aux_in[input->id].mode.pull_mode = config->pull_mode;
            sprintf(buf, "[EXP:io.%d=in,high,%s]\n", input->id, config->pull_mode == PullMode_Down ? "pd" : "pu");
            expander.write(buf);
        }

        // input->mode.debounce = config->debounce; default on, cannot be disabled?

        if(persistent)
            ioport_save_input_settings(input, config);
    }

    return input->id < digital.in.n_ports;
}

static float digital_in_state (xbar_t *input)
{
    float value = -1.0f;

    if(input->id < digital.in.n_ports)
        value = (float)(((*(uint32_t *)input->port & (1 << input->id)) != 0) ^ aux_in[input->id].mode.inverted);

    return value;
}

inline static __attribute__((always_inline)) int32_t get_input (const xbar_t *input, wait_mode_t wait_mode, float timeout)
{
    if(wait_mode == WaitMode_Immediate)
        return !!(*(uint32_t *)input->port & (1 << input->id)) ^ input->mode.inverted;

    int32_t value = -1;
    uint32_t mask = 1 << input->id;
    uint_fast16_t delay = (uint_fast16_t)ceilf((1000.0f / 50.0f) * timeout) + 1;

    if(wait_mode == WaitMode_Rise || wait_mode == WaitMode_Fall) {

        pin_irq_mode_t mode = wait_mode == WaitMode_Rise ? IRQ_Mode_Rising : IRQ_Mode_Falling;

        if(input->cap.irq_mode & mode) {

            event_bits &= ~mask;
            irq[input->id].mode = mode;

            do {
                if(event_bits & mask) {
                    value = !!(*(uint32_t *)input->port & mask) ^ input->mode.inverted;
                    break;
                }
                if(delay) {
                    protocol_execute_realtime();
                    hal.delay_ms(50, NULL);
                } else
                    break;
            } while(--delay && !sys.abort);

            irq[input->id].mode = input->mode.irq_mode;    // Restore pin interrupt status
        }

    } else {

        bool wait_for = wait_mode != WaitMode_Low;

        do {
            if((!!(*(uint32_t *)input->port & mask) ^ input->mode.inverted) == wait_for) {
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
            irq[input->id].callback = interrupt_callback;
            irq[input->id].mode = input->mode.irq_mode = irq_mode;
        }

        if(irq_mode == IRQ_Mode_None || !ok) {
            irq[input->id].callback = NULL;
            irq[input->id].mode = input->mode.irq_mode = IRQ_Mode_None;
        }
    }

    return ok;
}

static bool set_pin_function (xbar_t *port, pin_function_t function)
{
    if(port->mode.input)
        aux_in[port->id].function = function;
    else
        aux_out[port->id].function = function;

    return true;
}

static void set_pin_description (io_port_direction_t dir, uint8_t port, const char *description)
{
    if(dir == Port_Input && port < digital.in.n_ports)
        aux_in[port].description = description;
    else if(dir == Port_Output && port < digital.out.n_ports)
        aux_out[port].description = description;
}

static xbar_t *get_pin_info (io_port_direction_t dir, uint8_t port)
{
    static xbar_t pin;

    xbar_t *info = NULL;

    if(dir == Port_Input && port < digital.in.n_ports) {
        memcpy(&pin, &aux_in[port], sizeof(xbar_t));
        pin.pin += digital.in.n_start;
        pin.get_value = digital_in_state;
        pin.set_function = set_pin_function;
        pin.config = digital_in_cfg;
        info = &pin;
    } else if(dir == Port_Output && port < digital.out.n_ports) {
        memcpy(&pin, &aux_out[port], sizeof(xbar_t));
        pin.pin += digital.out.n_start;
        pin.get_value = digital_out_state;
        pin.set_value = digital_out_ll;
        pin.set_function = set_pin_function;
        pin.config = digital_out_cfg;
        info = &pin;
    }

    return info;
}

static ISR_CODE bool ISR_FUNC(fnc_response)(uint8_t c)
{
    static char prefix = 0;

    if(c == FNC_RST) {
    	reset_pending = true;
    	system_raise_alarm(Alarm_ExpanderException);
    	return true;
    }

    bool claimed = false;

    if(c == FNC_LOW || c == FNC_HIGH) {
        prefix = c;
        claimed = true;
    } else if(prefix && (claimed = c >= FNC_PINBASE && c - FNC_PINBASE < digital.in.n_ports)) {

        xbar_t *input = &aux_in[c - FNC_PINBASE];

        if(input->port) {

            uint32_t event = 0, bit = 1 << input->id;

            switch(irq[input->id].mode) {

                case IRQ_Mode_Rising:
                    if(prefix == FNC_HIGH && !(*(uint32_t *)input->port & bit))
                        event |= bit;
                    break;

                case IRQ_Mode_Falling:
                    if(prefix == FNC_LOW && (*(uint32_t *)input->port & bit))
                        event |= bit;
                    break;

                case IRQ_Mode_Change:
                    if((prefix == FNC_HIGH) != !!(*(uint32_t *)input->port & bit))
                        event |= bit;
                    break;

                default: break;
            }

            if(prefix == FNC_HIGH)
                *(uint32_t *)input->port |= bit;
            else
                *(uint32_t *)input->port &= ~bit;

            if((event & bit) && irq[input->id].callback)
                irq[input->id].callback(digital.in.n_start + input->id, !!(*(uint32_t *)input->port & bit));

            event_bits |= event;
        }
    } else
        prefix = 0;

#if MPG_ENABLE && FNC_STREAM == MPG_STREAM
    return claimed || org_handler(c);
#else
    return true;
#endif
}

static void get_aux_out_max (xbar_t *pin, void *fn)
{
    if(pin->group == PinGroup_AuxOutput)
        *(pin_function_t *)fn = max(*(pin_function_t *)fn, pin->function + 1);
}

static void get_aux_in_max (xbar_t *pin, void *fn)
{
    if(pin->group == PinGroup_AuxInput)
        *(pin_function_t *)fn = max(*(pin_function_t *)fn, pin->function + 1);
}

#if FNC_N_AOUT

static float pwm_get_value (xbar_t *output)
{
    return output->id < analog.out.n_ports ? pwm[output->id].value : -1.0f;
}

static void pwm_out_ll (xbar_t *output, float value)
{
	uint8_t buf[4];

	pwm[output->id].value = constrain(value, pwm[output->id].min_value, pwm[output->id].max_value);

	uint16_t len = utf32_to_utf8(buf, FNC_PWMBASE | ((output->id + aout_pin_base) << 10) | (uint32_t)(pwm[output->id].value * 10.0f));
	expander.write_n(buf, len);
}

static bool pwm_out (uint8_t port, float value)
{
    if(port < analog.out.n_ports)
    	pwm_out_ll(&aux_aout[port], value);

    return port < analog.out.n_ports;
}

static bool init_pwm (xbar_t *output, pwm_config_t *config, bool persistent)
{
    char buf[40];

    sprintf(buf, "[EXP:io.%d=pwm,frequency=%ld]\n", output->id + aout_pin_base, (uint32_t)config->freq_hz);
    expander.write(buf);

    pwm[output->id].min_value = config->min_value;
    pwm[output->id].max_value = config->max_value;

    aux_aout[output->id].mode.pwm = !config->servo_mode;
    aux_aout[output->id].mode.servo_pwm = config->servo_mode;

    pwm_out(output->id, config->min_value);

    return true;
}

static bool analog_set_function (xbar_t *port, pin_function_t function)
{
    if(port->mode.output)
    	aux_aout[port->id].id = function;

    return true;
}

static xbar_t *analog_get_pin_info (io_port_direction_t dir, uint8_t port)
{
    static xbar_t pin;

    xbar_t *info = NULL;

    if(dir == Port_Output && port < analog.out.n_ports) {
        memcpy(&pin, &aux_aout[port], sizeof(xbar_t));
		pin.config = init_pwm;
		pin.get_value = pwm_get_value;
	    pin.set_value = pwm_out_ll;
	    pin.set_function = analog_set_function;
		info = &pin;
    }

    return info;
}

static void analog_set_pin_description (io_port_direction_t dir, uint8_t port, const char *description)
{
    if(dir == Port_Output && port < analog.out.n_ports)
    	aux_aout[port].description = description;
}

static void get_aux_aout_max (xbar_t *pin, void *fn)
{
    if(pin->group == PinGroup_AuxOutputAnalog)
        *(pin_function_t *)fn = max(*(pin_function_t *)fn, pin->function + 1);
}

#endif // FNC_N_AOUT

static void fnc_config (void *data)
{
    if(expander.write) {

        char buf[40];
        uint_fast8_t idx;

        reset_pending = false;

        for(idx = 0; idx < digital.in.n_ports; idx++) {
            if(aux_in[idx].port) {
                sprintf(buf, "[EXP:io.%d=in,high,pu]\n", aux_in[idx].id);
                expander.write(buf);
            }
        }

        for(idx = 0; idx < digital.out.n_ports; idx++) {
            if(aux_out[idx].port) {
                sprintf(buf, "[EXP:io.%d=out]\n", aux_out[idx].id + 8);
                expander.write(buf);
            }
        }

#if FNC_N_AOUT

        xbar_t *pin;
        pwm_config_t config = {
            .freq_hz = 5000.0f,
            .min = 0.0f,
            .max = 100.0f,
            .off_value = 0.0f,
            .min_value = 0.0f,
            .max_value = 100.0f,
            .invert = Off
        };

        for(idx = 0; idx < analog.out.n_ports; idx++) {
            if((pin = analog_get_pin_info(Port_Output, idx)))
                pin->config(pin, &config, false);
        }

#endif // FNC_N_AOUT
    }
}

static void driverReset (void)
{
    driver_reset();

    if(reset_pending)
    	task_add_immediate(fnc_config, NULL);
}

static bool fnc_init (const io_stream_t *stream)
{
    if(stream == NULL || stream->write == NULL)
        return false;

    int16_t c;
    volatile uint32_t t1, t2;
    char buf[50], *p = buf;

    stream->set_enqueue_rt_handler(stream_buffer_all);
    stream->write("\n[MSG:RST]\n[EXP:ID]\n");

    while(stream->get_tx_buffer_count());

    t1 = hal.get_elapsed_ticks() + 1;
    do {
        if((c = stream->read()) != -1) {
            if(c == FNC_NAK) //??
                t1 = 0;
            else if(p) {
                if(c == '\n') {
                    *p = '\0';
                    p = NULL;
                    t1=0;
                } else
                    *p++ = c;
            }
        }
        if((t2 = hal.get_elapsed_ticks()) > t1 && t2 - t1 > 5) // something is resetting the systick timer...
            break;
    } while(t1 != 0);

    if(t1 == 0 && (t1 = strncmp(buf, "(EXP,BOARD:Airedale", 19)) == 0) {

        if((p = strchr(buf + 19, ',')))
            *p = '\0';
        strcat(strcpy(expander_id, "FNC_Airedale "), strchr(buf, ' ') + 2);

#if MPG_ENABLE && FNC_STREAM == MPG_STREAM
        stream_close(stream);
#else

        driver_reset = hal.driver_reset;
        hal.driver_reset = driverReset;

        expander.write = stream->write;
        expander.write_char = stream->write_char;
        expander.write_n = stream->write_n;

        stream->set_enqueue_rt_handler(fnc_response);
#endif
    } else
        stream_close(stream);

    return t1 == 0;
}

static void onEnumeratePins (bool low_level, pin_info_ptr pin_info, void *data)
{
    static xbar_t pin = {};

    on_enumerate_pins(low_level, pin_info, data);

    uint_fast8_t idx;

    for(idx = 0; idx < digital.in.n_ports; idx ++) {

        memcpy(&pin, &aux_in[idx], sizeof(xbar_t));

        if(!low_level)
            pin.port = "FNC:";

        pin_info(&pin, data);
    }

    for(idx = 0; idx < digital.out.n_ports; idx ++) {

        memcpy(&pin, &aux_out[idx], sizeof(xbar_t));

        pin.pin += 8;

        if(!low_level)
            pin.port = "FNC:";

        pin_info(&pin, data);
    }

#if FNC_N_AOUT
    for(idx = 0; idx < analog.out.n_ports; idx ++) {

        memcpy(&pin, &aux_aout[idx], sizeof(xbar_t));

        if(!low_level)
            pin.port = "FNC:";

        pin.pin += aout_pin_base;

        pin_info(&pin, data);
    }
#endif
}

static void onReportOptions (bool newopt)
{
    on_report_options(newopt);

    if(!newopt)
        report_plugin(expander.write
                       ? expander_id
                       : "FNC Expander (N/A)", "0.02");
}

void fnc_expander_init (void)
{
    uint_fast8_t idx;
    pin_function_t aux_in_base = Input_Aux0, aux_out_base = Output_Aux0;

    io_digital_t dports = {
        .ports = &digital,
        .digital_out = digital_out,
        .get_pin_info = get_pin_info,
        .wait_on_input = wait_on_input,
        .set_pin_description = set_pin_description,
        .register_interrupt_handler = register_interrupt_handler
    };

    on_report_options = grbl.on_report_options;
    grbl.on_report_options = onReportOptions;

    if(fnc_init(stream_open_instance(FNC_STREAM, FNC_BAUD, NULL, "FNC Expander"))) {

        hal.enumerate_pins(false, get_aux_in_max, &aux_in_base);
        hal.enumerate_pins(false, get_aux_out_max, &aux_out_base);

        digital.in.n_ports = max(FNC_N_DIN, N_AUX_DIN_MAX - aux_in_base);

        for(idx = 0; idx < digital.in.n_ports; idx++) {
            aux_in[idx].id = idx;
            aux_in[idx].pin = idx;
            aux_in[idx].port = &d_in;
            aux_in[idx].function = aux_in_base + idx;
            aux_in[idx].group = PinGroup_AuxInput;
            aux_in[idx].cap.input = On;
            aux_in[idx].cap.irq_mode = IRQ_Mode_Edges;
            aux_in[idx].cap.pull_mode = PullMode_UpDown;
            aux_in[idx].cap.external = On;
            aux_in[idx].cap.claimable = On;
            aux_in[idx].mode.input = On;
        }

        digital.out.n_ports = max(FNC_N_DOUT, N_AUX_DOUT_MAX - aux_out_base);

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

        ioports_add_digital(&dports);

#if FNC_N_AOUT

        io_analog_t aports = {
            .ports = &analog,
            .analog_out = pwm_out,
            .get_pin_info = analog_get_pin_info,
            .set_pin_description = analog_set_pin_description
        };

        pin_function_t aux_aout_base = Output_Analog_Aux0;
        hal.enumerate_pins(false, get_aux_aout_max, &aux_out_base);

        aout_pin_base = digital.out.n_ports + 8;
        analog.out.n_ports = max(FNC_N_AOUT, N_AUX_AOUT_MAX - aux_aout_base);

        for(idx = 0; idx < analog.out.n_ports; idx++) {
            aux_aout[idx].id = idx;
            aux_aout[idx].pin = idx;
            aux_aout[idx].port = &d_out;
            aux_aout[idx].function = aux_aout_base + idx;
            aux_aout[idx].group = PinGroup_AuxOutputAnalog;
            aux_aout[idx].cap.output = On;
            aux_aout[idx].cap.pwm = On;
            aux_aout[idx].cap.external = On;
            aux_aout[idx].cap.claimable = On;
            aux_aout[idx].mode.output = On;
        }

        ioports_add_analog(&aports);

#endif // FNC_N_AOUT

        on_enumerate_pins = hal.enumerate_pins;
        hal.enumerate_pins = onEnumeratePins;

        if(hal.rgb0.out == NULL) {
            hal.rgb0.out = led_out;
            hal.rgb0.out_masked = led_out_masked;
            hal.rgb0.num_devices = 1;
            hal.rgb0.cap = (rgb_color_t){ .R = 1, .G = 1, .B = 1 };
        }

#if MPG_ENABLE && FNC_STREAM == MPG_STREAM
        on_mpg_registered = grbl.on_mpg_registered;
        grbl.on_mpg_registered = onMpgRegistered;
#endif
        task_run_on_startup(fnc_config, NULL);
    }
}

#endif // FNC_EXPANDER_ENABLE
