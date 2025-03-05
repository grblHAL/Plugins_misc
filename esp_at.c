/*

  esp_at.c - ESP-AT module interface plugin for "raw" Telnet streaming

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

#if ESP_AT_ENABLE == 1

#include <stdio.h>
#include <string.h>

#include "grbl/hal.h"
#include "grbl/task.h"
#include "grbl/protocol.h"
#include "grbl/state_machine.h"
#include "grbl/nvs_buffer.h"

#ifndef COPROC_STREAM
#define COPROC_STREAM 255 // Claim first free stream
#endif

typedef struct {
    uint8_t boot0;
    uint8_t reset;
} at_ports_t;

typedef struct {
    ssid_t ssid;
    password_t password;
    hostname_t hostname;
    ip_mode_t ip_mode;
    char ip[16];
    char gateway[16];
    char mask[16];
} esp_at_wifi_settings_t;

typedef struct {
    grbl_wifi_mode_t mode;
    uint16_t telnet_port;
    esp_at_wifi_settings_t ap;
    esp_at_wifi_settings_t sta;
    uint8_t ap_channel;
    char ap_country[3];
} esp_at_settings_t;

static uint32_t timeout;
static bool esp_at_running;
static char ip[16];
static char gateway[16];
static char netmask[16];
static char mac[18];
static char buf[130];
static on_report_options_ptr on_report_options;
static nvs_address_t nvs_address;
static io_stream_t at_cmd_stream;
static esp_at_settings_t esp_at_settings;
static const io_stream_t *session_stream;

static void await_connect (void *data);

#if ETHERNET_ENABLE

#include "networking/networking.h"

static char const if_name[] = "at0";
static network_flags_t network_status = {};

static networking_get_info get_info = NULL;

static network_info_t *getInfo (const char *interface)
{
    static network_info_t info = { .interface = if_name };

    if(interface == info.interface) {

        strcpy(info.mac, mac);
        strcpy(info.status.ip, ip);

        if(info.status.ip_mode == IpMode_DHCP) {
            *info.status.gateway = '\0';
            *info.status.mask = '\0';
        }

        info.status.services = (network_services_t){ .telnet = On };

        return &info;
    }

    return get_info ? get_info(interface) : NULL;
}

static void status_event_out (void *data)
{
    networking.event(if_name, (network_status_t){ .value = (uint32_t)data });
}

static void status_event_publish (network_flags_t changed)
{
    task_add_immediate(status_event_out, (void *)((network_status_t){ .changed = changed, .flags = network_status }).value);
}

#endif // ETHERNET_ENABLE

/////////////////////////

static stream_rx_buffer_t rxbuf = {0};
static enqueue_realtime_command_ptr enqueue_realtime_command = protocol_enqueue_realtime_command;

//
// Returns number of free characters in the input buffer
//
static uint16_t atStreamRxFree (void)
{
    uint16_t tail = rxbuf.tail, head = rxbuf.head;

    return RX_BUFFER_SIZE - BUFCOUNT(head, tail, RX_BUFFER_SIZE);
}

//
// Flushes the input buffer
//
static void atStreamRxFlush (void)
{
    rxbuf.tail = rxbuf.head;
}

//
// Flushes and adds a CAN character to the input buffer
//
static void atStreamRxCancel (void)
{
    rxbuf.data[rxbuf.head] = ASCII_CAN;
    rxbuf.tail = rxbuf.head;
    rxbuf.head = BUFNEXT(rxbuf.head, rxbuf);
}

//
// Returns number of characters pending transmission
//
static uint16_t atStreamTxCount (void)
{
    return at_cmd_stream.get_tx_buffer_count();
}

//
// Writes a character to the output stream, blocks if buffer full
//
static bool atStreamPutC (const char c)
{
    return at_cmd_stream.write_char(c);
}

//
// Writes a null terminated string to the output stream
//
static void atStreamWriteS (const char *s)
{
    char c, *ptr = (char *)s;

    while((c = *ptr++) != '\0')
        atStreamPutC(c);
}

//
// Writes a number of characters from string to the output stream followed by EOL
//
static void atStreamWrite (const char *s, uint16_t length)
{
    char *ptr = (char *)s;

    while(length--)
        atStreamPutC(*ptr++);
}

//
// atStreamGetC - returns -1 if no data available
//
static int16_t atStreamGetC (void)
{
    uint_fast16_t tail = rxbuf.tail;    // Get buffer pointer

    if(tail == rxbuf.head)
        return -1; // no data available

    char data = rxbuf.data[tail];       // Get next character
    rxbuf.tail = BUFNEXT(tail, rxbuf);  // and update pointer

    return (int16_t)data;
}

static bool atStreamSuspendInput (bool suspend)
{
    return stream_rx_suspend(&rxbuf, suspend);
}

static bool atStreamEnqueueRtCommand (char c)
{
    return enqueue_realtime_command(c);
}

static enqueue_realtime_command_ptr atStreamSetRtHandler (enqueue_realtime_command_ptr handler)
{
    enqueue_realtime_command_ptr prev = enqueue_realtime_command;

    if(handler)
        enqueue_realtime_command = handler;

    return prev;
}

static void atStream_rx_insert (char c)
{
    if(!enqueue_realtime_command(c)) {                          // Check and strip realtime commands...

        uint_fast16_t next_head = BUFNEXT(rxbuf.head, rxbuf);   // Get and increment buffer pointer
        if(next_head == rxbuf.tail)                             // If buffer full
            rxbuf.overflow = 1;                                 // flag overflow
        else {
            rxbuf.data[rxbuf.head] = c;                         // if not add data to buffer
            rxbuf.head = next_head;                             // and update pointer
        }
    }
}

/////////////////////////

static bool is_done (char *s, bool *status)
{
    bool ok = strcmp(s, "OK") == 0;

    if(status)
        *status = ok;

    return ok || strncmp(s, "ERROR", 5) == 0;
}

static bool send_command (char *command)
{
    int16_t c;
    char *s = buf;

    *buf = '\0';

    debug_printf("%s", command);

    at_cmd_stream.reset_read_buffer();
    at_cmd_stream.write(command);
    at_cmd_stream.write(ASCII_EOL);

    timeout = hal.get_elapsed_ticks() + 1000;

    while(hal.get_elapsed_ticks() <= timeout) {

        if((c = at_cmd_stream.read()) != SERIAL_NO_DATA) {

            if(s - buf >= sizeof(buf) - 1)
                return false;

            if(c == ASCII_LF) {

                *s = '\0';
                if(*buf >= ' ') {
                    debug_printf("%s", buf);
                    break;
                }
                else {
                    *buf = '\0';
                    s = buf;
                }
            } else if(c != ASCII_CR)
                *s++ = (char)c;
        }
    }

    return !strcmp(buf, "OK");
}

static char *get_reply (char *command)
{
    int16_t c;
    char *s = buf;

    *buf = '\0';

    if(command) {

        debug_printf("%s", command);

        at_cmd_stream.reset_read_buffer();
        at_cmd_stream.write(command);
        at_cmd_stream.write(ASCII_EOL);
    }

    uint32_t timeout = hal.get_elapsed_ticks() + 5000;

    while(hal.get_elapsed_ticks() <= timeout) {

        if((c = at_cmd_stream.read()) != SERIAL_NO_DATA) {

            if(c == ASCII_LF) {

                *s = '\0';
                if(*buf >= ' ') {
                    debug_printf("%s", buf);
                    break;
                }
                else {
                    *buf = '\0';
                    s=buf;
                }
            } else
            if(c != ASCII_CR)
                *s++ = (char)c;
        }
    }

    return *buf ? buf : NULL;
}

static void close_session (void *data)
{
    if(session_stream) {
        stream_disconnect(session_stream);
        session_stream = NULL;
    }

    at_cmd_stream.set_enqueue_rt_handler(stream_buffer_all);

    hal.delay_ms(20, NULL);
    at_cmd_stream.write("+++");
    hal.delay_ms(1000, NULL);

    if(send_command("AT+CIPMODE=0")) {

        if(esp_at_settings.mode == WiFiMode_AP)
            send_command("AT+CWQIF"); // disconnect client

        task_add_delayed(await_connect, NULL, 100);

        if(data)
            *((bool *)data) = true;
    }

#if ETHERNET_ENABLE
    network_status.ip_aquired = Off;
    status_event_publish((network_flags_t){ .ip_aquired = On });
#endif

    at_cmd_stream.reset_read_buffer();
}

static ISR_CODE bool ISR_FUNC(esp_at_receive)(char c)
{
    static const char *cmds[] =
    {
        "",
        "CLOSED" ASCII_EOL,
        "+STA_DISCONNECTED:",
        "WIFI DISCONNECT" ASCII_EOL
    };
    static const char *s = NULL;
    static uint32_t cmd = 0;

    if(s == NULL && (c == 'C' || c == '+' || c == 'W')) {
        cmd = c == 'C' ? 1 : (c == '+' ? 2 : 3);
        s = cmds[cmd];
        return true;
    } else if(cmd) {

        if(*s != '\0') {
            if(c != *(++s)) {
                if(*s) {
                    const char *s2 = cmds[cmd];
                    do {
                        atStream_rx_insert(*s2++);
                    } while(s2 != s);
                    atStream_rx_insert(c);
                    cmd = 0;
                }
            }
        }

        if(c == ASCII_LF) {
            task_add_immediate(close_session, NULL);
            cmd = 0;
            s = NULL;
        }

        return true;
    }

    atStream_rx_insert(c);

    if(c == ASCII_CR || c == ASCII_LF) {
        cmd = 0;
        s = NULL;
    }

    return true;
}

static void await_connected (void *data)
{
    // NOTE: ESP-AT sends an ASCII_CAN character following the > character,
    //       this is ok since it flushes the protocol line buffer.
    if(at_cmd_stream.read() == '>') {
        hal.stream.cancel_read_buffer();
        at_cmd_stream.set_enqueue_rt_handler(esp_at_receive);
        return;
    }

    if(--timeout == 0)
        close_session(NULL);
    else
        task_add_delayed(await_connected, NULL, 2);
}

static void await_connect (void *data)
{
    static const io_stream_t telnet_stream = {
        .type = StreamType_Telnet,
        .is_connected = stream_connected,
        .read = atStreamGetC,
        .write_n = atStreamWrite,
        .write = atStreamWriteS,
        .write_char = atStreamPutC,
        .enqueue_rt_command = atStreamEnqueueRtCommand,
        .get_rx_buffer_free = atStreamRxFree,
        .get_tx_buffer_count = atStreamTxCount,
        .reset_read_buffer = atStreamRxFlush,
        .cancel_read_buffer = atStreamRxCancel,
        .suspend_read = atStreamSuspendInput,
        .set_enqueue_rt_handler = atStreamSetRtHandler
    };

    static uint_fast8_t idx = 0;

    int16_t c;

    if((c = at_cmd_stream.read()) != SERIAL_NO_DATA) {

        if(c == ASCII_LF) {

            buf[idx] = '\0';
            idx = 0;

            debug_printf("%s", buf);

            if(!strcmp(buf, "0,CONNECT")) {
                if(send_command("AT+CIPMODE=1") &&
                    send_command("AT+CIPSEND") &&
                     stream_connect(&telnet_stream)) {
                    session_stream = &telnet_stream;
                    idx = 0;
                    timeout = 10;
                    task_add_delayed(await_connected, NULL, 2);
                } // else disconnect!
                return;
            }
        }

        if(!(c == ASCII_CR || c == ASCII_LF)) {
            if(idx >= sizeof(buf) - 1)
                idx = 0;
            else
                buf[idx++] = (char)c;
        }
    }

    task_add_delayed(await_connect, NULL, c == SERIAL_NO_DATA ? 200 : 2);
}

static bool wifi_set_mode (char *mode)
{
    bool ok = false;
    char *s = get_reply(mode);

    while(s) {

        if(is_done(s, &ok))
            break;

        s = get_reply(NULL);
    }

    return ok;
}

static bool start_sta (esp_at_wifi_settings_t *network)
{
    if(!(*network->ssid && wifi_set_mode("AT+CWMODE=1,0")))
        return false;

    bool ok;
    char *s;
    char cmd[sizeof(ssid_t) + sizeof(password_t) + 15];

    if(network->ip_mode == IpMode_Static) {
        sprintf(cmd, "AT+CIPSTA=\"%s\"", network->ip);
        ok = send_command(cmd);
    } else
        ok = send_command("AT+CWDHCP=1,1");

    if(ok && *network->hostname) {
        sprintf(cmd, "AT+CWHOSTNAME=\"%s\"", network->hostname);
        ok = send_command(cmd);
    }

    if(ok) {

        sprintf(cmd, "AT+CWJAP=\"%s\",\"%s\"", network->ssid, network->password);
        s = get_reply(cmd);
        ok = false;
        while(s) {

    // WIFI DISCONNECT
    // WIFI GOT IP
            if(!strcmp(s, "WIFI GOT IP"))
                ok = true;
            else if(is_done(s, NULL))
                break;

            s = get_reply(NULL);
        }
    }

    if(ok) {
        ok = false;
        s = get_reply("AT+CIPSTA?");
        while(s) {

            if(!strncmp(s, "+CIPSTA:", 8)) {
                if(!strncmp(s + 8, "ip:\"", 4)) {
                    s += 12;
                    *strchr(s, '"') = '\0';
                    strcpy(ip, s);
                }
                if(!strncmp(s + 8, "gateway:\"", 9)) {
                    s += 17;
                    *strchr(s, '"') = '\0';
                    strcpy(gateway, s);
                }
                if(!strncmp(s + 8, "netmask:\"", 9)) {
                    s += 17;
                    *strchr(s, '"') = '\0';
                    strcpy(netmask, s);
                }
            } else if(is_done(s, &ok))
                break;

            s = get_reply(NULL);
        }
    }

    if(ok) {
        ok = false;
        s = get_reply("AT+CIPSTAMAC?");
        while(s) {

            if(!strncmp(s, "+CIPSTAMAC:\"", 12)) {
                s += 12;
                *strchr(s, '"') = '\0';
                strcpy(mac, s);
            } else if(is_done(s, &ok))
                break;

            s = get_reply(NULL);
        }
    }

#if ETHERNET_ENABLE
    if(ok && !network_status.ip_aquired) {
        network_status.ip_aquired = On;
        status_event_publish((network_flags_t){ .ip_aquired = On });
    }
#endif

    return ok;
}

static bool start_ap (esp_at_wifi_settings_t *network)
{
    if(!(*network->ssid && wifi_set_mode("AT+CWMODE=2")))
        return false;

    bool ok;
    char *s;
    char cmd[sizeof(ssid_t) + sizeof(password_t) + 30];

    if(network->ip_mode == IpMode_Static) {
        sprintf(cmd, "AT+CIPAP=\"%s\"", network->ip);
        ok = send_command(cmd);
    } else
        ok = send_command("AT+CWDHCP=1,1");

    if(ok && *network->hostname) {
        sprintf(cmd, "AT+CWHOSTNAME=\"%s\"", network->hostname);
        ok = send_command(cmd);
    }

    if(ok) {
        sprintf(cmd, "AT+CWSAP=\"%s\",\"%s\",%d,4,1,0", network->ssid, network->password, esp_at_settings.ap_channel);
        ok = send_command(cmd);
    }

    if(ok) {

        ok = false;
        s = get_reply("AT+CIPAP?");
        while(s) {

            if(!strncmp(s, "+CIPAP:", 7)) {
                if(!strncmp(s + 7, "ip:\"", 4)) {
                    s += 11;
                    *strchr(s, '"') = '\0';
                    strcpy(ip, s);
                }
                if(!strncmp(s + 7, "gateway:\"", 9)) {
                    s += 16;
                    *strchr(s, '"') = '\0';
                    strcpy(gateway, s);
                }
                if(!strncmp(s + 7, "netmask:\"", 9)) {
                    s += 16;
                    *strchr(s, '"') = '\0';
                    strcpy(netmask, s);
                }
            } else if(is_done(s, &ok))
                break;

            s = get_reply(NULL);
        }
    }

    if(ok) {

        ok = false;
        s = get_reply("AT+CIPAPMAC?");
        while(s) {

            if(!strncmp(s, "+CIPAPMAC:\"", 11)) {
                s += 11;
                *strchr(s, '"') = '\0';
                strcpy(mac, s);
            } else if(is_done(s, &ok))
                break;

            s = get_reply(NULL);
        }
    }

#if ETHERNET_ENABLE
    if(ok && !network_status.ip_aquired) {
        network_status.ip_aquired = network_status.ap_started = On;
        status_event_publish((network_flags_t){ .ap_started = On, .ip_aquired = On });
    }
#endif

    return ok;
}

static void esp_at_initialize (void *data)
{
    char *s;
    bool ok;

    if(!(ok = send_command("ATE0")))
        ok = send_command("ATE0");

    if(!(esp_at_running = ok)) {
        close_session(&esp_at_running);
        if(!esp_at_running)
            return;
    }

    send_command("AT+SYSMSG=4");

    hal.delay_ms(2, NULL);

    esp_at_running = false;
    s = get_reply("AT+CIPSERVER?");
    while(s) {

        if(!strncmp(s, "+CIPSERVER:0", 12))
            esp_at_running = false;
        else if(!strncmp(s, "+CIPSERVER:", 11))
            esp_at_running = true;
        else if(is_done(s, &ok))
            break;

        s = get_reply(NULL);
    }

    ok = !esp_at_running || send_command("AT+CIPSERVER=0,1");

#if ETHERNET_ENABLE
    network_status.interface_up = ok;
    status_event_publish((network_flags_t){ .interface_up = On });
#endif

    hal.delay_ms(10, 0);

    if(ok) switch(esp_at_settings.mode) {

        case WiFiMode_STA: // Station
            ok = start_sta(&esp_at_settings.sta);
            break;

        case WiFiMode_AP: // Access point
            ok = start_ap(&esp_at_settings.ap);
            break;

        default:
            // TODO: turn off wifi?
            esp_at_running = ok = false;
            break;
    }

    if(ok) {

        char cmd[25];

        ok = send_command("AT+CIPMODE=0");
        ok = ok && send_command("AT+CIPMUX=1");
        ok = ok && send_command("AT+CIPSERVERMAXCONN=1");
        sprintf(cmd, "AT+CIPSERVER=1,%d", esp_at_settings.telnet_port);
        if((esp_at_running = ok && send_command(cmd)))
            task_add_delayed(await_connect, NULL, 100);
    }
}

static bool get_ports (xbar_t *properties, uint8_t port, void *ports)
{
    switch(properties->function) {

        case Output_CoProc_Reset:
            ((at_ports_t *)ports)->reset = port;
            break;

        case Output_CoProc_Boot0:
            ((at_ports_t *)ports)->boot0 = port;
            break;

        default:
            break;
    }

    return ((at_ports_t *)ports)->reset != 0xFF && ((at_ports_t *)ports)->boot0 != 0xFF;
}

static void esp_at_startup (void *data)
{
    at_ports_t ports = { 0xFF, 0xFF };

    // Claim control ports and reset ESP-AT processor if ports available.
    if(ioports_enumerate(Port_Digital, Port_Output, (pin_cap_t){ .output = On }, get_ports, (void *)&ports)) {
        hal.port.digital_out(ports.boot0, 1);
        hal.port.digital_out(ports.reset, 0);
        hal.delay_ms(2, NULL);
        hal.port.digital_out(ports.reset, 1);
    }

    // Allow ESP-AT processor time to boot.
    task_add_delayed(esp_at_initialize, NULL, 1500);
}

/*
static bool is_setting_available (const setting_detail_t *setting)
{
    return ESP-AT available?
}
*/

static bool validate_ip (char *ip)
{
    bool ok;
    uint_fast8_t idx = 0, pos = 0;
    uint32_t res;
    char c, *s = ip, *s2 = ip;

    if(!(ok = strchr(ip, '.') || strlen(ip) < sizeof(esp_at_settings.sta.ip)))
        return false;

    // remove spaces
    while((c = *s)) {
        if(c != ' ')
            *s2++ = *s;
        s++;
    }
    *s2 = '\0';

    s = strchr(ip, '.');
    while(s && idx < 4) {
        *s = '\0';
        if(read_uint(ip, &pos, &res) == Status_OK && res <= 255) {
            if(idx < 3)
                *s = '.';
            if(ip[pos] != (idx == 3 ? '\0' : '.')) {
                ok = false;
                break;
            }
            pos = (s - ip) + 1;
            s = strchr(++s, ++idx == 3 ? '\0' : '.');
        } else {
            ok = false;
            break;
        }
    }

    return ok;
}

static status_code_t wifi_set_ip (setting_id_t setting, char *value)
{
    status_code_t status = Status_OK;

    if(validate_ip(value)) switch(setting) {

        case Setting_IpAddress3:
            strcpy(esp_at_settings.sta.ip, value);
            break;

        case Setting_Gateway3:
            strcpy(esp_at_settings.sta.gateway, value);
            break;

        case Setting_NetMask3:
            strcpy(esp_at_settings.sta.mask, value);
            break;

        case Setting_IpAddress2:
            strcpy(esp_at_settings.ap.ip, value);
            break;

        case Setting_Gateway2:
            strcpy(esp_at_settings.ap.gateway, value);
            break;

        case Setting_NetMask2:
            strcpy(esp_at_settings.ap.mask, value);
            break;

        default:
            status = Status_Unhandled;
            break;
    } else
       status =  Status_InvalidStatement;

    return status;
}

static char *wifi_get_ip (setting_id_t setting)
{
    char *ip;

    switch(setting) {

        case Setting_IpAddress3:
            ip = esp_at_settings.sta.ip;
            break;

        case Setting_Gateway3:
            ip = esp_at_settings.sta.gateway;
            break;

        case Setting_NetMask3:
            ip = esp_at_settings.sta.mask;
            break;

        case Setting_IpAddress2:
            ip = esp_at_settings.ap.ip;
            break;

        case Setting_Gateway2:
            ip = esp_at_settings.ap.gateway;
            break;

        case Setting_NetMask2:
            ip = esp_at_settings.ap.mask;
            break;

        default:
            ip = NULL;
            break;
    }

    return ip ? ip : "";
}

static const setting_group_detail_t esp_at_groups [] = {
    { Group_Root, Group_Networking, "Networking" },
    { Group_Networking, Group_Networking_Wifi, "WiFi" }
};

static const setting_detail_t xesp_at_settings[] = {
    { Setting_WifiMode, Group_Networking_Wifi, "WiFi Mode", NULL, Format_RadioButtons, "Off,Station,Access Point", NULL, NULL, Setting_NonCore, &esp_at_settings.mode, NULL, NULL },
    { Setting_WiFi_STA_SSID, Group_Networking_Wifi, "WiFi Station (STA) SSID", NULL, Format_String, "x(64)", NULL, "64", Setting_NonCore, &esp_at_settings.sta.ssid, NULL, NULL },
    { Setting_WiFi_STA_Password, Group_Networking_Wifi, "WiFi Station (STA) Password", NULL, Format_Password, "x(32)", "8", "32", Setting_NonCore, &esp_at_settings.sta.password, NULL, NULL, { .allow_null = On } },
    { Setting_Hostname3, Group_Networking, "Hostname (STA)", NULL, Format_String, "x(64)", NULL, "64", Setting_NonCore, esp_at_settings.sta.hostname, NULL, NULL, { .reboot_required = On } },
    { Setting_IpMode3, Group_Networking, "IP Mode (STA)", NULL, Format_RadioButtons, "Static,DHCP", NULL, NULL, Setting_NonCore, &esp_at_settings.sta.ip_mode, NULL, NULL, { .reboot_required = On } },
    { Setting_IpAddress3, Group_Networking, "IP Address (STA)", NULL, Format_IPv4, NULL, NULL, NULL, Setting_NonCoreFn, wifi_set_ip, wifi_get_ip, NULL, { .reboot_required = On } },
    { Setting_Gateway3, Group_Networking, "Gateway (STA)", NULL, Format_IPv4, NULL, NULL, NULL, Setting_NonCoreFn, wifi_set_ip, wifi_get_ip, NULL, { .reboot_required = On } },
    { Setting_NetMask3, Group_Networking, "Netmask (STA)", NULL, Format_IPv4, NULL, NULL, NULL, Setting_NonCoreFn, wifi_set_ip, wifi_get_ip, NULL, { .reboot_required = On } },
    { Setting_TelnetPort3, Group_Networking, "Telnet port (STA)", NULL, Format_Int16, "####0", "1", "65535", Setting_NonCore, &esp_at_settings.telnet_port, NULL, NULL, { .reboot_required = On } },
    { Setting_WiFi_AP_SSID, Group_Networking_Wifi, "WiFi Access Point (AP) SSID", NULL, Format_String, "x(64)", NULL, "64", Setting_NonCore, &esp_at_settings.ap.ssid, NULL, NULL },
    { Setting_WiFi_AP_Password, Group_Networking_Wifi, "WiFi Access Point (AP) Password", NULL, Format_Password, "x(32)", "8", "32", Setting_NonCore, &esp_at_settings.ap.password, NULL, NULL, { .allow_null = On } },
//    { Setting_Wifi_AP_Country, Group_Networking_Wifi, "WiFi Country Code", NULL, Format_String, "x(2)", "2", "2", Setting_NonCoreFn, wifi_set_country, wifi_get_country, NULL, { .allow_null = On, .reboot_required = On } },
    { Setting_Wifi_AP_Channel, Group_Networking_Wifi, "WiFi Channel (AP)", NULL, Format_Int8, "#0", "1", "13", Setting_NonCore, &esp_at_settings.ap_channel, NULL, NULL, { .reboot_required = On } },
    { Setting_Hostname2, Group_Networking, "Hostname (AP)", NULL, Format_String, "x(64)", NULL, "64", Setting_NonCore, &esp_at_settings.ap.hostname, NULL, NULL, { .reboot_required = On } },
    { Setting_IpAddress2, Group_Networking, "IP Address (AP)", NULL, Format_IPv4, NULL, NULL, NULL, Setting_NonCoreFn, wifi_set_ip, wifi_get_ip, NULL, { .reboot_required = On } },
    { Setting_Gateway2, Group_Networking, "Gateway (AP)", NULL, Format_IPv4, NULL, NULL, NULL, Setting_NonCoreFn, wifi_set_ip, wifi_get_ip, NULL, { .reboot_required = On } },
    { Setting_NetMask2, Group_Networking, "Netmask (AP)", NULL, Format_IPv4, NULL, NULL, NULL, Setting_NonCoreFn, wifi_set_ip, wifi_get_ip, NULL, { .reboot_required = On } }
};

#ifndef NO_SETTINGS_DESCRIPTIONS

static const setting_descr_t esp_at_settings_descr[] = {
    { Setting_WifiMode, "WiFi Mode." },
    { Setting_WiFi_STA_SSID, "WiFi Station (STA) SSID." },
    { Setting_WiFi_STA_Password, "WiFi Station (STA) Password." },
    { Setting_Hostname3, "WiFi Station (STA) network hostname." },
    { Setting_IpAddress3, "WiFi Station (STA) static IP address." },
    { Setting_Gateway3, "WiFi Station (STA) static gateway address." },
    { Setting_NetMask3, "WiFi Station (STA) static netmask." },    { Setting_TelnetPort3, "(Raw) Telnet port number listening for incoming connections." },
    { Setting_IpMode3, "WiFi Station (STA) IP Mode." },
    { Setting_WiFi_AP_SSID, "WiFi Access Point (AP) SSID." },
    { Setting_WiFi_AP_Password, "WiFi Access Point (AP) Password." },
//    { Setting_Wifi_AP_Country, "ISO3166 country code, controls availability of channels 12-14.\\n"
//                               "Set to ""01"" for generic worldwide channels." },
    { Setting_Wifi_AP_Channel, "WiFi Access Point (AP) channel to use.\\n May be overridden when connecting to an Access Point as station or by country setting." },
    { Setting_Hostname2, "WiFi Access Point (AP) network hostname." },
    { Setting_IpAddress2, "WiFi Access Point (AP) static IP address." },
    { Setting_Gateway2, "WiFi Access Point (AP) static gateway address." },
    { Setting_NetMask2, "WiFi Access Point (AP) static netmask." },
};

#endif

static void esp_at_settings_save (void)
{
    hal.nvs.memcpy_to_nvs(nvs_address, (uint8_t *)&esp_at_settings, sizeof(esp_at_settings_t), true);
}

static void esp_at_settings_restore (void)
{
    memset(&esp_at_settings, 0xFF, sizeof(esp_at_settings_t));

    esp_at_settings.mode = WiFiMode_STA;
    esp_at_settings.telnet_port = 23;
    *esp_at_settings.ap.ssid = '\0';
    *esp_at_settings.ap.password = '\0';
    *esp_at_settings.sta.ssid = '\0';
    *esp_at_settings.sta.password = '\0';

    esp_at_settings.sta.ip_mode = IpMode_DHCP;
    esp_at_settings.ap.ip_mode = IpMode_Static;
    esp_at_settings.ap_channel = 5;
    *esp_at_settings.ap_country = '\0';

#ifdef NETWORK_STA_HOSTNAME
    strcpy(esp_at_settings.sta.hostname, NETWORK_STA_HOSTNAME);
#else
    strcpy(esp_at_settings.sta.hostname, "grblHAL");
#endif
#ifdef NETWORK_STA_IP
    strcpy(esp_at_settings.sta.ip, NETWORK_STA_IP);
#else
    strcpy(esp_at_settings.sta.ip, "192.168.5.1");
#endif
#ifdef NETWORK_STA_GATEWAY
    strcpy(esp_at_settings.sta.gateway, NETWORK_STA_GATEWAY);
#else
    strcpy(esp_at_settings.sta.gateway, "192.168.5.1");
#endif
#ifdef NETWORK_STA_MASK
    strcpy(esp_at_settings.sta.mask, NETWORK_STA_MASK);
#else
    strcpy(esp_at_settings.sta.mask, "255.255.255.0");
#endif
#ifdef NETWORK_AP_HOSTNAME
    strcpy(esp_at_settings.ap.hostname, NETWORK_AP_HOSTNAME);
#else
    strcpy(esp_at_settings.ap.hostname, "grblHAL_AP");
#endif
#ifdef NETWORK_AP_IP
    strcpy(esp_at_settings.ap.ip, NETWORK_AP_IP);
#else
    strcpy(esp_at_settings.ap.ip, "192.168.5.1");
#endif
#ifdef NETWORK_AP_GATEWAY
    strcpy(esp_at_settings.ap.gateway, NETWORK_AP_GATEWAY);
#else
    strcpy(esp_at_settings.ap.gateway, "192.168.5.1");
#endif
#ifdef NETWORK_AP_MASK
    strcpy(esp_at_settings.ap.mask, NETWORK_AP_MASK);
#else
    strcpy(esp_at_settings.ap.mask, "255.255.255.0");
#endif
#ifdef NETWORK_AP_SSID
    strcpy(esp_at_settings.ap.ssid, NETWORK_AP_SSID);
#else
    strcpy(esp_at_settings.ap.ssid, "grblHAL_AP");
#endif
#ifdef NETWORK_AP_PASSWORD
    strcpy(esp_at_settings.ap.password, NETWORK_AP_PASSWORD);
#else
    strcpy(esp_at_settings.ap.password, "grblHALpwd");
#endif

    esp_at_settings_save();
}

static void esp_at_settings_load (void)
{
    if(hal.nvs.memcpy_from_nvs((uint8_t *)&esp_at_settings, nvs_address, sizeof(esp_at_settings_t), true) != NVS_TransferResult_OK)
        esp_at_settings_restore();
}

static void report_options (bool newopt)
{
    on_report_options(newopt);

    if(!newopt) {

        if(*mac) {
            hal.stream.write("[WIFI MAC:");
            hal.stream.write(mac);
            hal.stream.write("]" ASCII_EOL);
        }

        if(*ip) {
            hal.stream.write("[IP:");
            hal.stream.write(ip);
            hal.stream.write("]" ASCII_EOL);
        }

        report_plugin(esp_at_running ? "ESP-AT" : "ESP-AT (disabled)", "0.05");
    }
}

void esp_at_init (void)
{
    static setting_details_t setting_details = {
        .groups = esp_at_groups,
        .n_groups = sizeof(esp_at_groups) / sizeof(setting_group_detail_t),
        .settings = xesp_at_settings,
        .n_settings = sizeof(xesp_at_settings) / sizeof(setting_detail_t),
#ifndef NO_SETTINGS_DESCRIPTIONS
        .descriptions = esp_at_settings_descr,
        .n_descriptions = sizeof(esp_at_settings_descr) / sizeof(setting_descr_t),
#endif
        .save = esp_at_settings_save,
        .load = esp_at_settings_load,
        .restore = esp_at_settings_restore,
    };

    bool ok;
    io_stream_t const *stream = stream_open_instance(COPROC_STREAM, 115200, NULL, "ESP-AT");
    if((ok = stream != NULL)) {
        memcpy(&at_cmd_stream, stream, sizeof(io_stream_t));
        at_cmd_stream.set_enqueue_rt_handler(stream_buffer_all);
    }

    if(ok && (nvs_address = nvs_alloc(sizeof(esp_at_settings_t)))) {

#if ETHERNET_ENABLE
        networking_init();

        get_info = networking.get_info;
        networking.get_info = getInfo;
#endif

        on_report_options = grbl.on_report_options;
        grbl.on_report_options = report_options;

        settings_register(&setting_details);

        protocol_enqueue_foreground_task(esp_at_startup, NULL);
    } else
        protocol_enqueue_foreground_task(report_warning, "ESP-AT plugin failed to initialize!");
}

#endif // ESP_AT_ENABLE
