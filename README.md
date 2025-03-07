## Assorted small plugins

### Feed override

Adds Marlin style [M220](https://marlinfw.org/docs/gcode/M220.html) command for setting feed overrides.

```
  M220 [B] [R] [S<percent>]

  B - backup current values
  R - restore values from backup
  S<percent> - percentage of current feedrate.
```
> [!NOTE]
> `M220RS<percentage>` can be used to override the rapids rate, if R is not specified the feed rate will be overridden. This deviates from the Marlin specification.

Configuration:

Add/uncomment `#define FEED_OVERRIDE_ENABLE 1` in _my_machine.h_.

### Homing pulloff

*** Experimental ***

Adds per axis setting for homing pulloff distance. `$290` for X-axis pulloff, `$291` for Y-axis, ...

Configuration:

Add/uncomment `#define HOMING_PULLOFF_ENABLE 1` in _my_machine.h_.

### ESP-AT

*** Experimental ***

Adds Telnet support via [ESP-AT](https://docs.espressif.com/projects/esp-at/en/latest/esp32/Get_Started/index.html) running on a supported ESP MCU.
Allows senders to connect to the controller via WiFi.

Dependencies:

Driver and board support for a serial port \(UART\) that is not claimed by a different plugin.

Configuration:

Add/uncomment `#define ESP_AT_ENABLE 1` in _my_machine.h_.

Adds many networking and WiFi settings for configuring mode \(Station, Access Point\), Telnet port, IP adress etc.

> [!NOTE]
> The ESP32 is a 3.3V device and pins are not 5V tolerant. If the controller serial port uses 5V signalling add a 5V to 3.3V level shifter, at least for the controller TX line.

### RGB LED strips

Adds one or two settings, `$536` and `$537`, for setting number of LEDs in NeoPixel/WS2812 LED strips.

Dependencies:

Driver and board support for LED strips.

Configuration:

Add/uncomment `#define RGB_LED_ENABLE 1` in _my_machine.h_.

### RGB LED strip control

Adds support for Marlin style [M150 command](https://marlinfw.org/docs/gcode/M150.html).

```
M150 [B<intensity>] [I<pixel>] [K] [P<intensity>] [R<intensity>] [S<strip>] [U<intensity>] [W<intensity>]

    B<intensity> - blue component, 0 - 255.
    I<pixel>     - LED index, 0 - 255. Available if number of LEDs in strip is > 1.
    K            - keep unspecified values.
    P<intensity> - brightness, 0 - 255.
    S<strip>     - strip index, 0 or 1. Default is 0.
    R<intensity> - red component, 0 - 255.
    U<intensity> - red component, 0 - 255.
```

Dependencies:

Driver and board support for LED strips.

Configuration:

Add/uncomment `#define RGB_LED_ENABLE` in _my_machine.h_ and set/change the define value to `2`.

Dependencies:

Driver and board support for LED strips.  
> [!NOTE]
> If B axis is enabled then the plugin will not be activated.

### Bind events to aux output ports

Configuration:

Add/uncomment `#define EVENTOUT_ENABLE 1` in _my_machine.h_ .

Depending on the number of free output ports up to four events can be selected.
Settings `$750+<n>` is used to select the event \(trigger\) to bind to the port selected by setting `$760+<n>`
where `<n>` is the event number, currently 0 - 3.

Dependencies:

The selected driver/board must provide at least one free auxiliary output port.

Credits:

Based on idea from [plugin](https://github.com/Sienci-Labs/grblhal-switchbank) by @Sienci-Labs.

### PWM Servo

*** Experimental ***

Adds support for Marlin style [M280](https://marlinfw.org/docs/gcode/M280.html) command.

```
M280 [P<index>] [S<position>]

    P<index>    - servo to set or get position for. Default is 0.
    S<position> - set position in degrees, 0 - 180.
                  If omitted the current position is reported: 
                  [Servo <index> position: <position> degrees]

```

Configuration:

Add/uncomment `#define PWM_SERVO_ENABLE 1` in _my_machine.h_.

Dependencies:

The selected driver/board must provide at least one free auxiliary analog output port that is PWM capable.

Credits:

Based on [code](https://github.com/wakass/grlbhal_servo) by @wakass.

### BLTouch probe

*** Experimental ***

Adds support for Marlin style [M401](https://marlinfw.org/docs/gcode/M401.html) and [M402](https://marlinfw.org/docs/gcode/M402.html) commands.

```
M401 [H] [R<0|1>] [S<0|1>] - deploy probe

    H      - report high speed mode.
    R<0|1> - currently ignored.
    S<0|1> - S0: disable high speed mode, S1: enable high speed mode.
```

```
M402 [R<0|1>] - stow probe

    R<0|1> - currently ignored.
```

Configuration:

Add/uncomment `#define PWM_SERVO_ENABLE 1` and `#define BLTOUCH_ENABLE 1` in _my_machine.h_.

Dependencies:

The selected driver/board must provide at least one free auxiliary analog output port that is PWM capable.

Credits:

Based on [code](https://github.com/wakass/grlbhal_servo) by @wakass.

---
2024-12-20
