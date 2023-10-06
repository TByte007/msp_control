# MSP_SET_RAW RC "considered harmful?"

## Overview

This golang program exercises `MSP_SET_RAW_RC`. This is intended to be a learning / experimental / educational tool rather than an end user application.

### Why

Every few months, someone will come along on INAV Github / RC Groups / Telegram / Discord / some other random support channel and state that `RX_MSP` / `set receiver_type = MSP`  doesn't work.

Well it does, if you do it correctly. This example demonstrates usage.

As Go is available on pretty much any OS, you can even verify that it works, with this small application.

**NOTE: The previous, verbose, timer activated version of this tool is now in the `legacy` branch, should you want to play with that**

## FC Prerequisites

* A supported FC
* INAV v6 or later, earlier versions may also work
* The FC should have been configured to a state in which it can be armed:
  - The sensors are calibrated
  - Required sensors are powered (e.g. GPS requiring battery)
  - If necessary `nav_extra_arming_safety` may be set to `ALLOW_BYPASS`
  - The arming channel is defined, with a range less than 1000

### Set the correct RX type

Modern firmware (e.g INAV 1.8 and later)
```
set receiver_type = MSP
```

For older versions of this example, with INAV prior to INAV 1.8, you can try:
```
# for ancient firmware
feature RX_MSP
```
However, don't be surprised if ancient versions fail to work with `msp_set_rx`.

## Building

* Clone this repository
* Build the test application

 ```
 make
 ```

This should result in a `msp_set_rx` application.

## Usage

```
$ msp_set_rx --help
Usage of msp_set_rx [options]
  -b int
    	Baud rate (default 115200)
  -d string
    	Serial Device
  -throttle int
    	Low throttle (µs) (default -1)
```

When initialised, the application will accept keypresses:

* `A`, `a` : Toggle arming state
  - If the FC is in a "ready to arm" condition, it will be armed
  - If the FC is armed, it will be disarmed
* `Q`,`q`,`Ctrl-C` : Clean exit. If the FC is armed, it will be disarmed first.
* `F`: Unclean exit, potentially causing fail-safe. Be prepared to handle the consequences.

If a `-throttle` value has been set, then, when armed it will run the motors at that value and the throttle will not be randomly perturbed. Two additional keypresses are recognised:

* `+`, `-` raise / lower throttle by 25µs

If the application is exited uncleanly, then on restarting `msp_set_rx`, the FC should recover from fail-safe (note roll and pitch are perturbed to force recovery).

```
$ ./msp_set_rx -d /dev/ttyUSB0 [-b baud]
# and hence, probably, for example
C:\> msp_set_rx.exe -d COM42 -b 115200
# Linux, autodetect
$ ./msp_set_rx
```

Note: On Linux, `/dev/ttyUSB0` and `/dev/ttyACM0` are automatically detected.

While this tool attempts to arm at a safe throttle value, removing props or using a current limiter is recommended. Using the [INAV_SITL](https://github.com/iNavFlight/inav/blob/master/docs/SITL/SITL.md) is also a good option. A suitable configuration for such experiments is described in the [fl2sitl wiki](https://github.com/stronnag/bbl2kml/wiki/fl2sitl#sitl-configuration)

Please also note that if you do not define a "low throttle" (`-throttle`) value, then when armed, the motors will run at randomly changing throttle between 1100us and 1300us. Please ensure you and your hardware are content with this.

## Examples

### FC example

```
./msp_set_rx
[msp_set_rx] 19:20:08.885594 Using device /dev/ttyACM0
INAV v7.0.0 WINGFC (497c01eb) API 2.5
nav_extra_arming_safety: 2 (bypass true)
map: AETR
name: "BENCHYMCTESTY"
box: ARM;PREARM;MULTI FUNCTION;ANGLE;HORIZON;TURN ASSIST;HEADING HOLD;CAMSTAB;NAV POSHOLD;LOITER CHANGE;NAV RTH;NAV WP;NAV CRUISE;NAV COURSE HOLD;HOME RESET;GCS NAV;WP PLANNER;MISSION CHANGE;SOARING;NAV ALTHOLD;MANUAL;NAV LAUNCH;SERVO AUTOTRIM;AUTO TUNE;AUTO LEVEL TRIM;BEEPER;BEEPER MUTE;OSD OFF;BLACKBOX;KILLSWITCH;FAILSAFE;CAMERA CONTROL 1;CAMERA CONTROL 2;CAMERA CONTROL 3;OSD ALT 1;OSD ALT 2;OSD ALT 3;MIXER PROFILE 2;MIXER TRANSITION;
chan:  5, start: 1300, end: 1700 NAV POSHOLD
chan:  5, start: 1700, end: 2100 NAV RTH
chan:  6, start: 1300, end: 2100 NAV WP
chan:  7, start: 1700, end: 2100 NAV ALTHOLD
chan:  7, start: 1700, end: 2100 NAV COURSE HOLD
chan:  8, start: 1375, end: 1600 NAV LAUNCH
chan: 10, start: 1500, end: 2100 ARM
chan: 11, start: 1450, end: 2100 MANUAL
chan: 12, start: 1600, end: 2100 BEEPER
Arming set for channel 10 / 1800us
Keypresses: 'A'/'a': toggle arming, 'Q'/'q': quit, 'F': quit to failsafe
[msp_set_rx] 19:20:09.101738 Start TX loop
[msp_set_rx] 19:20:09.228487 Box: FAILSAFE (40000000) Arm: RCLink (0x40000)
[msp_set_rx] 19:20:09.932720 Box:  (0) Arm: Ready to arm (0x0)
```
Depending on how early in the boot process you start `msp_set_rx`, you may also see some calibration messages.

Having reached the "Ready to arm" state, if you press `A`, the FC will be armed:
```
[msp_set_rx] 19:31:55.324435 Box: ARM (1) Arm: Armed (0xc)
```

And if `A` is pressed again, the FC is disarmed:
```
[msp_set_rx] 19:33:36.633957 Box:  (0) Arm: Ready to arm (0x8)
```

Pressing `Q` or `Ctrl-C` will exit the application, if the FC is armed, it will be disarmed first.

```
$ ./msp_set_rx
...
[msp_set_rx] 19:35:13.895771 Box:  (0) Arm: Ready to arm (0x8)
######## Press 'A'
[msp_set_rx] 19:35:15.191735 Box: ARM (1) Arm: Armed (0xc)
######## Press `Q`
[msp_set_rx] 19:35:17.895726 Box:  (0) Arm: Ready to arm (0x8)
######## FC was automatically disarmed prior to exit
$
```

Use `F` key press to exit without disarming, causing fail-safe:

```
$ ./msp_set_rx
...
[msp_set_rx] 19:40:32.608377 Box:  (0) Arm: Ready to arm (0x8)
[msp_set_rx] 19:40:34.000353 Box: ARM (1) Arm: Armed (0xc)
######## Press `F`
######## Immediate exit with disarming, we're in failsafe ...
$
```

If we restart after failsafe:
```
$ ./msp_set_rx
...
[msp_set_rx] 19:40:59.297024 Start TX loop
[msp_set_rx] 19:40:59.423609 Box: ARM,ANGLE,FAILSAFE (40000009) Arm: Armed (0xc)
[msp_set_rx] 19:41:00.127562 Box: ARM (1) Arm: Armed (0xc)

```

Note the FC "Box" state shows `ARM,ANGLE,FAILSAFE`, and that we are still armed. It then quickly recovers (0.5s, i.e. meeting the required 5Hz update rate) to a normal armed state, from which we can disarm /  quit cleanly.

### SITL example

You can also use the INAV SITL to test. This has the advantage of not requiring hardware. The same arming prerequisites apply.

```
$ ./msp_set_rx -d tcp://localhost:5761 -throttle 1200
[msp_set_rx] 21:09:56.436998 Using device localhost
INAV v7.0.0 SITL (3a2412e5) API 2.5
nav_extra_arming_safety: 2 (bypass true)
map: AETR
name: "BENCHYMCTESTY"
box: ARM;PREARM;MULTI FUNCTION;ANGLE;HORIZON;TURN ASSIST;HEADING HOLD;CAMSTAB;NAV POSHOLD;LOITER CHANGE;NAV RTH;NAV WP;NAV CRUISE;NAV COURSE HOLD;HOME RESET;GCS NAV;WP PLANNER;MISSION CHANGE;SOARING;NAV ALTHOLD;MANUAL;NAV LAUNCH;SERVO AUTOTRIM;AUTO TUNE;AUTO LEVEL TRIM;BEEPER;BEEPER MUTE;OSD OFF;BLACKBOX;KILLSWITCH;FAILSAFE;CAMERA CONTROL 1;CAMERA CONTROL 2;CAMERA CONTROL 3;OSD ALT 1;OSD ALT 2;OSD ALT 3;
chan:  5, start: 1300, end: 1700 NAV POSHOLD
chan:  5, start: 1700, end: 2100 NAV RTH
chan:  6, start: 1300, end: 2100 NAV WP
chan:  7, start: 1700, end: 2100 NAV ALTHOLD
chan:  7, start: 1700, end: 2100 NAV COURSE HOLD
chan:  8, start: 1375, end: 1600 NAV LAUNCH
chan: 10, start: 1500, end: 2100 ARM
chan: 11, start: 1450, end: 2100 MANUAL
chan: 12, start: 1600, end: 2100 BEEPER
Arming set for channel 10 / 1800us
Keypresses: 'A'/'a': toggle arming, 'Q'/'q': quit, 'F': quit to failsafe
            '+'/'-' raise / lower throttle by 25µs
[msp_set_rx] 21:09:57.497876 Start TX loop
[msp_set_rx] 21:09:57.599109 Box: MANUAL,FAILSAFE (40100000) Arm: Calibrate RCLink (0x40220)
[msp_set_rx] 21:09:58.299169 Box: MANUAL (100000) Arm: Calibrate (0x220)
[msp_set_rx] 21:10:01.499031 Box:  (0) Arm: Ready to arm (0x20)
[msp_set_rx] 21:10:23.699638 Box: ARM (1) Arm: Armed (0x2c)
[msp_set_rx] 21:10:30.099620 Box:  (0) Arm: Ready to arm (0x28)
[msp_set_rx] 21:10:31.598849 Box: ARM (1) Arm: Armed (0x2c)
Throttle: 1225
Throttle: 1250
Throttle: 1225
Throttle: 1200
Throttle: 1175
[msp_set_rx] 21:10:35.299420 Box:  (0) Arm: Ready to arm (0x28)
```
Note that the SITL captures some of the early status / calibration changes. The /- keys are used to manually change the throttle value.

#### SITL usage

In one terminal (with `eeprom.bin` pre-configured for MSP receiver):
```
inav_SITL --path /path/to/eeprom.bin --sim xp
```

In another terminal:
```
fl2sitl -minimal
```

Or, more simply, just combine the above steps into:
```
(inav_SITL --path ~/sitl-eeproms/fw-eeprom.bin --sim xp &) ;  fl2sitl --minimal
# don't forget to kill the inav_SITL when done
```
Then:
```
msp_set_rx -d tcp://localhost:5761
```

Note that in order for the SITL to arm, a simulator is needed.In the example we use [fl2sitl](https://github.com/stronnag/bbl2kml/wiki/fl2sitl) with `-minimal` to provide the simplest simulation that will unlock the SITL sensors and allow arming.

## arm_status

There is a simple tool to interpret arming status `arm_status`. It accepts one or more numeric status codes and displays human readable interpretation:

```
./arm_status 28 84c00
Status 00000028:
 00000008 => Ever Armed
 00000020 => SITL
Status 00084c00:
 00000400 => Overload
 00000800 => Navigation unsafe
 00004000 => Arm switch
 00080000 => Throttle
```

Copy it onto `$PATH` if you wish.

## Other examples

The [flightlog2kml](https://github.com/stronnag/bbl2kml) project contains a tool [fl2sitl](https://github.com/stronnag/bbl2kml/wiki/fl2sitl) that replays a blackbox log using the [INAV SITL](https://github.com/iNavFlight/inav/blob/master/docs/SITL/SITL.md). Specifically, this uses MSP and MSP_SET_RAW_RC to establish vehicle characteristics, monitor the vehicle status, arm the vehicle and set RC values for AETR and switches during log replay simulation to effectively "fly" the SITL for the recorded flight.

The MSP initialisation, MSP status monitoring and MSP RC management code is in [msp.go](https://github.com/stronnag/bbl2kml/blob/master/pkg/sitlgen/msp.go), specifically the `init()` and `run()` functions. Arming / disarming in [sitlgen.go](https://github.com/stronnag/bbl2kml/blob/master/pkg/sitlgen/sitlgen.go), `arm_action()` function.

This is more comprehensive (and complex) example.

## Caveats

* Ensure you provide (at least) 5Hz RX data, but don't overload the FC; MSP is a request-response protocol, don't just "spam" the FC via a high frequency timer and ignore the responses.
* Ensure you've set a correct, valid AUX range to arm. In particular and for safety, the ARM range must be less or equal to 1000 in order to allow disarming.
* Ensure you've met the required arming conditions
* Use a supported FC or the SITL
* Remove the props etc.

## Postscript

Really, it's all about preparation, in particular the interrogation of the FC prior to sending any `MSP_SET_RAW_RC` data in order to get the box names and switch settings that will be necessary to arm and monitor the "box" and "arming" flags that enable your application to monitor the state of the automated FC.

## Licence

Whatever approximates to none / public domain in your locale. 0BSD (Zero clause BSD)  if an actual license is required by law.
