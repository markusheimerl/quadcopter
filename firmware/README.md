# quadcopter firmware

Firmware for the quadcopter flight controller in `../pcb` (ESP32-S3-WROOM-1
module, BMI323 IMU, four brushed motors on low-side FETs). Written in C
against [ESP-IDF](https://github.com/espressif/esp-idf), with as little
framework code between us and the silicon as practical.

It reads the IMU at 200 Hz, levels the quad with an angle PD loop and a
quad-X mixer, and takes commands over BLE (Nordic UART service, device
name `QuadFW`) from `tools/motor_keys.py`, which also streams the log back.

## Hardware

- Pin map: see `../pcb/README.md` (PWM on IO1/IO2/IO16/IO4, I2C on
  IO11/IO12, LED IO21 active-low, battery/2 on IO10).
- Programming/console link: the S3's **built-in USB Serial/JTAG** peripheral.
  No CP210x/CH340 bridge -- the chip enumerates directly as
  `/dev/ttyACM0` (VID:PID `303a:1001`). Plug a battery in first, then USB.

## Flying it (BLE keys, `tools/motor_keys.py`)

| key | action |
|---|---|
| `1`..`4` | spin M0 front right / M1 back right / M2 back left / M3 front left for 1 s (disarmed; props off) |
| `c` | redo the level calibration (disarmed, board still on a level surface) |
| `i` | status: reset reason, IMU id and error register, calibration, battery |
| `a` | arm (needs a live heartbeat, a finished level calibration, a battery >= 3.5 V and a board within 5 degrees of level) |
| `w` `+` `=` / `x` `-` | throttle up / down (steps of 5, max 220) |
| `d` `s` `0` | disarm |
| `q` | quit the client (it disarms first) |

`tools/motor_keys.py` needs `pip install bleak` and a POSIX terminal. It
sends a heartbeat `h` every 200 ms; any other BLE UART client must send `h`
at least every 500 ms to arm and every second to stay armed, and a client
that sends none for 5 s is disconnected (a crashed client must not keep the
link, or the board stops advertising).

Nothing spins at power-up. The level calibration runs once the board has
been still and within 5 degrees of level for 1 s. Safety: the quad
disarms when the BLE link drops, when the heartbeat stops for 1 s, after
20 s armed at zero throttle, after 2 s tilted over 30 degrees with the
throttle up, and on a tilt beyond 50 degrees. A disarmed board below
3.3 V goes to deep sleep and checks the battery every 5 minutes (every 30
below 3.0 V; press RST to wake it at once).

## One-time setup

### 1. Serial port access

Add your user to the `dialout` group so you can talk to `/dev/ttyACM0`
without `sudo`, then log out + back in (or `newgrp dialout`):

```bash
sudo usermod -aG dialout "$USER"
```

### 2. Host build dependencies

```bash
sudo apt install -y git wget flex bison gperf \
    python3 python3-venv python3-pip \
    cmake ninja-build ccache \
    libffi-dev libssl-dev dfu-util libusb-1.0-0
```

### 3. ESP-IDF

We pin to a release branch so builds stay reproducible.

```bash
mkdir -p ~/esp && cd ~/esp
git clone --recursive --depth 1 -b v5.3.1 \
    https://github.com/espressif/esp-idf.git
cd esp-idf
./install.sh esp32s3
```

This downloads the Xtensa toolchain (`xtensa-esp32s3-elf-gcc`),
`esptool.py`, and Python deps into `~/.espressif/` (~1 GB).

### 4. Activate the toolchain in your shell

Every new shell needs the IDF environment sourced:

```bash
. ~/esp/esp-idf/export.sh
```

(Add it to your `~/.bashrc` if you want it automatic.)

## Build

```bash
cd firmware
idf.py set-target esp32s3   # only needed once, creates sdkconfig
idf.py build
```

## Flash & monitor

With the board plugged in:

```bash
sg dialout -c '. ~/esp/esp-idf/export.sh && cd ~/quadcopter/firmware && idf.py -p /dev/ttyACM0 flash monitor'
```

`Ctrl-]` quits the monitor.

If the chip is not in download mode and flashing fails (or the board is in
its low-battery sleep), hold **BOOT**, tap **RST**, release **BOOT**, and
retry.

## Clean

```bash
idf.py fullclean
```

## Layout

```
firmware/
  CMakeLists.txt        top-level ESP-IDF project
  sdkconfig.defaults    pinned config (target, console over USB-JTAG, ...)
  main/
    CMakeLists.txt      'main' component registration
    main.c              app_main(), control loop, BLE, battery guard
  tools/
    motor_keys.py       BLE keyboard client + log viewer
```
