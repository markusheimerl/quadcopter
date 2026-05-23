# quadcopter firmware

Firmware for the Seeed Studio **XIAO ESP32-S3 (Sense)** flight controller.
Written in C against [ESP-IDF](https://github.com/espressif/esp-idf), with as
little framework code between us and the silicon as practical -- peripherals
are driven by writing the SoC's memory-mapped registers directly where it
makes sense.

Right now this just blinks the on-board user LED (GPIO21, active-low). It
will grow into the full flight stack.

## Hardware

- MCU: ESP32-S3 (dual Xtensa LX7 @ 240 MHz, 512 KB SRAM, 8 MB PSRAM, 8 MB flash on the *Sense* variant)
- Programming/console link: the S3's **built-in USB Serial/JTAG** peripheral.
  No CP210x/CH340 bridge -- the chip enumerates directly as
  `/dev/ttyACM0` (VID:PID `303a:1001`).
- User LED: GPIO21, active-LOW.

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

If the chip is not in download mode and flashing fails, hold **BOOT**, tap
**RESET**, release **BOOT**, and retry.

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
    main.c              app_main()
```
