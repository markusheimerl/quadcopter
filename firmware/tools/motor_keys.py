#!/usr/bin/env python3
"""Send single-key motor commands over BLE (Nordic UART Service) to the
quadcopter.

Requirements:
    pip install bleak

Usage:
    ./tools/motor_keys.py            # auto-scan for "QuadFW"
    ./tools/motor_keys.py <MAC>      # connect to a specific address

Keys:
    a     ARM (refused unless drone is roughly level)
    d/s/0 DISARM (also zeroes throttle)
    w/+/= throttle UP   (step 5, max 220)
    x/-   throttle DOWN (step 5, min 0)
    1..4  test motor 1..4 (200 ms, disarmed only)
    q     quit client
"""
import asyncio
import sys
import termios
import tty

from bleak import BleakClient, BleakScanner

DEVICE_NAME = "QuadFW"
NUS_RX_UUID = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"


async def find_device(address_or_name):
    if address_or_name and ":" in address_or_name:
        return address_or_name
    target = address_or_name or DEVICE_NAME
    print(f"scanning for {target}...")
    dev = await BleakScanner.find_device_by_name(target, timeout=10.0)
    if dev is None:
        print(f"could not find BLE device named {target!r}")
        sys.exit(1)
    return dev


async def keyboard_loop(client):
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    print("connected. keys: a arm | d/s disarm | w/x throttle | 1-4 test | q quit")
    allowed = "1234sad0wx+-="
    try:
        tty.setcbreak(fd)
        loop = asyncio.get_event_loop()
        while True:
            ch = await loop.run_in_executor(None, sys.stdin.read, 1)
            if ch == "q":
                return
            if ch in allowed:
                await client.write_gatt_char(
                    NUS_RX_UUID, ch.encode(), response=False)
                print(f"-> {ch}")
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


async def main():
    arg = sys.argv[1] if len(sys.argv) > 1 else None
    target = await find_device(arg)
    async with BleakClient(target) as client:
        await keyboard_loop(client)


if __name__ == "__main__":
    asyncio.run(main())
