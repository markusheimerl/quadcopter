#!/usr/bin/env python3
"""Send single-key motor commands over BLE (Nordic UART Service) to the
quadcopter.

Requirements:
    pip install bleak

Usage:
    ./tools/motor_keys.py            # auto-scan for "QuadFW"
    ./tools/motor_keys.py <MAC>      # connect to a specific address

Keys:
    a       ARM (refused unless drone is roughly level)
    d/s/0   DISARM (also zeroes throttle)
    w/+/=   throttle UP   (step 5, max 220)
    x/-     throttle DOWN (step 5, min 0)
    arrows  steer: tilt 4 deg that way for 0.7 s (hold the key to keep going)
    i/k/j/l trim forward/back/left/right by 0.5 deg: press toward where it
            should go when it keeps drifting (saved on the drone)
    [ / ]   rate-loop gain down/up x1.25 (fast wobble: down; slow, soft
            or overshooting: up; saved on the drone)
    1..4    test motor M0 front right, M1 back right, M2 back left, M3 front
            left for 1 s (disarmed only; the arms are labelled M0..M3)
    c       redo the level calibration (disarmed, board still)
    ?       status (reset reason, IMU, calibration, battery, trim, gain)
    q       quit client

While connected the client sends a heartbeat byte 'h' every 200 ms. The
drone only arms with a fresh heartbeat and disarms when it stops for 1 s,
so any other BLE UART client must send 'h' at least every 500 ms too.
POSIX terminals only (termios).
"""
import asyncio
import os
import re
import signal
import sys
import termios
import tty

from bleak import BleakClient, BleakScanner

DEVICE_NAME = "QuadFW"
NUS_RX_UUID = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"
NUS_TX_UUID = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"


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


def _make_log_handler():
    """Reassemble incoming BLE chunks into newline-terminated log lines."""
    buf = bytearray()

    def handler(_sender, data: bytearray):
        nonlocal buf
        buf.extend(data)
        while True:
            nl = buf.find(b"\n")
            if nl < 0:
                break
            line = bytes(buf[:nl]).decode("utf-8", errors="replace").rstrip("\r")
            del buf[: nl + 1]
            print(f"<< {line}")
    return handler


# terminal escape sequences (F-keys, keypad, Alt-x): their digits and letters
# are not commands; the arrows (ESC [ A or ESC O A, with or without a
# modifier) become the steering bytes. A sequence never swallows a following
# ESC (Esc then an arrow, or Alt+arrow as ESC ESC [ A). ESC_TAIL is one cut
# off at the end of a read.
ESC = re.compile(rb"\x1b(?:\[[0-?]*[ -/]*[@-~]|O[^\x1b]|[^\x1b])?", re.S)
ESC_TAIL = re.compile(rb"\x1b(?:\[[0-?]*[ -/]*|O)?$")
ARROWS = {b"A": b"^", b"B": b"v", b"C": b">", b"D": b"<"}


def _keys(data):
    """Command bytes from terminal input. Only the arrows make steering
    bytes: typed ^ v < > are dropped."""
    out, pos = [], 0
    for m in ESC.finditer(data):
        seq = m.group()
        out.append(data[pos:m.start()].translate(None, b"^v<>"))
        if seq[1:2] in (b"[", b"O") and len(seq) > 2:
            out.append(ARROWS.get(seq[-1:], b""))
        pos = m.end()
    out.append(data[pos:].translate(None, b"^v<>"))
    return b"".join(out)


async def keyboard_loop(client, lost):
    """Forward keys until 'q', end of input, or the link drops."""
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    loop = asyncio.get_running_loop()
    keys = asyncio.Queue()

    def on_input():
        try:
            data = os.read(fd, 64)   # a whole escape sequence arrives in one read
        except OSError:
            data = b""               # terminal gone: same as end of input
        keys.put_nowait(data)

    loop.add_reader(fd, on_input)
    loop.add_signal_handler(signal.SIGTSTP, lambda: keys.put_nowait(None))   # Ctrl-Z quits (disarms)
    print("connected. keys: a arm | d/s disarm | w/x throttle | arrows steer | ijkl trim | [ ] gain"
          " | 1-4 test M0-M3 | c cal | ? status | q quit")
    allowed = "1234sad0wx+-=c?ijkl[]^v<>"
    pend = b""
    try:
        tty.setcbreak(fd)
        mode = termios.tcgetattr(fd)
        mode[0] &= ~(termios.IXON | termios.IXOFF)   # Ctrl-S must not freeze output (and the heartbeat)
        termios.tcsetattr(fd, termios.TCSANOW, mode)
        key = None
        while True:
            key = key or asyncio.ensure_future(keys.get())
            done, _ = await asyncio.wait({key, lost}, timeout=0.1 if pend else None,
                                         return_when=asyncio.FIRST_COMPLETED)
            if lost in done:
                key.cancel()
                print("LINK LOST: the drone disarms itself (at once, or within 1 s)")
                return
            if not done:                   # nothing completes a lone Esc: drop it
                pend = b""
                continue
            data, key = key.result(), None
            if not data:                   # end of input, or Ctrl-Z
                return
            if pend == b"\x1b" and data[:1] not in (b"[", b"O"):
                pend = b""                 # a lone Esc: Alt+key comes in one read, so this is a new key
            data = pend + data
            tail = ESC_TAIL.search(data)   # an escape sequence split over two reads
            pend = data[tail.start():] if tail else b""
            data = _keys(data[:tail.start()] if tail else data)
            if b"q" in data:               # quit
                return
            for ch in data.decode("ascii", errors="ignore"):
                if ch in allowed:
                    try:
                        await client.write_gatt_char(
                            NUS_RX_UUID, ch.encode(), response=False)
                    except Exception as e:   # link died between key press and write
                        print(f"LINK LOST ({e}): the drone disarms itself within 1 s")
                        return
                    if ch not in "^v<>":   # a held arrow repeats 30 times a second
                        print(f"-> {ch}")
    finally:
        loop.remove_reader(fd)
        loop.remove_signal_handler(signal.SIGTSTP)
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


async def heartbeat(client, lost):
    """Tell the drone we are still here (it disarms after 1 s of silence)."""
    while not lost.done():
        try:
            await client.write_gatt_char(NUS_RX_UUID, b"h", response=False)
        except Exception as e:   # link gone: the drone disarms by itself
            print(f"heartbeat failed ({e}): the drone disarms within 1 s")
            if not lost.done():
                lost.set_result(None)
            return
        await asyncio.sleep(0.2)


async def main():
    arg = sys.argv[1] if len(sys.argv) > 1 else None
    target = await find_device(arg)
    lost = asyncio.get_running_loop().create_future()

    def on_disconnect(_client):
        if not lost.done():
            lost.set_result(None)

    async with BleakClient(target, disconnected_callback=on_disconnect) as client:
        await client.start_notify(NUS_TX_UUID, _make_log_handler())
        beat = asyncio.create_task(heartbeat(client, lost))
        try:
            await keyboard_loop(client, lost)
        finally:
            beat.cancel()
            if not lost.done():
                try:   # disarm before letting go of the link
                    await asyncio.wait_for(
                        client.write_gatt_char(NUS_RX_UUID, b"d", response=True), 2.0)
                    await client.stop_notify(NUS_TX_UUID)
                except Exception:
                    pass


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass
