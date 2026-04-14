#!/usr/bin/env python3
"""
Standalone pyserial workaround for reading Brainco Revo2Touch force sensors
on installations where the bundled libbc_stark_sdk.so version is too old to
expose the touch API.

Problem
-------
The `libbc_stark_sdk.so` shipped in this repo (version 0.4.3) hard-codes the
touch-sensor APIs as "deprecated for current firmware, return empty":

    WARN touch_sensor_setup: deprecated for current firmware
    WARN touch_sensor_calibrate: deprecated for current firmware
    WARN get_touch_sensor_status: deprecated for current firmware, return empty

even on hardware where `modbus_get_device_info` returns
`hardware_type = 4` (STARK_HARDWARE_TYPE_REVO2_TOUCH).  Version 0.8.1 of
the SDK (shipped with the sibling repo `unitreerobotics/brainco_hand_service`)
does expose those APIs and successfully talks to the same hands — the
fix is purely on the SDK side, not the firmware.

This script sidesteps the SDK version mismatch entirely by speaking the
same Modbus RTU commands the newer SDK would send, using only `pyserial`.
It works on any Revo2Touch hand regardless of which version of
`libbc_stark_sdk.so` is installed.

Usage
-----

    pip install pyserial
    sudo chmod 666 /dev/ttyUSB1 /dev/ttyUSB2    # or add user to dialout
    python3 touch_sensor_pyserial.py --port /dev/ttyUSB1 --slave 0x7e

    # or point it at both hands at once
    python3 touch_sensor_pyserial.py --dual

Press a fingertip and watch the per-finger `normal_force` reading go from
~0 to ~2500.

Modbus registers (verified on Revo2Touch firmware 1.0.14.U)
-----------------------------------------------------------

  * Enable    : WRITE holding registers @4000, data = [1, 1, 1, 1, 1]
  * Calibrate : WRITE holding registers @4010, data = [1, 1, 1, 1, 1]
  * Status    : READ  input    registers @4200, count = 30

Register-4200 layout (30 × uint16):

  regs[0..14]  = 3 per finger, interleaved f0..f4 (normal, tangential, tang_dir)
  regs[15..24] = 2 per finger, interleaved f0..f4 (self_proximity lo/hi u32 halves)
  regs[25..29] = per-finger frame counter / status — ignore

Finger order: `f0=thumb, f1=index, f2=middle, f3=ring, f4=pinky`.

The firmware writes 0xFFFF as a "no data" sentinel on idle tangential
fields; this reader masks those to 0 so callers never see a bogus
~65535 as if it were a real force.
"""

import argparse
import struct
import sys
import time

try:
    import serial
except ImportError:
    print("ERROR: pyserial is required.  Install with `pip install pyserial`.")
    sys.exit(1)

# -----------------------------------------------------------------------------
# Modbus RTU helpers (function 0x04 read input regs, 0x10 write multiple regs)
# -----------------------------------------------------------------------------


def _crc16(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc


def read_input_registers(ser, slave_id, addr, count):
    """Function 0x04.  Returns a list of `count` uint16 or None on failure."""
    msg = struct.pack(">BBHH", slave_id, 0x04, addr, count)
    msg += struct.pack("<H", _crc16(msg))
    ser.reset_input_buffer()
    ser.write(msg)
    expected = 3 + count * 2 + 2
    resp = ser.read(expected)
    if len(resp) < expected:
        return None
    if _crc16(resp[:-2]) != struct.unpack("<H", resp[-2:])[0]:
        return None
    return list(struct.unpack(">" + "H" * count, resp[3:3 + count * 2]))


def write_multiple_registers(ser, slave_id, addr, values):
    """Function 0x10.  Returns True on success."""
    count = len(values)
    msg = struct.pack(">BBHHB", slave_id, 0x10, addr, count, count * 2)
    for v in values:
        msg += struct.pack(">H", int(v) & 0xFFFF)
    msg += struct.pack("<H", _crc16(msg))
    ser.reset_input_buffer()
    ser.write(msg)
    resp = ser.read(8)
    if len(resp) < 8:
        return False
    return _crc16(resp[:-2]) == struct.unpack("<H", resp[-2:])[0]


# -----------------------------------------------------------------------------
# Touch sensor bring-up sequence + parser
# -----------------------------------------------------------------------------

REG_TOUCH_ENABLE = 4000
REG_TOUCH_CALIBRATE = 4010
REG_TOUCH_STATUS = 4200
REG_TOUCH_STATUS_COUNT = 30

FINGER_NAMES = ["thumb", "index", "middle", "ring", "pinky"]


def enable_touch(ser, slave_id):
    """Enable + calibrate all 5 touch sensor channels on one hand.

    Returns True on success.  On Revo2Basic hands (hardware_type=3) these
    writes may still echo back valid responses but the hand will only return
    zeros + 0xFFFF sentinels on register 4200 — the caller should probe.
    """
    if not write_multiple_registers(ser, slave_id, REG_TOUCH_ENABLE, [1] * 5):
        return False
    time.sleep(0.3)
    if not write_multiple_registers(ser, slave_id, REG_TOUCH_CALIBRATE, [1] * 5):
        return False
    time.sleep(0.5)
    return True


def parse_touch_regs(regs):
    """Return per-finger (normal, tangential, tang_dir, self_prox_lo, self_prox_hi).

    Each returned list is length 5, indexed thumb..pinky.  0xFFFF sentinels
    are masked to 0 in the force fields.
    """
    def mask(v):
        return 0 if v == 0xFFFF else v
    normal = [mask(regs[3 * i + 0]) for i in range(5)]
    tangential = [mask(regs[3 * i + 1]) for i in range(5)]
    tang_dir = [mask(regs[3 * i + 2]) for i in range(5)]
    prox_lo = [regs[15 + 2 * i + 0] for i in range(5)]
    prox_hi = [regs[15 + 2 * i + 1] for i in range(5)]
    return normal, tangential, tang_dir, prox_lo, prox_hi


def read_touch(ser, slave_id):
    regs = read_input_registers(
        ser, slave_id, REG_TOUCH_STATUS, REG_TOUCH_STATUS_COUNT
    )
    if regs is None:
        return None
    return parse_touch_regs(regs)


# -----------------------------------------------------------------------------
# Demo main
# -----------------------------------------------------------------------------


def _open_and_enable(port, baud, slave_id, label):
    print(f"==> Opening {port} slave=0x{slave_id:02x} ({label})", flush=True)
    ser = serial.Serial(port, baud, timeout=0.3)
    if not enable_touch(ser, slave_id):
        print(f"    WARN: touch enable write failed on {label}", flush=True)
    else:
        print(f"    touch enable + calibrate: ok", flush=True)
    return ser


def _print_row(label, parsed):
    normal, tangential, _tdir, _plo, _phi = parsed
    cells = []
    for i, name in enumerate(FINGER_NAMES):
        marker = "*" if normal[i] > 500 else " "
        cells.append(f"{marker}{name[0]}={normal[i]:4d}")
    print(f"  {label}: " + " ".join(cells), flush=True)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--port", default="/dev/ttyUSB1", help="serial port (default /dev/ttyUSB1)")
    p.add_argument("--slave", type=lambda x: int(x, 0), default=0x7e,
                   help="Modbus slave id (left=0x7e, right=0x7f; default 0x7e)")
    p.add_argument("--baud", type=int, default=460800,
                   help="serial baudrate (default 460800)")
    p.add_argument("--dual", action="store_true",
                   help="poll both hands (/dev/ttyUSB1 + /dev/ttyUSB2)")
    p.add_argument("--seconds", type=float, default=30.0,
                   help="how long to poll (default 30)")
    p.add_argument("--rate", type=float, default=10.0,
                   help="poll rate in Hz (default 10)")
    args = p.parse_args()

    if args.dual:
        left = _open_and_enable("/dev/ttyUSB1", args.baud, 0x7e, "LEFT")
        right = _open_and_enable("/dev/ttyUSB2", args.baud, 0x7f, "RIGHT")
        handles = [("LEFT ", left, 0x7e), ("RIGHT", right, 0x7f)]
    else:
        ser = _open_and_enable(args.port, args.baud, args.slave, "HAND")
        handles = [("HAND ", ser, args.slave)]

    print("", flush=True)
    print(f"Polling for {args.seconds:.0f} seconds — press fingertips to see values.",
          flush=True)
    print("Format: finger=normal_force; '*' marks >500 (firm contact).", flush=True)
    print("", flush=True)

    t_end = time.time() + args.seconds
    interval = 1.0 / args.rate
    while time.time() < t_end:
        for label, ser, sid in handles:
            parsed = read_touch(ser, sid)
            if parsed is not None:
                _print_row(label, parsed)
            else:
                print(f"  {label}: (read failed)", flush=True)
        time.sleep(interval)

    for _, ser, _ in handles:
        ser.close()


if __name__ == "__main__":
    main()
