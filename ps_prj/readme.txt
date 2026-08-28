#!/usr/bin/env python3
"""
reg_rw_test.py -- write-then-read integrity test for a PCIe BAR register.

Writes a random 16-bit value to a register, reads it back, and verifies
the value matches. Repeats in a loop, printing a summary line per iteration.

Usage:
    sudo python3 reg_rw_test.py <resource_path> <byte_offset_hex> [iterations] [delay_s]

    resource_path    : e.g. /sys/bus/pci/devices/0000:01:00.0/resource0
    byte_offset_hex  : byte address of the register, e.g. 0x0080
    iterations       : number of write/read cycles (default: 200, 0 = infinite)
    delay_s          : pause between iterations in seconds (default: 0.01)

Examples:
    # DSP_ALIVE alias 0x0040, byte offset 0x0080
    sudo python3 reg_rw_test.py /sys/bus/pci/devices/0000:01:00.0/resource0 0x0080

    # FPGA_ALIVE alias 0x0000, byte offset 0x0000 (read-only, will always fail write-back)
    sudo python3 reg_rw_test.py /sys/bus/pci/devices/0000:01:00.0/resource0 0x0000

    # Infinite loop, no delay
    sudo python3 reg_rw_test.py /sys/bus/pci/devices/0000:01:00.0/resource0 0x0080 0 0
"""

import mmap
import os
import struct
import sys
import signal
import time
import random
import ctypes
from datetime import datetime

pass_count   = 0
fail_count   = 0
sigbus_count = 0
total_count  = 0
start_time   = 0.0

def sigbus_handler(signum, frame):
    global sigbus_count
    sigbus_count += 1
    print(f"\n[{datetime.now().isoformat()}] *** SIGBUS -- MMIO fault on iteration {total_count} ***")
    print("PCIe transaction did not complete. Bridge hung or hardware fault.")
    _print_summary()
    sys.exit(1)

def enable_device(resource_path):
    enable_path = os.path.join(os.path.dirname(resource_path), "enable")
    try:
        with open(enable_path, "w") as f:
            f.write("1")
        print(f"Device enabled via {enable_path}")
    except OSError as e:
        print(f"WARNING: could not enable device ({e}) -- continuing anyway")

def read_reg(mm, offset):
    addr = ctypes.addressof(ctypes.c_char.from_buffer(mm)) + offset
    return ctypes.cast(addr, ctypes.POINTER(ctypes.c_uint16))[0]

def write_reg(mm, offset, value):
    addr = ctypes.addressof(ctypes.c_char.from_buffer(mm)) + offset
    ctypes.cast(addr, ctypes.POINTER(ctypes.c_uint16))[0] = value

def _print_summary():
    elapsed = time.monotonic() - start_time
    print(f"\n{'='*60}")
    print(f"  Iterations : {total_count}")
    print(f"  PASS       : {pass_count}  ({100*pass_count/max(total_count,1):.1f}%)")
    print(f"  FAIL       : {fail_count}")
    print(f"  SIGBUS     : {sigbus_count}")
    print(f"  Elapsed    : {elapsed:.1f}s")
    print(f"{'='*60}")

def main():
    global pass_count, fail_count, total_count, start_time

    if len(sys.argv) < 3:
        print(__doc__)
        sys.exit(1)

    resource_path = sys.argv[1]
    offset        = int(sys.argv[2], 16)
    iterations    = int(sys.argv[3])   if len(sys.argv) > 3 else 200
    delay         = float(sys.argv[4]) if len(sys.argv) > 4 else 0.01
    infinite      = (iterations == 0)

    signal.signal(signal.SIGBUS, sigbus_handler)
    enable_device(resource_path)

    fd       = os.open(resource_path, os.O_RDWR | os.O_SYNC)
    bar_size = os.fstat(fd).st_size or 4096
    mm       = mmap.mmap(fd, bar_size, mmap.MAP_SHARED,
                         mmap.PROT_READ | mmap.PROT_WRITE)

    print(f"BAR      : {resource_path}")
    print(f"Offset   : 0x{offset:04x}")
    print(f"Cycles   : {'infinite' if infinite else iterations}")
    print(f"Delay    : {delay}s")
    print()
    print(f"{'#':>6}  {'Written':>8}  {'Read':>8}  {'Result'}")
    print("-" * 50)

    start_time = time.monotonic()
    i = 0

    try:
        while infinite or i < iterations:
            i += 1
            total_count = i

            original = read_reg(mm, offset)

            # Random 16-bit value, avoid 0x0000 and 0xFFFF
            wr_val = random.randint(1, 0xFFFE)
            write_reg(mm, offset, wr_val)

            time.sleep(0.001)  # let the write commit through the bridge

            rd_val = read_reg(mm, offset)

            ok = (rd_val == wr_val)
            if ok:
                pass_count += 1
                result = "OK"
            else:
                fail_count += 1
                result = f"FAIL  (expected 0x{wr_val:04x}, got 0x{rd_val:04x})"

            print(f"{i:>6}  0x{wr_val:04x}    0x{rd_val:04x}    {result}")

            # Restore original value
            write_reg(mm, offset, original)

            if fail_count >= 10:
                print("\nStopping after 10 failures.")
                break

            if delay > 0:
                time.sleep(delay)

    except KeyboardInterrupt:
        print("\nInterrupted by user.")
    finally:
        mm.close()
        os.close(fd)
        _print_summary()

if __name__ == "__main__":
    main()
