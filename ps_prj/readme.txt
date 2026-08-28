
#!/usr/bin/env python3
"""
reg_rw_test.py -- write-then-read integrity test for a PCIe BAR register.

Writes a random 16-bit value to a register, reads it back, and verifies
the value matches. Repeats in a loop, printing a summary line per iteration.
Designed to verify the AXI4-Lite -> Wishbone bridge is working correctly
on real hardware after the simulation was validated.

Target register: REG_DSP_ALIVE (syscon offset 1, byte address 0x0002)
This is the cleanest r/w register in syscon -- no side effects, no
inversion, direct write/read of dsp_alive_reg.

Usage:
    sudo python3 reg_rw_test.py /sys/bus/pci/devices/0000:01:00.0/resource0 [iterations] [delay_s]

    iterations  -- number of write/read cycles (default: 200, 0 = infinite)
    delay_s     -- seconds between iterations (default: 0.01)
"""

import mmap
import os
import struct
import sys
import signal
import time
import random
from datetime import datetime

# Byte address of DSP_ALIVE (alias register 0x0040, 16-bit -> byte offset 0x0080)
DEFAULT_REG_BYTE_OFFSET = 0x0080

pass_count    = 0
fail_count    = 0
sigbus_count  = 0
total_count   = 0

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
    mm.seek(offset)
    return struct.unpack('<H', mm.read(2))[0]

def write_reg(mm, offset, value):
    mm.seek(offset)
    mm.write(struct.pack('<H', value))

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

    if len(sys.argv) < 2:
        print(f"Usage: {sys.argv[0]} <resource_path> [iterations] [delay_s]")
        print(f"  resource_path : e.g. /sys/bus/pci/devices/0000:01:00.0/resource0")
        print(f"  iterations    : number of cycles, 0 = infinite (default: 200)")
        print(f"  delay_s       : pause between cycles in seconds (default: 0.01)")
        sys.exit(1)

    resource_path = sys.argv[1]
    iterations    = int(sys.argv[2])   if len(sys.argv) > 2 else 200
    delay         = float(sys.argv[3]) if len(sys.argv) > 3 else 0.01
    offset        = DEFAULT_REG_BYTE_OFFSET
    infinite      = (iterations == 0)

    signal.signal(signal.SIGBUS, sigbus_handler)
    enable_device(resource_path)

    fd       = os.open(resource_path, os.O_RDWR | os.O_SYNC)
    bar_size = os.fstat(fd).st_size or 4096
    mm       = mmap.mmap(fd, bar_size, mmap.MAP_SHARED,
                         mmap.PROT_READ | mmap.PROT_WRITE)

    print(f"BAR      : {resource_path}")
    print(f"Offset   : 0x{offset:04x}  (DSP_ALIVE, alias 0x0040)")
    print(f"Cycles   : {'infinite' if infinite else iterations}")
    print(f"Delay    : {delay}s")
    print()
    print(f"{'#':>6}  {'Written':>8}  {'Read':>8}  {'Result'}")
    print("-" * 40)

    start_time = time.monotonic()
    i = 0

    try:
        while infinite or i < iterations:
            i += 1
            total_count = i

            # Save original value before overwriting
            original = read_reg(mm, offset)

            # Write random 16-bit value (avoid 0x0000 to distinguish from
            # reset state, and avoid 0xFFFF which indicates a PCIe error)
            wr_val = random.randint(1, 0xFFFE)
            write_reg(mm, offset, wr_val)

            # Small pause to let the write commit through the bridge --
            # in practice the PCIe posted write + WB cycle completes well
            # within a millisecond, but a tiny sleep makes the test robust
            # against any unexpected latency on the host path
            time.sleep(0.001)

            rd_val = read_reg(mm, offset)

            ok = (rd_val == wr_val)
            if ok:
                pass_count += 1
                result = "OK"
            else:
                fail_count += 1
                result = f"FAIL  (expected 0x{wr_val:04x}, got 0x{rd_val:04x})"

            print(f"{i:>6}  0x{wr_val:04x}    0x{rd_val:04x}    {result}")

            # Restore original value so the register state is predictable
            # across repeated runs
            write_reg(mm, offset, original)

            if fail_count >= 10:
                print("\nStopping after 10 consecutive-or-total failures.")
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
