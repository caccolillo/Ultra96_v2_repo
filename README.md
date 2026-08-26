#!/usr/bin/env python3
"""
poll_reg.py -- repeatedly read a single 32-bit register through a PCIe
BAR (via sysfs mmap), printing each value with a timestamp and delta
from the previous read. Built to watch the offset-0 millisecond timer
and catch whatever anomaly produced the 0xFFFFFFFF readback earlier.

Usage:
    sudo python3 poll_reg.py /sys/bus/pci/devices/0000:01:00.0/resource0 0x0 [interval_seconds]

Ctrl+C to stop cleanly.

IMPORTANT: a failed MMIO access (e.g. the device returning a completion
error, or -- worse -- a transaction that never completes and trips a
host-side fault) delivers SIGBUS directly to this process on Linux; it
does NOT raise a normal Python exception from the mm[...] access. Without
a handler, the script would just die silently with no diagnostic at all,
which is exactly what makes these failures hard to pin down. The handler
below at least prints what it can before exiting -- but note that
continuing the loop after a real SIGBUS on the mapped page is not safe,
so it exits cleanly rather than trying to resume.
"""

import mmap
import os
import struct
import sys
import signal
import time
from datetime import datetime

def sigbus_handler(signum, frame):
    print(f"\n[{datetime.now().isoformat()}] *** SIGBUS caught -- MMIO access faulted. ***")
    print("This means the PCIe transaction did not complete cleanly (error")
    print("completion, or a fault at the host/kernel level). Exiting rather")
    print("than continuing, since the mapping may now be in an undefined state.")
    sys.stdout.flush()
    os._exit(1)  # os._exit, not sys.exit -- avoids running cleanup that
                 # could itself touch the now-suspect mapping

def enable_device(resource_path):
    """
    Writes '1' to the device's sysfs 'enable' attribute (Memory Space
    Enable in the PCI Command register) before touching the BAR. This is
    normally done automatically by a bound kernel driver's probe()
    function -- since we're accessing the device raw via sysfs with no
    driver bound, nothing does this for us, and it resets to whatever the
    firmware leaves it as on every fresh enumeration (reboot, rescan,
    board power cycle).
    """
    device_dir = os.path.dirname(resource_path)
    enable_path = os.path.join(device_dir, "enable")
    try:
        with open(enable_path, "w") as f:
            f.write("1")
        print(f"Enabled device via {enable_path}")
    except OSError as e:
        print(f"WARNING: could not write to {enable_path}: {e}")
        print("Continuing anyway -- if the device was already enabled this is harmless;")
        print("if not, the mmap/read below will likely fail.")


def main():
    if len(sys.argv) < 3:
        print(f"Usage: {sys.argv[0]} <resource_path> <offset_hex> [interval_seconds]")
        print(f"Example: {sys.argv[0]} /sys/bus/pci/devices/0000:01:00.0/resource0 0x0 0.5")
        sys.exit(1)

    resource_path = sys.argv[1]
    offset = int(sys.argv[2], 16)
    interval = float(sys.argv[3]) if len(sys.argv) > 3 else 0.5

    signal.signal(signal.SIGBUS, sigbus_handler)

    enable_device(resource_path)

    fd = os.open(resource_path, os.O_RDWR | os.O_SYNC)
    mm = mmap.mmap(fd, 4096)

    print(f"Polling offset 0x{offset:x} on {resource_path} every {interval}s. Ctrl+C to stop.\n")
    print(f"{'timestamp':<28} {'hex':<12} {'decimal':<12} {'delta':<12} {'note'}")

    prev_val = None
    read_count = 0
    suspicious_count = 0

    try:
        while True:
            val = struct.unpack('<I', mm[offset:offset+4])[0]
            read_count += 1
            ts = datetime.now().isoformat(timespec='milliseconds')

            delta_str = ""
            note = ""

            if prev_val is not None:
                delta = val - prev_val
                delta_str = str(delta)
                if val == 0xFFFFFFFF:
                    note = "*** SUSPICIOUS: all-Fs, likely a failed completion, not real counter data ***"
                    suspicious_count += 1
                elif delta < 0:
                    note = "*** counter went backwards -- unexpected ***"
                elif delta == 0:
                    note = "(unchanged since last read)"
            else:
                if val == 0xFFFFFFFF:
                    note = "*** SUSPICIOUS: all-Fs on the very first read ***"
                    suspicious_count += 1

            print(f"{ts:<28} 0x{val:08x}   {val:<12} {delta_str:<12} {note}")

            prev_val = val
            time.sleep(interval)

    except KeyboardInterrupt:
        print(f"\nStopped. {read_count} reads total, {suspicious_count} suspicious (0xFFFFFFFF).")

    finally:
        mm.close()
        os.close(fd)

if __name__ == "__main__":
    main()
