#!/usr/bin/env python3
"""
check_syscon.py — Read SYSCON registers and check FPGA alive counter.

Usage:
  sudo python3 check_syscon.py [resource0_path]

Default resource0: /sys/bus/pci/devices/0000:01:00.0/resource0
"""

import os
import mmap
import time
import sys

RESOURCE = sys.argv[1] if len(sys.argv) > 1 else \
           "/sys/bus/pci/devices/0000:01:00.0/resource0"

SYSCON_REGS = [
    (0x0800, "SYSCON_ID        "),
    (0x0802, "DSP_ALIVE        "),
    (0x0804, "ARM_ALIVE        "),
    (0x0806, "SYSCON_STATUS    "),
    (0x0808, "FPGA_ALIVE       "),
    (0x080A, "SYSCON_REG5      "),
    (0x080C, "SYSCON_REG6      "),
    (0x080E, "SYSCON_REG7      "),
]

ALIVE_OFFSET = 0x0808
ALIVE_WAIT   = 2.0  # seconds between two alive reads

try:
    fd  = os.open(RESOURCE, os.O_RDWR | os.O_SYNC)
    bar = mmap.mmap(fd, 65536, mmap.MAP_SHARED,
                    mmap.PROT_READ | mmap.PROT_WRITE)
except PermissionError:
    print("ERROR: run with sudo")
    sys.exit(1)
except Exception as e:
    print(f"ERROR: {e}")
    sys.exit(1)

def read16(offset):
    bar.seek(offset)
    return int.from_bytes(bar.read(2), byteorder='little')

def write16(offset, value):
    bar.seek(offset)
    bar.write(value.to_bytes(2, byteorder='little'))

# --- Static register dump ---
print(f"Resource : {RESOURCE}")
print(f"{'Register':<20} {'Offset':>6}  {'Value':>6}")
print("-" * 40)
for offset, name in SYSCON_REGS:
    val = read16(offset)
    flag = " <-- 0xFFFF (no ACK)" if val == 0xFFFF else ""
    print(f"{name} 0x{offset:04X}   0x{val:04X}{flag}")

# --- FPGA alive counter check ---
print(f"\nChecking FPGA_ALIVE counter (waiting {ALIVE_WAIT}s) ...")
v1 = read16(ALIVE_OFFSET)
time.sleep(ALIVE_WAIT)
v2 = read16(ALIVE_OFFSET)

print(f"  t=0s : 0x{v1:04X} ({v1})")
print(f"  t={ALIVE_WAIT}s : 0x{v2:04X} ({v2})")

if v1 == 0xFFFF or v2 == 0xFFFF:
    print("  RESULT: no ACK — bridge not responding")
elif v2 > v1:
    print(f"  RESULT: PASS — counter incremented by {v2 - v1}")
elif v2 == v1:
    print(f"  RESULT: FAIL — counter static ({v1}) — SYSCON not running or reset stuck")
else:
    print(f"  RESULT: WARN — counter went backwards ({v1} -> {v2}) — possible wrap")

# --- ARM_ALIVE write/readback ---
print(f"\nWrite/readback test on ARM_ALIVE (0x0804) ...")
PROBE = 0xA55A
write16(0x0804, PROBE)
time.sleep(0.01)
rb = read16(0x0804)
if rb == PROBE:
    print(f"  RESULT: PASS — readback 0x{rb:04X}")
elif rb == 0xFFFF:
    print(f"  RESULT: no ACK — bridge not responding")
else:
    print(f"  RESULT: FAIL — readback 0x{rb:04X} (expected 0x{PROBE:04X})")

bar.close()
os.close(fd)
