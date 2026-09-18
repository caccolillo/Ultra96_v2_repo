#!/usr/bin/env python3
"""
dump_syscon.py — Full SYSCON register dump over PCIe BAR.

Usage:
  sudo python3 dump_syscon.py [resource0_path]

Default: /sys/bus/pci/devices/0000:01:00.0/resource0
"""

import os
import mmap
import sys
import time

RESOURCE = sys.argv[1] if len(sys.argv) > 1 else \
           "/sys/bus/pci/devices/0000:01:00.0/resource0"

# SYSCON base address in BAR (WB_STB_SYS_CON=4, WB_BLOCK_ADR_WIDTH=8)
# BAR byte offset = 4 * 256 * 2 = 0x0800
SYSCON_BASE = 0x0800

# Register map from syscon.vhd constants
REGS = [
    (0,  "ID_REV",             "Wishbone block revision ID"),
    (1,  "DSP_ALIVE",          "DSP alive counter (written by DSP)"),
    (2,  "ARM_ALIVE",          "ARM alive counter (written by ARM)"),
    (3,  "FPGA_ALIVE_DSP",     "FPGA alive counter (read by DSP)"),
    (4,  "FPGA_ALIVE_ARM",     "FPGA alive counter (read by ARM)"),
    (5,  "DISABLE_FP_CLK",     "Disable front panel clock output"),
    (6,  "DISABLE_CODEC_CLK",  "Disable codec clock output"),
    (7,  "INIT_COMPLETE",      "Initialisation complete flag"),
    (8,  "PLL_LOCK",           "PLL lock pin state"),
    (9,  "ACTIVE_OUTPUT",      "Active output pin state"),
    (10, "SCM_VERSION_HIGH",   "SCM firmware version (high word)"),
    (11, "SCM_VERSION_LOW",    "SCM firmware version (low word)"),
    (12, "BUILD_DATE_DDMM",    "Build date (DD/MM)"),
    (13, "BUILD_DATE_YYYY",    "Build date (year)"),
    (14, "FULL_SAMPLE_RATE",   "Full sample rate flag (1=168ksps, 0=84ksps)"),
    (15, "SW_RST",             "Software reset register"),
]

# Open BAR
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

def bar_offset(reg_num):
    return SYSCON_BASE + reg_num * 2

# Header
print(f"Resource : {RESOURCE}")
print(f"SYSCON base: 0x{SYSCON_BASE:04X}")
print(f"Timestamp  : {time.strftime('%Y-%m-%d %H:%M:%S')}")
print()
print(f"{'Register':<24} {'Offset':>6}  {'Value':>6}  {'Dec':>6}  Description")
print("-" * 80)

no_ack_count = 0
for reg_num, name, desc in REGS:
    offset = bar_offset(reg_num)
    val    = read16(offset)
    if val == 0xFFFF:
        flag = "  <-- NO ACK"
        no_ack_count += 1
    else:
        flag = ""
    print(f"{name:<24} 0x{offset:04X}   0x{val:04X}  {val:>6}  {desc}{flag}")

# FPGA alive counter check
print()
print(f"Checking FPGA_ALIVE_ARM counter (waiting 2s) ...")
alive_offset = bar_offset(4)
v1 = read16(alive_offset)
time.sleep(2.0)
v2 = read16(alive_offset)
print(f"  t=0s  : 0x{v1:04X} ({v1})")
print(f"  t=2.0s: 0x{v2:04X} ({v2})")
if v1 == 0xFFFF or v2 == 0xFFFF:
    print("  RESULT: no ACK — bridge not responding")
elif v2 > v1:
    print(f"  RESULT: PASS — counter incremented by {v2 - v1}")
elif v2 == v1:
    print(f"  RESULT: FAIL — counter static ({v1}) — SYSCON not running")
else:
    print(f"  RESULT: WARN — counter wrapped ({v1} -> {v2})")

# Build version string
print()
ver_hi = read16(bar_offset(10))
ver_lo = read16(bar_offset(11))
ddmm   = read16(bar_offset(12))
yyyy   = read16(bar_offset(13))
if ver_hi != 0xFFFF and ver_lo != 0xFFFF:
    version = (ver_hi << 16) | ver_lo
    # Date registers use BCD encoding
    dd   = ((ddmm >> 12) & 0xF) * 10 + ((ddmm >> 8) & 0xF)
    mm   = ((ddmm >> 4) & 0xF) * 10 + (ddmm & 0xF)
    yy_h = ((yyyy >> 12) & 0xF) * 10 + ((yyyy >> 8) & 0xF)
    yy_l = ((yyyy >> 4) & 0xF) * 10 + (yyyy & 0xF)
    year = yy_h * 100 + yy_l
    print(f"Firmware version : 0x{version:08X}")
    if year > 0:
        print(f"Build date       : {dd:02d}/{mm:02d}/{year:04d}")
    else:
        print(f"Build date       : {dd:02d}/{mm:02d} (year not set)")

if no_ack_count > 0:
    print(f"\nWARNING: {no_ack_count} register(s) returned 0xFFFF (no ACK)")

bar.close()
os.close(fd)
