
#!/usr/bin/env python3
"""
find_active_reg.py — Scan a PCIe BAR for non-0xFFFF registers.
Saves results to bar_scan_<timestamp>.csv.
On 0xFFFF read, runs pci_reset.sh automatically and re-opens the BAR.

Usage:
  sudo python3 find_active_reg.py <resource0_path> [start_offset] [end_offset] [step]

Defaults:
  start_offset = 0x0000
  end_offset   = 0x8000  (full 32KB)
  step         = 0x0002  (16-bit word stride)

Examples:
  sudo python3 find_active_reg.py /sys/bus/pci/devices/0000:01:00.0/resource0
  sudo python3 find_active_reg.py /sys/bus/pci/devices/0000:01:00.0/resource0 0x0800 0x0A00
"""

import sys
import mmap
import os
import signal
import time
import csv
from datetime import datetime

# --- SIGBUS handler ---
bus_error = False

def sigbus_handler(signum, frame):
    global bus_error
    bus_error = True

signal.signal(signal.SIGBUS, sigbus_handler)

# --- Args ---
if len(sys.argv) < 2:
    print(__doc__)
    sys.exit(1)

resource_path = sys.argv[1]
start_offset  = int(sys.argv[2], 0) if len(sys.argv) > 2 else 0x0000
end_offset    = int(sys.argv[3], 0) if len(sys.argv) > 3 else 0x8000
step          = int(sys.argv[4], 0) if len(sys.argv) > 4 else 0x0002

SCRIPT_DIR       = os.path.dirname(os.path.abspath(__file__))
PCI_RESET_SCRIPT = os.path.join(SCRIPT_DIR, "pci_reset.sh")
OUTPUT_CSV       = os.path.join(SCRIPT_DIR, f"bar_scan_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv")

# --- Open BAR ---
fd = bar = None

def open_bar():
    global fd, bar, bar_size
    fd = os.open(resource_path, os.O_RDWR | os.O_SYNC)
    bar_size = os.fstat(fd).st_size
    if bar_size == 0:
        bar_size = 0x10000
    bar = mmap.mmap(fd, bar_size, mmap.MAP_SHARED,
                    mmap.PROT_READ | mmap.PROT_WRITE)

try:
    open_bar()
except PermissionError:
    print("ERROR: run with sudo")
    sys.exit(1)
except Exception as e:
    print(f"ERROR opening {resource_path}: {e}")
    sys.exit(1)

# --- PCI reset ---
def pci_reset():
    print(f"\n  [!] 0xFFFF detected — running {PCI_RESET_SCRIPT} ...")
    if not os.path.isfile(PCI_RESET_SCRIPT):
        print(f"  [!] {PCI_RESET_SCRIPT} not found — aborting.")
        sys.exit(1)
    ret = os.system(f"sudo bash {PCI_RESET_SCRIPT}")
    if ret != 0:
        print(f"  [!] pci_reset.sh failed (code {ret}) — aborting.")
        sys.exit(1)
    print("  [!] PCIe reset OK, waiting 1s then re-opening BAR ...")
    time.sleep(1.0)
    try:
        bar.close()
        os.close(fd)
        open_bar()
        print("  [!] BAR re-opened OK\n")
    except Exception as e:
        print(f"  [!] ERROR re-opening BAR: {e}")
        sys.exit(1)

# --- Register access ---
def read_reg(offset):
    global bus_error
    bus_error = False
    try:
        bar.seek(offset)
        raw = bar.read(2)
    except Exception:
        return None
    if bus_error:
        return None
    val = int.from_bytes(raw, byteorder='little')
    if val == 0xFFFF:
        pci_reset()
        return None
    return val

def write_reg(offset, value):
    global bus_error
    bus_error = False
    try:
        bar.seek(offset)
        bar.write(value.to_bytes(2, byteorder='little'))
    except Exception:
        return False
    return not bus_error

# --- Scan ---
print(f"Scanning BAR : {resource_path}")
print(f"Range        : 0x{start_offset:04X} – 0x{end_offset:04X}, step 0x{step:X}")
print(f"Output CSV   : {OUTPUT_CSV}")
print(f"{'Offset':<10} {'Read1':>6} {'Read2':>6}  Type")
print("-" * 55)

results = []   # (offset, v1, v2, tag)

offsets = range(start_offset, end_offset, step)
total   = len(offsets)

for i, offset in enumerate(offsets):
    # Progress every 256 offsets
    if i % 256 == 0:
        pct = i * 100 // total
        print(f"  ... {pct}%  (0x{offset:04X})", flush=True)

    v1 = read_reg(offset)
    if v1 is None:
        continue   # timeout or post-reset skip

    time.sleep(0.05)   # 50ms — alive counter ticks every 1ms

    v2 = read_reg(offset)
    if v2 is None:
        continue

    if v2 != v1:
        delta = (v2 - v1) & 0xFFFF
        tag   = f"INCREMENTING delta={delta}"
        print(f"0x{offset:04X}     0x{v1:04X}  0x{v2:04X}  {tag}")
        results.append((offset, v1, v2, tag))
        continue

    # RW test
    PROBE = 0xA55A
    write_reg(offset, PROBE)
    time.sleep(0.001)
    v3 = read_reg(offset)
    write_reg(offset, v1)   # restore

    if v3 == PROBE:
        tag = "READ-WRITE"
        print(f"0x{offset:04X}     0x{v1:04X}  0x{v2:04X}  {tag}")
        results.append((offset, v1, v2, tag))
    else:
        # Non-zero static read-only — save it too
        if v1 != 0x0000:
            tag = "STATIC-RO"
            print(f"0x{offset:04X}     0x{v1:04X}  0x{v2:04X}  {tag}")
            results.append((offset, v1, v2, tag))
        # Zero static RO — skip (not interesting)

# --- Write CSV ---
with open(OUTPUT_CSV, "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["offset_hex", "read1_hex", "read2_hex", "type"])
    for offset, v1, v2, tag in results:
        w.writerow([f"0x{offset:04X}", f"0x{v1:04X}", f"0x{v2:04X}", tag])

# --- Summary ---
print()
print("=" * 55)
inc  = [r for r in results if "INCREMENTING" in r[3]]
rw   = [r for r in results if r[3] == "READ-WRITE"]
ro   = [r for r in results if r[3] == "STATIC-RO"]

print(f"INCREMENTING  : {len(inc)}")
for r in inc:
    print(f"  0x{r[0]:04X}  {r[1]} → {r[2]}  ({r[3]})")

print(f"READ-WRITE    : {len(rw)}")
for r in rw:
    print(f"  0x{r[0]:04X}  value=0x{r[1]:04X}")

print(f"STATIC RO ≠0  : {len(ro)}")
for r in ro:
    print(f"  0x{r[0]:04X}  value=0x{r[1]:04X}")

print(f"\nResults saved : {OUTPUT_CSV}")

bar.close()
os.close(fd)
