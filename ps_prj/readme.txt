#!/usr/bin/env python3
# =============================================================================
# File: a53_rx_dma_init.py
# Project: PLAT-150 / S0841D PCIe IQ Streaming Interface
# Platform: TI AM6442 (Dual Cortex-A53 Linux + Dual Cortex-R5F Zephyr RTOS)
#
# A53 one-off start-up for the RX (C2H0) path.
# Register map, descriptor format and RX packing are taken from
# rx_optionA_verify.py. Structure follows a53_dma_init.py (TX).
#
# A53 (this script):
#   1. Checks the u-dma-buf region (below 4 GB, large enough, optional fixed
#      base address shared with the R5F).
#   2. Checks PCI memory space and bus mastering, and the FPGA identifiers.
#   3. Stops the RX path and C2H0 cleanly. TX bits in REG_CTRL are preserved.
#   4. Sets the sample rate.
#   5. Builds a looping C2H0 descriptor ring: rx_bram ping-pong -> DDR slots.
#   6. Programs C2H0: poll-mode writeback, descriptor credit mode, run.
#   7. Enables RX mover, RX select, test pattern (optional) and pacer.
#   8. Checks that frames are flowing at the expected rate.
#   9. Prints the R5F handover parameters.
#  10. Optional: --verify N (A53-side LUT check of live frames), --monitor.
#
# R5F (Zephyr, per frame):
#   - Never reads or writes BAR0/BAR1.
#   - Works only on the DDR slots and the writeback word.
#
# ASSUMPTIONS TO CONFIRM AGAINST iq_dma_regs.vhd:
#   - CTRL bit 3 (PACER_EN) is the same bit in the TX and RX scripts. It is
#     treated here as shared: it is only cleared if TX_EN is also clear.
#   - REG_RX_OVERRUN clear semantics are unknown, so it is never written.
#     A baseline is recorded instead.
#   - The FPGA RX mover fills rx_bram slot 0 first after RX_EN, so DDR
#     descriptor k reads rx_bram slot (k % 2). The test pattern cannot
#     detect a violation of this, because every frame is identical.
# =============================================================================

import argparse
import ast
import os
import struct
import sys
import time
import mmap

# -----------------------------------------------------------------------------
# 1. PCIe BAR MEMORY MAP
# -----------------------------------------------------------------------------
BAR_SIZE = 0x10000

# BAR0: Wishbone registers (16-bit)
SYSCON_ID_REV           = 0x0800   # Read-only, expect 0x0004
SYSCON_FULL_SAMPLE_RATE = 0x081C   # Bit 0: 1 = 168 ksps, 0 = 84 ksps
SYSCON_SW_RST           = 0x081E   # NEVER write

IQ_DMA_BASE     = 0x1E00           # Strobe 15 (iq_dma_regs.vhd)
REG_ID          = IQ_DMA_BASE + 0x00   # Expect 0xDA02 (Option A)
REG_CTRL        = IQ_DMA_BASE + 0x02
REG_RX_OVERRUN  = IQ_DMA_BASE + 0x0E
REG_PACER_STS   = IQ_DMA_BASE + 0x16
REG_RX_DONE_CNT = IQ_DMA_BASE + 0x1C   # 16-bit running count

CTRL_TX_EN    = 1 << 0
CTRL_FWD_SEL  = 1 << 2
CTRL_PACER_EN = 1 << 3
CTRL_RX_EN    = 1 << 4
CTRL_RX_SEL   = 1 << 5
CTRL_RX_PAT   = 1 << 6
CTRL_RX_MASK  = CTRL_RX_EN | CTRL_RX_SEL | CTRL_RX_PAT

EXPECT_SYSCON_ID = 0x0004
EXPECT_DMA_ID    = 0xDA02

# BAR1: XDMA registers (32-bit, PG195)
C2H0         = 0x1000
CH_ID        = 0x00
CH_CTRL      = 0x04
CH_CTRL_W1C  = 0x0C
CH_STATUS    = 0x40
CH_STATUS_RC = 0x44
CH_COMPL     = 0x48
CH_WB_LO     = 0x88
CH_WB_HI     = 0x8C

CTRL_RUN     = 1 << 0
CTRL_POLL_WB = 1 << 26
STATUS_BUSY  = 1 << 0

SGDMA_C2H0 = 0x5000
SG_DESC_LO  = 0x80
SG_DESC_HI  = 0x84
SG_DESC_ADJ = 0x88

SGDMA_COMMON     = 0x6000
CREDIT_MODE_W1S  = 0x24
CREDIT_MODE_W1C  = 0x28
CREDIT_BIT_C2H0  = 1 << 16

# -----------------------------------------------------------------------------
# 2. DESCRIPTOR FORMAT, RX GEOMETRY, DDR LAYOUT
# -----------------------------------------------------------------------------
DESC_MAGIC     = 0xAD4B
DESC_COMPLETED = 1 << 1
DESC_EOP       = 1 << 4
DESC_BYTES     = 32

RX_BRAM_BASE   = 0x00010000
RX_BRAM_SLOTS  = 2
RX_FRAME_BYTES = 256            # 32 x 64-bit packed I/Q words
RX_SAMPLES     = RX_FRAME_BYTES // 8
LUT_LEN        = 32

# Shared 1 MiB region reserved in Linux, also used by TX (a53_dma_init.py).
# TX occupies 0x00000..0x02FFF (desc 0x0, WB 0x800, data 0x1000, max 64 x 128 B).
# RX sits in its own 64 KiB window so both paths can run together.
R5F_FPGA_DMA_BASE_ADDR = 0xA5800000
REGION_SIZE            = 0x100000
RX_REGION_OFF          = 0x10000

DESC_OFF  = RX_REGION_OFF + 0x0000
WB_OFF    = RX_REGION_OFF + 0x0800  # 64 descriptors x 32 B fit exactly below this
DATA_OFF  = RX_REGION_OFF + 0x1000  # max 64 x 256 B = 16 KiB
MAX_CHAIN = (WB_OFF - DESC_OFF) // DESC_BYTES

WB_ERR        = 1 << 31
WB_COUNT_MASK = 0xFFFFFF        # 24-bit completed descriptor count

POISON = 0xA5                   # Initial fill of DDR slots

FRAME_RATE_HZ = {"168": 168000 / RX_SAMPLES, "84": 84000 / RX_SAMPLES}


# -----------------------------------------------------------------------------
# 3. MMIO AND SHARED MEMORY HELPERS
# -----------------------------------------------------------------------------
class Mmio:
    """16/32-bit access to a PCIe BAR through its sysfs resource file."""

    def __init__(self, path, size=BAR_SIZE):
        self.fd = os.open(path, os.O_RDWR | os.O_SYNC)
        self.m = mmap.mmap(self.fd, size, mmap.MAP_SHARED,
                           mmap.PROT_READ | mmap.PROT_WRITE)

    def rd16(self, off):
        return struct.unpack_from("<H", self.m, off)[0]

    def wr16(self, off, val):
        if off == SYSCON_SW_RST:
            raise RuntimeError("CRITICAL SAFETY: write to SYSCON_SW_RST blocked")
        struct.pack_into("<H", self.m, off, val & 0xFFFF)

    def rd32(self, off):
        return struct.unpack_from("<I", self.m, off)[0]

    def wr32(self, off, val):
        struct.pack_into("<I", self.m, off, val & 0xFFFFFFFF)

    def close(self):
        self.m.close()
        os.close(self.fd)


class UdmaBuf:
    """Contiguous DDR region from the u-dma-buf driver, mapped uncached."""

    def __init__(self, name):
        sysdir = None
        for cls in ("u-dma-buf", "udmabuf"):
            d = f"/sys/class/{cls}/{name}"
            if os.path.isdir(d):
                sysdir = d
                break
        if sysdir is None:
            raise RuntimeError(f"u-dma-buf device '{name}' not found. Driver loaded?")
        with open(os.path.join(sysdir, "phys_addr")) as f:
            self.phys = int(f.read().strip(), 0)
        with open(os.path.join(sysdir, "size")) as f:
            self.size = int(f.read().strip(), 0)
        # O_SYNC gives an uncached mapping in u-dma-buf
        self.fd = os.open(f"/dev/{name}", os.O_RDWR | os.O_SYNC)
        self.m = mmap.mmap(self.fd, self.size, mmap.MAP_SHARED,
                           mmap.PROT_READ | mmap.PROT_WRITE)

    def write(self, off, data):
        self.m[off:off + len(data)] = data

    def read(self, off, n):
        return bytes(self.m[off:off + n])

    def rd32(self, off):
        return struct.unpack_from("<I", self.m, off)[0]

    def wr32(self, off, val):
        struct.pack_into("<I", self.m, off, val & 0xFFFFFFFF)

    def close(self):
        self.m.close()
        os.close(self.fd)


# -----------------------------------------------------------------------------
# 4. DESCRIPTORS
# -----------------------------------------------------------------------------
def pack_desc(length, src, dst, nxt):
    """One 32-byte XDMA descriptor (PG195)."""
    w0 = (DESC_MAGIC << 16) | DESC_COMPLETED | DESC_EOP
    return struct.pack("<8I",
                       w0,
                       length & 0x0FFFFFFF,
                       src & 0xFFFFFFFF, src >> 32,   # FPGA AXI (rx_bram)
                       dst & 0xFFFFFFFF, dst >> 32,   # DDR slot
                       nxt & 0xFFFFFFFF, nxt >> 32)   # next descriptor


def build_ring(buf, chain):
    """Looping ring: descriptor k reads rx_bram slot k%2 into DDR slot k."""
    for k in range(chain):
        src = RX_BRAM_BASE + RX_FRAME_BYTES * (k % RX_BRAM_SLOTS)
        dst = buf.phys + DATA_OFF + RX_FRAME_BYTES * k
        nxt = buf.phys + DESC_OFF + DESC_BYTES * ((k + 1) % chain)
        buf.write(DESC_OFF + DESC_BYTES * k,
                  pack_desc(RX_FRAME_BYTES, src, dst, nxt))


# -----------------------------------------------------------------------------
# 5. CONTROL HELPERS
# -----------------------------------------------------------------------------
def stop_rx(bar0, bar1, timeout_s=0.1):
    """Stop RX mover and C2H0. TX bits are preserved; pacer kept if TX runs."""
    ctrl = bar0.rd16(REG_CTRL)
    new = ctrl & ~CTRL_RX_MASK
    if not (ctrl & CTRL_TX_EN):
        new &= ~CTRL_PACER_EN
    bar0.wr16(REG_CTRL, new)

    bar1.wr32(C2H0 + CH_CTRL_W1C, CTRL_RUN)
    deadline = time.monotonic() + timeout_s
    while bar1.rd32(C2H0 + CH_STATUS) & STATUS_BUSY:
        if time.monotonic() > deadline:
            print("[WARN] C2H0 still busy after stop request")
            break
        time.sleep(0.001)

    bar1.wr32(SGDMA_COMMON + CREDIT_MODE_W1C, CREDIT_BIT_C2H0)


def is_pow2(n):
    return n > 0 and (n & (n - 1)) == 0


# -----------------------------------------------------------------------------
# 6. OPTIONAL A53-SIDE VERIFICATION (same rules as rx_optionA_verify.py)
# -----------------------------------------------------------------------------
def load_lut(path):
    """Read LUT_I / LUT_Q verbatim from rx_optionA_verify.py without importing
    it (avoids the numpy/matplotlib dependency on the target)."""
    with open(path) as f:
        tree = ast.parse(f.read(), filename=path)
    luts = {}
    for node in ast.walk(tree):
        if (isinstance(node, ast.Assign) and len(node.targets) == 1
                and isinstance(node.targets[0], ast.Name)
                and node.targets[0].id in ("LUT_I", "LUT_Q")):
            v = node.value
            if isinstance(v, ast.Call) and v.args:   # np.array([...])
                v = v.args[0]
            luts[node.targets[0].id] = [int(x) for x in ast.literal_eval(v)]
    for name in ("LUT_I", "LUT_Q"):
        if len(luts.get(name, [])) != LUT_LEN:
            raise RuntimeError(f"{name} not found or not {LUT_LEN} entries in {path}")
    return luts["LUT_I"], luts["LUT_Q"]


def sext28(v):
    return v - 0x10000000 if v & 0x08000000 else v


def unpack_frame(raw):
    """Decode one RX frame. Each 64-bit word is read as four LE 16-bit words:
       w0[15:12] meta, w0[11:0] I[27:16], w1 I[15:0],
       w2[15:12] meta, w2[11:0] Q[27:16], w3 Q[15:0].
       Reference check: w0[13:12] == 01 (as rx_optionA_verify.py)."""
    i, q, meta = [], [], set()
    meta_ok = True
    for k in range(0, len(raw), 8):
        w0, w1, w2, w3 = struct.unpack_from("<4H", raw, k)
        if (w0 & 0x3000) != 0x1000:
            meta_ok = False
        meta.add(((w0 >> 12) & 0xF, (w2 >> 12) & 0xF))
        i.append(sext28(((w0 & 0x0FFF) << 16) | w1))
        q.append(sext28(((w2 & 0x0FFF) << 16) | w3))
    return i, q, meta_ok, meta


def check_frame(i, q, lut_i, lut_q):
    """Return (phase, mismatches). phase is None if alignment fails."""
    for p in range(LUT_LEN):
        if lut_i[p] == i[0] and lut_q[p] == q[0]:
            bad = sum(1 for n in range(len(i))
                      if i[n] != lut_i[(n + p) % LUT_LEN]
                      or q[n] != lut_q[(n + p) % LUT_LEN])
            return p, bad
    return None, len(i)


def snapshot(buf, chain, max_tries=10000):
    """Read the most recently completed slot with a seqlock-style check.
    Slot (n-1) is safe until completion n+chain-2 has started writing it,
    so the read is accepted only if the count advanced by <= chain-2."""
    for tries in range(max_tries):
        w1 = buf.rd32(WB_OFF)
        if w1 & WB_ERR:
            raise RuntimeError(f"Writeback error bit set (0x{w1:08X})")
        n1 = w1 & WB_COUNT_MASK
        slot = (n1 - 1) & (chain - 1)
        raw = buf.read(DATA_OFF + slot * RX_FRAME_BYTES, RX_FRAME_BYTES)
        n2 = buf.rd32(WB_OFF) & WB_COUNT_MASK
        if ((n2 - n1) & WB_COUNT_MASK) <= chain - 2:
            return n1, raw, tries
    raise RuntimeError("Could not read a frame without it being overwritten. "
                       "Try a longer --chain for A53-side verification.")


def run_verify(buf, chain, n_frames, lut_src):
    lut_i, lut_q = load_lut(lut_src)
    phases, metas = set(), set()
    bad_frames = late = 0
    meta_all_ok = True
    last_n = None
    got = 0
    while got < n_frames:
        n, raw, tries = snapshot(buf, chain)
        if n == last_n:
            continue
        last_n = n
        late += tries
        i, q, meta_ok, meta = unpack_frame(raw)
        meta_all_ok &= meta_ok
        metas |= meta
        p, bad = check_frame(i, q, lut_i, lut_q)
        phases.add(p)
        if p is None or bad:
            bad_frames += 1
        got += 1

    ok = bad_frames == 0 and meta_all_ok and len(phases) == 1 and None not in phases
    print(f"[*] Verify       : {'PASS' if ok else 'FAIL'}  frames={n_frames} "
          f"bad={bad_frames} phases={sorted(p for p in phases if p is not None)} "
          f"late_reads={late}")
    print(f"[*] Metadata     : reference bits OK = {meta_all_ok}, "
          f"seen (w0[15:12], w2[15:12]) = {sorted(metas)}")
    print("[i] The test pattern repeats every frame (32 samples, 32-entry LUT),")
    print("    so this check cannot detect lost, duplicated or stale frames.")
    print("    Frame continuity relies on the writeback and RX_DONE counts.")
    return ok


# -----------------------------------------------------------------------------
# 7. MAIN
# -----------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(description="PLAT-150 A53 RX (C2H0) DMA initialiser")
    ap.add_argument("--pci", default="0000:01:00.0", help="PCI slot address")
    ap.add_argument("--udmabuf", default="udmabuf0", help="u-dma-buf device name")
    ap.add_argument("--rate", choices=["84", "168"], default="168",
                    help="Sample rate in ksps")
    ap.add_argument("--chain", type=int, default=2,
                    help=f"Descriptors / DDR slots, power of two, 2..{MAX_CHAIN} "
                         "(2 for R5F ping-pong)")
    ap.add_argument("--expect-phys", type=lambda x: int(x, 0),
                    default=R5F_FPGA_DMA_BASE_ADDR,
                    help="Required u-dma-buf physical base (the address the R5F maps); "
                         "pass 0 to skip the check")
    ap.add_argument("--no-pattern", action="store_true",
                    help="Do not select the FPGA synthetic RX test pattern")
    ap.add_argument("--verify", type=int, default=0, metavar="N",
                    help="After start-up, check N live frames against the LUT")
    ap.add_argument("--lut-src", default="rx_optionA_verify.py",
                    help="File to read LUT_I / LUT_Q from")
    ap.add_argument("--monitor", action="store_true",
                    help="Keep running and log counters once per second")
    args = ap.parse_args()

    chain = args.chain
    if not (is_pow2(chain) and 2 <= chain <= MAX_CHAIN):
        print(f"[FAIL] --chain must be a power of two in 2..{MAX_CHAIN} "
              "(required for the 24-bit writeback count to wrap cleanly)")
        return 1

    print("=" * 80)
    print(" PLAT-150 Option A: A53 RX (C2H0) DMA Initialiser")
    print("=" * 80)

    # STEP 1: DDR region
    buf = UdmaBuf(args.udmabuf)
    need = DATA_OFF + chain * RX_FRAME_BYTES
    if buf.phys + buf.size > (1 << 32):
        raise RuntimeError(f"u-dma-buf at 0x{buf.phys:X} extends above 4 GB")
    if buf.size < need:
        raise RuntimeError(f"u-dma-buf too small: {buf.size} B, need {need} B")
    if args.expect_phys and buf.phys != args.expect_phys:
        raise RuntimeError(f"u-dma-buf at 0x{buf.phys:08X}, R5F expects "
                           f"0x{args.expect_phys:08X}")
    print(f"[*] DDR region   : 0x{buf.phys:08X} ({buf.size} B, {need} B used)")

    # STEP 2: BARs and identifiers
    base = f"/sys/bus/pci/devices/{args.pci}"
    for r in ("resource0", "resource1"):
        if not os.path.exists(f"{base}/{r}"):
            raise FileNotFoundError(f"{base}/{r} not found. Check lspci / drivers.")
    # PCI COMMAND register (config offset 0x04): bit 1 memory space, bit 2 bus master
    with open(f"{base}/config", "rb") as f:
        f.seek(0x04)
        cmd = struct.unpack("<H", f.read(2))[0]
    print(f"[*] PCI COMMAND  : 0x{cmd:04X} (need bits 1 and 2: mem space, bus master)")
    if (cmd & 0x0006) != 0x0006:
        raise RuntimeError(f"Memory space or bus mastering not enabled on {args.pci} "
                           f"(COMMAND=0x{cmd:04X}). Check the driver load, or "
                           f"'setpci -s {args.pci} COMMAND=0x06:0x06'.")

    bar0 = Mmio(f"{base}/resource0")
    bar1 = Mmio(f"{base}/resource1")

    sys_id = bar0.rd16(SYSCON_ID_REV)
    dma_id = bar0.rd16(REG_ID)
    c2h_id = bar1.rd32(C2H0 + CH_ID)
    print(f"[*] syscon ID    : 0x{sys_id:04X} (expect 0x{EXPECT_SYSCON_ID:04X})")
    print(f"[*] iq_dma ID    : 0x{dma_id:04X} (expect 0x{EXPECT_DMA_ID:04X})")
    print(f"[*] XDMA C2H0 ID : 0x{c2h_id:08X}")
    if sys_id != EXPECT_SYSCON_ID or dma_id != EXPECT_DMA_ID:
        raise RuntimeError("FPGA identification failed. Is the Option A build loaded?")
    if (c2h_id >> 16) != 0x1FC1:
        print("[WARN] C2H0 identifier does not look like an XDMA C2H channel")

    # STEP 3: stop RX cleanly
    stop_rx(bar0, bar1)

    # STEP 4: sample rate
    want = 1 if args.rate == "168" else 0
    fsr = bar0.rd16(SYSCON_FULL_SAMPLE_RATE)
    if (fsr & 1) != want and (bar0.rd16(REG_CTRL) & CTRL_TX_EN):
        print("[WARN] Changing sample rate while TX is enabled")
    bar0.wr16(SYSCON_FULL_SAMPLE_RATE, (fsr & ~1) | want)
    time.sleep(0.005)
    frame_hz = FRAME_RATE_HZ[args.rate]
    print(f"[*] Rate         : {args.rate} ksps ({1e6 / frame_hz:.1f} us per frame)")

    # STEP 5: DDR slots, writeback word, descriptor ring
    for k in range(chain):
        buf.write(DATA_OFF + k * RX_FRAME_BYTES, bytes([POISON]) * RX_FRAME_BYTES)
    buf.wr32(WB_OFF, 0)
    build_ring(buf, chain)

    # STEP 6: program C2H0
    wb_addr = buf.phys + WB_OFF
    first_desc = buf.phys + DESC_OFF
    bar1.rd32(C2H0 + CH_STATUS_RC)                     # clear stale status
    bar1.wr32(C2H0 + CH_WB_LO, wb_addr & 0xFFFFFFFF)
    bar1.wr32(C2H0 + CH_WB_HI, wb_addr >> 32)
    bar1.wr32(SGDMA_C2H0 + SG_DESC_LO, first_desc & 0xFFFFFFFF)
    bar1.wr32(SGDMA_C2H0 + SG_DESC_HI, first_desc >> 32)
    bar1.wr32(SGDMA_C2H0 + SG_DESC_ADJ, 0)
    bar1.wr32(SGDMA_COMMON + CREDIT_MODE_W1S, CREDIT_BIT_C2H0)
    bar1.wr32(C2H0 + CH_CTRL, CTRL_RUN | CTRL_POLL_WB)
    time.sleep(0.005)
    compl0 = bar1.rd32(C2H0 + CH_COMPL)
    if compl0 != 0:
        raise RuntimeError(f"C2H0 completed {compl0} descriptors without credits")
    print("[*] XDMA C2H0    : armed, waiting for FPGA credits")

    # STEP 7: enable RX path (read-modify-write, TX bits untouched)
    ovr0 = bar0.rd16(REG_RX_OVERRUN)
    rx_bits = CTRL_RX_EN | CTRL_RX_SEL | CTRL_PACER_EN
    if not args.no_pattern:
        rx_bits |= CTRL_RX_PAT
    bar0.wr16(REG_CTRL, bar0.rd16(REG_CTRL) | rx_bits)
    print(f"[*] REG_CTRL     : 0x{bar0.rd16(REG_CTRL):04X} "
          f"(test pattern {'off' if args.no_pattern else 'on'})")

    # STEP 8: check frames are flowing
    w_a, d_a, t_a = buf.rd32(WB_OFF), bar0.rd16(REG_RX_DONE_CNT), time.monotonic()
    time.sleep(0.2)
    w_b, d_b, t_b = buf.rd32(WB_OFF), bar0.rd16(REG_RX_DONE_CNT), time.monotonic()
    n_wb = ((w_b & WB_COUNT_MASK) - (w_a & WB_COUNT_MASK)) & WB_COUNT_MASK
    n_done = (d_b - d_a) & 0xFFFF
    meas_hz = n_wb / (t_b - t_a)
    compl = bar1.rd32(C2H0 + CH_COMPL)
    ovr = bar0.rd16(REG_RX_OVERRUN)
    print(f"[*] Flow check   : {meas_hz:.0f} frames/s (expect {frame_hz:.0f}), "
          f"WB +{n_wb}, RX_DONE +{n_done}, CH_COMPL={compl}, "
          f"RX_OVERRUN {ovr0}->{ovr}")
    healthy = True
    if w_b & WB_ERR:
        print("[FAIL] Writeback error bit set"); healthy = False
    if abs(meas_hz - frame_hz) > 0.05 * frame_hz:
        print("[FAIL] Frame rate outside 5% of expected"); healthy = False
    if abs(n_wb - n_done) > 2:
        print("[FAIL] Writeback and RX_DONE_CNT disagree"); healthy = False
    if (compl & WB_COUNT_MASK) < (w_b & WB_COUNT_MASK):
        print("[FAIL] Writeback ahead of CH_COMPL"); healthy = False
    if ovr != ovr0:
        print("[FAIL] RX_OVERRUN changed during start-up"); healthy = False

    # STEP 9: R5F handover
    print("\n" + "=" * 80)
    print(" R5F HANDOVER (RX)")
    print("=" * 80)
    print(f" Writeback word      : 0x{wb_addr:08X} (bit 31 error, bits 23..0 count)")
    print(f" Frame size          : {RX_FRAME_BYTES} B (32 x 64-bit packed I/Q words)")
    print(f" DDR slots           : {chain}, first descriptor index 0")
    for k in range(chain):
        print(f"   Slot[{k}]           : 0x{buf.phys + DATA_OFF + k * RX_FRAME_BYTES:08X}")
    print(f" Frame period        : {1e6 / frame_hz:.1f} us")
    print(f" Read deadline       : about one frame period with --chain 2")
    print(" Per frame           : invalidate WB word, read count n,")
    print(f"                       slot = (n - 1) & {chain - 1},")
    print("                       invalidate slot, process, re-read count;")
    print(f"                       discard if count advanced by more than {chain - 2}.")
    print(" Word layout (LE u16): w0[15:12] meta, w0[11:0] I[27:16], w1 I[15:0],")
    print("                       w2[15:12] meta, w2[11:0] Q[27:16], w3 Q[15:0]")
    print(" GOLDEN RULE         : the R5F never reads or writes BAR0/BAR1")
    print("=" * 80 + "\n")

    rc = 0 if healthy else 2
    if args.verify > 0 and healthy:
        if not run_verify(buf, chain, args.verify, args.lut_src):
            rc = 2

    if not args.monitor:
        print("[*] RX streaming is active. Exiting; hardware keeps running.")
        bar0.close(); bar1.close(); buf.close()
        return rc

    print("[*] Monitoring (Ctrl+C to stop RX)")
    last_wb = buf.rd32(WB_OFF) & WB_COUNT_MASK
    last_t = time.monotonic()
    try:
        while True:
            time.sleep(1.0)
            wb = buf.rd32(WB_OFF)
            now = time.monotonic()
            n = wb & WB_COUNT_MASK
            fps = ((n - last_wb) & WB_COUNT_MASK) / (now - last_t)
            last_wb, last_t = n, now
            print(f" WB={n:<8} err={wb >> 31} "
                  f"COMPL={bar1.rd32(C2H0 + CH_COMPL):<10} "
                  f"RX_DONE={bar0.rd16(REG_RX_DONE_CNT):<5} "
                  f"OVR={bar0.rd16(REG_RX_OVERRUN):<3} "
                  f"Pacer=0x{bar0.rd16(REG_PACER_STS):04X} "
                  f"{fps:.0f} f/s")
    except KeyboardInterrupt:
        print("\n[*] Stopping RX path...")
        stop_rx(bar0, bar1)
        bar0.close(); bar1.close(); buf.close()
        print("[*] RX stopped (TX bits left as they were).")
    return rc


if __name__ == "__main__":
    if os.geteuid() != 0:
        print("[ERROR] Must run as root to access PCIe BARs and u-dma-buf.")
        sys.exit(1)
    sys.exit(main())
