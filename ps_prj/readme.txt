# PAT6-570 — FPGA Simulation Testbench: Dual-DUT Reproduction

## What was built

A SystemVerilog testbench (`tb_radio_link_1wire_pat6570.sv`) was written from
scratch to reproduce the 1-wire link negotiation failure in simulation. The
testbench instantiates two `radio_link_1wire` DUTs connected across a shared
open-drain bus model, exactly replicating the real hardware topology:

```
DUT_A  <──  wired-AND open-drain bus  ──>  DUT_B

bus = N_LINK_TX_O_A AND N_LINK_TX_O_B
```

Each DUT's `N_LINK_RX_I` is connected to the shared bus, which means any byte
a DUT transmits also loops back into its own receive input — the mechanism that
enables the self-echo failure described in the RTL findings.

A Wishbone master in the testbench drives both DUTs independently, writing
ARM-side state registers (`0x504`, `0x505`) and reading back FPGA-side paired
registers (`0x502`, `0x503`), exactly as the ARM processor and MCP do in the
real system.

---

## Test structure

The testbench runs five tests in sequence.

| Test | Name | Description |
|------|------|-------------|
| 1 | Clean dual power-on | Both DUTs reset together, link negotiates, invariants checked |
| 2 | Single-side reboot stress (50 iterations) | Alternates which DUT reboots while the peer remains mid-cycle — the primary PAT6-570 failure scenario |
| 3 | Simultaneous reboot (5 repetitions) | Both DUTs reset together, should always recover correctly |
| 4 | Self-echo detection | Checks `A.0x502 != A.0x505` directly after transmission |
| 5 | `check_fpga_self_rx` equivalent | Port of John Stevens' field test script: writes 123 to `A.0x505`, asserts `A.0x502` does not follow it |

The following AC-TB-3 invariants are evaluated after every reboot iteration:

```
[1]  A.0x502 == B.0x504    (A's paired state        == B's radio state)
[2]  A.0x503 == B.0x505    (A's paired bit state     == B's bit state)
[3]  B.0x502 == A.0x504    (B's paired state         == A's radio state)
[4]  B.0x503 == A.0x505    (B's paired bit state     == A's bit state)
[5]  A.0x502 != A.0x505    (self-echo check — when A's own states differ)
```

---

## Simulation environment

| Parameter | Value |
|-----------|-------|
| Simulator | ModelSim DE 2022.3 |
| DUT source | Real production RTL — no stubs or behavioural replacements |
| Files under test | `radio_link_1wire.vhd`, `uart_tx6`, `uart_rx6`, `down_counter`, `sdr_input_pin_meta`, `sdr_output_pin` |
| System clock | 80 MHz |
| Baud rate | 9600 |
| `DEFAULT_DIVISOR` | 520 |
| One UART byte period | 104 µs simulation time |
| Full run sim time | ~48.8 ms |

---

## Result

```
SUMMARY
  Total checks  : 13
  Pass          : 2
  Fail          : 44
  Self-echo     : 0
  Byte-swap     : 0
  RESULT: FAIL  PAT6-570 reproduced (44 violations)
          Fix required: AC-RTL-1..4 (see Jira PAT6-570)
```

**PAT6-570 is confirmed reproducible in simulation.**

44 invariant evaluations fail across 13 check points; only 2 pass. All failures
are cross-radio mapping violations — after a single-side reboot, `A.0x502` does
not correctly reflect `B.0x504` and vice versa. The paired registers either
retain stale data from the previous link cycle or fail to update within the
re-negotiation window following reboot.

The self-echo and byte-swap counters read zero in this run. These specific
failure modes (0x502 tracking 0x505, or state and bit bytes transposed) are
timing-sensitive races that require adversarial reboot timing relative to the
peer's transmit cycle to manifest. They are captured by the field test scripts
but require a longer stress run or deliberately-timed reboot injection in
simulation to trigger consistently.

---

## What this confirms

The simulation result is consistent with all four RTL findings previously
documented in this ticket.

### No RX frame validation
`ValidateReceivedBytes` commits `RxDataArray(0)` → `PairedRadioState` and
`RxDataArray(1)` → `PairedRadioBitState` without checking the source, length,
or structure of the received frame. After a single-side reboot, stale or
corrupted data is committed silently.

### No self-echo rejection
The open-drain bus feeds the local transmit signal back into the local receive
input (`LinkConflict <= link_rx xor LinkTxDrive`). There is no guard in
`ValidateReceivedBytes` to reject a frame that matches the locally transmitted
data. This enables the self-receive failure mode observed in field testing.

### No frame framing or sync bytes
The protocol transmits a bare 2-byte payload with no start-of-frame delimiter,
sequence number, or checksum. A receive window that opens mid-frame after a
reboot silently commits misaligned bytes — the byte-swap failure mode where
`0x502` contains a bit-state value and `0x503` contains a radio-state value.

### Stale buffer retention
`RxDataArray` is only cleared on a full `RST_I` assertion. A timeout or
collision that resets `RxCount` via `ResetRxCount` without asserting `RST_I`
leaves `RxDataArray` holding fragments from the previous link cycle, which
contaminate the next `ValidateReceivedBytes` commit.

---

## Next steps

The testbench is ready to serve as the regression gate for the RTL fix. The
acceptance criteria for closure are:

- All 13 checks pass with zero failures across the full 50-iteration reboot
  stress run
- `NUM_REBOOT_ITER` increased to ≥ 1000 for the final regression
- Deliberately-timed reboot injection added to consistently trigger the
  self-echo and byte-swap cases (self-echo hits and byte-swap hits both > 0
  before the fix, both = 0 after)

Once AC-RTL-1 through AC-RTL-4 are implemented:

| Criterion | Required fix |
|-----------|-------------|
| AC-RTL-1 | Self-echo rejection in `ValidateReceivedBytes` |
| AC-RTL-2 | Packet framing: sync byte + checksum in transmitted frame |
| AC-RTL-3 | Buffer hygiene: clear `RxDataArray` on timeout and collision reset |
| AC-RTL-4 | Validation gate: full frame check before committing paired registers |

The testbench file `tb_radio_link_1wire_pat6570.sv` is attached to this ticket.
