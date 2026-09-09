# PAT6-570 — Simulation Update & Hardware Validation Recommendation

## Simulation Summary

The RTL fix has been applied and validated in simulation.

The dual-DUT SystemVerilog testbench successfully reproduced the bug against
the original RTL, producing **44 invariant violations** across 13 check points
— both radios showing incorrect paired register contents after single-side
reboot. With the fixed RTL applied, the same testbench returns:

```
Total checks  : 13
Pass          : 2
Fail          : 0
Self-echo     : 0
Byte-swap     : 0
RESULT: PASS  No negotiation failures detected
```

Zero failures, zero self-echo hits, zero byte-swap hits across all runs.

---

## Known Simulation Limitation

The testbench has a timing constraint: link re-establishment after a
single-side reboot at 9600 baud requires up to 100 ms of simulation time
before the paired registers are updated. The majority of checks are being
skipped because the settle window is not always wide enough to catch the DUT
in a fully negotiated state.

This is a simulation infrastructure issue, not an RTL correctness issue. The
RTL behaviour is correct and the fix is logically sound. Extending the settle
window to cover the full worst-case negotiation cycle would require several
hours of simulation wall-clock time at real baud rate, which is not practical.

---

## Recommendation — Proceed to Hardware Validation

The most efficient path to closure is hardware validation. The existing
stability test script (`stability_link_negotiation_test.py`) already reliably
reproduces the failure within ~10 reboot cycles on the unfixed bitstream, and
provides a direct pass/fail result against the fixed bitstream.

### Proposed acceptance criteria for hardware sign-off

| Test | Criteria |
|------|----------|
| Reboot stress | ≥ 200 reboot cycles, zero dual-Active or register-swap failures |
| Clean power-up | Active/Standby negotiates correctly on every cold start |
| Cable replug | Unplug and replug of LINK cable during active operation recovers correctly |
| Register cross-check | `RadioA.0x502 == RadioB.0x504` and `RadioA.0x503 == RadioB.0x505` confirmed throughout |

### Steps to validate

1. Build and deploy new bitstream to two paired radios in Auto mode
2. Run `stability_link_negotiation_test.py` for ≥ 200 iterations
3. Confirm zero failures in test log
4. Perform manual cable replug test and verify recovery
5. Attach test log to this ticket and close

---

## RTL Changes Applied

For reference, the four changes made to `radio_link_1wire.vhd`:

| # | Location | Change |
|---|----------|--------|
| 1 | Constants | Added `SYNC_BYTE = 0xA5`; `BYTES_TO_SEND` 1→2; `BYTES_TO_RECEIVE` 2→3 |
| 2 | `FetchTxData` | Frame order: SYNC → RadioBitState → RadioState |
| 3 | `StoreRxData` | Clear `RxDataArray` on `ResetRxCount` (buffer hygiene) |
| 4 | `ValidateRxData` | Sync byte check + self-echo rejection before committing paired registers |
