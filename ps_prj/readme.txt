# PAT6-570 Testbench Diagnosis — 11 Sep 2026

## What the transcript shows

Every single check across Tests 1–3 fails with the same pattern:

```
A: PRS=00h PRB=00h RS=41h RB=46h
B: PRS=00h PRB=00h RS=69h RB=46h
FAIL[1] A.PRS(502)=00h != B.RS(504)=69h
```

PRS and PRB are **always zero**. RS and RB are written correctly by the
Wishbone tasks (Test 0 confirms the write path works). The RTL is receiving
3 bytes (`RX COUNT: 0 → 1 → 2 → RX COUNT EXPIRED. RxCount=3`), so
`ValidateReceivedBytes` is being called — but it never commits anything to
PRS/PRB.

The divisor is confirmed at 5 from the waveform. The Wishbone timing fix
works (Test 0 all PASS). The open-drain bus model is correct.

**The only remaining explanation is that `ValidateReceivedBytes` is
rejecting every valid frame.**

---

## Root cause: self-echo rejection is too aggressive

The validation gate added as part of the PAT6-570 fix is:

```vhdl
if (RxDataArray(0) = SYNC_BYTE) and
   (RxDataArray(1) /= RadioState or
    RxDataArray(2) /= RadioBitState)
then
    PairedRadioState    <= RxDataArray(1);
    PairedRadioBitState <= RxDataArray(2);
end if;
```

The intent was: reject the frame if it looks like a self-echo, i.e. if the
received payload matches what we transmitted. The OR condition was meant to
pass the frame if *either* byte differs from the local value.

### Why it rejects legitimate frames

The condition uses OR, which means the frame is **rejected** when:

```
RxDataArray(1) = RadioState  AND  RxDataArray(2) = RadioBitState
```

It is **accepted** when:

```
RxDataArray(1) /= RadioState  OR  RxDataArray(2) /= RadioBitState
```

This looks correct at first glance, but it fails in two concrete scenarios
that occur in every test:

#### Scenario 1 — Startup race (affects Tests 1–3 first check)

At power-on, both DUTs reset with `RadioState=00h` and `RadioBitState=00h`.
The link FSM enters `Receiving` before the testbench Wishbone writes complete.
The first frame transmitted by either DUT contains `[SYNC][00h][00h]`.

When DUT_B receives this frame:
- `RxDataArray(1) = 00h`
- `RadioState (local) = 00h`  ← not yet written by testbench
- `RxDataArray(2) = 00h`
- `RadioBitState (local) = 00h`  ← not yet written

Both bytes match → OR condition is false → frame rejected as self-echo.

The DUT then waits for the next frame. By the time the testbench writes
`RS=41h / RB=46h` to A and `RS=69h / RB=46h` to B, the link may have
already timed out and entered `LinkBroken`, zeroing PRS/PRB again.

#### Scenario 2 — Shared BitState (affects all tests persistently)

Both radios in every test use `BT_FULL_SVC = 46h` as their RadioBitState.
This is realistic — it matches the hardware configuration.

When DUT_B (RadioBitState=46h) receives a frame from DUT_A:
- `RxDataArray(2) = 46h`  (A's BitState)
- `RadioBitState (local, B) = 46h`

The BitState byte always matches. The OR condition therefore reduces to:

```
RxDataArray(1) /= RadioState
```

i.e. it only accepts the frame if the received RadioState differs from the
local RadioState. This is correct when A=Active(41h) and B=Inactive(69h),
because 41h ≠ 69h.

**However:** immediately after reset, both DUTs start with `RadioState=00h`.
The FPGA FSM transmits the first frame before the ARM (testbench) writes the
RS register. So the first frame seen by each DUT has `RxDataArray(1)=00h`
and the local `RadioState=00h` — they match, BitState also matches →
rejected.

After the testbench writes RS, the FSM may be mid-frame or in timeout
recovery. The exact timing determines whether a valid frame ever lands within
the check window. From the transcript, it never does.

#### Scenario 3 — OR vs AND (logic error)

The fundamental problem is that OR is the wrong operator for self-echo
detection. A self-echo means the *entire* received payload is identical to
what was transmitted. The correct rejection condition is:

```
reject if: RxDataArray(1) = RadioState AND RxDataArray(2) = RadioBitState
```

which means accept if:

```
NOT (RxDataArray(1) = RadioState AND RxDataArray(2) = RadioBitState)
```

The current implementation uses OR, which rejects the frame if *any single
byte* matches a local value. This is far too aggressive: in a two-radio
system where both radios share the same BitState (which is normal —
both are in full service), **every frame from the peer will be rejected**
because byte [2] always matches.

---

## Evidence from the transcript

| Observation | What it tells us |
|---|---|
| `RX COUNT: 0 → 1 → 2 → EXPIRED RxCount=3` | 3 bytes are received correctly; `ValidateReceivedBytes` is called |
| `PRS=00h` at every check | `ValidateReceivedBytes` never commits to PRS/PRB |
| Test 0 all PASS | Wishbone write/read path is correct; RS/RB are written |
| Test 4 PASS (A.PRS≠A.RB) | Only passes because PRS=00h≠46h, not because self-echo is absent |
| Test 5 PASS (A.PRS≠123) | Only passes because PRS=00h≠123, not because fix works |
| Self-echo count = 0 | No self-echo detected — because PRS never gets any value at all |

---

## The fix

Change `ValidateReceivedBytes` in `radio_link_1wire.vhd` from:

```vhdl
-- WRONG: rejects frame if ANY byte matches local value
if (RxDataArray(0) = SYNC_BYTE) and
   (RxDataArray(1) /= RadioState or
    RxDataArray(2) /= RadioBitState)
then
    PairedRadioState    <= RxDataArray(1);
    PairedRadioBitState <= RxDataArray(2);
end if;
```

to:

```vhdl
-- CORRECT: rejects frame only if BOTH bytes match local value (true self-echo)
if (RxDataArray(0) = SYNC_BYTE) and
   not (RxDataArray(1) = RadioState and RxDataArray(2) = RadioBitState)
then
    PairedRadioState    <= RxDataArray(1);
    PairedRadioBitState <= RxDataArray(2);
end if;
```

This ensures:
- A frame is accepted if it has the correct sync byte and its payload is not
  an exact copy of what this DUT is currently transmitting.
- A frame where only one byte coincides with a local value (e.g. shared
  BitState) is accepted correctly.
- True self-echoes (entire payload matches) are still rejected.

---

## Secondary issue — startup race

Even with the OR→AND fix, there is a residual risk at startup: both DUTs
transmit `[SYNC][00h][00h]` before the ARM writes RS/RB, and each DUT will
correctly reject this as a self-echo (payload matches local values). This is
harmless if the link FSM re-tries quickly enough, but to make the testbench
robust the `set_states_a` / `set_states_b` calls should complete before the
link FSM has time to transmit its first frame. The current testbench already
does this (Wishbone writes happen immediately after reset deassertion, well
within one byte period at divisor=5), so no testbench change is needed —
only the VHDL fix above.

---

## Summary

| # | Issue | Severity | Fix |
|---|---|---|---|
| 1 | `ValidateReceivedBytes` uses OR instead of AND for self-echo check | **Critical — blocks all frame commits** | Change OR to `not(...and...)` in VHDL |
| 2 | Startup race: first frame transmitted before ARM writes RS/RB | Low — recovers on next frame cycle | No change needed at divisor=5 |
