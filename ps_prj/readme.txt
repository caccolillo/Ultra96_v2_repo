
Following detailed analysis of the ARM software (radio_link.cpp) in conjunction with the FPGA register dump provided by Scott, I have identified the root cause of the hardware failure with FPGA commit 123f3bfa.

---

*Summary*

The failure is caused by an incompatibility between the FPGA self-echo rejection logic introduced in our rewrite and the LINK_STATE_NEW (0xAA) bootstrap mechanism used by the ARM software. The two components are individually correct but make conflicting assumptions about the startup sequence.

---

*Software startup sequence (from radio_link.cpp)*

1. Constructor (line 117) writes LINK_STATE_NEW (0xAA) to REG_RS on both units:
      CFpga::SetLinkRadiostate( LINK_STATE_NEW ); // Invalidates link to other end

2. ReportStatus() (line 350) writes LINK_STATE_FULL_SERVICE (0x46 = 'F') to REG_RB:
      CFpga::SetLinkRadioBitstate( LINK_STATE_FULL_SERVICE | lineError );

3. Both units therefore transmit the frame: [SYNC=0xA5, RS=0xAA, RB=0x46]

4. GetLinkedRadioActive() (line 443) reads REG_PRS. When PRS=0xAA it detects a
   new link and starts a random timer (CalculateRandomActiveTimer).

5. When the timer expires (line 252), GoActive(RADIO_ACTIVE) is called, which
   writes LINK_STATE_ACTIVE (0x41) to REG_RS:
      CFpga::SetLinkRadiostate( LINK_STATE_ACTIVE );

6. The other unit sees PRS=0x41 (Active), detects a counterpart is already active,
   and calls GoActive(RADIO_INACTIVE), writing 0x69 to its REG_RS.

This is a clean and correct design. The LINK_STATE_NEW bootstrap state is the
mechanism by which the two radios discover each other and negotiate roles.

---

*How our FPGA rewrite breaks this*

Our rewrite introduced self-echo rejection to fix PAT6-570. The rejection
condition in p_fsm_tx is:

   if RxBuf(0) = SYNC_BYTE and
      (RxBuf(1) /= RadioState or RxBuf(2) /= RadioBitState)
   then
      -- accept frame

At startup, both units have RS=0xAA and RB=0x46. The received frame is
[A5, AA, 46]. The rejection check evaluates as:

   RxBuf(1) /= RadioState    →  0xAA /= 0xAA  →  false
   RxBuf(2) /= RadioBitState →  0x46 /= 0x46  →  false
   OR condition = false → frame rejected

PRS stays 0x00 on both units. GetLinkedRadioActive() reads PRS=0x00, returns
success=false, m_linkConnected=false. GoActive() is never called. RS stays
at 0xAA permanently.

This creates a deadlock:
   - The FPGA rejects the NEW_LINK frame because it looks like self-echo
   - The SW never progresses past NEW_LINK because PRS never updates
   - RS never changes from 0xAA because GoActive() is never called
   - The FPGA keeps rejecting the frame because RS is still 0xAA

---

*Why the previous FPGA build (Jenkins Build 334) worked*

The previous build had no self-echo rejection. It accepted all frames that
passed the SYNC byte check, including [A5, AA, 46] from the peer. PRS was
set to 0xAA, the SW detected a new link, started the timer, called
GoActive(RADIO_ACTIVE), and normal negotiation proceeded. The PAT6-570 bug
was present in that build — it manifested as both radios going Active
independently under certain timing conditions — but the NEW_LINK bootstrap
worked because there was no rejection of identical-state frames.

---

*Register dump correlation*

Failed build (FPGA commit 123f3bfa):
   REG_PRS (0x502) = 0x00 on both  → FPGA rejected all frames, SW stuck
   REG_PRB (0x503) = 0x00 on both  → FPGA cleared PRB (LinkBroken=1)
   REG_RS  (0x504) = 0xAA on both  → SW never called GoActive()
   REG_RB  (0x505) = 0x46 on both  → ReportStatus() wrote correctly

This is fully consistent with the deadlock described above.

---

*Proposed fix — FPGA (radio_link_1wire.vhd)*

The self-echo rejection must be relaxed to allow LINK_STATE_NEW frames
through, because NEW_LINK is a special bootstrap state, not a meaningful
operational state. The updated acceptance condition:

   constant LINK_STATE_NEW : std_logic_vector(7 downto 0) := x"AA";

   if RxBuf(0) = SYNC_BYTE and
      (RxBuf(1) /= RadioState or
       RxBuf(2) /= RadioBitState or
       RxBuf(1) = LINK_STATE_NEW)   -- always accept NEW_LINK from peer
   then
      -- accept frame

This means:
   - At startup (both RS=0xAA): NEW_LINK frames are accepted → PRS=0xAA
     → SW detects new link → timer runs → GoActive() called → RS changes
   - Once one unit has RS=0x41 (Active): normal self-echo rejection resumes
     because 0x41 /= 0xAA, so the condition is naturally satisfied
   - PAT6-570 is still prevented: once both units have distinct RS values
     (0x41 and 0x69), the OR condition catches any genuine self-echo

The fix requires a one-line change to the FPGA RTL. Simulation testbench
will need a new test case covering the NEW_LINK bootstrap scenario with
both units starting at RS=0xAA.

---

*Action items*

1. [FPGA] Update self-echo rejection condition in radio_link_1wire.vhd
   as described above. Update testbench with NEW_LINK bootstrap test.

2. [SW] No changes required. The radio_link.cpp bootstrap logic is correct
   and does not need modification.

3. [HW] Confirm presence of pull-up resistor on the 1-wire line. Scott
   observed a single-pulse pattern on the scope with one radio and no peer
   connected. This is consistent with a floating line being read as
   permanently busy by the FPGA collision detection. A pull-up (typically
   4.7kΩ to 3.3V) is required for correct open-drain operation.

Thanks,
Marco
