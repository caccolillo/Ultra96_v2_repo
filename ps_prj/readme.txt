Hi all,

Following Scott's updated register dump, I have analysed the failure and have a hypothesis for why the link is not establishing with FPGA commit 123f3bfa.

---

*Observed evidence*

With FPGA commit 123f3bfa (our build):
- Both units: RS=170 (0xAA = ST_NEW_LINK), RB=70 (0x46 = Full Service)
- Both units: PRS=0, PRB=0
- Both units labelled Active by SW

With Jenkins Build 334 (working):
- Radio A: RS=65 (0x41 = Active), Radio B: RS=105 (0x69 = Inactive)
- Both units show valid cross-populated PRS values

---

*Hypothesis*

The radio management software writes ST_NEW_LINK (0xAA) to REG_RS on *both* units at startup, before Active/Inactive role negotiation has completed. With both units transmitting identical frames [A5, AA, 46], the FPGA self-echo rejection logic discards every received frame because the payload matches the local state exactly:

  RxBuf[1] /= RadioState    →  0xAA /= 0xAA  →  false
  RxBuf[2] /= RadioBitState →  0x46 /= 0x46  →  false
  OR condition = false → frame rejected

PRS stays 0x00 on both units permanently. The link never establishes. The SW never receives confirmation that the peer is present, so it never progresses beyond ST_NEW_LINK and never writes the final Active/Inactive states to REG_RS. This creates a deadlock: the FPGA is waiting for the SW to write distinct states, and the SW is waiting for the FPGA link to establish before writing distinct states.

---

*Why it worked with Build 334*

Build 334 is a different FPGA implementation that had a different or no self-echo rejection mechanism. The same SW startup sequence — both units writing ST_NEW_LINK=0xAA initially — worked because the old FPGA accepted frames regardless of whether the payload matched the local state. The SW was then able to establish the link, complete negotiation, and write the final RS=0x41/0x69 values. This is exactly what Scott's working register dump shows.

---

*Why our build breaks it*

Our rewrite introduced strict self-echo rejection to fix PAT6-570. This was correct and necessary — without it, a unit updates PairedRadioState with its own loopback frame, causing both units to go Active (the original bug). However, the rejection condition is too strict when both units start with the same initial state. The FPGA cannot distinguish between a genuine self-echo and a valid frame from a peer that happens to be in the same state.

---

*Options to fix*

1. SW fix (preferred): Write distinct initial RS values to the two units at startup — for example unit A always writes RS=0x41 and unit B always writes RS=0x69 before waiting for link establishment. Final negotiated state can still be updated afterwards. This requires the SW team to clarify how the two units are assigned their roles at boot.

2. FPGA fix (alternative): Replace the payload-match rejection with a post-transmission inhibit window — discard incoming frames for a fixed period immediately after the local unit finishes transmitting, regardless of content. This avoids the identical-state deadlock and is immune to the circular dependency, but adds complexity to the RTL.

3. FPGA fix (simpler but weaker): Remove the payload check entirely and only validate the SYNC byte. This eliminates the deadlock but reintroduces a weaker form of the PAT6-570 vulnerability and would need careful analysis before adoption.

---

*Immediate question for SW team*

Is there a mechanism in the radio management software that assigns distinct roles (Active/Inactive) to the two units independently of the 1-wire link? If the SW relies on the FPGA link to determine which unit should be Active, and the FPGA relies on the SW to write distinct states before the link can establish, neither side can break the deadlock on its own.

Please can SW confirm what values are written to REG_RS and REG_RB on each unit at startup, and at what point in the boot sequence those writes occur?

Thanks,
Marco
