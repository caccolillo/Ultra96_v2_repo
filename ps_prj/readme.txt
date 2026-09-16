

PAT6-570 — Request for Radio Link Register Logging Application



Hi,

As part of the PAT6-570 fix (1-Wire LINK negotiation failure), we have updated the radio_link_1wire FPGA module and validated the fix in simulation (19/19 tests passing). However, initial hardware testing with FPGA commit 123f3bfa has shown that the link is still not establishing correctly in the field.

Scott has provided the following register dump from the hardware under test:

   FPGA Register   Radio A (Active)   Radio B (Active)
   0x502           0                  0
   0x504           170                170

REG_PRS (0x502) = 0 on both units means neither radio has received a valid frame from the other. This is consistent with the original PAT6-570 failure mode. 170 = 0xAA = ST_NEW_LINK, which suggests both units are in the new-link state.

At this stage we cannot determine the root cause because we have insufficient visibility into the system. Specifically we cannot confirm:

   1. Whether the FPGA bitstream was built with the correct baud rate divisor
      (DEFAULT_DIVISOR must be 520 for 9600 baud at 80 MHz; if left at the
      simulation value of 5 the UART runs at 833 kbaud and no frames are decoded)

   2. Whether the radio management software wrote REG_RS and REG_RB on both
      units at startup (if both units transmit RadioState=0x00 the self-echo
      rejection logic will discard every received frame and PRS will stay 0x00
      regardless of whether the FPGA is working correctly)

   3. Whether the register map offsets used by the software match the FPGA
      register map (REG_RS=offset 4, REG_RB=offset 5)

To resolve this and provide ongoing diagnostic capability, I would like to request a small Linux daemon that appends periodic register dumps to the radio log file. This will allow us and future test engineers to diagnose link failures without requiring direct hardware access or a logic analyser.

Functional requirements:

1. Read-only access to the following FPGA registers on both radio units:
   - REG_ID      (offset 0): Block ID, should always read 1
   - REG_DIVISOR (offset 1): Baud divisor, should read 520 (0x208) in hardware
   - REG_PRS     (offset 2): Paired Radio State — state received from remote unit
   - REG_PRB     (offset 3): Paired Bit State — bit state received from remote unit
   - REG_RS      (offset 4): Local Radio State — what this unit is transmitting
   - REG_RB      (offset 5): Local Radio Bit State — what this unit is transmitting

   Note: reading REG_ID and REG_DIVISOR at startup is essential — it will
   immediately confirm whether the correct bitstream is loaded and whether the
   baud rate divisor is set correctly, resolving uncertainty point 1 above.

2. Poll both units at a configurable interval (suggested default: 1 second).

3. Always append to the radio log file, never overwrite it, so that the full
   history is preserved for post-incident inspection. Example log entries:

   [timestamp] RADIO_LINK_LOGGER started version=1.0 poll_interval=1s base=0x500
   [timestamp] RADIO_LINK unit=A ID=1 DIV=520 PRS=0x69 PRB=0x46 RS=0x41 RB=0x46 status=OK
   [timestamp] RADIO_LINK unit=A ID=1 DIV=5   PRS=0x00 PRB=0x00 RS=0x41 RB=0x00 status=BROKEN (wrong divisor?)
   [timestamp] RADIO_LINK unit=A ID=1 DIV=520 PRS=0x00 PRB=0x00 RS=0x00 RB=0x00 status=BROKEN (RS/RB not initialised?)

4. Derive and log a human-readable link status on every entry:
   - OK:           PRS is non-zero and differs between the two units
   - BROKEN:       PRS is 0x00
   - NEW_LINK:     PRS is 0xAA on both units (PAT6-570 symptom)
   - WRONG_DIVISOR: DIV is not 520 (flags incorrect bitstream or configuration)
   - NOT_INIT:     RS and RB are both 0x00 (flags uninitialised software state)

5. Log an explicit event entry on any of the following transitions:
   - PRS transitions from non-zero to zero (link drop)
   - PRS transitions from zero to non-zero (link established)
   - Both units show PRS=0xAA simultaneously (PAT6-570 condition)
   - DIV reads a value other than 520 at startup (wrong bitstream)
   - RS or RB reads 0x00 at startup (software not initialised)

6. The application must be strictly read-only — it must never write any register.

7. Run as a background daemon from Linux startup, alongside the radio management
   software. Tolerate log file rotation (logrotate or equivalent) without losing
   entries or crashing.

The base address of the radio_link_1wire block is TBC — please confirm before
implementation. The test engineer's dump suggests REG_PRS at 0x502 and REG_PRB
at 0x504, implying a word-addressed interface with base address around 0x500.

I can provide the FPGA register map, VHDL source, and simulation testbench
results as reference. Please treat this as urgent given the ongoing hardware
failure.

Thanks,
Marco
