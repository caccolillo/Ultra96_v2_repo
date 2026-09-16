

PAT6-570 — Request for Radio Link Register Logging Application





Hi,

As part of the PAT6-570 fix (1-Wire LINK negotiation failure), we have updated the radio_link_1wire FPGA module to resolve the Active/Active arbitration bug. The fix has been validated in simulation (19/19 tests passing) and verified on hardware via register dump and scope traces.

To support ongoing field diagnostics and provide early warning of any link instability, I would like to request a small Linux application that periodically reads and logs the FPGA radio link registers to the system radio log file.

Functional requirements:

1. Read-only access to the following FPGA registers on both radio units:
   - REG_PRS (offset 2): Paired Radio State — state reported by the remote unit
   - REG_PRB (offset 3): Paired Bit State — bit state reported by the remote unit
   - REG_RS  (offset 4): Local Radio State — what this unit is transmitting
   - REG_RB  (offset 5): Local Radio Bit State — what this unit is transmitting

2. Poll both units at a configurable interval (suggested default: 1 second).

3. Append a structured log entry to the existing radio log file on each poll, for example:
   [timestamp] RADIO_LINK unit=A PRS=0x69 PRB=0x46 RS=0x41 RB=0x46 status=OK
   [timestamp] RADIO_LINK unit=A PRS=0x00 PRB=0x00 RS=0x41 RB=0x46 status=BROKEN

   The application must always append to the log file, never overwrite it, so that the full
   history of register values is preserved for post-incident inspection and analysis.

4. Derive and log a human-readable link status on every entry:
   - OK:       PRS is non-zero and differs between the two units
   - BROKEN:   PRS is 0x00
   - NEW_LINK: PRS is 0xAA on both units (PAT6-570 symptom — both went Active independently)

5. Log an explicit event entry when any of the following transitions occur:
   - PRS transitions from non-zero to zero (link drop)
   - PRS transitions from zero to non-zero (link established)
   - Both units simultaneously show PRS=0xAA (PAT6-570 condition detected)
   Each event entry should include the previous and new register values and a timestamp.

6. On startup, write a header entry to the log recording the application version, poll
   interval, and register base address, so that log files are self-describing:
   [timestamp] RADIO_LINK_LOGGER started version=1.0 poll_interval=1s base=0x500

7. The application must be strictly read-only — it must never write to REG_RS, REG_RB,
   or REG_DIVISOR. Those registers are owned by the main radio management software.

8. The application should be loaded at Linux startup alongside the existing radio
   management software and run as a background daemon.

9. Log file rotation should be handled by the existing system log rotation mechanism
   (logrotate or equivalent) — the application should tolerate the log file being
   rotated without losing entries or crashing.

Non-requirements (handled entirely by the FPGA):
   - The app does not need to manage the UART, timing, collisions, back-off, or PRNG
     seeding. All of that is autonomous inside radio_link_1wire.

The base address of the radio_link_1wire block in the address map is TBC — please
confirm with me before implementation. Scott Hisee's register dump from Jenkins Build
334 showed REG_PRS at 0x502 and REG_PRB at 0x504, which suggests a word-addressed
interface with base address around 0x500.

Please let me know if you need the FPGA register map, the VHDL source, or the
simulation testbench results as reference.

Thanks,
Marco
