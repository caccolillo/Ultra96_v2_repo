Hi John,

Thank you for the clear summary. I agree this is the right direction and it addresses the root cause cleanly.

The problem is exactly as you describe: the FPGA starts running as soon as the bootloader programs it, at which point both units have identical reset-state payloads (RS=0xAA, RB=0x46). The self-echo rejection logic blocks all frames because the two radios are transmitting the same values, and by the time the application writes the correct states the link has already failed. Holding the FSM in reset until the application is ready eliminates this entirely.

A few notes on the implementation:

No delay is needed between writing RESET=1 and RESET=0. The DNA seeding FSM runs independently of the Wishbone reset register and completes within microseconds of FPGA power-on, long before the bootloader finishes. By the time the application writes RESET=0, PrngReady will already be asserted and the FSM will start cleanly from a fully seeded state.

For the register offset I suggest offset 6, immediately after REG_RB at offset 5, so the existing register map is preserved. The application startup sequence would then be:

    write(REG_RESET, 1)        // ensure FSM is held (already default)
    write(REG_RS, local_state) // write correct radio state
    write(REG_RB, local_bits)  // write correct bit state
    write(REG_RESET, 0)        // release FSM, link starts cleanly

I am ready to implement the VHDL change and update the testbench with a startup sequence test case. Before I proceed, could you clarify the following:

1. Will you be creating the new branch from master, or would you like me to do that?
2. What is the branch name I should be working on?
3. Would you like me to base the new implementation on the original radio_link_1wire.vhd code prior to any of my modifications, adding only the RESET register on top of that? Or should I carry forward the improvements made during this work (inline UART, DNA seeding, collision detection, self-echo fix)?

Thanks,
Marco
