
Hi Scott,

You're right, the single pulse is not the random wait — the random wait is silent (the unit is just waiting, not transmitting). What you're seeing is the start bit of the first byte being transmitted, followed by an immediate abort.

The most likely cause with a single radio and no peer connected is that the 1-wire line is floating. If N_LINK_RX_I floats low with nothing connected, the FPGA reads the line as busy (someone is pulling it low) even though nothing is there. When the unit tries to transmit, it immediately sees a conflict between what it's driving and what it reads back, aborts, and backs off. That produces exactly one pulse — the start bit — then silence for the back-off period, then one pulse again, repeating.

Two questions that would help confirm this:
1. Is there a pull-up resistor on the 1-wire line in the hardware? The open-drain bus needs one to return to idle when nothing is driving it.
2. What does the line voltage read when nothing is connected and the FPGA is idle — is it sitting at 0V or at the pull-up voltage?

If the line is floating low, connecting a pull-up resistor (typically 4.7kΩ to 3.3V) should immediately restore the multi-pulse burst pattern you saw in previous builds.

Note: this would be a hardware issue independent of the FPGA build — the previous build may have had a different collision detection mechanism that was less sensitive to a floating line.

Thanks,
Marco
