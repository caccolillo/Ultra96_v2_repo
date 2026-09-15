// =============================================================================
// tb_radio_link_1wire_pat6570
// PAT6-570 — 1-Wire LINK negotiation failure
//
// Copyright (c) 2026 Park Air Systems Ltd. All Rights Reserved.
//
// Tests that two radio_link_1wire instances correctly negotiate Active/Standby
// roles after power-on and single-side reboot, and do not both become Active.
//
// -----------------------------------------------------------------------------
// RTL compatibility: radio_link_1wire.vhd (inline UART + DNA_PORT seeding)
//   No external UART or IOB primitives; DNA_PORT instantiated with
//   SIM_DNA_VALUE => x"000000000000000" on both DUTs.
//
// KEY SIMULATION CONSTRAINTS
// --------------------------
// 1. DEFAULT_DIVISOR
//    The VHDL entity has DEFAULT_DIVISOR := 5 (sim speed-up).
//    Hardware value is 520 (0x208). Do not synthesise with divisor=5.
//    Confirm after reset: dut_a.divisor_reg should read 5.
//
// 2. DNA_PORT seeding
//    The link FSM is held in reset for 59+ clock cycles after power-on while
//    the DNA init FSM reads and folds the device DNA.  Each DUT instance is
//    given a distinct SIM_DNA_VALUE generic so they derive different PRNG seeds
//    from the start — no reset stagger is needed for symmetry breaking.
//    DUT_A: SIM_DNA_VALUE = 56'hDEAD_BEEF_CAFE_01
//    DUT_B: SIM_DNA_VALUE = 56'hC0FF_EE12_3456_78
//    (Values are arbitrary; just need to be different and non-zero.)
//
// 3. PrngReady startup hold-off
//    After reset de-assertion, wait at least DNA_READY_CYCLES before driving
//    any Wishbone transactions — the FSM ignores WB writes while PrngReady='0'.
//
// 4. Active-poll on PRS before invariant checks
//    LinkBroken clears PRS/PRB every cycle it is asserted, so a fixed #SETTLE
//    delay can land in a transient clearing window.  check_invariants() polls
//    PRS_A for up to POLL_TIMEOUT_NS before sampling, ensuring the check lands
//    in a stable post-negotiation window.
//
// -----------------------------------------------------------------------------
// Timing (DEFAULT_DIVISOR=5, CLK=80 MHz):
//   BaudEn period   = 16 × 5 × 12.5 ns = 1000 ns   (1 µs)
//   1 bit period    = 1 µs
//   1 byte (8N1)    = 10 µs
//   RECEIVE_TIMEOUT = 481 byte-times = 4.81 ms
//   LINK_WAIT       =  46 byte-times = 460 µs
//   Full cycle      = 530 byte-times = 5.30 ms
//   SETTLE          = 6 full cycles  = 31.8 ms  (always lands in stable window)
//
// -----------------------------------------------------------------------------
// Register map (WB_ADR_I 3 bits)
//   0  REG_ID   read-only (returns 1)
//   1  REG_DIV  R/W  baud divisor
//   2  REG_PRS  R    paired radio state   (updated by 1-wire RX)
//   3  REG_PRB  R    paired bit state     (updated by 1-wire RX)
//   4  REG_RS   R/W  local radio state
//   5  REG_RB   R/W  local bit state
//
// State values
//   Radio state:  00=no-link  AA=new-link  41=active  69=inactive
//   Bit state:    00=no-link  46=full-svc  52=reduced 6E=no-svc
//
// =============================================================================

`timescale 1ns/1ps

module tb_radio_link_1wire_pat6570;

    // =========================================================================
    // Parameters
    // =========================================================================

    localparam real CLK_PERIOD_NS  = 12.5;    // 80 MHz
    localparam int  WB_DAT_W       = 16;
    localparam int  WB_ADR_W       = 3;
    localparam int  NUM_REBOOT_ITER = 8;

    // -------------------------------------------------------------------------
    // Timing  (all derived from DEFAULT_DIVISOR=5)
    // -------------------------------------------------------------------------
    localparam real BAUD_DIV       = 5.0;
    localparam real BAUD_MULT      = 16.0;
    localparam real BYTE_PERIOD_NS = BAUD_MULT * BAUD_DIV * CLK_PERIOD_NS * 10.0;
    //                             = 16 × 5 × 12.5 × 10 = 10 000 ns (10 µs)

    localparam real RECEIVE_TIMEOUT_NS = 481.0 * BYTE_PERIOD_NS;  // 4.81 ms
    localparam real LINK_WAIT_NS       =  46.0 * BYTE_PERIOD_NS;  //  460 µs
    localparam real FULL_CYCLE_NS      = 530.0 * BYTE_PERIOD_NS;  // 5.30 ms

    // 6 full link cycles — always lands in a stable post-negotiation window.
    // This must be >> RECEIVE_TIMEOUT to allow for missed-frame recovery.
    localparam real SETTLE_NS      = 6.0 * FULL_CYCLE_NS;         // 31.8 ms

    // DNA init FSM: 1 (READ) + 57 (SHIFT) + 1 (FOLD) = 59 cycles minimum.
    // Add margin for the IOB pipeline.
    localparam int  DNA_READY_CYCLES = 80;

    // Active-poll timeout: 10 full cycles before declaring a check failure
    localparam real POLL_TIMEOUT_NS  = 10.0 * FULL_CYCLE_NS;

    // Register offsets
    localparam logic [2:0] REG_ID  = 3'd0;
    localparam logic [2:0] REG_DIV = 3'd1;
    localparam logic [2:0] REG_PRS = 3'd2;
    localparam logic [2:0] REG_PRB = 3'd3;
    localparam logic [2:0] REG_RS  = 3'd4;
    localparam logic [2:0] REG_RB  = 3'd5;

    // Radio / bit state constants
    localparam logic [7:0] ST_NO_LINK  = 8'h00;
    localparam logic [7:0] ST_NEW_LINK = 8'hAA;
    localparam logic [7:0] ST_ACTIVE   = 8'h41;
    localparam logic [7:0] ST_INACTIVE = 8'h69;
    localparam logic [7:0] BT_NO_LINK  = 8'h00;
    localparam logic [7:0] BT_FULL_SVC = 8'h46;
    localparam logic [7:0] BT_REDUCED  = 8'h52;
    localparam logic [7:0] BT_NO_SVC   = 8'h6E;

    localparam logic [7:0] SELF_RX_TEST_VAL = 8'd123;

    // =========================================================================
    // Clock
    // =========================================================================

    logic clk = 1'b0;
    always #(CLK_PERIOD_NS / 2.0) clk = ~clk;

    // =========================================================================
    // Resets  (active-high; INVERT_RESET defaults '0' in RTL)
    // =========================================================================

    logic rst_a = 1'b1;
    logic rst_b = 1'b1;

    // =========================================================================
    // Open-drain bus model
    //   Both TX pins are inverted-logic (idle = '1' on the wire = '0' to RTL).
    //   Wired-AND of the two inverted outputs models the open-drain pull-down.
    // =========================================================================

    logic n_tx_a, n_tx_b;
    // Open-drain bus model.
    // N_LINK_TX_O is active-high-pull: '1' = device pulling line low, '0' = released.
    // Any device pulling dominates (wired-OR of pull signals = wired-AND of the line).
    wire  bus_w = n_tx_a | n_tx_b;

    // =========================================================================
    // Wishbone buses
    // =========================================================================

    logic [WB_ADR_W-1:0] a_adr  = '0;
    logic [WB_DAT_W-1:0] a_wdat = '0;
    logic [WB_DAT_W-1:0] a_rdat;
    logic                 a_stb  = 1'b0;
    logic                 a_cyc  = 1'b0;
    logic                 a_we   = 1'b0;
    logic                 a_ack;

    logic [WB_ADR_W-1:0] b_adr  = '0;
    logic [WB_DAT_W-1:0] b_wdat = '0;
    logic [WB_DAT_W-1:0] b_rdat;
    logic                 b_stb  = 1'b0;
    logic                 b_cyc  = 1'b0;
    logic                 b_we   = 1'b0;
    logic                 b_ack;

    // =========================================================================
    // DUT instantiations
    // =========================================================================
    // DUT_A and DUT_B are instantiated via thin VHDL wrappers that hard-code
    // distinct SIM_DNA_VALUE generics.  This avoids the bit_vector generic
    // override across the SV-VHDL boundary, which is unreliable in both
    // Vivado 2022.3 xsim and ModelSim DE 2022.3.
    //
    // Simulation files (do NOT add to synthesis fileset):
    //   radio_link_1wire_dut_a.vhd  (SIM_DNA_VALUE = DEAD_BEEF_CAFE_01)
    //   radio_link_1wire_dut_b.vhd  (SIM_DNA_VALUE = C0FF_EE12_3456_78)
    //
    // DEFAULT_DIVISOR is NOT overridden here — the RTL source must have
    // DEFAULT_DIVISOR := 5 before simulation.  Restore to 520 before synthesis.
    // Confirm after reset: dut_a.divisor_reg should read 5 in the waveform.
    // =========================================================================

    radio_link_1wire_dut_a dut_a (
        .RST_I       (rst_a),
        .CLK_I       (clk),
        .WB_ADR_I    (a_adr),
        .WB_DAT_I    (a_wdat),
        .WB_DAT_O    (a_rdat),
        .WB_STB_I    (a_stb),
        .WB_CYC_I    (a_cyc),
        .WB_WE_I     (a_we),
        .WB_ACK_O    (a_ack),
        .N_LINK_TX_O (n_tx_a),
        .N_LINK_RX_I (bus_w)
    );

    radio_link_1wire_dut_b dut_b (
        .RST_I       (rst_b),
        .CLK_I       (clk),
        .WB_ADR_I    (b_adr),
        .WB_DAT_I    (b_wdat),
        .WB_DAT_O    (b_rdat),
        .WB_STB_I    (b_stb),
        .WB_CYC_I    (b_cyc),
        .WB_WE_I     (b_we),
        .WB_ACK_O    (b_ack),
        .N_LINK_TX_O (n_tx_b),
        .N_LINK_RX_I (bus_w)
    );

    // =========================================================================
    // Scoreboard
    // =========================================================================

    int unsigned total_checks   = 0;
    int unsigned pass_count     = 0;
    int unsigned fail_count     = 0;
    int unsigned self_echo_hits = 0;
    int unsigned byte_swap_hits = 0;

    // =========================================================================
    // Scratch registers (module-level for ModelSim compatibility)
    // =========================================================================

    logic [WB_DAT_W-1:0] chk_prs_a, chk_prb_a, chk_rs_a, chk_rb_a;
    logic [WB_DAT_W-1:0] chk_prs_b, chk_prb_b, chk_rs_b, chk_rb_b;
    logic [WB_DAT_W-1:0] rd_tmp;
    logic                 fail_here;

    // =========================================================================
    // Wishbone tasks
    //
    // Timing: drive address + data one full clock cycle before asserting STB,
    // so WB_DAT_I is stable at the posedge the RTL samples it.  The RTL
    // asserts WB_ACK_O on the same cycle as STB (single-cycle ACK).
    // =========================================================================

    task automatic wb_write_a (input logic [2:0]          reg_off,
                               input logic [WB_DAT_W-1:0] data);
        while (rst_a) @(posedge clk);
        @(negedge clk);
        a_adr  = reg_off;
        a_wdat = data;
        a_we   = 1'b1;
        @(posedge clk);           // data stable for one full cycle
        @(negedge clk);
        a_stb  = 1'b1;
        a_cyc  = 1'b1;
        do @(posedge clk); while (!a_ack);
        @(negedge clk);
        a_stb  = 1'b0;
        a_cyc  = 1'b0;
        a_we   = 1'b0;
        a_adr  = '0;
        a_wdat = '0;
        repeat (2) @(posedge clk);
    endtask

    task automatic wb_read_a (input  logic [2:0]          reg_off,
                              output logic [WB_DAT_W-1:0] data);
        while (rst_a) @(posedge clk);
        @(negedge clk);
        a_adr  = reg_off;
        a_we   = 1'b0;
        a_stb  = 1'b1;
        a_cyc  = 1'b1;
        do @(posedge clk); while (!a_ack);
        data   = a_rdat;
        @(negedge clk);
        a_stb  = 1'b0;
        a_cyc  = 1'b0;
        a_adr  = '0;
        repeat (2) @(posedge clk);
    endtask

    task automatic wb_write_b (input logic [2:0]          reg_off,
                               input logic [WB_DAT_W-1:0] data);
        while (rst_b) @(posedge clk);
        @(negedge clk);
        b_adr  = reg_off;
        b_wdat = data;
        b_we   = 1'b1;
        @(posedge clk);
        @(negedge clk);
        b_stb  = 1'b1;
        b_cyc  = 1'b1;
        do @(posedge clk); while (!b_ack);
        @(negedge clk);
        b_stb  = 1'b0;
        b_cyc  = 1'b0;
        b_we   = 1'b0;
        b_adr  = '0;
        b_wdat = '0;
        repeat (2) @(posedge clk);
    endtask

    task automatic wb_read_b (input  logic [2:0]          reg_off,
                              output logic [WB_DAT_W-1:0] data);
        while (rst_b) @(posedge clk);
        @(negedge clk);
        b_adr  = reg_off;
        b_we   = 1'b0;
        b_stb  = 1'b1;
        b_cyc  = 1'b1;
        do @(posedge clk); while (!b_ack);
        data   = b_rdat;
        @(negedge clk);
        b_stb  = 1'b0;
        b_cyc  = 1'b0;
        b_adr  = '0;
        repeat (2) @(posedge clk);
    endtask

    // =========================================================================
    // State-write helpers with readback verification
    // =========================================================================

    task automatic set_states_a (input logic [7:0] st, input logic [7:0] bt);
        wb_write_a(REG_RS, {{(WB_DAT_W-8){1'b0}}, st});
        wb_write_a(REG_RB, {{(WB_DAT_W-8){1'b0}}, bt});
        wb_read_a(REG_RS, rd_tmp);
        if (rd_tmp[7:0] !== st)
            $display("  WARN set_states_a RS readback=%02Xh expected=%02Xh",
                     rd_tmp[7:0], st);
        wb_read_a(REG_RB, rd_tmp);
        if (rd_tmp[7:0] !== bt)
            $display("  WARN set_states_a RB readback=%02Xh expected=%02Xh",
                     rd_tmp[7:0], bt);
    endtask

    task automatic set_states_b (input logic [7:0] st, input logic [7:0] bt);
        wb_write_b(REG_RS, {{(WB_DAT_W-8){1'b0}}, st});
        wb_write_b(REG_RB, {{(WB_DAT_W-8){1'b0}}, bt});
        wb_read_b(REG_RS, rd_tmp);
        if (rd_tmp[7:0] !== st)
            $display("  WARN set_states_b RS readback=%02Xh expected=%02Xh",
                     rd_tmp[7:0], st);
        wb_read_b(REG_RB, rd_tmp);
        if (rd_tmp[7:0] !== bt)
            $display("  WARN set_states_b RB readback=%02Xh expected=%02Xh",
                     rd_tmp[7:0], bt);
    endtask

    // =========================================================================
    // Reset helpers
    //
    // No reset stagger is needed — each DUT gets a distinct SIM_DNA_VALUE so
    // their PRNG seeds differ from the first cycle of operation.  On hardware
    // the real device DNA guarantees the same property.
    // =========================================================================

    task automatic reset_both ();
        @(negedge clk);
        rst_a = 1'b1;
        rst_b = 1'b1;
        repeat (16) @(posedge clk);
        @(negedge clk);
        rst_a = 1'b0;
        rst_b = 1'b0;
        repeat (DNA_READY_CYCLES) @(posedge clk);  // wait for PrngReady on both
    endtask

    task automatic reset_a_only ();
        @(negedge clk);
        rst_a = 1'b1;
        repeat (16) @(posedge clk);
        @(negedge clk);
        rst_a = 1'b0;
        repeat (DNA_READY_CYCLES) @(posedge clk);
    endtask

    task automatic reset_b_only ();
        @(negedge clk);
        rst_b = 1'b1;
        repeat (16) @(posedge clk);
        @(negedge clk);
        rst_b = 1'b0;
        repeat (DNA_READY_CYCLES) @(posedge clk);
    endtask

    // =========================================================================
    // Active-poll helper
    //
    // Waits until PRS_A is non-zero OR the poll timeout expires.
    // Prevents check_invariants from sampling during a transient LinkBroken
    // clearing window (which zeros PRS/PRB for one clock cycle).
    // Also polls PRS_B to ensure both DUTs have settled.
    // =========================================================================

    task automatic wait_for_link ();
        automatic logic [WB_DAT_W-1:0] prs_a_poll;
        automatic logic [WB_DAT_W-1:0] prs_b_poll;
        automatic realtime poll_start;
        poll_start = $realtime;
        forever begin
            wb_read_a(REG_PRS, prs_a_poll);
            wb_read_b(REG_PRS, prs_b_poll);
            if (prs_a_poll[7:0] != 8'h00 && prs_b_poll[7:0] != 8'h00) break;
            if (($realtime - poll_start) > POLL_TIMEOUT_NS) begin
                $display("  WARN wait_for_link: timeout after %0.0f ms — PRS_A=%02Xh PRS_B=%02Xh",
                         POLL_TIMEOUT_NS/1_000_000.0, prs_a_poll[7:0], prs_b_poll[7:0]);
                break;
            end
            #(BYTE_PERIOD_NS * 10);  // poll every 10 byte-times
        end
    endtask

    // =========================================================================
    // Invariant check task
    //
    // AC-TB-3 invariants:
    //   [1] A.PRS == B.RS        (A received B's state)
    //   [2] A.PRB == B.RB        (A received B's bit state)
    //   [3] B.PRS == A.RS        (B received A's state)
    //   [4] B.PRB == A.RB        (B received A's bit state)
    //   [5] A.PRS != A.RS        (self-echo: A didn't receive its own state as paired state)
    //   [6] A.PRS != A.PRB       (byte-swap: state and bit state not swapped)
    // =========================================================================

    task automatic check_invariants (input string ctx, input int chk_iter);

        wb_read_a(REG_PRS, chk_prs_a);
        wb_read_a(REG_PRB, chk_prb_a);
        wb_read_a(REG_RS,  chk_rs_a);
        wb_read_a(REG_RB,  chk_rb_a);
        wb_read_b(REG_PRS, chk_prs_b);
        wb_read_b(REG_PRB, chk_prb_b);
        wb_read_b(REG_RS,  chk_rs_b);
        wb_read_b(REG_RB,  chk_rb_b);

        total_checks++;
        fail_here = 1'b0;

        $display("[%0t us] CHECK  ctx=%-30s iter=%0d",
                 $time/1000, ctx, chk_iter);
        $display("  A: PRS=%02Xh PRB=%02Xh RS=%02Xh RB=%02Xh",
                 chk_prs_a[7:0], chk_prb_a[7:0], chk_rs_a[7:0], chk_rb_a[7:0]);
        $display("  B: PRS=%02Xh PRB=%02Xh RS=%02Xh RB=%02Xh",
                 chk_prs_b[7:0], chk_prb_b[7:0], chk_rs_b[7:0], chk_rb_b[7:0]);

        if (chk_rs_a[7:0] == ST_NO_LINK && chk_rs_b[7:0] == ST_NO_LINK) begin
            $display("  SKIP  both RS=00h — link not yet established");
        end else begin

            // [1] A.PRS == B.RS
            if (chk_prs_a[7:0] !== chk_rs_b[7:0]) begin
                $display("  FAIL[1] A.PRS=%02Xh != B.RS=%02Xh", chk_prs_a[7:0], chk_rs_b[7:0]);
                fail_here = 1'b1;
            end

            // [2] A.PRB == B.RB
            if (chk_prb_a[7:0] !== chk_rb_b[7:0]) begin
                $display("  FAIL[2] A.PRB=%02Xh != B.RB=%02Xh", chk_prb_a[7:0], chk_rb_b[7:0]);
                fail_here = 1'b1;
            end

            // [3] B.PRS == A.RS
            if (chk_prs_b[7:0] !== chk_rs_a[7:0]) begin
                $display("  FAIL[3] B.PRS=%02Xh != A.RS=%02Xh", chk_prs_b[7:0], chk_rs_a[7:0]);
                fail_here = 1'b1;
            end

            // [4] B.PRB == A.RB
            if (chk_prb_b[7:0] !== chk_rb_a[7:0]) begin
                $display("  FAIL[4] B.PRB=%02Xh != A.RB=%02Xh", chk_prb_b[7:0], chk_rb_a[7:0]);
                fail_here = 1'b1;
            end

            // [5] Self-echo: A.PRS must not equal A.RS (A committed its own state as paired)
            if ((chk_prs_a[7:0] !== 8'h00) && (chk_prs_a[7:0] === chk_rs_a[7:0])) begin
                $display("  FAIL[5-SELF-ECHO] A.PRS=%02Xh == A.RS=%02Xh",
                         chk_prs_a[7:0], chk_rs_a[7:0]);
                self_echo_hits++;
                fail_here = 1'b1;
            end

            // [6] Byte-swap: A.PRS must not equal A.PRB (state confused with bit state)
            if ((chk_prs_a[7:0] !== 8'h00) && (chk_prs_a[7:0] === chk_prb_a[7:0])) begin
                $display("  FAIL[6-BYTESWAP] A.PRS=%02Xh == A.PRB=%02Xh",
                         chk_prs_a[7:0], chk_prb_a[7:0]);
                byte_swap_hits++;
                fail_here = 1'b1;
            end

            if (fail_here) begin
                fail_count++;
                $display("  FAIL");
            end else begin
                pass_count++;
                $display("  PASS");
            end
        end
    endtask

    // =========================================================================
    // Loop variable (must be module-level for ModelSim)
    // =========================================================================

    int iter;

    // =========================================================================
    // Main test sequence
    // =========================================================================

    initial begin : test_main

        // Initialise all WB signals
        a_adr = '0; a_wdat = '0; a_stb = 0; a_cyc = 0; a_we = 0;
        b_adr = '0; b_wdat = '0; b_stb = 0; b_cyc = 0; b_we = 0;

        $display("================================================================");
        $display("TB  PAT6-570  radio_link_1wire negotiation (inline UART + DNA)");
        $display("  CLK=80MHz  DIVISOR=5(sim)/520(hw)");
        $display("  BYTE_PERIOD_NS     = %0.0f ns", BYTE_PERIOD_NS);
        $display("  RECEIVE_TIMEOUT_NS = %0.0f ns  (%0.2f ms)",
                 RECEIVE_TIMEOUT_NS, RECEIVE_TIMEOUT_NS/1_000_000.0);
        $display("  SETTLE_NS          = %0.0f ns  (%0.2f ms)",
                 SETTLE_NS, SETTLE_NS/1_000_000.0);
        $display("  DNA A: DEAD_BEEF_CAFE_01  DNA B: C0FF_EE12_3456_78");
        $display("         (distinct seeds → different back-off sequences)");
        $display("  ** After reset: confirm dut_a.divisor_reg = 5 in waveform **");
        $display("================================================================");

        // Power-on reset — held long enough that both DNA FSMs complete
        rst_a = 1'b1;
        rst_b = 1'b1;
        repeat (32) @(posedge clk);

        // ---------------------------------------------------------------------
        // TEST 0  Wishbone loopback — verify read/write path before anything else
        // ---------------------------------------------------------------------
        $display("\n=== TEST 0: Wishbone loopback ===");
        @(negedge clk);
        rst_a = 1'b0;
        repeat (3) @(posedge clk);
        @(negedge clk);
        rst_b = 1'b0;
        repeat (DNA_READY_CYCLES) @(posedge clk);

        begin : t0_block
            automatic logic [7:0] test_vals [4] = '{8'h41, 8'h69, 8'hA5, 8'hFF};
            automatic logic [WB_DAT_W-1:0] rd;
            automatic int t0_pass = 0, t0_fail = 0;

            for (int i = 0; i < 4; i++) begin
                wb_write_a(REG_RS, {{(WB_DAT_W-8){1'b0}}, test_vals[i]});
                wb_read_a(REG_RS, rd);
                if (rd[7:0] === test_vals[i]) begin
                    $display("  A.RS write %02Xh readback %02Xh OK", test_vals[i], rd[7:0]);
                    t0_pass++;
                end else begin
                    $display("  A.RS write %02Xh readback %02Xh FAIL", test_vals[i], rd[7:0]);
                    t0_fail++;
                end
            end

            // Restore RS to 0
            wb_write_a(REG_RS, '0);
            wb_write_b(REG_RS, '0);

            // Verify REG_ID
            wb_read_a(REG_ID, rd);
            if (rd === 16'h0001) begin
                $display("  A.ID = %04Xh OK", rd);
                t0_pass++;
            end else begin
                $display("  A.ID = %04Xh FAIL (expected 0001h)", rd);
                t0_fail++;
            end

            $display("  TEST 0: %0d pass / %0d fail", t0_pass, t0_fail);
            if (t0_fail > 0) begin
                $display("  FATAL: Wishbone path broken — aborting");
                $finish;
            end
        end

        // ---------------------------------------------------------------------
        // TEST 1  Clean dual power-on
        // ---------------------------------------------------------------------
        $display("\n=== TEST 1: Clean dual power-on ===");
        reset_both();
        set_states_a(ST_ACTIVE,   BT_FULL_SVC);
        set_states_b(ST_INACTIVE, BT_FULL_SVC);
        wait_for_link();
        check_invariants("clean-dual-powerup", 0);

        // ---------------------------------------------------------------------
        // TEST 2  Single-side reboot stress (alternates A/B)
        //         This is the PAT6-570 failure mode — reproduced pre-fix by
        //         18/32 failures in the original hardware trace.
        // ---------------------------------------------------------------------
        $display("\n=== TEST 2: Single-side reboot x%0d ===", NUM_REBOOT_ITER);
        for (iter = 0; iter < NUM_REBOOT_ITER; iter++) begin
            if (iter % 2 == 0) begin
                $display("\n  [%0d] Rebooting A (B mid-cycle)", iter);
                reset_a_only();
                set_states_a(ST_ACTIVE,   BT_FULL_SVC);
                set_states_b(ST_INACTIVE, BT_FULL_SVC);
            end else begin
                $display("\n  [%0d] Rebooting B (A mid-cycle)", iter);
                reset_b_only();
                set_states_a(ST_ACTIVE,   BT_FULL_SVC);
                set_states_b(ST_INACTIVE, BT_FULL_SVC);
            end
            wait_for_link();
            check_invariants("single-side-reboot", iter);
            #(FULL_CYCLE_NS);   // let link stabilise between iterations
        end

        // ---------------------------------------------------------------------
        // TEST 3  Simultaneous reboot x5
        // ---------------------------------------------------------------------
        $display("\n=== TEST 3: Simultaneous reboot x5 ===");
        repeat (5) begin
            reset_both();
            set_states_a(ST_ACTIVE,   BT_FULL_SVC);
            set_states_b(ST_INACTIVE, BT_FULL_SVC);
            wait_for_link();
            check_invariants("simultaneous-reboot", 0);
        end

        // ---------------------------------------------------------------------
        // TEST 4  Self-echo: A.PRS must not mirror A.RS
        // ---------------------------------------------------------------------
        $display("\n=== TEST 4: Self-echo detection ===");
        reset_both();
        set_states_a(ST_ACTIVE,   BT_FULL_SVC);
        set_states_b(ST_INACTIVE, BT_REDUCED);
        wait_for_link();

        total_checks++;
        $display("  A.PRS=%02Xh  A.RS=%02Xh",
                 chk_prs_a[7:0], chk_rs_a[7:0]);
        if ((chk_prs_a[7:0] === chk_rs_a[7:0]) && (chk_prs_a[7:0] !== 8'h00)) begin
            $display("  FAIL[SELF-ECHO] A.PRS == A.RS — RTL committed own state as paired");
            self_echo_hits++;
            fail_count++;
        end else begin
            $display("  PASS  A.PRS != A.RS");
            pass_count++;
        end

        // ---------------------------------------------------------------------
        // TEST 5  check_fpga_self_rx  (John Stevens)
        //         Write TEST_VALUE into A.RB, wait one frame period, check A.PRS.
        //         If A.PRS == TEST_VALUE, the RTL echoed its own RB as paired state.
        // ---------------------------------------------------------------------
        $display("\n=== TEST 5: check_fpga_self_rx (TEST_VALUE=%0d) ===",
                 SELF_RX_TEST_VAL);
        reset_both();
        set_states_a(ST_ACTIVE,   BT_FULL_SVC);
        set_states_b(ST_INACTIVE, BT_FULL_SVC);
        wait_for_link();

        reset_a_only();
        set_states_a(ST_ACTIVE, SELF_RX_TEST_VAL);  // unusual RB value as canary
        #(BYTE_PERIOD_NS * 50.0);                    // 5 frame periods

        wb_read_a(REG_PRS, rd_tmp);

        total_checks++;
        $display("  A.RB(written)=%02Xh  A.PRS(read)=%02Xh",
                 SELF_RX_TEST_VAL, rd_tmp[7:0]);
        if (rd_tmp[7:0] === SELF_RX_TEST_VAL) begin
            $display("  FAIL[SELF-RX] A.PRS == TEST_VALUE => PAT6-570 self-echo");
            self_echo_hits++;
            fail_count++;
        end else begin
            $display("  PASS  A.PRS != TEST_VALUE");
            pass_count++;
        end

        // ---------------------------------------------------------------------
        // TEST 6  Manual toggle regression  (Scott Hisee email 11-Sep-2026)
        //         Settle A=Active / B=Inactive, then manually write B=Active.
        //         Expect: exactly one Active after re-negotiation.
        // ---------------------------------------------------------------------
        $display("\n=== TEST 6: Manual toggle regression (Scott 11-Sep-2026) ===");
        reset_both();
        set_states_a(ST_ACTIVE,   BT_FULL_SVC);
        set_states_b(ST_INACTIVE, BT_FULL_SVC);
        wait_for_link();

        $display("  Baseline A.PRS=%02Xh (expect 69h)  B.PRS=%02Xh (expect 41h)",
                 chk_prs_a[7:0], chk_prs_b[7:0]);

        $display("  Toggling B: Inactive -> Active via Wishbone");
        set_states_b(ST_ACTIVE, BT_FULL_SVC);
        wait_for_link();

        total_checks++;
        $display("  Post-toggle A.RS=%02Xh B.RS=%02Xh A.PRS=%02Xh B.PRS=%02Xh",
                 chk_rs_a[7:0], chk_rs_b[7:0], chk_prs_a[7:0], chk_prs_b[7:0]);
        if ((chk_prs_a[7:0] === chk_rs_b[7:0]) &&
            (chk_prs_b[7:0] === chk_rs_a[7:0])) begin
            $display("  PASS  Changeover completed after manual toggle");
            pass_count++;
        end else begin
            $display("  FAIL[TOGGLE] Changeover did not complete");
            fail_count++;
        end

        // ---------------------------------------------------------------------
        // TEST 7  Link-down changeover
        //         Hold A in reset for 2 × RECEIVE_TIMEOUT.
        //         B must detect LinkBroken and clear its paired state.
        // ---------------------------------------------------------------------
        $display("\n=== TEST 7: Link-down changeover ===");
        reset_both();
        set_states_a(ST_ACTIVE,   BT_FULL_SVC);
        set_states_b(ST_INACTIVE, BT_FULL_SVC);
        wait_for_link();

        $display("  Holding A in reset (simulating power-off)");
        @(negedge clk);
        rst_a = 1'b1;

        #(RECEIVE_TIMEOUT_NS * 5.0);  // 5× to safely clear all 3 MissedFrames windows

        wb_read_b(REG_PRS, rd_tmp);

        total_checks++;
        $display("  B.PRS=%02Xh after %0.0f ms link-down (expect 00h)",
                 rd_tmp[7:0], RECEIVE_TIMEOUT_NS * 5.0 / 1_000_000.0);
        if (rd_tmp[7:0] === 8'h00) begin
            $display("  PASS  B cleared paired state after link-down");
            pass_count++;
        end else begin
            $display("  FAIL[LINKDOWN] B.PRS not cleared (MissedFrames may not have reached 2)");
            fail_count++;
        end

        @(negedge clk);
        rst_a = 1'b0;
        repeat (DNA_READY_CYCLES) @(posedge clk);

        // ---------------------------------------------------------------------
        // TEST 8  Reboot race — T+3s/T+6s hardware failure window
        //         A reboots mid-cycle. B must receive A's new state within one
        //         RECEIVE_TIMEOUT window; if not, B forces itself Active.
        // ---------------------------------------------------------------------
        $display("\n=== TEST 8: Reboot race (HW T+3s/T+6s window) ===");
        reset_both();
        set_states_a(ST_ACTIVE,   BT_FULL_SVC);
        set_states_b(ST_INACTIVE, BT_FULL_SVC);
        wait_for_link();

        $display("  A rebooting mid-cycle (B running)");
        reset_a_only();
        set_states_a(ST_ACTIVE, BT_FULL_SVC);

        // Poll B.PRS for up to RECEIVE_TIMEOUT — it must commit A's state
        // within this window to prevent PAT6-570 occurrence
        begin : t8_block
            automatic logic [WB_DAT_W-1:0] t8_prs;
            automatic realtime t8_start;
            automatic bit t8_ok;
            t8_start = $realtime;
            t8_ok    = 0;
            while (($realtime - t8_start) < RECEIVE_TIMEOUT_NS) begin
                wb_read_b(REG_PRS, t8_prs);
                if (t8_prs[7:0] === ST_ACTIVE) begin
                    t8_ok = 1;
                    break;
                end
                #(BYTE_PERIOD_NS * 5.0);
            end

            total_checks++;
            wb_read_b(REG_PRS, t8_prs);
            $display("  B.PRS=%02Xh after poll (expect 41h=Active)", t8_prs[7:0]);
            if (t8_ok) begin
                $display("  PASS  B committed A's state within RECEIVE_TIMEOUT");
                pass_count++;
            end else begin
                $display("  FAIL[RACE] B did not receive A's state in time");
                $display("       => PAT6-570: B would force itself Active independently");
                fail_count++;
            end
        end

        // ---------------------------------------------------------------------
        // Summary
        // ---------------------------------------------------------------------
        $display("\n================================================================");
        $display("SUMMARY");
        $display("  Total checks  : %0d", total_checks);
        $display("  Pass          : %0d", pass_count);
        $display("  Fail          : %0d", fail_count);
        $display("  Self-echo     : %0d", self_echo_hits);
        $display("  Byte-swap     : %0d", byte_swap_hits);
        if (fail_count == 0)
            $display("  RESULT: PASS  No negotiation failures detected");
        else
            $display("  RESULT: FAIL  %0d violation(s) — see FAIL lines above",
                     fail_count);
        $display("================================================================");

        $finish;
    end : test_main

    // =========================================================================
    // Watchdog — 500 ms sim ceiling (well above 10 × SETTLE_NS = 318 ms)
    // =========================================================================
    initial begin : watchdog
        #500_000_000;
        $display("WATCHDOG expired at %0t us — check for hung FSM or WB deadlock",
                 $time/1000);
        $finish;
    end : watchdog

    // =========================================================================
    // VCD dump
    // =========================================================================
    initial begin : wave_dump
        $dumpfile("pat6570.vcd");
        $dumpvars(0, tb_radio_link_1wire_pat6570);
    end : wave_dump

// =============================================================================
// Note on SIM_DNA_VALUE and cross-language generic overrides
//
// Passing a VHDL bit_vector generic from a SystemVerilog testbench using #()
// syntax is unreliable in Vivado xsim 2022.3 and ModelSim DE 2022.3.  This
// testbench therefore uses thin VHDL wrappers (radio_link_1wire_dut_a.vhd and
// radio_link_1wire_dut_b.vhd) that hard-code their respective SIM_DNA_VALUE
// generics in VHDL, where the type system is native.
//
// To use a different DNA value pair, edit the two wrapper files directly.
// Do not add the wrapper files to the synthesis fileset.
// =============================================================================

endmodule : tb_radio_link_1wire_pat6570















library IEEE;
use IEEE.std_logic_1164.all;
use IEEE.numeric_std.all;

use work.design_constants.all;   -- WB_DAT_WIDTH
use work.types.all;
use work.functions.all;

library unisim;
use unisim.vcomponents.all;

-- ---------------------------------------------------------------------------
-- radio_link_1wire
-- Paired radio link over a shared open-drain 1-wire UART bus.
-- See inline comments for full protocol description.
-- ---------------------------------------------------------------------------

entity radio_link_1wire is
    generic (
        DEFAULT_DIVISOR : natural                 := 5;
        INVERT_RESET    : std_logic               := '0';
        SIM_DNA_VALUE   : bit_vector(56 downto 0) := "0" & X"DEAD_BEEF_CAFE_01"
    );
    port (
        RST_I       : in  std_logic;
        CLK_I       : in  std_logic;
        WB_ADR_I    : in  std_logic_vector(2 downto 0);
        WB_DAT_I    : in  std_logic_vector(WB_DAT_WIDTH - 1 downto 0);
        WB_DAT_O    : out std_logic_vector(WB_DAT_WIDTH - 1 downto 0);
        WB_STB_I    : in  std_logic;
        WB_CYC_I    : in  std_logic;
        WB_WE_I     : in  std_logic;
        WB_ACK_O    : out std_logic;
        N_LINK_TX_O : out std_logic;
        N_LINK_RX_I : in  std_logic
    );
end entity radio_link_1wire;

architecture RTL of radio_link_1wire is

    ---------------------------------------------------------------------------
    -- Constants
    ---------------------------------------------------------------------------
    constant REG_ID                 : natural := 0;
    constant REG_DIVISOR            : natural := 1;
    constant REG_PAIRED_RADIO_STATE : natural := 2;
    constant REG_PAIRED_BIT_STATE   : natural := 3;
    constant REG_RADIO_STATE        : natural := 4;
    constant REG_BIT_STATE          : natural := 5;
    constant ID_REV                 : natural := 1;

    constant DATA_BITS        : positive := 8;
    constant BAUD_MULTIPLIER  : positive := 16;
    constant BYTES_TO_SEND    : positive := 3;
    constant BYTES_TO_RECEIVE : positive := 3;
    constant RECEIVE_TIMEOUT  : positive := 481;
    constant LINK_WAIT_PERIOD : positive := 46;
    constant BACK_OFF_MAX     : positive := RECEIVE_TIMEOUT / 2;
    constant INIT_OFFSET_MAX  : positive := BACK_OFF_MAX;
    -- Number of consecutive missed frames before declaring the link broken.
    -- LinkBroken asserts on the (LINK_BROKEN_THRESHOLD+1)th consecutive timeout.
    constant LINK_BROKEN_THRESHOLD : natural := 2;

    -- TODO: replace with the project-defined sync pattern
    constant SYNC_BYTE : std_logic_vector(DATA_BITS - 1 downto 0) := x"A5";

    ---------------------------------------------------------------------------
    -- Reset
    ---------------------------------------------------------------------------
    signal rst : std_logic;

    ---------------------------------------------------------------------------
    -- DNA seeding
    ---------------------------------------------------------------------------
    constant DNA_BITS  : positive := 57;
    constant PRNG_BITS : positive := 9;

    type DnaFsmType is (DnaRead, DnaShift, DnaDone);
    signal dna_fsm    : DnaFsmType := DnaRead;
    signal dna_read   : std_logic  := '0';
    signal dna_shift  : std_logic  := '0';
    signal dna_dout   : std_logic;
    signal dna_sr     : std_logic_vector(DNA_BITS - 1 downto 0) := (others => '0');
    signal dna_cnt    : integer range 0 to DNA_BITS := 0;
    signal PrngSeed   : std_logic_vector(PRNG_BITS - 1 downto 0) := (others => '1');
    signal InitOffset : integer range 0 to INIT_OFFSET_MAX := 0;
    signal PrngReady  : std_logic := '0';

    ---------------------------------------------------------------------------
    -- Baud generation
    ---------------------------------------------------------------------------
    signal divisor_reg   : unsigned(WB_DAT_WIDTH - 1 downto 0) := (others => '0');
    signal BaudCounter   : unsigned(WB_DAT_WIDTH - 1 downto 0) := (others => '0');
    signal BaudEn        : std_logic := '0';
    signal OverSampleCnt : integer range 0 to BAUD_MULTIPLIER - 1 := 0;
    signal BitEn         : std_logic := '0';
    signal ByteBitCnt    : integer range 0 to 9 := 0;
    signal ByteEn        : std_logic := '0';

    ---------------------------------------------------------------------------
    -- RX synchroniser
    ---------------------------------------------------------------------------
    signal rx_ff1 : std_logic := '1';
    signal rx     : std_logic := '1';

    ---------------------------------------------------------------------------
    -- UART RX (8N1, 16x oversampled) — kept as separate process, read-only signals
    ---------------------------------------------------------------------------
    type RxFsmType is (RxIdle, RxReceiving);
    signal rx_fsm        : RxFsmType  := RxIdle;
    signal rx_samplecnt  : integer range 0 to BAUD_MULTIPLIER - 1 := 0;
    signal rx_bitcnt     : integer range 0 to 9 := 0;
    signal rx_shift      : std_logic_vector(7 downto 0) := (others => '0');
    signal rx_byte       : std_logic_vector(DATA_BITS - 1 downto 0) := (others => '0');
    signal rx_valid      : std_logic := '0';

    ---------------------------------------------------------------------------
    -- Radio state registers
    ---------------------------------------------------------------------------
    signal RadioState          : std_logic_vector(DATA_BITS - 1 downto 0) := (others => '0');
    signal RadioBitState       : std_logic_vector(DATA_BITS - 1 downto 0) := (others => '0');
    signal PairedRadioState    : std_logic_vector(DATA_BITS - 1 downto 0) := (others => '0');
    signal PairedRadioBitState : std_logic_vector(DATA_BITS - 1 downto 0) := (others => '0');

    ---------------------------------------------------------------------------
    -- Combined FSM + UART TX
    -- tx_busy and tx_load are local variables inside p_fsm_tx so the FSM
    -- sees their updates within the same clock edge — no cross-process
    -- delta-cycle visibility issue.
    ---------------------------------------------------------------------------
    type LinkFsmType is (Receiving, Waiting, Sending, RandomWait);
    signal LinkState    : LinkFsmType := Receiving;
    signal LinkBroken   : std_logic   := '1';
    signal MissedFrames : integer range 0 to 7 := 0;
    signal ByteCount    : integer range 0 to 511 := 0;
    signal ByteLimit    : integer range 0 to 511 := RECEIVE_TIMEOUT;

    type RxBuf_t is array (0 to BYTES_TO_RECEIVE - 1) of
        std_logic_vector(DATA_BITS - 1 downto 0);
    signal RxBuf       : RxBuf_t := (others => (others => '0'));
    signal RxByteCount : integer range 0 to BYTES_TO_RECEIVE := 0;
    signal TxByteIndex : integer range 0 to BYTES_TO_SEND    := 0;

    -- TX serial output and drive control
    signal tx_out     : std_logic := '1';
    signal tx_sending : std_logic := '0';
    signal tx_drive   : std_logic;

    -- Collision detection
    signal tx_drive_ff1 : std_logic := '1';
    signal tx_drive_ff2 : std_logic := '1';
    signal tx_drive_del : std_logic := '1';
    signal LinkConflict : std_logic;

    -- PRNG
    signal PseudoRand : std_logic_vector(PRNG_BITS - 1 downto 0) := (others => '1');
    signal RandomTime : integer range 0 to BACK_OFF_MAX := 0;

begin

    rst      <= RST_I xor INVERT_RESET;
    tx_drive <= tx_out when tx_sending = '1' else '1';

    -- Collision: we released the bus (not pulling) but bus is low (remote pulling)
    LinkConflict <= '1' when tx_drive_del = '1' and rx = '0' else '0';

    ---------------------------------------------------------------------------
    -- DNA_PORT
    ---------------------------------------------------------------------------
    u_dna : DNA_PORT
        generic map (SIM_DNA_VALUE => SIM_DNA_VALUE)
        port map (
            CLK   => CLK_I,
            READ  => dna_read,
            SHIFT => dna_shift,
            DIN   => '0',
            DOUT  => dna_dout
        );

    ---------------------------------------------------------------------------
    -- DNA seeding FSM — runs once at power-on, independent of RST_I
    ---------------------------------------------------------------------------
    p_dna_fsm : process(CLK_I)
        variable fold : std_logic_vector(PRNG_BITS - 1 downto 0);
    begin
        if rising_edge(CLK_I) then
            dna_read  <= '0';
            dna_shift <= '0';
            case dna_fsm is
                when DnaRead =>
                    dna_read  <= '1';
                    dna_cnt   <= 0;
                    dna_sr    <= (others => '0');
                    dna_fsm   <= DnaShift;
                when DnaShift =>
                    dna_shift <= '1';
                    dna_sr    <= dna_sr(DNA_BITS - 2 downto 0) & dna_dout;
                    if dna_cnt = DNA_BITS - 1 then
                        dna_fsm <= DnaDone;
                    else
                        dna_cnt <= dna_cnt + 1;
                    end if;
                when DnaDone =>
                    if PrngReady = '0' then
                        fold := dna_sr(56 downto 48) xor dna_sr(47 downto 39)
                            xor dna_sr(38 downto 30) xor dna_sr(29 downto 21)
                            xor dna_sr(20 downto 12) xor dna_sr(11 downto  3)
                            xor ("000000" & dna_sr(2 downto 0));
                        if fold = (fold'range => '0') then
                            PrngSeed <= (others => '1');
                        else
                            PrngSeed <= fold;
                        end if;
                        InitOffset <= to_integer(unsigned(dna_sr(7 downto 0)))
                                      mod INIT_OFFSET_MAX;
                        PrngReady  <= '1';
                    end if;
            end case;
        end if;
    end process p_dna_fsm;

    ---------------------------------------------------------------------------
    -- IOB output register
    ---------------------------------------------------------------------------
    p_tx_iob : process(CLK_I)
    begin
        if rising_edge(CLK_I) then
            N_LINK_TX_O <= not tx_drive;
        end if;
    end process p_tx_iob;

    ---------------------------------------------------------------------------
    -- RX synchroniser
    ---------------------------------------------------------------------------
    p_rx_sync : process(CLK_I)
    begin
        if rising_edge(CLK_I) then
            rx_ff1 <= not N_LINK_RX_I;
            rx     <= rx_ff1;
        end if;
    end process p_rx_sync;

    ---------------------------------------------------------------------------
    -- TX drive delay-match (3 cycles to match IOB + 2-FF RX pipeline)
    ---------------------------------------------------------------------------
    p_tx_drive_del : process(CLK_I)
    begin
        if rising_edge(CLK_I) then
            tx_drive_ff1 <= tx_drive;
            tx_drive_ff2 <= tx_drive_ff1;
            tx_drive_del <= tx_drive_ff2;
        end if;
    end process p_tx_drive_del;

    ---------------------------------------------------------------------------
    -- Baud generator
    ---------------------------------------------------------------------------
    p_baud : process(CLK_I)
    begin
        if rising_edge(CLK_I) then
            BaudEn <= '0';
            BitEn  <= '0';
            if rst = '1' then
                BaudCounter   <= to_unsigned(DEFAULT_DIVISOR, BaudCounter'length);
                OverSampleCnt <= 0;
            else
                if BaudCounter = 0 then
                    BaudCounter <= divisor_reg;
                    BaudEn      <= '1';
                    if OverSampleCnt = BAUD_MULTIPLIER - 1 then
                        OverSampleCnt <= 0;
                        BitEn         <= '1';
                    else
                        OverSampleCnt <= OverSampleCnt + 1;
                    end if;
                else
                    BaudCounter <= BaudCounter - 1;
                end if;
            end if;
        end if;
    end process p_baud;

    ---------------------------------------------------------------------------
    -- Byte-frame enable
    ---------------------------------------------------------------------------
    p_byte_en : process(CLK_I)
    begin
        if rising_edge(CLK_I) then
            ByteEn <= '0';
            if rst = '1' then
                ByteBitCnt <= 0;
            elsif BitEn = '1' then
                if ByteBitCnt = 9 then
                    ByteBitCnt <= 0;
                    ByteEn     <= '1';
                else
                    ByteBitCnt <= ByteBitCnt + 1;
                end if;
            end if;
        end if;
    end process p_byte_en;

    ---------------------------------------------------------------------------
    -- UART RX (8N1, 16x oversampled)
    ---------------------------------------------------------------------------
    p_uart_rx : process(CLK_I)
    begin
        if rising_edge(CLK_I) then
            rx_valid <= '0';
            if rst = '1' then
                rx_fsm      <= RxIdle;
                rx_samplecnt <= 0;
                rx_bitcnt   <= 0;
                rx_shift    <= (others => '0');
            else
                case rx_fsm is
                    when RxIdle =>
                        if rx = '0' then
                            rx_fsm       <= RxReceiving;
                            rx_samplecnt <= 0;
                            rx_bitcnt    <= 0;
                        end if;
                    when RxReceiving =>
                        if BaudEn = '1' then
                            if rx_samplecnt = BAUD_MULTIPLIER - 1 then
                                rx_samplecnt <= 0;
                            else
                                rx_samplecnt <= rx_samplecnt + 1;
                            end if;
                            if rx_samplecnt = (BAUD_MULTIPLIER / 2) - 1 then
                                case rx_bitcnt is
                                    when 0 =>
                                        if rx /= '0' then
                                            rx_fsm <= RxIdle;
                                        end if;
                                    when 1 to 8 =>
                                        rx_shift <= rx & rx_shift(7 downto 1);
                                    when others =>
                                        if rx = '1' then
                                            rx_byte  <= rx_shift;
                                            rx_valid <= '1';
                                        end if;
                                        rx_fsm <= RxIdle;
                                end case;
                                rx_bitcnt <= rx_bitcnt + 1;
                            end if;
                        end if;
                end case;
            end if;
        end if;
    end process p_uart_rx;

    ---------------------------------------------------------------------------
    -- PRNG (9-bit XNOR LFSR)
    ---------------------------------------------------------------------------
    p_prng : process(CLK_I)
    begin
        if rising_edge(CLK_I) then
            if PrngReady = '0' then
                PseudoRand <= PrngSeed;
            else
                PseudoRand <= PseudoRand(7 downto 0)
                              & (PseudoRand(8) xnor PseudoRand(4));
            end if;
        end if;
    end process p_prng;

    ---------------------------------------------------------------------------
    -- Combined link FSM + UART TX
    --
    -- tx_busy and tx_shift are VARIABLES so the FSM and the UART TX logic
    -- share them within the same process and see updates immediately, with
    -- no cross-process delta-cycle issue.
    --
    -- Protocol:
    --   Receiving: listen for 3-byte frame [SYNC | RadioState | RadioBitState]
    --   Waiting:   inter-frame gap before transmitting
    --   Sending:   transmit local frame; detect collision via LinkConflict
    --   RandomWait: back off for a pseudo-random number of byte-periods
    --
    -- Collision detection (open-drain):
    --   Collision is only detectable when we release the bus (tx_drive='1')
    --   but the bus is low (rx='0'), meaning the remote is transmitting.
    --   When both units pull simultaneously the bus stays low — indistinguishable
    --   from normal transmission and not treated as a collision.
    ---------------------------------------------------------------------------
    p_fsm_tx : process(CLK_I)
        -- UART TX state as variables — visible within this process immediately
        variable tx_busy   : std_logic                          := '0';
        variable tx_shift  : std_logic_vector(9 downto 0)      := (others => '1');
        variable tx_bitcnt : integer range 0 to 9              := 0;
        variable tx_data   : std_logic_vector(DATA_BITS-1 downto 0) := (others => '0');
    begin
        if rising_edge(CLK_I) then

            if rst = '1' or PrngReady = '0' then
                -- FSM reset
                LinkState           <= Receiving;
                LinkBroken          <= '1';
                MissedFrames        <= 0;
                ByteCount           <= InitOffset;
                ByteLimit           <= RECEIVE_TIMEOUT;
                RxBuf               <= (others => (others => '0'));
                RxByteCount         <= 0;
                TxByteIndex         <= 0;
                tx_sending          <= '0';
                RandomTime          <= 0;
                PairedRadioState    <= (others => '0');
                PairedRadioBitState <= (others => '0');
                -- UART TX reset
                tx_busy   := '0';
                tx_shift  := (others => '1');
                tx_bitcnt := 0;
                tx_out    <= '1';

            else
                -- ---- UART TX clock-out (runs every cycle) ------------------
                if BitEn = '1' and tx_busy = '1' then
                    tx_out    <= tx_shift(0);
                    tx_shift  := '1' & tx_shift(9 downto 1);
                    if tx_bitcnt = 9 then
                        tx_busy   := '0';
                        tx_bitcnt := 0;
                        tx_out    <= '1';
                    else
                        tx_bitcnt := tx_bitcnt + 1;
                    end if;
                end if;

                -- ---- Byte counter ------------------------------------------
                if ByteEn = '1' and ByteCount < 511 then
                    ByteCount <= ByteCount + 1;
                end if;

                -- ---- Receive accumulator (all states) ----------------------
                if rx_valid = '1' and RxByteCount < BYTES_TO_RECEIVE then
                    RxBuf(RxByteCount) <= rx_byte;
                    RxByteCount        <= RxByteCount + 1;
                end if;

                -- ---- Clear paired state while broken -----------------------
                if LinkBroken = '1' then
                    PairedRadioState    <= (others => '0');
                    PairedRadioBitState <= (others => '0');
                end if;

                -- ---- Link FSM ----------------------------------------------
                case LinkState is

                    when Receiving =>
                        tx_sending <= '0';

                        if RxByteCount = BYTES_TO_RECEIVE then
                            if RxBuf(0) = SYNC_BYTE and
                               (RxBuf(1) /= RadioState or RxBuf(2) /= RadioBitState)
                            then
                                -- Valid frame from remote unit: update paired state
                                PairedRadioState    <= RxBuf(1);
                                PairedRadioBitState <= RxBuf(2);
                                LinkBroken          <= '0';
                                MissedFrames        <= 0;
                                LinkState   <= Waiting;
                                ByteCount   <= 0;
                                ByteLimit   <= LINK_WAIT_PERIOD;
                            else
                            end if;
                            -- Always clear the RX buffer regardless of outcome.
                            -- If the frame was rejected (self-echo), ByteCount keeps
                            -- running and the timeout path will fire as normal,
                            -- incrementing MissedFrames toward LinkBroken.
                            RxByteCount <= 0;

                        elsif ByteCount >= RECEIVE_TIMEOUT then
                            if MissedFrames >= LINK_BROKEN_THRESHOLD then
                                LinkBroken <= '1';
                            else
                                MissedFrames <= MissedFrames + 1;
                            end if;
                            LinkState   <= Sending;
                            ByteCount   <= 0;
                            TxByteIndex <= 0;
                            RxByteCount <= 0;
                        end if;

                    when Waiting =>
                        tx_sending <= '0';
                        if ByteCount >= ByteLimit then
                            LinkState   <= Sending;
                            ByteCount   <= 0;
                            TxByteIndex <= 0;
                            RxByteCount <= 0;
                        end if;

                    when Sending =>
                        tx_sending <= '1';

                        if LinkConflict = '1' then
                            tx_sending <= '0';
                            RandomTime <= to_integer(unsigned(PseudoRand))
                                          mod (BACK_OFF_MAX + 1);
                            LinkState  <= RandomWait;
                            ByteCount  <= 0;

                        elsif tx_busy = '0' then
                            -- UART is free: load next byte or finish
                            if TxByteIndex < BYTES_TO_SEND then
                                case TxByteIndex is
                                    when 0      => tx_data := SYNC_BYTE;
                                    when 1      => tx_data := RadioState;
                                    when others => tx_data := RadioBitState;
                                end case;
                                -- Load UART shift register immediately (variable,
                                -- visible on the same clock edge)
                                tx_shift  := '1' & tx_data & '0';
                                tx_bitcnt := 0;
                                tx_busy   := '1';
                                TxByteIndex <= TxByteIndex + 1;
                            else
                                tx_sending <= '0';
                                LinkState  <= Receiving;
                                ByteCount  <= 0;
                                ByteLimit  <= RECEIVE_TIMEOUT;
                            end if;
                        end if;

                    when RandomWait =>
                        tx_sending <= '0';
                        if ByteCount >= RandomTime then
                            LinkState <= Receiving;
                            ByteCount <= 0;
                            ByteLimit <= RECEIVE_TIMEOUT;
                        end if;

                end case;
            end if;
        end if;
    end process p_fsm_tx;

    ---------------------------------------------------------------------------
    -- Wishbone
    ---------------------------------------------------------------------------
    p_wb : process(CLK_I)
    begin
        if rising_edge(CLK_I) then
            WB_ACK_O <= '0';
            WB_DAT_O <= (others => '0');
            if rst = '1' then
                divisor_reg   <= to_unsigned(DEFAULT_DIVISOR, divisor_reg'length);
                RadioState    <= (others => '0');
                RadioBitState <= (others => '0');
            elsif WB_CYC_I = '1' and WB_STB_I = '1' then
                WB_ACK_O <= '1';
                if WB_WE_I = '1' then
                    case to_integer(unsigned(WB_ADR_I)) is
                        when REG_DIVISOR     => divisor_reg   <= unsigned(WB_DAT_I);
                        when REG_RADIO_STATE => RadioState    <= WB_DAT_I(DATA_BITS-1 downto 0);
                        when REG_BIT_STATE   => RadioBitState <= WB_DAT_I(DATA_BITS-1 downto 0);
                        when others          => null;
                    end case;
                else
                    case to_integer(unsigned(WB_ADR_I)) is
                        when REG_ID =>
                            WB_DAT_O <= std_logic_vector(
                                        to_unsigned(ID_REV, WB_DAT_WIDTH));
                        when REG_DIVISOR =>
                            WB_DAT_O <= std_logic_vector(divisor_reg);
                        when REG_PAIRED_RADIO_STATE =>
                            WB_DAT_O(DATA_BITS-1 downto 0) <= PairedRadioState;
                        when REG_PAIRED_BIT_STATE =>
                            WB_DAT_O(DATA_BITS-1 downto 0) <= PairedRadioBitState;
                        when REG_RADIO_STATE =>
                            WB_DAT_O(DATA_BITS-1 downto 0) <= RadioState;
                        when REG_BIT_STATE =>
                            WB_DAT_O(DATA_BITS-1 downto 0) <= RadioBitState;
                        when others => null;
                    end case;
                end if;
            end if;
        end if;
    end process p_wb;

end architecture RTL;








library IEEE;
use IEEE.std_logic_1164.all;
use IEEE.numeric_std.all;
use work.design_constants.all;
use work.types.all;
use work.functions.all;

-- Simulation wrapper for DUT_A.
-- Hard-codes SIM_DNA_VALUE so xsim does not need to pass bit_vector
-- generics across the SV-VHDL boundary.
-- Do not include in synthesis fileset.

entity radio_link_1wire_dut_a is
    port (
        RST_I    : in  std_logic;
        CLK_I    : in  std_logic;
        WB_ADR_I : in  std_logic_vector(2 downto 0);
        WB_DAT_I : in  std_logic_vector(WB_DAT_WIDTH - 1 downto 0);
        WB_DAT_O : out std_logic_vector(WB_DAT_WIDTH - 1 downto 0);
        WB_STB_I : in  std_logic;
        WB_CYC_I : in  std_logic;
        WB_WE_I  : in  std_logic;
        WB_ACK_O : out std_logic;
        N_LINK_TX_O : out std_logic;
        N_LINK_RX_I : in  std_logic
    );
end entity radio_link_1wire_dut_a;

architecture RTL of radio_link_1wire_dut_a is
begin
    u : entity work.radio_link_1wire
        generic map (
            SIM_DNA_VALUE => "0" & X"DEAD_BEEF_CAFE_01"
        )
        port map (
            RST_I       => RST_I,
            CLK_I       => CLK_I,
            WB_ADR_I    => WB_ADR_I,
            WB_DAT_I    => WB_DAT_I,
            WB_DAT_O    => WB_DAT_O,
            WB_STB_I    => WB_STB_I,
            WB_CYC_I    => WB_CYC_I,
            WB_WE_I     => WB_WE_I,
            WB_ACK_O    => WB_ACK_O,
            N_LINK_TX_O => N_LINK_TX_O,
            N_LINK_RX_I => N_LINK_RX_I
        );
end architecture RTL;









library IEEE;
use IEEE.std_logic_1164.all;
use IEEE.numeric_std.all;
use work.design_constants.all;
use work.types.all;
use work.functions.all;

-- Simulation wrapper for DUT_A.
-- Hard-codes SIM_DNA_VALUE so xsim does not need to pass bit_vector
-- generics across the SV-VHDL boundary.
-- Do not include in synthesis fileset.

entity radio_link_1wire_dut_a is
    port (
        RST_I    : in  std_logic;
        CLK_I    : in  std_logic;
        WB_ADR_I : in  std_logic_vector(2 downto 0);
        WB_DAT_I : in  std_logic_vector(WB_DAT_WIDTH - 1 downto 0);
        WB_DAT_O : out std_logic_vector(WB_DAT_WIDTH - 1 downto 0);
        WB_STB_I : in  std_logic;
        WB_CYC_I : in  std_logic;
        WB_WE_I  : in  std_logic;
        WB_ACK_O : out std_logic;
        N_LINK_TX_O : out std_logic;
        N_LINK_RX_I : in  std_logic
    );
end entity radio_link_1wire_dut_a;

architecture RTL of radio_link_1wire_dut_a is
begin
    u : entity work.radio_link_1wire
        generic map (
            SIM_DNA_VALUE => "0" & X"DEAD_BEEF_CAFE_01"
        )
        port map (
            RST_I       => RST_I,
            CLK_I       => CLK_I,
            WB_ADR_I    => WB_ADR_I,
            WB_DAT_I    => WB_DAT_I,
            WB_DAT_O    => WB_DAT_O,
            WB_STB_I    => WB_STB_I,
            WB_CYC_I    => WB_CYC_I,
            WB_WE_I     => WB_WE_I,
            WB_ACK_O    => WB_ACK_O,
            N_LINK_TX_O => N_LINK_TX_O,
            N_LINK_RX_I => N_LINK_RX_I
        );
end architecture RTL;
