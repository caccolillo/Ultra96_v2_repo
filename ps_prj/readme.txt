// =============================================================================
// @file  tb_radio_link_1wire_pat6570.sv
// @brief Dual-DUT SystemVerilog testbench for radio_link_1wire.
//        Reproduces PAT6-570: 1-wire LINK negotiation failure where both
//        radios become Active after single-side power cycle.
//
// @copyright Copyright (c) 2026: Park Air Systems Ltd. All Rights Reserved.
//
// Compatible with: Vivado 2023.1 xsim (mixed VHDL + SV)
//
// Topology
// --------
//   DUT_A (radio_link_1wire) <--open-drain wire model--> DUT_B (radio_link_1wire)
//
//   Open-drain bus: wired-AND of both TX outputs (active-low).
//   Any DUT pulling low dominates. Each DUT's RX_I sees the shared bus.
//
// Wishbone address: (WB_STB_1_WIRE=5) << 8 | reg_offset = 0x0500 | offset
//
// Register offsets
//   0 = REG_ID                  (read: ID_REV)
//   1 = REG_DIVISOR             (R/W: baud divisor)
//   2 = REG_PAIRED_RADIO_STATE  (read: FPGA-written from 1-wire RX)
//   3 = REG_PAIRED_BIT_STATE    (read: FPGA-written from 1-wire RX)
//   4 = REG_RADIO_STATE         (R/W: ARM-written local state)
//   5 = REG_BIT_STATE           (R/W: ARM-written local bit state)
//
// State values (from Jira PAT6-570)
//   Radio State: 0x00=no link  0xAA=new link  0x41=active  0x69=inactive
//   Bit State:   0x00=no link  0x46=full svc  0x52=reduced  0x6E=no svc
//
// AC-TB-3 invariants checked after every reboot:
//   [1] A.0x502 == B.0x504
//   [2] A.0x503 == B.0x505
//   [3] B.0x502 == A.0x504
//   [4] B.0x503 == A.0x505
//   [5] A.0x502 != A.0x505  (self-echo check, when states differ)
// =============================================================================

`timescale 1ns/1ps

module tb_radio_link_1wire_pat6570;

    // =========================================================================
    // Parameters
    // =========================================================================

    localparam real         CLK_PERIOD_NS        = 12.5;   // 80 MHz
    localparam int          WB_DAT_W             = 16;
    localparam int          WB_ADR_W             = 15;
    localparam logic [14:0] WB_BASE              = 15'h0500;
    localparam int          NUM_REBOOT_ITER       = 50;

    // Timing derived from RTL constants:
    //   BAUD_MULTIPLIER=16, DEFAULT_DIVISOR=520 => 9600 baud @ 80 MHz
    //   One byte = 10 bits = 160 baud-clocks = 160 * 520 * 12.5 ns = 1.04 ms
    localparam real BYTE_PERIOD_NS  = 16.0 * 520.0 * CLK_PERIOD_NS; // ~104 us
    localparam real FRAME_PERIOD_NS = 3.0  * BYTE_PERIOD_NS;         // 2 bytes + margin
    localparam real SETTLE_NS       = 10.0 * BYTE_PERIOD_NS;         // ~1.04 ms

    // Register offsets
    localparam logic [2:0] REG_ID     = 3'd0;
    localparam logic [2:0] REG_DIV    = 3'd1;
    localparam logic [2:0] REG_PRS    = 3'd2; // PAIRED_RADIO_STATE  0x502
    localparam logic [2:0] REG_PRB    = 3'd3; // PAIRED_BIT_STATE    0x503
    localparam logic [2:0] REG_RS     = 3'd4; // RADIO_STATE         0x504
    localparam logic [2:0] REG_RB     = 3'd5; // BIT_STATE           0x505

    // Known state values
    localparam logic [7:0] ST_NO_LINK  = 8'h00;
    localparam logic [7:0] ST_NEW_LINK = 8'hAA;
    localparam logic [7:0] ST_ACTIVE   = 8'h41;
    localparam logic [7:0] ST_INACTIVE = 8'h69;
    localparam logic [7:0] BT_NO_LINK  = 8'h00;
    localparam logic [7:0] BT_FULL_SVC = 8'h46;
    localparam logic [7:0] BT_REDUCED  = 8'h52;
    localparam logic [7:0] BT_NO_SVC   = 8'h6E;

    // Test 5 sentinel value (matches John Stevens' Python test: TEST_VALUE=123)
    localparam logic [7:0] SELF_RX_TEST_VAL = 8'd123;

    // =========================================================================
    // Clock and resets
    // =========================================================================

    logic clk   = 1'b0;
    always #(CLK_PERIOD_NS / 2.0) clk = ~clk;

    logic rst_a = 1'b1;
    logic rst_b = 1'b1;

    // =========================================================================
    // Open-drain bus
    // =========================================================================

    logic n_tx_a, n_tx_b;
    wire  bus_w = n_tx_a & n_tx_b;   // wired-AND = open-drain

    // =========================================================================
    // Wishbone — DUT_A
    // =========================================================================

    logic [WB_ADR_W-1:0] a_adr  = '0;
    logic [WB_DAT_W-1:0] a_wdat = '0;
    logic [WB_DAT_W-1:0] a_rdat;
    logic                 a_stb  = 1'b0;
    logic                 a_cyc  = 1'b0;
    logic                 a_we   = 1'b0;
    logic                 a_ack;

    // =========================================================================
    // Wishbone — DUT_B
    // =========================================================================

    logic [WB_ADR_W-1:0] b_adr  = '0;
    logic [WB_DAT_W-1:0] b_wdat = '0;
    logic [WB_DAT_W-1:0] b_rdat;
    logic                 b_stb  = 1'b0;
    logic                 b_cyc  = 1'b0;
    logic                 b_we   = 1'b0;
    logic                 b_ack;

    // =========================================================================
    // DUT instantiations  (VHDL entities, bound by xelab mixed-language)
    // =========================================================================

    radio_link_1wire #(
        .DEFAULT_DIVISOR (520),
        .INVERT_RESET    (1'b0)
    ) dut_a (
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

    radio_link_1wire #(
        .DEFAULT_DIVISOR (520),
        .INVERT_RESET    (1'b0)
    ) dut_b (
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
    // Scoreboard counters  (module-level so all tasks can access)
    // =========================================================================

    int unsigned total_checks    = 0;
    int unsigned pass_count      = 0;
    int unsigned fail_count      = 0;
    int unsigned self_echo_hits  = 0;
    int unsigned byte_swap_hits  = 0;

    // Scratchpad registers for check task — declared at module level
    // to avoid xsim restrictions on declarations inside begin/end blocks
    logic [WB_DAT_W-1:0] chk_prs_a, chk_prb_a, chk_rs_a, chk_rb_a;
    logic [WB_DAT_W-1:0] chk_prs_b, chk_prb_b, chk_rs_b, chk_rb_b;
    logic [WB_DAT_W-1:0] t4_prs_a, t4_rb_a;
    logic [WB_DAT_W-1:0] t5_prs_a;

    // =========================================================================
    // Wishbone helper tasks
    // =========================================================================

    task automatic wb_write_a (input logic [2:0] reg_off,
                                input logic [WB_DAT_W-1:0] data);
        @(posedge clk);
        a_adr  <= WB_BASE | {{(WB_ADR_W-3){1'b0}}, reg_off};
        a_wdat <= data;
        a_we   <= 1'b1;
        a_stb  <= 1'b1;
        a_cyc  <= 1'b1;
        @(posedge clk);
        while (!a_ack) @(posedge clk);
        a_stb  <= 1'b0;
        a_cyc  <= 1'b0;
        a_we   <= 1'b0;
        @(posedge clk);
    endtask

    task automatic wb_read_a (input  logic [2:0] reg_off,
                               output logic [WB_DAT_W-1:0] data);
        @(posedge clk);
        a_adr  <= WB_BASE | {{(WB_ADR_W-3){1'b0}}, reg_off};
        a_we   <= 1'b0;
        a_stb  <= 1'b1;
        a_cyc  <= 1'b1;
        @(posedge clk);
        while (!a_ack) @(posedge clk);
        data    = a_rdat;
        a_stb  <= 1'b0;
        a_cyc  <= 1'b0;
        @(posedge clk);
    endtask

    task automatic wb_write_b (input logic [2:0] reg_off,
                                input logic [WB_DAT_W-1:0] data);
        @(posedge clk);
        b_adr  <= WB_BASE | {{(WB_ADR_W-3){1'b0}}, reg_off};
        b_wdat <= data;
        b_we   <= 1'b1;
        b_stb  <= 1'b1;
        b_cyc  <= 1'b1;
        @(posedge clk);
        while (!b_ack) @(posedge clk);
        b_stb  <= 1'b0;
        b_cyc  <= 1'b0;
        b_we   <= 1'b0;
        @(posedge clk);
    endtask

    task automatic wb_read_b (input  logic [2:0] reg_off,
                               output logic [WB_DAT_W-1:0] data);
        @(posedge clk);
        b_adr  <= WB_BASE | {{(WB_ADR_W-3){1'b0}}, reg_off};
        b_we   <= 1'b0;
        b_stb  <= 1'b1;
        b_cyc  <= 1'b1;
        @(posedge clk);
        while (!b_ack) @(posedge clk);
        data    = b_rdat;
        b_stb  <= 1'b0;
        b_cyc  <= 1'b0;
        @(posedge clk);
    endtask

    // =========================================================================
    // State-writing helpers
    // =========================================================================

    task automatic set_states_a (input logic [7:0] st, input logic [7:0] bt);
        wb_write_a(REG_RS, {{(WB_DAT_W-8){1'b0}}, st});
        wb_write_a(REG_RB, {{(WB_DAT_W-8){1'b0}}, bt});
    endtask

    task automatic set_states_b (input logic [7:0] st, input logic [7:0] bt);
        wb_write_b(REG_RS, {{(WB_DAT_W-8){1'b0}}, st});
        wb_write_b(REG_RB, {{(WB_DAT_W-8){1'b0}}, bt});
    endtask

    // =========================================================================
    // Reset helpers
    // =========================================================================

    task automatic reset_both ();
        rst_a <= 1'b1; rst_b <= 1'b1;
        repeat (8) @(posedge clk);
        rst_a <= 1'b0; rst_b <= 1'b0;
    endtask

    task automatic reset_a_only ();
        rst_a <= 1'b1;
        repeat (8) @(posedge clk);
        rst_a <= 1'b0;
    endtask

    task automatic reset_b_only ();
        rst_b <= 1'b1;
        repeat (8) @(posedge clk);
        rst_b <= 1'b0;
    endtask

    // =========================================================================
    // Scoreboard check task
    // Uses module-level scratchpad signals (xsim-safe — no declarations
    // inside begin/end blocks)
    // =========================================================================

    task automatic check_invariants (input string ctx, input int iter);
        logic fail_here;

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

        $display("[%0t ns] CHECK  ctx=%-25s iter=%0d",
                 $time/1000, ctx, iter);
        $display("  A: PRS=%02Xh PRB=%02Xh RS=%02Xh RB=%02Xh",
                 chk_prs_a[7:0], chk_prb_a[7:0],
                 chk_rs_a[7:0],  chk_rb_a[7:0]);
        $display("  B: PRS=%02Xh PRB=%02Xh RS=%02Xh RB=%02Xh",
                 chk_prs_b[7:0], chk_prb_b[7:0],
                 chk_rs_b[7:0],  chk_rb_b[7:0]);

        // Skip check if link not yet established on either side
        if (chk_rs_a[7:0] == ST_NO_LINK || chk_rs_b[7:0] == ST_NO_LINK) begin
            $display("  SKIP  (link not established)");
        end else begin

            // [1] A.PRS == B.RS
            if (chk_prs_a[7:0] !== chk_rs_b[7:0]) begin
                $display("  FAIL[1] A.PRS(502)=%02Xh != B.RS(504)=%02Xh",
                         chk_prs_a[7:0], chk_rs_b[7:0]);
                fail_here = 1'b1;
                fail_count++;
            end

            // [2] A.PRB == B.RB
            if (chk_prb_a[7:0] !== chk_rb_b[7:0]) begin
                $display("  FAIL[2] A.PRB(503)=%02Xh != B.RB(505)=%02Xh",
                         chk_prb_a[7:0], chk_rb_b[7:0]);
                fail_here = 1'b1;
                fail_count++;
            end

            // [3] B.PRS == A.RS
            if (chk_prs_b[7:0] !== chk_rs_a[7:0]) begin
                $display("  FAIL[3] B.PRS(502)=%02Xh != A.RS(504)=%02Xh",
                         chk_prs_b[7:0], chk_rs_a[7:0]);
                fail_here = 1'b1;
                fail_count++;
            end

            // [4] B.PRB == A.RB
            if (chk_prb_b[7:0] !== chk_rb_a[7:0]) begin
                $display("  FAIL[4] B.PRB(503)=%02Xh != A.RB(505)=%02Xh",
                         chk_prb_b[7:0], chk_rb_a[7:0]);
                fail_here = 1'b1;
                fail_count++;
            end

            // [5] Self-echo: A.PRS != A.RB  (when A's own RS != RB)
            if (chk_rs_a[7:0] !== chk_rb_a[7:0]) begin
                if (chk_prs_a[7:0] === chk_rb_a[7:0]) begin
                    $display("  FAIL[5-SELF-ECHO] A.PRS(502)=%02Xh == A.RB(505)=%02Xh",
                             chk_prs_a[7:0], chk_rb_a[7:0]);
                    self_echo_hits++;
                    fail_here = 1'b1;
                    fail_count++;
                end
            end

            // [6] Byte-swap: A.PRS == B.RB and A.PRS != B.RS
            if ((chk_prs_a[7:0] === chk_rb_b[7:0]) &&
                (chk_prs_a[7:0] !== chk_rs_b[7:0])) begin
                $display("  FAIL[6-BYTESWAP] A.PRS(502)=%02Xh == B.RB(505)=%02Xh (B.RS=%02Xh)",
                         chk_prs_a[7:0], chk_rb_b[7:0], chk_rs_b[7:0]);
                byte_swap_hits++;
                fail_here = 1'b1;
            end

            if (!fail_here) begin
                pass_count++;
                $display("  PASS");
            end

        end
    endtask

    // =========================================================================
    // Main test sequence
    // =========================================================================

    initial begin : test_main

        int iter;
        logic reboot_a;

        $display("=============================================================");
        $display("TB  PAT6-570  1-Wire Link Negotiation Failure Reproduction");
        $display("    Tool  : Vivado 2023.1 xsim");
        $display("    Clock : 80 MHz   Baud : 9600   DIVISOR : 520");
        $display("    Byte period : %0.0f ns   Settle : %0.0f ns",
                 BYTE_PERIOD_NS, SETTLE_NS);
        $display("=============================================================");

        // ------------------------------------------------------------------
        // TEST 1  Clean dual power-on
        //         Both DUTs reset together — link should negotiate correctly
        // ------------------------------------------------------------------
        $display("\n=== TEST 1: Clean dual power-on ===");
        reset_both();
        set_states_a(ST_ACTIVE,   BT_FULL_SVC);
        set_states_b(ST_INACTIVE, BT_FULL_SVC);
        #(SETTLE_NS);
        check_invariants("clean-dual-powerup", 0);

        // ------------------------------------------------------------------
        // TEST 2  Single-side reboot stress (AC-TB-2: >= 10000 in full run;
        //         NUM_REBOOT_ITER=50 here for rapid reproduction)
        //         DUT_A or DUT_B reboots while the peer is mid-cycle.
        //         With the buggy RTL this reproduces PAT6-570 within ~10 iter.
        // ------------------------------------------------------------------
        $display("\n=== TEST 2: Single-side reboot x%0d ===", NUM_REBOOT_ITER);

        for (iter = 0; iter < NUM_REBOOT_ITER; iter++) begin

            reboot_a = (iter % 2 == 0);

            if (reboot_a) begin
                $display("\n  [%0d] Rebooting A only (B running mid-cycle)", iter);
                reset_a_only();
                set_states_a(ST_ACTIVE,   BT_FULL_SVC);
            end else begin
                $display("\n  [%0d] Rebooting B only (A running mid-cycle)", iter);
                reset_b_only();
                set_states_b(ST_INACTIVE, BT_FULL_SVC);
            end

            // Wait for re-negotiation attempt
            #(FRAME_PERIOD_NS * 20.0);
            check_invariants("single-side-reboot", iter);

            // Short gap between iterations
            #(FRAME_PERIOD_NS * 5.0);
        end

        // ------------------------------------------------------------------
        // TEST 3  Simultaneous reboot — should always recover
        // ------------------------------------------------------------------
        $display("\n=== TEST 3: Simultaneous reboot x5 ===");
        repeat (5) begin
            reset_both();
            set_states_a(ST_ACTIVE,   BT_FULL_SVC);
            set_states_b(ST_INACTIVE, BT_FULL_SVC);
            #(SETTLE_NS);
            check_invariants("simultaneous-reboot", 0);
        end

        // ------------------------------------------------------------------
        // TEST 4  Self-echo: A.PRS(0x502) must not mirror A.RB(0x505)
        //         AC-RTL-1 / AC-TB-3 criterion [5]
        //         Uses module-level signals t4_prs_a, t4_rb_a
        // ------------------------------------------------------------------
        $display("\n=== TEST 4: Self-echo detection ===");
        set_states_a(8'h41, 8'h46);   // Active / Full-service
        set_states_b(8'h69, 8'h52);   // Inactive / Reduced
        #(FRAME_PERIOD_NS * 3.0);

        wb_read_a(REG_PRS, t4_prs_a);
        wb_read_a(REG_RB,  t4_rb_a);

        total_checks++;
        $display("  A.PRS(502)=%02Xh  A.RB(505)=%02Xh", t4_prs_a[7:0], t4_rb_a[7:0]);
        if (t4_prs_a[7:0] === t4_rb_a[7:0]) begin
            $display("  FAIL[SELF-ECHO] A.PRS == A.RB  (self-receive confirmed)");
            self_echo_hits++;
            fail_count++;
        end else begin
            $display("  PASS  A.PRS != A.RB");
            pass_count++;
        end

        // ------------------------------------------------------------------
        // TEST 5  Jira check_fpga_self_rx equivalent
        //         (John Stevens comment 18-Aug-2026)
        //         Write SELF_RX_TEST_VAL=123 to A.0x505.
        //         In broken state A.0x502 follows A.0x505.
        //         Uses module-level signal t5_prs_a
        // ------------------------------------------------------------------
        $display("\n=== TEST 5: check_fpga_self_rx (TEST_VALUE=%0d) ===",
                 SELF_RX_TEST_VAL);

        reset_both();
        set_states_a(ST_ACTIVE,   BT_FULL_SVC);
        set_states_b(ST_INACTIVE, BT_FULL_SVC);
        #(SETTLE_NS);

        // Single-side reboot of A — this is the trigger
        reset_a_only();
        wb_write_a(REG_RB, {{(WB_DAT_W-8){1'b0}}, SELF_RX_TEST_VAL});
        set_states_a(ST_ACTIVE, SELF_RX_TEST_VAL);

        // Wait for A to transmit and potentially self-receive
        #(FRAME_PERIOD_NS * 5.0);

        wb_read_a(REG_PRS, t5_prs_a);

        total_checks++;
        $display("  Wrote A.BIT_STATE(505)=%0d  Read A.PRS(502)=%0d",
                 SELF_RX_TEST_VAL, t5_prs_a[7:0]);
        if (t5_prs_a[7:0] === SELF_RX_TEST_VAL) begin
            $display("  FAIL[SELF-RX] A.PRS == TEST_VALUE => PAT6-570 reproduced");
            self_echo_hits++;
            fail_count++;
        end else begin
            $display("  PASS  A.PRS != TEST_VALUE");
            pass_count++;
        end

        // ------------------------------------------------------------------
        // Summary
        // ------------------------------------------------------------------
        $display("\n=============================================================");
        $display("SUMMARY");
        $display("  Total checks  : %0d", total_checks);
        $display("  Pass          : %0d", pass_count);
        $display("  Fail          : %0d", fail_count);
        $display("  Self-echo     : %0d", self_echo_hits);
        $display("  Byte-swap     : %0d", byte_swap_hits);

        if (fail_count > 0) begin
            $display("  RESULT: FAIL  PAT6-570 reproduced (%0d violations)",
                     fail_count);
            $display("          Fix required: AC-RTL-1..4 (see Jira PAT6-570)");
        end else begin
            $display("  RESULT: PASS  No negotiation failures detected");
        end
        $display("=============================================================");

        $finish;
    end : test_main

    // =========================================================================
    // Watchdog  — prevents infinite run if WB ack never arrives
    // =========================================================================
    initial begin : watchdog
        #(200_000_000); // 200 ms sim time ceiling
        $display("WATCHDOG TIMEOUT at %0t ns", $time);
        $finish;
    end : watchdog

    // =========================================================================
    // VCD waveform dump  (xsim converts to .wdb automatically with -wdb flag)
    // =========================================================================
    initial begin : wave_dump
        $dumpfile("pat6570.vcd");
        $dumpvars(0, tb_radio_link_1wire_pat6570);
    end : wave_dump

endmodule : tb_radio_link_1wire_pat6570









---------------------------------------------------------------------------------------------------------------------------

Setup

Two instances of radio_link_1wire — DUT_A and DUT_B — are connected together on a shared open-drain bus modelled as a wired-AND:

bus = n_tx_a AND n_tx_b

Both DUTs receive the same bus on their N_LINK_RX_I pin. This exactly matches the real hardware wiring. Because the bus is active-low open-drain, anything a DUT transmits also loops back into its own receive input — which is the root cause of PAT6-570.

A Wishbone master in the testbench talks to each DUT independently to write state values and read back registers, exactly as the ARM processor does in the real system.

TEST 1 — Clean dual power-on

Both DUTs are reset together, then released at the same time. The ARM state registers are written:

DUT_A: RadioState = 0x41 (Active), BitState = 0x46 (Full service)
DUT_B: RadioState = 0x69 (Inactive), BitState = 0x46 (Full service)

The testbench waits ~1 ms for the link to settle (10 UART byte periods), then reads all four paired/local registers from both DUTs and checks the four AC-TB-3 invariants:

A.0x502 == B.0x504    (A's view of B's state == B's actual state)
A.0x503 == B.0x505    (A's view of B's bit state == B's actual bit state)
B.0x502 == A.0x504    (B's view of A's state == A's actual state)
B.0x503 == A.0x505    (B's view of A's bit state == A's actual bit state)

This establishes a baseline. With the buggy RTL, this may already fail on first run depending on timing.

TEST 2 — Single-side reboot stress (50 iterations)

This is the core PAT6-570 reproduction test. It loops 50 times. On even iterations it reboots DUT_A only while DUT_B keeps running; on odd iterations it reboots DUT_B only while DUT_A keeps running.

Each iteration:

Assert reset on one DUT for 8 clock cycles, release it
Re-write the ARM state registers on the rebooted DUT (simulating MCP restart)
Wait ~2 ms for re-negotiation to attempt
Read all 8 registers (4 from each DUT) and check all 4 invariants plus two extra checks:

Self-echo check: A.0x502 == A.0x505 — if A's paired radio state matches A's own bit state, A has received its own transmission and committed it as the peer's data.

Byte-swap check: A.0x502 == B.0x505 and A.0x502 != B.0x504 — the state register contains the bit-state byte and vice versa, which is the exact register swap John Stevens documented in the Jira comments.

With the unmodified RTL the field reports show failure within ~10 cycles. The testbench prints a FAIL line for each violation as it happens so you can see exactly which iteration triggers it and which invariant breaks first.

TEST 3 — Simultaneous reboot (5 repetitions)

Both DUTs are reset together 5 times with the same state values as TEST 1. This should always pass because both radios start from the same initial condition and race symmetrically. It serves as a sanity check — if this fails, something is wrong with the testbench setup rather than the RTL bug.

TEST 4 — Direct self-echo detection

Without any reboot, distinctive values are written to both DUTs:

DUT_A: 0x41 / 0x46
DUT_B: 0x69 / 0x52

After 3 frame periods, DUT_A's paired radio state register (0x502) is read back and compared against DUT_A's own bit state register (0x505). If they match, DUT_A has self-received its own transmission — AC-RTL-1 is violated.

TEST 5 — Jira check_fpga_self_rx equivalent

This is a direct port of the Python test function John Stevens posted in the Jira comments on 18 August 2026. It works as follows:

Both DUTs start cleanly and settle
DUT_A only is rebooted
The value 123 (decimal) is written to DUT_A's BitState register (0x505)
After 5 frame periods, DUT_A's PairedRadioState register (0x502) is read back
If A.0x502 == 123, DUT_A has self-received — its own transmitted bit state has come back through the open-drain loopback and been committed as the peer's radio state

John Stevens ran this test over 40 times without the link cable connected and it passed every time. With the cable connected it failed after 3 iterations, because the cable creates the open-drain loopback path that makes self-receive possible. The testbench replicates that path in simulation.

What the output looks like when the bug is present

[iter 3] Rebooting DUT_A only (DUT_B running)
  A: PRS=46h PRB=41h RS=41h RB=46h
  B: PRS=41h PRB=46h RS=69h RB=46h
  FAIL[1] A.PRS(502)=46h != B.RS(504)=69h
  FAIL[5-SELF-ECHO] A.PRS(502)=46h == A.RB(505)=46h
  FAIL[6-BYTESWAP]  A.PRS(502)=46h == B.RB(505)=46h (B.RS=69h)

That output shows all three failure modes simultaneously: A's paired state register contains 0x46 (Full Service — a bit-state value), which matches A's own bit state (self-echo) and also matches B's bit state (byte-swap), when it should contain B's radio state 0x69 (Inactive).

What the testbench does NOT do

It does not test the fix — it only reproduces the bug. Once AC-RTL-1 through AC-RTL-4 are implemented (self-echo rejection, frame sync bytes, buffer clearing, validation gate), all five tests should pass. The testbench then becomes the regression gate that confirms the fix holds across 50 reboot cycles.
