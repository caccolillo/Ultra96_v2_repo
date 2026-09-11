// =============================================================================
// Testbench: tb_radio_link_1wire_pat6570
// =============================================================================
// PAT6-570: 1-Wire LINK negotiation failure — both radios can become Active
// after single-side power cycle or soft reboot.
//
// @copyright Copyright (c) 2026: Park Air Systems Ltd. All Rights Reserved.
//
// Topology
// --------
//   DUT_A (radio_link_1wire) <--open-drain wire model--> DUT_B (radio_link_1wire)
//
//   Open-drain bus: N_LINK_TX_O/RX_I are inverted. Bus = wired-OR of
//   inverted TX outputs. Any DUT asserting (driving 1) dominates.
//
// Wishbone address:
//   WB_ADR_I is 3 bits = register offset only (block selected externally)
//
// Register offsets
//   0 = REG_ID         (read: ID_REV)
//   1 = REG_DIVISOR    (R/W: baud divisor)
//   2 = REG_PRS        (read: FPGA-written from 1-wire RX)  0x502
//   3 = REG_PRB        (read: FPGA-written from 1-wire RX)  0x503
//   4 = REG_RS         (R/W: ARM-written local state)        0x504
//   5 = REG_RB         (R/W: ARM-written local bit state)    0x505
//
// State values (from Jira PAT6-570)
//   Radio State: 0x00=no link  0xAA=new link  0x41=active  0x69=inactive
//   Bit State:   0x00=no link  0x46=full svc  0x52=reduced 0x6E=no svc
//
// AC-TB-3 invariants checked after every reboot:
//   [1] A.PRS == B.RS
//   [2] A.PRB == B.RB
//   [3] B.PRS == A.RS
//   [4] B.PRB == A.RB
//   [5] A.PRS != A.RB  (self-echo check, when states differ)
//   [6] A.PRS != B.RB and A.PRS == B.RS  (byte-swap check)
//
// New tests added from Scott Hisee email 11-Sep-2026 and hardware log analysis:
//   TEST 6 — Manual toggle regression
//   TEST 7 — Link-down changeover
//   TEST 8 — Reboot race (T+3s/T+6s window from hardware logs)
//
// Wishbone timing notes:
//   - adr/wdat driven one full cycle before STB assertion
//   - adr/wdat/stb/cyc/we cleared atomically at negedge after ACK
//     (prevents rogue write during signal transitions)
//   - wb_write tasks guard on rst=0 PLUS wait one extra posedge to
//     guarantee the VHDL wb_registers process has exited the reset branch
//   - Reset tasks use #1 after deassertion to force a delta-cycle flush
//     so rst=0 propagates across the SV->VHDL boundary before any write
//
// =============================================================================
// Compatible with: ModelSim DE 2022.3 (mixed VHDL + SV)
// =============================================================================

`timescale 1ns/1ps

module tb_radio_link_1wire_pat6570;

    // =========================================================================
    // Parameters
    // =========================================================================

    localparam real    CLK_PERIOD_NS   = 12.5;   // 80 MHz
    localparam int     WB_DAT_W        = 16;
    localparam int     WB_ADR_W        = 3;
    localparam int     NUM_REBOOT_ITER = 5;

    // Timing derived from RTL constants (DEFAULT_DIVISOR=5 for sim):
    //   One byte = 16 * 5 * 12.5 ns = 1000 ns = 1 us
    localparam real BYTE_PERIOD_NS     = 16.0 * 5.0 * CLK_PERIOD_NS;
    localparam real FRAME_PERIOD_NS    = 3.0  * BYTE_PERIOD_NS;
    localparam real SETTLE_NS          = 2000.0 * BYTE_PERIOD_NS;
    localparam real RECEIVE_TIMEOUT_NS = 481.0  * BYTE_PERIOD_NS;
    localparam real LINK_WAIT_NS       = 46.0   * BYTE_PERIOD_NS;

    // Register offsets
    localparam logic [2:0] REG_ID  = 3'd0;
    localparam logic [2:0] REG_DIV = 3'd1;
    localparam logic [2:0] REG_PRS = 3'd2;
    localparam logic [2:0] REG_PRB = 3'd3;
    localparam logic [2:0] REG_RS  = 3'd4;
    localparam logic [2:0] REG_RB  = 3'd5;

    // State values
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
    // Clock and resets
    // =========================================================================

    logic clk = 1'b0;
    always #(CLK_PERIOD_NS / 2.0) clk = ~clk;

    logic rst_a = 1'b0;
    logic rst_b = 1'b0;

    // =========================================================================
    // Open-drain bus
    // =========================================================================

    logic n_tx_a, n_tx_b;
    // Open-drain bus: N_LINK_TX_O and N_LINK_RX_I are INVERTED signals.
    //   0 = DUT releasing bus (idle)   1 = DUT asserting (pulling bus low)
    // In the inverted domain any DUT asserting pulls the bus low:
    //   bus = n_tx_a OR n_tx_b  (wired-OR of inverted outputs)
    // Equivalent to wired-AND in the non-inverted domain.
    wire  bus_w = n_tx_a | n_tx_b;

    // =========================================================================
    // Wishbone @ DUT_A
    // =========================================================================

    logic [WB_ADR_W-1:0] a_adr  = '0;
    logic [WB_DAT_W-1:0] a_wdat = '0;
    logic [WB_DAT_W-1:0] a_rdat;
    logic                 a_stb  = 1'b0;
    logic                 a_cyc  = 1'b0;
    logic                 a_we   = 1'b0;
    logic                 a_ack;

    // =========================================================================
    // Wishbone @ DUT_B
    // =========================================================================

    logic [WB_ADR_W-1:0] b_adr  = '0;
    logic [WB_DAT_W-1:0] b_wdat = '0;
    logic [WB_DAT_W-1:0] b_rdat;
    logic                 b_stb  = 1'b0;
    logic                 b_cyc  = 1'b0;
    logic                 b_we   = 1'b0;
    logic                 b_ack;

    // =========================================================================
    // DUT instantiations
    // No generic map: uses VHDL defaults DEFAULT_DIVISOR=5, INVERT_RESET='0'
    // =========================================================================

    radio_link_1wire dut_a (
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

    radio_link_1wire dut_b (
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
    // Module-level variables (ModelSim: no declarations inside named blocks)
    // =========================================================================

    int   iter;
    logic reboot_a;

    int unsigned total_checks   = 0;
    int unsigned pass_count     = 0;
    int unsigned fail_count     = 0;
    int unsigned self_echo_hits = 0;
    int unsigned byte_swap_hits = 0;

    logic [WB_DAT_W-1:0] chk_prs_a, chk_prb_a, chk_rs_a, chk_rb_a;
    logic [WB_DAT_W-1:0] chk_prs_b, chk_prb_b, chk_rs_b, chk_rb_b;
    logic                 fail_here;
    logic [WB_DAT_W-1:0] t4_prs_a, t4_rb_a;
    logic [WB_DAT_W-1:0] t5_prs_a;
    logic [WB_DAT_W-1:0] t6_prs_a, t6_prs_b, t6_rs_a, t6_rs_b;
    logic [WB_DAT_W-1:0] t7_prs_b;
    logic [WB_DAT_W-1:0] t8_prs_b;

    // =========================================================================
    // Wishbone helper tasks
    //
    // wb_write timing:
    //   1. Guard: wait until rst=0 AND one extra posedge so the VHDL
    //      wb_registers process has definitely exited the reset branch.
    //      Without this, a write immediately after reset deassertion lands
    //      while RST_I is still seen as '1' inside the VHDL process (delta
    //      cycle ordering across the SV->VHDL boundary in ModelSim).
    //   2. Drive adr/wdat/we at negedge — one full cycle before STB.
    //   3. Assert stb/cyc at the next negedge.
    //   4. Wait for ACK on posedge.
    //   5. Deassert stb/cyc/we AND clear adr/wdat atomically at negedge.
    //      Atomic cleardown prevents a rogue write of 0x0000 to whatever
    //      address the bus transitions through during teardown.
    // =========================================================================

    task automatic wb_write_a (input logic [2:0]          reg_off,
                               input logic [WB_DAT_W-1:0] data);
        // Guard: wait for reset deasserted plus one posedge propagation margin
        while (rst_a) @(posedge clk);
        @(posedge clk);  // one extra cycle: guarantees VHDL has exited reset branch
        // Step 1: drive adr/wdat/we one full cycle before STB
        @(negedge clk);
        a_adr  = reg_off;
        a_wdat = data;
        a_we   = 1'b1;
        @(posedge clk);  // data settles through this posedge
        // Step 2: assert STB/CYC
        @(negedge clk);
        a_stb  = 1'b1;
        a_cyc  = 1'b1;
        // Step 3: wait for ACK
        do @(posedge clk); while (!a_ack);
        // Step 4: deassert all control signals AND clear bus atomically
        @(negedge clk);
        a_stb  = 1'b0;
        a_cyc  = 1'b0;
        a_we   = 1'b0;
        a_adr  = 3'h0;
        a_wdat = 16'h0000;
        repeat (4) @(posedge clk);
    endtask

    task automatic wb_read_a (input  logic [2:0]          reg_off,
                              output logic [WB_DAT_W-1:0] data);
        while (rst_a) @(posedge clk);
        @(posedge clk);
        @(negedge clk);
        a_adr  = reg_off;
        a_we   = 1'b0;
        a_stb  = 1'b1;
        a_cyc  = 1'b1;
        do @(posedge clk); while (!a_ack);
        // #1 after the ACK posedge lets all VHDL deltas for this time step
        // complete before we sample a_rdat. WB_DAT_O is cleared then driven
        // within the same rising edge in VHDL delta cycles — the #1 advances
        // past all deltas so the SV side sees the final settled value.
        #1;
        data   = a_rdat;
        @(negedge clk);
        a_stb  = 1'b0;
        a_cyc  = 1'b0;
        a_adr  = 3'h0;
        repeat (4) @(posedge clk);
    endtask

    task automatic wb_write_b (input logic [2:0]          reg_off,
                               input logic [WB_DAT_W-1:0] data);
        while (rst_b) @(posedge clk);
        @(posedge clk);
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
        b_adr  = 3'h0;
        b_wdat = 16'h0000;
        repeat (4) @(posedge clk);
    endtask

    task automatic wb_read_b (input  logic [2:0]          reg_off,
                              output logic [WB_DAT_W-1:0] data);
        while (rst_b) @(posedge clk);
        @(posedge clk);
        @(negedge clk);
        b_adr  = reg_off;
        b_we   = 1'b0;
        b_stb  = 1'b1;
        b_cyc  = 1'b1;
        do @(posedge clk); while (!b_ack);
        // Same #1 delta settle fix as wb_read_a
        #1;
        data   = b_rdat;
        @(negedge clk);
        b_stb  = 1'b0;
        b_cyc  = 1'b0;
        b_adr  = 3'h0;
        repeat (4) @(posedge clk);
    endtask

    // =========================================================================
    // State-writing helpers
    // =========================================================================

    task automatic set_states_a (input logic [7:0] st, input logic [7:0] bt);
        logic [WB_DAT_W-1:0] rb;
        wb_write_a(REG_RS, {{(WB_DAT_W-8){1'b0}}, st});
        wb_write_a(REG_RB, {{(WB_DAT_W-8){1'b0}}, bt});
        wb_read_a(REG_RS, rb);
        if (rb[7:0] !== st)
            $display("  WARN set_states_a: RS readback %02Xh != written %02Xh",
                     rb[7:0], st);
        wb_read_a(REG_RB, rb);
        if (rb[7:0] !== bt)
            $display("  WARN set_states_a: RB readback %02Xh != written %02Xh",
                     rb[7:0], bt);
    endtask

    task automatic set_states_b (input logic [7:0] st, input logic [7:0] bt);
        logic [WB_DAT_W-1:0] rb;
        wb_write_b(REG_RS, {{(WB_DAT_W-8){1'b0}}, st});
        wb_write_b(REG_RB, {{(WB_DAT_W-8){1'b0}}, bt});
        wb_read_b(REG_RS, rb);
        if (rb[7:0] !== st)
            $display("  WARN set_states_b: RS readback %02Xh != written %02Xh",
                     rb[7:0], st);
        wb_read_b(REG_RB, rb);
        if (rb[7:0] !== bt)
            $display("  WARN set_states_b: RB readback %02Xh != written %02Xh",
                     rb[7:0], bt);
    endtask

    // =========================================================================
    // Reset helpers
    //
    // #1 after deassertion forces a delta-cycle flush so the blocking
    // assignment rst=0 propagates to the VHDL process before any
    // subsequent posedge evaluation. Without this, the VHDL wb_registers
    // process can still see RST_I='1' on the first posedge after the SV
    // assignment due to mixed-language delta-cycle ordering in ModelSim.
    // 32 post-reset cycles guarantees the VHDL process has exited the
    // reset branch before any Wishbone write is attempted.
    // =========================================================================

    task automatic reset_both ();
        @(negedge clk);
        rst_a = 1'b1; rst_b = 1'b1;
        repeat (16) @(posedge clk);
        @(negedge clk);
        rst_a = 1'b0; rst_b = 1'b0;
        #1;                          // delta-cycle flush
        repeat (32) @(posedge clk);
    endtask

    task automatic reset_a_only ();
        @(negedge clk);
        rst_a = 1'b1;
        repeat (16) @(posedge clk);
        @(negedge clk);
        rst_a = 1'b0;
        #1;                          // delta-cycle flush
        repeat (32) @(posedge clk);
    endtask

    task automatic reset_b_only ();
        @(negedge clk);
        rst_b = 1'b1;
        repeat (16) @(posedge clk);
        @(negedge clk);
        rst_b = 1'b0;
        #1;                          // delta-cycle flush
        repeat (32) @(posedge clk);
    endtask

    // =========================================================================
    // Scoreboard check task
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

        $display("[%0t ns] CHECK  ctx=%-25s iter=%0d",
                 $time/1000, ctx, chk_iter);
        $display("  A: PRS=%02Xh PRB=%02Xh RS=%02Xh RB=%02Xh",
                 chk_prs_a[7:0], chk_prb_a[7:0],
                 chk_rs_a[7:0],  chk_rb_a[7:0]);
        $display("  B: PRS=%02Xh PRB=%02Xh RS=%02Xh RB=%02Xh",
                 chk_prs_b[7:0], chk_prb_b[7:0],
                 chk_rs_b[7:0],  chk_rb_b[7:0]);

        if (chk_rs_a[7:0] == ST_NO_LINK && chk_rs_b[7:0] == ST_NO_LINK) begin
            $display("  SKIP (neither radio has link state)");
        end else begin

            // [1] A.PRS == B.RS
            if (chk_prs_a[7:0] !== chk_rs_b[7:0]) begin
                $display("  FAIL[1] A.PRS(502)=%02Xh != B.RS(504)=%02Xh",
                         chk_prs_a[7:0], chk_rs_b[7:0]);
                fail_here = 1'b1; fail_count++;
            end

            // [2] A.PRB == B.RB
            if (chk_prb_a[7:0] !== chk_rb_b[7:0]) begin
                $display("  FAIL[2] A.PRB(503)=%02Xh != B.RB(505)=%02Xh",
                         chk_prb_a[7:0], chk_rb_b[7:0]);
                fail_here = 1'b1; fail_count++;
            end

            // [3] B.PRS == A.RS
            if (chk_prs_b[7:0] !== chk_rs_a[7:0]) begin
                $display("  FAIL[3] B.PRS(502)=%02Xh != A.RS(504)=%02Xh",
                         chk_prs_b[7:0], chk_rs_a[7:0]);
                fail_here = 1'b1; fail_count++;
            end

            // [4] B.PRB == A.RB
            if (chk_prb_b[7:0] !== chk_rb_a[7:0]) begin
                $display("  FAIL[4] B.PRB(503)=%02Xh != A.RB(505)=%02Xh",
                         chk_prb_b[7:0], chk_rb_a[7:0]);
                fail_here = 1'b1; fail_count++;
            end

            // [5] Self-echo: A.PRS != A.RB (only when values non-zero)
            if ((chk_rs_a[7:0]  !== 8'h00) &&
                (chk_rb_a[7:0]  !== 8'h00) &&
                (chk_prs_a[7:0] !== 8'h00)) begin
                if (chk_prs_a[7:0] === chk_rb_a[7:0]) begin
                    $display("  FAIL[5-SELF-ECHO] A.PRS(502)=%02Xh == A.RB(505)=%02Xh",
                             chk_prs_a[7:0], chk_rb_a[7:0]);
                    self_echo_hits++; fail_here = 1'b1; fail_count++;
                end
            end

            // [6] Byte-swap: A.PRS == B.RB and A.PRS != B.RS
            if ((chk_prs_a[7:0] === chk_rb_b[7:0]) &&
                (chk_prs_a[7:0] !== chk_rs_b[7:0])) begin
                $display("  FAIL[6-BYTESWAP] A.PRS(502)=%02Xh == B.RB(505)=%02Xh (B.RS=%02Xh)",
                         chk_prs_a[7:0], chk_rb_b[7:0], chk_rs_b[7:0]);
                byte_swap_hits++; fail_here = 1'b1; fail_count++;
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

        a_adr  = 3'h0;  a_wdat = 16'h0000;
        a_stb  = 1'b0;  a_cyc  = 1'b0;  a_we = 1'b0;
        b_adr  = 3'h0;  b_wdat = 16'h0000;
        b_stb  = 1'b0;  b_cyc  = 1'b0;  b_we = 1'b0;

        // Initial reset — required so DEFAULT_DIVISOR=5 loads into divisor_reg
        rst_a = 1'b1; rst_b = 1'b1;
        repeat (32) @(posedge clk);
        @(negedge clk);
        rst_a = 1'b0; rst_b = 1'b0;
        #1;                          // delta-cycle flush
        repeat (32) @(posedge clk);

        $display("================================================================");
        $display("TB  PAT6-570  1-Wire Link Negotiation Failure Reproduction");
        $display("  Tool  : ModelSim DE 2022.3");
        $display("  Clock : 80 MHz   Baud : 9600   DIVISOR : 5 (sim) / 520 (HW)");
        $display("  Byte period : %0.1f ns   Settle : %0.1f ns",
                 BYTE_PERIOD_NS, SETTLE_NS);
        $display("================================================================");

        // -----------------------------------------------------------------
        // TEST 0  Wishbone loopback — write known values, read back
        //         This test runs first and is independent of all negotiation
        //         logic. If it fails, the Wishbone interface in the testbench
        //         is broken and no other test result is meaningful.
        //         Writes to REG_RS (0x504) and REG_RB (0x505) on both DUTs
        //         using a set of known patterns and reads back immediately.
        // -----------------------------------------------------------------
        $display("\n=== TEST 0: Wishbone loopback (write-then-readback) ===");
        begin
            logic [WB_DAT_W-1:0] rb0;
            logic [7:0] patterns [0:3];
            logic        t0_fail;
            int          p;

            patterns[0] = 8'h41;  // ST_ACTIVE
            patterns[1] = 8'h69;  // ST_INACTIVE
            patterns[2] = 8'h46;  // BT_FULL_SVC
            patterns[3] = 8'hA5;  // arbitrary pattern

            t0_fail = 1'b0;

            for (p = 0; p < 4; p++) begin
                // --- DUT_A REG_RS ---
                wb_write_a(REG_RS, {{(WB_DAT_W-8){1'b0}}, patterns[p]});
                wb_read_a(REG_RS, rb0);
                total_checks++;
                if (rb0[7:0] !== patterns[p]) begin
                    $display("  FAIL[WB-LOOPBACK] A.RS wrote %02Xh read %02Xh",
                             patterns[p], rb0[7:0]);
                    t0_fail = 1'b1; fail_count++;
                end else begin
                    $display("  PASS  A.RS write %02Xh readback %02Xh OK",
                             patterns[p], rb0[7:0]);
                    pass_count++;
                end

                // --- DUT_A REG_RB ---
                wb_write_a(REG_RB, {{(WB_DAT_W-8){1'b0}}, patterns[p]});
                wb_read_a(REG_RB, rb0);
                total_checks++;
                if (rb0[7:0] !== patterns[p]) begin
                    $display("  FAIL[WB-LOOPBACK] A.RB wrote %02Xh read %02Xh",
                             patterns[p], rb0[7:0]);
                    t0_fail = 1'b1; fail_count++;
                end else begin
                    $display("  PASS  A.RB write %02Xh readback %02Xh OK",
                             patterns[p], rb0[7:0]);
                    pass_count++;
                end

                // --- DUT_B REG_RS ---
                wb_write_b(REG_RS, {{(WB_DAT_W-8){1'b0}}, patterns[p]});
                wb_read_b(REG_RS, rb0);
                total_checks++;
                if (rb0[7:0] !== patterns[p]) begin
                    $display("  FAIL[WB-LOOPBACK] B.RS wrote %02Xh read %02Xh",
                             patterns[p], rb0[7:0]);
                    t0_fail = 1'b1; fail_count++;
                end else begin
                    $display("  PASS  B.RS write %02Xh readback %02Xh OK",
                             patterns[p], rb0[7:0]);
                    pass_count++;
                end

                // --- DUT_B REG_RB ---
                wb_write_b(REG_RB, {{(WB_DAT_W-8){1'b0}}, patterns[p]});
                wb_read_b(REG_RB, rb0);
                total_checks++;
                if (rb0[7:0] !== patterns[p]) begin
                    $display("  FAIL[WB-LOOPBACK] B.RB wrote %02Xh read %02Xh",
                             patterns[p], rb0[7:0]);
                    t0_fail = 1'b1; fail_count++;
                end else begin
                    $display("  PASS  B.RB write %02Xh readback %02Xh OK",
                             patterns[p], rb0[7:0]);
                    pass_count++;
                end
            end

            if (t0_fail) begin
                $display("  TEST 0 FAILED — Wishbone read/write path broken.");
                $display("  All subsequent test results are unreliable.");
                $display("  Fix the testbench Wishbone tasks before proceeding.");
            end else begin
                $display("  TEST 0 PASSED — Wishbone read/write path verified.");
            end
        end

        // -----------------------------------------------------------------
        // TEST 1  Clean dual power-on
        // -----------------------------------------------------------------
        $display("\n=== TEST 1: Clean dual power-on ===");
        reset_both();
        set_states_a(ST_ACTIVE,   BT_FULL_SVC);
        set_states_b(ST_INACTIVE, BT_FULL_SVC);
        #(SETTLE_NS);
        check_invariants("clean-dual-powerup", 0);

        // -----------------------------------------------------------------
        // TEST 2  Single-side reboot stress
        //         Matches hardware failure window:
        //           T6-237006 forces Active at T+3s (LINK_WAIT_PERIOD)
        //           T6-216882 times out at T+6s (RECEIVE_TIMEOUT)
        // -----------------------------------------------------------------
        $display("\n=== TEST 2: Single-side reboot x%0d ===", NUM_REBOOT_ITER);

        for (iter = 0; iter < NUM_REBOOT_ITER; iter++) begin

            reboot_a = (iter % 2 == 0);

            if (reboot_a) begin
                $display("\n  [%0d] Rebooting A only (B running mid-cycle)", iter);
                reset_a_only();
                set_states_a(ST_ACTIVE,   BT_FULL_SVC);
                set_states_b(ST_INACTIVE, BT_FULL_SVC);
            end else begin
                $display("\n  [%0d] Rebooting B only (A running mid-cycle)", iter);
                reset_b_only();
                set_states_a(ST_ACTIVE,   BT_FULL_SVC);
                set_states_b(ST_INACTIVE, BT_FULL_SVC);
            end

            #(SETTLE_NS);
            check_invariants("single-side-reboot", iter);
            #(SETTLE_NS);
        end

        // -----------------------------------------------------------------
        // TEST 3  Simultaneous reboot
        // -----------------------------------------------------------------
        $display("\n=== TEST 3: Simultaneous reboot x5 ===");
        repeat (5) begin
            reset_both();
            set_states_a(ST_ACTIVE,   BT_FULL_SVC);
            set_states_b(ST_INACTIVE, BT_FULL_SVC);
            #(SETTLE_NS);
            check_invariants("simultaneous-reboot", 0);
        end

        // -----------------------------------------------------------------
        // TEST 4  Self-echo: A.PRS must not mirror A.RB
        // -----------------------------------------------------------------
        $display("\n=== TEST 4: Self-echo detection ===");
        set_states_a(8'h41, 8'h46);
        set_states_b(8'h69, 8'h52);
        #(FRAME_PERIOD_NS * 3.0);

        wb_read_a(REG_PRS, t4_prs_a);
        wb_read_a(REG_RB,  t4_rb_a);

        total_checks++;
        $display("  A.PRS(502)=%02Xh  A.RB(505)=%02Xh",
                 t4_prs_a[7:0], t4_rb_a[7:0]);
        if ((t4_prs_a[7:0] === t4_rb_a[7:0]) &&
            (t4_prs_a[7:0] !== 8'h00) &&
            (t4_rb_a[7:0]  !== 8'h00)) begin
            $display("  FAIL[SELF-ECHO] A.PRS == A.RB");
            self_echo_hits++; fail_count++;
        end else begin
            $display("  PASS  A.PRS != A.RB");
            pass_count++;
        end

        // -----------------------------------------------------------------
        // TEST 5  check_fpga_self_rx (John Stevens, 18-Aug-2026)
        // -----------------------------------------------------------------
        $display("\n=== TEST 5: check_fpga_self_rx (TEST_VALUE=%0d) ===",
                 SELF_RX_TEST_VAL);

        reset_both();
        set_states_a(ST_ACTIVE,   BT_FULL_SVC);
        set_states_b(ST_INACTIVE, BT_FULL_SVC);
        #(SETTLE_NS);

        reset_a_only();
        wb_write_a(REG_RB, {{(WB_DAT_W-8){1'b0}}, SELF_RX_TEST_VAL});
        set_states_a(ST_ACTIVE, SELF_RX_TEST_VAL);
        #(FRAME_PERIOD_NS * 5.0);

        wb_read_a(REG_PRS, t5_prs_a);

        total_checks++;
        $display("  Wrote A.BIT_STATE(505)=%0d  Read A.PRS(502)=%0d",
                 SELF_RX_TEST_VAL, t5_prs_a[7:0]);
        if (t5_prs_a[7:0] === SELF_RX_TEST_VAL) begin
            $display("  FAIL[SELF-RX] A.PRS == TEST_VALUE => PAT6-570 reproduced");
            self_echo_hits++; fail_count++;
        end else begin
            $display("  PASS  A.PRS != TEST_VALUE");
            pass_count++;
        end

        // -----------------------------------------------------------------
        // TEST 6  Manual toggle regression (Scott Hisee, 11-Sep-2026)
        //         After A=Active/B=Inactive negotiation, toggle B to Active.
        //         Both should re-negotiate to one Active / one Inactive.
        // -----------------------------------------------------------------
        $display("\n=== TEST 6: Manual toggle regression ===");

        reset_both();
        set_states_a(ST_ACTIVE,   BT_FULL_SVC);
        set_states_b(ST_INACTIVE, BT_FULL_SVC);
        #(SETTLE_NS);

        wb_read_a(REG_PRS, t6_prs_a);
        wb_read_b(REG_PRS, t6_prs_b);
        $display("  Baseline: A.PRS=%02Xh (expect %02Xh)  B.PRS=%02Xh (expect %02Xh)",
                 t6_prs_a[7:0], ST_INACTIVE, t6_prs_b[7:0], ST_ACTIVE);

        $display("  Toggling B: Inactive -> Active (manual override)");
        set_states_b(ST_ACTIVE, BT_FULL_SVC);
        #(SETTLE_NS);

        wb_read_a(REG_RS,  t6_rs_a);
        wb_read_b(REG_RS,  t6_rs_b);
        wb_read_a(REG_PRS, t6_prs_a);
        wb_read_b(REG_PRS, t6_prs_b);

        total_checks++;
        $display("  After toggle: A.RS=%02Xh B.RS=%02Xh A.PRS=%02Xh B.PRS=%02Xh",
                 t6_rs_a[7:0], t6_rs_b[7:0], t6_prs_a[7:0], t6_prs_b[7:0]);
        if ((t6_prs_a[7:0] === t6_rs_b[7:0]) &&
            (t6_prs_b[7:0] === t6_rs_a[7:0]) &&
            (t6_prs_a[7:0] !== t6_prs_b[7:0])) begin
            $display("  PASS  Changeover negotiated correctly after manual toggle");
            pass_count++;
        end else begin
            $display("  FAIL[TOGGLE] Changeover did not complete after manual toggle");
            fail_count++;
        end

        // -----------------------------------------------------------------
        // TEST 7  Link-down changeover
        //         Hold A in reset (power-off). B should clear PRS within
        //         RECEIVE_TIMEOUT. Hardware logs confirm this path works.
        // -----------------------------------------------------------------
        $display("\n=== TEST 7: Link-down changeover (power-off simulation) ===");

        reset_both();
        set_states_a(ST_ACTIVE,   BT_FULL_SVC);
        set_states_b(ST_INACTIVE, BT_FULL_SVC);
        #(SETTLE_NS);

        $display("  Holding A in reset (simulating power-off)");
        @(negedge clk);
        rst_a = 1'b1;
        #1;
        #(RECEIVE_TIMEOUT_NS * 2.0);

        wb_read_b(REG_PRS, t7_prs_b);

        total_checks++;
        $display("  B.PRS(502)=%02Xh after link-down (expect 00h)",
                 t7_prs_b[7:0]);
        if (t7_prs_b[7:0] === 8'h00) begin
            $display("  PASS  B correctly cleared PRS after link-down");
            pass_count++;
        end else begin
            $display("  FAIL[LINKDOWN] B.PRS not cleared after link-down");
            fail_count++;
        end

        @(negedge clk);
        rst_a = 1'b0;
        #1;
        repeat (32) @(posedge clk);

        // -----------------------------------------------------------------
        // TEST 8  Reboot race — hardware log T+3s/T+6s window
        //         From messages_237006.txt / messages_216882.txt:
        //           A reboots, forces Active after LINK_WAIT_PERIOD (~3s HW)
        //           B times out after RECEIVE_TIMEOUT (~6s HW) if no frame
        //         With fixed RTL, B must commit A's state within timeout.
        // -----------------------------------------------------------------
        $display("\n=== TEST 8: Reboot race (hardware log T+3s/T+6s window) ===");

        reset_both();
        set_states_a(ST_ACTIVE,   BT_FULL_SVC);
        set_states_b(ST_INACTIVE, BT_FULL_SVC);
        #(SETTLE_NS);

        $display("  A rebooting mid-cycle (B running, peer link active)");
        reset_a_only();
        set_states_a(ST_ACTIVE, BT_FULL_SVC);

        // Wait exactly one RECEIVE_TIMEOUT — the critical window
        #(RECEIVE_TIMEOUT_NS);

        wb_read_b(REG_PRS, t8_prs_b);

        total_checks++;
        $display("  B.PRS(502)=%02Xh at T+RECEIVE_TIMEOUT (expect %02Xh=A's RadioState)",
                 t8_prs_b[7:0], ST_ACTIVE);
        if (t8_prs_b[7:0] === ST_ACTIVE) begin
            $display("  PASS  B committed A's state within RECEIVE_TIMEOUT window");
            pass_count++;
        end else begin
            $display("  FAIL[RACE] B did not receive A's state within timeout window");
            $display("         PAT6-570: B will go Active independently");
            fail_count++;
        end

        // -----------------------------------------------------------------
        // Summary
        // -----------------------------------------------------------------
        $display("\n================================================================");
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
        $display("================================================================");

        $finish;
    end : test_main

    // =========================================================================
    // Watchdog
    // =========================================================================
    initial begin : watchdog
        #(64'd10_000_000_000);
        $display("WATCHDOG TIMEOUT at %0t us", $time/1000);
        $finish;
    end : watchdog

    // =========================================================================
    // VCD waveform dump
    // =========================================================================
    initial begin : wave_dump
        $dumpfile("pat6570.vcd");
        $dumpvars(0, tb_radio_link_1wire_pat6570);
    end : wave_dump

endmodule : tb_radio_link_1wire_pat6570
