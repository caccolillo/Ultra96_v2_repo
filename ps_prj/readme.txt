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
//   Open-drain bus: wired-AND of both TX outputs (active-low).
//   Any DUT pulling low dominates. Each DUT's RX_I sees the shared bus.
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
//   TEST 6 — Manual toggle regression: after negotiation, manually toggling
//            the Inactive radio to Active should trigger changeover on peer
//   TEST 7 — Link-down changeover: Active radio loses link, Inactive peer
//            should go Active within RECEIVE_TIMEOUT
//   TEST 8 — Reboot race: matches the 3s/6s hardware failure window from
//            messages_237006.txt / messages_216882.txt logs
//
// Wishbone write protocol note:
//   The RTL (wb_registers process) samples WB_DAT_I on the rising edge that
//   also asserts WB_ACK_O. adr and wdat MUST be cleared atomically with
//   stb/cyc/we at the negedge after ACK to prevent a rogue zero-write to
//   whatever address is on the bus during signal transitions.
//
// =============================================================================
// Compatible with: ModelSim DE 2022.3 (mixed VHDL + SV, xsim also supported)
// =============================================================================

`timescale 1ns/1ps

module tb_radio_link_1wire_pat6570;

    // =========================================================================
    // Parameters
    // =========================================================================

    localparam real    CLK_PERIOD_NS   = 12.5;   // 80 MHz
    localparam int     WB_DAT_W        = 16;
    localparam int     WB_ADR_W        = 3;
    localparam logic [2:0] WB_BASE     = 3'h0;   // addr is reg offset only
    localparam int     NUM_REBOOT_ITER = 5;       // 5 x ~4ms = ~20ms sim time at divisor=5

    // Timing derived from RTL constants:
    //   BAUD_MULTIPLIER=16, DEFAULT_DIVISOR=520 => 9600 baud @ 80 MHz
    //   One byte = 10 bits = 160 baud-clocks = 160 * 520 * 12.5 ns = 1.04 ms
    //   Sim uses DEFAULT_DIVISOR=5 (VHDL default set for sim; real HW uses 520)
    localparam real BYTE_PERIOD_NS  = 16.0 * 5.0  * CLK_PERIOD_NS;  // ~1 us at divisor=5
    localparam real FRAME_PERIOD_NS = 3.0  * BYTE_PERIOD_NS;         // 3 bytes = ~3 us
    localparam real SETTLE_NS       = 2000.0 * BYTE_PERIOD_NS;       // covers 4*RECEIVE_TIMEOUT

    // Hardware-accurate timeout from RTL constants (RECEIVE_TIMEOUT=481 byte-times)
    localparam real RECEIVE_TIMEOUT_NS = 481.0 * BYTE_PERIOD_NS;

    // Link-wait period from RTL (LINK_WAIT_PERIOD=46 byte-times)
    localparam real LINK_WAIT_NS = 46.0 * BYTE_PERIOD_NS;

    // Register offsets
    localparam logic [2:0] REG_ID  = 3'd0;
    localparam logic [2:0] REG_DIV = 3'd1;
    localparam logic [2:0] REG_PRS = 3'd2;  // PAIRED_RADIO_STATE  0x502
    localparam logic [2:0] REG_PRB = 3'd3;  // PAIRED_BIT_STATE    0x503
    localparam logic [2:0] REG_RS  = 3'd4;  // RADIO_STATE         0x504
    localparam logic [2:0] REG_RB  = 3'd5;  // BIT_STATE           0x505

    // Known state values
    localparam logic [7:0] ST_NO_LINK   = 8'h00;
    localparam logic [7:0] ST_NEW_LINK  = 8'hAA;
    localparam logic [7:0] ST_ACTIVE    = 8'h41;
    localparam logic [7:0] ST_INACTIVE  = 8'h69;
    localparam logic [7:0] BT_NO_LINK   = 8'h00;
    localparam logic [7:0] BT_FULL_SVC  = 8'h46;
    localparam logic [7:0] BT_REDUCED   = 8'h52;
    localparam logic [7:0] BT_NO_SVC    = 8'h6E;

    // Test 5 sentinel value (matches John Stevens' Python test: TEST_VALUE=123)
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
    wire  bus_w = n_tx_a & n_tx_b;  // wired-AND = open-drain

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
    // DUT instantiations  (VHDL entities, bound by xelab mixed-language)
    // =========================================================================

    // No generic map — ModelSim does not pass VHDL generics from SV reliably.
    // Uses VHDL defaults: DEFAULT_DIVISOR=5 (set for sim), INVERT_RESET='0'.
    radio_link_1wire dut_a (
        .RST_I     (rst_a),
        .CLK_I     (clk),
        .WB_ADR_I  (a_adr),
        .WB_DAT_I  (a_wdat),
        .WB_DAT_O  (a_rdat),
        .WB_STB_I  (a_stb),
        .WB_CYC_I  (a_cyc),
        .WB_WE_I   (a_we),
        .WB_ACK_O  (a_ack),
        .N_LINK_TX_O (n_tx_a),
        .N_LINK_RX_I (bus_w)
    );

    radio_link_1wire dut_b (
        .RST_I     (rst_b),
        .CLK_I     (clk),
        .WB_ADR_I  (b_adr),
        .WB_DAT_I  (b_wdat),
        .WB_DAT_O  (b_rdat),
        .WB_STB_I  (b_stb),
        .WB_CYC_I  (b_cyc),
        .WB_WE_I   (b_we),
        .WB_ACK_O  (b_ack),
        .N_LINK_TX_O (n_tx_b),
        .N_LINK_RX_I (bus_w)
    );

    // =========================================================================
    // Loop variables — must be module-level for ModelSim compatibility
    // =========================================================================

    int         iter;
    logic       reboot_a;

    // =========================================================================
    // Scoreboard counters
    // =========================================================================

    int unsigned total_checks   = 0;
    int unsigned pass_count     = 0;
    int unsigned fail_count     = 0;
    int unsigned self_echo_hits = 0;
    int unsigned byte_swap_hits = 0;

    // =========================================================================
    // Scratchpad registers — module-level for xsim/ModelSim compatibility
    // =========================================================================

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
    // =========================================================================
    //
    // KEY FIX: a_adr and a_wdat are cleared ATOMICALLY with a_stb/a_cyc/a_we
    // at the negedge after ACK. This prevents a rogue write of 0x0000 to
    // address 0 (or address 4 during transition) that was overwriting RadioState
    // immediately after each successful write.
    //
    // Write sequence:
    //   negedge: drive adr, wdat, we=1  (data stable one full cycle early)
    //   posedge: data setup complete
    //   negedge: assert stb, cyc
    //   posedge: RTL samples wdat, asserts ack
    //   negedge: deassert stb, cyc, we AND clear adr, wdat atomically
    //   +4 posedge: bus settle

    task automatic wb_write_a (input logic [2:0]          reg_off,
                               input logic [WB_DAT_W-1:0] data);
        while (rst_a) @(posedge clk);
        // Step 1: drive address and data one full cycle before STB
        @(negedge clk);
        a_adr  = reg_off;
        a_wdat = data;
        a_we   = 1'b1;
        // Step 2: let data settle through one posedge
        @(posedge clk);
        // Step 3: assert STB/CYC
        @(negedge clk);
        a_stb = 1'b1;
        a_cyc = 1'b1;
        // Step 4: wait for ACK
        do @(posedge clk); while (!a_ack);
        // Step 5: deassert control AND clear address/data atomically
        // — no window where stb/we/cyc are high with adr/wdat transitioning
        @(negedge clk);
        a_stb  = 1'b0;
        a_cyc  = 1'b0;
        a_we   = 1'b0;
        a_adr  = 3'h0;
        a_wdat = 16'h0000;
        // Step 6: bus settle
        repeat (4) @(posedge clk);
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
        a_adr  = 3'h0;
        repeat (4) @(posedge clk);
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
        b_adr  = 3'h0;
        b_wdat = 16'h0000;
        repeat (4) @(posedge clk);
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
        // Readback verification
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
    // =========================================================================

    task automatic reset_both ();
        @(negedge clk);
        rst_a = 1'b1; rst_b = 1'b1;
        repeat (16) @(posedge clk);
        @(negedge clk);
        rst_a = 1'b0; rst_b = 1'b0;
        repeat (16) @(posedge clk);  // generous settle after reset
    endtask

    task automatic reset_a_only ();
        @(negedge clk);
        rst_a = 1'b1;
        repeat (16) @(posedge clk);
        @(negedge clk);
        rst_a = 1'b0;
        repeat (16) @(posedge clk);  // generous settle after reset
    endtask

    task automatic reset_b_only ();
        @(negedge clk);
        rst_b = 1'b1;
        repeat (16) @(posedge clk);
        @(negedge clk);
        rst_b = 1'b0;
        repeat (16) @(posedge clk);
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

        // Skip only when BOTH radios show no link
        if (chk_rs_a[7:0] == ST_NO_LINK && chk_rs_b[7:0] == ST_NO_LINK) begin
            $display("  SKIP (neither radio has link state)");
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

            // [5] Self-echo: A.PRS != A.RB (only when values non-zero)
            if ((chk_rs_a[7:0]  !== 8'h00) &&
                (chk_rb_a[7:0]  !== 8'h00) &&
                (chk_prs_a[7:0] !== 8'h00)) begin
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
                fail_count++;
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

        // Initialise all Wishbone signals
        a_adr  = 3'h0;  a_wdat = 16'h0000;
        a_stb  = 1'b0;  a_cyc  = 1'b0;  a_we = 1'b0;
        b_adr  = 3'h0;  b_wdat = 16'h0000;
        b_stb  = 1'b0;  b_cyc  = 1'b0;  b_we = 1'b0;

        // Assert reset at time 0 — required so divisor_reg initialises to
        // DEFAULT_DIVISOR=5. Without this, Uart16xBaudEn stays high.
        rst_a = 1'b1;
        rst_b = 1'b1;
        repeat (32) @(posedge clk);
        @(negedge clk);
        rst_a = 1'b0;
        rst_b = 1'b0;
        repeat (16) @(posedge clk);

        $display("================================================================");
        $display("TB  PAT6-570  1-Wire Link Negotiation Failure Reproduction");
        $display("  Tool  : ModelSim DE 2022.3");
        $display("  Clock : 80 MHz   Baud : 9600   DIVISOR : 5 (sim) / 520 (HW)");
        $display("  Byte period : %0.1f ns   Settle : %0.1f ns",
                 BYTE_PERIOD_NS, SETTLE_NS);
        $display("================================================================");

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
        //         Matches hardware failure window from log analysis:
        //           T6-237006 forces Active at T+3s (LINK Down timeout)
        //           T6-216882 times out at T+6s (New link timeout)
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
        $display("  A.PRS(502)=%02Xh  A.RB(505)=%02Xh", t4_prs_a[7:0], t4_rb_a[7:0]);
        if ((t4_prs_a[7:0] === t4_rb_a[7:0]) &&
            (t4_prs_a[7:0] !== 8'h00) &&
            (t4_rb_a[7:0]  !== 8'h00)) begin
            $display("  FAIL[SELF-ECHO] A.PRS == A.RB  (self-receive confirmed)");
            self_echo_hits++;
            fail_count++;
        end else begin
            $display("  PASS  A.PRS != A.RB");
            pass_count++;
        end

        // -----------------------------------------------------------------
        // TEST 5  check_fpga_self_rx (John Stevens, 18-Aug-2026)
        //         Write TEST_VALUE=123 to A.0x505, check A.0x502 != TEST_VALUE
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
            self_echo_hits++;
            fail_count++;
        end else begin
            $display("  PASS  A.PRS != TEST_VALUE");
            pass_count++;
        end

        // -----------------------------------------------------------------
        // TEST 6  Manual toggle regression (Scott Hisee email 11-Sep-2026)
        //         After negotiation (A=Active, B=Inactive), toggle B to Active.
        //         Both radios should re-negotiate — one Active, one Inactive.
        // -----------------------------------------------------------------
        $display("\n=== TEST 6: Manual toggle regression (Scott 11-Sep-2026) ===");

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
        //         Hold A in reset (power-off simulation).
        //         B should clear PRS within RECEIVE_TIMEOUT.
        //         From hardware logs: this path WORKS on real HW.
        // -----------------------------------------------------------------
        $display("\n=== TEST 7: Link-down changeover (power-off simulation) ===");

        reset_both();
        set_states_a(ST_ACTIVE,   BT_FULL_SVC);
        set_states_b(ST_INACTIVE, BT_FULL_SVC);
        #(SETTLE_NS);

        $display("  Holding A in reset (simulating power-off)");
        @(negedge clk);
        rst_a = 1'b1;
        #(RECEIVE_TIMEOUT_NS * 2.0);

        wb_read_b(REG_PRS, t7_prs_b);

        total_checks++;
        $display("  B.PRS(502)=%02Xh after link-down (expect 00h=LinkBroken)",
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
        repeat (16) @(posedge clk);

        // -----------------------------------------------------------------
        // TEST 8  Reboot race — hardware log T+3s/T+6s window
        //         From messages_237006.txt / messages_216882.txt (11-Sep-2026):
        //           A reboots, forces Active after LINK_WAIT_PERIOD
        //           B times out after RECEIVE_TIMEOUT if no valid frame seen
        //         With fixed RTL, B must commit A's state within RECEIVE_TIMEOUT
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
            $display("         This is the PAT6-570 failure: B will go Active independently");
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
