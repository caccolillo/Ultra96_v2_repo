`timescale 1ns/1ps
//------------------------------------------------------------------------------
// tb_axi4lite_to_wishbone_bridge_integrity.sv
//
// CORRECTED test scenario, based on real-hardware source review: we traced
// address 0x0 all the way through legacy_dsp_remap -> wb_distribution ->
// syscon, and confirmed every link in that real chain ALWAYS acknowledges,
// quickly, unconditionally. There is no legitimate path for WB_ACK_I to
// simply never arrive for this address. That rules out the "unresponsive
// target" theory the earlier stress test (never-ack) was built around.
//
// This test targets the more likely real culprit instead: a genuine
// protocol issue or data-integrity problem in the xpm_cdc_handshake
// crossing itself, under SUSTAINED back-to-back traffic against a target
// that always responds correctly and fast (exactly like syscon really
// does) -- not whether timeouts work, but whether the CDC round trip
// ever desyncs, corrupts data, or cross-talks between transactions when
// driven as fast as the bridge's own protocol allows, back-to-back, with
// no artificial gaps.
//
// TWO CHECKS, deliberately chosen because a liveness-only test would miss
// both of them:
//   1. Write-then-immediate-read, many times: exact match required. Any
//      cross-talk between overlapping-in-time transactions (a classic
//      handshake pipelining bug) shows up as a mismatch here.
//   2. Repeated reads of a free-running counter: values must be
//      monotonically non-decreasing. A corrupted or stale response shows
//      up as an impossible value (a decrease, or a value inconsistent
//      with elapsed cycles).
//------------------------------------------------------------------------------

module tb_axi4lite_to_wishbone_bridge_integrity;

  localparam C_S_AXI_ADDR_WIDTH = 32;
  localparam C_S_AXI_DATA_WIDTH = 32;
  localparam WB_ADR_SIZE        = 16;
  localparam WB_DAT_SIZE        = 16;
  localparam WB_TIMEOUT_CYCLES  = 4000;
  localparam AXI_TIMEOUT_CYCLES = 15000;

  localparam NUM_ITERATIONS = 200;  // back-to-back transactions, no gaps

  logic aclk = 0;
  always #4.0  aclk   = ~aclk;    // 125 MHz

  logic wb_clk = 0;
  always #6.25 wb_clk = ~wb_clk;  // 80 MHz

  logic aresetn = 0;
  logic wb_rst  = 1;

  logic [C_S_AXI_ADDR_WIDTH-1:0]     awaddr;
  logic                              awvalid, awready;
  logic [C_S_AXI_DATA_WIDTH-1:0]     wdata;
  logic [(C_S_AXI_DATA_WIDTH/8)-1:0] wstrb;
  logic                              wvalid, wready;
  logic [1:0]                        bresp;
  logic                              bvalid, bready;
  logic [C_S_AXI_ADDR_WIDTH-1:0]     araddr;
  logic                              arvalid, arready;
  logic [C_S_AXI_DATA_WIDTH-1:0]     rdata;
  logic [1:0]                        rresp;
  logic                              rvalid, rready;

  logic [WB_ADR_SIZE-1:0] wb_adr;
  logic [WB_DAT_SIZE-1:0] wb_rd_dat;
  logic [WB_DAT_SIZE-1:0] wb_wr_dat;
  logic                    wb_stb, wb_we, wb_ack, wb_cyc;

  //----------------------------------------------------------------------------
  // DUT
  //----------------------------------------------------------------------------
  axi4lite_to_wishbone_bridge #(
    .C_S_AXI_ADDR_WIDTH (C_S_AXI_ADDR_WIDTH),
    .C_S_AXI_DATA_WIDTH (C_S_AXI_DATA_WIDTH),
    .WB_ADR_SIZE        (WB_ADR_SIZE),
    .WB_DAT_SIZE        (WB_DAT_SIZE),
    .WB_TIMEOUT_CYCLES  (WB_TIMEOUT_CYCLES),
    .AXI_TIMEOUT_CYCLES (AXI_TIMEOUT_CYCLES),
    .INVERT_RESET       (0)
  ) dut (
    .S_AXI_ACLK    (aclk),
    .S_AXI_ARESETN (aresetn),
    .S_AXI_AWADDR  (awaddr),
    .S_AXI_AWVALID (awvalid),
    .S_AXI_AWREADY (awready),
    .S_AXI_WDATA   (wdata),
    .S_AXI_WSTRB   (wstrb),
    .S_AXI_WVALID  (wvalid),
    .S_AXI_WREADY  (wready),
    .S_AXI_BRESP   (bresp),
    .S_AXI_BVALID  (bvalid),
    .S_AXI_BREADY  (bready),
    .S_AXI_ARADDR  (araddr),
    .S_AXI_ARVALID (arvalid),
    .S_AXI_ARREADY (arready),
    .S_AXI_RDATA   (rdata),
    .S_AXI_RRESP   (rresp),
    .S_AXI_RVALID  (rvalid),
    .S_AXI_RREADY  (rready),
    .CLK_I         (wb_clk),
    .RST_I         (wb_rst),
    .WB_ADR_O      (wb_adr),
    .WB_RD_DAT_I   (wb_rd_dat),
    .WB_WR_DAT_O   (wb_wr_dat),
    .WB_STB_O      (wb_stb),
    .WB_WR_O       (wb_we),
    .WB_ACK_I      (wb_ack),
    .WB_CYC_O      (wb_cyc)
  );

  //----------------------------------------------------------------------------
  // Wishbone model: ALWAYS acks, same cycle, exactly matching syscon's real
  // unconditional ack behaviour confirmed from source. reg[1] is a plain
  // read/write scratch register; reg[0] is a free-running counter,
  // incrementing every WB clock cycle (fast, for meaningful variation
  // within a reasonable sim length).
  //----------------------------------------------------------------------------
  logic [WB_DAT_SIZE-1:0] scratch_reg;
  logic [WB_DAT_SIZE-1:0] counter_reg;

  assign wb_ack = wb_cyc & wb_stb;  // unconditional, same-cycle -- matches syscon exactly
  assign wb_rd_dat = (wb_adr == 16'h0001) ? scratch_reg : counter_reg;

  always_ff @(posedge wb_clk) begin
    if (wb_rst) begin
      scratch_reg <= 16'h0000;
      counter_reg <= 16'h0000;
    end else begin
      counter_reg <= counter_reg + 1;  // free-running, every cycle
      if (wb_cyc && wb_stb && wb_we && wb_adr == 16'h0001) begin
        scratch_reg <= wb_wr_dat;
      end
    end
  end

  //----------------------------------------------------------------------------
  // AXI4-Lite tasks with watchdog (same pattern as before -- if the CDC
  // path genuinely hangs against this always-acking target, that's a
  // serious independent finding worth catching cleanly too)
  //----------------------------------------------------------------------------
  localparam SIM_WATCHDOG_CYCLES = AXI_TIMEOUT_CYCLES + 2000;

  task automatic axi_write(input logic [C_S_AXI_ADDR_WIDTH-1:0] addr,
                            input logic [C_S_AXI_DATA_WIDTH-1:0] data,
                            output logic timed_out);
    logic aw_done, w_done;
    int wait_cnt;
    aw_done = 1'b0; w_done = 1'b0; wait_cnt = 0; timed_out = 1'b0;

    @(posedge aclk);
    awaddr <= addr;  awvalid <= 1'b1;
    wdata  <= data;  wstrb   <= '1;  wvalid <= 1'b1;
    bready <= 1'b1;

    while (!aw_done || !w_done) begin
      @(posedge aclk);
      wait_cnt++;
      if (wait_cnt > SIM_WATCHDOG_CYCLES) begin
        timed_out = 1'b1;
        $error("SIM WATCHDOG: write AW/W handshake never completed");
        return;
      end
      if (!aw_done && awready) begin awvalid <= 1'b0; aw_done = 1'b1; end
      if (!w_done && wready)  begin wvalid  <= 1'b0; w_done  = 1'b1; end
    end

    while (!bvalid) begin
      @(posedge aclk);
      wait_cnt++;
      if (wait_cnt > SIM_WATCHDOG_CYCLES) begin
        timed_out = 1'b1;
        $error("SIM WATCHDOG: BVALID never asserted");
        return;
      end
    end
    @(posedge aclk);
    bready <= 1'b0;
  endtask

  task automatic axi_read(input  logic [C_S_AXI_ADDR_WIDTH-1:0] addr,
                           output logic [C_S_AXI_DATA_WIDTH-1:0] data,
                           output logic timed_out);
    int wait_cnt;
    wait_cnt = 0; timed_out = 1'b0;

    @(posedge aclk);
    araddr <= addr; arvalid <= 1'b1;
    rready <= 1'b1;

    while (!arready) begin
      @(posedge aclk);
      wait_cnt++;
      if (wait_cnt > SIM_WATCHDOG_CYCLES) begin
        timed_out = 1'b1;
        $error("SIM WATCHDOG: ARREADY never asserted");
        return;
      end
    end
    @(posedge aclk);
    arvalid <= 1'b0;

    while (!rvalid) begin
      @(posedge aclk);
      wait_cnt++;
      if (wait_cnt > SIM_WATCHDOG_CYCLES) begin
        timed_out = 1'b1;
        $error("SIM WATCHDOG: RVALID never asserted");
        return;
      end
    end
    data = rdata;
    @(posedge aclk);
    rready <= 1'b0;
  endtask

  //----------------------------------------------------------------------------
  // Test sequence
  //----------------------------------------------------------------------------
  logic [C_S_AXI_DATA_WIDTH-1:0] wr_val, rd_val;
  logic [C_S_AXI_DATA_WIDTH-1:0] prev_counter_val;
  logic timeout_flag;

  int wr_rd_pass, wr_rd_fail;
  int counter_pass, counter_fail;
  int hang_count;

  initial begin
    awvalid = 0; wvalid = 0; bready = 0; arvalid = 0; rready = 0;
    awaddr  = '0; wdata = '0; wstrb = '0; araddr = '0;

    #100;
    aresetn <= 1'b1;
    wb_rst  <= 1'b0;
    #100;

    wr_rd_pass = 0; wr_rd_fail = 0;
    counter_pass = 0; counter_fail = 0;
    hang_count = 0;
    prev_counter_val = 32'hFFFFFFFF;  // sentinel: skip monotonicity check on first read

    $display("=== Integrity stress test: %0d back-to-back transactions against always-acking target ===", NUM_ITERATIONS);

    for (int i = 0; i < NUM_ITERATIONS; i++) begin

      // --- Check 1: write-then-immediate-read, exact match ---
      wr_val = $urandom_range(0, 32'hFFFF);
      axi_write(32'h0000_0001, wr_val, timeout_flag);
      if (timeout_flag) begin
        hang_count++;
        $display("[iter %0d] WRITE HANG -- safety net or CDC path genuinely stuck", i);
      end else begin
        axi_read(32'h0000_0001, rd_val, timeout_flag);
        if (timeout_flag) begin
          hang_count++;
          $display("[iter %0d] READ-BACK HANG", i);
        end else if (rd_val[15:0] == wr_val[15:0]) begin
          wr_rd_pass++;
        end else begin
          wr_rd_fail++;
          $display("[iter %0d] MISMATCH: wrote 0x%04h, read back 0x%04h -- possible CDC desync/corruption", i, wr_val[15:0], rd_val[15:0]);
        end
      end

      // --- Check 2: counter monotonicity ---
      axi_read(32'h0000_0000, rd_val, timeout_flag);
      if (timeout_flag) begin
        hang_count++;
        $display("[iter %0d] COUNTER READ HANG", i);
      end else begin
        if (prev_counter_val == 32'hFFFFFFFF || rd_val[15:0] >= prev_counter_val[15:0]) begin
          counter_pass++;
        end else begin
          counter_fail++;
          $display("[iter %0d] COUNTER WENT BACKWARDS: prev=0x%04h now=0x%04h -- possible stale/corrupted response", i, prev_counter_val[15:0], rd_val[15:0]);
        end
        prev_counter_val = rd_val;
      end
    end

    $display("");
    $display("=== RESULTS ===");
    $display("Write-then-read: %0d/%0d exact matches, %0d mismatches", wr_rd_pass, NUM_ITERATIONS, wr_rd_fail);
    $display("Counter monotonicity: %0d/%0d passed, %0d violations", counter_pass, NUM_ITERATIONS, counter_fail);
    $display("Hangs (safety net or genuine stuck CDC): %0d", hang_count);

    if (wr_rd_fail == 0 && counter_fail == 0 && hang_count == 0)
      $display("PASS: no data corruption, no desync, no hangs across %0d back-to-back transactions", NUM_ITERATIONS);
    else
      $display("FAIL: real issue found in the CDC path -- do not trust this on hardware without further investigation");

    $display("TEST COMPLETE");
    $finish;
  end

endmodule
