`timescale 1ns/1ps
//------------------------------------------------------------------------------
// tb_axi4lite_to_wishbone_bridge_integrity.sv
//
// Integrity stress test: 200 back-to-back write-then-read transactions
// against an always-acking Wishbone target model, plus counter monotonicity
// check. Tests CDC data integrity, not just liveness.
//
// FIX: The WB model uses a Mealy-style read path -- the read data mux
// is combinatorial off wb_adr (always valid while CYC/STB are high), so
// WB_RD_DAT_I is stable when the bridge samples it in WB_WAIT_ACK.
// scratch_reg is written in the same clock edge as the ACK -- SV
// non-blocking assignments mean the OLD value is read by the bridge
// (correct Wishbone behaviour: write completes, THEN the next read
// sees the new value). The write-then-read test exercises exactly this:
// write addr 1, then read addr 1 -- two separate AXI transactions,
// so the read always happens AFTER the write has committed.
//
// Root cause of 0/200: the WB model's scratch_reg write condition used
//   if (wb_cyc && wb_stb && wb_we && wb_adr == 16'h0001)
// which fires on the ACK edge -- but the bridge also reads WB_RD_DAT_I
// on that same edge (in WB_WAIT_ACK). Because both are clocked on the
// same wb_clk rising edge, scratch_reg hasn't committed yet when the
// bridge samples wb_rd_dat. This only matters for a read that arrives
// in the SAME clock edge as a write -- which can't happen across two
// separate AXI transactions. So the real fix is: the bridge must NOT
// sample WB_RD_DAT_I on the ACK edge when the target uses a registered
// (non-transparent) memory model. Instead, sample one cycle later --
// restore WB_CAPTURE state. But for this testbench, the simpler fix is
// to make scratch_reg immediately visible by using a continuous assign
// for the write (i.e. make the WB target model's read path see the
// new value in the same cycle as the write, as a real synchronous SRAM
// with registered output would NOT do, but as a register file with
// combinatorial read DOES do). The cleanest fix: keep the registered
// write but add one pipeline register on the read side so the captured
// value is always the post-write value.
//
// ACTUAL FIX APPLIED: restore WB_CAPTURE in the bridge (sample
// WB_RD_DAT_I one cycle after ACK, when scratch_reg has committed),
// AND ensure WB_ADR_O and WB_STB_O/WB_CYC_O are held stable through
// that extra cycle so the mux output is valid.
//------------------------------------------------------------------------------

module tb_axi4lite_to_wishbone_bridge_integrity;

  localparam C_S_AXI_ADDR_WIDTH = 32;
  localparam C_S_AXI_DATA_WIDTH = 32;
  localparam WB_ADR_SIZE        = 16;
  localparam WB_DAT_SIZE        = 16;
  localparam WB_TIMEOUT_CYCLES  = 4000;
  localparam AXI_TIMEOUT_CYCLES = 15000;
  localparam NUM_ITERATIONS     = 200;

  logic aclk   = 0;
  always #4.0  aclk   = ~aclk;    // 125 MHz

  logic wb_clk = 0;
  always #6.25 wb_clk = ~wb_clk;  // 80 MHz

  logic aresetn = 0;
  logic wb_rst  = 1;

  logic [C_S_AXI_ADDR_WIDTH-1:0]     awaddr;
  logic                               awvalid, awready;
  logic [C_S_AXI_DATA_WIDTH-1:0]     wdata;
  logic [(C_S_AXI_DATA_WIDTH/8)-1:0] wstrb;
  logic                               wvalid, wready;
  logic [1:0]                         bresp;
  logic                               bvalid, bready;
  logic [C_S_AXI_ADDR_WIDTH-1:0]     araddr;
  logic                               arvalid, arready;
  logic [C_S_AXI_DATA_WIDTH-1:0]     rdata;
  logic [1:0]                         rresp;
  logic                               rvalid, rready;

  logic [WB_ADR_SIZE-1:0] wb_adr;
  logic [WB_DAT_SIZE-1:0] wb_rd_dat;
  logic [WB_DAT_SIZE-1:0] wb_wr_dat;
  logic                    wb_stb, wb_we, wb_ack, wb_cyc;

  //--------------------------------------------------------------------------
  // DUT
  //--------------------------------------------------------------------------
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

  //--------------------------------------------------------------------------
  // Wishbone target model
  //
  // scratch_reg: read/write register at addr 0x0001
  // counter_reg: free-running read-only counter at addr 0x0000
  //
  // ACK is combinatorial (same cycle as CYC+STB) -- matches syscon.
  //
  // READ DATA PATH: combinatorial mux off wb_adr. For a read transaction,
  // the bridge holds WB_ADR_O stable from WB_DRIVE through WB_WAIT_ACK and
  // into WB_CAPTURE (one extra cycle after ACK). The mux output is valid
  // throughout -- scratch_reg is already committed from any prior write.
  //
  // KEY POINT on write-then-read timing:
  //   - Write transaction:  ACK fires, scratch_reg commits on that edge
  //   - Read transaction:   arrives as a completely separate AXI transaction,
  //     long after the write has committed (several CDC handshake cycles later)
  //   So scratch_reg is ALWAYS committed before the read samples it.
  //   The 0/200 failure was the bridge sampling WB_RD_DAT_I on the ACK edge
  //   itself (WB_WAIT_ACK) -- at that point the mux input (scratch_reg) has
  //   just been written by the same edge's always_ff, so the non-blocking
  //   assignment hasn't committed yet in simulation. Restoring WB_CAPTURE
  //   (sample one cycle later) fixes this without changing real-hardware
  //   behaviour, where the registered write commits before the next clock.
  //--------------------------------------------------------------------------
  logic [WB_DAT_SIZE-1:0] scratch_reg;
  logic [WB_DAT_SIZE-1:0] counter_reg;

  // Combinatorial ACK -- unconditional, same cycle
  assign wb_ack    = wb_cyc & wb_stb;

  // Combinatorial read data mux
  assign wb_rd_dat = (wb_adr == 16'h0001) ? scratch_reg : counter_reg;

  always_ff @(posedge wb_clk) begin
    if (wb_rst) begin
      scratch_reg <= 16'h0000;
      counter_reg <= 16'h0000;
    end else begin
      counter_reg <= counter_reg + 1;
      if (wb_cyc && wb_stb && wb_we && wb_adr == 16'h0001)
        scratch_reg <= wb_wr_dat;
    end
  end

  // Debug monitor -- unconditional, every wb_clk cycle
  always_ff @(posedge wb_clk) begin
    $display("[%0t WB] cyc=%b stb=%b we=%b adr=%04h wrdat=%04h rddat=%04h ack=%b scratch=%04h",
             $time, wb_cyc, wb_stb, wb_we, wb_adr, wb_wr_dat, wb_rd_dat, wb_ack, scratch_reg);
  end

  //--------------------------------------------------------------------------
  // AXI4-Lite tasks
  //--------------------------------------------------------------------------
  localparam SIM_WATCHDOG_CYCLES = AXI_TIMEOUT_CYCLES + 2000;

  task automatic axi_write(
    input  logic [C_S_AXI_ADDR_WIDTH-1:0] addr,
    input  logic [C_S_AXI_DATA_WIDTH-1:0] data,
    output logic timed_out
  );
    logic aw_done, w_done;
    int   wait_cnt;
    aw_done = 0; w_done = 0; wait_cnt = 0; timed_out = 0;

    @(posedge aclk);
    awaddr <= addr;  awvalid <= 1;
    wdata  <= data;  wstrb   <= '1;  wvalid <= 1;
    bready <= 1;

    while (!aw_done || !w_done) begin
      @(posedge aclk);
      if (++wait_cnt > SIM_WATCHDOG_CYCLES) begin
        timed_out = 1;
        $error("WATCHDOG: AW/W handshake never completed");
        return;
      end
      if (!aw_done && awready) begin awvalid <= 0; aw_done = 1; end
      if (!w_done  && wready)  begin wvalid  <= 0; w_done  = 1; end
    end

    while (!bvalid) begin
      @(posedge aclk);
      if (++wait_cnt > SIM_WATCHDOG_CYCLES) begin
        timed_out = 1;
        $error("WATCHDOG: BVALID never asserted");
        return;
      end
    end
    @(posedge aclk);
    bready <= 0;
  endtask

  task automatic axi_read(
    input  logic [C_S_AXI_ADDR_WIDTH-1:0] addr,
    output logic [C_S_AXI_DATA_WIDTH-1:0] data,
    output logic timed_out
  );
    int wait_cnt;
    wait_cnt = 0; timed_out = 0;

    @(posedge aclk);
    araddr  <= addr;
    arvalid <= 1;
    rready  <= 1;

    while (!arready) begin
      @(posedge aclk);
      if (++wait_cnt > SIM_WATCHDOG_CYCLES) begin
        timed_out = 1;
        $error("WATCHDOG: ARREADY never asserted");
        return;
      end
    end
    @(posedge aclk);
    arvalid <= 0;

    while (!rvalid) begin
      @(posedge aclk);
      if (++wait_cnt > SIM_WATCHDOG_CYCLES) begin
        timed_out = 1;
        $error("WATCHDOG: RVALID never asserted");
        return;
      end
    end
    data = rdata;
    @(posedge aclk);
    rready <= 0;
  endtask

  //--------------------------------------------------------------------------
  // Test
  //--------------------------------------------------------------------------
  logic [C_S_AXI_DATA_WIDTH-1:0] wr_val, rd_val, prev_ctr;
  logic timeout_flag;
  int   wr_rd_pass, wr_rd_fail, ctr_pass, ctr_fail, hangs;

  initial begin
    awvalid = 0; wvalid = 0; bready = 0;
    arvalid = 0; rready = 0;
    awaddr  = '0; wdata = '0; wstrb = '0; araddr = '0;

    #100;
    aresetn <= 1;
    wb_rst  <= 0;
    #200;  // extra settle time after reset

    wr_rd_pass = 0; wr_rd_fail = 0;
    ctr_pass   = 0; ctr_fail   = 0;
    hangs      = 0;
    prev_ctr   = 32'hFFFF_FFFF;

    $display("=== Integrity test: %0d iterations ===", NUM_ITERATIONS);

    for (int i = 0; i < NUM_ITERATIONS; i++) begin

      // --- Check 1: write then read back ---
      wr_val = $urandom_range(1, 32'hFFFF);  // avoid 0 so a zero readback is unambiguous

      axi_write(32'h0000_0001, wr_val, timeout_flag);
      if (timeout_flag) begin
        hangs++;
        $display("[%0d] WRITE HANG", i);
      end else begin
        axi_read(32'h0000_0001, rd_val, timeout_flag);
        if (timeout_flag) begin
          hangs++;
          $display("[%0d] READ HANG", i);
        end else begin
          $display("[%0d] wr=%04h rd=%04h %s",
                   i, wr_val[15:0], rd_val[15:0],
                   (rd_val[15:0] == wr_val[15:0]) ? "OK" : "MISMATCH");
          if (rd_val[15:0] == wr_val[15:0]) wr_rd_pass++;
          else                              wr_rd_fail++;
        end
      end

      // --- Check 2: counter monotonicity ---
      axi_read(32'h0000_0000, rd_val, timeout_flag);
      if (timeout_flag) begin
        hangs++;
        $display("[%0d] COUNTER READ HANG", i);
      end else begin
        if (prev_ctr == 32'hFFFF_FFFF || rd_val[15:0] >= prev_ctr[15:0])
          ctr_pass++;
        else begin
          ctr_fail++;
          $display("[%0d] COUNTER BACKWARDS: prev=%04h now=%04h", i, prev_ctr[15:0], rd_val[15:0]);
        end
        prev_ctr = rd_val;
      end

    end

    $display("\n=== RESULTS ===");
    $display("Write-then-read : %0d/%0d pass, %0d fail", wr_rd_pass, NUM_ITERATIONS, wr_rd_fail);
    $display("Counter mono    : %0d/%0d pass, %0d fail", ctr_pass,   NUM_ITERATIONS, ctr_fail);
    $display("Hangs           : %0d", hangs);
    if (wr_rd_fail == 0 && ctr_fail == 0 && hangs == 0)
      $display("PASS");
    else
      $display("FAIL");
    $display("TEST COMPLETE");
    $finish;
  end

endmodule
