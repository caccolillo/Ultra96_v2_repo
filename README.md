`timescale 1ns/1ps
//------------------------------------------------------------------------------
// tb_axi4lite_to_wishbone_bridge.sv
//
// Instantiates the VHDL axi4lite_to_wishbone_bridge directly (mixed-language
// sim, same as the EMIF-bridge testbench earlier). TWO independent, free-
// running clocks are used -- 100 MHz on the AXI side, 80 MHz on the
// Wishbone side -- genuinely asynchronous to each other, which is the
// whole point of exercising the xpm_cdc_handshake macros for real rather
// than accidentally testing a same-clock case.
//
// The Wishbone-side target is a simple RAM model with a DELIBERATE
// multi-cycle delay before asserting ACK (WB_MODEL_WAIT_CYCLES below) --
// this is important: a same-cycle-ACK model would pass even if the
// bridge's WB_WAIT_ACK backpressure logic were broken, since there'd be
// nothing to wait for. Forcing a real multi-cycle wait actually exercises
// the thing this design was built to get right.
//------------------------------------------------------------------------------

module tb_axi4lite_to_wishbone_bridge;

  localparam C_S_AXI_ADDR_WIDTH = 32;
  localparam C_S_AXI_DATA_WIDTH = 32;
  localparam WB_ADR_SIZE        = 16;
  localparam WB_DAT_SIZE        = 16;
  localparam WB_MODEL_WAIT_CYCLES = 3;  // deliberate non-zero ACK latency

  // ---------------- Two independent, free-running clocks -------------------
  logic aclk = 0;
  always #5.0  aclk   = ~aclk;    // 100 MHz (10 ns period)

  logic wb_clk = 0;
  always #6.25 wb_clk = ~wb_clk;  // 80 MHz (12.5 ns period)

  logic aresetn = 0;  // AXI side: active-low
  logic wb_rst  = 1;  // WB side: active-high (INVERT_RESET default '0' in the DUT)

  // ---------------- AXI4-Lite signals ---------------------------------------
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

  // ---------------- Wishbone signals between DUT and RAM model -------------
  logic [WB_ADR_SIZE-1:0] wb_adr;
  logic [WB_DAT_SIZE-1:0] wb_rd_dat;   // DUT input  <- model drives
  logic [WB_DAT_SIZE-1:0] wb_wr_dat;   // DUT output -> model reads
  logic                    wb_stb, wb_we, wb_ack, wb_cyc;

  //----------------------------------------------------------------------------
  // DUT -- VHDL entity instantiated directly from this SV testbench
  //----------------------------------------------------------------------------
  axi4lite_to_wishbone_bridge #(
    .C_S_AXI_ADDR_WIDTH (C_S_AXI_ADDR_WIDTH),
    .C_S_AXI_DATA_WIDTH (C_S_AXI_DATA_WIDTH),
    .WB_ADR_SIZE        (WB_ADR_SIZE),
    .WB_DAT_SIZE        (WB_DAT_SIZE),
    .INVERT_RESET       (1'b0)
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
  // Wishbone RAM model -- stand-in for whatever real target sits on the
  // Wishbone bus (registers, CIC/FFT logic, etc). Deliberately waits
  // WB_MODEL_WAIT_CYCLES before ACKing, to genuinely exercise the bridge's
  // variable-latency backpressure rather than a trivial zero-wait case.
  //----------------------------------------------------------------------------
  typedef enum logic [1:0] {WBM_IDLE, WBM_WAIT, WBM_ACK} wbm_state_t;
  wbm_state_t wbm_state = WBM_IDLE;
  int         wbm_cnt;

  logic [WB_DAT_SIZE-1:0] wbmem [0:(1<<WB_ADR_SIZE)-1];
  logic [WB_DAT_SIZE-1:0] wb_rdata_reg;

  assign wb_rd_dat = wb_rdata_reg;

  always_ff @(posedge wb_clk) begin
    if (wb_rst) begin
      wbm_state <= WBM_IDLE;
      wb_ack    <= 1'b0;
    end else begin
      case (wbm_state)
        WBM_IDLE: begin
          wb_ack <= 1'b0;
          if (wb_cyc && wb_stb) begin
            wbm_cnt   <= WB_MODEL_WAIT_CYCLES;
            wbm_state <= WBM_WAIT;
          end
        end

        WBM_WAIT: begin
          if (wbm_cnt == 0) begin
            wb_ack <= 1'b1;
            if (wb_we) begin
              wbmem[wb_adr] <= wb_wr_dat;
              $display("[%0t] WB_MODEL: WRITE captured -- addr=0x%04h data=0x%04h", $time, wb_adr, wb_wr_dat);
            end else begin
              wb_rdata_reg <= wbmem[wb_adr];
              $display("[%0t] WB_MODEL: READ drive -- addr=0x%04h data=0x%04h", $time, wb_adr, wbmem[wb_adr]);
            end
            wbm_state <= WBM_ACK;
          end else begin
            wbm_cnt <= wbm_cnt - 1;
          end
        end

        WBM_ACK: begin
          wb_ack    <= 1'b0;
          wbm_state <= WBM_IDLE;
        end

        default: wbm_state <= WBM_IDLE;
      endcase
    end
  end

  //----------------------------------------------------------------------------
  // Task 1: AXI4-Lite write (clock-synchronized polling, no fork/join races)
  //----------------------------------------------------------------------------
  task automatic axi_write(input logic [C_S_AXI_ADDR_WIDTH-1:0] addr,
                            input logic [C_S_AXI_DATA_WIDTH-1:0] data);
    logic aw_done, w_done;
    aw_done = 1'b0;
    w_done  = 1'b0;

    @(posedge aclk);
    awaddr <= addr;  awvalid <= 1'b1;
    wdata  <= data;  wstrb   <= '1;  wvalid <= 1'b1;
    bready <= 1'b1;

    while (!aw_done || !w_done) begin
      @(posedge aclk);
      if (!aw_done && awready) begin
        awvalid <= 1'b0;
        aw_done = 1'b1;
      end
      if (!w_done && wready) begin
        wvalid <= 1'b0;
        w_done = 1'b1;
      end
    end

    while (!bvalid) @(posedge aclk);
    if (bresp !== 2'b00)
      $error("WRITE to 0x%08h: non-OKAY BRESP = %0d", addr, bresp);
    @(posedge aclk);
    bready <= 1'b0;
  endtask

  //----------------------------------------------------------------------------
  // Task 2: AXI4-Lite read
  //----------------------------------------------------------------------------
  task automatic axi_read(input  logic [C_S_AXI_ADDR_WIDTH-1:0] addr,
                           output logic [C_S_AXI_DATA_WIDTH-1:0] data);
    @(posedge aclk);
    araddr <= addr; arvalid <= 1'b1;
    rready <= 1'b1;

    while (!arready) @(posedge aclk);
    @(posedge aclk);
    arvalid <= 1'b0;

    while (!rvalid) @(posedge aclk);
    data = rdata;
    if (rresp !== 2'b00)
      $error("READ from 0x%08h: non-OKAY RRESP = %0d", addr, rresp);
    @(posedge aclk);
    rready <= 1'b0;
  endtask

  //----------------------------------------------------------------------------
  // Simple test: random address, random data, write then read back, check
  //----------------------------------------------------------------------------
  localparam MEM_DEPTH = 1 << WB_ADR_SIZE;

  logic [C_S_AXI_ADDR_WIDTH-1:0] test_addr;
  logic [C_S_AXI_DATA_WIDTH-1:0] test_wdata;
  logic [C_S_AXI_DATA_WIDTH-1:0] test_rdata;

  initial begin
    awvalid = 0; wvalid = 0; bready = 0; arvalid = 0; rready = 0;
    awaddr  = '0; wdata = '0; wstrb = '0; araddr = '0;

    repeat (5) @(posedge aclk);
    aresetn <= 1'b1;
    repeat (5) @(posedge wb_clk);
    wb_rst  <= 1'b0;
    repeat (5) @(posedge aclk);

    test_addr  = $urandom_range(0, MEM_DEPTH-1);
    test_wdata = {16'h0000, $urandom_range(0, 32'hFFFF)};

    $display("[%0t] Writing 0x%08h to address 0x%08h", $time, test_wdata, test_addr);
    axi_write(test_addr, test_wdata);

    axi_read(test_addr, test_rdata);
    $display("[%0t] Read back 0x%08h from address 0x%08h", $time, test_rdata, test_addr);

    if (test_rdata === test_wdata)
      $display("PASS: read-back matches write, correctly crossed 100MHz <-> 80MHz");
    else
      $error("FAIL: wrote 0x%08h, read 0x%08h", test_wdata, test_rdata);

    $display("TEST COMPLETE");
    $finish;
  end

endmodule
