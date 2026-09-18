// =============================================================================
// tb_pcie_wb.sv
// Testbench for S0841D_sim.vhd
// Full SYSCON register dump + functional checks
// =============================================================================

`timescale 1ns/1ps

module tb_pcie_wb;

    // -------------------------------------------------------------------------
    // Parameters
    // -------------------------------------------------------------------------
    parameter AXI_CLK_PERIOD = 8;     // 125 MHz
    parameter TIMEOUT_CYCLES = 2000;

    // SYSCON base address
    parameter [31:0] SYSCON_BASE = 32'h0000_0800;

    // Register offsets (word offset * 2 = byte offset)
    parameter [31:0] ADDR_ID_REV             = SYSCON_BASE + 32'h00;
    parameter [31:0] ADDR_DSP_ALIVE          = SYSCON_BASE + 32'h02;
    parameter [31:0] ADDR_ARM_ALIVE          = SYSCON_BASE + 32'h04;
    parameter [31:0] ADDR_FPGA_ALIVE_DSP     = SYSCON_BASE + 32'h06;
    parameter [31:0] ADDR_FPGA_ALIVE_ARM     = SYSCON_BASE + 32'h08;
    parameter [31:0] ADDR_DISABLE_FP_CLK     = SYSCON_BASE + 32'h0A;
    parameter [31:0] ADDR_DISABLE_CODEC_CLK  = SYSCON_BASE + 32'h0C;
    parameter [31:0] ADDR_INIT_COMPLETE      = SYSCON_BASE + 32'h0E;
    parameter [31:0] ADDR_PLL_LOCK           = SYSCON_BASE + 32'h10;
    parameter [31:0] ADDR_ACTIVE_OUTPUT      = SYSCON_BASE + 32'h12;
    parameter [31:0] ADDR_SCM_VERSION_HIGH   = SYSCON_BASE + 32'h14;
    parameter [31:0] ADDR_SCM_VERSION_LOW    = SYSCON_BASE + 32'h16;
    parameter [31:0] ADDR_BUILD_DATE_DDMM    = SYSCON_BASE + 32'h18;
    parameter [31:0] ADDR_BUILD_DATE_YYYY    = SYSCON_BASE + 32'h1A;
    parameter [31:0] ADDR_FULL_SAMPLE_RATE   = SYSCON_BASE + 32'h1C;
    parameter [31:0] ADDR_SW_RST             = SYSCON_BASE + 32'h1E;

    // Expected values from versions.vhd and syscon.vhd
    parameter [15:0] EXP_ID_REV            = 16'h0003;
    parameter [15:0] EXP_SCM_VERSION_HIGH  = 16'hDEF8;
    parameter [15:0] EXP_SCM_VERSION_LOW   = 16'h7C4A;
    parameter [15:0] EXP_BUILD_DATE_DDMM   = 16'h3007;
    parameter [15:0] EXP_BUILD_DATE_YYYY   = 16'h2026;

    // -------------------------------------------------------------------------
    // AXI4-Lite signals
    // -------------------------------------------------------------------------
    logic        aclk    = 0;
    logic        aresetn = 0;
    logic [31:0] awaddr  = 0;
    logic        awvalid = 0;
    logic        awready;
    logic [31:0] wdata   = 0;
    logic [3:0]  wstrb   = 4'hF;
    logic        wvalid  = 0;
    logic        wready;
    logic [1:0]  bresp;
    logic        bvalid;
    logic        bready  = 1;
    logic [31:0] araddr  = 0;
    logic        arvalid = 0;
    logic        arready;
    logic [31:0] rdata;
    logic [1:0]  rresp;
    logic        rvalid;
    logic        rready  = 1;

    // -------------------------------------------------------------------------
    // DUT
    // -------------------------------------------------------------------------
    S0841D_sim dut (
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
        .S_AXI_RREADY  (rready)
    );

    // -------------------------------------------------------------------------
    // Clock
    // -------------------------------------------------------------------------
    always #(AXI_CLK_PERIOD/2) aclk = ~aclk;

    // -------------------------------------------------------------------------
    // Counters
    // -------------------------------------------------------------------------
    int pass_count = 0;
    int fail_count = 0;

    // -------------------------------------------------------------------------
    // Tasks
    // -------------------------------------------------------------------------
    task automatic axi_read(
        input  logic [31:0] addr,
        output logic [31:0] data,
        output logic [1:0]  resp
    );
        int cycles;
        @(posedge aclk);
        araddr  <= addr;
        arvalid <= 1;
        cycles = 0;
        while (!arready) begin
            @(posedge aclk);
            if (++cycles > TIMEOUT_CYCLES) begin
                $display("ERROR: ARREADY timeout at addr 0x%08X", addr);
                fail_count++;
                arvalid <= 0;
                return;
            end
        end
        @(posedge aclk);
        arvalid <= 0;
        cycles = 0;
        while (!rvalid) begin
            @(posedge aclk);
            if (++cycles > TIMEOUT_CYCLES) begin
                $display("ERROR: RVALID timeout at addr 0x%08X", addr);
                fail_count++;
                return;
            end
        end
        data = rdata;
        resp = rresp;
        rready <= 1;
        @(posedge aclk);
    endtask

    task automatic axi_write(
        input logic [31:0] addr,
        input logic [31:0] data
    );
        int cycles;
        @(posedge aclk);
        awaddr  <= addr;
        awvalid <= 1;
        wdata   <= data;
        wvalid  <= 1;
        wstrb   <= 4'hF;
        cycles = 0;
        while (!(awready && wready)) begin
            @(posedge aclk);
            if (++cycles > TIMEOUT_CYCLES) begin
                $display("ERROR: AW/WREADY timeout at addr 0x%08X", addr);
                fail_count++;
                awvalid <= 0; wvalid <= 0;
                return;
            end
        end
        @(posedge aclk);
        awvalid <= 0;
        wvalid  <= 0;
        cycles = 0;
        while (!bvalid) begin
            @(posedge aclk);
            if (++cycles > TIMEOUT_CYCLES) begin
                $display("ERROR: BVALID timeout at addr 0x%08X", addr);
                fail_count++;
                return;
            end
        end
        @(posedge aclk);
    endtask

    // Check exact value
    task automatic check_exact(
        input string       name,
        input logic [31:0] got,
        input logic [15:0] expected
    );
        if (got[15:0] === expected) begin
            $display("PASS  %-24s = 0x%04X", name, got[15:0]);
            pass_count++;
        end else begin
            $display("FAIL  %-24s = 0x%04X  (expected 0x%04X)", name, got[15:0], expected);
            fail_count++;
        end
    endtask

    // Check not timeout (register responds)
    task automatic check_ack(
        input string       name,
        input logic [31:0] got
    );
        if (got[15:0] === 16'hFFFF) begin
            $display("FAIL  %-24s = 0xFFFF  (no ACK — timeout)", name);
            fail_count++;
        end else begin
            $display("INFO  %-24s = 0x%04X  (%0d)", name, got[15:0], got[15:0]);
            pass_count++;
        end
    endtask

    // -------------------------------------------------------------------------
    // Main
    // -------------------------------------------------------------------------
    logic [31:0] rd;
    logic [1:0]  rsp;
    logic [31:0] alive_t0, alive_t1;

    initial begin
        $display("=== tb_pcie_wb: SYSCON full dump ===");

        // Reset
        aresetn = 0;
        repeat(20) @(posedge aclk);
        aresetn = 1;
        #2000;

        // ------------------------------------------------------------------
        // SYSCON register dump
        // ------------------------------------------------------------------
        $display("\n--- SYSCON register dump ---");
        $display("%-26s  %6s  %6s", "Register", "Offset", "Value");
        $display("%s", {60{"-"}});

        axi_read(ADDR_ID_REV,            rd, rsp); check_exact("ID_REV",            rd, EXP_ID_REV);
        axi_read(ADDR_DSP_ALIVE,         rd, rsp); check_ack  ("DSP_ALIVE",         rd);
        axi_read(ADDR_ARM_ALIVE,         rd, rsp); check_ack  ("ARM_ALIVE",         rd);
        axi_read(ADDR_FPGA_ALIVE_DSP,    rd, rsp); check_ack  ("FPGA_ALIVE_DSP",    rd);
        axi_read(ADDR_FPGA_ALIVE_ARM,    rd, rsp); check_ack  ("FPGA_ALIVE_ARM",    rd);
        axi_read(ADDR_DISABLE_FP_CLK,    rd, rsp); check_ack  ("DISABLE_FP_CLK",    rd);
        axi_read(ADDR_DISABLE_CODEC_CLK, rd, rsp); check_ack  ("DISABLE_CODEC_CLK", rd);
        axi_read(ADDR_INIT_COMPLETE,     rd, rsp); check_ack  ("INIT_COMPLETE",      rd);
        axi_read(ADDR_PLL_LOCK,          rd, rsp); check_ack  ("PLL_LOCK",           rd);
        axi_read(ADDR_ACTIVE_OUTPUT,     rd, rsp); check_ack  ("ACTIVE_OUTPUT",      rd);
        axi_read(ADDR_SCM_VERSION_HIGH,  rd, rsp); check_exact("SCM_VERSION_HIGH",  rd, EXP_SCM_VERSION_HIGH);
        axi_read(ADDR_SCM_VERSION_LOW,   rd, rsp); check_exact("SCM_VERSION_LOW",   rd, EXP_SCM_VERSION_LOW);
        axi_read(ADDR_BUILD_DATE_DDMM,   rd, rsp); check_exact("BUILD_DATE_DDMM",   rd, EXP_BUILD_DATE_DDMM);
        axi_read(ADDR_BUILD_DATE_YYYY,   rd, rsp); check_exact("BUILD_DATE_YYYY",   rd, EXP_BUILD_DATE_YYYY);
        axi_read(ADDR_FULL_SAMPLE_RATE,  rd, rsp); check_ack  ("FULL_SAMPLE_RATE",  rd);
        axi_read(ADDR_SW_RST,            rd, rsp); check_ack  ("SW_RST",            rd);

        // ------------------------------------------------------------------
        // FPGA alive counter increments
        // ------------------------------------------------------------------
        $display("\n--- FPGA_ALIVE_ARM increments over 5ms ---");
        axi_read(ADDR_FPGA_ALIVE_ARM, alive_t0, rsp);
        #5_000_000;
        axi_read(ADDR_FPGA_ALIVE_ARM, alive_t1, rsp);
        $display("  t=0ms : 0x%04X (%0d)", alive_t0[15:0], alive_t0[15:0]);
        $display("  t=5ms : 0x%04X (%0d)", alive_t1[15:0], alive_t1[15:0]);
        if (alive_t1 > alive_t0) begin
            $display("PASS  FPGA_ALIVE_ARM incremented (%0d -> %0d)", alive_t0[15:0], alive_t1[15:0]);
            pass_count++;
        end else begin
            $display("FAIL  FPGA_ALIVE_ARM did not increment");
            fail_count++;
        end

        // ------------------------------------------------------------------
        // ARM_ALIVE write/readback
        // ------------------------------------------------------------------
        $display("\n--- ARM_ALIVE write/readback ---");
        axi_write(ADDR_ARM_ALIVE, 32'h0000_A55A);
        axi_read (ADDR_ARM_ALIVE, rd, rsp);
        check_exact("ARM_ALIVE readback", rd, 16'hA55A);

        // ------------------------------------------------------------------
        // DSP_ALIVE write/readback
        // ------------------------------------------------------------------
        $display("\n--- DSP_ALIVE write/readback ---");
        axi_write(ADDR_DSP_ALIVE, 32'h0000_1234);
        axi_read (ADDR_DSP_ALIVE, rd, rsp);
        check_exact("DSP_ALIVE readback", rd, 16'h1234);

        // ------------------------------------------------------------------
        // Summary
        // ------------------------------------------------------------------
        $display("\n=== SUMMARY: %0d PASS, %0d FAIL ===", pass_count, fail_count);
        if (fail_count == 0)
            $display("ALL TESTS PASSED");
        else
            $display("FAILURES DETECTED");

        $finish;
    end

    // Watchdog
    initial begin
        #50_000_000;
        $display("WATCHDOG: simulation exceeded 50ms");
        $finish;
    end

endmodule
