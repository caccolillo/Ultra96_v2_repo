--------------------------------------------------------------------------------
-- axi4lite_to_emif_bridge.vhd
--
-- Connects DIRECTLY to xdma_0's M_AXI_LITE master port -- REPLACES
-- axi_bram_ctrl_0 entirely. No BRAM, no fixed-latency padding, no mailbox
-- protocol. This is a real AXI4-Lite slave: it uses genuine READY/VALID
-- backpressure, so it can stall for however many cycles the underlying
-- EMIF transaction (including a variable-length EMIF_WAIT_I) actually
-- takes -- no overrun risk, unlike the native-BRAM-port approach, because
-- AXI4-Lite is DESIGNED to support arbitrary-length stalls on both the
-- write-response (BVALID) and read-data (RVALID) channels.
--
-- Every PC-side AXI-Lite write/read directly triggers one synthesized
-- EMIF transaction into your existing, UNTOUCHED emif_wishbone_if
-- converter. The AXI transaction simply doesn't complete until the real
-- EMIF cycle does -- which is exactly the correctness property the
-- native-BRAM-port version couldn't give you.
--
-- ============================== ASSUMPTIONS (verify before use) =============
-- 1. EMIF_ADDR_SIZE=16, EMIF_DATA_SIZE=16, EMIF_WAIT_SIZE=2,
--    EMIF_WAIT_POLARITY='1' -- matching your emif_wishbone_if.vhd generics.
-- 2. AXI address maps directly to EMIF address (lower bits); adjust the
--    slice in the EMIF_ADDR_O assignment if your address needs shifting
--    (e.g. if AXI addresses are byte-addressed and EMIF is word-addressed).
-- 3. Full 32-bit AXI data width, truncated to 16-bit EMIF data on write
--    (upper 16 bits ignored) and zero-extended on read (upper 16 bits
--    returned as 0). Adjust if you want different packing behaviour.
-- 4. BA1 (byte-lane select) tied to a fixed value -- full-word EMIF
--    transactions only through this path, matching the same simplification
--    used in the earlier bridge attempts.
-- 5. This assumes AWVALID and WVALID can arrive independently (not
--    necessarily the same cycle) -- handled via separate latch flags
--    below, which is the more general/robust case per the AXI4 spec
--    rather than assuming a simplified simultaneous-arrival master.
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity axi4lite_to_emif_bridge is
  generic (
    C_S_AXI_ADDR_WIDTH : integer := 32;
    C_S_AXI_DATA_WIDTH : integer := 32;
    EMIF_ADDR_SIZE      : positive := 16;
    EMIF_DATA_SIZE      : positive := 16;
    EMIF_WAIT_SIZE       : positive := 2;
    EMIF_WAIT_POLARITY   : natural range 0 to 1 := 1;  -- 1 = active-high wait
                                                          -- (changed from std_logic:
                                                          --  natural generics pass more
                                                          --  reliably across VHDL/SV
                                                          --  mixed-language sim boundaries)
    BA1_TIE              : std_logic := '0'
  );
  port (
    -- ============ AXI4-Lite slave -- connects directly to xdma_0's =========
    -- ============ M_AXI_LITE master port ====================================
    S_AXI_ACLK    : in  std_logic;
    S_AXI_ARESETN : in  std_logic;

    S_AXI_AWADDR  : in  std_logic_vector(C_S_AXI_ADDR_WIDTH-1 downto 0);
    S_AXI_AWVALID : in  std_logic;
    S_AXI_AWREADY : out std_logic;

    S_AXI_WDATA   : in  std_logic_vector(C_S_AXI_DATA_WIDTH-1 downto 0);
    S_AXI_WSTRB   : in  std_logic_vector((C_S_AXI_DATA_WIDTH/8)-1 downto 0);
    S_AXI_WVALID  : in  std_logic;
    S_AXI_WREADY  : out std_logic;

    S_AXI_BRESP   : out std_logic_vector(1 downto 0);
    S_AXI_BVALID  : out std_logic;
    S_AXI_BREADY  : in  std_logic;

    S_AXI_ARADDR  : in  std_logic_vector(C_S_AXI_ADDR_WIDTH-1 downto 0);
    S_AXI_ARVALID : in  std_logic;
    S_AXI_ARREADY : out std_logic;

    S_AXI_RDATA   : out std_logic_vector(C_S_AXI_DATA_WIDTH-1 downto 0);
    S_AXI_RRESP   : out std_logic_vector(1 downto 0);
    S_AXI_RVALID  : out std_logic;
    S_AXI_RREADY  : in  std_logic;

    -- ============ Drives directly into the existing emif_wishbone_if ======
    EMIF_CLK_O   : out std_logic;   -- == S_AXI_ACLK, same clock throughout
    EMIF_ADDR_O  : out std_logic_vector(EMIF_ADDR_SIZE-1 downto 0);
    EMIF_DATA_IO : inout std_logic_vector(EMIF_DATA_SIZE-1 downto 0);
    EMIF_WAIT_I  : in  std_logic_vector(EMIF_WAIT_SIZE-1 downto 0);
    EMIF_CS_N_O  : out std_logic;
    EMIF_WE_N_O  : out std_logic;
    EMIF_OE_N_O  : out std_logic;
    EMIF_BA1_O   : out std_logic
  );
end entity axi4lite_to_emif_bridge;

architecture rtl of axi4lite_to_emif_bridge is

  type state_t is (
    IDLE,
    WRITE_DRIVE, WRITE_WAIT,               -- write-side EMIF cycle
    READ_DRIVE,  READ_WAIT, READ_CAPTURE,  -- read-side EMIF cycle
    RESP_WRITE, RESP_READ                  -- present AXI response, wait for *READY
  );
  signal state : state_t := IDLE;

  signal awaddr_reg : std_logic_vector(C_S_AXI_ADDR_WIDTH-1 downto 0);
  signal wdata_reg  : std_logic_vector(C_S_AXI_DATA_WIDTH-1 downto 0);
  signal araddr_reg : std_logic_vector(C_S_AXI_ADDR_WIDTH-1 downto 0);
  signal rdata_reg  : std_logic_vector(C_S_AXI_DATA_WIDTH-1 downto 0);

  signal aw_latched : std_logic := '0';
  signal w_latched  : std_logic := '0';

  -- Internal copies of the AXI *READY signals -- needed because VHDL
  -- (pre-2008) does not permit reading a port of mode OUT inside a
  -- process. Drive the actual output ports from these via a concurrent
  -- assignment instead of reading the ports directly.
  signal awready_int : std_logic;
  signal wready_int  : std_logic;
  signal arready_int : std_logic;

  signal drive_data_out : std_logic := '0';

  function wait_active(w : std_logic_vector; pol : natural) return boolean is
  begin
    if pol = 1 then
      return unsigned(w) /= 0;
    else
      return unsigned(w) = 0;
    end if;
  end function;

begin

  EMIF_CLK_O <= S_AXI_ACLK;
  EMIF_BA1_O <= BA1_TIE;

  EMIF_DATA_IO <= wdata_reg(EMIF_DATA_SIZE-1 downto 0) when drive_data_out = '1' else (others => 'Z');

  -- AXI channel acceptance: only ready to accept a new AW/W/AR while IDLE.
  -- Computed into internal signals, then the output ports are driven from
  -- these -- avoids reading a mode-OUT port inside the process below.
  awready_int <= '1' when (state = IDLE and aw_latched = '0') else '0';
  wready_int  <= '1' when (state = IDLE and w_latched  = '0') else '0';
  arready_int <= '1' when (state = IDLE) else '0';

  S_AXI_AWREADY <= awready_int;
  S_AXI_WREADY  <= wready_int;
  S_AXI_ARREADY <= arready_int;

  S_AXI_BRESP <= "00";  -- OKAY
  S_AXI_RRESP <= "00";  -- OKAY
  S_AXI_RDATA <= rdata_reg;

  process(S_AXI_ACLK)
  begin
    if rising_edge(S_AXI_ACLK) then
      if S_AXI_ARESETN = '0' then
        state          <= IDLE;
        aw_latched     <= '0';
        w_latched      <= '0';
        drive_data_out <= '0';
        EMIF_CS_N_O    <= '1';
        EMIF_WE_N_O    <= '1';
        EMIF_OE_N_O    <= '1';
        S_AXI_BVALID   <= '0';
        S_AXI_RVALID   <= '0';
      else

        -- latch AW/W independently as they arrive, since AXI4 permits
        -- them in either order or the same cycle. Uses the internal
        -- *_int copies of the ready signals (see declarations above) --
        -- reading the S_AXI_*READY output ports directly here is not
        -- legal in VHDL-93/2002, which is why this was rewritten.
        if S_AXI_AWVALID = '1' and awready_int = '1' then
          awaddr_reg <= S_AXI_AWADDR;
          aw_latched <= '1';
        end if;
        if S_AXI_WVALID = '1' and wready_int = '1' then
          wdata_reg <= S_AXI_WDATA;
          w_latched <= '1';
        end if;

        case state is

          ------------------------------------------------------------------
          when IDLE =>
            if aw_latched = '1' and w_latched = '1' then
              -- write transaction ready to execute
              EMIF_ADDR_O    <= awaddr_reg(EMIF_ADDR_SIZE-1 downto 0);
              EMIF_CS_N_O    <= '0';
              EMIF_WE_N_O    <= '0';
              drive_data_out <= '1';
              state          <= WRITE_DRIVE;
            elsif S_AXI_ARVALID = '1' then
              araddr_reg  <= S_AXI_ARADDR;
              EMIF_ADDR_O <= S_AXI_ARADDR(EMIF_ADDR_SIZE-1 downto 0);
              EMIF_CS_N_O <= '0';
              EMIF_OE_N_O <= '0';
              state       <= READ_DRIVE;
            end if;

          ------------------------------------------------------------------
          -- WRITE side: genuine backpressure -- stays here as long as
          -- EMIF_WAIT_I says so, however many cycles that takes
          ------------------------------------------------------------------
          when WRITE_DRIVE =>
            if wait_active(EMIF_WAIT_I, EMIF_WAIT_POLARITY) then
              state <= WRITE_WAIT;
            else
              EMIF_CS_N_O    <= '1';
              EMIF_WE_N_O    <= '1';
              drive_data_out <= '0';
              aw_latched     <= '0';
              w_latched      <= '0';
              S_AXI_BVALID   <= '1';
              state          <= RESP_WRITE;
            end if;

          when WRITE_WAIT =>
            if not wait_active(EMIF_WAIT_I, EMIF_WAIT_POLARITY) then
              EMIF_CS_N_O    <= '1';
              EMIF_WE_N_O    <= '1';
              drive_data_out <= '0';
              aw_latched     <= '0';
              w_latched      <= '0';
              S_AXI_BVALID   <= '1';
              state          <= RESP_WRITE;
            end if;
            -- else: stay here -- no cycle limit, genuinely arbitrary length

          when RESP_WRITE =>
            if S_AXI_BREADY = '1' then
              S_AXI_BVALID <= '0';
              state        <= IDLE;
            end if;

          ------------------------------------------------------------------
          -- READ side: same genuine backpressure property
          ------------------------------------------------------------------
          when READ_DRIVE =>
            if wait_active(EMIF_WAIT_I, EMIF_WAIT_POLARITY) then
              state <= READ_WAIT;
            else
              state <= READ_CAPTURE;
            end if;

          when READ_WAIT =>
            if not wait_active(EMIF_WAIT_I, EMIF_WAIT_POLARITY) then
              state <= READ_CAPTURE;
            end if;
            -- else: stay here -- no cycle limit, genuinely arbitrary length

          when READ_CAPTURE =>
            rdata_reg    <= (others => '0');
            rdata_reg(EMIF_DATA_SIZE-1 downto 0) <= EMIF_DATA_IO;
            EMIF_CS_N_O  <= '1';
            EMIF_OE_N_O  <= '1';
            S_AXI_RVALID <= '1';
            state        <= RESP_READ;

          when RESP_READ =>
            if S_AXI_RREADY = '1' then
              S_AXI_RVALID <= '0';
              state        <= IDLE;
            end if;

          ------------------------------------------------------------------
          when others =>
            state <= IDLE;

        end case;
      end if;
    end if;
  end process;

end architecture rtl;


`timescale 1ns/1ps
//------------------------------------------------------------------------------
// tb_axi4lite_to_emif_bridge.sv
//
// Instantiates the VHDL axi4lite_to_emif_bridge directly (mixed-language
// simulation -- Vivado XSim handles this fine as long as both files are
// added to the same simulation set).
//
// The EMIF side is NOT your real emif_wishbone_if here -- it's a simple
// zero-wait behavioural memory model standing in for it, just enough to
// validate that a write through the bridge lands at the right address and
// a read gets the right data back. Swap in your real emif_wishbone_if (or
// a Wishbone-side memory behind it) once you want a more faithful test --
// at that point also consider forcing the model to assert a few wait
// cycles deliberately, to prove the bridge's backpressure genuinely holds
// BVALID/RVALID off rather than just working by coincidence at zero wait.
//------------------------------------------------------------------------------

module tb_axi4lite_to_emif_bridge;

  localparam C_S_AXI_ADDR_WIDTH = 32;
  localparam C_S_AXI_DATA_WIDTH = 32;
  localparam EMIF_ADDR_SIZE     = 16;
  localparam EMIF_DATA_SIZE     = 16;
  localparam EMIF_WAIT_SIZE     = 2;

  logic aclk = 0;
  logic aresetn = 0;
  always #5 aclk = ~aclk;  // 100 MHz

  // AXI4-Lite signals
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

  // EMIF-facing signals between DUT and the simple target model
  logic                      emif_clk;
  logic [EMIF_ADDR_SIZE-1:0] emif_addr;
  wire  [EMIF_DATA_SIZE-1:0] emif_data;   // inout, driven by DUT (write) or model (read)
  logic [EMIF_WAIT_SIZE-1:0] emif_wait;
  logic                      emif_cs_n, emif_we_n, emif_oe_n, emif_ba1;

  //----------------------------------------------------------------------------
  // DUT -- VHDL entity instantiated directly from this SV testbench
  //----------------------------------------------------------------------------
  axi4lite_to_emif_bridge #(
    .C_S_AXI_ADDR_WIDTH (C_S_AXI_ADDR_WIDTH),
    .C_S_AXI_DATA_WIDTH (C_S_AXI_DATA_WIDTH),
    .EMIF_ADDR_SIZE     (EMIF_ADDR_SIZE),
    .EMIF_DATA_SIZE     (EMIF_DATA_SIZE),
    .EMIF_WAIT_SIZE     (EMIF_WAIT_SIZE),
    .EMIF_WAIT_POLARITY (1),
    .BA1_TIE            (1'b0)
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
    .EMIF_CLK_O    (emif_clk),
    .EMIF_ADDR_O   (emif_addr),
    .EMIF_DATA_IO  (emif_data),
    .EMIF_WAIT_I   (emif_wait),
    .EMIF_CS_N_O   (emif_cs_n),
    .EMIF_WE_N_O   (emif_we_n),
    .EMIF_OE_N_O   (emif_oe_n),
    .EMIF_BA1_O    (emif_ba1)
  );

  //----------------------------------------------------------------------------
  // Simple EMIF target model -- stand-in for emif_wishbone_if.
  // Zero-wait: always completes in the cycle it's presented. Good enough
  // to validate address decode and data correctness through the bridge.
  //----------------------------------------------------------------------------
  localparam MEM_DEPTH = 1 << EMIF_ADDR_SIZE;
  logic [EMIF_DATA_SIZE-1:0] mem [0:MEM_DEPTH-1];

  logic [EMIF_DATA_SIZE-1:0] emif_data_drive;
  logic                      emif_data_drive_en;

  assign emif_data = emif_data_drive_en ? emif_data_drive : {EMIF_DATA_SIZE{1'bz}};
  assign emif_wait = '0;  // zero wait -- always ready

  always_ff @(posedge aclk) begin
    emif_data_drive_en <= 1'b0;
    if (!emif_cs_n) begin
      if (!emif_we_n) begin
        mem[emif_addr] <= emif_data;
        $display("[%0t] MODEL: WRITE captured -- addr=0x%04h data=0x%04h", $time, emif_addr, emif_data);
      end else if (!emif_oe_n) begin
        emif_data_drive    <= mem[emif_addr];
        emif_data_drive_en <= 1'b1;
        $display("[%0t] MODEL: READ drive scheduled -- addr=0x%04h data=0x%04h", $time, emif_addr, mem[emif_addr]);
      end
    end
  end

  //----------------------------------------------------------------------------
  // Task 1: AXI4-Lite write
  //----------------------------------------------------------------------------
  task automatic axi_write(input logic [C_S_AXI_ADDR_WIDTH-1:0] addr,
                            input logic [C_S_AXI_DATA_WIDTH-1:0] data);
    logic aw_done, w_done;
    aw_done = 1'b0;
    w_done  = 1'b0;

    @(posedge aclk);
    awaddr  <= addr;  awvalid <= 1'b1;
    wdata   <= data;  wstrb   <= '1;  wvalid <= 1'b1;
    bready  <= 1'b1;

    // Sample AWREADY/WREADY only at clock edges -- never in between.
    // The previous fork/join + wait(signal) version could race, since
    // wait() re-evaluates continuously rather than sampling synchronously,
    // and can miss a ready pulse that's only valid for exactly one cycle.
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
  logic [C_S_AXI_ADDR_WIDTH-1:0] test_addr;
  logic [C_S_AXI_DATA_WIDTH-1:0] test_wdata;
  logic [C_S_AXI_DATA_WIDTH-1:0] test_rdata;

  initial begin
    awvalid = 0; wvalid = 0; bready = 0; arvalid = 0; rready = 0;
    awaddr  = '0; wdata = '0; wstrb = '0; araddr = '0;

    repeat (5) @(posedge aclk);
    aresetn <= 1'b1;
    repeat (5) @(posedge aclk);

    // random address within the EMIF address space; random 16-bit data
    // (upper 16 bits of the 32-bit AXI word read back as 0, per the DUT's
    // zero-extend-on-read behaviour)
    test_addr  = $urandom_range(0, MEM_DEPTH-1);
    test_wdata = {16'h0000, $urandom_range(0, 32'hFFFF)};

    $display("[%0t] Writing 0x%08h to address 0x%08h", $time, test_wdata, test_addr);
    axi_write(test_addr, test_wdata);

    axi_read(test_addr, test_rdata);
    $display("[%0t] Read back 0x%08h from address 0x%08h", $time, test_rdata, test_addr);

    if (test_rdata === test_wdata)
      $display("PASS: read-back matches write");
    else
      $error("FAIL: wrote 0x%08h, read 0x%08h", test_wdata, test_rdata);

    $display("TEST COMPLETE");
    $finish;
  end

endmodule
