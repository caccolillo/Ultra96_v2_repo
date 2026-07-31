--------------------------------------------------------------------------------
-- axi4lite_to_wishbone_bridge.vhd
--
-- Direct AXI4-Lite -> Wishbone bridge. Replaces the EMIF-emulation approach
-- entirely -- no synthesized EMIF bus cycles, no legacy emif_wishbone_if
-- dependency. This talks Wishbone natively, on its own clock, matching the
-- WB_* port/generic naming convention already used by emif_wishbone_if.vhd
-- for consistency across the codebase.
--
-- CLOCK DOMAIN CROSSING: the AXI-Lite side (S_AXI_ACLK, 100 MHz from
-- xdma_0) and the Wishbone side (CLK_I, 80 MHz -- matching the rest of
-- your signal-processing logic) are genuinely unrelated clocks. This
-- bridge uses TWO xpm_cdc_handshake macros (Xilinx's built-in, formally
-- verified CDC library primitives -- no IP catalog customization needed,
-- just instantiate directly from the xpm library) to safely cross the
-- request (address+data+r/w) one way and the response (read data) back.
-- Each xpm_cdc_handshake provides a full request/acknowledge handshake
-- with proper double-flop synchronization on the control signals AND
-- guarantees the multi-bit data bus is stable and consistent when sampled
-- on the destination side -- the thing naive per-bit synchronizers can't
-- guarantee for a bus.
--
-- Single-outstanding by construction: the AXI-side FSM will not start a
-- new transaction until the previous one's response has come back through
-- the second handshake -- matches your actual traffic (register pokes,
-- occasional FFT snapshot reads), and avoids needing any outstanding-
-- transaction tracking.
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
library xpm;
use xpm.vcomponents.all;

entity axi4lite_to_wishbone_bridge is
  generic (
    C_S_AXI_ADDR_WIDTH : integer := 32;
    C_S_AXI_DATA_WIDTH : integer := 32;
    WB_ADR_SIZE         : positive := 16;
    WB_DAT_SIZE         : positive := 16;
    INVERT_RESET        : std_logic := '0'  -- matches emif_wishbone_if's convention
  );
  port (
    -- ============ AXI4-Lite slave -- connects directly to xdma_0's =========
    -- ============ M_AXI_LITE master port, 100 MHz domain ====================
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

    -- ============ Wishbone master -- same naming convention as ============
    -- ============ emif_wishbone_if.vhd, 80 MHz domain ========================
    CLK_I       : in  std_logic;   -- Wishbone-side clock, 80 MHz
    RST_I       : in  std_logic;

    WB_ADR_O    : out std_logic_vector(WB_ADR_SIZE-1 downto 0);
    WB_RD_DAT_I : in  std_logic_vector(WB_DAT_SIZE-1 downto 0);
    WB_WR_DAT_O : out std_logic_vector(WB_DAT_SIZE-1 downto 0);
    WB_STB_O    : out std_logic;
    WB_WR_O     : out std_logic;
    WB_ACK_I    : in  std_logic;
    WB_CYC_O    : out std_logic
  );
end entity axi4lite_to_wishbone_bridge;

architecture rtl of axi4lite_to_wishbone_bridge is

  -- Request bundle crossing AXI (100MHz) -> WB (80MHz):
  -- [ RNW(1) | ADDR(WB_ADR_SIZE) | WDATA(WB_DAT_SIZE) ]
  constant REQ_WIDTH : integer := 1 + WB_ADR_SIZE + WB_DAT_SIZE;

  -- Response bundle crossing WB (80MHz) -> AXI (100MHz): just the read data
  -- (unused/don't-care for writes, but always sent to keep one uniform
  -- completion handshake for both read and write)
  constant RESP_WIDTH : integer := WB_DAT_SIZE;

  signal req_bundle   : std_logic_vector(REQ_WIDTH-1 downto 0);
  signal req_send     : std_logic := '0';
  signal req_rcv      : std_logic;
  signal req_dest_out : std_logic_vector(REQ_WIDTH-1 downto 0);
  signal req_dest_req : std_logic;
  signal req_dest_ack : std_logic := '0';

  signal resp_bundle   : std_logic_vector(RESP_WIDTH-1 downto 0);
  signal resp_send     : std_logic := '0';
  signal resp_rcv      : std_logic;
  signal resp_dest_out : std_logic_vector(RESP_WIDTH-1 downto 0);
  signal resp_dest_req : std_logic;
  signal resp_dest_ack : std_logic := '0';

  ------------------------------------------------------------------------
  -- AXI-side FSM (S_AXI_ACLK domain)
  ------------------------------------------------------------------------
  type axi_state_t is (AXI_IDLE, AXI_SEND_REQ, AXI_WAIT_RESP, AXI_RESP_WRITE, AXI_RESP_READ);
  signal axi_state : axi_state_t := AXI_IDLE;

  signal awaddr_reg, araddr_reg : std_logic_vector(C_S_AXI_ADDR_WIDTH-1 downto 0);
  signal wdata_reg               : std_logic_vector(C_S_AXI_DATA_WIDTH-1 downto 0);
  signal rdata_reg               : std_logic_vector(C_S_AXI_DATA_WIDTH-1 downto 0);
  signal aw_latched, w_latched   : std_logic := '0';
  signal is_read_reg             : std_logic;

  signal awready_int, wready_int, arready_int : std_logic;

  ------------------------------------------------------------------------
  -- WB-side FSM (CLK_I domain)
  ------------------------------------------------------------------------
  type wb_state_t is (WB_IDLE, WB_DRIVE, WB_WAIT_ACK, WB_CAPTURE, WB_SEND_RESP, WB_WAIT_RCV);
  signal wb_state : wb_state_t := WB_IDLE;

  signal wb_rnw     : std_logic;
  signal wb_addr    : std_logic_vector(WB_ADR_SIZE-1 downto 0);
  signal wb_wdata   : std_logic_vector(WB_DAT_SIZE-1 downto 0);
  signal wb_rdata   : std_logic_vector(WB_DAT_SIZE-1 downto 0);

begin

  ----------------------------------------------------------------------------
  -- AXI-side ready signals (concurrent, internal signals per the earlier
  -- fix -- avoids reading OUT-mode ports inside a process)
  ----------------------------------------------------------------------------
  awready_int <= '1' when (axi_state = AXI_IDLE and aw_latched = '0') else '0';
  wready_int  <= '1' when (axi_state = AXI_IDLE and w_latched  = '0') else '0';
  arready_int <= '1' when (axi_state = AXI_IDLE) else '0';

  S_AXI_AWREADY <= awready_int;
  S_AXI_WREADY  <= wready_int;
  S_AXI_ARREADY <= arready_int;

  S_AXI_BRESP <= "00";
  S_AXI_RRESP <= "00";
  S_AXI_RDATA <= rdata_reg;

  ------------------------------------------------------------------------
  -- AXI-side FSM
  ------------------------------------------------------------------------
  process(S_AXI_ACLK)
  begin
    if rising_edge(S_AXI_ACLK) then
      if S_AXI_ARESETN = '0' then
        axi_state    <= AXI_IDLE;
        aw_latched   <= '0';
        w_latched    <= '0';
        req_send     <= '0';
        resp_dest_ack <= '0';
        S_AXI_BVALID <= '0';
        S_AXI_RVALID <= '0';
      else
        resp_dest_ack <= '0';  -- default: single-cycle pulse (ack, not send -- fine as pulse)

        if S_AXI_AWVALID = '1' and awready_int = '1' then
          awaddr_reg <= S_AXI_AWADDR;
          aw_latched <= '1';
        end if;
        if S_AXI_WVALID = '1' and wready_int = '1' then
          wdata_reg <= S_AXI_WDATA;
          w_latched <= '1';
        end if;

        case axi_state is

          when AXI_IDLE =>
            if aw_latched = '1' and w_latched = '1' then
              is_read_reg <= '0';
              req_bundle  <= '0' & awaddr_reg(WB_ADR_SIZE-1 downto 0) & wdata_reg(WB_DAT_SIZE-1 downto 0);
              req_send    <= '1';   -- held high, per xpm_cdc_handshake protocol,
                                     -- until req_rcv confirms (cleared in AXI_SEND_REQ below)
              axi_state   <= AXI_SEND_REQ;
            elsif S_AXI_ARVALID = '1' then
              araddr_reg  <= S_AXI_ARADDR;
              is_read_reg <= '1';
              req_bundle  <= '1' & S_AXI_ARADDR(WB_ADR_SIZE-1 downto 0) & (WB_DAT_SIZE-1 downto 0 => '0');
              req_send    <= '1';   -- held high until req_rcv confirms
              axi_state   <= AXI_SEND_REQ;
            end if;

          when AXI_SEND_REQ =>
            -- req_send stays asserted (not reset each cycle) until the
            -- macro confirms receipt via req_rcv -- per xpm_cdc_handshake's
            -- documented protocol: src_send must only deassert once
            -- src_rcv is asserted, not before.
            if req_rcv = '1' then
              req_send  <= '0';
              axi_state <= AXI_WAIT_RESP;
            end if;

          when AXI_WAIT_RESP =>
            -- genuine backpressure: waits however long the real Wishbone
            -- cycle takes on the other side, no fixed budget
            if resp_dest_req = '1' then
              rdata_reg     <= (others => '0');
              rdata_reg(WB_DAT_SIZE-1 downto 0) <= resp_dest_out;
              resp_dest_ack <= '1';
              aw_latched    <= '0';
              w_latched     <= '0';
              if is_read_reg = '1' then
                axi_state <= AXI_RESP_READ;
              else
                axi_state <= AXI_RESP_WRITE;
              end if;
            end if;

          when AXI_RESP_WRITE =>
            S_AXI_BVALID <= '1';
            if S_AXI_BREADY = '1' and S_AXI_BVALID = '1' then
              S_AXI_BVALID <= '0';
              axi_state    <= AXI_IDLE;
            end if;

          when AXI_RESP_READ =>
            S_AXI_RVALID <= '1';
            if S_AXI_RREADY = '1' and S_AXI_RVALID = '1' then
              S_AXI_RVALID <= '0';
              axi_state    <= AXI_IDLE;
            end if;

          when others =>
            axi_state <= AXI_IDLE;

        end case;
      end if;
    end if;
  end process;

  ------------------------------------------------------------------------
  -- Request handshake: AXI (100MHz) -> WB (80MHz)
  ------------------------------------------------------------------------
  xpm_cdc_handshake_req : xpm_cdc_handshake
    generic map (
      DEST_EXT_HSK   => 1,   -- WB-side FSM controls exactly when to ack
      DEST_SYNC_FF   => 4,
      INIT_SYNC_FF   => 0,
      SIM_ASSERT_CHK => 0,
      SRC_SYNC_FF    => 4,
      WIDTH          => REQ_WIDTH
    )
    port map (
      src_clk  => S_AXI_ACLK,
      src_in   => req_bundle,
      src_send => req_send,
      src_rcv  => req_rcv,
      dest_clk => CLK_I,
      dest_out => req_dest_out,
      dest_req => req_dest_req,
      dest_ack => req_dest_ack
    );

  ------------------------------------------------------------------------
  -- Response handshake: WB (80MHz) -> AXI (100MHz)
  ------------------------------------------------------------------------
  xpm_cdc_handshake_resp : xpm_cdc_handshake
    generic map (
      DEST_EXT_HSK   => 1,   -- AXI-side FSM controls exactly when to ack
      DEST_SYNC_FF   => 4,
      INIT_SYNC_FF   => 0,
      SIM_ASSERT_CHK => 0,
      SRC_SYNC_FF    => 4,
      WIDTH          => RESP_WIDTH
    )
    port map (
      src_clk  => CLK_I,
      src_in   => resp_bundle,
      src_send => resp_send,
      src_rcv  => resp_rcv,
      dest_clk => S_AXI_ACLK,
      dest_out => resp_dest_out,
      dest_req => resp_dest_req,
      dest_ack => resp_dest_ack
    );

  ------------------------------------------------------------------------
  -- WB-side FSM (CLK_I, 80 MHz domain) -- a real Wishbone master
  ------------------------------------------------------------------------
  process(CLK_I)
  begin
    if rising_edge(CLK_I) then
      if RST_I = INVERT_RESET then  -- normal operation, matching emif_wishbone_if's convention
        req_dest_ack <= '0';  -- default: single-cycle pulse (ack, fine as pulse)

        case wb_state is

          when WB_IDLE =>
            if req_dest_req = '1' then
              wb_rnw       <= req_dest_out(REQ_WIDTH-1);
              wb_addr      <= req_dest_out(WB_ADR_SIZE+WB_DAT_SIZE-1 downto WB_DAT_SIZE);
              wb_wdata     <= req_dest_out(WB_DAT_SIZE-1 downto 0);
              req_dest_ack <= '1';   -- confirm receipt, frees AXI side's req handshake
              wb_state     <= WB_DRIVE;
            end if;

          when WB_DRIVE =>
            WB_ADR_O    <= wb_addr;
            WB_CYC_O    <= '1';
            WB_STB_O    <= '1';
            WB_WR_O     <= not wb_rnw;
            if wb_rnw = '0' then
              WB_WR_DAT_O <= wb_wdata;
            end if;
            wb_state <= WB_WAIT_ACK;

          when WB_WAIT_ACK =>
            -- genuine Wishbone backpressure -- arbitrary length wait,
            -- exactly matching classic Wishbone semantics, no fixed
            -- cycle budget of any kind
            if WB_ACK_I = '1' then
              WB_CYC_O <= '0';
              WB_STB_O <= '0';
              WB_WR_O  <= '0';
              if wb_rnw = '1' then
                wb_state <= WB_CAPTURE;
              else
                resp_bundle <= (others => '0');
                resp_send   <= '1';   -- held high, per xpm_cdc_handshake
                                        -- protocol, until resp_rcv confirms
                wb_state    <= WB_SEND_RESP;
              end if;
            end if;

          when WB_CAPTURE =>
            wb_rdata    <= WB_RD_DAT_I;
            resp_bundle <= WB_RD_DAT_I;
            resp_send   <= '1';   -- held high until resp_rcv confirms
            wb_state    <= WB_SEND_RESP;

          when WB_SEND_RESP =>
            -- resp_send remains asserted (set above, not reset here) while
            -- we wait in this and the next state for the macro to confirm
            wb_state <= WB_WAIT_RCV;

          when WB_WAIT_RCV =>
            if resp_rcv = '1' then
              resp_send <= '0';
              wb_state  <= WB_IDLE;
            end if;

          when others =>
            wb_state <= WB_IDLE;

        end case;
      else
        -- in reset (matches emif_wishbone_if's convention: reset is the
        -- else branch, normal operation is RST_I = INVERT_RESET)
        wb_state     <= WB_IDLE;
        req_dest_ack <= '0';
        resp_send    <= '0';
        WB_CYC_O     <= '0';
        WB_STB_O     <= '0';
        WB_WR_O      <= '0';
      end if;
    end if;
  end process;

end architecture rtl;






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
    wb_ack  = 0;

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


