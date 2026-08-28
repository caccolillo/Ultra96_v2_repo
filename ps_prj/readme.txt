
-- axi4lite_to_wishbone_bridge.vhd
--
-- Direct AXI4-Lite -> Wishbone bridge. This talks Wishbone natively, on its own
-- clock, matching the WB_* port/generic naming convention already used by
-- emif_wishbone_if.vhd for consistency across the codebase.
--
-- CLOCK DOMAIN CROSSING: the AXI-Lite side (S_AXI_ACLK, 125 MHz -- the
-- configured AXI Clock Frequency in xdma_0's IP settings) and the
-- Wishbone side (CLK_I, 80 MHz -- matching the rest of your signal-
-- processing logic) are genuinely unrelated clocks. This bridge uses TWO
-- xpm_cdc_handshake macros to safely cross the request (address+data+r/w)
-- one way and the response (read data) back.
--
-- Single-outstanding by construction.
--
-- SAFETY: WB_TIMEOUT_CYCLES bounds the wait for WB_ACK_I; AXI_TIMEOUT_CYCLES
-- is a comprehensive safety net covering the entire round trip (both CDC
-- handshakes + the WB-side wait), guaranteeing the host is never blocked
-- indefinitely regardless of what fails on the far side of the CDC
-- boundary. Both report AXI SLVERR on timeout instead of hanging.

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
    WB_TIMEOUT_CYCLES   : positive := 4000;   -- 50us @ 80MHz
    AXI_TIMEOUT_CYCLES  : positive := 15000;  -- 120us @ 125MHz, comprehensive
                                                -- round-trip safety net
    INVERT_RESET        : natural range 0 to 1 := 0
  );
  port (
    -- ============ AXI4-Lite slave -- M_AXI_LITE master port, 125 MHz domain
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

    -- ============ Wishbone master -- same naming convention as emif_wishbone_if
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

  constant REQ_WIDTH : integer := 1 + WB_ADR_SIZE + WB_DAT_SIZE;
  constant RESP_WIDTH : integer := 1 + WB_DAT_SIZE;

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

  type axi_state_t is (AXI_IDLE, AXI_SEND_REQ, AXI_WAIT_RESP, AXI_RESP_WRITE, AXI_RESP_READ);
  signal axi_state : axi_state_t := AXI_IDLE;

  signal awaddr_reg, araddr_reg : std_logic_vector(C_S_AXI_ADDR_WIDTH-1 downto 0);
  signal wdata_reg               : std_logic_vector(C_S_AXI_DATA_WIDTH-1 downto 0);
  signal rdata_reg               : std_logic_vector(C_S_AXI_DATA_WIDTH-1 downto 0);
  signal aw_latched, w_latched   : std_logic := '0';
  signal is_read_reg             : std_logic;
  signal resp_err_reg            : std_logic := '0';
  signal src_arst_inv            : std_logic;

  signal awready_int, wready_int, arready_int : std_logic;
  signal bvalid_int, rvalid_int : std_logic := '0';
  signal axi_timeout_cnt : integer range 0 to AXI_TIMEOUT_CYCLES := 0;

  type wb_state_t is (WB_IDLE, WB_DRIVE, WB_WAIT_ACK, WB_CAPTURE, WB_SEND_RESP, WB_WAIT_RCV);
  signal wb_state : wb_state_t := WB_IDLE;

  signal wb_rnw   : std_logic;
  signal wb_addr  : std_logic_vector(WB_ADR_SIZE-1 downto 0);
  signal wb_wdata : std_logic_vector(WB_DAT_SIZE-1 downto 0);
  signal wb_rdata : std_logic_vector(WB_DAT_SIZE-1 downto 0);
  signal wb_timeout_cnt : integer range 0 to WB_TIMEOUT_CYCLES := 0;

  signal axi_rst_sync : std_logic;
  signal wb_rst_sync  : std_logic;
  signal wb_rst_raw_active_high : std_logic;

begin

  src_arst_inv <= not S_AXI_ARESETN;

  xpm_cdc_async_rst_axi : xpm_cdc_async_rst
    generic map (
      DEST_SYNC_FF    => 4,
      RST_ACTIVE_HIGH => 1
    )
    port map (
      src_arst  => src_arst_inv,
      dest_clk  => S_AXI_ACLK,
      dest_arst => axi_rst_sync
    );

  wb_rst_raw_active_high <= RST_I when INVERT_RESET = 0 else not RST_I;

  xpm_cdc_async_rst_wb : xpm_cdc_async_rst
    generic map (
      DEST_SYNC_FF    => 4,
      RST_ACTIVE_HIGH => 1
    )
    port map (
      src_arst  => wb_rst_raw_active_high,
      dest_clk  => CLK_I,
      dest_arst => wb_rst_sync
    );

  awready_int <= '1' when (axi_state = AXI_IDLE and aw_latched = '0') else '0';
  wready_int  <= '1' when (axi_state = AXI_IDLE and w_latched  = '0') else '0';
  arready_int <= '1' when (axi_state = AXI_IDLE) else '0';

  S_AXI_AWREADY <= awready_int;
  S_AXI_WREADY  <= wready_int;
  S_AXI_ARREADY <= arready_int;
  S_AXI_RDATA   <= rdata_reg;
  S_AXI_BVALID  <= bvalid_int;
  S_AXI_RVALID  <= rvalid_int;

  S_AXI_BRESP <= "10" when resp_err_reg = '1' else "00";
  S_AXI_RRESP <= "10" when resp_err_reg = '1' else "00";

  process(S_AXI_ACLK)
  begin
    if rising_edge(S_AXI_ACLK) then
      if axi_rst_sync = '1' then
        axi_state     <= AXI_IDLE;
        aw_latched    <= '0';
        w_latched     <= '0';
        req_send      <= '0';
        resp_dest_ack <= '0';
        bvalid_int    <= '0';
        rvalid_int    <= '0';
      else
        resp_dest_ack <= '0';

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
            axi_timeout_cnt <= 0;
            if aw_latched = '1' and w_latched = '1' then
              is_read_reg <= '0';
              req_bundle  <= '0' & awaddr_reg(WB_ADR_SIZE-1 downto 0) & wdata_reg(WB_DAT_SIZE-1 downto 0);
              req_send    <= '1';
              axi_state   <= AXI_SEND_REQ;
            elsif S_AXI_ARVALID = '1' then
              araddr_reg  <= S_AXI_ARADDR;
              is_read_reg <= '1';
              req_bundle  <= '1' & S_AXI_ARADDR(WB_ADR_SIZE-1 downto 0) & (WB_DAT_SIZE-1 downto 0 => '0');
              req_send    <= '1';
              axi_state   <= AXI_SEND_REQ;
            end if;

          when AXI_SEND_REQ =>
            if req_rcv = '1' then
              req_send  <= '0';
              axi_state <= AXI_WAIT_RESP;
            elsif axi_timeout_cnt >= AXI_TIMEOUT_CYCLES then
              req_send     <= '0';
              resp_err_reg <= '1';
              rdata_reg    <= (others => '0');
              aw_latched   <= '0';
              w_latched    <= '0';
              if is_read_reg = '1' then
                axi_state <= AXI_RESP_READ;
              else
                axi_state <= AXI_RESP_WRITE;
              end if;
            else
              axi_timeout_cnt <= axi_timeout_cnt + 1;
            end if;

          when AXI_WAIT_RESP =>
            if resp_dest_req = '1' then
              rdata_reg    <= (others => '0');
              rdata_reg(WB_DAT_SIZE-1 downto 0) <= resp_dest_out(WB_DAT_SIZE-1 downto 0);
              resp_err_reg <= resp_dest_out(RESP_WIDTH-1);
              resp_dest_ack <= '1';
              aw_latched   <= '0';
              w_latched    <= '0';
              if is_read_reg = '1' then
                axi_state <= AXI_RESP_READ;
              else
                axi_state <= AXI_RESP_WRITE;
              end if;
            elsif axi_timeout_cnt >= AXI_TIMEOUT_CYCLES then
              resp_err_reg <= '1';
              rdata_reg    <= (others => '0');
              aw_latched   <= '0';
              w_latched    <= '0';
              if is_read_reg = '1' then
                axi_state <= AXI_RESP_READ;
              else
                axi_state <= AXI_RESP_WRITE;
              end if;
            else
              axi_timeout_cnt <= axi_timeout_cnt + 1;
            end if;

          when AXI_RESP_WRITE =>
            bvalid_int <= '1';
            if S_AXI_BREADY = '1' and bvalid_int = '1' then
              bvalid_int <= '0';
              axi_state  <= AXI_IDLE;
            end if;

          when AXI_RESP_READ =>
            rvalid_int <= '1';
            if S_AXI_RREADY = '1' and rvalid_int = '1' then
              rvalid_int <= '0';
              axi_state  <= AXI_IDLE;
            end if;

          when others =>
            axi_state <= AXI_IDLE;

        end case;
      end if;
    end if;
  end process;

  xpm_cdc_handshake_req : xpm_cdc_handshake
    generic map (
      DEST_EXT_HSK   => 1,
      DEST_SYNC_FF   => 4,
      INIT_SYNC_FF   => 1,
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

  xpm_cdc_handshake_resp : xpm_cdc_handshake
    generic map (
      DEST_EXT_HSK   => 1,
      DEST_SYNC_FF   => 4,
      INIT_SYNC_FF   => 1,
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

  process(CLK_I)
  begin
    if rising_edge(CLK_I) then
      if wb_rst_sync = '1' then  -- '1' = reset attivo (xpm_cdc_async_rst RST_ACTIVE_HIGH=1)
        wb_state     <= WB_IDLE;
        req_dest_ack <= '0';
        resp_send    <= '0';
        WB_CYC_O     <= '0';
        WB_STB_O     <= '0';
        WB_WR_O      <= '0';
      else                       -- reset inattivo = operazione normale
        req_dest_ack <= '0';

        case wb_state is

          when WB_IDLE =>
            if req_dest_req = '1' then
              wb_rnw         <= req_dest_out(REQ_WIDTH-1);
              wb_addr        <= req_dest_out(WB_ADR_SIZE+WB_DAT_SIZE-1 downto WB_DAT_SIZE);
              wb_wdata       <= req_dest_out(WB_DAT_SIZE-1 downto 0);
              req_dest_ack   <= '1';
              wb_timeout_cnt <= 0;
              wb_state       <= WB_DRIVE;
            end if;

          when WB_DRIVE =>
            WB_ADR_O    <= wb_addr;
            WB_CYC_O    <= '1';
            WB_STB_O    <= '1';
            WB_WR_O     <= not wb_rnw;
            if wb_rnw = '0' then
              WB_WR_DAT_O <= wb_wdata;
            end if;
            wb_timeout_cnt <= 0;
            wb_state       <= WB_WAIT_ACK;

          when WB_WAIT_ACK =>
            if WB_ACK_I = '1' then
              WB_CYC_O <= '0';
              WB_STB_O <= '0';
              WB_WR_O  <= '0';
              if wb_rnw = '1' then
                wb_state <= WB_CAPTURE;
              else
                resp_bundle <= '0' & std_logic_vector(to_unsigned(0, WB_DAT_SIZE));
                resp_send   <= '1';
                wb_state    <= WB_SEND_RESP;
              end if;
            elsif wb_timeout_cnt >= WB_TIMEOUT_CYCLES then
              WB_CYC_O    <= '0';
              WB_STB_O    <= '0';
              WB_WR_O     <= '0';
              resp_bundle <= '1' & std_logic_vector(to_unsigned(0, WB_DAT_SIZE));
              resp_send   <= '1';
              wb_state    <= WB_SEND_RESP;
            else
              wb_timeout_cnt <= wb_timeout_cnt + 1;
            end if;

          when WB_CAPTURE =>
            wb_rdata    <= WB_RD_DAT_I;
            resp_bundle <= '0' & WB_RD_DAT_I;
            resp_send   <= '1';
            wb_state    <= WB_SEND_RESP;

          when WB_SEND_RESP =>
            wb_state <= WB_WAIT_RCV;

          when WB_WAIT_RCV =>
            if resp_rcv = '1' then
              resp_send <= '0';
              wb_state  <= WB_IDLE;
            end if;

          when others =>
            wb_state <= WB_IDLE;

        end case;
      end if;
    end if;
  end process;

end architecture rtl;
