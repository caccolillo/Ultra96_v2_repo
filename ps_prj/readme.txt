-- =============================================================================
-- ILA_1 — add to S0841D.vhd
-- Clock: SYS_CLK (80 MHz)
-- Probes: Wishbone master bus
-- =============================================================================

-- Component declaration (add in architecture declarative region, before begin):

component ila_1
    port (
        clk      : in std_logic;
        probe0   : in std_logic_vector(0 downto 0);   -- wb_m_stb
        probe1   : in std_logic_vector(0 downto 0);   -- wb_m_cyc
        probe2   : in std_logic_vector(0 downto 0);   -- wb_m_ack
        probe3   : in std_logic_vector(0 downto 0);   -- wb_m_we
        probe4   : in std_logic_vector(14 downto 0);  -- wb_m_adr
        probe5   : in std_logic_vector(15 downto 0);  -- wb_m_rd_dat
        probe6   : in std_logic_vector(15 downto 0);  -- wb_m_wr_dat
        probe7   : in std_logic_vector(0 downto 0);   -- wb_s_cyc
        probe8   : in std_logic_vector(0 downto 0);   -- wb_s_we
        probe9   : in std_logic_vector(15 downto 0)   -- wb_s_rd_dat(WB_STB_SYS_CON)
    );
end component;

-- Instantiation (add after begin in S0841D.vhd):

ila_wb : ila_1
    port map (
        clk        => SYS_CLK,
        probe0(0)  => wb_m_stb,
        probe1(0)  => wb_m_cyc,
        probe2(0)  => wb_m_ack,
        probe3(0)  => wb_m_we,
        probe4     => wb_m_adr,
        probe5     => wb_m_rd_dat,
        probe6     => wb_m_wr_dat,
        probe7(0)  => wb_s_cyc,
        probe8(0)  => wb_s_we,
        probe9     => wb_s_rd_dat(WB_STB_SYS_CON)
    );


-- =============================================================================
-- ILA_2 — add to axi4lite_to_wishbone_bridge.vhd
-- Clock: S_AXI_ACLK (125 MHz) — both state machines run here via CDC
-- Probes: CDC handshake and state machines
-- =============================================================================

-- Component declaration (add in architecture declarative region, before begin):

component ila_2
    port (
        clk      : in std_logic;
        probe0   : in std_logic_vector(0 downto 0);  -- req_send
        probe1   : in std_logic_vector(0 downto 0);  -- req_rcv
        probe2   : in std_logic_vector(0 downto 0);  -- resp_send
        probe3   : in std_logic_vector(0 downto 0);  -- resp_rcv
        probe4   : in std_logic_vector(2 downto 0);  -- axi_state (6 values)
        probe5   : in std_logic_vector(2 downto 0);  -- wb_state  (6 values)
        probe6   : in std_logic_vector(0 downto 0);  -- axi_rst_sync
        probe7   : in std_logic_vector(0 downto 0)   -- wb_rst_sync
    );
end component;

-- Helper signals for encoding enums to std_logic_vector
-- (add in architecture declarative region, before begin):

signal axi_state_slv : std_logic_vector(2 downto 0);
signal wb_state_slv  : std_logic_vector(2 downto 0);

-- Enum encoding (add after begin, before ila_bridge instantiation):

with axi_state select axi_state_slv <=
    "000" when AXI_IDLE,
    "001" when AXI_PREP_REQ,
    "010" when AXI_SEND_REQ,
    "011" when AXI_WAIT_RESP,
    "100" when AXI_RESP_WRITE,
    "101" when AXI_RESP_READ,
    "000" when others;

with wb_state select wb_state_slv <=
    "000" when WB_IDLE,
    "001" when WB_DRIVE,
    "010" when WB_WAIT_ACK,
    "011" when WB_CAPTURE,
    "100" when WB_SEND_RESP,
    "101" when WB_WAIT_RCV,
    "000" when others;

-- Instantiation (add after begin in axi4lite_to_wishbone_bridge.vhd):

ila_bridge : ila_2
    port map (
        clk        => S_AXI_ACLK,
        probe0(0)  => req_send,
        probe1(0)  => req_rcv,
        probe2(0)  => resp_send,
        probe3(0)  => resp_rcv,
        probe4     => axi_state_slv,
        probe5     => wb_state_slv,
        probe6(0)  => axi_rst_sync,
        probe7(0)  => wb_rst_sync
    );
