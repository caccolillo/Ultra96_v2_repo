
-- =============================================================================
-- S0841D_sim.vhd
-- Simulation-only copy of S0841D architecture.
-- Changes vs production S0841D.vhd:
--   1. Entity ports: only AXI4-Lite (from testbench) + hardware I/O that
--      sub-blocks need internally. All top-level hardware pins removed.
--   2. Clock/reset generated internally (no XDMA clocking wizard needed).
--   3. PCIe_to_wishbone_bridge instance replaced with axi4lite_to_wishbone_bridge.
--   4. legacy_dsp_remap_inst removed; wb_m_* connected directly to
--      wb_distribution_1 (same bypass already applied in HW).
--   5. All external hardware outputs left open / inputs driven '0'.
-- =============================================================================

library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;
use work.design_constants.all;
use work.types.all;

entity S0841D_sim is
    port (
        -- AXI4-Lite slave (driven by testbench, 125 MHz domain)
        S_AXI_ACLK     : in  std_logic;
        S_AXI_ARESETN  : in  std_logic;

        S_AXI_AWADDR   : in  std_logic_vector(31 downto 0);
        S_AXI_AWVALID  : in  std_logic;
        S_AXI_AWREADY  : out std_logic;

        S_AXI_WDATA    : in  std_logic_vector(31 downto 0);
        S_AXI_WSTRB    : in  std_logic_vector(3 downto 0);
        S_AXI_WVALID   : in  std_logic;
        S_AXI_WREADY   : out std_logic;

        S_AXI_BRESP    : out std_logic_vector(1 downto 0);
        S_AXI_BVALID   : out std_logic;
        S_AXI_BREADY   : in  std_logic;

        S_AXI_ARADDR   : in  std_logic_vector(31 downto 0);
        S_AXI_ARVALID  : in  std_logic;
        S_AXI_ARREADY  : out std_logic;

        S_AXI_RDATA    : out std_logic_vector(31 downto 0);
        S_AXI_RRESP    : out std_logic_vector(1 downto 0);
        S_AXI_RVALID   : out std_logic;
        S_AXI_RREADY   : in  std_logic
    );
end entity S0841D_sim;

architecture behavioral of S0841D_sim is

    -- -------------------------------------------------------------------------
    -- Copied verbatim from S0841D architecture declarations
    -- -------------------------------------------------------------------------
    constant INVERT_RESET : std_logic := '0';

    -- system clock, 20MHz
    signal CLOCK_20MHZ_I : std_logic := '0';
    -- synchronous system reset
    signal GSR : std_logic_vector(0 downto 0);

    -- system clock, 80MHz
    signal SYS_CLK : std_logic;
    -- synchronous system reset
    signal SYS_RST : std_logic;

    -- Wishbone controller bus (master, from bridge)
    signal wb_m_adr    : std_logic_vector(WB_ADR_WIDTH - 1 downto 0);
    signal wb_m_rd_dat : std_logic_vector(WB_DAT_WIDTH - 1 downto 0);
    signal wb_m_wr_dat : std_logic_vector(WB_DAT_WIDTH - 1 downto 0);
    signal wb_m_we     : std_logic;
    signal wb_m_stb    : std_logic;
    signal wb_m_ack    : std_logic;
    signal wb_m_cyc    : std_logic;

    -- Wishbone translated bus (legacy_dsp_remap output — not used in sim, tied off)
    signal wb_t_adr    : std_logic_vector(WB_ADR_WIDTH - 1 downto 0);
    signal wb_t_rd_dat : std_logic_vector(WB_DAT_WIDTH - 1 downto 0);
    signal wb_t_wr_dat : std_logic_vector(WB_DAT_WIDTH - 1 downto 0);
    signal wb_t_we     : std_logic;
    signal wb_t_stb    : std_logic;
    signal wb_t_ack    : std_logic;
    signal wb_t_cyc    : std_logic;

    -- Wishbone peripherals bus
    signal wb_s_adr    : std_logic_vector(WB_BLOCK_ADR_WIDTH - 1 downto 0);
    signal wb_s_rd_dat : wb_data_array(0 to WB_STB_COUNT - 1);
    signal wb_s_wr_dat : std_logic_vector(WB_DAT_WIDTH - 1 downto 0);
    signal wb_s_we     : std_logic;
    signal wb_s_ack    : wb_ctrl_array(0 to WB_STB_COUNT - 1);
    signal wb_s_cyc    : std_logic;
    signal wb_s_stb    : wb_ctrl_array(0 to WB_STB_COUNT - 1);

    -- AMC7891 DAC/ADC data
    signal dac1_data   : std_logic_vector(AMC7891_DAC_WIDTH - 1 downto 0);
    signal dac2_data   : std_logic_vector(AMC7891_DAC_WIDTH - 1 downto 0);
    signal dac3_data   : std_logic_vector(AMC7891_DAC_WIDTH - 1 downto 0);
    signal adc7_data   : std_logic_vector(AMC7891_ADC_WIDTH - 1 downto 0);
    signal adc7_stb    : std_logic;

    -- EMIF signals (not used in sim)
    signal EMIF_CLK_I   : std_logic;
    signal nEMIF_OE_I   : std_logic;
    signal nEMIF_WE_I   : std_logic;
    signal EMIF_A_I     : std_logic_vector(EMIF_ADDR_WIDTH - 1 downto 0);
    signal EMIF_BA1_I   : std_logic;
    signal EMIF_D_IO    : std_logic_vector(EMIF_DATA_WIDTH - 1 downto 0);
    signal EMIF_DATA_I  : STD_LOGIC_VECTOR(EMIF_DATA_WIDTH - 1 DOWNTO 0);
    signal EMIF_DATA_O  : STD_LOGIC_VECTOR(EMIF_DATA_WIDTH - 1 DOWNTO 0);
    signal EMIF_DATA_T  : STD_LOGIC;
    signal EMIF_WAIT_O  : std_logic_vector(EMIF_WAIT_WIDTH - 1 downto 0);
    signal nEMIF_CS2_I  : std_logic;

    -- Misc internal signals
    signal rf_sw_transmitting : std_logic;
    signal el_ptt             : std_logic;
    signal init_complete      : std_logic;
    signal full_sample_rate   : std_logic;
    signal current_load       : std_logic_vector(ADS7868_WIDTH - 1 downto 0);
    signal voltage_ctrl       : std_logic_vector(DAC5311_WIDTH - 1 downto 0);
    signal dsp_sync           : std_logic;
    signal debug              : std_logic_vector(5 downto 0);

    -- -------------------------------------------------------------------------
    -- Component declarations
    -- -------------------------------------------------------------------------
    component axi4lite_to_wishbone_bridge is
        generic (
            C_S_AXI_ADDR_WIDTH  : integer  := 32;
            C_S_AXI_DATA_WIDTH  : integer  := 32;
            WB_ADR_SIZE         : positive := 16;
            WB_DAT_SIZE         : positive := 16;
            WB_TIMEOUT_CYCLES   : positive := 4000;
            AXI_TIMEOUT_CYCLES  : positive := 15000;
            INVERT_RESET        : natural range 0 to 1 := 0
        );
        port (
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
            CLK_I         : in  std_logic;
            RST_I         : in  std_logic;
            WB_ADR_O      : out std_logic_vector(WB_ADR_SIZE-1 downto 0);
            WB_RD_DAT_I   : in  std_logic_vector(WB_DAT_SIZE-1 downto 0);
            WB_WR_DAT_O   : out std_logic_vector(WB_DAT_SIZE-1 downto 0);
            WB_STB_O      : out std_logic;
            WB_WR_O       : out std_logic;
            WB_ACK_I      : in  std_logic;
            WB_CYC_O      : out std_logic
        );
    end component;

begin

    -- =========================================================================
    -- Internal clock generation (20 MHz, 50 ns period)
    -- =========================================================================
    clk_20_proc : process
    begin
        CLOCK_20MHZ_I <= '0'; wait for 25 ns;
        CLOCK_20MHZ_I <= '1'; wait for 25 ns;
    end process;

    -- SYS_CLK mirrors CLOCK_20MHZ_I in simulation
    -- (in HW this would be 80 MHz from XDMA; 20 MHz is sufficient for sim)
    SYS_CLK <= CLOCK_20MHZ_I;

    -- GSR and SYS_RST: hold high for ~1 us then release
    rst_proc : process
    begin
        GSR(0)  <= '1';
        SYS_RST <= '1';
        wait for 1050 ns;
        GSR(0)  <= '0';
        SYS_RST <= '0';
        wait;
    end process;

    -- =========================================================================
    -- Debug output bus (copied from S0841D)
    -- =========================================================================
    ext_debug : entity work.sdr_output_bus
        port map (
            CLK_I   => SYS_CLK,
            LOGIC_I => debug,
            PIN_O   => open
        );

    debug <= (others => '0');

    -- unused IRQ
    -- FPGA_IRQ7_O <= '1';  -- not a port in sim entity, ignore

    -- unused Wishbone signals (copied verbatim from S0841D)
    wb_s_ack(WB_STB_LEGACY_0 to WB_STB_LEGACY_3)           <= (others => '0');
    wb_s_ack(WB_STB_SPARE_15 to WB_STB_SPARE_15)           <= (others => '0');
    wb_s_ack(WB_STB_SPARE_23 to WB_STB_TX_DMA_ALIAS)       <= (others => '0');
    wb_s_ack(WB_STB_SPARE_38 to WB_STB_RX_DMA_ALIAS)       <= (others => '0');
    wb_s_rd_dat(WB_STB_LEGACY_0 to WB_STB_LEGACY_3)        <= (others => (others => '0'));
    wb_s_rd_dat(WB_STB_SPARE_15 to WB_STB_SPARE_15)        <= (others => (others => '0'));
    wb_s_rd_dat(WB_STB_SPARE_23 to WB_STB_TX_DMA_ALIAS)    <= (others => (others => '0'));
    wb_s_rd_dat(WB_STB_SPARE_38 to WB_STB_RX_DMA_ALIAS)    <= (others => (others => '0'));

    -- DAC data tied off
    dac1_data <= (others => '0');
    dac2_data <= (others => '0');
    dac3_data <= (others => '0');

    -- =========================================================================
    -- PCIe / Wishbone bridge
    -- REPLACED: PCIe_to_wishbone_bridge (block design wrapper)
    -- WITH:     axi4lite_to_wishbone_bridge (RTL, directly AXI-driven)
    -- legacy_dsp_remap_inst REMOVED: wb_m_* goes directly to wb_distribution_1
    -- =========================================================================
    PCIe_to_wishbone_bridge : entity work.axi4lite_to_wishbone_bridge
        generic map (
            C_S_AXI_ADDR_WIDTH  => 32,
            C_S_AXI_DATA_WIDTH  => 32,
            WB_ADR_SIZE         => WB_ADR_WIDTH,
            WB_DAT_SIZE         => WB_DAT_WIDTH,
            WB_TIMEOUT_CYCLES   => 200,    -- shorter for simulation
            AXI_TIMEOUT_CYCLES  => 600,
            INVERT_RESET        => 0
        )
        port map (
            S_AXI_ACLK    => S_AXI_ACLK,
            S_AXI_ARESETN => S_AXI_ARESETN,
            S_AXI_AWADDR  => S_AXI_AWADDR,
            S_AXI_AWVALID => S_AXI_AWVALID,
            S_AXI_AWREADY => S_AXI_AWREADY,
            S_AXI_WDATA   => S_AXI_WDATA,
            S_AXI_WSTRB   => S_AXI_WSTRB,
            S_AXI_WVALID  => S_AXI_WVALID,
            S_AXI_WREADY  => S_AXI_WREADY,
            S_AXI_BRESP   => S_AXI_BRESP,
            S_AXI_BVALID  => S_AXI_BVALID,
            S_AXI_BREADY  => S_AXI_BREADY,
            S_AXI_ARADDR  => S_AXI_ARADDR,
            S_AXI_ARVALID => S_AXI_ARVALID,
            S_AXI_ARREADY => S_AXI_ARREADY,
            S_AXI_RDATA   => S_AXI_RDATA,
            S_AXI_RRESP   => S_AXI_RRESP,
            S_AXI_RVALID  => S_AXI_RVALID,
            S_AXI_RREADY  => S_AXI_RREADY,
            CLK_I         => SYS_CLK,
            RST_I         => SYS_RST,
            WB_ADR_O      => wb_m_adr,
            WB_RD_DAT_I   => wb_m_rd_dat,
            WB_WR_DAT_O   => wb_m_wr_dat,
            WB_STB_O      => wb_m_stb,
            WB_WR_O       => wb_m_we,
            WB_ACK_I      => wb_m_ack,
            WB_CYC_O      => wb_m_cyc
        );

    -- =========================================================================
    -- wb_distribution_1 — connected directly to wb_m_* (no legacy_dsp_remap)
    -- Copied verbatim from S0841D lines 620-646
    -- =========================================================================
    wb_distribution_1 : entity work.wb_distribution
        generic map (
            STB_COUNT    => WB_STB_COUNT,
            BLOCK_WIDTH  => WB_BLOCK_ADR_WIDTH,
            WB_ADR_SIZE  => WB_ADR_WIDTH,
            INVERT_RESET => INVERT_RESET
        )
        port map (
            RST_I        => SYS_RST,
            CLK_I        => SYS_CLK,

            WB_ADR_I     => wb_m_adr,
            WB_RD_DAT_O  => wb_m_rd_dat,
            WB_WR_DAT_I  => wb_m_wr_dat,
            WB_WE_I      => wb_m_we,
            WB_CYC_I     => wb_m_cyc,
            WB_ACK_O     => wb_m_ack,
            WB_STB_I     => wb_m_stb,

            WB_ADR_O     => wb_s_adr,
            WB_RD_DAT_I  => wb_s_rd_dat,
            WB_WR_DAT_O  => wb_s_wr_dat,
            WB_WE_O      => wb_s_we,
            WB_ACK_I     => wb_s_ack,
            WB_CYC_O     => wb_s_cyc,
            WB_STB_O     => wb_s_stb
        );

    -- =========================================================================
    -- led_flash_pattern_1 — copied verbatim from S0841D lines 651-674
    -- =========================================================================
    led_flash_pattern_1 : entity work.led_flash_pattern
        generic map (
            CLK_FREQ      => SYS_CLK_FREQ_HZ,
            FLASH_FREQ    => LED_FLASH_FREQ,
            FLASH_COUNT   => LED_FLASH_COUNT,
            OFF_COUNT     => LED_OFF_COUNT,
            ACTIVE_STATE  => LED_ACTIVE_STATE,
            INVERT_RESET  => INVERT_RESET
        )
        port map (
            CLK_I    => SYS_CLK,
            RST_I    => SYS_RST,
            WB_ADR_I => wb_s_adr,
            WB_DAT_I => wb_s_wr_dat,
            WB_DAT_O => wb_s_rd_dat(WB_STB_LED_FLASH_PATTERN),
            WB_STB_I => wb_s_stb(WB_STB_LED_FLASH_PATTERN),
            WB_CYC_I => wb_s_cyc,
            WB_WE_I  => wb_s_we,
            WB_ACK_O => wb_s_ack(WB_STB_LED_FLASH_PATTERN),
            LED_O    => open
        );

    -- =========================================================================
    -- misc_1 — copied verbatim from S0841D lines 679-692
    -- =========================================================================
    misc_1 : entity work.misc
        port map (
            EXT_CLK_I => CLOCK_20MHZ_I,
            EXT_RST_I => '0',

            WB_ADR_I  => wb_s_adr,
            WB_DAT_I  => wb_s_wr_dat,
            WB_CYC_I  => wb_s_cyc,
            WB_WE_I   => wb_s_we,
            WB_DAT_O  => wb_s_rd_dat(WB_STB_SYS_CON to WB_STB_CONNECT_LINES),
            WB_ACK_O  => wb_s_ack(WB_STB_SYS_CON to WB_STB_CONNECT_LINES),
            WB_STB_I  => wb_s_stb(WB_STB_SYS_CON to WB_STB_CONNECT_LINES),

            -- syscon
            GSR_I              => GSR(0),
            CLK_O              => open,
            FP_20M_REF_O       => open,
            CODEC_MCLK_O       => open,
            RST_O              => open,
            INIT_COMPLETE_O    => init_complete,
            PLL_LOCK_I         => '1',
            ACTIVE_OUTPUT_O    => open,
            FULL_SAMPLE_RATE_O => full_sample_rate,

            -- AMC
            AMC7891_SDO_I  => '0',
            AMC7891_DAV_N_I=> '1',
            AMC7891_SCLK_O => open,
            AMC7891_SDI_O  => open,
            AMC7891_CS_N_O => open,
            DAC1_DATA_I    => dac1_data,
            DAC2_DATA_I    => dac2_data,
            DAC3_DATA_I    => dac3_data,
            ADC7_DATA_O    => adc7_data,
            ADC7_STB_O     => adc7_stb,

            -- RF switch
            RF_SW_TRANSMITTING_I => rf_sw_transmitting,

            -- connect lines
            CONNECT_LINES_O => open,
            IO_REF_SEL_A_O  => open,
            IO_REF_SEL_B_O  => open,

            -- RS422
            RS422_DATA_IO(0) => open,
            RS422_DATA_IO(1) => open,
            RS422_DIR_O(0)   => open,
            RS422_DIR_O(1)   => open,
            RS422_IO_LINK_DATA_I => '0',
            RS422_IO_LINK_DATA_O => open,
            IRQ_O            => open,

            -- Link
            N_LINK_TX_O => open,
            N_LINK_RX_I => '1',

            -- E1
            E1_RX_DATA_I     => '0',
            E1_SCLK_I        => '0',
            E1_RX_CAS_I      => '0',
            E1_RX_FRAME_SYNC_I => '0',
            E1_FREEZE_I      => '0',
            E1_TX_DATA_O     => open,
            E1_TX_CAS_O      => open,
            E1_TX_FRAME_SYNC_O => open,
            E1_PTT_O         => el_ptt,

            -- RSSI
            RSSI_DAC_SCLK_O  => open,
            RSSI_DAC_SYNC_N_O=> open,
            RSSI_DAC_DIN_O   => open,

            -- Serial converter
            SER_CONV_RF_LEVEL_ADC_DATA_I  => '0',
            SER_CONV_TRACKING_ADC_DATA_I  => '0',
            SER_CONV_TRACKING_DAC_DATA_O  => open,
            SER_CONV_ADC_CS_N_O           => open,
            SER_CONV_DAC_SYNC_N_O         => open,
            SER_CONV_SCLK_O               => open,
            SER_CONV_CURRENT_LOAD_O       => current_load,
            SER_CONV_VOLTAGE_CTRL_O       => voltage_ctrl
        );

    -- =========================================================================
    -- receiver_1 — copied verbatim from S0841D lines 773-818
    -- =========================================================================
    receiver_1 : entity work.receiver
        generic map (
            INVERT_RESET => INVERT_RESET
        )
        port map (
            RST_I => SYS_RST,
            CLK_I => SYS_CLK,

            WB_ADR_I => wb_s_adr(7 downto 0),
            WB_DAT_I => wb_s_wr_dat,
            WB_CYC_I => wb_s_cyc,
            WB_WE_I  => wb_s_we,
            WB_DAT_O => wb_s_rd_dat(WB_STB_RX_ADC to WB_STB_RX_MISC_IO),
            WB_ACK_O => wb_s_ack(WB_STB_RX_ADC to WB_STB_RX_MISC_IO),
            WB_STB_I => wb_s_stb(WB_STB_RX_ADC to WB_STB_RX_MISC_IO),

            RX_EN_I          => init_complete,
            FULL_SAMPLE_RATE_I => full_sample_rate,

            ADC_CLK_I_P  => '0',
            ADC_CLK_I_N  => '0',
            ADC_DAT_I    => (others => '0'),
            ADC_OVF_I    => '0',
            ADC_SHDN_O   => open,
            ADC_DITH_O   => open,
            ADC_PGA_O    => open,

            RX_SPI_CLK_O  => open,
            RX_SPI_CS_N_O => open,
            RX_SPI_SDO_O  => open,
            RX_SPI_SDI_I  => '0',
            LE_40M_O      => open,

            AGC_CLK_O     => open,
            AGC_CS_N_O    => open,
            AGC_DATA_O    => open,

            PHANTOM_SQUELCH_O => open,
            SQUELCH_O         => open,
            AUX_SQUELCH_O     => open,
            EN_O              => open,

            DSP_SYNC_O => dsp_sync,
            IRQ_O      => open
        );

    -- =========================================================================
    -- transmitter_1 — copied verbatim from S0841D lines 823-895
    -- =========================================================================
    transmitter_1 : entity work.transmitter
        generic map (
            TGT_DELAY      => TX_TGT_DELAY,
            IQ_DATA_WIDTH  => IQ_DATA_WIDTH,
            DAC_DATA_WIDTH => TX_DAC_DATA_WIDTH,
            INVERT_RESET   => INVERT_RESET
        )
        port map (
            CLK_I => SYS_CLK,
            RST_I => SYS_RST,

            WB_ADR_I => wb_s_adr,
            WB_DAT_I => wb_s_wr_dat,
            WB_DAT_O => wb_s_rd_dat(WB_STB_TX_SPI_VTR to WB_STB_TX_KEYING),
            WB_STB_I => wb_s_stb(WB_STB_TX_SPI_VTR to WB_STB_TX_KEYING),
            WB_CYC_I => wb_s_cyc,
            WB_WE_I  => wb_s_we,
            WB_ACK_O => wb_s_ack(WB_STB_TX_SPI_VTR to WB_STB_TX_KEYING),

            INIT_COMPLETE_I    => init_complete,
            FULL_SAMPLE_RATE_I => full_sample_rate,

            SPI_VTR_CLK_O  => open,
            SPI_VTR_CS_N_O => open,
            SPI_VTR_SDO_O  => open,

            SPI_IQ_CLK_O     => open,
            SPI_IQ_DAC_CS_N_O=> open,
            SPI_IQ_ADC_CS_N_O=> open,
            SPI_IQ_SDO_O     => open,

            SPI_SDI_I        => '0',

            DSP_SYNC_I       => dsp_sync,

            REV_PWR_I        => adc7_data,
            REV_PWR_STB_I    => adc7_stb,

            CURRENT_LOAD_I   => current_load,
            VOLTAGE_CTRL_I   => voltage_ctrl,

            DAC_DATA_O => open,
            DAC_CLK_O  => open,

            ADC_DATA_I => (others => '0'),
            ADC_CLK_I  => '0',

            TX_ENABLE_O      => open,
            PA_ENABLE_O      => open,
            IQ_PSU_ENABLE_O  => open,

            ANT_CHANGE_OVER_O    => open,
            DAC_SPI_RESET_EXT_O  => open,
            ADC_PDWN_EXT_O       => open,

            RF_SW_TRANSMITTING_O => rf_sw_transmitting,

            PTT_MIC_I         => '0',
            PTT_E1_I          => el_ptt,
            PTT_REMOTE_I      => '0',
            PTT_REMOTE_PHANTOM_I => '0',

            EBIT_VSWR_INPUT_I    => '0',
            INHIBIT_INPUT_I      => '0',
            AMP_INHIBIT_INPUT_I  => '0',

            PTT_AMPLIFIER_O   => open,

            PA_HARDWARE_ID    => '0'
        );

end architecture behavioral;





// =============================================================================
// tb_pcie_wb.sv
// Testbench for S0841D_sim.vhd
// Drives AXI4-Lite reads/writes and checks:
//   - SYSCON_ID at BAR 0x0800 reads 0x0000
//   - SYSCON_FPGA_ALIVE at BAR 0x0808 increments over time
//   - Write/readback to a scratchpad register
// =============================================================================

`timescale 1ns/1ps

module tb_pcie_wb;

    // -------------------------------------------------------------------------
    // Parameters
    // -------------------------------------------------------------------------
    parameter AXI_CLK_PERIOD = 8;    // 125 MHz AXI clock
    parameter TIMEOUT_CYCLES = 2000; // max cycles to wait for AXI response

    // SYSCON BAR addresses (post bridge-fix, post legacy_dsp_remap bypass)
    parameter [31:0] ADDR_SYSCON_ID         = 32'h0000_0800;
    parameter [31:0] ADDR_SYSCON_DSP_ALIVE  = 32'h0000_0802;
    parameter [31:0] ADDR_SYSCON_ARM_ALIVE  = 32'h0000_0804;
    parameter [31:0] ADDR_SYSCON_FPGA_ALIVE = 32'h0000_0808;

    // -------------------------------------------------------------------------
    // AXI4-Lite signals
    // -------------------------------------------------------------------------
    logic        aclk    = 0;
    logic        aresetn = 0;

    // Write address
    logic [31:0] awaddr  = 0;
    logic        awvalid = 0;
    logic        awready;

    // Write data
    logic [31:0] wdata   = 0;
    logic [3:0]  wstrb   = 4'hF;
    logic        wvalid  = 0;
    logic        wready;

    // Write response
    logic [1:0]  bresp;
    logic        bvalid;
    logic        bready  = 1;

    // Read address
    logic [31:0] araddr  = 0;
    logic        arvalid = 0;
    logic        arready;

    // Read data
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
    // Test results
    // -------------------------------------------------------------------------
    int pass_count = 0;
    int fail_count = 0;

    // -------------------------------------------------------------------------
    // Tasks
    // -------------------------------------------------------------------------

    // AXI4-Lite read
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

    // AXI4-Lite write
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
        if (bresp != 2'b00)
            $display("WARN: BRESP=0x%0X (SLVERR?) at addr 0x%08X", bresp, addr);
        @(posedge aclk);
    endtask

    // Check helper
    task automatic check(
        input string  name,
        input logic [31:0] got,
        input logic [31:0] expected,
        input logic        exact  // 1=exact match, 0=non-zero check
    );
        if (exact) begin
            if (got === expected) begin
                $display("PASS  %s = 0x%04X", name, got[15:0]);
                pass_count++;
            end else begin
                $display("FAIL  %s = 0x%04X (expected 0x%04X)", name, got[15:0], expected[15:0]);
                fail_count++;
            end
        end else begin
            if (got !== 32'h0000_0000 && got !== 32'hFFFF_FFFF) begin
                $display("PASS  %s = 0x%04X (non-zero)", name, got[15:0]);
                pass_count++;
            end else begin
                $display("FAIL  %s = 0x%04X (expected non-zero, non-FFFF)", name, got[15:0]);
                fail_count++;
            end
        end
    endtask

    // -------------------------------------------------------------------------
    // Main test sequence
    // -------------------------------------------------------------------------
    logic [31:0] rd_data;
    logic [1:0]  rd_resp;
    logic [31:0] alive_t0, alive_t1;

    initial begin
        $display("=== tb_pcie_wb start ===");

        // Reset
        aresetn = 0;
        repeat(20) @(posedge aclk);
        aresetn = 1;

        // Wait for fabric reset to release (~1 us @ 20MHz internal)
        #2000;

        // ------------------------------------------------------------------
        // Test 1: SYSCON_ID should be 0x0000
        // ------------------------------------------------------------------
        $display("\n--- Test 1: SYSCON_ID at 0x%08X ---", ADDR_SYSCON_ID);
        axi_read(ADDR_SYSCON_ID, rd_data, rd_resp);
        $display("  RRESP = 0b%02b", rd_resp);
        check("SYSCON_ID", rd_data, 32'h0000_0000, 1);

        // ------------------------------------------------------------------
        // Test 2: SYSCON_FPGA_ALIVE increments over ~5ms
        // ------------------------------------------------------------------
        $display("\n--- Test 2: SYSCON_FPGA_ALIVE increments ---");
        axi_read(ADDR_SYSCON_FPGA_ALIVE, alive_t0, rd_resp);
        $display("  alive_t0 = 0x%04X", alive_t0[15:0]);

        // Wait ~5ms (20 MHz internal clock: 100,000 cycles × 50ns)
        #5_000_000;

        axi_read(ADDR_SYSCON_FPGA_ALIVE, alive_t1, rd_resp);
        $display("  alive_t1 = 0x%04X", alive_t1[15:0]);

        if (alive_t1 > alive_t0) begin
            $display("PASS  FPGA alive counter incremented (%0d → %0d)",
                     alive_t0[15:0], alive_t1[15:0]);
            pass_count++;
        end else begin
            $display("FAIL  FPGA alive counter did NOT increment (%0d → %0d)",
                     alive_t0[15:0], alive_t1[15:0]);
            fail_count++;
        end

        // ------------------------------------------------------------------
        // Test 3: Write/readback to ARM_ALIVE (writable scratchpad in SYSCON)
        // ------------------------------------------------------------------
        $display("\n--- Test 3: Write/readback ARM_ALIVE at 0x%08X ---", ADDR_SYSCON_ARM_ALIVE);
        axi_write(ADDR_SYSCON_ARM_ALIVE, 32'h0000_A55A);
        axi_read (ADDR_SYSCON_ARM_ALIVE, rd_data, rd_resp);
        check("ARM_ALIVE readback", rd_data, 32'h0000_A55A, 1);

        // ------------------------------------------------------------------
        // Test 4: Read DSP_ALIVE — should be 0 (DSP not present in sim)
        // ------------------------------------------------------------------
        $display("\n--- Test 4: DSP_ALIVE at 0x%08X ---", ADDR_SYSCON_DSP_ALIVE);
        axi_read(ADDR_SYSCON_DSP_ALIVE, rd_data, rd_resp);
        check("DSP_ALIVE", rd_data, 32'h0000_0000, 1);

        // ------------------------------------------------------------------
        // Summary
        // ------------------------------------------------------------------
        $display("\n=== SUMMARY: %0d PASS, %0d FAIL ===", pass_count, fail_count);
        if (fail_count == 0)
            $display("ALL TESTS PASSED");
        else
            $display("FAILURES DETECTED — check WB addressing and SYSCON clock/reset");

        $finish;
    end

    // Watchdog
    initial begin
        #50_000_000;
        $display("WATCHDOG: simulation exceeded 50ms, aborting");
        $finish;
    end

endmodule

