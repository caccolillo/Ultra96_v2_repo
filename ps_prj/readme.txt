-- =============================================================================
-- Add this component declaration before the 'begin' of syscon architecture
-- (around line 248, inside the architecture declarative region)
-- =============================================================================

component ila_0
    port (
        clk     : in std_logic;
        probe0  : in std_logic_vector(0 downto 0);  -- locked
        probe1  : in std_logic_vector(0 downto 0);  -- int_rst
        probe2  : in std_logic_vector(0 downto 0);  -- int_pre_rst
        probe3  : in std_logic_vector(0 downto 0);  -- bridge_rst
        probe4  : in std_logic_vector(0 downto 0);  -- sync_rst
        probe5  : in std_logic_vector(0 downto 0);  -- sys_rst
        probe6  : in std_logic_vector(0 downto 0);  -- sw_rst
        probe7  : in std_logic_vector(0 downto 0)   -- GSR_I
    );
end component;

-- =============================================================================
-- Add this instantiation after 'begin' of syscon architecture
-- (around line 250, before the clk_gen instantiation)
-- =============================================================================

ila_syscon_rst : ila_0
    port map (
        clk        => ref_clk,
        probe0(0)  => locked,
        probe1(0)  => int_rst,
        probe2(0)  => int_pre_rst,
        probe3(0)  => bridge_rst,
        probe4(0)  => sync_rst,
        probe5(0)  => sys_rst,
        probe6(0)  => sw_rst,
        probe7(0)  => GSR_I
    );
