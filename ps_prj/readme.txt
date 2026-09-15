-- -----------------------------------------------------------------------------
--! @file   radio_link_1wire.vhd
--! @brief  Contains the radio_link_1wire entity
--!
--! @copyright Copyright (c) 2026: Park Air Systems Ltd. All Rights Reserved.
-- -----------------------------------------------------------------------------

-- Standard Libraries
library IEEE;
use IEEE.std_logic_1164.all;
use IEEE.numeric_std.all;

-- Project-specific packages
use work.design_constants.all;   -- WB_DAT_WIDTH
use work.types.all;
use work.functions.all;

-- Xilinx primitives
library unisim;
use unisim.vcomponents.all;     -- DNA_PORT

-- -----------------------------------------------------------------------------
--! @brief Paired radio communications over a shared open-drain 1-wire UART link.
--!
--! Periodically transmits and receives radio state and bit state to/from a
--! remote unit via a UART link.  The remote unit is another instance of this
--! entity.  The UART TX has an open-drain connection to the RX in hardware,
--! forming a logical AND between the local and remote TX.  Therefore, the two
--! transmitters can interfere with each other.  To detect and recover from
--! this, the entity checks the read-back of the transmitted data to ensure
--! that what was sent was also received.  If not, a random back-off time is
--! applied and the entity retries the transmission.  After receiving data from
--! the remote unit, the appropriate Wishbone registers are updated.  A fixed
--! number of frames after receiving data, the entity transmits its states.
--! The two units fall into a repeating pattern in a short amount of time.
--! Baud rate can be changed by writing the divisor register.  Transmit states
--! can be changed by writing the radio state and bit state registers.  Remote
--! states can be retrieved by reading the paired radio state and paired bit
--! state registers.
--!
--! Frame format (3 bytes): SYNC_BYTE | RadioState | RadioBitState
--!
--! State machine:
--!
--!   Receiving ──(3 valid bytes)──► Waiting ──► Sending ──► Receiving
--!       │                                          │
--!       └──────────(timeout)───────────────────────┤
--!                                                  │ (collision)
--!                                                  ▼
--!                                             RandomWait ──► Receiving
--!
--! PRNG seeding via DNA_PORT:
--!   On Artix-7, each device has a unique 57-bit DNA value.  At startup the
--!   DnaInit FSM reads this value serially (57 shift cycles) and XOR-folds it
--!   into a 9-bit PRNG seed.  The link FSM is held in reset until seeding is
--!   complete, ensuring the two units always start from different back-off
--!   sequences even when programmed with identical bitstreams.
--!
--! Register map (WB_ADR_I 3 bits):
--!   0 = REG_ID         (read-only:  block ID / revision)
--!   1 = REG_DIVISOR    (read/write: baud divisor, CLK / divisor = 16 × baud)
--!   2 = REG_PRS        (read-only:  paired radio state, updated by 1-wire RX)
--!   3 = REG_PRB        (read-only:  paired bit state,   updated by 1-wire RX)
--!   4 = REG_RS         (read/write: local radio state,  included in TX frame)
--!   5 = REG_RB         (read/write: local bit state,    included in TX frame)
--!
--! @param DEFAULT_DIVISOR  Amount CLK_I is divided by to generate baud rate
--!                         × 16 on reset.  Set to 5 for simulation (9600 baud
--!                         @ 80 MHz gives divisor 520 for hardware).
--! @param INVERT_RESET     '1' if RST_I is active-low, '0' if active-high.
--! @param SIM_DNA_VALUE    Passed to DNA_PORT SIM_DNA_VALUE generic.  Ignored
--!                         in synthesis.  Set to a different value for each
--!                         DUT instance in simulation so that the two units
--!                         derive different PRNG seeds.  Must be 57 bits.
-- -----------------------------------------------------------------------------
entity radio_link_1wire is
    generic (
        --! Baud rate divisor loaded at reset (sim=5, hw=520 for 9600 @ 80 MHz)
        DEFAULT_DIVISOR : natural                 := 5;   -- 520
        --! '1' for active-low RST_I, '0' for active-high
        INVERT_RESET    : std_logic               := '0';
        --! DNA_PORT simulation value — 57 bits, distinct per instance in sim
        SIM_DNA_VALUE   : bit_vector(56 downto 0) := "0" & X"DEAD_BEEF_CAFE_01"
    );
    port (
        -- Syscon Interface
        --! Synchronous reset
        RST_I       : in  std_logic;
        --! System clock
        CLK_I       : in  std_logic;
        -- Wishbone Interface
        --! Wishbone address
        WB_ADR_I    : in  std_logic_vector(2 downto 0);
        --! Wishbone write data
        WB_DAT_I    : in  std_logic_vector(WB_DAT_WIDTH - 1 downto 0);
        --! Wishbone read data
        WB_DAT_O    : out std_logic_vector(WB_DAT_WIDTH - 1 downto 0);
        --! Wishbone strobe
        WB_STB_I    : in  std_logic;
        --! Wishbone cycle
        WB_CYC_I    : in  std_logic;
        --! Wishbone write enable
        WB_WE_I     : in  std_logic;
        --! Wishbone acknowledge
        WB_ACK_O    : out std_logic;
        -- Non-Wishbone signals
        --! Inverted UART transmit pin; must be hierarchically connected to FPGA pin
        N_LINK_TX_O : out std_logic;
        --! Inverted UART receive pin; must be hierarchically connected to FPGA pin
        N_LINK_RX_I : in  std_logic
    );
end entity radio_link_1wire;

-- -----------------------------------------------------------------------------
--! @brief See entity declaration
-- -----------------------------------------------------------------------------
architecture RTL of radio_link_1wire is

    -- -------------------------------------------------------------------------
    --! @brief Number of data bits in UART
    -- -------------------------------------------------------------------------
    constant DATA_BITS : positive := 8;

    -- -------------------------------------------------------------------------
    --! @brief Wishbone ID for this block
    -- -------------------------------------------------------------------------
    constant ID_REV : natural := 1;

    -- -------------------------------------------------------------------------
    --! @brief Revision ID register offset
    -- -------------------------------------------------------------------------
    constant REG_ID : natural := 0;

    -- -------------------------------------------------------------------------
    --! @brief Clock divisor register offset
    -- -------------------------------------------------------------------------
    constant REG_DIVISOR : natural := 1;

    -- -------------------------------------------------------------------------
    --! @brief Remote radio state register offset
    -- -------------------------------------------------------------------------
    constant REG_PAIRED_RADIO_STATE : natural := 2;

    -- -------------------------------------------------------------------------
    --! @brief Remote radio bit state register offset
    -- -------------------------------------------------------------------------
    constant REG_PAIRED_BIT_STATE : natural := 3;

    -- -------------------------------------------------------------------------
    --! @brief Local radio state register offset
    -- -------------------------------------------------------------------------
    constant REG_RADIO_STATE : natural := 4;

    -- -------------------------------------------------------------------------
    --! @brief Local radio bit state register offset
    -- -------------------------------------------------------------------------
    constant REG_BIT_STATE : natural := 5;

    -- -------------------------------------------------------------------------
    --! @brief Sync byte used to validate frame alignment and reject self-echo
    -- -------------------------------------------------------------------------
    constant SYNC_BYTE : std_logic_vector(DATA_BITS - 1 downto 0) := x"A5";

    -- -------------------------------------------------------------------------
    --! @brief Number of data bytes to send (SYNC + RadioState + RadioBitState)
    -- -------------------------------------------------------------------------
    constant BYTES_TO_SEND : positive := 3;

    -- -------------------------------------------------------------------------
    --! @brief Number of data bytes to receive
    -- -------------------------------------------------------------------------
    constant BYTES_TO_RECEIVE : positive := 3;

    -- -------------------------------------------------------------------------
    --! @brief Number of pulses per UART bit required by the UART (16x oversample)
    -- -------------------------------------------------------------------------
    constant BAUD_MULTIPLIER : positive := 16;

    -- -------------------------------------------------------------------------
    --! @brief Number of byte-frame times to allow for receive (500 ms @ 9600 baud)
    -- -------------------------------------------------------------------------
    constant RECEIVE_TIMEOUT : positive := 481;

    -- -------------------------------------------------------------------------
    --! @brief Number of byte-frame times to allow for link wait (48 ms @ 9600 baud)
    -- -------------------------------------------------------------------------
    constant LINK_WAIT_PERIOD : positive := 46;

    -- -------------------------------------------------------------------------
    --! @brief Maximum value of the pseudo-random back-off time (byte-frames).
    --!        Must be strictly less than RECEIVE_TIMEOUT so a backed-off unit
    --!        is always listening again before the peer's timeout fires.
    -- -------------------------------------------------------------------------
    constant BACK_OFF_MAX : positive := RECEIVE_TIMEOUT / 2;

    -- -------------------------------------------------------------------------
    --! @brief Number of consecutive missed frames before declaring link broken.
    --!        LinkBroken asserts on the (LINK_BROKEN_THRESHOLD + 1)th timeout.
    -- -------------------------------------------------------------------------
    constant LINK_BROKEN_THRESHOLD : natural := 2;

    -- -------------------------------------------------------------------------
    --! @brief Initial ByteCount offset derived from the DNA fold.
    --!        Gives each unit a unique starting point in the receive window,
    --!        preventing both units from timing out simultaneously after reset.
    -- -------------------------------------------------------------------------
    constant INIT_OFFSET_MAX : positive := BACK_OFF_MAX;

    -- -------------------------------------------------------------------------
    --! @brief Number of bits in the DNA shift register (Artix-7 DNA_PORT)
    -- -------------------------------------------------------------------------
    constant DNA_BITS : positive := 57;

    -- -------------------------------------------------------------------------
    --! @brief Number of bits in the PRNG LFSR
    -- -------------------------------------------------------------------------
    constant PRNG_BITS : positive := 9;

    -- -------------------------------------------------------------------------
    -- Active-high reset (normalised from port)
    -- -------------------------------------------------------------------------
    signal rst : std_logic;

    -- -------------------------------------------------------------------------
    -- DNA seeding FSM
    -- -------------------------------------------------------------------------

    --! @brief DNA seeding state machine states
    type DnaFsmType is (
        DnaRead,    --! Assert READ for one cycle to load DNA shift register
        DnaShift,   --! Shift out 57 bits serially via SHIFT/DOUT
        DnaDone     --! XOR-fold complete; PrngSeed and InitOffset are valid
    );

    --! DNA FSM current state
    signal dna_fsm   : DnaFsmType  := DnaRead;
    --! DNA_PORT READ strobe (one cycle)
    signal dna_read  : std_logic   := '0';
    --! DNA_PORT SHIFT strobe (one cycle per bit)
    signal dna_shift : std_logic   := '0';
    --! DNA_PORT serial data output
    signal dna_dout  : std_logic;
    --! Shift register accumulating DNA bits
    signal dna_sr    : std_logic_vector(DNA_BITS - 1 downto 0) := (others => '0');
    --! Bit counter for DNA shift sequence
    signal dna_cnt   : integer range 0 to DNA_BITS := 0;
    --! XOR-folded 9-bit PRNG seed derived from DNA (non-zero guaranteed)
    signal PrngSeed  : std_logic_vector(PRNG_BITS - 1 downto 0) := (others => '1');
    --! Initial ByteCount offset derived from DNA; unique per device
    signal InitOffset : integer range 0 to INIT_OFFSET_MAX := 0;
    --! '1' once DNA seeding is complete; gates the link FSM out of reset
    signal PrngReady : std_logic := '0';

    -- -------------------------------------------------------------------------
    -- Baud rate generation
    -- -------------------------------------------------------------------------

    --! Baud rate divisor register (written via Wishbone REG_DIVISOR)
    signal divisor_reg   : unsigned(WB_DAT_WIDTH - 1 downto 0) := (others => '0');
    --! Down-counter; BaudEn pulses when it reaches zero
    signal BaudCounter   : unsigned(WB_DAT_WIDTH - 1 downto 0) := (others => '0');
    --! Pulse at 16x baud rate (one cycle per BaudCounter wrap)
    signal BaudEn        : std_logic := '0';
    --! Divides BaudEn by BAUD_MULTIPLIER to produce BitEn
    signal OverSampleCnt : integer range 0 to BAUD_MULTIPLIER - 1 := 0;
    --! Pulse at 1x baud rate; master timing tick for UART TX and RX
    signal BitEn         : std_logic := '0';
    --! Counts BitEn pulses 0..9; wraps to produce ByteEn
    signal ByteBitCnt    : integer range 0 to 9 := 0;
    --! Pulse once per complete 8N1 byte frame (every 10 BitEn pulses)
    signal ByteEn        : std_logic := '0';

    -- -------------------------------------------------------------------------
    -- RX input synchroniser
    -- -------------------------------------------------------------------------

    --! First synchroniser FF (metastability protection stage 1)
    signal rx_ff1 : std_logic := '1';
    --! Stable active-high RX sample (line high = idle = '1')
    signal rx     : std_logic := '1';

    -- -------------------------------------------------------------------------
    -- UART RX (8N1, 16x oversampled)
    -- -------------------------------------------------------------------------

    --! @brief UART RX state machine states
    type RxFsmType is (
        RxIdle,       --! Waiting for start bit (rx='0')
        RxReceiving   --! Receiving bits; sampling at mid-point of each window
    );

    --! UART RX state machine current state
    signal rx_fsm       : RxFsmType := RxIdle;
    --! BaudEn pulse counter within each bit window (0..BAUD_MULTIPLIER-1)
    signal rx_samplecnt : integer range 0 to BAUD_MULTIPLIER - 1 := 0;
    --! Bit counter within current frame (0=start, 1-8=data, 9=stop)
    signal rx_bitcnt    : integer range 0 to 10 := 0;
    --! RX shift register; filled LSB-first during data bits
    signal rx_shift     : std_logic_vector(7 downto 0) := (others => '0');
    --! Received byte; valid for one cycle when rx_valid='1'
    signal rx_byte      : std_logic_vector(DATA_BITS - 1 downto 0) := (others => '0');
    --! One-cycle strobe indicating rx_byte contains a valid received byte
    signal rx_valid     : std_logic := '0';

    -- -------------------------------------------------------------------------
    -- Radio state registers
    -- -------------------------------------------------------------------------

    --! Local radio state (ARM-written via REG_RS; included in TX frame byte 1)
    signal RadioState          : std_logic_vector(DATA_BITS - 1 downto 0) := (others => '0');
    --! Local radio bit state (ARM-written via REG_RB; included in TX frame byte 2)
    signal RadioBitState       : std_logic_vector(DATA_BITS - 1 downto 0) := (others => '0');
    --! Remote radio state (FPGA-written from 1-wire RX; read via REG_PRS)
    signal PairedRadioState    : std_logic_vector(DATA_BITS - 1 downto 0) := (others => '0');
    --! Remote radio bit state (FPGA-written from 1-wire RX; read via REG_PRB)
    signal PairedRadioBitState : std_logic_vector(DATA_BITS - 1 downto 0) := (others => '0');

    -- -------------------------------------------------------------------------
    -- Link FSM + UART TX (combined in p_fsm_tx; TX state held as variables)
    -- -------------------------------------------------------------------------

    --! @brief Link state machine states
    type LinkFsmType is (
        Receiving,             --! Listening for incoming frame from remote unit
        Waiting,               --! Inter-frame gap before transmitting
        Sending,               --! Transmitting local frame
        RandomWait             --! Collision back-off delay
    );

    --! Link FSM current state
    signal LinkState    : LinkFsmType := Receiving;
    --! '1' when link has not received a valid remote frame recently
    signal LinkBroken   : std_logic   := '1';
    --! Number of consecutive receive timeouts since last valid frame
    signal MissedFrames : integer range 0 to 7 := 0;
    --! Byte-frame counter; incremented on ByteEn; reset at each state transition
    signal ByteCount    : integer range 0 to 511 := 0;
    --! Timeout threshold for current state (byte-frames)
    signal ByteLimit    : integer range 0 to 511 := RECEIVE_TIMEOUT;

    --! @brief Received frame buffer type
    type RxBuf_t is array (0 to BYTES_TO_RECEIVE - 1) of
        std_logic_vector(DATA_BITS - 1 downto 0);

    --! Buffer holding the last BYTES_TO_RECEIVE bytes received
    signal RxBuf       : RxBuf_t := (others => (others => '0'));
    --! Number of valid bytes accumulated in RxBuf
    signal RxByteCount : integer range 0 to BYTES_TO_RECEIVE := 0;
    --! Index of the next byte to transmit (0=SYNC, 1=RadioState, 2=RadioBitState)
    signal TxByteIndex : integer range 0 to BYTES_TO_SEND    := 0;

    --! Serial output from the UART TX shift register (idle = '1')
    signal tx_out     : std_logic := '1';
    --! '1' while the FSM is in Sending state (gates tx_drive)
    signal tx_sending : std_logic := '0';
    --! Combinatorial TX drive signal: tx_out when Sending, '1' (idle) otherwise
    signal tx_drive   : std_logic;

    -- -------------------------------------------------------------------------
    -- Collision detection
    -- -------------------------------------------------------------------------

    --! TX drive delayed 3 cycles to match IOB + 2-FF RX pipeline latency
    signal tx_drive_ff1 : std_logic := '1';
    --! TX drive delay stage 2
    signal tx_drive_ff2 : std_logic := '1';
    --! TX drive delay stage 3; compared against rx for collision detection
    signal tx_drive_del : std_logic := '1';
    --! '1' when we released the bus but rx is low (remote is transmitting)
    signal LinkConflict : std_logic;

    -- -------------------------------------------------------------------------
    -- PRNG (9-bit maximal-length XNOR LFSR, Xilinx XAPP211, taps 9 & 5)
    -- -------------------------------------------------------------------------

    --! Current LFSR state; sampled at collision time for back-off duration
    signal PseudoRand : std_logic_vector(PRNG_BITS - 1 downto 0) := (others => '1');
    --! Captured back-off duration (byte-frames) at collision detection
    signal RandomTime : integer range 0 to BACK_OFF_MAX := 0;

begin

    rst      <= RST_I xor INVERT_RESET;

    --! tx_drive is combinatorial: no registration lag on the first TX bit
    tx_drive <= tx_out when tx_sending = '1' else '1';

    --! Collision: we released the bus (tx_drive_del='1') but bus is low
    --! (rx='0'), meaning the remote unit is transmitting at the same time.
    --! When both units pull simultaneously the bus stays low and is
    --! indistinguishable from normal transmission — not a collision.
    LinkConflict <= '1' when tx_drive_del = '1' and rx = '0' else '0';

    -- =========================================================================
    -- DNA_PORT primitive (Artix-7)
    -- SIM_DNA_VALUE is passed from the entity generic so each simulation
    -- instance can be given a unique value without modifying the RTL source.
    -- =========================================================================
    u_dna : DNA_PORT
        generic map (SIM_DNA_VALUE => SIM_DNA_VALUE)
        port map (
            CLK   => CLK_I,
            READ  => dna_read,
            SHIFT => dna_shift,
            DIN   => '0',       --! Serial input not used; tied low
            DOUT  => dna_dout
        );

    -- =========================================================================
    --! @brief DNA seeding FSM
    --!
    --! Runs once at power-on, unaffected by RST_I.  Reads the 57-bit device
    --! DNA serially from DNA_PORT, then XOR-folds it into a 9-bit PRNG seed
    --! and an initial ByteCount offset.  Asserts PrngReady on completion,
    --! releasing the link FSM from reset.
    --!
    --! Sequence:
    --!   DnaRead  (1 cycle):  assert READ to load DNA into primitive SR
    --!   DnaShift (57 cycles): pulse SHIFT; capture DOUT into dna_sr
    --!   DnaDone  (1 cycle):  XOR-fold dna_sr; assert PrngReady
    --!
    --! XOR fold: 57 = 6×9 + 3 bits.  Six 9-bit slices XORed together, then
    --! XORed with the remaining 3 bits zero-padded.  Result is non-zero
    --! because DNA bit 56 is always '1' in hardware (Xilinx guarantee).
    -- =========================================================================
    p_dna_fsm : process(CLK_I)
        variable fold : std_logic_vector(PRNG_BITS - 1 downto 0) := (others => '0');
    begin
        if rising_edge(CLK_I) then
            dna_read  <= '0';
            dna_shift <= '0';
            case dna_fsm is
                when DnaRead =>
                    dna_read  <= '1';
                    dna_cnt   <= 0;
                    dna_sr    <= (others => '0');
                    dna_fsm   <= DnaShift;
                when DnaShift =>
                    dna_shift <= '1';
                    dna_sr    <= dna_sr(DNA_BITS - 2 downto 0) & dna_dout;
                    if dna_cnt = DNA_BITS - 1 then
                        dna_fsm <= DnaDone;
                    else
                        dna_cnt <= dna_cnt + 1;
                    end if;
                when DnaDone =>
                    if PrngReady = '0' then
                        fold := dna_sr(56 downto 48) xor dna_sr(47 downto 39)
                            xor dna_sr(38 downto 30) xor dna_sr(29 downto 21)
                            xor dna_sr(20 downto 12) xor dna_sr(11 downto  3)
                            xor ("000000" & dna_sr(2 downto 0));
                        -- Guard against all-zero (LFSR lock-up state)
                        if fold = (fold'range => '0') then
                            PrngSeed <= (others => '1');
                        else
                            PrngSeed <= fold;
                        end if;
                        -- InitOffset: unique per device, breaks initial synchrony
                        InitOffset <= to_integer(unsigned(dna_sr(7 downto 0)))
                                      mod INIT_OFFSET_MAX;
                        PrngReady  <= '1';
                    end if;
            end case;
        end if;
    end process p_dna_fsm;

    -- =========================================================================
    --! @brief IOB output register
    --!
    --! Registered assignment causes Vivado to infer the IOB output FF.
    --! Inversion maps active-high tx_drive to the inverted-logic pin.
    -- =========================================================================
    p_tx_iob : process(CLK_I)
    begin
        if rising_edge(CLK_I) then
            N_LINK_TX_O <= not tx_drive;
        end if;
    end process p_tx_iob;

    -- =========================================================================
    --! @brief RX input metastability synchroniser
    --!
    --! Two-FF chain on N_LINK_RX_I with polarity correction on the first
    --! stage.  rx is active-high: '1' = line idle (high), '0' = line pulled
    --! low (data or start bit).
    -- =========================================================================
    p_rx_sync : process(CLK_I)
    begin
        if rising_edge(CLK_I) then
            rx_ff1 <= not N_LINK_RX_I;
            rx     <= rx_ff1;
        end if;
    end process p_rx_sync;

    -- =========================================================================
    --! @brief TX drive delay-match pipeline
    --!
    --! Full round-trip latency from tx_drive changing to rx seeing the result:
    --!   Cycle 1: N_LINK_TX_O registers (IOB output FF in p_tx_iob)
    --!   Cycle 2: bus_w changes; rx_ff1 samples it (first sync FF)
    --!   Cycle 3: rx samples rx_ff1 (second sync FF)
    --! Delaying tx_drive by 3 cycles makes LinkConflict compare two signals
    --! with identical pipeline latency, avoiding false conflict on transitions.
    -- =========================================================================
    p_tx_drive_del : process(CLK_I)
    begin
        if rising_edge(CLK_I) then
            tx_drive_ff1 <= tx_drive;
            tx_drive_ff2 <= tx_drive_ff1;
            tx_drive_del <= tx_drive_ff2;
        end if;
    end process p_tx_drive_del;

    -- =========================================================================
    --! @brief Baud rate generator
    --!
    --! BaudCounter counts down from divisor_reg to 0, then wraps and pulses
    --! BaudEn for one cycle, giving a frequency of CLK_I / (divisor_reg + 1).
    --! OverSampleCnt then divides BaudEn by BAUD_MULTIPLIER (16) to produce
    --! BitEn at the UART baud rate.
    --!
    --! At reset, BaudCounter is loaded with DEFAULT_DIVISOR directly (not from
    --! divisor_reg) to avoid a one-clock glitch before p_wb initialises
    --! divisor_reg.
    -- =========================================================================
    p_baud : process(CLK_I)
    begin
        if rising_edge(CLK_I) then
            BaudEn <= '0';
            BitEn  <= '0';
            if rst = '1' then
                BaudCounter   <= to_unsigned(DEFAULT_DIVISOR, BaudCounter'length);
                OverSampleCnt <= 0;
            else
                if BaudCounter = 0 then
                    BaudCounter <= divisor_reg;
                    BaudEn      <= '1';
                    if OverSampleCnt = BAUD_MULTIPLIER - 1 then
                        OverSampleCnt <= 0;
                        BitEn         <= '1';
                    else
                        OverSampleCnt <= OverSampleCnt + 1;
                    end if;
                else
                    BaudCounter <= BaudCounter - 1;
                end if;
            end if;
        end if;
    end process p_baud;

    -- =========================================================================
    --! @brief Byte-frame enable generator
    --!
    --! Divides BitEn by 10 (one complete 8N1 frame: start + 8 data + stop).
    --! ByteCount is incremented on ByteEn so all timeout constants are in
    --! byte-frame units, matching the protocol timing comments.
    -- =========================================================================
    p_byte_en : process(CLK_I)
    begin
        if rising_edge(CLK_I) then
            ByteEn <= '0';
            if rst = '1' then
                ByteBitCnt <= 0;
            elsif BitEn = '1' then
                if ByteBitCnt = 9 then
                    ByteBitCnt <= 0;
                    ByteEn     <= '1';
                else
                    ByteBitCnt <= ByteBitCnt + 1;
                end if;
            end if;
        end if;
    end process p_byte_en;

    -- =========================================================================
    --! @brief UART RX (8N1, 16x oversampled)
    --!
    --! Detects a start bit on the falling edge of rx, then samples each
    --! subsequent bit at the midpoint of its window (BaudEn tick 7 of 16,
    --! counting from the start edge).  False start bits are detected by
    --! re-checking rx at the midpoint of the start bit window.
    --!
    --! rx_valid is asserted for exactly one clock cycle when rx_byte is valid.
    -- =========================================================================
    p_uart_rx : process(CLK_I)
    begin
        if rising_edge(CLK_I) then
            rx_valid <= '0';
            if rst = '1' then
                rx_fsm       <= RxIdle;
                rx_samplecnt <= 0;
                rx_bitcnt    <= 0;
                rx_shift     <= (others => '0');
            else
                case rx_fsm is
                    when RxIdle =>
                        if rx = '0' then
                            rx_fsm       <= RxReceiving;
                            rx_samplecnt <= 0;
                            rx_bitcnt    <= 0;
                        end if;
                    when RxReceiving =>
                        if BaudEn = '1' then
                            if rx_samplecnt = BAUD_MULTIPLIER - 1 then
                                rx_samplecnt <= 0;
                            else
                                rx_samplecnt <= rx_samplecnt + 1;
                            end if;
                            if rx_samplecnt = (BAUD_MULTIPLIER / 2) - 1 then
                                case rx_bitcnt is
                                    when 0 =>   -- start bit
                                        if rx /= '0' then
                                            rx_fsm    <= RxIdle;  -- false start
                                        else
                                            rx_bitcnt <= rx_bitcnt + 1;
                                        end if;
                                    when 1 to 8 =>   -- data bits D0..D7
                                        rx_shift  <= rx & rx_shift(7 downto 1);
                                        rx_bitcnt <= rx_bitcnt + 1;
                                    when others =>   -- stop bit
                                        if rx = '1' then
                                            rx_byte  <= rx_shift;
                                            rx_valid <= '1';
                                        end if;
                                        rx_fsm    <= RxIdle;
                                        rx_bitcnt <= 0;
                                end case;
                            end if;
                        end if;
                end case;
            end if;
        end if;
    end process p_uart_rx;

    -- =========================================================================
    --! @brief PRNG — 9-bit maximal-length XNOR LFSR (Xilinx XAPP211, taps 9 & 5)
    --!
    --! Loaded with PrngSeed while PrngReady='0'; free-running thereafter.
    --! PrngSeed is set to a non-zero value by p_dna_fsm before PrngReady
    --! asserts, so PseudoRand is never in the all-zero lock-up state.
    -- =========================================================================
    p_prng : process(CLK_I)
    begin
        if rising_edge(CLK_I) then
            if PrngReady = '0' then
                PseudoRand <= PrngSeed;
            else
                PseudoRand <= PseudoRand(7 downto 0)
                              & (PseudoRand(8) xnor PseudoRand(4));
            end if;
        end if;
    end process p_prng;

    -- =========================================================================
    --! @brief Combined link FSM and UART TX
    --!
    --! The UART TX state (tx_busy, tx_shift, tx_bitcnt, tx_data) is held as
    --! process variables so the FSM and UART TX logic share them within the
    --! same clock edge, avoiding cross-process delta-cycle visibility issues
    --! that cause double-loading of the shift register.
    --!
    --! Key design decisions:
    --!
    --!   1. InitOffset: ByteCount is pre-loaded to InitOffset on reset release.
    --!      Each unit's offset is derived from its DNA, so the first
    --!      RECEIVE_TIMEOUT fires at different times, breaking initial synchrony.
    --!
    --!   2. UART TX as variables: tx_busy is visible immediately within the
    --!      same clock edge.  When the FSM sets tx_busy:='1', the check
    --!      'elsif tx_busy = 0' in the same edge sees the updated value and
    --!      will not attempt to load a second byte until tx_busy clears.
    --!
    --!   3. tx_sending: registered flag controlling tx_drive mux.  tx_drive
    --!      is combinatorial (tx_out when tx_sending='1'), so there is no
    --!      latency on the first transmitted bit.
    --!
    --!   4. Frame rejection on self-echo: a rejected frame (SYNC matched but
    --!      payload matches local state) does NOT transition to Waiting.
    --!      ByteCount keeps running so the timeout path fires and increments
    --!      MissedFrames, eventually asserting LinkBroken.
    -- =========================================================================
    p_fsm_tx : process(CLK_I)
        --! UART TX shift register — stop(1) & data[7:0] & start(0), shifts right
        variable tx_shift  : std_logic_vector(9 downto 0)           := (others => '1');
        --! '1' while the shift register is clocking out a byte
        variable tx_busy   : std_logic                               := '0';
        --! Bit counter within current TX frame (0..9)
        variable tx_bitcnt : integer range 0 to 9                   := 0;
        --! Byte to load into the shift register
        variable tx_data   : std_logic_vector(DATA_BITS - 1 downto 0) := (others => '0');
    begin
        if rising_edge(CLK_I) then

            if rst = '1' or PrngReady = '0' then
                -- ----- FSM reset -------------------------------------------
                LinkState           <= Receiving;
                LinkBroken          <= '1';
                MissedFrames        <= 0;
                ByteCount           <= InitOffset;  -- unique offset per device
                ByteLimit           <= RECEIVE_TIMEOUT;
                RxBuf               <= (others => (others => '0'));
                RxByteCount         <= 0;
                TxByteIndex         <= 0;
                tx_sending          <= '0';
                RandomTime          <= 0;
                PairedRadioState    <= (others => '0');
                PairedRadioBitState <= (others => '0');
                -- ----- UART TX reset ----------------------------------------
                tx_busy             := '0';
                tx_shift            := (others => '1');
                tx_bitcnt           := 0;
                tx_out              <= '1';

            else

                -- ---- UART TX clock-out (runs every cycle) ------------------
                --! Clock out one bit per BitEn while the shift register is busy
                if BitEn = '1' and tx_busy = '1' then
                    tx_out    <= tx_shift(0);
                    tx_shift  := '1' & tx_shift(9 downto 1);
                    if tx_bitcnt = 9 then
                        tx_busy   := '0';
                        tx_bitcnt := 0;
                        tx_out    <= '1';
                    else
                        tx_bitcnt := tx_bitcnt + 1;
                    end if;
                end if;

                -- ---- Byte-frame counter (all states) -----------------------
                if ByteEn = '1' and ByteCount < 511 then
                    ByteCount <= ByteCount + 1;
                end if;

                -- ---- RX byte accumulator (all states) ----------------------
                --! Bytes received during Sending accumulate here; they are the
                --! open-drain loopback of the local frame or the peer's frame.
                --! Preserved across Sending->Receiving so the loopback frame
                --! is validated on the first clock in Receiving state.
                if rx_valid = '1' and RxByteCount < BYTES_TO_RECEIVE then
                    RxBuf(RxByteCount) <= rx_byte;
                    RxByteCount        <= RxByteCount + 1;
                end if;

                -- ---- Clear paired state while link is broken ---------------
                if LinkBroken = '1' then
                    PairedRadioState    <= (others => '0');
                    PairedRadioBitState <= (others => '0');
                end if;

                -- ---- Link FSM ----------------------------------------------
                case LinkState is

                    -- ---------------------------------------------------------
                    --! Receiving: listen for a 3-byte frame from the remote unit.
                    --!
                    --! On success (3 bytes received, SYNC valid, payload not
                    --! matching local state): update PairedRadioState, clear
                    --! LinkBroken, go to Waiting.
                    --!
                    --! On self-echo (payload matches local state): do NOT go to
                    --! Waiting.  Let ByteCount run to timeout so MissedFrames
                    --! accumulates and eventually asserts LinkBroken.
                    --!
                    --! On timeout (ByteCount >= RECEIVE_TIMEOUT): increment
                    --! MissedFrames; assert LinkBroken once threshold is reached;
                    --! go to Sending to transmit the local frame.
                    -- ---------------------------------------------------------
                    when Receiving =>
                        tx_sending <= '0';

                        if RxByteCount = BYTES_TO_RECEIVE then
                            if RxBuf(0) = SYNC_BYTE and
                               (RxBuf(1) /= RadioState or
                                RxBuf(2) /= RadioBitState)
                            then
                                -- Valid frame from remote unit
                                PairedRadioState    <= RxBuf(1);
                                PairedRadioBitState <= RxBuf(2);
                                LinkBroken          <= '0';
                                MissedFrames        <= 0;
                                LinkState   <= Waiting;
                                ByteCount   <= 0;
                                ByteLimit   <= LINK_WAIT_PERIOD;
                            end if;
                            -- Always clear RX buffer; if rejected, ByteCount
                            -- keeps running toward RECEIVE_TIMEOUT
                            RxByteCount <= 0;

                        elsif ByteCount >= RECEIVE_TIMEOUT then
                            if MissedFrames >= LINK_BROKEN_THRESHOLD then
                                LinkBroken <= '1';
                            else
                                MissedFrames <= MissedFrames + 1;
                            end if;
                            LinkState   <= Sending;
                            ByteCount   <= 0;
                            TxByteIndex <= 0;
                            RxByteCount <= 0;
                        end if;

                    -- ---------------------------------------------------------
                    --! Waiting: inter-frame gap before transmitting.
                    --! ByteLimit is set to LINK_WAIT_PERIOD at Receiving->Waiting.
                    -- ---------------------------------------------------------
                    when Waiting =>
                        tx_sending <= '0';
                        if ByteCount >= ByteLimit then
                            LinkState   <= Sending;
                            ByteCount   <= 0;
                            TxByteIndex <= 0;
                            RxByteCount <= 0;
                        end if;

                    -- ---------------------------------------------------------
                    --! Sending: transmit the local 3-byte frame.
                    --!
                    --! Bytes are loaded into the UART shift register (variable)
                    --! one at a time as tx_busy clears.  The variable is visible
                    --! immediately within this process, preventing double-loading.
                    --!
                    --! Collision detection: if LinkConflict fires while Sending,
                    --! abort, capture a random back-off time from the LFSR and
                    --! go to RandomWait.
                    --!
                    --! On completion (all bytes sent): return to Receiving.
                    --! RxByteCount is NOT reset here — bytes received via the
                    --! open-drain loopback during Sending are preserved and
                    --! validated on the first clock in Receiving.
                    -- ---------------------------------------------------------
                    when Sending =>
                        tx_sending <= '1';

                        if LinkConflict = '1' then
                            tx_sending <= '0';
                            RandomTime <= to_integer(unsigned(PseudoRand))
                                          mod (BACK_OFF_MAX + 1);
                            LinkState  <= RandomWait;
                            ByteCount  <= 0;

                        elsif tx_busy = '0' then
                            if TxByteIndex < BYTES_TO_SEND then
                                case TxByteIndex is
                                    when 0      => tx_data := SYNC_BYTE;
                                    when 1      => tx_data := RadioState;
                                    when others => tx_data := RadioBitState;
                                end case;
                                tx_shift  := '1' & tx_data & '0';
                                tx_bitcnt := 0;
                                tx_busy   := '1';
                                TxByteIndex <= TxByteIndex + 1;
                            else
                                tx_sending <= '0';
                                LinkState  <= Receiving;
                                ByteCount  <= 0;
                                ByteLimit  <= RECEIVE_TIMEOUT;
                            end if;
                        end if;

                    -- ---------------------------------------------------------
                    --! RandomWait: back off for RandomTime byte-frames, then
                    --! return to Receiving.  RandomTime is captured from the
                    --! LFSR at the moment of collision detection.
                    -- ---------------------------------------------------------
                    when RandomWait =>
                        tx_sending <= '0';
                        if ByteCount >= RandomTime then
                            LinkState <= Receiving;
                            ByteCount <= 0;
                            ByteLimit <= RECEIVE_TIMEOUT;
                        end if;

                end case;
            end if;
        end if;
    end process p_fsm_tx;

    -- =========================================================================
    --! @brief Wishbone register interface
    --!
    --! Single-cycle ACK.  On write: updates divisor_reg, RadioState, or
    --! RadioBitState depending on WB_ADR_I.  On read: returns the register
    --! contents for the addressed offset; unaddressed bits read as zero.
    -- =========================================================================
    p_wb : process(CLK_I)
    begin
        if rising_edge(CLK_I) then
            WB_ACK_O <= '0';
            WB_DAT_O <= (others => '0');
            if rst = '1' then
                divisor_reg   <= to_unsigned(DEFAULT_DIVISOR, divisor_reg'length);
                RadioState    <= (others => '0');
                RadioBitState <= (others => '0');
            elsif WB_CYC_I = '1' and WB_STB_I = '1' then
                WB_ACK_O <= '1';
                if WB_WE_I = '1' then
                    case to_integer(unsigned(WB_ADR_I)) is
                        when REG_DIVISOR      => divisor_reg   <= unsigned(WB_DAT_I);
                        when REG_RADIO_STATE  => RadioState    <= WB_DAT_I(DATA_BITS-1 downto 0);
                        when REG_BIT_STATE    => RadioBitState <= WB_DAT_I(DATA_BITS-1 downto 0);
                        when others           => null;
                    end case;
                else
                    case to_integer(unsigned(WB_ADR_I)) is
                        when REG_ID =>
                            WB_DAT_O <= std_logic_vector(
                                        to_unsigned(ID_REV, WB_DAT_WIDTH));
                        when REG_DIVISOR =>
                            WB_DAT_O <= std_logic_vector(divisor_reg);
                        when REG_PAIRED_RADIO_STATE =>
                            WB_DAT_O(DATA_BITS-1 downto 0) <= PairedRadioState;
                        when REG_PAIRED_BIT_STATE =>
                            WB_DAT_O(DATA_BITS-1 downto 0) <= PairedRadioBitState;
                        when REG_RADIO_STATE =>
                            WB_DAT_O(DATA_BITS-1 downto 0) <= RadioState;
                        when REG_BIT_STATE =>
                            WB_DAT_O(DATA_BITS-1 downto 0) <= RadioBitState;
                        when others => null;
                    end case;
                end if;
            end if;
        end if;
    end process p_wb;

end architecture RTL;
