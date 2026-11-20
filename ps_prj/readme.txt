## ALINX AX7103 XADC header (2x8, 0.1 in, not fitted by default)
## Port names match xadc_reader_wb. If the core sits in a block design,
## rename to the external port names Vivado gives the wrapper.

## Dedicated VP/VN (header pins 1/2). Placement is automatic; LOC is optional.
set_property PACKAGE_PIN L10 [get_ports VP_I]
set_property PACKAGE_PIN M9  [get_ports VN_I]

## VAUX9 (header pins 5/6)
set_property PACKAGE_PIN J15 [get_ports VAUXP9_I]
set_property PACKAGE_PIN H15 [get_ports VAUXN9_I]

## VAUX0 (header pins 9/10)
set_property PACKAGE_PIN H13 [get_ports VAUXP0_I]
set_property PACKAGE_PIN G13 [get_ports VAUXN0_I]

## Auxiliary analogue inputs still need an IOSTANDARD that matches the bank
## VCCO. Check the AX7103 schematic for the bank voltage and adjust if needed.
set_property IOSTANDARD LVCMOS33 [get_ports {VAUXP0_I VAUXN0_I VAUXP9_I VAUXN9_I}]










#!/usr/bin/env python3
"""
gen_stim.py - writes design.txt, the analogue stimulus for the UNISIM XADC
model (Vivado xsim). One row per time step; TIME in ns, voltages in V,
temperature in degC.

Channel assignment (unipolar, kept inside 0.05..0.95 V to avoid clipping):
  VP/VN   : sine      1.0 kHz
  VAUX0   : cosine    1.0 kHz (90 deg ahead of VP)
  VAUX9   : triangle  1.5 kHz
  TEMP    : square    500 Hz, 25 <-> 85 degC
  VCCINT  : constant  1.00 V
  VCCAUX  : constant  1.80 V
  VCCBRAM : sawtooth  2.0 kHz, 0.95 -> 1.05 V

N inputs are held at 0 V (grounded, as on the board).
Usage: python3 gen_stim.py [output_file]
"""
import math
import sys

T_END_NS  = 2_000_000      # 2 ms of stimulus
T_STEP_NS = 1_000          # 1 us steps, well below the ~8 us per-channel sample spacing

V_MID, V_AMP = 0.50, 0.45  # external channels: 0.05 .. 0.95 V

F_SINE = 1_000.0
F_TRI  = 1_500.0
F_SQR  = 500.0
F_SAW  = 2_000.0


def frac(x):
    return x - math.floor(x)


def triangle(t, f):
    """-1 .. +1 triangle, starting at -1."""
    p = frac(t * f)
    return 4.0 * p - 1.0 if p < 0.5 else 3.0 - 4.0 * p


def main():
    out = sys.argv[1] if len(sys.argv) > 1 else "design.txt"
    with open(out, "w") as fh:
        fh.write("TIME VP VN VAUXP[0] VAUXN[0] VAUXP[9] VAUXN[9] "
                 "TEMP VCCINT VCCAUX VCCBRAM\n")
        for t_ns in range(0, T_END_NS + T_STEP_NS, T_STEP_NS):
            t = t_ns * 1e-9
            vp      = V_MID + V_AMP * math.sin(2 * math.pi * F_SINE * t)
            vaux0   = V_MID + V_AMP * math.cos(2 * math.pi * F_SINE * t)
            vaux9   = V_MID + V_AMP * triangle(t, F_TRI)
            temp    = 25.0 if frac(t * F_SQR) < 0.5 else 85.0
            vccint  = 1.00
            vccaux  = 1.80
            vccbram = 0.95 + 0.10 * frac(t * F_SAW)
            fh.write(f"{t_ns} {vp:.5f} 0.0 {vaux0:.5f} 0.0 {vaux9:.5f} 0.0 "
                     f"{temp:.2f} {vccint:.3f} {vccaux:.3f} {vccbram:.5f}\n")
    print(f"wrote {out}")


if __name__ == "__main__":
    main()









#!/bin/bash

# add vivado to path
alias source="."
if ! command -v vivado &> /dev/null
then
  vivado=$(find /opt/ -wholename "/opt/Xilinx*/Vivado/2023.1/settings64.sh")
  . $vivado
fi

rm -r ./project_1
vivado -mode batch -source ./prj.tcl










#create the project
create_project project_1 ./project_1 -part xc7a100tfgg484-2
set_property target_language VHDL [current_project]
#set up the ip-repo folder location
set_property ip_repo_paths ../ [current_project]
update_ip_catalog
#add source files
add_files -norecurse {./xadc_wb_pkg.vhd ./xadc_reader.vhd ./xadc_reader_wb.vhd}
set_property file_type {VHDL 2008} [get_files {xadc_wb_pkg.vhd xadc_reader.vhd xadc_reader_wb.vhd}]
set_property top xadc_reader_wb [get_filesets sources_1]
update_compile_order -fileset sources_1
#add testbench source files
set_property sim.use_ip_compiled_libs 0 [current_project]
#add simulation files to the project (design.txt is the XADC analogue stimulus,
#vivado copies it into the simulation run directory)
add_files -fileset sim_1 -norecurse {./tb_xadc_reader.sv ./design.txt}
update_compile_order -fileset sim_1
set_property simulator_language Mixed [current_project]
set_property top tb_xadc_reader [get_filesets sim_1]
set_property top_lib xil_defaultlib [get_filesets sim_1]
set_property -name {xsim.simulate.runtime} -value {-all} -objects [get_filesets sim_1]
update_compile_order -fileset sim_1
#package the project as an IP-core
ipx::package_project -root_dir ./ -vendor user.org -library user -taxonomy /UserIP -import_files -set_current false
ipx::open_ipxact_file ./component.xml
set_property display_name {XADC Reader (AX7103)} [ipx::current_core]
set_property description {Continuous XADC sampler: VP/VN, VAUX0, VAUX9, temperature and supplies on 16-bit ports and Wishbone registers} [ipx::current_core]
set_property supported_families {artix7 Production} [ipx::current_core]
#the simulation stimulus generic is not used in a block design
set_property enablement_value false [ipx::get_user_parameters G_SIM_MONITOR_FILE -of_objects [ipx::current_core]]
set_property core_revision 1 [ipx::current_core]
ipx::create_xgui_files [ipx::current_core]
ipx::update_checksums [ipx::current_core]
ipx::check_integrity [ipx::current_core]
ipx::save_core [ipx::current_core]
#add the constraint file to the project
#(added after packaging so the board pin LOCs are not embedded in the IP)
add_files -fileset constrs_1 -norecurse ./ax7103_xadc.xdc
exit












//-----------------------------------------------------------------------------
// tb_xadc_reader.sv
//
// Self-checking testbench for xadc_reader_wb (VHDL) using the UNISIM XADC
// model in Vivado 2023.1 xsim. Analogue stimulus comes from design.txt
// (generated by gen_stim.py):
//   VP/VN   sine     1.0 kHz  0.05..0.95 V
//   VAUX0   cosine   1.0 kHz  0.05..0.95 V
//   VAUX9   triangle 1.5 kHz  0.05..0.95 V
//   TEMP    square   500 Hz   25 / 85 degC
//   VCCINT  1.00 V, VCCAUX 1.80 V
//   VCCBRAM sawtooth 2.0 kHz  0.95..1.05 V
//
// Checks:
//   - ID register and write acknowledge over Wishbone
//   - every channel updates on its own 16-bit port
//   - min/max of each port over the run match the stimulus
//   - sine and cosine are in quadrature (cosine at peak when sine at mid)
//   - each Wishbone register matches its port
// Every sample is also written to xadc_samples.csv for plotting.
// In the xsim GUI, set the *_raw ports to Waveform Style > Analog.
//-----------------------------------------------------------------------------
`timescale 1ns / 1ps

module tb_xadc_reader;

    localparam int    CLK_FREQ_HZ = 80_000_000;
    localparam real   CLK_PERIOD  = 1.0e9 / CLK_FREQ_HZ;
    localparam int    AVG_SEL     = 0;       // no averaging: one result per pass
    localparam time   T_SETTLE    = 30us;    // ignore power-up and calibration
    localparam time   T_RUN       = 2ms;     // length of design.txt

    localparam int    ADR_W       = 4;       // WB_BLOCK_ADR_WIDTH
    localparam int    DAT_W       = 16;      // WB_DAT_WIDTH
    localparam logic [DAT_W-1:0] ID_REV = 16'hAD01;

    // Register word addresses
    localparam logic [ADR_W-1:0] REG_ID        = 4'd0;
    localparam logic [ADR_W-1:0] REG_STATUS    = 4'd1;
    localparam logic [ADR_W-1:0] REG_SEQ_COUNT = 4'd2;
    localparam logic [ADR_W-1:0] REG_VPVN      = 4'd3;
    localparam logic [ADR_W-1:0] REG_VAUX0     = 4'd4;
    localparam logic [ADR_W-1:0] REG_VAUX9     = 4'd5;
    localparam logic [ADR_W-1:0] REG_TEMP      = 4'd6;
    localparam logic [ADR_W-1:0] REG_VCCINT    = 4'd7;
    localparam logic [ADR_W-1:0] REG_VCCAUX    = 4'd8;
    localparam logic [ADR_W-1:0] REG_VCCBRAM   = 4'd9;

    // Channel index = bit position in 'updated'
    localparam int NCH = 7;
    localparam int C_VPVN = 0, C_VAUX0 = 1, C_VAUX9 = 2, C_TEMP = 3,
                   C_VCCINT = 4, C_VCCAUX = 5, C_VCCBRAM = 6;

    logic             clk = 1'b0;
    logic             rst = 1'b1;

    logic [ADR_W-1:0] wb_adr = '0;
    logic [DAT_W-1:0] wb_dat_w = '0;
    logic [DAT_W-1:0] wb_dat_r;
    logic             wb_stb = 1'b0;
    logic             wb_cyc = 1'b0;
    logic             wb_we  = 1'b0;
    logic             wb_ack;

    logic [15:0] vpvn_raw, vaux0_raw, vaux9_raw;
    logic [15:0] temp_raw, vccint_raw, vccaux_raw, vccbram_raw;
    logic [6:0]  updated;

    int errors = 0;

    always #(CLK_PERIOD / 2.0) clk = ~clk;

    //-------------------------------------------------------------------------
    // DUT. Analogue pins are tied off: the UNISIM model takes its values from
    // the SIM_MONITOR_FILE, not from these ports.
    //-------------------------------------------------------------------------
    xadc_reader_wb #(
        .G_CLK_FREQ_HZ      (CLK_FREQ_HZ),
        .G_AVG_SEL          (AVG_SEL),
        .G_SIM_MONITOR_FILE ("design.txt")
    ) dut (
        .CLK_I     (clk),
        .RST_I     (rst),
        .WB_ADR_I  (wb_adr),
        .WB_DAT_I  (wb_dat_w),
        .WB_DAT_O  (wb_dat_r),
        .WB_STB_I  (wb_stb),
        .WB_CYC_I  (wb_cyc),
        .WB_WE_I   (wb_we),
        .WB_ACK_O  (wb_ack),
        .VP_I      (1'b0),
        .VN_I      (1'b0),
        .VAUXP0_I  (1'b0),
        .VAUXN0_I  (1'b0),
        .VAUXP9_I  (1'b0),
        .VAUXN9_I  (1'b0),
        .VPVN_O    (vpvn_raw),
        .VAUX0_O   (vaux0_raw),
        .VAUX9_O   (vaux9_raw),
        .TEMP_O    (temp_raw),
        .VCCINT_O  (vccint_raw),
        .VCCAUX_O  (vccaux_raw),
        .VCCBRAM_O (vccbram_raw),
        .UPDATED_O (updated)
    );

    //-------------------------------------------------------------------------
    // Code / unit helpers
    //-------------------------------------------------------------------------
    function automatic int code12(logic [15:0] raw);
        return int'(raw[15:4]);
    endfunction

    function automatic real to_units(int ch, int code);
        case (ch)
            C_VPVN, C_VAUX0, C_VAUX9: return code / 4096.0;                   // V
            C_TEMP:                   return code * 503.975 / 4096.0 - 273.15; // degC
            default:                  return code * 3.0 / 4096.0;             // V
        endcase
    endfunction

    function automatic logic [15:0] port_of(int ch);
        case (ch)
            C_VPVN:    return vpvn_raw;
            C_VAUX0:   return vaux0_raw;
            C_VAUX9:   return vaux9_raw;
            C_TEMP:    return temp_raw;
            C_VCCINT:  return vccint_raw;
            C_VCCAUX:  return vccaux_raw;
            default:   return vccbram_raw;
        endcase
    endfunction

    //-------------------------------------------------------------------------
    // Sample monitor: min/max per channel, quadrature check, CSV log
    //-------------------------------------------------------------------------
    string names [NCH] = '{"VPVN", "VAUX0", "VAUX9", "TEMP", "VCCINT", "VCCAUX", "VCCBRAM"};
    real   vmin  [NCH];
    real   vmax  [NCH];
    int    count [NCH];
    int    quad_checks = 0;
    int    csv;

    initial begin
        foreach (vmin[i]) begin
            vmin[i]  = 1.0e9;
            vmax[i]  = -1.0e9;
            count[i] = 0;
        end
        $timeformat(-6, 1, " us", 0);
        csv = $fopen("xadc_samples.csv", "w");
        $fdisplay(csv, "time_ns,channel,code,value");
    end

    always @(posedge clk) begin
        if ($time > T_SETTLE) begin
            for (int ch = 0; ch < NCH; ch++) begin
                if (updated[ch]) begin
                    real v;
                    v = to_units(ch, code12(port_of(ch)));
                    count[ch]++;
                    if (v < vmin[ch]) vmin[ch] = v;
                    if (v > vmax[ch]) vmax[ch] = v;
                    $fdisplay(csv, "%0d,%s,%0d,%.4f", $time, names[ch], code12(port_of(ch)), v);
                end
            end

            // Quadrature: when the sine is near mid-scale, the cosine must be
            // near one of its peaks (|cos| close to 0.45 V from mid).
            if (updated[C_VPVN]) begin
                real s, c;
                s = to_units(C_VPVN,  code12(vpvn_raw))  - 0.5;
                c = to_units(C_VAUX0, code12(vaux0_raw)) - 0.5;
                if (s > -0.02 && s < 0.02) begin
                    quad_checks++;
                    if (c > -0.40 && c < 0.40) begin
                        $error("[%0t] quadrature: sine %.3f V from mid but cosine %.3f V", $time, s, c);
                        errors++;
                    end
                end
            end
        end
    end

    //-------------------------------------------------------------------------
    // Wishbone (classic) master tasks. The slave acks one cycle after
    // CYC & STB; CYC/STB drop on the cycle ACK is seen, then one idle cycle.
    //-------------------------------------------------------------------------
    task automatic wb_read(input logic [ADR_W-1:0] adr, output logic [DAT_W-1:0] data);
        int timeout = 0;
        @(posedge clk);
        wb_adr <= adr;
        wb_we  <= 1'b0;
        wb_cyc <= 1'b1;
        wb_stb <= 1'b1;
        do begin
            @(posedge clk);
            timeout++;
        end while (!wb_ack && timeout < 16);
        if (!wb_ack) begin
            $error("WB read 0x%0h: no ACK", adr);
            errors++;
        end
        data = wb_dat_r;
        wb_cyc <= 1'b0;
        wb_stb <= 1'b0;
        @(posedge clk);
    endtask

    task automatic wb_write(input logic [ADR_W-1:0] adr, input logic [DAT_W-1:0] data);
        int timeout = 0;
        @(posedge clk);
        wb_adr   <= adr;
        wb_dat_w <= data;
        wb_we    <= 1'b1;
        wb_cyc   <= 1'b1;
        wb_stb   <= 1'b1;
        do begin
            @(posedge clk);
            timeout++;
        end while (!wb_ack && timeout < 16);
        if (!wb_ack) begin
            $error("WB write 0x%0h: no ACK", adr);
            errors++;
        end
        wb_cyc <= 1'b0;
        wb_stb <= 1'b0;
        wb_we  <= 1'b0;
        @(posedge clk);
    endtask

    //-------------------------------------------------------------------------
    // Checks
    //-------------------------------------------------------------------------
    task automatic check_range(int ch, real exp_min, real exp_max, real tol);
        string unit = (ch == C_TEMP) ? "degC" : "V";
        if (count[ch] == 0) begin
            $error("%-8s no samples", names[ch]);
            errors++;
        end else if (vmin[ch] < exp_min - tol || vmin[ch] > exp_min + tol ||
                     vmax[ch] < exp_max - tol || vmax[ch] > exp_max + tol) begin
            $error("%-8s range %.3f..%.3f %s, expected %.3f..%.3f (+/-%.3f)",
                   names[ch], vmin[ch], vmax[ch], unit, exp_min, exp_max, tol);
            errors++;
        end else begin
            $display("  PASS %-8s %4d samples, range %8.3f .. %8.3f %s",
                     names[ch], count[ch], vmin[ch], vmax[ch], unit);
        end
    endtask

    // Port and register share one flop, but the port can update during the
    // Wishbone read, so retry once before reporting a mismatch.
    task automatic check_reg_vs_port(int ch, logic [ADR_W-1:0] addr);
        logic [DAT_W-1:0] d;
        logic [15:0] p;
        for (int attempt = 0; attempt < 2; attempt++) begin
            wb_read(addr, d);
            p = port_of(ch);
            if (d[15:0] == p) begin
                $display("  PASS %-8s register 0x%02h = port (0x%04h)", names[ch], addr, p);
                return;
            end
        end
        $error("%-8s register 0x%02h = 0x%04h, port = 0x%04h", names[ch], addr, d[15:0], p);
        errors++;
    endtask

    //-------------------------------------------------------------------------
    // Stimulus
    //-------------------------------------------------------------------------
    initial begin
        logic [DAT_W-1:0] d, cnt0, cnt1;

        repeat (20) @(posedge clk);
        rst <= 1'b0;

        wb_read(REG_ID, d);
        if (d !== ID_REV) begin
            $error("ID = 0x%04h, expected 0x%04h", d, ID_REV);
            errors++;
        end else begin
            $display("  PASS ID 0x%04h", d);
        end

        // Unmapped address: acked, reads 0x0000
        wb_read(4'hF, d);
        if (d !== '0) begin
            $error("unmapped read = 0x%04h", d);
            errors++;
        end

        // Writes are acked and change nothing
        wb_write(REG_VPVN, 16'hBEEF);

        wb_read(REG_SEQ_COUNT, cnt0);
        #(T_RUN / 2);

        $display("[%0t] Register vs port", $time);
        check_reg_vs_port(C_VPVN,    REG_VPVN);
        check_reg_vs_port(C_VAUX0,   REG_VAUX0);
        check_reg_vs_port(C_VAUX9,   REG_VAUX9);
        check_reg_vs_port(C_TEMP,    REG_TEMP);
        check_reg_vs_port(C_VCCINT,  REG_VCCINT);
        check_reg_vs_port(C_VCCAUX,  REG_VCCAUX);
        check_reg_vs_port(C_VCCBRAM, REG_VCCBRAM);

        wait_until_end();

        wb_read(REG_SEQ_COUNT, cnt1);
        if (cnt1 <= cnt0) begin
            $error("SEQ_COUNT not advancing (%0d -> %0d)", cnt0, cnt1);
            errors++;
        end else begin
            $display("  PASS SEQ_COUNT %0d -> %0d", cnt0, cnt1);
        end

        // Tolerances cover sampling missing the exact peak (about 8 us
        // between samples of one channel) plus a few LSB of model error.
        $display("[%0t] Waveform ranges", $time);
        check_range(C_VPVN,    0.05, 0.95, 0.02);   // sine
        check_range(C_VAUX0,   0.05, 0.95, 0.02);   // cosine
        check_range(C_VAUX9,   0.05, 0.95, 0.03);   // triangle (sharp peaks)
        check_range(C_TEMP,    25.0, 85.0, 1.0);    // square
        check_range(C_VCCINT,  1.00, 1.00, 0.01);   // constant
        check_range(C_VCCAUX,  1.80, 1.80, 0.01);   // constant
        check_range(C_VCCBRAM, 0.95, 1.05, 0.01);   // sawtooth

        if (quad_checks == 0) begin
            $error("quadrature check never ran");
            errors++;
        end else begin
            $display("  PASS sine/cosine quadrature (%0d checks)", quad_checks);
        end

        $fclose(csv);
        if (errors == 0) $display("*** TEST PASSED ***");
        else             $display("*** TEST FAILED: %0d error(s) ***", errors);
        $finish;
    end

    task automatic wait_until_end();
        while ($time < T_RUN) @(posedge clk);
    endtask

    initial begin
        #(T_RUN + 500us);
        $fatal(1, "Timeout");
    end

endmodule









-------------------------------------------------------------------------------
-- xadc_reader.vhd
--
-- Continuous XADC sampler for the ALINX AX7103 (XC7A100T-2FGG484I).
--
-- AX7103 XADC header (2x8, not fitted by default):
--   pins 1/2   VP_0 / VN_0  -> L10 / M9   dedicated input  (CHANNEL 0x03)
--   pins 5/6   AD9P / AD9N  -> J15 / H15  auxiliary VAUX9  (CHANNEL 0x19)
--   pins 9/10  AD0P / AD0N  -> H13 / G13  auxiliary VAUX0  (CHANNEL 0x10)
--   Input range 1 V p-p per pair.
--
-- The sequencer runs in continuous mode over calibration, temperature,
-- VCCINT, VCCAUX, VCCBRAM, VP/VN, VAUX0 and VAUX9. Each EOC triggers a DRP
-- read of the matching status register; results are held in output
-- registers with a one-cycle strobe per channel.
--
-- Result format: 16-bit status register. 12-bit code in (15 downto 4); with
-- averaging enabled the low nibble carries extra fractional bits.
--   Unipolar external : V    = code / 4096 * 1.0
--   Bipolar  external : V    = signed(code) / 2048 * 0.5
--   Temperature       : degC = code * 503.975 / 4096 - 273.15
--   Supplies          : V    = code / 4096 * 3.0
--
-- Clocking: DCLK = clk. ADCCLK = clk / CLK_DIV, with CLK_DIV derived from
-- G_CLK_FREQ_HZ so that ADCCLK <= 26 MHz (UG480): 200 MHz -> /8, 100 MHz -> /4.
-------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library unisim;
use unisim.vcomponents.all;

entity xadc_reader is
    generic (
        G_CLK_FREQ_HZ      : positive := 80_000_000;
        G_AVG_SEL          : natural range 0 to 3 := 1;  -- 0 off, 1: 16, 2: 64, 3: 256
        G_BIPOLAR          : boolean  := false;          -- external channels bipolar
        G_SIM_MONITOR_FILE : string   := "design.txt"    -- simulation stimulus only
    );
    port (
        clk         : in  std_logic;
        rst         : in  std_logic;                     -- active-high, synchronous

        -- Analogue inputs: connect straight to top-level ports, no buffers
        vp_in       : in  std_logic;
        vn_in       : in  std_logic;
        vauxp0      : in  std_logic;
        vauxn0      : in  std_logic;
        vauxp9      : in  std_logic;
        vauxn9      : in  std_logic;

        -- Latest results (raw status register values)
        vpvn_raw    : out std_logic_vector(15 downto 0);
        vaux0_raw   : out std_logic_vector(15 downto 0);
        vaux9_raw   : out std_logic_vector(15 downto 0);
        temp_raw    : out std_logic_vector(15 downto 0);
        vccint_raw  : out std_logic_vector(15 downto 0);
        vccaux_raw  : out std_logic_vector(15 downto 0);
        vccbram_raw : out std_logic_vector(15 downto 0);

        -- One-cycle strobes: (6) vccbram (5) vccaux (4) vccint (3) temp
        --                    (2) vaux9   (1) vaux0  (0) vpvn
        updated     : out std_logic_vector(6 downto 0);
        eos_pulse   : out std_logic;                     -- end of a sequence pass
        busy        : out std_logic
    );
end entity xadc_reader;

architecture rtl of xadc_reader is

    function to_bv16(x : natural) return bit_vector is
    begin
        return to_bitvector(std_logic_vector(to_unsigned(x, 16)));
    end function;

    function sel(c : boolean; a, b : natural) return natural is
    begin
        if c then return a; else return b; end if;
    end function;

    function clk_div_calc(f : positive) return natural is
        constant ADCCLK_MAX_HZ : positive := 26_000_000;
        variable d : natural;
    begin
        d := (f + ADCCLK_MAX_HZ - 1) / ADCCLK_MAX_HZ;
        if d < 2 then d := 2; end if;
        assert d <= 255 report "xadc_reader: DCLK too fast for ADCCLK divider" severity failure;
        return d;
    end function;

    constant C_CLK_DIV : natural := clk_div_calc(G_CLK_FREQ_HZ);

    -- Configuration registers (UG480, chapter 3)
    -- 40h: AVG in [13:12]; CAVG = 0 averages calibration coefficients
    constant C_CFG0       : natural := G_AVG_SEL * 16#1000#;
    -- 41h: SEQ = 0010 continuous; alarms and OT disabled;
    --      offset+gain calibration for ADC and supply sensor
    constant C_CFG1       : natural := 16#2FAF#;
    -- 42h: ADCCLK divider in [15:8]; both ADCs powered
    constant C_CFG2       : natural := C_CLK_DIV * 16#100#;
    -- 48h: cal(0) | temp(8) | vccint(9) | vccaux(10) | vp/vn(11) | vccbram(14)
    constant C_SEQ_CHSEL0 : natural := 16#4F01#;
    -- 49h: VAUX0 (bit 0) | VAUX9 (bit 9)
    constant C_SEQ_CHSEL1 : natural := 16#0201#;
    -- 4Ah/4Bh: averaging on the same channels
    constant C_SEQ_AVG0   : natural := sel(G_AVG_SEL /= 0, 16#4F00#, 0);
    constant C_SEQ_AVG1   : natural := sel(G_AVG_SEL /= 0, 16#0201#, 0);
    -- 4Ch/4Dh: bipolar mode for VP/VN, VAUX0, VAUX9
    constant C_SEQ_BIP0   : natural := sel(G_BIPOLAR, 16#0800#, 0);
    constant C_SEQ_BIP1   : natural := sel(G_BIPOLAR, 16#0201#, 0);

    -- Status register / CHANNEL codes
    constant CH_TEMP    : std_logic_vector(4 downto 0) := "00000";
    constant CH_VCCINT  : std_logic_vector(4 downto 0) := "00001";
    constant CH_VCCAUX  : std_logic_vector(4 downto 0) := "00010";
    constant CH_VPVN    : std_logic_vector(4 downto 0) := "00011";
    constant CH_VCCBRAM : std_logic_vector(4 downto 0) := "00110";
    constant CH_VAUX0   : std_logic_vector(4 downto 0) := "10000";
    constant CH_VAUX9   : std_logic_vector(4 downto 0) := "11001";

    signal drp_do     : std_logic_vector(15 downto 0);
    signal drp_rdy    : std_logic;
    signal drp_addr   : std_logic_vector(6 downto 0);
    signal channel    : std_logic_vector(4 downto 0);
    signal eoc        : std_logic;
    signal eos        : std_logic;
    signal vauxp      : std_logic_vector(15 downto 0);
    signal vauxn      : std_logic_vector(15 downto 0);
    signal rd_channel : std_logic_vector(4 downto 0);

begin

    vauxp <= (0 => vauxp0, 9 => vauxp9, others => '0');
    vauxn <= (0 => vauxn0, 9 => vauxn9, others => '0');

    -- EOC is a single DCLK pulse with CHANNEL valid alongside it, so it drives
    -- the DRP read directly. A conversion lasts 26 ADCCLK cycles, far longer
    -- than a DRP read, so no read is outstanding when the next EOC arrives.
    drp_addr <= "00" & channel;

    u_xadc : XADC
        generic map (
            INIT_40          => to_bv16(C_CFG0),
            INIT_41          => to_bv16(C_CFG1),
            INIT_42          => to_bv16(C_CFG2),
            INIT_48          => to_bv16(C_SEQ_CHSEL0),
            INIT_49          => to_bv16(C_SEQ_CHSEL1),
            INIT_4A          => to_bv16(C_SEQ_AVG0),
            INIT_4B          => to_bv16(C_SEQ_AVG1),
            INIT_4C          => to_bv16(C_SEQ_BIP0),
            INIT_4D          => to_bv16(C_SEQ_BIP1),
            INIT_4E          => X"0000",
            INIT_4F          => X"0000",
            SIM_DEVICE       => "7SERIES",
            SIM_MONITOR_FILE => G_SIM_MONITOR_FILE
        )
        port map (
            DCLK         => clk,
            RESET        => rst,
            DEN          => eoc,
            DADDR        => drp_addr,
            DWE          => '0',
            DI           => (others => '0'),
            DO           => drp_do,
            DRDY         => drp_rdy,
            CONVST       => '0',
            CONVSTCLK    => '0',
            VP           => vp_in,
            VN           => vn_in,
            VAUXP        => vauxp,
            VAUXN        => vauxn,
            CHANNEL      => channel,
            EOC          => eoc,
            EOS          => eos,
            BUSY         => busy,
            ALM          => open,
            OT           => open,
            MUXADDR      => open,
            JTAGBUSY     => open,
            JTAGLOCKED   => open,
            JTAGMODIFIED => open
        );

    p_capture : process (clk)
    begin
        if rising_edge(clk) then
            if rst = '1' then
                rd_channel  <= (others => '0');
                updated     <= (others => '0');
                eos_pulse   <= '0';
                vpvn_raw    <= (others => '0');
                vaux0_raw   <= (others => '0');
                vaux9_raw   <= (others => '0');
                temp_raw    <= (others => '0');
                vccint_raw  <= (others => '0');
                vccaux_raw  <= (others => '0');
                vccbram_raw <= (others => '0');
            else
                updated   <= (others => '0');
                eos_pulse <= eos;

                if eoc = '1' then
                    rd_channel <= channel;
                end if;

                if drp_rdy = '1' then
                    case rd_channel is
                        when CH_VPVN    => vpvn_raw    <= drp_do; updated(0) <= '1';
                        when CH_VAUX0   => vaux0_raw   <= drp_do; updated(1) <= '1';
                        when CH_VAUX9   => vaux9_raw   <= drp_do; updated(2) <= '1';
                        when CH_TEMP    => temp_raw    <= drp_do; updated(3) <= '1';
                        when CH_VCCINT  => vccint_raw  <= drp_do; updated(4) <= '1';
                        when CH_VCCAUX  => vccaux_raw  <= drp_do; updated(5) <= '1';
                        when CH_VCCBRAM => vccbram_raw <= drp_do; updated(6) <= '1';
                        when others     => null;   -- calibration (0x08)
                    end case;
                end if;
            end if;
        end if;
    end process;

end architecture rtl;












-------------------------------------------------------------------------------
--! @file xadc_reader_wb.vhd
--! @brief XADC reader with Wishbone register interface (ALINX AX7103)
--! @details Wraps xadc_reader, which runs the XADC sequencer continuously
--! over VP/VN, VAUX0, VAUX9, temperature, VCCINT, VCCAUX and VCCBRAM. Every
--! reading is available on its own 16-bit output port and as a read-only
--! Wishbone register.
--!
--! Register map (word addresses):
--!   0  REG_ID         ID_REV
--!   1  REG_STATUS     bit 0 XADC BUSY
--!   2  REG_SEQ_COUNT  completed sequencer passes (wraps)
--!   3  REG_VPVN       header pins 1/2
--!   4  REG_VAUX0      header pins 9/10
--!   5  REG_VAUX9      header pins 5/6
--!   6  REG_TEMP       die temperature
--!   7  REG_VCCINT
--!   8  REG_VCCAUX
--!   9  REG_VCCBRAM
--! Values are raw XADC status registers: 12-bit code in bits 15:4.
--! Writes are acked and ignored. Unmapped reads are acked and return 0x0000.
--! Single-cycle-ack pattern as iq_dma_regs.
-------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.xadc_wb_pkg.all;

entity xadc_reader_wb is
  generic(
    --! CLK_I frequency in Hz, sets the ADCCLK divider (ADCCLK <= 26 MHz)
    G_CLK_FREQ_HZ      : positive             := 80_000_000;
    --! XADC averaging: 0 off, 1 = 16, 2 = 64, 3 = 256 samples
    G_AVG_SEL          : natural range 0 to 3 := 1;
    --! analogue stimulus file, simulation only
    G_SIM_MONITOR_FILE : string               := "design.txt";
    --! '1' for active low reset, '0' for active high
    INVERT_RESET       : std_logic            := '0'
  );
  port(
    --! Clock input, also the XADC DRP clock
    CLK_I         : in  std_logic;
    --! Synchronous reset
    RST_I         : in  std_logic;

    --! Wishbone address block input
    WB_ADR_I      : in  std_logic_vector(WB_BLOCK_ADR_WIDTH - 1 downto 0);
    --! Wishbone write data bus input
    WB_DAT_I      : in  std_logic_vector(WB_DAT_WIDTH - 1 downto 0);
    --! Wishbone read data bus output
    WB_DAT_O      : out std_logic_vector(WB_DAT_WIDTH - 1 downto 0);
    --! Wishbone strobe signal input
    WB_STB_I      : in  std_logic;
    --! Wishbone cycle signal input
    WB_CYC_I      : in  std_logic;
    --! Wishbone write enable signal input
    WB_WE_I       : in  std_logic;
    --! Wishbone acknowledge signal output
    WB_ACK_O      : out std_logic;

    --! XADC header pins 1/2, dedicated VP/VN (connect to top-level ports)
    VP_I          : in  std_logic;
    VN_I          : in  std_logic;
    --! XADC header pins 9/10, VAUX0
    VAUXP0_I      : in  std_logic;
    VAUXN0_I      : in  std_logic;
    --! XADC header pins 5/6, VAUX9
    VAUXP9_I      : in  std_logic;
    VAUXN9_I      : in  std_logic;

    --! VP/VN reading
    VPVN_O        : out std_logic_vector(15 downto 0);
    --! VAUX0 reading
    VAUX0_O       : out std_logic_vector(15 downto 0);
    --! VAUX9 reading
    VAUX9_O       : out std_logic_vector(15 downto 0);
    --! die temperature reading
    TEMP_O        : out std_logic_vector(15 downto 0);
    --! VCCINT reading
    VCCINT_O      : out std_logic_vector(15 downto 0);
    --! VCCAUX reading
    VCCAUX_O      : out std_logic_vector(15 downto 0);
    --! VCCBRAM reading
    VCCBRAM_O     : out std_logic_vector(15 downto 0);
    --! single-cycle update strobes: 6 vccbram, 5 vccaux, 4 vccint, 3 temp,
    --! 2 vaux9, 1 vaux0, 0 vpvn
    UPDATED_O     : out std_logic_vector(6 downto 0)
  );
end entity;

-------------------------------------------------------------------------------
--! @brief See entity declaration
-------------------------------------------------------------------------------
architecture behavioral of xadc_reader_wb is
  --! block ID, distinct from 0xDEAD and from the IDs of other blocks
  constant ID_REV            : std_logic_vector(WB_DAT_WIDTH - 1 downto 0) := x"AD01";
  --! register offsets
  constant REG_ID            : natural := 0;
  constant REG_STATUS        : natural := 1;
  constant REG_SEQ_COUNT     : natural := 2;
  constant REG_VPVN          : natural := 3;
  constant REG_VAUX0         : natural := 4;
  constant REG_VAUX9         : natural := 5;
  constant REG_TEMP          : natural := 6;
  constant REG_VCCINT        : natural := 7;
  constant REG_VCCAUX        : natural := 8;
  constant REG_VCCBRAM       : natural := 9;

  --! active-high reset for the core
  signal rst       : std_logic;
  --! readings from the core
  signal vpvn      : std_logic_vector(15 downto 0);
  signal vaux0     : std_logic_vector(15 downto 0);
  signal vaux9     : std_logic_vector(15 downto 0);
  signal temp      : std_logic_vector(15 downto 0);
  signal vccint    : std_logic_vector(15 downto 0);
  signal vccaux    : std_logic_vector(15 downto 0);
  signal vccbram   : std_logic_vector(15 downto 0);
  --! end of sequence pulse and XADC busy
  signal eos_pulse : std_logic;
  signal busy      : std_logic;
  --! completed sequencer passes
  signal seq_count : unsigned(15 downto 0);

begin

  rst <= RST_I xor INVERT_RESET;

  VPVN_O    <= vpvn;
  VAUX0_O   <= vaux0;
  VAUX9_O   <= vaux9;
  TEMP_O    <= temp;
  VCCINT_O  <= vccint;
  VCCAUX_O  <= vccaux;
  VCCBRAM_O <= vccbram;

  -----------------------------------------------------------------------------
  --! @brief XADC sequencer and DRP capture
  -----------------------------------------------------------------------------
  core : entity work.xadc_reader
    generic map(
      G_CLK_FREQ_HZ      => G_CLK_FREQ_HZ,
      G_AVG_SEL          => G_AVG_SEL,
      G_BIPOLAR          => false,
      G_SIM_MONITOR_FILE => G_SIM_MONITOR_FILE
    )
    port map(
      clk         => CLK_I,
      rst         => rst,
      vp_in       => VP_I,
      vn_in       => VN_I,
      vauxp0      => VAUXP0_I,
      vauxn0      => VAUXN0_I,
      vauxp9      => VAUXP9_I,
      vauxn9      => VAUXN9_I,
      vpvn_raw    => vpvn,
      vaux0_raw   => vaux0,
      vaux9_raw   => vaux9,
      temp_raw    => temp,
      vccint_raw  => vccint,
      vccaux_raw  => vccaux,
      vccbram_raw => vccbram,
      updated     => UPDATED_O,
      eos_pulse   => eos_pulse,
      busy        => busy
    );

  -----------------------------------------------------------------------------
  --! @brief Sequencer pass counter
  -----------------------------------------------------------------------------
  seq_counter : process(CLK_I)
  begin
    if rising_edge(CLK_I) then
      if (rst = '1') then
        seq_count <= (others => '0');
      elsif (eos_pulse = '1') then
        seq_count <= seq_count + 1;
      end if;
    end if;
  end process;

  -----------------------------------------------------------------------------
  --! @brief Wishbone register interface
  -----------------------------------------------------------------------------
  wb_registers : process(CLK_I)
  begin
    if rising_edge(CLK_I) then
      -- clear outputs so that they only remain active for minimum time
      WB_ACK_O <= '0';
      WB_DAT_O <= (others => '0');

      if (RST_I = not (INVERT_RESET)) then
        null;
      elsif ((WB_CYC_I and WB_STB_I) = '1') then
        WB_ACK_O <= '1';

        if (WB_WE_I = '0') then -- read transaction, writes are acked and ignored
          case to_integer(unsigned(WB_ADR_I)) is
            when REG_ID        => WB_DAT_O <= ID_REV;
            when REG_STATUS    => WB_DAT_O <= (0 => busy, others => '0');
            when REG_SEQ_COUNT => WB_DAT_O <= std_logic_vector(resize(seq_count, WB_DAT_WIDTH));
            when REG_VPVN      => WB_DAT_O <= std_logic_vector(resize(unsigned(vpvn), WB_DAT_WIDTH));
            when REG_VAUX0     => WB_DAT_O <= std_logic_vector(resize(unsigned(vaux0), WB_DAT_WIDTH));
            when REG_VAUX9     => WB_DAT_O <= std_logic_vector(resize(unsigned(vaux9), WB_DAT_WIDTH));
            when REG_TEMP      => WB_DAT_O <= std_logic_vector(resize(unsigned(temp), WB_DAT_WIDTH));
            when REG_VCCINT    => WB_DAT_O <= std_logic_vector(resize(unsigned(vccint), WB_DAT_WIDTH));
            when REG_VCCAUX    => WB_DAT_O <= std_logic_vector(resize(unsigned(vccaux), WB_DAT_WIDTH));
            when REG_VCCBRAM   => WB_DAT_O <= std_logic_vector(resize(unsigned(vccbram), WB_DAT_WIDTH));
            when others        => null; -- acked, reads 0x0000
          end case;
        end if;
      end if;
    end if;
  end process;

end architecture;





























