#!/bin/bash
# flash_ax7103.sh
#
# Programs the N25Q128 QSPI flash on the ALINX AX7103.
# Finds the .bit file in the same directory as this script.
#
# Usage:
#   source /opt/Xilinx/Vivado/2023.1/settings64.sh
#   chmod +x flash_ax7103.sh && ./flash_ax7103.sh

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
FLASH_PART="mt25ql128-spi-x1_x2_x4"
TCL_GEN="$SCRIPT_DIR/gen_mcs.tcl"
TCL_PROG="$SCRIPT_DIR/program_flash.tcl"

# --- Find .bit file ---
BIT_FILES=("$SCRIPT_DIR"/*.bit)
if [[ ${#BIT_FILES[@]} -eq 0 || ! -f "${BIT_FILES[0]}" ]]; then
    echo "ERROR: No .bit file found in $SCRIPT_DIR"; exit 1
elif [[ ${#BIT_FILES[@]} -gt 1 ]]; then
    echo "ERROR: Multiple .bit files found -- leave only one:"
    printf '  %s\n' "${BIT_FILES[@]}"; exit 1
fi

BIT_FILE="$(readlink -f "${BIT_FILES[0]}")"
BIT_NAME="$(basename "${BIT_FILE%.bit}")"
MCS_FILE="$SCRIPT_DIR/${BIT_NAME}.mcs"
PRM_FILE="$SCRIPT_DIR/${BIT_NAME}.prm"

echo "Bitstream : $BIT_FILE"
echo "MCS       : $MCS_FILE"
echo ""

if [[ ! -f "$BIT_FILE" || ! -s "$BIT_FILE" ]]; then
    echo "ERROR: .bit file is empty or missing: $BIT_FILE"; exit 1
fi
echo "Bitstream size: $(du -h "$BIT_FILE" | cut -f1)"
echo ""

if ! command -v vivado &>/dev/null; then
    echo "ERROR: vivado not found. Source settings64.sh first."; exit 1
fi

# --- Write gen_mcs.tcl ---
# Use printf to write the Tcl file so bash does not interpret
# any special characters inside the Tcl content
printf 'set bit_file "%s"\n' "$BIT_FILE"           > "$TCL_GEN"
printf 'set mcs_file "%s"\n' "$MCS_FILE"           >> "$TCL_GEN"
printf 'puts "Bitstream: $bit_file"\n'             >> "$TCL_GEN"
printf 'puts "MCS      : $mcs_file"\n'             >> "$TCL_GEN"
printf 'if {![file exists $bit_file]} {\n'         >> "$TCL_GEN"
printf '    error "bit file not found: $bit_file"\n' >> "$TCL_GEN"
printf '}\n'                                        >> "$TCL_GEN"
printf 'write_cfgmem -format mcs -interface spix4 -size 16 -loadbit "up 0x0 $bit_file" -file $mcs_file -force\n' >> "$TCL_GEN"
printf 'puts "write_cfgmem done."\n'               >> "$TCL_GEN"

echo "--- gen_mcs.tcl ---"
cat "$TCL_GEN"
echo "-------------------"
echo ""

# --- Write program_flash.tcl ---
printf 'set bit_file   "%s"\n'  "$BIT_FILE"        > "$TCL_PROG"
printf 'set mcs_file   "%s"\n'  "$MCS_FILE"        >> "$TCL_PROG"
printf 'set prm_file   "%s"\n'  "$PRM_FILE"        >> "$TCL_PROG"
printf 'set flash_part "%s"\n'  "$FLASH_PART"      >> "$TCL_PROG"
cat >> "$TCL_PROG" << 'TCLEOF'
open_hw_manager
connect_hw_server -allow_non_jtag
open_hw_target
set dev [lindex [get_hw_devices] 0]
current_hw_device $dev
puts "Device: $dev"
puts "Programming FPGA..."
set_property PROGRAM.FILE $bit_file $dev
program_hw_devices $dev
refresh_hw_device $dev
puts "FPGA programmed OK."
set cfgmem [get_property PROGRAM.HW_CFGMEM $dev]
if {$cfgmem eq ""} {
    puts "Creating cfgmem..."
    create_hw_cfgmem -hw_device $dev -mem_dev [lindex [get_cfgmem_parts $flash_part] 0]
    set cfgmem [get_property PROGRAM.HW_CFGMEM $dev]
} else {
    puts "Using existing cfgmem: $cfgmem"
}
set_property PROGRAM.FILES       [list $mcs_file] $cfgmem
set_property PROGRAM.PRM_FILES   [list $prm_file] $cfgmem
set_property PROGRAM.ERASE       1                $cfgmem
set_property PROGRAM.CFG_PROGRAM 1                $cfgmem
set_property PROGRAM.VERIFY      1                $cfgmem
set_property PROGRAM.CHECKSUM    0                $cfgmem
puts "Programming flash (~2 min)..."
program_hw_cfgmem -hw_cfgmem $cfgmem
puts "Flash programming complete."
close_hw_target
disconnect_hw_server
close_hw_manager
TCLEOF

# --- Step 1: Generate MCS ---
echo "Step 1: Generating MCS..."
vivado -mode batch -nojournal -nolog -source "$TCL_GEN"

if [[ ! -f "$MCS_FILE" ]]; then
    echo "ERROR: MCS not generated."; exit 1
fi
echo "MCS OK: $MCS_FILE ($(du -h "$MCS_FILE" | cut -f1))"
echo ""

# --- Step 2: Program ---
echo "Step 2: Programming FPGA and flash..."
vivado -mode batch -nojournal -nolog -source "$TCL_PROG"

rm -f "$TCL_GEN" "$TCL_PROG"
echo ""
echo "Done -- power cycle the board to boot from flash."
