#!/bin/bash
# flash_ax7103.sh
#
# Programs the N25Q128 QSPI flash on the ALINX AX7103.
# Finds the .bit file in the same directory as this script,
# generates the MCS, then launches Vivado in batch mode to program flash.
#
# Usage:
#   chmod +x flash_ax7103.sh
#   ./flash_ax7103.sh

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
FLASH_PART="mt25ql128-spi-x1_x2_x4"

# --- Find .bit file ---
BIT_FILES=("$SCRIPT_DIR"/*.bit)
if [[ ${#BIT_FILES[@]} -eq 0 || ! -f "${BIT_FILES[0]}" ]]; then
    echo "ERROR: No .bit file found in $SCRIPT_DIR"
    exit 1
elif [[ ${#BIT_FILES[@]} -gt 1 ]]; then
    echo "ERROR: Multiple .bit files found -- leave only one:"
    printf '  %s\n' "${BIT_FILES[@]}"
    exit 1
fi

BIT_FILE="${BIT_FILES[0]}"
BIT_NAME="$(basename "${BIT_FILE%.bit}")"
MCS_FILE="$SCRIPT_DIR/${BIT_NAME}.mcs"
PRM_FILE="$SCRIPT_DIR/${BIT_NAME}.prm"

echo "Bitstream : $BIT_FILE"
echo "MCS       : $MCS_FILE"
echo "Flash     : $FLASH_PART"
echo ""

# --- Check Vivado is on PATH ---
if ! command -v vivado &>/dev/null; then
    echo "ERROR: vivado not found on PATH."
    echo "Source your Vivado settings first:"
    echo "  source /opt/Xilinx/Vivado/2023.1/settings64.sh"
    exit 1
fi

# --- Launch Vivado batch mode with inline Tcl ---
vivado -mode batch -nojournal -nolog -source /dev/stdin << TCLEOF

# --- Generate MCS ---
write_cfgmem \
    -format    mcs   \
    -interface spix4 \
    -size      16    \
    -loadbit   "up 0x0 {$BIT_FILE}" \
    -file      {$MCS_FILE} \
    -force

# --- Open hardware ---
open_hw_manager
connect_hw_server -allow_non_jtag
open_hw_target

set dev [lindex [get_hw_devices] 0]
current_hw_device \$dev
puts "Device: \$dev"

# --- Program FPGA first ---
puts "Programming FPGA..."
set_property PROGRAM.FILE {$BIT_FILE} \$dev
program_hw_devices \$dev
refresh_hw_device \$dev
puts "FPGA programmed OK."

# --- Use the cfgmem Vivado auto-created after program_hw_devices ---
set cfgmem [get_property PROGRAM.HW_CFGMEM \$dev]
if {\$cfgmem eq ""} {
    puts "Creating cfgmem..."
    create_hw_cfgmem \
        -hw_device \$dev \
        -mem_dev   [lindex [get_cfgmem_parts {$FLASH_PART}] 0]
    set cfgmem [get_property PROGRAM.HW_CFGMEM \$dev]
} else {
    puts "Using existing cfgmem: \$cfgmem"
}

set_property PROGRAM.FILES       { {$MCS_FILE} } \$cfgmem
set_property PROGRAM.PRM_FILES   { {$PRM_FILE} } \$cfgmem
set_property PROGRAM.ERASE       1               \$cfgmem
set_property PROGRAM.CFG_PROGRAM 1               \$cfgmem
set_property PROGRAM.VERIFY      1               \$cfgmem
set_property PROGRAM.CHECKSUM    0               \$cfgmem

puts "Programming flash (~2 min)..."
program_hw_cfgmem -hw_cfgmem \$cfgmem

puts "Flash programming complete."
close_hw_target
disconnect_hw_server
close_hw_manager

TCLEOF

echo ""
echo "Done -- power cycle the board to boot from flash."
