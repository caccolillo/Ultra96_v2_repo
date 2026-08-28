# flash_ax7103.tcl
#
# Programs the N25Q128 QSPI flash on the ALINX AX7103 (AC7100B core board).
# Automatically finds the .bit file in the same directory as this script.
#
# Usage -- from Vivado Tcl console:
#   source /path/to/flash_ax7103.tcl
#
# Or batch mode:
#   vivado -mode batch -source /path/to/flash_ax7103.tcl

set script_dir [file dirname [file normalize [info script]]]
set flash_part "n25q128-3.3v-spi-x1_x2_x4"

# --- Find .bit file ---
set bit_files [glob -nocomplain -directory $script_dir *.bit]

if {[llength $bit_files] == 0} {
    error "No .bit file found in $script_dir"
} elseif {[llength $bit_files] > 1} {
    error "Multiple .bit files found in $script_dir -- leave only one:\n[join $bit_files \n]"
}

set bit_file [lindex $bit_files 0]
set mcs_file [file join $script_dir [file rootname [file tail $bit_file]].mcs]
set prm_file [file join $script_dir [file rootname [file tail $bit_file]].prm]

puts "Bitstream : $bit_file"
puts "MCS       : $mcs_file"

# --- Step 1: Generate MCS ---
write_cfgmem \
    -format     mcs     \
    -interface  spix4   \
    -size       16      \
    -loadbit    "up 0x0 $bit_file" \
    -file       $mcs_file \
    -force

# --- Step 2: Program flash ---
open_hw_manager
connect_hw_server -allow_non_jtag
open_hw_target

set dev [lindex [get_hw_devices] 0]
create_hw_cfgmem -hw_device $dev -mem_dev [lindex [get_cfgmem_parts $flash_part] 0]
set cfgmem [get_property PROGRAM.HW_CFGMEM $dev]

set_property PROGRAM.FILES       [list $mcs_file] $cfgmem
set_property PROGRAM.PRM_FILES   [list $prm_file] $cfgmem
set_property PROGRAM.ERASE       1                $cfgmem
set_property PROGRAM.CFG_PROGRAM 1                $cfgmem
set_property PROGRAM.VERIFY      1                $cfgmem
set_property PROGRAM.CHECKSUM    0                $cfgmem

puts "Programming flash (~2 min)..."
program_hw_cfgmem -hw_cfgmem $cfgmem

puts "Done -- power cycle the board to boot from flash."

close_hw_target
disconnect_hw_server
close_hw_manager
