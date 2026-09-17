#!/bin/bash
# pci_reset.sh — safe PCIe device reset and re-enable

DEV="0000:01:00.0"

echo "  [pci_reset] removing $DEV ..."
echo 1 | tee /sys/bus/pci/devices/$DEV/remove 2>/dev/null || true

sleep 0.5

echo "  [pci_reset] rescanning bus ..."
echo 1 | tee /sys/bus/pci/rescan

sleep 0.5

echo "  [pci_reset] enabling BARs ..."
setpci -s $DEV COMMAND=0x02
echo 1 | tee /sys/bus/pci/devices/$DEV/enable 2>/dev/null || true

sleep 0.2

echo "  [pci_reset] done."
lspci -s $DEV
