sudo python3 -c "
import os, mmap
fd = os.open('/sys/bus/pci/devices/0000:01:00.0/resource0', os.O_RDWR|os.O_SYNC)
bar = mmap.mmap(fd, 65536, mmap.MAP_SHARED, mmap.PROT_READ|mmap.PROT_WRITE)
for off in [0x0814, 0x0816, 0x0818, 0x081A]:
    bar.seek(off)
    val = int.from_bytes(bar.read(2), 'little')
    print(f'0x{off:04X} = 0x{val:04X}')
bar.close(); os.close(fd)
"
