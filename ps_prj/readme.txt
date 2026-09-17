# Una sola lettura manuale
sudo python3 -c "
import os, mmap
fd = os.open('/sys/bus/pci/devices/0000:01:00.0/resource0', os.O_RDWR|os.O_SYNC)
bar = mmap.mmap(fd, 65536, mmap.MAP_SHARED, mmap.PROT_READ|mmap.PROT_WRITE)
bar.seek(0x0800)
print(hex(int.from_bytes(bar.read(2), 'little')))
bar.close()
os.close(fd)
"
