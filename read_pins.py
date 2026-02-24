import spidev
import struct
import time

spi = spidev.SpiDev()
spi.open(0, 0)
spi.max_speed_hz = 1000000

print("--- Scanning A0, A1, A2 ---")
try:
    while True:
        # Request 8 bytes (4 integers)
        raw = spi.xfer2([0x00] * 8)
        
        # Unpack 4 unsigned shorts (Little Endian)
        # vals[0]=A0, vals[1]=A1, vals[2]=A2, vals[3]=Magic
        vals = struct.unpack('<HHHH', bytes(raw))
        
        if vals[3] == 0xABCD:
            print(f"A0: {vals[0]:4} | A1: {vals[1]:4} | A2: {vals[2]:4}", end='\r')
        else:
            print(f"Sync Error: Received {hex(vals[3])} instead of 0xabcd", end='\r')
            
        time.sleep(0.1)
except KeyboardInterrupt:
    spi.close()
