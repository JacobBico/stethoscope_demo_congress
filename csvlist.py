import spidev
import csv
import struct
import sys

# SPI Setup
spi = spidev.SpiDev()
spi.open(0, 0)
spi.max_speed_hz = 4000000
spi.mode = 0b01 

BATCH_SIZE = 1024
TOTAL_BYTES = (BATCH_SIZE + 1) * 2
MAGIC = b'\x5a\xa5'
CSV_FILENAME = "raw_data.csv"

def run():
    print(f"Recording raw ADC to {CSV_FILENAME}. Press Ctrl+C to stop.")
    
    with open(CSV_FILENAME, mode='w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(["ADC_Value"])
        
        try:
            while True:
                # Get raw bytes from SPI
                raw_bytes = bytes(spi.xfer2([0x00] * TOTAL_BYTES))

                # Check for sync
                if raw_bytes[0:2] == MAGIC:
                    # Unpack 1024 signed shorts (int16)
                    samples = struct.unpack(f'<{BATCH_SIZE}h', raw_bytes[2:])
                    for s in samples:
                        writer.writerow([s])
                else:
                    # Search for magic marker if desynced
                    offset = raw_bytes.find(MAGIC)
                    if offset != -1 and (len(raw_bytes) - offset) >= (BATCH_SIZE * 2):
                        samples = struct.unpack(f'<{BATCH_SIZE}h', raw_bytes[offset+2 : offset+2+(BATCH_SIZE*2)])
                        for s in samples:
                            writer.writerow([s])
        except KeyboardInterrupt:
            print(f"\nSaved to {CSV_FILENAME}")
        finally:
            spi.close()

if __name__ == "__main__":
    run()
