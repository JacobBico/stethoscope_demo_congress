import spidev
import csv
import struct
import sys

# SPI Configuration
spi = spidev.SpiDev()
spi.open(0, 0)
spi.max_speed_hz = 4000000
spi.mode = 0b01  # CPHA=1 to match Arduino

BATCH_SIZE = 1024
# Magic (2 bytes) + Audio (2048 bytes)
TOTAL_BYTES = (BATCH_SIZE + 1) * 2
MAGIC = b'\x5a\xa5'

CSV_FILENAME = "readings.csv"

def run_to_csv():
    print(f"Starting SPI capture. Saving to {CSV_FILENAME}...")
    print("Press Ctrl+C to stop.")

    with open(CSV_FILENAME, mode='w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(["sample_index", "value"]) # Header
        
        sample_count = 0
        
        try:
            while True:
                # Request the full batch
                raw = spi.xfer2([0x00] * TOTAL_BYTES)
                data = bytes(raw)

                payload = None
                
                # Check for Magic Sync Marker
                if data[0:2] == MAGIC:
                    payload = data[2:]
                else:
                    offset = data.find(MAGIC)
                    if offset != -1 and (len(data) - offset) >= (BATCH_SIZE * 2):
                        payload = data[offset+2 : offset+2+(BATCH_SIZE * 2)]

                if payload:
                    # 'h' is for signed short (int16), '<' is little-endian
                    # We expect BATCH_SIZE number of shorts
                    samples = struct.unpack(f'<{BATCH_SIZE}h', payload)
                    
                    # Write to CSV
                    for s in samples:
                        writer.writerow([sample_count, s])
                        sample_count += 1
                    
                    # Optional: Print progress every 10k samples
                    if sample_count % 10240 == 0:
                        print(f"Captured {sample_count} samples...", end='\r')

        except KeyboardInterrupt:
            print(f"\nStopped. Total samples saved: {sample_count}")
        finally:
            spi.close()

if __name__ == "__main__":
    run_to_csv()
