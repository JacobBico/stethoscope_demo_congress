import spidev
import sys
import os

# SPI Configuration
spi = spidev.SpiDev()
spi.open(0, 0)
spi.max_speed_hz = 4000000
spi.mode = 0b01  # CPHA=1 to match Arduino

BATCH_SIZE = 1024
# Magic (2 bytes) + Audio (2048 bytes)
TOTAL_BYTES = (BATCH_SIZE + 1) * 2
MAGIC = b'\x5a\xa5'

def run_stream():
    stdout_ptr = sys.stdout.buffer
    
    while True:
        try:
            # Request the full batch
            raw = spi.xfer2([0x00] * TOTAL_BYTES)
            data = bytes(raw)

            # Check for Magic Sync Marker at index 0
            if data[0:2] == MAGIC:
                # Valid data! Write only the audio payload
                stdout_ptr.write(data[2:])
            else:
                # We are desynced. Search for the marker in the chunk
                # to see if we can recover alignment
                offset = data.find(MAGIC)
                if offset != -1 and (len(data) - offset) >= (BATCH_SIZE * 2):
                    # Found it late, grab what we can
                    stdout_ptr.write(data[offset+2 : offset+2+(BATCH_SIZE*2)])
            
            stdout_ptr.flush()

        except Exception as e:
            continue

if __name__ == "__main__":
    try:
        run_stream()
    except KeyboardInterrupt:
        spi.close()
        sys.exit(0)
