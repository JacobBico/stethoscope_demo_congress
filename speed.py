import spidev
import csv
import struct
import time

spi = spidev.SpiDev()
spi.open(0, 0)
spi.max_speed_hz = 8000000
spi.mode = 0b01

BATCH_SIZE = 512
TOTAL_BYTES = (BATCH_SIZE + 1) * 2

def run():
    print("--- DOUBLE-BUFFERED CAPTURE ---")
    duration = 10 
    all_data = []
    
    start = time.time()
    end = start + duration
    
    while time.time() < end:
        # Request a full batch
        raw = spi.xfer2([0x00] * TOTAL_BYTES)
        data = struct.unpack('<' + 'H' * (BATCH_SIZE + 1), bytes(raw))
        
        if data[0] == 0xABCD:
            all_data.extend(data[1:])
        
        # Micro-delay to let Arduino fill its next buffer
        # Adjust this if rate is still too low
        time.sleep(0.001) 

    actual_time = time.time() - start
    print(f"Captured {len(all_data)} samples in {actual_time:.2f}s")
    print(f"Sample Rate: {len(all_data)/actual_time:.1f} Hz")

    with open("clean_data.csv", "w", newline='') as f:
        writer = csv.writer(f)
        writer.writerow(["ADC_Value"])
        for val in all_data:
            writer.writerow([val])

if __name__ == "__main__":
    run()
