import spidev
import csv
import struct
import time
import sys

# SPI Configuration
spi = spidev.SpiDev()
spi.open(0, 0)
spi.max_speed_hz = 1000000
spi.mode = 0b01

# Constants
MAGIC_MARKER = 0xABCD
CSV_FILENAME = "raw_adc_a0_sine.csv"

def run_capture():
    # 1. Selection Menu
    print("--- RAW ADC CAPTURE (A0) ---")
    try:
        duration = int(input("Enter duration (5, 10, or 15 seconds): "))
        if duration not in [5, 10, 15]:
            print("Invalid choice. Defaulting to 5 seconds.")
            duration = 5
    except ValueError:
        print("Invalid input. Defaulting to 5 seconds.")
        duration = 5

    print(f"Starting capture for {duration} seconds...")
    
    samples_collected = 0
    start_time = time.time()
    end_time = start_time + duration

    # 2. Open CSV and Start Reading
    with open(CSV_FILENAME, mode='w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(["ADC_Value"]) # Simple header

        try:
            while time.time() < end_time:
                # Request 8 bytes (matching your Arduino buffer size)
                raw = spi.xfer2([0x00] * 8)
                
                # vals[0]=A0, vals[1]=A1, vals[2]=A2, vals[3]=Magic
                vals = struct.unpack('<HHHH', bytes(raw))
                
                # Check Magic Marker
                if vals[3] == MAGIC_MARKER:
                    writer.writerow([vals[0]]) # Write only A0
                    samples_collected += 1
                
                # Update status in terminal without spamming lines
                elapsed = time.time() - start_time
                print(f"Time: {elapsed:.1f}/{duration}s | Samples: {samples_collected}", end='\r')

        except KeyboardInterrupt:
            print("\nManual stop detected.")

    # 3. Final Report
    actual_duration = time.time() - start_time
    print(f"\n\n--- Capture Complete ---")
    print(f"File Saved: {CSV_FILENAME}")
    print(f"Total Samples: {samples_collected}")
    print(f"Actual Duration: {actual_duration:.2f} seconds")
    print(f"Avg Sample Rate: {samples_collected/actual_duration:.1f} Hz")

if __name__ == "__main__":
    run_capture()
    spi.close()
