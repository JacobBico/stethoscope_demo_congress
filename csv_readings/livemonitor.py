import serial
import struct

PORT = "/dev/ttyHS1"
BAUD = 2000000
MAGIC_B = bytes([0x5A, 0xA5]) # 0xA55A little-endian

def run_raw_monitor():
    print(f"--- Raw ADC Count Monitor (A0) ---")
    ser = serial.Serial(PORT, BAUD, timeout=1.0)
    
    try:
        while True:
            # Sync to Magic Bytes
            if ser.read(1) == b'\x5A':
                if ser.read(1) == b'\xA5':
                    # Skip Seq(4) and get Nsamp(2)
                    hdr = ser.read(6)
                    if len(hdr) < 6: continue
                    nsamp = struct.unpack("<H", hdr[4:6])[0]
                    
                    # Read all samples in this frame
                    payload = ser.read(nsamp * 2)
                    # Skip CRC
                    ser.read(2) 

                    if len(payload) >= 2:
                        # Unpack all samples, take the very last one for display
                        samples = struct.unpack(f"<{nsamp}h", payload)
                        last_raw = samples[-1] & 0xFFF
                        
                        # Print only the raw count
                        last_raw_14bit = samples[-1] & 0x3FFF
                        print(f"14bit: {last_raw_14bit}", end='\r')
                        

    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        ser.close()

if __name__ == "__main__":
    run_raw_monitor()
