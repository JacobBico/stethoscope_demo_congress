"""
raw_adc_csv.py  -  44.1 kHz ADC capture from Arduino UNO Q via Serial1 (ttyHS1)

Usage:
    python3 raw_adc_csv.py
    python3 raw_adc_csv.py --duration 10 --port /dev/ttyHS1 --output data.csv

Stop arduino-router.service before running:
    sudo systemctl stop arduino-router.service
"""

import serial
import csv
import struct
import time
import argparse

# Must match sketch constants
MAGIC         = 0xA55A
MAGIC_B       = bytes([0x5A, 0xA5])   # little-endian on wire
FRAME_SAMPLES = 512
BAUD          = 2_000_000
VREF          = 3.3
ADC_BITS      = 12

# Frame structure after magic:
#   seq(4) + nsamp(2) + samples(nsamp*2) + crc(2)
HDR_AFTER_MAGIC = 6   # seq(4) + nsamp(2)

def crc16(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = (crc << 1) ^ 0x1021 if crc & 0x8000 else crc << 1
        crc &= 0xFFFF
    return crc

def read_exact(ser, n: int) -> bytes:
    buf = b""
    while len(buf) < n:
        chunk = ser.read(n - len(buf))
        if not chunk:
            raise TimeoutError("Serial timeout — is the sketch running?")
        buf += chunk
    return buf

def sync(ser, timeout=5.0) -> bool:
    """Scan byte-by-byte until we see 0x5A 0xA5 (magic little-endian)."""
    print("  Syncing to stream...", end="", flush=True)
    deadline = time.monotonic() + timeout
    prev = b""
    while time.monotonic() < deadline:
        b = ser.read(1)
        if not b:
            continue
        if prev == bytes([0x5A]) and b == bytes([0xA5]):
            print(" OK")
            return True
        prev = b
    print(" TIMEOUT — check sketch is uploaded and Serial1 is free")
    return False

def run_capture(duration: int, port: str, filename: str):
    print(f"\n-- ADC Serial1 Capture ---------------------------------")
    print(f"   Port    : {port} @ {BAUD} baud")
    print(f"   Duration: {duration}s")
    print(f"   Output  : {filename}")
    print(f"--------------------------------------------------------\n")

    ser = serial.Serial(port, BAUD, timeout=1.0)
    time.sleep(0.3)
    ser.reset_input_buffer()

    if not sync(ser, timeout=5.0):
        ser.close()
        return

    total = bad_crc = seq_errors = 0
    last_seq  = None
    start     = time.monotonic()
    end_time  = start + duration

    with open(filename, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["sample_index", "adc_raw", "voltage_V"])

        try:
            while time.monotonic() < end_time:
                # Read header after magic: seq(4) + nsamp(2)
                hdr    = read_exact(ser, HDR_AFTER_MAGIC)
                seq_no = struct.unpack_from("<I", hdr, 0)[0]
                nsamp  = struct.unpack_from("<H", hdr, 4)[0]

                if nsamp == 0 or nsamp > FRAME_SAMPLES:
                    if not sync(ser, timeout=2.0):
                        break
                    continue

                payload = read_exact(ser, nsamp * 2)
                crc_raw = read_exact(ser, 2)
                rx_crc  = struct.unpack_from("<H", crc_raw)[0]

                # CRC covers magic + hdr + payload
                covered = MAGIC_B + hdr + payload
                if crc16(covered) != rx_crc:
                    bad_crc += 1
                    if not sync(ser, timeout=2.0):
                        break
                    continue

                if last_seq is not None:
                    expected = (last_seq + 1) & 0xFFFFFFFF
                    if seq_no != expected:
                        seq_errors += 1
                last_seq = seq_no

                # Samples are int16 in sketch but ADC is 12-bit unsigned
                # Cast back to unsigned for CSV
                samples = struct.unpack_from(f"<{nsamp}h", payload)
                for s in samples:
                    raw   = s & 0xFFF        # 12-bit mask
                    volts = raw * VREF / ((1 << ADC_BITS) - 1)
                    writer.writerow([total, raw, f"{volts:.5f}"])
                    total += 1

                # Stay framed: read next magic
                nxt = read_exact(ser, 2)
                if nxt != MAGIC_B:
                    if not sync(ser, timeout=2.0):
                        break

                elapsed = time.monotonic() - start
                rate    = total / elapsed if elapsed > 0 else 0
                print(
                    f"  {elapsed:5.1f}/{duration}s | "
                    f"samples={total:9,} | {rate:8.0f} Sa/s | "
                    f"crc_err={bad_crc} seq_err={seq_errors}",
                    end="\r", flush=True
                )

        except KeyboardInterrupt:
            print("\n  Stopped.")
        except TimeoutError as e:
            print(f"\n  {e}")

    ser.close()
    actual = time.monotonic() - start
    print(f"\n\n-- Done ------------------------------------------------")
    print(f"   File    : {filename}")
    print(f"   Samples : {total:,}")
    print(f"   Rate    : {total/actual if actual > 0 else 0:,.0f} Sa/s")
    print(f"   CRC err : {bad_crc} | Seq err: {seq_errors}")
    print(f"--------------------------------------------------------\n")

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", "-d", type=int, choices=[5, 10, 15], default=None)
    parser.add_argument("--output",   "-o", default="raw_adc_a0.csv")
    parser.add_argument("--port",     "-p", default="/dev/ttyHS1")
    args = parser.parse_args()

    if args.duration is None:
        try:
            duration = int(input("Enter duration (5, 10, or 15 seconds): "))
            if duration not in [5, 10, 15]:
                duration = 5
        except:
            duration = 5
    else:
        duration = args.duration

    run_capture(duration, args.port, args.output)

if __name__ == "__main__":
    main()
