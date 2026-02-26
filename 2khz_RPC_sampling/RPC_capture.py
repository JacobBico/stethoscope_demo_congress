"""
RPC_capture.py - Capture ADC frames from Arduino UNO Q via Bridge RPC.
Calls adc_get_frame() repeatedly, writes samples to CSV.

Frame: [count(2)][seq(4)][samples(count*2)][crc16(2)]

Usage:
    python3 RPC_capture.py
    python3 RPC_capture.py --duration 10 --output data.csv
"""

import csv
import struct
import time
import argparse
from arduino.app_utils import Bridge

VREF     = 3.3
ADC_BITS = 14

def crc16_ibm(data: bytes) -> int:
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc & 0xFFFF

def parse_frame(resp):
    if not resp:
        return None, None

    if isinstance(resp, (bytes, bytearray)):
        raw = bytes(resp)
        hex_chars = set(b"0123456789abcdefABCDEF")
        if raw and len(raw) % 2 == 0 and all(b in hex_chars for b in raw):
            try:
                buf = bytes.fromhex(raw.decode("ascii"))
            except ValueError:
                buf = raw
        else:
            buf = raw
    else:
        try:
            buf = bytes.fromhex(str(resp).strip())
        except ValueError:
            return None, None

    # Strip leading 0x21 overflow marker if present
    if buf and buf[0] == 0x21:
        buf = buf[1:]

    if len(buf) < 8:  # min: count(2)+seq(4)+crc(2)
        return None, None

    count = struct.unpack_from("<H", buf, 0)[0]
    seq   = struct.unpack_from("<I", buf, 2)[0]

    expected_len = 6 + count * 2 + 2
    if len(buf) != expected_len:
        return None, None

    # Verify CRC
    if crc16_ibm(buf[:-2]) != struct.unpack_from("<H", buf, len(buf)-2)[0]:
        return None, None

    samples = []
    for i in range(count):
        v = struct.unpack_from("<H", buf, 6 + i * 2)[0] & 0x3FFF
        samples.append(v)

    return seq, samples

def run_capture(duration, filename):
    print(f"\n-- ADC RPC Capture --------------------------------------")
    print(f"   Duration : {duration}s")
    print(f"   Output   : {filename}")
    print(f"---------------------------------------------------------\n")

    total      = 0
    bad        = 0
    empty      = 0
    last_seq   = None
    seq_errors = 0
    start      = time.monotonic()
    end_time   = start + duration

    with open(filename, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["sample_index", "adc_raw", "voltage_V"])

        try:
            while time.monotonic() < end_time:
                resp = Bridge.call("adc_get_frame")
                seq, samples = parse_frame(resp)

                if samples is None:
                    empty += 1
                    continue

                if seq is None:
                    bad += 1
                    continue

                if last_seq is not None:
                    expected = (last_seq + 1) & 0xFFFFFFFF
                    if seq != expected:
                        seq_errors += 1
                last_seq = seq

                for s in samples:
                    volts = s * VREF / ((1 << ADC_BITS) - 1)
                    writer.writerow([total, s, f"{volts:.5f}"])
                    total += 1

                elapsed = time.monotonic() - start
                rate    = total / elapsed if elapsed > 0 else 0
                print(
                    f"  {elapsed:5.1f}/{duration}s | "
                    f"samples={total:9,} | {rate:8.0f} Sa/s | "
                    f"seq_err={seq_errors} bad={bad} empty={empty}",
                    end="\r", flush=True
                )

        except KeyboardInterrupt:
            print("\n  Stopped.")

    actual = time.monotonic() - start
    print(f"\n\n-- Done ------------------------------------------------")
    print(f"   File    : {filename}")
    print(f"   Samples : {total:,}")
    print(f"   Rate    : {total/actual if actual > 0 else 0:,.0f} Sa/s")
    print(f"   Seq err : {seq_errors} | Bad: {bad} | Empty: {empty}")
    print(f"--------------------------------------------------------\n")

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", "-d", type=int, default=None)
    parser.add_argument("--output",   "-o", default="raw_adc_a0.csv")
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

    run_capture(duration, args.output)

if __name__ == "__main__":
    main()
