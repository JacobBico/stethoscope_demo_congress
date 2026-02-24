"""
raw_adc_csv.py  –  Capture raw ADC samples from Arduino UNO Q to CSV.
Uses Bridge.call("ecg_get_frame") — no Serial, no SPI.

Run:  python3 raw_adc_rpc.py
      python3 raw_adc_rpc.py --duration 10 --output my_data.csv
"""

from __future__ import annotations

import csv
import struct
import time
import argparse
from typing import List, Tuple

from arduino.app_utils import Bridge

CRC_POLY    = 0xA001
MAX_SAMPLES = 255

def crc16_ibm(data: bytes) -> int:
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ CRC_POLY if crc & 1 else crc >> 1
    return crc & 0xFFFF

def parse_frame(resp) -> Tuple[List[int], List[int]]:
    if not resp:
        return [], []
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
            return [], []

    if buf and buf[0] == 0x21 and len(buf) > 1:
        buf = buf[1:]

    if len(buf) < 7:
        return [], []

    count = buf[0]
    if count == 0 or count > MAX_SAMPLES:
        return [], []

    if len(buf) != 1 + 4 + count * 3 + 2:
        return [], []

    if crc16_ibm(buf[:-2]) != (buf[-2] | buf[-1] << 8):
        return [], []

    t0     = struct.unpack_from("<I", buf, 1)[0]
    offset = 5
    samples, timestamps = [], []
    prev_t = t0
    for _ in range(count):
        sample  = struct.unpack_from("<H", buf, offset)[0]
        offset += 2
        prev_t += buf[offset]
        offset += 1
        samples.append(sample)
        timestamps.append(prev_t)

    return samples, timestamps

def run_capture(duration: int, filename: str, vref: float = 3.3, bits: int = 12):
    print(f"\n-- ADC Capture (A0) -------------------------------------")
    print(f"   Duration : {duration} s")
    print(f"   Output   : {filename}")
    print(f"---------------------------------------------------------\n")

    total    = 0
    bad      = 0
    start    = time.monotonic()
    end_time = start + duration

    with open(filename, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["sample_index", "timestamp_ms", "adc_raw", "voltage_V"])

        try:
            while time.monotonic() < end_time:
                resp = Bridge.call("ecg_get_frame")
                samples, timestamps = parse_frame(resp)

                if not samples:
                    bad += 1
                    continue

                for raw, ts in zip(samples, timestamps):
                    volts = raw * vref / ((1 << bits) - 1)
                    writer.writerow([total, ts, raw, f"{volts:.5f}"])
                    total += 1

                elapsed = time.monotonic() - start
                rate    = total / elapsed if elapsed > 0 else 0
                print(f"  {elapsed:5.1f}/{duration}s  |  samples={total:8,}  |  "
                      f"{rate:6.0f} Sa/s  |  bad_frames={bad}",
                      end="\r", flush=True)

        except KeyboardInterrupt:
            print("\n  Stopped.")

    actual = time.monotonic() - start
    print(f"\n\n-- Done -------------------------------------------------")
    print(f"   File       : {filename}")
    print(f"   Samples    : {total:,}")
    print(f"   Duration   : {actual:.2f} s")
    print(f"   Sample rate: {total/actual if actual > 0 else 0:,.0f} Sa/s")
    print(f"   Bad frames : {bad}")
    print(f"---------------------------------------------------------\n")

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", "-d", type=int, choices=[5, 10, 15], default=None)
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
