import csv
import wave
import struct
import numpy as np

input_csv = "raw_adc_a0.csv"
output_wav = "output.wav"

sample_rate = 5000  # <-- set this to your ADC sampling rate (Hz)

samples = []

# Read voltage column (3rd column)
with open(input_csv, "r") as file:
    reader = csv.reader(file)
    next(reader)  # skip header if you have one

    for row in reader:
        voltage = float(row[2])
        samples.append(voltage)

samples = np.array(samples)

# Normalize to 16-bit audio range
samples = samples - np.mean(samples)          # remove DC offset
samples = samples / np.max(np.abs(samples))   # normalize
samples = (samples * 32767).astype(np.int16)  # convert to int16

# Write WAV file
with wave.open(output_wav, "w") as wav_file:
    wav_file.setnchannels(1)      # mono
    wav_file.setsampwidth(2)      # 16-bit
    wav_file.setframerate(sample_rate)
    wav_file.writeframes(samples.tobytes())

print("WAV file created:", output_wav)
