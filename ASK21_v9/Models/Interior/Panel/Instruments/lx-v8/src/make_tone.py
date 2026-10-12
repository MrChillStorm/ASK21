#!/usr/bin/env python3
"""Sounds/lxv8-tone.wav: one second of a 500 Hz triangle wave, seamless when looped.

FG9070 plays a triangle wave through a 3.5 kHz low-pass; the harmonics of the triangle that lie above
3.5 kHz (at the base frequency of 500 Hz: from the 9th on) are left out here. The sound system changes
the pitch (factor frequency / 500 Hz) and switches the volume to make the beeps."""
import math, os, struct, wave
rate, f0, seconds = 44100, 500, 1
ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..', '..', '..', '..'))
out = os.path.join(ROOT, 'Sounds', 'lxv8-tone.wav')
frames = []
for i in range(rate * seconds):
    t = i / rate
    v = sum(((-1) ** ((n - 1) // 2)) * math.sin(2 * math.pi * n * f0 * t) / (n * n) for n in (1, 3, 5, 7))
    frames.append(int(max(-1, min(1, v * 8 / math.pi ** 2 * 0.8)) * 32767))
with wave.open(out, 'wb') as w:
    w.setnchannels(1); w.setsampwidth(2); w.setframerate(rate)
    w.writeframes(struct.pack('<%dh' % len(frames), *frames))
print(out, len(frames), 'samples')
