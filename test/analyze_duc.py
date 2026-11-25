#!/usr/bin/env python3
"""Spektralpruefung der Ausgabe von host_duc: Traeger vor/nach dem Shiftwechsel (Sollwerte passend zu host_duc.c)."""
import sys
import numpy as np

fs_out, split = 4e6, 80000
x = np.fromfile(sys.argv[1], dtype=np.int8).astype(float)

def peak(seg):
    S = np.abs(np.fft.rfft(seg * np.hanning(len(seg))))
    f = np.fft.rfftfreq(len(seg), 1 / fs_out)
    i = S.argmax()
    S2 = S.copy(); S2[max(0, i - 40):i + 40] = 0
    return f[i], 20 * np.log10(S[i] / S2.max())

ok = True
for name, seg, want in (("vor dem Shiftwechsel ", x[8000:8000 + 65536], 1243e3),
                        ("nach dem Shiftwechsel", x[split + 20000:split + 20000 + 65536], 1223e3)):
    f, spur = peak(seg)
    good = abs(f - want) < 200 and spur > 40
    ok &= good
    print(f"{name}: {f/1e3:8.1f} kHz (soll {want/1e3:.1f}), naechste Spur -{spur:.1f} dB  {'OK' if good else 'FEHLER'}")
sys.exit(0 if ok else 1)
