/*
 * sdr_plan.h - Wahl der PARLIO-Ausgaberate zu gegebener WAV-Samplerate/Shift.
 *
 * Fs_out = N * fs mit N = 4, 8, 12, ... (Vielfache von 4: zwei Halbband-
 * Interpolatoren + lineare Rampe).  Fs_out muss ein ganzzahliger Teiler der
 * PARLIO-Quelltaktes (160 MHz) sein, damit der Takt jitterfrei und exakt ist.
 * Bevorzugt wird das kleinste N, bei dem der Traeger inkl. Seitenband sauber
 * unterhalb Nyquist liegt (Fs_out >= SDR_NYQ_FACTOR * (|fc| + fs/2)).
 */
#pragma once
#include <stdint.h>
#include <stdbool.h>

#define SDR_PLL_HZ      160000000u
#define SDR_NYQ_FACTOR  2.5          /* Fs_out >= 2.5 * fmax: analoger Rekonstruktionsfilter machbar */

typedef struct {
    uint32_t fs;          /* WAV-Samplerate                              */
    uint32_t fs_out;      /* PARLIO-Pixeltakt = N * fs                   */
    int      n;           /* Interpolationsfaktor (Vielfaches von 4)     */
    bool     exact;       /* SDR_PLL_HZ % fs_out == 0 (jitterfreier Teiler) */
    bool     nyquist_ok;  /* fs_out >= SDR_NYQ_FACTOR * fmax             */
    bool     alias;       /* fs_out < 2 * fmax: Signal nur als Alias/Bild (hoehere Nyquistzone) */
    double   fmax;        /* |fc| + fs/2                                 */
    double   alias_hz;    /* tiefste Alias-/Bildfrequenz im ersten Nyquistband (nur bei alias) */
} sdr_plan_t;

/* out_max_hz: hoechster erlaubter Pixeltakt (CPU/PARLIO-Grenze).
 * Rueckgabe: 0 = ok, <0 = nicht machbar (fs*4 > out_max_hz oder fs == 0). */
int sdr_plan_rate(uint32_t fs, double shift_hz, uint32_t out_max_hz, sdr_plan_t *p);

/* NCO-Phaseninkrement fuer shift_hz bei Abtastrate fs_out (modulo 2^32, Alias inklusive). */
uint32_t sdr_phase_inc(double shift_hz, uint32_t fs_out);
