/*
 * sdr_dsp.h - Integer-DSP-Kette: I/Q (int16) -> x4 Halbband-Interpolation
 *             -> lineare Rampe x(N/4) -> LUT-NCO-Mischer -> reelles HF-Signal (8/16 Bit)
 *
 * Reines C ohne IDF-Abhaengigkeit (auf dem Host testbar).
 */
#pragma once
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SDR_LUT_BITS    12                  /* NCO-LUT: 4096 Eintraege (wie DspWorker) */
#define SDR_LUT_SIZE    (1u << SDR_LUT_BITS)
#define SDR_BLOCK_MAX   1024                /* max. I/Q-Samples pro sdr_dsp_process() */
#define SDR_BB_FS       16384               /* Basisband-Vollaussteuerung (2^14)      */
#define SDR_BB_LIM      11585               /* je Komponente: |z| <= SDR_BB_FS         */
#ifndef SDR_HB1_K
#define SDR_HB1_K       6                   /* Halbband 1: 12 Stuetzstellen            */
#endif
#ifndef SDR_HB2_K
#define SDR_HB2_K       3                   /* Halbband 2: 6 Stuetzstellen             */
#endif
#define SDR_FAST_FS_HZ  600000              /* ab dieser WAV-Rate: FAST-Modus (1 Halbband) */
#define SDR_AGC_TARGET  0.5f                /* AGC-Ziel: Peak = 50 % Vollaussteuerung  */

typedef struct {
    int      bits;            /* 8 oder 16                                     */
    bool     offset_binary;   /* true: Offset-Binary (Mitte = 0x80 / 0x8000)   */
    int      hold;            /* Rampenlaenge: N/4 (HQ: 2 Halbbaender) bzw. N/2 (fast: 1 Halbband) */
    uint32_t phase_inc;       /* NCO: shift_hz / fs_out * 2^32                 */
    bool     fast;            /* nur 1 Halbband (fs >= SDR_FAST_FS_HZ; Bilder liegen >= 1,5*fs entfernt) */
} sdr_dsp_cfg_t;

typedef struct {
    sdr_dsp_cfg_t cfg;
    uint32_t phase;
    int32_t  prevI, prevQ;    /* letzter 4fs-Basisbandwert (Rampe ueber Blockgrenzen) */
    int32_t  recipM;          /* 65536 / hold */
    int      shift;           /* 30 - bits */
    /* Gain */
    bool     agc;
    float    manual_gain;     /* 1.0 = Eingangs-Vollaussteuerung -> DAC-Vollaussteuerung */
    float    gain, peak_hold, a_peak, a_gain;
    bool     first;
    float    last_peak;       /* letzter Blockpeak (0..1), fuer status */
    /* Filterzustand + Arbeitspuffer */
    int16_t  in_I[2 * SDR_HB1_K - 1 + SDR_BLOCK_MAX];
    int16_t  in_Q[2 * SDR_HB1_K - 1 + SDR_BLOCK_MAX];
    int16_t  s1_I[2 * SDR_HB2_K - 1 + 2 * SDR_BLOCK_MAX];
    int16_t  s1_Q[2 * SDR_HB2_K - 1 + 2 * SDR_BLOCK_MAX];
    int16_t  bb[2 * 4 * SDR_BLOCK_MAX];   /* I,Q @ 4*fs */
} sdr_dsp_t;

/* Globale Tabellen (LUT, Halbbandkoeffizienten) einmalig berechnen. */
void sdr_dsp_global_init(void);

/* Kontext (neu) initialisieren; Filterzustand und AGC werden zurueckgesetzt.
 * d->agc und d->manual_gain bleiben erhalten (vorher setzen). */
void sdr_dsp_init(sdr_dsp_t *d, const sdr_dsp_cfg_t *cfg);

/* Nur Shift/Phase-Inkrement aendern (nahtloser Dateiwechsel bei gleicher Rate). */
void sdr_dsp_set_phase_inc(sdr_dsp_t *d, uint32_t inc);

/* n_iq <= SDR_BLOCK_MAX I/Q-Paare (interleaved I,Q) verarbeiten.
 * Schreibt n_iq * 4 * hold Ausgangssamples (uint8_t bzw. uint16_t) nach out.
 * Rueckgabe: Anzahl Ausgangssamples. */
size_t sdr_dsp_process(sdr_dsp_t *d, const int16_t *iq, size_t n_iq, void *out);

/* Aufgeteilte Variante fuer zwei Cores (Ergebnis bit-identisch zu sdr_dsp_process):
 *   n = sdr_dsp_filter(d, iq, n_iq)           -> Basisband in d->bb, Rueckgabe: Anzahl bb-Werte
 *   sdr_dsp_mix(d, first, count, out)         -> mischt bb[first, first+count) nach out (count*hold Samples);
 *                                               darf auf mehreren Cores gleichzeitig mit disjunkten Bereichen laufen
 *   sdr_dsp_commit(d, nbb)                    -> Phase/Rampenzustand weiterschalten (nach allen mix-Aufrufen)
 * Fuer 64-Byte-Ausrichtung der Teilbereiche im Ausgabepuffer: first = Vielfaches von 64. */
size_t sdr_dsp_filter(sdr_dsp_t *d, const int16_t *iq, size_t n_iq);
size_t sdr_dsp_mix(const sdr_dsp_t *d, size_t first, size_t count, void *out);
void   sdr_dsp_commit(sdr_dsp_t *d, size_t nbb);

/* Geschaetzte Auslastung eines Cores in % (Modell aus Instruktionszaehlung). */
float sdr_est_load_pct(uint32_t fs, uint32_t fs_out, bool fast, uint32_t cpu_mhz);
/* Zyklen/s fuer Filterstufe und Mischer getrennt (Modell). */
void  sdr_est_cycles(uint32_t fs, uint32_t fs_out, bool fast, double *filter_cps, double *mix_cps);
/* Zwei Cores: Anteil des Mischers, den der Filter-Core mitrechnet (0..0.5), damit beide gleich ausgelastet sind. */
float sdr_est_split(uint32_t fs, uint32_t fs_out, bool fast);

/* Wert fuer "Stille" im gewaehlten Format. */
uint16_t sdr_silence_code(int bits, bool offset_binary);

#ifdef __cplusplus
}
#endif
