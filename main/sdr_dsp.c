#include "sdr_dsp.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>

#ifdef ESP_PLATFORM
#include "esp_attr.h"
#define SDR_HOT IRAM_ATTR
#else
#define SDR_HOT
#endif

#ifndef SDR_HB1_BETA
#define SDR_HB1_BETA 5.0
#endif
#ifndef SDR_HB2_BETA
#define SDR_HB2_BETA 4.0
#endif
typedef struct { int16_t c, s; } lut_t;

static lut_t   s_lut[SDR_LUT_SIZE];
static int32_t s_g1[SDR_HB1_K];   /* Q15, Summe = 16384 (HQ-Satz)   */
static int32_t s_g2[SDR_HB2_K];
static int32_t s_g1f[3];          /* leichter Satz fuer hohe fs      */

static bool    s_init;

static double bessel_i0(double x)
{
    double sum = 1.0, term = 1.0, q = x * x / 4.0;
    for (int k = 1; k < 50; k++) {
        term *= q / ((double)k * k);
        sum += term;
    }
    return sum;
}

/* Halbband-Halbschritt-Interpolator: g_k = sinc(k+1/2) * Kaiser, Summe g = 1/2 */
static void design_hb(int32_t *g, int K, double beta)
{
    double v[8], sum = 0.0;
    for (int k = 0; k < K; k++) {
        double x = k + 0.5;
        double sinc = sin(M_PI * x) / (M_PI * x);
        double r = x / K;
        double w = bessel_i0(beta * sqrt(1.0 - r * r)) / bessel_i0(beta);
        v[k] = sinc * w;
        sum += v[k];
    }
    int32_t isum = 0, imax = 0;
    for (int k = 0; k < K; k++) {
        g[k] = (int32_t)lrint(v[k] / sum * 0.5 * 32768.0);
        isum += g[k];
        if (llabs(g[k]) > llabs(g[imax])) imax = k;
    }
    g[imax] += 16384 - isum;            /* DC-Verstaerkung exakt 1 */
}

void sdr_dsp_global_init(void)
{
    if (s_init) return;
    for (uint32_t i = 0; i < SDR_LUT_SIZE; i++) {
        double a = 2.0 * M_PI * (double)i / (double)SDR_LUT_SIZE;
        s_lut[i].c = (int16_t)lrint(cos(a) * 32767.0);
        s_lut[i].s = (int16_t)lrint(sin(a) * 32767.0);
    }
    design_hb(s_g1, SDR_HB1_K, SDR_HB1_BETA);
    design_hb(s_g2, SDR_HB2_K, SDR_HB2_BETA);
    design_hb(s_g1f, 3, 4.0);
    s_init = true;
}

void sdr_dsp_init(sdr_dsp_t *d, const sdr_dsp_cfg_t *cfg)
{
    sdr_dsp_global_init();
    bool agc = d->agc;  float mg = d->manual_gain;
    memset(d, 0, sizeof(*d));
    d->cfg = *cfg;
    d->shift = 30 - cfg->bits;
    d->recipM = cfg->hold > 0 ? 65536 / cfg->hold : 65536;   /* hold = Rampenlaenge: N/4 (HQ) bzw. N/2 (FAST) */
    d->agc = agc;
    d->manual_gain = mg > 0 ? mg : 0.5f;
    d->gain = d->manual_gain;
    d->first = true;
    /* Zeitkonstanten wie im PC-Code pro 1024 Samples: 0.95 / 0.98 */
    d->a_peak = 0.05f;
    d->a_gain = 0.02f;
}

void sdr_dsp_set_phase_inc(sdr_dsp_t *d, uint32_t inc) { d->cfg.phase_inc = inc; }

uint16_t sdr_silence_code(int bits, bool ob)
{
    if (!ob) return 0;
    return bits == 8 ? 0x80 : 0x8000;
}

static inline int32_t clampi(int32_t v, int32_t lim)
{
    return v > lim ? lim : (v < -lim ? -lim : v);
}

/* ---- Mischer -------------------------------------------------------------
 * bb: I,Q @ 4*fs. Zwischen zwei Basisbandwerten wird linear ueber M Ausgangs-
 * samples interpoliert (Bilder von x(N/4)-Halten waeren sonst nur sinc-gedaempft
 * und fielen bei fc ~ k*2fs direkt neben das Nutzsignal). Der Mischer laeuft
 * um einen 4fs-Wert verzoegert (prev), die Blockgrenzen bleiben stetig.      */
#define MIX_BODY(OUT_T)                                                             \
    const lut_t *lut = s_lut;                                                       \
    const int32_t rnd = 1 << (sh - 1);                                              \
    if (M == 1) {                                                                   \
        for (size_t j = 0; j < nbb; j++) {                                          \
            ph += inc;                                                              \
            const lut_t e = lut[ph >> (32 - SDR_LUT_BITS)];                         \
            int32_t v = (bb[2*j] * (int32_t)e.c - bb[2*j+1] * (int32_t)e.s + rnd) >> sh; \
            *out++ = (OUT_T)(v + off);                                              \
        }                                                                           \
    } else {                                                                        \
        for (size_t j = 0; j < nbb; j++) {                                          \
            const int32_t nI = bb[2*j], nQ = bb[2*j+1];                             \
            int32_t aI = pI * 65536, aQ = pQ * 65536;                               \
            const int32_t dI = (nI - pI) * r, dQ = (nQ - pQ) * r;                   \
            for (int k = 0; k < M; k++) {                                           \
                ph += inc;                                                          \
                const lut_t e = lut[ph >> (32 - SDR_LUT_BITS)];                     \
                int32_t v = ((aI >> 16) * (int32_t)e.c                              \
                           - (aQ >> 16) * (int32_t)e.s + rnd) >> sh;                \
                aI += dI;  aQ += dQ;                                                \
                *out++ = (OUT_T)(v + off);                                          \
            }                                                                       \
            pI = nI;  pQ = nQ;                                                      \
        }                                                                           \
    }

/* Zustandslos: Startphase/-Rampenwert kommen als Argumente, so koennen zwei Cores
 * getrennte Bereiche desselben Blocks rechnen. */
static void SDR_HOT mix8(const int16_t *bb, size_t nbb, int M, int32_t r, uint32_t inc,
                         uint32_t ph, int32_t pI, int32_t pQ, int sh, uint8_t off, uint8_t *out)
{ MIX_BODY(uint8_t) }

static void SDR_HOT mix16(const int16_t *bb, size_t nbb, int M, int32_t r, uint32_t inc,
                          uint32_t ph, int32_t pI, int32_t pQ, int sh, uint16_t off, uint16_t *out)
{ MIX_BODY(uint16_t) }

/* Gain + Halbband 1 (fs->2fs) + Halbband 2 (2fs->4fs) + Klemmung; K1/K2/g* sind Konstanten
 * je Aufruf -> always_inline, Schleifen werden entrollt. */
static inline __attribute__((always_inline)) void run_filters(sdr_dsp_t *d, const int16_t *iq, size_t n,
        int32_t gq, const int K1, const int32_t *g1, const int K2, const int32_t *g2)
{
    const int H1 = 2 * K1 - 1, H2 = 2 * K2 - 1;
    int16_t *inI = d->in_I, *inQ = d->in_Q;
    for (size_t i = 0; i < n; i++) {
        int32_t a = (iq[2*i]   * gq) >> 12;
        int32_t b = (iq[2*i+1] * gq) >> 12;
        inI[H1 + i] = (int16_t)clampi(a, SDR_BB_LIM);
        inQ[H1 + i] = (int16_t)clampi(b, SDR_BB_LIM);
    }
    /* Halbband 1: Ausgabe p = K1-1 .. n+K1-2 */
    int16_t *s1I = d->s1_I, *s1Q = d->s1_Q;
    for (size_t p = K1 - 1; p < n + K1 - 1; p++) {
        int32_t aI = 0, aQ = 0;
        for (int k = 0; k < K1; k++) {
            aI += g1[k] * (inI[p - k] + inI[p + 1 + k]);
            aQ += g1[k] * (inQ[p - k] + inQ[p + 1 + k]);
        }
        size_t o = H2 + 2 * (p - (K1 - 1));
        s1I[o]     = inI[p];
        s1Q[o]     = inQ[p];
        s1I[o + 1] = (int16_t)(aI >> 15);
        s1Q[o + 1] = (int16_t)(aQ >> 15);
    }
    memmove(inI, inI + n, H1 * sizeof(int16_t));
    memmove(inQ, inQ + n, H1 * sizeof(int16_t));
    /* Halbband 2 inkl. Klemmung (garantiert kein Ueberlauf im Mischer) */
    const size_t n2 = 2 * n;
    int16_t *bb = d->bb;
    for (size_t p = K2 - 1; p < n2 + K2 - 1; p++) {
        int32_t aI = 0, aQ = 0;
        for (int k = 0; k < K2; k++) {
            aI += g2[k] * (s1I[p - k] + s1I[p + 1 + k]);
            aQ += g2[k] * (s1Q[p - k] + s1Q[p + 1 + k]);
        }
        size_t o = 2 * (p - (K2 - 1));
        bb[2*o]     = (int16_t)clampi(s1I[p], SDR_BB_LIM);
        bb[2*o + 1] = (int16_t)clampi(s1Q[p], SDR_BB_LIM);
        bb[2*o + 2] = (int16_t)clampi(aI >> 15, SDR_BB_LIM);
        bb[2*o + 3] = (int16_t)clampi(aQ >> 15, SDR_BB_LIM);
    }
    memmove(s1I, s1I + n2, H2 * sizeof(int16_t));
    memmove(s1Q, s1Q + n2, H2 * sizeof(int16_t));
}

/* FAST (hohe fs): Gain + ein Halbband (fs->2fs) direkt nach bb (2fs); danach lineare Rampe x(N/2). */
static inline __attribute__((always_inline)) void run_fast(sdr_dsp_t *d, const int16_t *iq, size_t n,
        int32_t gq, const int K1, const int32_t *g1)
{
    const int H1 = 2 * K1 - 1;
    int16_t *inI = d->in_I, *inQ = d->in_Q;
    for (size_t i = 0; i < n; i++) {
        int32_t a = (iq[2*i]   * gq) >> 12;
        int32_t b = (iq[2*i+1] * gq) >> 12;
        inI[H1 + i] = (int16_t)clampi(a, SDR_BB_LIM);
        inQ[H1 + i] = (int16_t)clampi(b, SDR_BB_LIM);
    }
    int16_t *bb = d->bb;
    for (size_t p = K1 - 1; p < n + K1 - 1; p++) {
        int32_t aI = 0, aQ = 0;
        for (int k = 0; k < K1; k++) {
            aI += g1[k] * (inI[p - k] + inI[p + 1 + k]);
            aQ += g1[k] * (inQ[p - k] + inQ[p + 1 + k]);
        }
        size_t o = 4 * (p - (K1 - 1));
        bb[o]     = inI[p];
        bb[o + 1] = inQ[p];
        bb[o + 2] = (int16_t)clampi(aI >> 15, SDR_BB_LIM);
        bb[o + 3] = (int16_t)clampi(aQ >> 15, SDR_BB_LIM);
    }
    memmove(inI, inI + n, H1 * sizeof(int16_t));
    memmove(inQ, inQ + n, H1 * sizeof(int16_t));
}

/* ---- Hauptfunktion ------------------------------------------------------ */
size_t SDR_HOT sdr_dsp_filter(sdr_dsp_t *d, const int16_t *iq, size_t n)
{
    if (n == 0 || n > SDR_BLOCK_MAX) return 0;

    /* 1) Gain / AGC (Blockpeak ueber approximierte Betragsbildung) */
    int32_t peak = 0;
    for (size_t i = 0; i < n; i++) {
        int32_t a = iq[2*i], b = iq[2*i+1];
        a = a < 0 ? -a : a;  b = b < 0 ? -b : b;
        int32_t mx = a > b ? a : b, mn = a > b ? b : a;
        int32_t m = mx + ((mn * 3) >> 3);
        if (m > peak) peak = m;
    }
    float pk = (float)peak / 32768.0f;
    d->last_peak = pk;
    float g;
    if (d->agc) {
        float a_p = d->a_peak, a_g = d->a_gain;
        if (n != 1024) {                      /* Zeitkonstante pro Sample-Anzahl */
            float f = (float)n / 1024.0f;
            a_p = 1.0f - powf(1.0f - d->a_peak, f);
            a_g = 1.0f - powf(1.0f - d->a_gain, f);
        }
        if (d->first) { d->peak_hold = pk; d->first = false;
                        d->gain = SDR_AGC_TARGET / (pk + 1e-4f); }
        d->peak_hold += a_p * (pk - d->peak_hold);
        float target = SDR_AGC_TARGET / (d->peak_hold + 1e-4f);
        if (target > 16.0f) target = 16.0f;
        d->gain += a_g * (target - d->gain);
        g = d->gain;
    } else {
        g = d->manual_gain;
        d->gain = g;
    }
    if (g > 16.0f) g = 16.0f;
    const int32_t gq = (int32_t)(g * 0.5f * 4096.0f);    /* Q12; 32768 -> 16384; |x*gq| <= 2^30 */

    if (d->cfg.fast) run_fast(d, iq, n, gq, 3, s_g1f);
    else             run_filters(d, iq, n, gq, SDR_HB1_K, s_g1, SDR_HB2_K, s_g2);
    return (d->cfg.fast ? 2 : 4) * n;      /* Anzahl 4fs- bzw. 2fs-Basisbandwerte in d->bb */
}

size_t SDR_HOT sdr_dsp_mix(const sdr_dsp_t *d, size_t first, size_t count, void *out)
{
    const int M = d->cfg.hold;
    const uint32_t inc = d->cfg.phase_inc;
    uint32_t ph = d->phase + inc * (uint32_t)(first * (size_t)M);
    int32_t pI = first ? d->bb[2 * (first - 1)]     : d->prevI;
    int32_t pQ = first ? d->bb[2 * (first - 1) + 1] : d->prevQ;
    const int16_t *bb = d->bb + 2 * first;
    if (d->cfg.bits == 8)
        mix8(bb, count, M, d->recipM, inc, ph, pI, pQ, d->shift,
             d->cfg.offset_binary ? 0x80 : 0, (uint8_t *)out);
    else
        mix16(bb, count, M, d->recipM, inc, ph, pI, pQ, d->shift,
              d->cfg.offset_binary ? 0x8000 : 0, (uint16_t *)out);
    return count * (size_t)M;
}

void sdr_dsp_commit(sdr_dsp_t *d, size_t nbb)
{
    d->phase += d->cfg.phase_inc * (uint32_t)(nbb * (size_t)d->cfg.hold);
    d->prevI = d->bb[2 * (nbb - 1)];
    d->prevQ = d->bb[2 * (nbb - 1) + 1];
}

size_t SDR_HOT sdr_dsp_process(sdr_dsp_t *d, const int16_t *iq, size_t n, void *out)
{
    size_t nbb = sdr_dsp_filter(d, iq, n);
    if (!nbb) return 0;
    size_t m = sdr_dsp_mix(d, 0, nbb, out);
    sdr_dsp_commit(d, nbb);
    return m;
}

/* Lastmodell aus gezaehlten RISC-V-Instruktionen (riscv32-esp-elf-gcc 15, -O3):
 *   Gain 12, HB1(K=6) 81, HB2(K=3) 2*55 je Eingangssample (HQ), Mischer 19 je Ausgangssample,
 *   1,2 Takte/Instruktion.  HQ stimmt mit der Messung (21 % @ 250 ksps/1 MS/s) ueberein.
 *   FAST wurde nach Messung (65 % @ 1,25 Msps/5 MS/s statt 48 %) auf 97 Instr./Eingangssample
 *   kalibriert. */
void sdr_est_cycles(uint32_t fs, uint32_t fs_out, bool fast, double *filter_cps, double *mix_cps)
{
    double ins_in = fast ? 97.0 : (12 + 81 + 110);   /* FAST an Messung kalibriert (65 % @ 1,25 Msps/5 MS/s) */
    *filter_cps = (double)fs * ins_in * 1.2;
    *mix_cps    = (double)fs_out * 19.0 * 1.2;
}

float sdr_est_load_pct(uint32_t fs, uint32_t fs_out, bool fast, uint32_t cpu_mhz)
{
    double f, m;
    sdr_est_cycles(fs, fs_out, fast, &f, &m);
    return (float)((f + m) / ((double)cpu_mhz * 1e6) * 100.0);
}

float sdr_est_split(uint32_t fs, uint32_t fs_out, bool fast)
{
    double f, m;
    sdr_est_cycles(fs, fs_out, fast, &f, &m);
    double s = (m - f) / (2.0 * m);                 /* Anteil des Mischers auf dem Filter-Core, Lastgleichheit */
    return (float)(s < 0 ? 0 : (s > 0.5 ? 0.5 : s));
}
