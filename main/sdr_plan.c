#include "sdr_plan.h"
#include <math.h>
#include <stddef.h>

uint32_t sdr_phase_inc(double shift_hz, uint32_t fs_out)
{
    double r = shift_hz / (double)fs_out;
    r -= floor(r);                               /* [0,1) - Alias durch Phasenueberlauf */
    return (uint32_t)(uint64_t)llround(r * 4294967296.0);
}

static void fill(sdr_plan_t *p, uint32_t fs, double fc, int n)
{
    p->fs = fs;
    p->n = n;
    p->fs_out = (uint32_t)((uint64_t)fs * (uint64_t)n);
    p->exact = (SDR_PLL_HZ % p->fs_out) == 0;
    p->fmax = fabs(fc) + 0.5 * fs;
    p->nyquist_ok = p->fs_out >= SDR_NYQ_FACTOR * p->fmax;
    p->alias = (double)p->fs_out < 2.0 * p->fmax;
    p->alias_hz = 0;
    if (p->alias) {
        double f = fmod(fabs(fc), (double)p->fs_out);
        p->alias_hz = f > 0.5 * p->fs_out ? p->fs_out - f : f;
    }
}

int sdr_plan_rate(uint32_t fs, double shift_hz, uint32_t out_max_hz, sdr_plan_t *p)
{
    if (fs == 0 || (uint64_t)fs * 4 > out_max_hz) return -1;
    const int nmax = (int)(out_max_hz / fs) / 4 * 4;

    sdr_plan_t c, best;
    bool have = false;

    /* 1) kleinstes N mit exaktem Teiler UND ausreichend Nyquist-Abstand */
    for (int n = 4; n <= nmax; n += 4) {
        fill(&c, fs, shift_hz, n);
        if (c.exact && c.nyquist_ok) { *p = c; return 0; }
    }
    /* 2) kein exaktes N erfuellt Nyquist: groesstes exaktes N (Alias/Bild-Betrieb) */
    for (int n = 4; n <= nmax; n += 4) {
        fill(&c, fs, shift_hz, n);
        if (c.exact) { best = c; have = true; }
    }
    /* 3) ... aber ein inexaktes N erfuellt Nyquist -> lieber Bruch-Teiler (Jitter) als Alias */
    for (int n = 4; n <= nmax; n += 4) {
        fill(&c, fs, shift_hz, n);
        if (!c.exact && c.nyquist_ok && (!have || best.alias)) { *p = c; return 0; }
    }
    if (have) { *p = best; return 0; }
    /* 4) gar nichts exakt: groesstes N */
    fill(p, fs, shift_hz, nmax);
    return 0;
}
