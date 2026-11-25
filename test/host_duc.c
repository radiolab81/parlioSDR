/*
 * host_duc.c - Host-Test fuer duc_proto.c (In-Band-Parser) und die Kette Parser -> sdr_dsp (ohne Hardware)
 *
 *   gcc -O2 -I main -o test/host_duc test/host_duc.c main/duc_proto.c main/sdr_dsp.c main/sdr_plan.c -lm
 *   ./test/host_duc [out.bin]       (out.bin: HF-Samples (int8) fuer test/analyze_duc.py)
 *
 * Der Sender wird 1:1 aus DspWorker.cpp (run_IQ_engine) nachgebildet.
 */
#include "duc_proto.h"
#include "sdr_dsp.h"
#include "sdr_plan.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

/* ---- aus DspWorker.cpp uebernommen ---- */
static uint32_t calculate_ftw(float frequency, float ref_clk) {
    double ftw = ((double)frequency / (double)ref_clk) * 4294967296.0;
    return (uint32_t)(int64_t)llround(ftw);    /* wie static_cast<uint32_t> auf x86-64 (Wrap fuer negative Werte) */
}
static uint16_t make_cmd_word(uint8_t ctrl, uint8_t index, uint8_t payload) {
    return ((ctrl & 0x03) << 14) | ((index & 0x3F) << 8) | (payload & 0xFF);
}

typedef struct { uint16_t *w; size_t n; } wbuf_t;
static void put(wbuf_t *b, uint16_t v) { b->w[b->n++] = v; }

static void put_cmd(wbuf_t *b, char c, uint32_t v) {
    put(b, make_cmd_word(2, 63, (uint8_t)c));
    for (int i = 0; i < 4; i++) put(b, make_cmd_word(2, i, (v >> (8 * i)) & 0xFF));
    put(b, make_cmd_word(3, 0, 'E'));
}
static void put_iq(wbuf_t *b, int16_t raw_i, int16_t raw_q) {
    int16_t si = raw_i >> 2, sq = raw_q >> 2;
    put(b, ((uint16_t)si & 0x3FFF) | (0x00 << 14));
    put(b, ((uint16_t)sq & 0x3FFF) | (0x01 << 14));
}

static int fails = 0;
#define CHECK(c, ...) do { if (!(c)) { printf("FEHLER: " __VA_ARGS__); printf("\n"); fails++; } } while (0)

/* Parser mit beliebiger Stueckelung fuettern; Ergebnisse (Samples + Ereignisse in Reihenfolge) sammeln */
typedef struct { int type; uint32_t value; size_t at_pair; } evrec_t;
static size_t run(const uint8_t *in, size_t n, size_t chunk_max, unsigned seed, int16_t *out, evrec_t *evs, size_t *nev,
                  duc_proto_t *p, size_t blk_cap)
{
    duc_proto_init(p);
    size_t pos = 0, pairs = 0; *nev = 0;
    srand(seed);
    while (pos < n) {
        size_t c = chunk_max ? 1 + (size_t)rand() % chunk_max : n - pos;
        if (c > n - pos) c = n - pos;
        size_t off = 0;
        while (off < c) {
            size_t used, got; duc_event_t ev;
            duc_feed_t r = duc_proto_feed(p, in + pos + off, c - off, &used, out + 2 * pairs, blk_cap, &got, &ev);
            off += used; pairs += got;
            if (r == DUC_FEED_EVENT) { evs[*nev].type = ev.type; evs[*nev].value = ev.value; evs[*nev].at_pair = pairs; (*nev)++; }
            if (r == DUC_FEED_FULL) { /* im echten Code: Block abliefern; hier: Platz wieder frei */ pairs += 0; }
            if (r == DUC_FEED_FULL && got == 0 && used == 0) { printf("Haenger!\n"); fails++; return pairs; }
        }
        pos += c;
    }
    return pairs;
}

int main(int argc, char **argv)
{
    const uint32_t fs = 250000; const float shift = 1233000.0f; const float ref = 50e6f;
    const size_t NIQ = 20000;
    wbuf_t b = { malloc((NIQ * 2 + 64) * 2), 0 };
    int16_t *raw = malloc(NIQ * 4);
    for (size_t i = 0; i < NIQ; i++) {                    /* Basisband: Ton bei +10 kHz, Betrag ~0,6, plus Q-Anteil */
        double ph = 2 * M_PI * 10000.0 * i / fs;
        raw[2*i] = (int16_t)lrint(cos(ph) * 20000.0); raw[2*i+1] = (int16_t)lrint(sin(ph) * 20000.0);
    }
    put_cmd(&b, 'S', calculate_ftw(shift, ref));
    put_cmd(&b, 'R', calculate_ftw((float)fs, ref));
    size_t split_at = 5000;                               /* Mitte im Strom: zweiter Shift (nahtlos) */
    for (size_t i = 0; i < NIQ; i++) {
        if (i == split_at) put_cmd(&b, 'S', calculate_ftw(-shift, ref));    /* negative Frequenz */
        put_iq(&b, raw[2*i], raw[2*i+1]);
    }
    const uint8_t *bytes = (const uint8_t *)b.w; size_t nbytes = b.n * 2;

    int16_t *ref_out = malloc(NIQ * 4 + 64); evrec_t ref_ev[16]; size_t ref_nev; duc_proto_t p;
    size_t pairs = run(bytes, nbytes, 0, 1, ref_out, ref_ev, &ref_nev, &p, NIQ + 8);
    CHECK(pairs == NIQ, "Paare %zu != %zu", pairs, NIQ);
    CHECK(ref_nev == 3, "Ereignisse %zu != 3", ref_nev);
    CHECK(ref_ev[0].type == DUC_EV_SHIFT && ref_ev[0].at_pair == 0, "Ereignis 0");
    CHECK(ref_ev[1].type == DUC_EV_RATE && ref_ev[1].at_pair == 0, "Ereignis 1");
    CHECK(ref_ev[2].type == DUC_EV_SHIFT && ref_ev[2].at_pair == split_at, "Ereignis 2 an Position %zu (soll %zu)", ref_ev[2].at_pair, split_at);
    CHECK(duc_ftw_to_rate_hz(ref_ev[1].value) == fs, "fs aus FTW: %u", duc_ftw_to_rate_hz(ref_ev[1].value));
    double s1 = duc_ftw_to_hz_signed(ref_ev[0].value), s2 = duc_ftw_to_hz_signed(ref_ev[2].value);
    printf("Shift aus FTW: %.4f Hz (soll %.1f), zweiter: %.4f Hz (soll %.1f), fs=%u\n", s1, (double)shift, s2, -(double)shift, duc_ftw_to_rate_hz(ref_ev[1].value));
    CHECK(fabs(s1 - shift) < 0.01 && fabs(s2 + shift) < 0.01, "Shift-Rueckrechnung");
    for (size_t i = 0; i < NIQ; i++) {                    /* 14-Bit-Quantisierung: genau (raw>>2)<<2 */
        int16_t ei = (int16_t)((raw[2*i] >> 2) * 4), eq = (int16_t)((raw[2*i+1] >> 2) * 4);
        if (ref_out[2*i] != ei || ref_out[2*i+1] != eq) { CHECK(0, "Sample %zu: %d,%d != %d,%d", i, ref_out[2*i], ref_out[2*i+1], ei, eq); break; }
    }

    /* Stueckelung: 1 Byte, ungerade, zufaellig bis 1500, Blockgroesse klein (erzwingt FULL) -> identisches Ergebnis */
    static const size_t chunks[] = { 1, 3, 7, 1440, 1501 };
    for (size_t ci = 0; ci < sizeof chunks / sizeof *chunks; ci++) {
        int16_t *o2 = malloc(NIQ * 4 + 64); evrec_t ev2[16]; size_t nev2; duc_proto_t p2;
        size_t pr2 = run(bytes, nbytes, chunks[ci], 42 + (unsigned)ci, o2, ev2, &nev2, &p2, NIQ + 8);
        int same = pr2 == pairs && nev2 == ref_nev && memcmp(o2, ref_out, pairs * 4) == 0;
        for (size_t e = 0; same && e < nev2; e++) same = ev2[e].type == ref_ev[e].type && ev2[e].value == ref_ev[e].value && ev2[e].at_pair == ref_ev[e].at_pair;
        CHECK(same, "Stueckelung bis %zu Byte weicht ab", chunks[ci]);
        free(o2);
    }

    /* Fehlerfaelle: verlorenes Q, Q ohne I, Execute ohne Beginn, unvollstaendiger FTW, unbekannter Befehl */
    {
        wbuf_t e = { malloc(64 * 2), 0 };
        put(&e, 0x0000 | 100);              /* I */
        put(&e, 0x0000 | 200);              /* I (vorheriges ohne Q) */
        put(&e, 0x4000 | 300);              /* Q -> Paar (200,300) */
        put(&e, 0x4000 | 5);                /* Q ohne I */
        put(&e, make_cmd_word(3, 0, 'E'));  /* Execute ohne Beginn */
        put(&e, make_cmd_word(2, 63, 'S')); put(&e, make_cmd_word(2, 0, 1)); put(&e, make_cmd_word(3, 0, 'E')); /* FTW unvollstaendig */
        put(&e, make_cmd_word(2, 63, 'X')); put(&e, make_cmd_word(3, 0, 'E'));                                  /* unbekannt */
        int16_t o[16]; evrec_t ev[8]; size_t nev; duc_proto_t pe;
        size_t pr = run((const uint8_t *)e.w, e.n * 2, 0, 1, o, ev, &nev, &pe, 8);
        CHECK(pr == 1 && o[0] == 200 * 4 && o[1] == 300 * 4, "Fehlerfall: Paar");
        CHECK(pe.drop_i == 1 && pe.drop_q == 1 && pe.n_bad == 2, "Fehlerfall: Zaehler %u/%u/%u", pe.drop_i, pe.drop_q, pe.n_bad);
        CHECK(nev == 3 && ev[0].type == DUC_EV_BAD && ev[1].type == DUC_EV_BAD && ev[2].type == DUC_EV_UNKNOWN, "Fehlerfall: Ereignisse");
        free(e.w);
    }
    /* Vorzeichen der 14-Bit-Werte */
    {
        wbuf_t e = { malloc(16), 0 };
        put(&e, 0x0000 | 0x2000); put(&e, 0x4000 | 0x1FFF);     /* -8192, +8191 */
        int16_t o[4]; evrec_t ev[2]; size_t nev; duc_proto_t pe;
        run((const uint8_t *)e.w, e.n * 2, 0, 1, o, ev, &nev, &pe, 2);
        CHECK(o[0] == -32768 && o[1] == 32764, "Vorzeichen: %d %d", o[0], o[1]);
        free(e.w);
    }

    /* ---- Kette: Parser -> Planer -> DSP (wie dsp_task), Ausgabe fuer die Spektralanalyse ---- */
    {
        sdr_plan_t pl; sdr_plan_rate(fs, shift, 10000000u, &pl);
        printf("Plan: N=%d fs_out=%u exakt=%d nyquist_ok=%d alias=%d\n", pl.n, pl.fs_out, pl.exact, pl.nyquist_ok, pl.alias);
        CHECK(pl.fs_out == 4000000u && pl.n == 16, "Planer: %u", pl.fs_out);
        sdr_dsp_t *d = calloc(1, sizeof *d);
        d->agc = false; d->manual_gain = 0.5f;
        sdr_dsp_cfg_t c = { .bits = 8, .offset_binary = false, .hold = pl.n / 4, .phase_inc = sdr_phase_inc(shift, pl.fs_out), .fast = false };
        sdr_dsp_init(d, &c);
        size_t cap = NIQ * pl.n; int8_t *out = malloc(cap); size_t no = 0;
        for (size_t done = 0; done < split_at; ) { size_t k = split_at - done > 1024 ? 1024 : split_at - done;
            no += sdr_dsp_process(d, ref_out + 2 * done, k, out + no); done += k; }
        sdr_dsp_set_phase_inc(d, sdr_phase_inc(-shift, pl.fs_out));      /* wie MSG_SHIFT mit gleicher Rate: nahtlos */
        for (size_t done = split_at; done < NIQ; ) { size_t k = NIQ - done > 1024 ? 1024 : NIQ - done;
            no += sdr_dsp_process(d, ref_out + 2 * done, k, out + no); done += k; }
        CHECK(no == NIQ * pl.n, "Ausgabelaenge %zu != %zu", no, NIQ * pl.n);
        if (argc > 1) { FILE *f = fopen(argv[1], "wb"); fwrite(out, 1, no, f); fclose(f); printf("%zu Samples -> %s (fs_out %u, split bei %zu)\n", no, argv[1], pl.fs_out, split_at * pl.n); }
        free(out); free(d);
    }

    printf(fails ? "\n%d FEHLER\n" : "\nalle Tests OK\n", fails);
    return fails ? 1 : 0;
}
