/*
 * duc.c - Software-DUC: I/Q-Strom (COHIRADIAStreamer, In-Band-Protokoll) -> Halbband/Rampe/LUT-NCO -> PARLIO
 *
 * Wichtigste Regel: Dieses Modul wird nur im Modus "duc" benutzt. Im Modus "raw" (Standard)
 * laeuft ausschliesslich der urspruengliche Code aus main.c; hier wird dann nichts angefasst.
 *
 * Datenfluss einer Sitzung (= eine TCP-Verbindung auf Port 1234)
 *
 *   net-Task (Aufrufer von duc_serve_client)        dsp-Task (Core 1, Prio 10)
 *   ---------------------------------------        ------------------------------------------
 *   recv() -> duc_proto_feed() -> I/Q-Block   -->  Queue  -->  Konfiguration (S/R) anwenden
 *   (Tags, Befehle, 14 -> 16 Bit)                              sdr_dsp_process() (ggf. 2 Cores)
 *                                                              -> ring_put() -> PSRAM-Ring
 *                                                              PARLIO-DMA liest den Ring endlos
 *
 *   Warum zwei Tasks: das Netz darf waehrend der Rechenzeit weiter Daten einsammeln, und die
 *   In-Band-Befehle bleiben ueber die Queue exakt an ihrer Stelle im Sample-Strom.
 *   Rueckstau: ist der Ring ausreichend gefuellt, schlaeft ring_put(); die Queue fuellt sich,
 *   der net-Task blockiert, TCP bremst den Sender (der Streamer hat zusaetzlich ein eigenes Pacing).
 *
 * Synchronisation Schreibzeiger <-> DMA (aus dem COHI-Player)
 *   Der DMA-Lesezeiger ist im Loop-Modus nicht abfragbar. CPU und PARLIO laufen vom selben
 *   Quarz, also gilt gespielt(t) = (t - t0) * fs_out. Der Schreibzeiger wird auf ca. 60..100 ms
 *   Vorsprung geregelt. Faellt er unter 1 ms, wird veraltetes Ringmaterial mit Stille ueberschrieben
 *   und ein Underrun gezaehlt (nie Altmaterial abspielen).
 */
#include "duc.h"
#include "duc_proto.h"
#include "sdr_dsp.h"
#include "sdr_plan.h"

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <ctype.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_cpu.h"
#include "esp_heap_caps.h"
#include "esp_cache.h"
#include "driver/parlio_tx.h"
#include "lwip/sockets.h"

static const char *TAG = "PARLIOSDR-DUC";

/* ---- Konfiguration ------------------------------------------------------- */
#define RAW_SAMPLES        2048                  /* I/Q-Paare pro Block (<= SDR_BLOCK_MAX * 2)      */
#define RAW_BLOCKS         12                    /* 12 * 8 KiB internes RAM = ca. 24k I/Q-Paare      */
#define RX_BYTES           8192                  /* recv()-Puffer                                    */
#define MSG_DEPTH          (RAW_BLOCKS + 16)     /* Datenbloecke + Platz fuer Befehlsmeldungen       */
#define RING_MAX_BYTES_CAP (8u * 1024 * 1024)
#define RING_MIN_BYTES     (1u * 1024 * 1024)
#define RING_TIME_MS       500
#define LEAD_TARGET_MS     60
#define LEAD_HIGH_MS       100
#define PREFILL_MS         80
#define LEAD_MIN_US        1000
#define DEFAULT_MAXRATE_HZ 10000000u

/* Ab dieser geschaetzten Last (% eines Cores) schaltet "duc cores auto" auf zwei Cores.
 * Gemessen (ESP32-P4, Netz + DSP gleichzeitig): fs=1,25 MSps FAST -> Schaetzung 65 %, real 55 % mit
 * 1 Core = Underruns; 2 Cores = 30 % und sauber. fs=250 kSps HQ -> Schaetzung 21 %, 1 Core genuegt.
 * Der DSP teilt sich Core 1 mit dem Netz-Empfang, deshalb liegt die Schwelle deutlich unter 85 %. */
#ifndef DUC_AUTO_2CORE_PCT
#define DUC_AUTO_2CORE_PCT 50.0f
#endif

/* Teilerlimit des PARLIO-Taktteilers (siehe Kommentar zur Taktauswahl in main.c) */
#define PARLIO_PLL_HZ   160000000u
#define PARLIO_XTAL_HZ   40000000u
#define PARLIO_MAX_DIV   255.0

_Static_assert(RAW_SAMPLES <= 2 * SDR_BLOCK_MAX, "Block darf nur wenige DSP-Teilbloecke ergeben");

/* ---- Typen --------------------------------------------------------------- */
typedef enum { MSG_DATA, MSG_SHIFT, MSG_RATE, MSG_END } msg_type_t;
typedef struct {
    msg_type_t type;
    int        blk;       /* MSG_DATA: Blocknummer                      */
    uint32_t   n_iq;      /* MSG_DATA: Anzahl I/Q-Paare                 */
    uint32_t   val;       /* MSG_SHIFT: FTW (int32 gelesen), MSG_RATE: fs in Hz */
} msg_t;

typedef struct {
    int      width;       /* DAC-Aufloesung 8 | 10 | 12 | 14 | 16 (rechtsbuendig, wie der Streamer) */
    bool     offset;      /* Offset-Binary (unsigned DAC)     */
    bool     agc;
    float    gain;        /* manueller Gain                   */
    uint32_t maxrate;     /* hoechster PARLIO-Takt in Hz      */
    int      cores;       /* 0 = auto, 1, 2                   */
} duc_set_t;

typedef struct {
    parlio_tx_unit_handle_t unit;
    uint8_t  *ring;
    size_t    ring_bytes;
    uint64_t  ring_samples;
    int       bits;                     /* DAC-Aufloesung (8, 10, 12, 14, 16)                 */
    int       width;                    /* PARLIO-Busbreite: 8 bei 8 Bit, sonst 16            */
    int       bps;                      /* Bytes pro Sample im Ring (width / 8)               */
    uint32_t  fs_out;
    uint16_t  silence;
    bool      dma_started;
    int64_t   t0_us;
    uint64_t  s_wr;                     /* absolut geschriebene Samples */
    int64_t   lead_target, lead_high, prefill;
    uint32_t  underruns;
} out_t;

typedef struct {
    uint32_t fs, fs_out;
    double   shift_hz;
    int      n, bits, cores;
    float    load_pct, lead_ms, gain, peak;
    uint32_t underruns;
    bool     output_running;
    bool     configured;
} st_t;

/* ---- Zustand ------------------------------------------------------------- */
static duc_hw_t  s_hw;
static bool      s_inited;
static volatile bool s_enabled;          /* Modus "duc" aktiv                                 */
static volatile bool s_stop;             /* aktuelle Sitzung beenden (Fehler oder Abbruch)     */
static volatile bool s_active;           /* Sitzung laeuft                                     */
static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;

static duc_set_t s_set = { 16, false, true, 0.5f, DEFAULT_MAXRATE_HZ, 0 };   /* width/offset kommen aus duc_hw_t (duc_init) */
static float     s_bus_hint_msps;

static out_t o;                          /* PARLIO-Ausgang; bleibt nach einer Sitzung (Stille) bestehen */
static st_t  s_st;

/* Sitzungszustand (nur waehrend s_active gueltig) */
static struct {
    duc_set_t        cfg;                /* Einstellungen, zu Sitzungsbeginn eingefroren */
    QueueHandle_t    free_q, msg_q;
    EventGroupHandle_t evt;
    uint8_t         *blk[RAW_BLOCKS];
    sdr_dsp_t       *dsp;
    uint8_t         *stage;
    size_t           stage_bytes;
    TaskHandle_t     dsp_hdl, mix_hdl;
    duc_proto_t      proto;
    uint32_t         dropped_iq;         /* Samples, die wegen fehlender/ungueltiger Konfiguration verworfen wurden */
} S;
#define EVT_DSP_DONE BIT0

static inline bool stopping(void) { return s_stop || !s_enabled; }

/* ---- Taktauswahl ---------------------------------------------------------- */
/* Wie in main.c: PLL_F160M, solange der Teiler <= 255 ist, sonst XTAL (40 MHz). */
static bool pick_clock(uint32_t fs_out, parlio_clock_source_t *src, const char **name, double *div)
{
    if (fs_out == 0) return false;
    double dp = (double)PARLIO_PLL_HZ / fs_out, dx = (double)PARLIO_XTAL_HZ / fs_out;
    if (dp >= 1.0 && dp <= PARLIO_MAX_DIV)      { *src = PARLIO_CLK_SRC_PLL_F160M; *name = "PLL_F160M"; *div = dp; return true; }
    if (dx >= 1.0 && dx <= PARLIO_MAX_DIV)      { *src = PARLIO_CLK_SRC_XTAL;      *name = "XTAL";      *div = dx; return true; }
    return false;
}

/* ---- Ring / PARLIO -------------------------------------------------------- */
static inline int64_t played_now(void)
{
    int64_t dt = esp_timer_get_time() - o.t0_us;
    if (dt < 0) dt = 0;
    return (int64_t)(((uint64_t)dt * o.fs_out) / 1000000ULL);
}

static void msync_c2m(void *p, size_t n)
{
    esp_cache_msync(p, n, ESP_CACHE_MSYNC_FLAG_DIR_C2M | ESP_CACHE_MSYNC_FLAG_UNALIGNED);
}

static void fill_code(uint8_t *dst, size_t n_samples, int bits, uint16_t code)
{
    if (bits == 8) memset(dst, (int)code, n_samples);
    else { uint16_t *d = (uint16_t *)dst; for (size_t i = 0; i < n_samples; i++) d[i] = code; }
}

/* Samples [first, first+n) (absolute Zaehlung) mit Stille fuellen. */
static void ring_silence(uint64_t first, uint64_t n)
{
    if (n > o.ring_samples) n = o.ring_samples;
    while (n) {
        uint64_t idx = first % o.ring_samples;
        uint64_t seg = o.ring_samples - idx;
        if (seg > n) seg = n;
        fill_code(o.ring + idx * o.bps, seg, o.width, o.silence);
        msync_c2m(o.ring + idx * o.bps, seg * o.bps);
        first += seg; n -= seg;
    }
}

static void out_close(void)
{
    if (o.unit) {
        parlio_tx_unit_disable(o.unit);
        parlio_del_tx_unit(o.unit);
        o.unit = NULL;
    }
    o.dma_started = false;
}

static esp_err_t out_open(const sdr_plan_t *pl, int bits, bool offset_binary)
{
    parlio_clock_source_t src; const char *cname; double div;
    if (!pick_clock(pl->fs_out, &src, &cname, &div)) {
        ESP_LOGE(TAG, "PARLIO-Takt %.3f MHz nicht erzeugbar (Teiler > %.0f)", pl->fs_out / 1e6, PARLIO_MAX_DIV);
        return ESP_ERR_INVALID_ARG;
    }
    out_close();
    o.bits = bits;
    o.width = bits == 8 ? 8 : 16;      /* 10/12/14 Bit liegen rechtsbuendig im 16-Bit-Bus (D0..D(n-1)) */
    o.bps = o.width / 8;
    o.fs_out = pl->fs_out;
    o.silence = sdr_silence_code(bits, offset_binary);

    size_t cap = s_hw.ring_mem_bytes < RING_MAX_BYTES_CAP ? s_hw.ring_mem_bytes : RING_MAX_BYTES_CAP;
    uint64_t want = (uint64_t)pl->fs_out * o.bps * RING_TIME_MS / 1000;
    if (want > cap) want = cap;
    if (want < RING_MIN_BYTES) want = RING_MIN_BYTES;
    o.ring_bytes = (size_t)(want & ~(uint64_t)4095);
    o.ring_samples = o.ring_bytes / o.bps;
    o.ring = s_hw.ring_mem;

    int64_t fs = pl->fs_out;
    o.lead_target = fs * LEAD_TARGET_MS / 1000;
    o.lead_high   = fs * LEAD_HIGH_MS / 1000;
    o.prefill     = fs * PREFILL_MS / 1000;
    if (o.lead_high   > (int64_t)o.ring_samples / 2) o.lead_high   = o.ring_samples / 2;
    if (o.lead_target > (int64_t)o.ring_samples / 4) o.lead_target = o.ring_samples / 4;
    if (o.prefill     > (int64_t)o.ring_samples / 3) o.prefill     = o.ring_samples / 3;
    o.s_wr = 0;
    o.underruns = 0;

    ring_silence(0, o.ring_samples);

    parlio_tx_unit_config_t cfg = {
        .clk_src = src,
        .data_width = (uint32_t)o.width,
        .clk_in_gpio_num = -1,
        .clk_out_gpio_num = s_hw.clk_pin,
        .valid_gpio_num = -1,
        .output_clk_freq_hz = pl->fs_out,
        .trans_queue_depth = 2,
        .max_transfer_size = o.ring_bytes,
    };
    for (int i = 0; i < 16; i++) cfg.data_gpio_nums[i] = (i < bits) ? s_hw.data_pins[i] : -1;   /* nur SD0..SD(n-1): ein 10-Bit-DAC belegt nur SD0..SD9, die Pins darueber bleiben frei */

    esp_err_t e = parlio_new_tx_unit(&cfg, &o.unit);
    if (e != ESP_OK) { ESP_LOGE(TAG, "parlio_new_tx_unit: %s", esp_err_to_name(e)); o.unit = NULL; return e; }
    e = parlio_tx_unit_enable(o.unit);
    if (e != ESP_OK) { ESP_LOGE(TAG, "parlio enable: %s", esp_err_to_name(e)); out_close(); return e; }
    ESP_LOGI(TAG, "PARLIO: %d Bit, %.3f MSPS (Takt: %s, Teiler %.2f), Ring %u KiB",
             bits, pl->fs_out / 1e6, cname, div, (unsigned)(o.ring_bytes / 1024));
    if (bits != 8 && bits != 16)
        ESP_LOGI(TAG, "%d-Bit-DAC rechtsbuendig an D0..D%d (16-Bit-Bus, wie im COHIRADIAStreamer)", bits, bits - 1);
    return ESP_OK;
}

static esp_err_t out_start_dma(void)
{
    if (o.dma_started || !o.unit) return ESP_OK;
    parlio_transmit_config_t tc = { .idle_value = o.silence };
    tc.flags.loop_transmission = 1;
    o.t0_us = esp_timer_get_time();
    esp_err_t e = parlio_tx_unit_transmit(o.unit, o.ring, o.ring_bytes * 8, &tc);
    if (e != ESP_OK) { ESP_LOGE(TAG, "parlio transmit: %s", esp_err_to_name(e)); return e; }
    o.dma_started = true;
    return ESP_OK;
}

static bool sleep_ms_stoppable(int ms)
{
    while (ms > 0 && !stopping()) {
        int d = ms > 10 ? 10 : ms;
        vTaskDelay(pdMS_TO_TICKS(d) ? pdMS_TO_TICKS(d) : 1);
        ms -= d;
    }
    return !stopping();
}

/* n Samples (aus Stage) in den Ring schreiben; bremst auf lead_target/high. */
static void ring_put(const uint8_t *src, size_t n)
{
    if (o.dma_started) {
        for (;;) {
            int64_t lead = (int64_t)o.s_wr - played_now();
            if (lead <= o.lead_high || stopping()) break;
            int ms = (int)((lead - o.lead_target) * 1000 / o.fs_out);
            if (ms < 1) ms = 1;
            if (ms > 20) ms = 20;
            vTaskDelay(pdMS_TO_TICKS(ms) ? pdMS_TO_TICKS(ms) : 1);   /* tick-unabhaengig (FREERTOS_HZ=100 -> 10 ms) */
        }
        int64_t pn = played_now();
        int64_t lead = (int64_t)o.s_wr - pn;
        if (lead * 1000000 / (int64_t)o.fs_out < LEAD_MIN_US) {          /* Underrun */
            uint64_t neu = (uint64_t)(pn + o.lead_target / 2);
            ring_silence(o.s_wr, neu - o.s_wr);                           /* Veraltetes (letzte Umdrehung) loeschen */
            o.underruns++;
            o.s_wr = neu;
        }
    }
    while (n && !stopping()) {
        uint64_t idx = o.s_wr % o.ring_samples;
        uint64_t seg = o.ring_samples - idx;
        if (seg > n) seg = n;
        memcpy(o.ring + idx * o.bps, src, seg * o.bps);
        msync_c2m(o.ring + idx * o.bps, seg * o.bps);
        o.s_wr += seg; src += seg * o.bps; n -= seg;
    }
    if (!o.dma_started && (int64_t)o.s_wr >= o.prefill) out_start_dma();
}

static void drain_and_wait(void)
{
    /* restliche (noch nicht abgespielte) Daten ausspielen lassen */
    if (!o.dma_started) out_start_dma();
    for (;;) {
        int64_t lead = (int64_t)o.s_wr - played_now();
        if (lead <= 0 || stopping()) break;
        if (!sleep_ms_stoppable((int)(lead * 1000 / o.fs_out) + 2)) break;
    }
}

/* ---- Mischer-Worker (Core 0) fuer die Zwei-Core-Variante ------------------ */
typedef struct { const sdr_dsp_t *d; size_t first, count; void *out; } mixjob_t;
static volatile mixjob_t s_job;

static void mix_worker(void *arg)
{
    (void)arg;
    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        mixjob_t j = { s_job.d, s_job.first, s_job.count, s_job.out };
        sdr_dsp_mix(j.d, j.first, j.count, j.out);
        xTaskNotifyGive(S.dsp_hdl);
    }
}

/* ---- DSP-Task ------------------------------------------------------------- */
typedef struct {
    bool     first_cfg;       /* noch keine Konfiguration in dieser Sitzung */
    bool     valid;           /* DSP-Kontext passt zur aktuellen Konfiguration */
    uint32_t fs;
    int      n;
    int      hold, bps, B;
    bool     use2;
    float    split;
} dctx_t;

/* Plant Rate/Ausgang zu (fs, shift) und richtet PARLIO + DSP ein. false = Daten verwerfen. */
static bool apply_cfg(dctx_t *c, uint32_t fs, double shift)
{
    sdr_plan_t pl;
    if (fs == 0) return false;
    if (sdr_plan_rate(fs, shift, S.cfg.maxrate, &pl) != 0) {
        ESP_LOGE(TAG, "fs=%u Hz zu hoch fuer maxrate=%u Hz (4*fs > maxrate)", (unsigned)fs, (unsigned)S.cfg.maxrate);
        return false;
    }
    const int bits = S.cfg.width;
    const bool first = c->first_cfg;
    const bool need_open = first || !o.unit || o.fs_out != pl.fs_out || o.bits != bits;
    c->first_cfg = false;

    if (need_open) {
        /* Beim Sitzungsbeginn laeuft nur Stille aus der Vorsitzung: kein Ausspielen noetig.
         * Bei Ratenwechsel mitten im Strom erst den Rest des alten Signals ausspielen. */
        if (!first && o.unit && o.dma_started) drain_and_wait();
        if (stopping()) return false;
        if (out_open(&pl, bits, S.cfg.offset) != ESP_OK) { s_stop = true; return false; }
    }
    if (!c->valid || c->fs != fs || c->n != pl.n || need_open) {
        const bool fast = fs >= SDR_FAST_FS_HZ;
        sdr_dsp_cfg_t dc = { .bits = bits, .offset_binary = S.cfg.offset, .hold = fast ? pl.n / 2 : pl.n / 4,
                             .phase_inc = sdr_phase_inc(shift, pl.fs_out), .fast = fast };
        S.dsp->agc = S.cfg.agc; S.dsp->manual_gain = S.cfg.gain;
        sdr_dsp_init(S.dsp, &dc);
        c->valid = true; c->fs = fs; c->n = pl.n;
        c->hold = dc.hold; c->bps = (bits == 8) ? 1 : 2;
    } else {
        /* gleiche Rate, nur anderer Shift: nahtlos, Phase und Filterzustand bleiben erhalten */
        sdr_dsp_set_phase_inc(S.dsp, sdr_phase_inc(shift, pl.fs_out));
    }

    c->B = SDR_BLOCK_MAX;
    while (c->B > 16 && (size_t)c->B * pl.n > 8192) c->B >>= 1;
    size_t need = (size_t)c->B * pl.n * (size_t)((bits == 8) ? 1 : 2);
    if (need > S.stage_bytes) {
        free(S.stage);
        S.stage = heap_caps_aligned_alloc(64, need, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        S.stage_bytes = S.stage ? need : 0;
        if (!S.stage) { ESP_LOGE(TAG, "kein Speicher fuer Stage"); s_stop = true; return false; }
    }

    const bool fast = fs >= SDR_FAST_FS_HZ;
    const uint32_t cpu_mhz = CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ;
    ESP_LOGI(TAG, "fs=%u Hz, Shift=%.3f kHz, N=%d -> PARLIO %.3f MHz, %d Bit%s",
             (unsigned)fs, shift / 1e3, pl.n, pl.fs_out / 1e6, bits,
             pl.exact ? ", exakter Teiler" : ", BRUCHTEILER (Jitter, Rate evtl. ungenau!)");
    if (pl.alias)
        ESP_LOGW(TAG, "ACHTUNG: Fs_out < 2*fmax - Nutzsignal liegt nur als Bild/Alias vor (Alias-Frequenz %.3f MHz); Bandpass noetig",
                 pl.alias_hz / 1e6);
    else if (!pl.nyquist_ok)
        ESP_LOGW(TAG, "Hinweis: Abstand zu Nyquist gering (Fs_out < %.1f x fmax)", SDR_NYQ_FACTOR);

    float est = sdr_est_load_pct(fs, pl.fs_out, fast, cpu_mhz);
    ESP_LOGI(TAG, "Filter: %s, geschaetzte DSP-Last %.0f %% eines Cores", fast ? "FAST (1 Halbband)" : "HQ (2 Halbbaender)", est);
    c->use2 = (S.cfg.cores == 2) || (S.cfg.cores == 0 && est >= DUC_AUTO_2CORE_PCT);
    c->split = 0.0f;
    if (c->use2) {
        c->split = sdr_est_split(fs, pl.fs_out, fast);
        double fc_, mc_;
        sdr_est_cycles(fs, pl.fs_out, fast, &fc_, &mc_);
        float l1 = (float)((fc_ + c->split * mc_) / (cpu_mhz * 1e6) * 100.0);
        float l0 = (float)(((1.0f - c->split) * mc_) / (cpu_mhz * 1e6) * 100.0);
        ESP_LOGI(TAG, "2 Cores: Core 1 = Filter + %.0f %% Mischer (~%.0f %%), Core 0 = %.0f %% Mischer (~%.0f %%)",
                 c->split * 100.0f, l1, (1.0f - c->split) * 100.0f, l0);
        if (l1 > 90.0f || l0 > 90.0f) ESP_LOGW(TAG, "Auch mit 2 Cores zu hohe Last - Underruns wahrscheinlich");
    } else if (est > 85.0f) {
        ESP_LOGW(TAG, "Last > 85 %%: Underruns sehr wahrscheinlich ('duc cores 2' oder niedrigeres 'duc maxrate')");
    }

    portENTER_CRITICAL(&s_mux);
    s_st.fs = fs; s_st.fs_out = pl.fs_out; s_st.shift_hz = shift; s_st.n = pl.n; s_st.bits = bits;
    s_st.cores = c->use2 ? 2 : 1; s_st.configured = true;
    portEXIT_CRITICAL(&s_mux);
    return true;
}

static void dsp_task(void *arg)
{
    (void)arg;
    dctx_t c = { .first_cfg = true };
    uint32_t pend_fs = 0;
    double   pend_shift = 0.0;
    bool     dirty = false, cfg_ok = false, warned_nocfg = false;
    bool     natural_end = false;
    int64_t  win_start = esp_timer_get_time(), last_rest = win_start;
    uint64_t busy64 = 0;
    const uint32_t cpu_mhz = CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ;
    S.dsp_hdl = xTaskGetCurrentTaskHandle();

    for (;;) {
        msg_t m;
        if (xQueueReceive(S.msg_q, &m, pdMS_TO_TICKS(100)) != pdTRUE) {
            if (stopping()) break;
            continue;
        }
        if (m.type == MSG_END)   { natural_end = true; break; }
        if (m.type == MSG_SHIFT) { pend_shift = duc_ftw_to_hz_signed(m.val); dirty = true; continue; }
        if (m.type == MSG_RATE)  { pend_fs = m.val; dirty = true; continue; }

        /* ---- MSG_DATA ---- */
        const int blk = m.blk;
        if (dirty) {                       /* S und R stehen meist direkt hintereinander: erst hier gemeinsam umsetzen */
            cfg_ok = apply_cfg(&c, pend_fs, pend_shift);
            dirty = false;
            warned_nocfg = false;
        }
        if (!cfg_ok) {
            if (!warned_nocfg && !stopping()) {
                ESP_LOGW(TAG, "I/Q-Daten ohne gueltige Samplerate/Konfiguration - verworfen (sendet der Streamer 'R'?)");
                warned_nocfg = true;
            }
            S.dropped_iq += m.n_iq;
            xQueueSend(S.free_q, &blk, 0);
            if (stopping()) break;
            continue;
        }

        const int16_t *iq = (const int16_t *)S.blk[blk];
        size_t done = 0; const size_t n = m.n_iq;
        while (done < n && !stopping()) {
            size_t k = n - done < (size_t)c.B ? n - done : (size_t)c.B;
            size_t outn;
            uint32_t c0 = esp_cpu_get_cycle_count();
            if (!c.use2) {
                outn = sdr_dsp_process(S.dsp, iq + 2 * done, k, S.stage);
                busy64 += (uint32_t)(esp_cpu_get_cycle_count() - c0);    /* reine DSP-Zeit (ohne Pacing-Schlaf) */
            } else {
                size_t nbb = sdr_dsp_filter(S.dsp, iq + 2 * done, k);
                /* 64er-Raster: Teilbereiche liegen auf Cacheline-Grenzen im Stage-Puffer */
                size_t j0 = nbb < 256 ? nbb : ((size_t)((float)nbb * c.split)) & ~(size_t)63;
                if (j0 < nbb) {
                    s_job.d = S.dsp; s_job.first = j0; s_job.count = nbb - j0;
                    s_job.out = S.stage + j0 * (size_t)c.hold * (size_t)c.bps;
                    xTaskNotifyGive(S.mix_hdl);
                }
                if (j0) sdr_dsp_mix(S.dsp, 0, j0, S.stage);
                busy64 += (uint32_t)(esp_cpu_get_cycle_count() - c0);
                if (j0 < nbb) ulTaskNotifyTake(pdTRUE, portMAX_DELAY);    /* Wartezeit zaehlt nicht als Last */
                sdr_dsp_commit(S.dsp, nbb);
                outn = nbb * (size_t)c.hold;
            }
            ring_put(S.stage, outn);
            done += k;
        }
        xQueueSend(S.free_q, &blk, 0);

        int64_t now = esp_timer_get_time();
        if (now - last_rest > 100000) {        /* im Rueckstand nie dauerhaft durchlaufen: IDLE-Task/Watchdog braucht Zeit */
            vTaskDelay(1);
            last_rest = esp_timer_get_time();
        }
        if (now - win_start > 500000) {
            float load = (float)((double)busy64 / ((double)(now - win_start) * cpu_mhz)) * 100.0f;
            float lead = o.dma_started ? (float)(((int64_t)o.s_wr - played_now()) * 1000.0 / o.fs_out) : 0.0f;
            portENTER_CRITICAL(&s_mux);
            s_st.load_pct = load; s_st.lead_ms = lead; s_st.underruns = o.underruns;
            s_st.gain = S.dsp->gain; s_st.peak = S.dsp->last_peak; s_st.output_running = o.dma_started;
            portEXIT_CRITICAL(&s_mux);
            win_start = now; busy64 = 0;
        }
    }

    /* Ende: normal (Client getrennt) -> Rest ausspielen, danach Stille; Abbruch -> sofort Stille */
    if (o.unit) {
        if (natural_end && !stopping()) {
            if (!o.dma_started) out_start_dma();
            int64_t lead = (int64_t)o.s_wr - played_now();
            uint64_t free_s = o.ring_samples - (lead > 0 ? lead : 0) - (o.fs_out / 500);
            ring_silence(o.s_wr, free_s);      /* Stille hinter den letzten Daten, restlicher Vorsprung wird noch abgespielt */
        } else {
            ring_silence(0, o.ring_samples);   /* sofort Stille */
        }
        portENTER_CRITICAL(&s_mux);
        s_st.output_running = o.dma_started; s_st.underruns = o.underruns;
        portEXIT_CRITICAL(&s_mux);
    }
    xEventGroupSetBits(S.evt, EVT_DSP_DONE);
    vTaskDelete(NULL);
}

/* ---- Sitzung (net-Task-Seite) --------------------------------------------- */
static void session_free(void)
{
    for (int i = 0; i < RAW_BLOCKS; i++) { free(S.blk[i]); S.blk[i] = NULL; }
    free(S.dsp);   S.dsp = NULL;
    free(S.stage); S.stage = NULL; S.stage_bytes = 0;
    if (S.free_q) vQueueDelete(S.free_q);
    if (S.msg_q)  vQueueDelete(S.msg_q);
    if (S.evt)    vEventGroupDelete(S.evt);
    S.free_q = S.msg_q = NULL; S.evt = NULL;
}

static bool session_alloc(void)
{
    memset(&S, 0, sizeof S);
    S.dsp = heap_caps_malloc(sizeof(sdr_dsp_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    S.free_q = xQueueCreate(RAW_BLOCKS, sizeof(int));
    S.msg_q  = xQueueCreate(MSG_DEPTH, sizeof(msg_t));
    S.evt    = xEventGroupCreate();
    bool ok = S.dsp && S.free_q && S.msg_q && S.evt;
    for (int i = 0; ok && i < RAW_BLOCKS; i++) {
        S.blk[i] = heap_caps_malloc((size_t)RAW_SAMPLES * 4, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        ok = S.blk[i] != NULL;
    }
    if (!ok) { session_free(); return false; }
    for (int i = 0; i < RAW_BLOCKS; i++) xQueueSend(S.free_q, &i, 0);
    memset(S.dsp, 0, sizeof *S.dsp);
    return true;
}

static bool send_msg(const msg_t *m)
{
    while (!stopping()) {
        if (xQueueSend(S.msg_q, m, pdMS_TO_TICKS(50)) == pdTRUE) return true;
    }
    return false;
}

static bool get_block(int *blk)
{
    while (!stopping()) {
        if (xQueueReceive(S.free_q, blk, pdMS_TO_TICKS(50)) == pdTRUE) return true;
    }
    return false;
}

void duc_serve_client(int sock)
{
    if (!s_inited || !s_enabled || s_active) return;
    if (!session_alloc()) { ESP_LOGE(TAG, "kein internes RAM fuer die DUC-Sitzung"); return; }

    portENTER_CRITICAL(&s_mux);
    S.cfg = s_set;                                 /* Einstellungen einfrieren (wirken ab der naechsten Verbindung) */
    const float hint = s_bus_hint_msps;
    memset(&s_st, 0, sizeof s_st);
    portEXIT_CRITICAL(&s_mux);
    duc_proto_init(&S.proto);
    s_stop = false;
    s_active = true;

    uint8_t *rx = heap_caps_malloc(RX_BYTES, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (!rx) { ESP_LOGE(TAG, "kein RX-Puffer"); s_active = false; session_free(); return; }

    xTaskCreatePinnedToCore(dsp_task, "duc_dsp", 6144, NULL, 10, NULL, 1);
    xTaskCreatePinnedToCore(mix_worker, "duc_mix", 3072, NULL, 11, &S.mix_hdl, 0);

    ESP_LOGI(TAG, "Software-DUC: Daten-Client verbunden (%d Bit, %s, %s, maxrate %.1f MHz)",
             S.cfg.width, S.cfg.offset ? "Offset-Binary" : "Zweierkomplement",
             S.cfg.agc ? "AGC" : "fester Gain", S.cfg.maxrate / 1e6);

    /* Rueckfall: Rate aus dem klassischen "rate"-Befehl (Busrate = 2*fs). Ein 'R' im Strom ueberschreibt sie. */
    if (hint > 0.0f) {
        msg_t r = { .type = MSG_RATE, .val = (uint32_t)llround((double)hint * 1e6 / 2.0) };
        send_msg(&r);
    }

    int blk = -1;
    int16_t *cur = NULL;
    size_t fill = 0;
    uint32_t unknown_logged = 0;

    while (!stopping()) {
        int n = recv(sock, rx, RX_BYTES, 0);
        if (n <= 0) break;                         /* Client getrennt (oder per shutdown() abgebrochen) */

        size_t off = 0;
        while (off < (size_t)n && !stopping()) {
            if (!cur) {
                if (!get_block(&blk)) break;
                cur = (int16_t *)S.blk[blk]; fill = 0;
            }
            size_t used = 0, got = 0;
            duc_event_t ev;
            duc_feed_t r = duc_proto_feed(&S.proto, rx + off, (size_t)n - off, &used,
                                          cur + 2 * fill, RAW_SAMPLES - fill, &got, &ev);
            off += used; fill += got;

            /* Daten vor einem Befehl oder bei vollem Block abliefern (Reihenfolge Daten <-> Befehl bleibt erhalten) */
            if (fill > 0 && (r == DUC_FEED_FULL || r == DUC_FEED_EVENT || off >= (size_t)n)) {
                msg_t d = { .type = MSG_DATA, .blk = blk, .n_iq = (uint32_t)fill };
                if (!send_msg(&d)) { cur = NULL; break; }
                cur = NULL; fill = 0;
            }
            if (r == DUC_FEED_EVENT) {
                msg_t c = { 0 };
                switch (ev.type) {
                case DUC_EV_SHIFT: c.type = MSG_SHIFT; c.val = ev.value; if (!send_msg(&c)) goto done; break;
                case DUC_EV_RATE:  c.type = MSG_RATE;  c.val = duc_ftw_to_rate_hz(ev.value); if (!send_msg(&c)) goto done; break;
                case DUC_EV_UNKNOWN:
                    if (unknown_logged++ < 5) ESP_LOGW(TAG, "unbekannter In-Band-Befehl '%c' ignoriert", ev.cmd >= 32 && ev.cmd < 127 ? ev.cmd : '?');
                    break;
                default:
                    if (unknown_logged++ < 5) ESP_LOGW(TAG, "defekter In-Band-Befehl ignoriert");
                    break;
                }
            }
        }
    }
done:
    /* Block zurueckgeben, falls einer offen ist (kein Verlust: nur Teilblock ohne Daten) */
    if (cur && fill == 0) xQueueSend(S.free_q, &blk, 0);

    ESP_LOGI(TAG, "Daten-Client getrennt (%llu I/Q-Paare, %u Befehle, Verwerfungen I/Q/Befehl: %u/%u/%u)",
             (unsigned long long)S.proto.n_pairs, (unsigned)S.proto.n_cmds,
             (unsigned)S.proto.drop_i, (unsigned)S.proto.drop_q, (unsigned)S.proto.n_bad);

    /* DSP-Task beenden: normal -> restliche Daten ausspielen lassen; Abbruch -> sofort */
    if (!stopping()) { msg_t e = { .type = MSG_END }; send_msg(&e); }
    else s_stop = true;
    for (int waited = 0; waited < 100; waited++) {                       /* max. 10 s */
        if (xEventGroupWaitBits(S.evt, EVT_DSP_DONE, pdFALSE, pdTRUE, pdMS_TO_TICKS(100)) & EVT_DSP_DONE) break;
        if (!s_enabled) s_stop = true;
    }
    s_stop = true;                                                       /* DSP-Task sicher beenden */
    xEventGroupWaitBits(S.evt, EVT_DSP_DONE, pdFALSE, pdTRUE, pdMS_TO_TICKS(2000));

    if (S.mix_hdl) vTaskDelete(S.mix_hdl);
    free(rx);
    session_free();
    s_active = false;
}

/* ---- API ------------------------------------------------------------------- */
esp_err_t duc_init(const duc_hw_t *hw)
{
    if (!hw || !hw->data_pins || !hw->ring_mem || hw->ring_mem_bytes < RING_MIN_BYTES) return ESP_ERR_INVALID_ARG;
    if (hw->dac_bits != 8 && hw->dac_bits != 10 && hw->dac_bits != 12 && hw->dac_bits != 14 && hw->dac_bits != 16)
        return ESP_ERR_INVALID_ARG;
    s_hw = *hw;
    s_set.width = hw->dac_bits;                /* fest: nur der Kompilierwert, keine Umschaltung zur Laufzeit */
    s_set.offset = hw->dac_offset_binary;
    sdr_dsp_global_init();          /* LUT + Filterkoeffizienten (ca. 20 KB, einmalig) */
    s_inited = true;
    return ESP_OK;
}

void duc_set_enabled(bool en)
{
    s_enabled = en;                 /* false: laufende Sitzung bricht ab (stopping()) */
}

bool duc_is_active(void) { return s_active; }

bool duc_wait_idle(uint32_t timeout_ms)
{
    for (uint32_t t = 0; s_active && t <= timeout_ms; t += 10) vTaskDelay(pdMS_TO_TICKS(10) ? pdMS_TO_TICKS(10) : 1);
    return !s_active;
}

void duc_release_output(void)
{
    if (s_active) return;
    out_close();
    portENTER_CRITICAL(&s_mux);
    s_st.output_running = false; s_st.configured = false;
    portEXIT_CRITICAL(&s_mux);
}

void duc_set_bus_rate_hint(float msps) { s_bus_hint_msps = msps > 0.0f ? msps : 0.0f; }

/* ---- Textbefehle "duc ..." --------------------------------------------------- */
static void status_text(char *reply, size_t n)
{
    st_t st; duc_set_t set; float hint;
    portENTER_CRITICAL(&s_mux);
    st = s_st; set = s_set; hint = s_bus_hint_msps;
    portEXIT_CRITICAL(&s_mux);
    int w = snprintf(reply, n, "duc: %s, Sitzung %s | Einstellungen: DAC %d Bit (fest), %s, %s %.2f, cores %s, maxrate %.1f MHz, rate-hint %.1f MSPS\n",
                     s_enabled ? "aktiv" : "aus", s_active ? "laeuft" : "keine",
                     set.width, set.offset ? "offset-binary" : "signed", set.agc ? "AGC" : "gain", set.gain,
                     set.cores == 0 ? "auto" : set.cores == 1 ? "1" : "2", set.maxrate / 1e6, hint);
    if (w > 0 && (size_t)w < n && s_active && st.configured)
        snprintf(reply + w, n - (size_t)w,
                 "     fs=%u Hz, shift=%.3f kHz, N=%d, PARLIO %.3f MHz, %d Bit, %d Core(s), Last %.0f%%, Vorsprung %.1f ms, Underruns %u, Gain %.2f, Peak %.2f, I/Q-Paare %llu, verworfen %u\n",
                 (unsigned)st.fs, st.shift_hz / 1e3, st.n, st.fs_out / 1e6, st.bits, st.cores, st.load_pct, st.lead_ms,
                 (unsigned)st.underruns, st.gain, st.peak, (unsigned long long)S.proto.n_pairs, (unsigned)S.dropped_iq);
}

void duc_command(const char *line, char *reply, size_t n)
{
    char buf[64];
    strlcpy(buf, line, sizeof buf);
    for (char *p = buf; *p; p++) *p = (char)tolower((unsigned char)*p);
    char *save = NULL;
    char *t0 = strtok_r(buf, " \t\r\n", &save);          /* "duc" */
    char *t1 = strtok_r(NULL, " \t\r\n", &save);
    char *t2 = strtok_r(NULL, " \t\r\n", &save);
    (void)t0;
    const char *ok = "ok (wirkt ab der naechsten Verbindung)\n";

    if (!t1 || strcmp(t1, "status") == 0) {
        status_text(reply, n);
    } else if (strcmp(t1, "bits") == 0 || strcmp(t1, "width") == 0) {
        snprintf(reply, n, "DAC-Breite ist fest: %d Bit (PARLIOSDR_DAC_BITS beim Kompilieren)\n", s_set.width);
    } else if (strcmp(t1, "offset") == 0 && t2 && (strcmp(t2, "on") == 0 || strcmp(t2, "off") == 0)) {
        s_set.offset = strcmp(t2, "on") == 0;
        snprintf(reply, n, "%s", ok);
    } else if (strcmp(t1, "gain") == 0 && t2) {
        bool agc = strcmp(t2, "auto") == 0;
        float g = agc ? 0.0f : (float)atof(t2);
        if (!agc && !(g >= 0.01f && g <= 4.0f)) { snprintf(reply, n, "Fehler: duc gain auto | 0.01..4.0\n"); return; }
        portENTER_CRITICAL(&s_mux);
        s_set.agc = agc; if (!agc) s_set.gain = g;
        portEXIT_CRITICAL(&s_mux);
        if (s_active && S.dsp) { S.dsp->agc = agc; if (!agc) S.dsp->manual_gain = g; }   /* live, wird blockweise gelesen */
        snprintf(reply, n, "ok (live)\n");
    } else if (strcmp(t1, "cores") == 0 && t2 && (strcmp(t2, "auto") == 0 || strcmp(t2, "1") == 0 || strcmp(t2, "2") == 0)) {
        s_set.cores = strcmp(t2, "auto") == 0 ? 0 : atoi(t2);
        snprintf(reply, n, "%s", ok);
    } else if (strcmp(t1, "maxrate") == 0 && t2) {
        double mhz = atof(t2);
        if (!(mhz >= 0.5 && mhz <= 40.0)) { snprintf(reply, n, "Fehler: duc maxrate 0.5..40 (MHz)\n"); return; }
        s_set.maxrate = (uint32_t)(mhz * 1e6 + 0.5);
        snprintf(reply, n, "%s", ok);
    } else {
        snprintf(reply, n, "duc status | offset on|off | gain auto|<0.01..4> | cores auto|1|2 | maxrate <MHz>\n");
    }
}
