/*
 * duc_proto.h - Parser fuer den In-Band-Datenstrom des COHIRADIAStreamer (I/Q-Engine)
 *
 * Der Streamer (DspWorker::run_IQ_engine) schickt auf Port 1234 einen Strom aus
 * 16-Bit-Woertern (little endian). Die oberen zwei Bit sind ein Tag:
 *
 *   Bit 15:14 = 00  I-Sample, Bit 13:0 = 14-Bit-Zweierkomplement
 *               01  Q-Sample, Bit 13:0 = 14-Bit-Zweierkomplement
 *               10  Befehlswort: Bit 13:8 = Index, Bit 7:0 = Nutzbyte
 *                     Index 63 = Befehl beginnen (Nutzbyte = Befehlsbuchstabe)
 *                     Index 0..3 = Datenbyte 0..3 (LSB zuerst)
 *               11  Ausfuehren ('E')
 *
 * Befehle: 'S' = NCO-Frequenz (FTW), 'R' = Eingangs-Samplerate (FTW).
 * FTW = round(f / DUC_FPGA_REF_HZ * 2^32) - der FPGA rechnet mit 50 MHz.
 *
 * Mit dem FPGA wird dieser Strom 1:1 durchgereicht. Beim Software-DUC wertet der
 * ESP32 ihn selbst aus. Dieses Modul ist reines C ohne IDF-Abhaengigkeit und
 * laeuft deshalb auch im Host-Test (test/).
 *
 * Eigenschaften:
 *  - TCP liefert beliebige Stuecke: ungerade Byteanzahlen werden zwischengespeichert,
 *    ein einzelnes Wort darf ueber zwei Aufrufe verteilt sein.
 *  - I/Q werden ueber ihre Tags gepaart (nicht ueber die Position), ein verlorenes
 *    Wort bringt den Strom nicht dauerhaft aus dem Takt.
 *  - Befehle wirken genau an der Stelle im Strom, an der sie stehen: feed() kehrt
 *    unmittelbar nach einem ausgefuehrten Befehl zurueck, die davor dekodierten
 *    Samples liegen schon im Ausgabepuffer.
 */
#pragma once
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Referenztakt der FTW-Berechnung im Streamer (DspWorker::calculate_ftw, 50 MHz FPGA-Takt). */
#ifndef DUC_FPGA_REF_HZ
#define DUC_FPGA_REF_HZ 50000000.0
#endif

#define DUC_TAG_I    0u
#define DUC_TAG_Q    1u
#define DUC_TAG_CMD  2u
#define DUC_TAG_EXEC 3u
#define DUC_CMD_BEGIN_INDEX 63u

typedef enum {
    DUC_EV_SHIFT,     /* 'S': value = FTW der NCO-Frequenz           */
    DUC_EV_RATE,      /* 'R': value = FTW der Eingangs-Samplerate    */
    DUC_EV_UNKNOWN,   /* anderer, vollstaendiger Befehl (cmd gesetzt) */
    DUC_EV_BAD        /* Execute ohne gueltigen Befehl / Daten unvollstaendig */
} duc_ev_type_t;

typedef struct {
    duc_ev_type_t type;
    uint32_t      value;
    uint8_t       cmd;
} duc_event_t;

typedef enum {
    DUC_FEED_DONE,    /* alle Eingabebytes verbraucht                         */
    DUC_FEED_FULL,    /* Ausgabepuffer voll (Eingabe evtl. noch nicht leer)   */
    DUC_FEED_EVENT    /* ein Befehl wurde ausgefuehrt, *ev ist gesetzt        */
} duc_feed_t;

typedef struct {
    /* Byte-Zwischenspeicher (ungerade Stuecke) */
    bool     have_lo;
    uint8_t  lo;
    /* halbfertiges I/Q-Paar */
    bool     have_i;
    int16_t  i_val;
    /* Befehlsaufbau */
    bool     cmd_valid;
    uint8_t  cmd;
    uint8_t  data[4];
    uint8_t  data_mask;
    /* Zaehler (Diagnose, `duc status`) */
    uint64_t n_pairs;
    uint32_t n_cmds;
    uint32_t drop_i;      /* I ueberschrieben, weil kein Q folgte  */
    uint32_t drop_q;      /* Q ohne vorangehendes I                */
    uint32_t n_bad;       /* defekte Befehle                       */
} duc_proto_t;

void duc_proto_init(duc_proto_t *p);

/*
 * Verarbeitet bis zu n Eingabebytes.
 *   *used   : tatsaechlich verbrauchte Eingabebytes
 *   iq      : Ausgabepuffer (I,Q,I,Q,... als int16, auf 16 Bit hochskaliert = 14-Bit-Wert << 2)
 *   iq_cap  : Platz im Ausgabepuffer in I/Q-PAAREN
 *   *n_iq   : (Ausgabe) neu geschriebene I/Q-Paare
 */
duc_feed_t duc_proto_feed(duc_proto_t *p, const uint8_t *in, size_t n, size_t *used,
                          int16_t *iq, size_t iq_cap, size_t *n_iq, duc_event_t *ev);

/* FTW -> Hertz. Die Frequenz ist vorzeichenbehaftet (FTW als int32 gelesen), die Samplerate nicht. */
double   duc_ftw_to_hz_signed(uint32_t ftw);
uint32_t duc_ftw_to_rate_hz(uint32_t ftw);

#ifdef __cplusplus
}
#endif
