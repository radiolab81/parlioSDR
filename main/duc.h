/*
 * duc.h - Software-DUC fuer parlioSDR (Ersatz des FPGA im I/Q-Betrieb)
 *
 * Ueberblick
 *   Im Modus "raw" (Standard, unveraendertes Verhalten) wird der TCP-Strom auf Port 1234
 *   1:1 in den PARLIO-Ring geschrieben (fertige HF-Samples oder I/Q fuer einen FPGA).
 *   Im Modus "duc" erwartet parlioSDR denselben I/Q-Strom, den sonst der FPGA bekommt
 *   (COHIRADIAStreamer, I/Q-Engine, In-Band-Befehle 'S' = Shift, 'R' = Samplerate),
 *   und rechnet das Hochmischen selbst:
 *
 *     TCP (net-Task) -> duc_proto (Tags, Befehle) -> I/Q-Bloecke -> Queue
 *       -> DSP-Task: Halbband-Interpolation, Rampe, LUT-NCO -> Ring (PSRAM) -> PARLIO-DMA
 *
 *   Der DSP-Kern (sdr_dsp.c) und die Ratenwahl (sdr_plan.c) sind unveraendert aus dem
 *   COHI-Player-ESP32 uebernommen, dort auf Hardware erprobt.
 *
 * Umschalten (Steuerport 5000, siehe main.c):
 *   mode duc | mode raw | mode        (Status)
 *   duc status | duc offset on|off | duc gain auto|<g> |
 *   duc cores auto|1|2 | duc maxrate <MHz>
 * Die DAC-Breite ist NICHT umschaltbar, sie steht beim Kompilieren fest (PARLIOSDR_DAC_BITS).
 */
#pragma once
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    const int *data_pins;      /* 16 Eintraege SD0..SD15 (aus dem Board-Header)   */
    int        dac_bits;       /* fest verbauter DAC: 8, 10, 12, 14 oder 16 Bit, rechtsbuendig an SD0..SD(n-1)
                                * (Kompilierzeit, PARLIOSDR_DAC_BITS in main.c). Der Streamer sendet im I/Q-Modus
                                * immer "width 16" und 14-Bit-Nutzdaten - das sagt nichts ueber diesen DAC.   */
    bool       dac_offset_binary; /* Startwert fuer "duc offset" (PARLIOSDR_DAC_OFFSET_BINARY)            */
    int        clk_pin;        /* DAC_CLK_PIN                                      */
    uint8_t   *ring_mem;       /* DMA-faehiger PSRAM-Puffer (64-Byte ausgerichtet) */
    size_t     ring_mem_bytes;
} duc_hw_t;

/* Einmalig beim Start. Reserviert keinen Speicher (der kommt erst mit einer Sitzung). */
esp_err_t duc_init(const duc_hw_t *hw);

/* Modus freigeben/sperren. Beim Sperren bricht eine laufende Sitzung ab. */
void duc_set_enabled(bool en);

/* Laeuft gerade eine DUC-Sitzung (Verbindung auf Port 1234)? */
bool duc_is_active(void);
/* Wartet, bis keine Sitzung mehr laeuft. true = idle. */
bool duc_wait_idle(uint32_t timeout_ms);
/* PARLIO-Einheit des DUC freigeben (nur ohne Sitzung aufrufen), z.B. vor dem Rueckwechsel auf raw. */
void duc_release_output(void);

/* Blockierend: bedient eine bereits akzeptierte Daten-Verbindung bis zum Ende. Schliesst den Socket NICHT. */
void duc_serve_client(int sock);

/* Der klassische "rate"-Befehl im DUC-Modus: Busrate in MSPS (= 2 * fs). Nur Rueckfall, falls der
 * Strom kein 'R' enthaelt; die In-Band-Rate hat Vorrang (und ist exakt, "rate" ist auf 0,1 MSPS gerundet). */
void duc_set_bus_rate_hint(float msps);

/* Textbefehl "duc ..." auswerten; reply (immer mit '\n' beendet) wird an den Client geschickt. */
void duc_command(const char *line, char *reply, size_t reply_size);

#ifdef __cplusplus
}
#endif
