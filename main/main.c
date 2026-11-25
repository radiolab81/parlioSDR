//Konfiguration Waveshare ESP32_P4_MODULE_DEV_KIT
#include "esp32_p4_module_dev_kit.h"
// #include "esp32_p4_nano.h"
// #include "esp32_p4_wifi6_poe_ethernet.h"
// #include "esp32_p4_wifi6_dev_kit.h"
// #include "esp32_p4_eth.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

// ESP System & FreeRTOS
#include "esp_log.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_system.h"
#include "esp_heap_caps.h"
#include "esp_cache.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

// Ethernet Standard
#include "esp_eth.h"
#include "esp_eth_mac.h"
#include "esp_eth_phy.h"
#include "esp_eth_phy_ip101.h" 

// Hardware Treiber
#include "driver/parlio_tx.h"
#include "driver/gpio.h"
#include "lwip/sockets.h"

// Software-DUC (Erweiterung, siehe duc.h): Ersatz des FPGA im I/Q-Betrieb
#include "duc.h"

#define DATA_PORT 1234
#define CTRL_PORT 5000
#define BUFFER_SIZE (8 * 1024 * 1024) // 8 MB Ringbuffer für HF-Stream

#define TAG "PARLIOSDR"

//#define DAC_16BIT_MODE

#define CACHE_ALIGN 64
#define ALIGN_UP(size, align) (((size) + (align) - 1) & ~((align) - 1))
#define ALIGN_DOWN(addr, align) ((addr) & ~((align) - 1))

uint8_t *buffer_a = NULL;             
SemaphoreHandle_t config_mutex;
parlio_tx_unit_handle_t tx_unit = NULL;
size_t write_ptr = 0;

// Globale Variablen für den aktuellen Status
float current_rate = 5.0f;
int current_width = 8;

// --- ETHERNET ---

static void got_ip_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
    ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
    ESP_LOGI(TAG, "Ethernet OK! IP: " IPSTR, IP2STR(&event->ip_info.ip));
}

void init_ethernet(void) {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    
    esp_netif_config_t netif_cfg = ESP_NETIF_DEFAULT_ETH();
    esp_netif_t *eth_netif = esp_netif_new(&netif_cfg);

    eth_esp32_emac_config_t esp32_emac_config = ETH_ESP32_EMAC_DEFAULT_CONFIG();
    esp32_emac_config.interface = EMAC_DATA_INTERFACE_RMII;

    // SMI Pins für Rev 1.3
    esp32_emac_config.smi_gpio.mdc_num = SMI_GPIO_MDC_NUM;
    esp32_emac_config.smi_gpio.mdio_num = SMI_GPIO_MDIO_NUM;

    // RMII Pins (Slot 0)
    esp32_emac_config.emac_dataif_gpio.rmii.tx_en_num = TX_EN_NUM;
    esp32_emac_config.emac_dataif_gpio.rmii.txd0_num = TXD0_NUM;
    esp32_emac_config.emac_dataif_gpio.rmii.txd1_num = TXD1_NUM;
    esp32_emac_config.emac_dataif_gpio.rmii.crs_dv_num = CRS_DV_NUM;
    esp32_emac_config.emac_dataif_gpio.rmii.rxd0_num = RXD0_NUM; 
    esp32_emac_config.emac_dataif_gpio.rmii.rxd1_num = RXD1_NUM; 

    // Clock
    esp32_emac_config.clock_config.rmii.clock_mode = EMAC_CLK_EXT_IN;
    esp32_emac_config.clock_config.rmii.clock_gpio = REF_CLK_NUM;
    
    eth_mac_config_t mac_config = ETH_MAC_DEFAULT_CONFIG();
    esp_eth_mac_t *mac = esp_eth_mac_new_esp32(&esp32_emac_config, &mac_config);

    eth_phy_config_t phy_config = ETH_PHY_DEFAULT_CONFIG();
    phy_config.phy_addr = 1;
    phy_config.reset_gpio_num = PHY_RESET_NUM; 
    
    esp_eth_phy_t *phy = esp_eth_phy_new_ip101(&phy_config);

    esp_eth_config_t eth_config = ETH_DEFAULT_CONFIG(mac, phy);
    esp_eth_handle_t eth_handle = NULL;
    
    ESP_ERROR_CHECK(esp_eth_driver_install(&eth_config, &eth_handle));
    ESP_ERROR_CHECK(esp_netif_attach(eth_netif, esp_eth_new_netif_glue(eth_handle)));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_GOT_IP, &got_ip_event_handler, NULL));
    ESP_ERROR_CHECK(esp_eth_start(eth_handle));
}

// --- PARLIO ---

/* BEGIN PARLIO-BLOCK */

// --- PARLIO-TAKTAUSWAHL ---------------------------------------------------------
// Der Ausgangstakt des PARLIO-TX entsteht durch Teilung des Quelltakts. Der Teiler
// hat einen ganzzahligen Anteil von maximal 256 (assert in _parlio_ll_tx_set_clock_div,
// parlio_ll.h: clk_div->integer <= PARLIO_LL_RX_MAX_CLK_INT_DIV). Mit PLL_F160M (160 MHz)
// sind deshalb nur Raten ab ca. 0.625 MSPS moeglich (gemessen: 0.7 laeuft, 0.6 -> assert).
// Darum wird die Quelle je nach Rate gewaehlt:
//   1. PLL_F160M wie bisher, solange der Teiler <= PARLIO_MAX_DIV ist
//      (alle bisher funktionierenden Raten bleiben unveraendert)
//   2. sonst XTAL (40 MHz): Teiler bis 255 -> Raten ab ca. 0.157 MSPS
// Die Bus-Raten des COHI-Players im I/Q-Modus (2 x Samplerate): 0.5 / 1.0 / 2.5 MSPS
// teilen 40 MHz ganzzahlig (Teiler 80 / 40 / 16).
#define PARLIO_PLL_HZ    160000000u
#define PARLIO_XTAL_HZ    40000000u
#define PARLIO_MAX_DIV   255.0     // Hardware-Grenze 256, ein Schritt Reserve
#define PARLIO_MAX_MSPS   40.0     // maximaler IO-Takt

typedef struct {
    parlio_clock_source_t src;
    const char *name;
    double div;
} parlio_clk_plan_t;

static uint32_t parlio_msps_to_hz(float msps) {
    return (uint32_t)((double)msps * 1000000.0 + 0.5);
}

// Kleinste moegliche Rate in MSPS (XTAL, maximaler Teiler)
static double parlio_min_msps(void) {
    return (double)PARLIO_XTAL_HZ / PARLIO_MAX_DIV / 1000000.0;
}

// Waehlt Taktquelle und berechnet den Teiler; false = Rate nicht erzeugbar.
static bool parlio_plan_clock(uint32_t freq_hz, parlio_clk_plan_t *plan) {
    if (freq_hz == 0 || freq_hz > (uint32_t)(PARLIO_MAX_MSPS * 1000000.0)) return false;

    double div_pll  = (double)PARLIO_PLL_HZ  / (double)freq_hz;
    double div_xtal = (double)PARLIO_XTAL_HZ / (double)freq_hz;
    parlio_clk_plan_t p;

    if (div_pll >= 1.0 && div_pll <= PARLIO_MAX_DIV) {
        p.src = PARLIO_CLK_SRC_PLL_F160M; p.name = "PLL_F160M"; p.div = div_pll;
    } else if (div_xtal >= 1.0 && div_xtal <= PARLIO_MAX_DIV) {
        p.src = PARLIO_CLK_SRC_XTAL;      p.name = "XTAL";      p.div = div_xtal;
    } else {
        return false;
    }
    if (plan) *plan = p;
    return true;
}

// Letzte Einstellung, mit der die Hardware nachweislich lief (Rueckfall bei Fehlern)
static float last_good_rate = 0.0f;
static int   last_good_width = 0;

// Baut die TX-Einheit mit den angegebenen Werten neu auf. Gibt einen Fehlercode zurueck
// statt abzubrechen. Wird eine ungueltige Rate erkannt, bevor die laufende Einheit
// angefasst wird, laeuft die bisherige Einstellung unveraendert weiter.
// Aufruf nur mit gehaltenem config_mutex.
static esp_err_t parlio_apply(float msps, int width) {
    // Umrechnung von Mega-Samples in Hertz und Wahl der Taktquelle - VOR dem Abbau
    uint32_t freq_hz = parlio_msps_to_hz(msps);
    parlio_clk_plan_t plan;
    if (!parlio_plan_clock(freq_hz, &plan)) {
        ESP_LOGE(TAG, "Rate %.3f MSPS nicht erzeugbar (erlaubt: %.3f ... %.1f MSPS)",
                 msps, parlio_min_msps(), PARLIO_MAX_MSPS);
        return ESP_ERR_INVALID_ARG;
    }

    if (tx_unit != NULL) {
        parlio_tx_unit_disable(tx_unit);
        parlio_del_tx_unit(tx_unit);
        tx_unit = NULL; // Wichtig, nach Loeschung nullen!
    }

    // Dynamische Konfiguration (ohne #ifdef)
    parlio_tx_unit_config_t unit_config = {
        .clk_src = plan.src,
        .data_width = (uint32_t)width,
        .clk_out_gpio_num = DAC_CLK_PIN,
        .clk_in_gpio_num = ADC_CLK_PIN,
        .valid_gpio_num = -1,
        .output_clk_freq_hz = freq_hz,
        .trans_queue_depth = 2,
        .max_transfer_size = BUFFER_SIZE,
    };

    // Pins dynamisch je nach gewaehlter Breite zuweisen
    if (width == 16) {
        int pins_16[] = {SD0, SD1, SD2, SD3, SD4, SD5, SD6, SD7, SD8, SD9, SD10, SD11, SD12, SD13, SD14, SD15};
        for(int i=0; i<16; i++) unit_config.data_gpio_nums[i] = pins_16[i];
    } else {
        // Fallback: 8 Bit
        int pins_8[] = {SD0, SD1, SD2, SD3, SD4, SD5, SD6, SD7};
        for(int i=0; i<8; i++) unit_config.data_gpio_nums[i] = pins_8[i];
    }

    esp_err_t err = parlio_new_tx_unit(&unit_config, &tx_unit);
    if (err != ESP_OK) { tx_unit = NULL; return err; }

    err = parlio_tx_unit_enable(tx_unit);
    if (err != ESP_OK) { parlio_del_tx_unit(tx_unit); tx_unit = NULL; return err; }

    // Starte permanenten Hardware-Loop
    parlio_transmit_config_t transmit_config = {
        .flags.loop_transmission = 1,
    };

    // Buffergroesse bleibt in Bits = BUFFER_SIZE * 8
    err = parlio_tx_unit_transmit(tx_unit, buffer_a, (size_t)BUFFER_SIZE * 8, &transmit_config);
    if (err != ESP_OK) {
        parlio_tx_unit_disable(tx_unit);
        parlio_del_tx_unit(tx_unit);
        tx_unit = NULL;
        return err;
    }

    ESP_LOGI(TAG, "Hardware Loop neugestartet: %d Bit, %.2f MSPS (Takt: %s, Teiler %.2f)",
             width, msps, plan.name, plan.div);
    return ESP_OK;
}

void update_parlio_settings(float msps, int width) {
    xSemaphoreTake(config_mutex, portMAX_DELAY);

    esp_err_t err = parlio_apply(msps, width);
    if (err == ESP_OK) {
        last_good_rate = msps;
        last_good_width = width;
    } else if (last_good_width == 0) {
        // Allererster Start fehlgeschlagen: es gibt nichts, worauf man zurueckfallen koennte
        ESP_ERROR_CHECK(err);
    } else {
        ESP_LOGE(TAG, "PARLIO %.3f MSPS / %d Bit fehlgeschlagen (%s) - behalte %.3f MSPS / %d Bit",
                 msps, width, esp_err_to_name(err), last_good_rate, last_good_width);
        // Soll-Werte wieder an den tatsaechlichen Zustand angleichen
        current_rate = last_good_rate;
        current_width = last_good_width;
        if (tx_unit == NULL) {
            // Die laufende Einheit war schon abgebaut -> letzte funktionierende Einstellung wiederherstellen
            ESP_ERROR_CHECK(parlio_apply(last_good_rate, last_good_width));
        }
    }

    xSemaphoreGive(config_mutex);
}

/* END PARLIO-BLOCK */


/* BEGIN SOFTWARE-DUC-BLOCK ----------------------------------------------------------
 * Erweiterung: zwei Betriebsarten fuer den Datenport 1234.
 *
 *   SDR_MODE_RAW  (Standard) : unveraendertes bisheriges Verhalten. Der TCP-Strom (fertige
 *                              HF-Samples oder I/Q fuer einen FPGA) geht 1:1 in buffer_a, der
 *                              Hardware-Loop laeuft wie gehabt. Kein Zeichen davon wurde geaendert.
 *   SDR_MODE_DUC             : der ESP32 uebernimmt die Rolle des FPGA. Er erwartet den I/Q-Strom
 *                              der I/Q-Engine des COHIRADIAStreamer (In-Band-Befehle 'S'/'R'),
 *                              mischt per Software hoch (duc.c) und treibt PARLIO selbst.
 *
 * Umschalten zur Laufzeit ueber den Steuerport 5000 ("mode duc" / "mode raw"), alle anderen
 * Befehle ("rate", "width") verhalten sich im Modus raw exakt wie vorher.
 * Beide Betriebsarten nutzen dieselbe PARLIO-Einheit (der P4 hat nur eine TX-Einheit) und
 * denselben PSRAM-Puffer; beim Umschalten wird die jeweils andere Seite sauber abgebaut.
 * Standardmodus beim Booten per Build-Flag aenderbar: -DPARLIOSDR_DEFAULT_MODE=SDR_MODE_DUC
 */
typedef enum { SDR_MODE_RAW = 0, SDR_MODE_DUC = 1 } sdr_mode_t;

#ifndef PARLIOSDR_DEFAULT_MODE
#define PARLIOSDR_DEFAULT_MODE SDR_MODE_RAW
#endif

// Wird nur vom Control-Task (und einmalig in app_main) geschrieben, vom Network-Task gelesen.
static volatile sdr_mode_t g_mode = PARLIOSDR_DEFAULT_MODE;

// Aktuelle Daten-Verbindung, damit der Control-Task sie bei einem Moduswechsel per shutdown()
// beenden kann (recv() im Network-Task kehrt dann mit 0 zurueck). Der Mutex verhindert, dass
// ein bereits geschlossener (und evtl. wiederverwendeter) Deskriptor angefasst wird.
static int g_data_client = -1;
static SemaphoreHandle_t g_client_mutex = NULL;

static void publish_data_client(int fd) {
    xSemaphoreTake(g_client_mutex, portMAX_DELAY);
    g_data_client = fd;
    xSemaphoreGive(g_client_mutex);
}

// Deskriptor zurueckziehen und schliessen (ersetzt das direkte close(client) des Network-Tasks)
static void unpublish_and_close_data_client(int fd) {
    xSemaphoreTake(g_client_mutex, portMAX_DELAY);
    g_data_client = -1;
    close(fd);
    xSemaphoreGive(g_client_mutex);
}

static void kick_data_client(void) {
    xSemaphoreTake(g_client_mutex, portMAX_DELAY);
    if (g_data_client >= 0) shutdown(g_data_client, SHUT_RDWR);
    xSemaphoreGive(g_client_mutex);
}

// Raw-Ausgang abbauen (PARLIO-Einheit freigeben), z.B. beim Wechsel nach DUC.
// Der Zustand (current_rate/current_width, last_good_*) bleibt erhalten, damit der Rueckwechsel
// mit update_parlio_settings() exakt die vorherige Einstellung wiederherstellt.
static void raw_output_stop(void) {
    xSemaphoreTake(config_mutex, portMAX_DELAY);
    if (tx_unit != NULL) {
        parlio_tx_unit_disable(tx_unit);
        parlio_del_tx_unit(tx_unit);
        tx_unit = NULL;
    }
    xSemaphoreGive(config_mutex);
}

// Nur vom Control-Task aufrufen.
static void sdr_set_mode(sdr_mode_t m) {
    if (m == g_mode) return;

    if (m == SDR_MODE_DUC) {
        ESP_LOGI(TAG, "Modus -> DUC (Software-Hochmischer)");
        g_mode = SDR_MODE_DUC;
        duc_set_enabled(true);
        kick_data_client();            // laufende Raw-Verbindung beenden (Network-Task raeumt auf)
        raw_output_stop();             // PARLIO-Einheit fuer den DUC freigeben
    } else {
        ESP_LOGI(TAG, "Modus -> RAW (Durchreichen)");
        g_mode = SDR_MODE_RAW;
        duc_set_enabled(false);        // DUC-Sitzung bricht ab
        kick_data_client();
        if (!duc_wait_idle(3000)) ESP_LOGW(TAG, "DUC-Sitzung beendet sich nicht - fahre trotzdem fort");
        duc_release_output();
        update_parlio_settings(current_rate, current_width);   // bisheriges Verhalten wiederherstellen
    }
}

// Erweiterte Steuerbefehle. Rueckgabe true = Befehl wurde hier vollstaendig behandelt.
// Im Modus raw wird fuer "rate"/"width" bewusst false geliefert -> der unveraenderte Altcode laeuft.
static bool ext_command(int client, const char *cmd) {
    char reply[512] = "";
    bool handled = false;

    if (strncmp(cmd, "mode", 4) == 0) {
        handled = true;
        const char *arg = cmd + 4;
        while (*arg == ' ' || *arg == '\t') arg++;
        if (strncmp(arg, "duc", 3) == 0)      { sdr_set_mode(SDR_MODE_DUC); }
        else if (strncmp(arg, "raw", 3) == 0) { sdr_set_mode(SDR_MODE_RAW); }
        snprintf(reply, sizeof(reply), "mode %s\n", g_mode == SDR_MODE_DUC ? "duc" : "raw");
    } else if (strncmp(cmd, "duc", 3) == 0) {
        handled = true;
        duc_command(cmd, reply, sizeof(reply));
    } else if (g_mode == SDR_MODE_DUC) {
        // Klassische Befehle im DUC-Modus: der DUC bestimmt PARLIO-Takt und Breite selbst.
        if (strncmp(cmd, "rate ", 5) == 0) {
            handled = true;
            float val = atof(cmd + 5);
            duc_set_bus_rate_hint(val);   // = 2*fs; nur Rueckfall, falls kein 'R' im Strom kommt
            ESP_LOGI(TAG, "DUC-Modus: rate %.3f MSPS (Busrate) als Hinweis gemerkt - PARLIO-Takt folgt aus der Samplerate", val);
            snprintf(reply, sizeof(reply), "ok (duc: Hinweis, Takt wird aus fs abgeleitet)\n");
        } else if (strncmp(cmd, "width ", 6) == 0) {
            handled = true;
            ESP_LOGI(TAG, "DUC-Modus: 'width' des Streamers (Busbreite) ignoriert - Ausgangsbreite per 'duc width 8|16'");
            snprintf(reply, sizeof(reply), "ok (duc: ignoriert, siehe 'duc width')\n");
        }
    }

    if (handled && reply[0]) send(client, reply, strlen(reply), 0);
    return handled;
}

/* END SOFTWARE-DUC-BLOCK */


// --- TASKS ---

// Control Task für Plaintext-Befehle via Port 5000
void control_task(void *pv) {
    int server_fd = socket(AF_INET, SOCK_STREAM, 0);
    struct sockaddr_in addr = { .sin_family = AF_INET, .sin_addr.s_addr = INADDR_ANY, .sin_port = htons(CTRL_PORT) };
    
    int opt = 1;
    setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    bind(server_fd, (struct sockaddr *)&addr, sizeof(addr));
    listen(server_fd, 5);

    ESP_LOGI(TAG, "Control-Server lauscht auf Port %d", CTRL_PORT);

    while (1) {
        int client = accept(server_fd, NULL, NULL);
        if (client < 0) continue;

        char cmd[64] = {0};
        int len = recv(client, cmd, sizeof(cmd) - 1, 0);
        
        if (len > 0) {
            cmd[len] = '\0'; // String terminieren
            bool changed = false;

            // Erweiterung: "mode ...", "duc ..." sowie "rate"/"width" im DUC-Modus.
            // Im Modus raw liefert ext_command() false -> der Code unten laeuft unveraendert.
            if (ext_command(client, cmd)) {
                close(client);
                continue;
            }

            if (strncmp(cmd, "rate ", 5) == 0) {
                float val = atof(cmd + 5);
                // Plausibilitaetspruefung: Die Rate muss mit PLL_F160M oder XTAL erzeugbar sein,
                // sonst bricht der PARLIO-Treiber ab (assert im Taktteiler).
                if (val > 0.1f && parlio_plan_clock(parlio_msps_to_hz(val), NULL)) {
                    current_rate = val;
                    changed = true;
                } else {
                    ESP_LOGW(TAG, "rate %.3f MSPS nicht moeglich (erlaubt: %.3f ... %.1f MSPS) - ignoriert",
                             val, parlio_min_msps(), PARLIO_MAX_MSPS);
                }
            } else if (strncmp(cmd, "width ", 6) == 0) {
                int w = atoi(cmd + 6);
                if (w == 8 || w == 16) {
                    current_width = w;
                    changed = true;
                }
            }

            // Wenn sich Parameter geändert haben, PARLIO aktualisieren
            if (changed) {
                if (current_width != 8) current_width = 16;
                
                ESP_LOGI(TAG, "Control Befehl empfangen, rekonfiguriere PARLIO...");
                if (current_rate * current_width == 80.0) {
                   ESP_LOGI(TAG, "Grenzbereich 100 MBit Ethernet...");
                } 
                if (current_rate * current_width > 80.0) {
                   ESP_LOGI(TAG, "nur mit Gigabit-Ethernet PHY...");
                } 
                update_parlio_settings(current_rate, current_width);
            }
        }
        close(client);
    }
}

void network_task(void *pv) {
    int s = socket(AF_INET, SOCK_STREAM, 0);
    struct sockaddr_in addr = { .sin_family = AF_INET, .sin_port = htons(DATA_PORT), .sin_addr.s_addr = INADDR_ANY };
    bind(s, (struct sockaddr *)&addr, sizeof(addr));
    listen(s, 1);

    while (1) {
        ESP_LOGI(TAG, "Warte auf TCP Daten-Verbindung (Port %d)...", DATA_PORT);
        int client = accept(s, NULL, NULL);
        if (client < 0) continue;

        // --- Erweiterung: Software-DUC statt Durchreichen ---------------------------
        // Im Modus DUC wird die Verbindung vollstaendig von duc.c bedient (I/Q-Strom auswerten,
        // hochmischen, PARLIO treiben). Der Raw-Code darunter wird dann nicht durchlaufen und
        // beruehrt buffer_a nicht. Der Deskriptor wird veroeffentlicht, damit ein Moduswechsel
        // die Verbindung per shutdown() beenden kann; nach dem Veroeffentlichen wird der Modus
        // noch einmal geprueft (Wechsel zwischen accept() und publish).
        publish_data_client(client);
        if (g_mode == SDR_MODE_DUC) {
            duc_serve_client(client);
            unpublish_and_close_data_client(client);
            continue;
        }
        // ----------------------------------------------------------------------------

        ESP_LOGI(TAG, "Daten-Client verbunden. Stream beginnt...");

        while (1) {
            size_t space_left = BUFFER_SIZE - write_ptr;
            // Erzwinge gerade Anzahl für späteres 16-Bit Upgrade
            size_t to_read = (space_left > 65536) ? 65536 : space_left;

            // Universelles Alignment: Erzwinge immer ein Vielfaches von 4 Byte.
            // Ersetzt das alte #ifdef DAC_16BIT_MODE und ist sicher für 8-Bit, 16-Bit etc.
            to_read &= ~3; 
    
            int n = recv(client, buffer_a + write_ptr, to_read, 0);
            if (n <= 0) break; // Client hat die Verbindung getrennt
            // Erweiterung: wurde inzwischen auf DUC umgeschaltet, gehoert buffer_a dem DUC -> Daten nicht mehr schreiben
            if (g_mode != SDR_MODE_RAW) break;

            // Cache-Sync für die neuen Daten
            uintptr_t sync_addr = ALIGN_DOWN((uintptr_t)(buffer_a + write_ptr), CACHE_ALIGN);
            size_t sync_size = ALIGN_UP((uintptr_t)(buffer_a + write_ptr + n) - sync_addr, CACHE_ALIGN);
            esp_cache_msync((void *)sync_addr, sync_size, ESP_CACHE_MSYNC_FLAG_DIR_C2M);

            write_ptr += n;
            if (write_ptr >= BUFFER_SIZE) write_ptr = 0;
        }

        // --- DISCONNECT HANDLING: DEN CQ-PAPAGEI STUMMSCHALTEN ---
        unpublish_and_close_data_client(client);   // (frueher: close(client)); zusaetzlich Deskriptor zurueckziehen
        ESP_LOGW(TAG, "Client getrennt. Leere Ringbuffer...");
        // 1. Gesamten Puffer im RAM nullen
        memset(buffer_a, 0, BUFFER_SIZE);
        
        // 2. Den Cache für den gesamten Puffer in den PSRAM pushen, 
        // damit der DMA die Nullen auch wirklich sieht!
        esp_cache_msync(buffer_a, BUFFER_SIZE, ESP_CACHE_MSYNC_FLAG_DIR_C2M);
        
        // 3. Schreib-Pointer zurücksetzen für den nächsten Client
        write_ptr = 0;
    }
}

void app_main(void) {
    ESP_ERROR_CHECK(nvs_flash_init());

    // 1. PSRAM Ringbuffer allokieren
    buffer_a = heap_caps_aligned_alloc(64, BUFFER_SIZE, MALLOC_CAP_DMA | MALLOC_CAP_SPIRAM);
    memset(buffer_a, 0, BUFFER_SIZE);
    esp_cache_msync(buffer_a, BUFFER_SIZE, ESP_CACHE_MSYNC_FLAG_DIR_C2M);

    config_mutex = xSemaphoreCreateMutex();
    g_client_mutex = xSemaphoreCreateMutex();   // Erweiterung (Moduswechsel, siehe SOFTWARE-DUC-BLOCK)

    // Erweiterung: Software-DUC vorbereiten (reserviert keinen Speicher, nutzt buffer_a als Ring)
    static const int duc_pins[16] = { SD0, SD1, SD2, SD3, SD4, SD5, SD6, SD7,
                                      SD8, SD9, SD10, SD11, SD12, SD13, SD14, SD15 };
    const duc_hw_t duc_hw = { .data_pins = duc_pins, .clk_pin = DAC_CLK_PIN,
                              .ring_mem = buffer_a, .ring_mem_bytes = BUFFER_SIZE };
    ESP_ERROR_CHECK(duc_init(&duc_hw));

    // 2. Ethernet & Parlio initialisieren
    init_ethernet();
    if (g_mode == SDR_MODE_RAW) {
        update_parlio_settings(current_rate, current_width);   // Standard: wie bisher
    } else {
        duc_set_enabled(true);                                  // Boot direkt im DUC-Modus (Build-Flag)
    }

    // 3. Network Task (Schreibt von Port 1234 direkt in den laufenden DMA-Ring)
    xTaskCreatePinnedToCore(network_task, "net", 8192, NULL, 5, NULL, 1);
    
    // 4. Control Task (Lauscht auf Befehle via Port 5000)
    xTaskCreatePinnedToCore(control_task, "ctrl", 4096, NULL, 4, NULL, 0);
}
