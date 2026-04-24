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

void update_parlio_settings(float msps, int width) {
    xSemaphoreTake(config_mutex, portMAX_DELAY);
    
    if (tx_unit != NULL) {
        parlio_tx_unit_disable(tx_unit);
        parlio_del_tx_unit(tx_unit);
        tx_unit = NULL; // Wichtig, nach Löschung nullen!
    }

    // Umrechnung von Mega-Samples in Hertz
    uint32_t freq_hz = (uint32_t)(msps * 1000000.0f);

    // Dynamische Konfiguration (ohne #ifdef)
    parlio_tx_unit_config_t unit_config = {
        .clk_src = PARLIO_CLK_SRC_PLL_F160M,
        .data_width = (uint32_t)width,
        .clk_out_gpio_num = DAC_CLK_PIN,
        .clk_in_gpio_num = ADC_CLK_PIN,
        .valid_gpio_num = -1,
        .output_clk_freq_hz = freq_hz,
        .trans_queue_depth = 2,
        .max_transfer_size = BUFFER_SIZE,
    };


    // Pins dynamisch je nach gewählter Breite zuweisen
    if (width == 16) {
        int pins_16[] = {SD0, SD1, SD2, SD3, SD4, SD5, SD6, SD7, SD8, SD9, SD10, SD11, SD12, SD13, SD14, SD15};
        for(int i=0; i<16; i++) unit_config.data_gpio_nums[i] = pins_16[i];
    } else {
        // Fallback: 8 Bit
        int pins_8[] = {SD0, SD1, SD2, SD3, SD4, SD5, SD6, SD7};
        for(int i=0; i<8; i++) unit_config.data_gpio_nums[i] = pins_8[i];
    }

    ESP_ERROR_CHECK(parlio_new_tx_unit(&unit_config, &tx_unit));
    ESP_ERROR_CHECK(parlio_tx_unit_enable(tx_unit));

    // Starte permanenten Hardware-Loop
    parlio_transmit_config_t transmit_config = {
        .flags.loop_transmission = 1,
    };

    ESP_LOGI(TAG, "Hardware Loop neugestartet: %d Bit, %.2f MSPS", width, msps);
    // Buffergröße bleibt in Bits = BUFFER_SIZE * 8
    ESP_ERROR_CHECK(parlio_tx_unit_transmit(tx_unit, buffer_a, (size_t)BUFFER_SIZE * 8, &transmit_config));
    
    xSemaphoreGive(config_mutex);
}


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

            if (strncmp(cmd, "rate ", 5) == 0) {
                current_rate = atof(cmd + 5);
                changed = true;
            } else if (strncmp(cmd, "width ", 6) == 0) {
                current_width = atoi(cmd + 6);
                changed = true;
            }

            // Wenn sich Parameter geändert haben, PARLIO aktualisieren
            if (changed) {
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

            // Cache-Sync für die neuen Daten
            uintptr_t sync_addr = ALIGN_DOWN((uintptr_t)(buffer_a + write_ptr), CACHE_ALIGN);
            size_t sync_size = ALIGN_UP((uintptr_t)(buffer_a + write_ptr + n) - sync_addr, CACHE_ALIGN);
            esp_cache_msync((void *)sync_addr, sync_size, ESP_CACHE_MSYNC_FLAG_DIR_C2M);

            write_ptr += n;
            if (write_ptr >= BUFFER_SIZE) write_ptr = 0;
        }

        // --- DISCONNECT HANDLING: DEN CQ-PAPAGEI STUMMSCHALTEN ---
        close(client);
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

    // 2. Ethernet & Parlio initialisieren
    init_ethernet();
    update_parlio_settings(current_rate, current_width);

    // 3. Network Task (Schreibt von Port 1234 direkt in den laufenden DMA-Ring)
    xTaskCreatePinnedToCore(network_task, "net", 8192, NULL, 5, NULL, 1);
    
    // 4. Control Task (Lauscht auf Befehle via Port 5000)
    xTaskCreatePinnedToCore(control_task, "ctrl", 4096, NULL, 4, NULL, 0);
}
