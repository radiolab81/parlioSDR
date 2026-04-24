#ifndef ESP32_P4_WIFI6_POE_ETH
#define ESP32_P4_WIFI6_POE_ETH  
// --- Konfiguration Waveshare ESP32_P4_WIFI6_POE_ETH ---
/*
 * PARLIO 16-Bit Pin-Belegung auf dem:
 * --------------------------------------------------
 * Daten-Bits (D0-D15):
*/
#define SD0 0
#define SD1 1
#define SD2 2
#define SD3 3
#define SD4 4
#define SD5 5
#define SD6 6
#define SD7 20

#define SD8 21
#define SD9 22
#define SD10 23
#define SD11 24
#define SD12 25
#define SD13 26
#define SD14 27
#define SD15 45

/* Steuer-Signale: (Taktet die Daten in den DAC, wenn benötigt) */
#define DAC_CLK_PIN 47
#define ADC_CLK_PIN -1
#define DATA_VALID_PIN -1

// SMI Pins für Rev 1.3
#define  SMI_GPIO_MDC_NUM 31
#define  SMI_GPIO_MDIO_NUM 52

// RMII Pins (Slot 0)
#define TX_EN_NUM 49
#define TXD0_NUM 34
#define TXD1_NUM 35
#define CRS_DV_NUM 28
#define RXD0_NUM 29 
#define RXD1_NUM 30 

#define REF_CLK_NUM 50
#define PHY_RESET_NUM 51

#endif
