#ifndef ESP32_P4_NANO
#define ESP32_P4_NANO  
// --- Konfiguration Waveshare ESP32_P4_NANO ---
/*
 * PARLIO 16-Bit Pin-Belegung auf dem:
 * --------------------------------------------------
 * Daten-Bits (D0-D15):
*/
#define SD0 6
#define SD1 53
#define SD2 48
#define SD3 45
#define SD4 46
#define SD5 47
#define SD6 54
#define SD7 2

#define SD8 3
#define SD9 36
#define SD10 33
#define SD11 22
#define SD12 4
#define SD13 5
#define SD14 20
#define SD15 21

/* Steuer-Signale: (Taktet die Daten in den DAC, wenn benötigt) */
#define DAC_CLK_PIN 23
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
