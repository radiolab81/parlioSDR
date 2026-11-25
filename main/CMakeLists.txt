idf_component_register(SRCS "main.c" "duc.c" "duc_proto.c" "sdr_dsp.c" "sdr_plan.c"
                    INCLUDE_DIRS "."
                    REQUIRES esp_eth esp_netif esp_event nvs_flash esp_mm esp_driver_parlio esp_driver_gpio lwip driver ip101 esp_timer esp_hw_support)

# Software-DUC: DSP-Kern mit -O3 bauen, damit die Filterschleifen (Konstanten SDR_HB*_K) vollstaendig
# entrollt werden. Die Projektvorgabe ist sonst -Os (CONFIG_COMPILER_OPTIMIZATION_SIZE), was fuer
# die Mischer-/Filterschleifen deutlich zu langsam waere. Betrifft nur diese Datei.
set_source_files_properties(sdr_dsp.c PROPERTIES COMPILE_OPTIONS "-O3")
