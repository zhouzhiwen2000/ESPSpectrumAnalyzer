idf_component_register(
    SRCS "app.c" "board.c" "display.c" "spectrum_model.c" "radio.c" "zero_span.c"
         "esp_sdr/ring_capture.c" "esp_sdr/s3_core1.S" "esp_sdr/s3_fft_rnd.S"
         "esp_sdr/s3_unpack.S" "esp_sdr/rx_gain_verify.c"
         "esp_sdr/rx_recalibration.c" "esp_sdr/rx_calibration_state.c"
    PRIV_INCLUDE_DIRS "." "esp_sdr"
    LDFRAGMENTS "esp_sdr/linker.lf"
    PRIV_REQUIRES esp_driver_gpio esp_driver_i2c esp_driver_ledc esp_driver_spi esp_lcd esp_hw_support esp_mm
                  esp_phy esp_psram esp_wifi esp_event nvs_flash esp_timer)

target_link_options(${COMPONENT_LIB} INTERFACE
    "-T${CMAKE_CURRENT_LIST_DIR}/esp_sdr/sram_guard.ld"
    "-Wl,--wrap=chip_v7_set_chan_ana" "-Wl,--wrap=set_rx_gain_cal_dc" "-Wl,--wrap=set_rx_gain_table")
