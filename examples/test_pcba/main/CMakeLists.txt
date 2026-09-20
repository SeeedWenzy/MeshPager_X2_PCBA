idf_component_register(
    SRCS main.c app_storage.c lora_cmd.c ble_cmd.c gnss_cmd.c ble_app.c ble_gatt_svr.c gpio_cmd.c i2c_cmd.c io_expander_cmd.c sensor_cmd.c audio_cmd.c lcd_cmd.c sd_lcd_cmd.c power_cmd.c
    INCLUDE_DIRS .
    REQUIRES
        driver
        pngle
        spi_flash
        esp_timer
        nvs_flash
        bt
        console
        spiffs
        meshpager_x2
        esp_io_expander
        esp_io_expander_tca6424
        bsp_extra
        ping-cmd
        wifi-cmd
        libnmea
        sensor
        lvgl
        sr_ringbuf
        kodediy__kode_lsm6dsox)

# Bake the host `spiffs/` folder (audio/, images/, rgb565/) into the `storage`
# SPIFFS partition at build time so the test assets are available in flash even
# when no SD card is inserted. Flashed automatically with `idf.py flash`.
spiffs_create_partition_image(storage ../spiffs FLASH_IN_PROJECT)
