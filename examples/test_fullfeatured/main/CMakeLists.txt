idf_component_register(
    SRCS main.c periodic_runner.c peripherals.c ble_adv_test.c
    INCLUDE_DIRS .
    REQUIRES
        driver
        esp_event
        esp_netif
        esp_timer
        esp_wifi
        bt
        nvs_flash
        console
        meshpager_x2
        bsp_extra
        esp-sr
        libnmea
        sensor
        lr20xx
        kodediy__kode_lsm6dsox)
