idf_component_register(
    SRCS main.c lora_rx_test.c wifi_ap_test.c ble_coex.c ble_coex_gatt.c
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
        lr20xx
        wifi-cmd
        iperf-cmd
)
