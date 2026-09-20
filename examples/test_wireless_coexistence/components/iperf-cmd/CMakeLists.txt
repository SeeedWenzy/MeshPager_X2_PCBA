idf_component_register(
    SRCS iperf_cmd.c
    INCLUDE_DIRS include
    PRIV_REQUIRES console esp_netif iperf
)
