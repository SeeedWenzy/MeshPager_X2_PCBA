set(SRCS "")
list(APPEND SRCS
    "src/meshpager_x2.c" 
    "src/bsp_gnss_control.c" 
    "src/bsp_i2c_control.c" 
    "src/bsp_lcd_st7789p3.c"
)

foreach(SRC_DIR IN LISTS SRC_DIRS)
    file(GLOB_RECURSE SRC ${SRC_DIR}/*.c)
    list(APPEND SRCS ${SRC})
endforeach()

idf_component_register(
    SRCS ${SRCS}
    INCLUDE_DIRS "include"
    REQUIRES driver esp_adc esp_lcd lr20xx nvs_flash
    PRIV_REQUIRES usb spiffs fatfs 
)
