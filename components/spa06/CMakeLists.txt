set(srcs "spa06.c"
         "common.c")

set(incs "include")

idf_component_register(SRCS "${srcs}"
                     INCLUDE_DIRS "${incs}"
                     REQUIRES driver i2cdev esp_idf_lib_helpers log esp_timer)
