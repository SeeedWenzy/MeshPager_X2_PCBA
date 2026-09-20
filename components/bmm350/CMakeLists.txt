set(srcs "bmm350_oor.c"
         "bmm350.c"
         "common.c")


set(incs "include")
		 
 idf_component_register(SRCS "${srcs}"
                     INCLUDE_DIRS "${incs}"
                     REQUIRES driver i2cdev log esp_idf_lib_helpers esp_timer)



