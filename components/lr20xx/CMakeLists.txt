set(srcs "src/lr20xx_hal.c"
         "src/radio_utilities.c"
         "src/ral_lr20xx_bsp.c"
         "src/smtc_ral/ral_lr20xx.c"
        #  "src/smtc_ral/ral_udp_pf.c"
         "src/smtc_ralf/ralf_lr20xx.c"
        #  "src/smtc_ralf/ralf_udp_pf.c"
         "src/lr20xx_driver/src/lr20xx_driver_version.c"
         "src/lr20xx_driver/src/lr20xx_patch.c"
         "src/lr20xx_driver/src/lr20xx_pram_load.c"
         "src/lr20xx_driver/src/lr20xx_radio_bluetooth_le.c"
         "src/lr20xx_driver/src/lr20xx_radio_bpsk.c"
         "src/lr20xx_driver/src/lr20xx_radio_common.c"
         "src/lr20xx_driver/src/lr20xx_radio_fifo.c"
         "src/lr20xx_driver/src/lr20xx_radio_flrc.c"
         "src/lr20xx_driver/src/lr20xx_radio_fsk.c"
         "src/lr20xx_driver/src/lr20xx_radio_lora.c"
         "src/lr20xx_driver/src/lr20xx_radio_lr_fhss.c"
         "src/lr20xx_driver/src/lr20xx_radio_ook.c"
         "src/lr20xx_driver/src/lr20xx_radio_oqpsk_15_4.c"
         "src/lr20xx_driver/src/lr20xx_radio_wi_sun.c"
         "src/lr20xx_driver/src/lr20xx_radio_wm_bus.c"
         "src/lr20xx_driver/src/lr20xx_radio_z_wave.c"
         "src/lr20xx_driver/src/lr20xx_regmem.c"
         "src/lr20xx_driver/src/lr20xx_rttof.c"
         "src/lr20xx_driver/src/lr20xx_system.c"
         "src/lr20xx_driver/src/lr20xx_workarounds.c")


set(incs "include"
         "include/lr20xx_driver"
         "include/smtc_ral"
         "include/smtc_ralf")
		 
idf_component_register(SRCS "${srcs}"
                    INCLUDE_DIRS "${incs}"
                    REQUIRES driver )


target_compile_definitions(${COMPONENT_LIB} PRIVATE
    LR20XX
    LR2021
)

include(package_manager)
cu_pkg_define_version(${CMAKE_CURRENT_LIST_DIR})
