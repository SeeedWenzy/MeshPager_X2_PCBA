idf_component_register(
    SRCS "kode_lsm6dsox.c" "src/lsm6dsox_reg.c"
    INCLUDE_DIRS "." "src"
    REQUIRES driver
)

include(package_manager)
cu_pkg_define_version(${CMAKE_CURRENT_LIST_DIR})