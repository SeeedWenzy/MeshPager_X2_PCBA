cmake_minimum_required(VERSION 3.5)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

set(EXTRA_COMPONENT_DIRS
    ../../components
    components/iperf
    components/iperf-cmd
)

add_compile_options(-Wno-format)
add_compile_options(-Wno-ignored-qualifiers)
project(test_wireless_coexistence)
