cmake_minimum_required(VERSION 3.20)

project(edgez_rf_morsemicro LANGUAGES C)

set(EDGEZ_RF_API_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../edgez-rf-api"
    CACHE PATH "Path to edgez-rf-api")
if(NOT EXISTS "${EDGEZ_RF_API_ROOT}/include/edgez_rf_api.h")
    message(FATAL_ERROR "edgez-rf-api is required: ${EDGEZ_RF_API_ROOT}")
endif()

option(EDGEZ_RF_MORSEMICRO_BUILD_LIBRARY "Build the Morse RF adapter" ON)
option(EDGEZ_RF_MORSEMICRO_BUILD_TESTS "Build host-side policy tests" OFF)

# This repository is an adapter, not a standalone RF stack. Consumers provide
# the vendor-neutral EdgeZ UMAC headers and Morse Micro's mmdrv headers.
if(EDGEZ_RF_MORSEMICRO_BUILD_LIBRARY)
    add_library(edgez_rf_morsemicro STATIC
        src/edgez_mesh_ccm_morse.c
        src/edgez_morse_channel.c
        src/edgez_umac_morse_adapter.c
        src/edgez_umac_rf_binding.c
    )

    target_include_directories(edgez_rf_morsemicro
        PUBLIC
            ${CMAKE_CURRENT_SOURCE_DIR}/include
            ${EDGEZ_RF_API_ROOT}/include
    )

    target_compile_features(edgez_rf_morsemicro PUBLIC c_std_11)
endif()

if(EDGEZ_RF_MORSEMICRO_BUILD_TESTS)
    enable_testing()
    add_executable(test_edgez_morse_tx_policy
        tests/test_edgez_morse_tx_policy.c
    )
    target_include_directories(test_edgez_morse_tx_policy PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${EDGEZ_RF_API_ROOT}/include
    )
    target_compile_features(test_edgez_morse_tx_policy PRIVATE c_std_11)
    add_test(NAME edgez_morse_tx_policy COMMAND test_edgez_morse_tx_policy)
    add_executable(test_edgez_morse_rx_policy tests/test_edgez_morse_rx_policy.c)
    target_include_directories(test_edgez_morse_rx_policy PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include)
    target_compile_features(test_edgez_morse_rx_policy PRIVATE c_std_11)
    add_test(NAME edgez_morse_rx_policy COMMAND test_edgez_morse_rx_policy)
endif()
