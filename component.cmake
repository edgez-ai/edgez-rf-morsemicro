# Copyright 2026 EdgeZeron AI AB
# SPDX-License-Identifier: Apache-2.0

set(_EDGEZ_RF_MORSEMICRO_ROOT "${CMAKE_CURRENT_LIST_DIR}")

set(EDGEZ_RF_MORSEMICRO_INCLUDE_DIRS
    "${_EDGEZ_RF_MORSEMICRO_ROOT}/include"
    ${EDGEZ_RF_API_INCLUDE_DIRS}
)

# Adapter between edgez_rf_backend.h and Morse Micro's mmdrv API.
set(EDGEZ_RF_MORSEMICRO_ADAPTER_SOURCES
    "${_EDGEZ_RF_MORSEMICRO_ROOT}/src/edgez_morse_diagnostics.c"
    "${_EDGEZ_RF_MORSEMICRO_ROOT}/src/edgez_mesh_ccm_morse.c"
    "${_EDGEZ_RF_MORSEMICRO_ROOT}/src/edgez_morse_channel.c"
    "${_EDGEZ_RF_MORSEMICRO_ROOT}/src/edgez_umac_morse_adapter.c"
    "${_EDGEZ_RF_MORSEMICRO_ROOT}/src/edgez_umac_rf_binding.c"
)

# Host callback implementation compiled into the Morse RF-only archive.
set(EDGEZ_RF_MORSEMICRO_HOST_SOURCE
    "${_EDGEZ_RF_MORSEMICRO_ROOT}/src/edgez_mmdrv_host.c"
)

unset(_EDGEZ_RF_MORSEMICRO_ROOT)
