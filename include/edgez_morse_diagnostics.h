/* Copyright 2026 EdgeZeron AI AB
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define EDGEZ_MORSE_MAC_DIAGNOSTICS_CAPACITY 1600U
#define EDGEZ_MORSE_DIAGNOSTICS_INVALID_ARGUMENT (-1)
#define EDGEZ_MORSE_DIAGNOSTICS_MALFORMED_RESPONSE (-2)

/**
 * Read one MAC statistics response into caller-owned storage (header included).
 * Call from task context with the radio running, never from an RF callback.
 * No allocation, counter reset, RF transmission or PS/STA/TWT reconfiguration.
 * The command temporarily wakes the transport: this is not a power measurement.
 * No timer or polling is installed. Serialize access to the caller's buffer.
 *
 * capacity must be at least EDGEZ_MORSE_MAC_DIAGNOSTICS_CAPACITY. On return,
 * length is the bounded response size, or zero if no usable response arrived.
 * Returns 0 on validated success, a positive native driver error, or a negative
 * argument/protocol error above. Decode using the matching firmware metadata.
 * An error response may be retained for inspection; never decode it as stats.
 */
int edgez_morse_read_mac_diagnostics(uint8_t *response, uint32_t capacity,
                                    uint32_t *length);

#ifdef __cplusplus
}
#endif
