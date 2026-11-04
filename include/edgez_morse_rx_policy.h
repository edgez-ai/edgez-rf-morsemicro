/* Copyright 2026 EdgeZeron AI AB
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>

/* Only remove an FCS explicitly identified by the firmware. Do not infer it
 * from packet length or relax the upper-layer protocol's length validation. */
static inline bool edgez_morse_rx_payload_length(size_t wire_length,
                                                bool fcs_included,
                                                bool receive_error,
                                                size_t *payload_length)
{
    if (payload_length == NULL || receive_error || wire_length == 0U ||
        (fcs_included && wire_length <= 4U))
    {
        return false;
    }
    *payload_length = wire_length - (fcs_included ? 4U : 0U);
    return true;
}
