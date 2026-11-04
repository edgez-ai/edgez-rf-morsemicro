/* Copyright 2026 EdgeZeron AI AB
 * SPDX-License-Identifier: Apache-2.0
 */
#include <assert.h>
#include "edgez_morse_rx_policy.h"

int main(void)
{
    size_t length = 123U;
    assert(edgez_morse_rx_payload_length(65U, true, false, &length));
    assert(length == 61U);
    assert(edgez_morse_rx_payload_length(65U, false, false, &length));
    assert(length == 65U);
    assert(edgez_morse_rx_payload_length(61U, false, false, &length));
    assert(length == 61U);
    assert(!edgez_morse_rx_payload_length(65U, true, true, &length));
    assert(!edgez_morse_rx_payload_length(61U, false, true, &length));
    for (size_t n = 0U; n <= 4U; ++n)
        assert(!edgez_morse_rx_payload_length(n, true, false, &length));
    assert(!edgez_morse_rx_payload_length(0U, false, false, &length));
    assert(!edgez_morse_rx_payload_length(65U, true, false, NULL));
    assert(length == 61U);
    return 0;
}
