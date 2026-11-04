/* Copyright 2026 EdgeZeron AI AB
 * SPDX-License-Identifier: Apache-2.0
 */
#include "edgez_morse_diagnostics.h"
#include "mmdrv.h"
#include "common/morse_commands.h"
#include "common/morse_command_utils.h"

#include <string.h>

int edgez_morse_read_mac_diagnostics(uint8_t *response, uint32_t capacity,
                                    uint32_t *length)
{
    if (length == NULL) return EDGEZ_MORSE_DIAGNOSTICS_INVALID_ARGUMENT;
    *length = 0;
    if (response == NULL || capacity < EDGEZ_MORSE_MAC_DIAGNOSTICS_CAPACITY)
        return EDGEZ_MORSE_DIAGNOSTICS_INVALID_ARGUMENT;

    struct morse_cmd_req cmd = MORSE_COMMAND_INIT(cmd,
        MORSE_CMD_ID_MAC_STATS_LOG, UINT16_MAX);
    uint32_t bytes = capacity;
    enum mmwlan_status rc = mmdrv_execute_command((uint8_t *)&cmd, response, &bytes);
    if (rc != MMWLAN_SUCCESS && rc != MMWLAN_COMMAND_ERROR) return (int)rc;
    if (bytes < sizeof(struct morse_cmd_resp) || bytes > capacity)
        return EDGEZ_MORSE_DIAGNOSTICS_MALFORMED_RESPONSE;

    struct morse_cmd_resp header;
    memcpy(&header, response, sizeof(header));
    if (le16toh(header.hdr.message_id) != MORSE_CMD_ID_MAC_STATS_LOG ||
        le16toh(header.hdr.vif_id) != UINT16_MAX ||
        (uint32_t)le16toh(header.hdr.len) + sizeof(header.hdr) != bytes)
        return EDGEZ_MORSE_DIAGNOSTICS_MALFORMED_RESPONSE;
    *length = bytes;
    if (rc != MMWLAN_SUCCESS) return (int)rc;
    if (le32toh(header.status) != 0)
        return EDGEZ_MORSE_DIAGNOSTICS_MALFORMED_RESPONSE;
    return 0;
}
