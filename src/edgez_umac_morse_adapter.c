/*
 * Copyright 2026 EdgeZeron AI AB
 * SPDX-License-Identifier: Apache-2.0
 */

#include "edgez_umac_morse_adapter.h"
#include "edgez_morse_tx_policy.h"
#include "edgez_rf_api.h"

#include "mmhal_wlan.h"
#include "mmdrv.h"

/* This adapter owns the peer/SAE boundary, so retain its INFO diagnostics even
 * when the rest of the Morse stack is built at the default ERROR level. */
#ifndef MMLOG_LEVEL_OVRD
#define MMLOG_LEVEL_OVRD 5
#endif
#include "mmlog.h"

#include <stdatomic.h>
#include <string.h>

/* Use the native adapter logger here. PlatformIO compiles this source in a
 * second pass that intentionally does not expose Zephyr's private include
 * paths, while MMLOG is part of the public Morse RF boundary in both passes. */
#define EDGEZ_RF_LOG_DBG(...) MMLOG_INF(__VA_ARGS__)
#define EDGEZ_RF_LOG_INF(...) MMLOG_INF(__VA_ARGS__)
#define EDGEZ_RF_LOG_WRN(...) MMLOG_WRN(__VA_ARGS__)

#define EDGEZ_MORSE_TX_TRACKED_MAX 16U
#define EDGEZ_MORSE_BEACON_MAX_LEN 768U
/* The MM8108 packet pool accepts up to 1668 bytes.  Keep enough room for a
 * normal HaLow-sized public action frame while staying below that RF boundary. */
#define EDGEZ_MORSE_MGMT_MAX_LEN 1536U
#define EDGEZ_MORSE_ACTION_HEADER_LEN 24U
#define EDGEZ_MORSE_SELF_PROTECTED_CATEGORY 15U
#define EDGEZ_MORSE_MPM_OPEN 1U
#define EDGEZ_MORSE_MPM_CONFIRM 2U
#define EDGEZ_MORSE_HT_CAPABILITIES_EID 45U
#define EDGEZ_MORSE_HT_OPERATION_EID 61U
#define EDGEZ_MORSE_VHT_CAPABILITIES_EID 191U
#define EDGEZ_MORSE_VHT_OPERATION_EID 192U
#define EDGEZ_MORSE_S1G_CAPABILITIES_EID 217U
#define EDGEZ_MORSE_S1G_OPERATION_EID 232U
#define EDGEZ_MORSE_S1G_CAPABILITIES_IE_LEN 17U

/* Native S1G operation matching the HC01 join-only profile: US operating
 * class 2, 2 MHz centred on channel 26 (915 MHz), with primary channel 25.
 * The legacy compatibility block below carries the same fixed profile; this
 * IE completes its translation for Linux Morse peers, whose management RX
 * validation requires both native S1G capability and operation elements. */
static const uint8_t edgez_morse_s1g_operation_ie[] = {
    EDGEZ_MORSE_S1G_OPERATION_EID, 6U,
    0x03U, 2U, 25U, 26U, 0xfcU, 0xffU,
};

struct edgez_morse_tx_tracking
{
    struct mmpkt *packet;
    uintptr_t token;
};

static struct edgez_morse_tx_tracking tx_tracking[EDGEZ_MORSE_TX_TRACKED_MAX];
static uint8_t beacon_frame[EDGEZ_MORSE_BEACON_MAX_LEN];
static atomic_size_t beacon_frame_length;
static uint16_t beacon_vif_id;
static uint8_t beacon_mcs;
static uint8_t beacon_bandwidth_mhz;
static bool beacon_short_guard;
static struct edgez_umac callback_observer;
static atomic_bool callback_bridge_enabled;
static atomic_uint callback_rx_frames;
static atomic_uint callback_tx_status_count;
static atomic_uint callback_tx_pause_changes;
static atomic_uint callback_hardware_restarts;
static atomic_uint callback_scan_completions;
static char active_country_code[MMWLAN_COUNTRY_CODE_LEN];

/* Keep Morse packet-memory diagnostics behind the vendor adapter so the
 * public EdgeZ interface does not expose mmhal symbols. */
extern uint32_t mmhal_wlan_pktmem_tx_free_count(void) __attribute__((weak));
extern uint32_t mmhal_wlan_pktmem_tx_total_count(void) __attribute__((weak));

static void edgez_morse_clear_tx_tracking(void)
{
    MMOSAL_TASK_ENTER_CRITICAL();
    memset(tx_tracking, 0, sizeof(tx_tracking));
    MMOSAL_TASK_EXIT_CRITICAL();
}

uint32_t edgez_rf_tx_packet_free_count(void)
{
    return mmhal_wlan_pktmem_tx_free_count != NULL ?
        mmhal_wlan_pktmem_tx_free_count() : 0U;
}

uint32_t edgez_rf_tx_packet_total_count(void)
{
    return mmhal_wlan_pktmem_tx_total_count != NULL ?
        mmhal_wlan_pktmem_tx_total_count() : 0U;
}

static uint16_t edgez_morse_get_le16(const uint8_t *value)
{
    return (uint16_t)value[0] | ((uint16_t)value[1] << 8);
}

/* EdgeZ includes both legacy HT/VHT compatibility IEs and the native S1G
 * Capabilities IE so RF backends can choose their representation.  The Morse
 * raw-management path transmits these caller-owned frames verbatim.  Keep the
 * native S1G IE expected by the Linux HaLow peer and remove the four legacy
 * compatibility IEs at this vendor boundary. */
static size_t edgez_morse_prepare_management_frame(
    const uint8_t *input, size_t input_length,
    uint8_t output[EDGEZ_MORSE_MGMT_MAX_LEN])
{
    if (input == NULL || input_length > EDGEZ_MORSE_MGMT_MAX_LEN)
    {
        return 0U;
    }
    memcpy(output, input, input_length);
    if (input_length < EDGEZ_MORSE_ACTION_HEADER_LEN + 4U ||
        (edgez_morse_get_le16(input) & 0x00fcU) != 0x00d0U ||
        input[EDGEZ_MORSE_ACTION_HEADER_LEN] !=
            EDGEZ_MORSE_SELF_PROTECTED_CATEGORY)
    {
        return input_length;
    }

    const uint8_t action = input[EDGEZ_MORSE_ACTION_HEADER_LEN + 1U];
    size_t cursor;
    if (action == EDGEZ_MORSE_MPM_OPEN)
    {
        cursor = EDGEZ_MORSE_ACTION_HEADER_LEN + 4U;
    }
    else if (action == EDGEZ_MORSE_MPM_CONFIRM)
    {
        cursor = EDGEZ_MORSE_ACTION_HEADER_LEN + 6U;
    }
    else
    {
        return input_length;
    }

    const size_t legacy_ie_length = 73U;
    while (cursor + legacy_ie_length <= input_length)
    {
        if (output[cursor] == EDGEZ_MORSE_HT_CAPABILITIES_EID &&
            output[cursor + 1U] == 26U &&
            output[cursor + 28U] == EDGEZ_MORSE_HT_OPERATION_EID &&
            output[cursor + 29U] == 22U &&
            output[cursor + 52U] == EDGEZ_MORSE_VHT_CAPABILITIES_EID &&
            output[cursor + 53U] == 12U &&
            output[cursor + 66U] == EDGEZ_MORSE_VHT_OPERATION_EID &&
            output[cursor + 67U] == 5U)
        {
            memmove(&output[cursor], &output[cursor + legacy_ie_length],
                    input_length - cursor - legacy_ie_length);
            size_t output_length = input_length - legacy_ie_length;
            const size_t capability_end =
                cursor + EDGEZ_MORSE_S1G_CAPABILITIES_IE_LEN;
            if (capability_end <= output_length &&
                output[cursor] == EDGEZ_MORSE_S1G_CAPABILITIES_EID &&
                output[cursor + 1U] == 15U &&
                output_length + sizeof(edgez_morse_s1g_operation_ie) <=
                    EDGEZ_MORSE_MGMT_MAX_LEN)
            {
                memmove(&output[capability_end +
                                sizeof(edgez_morse_s1g_operation_ie)],
                        &output[capability_end],
                        output_length - capability_end);
                memcpy(&output[capability_end],
                       edgez_morse_s1g_operation_ie,
                       sizeof(edgez_morse_s1g_operation_ie));
                output_length += sizeof(edgez_morse_s1g_operation_ie);
            }
            EDGEZ_RF_LOG_INF("MPM compatibility filter action=%u len=%u->%u",
                             (unsigned)action, (unsigned)input_length,
                             (unsigned)output_length);
            return output_length;
        }
        ++cursor;
    }
    return input_length;
}

bool edgez_umac_morse_normalize_rx(struct edgez_umac_rx_frame *frame)
{
    if (frame == NULL || frame->data == NULL || frame->length < 26U)
    {
        return false;
    }

    frame->format = EDGEZ_UMAC_RX_RAW_80211;
    memset(&frame->mesh_data, 0, sizeof(frame->mesh_data));

    const uint16_t fc = edgez_morse_get_le16(frame->data);
    const uint8_t type = (uint8_t)((fc >> 2) & 0x03U);
    const uint8_t subtype = (uint8_t)((fc >> 4) & 0x0fU);
    if (type != 2U || subtype != 8U || (fc & 0x4000U) == 0U)
    {
        return false;
    }

    const bool has_address4 = (fc & 0x0300U) == 0x0300U;
    const size_t header_length = has_address4 ? 32U : 26U;
    if (frame->length <= header_length)
    {
        return false;
    }
    const uint16_t qos = edgez_morse_get_le16(
        &frame->data[header_length - 2U]);

    frame->format = EDGEZ_UMAC_RX_PROTECTED_MESH_DATA;
    memcpy(frame->mesh_data.receiver, &frame->data[4], EDGEZ_UMAC_MAC_LEN);
    memcpy(frame->mesh_data.transmitter, &frame->data[10], EDGEZ_UMAC_MAC_LEN);
    memcpy(frame->mesh_data.address3, &frame->data[16], EDGEZ_UMAC_MAC_LEN);
    if (has_address4)
    {
        memcpy(frame->mesh_data.address4, &frame->data[24],
               EDGEZ_UMAC_MAC_LEN);
        memcpy(frame->mesh_data.destination, &frame->data[16],
               EDGEZ_UMAC_MAC_LEN);
        memcpy(frame->mesh_data.source, &frame->data[24],
               EDGEZ_UMAC_MAC_LEN);
    }
    else
    {
        /* Morse firmware reports received mesh QoS data in a three-address
         * compatibility form.  Normalize its link destination/source here so
         * the private UMAC remains independent of that vendor representation. */
        memcpy(frame->mesh_data.destination, &frame->data[4],
               EDGEZ_UMAC_MAC_LEN);
        memcpy(frame->mesh_data.source, &frame->data[10],
               EDGEZ_UMAC_MAC_LEN);
    }
    frame->mesh_data.sequence_number =
        edgez_morse_get_le16(&frame->data[22]) >> 4;
    frame->mesh_data.tid = (uint8_t)(qos & 0x0fU);
    frame->mesh_data.has_address4 = has_address4;
    /* Some Morse firmware revisions normalize the Mesh Control Present bit
     * out of received QoS control.  Keep the protected mesh frame and let the
     * vendor-independent plaintext decoder infer Mesh Control after CCMP. */
    frame->mesh_data.mesh_control_present = (qos & 0x0100U) != 0U;
    frame->mesh_data.mac_header_length = header_length;
    frame->mesh_data.protected_body_offset = header_length;
    return true;
}

static bool edgez_morse_track_tx(struct mmpkt *packet, uintptr_t token)
{
    bool tracked = false;
    MMOSAL_TASK_ENTER_CRITICAL();
    for (size_t i = 0; i < EDGEZ_MORSE_TX_TRACKED_MAX; ++i)
    {
        if (tx_tracking[i].packet == NULL)
        {
            tx_tracking[i].packet = packet;
            tx_tracking[i].token = token;
            tracked = true;
            break;
        }
    }
    MMOSAL_TASK_EXIT_CRITICAL();
    return tracked;
}

static uintptr_t edgez_morse_take_tx_token(struct mmpkt *packet)
{
    uintptr_t token = 0;
    MMOSAL_TASK_ENTER_CRITICAL();
    for (size_t i = 0; i < EDGEZ_MORSE_TX_TRACKED_MAX; ++i)
    {
        if (tx_tracking[i].packet == packet)
        {
            token = tx_tracking[i].token;
            tx_tracking[i].packet = NULL;
            tx_tracking[i].token = 0;
            break;
        }
    }
    MMOSAL_TASK_EXIT_CRITICAL();
    return token;
}

static uintptr_t edgez_morse_peek_tx_token(struct mmpkt *packet)
{
    uintptr_t token = 0;
    MMOSAL_TASK_ENTER_CRITICAL();
    for (size_t i = 0; i < EDGEZ_MORSE_TX_TRACKED_MAX; ++i)
    {
        if (tx_tracking[i].packet == packet)
        {
            token = tx_tracking[i].token;
            break;
        }
    }
    MMOSAL_TASK_EXIT_CRITICAL();
    return token;
}

static int edgez_morse_prepare(void *driver_ctx)
{
    (void)driver_ctx;
    edgez_morse_clear_tx_tracking();
    mmdrv_pre_init();
    return 0;
}

static void edgez_morse_release(void *driver_ctx)
{
    (void)driver_ctx;
    atomic_store_explicit(&beacon_frame_length, 0U, memory_order_release);
    mmdrv_post_deinit();
}

static int edgez_morse_radio_start(void *driver_ctx, const char country_code[3],
                                   struct edgez_umac_radio_info *info)
{
    struct mmdrv_chip_info chip_info = { 0 };
    enum mmwlan_status status;
    (void)driver_ctx;

    status = mmdrv_init(&chip_info, country_code);
    if (status != MMWLAN_SUCCESS)
    {
        return (int)status;
    }
    memcpy(active_country_code, country_code, sizeof(active_country_code));
    /* Some firmware/BCF combinations leave the extended-host-table MAC at
     * zero. The platform HAL preserves a valid firmware address and otherwise
     * derives a stable locally administered address from the host device ID. */
    mmhal_read_mac_addr(chip_info.mac_addr);
    if (info != NULL)
    {
        memcpy(info->mac_addr, chip_info.mac_addr, sizeof(info->mac_addr));
        info->chip_id = chip_info.morse_chip_id;
        info->firmware_major = chip_info.fw_version.major;
        info->firmware_minor = chip_info.fw_version.minor;
        info->firmware_patch = chip_info.fw_version.patch;
    }
    return 0;
}

static void edgez_morse_radio_stop(void *driver_ctx)
{
    (void)driver_ctx;
    mmdrv_deinit();
    memset(active_country_code, 0, sizeof(active_country_code));
    /* A stopped radio may not deliver completion callbacks for packets that
     * were still owned by the firmware.  The packet pool is reset by deinit;
     * discard the corresponding host-side tokens before the next wake cycle. */
    edgez_morse_clear_tx_tracking();
}

static int edgez_morse_add_mesh_interface(void *driver_ctx,
                                          const uint8_t mac_addr[EDGEZ_UMAC_MAC_LEN],
                                          uint16_t *vif_id)
{
    (void)driver_ctx;
    return (int)mmdrv_add_if(vif_id, mac_addr, MMDRV_INTERFACE_TYPE_MESH);
}

static int edgez_morse_remove_interface(void *driver_ctx, uint16_t vif_id)
{
    (void)driver_ctx;
    return (int)mmdrv_rm_if(vif_id);
}

static int edgez_morse_set_channel(void *driver_ctx,
                                   const struct edgez_umac_channel *channel)
{
    struct edgez_rf_regulatory_channel regulatory = { 0 };
    enum mmwlan_status status;
    (void)driver_ctx;

    if (!edgez_rf_resolve_regulatory_channel(
            active_country_code, channel->operating_frequency_hz,
            channel->operating_bw_mhz, &regulatory))
    {
        EDGEZ_RF_LOG_WRN("Channel %lu Hz/%u MHz is not valid for %.2s\n",
                          (unsigned long)channel->operating_frequency_hz,
                          channel->operating_bw_mhz, active_country_code);
        return (int)MMWLAN_CHANNEL_INVALID;
    }

    status = mmdrv_set_channel(channel->operating_frequency_hz,
                               channel->primary_1mhz_channel_index,
                               channel->operating_bw_mhz,
                               channel->primary_bw_mhz, false);
    if (status != MMWLAN_SUCCESS)
    {
        return (int)status;
    }

    int32_t actual_txpower = 0;
    status = mmdrv_set_txpower(&actual_txpower, regulatory.max_eirp_dbm);
    if (status != MMWLAN_SUCCESS)
    {
        return (int)status;
    }
    status = mmdrv_set_duty_cycle(regulatory.duty_cycle_centi_percent,
                                  regulatory.duty_cycle_omit_control_response,
                                  MMWLAN_DUTY_CYCLE_MODE_SPREAD);
    if (status != MMWLAN_SUCCESS)
    {
        return (int)status;
    }

    EDGEZ_RF_LOG_INF("Regulatory profile %.2s: %lu Hz/%u MHz, duty %u.%02u%%, tx EIRP requested %d dBm actual %ld dBm\n",
                      active_country_code,
                      (unsigned long)channel->operating_frequency_hz,
                      channel->operating_bw_mhz,
                      regulatory.duty_cycle_centi_percent / 100U,
                      regulatory.duty_cycle_centi_percent % 100U,
                      regulatory.max_eirp_dbm,
                      (long)actual_txpower);
    return (int)MMWLAN_SUCCESS;
}

static int edgez_morse_configure_bss(void *driver_ctx, uint16_t vif_id,
                                     uint16_t beacon_interval_tu,
                                     uint8_t dtim_period, uint32_t compressed_ssid)
{
    (void)driver_ctx;
    return (int)mmdrv_cfg_bss(vif_id, beacon_interval_tu, dtim_period,
                              compressed_ssid);
}

static int edgez_morse_set_beaconing(void *driver_ctx, uint16_t vif_id, bool enabled)
{
    (void)driver_ctx;
    return enabled ? (int)mmdrv_start_beaconing(vif_id) :
                     (int)mmdrv_stop_beaconing();
}

static int edgez_morse_set_beacon_template(
    void *driver_ctx, uint16_t vif_id,
    const struct edgez_rf_beacon_template *beacon_template)
{
    (void)driver_ctx;
    atomic_store_explicit(&beacon_frame_length, 0U, memory_order_release);
    if (beacon_template == NULL)
    {
        return 0;
    }
    if (vif_id > UINT8_MAX || beacon_template->frame == NULL ||
        beacon_template->frame_length == 0U ||
        beacon_template->frame_length > sizeof(beacon_frame))
    {
        return (int)MMWLAN_INVALID_ARGUMENT;
    }
    switch (beacon_template->bandwidth_mhz)
    {
        case 1:
        case 2:
        case 4:
        case 8:
        case 16:
            break;
        default:
            return (int)MMWLAN_INVALID_ARGUMENT;
    }
    memcpy(beacon_frame, beacon_template->frame,
           beacon_template->frame_length);
    beacon_vif_id = vif_id;
    beacon_mcs = beacon_template->mcs;
    beacon_bandwidth_mhz = beacon_template->bandwidth_mhz;
    beacon_short_guard = beacon_template->short_guard;
    atomic_store_explicit(&beacon_frame_length,
                          beacon_template->frame_length,
                          memory_order_release);
    return 0;
}

static bool edgez_morse_set_rate(struct mmdrv_tx_metadata *metadata,
                                 uint8_t mcs, uint8_t bandwidth_mhz,
                                 bool short_guard, bool rts_cts)
{
    metadata->rc_data.rates[0].attempts = 1U;
    metadata->rc_data.rates[0].rate = mcs;
    switch (bandwidth_mhz)
    {
        case 1: metadata->rc_data.rates[0].bw = MMRC_BW_1MHZ; break;
        case 2: metadata->rc_data.rates[0].bw = MMRC_BW_2MHZ; break;
        case 4: metadata->rc_data.rates[0].bw = MMRC_BW_4MHZ; break;
        case 8: metadata->rc_data.rates[0].bw = MMRC_BW_8MHZ; break;
        case 16: metadata->rc_data.rates[0].bw = MMRC_BW_16MHZ; break;
        default: return false;
    }
    metadata->rc_data.rates[0].guard = short_guard ?
        MMRC_GUARD_SHORT : MMRC_GUARD_LONG;
    metadata->rc_data.rates[0].ss = MMRC_SPATIAL_STREAM_1;
    metadata->rc_data.rates[0].flags = rts_cts ?
        MMRC_MASK(MMRC_FLAGS_CTS_RTS) : 0U;
    for (size_t i = 1; i < MMRC_MAX_CHAIN_LENGTH; ++i)
    {
        metadata->rc_data.rates[i].rate = MMRC_MCS_UNUSED;
    }
    return true;
}

struct mmpkt *edgez_umac_morse_get_beacon(void)
{
    const size_t frame_length = atomic_load_explicit(
        &beacon_frame_length, memory_order_acquire);
    if (frame_length == 0U)
    {
        return NULL;
    }
    struct mmpkt *packet = mmdrv_alloc_mmpkt_for_tx(
        MMDRV_PKT_CLASS_MGMT, 0U, (uint32_t)frame_length);
    if (packet == NULL)
    {
        return NULL;
    }
    struct mmpktview *view = mmpkt_open(packet);
    mmpkt_append_data(view, beacon_frame, (uint32_t)frame_length);
    mmpkt_close(&view);

    struct mmdrv_tx_metadata *metadata = mmdrv_get_tx_metadata(packet);
    metadata->vif_id = (uint8_t)beacon_vif_id;
    metadata->tid = MMWLAN_MAX_QOS_TID;
    metadata->flags = MMDRV_TX_FLAG_IMMEDIATE_REPORT |
                      MMDRV_TX_FLAG_NO_ACK;
    metadata->is_mgmt = true;
    if (!edgez_morse_set_rate(metadata, beacon_mcs, beacon_bandwidth_mhz,
                              beacon_short_guard, false))
    {
        mmpkt_release(packet);
        return NULL;
    }
    return packet;
}

static int edgez_morse_set_peer_state(void *driver_ctx, uint16_t vif_id, uint16_t aid,
                                      const uint8_t peer_addr[EDGEZ_UMAC_MAC_LEN],
                                      enum edgez_umac_peer_state state)
{
    static const enum morse_sta_state states[] = {
        MORSE_STA_NOTEXIST,
        MORSE_STA_NONE,
        MORSE_STA_AUTHENTICATED,
        /* Morse's management TX path requires AUTHORIZED before it will put
         * Mesh Peering Action frames on air.  This mirrors the working hostap
         * integration, which programs AUTHENTICATED|AUTHORIZED immediately
         * after SAE.  EdgeZ still treats this as only ASSOCIATED and gates
         * protected data in its own UMAC until AMPE establishes the link. */
        MORSE_STA_AUTHORIZED,
        MORSE_STA_AUTHORIZED,
    };
    (void)driver_ctx;

    if ((unsigned)state >= (sizeof(states) / sizeof(states[0])))
    {
        return (int)MMWLAN_INVALID_ARGUMENT;
    }
    const int status = (int)mmdrv_update_sta_state(
        vif_id, aid, peer_addr, states[state]);
    EDGEZ_RF_LOG_INF("peer state mac=%02x:%02x:%02x:%02x:%02x:%02x "
                     "aid=%u edgez=%u morse=%u status=%d",
                     peer_addr[0], peer_addr[1], peer_addr[2],
                     peer_addr[3], peer_addr[4], peer_addr[5],
                     (unsigned)aid, (unsigned)state,
                     (unsigned)states[state], status);
    return status;
}

static int edgez_morse_install_key(void *driver_ctx, uint16_t vif_id, uint16_t aid,
                                   const struct edgez_umac_key *key,
                                   uint8_t *hardware_key_index)
{
    (void)driver_ctx;
    (void)vif_id;
    (void)aid;

    /* EdgeZ mesh CCMP is deliberately owned by the private UMAC.  MM8108's
     * hardware CCMP path forms a three-address AAD for mesh data represented
     * by firmware in its compatibility layout.  Programming the pairwise MTK
     * therefore makes RX hardware reject valid Linux four-address mesh frames
     * before the private UMAC can authenticate them.  Keep a logical key slot
     * only; the provider retains the key and performs both RX and TX CCMP. */
    if (!edgez_morse_mesh_ccmp_software_only())
    {
        return (int)MMWLAN_UNAVAILABLE;
    }
    *hardware_key_index = key->key_index;
    return (int)MMWLAN_SUCCESS;
}

static int edgez_morse_disable_key(void *driver_ctx, uint16_t vif_id, uint16_t aid,
                                   uint8_t hardware_key_index, bool pairwise)
{
    (void)driver_ctx;
    (void)vif_id;
    (void)aid;
    (void)hardware_key_index;
    (void)pairwise;
    return (int)MMWLAN_SUCCESS;
}

static int edgez_morse_transmit(void *driver_ctx, uint16_t vif_id,
                                const struct edgez_umac_tx_request *request)
{
    uint8_t packet_class;
    uint8_t management_frame[EDGEZ_MORSE_MGMT_MAX_LEN];
    const uint8_t *frame = request->frame;
    size_t frame_length = request->frame_length;
    (void)driver_ctx;

    if (vif_id > UINT8_MAX || request->frame_length > UINT32_MAX)
    {
        return (int)MMWLAN_INVALID_ARGUMENT;
    }
    if (request->frame_type == EDGEZ_UMAC_TX_MANAGEMENT)
    {
        packet_class = MMDRV_PKT_CLASS_MGMT;
        frame_length = edgez_morse_prepare_management_frame(
            request->frame, request->frame_length, management_frame);
        if (frame_length == 0U)
        {
            return (int)MMWLAN_INVALID_ARGUMENT;
        }
        frame = management_frame;
    }
    else if (request->tid <= 7U)
    {
        packet_class = (uint8_t)(MMDRV_PKT_CLASS_DATA_TID0 + request->tid);
    }
    else
    {
        return (int)MMWLAN_INVALID_ARGUMENT;
    }

    struct mmpkt *packet = mmdrv_alloc_mmpkt_for_tx(packet_class, 0,
                                                     (uint32_t)frame_length);
    if (packet == NULL)
    {
        return (int)MMWLAN_NO_MEM;
    }
    struct mmpktview *view = mmpkt_open(packet);
    mmpkt_append_data(view, frame, (uint32_t)frame_length);
    mmpkt_close(&view);

    struct mmdrv_tx_metadata *metadata = mmdrv_get_tx_metadata(packet);
    metadata->vif_id = (uint8_t)vif_id;
    metadata->aid = request->aid;
    /* EdgeZ uses a non-QoS TID sentinel for management frames.  The Morse
     * firmware ABI does not accept that sentinel in the TX descriptor: its
     * native management path maps management frames to the highest supported
     * QoS TID instead.  Keep this vendor-specific translation at the RF
     * boundary so the generic UMAC does not depend on Morse constants. */
    metadata->tid = request->frame_type == EDGEZ_UMAC_TX_MANAGEMENT ?
        MMWLAN_MAX_QOS_TID : request->tid;
    metadata->key_idx = edgez_morse_tx_descriptor_key_index(
        request->hardware_encrypt, request->key_index);
    metadata->flags = 0;
    if (request->hardware_encrypt)
    {
        metadata->flags |= MMDRV_TX_FLAG_HW_ENC;
    }
    if (request->no_ack)
    {
        metadata->flags |= MMDRV_TX_FLAG_NO_ACK;
    }
    if (request->immediate_status)
    {
        metadata->flags |= MMDRV_TX_FLAG_IMMEDIATE_REPORT;
    }
    if (request->ampdu_enabled)
    {
        metadata->flags |= MMDRV_TX_FLAG_AMPDU_ENABLED;
    }
    metadata->tid_max_reorder_buf_size = request->reorder_buffer_size;
    metadata->is_mgmt = request->frame_type == EDGEZ_UMAC_TX_MANAGEMENT;

    const bool management =
        request->frame_type == EDGEZ_UMAC_TX_MANAGEMENT;
    if (!edgez_morse_set_rate(metadata, request->mcs,
                              request->bandwidth_mhz,
                              edgez_morse_tx_short_guard(
                                  management, request->short_guard),
                              request->rts_cts))
    {
        mmpkt_release(packet);
        return (int)MMWLAN_INVALID_ARGUMENT;
    }
    metadata->rc_data.rates[0].attempts = edgez_morse_tx_attempts(
        management, request->attempts);
    if (request->frame_type == EDGEZ_UMAC_TX_MANAGEMENT)
    {
        MMLOG_DBG("EdgeZ raw management TX len=%lu vif=%u aid=%u tid=%u "
                  "mcs=%u bw=%u attempts=%u flags=0x%02x\n",
                  (unsigned long)frame_length,
                  (unsigned)vif_id, (unsigned)request->aid,
                  (unsigned)request->tid, (unsigned)request->mcs,
                  (unsigned)request->bandwidth_mhz,
                  (unsigned)metadata->rc_data.rates[0].attempts,
                  (unsigned)metadata->flags);
        MMLOG_DBG("EdgeZ raw management FC=%02x%02x DA=%02x:%02x:%02x:%02x:%02x:%02x "
                  "SA=%02x:%02x:%02x:%02x:%02x:%02x alg=%02x%02x txn=%02x%02x\n",
                  frame[1], frame[0],
                  frame[4], frame[5], frame[6],
                  frame[7], frame[8], frame[9],
                  frame[10], frame[11], frame[12],
                  frame[13], frame[14], frame[15],
                  frame[25], frame[24], frame[27], frame[26]);
    }
    if (!edgez_morse_track_tx(packet, request->token))
    {
        mmpkt_release(packet);
        return (int)MMWLAN_UNAVAILABLE;
    }
    enum mmwlan_status status = mmdrv_tx_frame(
        packet, request->frame_type == EDGEZ_UMAC_TX_MANAGEMENT);
    if (request->frame_type == EDGEZ_UMAC_TX_MANAGEMENT)
    {
        const uint16_t frame_control = frame_length >= 2U ?
            edgez_morse_get_le16(frame) : 0U;
        const uint16_t algorithm = frame_length >= 30U ?
            edgez_morse_get_le16(&frame[24]) : 0xffffU;
        const uint16_t transaction = frame_length >= 30U ?
            edgez_morse_get_le16(&frame[26]) : 0xffffU;
        EDGEZ_RF_LOG_INF("management TX fc=0x%04x len=%u aid=%u "
                         "alg=%u transaction=%u status=%d",
                         frame_control, (unsigned)frame_length,
                         (unsigned)request->aid, (unsigned)algorithm,
                         (unsigned)transaction, (int)status);
    }
    if (status != MMWLAN_SUCCESS)
    {
        (void)edgez_morse_take_tx_token(packet);
    }
    return (int)status;
}

const struct edgez_rf_backend_ops *edgez_rf_driver_ops(void)
{
    static const struct edgez_rf_backend_ops ops = {
        .abi_version = EDGEZ_RF_BACKEND_ABI_VERSION,
        .size = sizeof(struct edgez_rf_backend_ops),
        .name = "morse-mm-iot-sdk",
        .prepare = edgez_morse_prepare,
        .release = edgez_morse_release,
        .radio_start = edgez_morse_radio_start,
        .radio_stop = edgez_morse_radio_stop,
        .add_mesh_interface = edgez_morse_add_mesh_interface,
        .remove_interface = edgez_morse_remove_interface,
        .set_channel = edgez_morse_set_channel,
        .configure_bss = edgez_morse_configure_bss,
        .set_beacon_template = edgez_morse_set_beacon_template,
        .set_beaconing = edgez_morse_set_beaconing,
        .set_peer_state = edgez_morse_set_peer_state,
        .install_key = edgez_morse_install_key,
        .disable_key = edgez_morse_disable_key,
        .transmit = edgez_morse_transmit,
    };
    return &ops;
}

/* Temporary binary-compatibility alias for pre-edgez-rf-api archives. */
const struct edgez_rf_backend_ops *edgez_umac_morse_driver_ops(void)
{
    return edgez_rf_driver_ops();
}

int edgez_umac_morse_dispatch_rx(struct edgez_umac *umac,
                                 struct mmpkt *packet, uint16_t driver_channel)
{
    if (packet == NULL)
    {
        return EDGEZ_UMAC_INVALID_ARGUMENT;
    }

    struct mmdrv_rx_metadata *metadata = mmdrv_get_rx_metadata(packet);
    struct mmpktview *view = mmpkt_open(packet);
    struct edgez_umac_rx_frame frame = {
        .data = mmpkt_get_data_start(view),
        .length = mmpkt_get_data_length(view),
        .frequency_hz = (uint32_t)metadata->freq_100khz * 100000U,
        .timestamp_ms = metadata->read_timestamp_ms,
        .rssi_dbm = metadata->rssi,
        .noise_dbm = metadata->noise_dbm,
        .driver_channel = driver_channel,
        .bandwidth_mhz = metadata->bw_mhz,
        .vif_id = metadata->vif_id,
        .decrypted = (metadata->flags & MMDRV_RX_FLAG_DECRYPTED) != 0U,
    };
    const uint16_t frame_control = frame.length >= 2U ?
        edgez_morse_get_le16(frame.data) : 0U;
    const bool normalized = edgez_umac_morse_normalize_rx(&frame);
    int result = edgez_umac_dispatch_rx_frame(umac, &frame);
    EDGEZ_RF_LOG_DBG("RX fc=0x%04x type=%u subtype=%u len=%u "
                     "freq=%u bw=%u rssi=%d normalized=%u format=%u result=%d",
                     frame_control, (unsigned)((frame_control >> 2) & 0x03U),
                     (unsigned)((frame_control >> 4) & 0x0fU),
                     (unsigned)frame.length, (unsigned)frame.frequency_hz,
                     (unsigned)frame.bandwidth_mhz, (int)frame.rssi_dbm,
                     normalized ? 1U : 0U, (unsigned)frame.format, result);
    mmpkt_close(&view);
    mmpkt_release(packet);
    return result;
}

static int edgez_umac_morse_observe_rx(struct edgez_umac *umac,
                                       struct mmpkt *packet,
                                       uint16_t driver_channel)
{
    if (packet == NULL)
    {
        return EDGEZ_UMAC_INVALID_ARGUMENT;
    }
    struct mmdrv_rx_metadata *metadata = mmdrv_get_rx_metadata(packet);
    struct mmpktview *view = mmpkt_open(packet);
    struct edgez_umac_rx_frame frame = {
        .data = mmpkt_get_data_start(view),
        .length = mmpkt_get_data_length(view),
        .frequency_hz = (uint32_t)metadata->freq_100khz * 100000U,
        .timestamp_ms = metadata->read_timestamp_ms,
        .rssi_dbm = metadata->rssi,
        .noise_dbm = metadata->noise_dbm,
        .driver_channel = driver_channel,
        .bandwidth_mhz = metadata->bw_mhz,
        .vif_id = metadata->vif_id,
        .decrypted = (metadata->flags & MMDRV_RX_FLAG_DECRYPTED) != 0U,
    };
    (void)edgez_umac_morse_normalize_rx(&frame);
    int result = edgez_umac_dispatch_rx_frame(umac, &frame);
    mmpkt_close(&view);
    return result;
}

int edgez_umac_morse_dispatch_tx_status(struct edgez_umac *umac,
                                        struct mmpkt *packet)
{
    if (packet == NULL)
    {
        return EDGEZ_UMAC_INVALID_ARGUMENT;
    }

    struct mmdrv_tx_metadata *metadata = mmdrv_get_tx_metadata(packet);
    struct mmpktview *view = mmpkt_open(packet);
    struct edgez_umac_tx_status status = {
        .token = edgez_morse_take_tx_token(packet),
        .frame = mmpkt_get_data_start(view),
        .frame_length = mmpkt_get_data_length(view),
        .vif_id = metadata->vif_id,
        .attempts = metadata->attempts,
        .status_flags =
            ((metadata->status_flags & MMDRV_TX_STATUS_FLAGS_FAIL_MASK) != 0U ?
                EDGEZ_RF_TX_STATUS_FAILED : 0U) |
            ((metadata->status_flags & MMDRV_TX_STATUS_WAS_AGGREGATED) != 0U ?
                EDGEZ_RF_TX_STATUS_AGGREGATED : 0U),
    };
    int result = edgez_umac_dispatch_tx_status(umac, &status);
    mmpkt_close(&view);
    mmpkt_release(packet);
    return result;
}

static int edgez_umac_morse_observe_tx_status(struct edgez_umac *umac,
                                               struct mmpkt *packet)
{
    if (packet == NULL)
    {
        return EDGEZ_UMAC_INVALID_ARGUMENT;
    }
    struct mmdrv_tx_metadata *metadata = mmdrv_get_tx_metadata(packet);
    struct mmpktview *view = mmpkt_open(packet);
    struct edgez_umac_tx_status status = {
        .token = edgez_morse_peek_tx_token(packet),
        .frame = mmpkt_get_data_start(view),
        .frame_length = mmpkt_get_data_length(view),
        .vif_id = metadata->vif_id,
        .attempts = metadata->attempts,
        .status_flags =
            ((metadata->status_flags & MMDRV_TX_STATUS_FLAGS_FAIL_MASK) != 0U ?
                EDGEZ_RF_TX_STATUS_FAILED : 0U) |
            ((metadata->status_flags & MMDRV_TX_STATUS_WAS_AGGREGATED) != 0U ?
                EDGEZ_RF_TX_STATUS_AGGREGATED : 0U),
    };
    int result = edgez_umac_dispatch_tx_status(umac, &status);
    mmpkt_close(&view);
    return result;
}

int edgez_umac_morse_dispatch_tx_pause(struct edgez_umac *umac,
                                       uint16_t source_mask, bool paused)
{
    return edgez_umac_dispatch_tx_pause(umac, source_mask, paused);
}

int edgez_umac_morse_dispatch_hardware_restart(struct edgez_umac *umac)
{
    return edgez_umac_dispatch_hardware_restart(umac);
}

int edgez_umac_morse_dispatch_scan_complete(struct edgez_umac *umac,
                                            int driver_scan_status)
{
    return edgez_umac_dispatch_scan_complete(umac, driver_scan_status);
}

static void callback_rx(void *event_ctx, const struct edgez_umac_rx_frame *frame)
{
    (void)event_ctx;
    (void)frame;
    atomic_fetch_add(&callback_rx_frames, 1U);
}

static void callback_tx_status(void *event_ctx,
                               const struct edgez_umac_tx_status *status)
{
    (void)event_ctx;
    (void)status;
    atomic_fetch_add(&callback_tx_status_count, 1U);
}

static void callback_tx_pause(void *event_ctx, uint16_t source_mask, bool paused)
{
    (void)event_ctx;
    (void)source_mask;
    (void)paused;
    atomic_fetch_add(&callback_tx_pause_changes, 1U);
}

static void callback_restart(void *event_ctx)
{
    (void)event_ctx;
    atomic_fetch_add(&callback_hardware_restarts, 1U);
}

static void callback_scan(void *event_ctx, int driver_scan_status)
{
    (void)event_ctx;
    (void)driver_scan_status;
    atomic_fetch_add(&callback_scan_completions, 1U);
}

static const struct edgez_umac_event_ops callback_observer_ops = {
    .rx_frame = callback_rx,
    .tx_status = callback_tx_status,
    .tx_pause_changed = callback_tx_pause,
    .hardware_restart_required = callback_restart,
    .scan_complete = callback_scan,
};

int edgez_rf_callback_bridge_start(void)
{
    if (atomic_load(&callback_bridge_enabled))
    {
        return EDGEZ_UMAC_INVALID_STATE;
    }
    memset(&callback_observer, 0, sizeof(callback_observer));
    atomic_store(&callback_rx_frames, 0U);
    atomic_store(&callback_tx_status_count, 0U);
    atomic_store(&callback_tx_pause_changes, 0U);
    atomic_store(&callback_hardware_restarts, 0U);
    atomic_store(&callback_scan_completions, 0U);
    int result = edgez_umac_attach_observer(
        &callback_observer, &callback_observer_ops, NULL);
    if (result == EDGEZ_UMAC_OK)
    {
        atomic_store(&callback_bridge_enabled, true);
    }
    return result;
}

void edgez_rf_callback_bridge_stop(void)
{
    if (!atomic_exchange(&callback_bridge_enabled, false))
    {
        return;
    }
    (void)edgez_umac_detach_observer(&callback_observer);
}

void edgez_rf_callback_bridge_get_metrics(
    struct edgez_rf_callback_metrics *metrics)
{
    if (metrics == NULL)
    {
        return;
    }
    metrics->rx_frames = atomic_load(&callback_rx_frames);
    metrics->tx_status = atomic_load(&callback_tx_status_count);
    metrics->tx_pause_changes = atomic_load(&callback_tx_pause_changes);
    metrics->hardware_restarts = atomic_load(&callback_hardware_restarts);
    metrics->scan_completions = atomic_load(&callback_scan_completions);
}

bool edgez_rf_host_rx(void *native_packet, uint16_t driver_channel)
{
    struct mmpkt *packet = native_packet;
    if (atomic_load(&callback_bridge_enabled))
    {
        (void)edgez_umac_morse_observe_rx(&callback_observer, packet,
                                           driver_channel);
    }
    return false;
}

bool edgez_rf_host_tx_status(void *native_packet)
{
    struct mmpkt *packet = native_packet;
    if (atomic_load(&callback_bridge_enabled))
    {
        (void)edgez_umac_morse_observe_tx_status(&callback_observer, packet);
    }
    return false;
}

bool edgez_rf_host_tx_pause(uint16_t source_mask, bool paused)
{
    if (atomic_load(&callback_bridge_enabled))
    {
        (void)edgez_umac_morse_dispatch_tx_pause(
            &callback_observer, source_mask, paused);
    }
    return false;
}

bool edgez_rf_host_hardware_restart(void)
{
    if (atomic_load(&callback_bridge_enabled))
    {
        (void)edgez_umac_morse_dispatch_hardware_restart(&callback_observer);
    }
    return false;
}

bool edgez_rf_host_scan_complete(int driver_scan_status)
{
    if (atomic_load(&callback_bridge_enabled))
    {
        (void)edgez_umac_morse_dispatch_scan_complete(
            &callback_observer, driver_scan_status);
    }
    return false;
}
