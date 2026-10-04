/*
  gnss_ubx.h - decode and build u-blox UBX messages

  Layouts follow the u-blox interface descriptions for the u-blox 7 (protocol
  14), M8 (protocol 15-23) and M10 (protocol 34); the host tests check every
  message against pyubx2.

  SPDX-License-Identifier: GPL-3.0-or-later
*/

#ifndef GNSS_UBX_H
#define GNSS_UBX_H

#include <stddef.h>

#include "gnss_state.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
  GNSS_UBX_OTHER = 0,
  GNSS_UBX_NAV_PVT,
  GNSS_UBX_NAV_DOP,
  GNSS_UBX_NAV_SAT,
  GNSS_UBX_NAV_SVINFO,
  GNSS_UBX_MON_VER,
  GNSS_UBX_MON_HW,
  GNSS_UBX_MON_RF,
  GNSS_UBX_RXM_RTCM,
  GNSS_UBX_TIM_TP,
  GNSS_UBX_ACK,
  GNSS_UBX_NAK,
  GNSS_UBX_SHORT,  /* a known message too short to decode */
} gnss_ubx_kind_t;

/* Decode one frame's payload into the state. */
gnss_ubx_kind_t gnss_ubx_parse(gnss_state_t *s, uint8_t cls, uint8_t id, const uint8_t *payload, uint16_t len);

/* Constellation and satellite ID for a satellite number in u-blox 7 UBX
 * numbering (NAV-SVINFO). Returns false for numbers it cannot place. */
bool gnss_ubx_svid(uint8_t svid, uint8_t *gnss, uint8_t *sat);

/* Build a complete UBX frame (sync, header, payload, checksum) into out.
 * Returns its length, or 0 when out is too small. */
size_t gnss_ubx_frame(uint8_t *out, size_t out_len, uint8_t cls, uint8_t id, const uint8_t *payload, uint16_t len);

/* A poll: the message with no payload. */
size_t gnss_ubx_poll(uint8_t *out, size_t out_len, uint8_t cls, uint8_t id);

/* Legacy CFG-MSG (u-blox 7, M8): output rate of one message on the port the
 * command arrives on, in navigation solutions (0 = off). */
size_t gnss_ubx_cfg_msg(uint8_t *out, size_t out_len, uint8_t cls, uint8_t id, uint8_t rate);

/* CFG-VALSET (M9, M10) into the given layers (1 = RAM, 2 = BBR, 4 = flash).
 * The size of each value comes from its key ID. Returns 0 when out is too
 * small or a key's size is not known. */
size_t gnss_ubx_cfg_valset(uint8_t *out, size_t out_len, uint8_t layers,
                           const uint32_t *keys, const uint64_t *values, uint8_t n);

#ifdef __cplusplus
}
#endif

#endif /* GNSS_UBX_H */
