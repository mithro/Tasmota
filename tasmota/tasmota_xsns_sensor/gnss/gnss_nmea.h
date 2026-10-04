/*
  gnss_nmea.h - decode NMEA 0183 sentences into the receiver state

  Standard sentences from any talker (GP, GL, GA, GB, BD, GQ, GI, GN),
  including the NMEA 4.10 / 4.11 system and signal IDs, plus the Quectel
  replies the driver asks for.

  SPDX-License-Identifier: GPL-3.0-or-later
*/

#ifndef GNSS_NMEA_H
#define GNSS_NMEA_H

#include "gnss_state.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
  GNSS_NMEA_OTHER = 0,  /* valid, but not one this parser uses */
  GNSS_NMEA_GGA,
  GNSS_NMEA_RMC,
  GNSS_NMEA_GSA,
  GNSS_NMEA_GSV,
  GNSS_NMEA_VTG,
  GNSS_NMEA_GLL,
  GNSS_NMEA_ZDA,
  GNSS_NMEA_GST,
  GNSS_NMEA_TXT,
  GNSS_NMEA_PQTMVERNO,  /* Quectel firmware version reply */
  GNSS_NMEA_PAIR_ACK,   /* Quectel $PAIR001 command acknowledgement */
  GNSS_NMEA_PAIR021,    /* Quectel firmware / chip version reply */
} gnss_nmea_kind_t;

/* Decode one sentence, as framed by gnss_stream: from '$' up to (not
 * including) '*'. Returns what kind of sentence it was. */
gnss_nmea_kind_t gnss_nmea_parse(gnss_state_t *s, const char *sentence);

/* Constellation and satellite ID for an NMEA satellite number. talker is
 * the constellation the talker ID names (GNSS_UNKNOWN for GN), system_id
 * the NMEA 4.10 system ID (0 when absent). Returns false when the number is
 * not one this firmware can place. */
bool gnss_nmea_satellite(uint8_t talker, uint8_t system_id, int prn, uint8_t *gnss, uint8_t *svid);

#ifdef __cplusplus
}
#endif

#endif /* GNSS_NMEA_H */
