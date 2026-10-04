/*
  gnss_state.h - everything known about the attached GNSS receiver

  Part of the esp32-to-gps extensions to xsns_60_GPS (github.com/mithro/esp32-to-gps).
  Pure C99: no Tasmota or Arduino dependency, so it is compiled and tested on the host.

  SPDX-License-Identifier: GPL-3.0-or-later
*/

#ifndef GNSS_STATE_H
#define GNSS_STATE_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define GNSS_MAX_SATS 64
#define GNSS_TEXT_LEN 32

/* Constellations, numbered as UBX gnssId (u-blox interface description). */
typedef enum {
  GNSS_GPS = 0,
  GNSS_SBAS = 1,
  GNSS_GALILEO = 2,
  GNSS_BEIDOU = 3,
  GNSS_IMES = 4,
  GNSS_QZSS = 5,
  GNSS_GLONASS = 6,
  GNSS_NAVIC = 7,
  GNSS_N_CONSTELLATIONS = 8,
  GNSS_UNKNOWN = 255,
} gnss_constellation_t;

/* Fix type, as UBX NAV-PVT fixType. */
typedef enum {
  GNSS_FIX_NONE = 0,
  GNSS_FIX_DEAD_RECKONING = 1,
  GNSS_FIX_2D = 2,
  GNSS_FIX_3D = 3,
  GNSS_FIX_GNSS_DR = 4,
  GNSS_FIX_TIME_ONLY = 5,
} gnss_fix_type_t;

/* Fix quality, as the NMEA GGA quality indicator. */
typedef enum {
  GNSS_QUALITY_INVALID = 0,
  GNSS_QUALITY_GPS = 1,
  GNSS_QUALITY_DGPS = 2,
  GNSS_QUALITY_PPS = 3,
  GNSS_QUALITY_RTK_FIXED = 4,
  GNSS_QUALITY_RTK_FLOAT = 5,
  GNSS_QUALITY_ESTIMATED = 6,
  GNSS_QUALITY_MANUAL = 7,
  GNSS_QUALITY_SIMULATION = 8,
} gnss_quality_t;

/* Receiver families this firmware knows how to configure. */
typedef enum {
  GNSS_MODULE_UNKNOWN = 0,
  GNSS_MODULE_UBLOX7,
  GNSS_MODULE_UBLOX_M8,     /* includes the LEA-M8T */
  GNSS_MODULE_UBLOX_M10,
  GNSS_MODULE_QUECTEL_LC29H,
  GNSS_MODULE_UBLOX_OTHER,  /* a u-blox generation not listed above */
} gnss_module_t;

/* Antenna status, as UBX MON-HW aStatus. */
typedef enum {
  GNSS_ANT_INIT = 0,
  GNSS_ANT_UNKNOWN = 1,
  GNSS_ANT_OK = 2,
  GNSS_ANT_SHORT = 3,
  GNSS_ANT_OPEN = 4,
  GNSS_ANT_NOT_REPORTED = 255,
} gnss_antenna_t;

typedef struct {
  uint8_t gnss;      /* gnss_constellation_t */
  uint8_t svid;      /* within the constellation (GPS 1-32, SBAS 120-158, ...) */
  int8_t elev;       /* degrees, -91 when not known */
  int16_t azim;      /* degrees 0-359, -1 when not known */
  uint8_t cno;       /* dB-Hz, 0 when not tracked */
  bool used;         /* used in the navigation solution */
  uint32_t epoch;    /* gnss_state_t.epoch when last reported */
} gnss_sat_t;

typedef struct {
  /* Time (UTC). */
  bool time_valid, date_valid;
  uint16_t year;
  uint8_t month, day, hour, minute, second;
  uint16_t millisecond;
  uint32_t time_accuracy_ns;        /* UBX NAV-PVT tAcc; 0 when not reported */
  uint32_t epoch;                   /* counts navigation epochs (time-of-day changes) */
  uint32_t nmea_tod_ms;             /* time of day of the current NMEA epoch, ms */

  /* Fix and position. */
  uint8_t fix_type;                 /* gnss_fix_type_t */
  uint8_t quality;                  /* gnss_quality_t */
  bool position_valid;
  int32_t lat_e7, lon_e7;           /* degrees x 1e7 */
  int32_t alt_msl_mm;               /* height above mean sea level */
  int32_t alt_ellipsoid_mm;         /* height above the WGS84 ellipsoid */
  bool alt_valid, ellipsoid_valid;
  int32_t geoid_sep_mm;             /* ellipsoid minus mean sea level */
  bool geoid_valid;
  uint32_t h_acc_mm, v_acc_mm;      /* estimated accuracy; 0 when not reported */
  bool acc_valid;

  /* Motion. */
  bool motion_valid;
  uint32_t speed_mm_s;              /* ground speed */
  int32_t course_e5;                /* degrees x 1e5 */
  uint32_t speed_acc_mm_s;

  /* Dilution of precision, x100. 0xffff when not reported. */
  uint16_t gdop, pdop, hdop, vdop, tdop;

  /* Satellites. */
  uint8_t sats_used_reported;       /* as the receiver reports it (GGA / NAV-PVT numSV) */
  uint8_t n_sats;
  gnss_sat_t sats[GNSS_MAX_SATS];

  /* Corrections, as the receiver sees them. */
  int32_t diff_age_ds;              /* GGA age of differential data, 0.1 s; -1 when none */
  int32_t diff_station;             /* GGA differential station id; -1 when none */
  bool diff_soln;                   /* UBX NAV-PVT flags.diffSoln */
  uint8_t carr_soln;                /* UBX NAV-PVT carrSoln: 0 none, 1 float, 2 fixed */
  uint32_t rtcm_used, rtcm_failed;  /* UBX RXM-RTCM: messages used / failing CRC */
  uint16_t rtcm_last_type;          /* UBX RXM-RTCM msgType */

  /* Receiver. */
  uint8_t module;                   /* gnss_module_t */
  char model[GNSS_TEXT_LEN];        /* "MAX-M10S", "LEA-M8T", "LC29HAA", ... when known */
  char sw_version[GNSS_TEXT_LEN];
  char hw_version[GNSS_TEXT_LEN];
  char protocol[GNSS_TEXT_LEN];     /* "34.10" */
  uint8_t antenna;                  /* gnss_antenna_t */
  uint8_t jamming;                  /* MON-HW jamInd / MON-RF jamInd, 0-255; 0 when not reported */
  uint16_t noise;                   /* MON-HW noisePerMS / MON-RF noisePerMS */
  bool rf_valid;
  int32_t qerr_ps;                  /* UBX TIM-TP quantization error, ps */
  bool qerr_valid;
  uint32_t ttff_ms;                 /* time to first fix */

  /* Health. */
  uint32_t nmea_ok, nmea_bad;
  uint32_t ubx_ok, ubx_bad;
  uint32_t ack, nak;
  uint8_t last_ack_cls, last_ack_id, last_nak_cls, last_nak_id;
} gnss_state_t;

/* Start from nothing known. */
void gnss_state_init(gnss_state_t *s);

/* Start a new navigation epoch: satellites not reported again are dropped
 * from the table two epochs later. */
void gnss_new_epoch(gnss_state_t *s);

/* Find or add a satellite. Returns NULL when the table is full. */
gnss_sat_t *gnss_sat(gnss_state_t *s, uint8_t gnss, uint8_t svid);

/* Remove satellites not reported in the current or previous epoch. */
void gnss_prune_sats(gnss_state_t *s);

/* Satellites in view / used for one constellation, or every one when
 * gnss is GNSS_UNKNOWN. */
uint8_t gnss_count_sats(const gnss_state_t *s, uint8_t gnss, bool used_only);

/* C/N0 of the satellites used in the fix: returns how many contributed. */
uint8_t gnss_cno_stats(const gnss_state_t *s, uint8_t *min, uint8_t *avg, uint8_t *max);

/* Short name for a constellation: "GPS", "GLONASS", ... */
const char *gnss_constellation_name(uint8_t gnss);

/* One-letter RINEX-style prefix: G, S, E, C, I, J, R. */
char gnss_constellation_letter(uint8_t gnss);

#ifdef __cplusplus
}
#endif

#endif /* GNSS_STATE_H */
