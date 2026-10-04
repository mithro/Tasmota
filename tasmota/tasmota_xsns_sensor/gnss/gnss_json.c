/*
  gnss_json.c - the receiver state as JSON, and its Home Assistant discovery

  SPDX-License-Identifier: GPL-3.0-or-later
*/

#include "gnss_json.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

/* --- a bounded JSON writer --------------------------------------------------- */

typedef struct {
  char *p;
  size_t len, cap;
  bool overflow;
  uint8_t depth;
  bool first[8];  /* no member written yet at this depth */
} jw_t;

static void jw_init(jw_t *w, char *out, size_t cap) {
  memset(w, 0, sizeof(*w));
  w->p = out;
  w->cap = cap;
  w->first[0] = true;
  if (cap) { out[0] = 0; }
}

static void jw_raw(jw_t *w, const char *s) {
  size_t n = strlen(s);
  if (w->overflow || w->len + n + 1 > w->cap) { w->overflow = true; return; }
  memcpy(w->p + w->len, s, n + 1);
  w->len += n;
}

static void jw_fmt(jw_t *w, const char *fmt, ...) {
  if (w->overflow) { return; }
  va_list ap;
  va_start(ap, fmt);
  int n = vsnprintf(w->p + w->len, w->cap - w->len, fmt, ap);
  va_end(ap);
  if (n < 0 || w->len + (size_t)n + 1 > w->cap) { w->overflow = true; return; }
  w->len += (size_t)n;
}

static void jw_str(jw_t *w, const char *s) {
  jw_raw(w, "\"");
  for (; *s && !w->overflow; s++) {
    unsigned char c = (unsigned char)*s;
    if (c == '"' || c == '\\') { jw_fmt(w, "\\%c", c); }
    else if (c < 0x20) { jw_fmt(w, "\\u%04x", c); }
    else { char t[2] = {(char)c, 0}; jw_raw(w, t); }
  }
  jw_raw(w, "\"");
}

static void jw_key(jw_t *w, const char *key) {
  if (!w->first[w->depth]) { jw_raw(w, ","); }
  w->first[w->depth] = false;
  if (key) {
    jw_str(w, key);
    jw_raw(w, ":");
  }
}

static void jw_open(jw_t *w, const char *key, const char *bracket) {
  if (w->depth) { jw_key(w, key); }
  jw_raw(w, bracket);
  if (w->depth < 7) { w->first[++w->depth] = true; }
}

static void jw_close(jw_t *w, const char *bracket) {
  jw_raw(w, bracket);
  if (w->depth) { w->depth--; }
}

static void jw_null(jw_t *w, const char *key) { jw_key(w, key); jw_raw(w, "null"); }
static void jw_bool(jw_t *w, const char *key, bool v) { jw_key(w, key); jw_raw(w, v ? "true" : "false"); }
static void jw_int(jw_t *w, const char *key, int64_t v) { jw_key(w, key); jw_fmt(w, "%lld", (long long)v); }
static void jw_text(jw_t *w, const char *key, const char *v) {
  if (v && *v) { jw_key(w, key); jw_str(w, v); } else { jw_null(w, key); }
}

/* value / 10^decimals, printed exactly: no floating point. */
static void jw_fixed(jw_t *w, const char *key, int64_t v, int decimals, bool valid) {
  if (!valid) { jw_null(w, key); return; }
  jw_key(w, key);
  int64_t scale = 1;
  for (int i = 0; i < decimals; i++) { scale *= 10; }
  uint64_t a = (uint64_t)(v < 0 ? -v : v);
  if (decimals) {
    jw_fmt(w, "%s%llu.%0*llu", v < 0 ? "-" : "", (unsigned long long)(a / (uint64_t)scale), decimals,
           (unsigned long long)(a % (uint64_t)scale));
  } else {
    jw_fmt(w, "%s%llu", v < 0 ? "-" : "", (unsigned long long)a);
  }
}

static size_t jw_done(jw_t *w) {
  return w->overflow ? 0 : w->len;
}

/* --- names ---------------------------------------------------------------------- */

static const char *fix_name(uint8_t f) {
  switch (f) {
    case GNSS_FIX_NONE: return "No fix";
    case GNSS_FIX_DEAD_RECKONING: return "Dead reckoning";
    case GNSS_FIX_2D: return "2D";
    case GNSS_FIX_3D: return "3D";
    case GNSS_FIX_GNSS_DR: return "GNSS + dead reckoning";
    case GNSS_FIX_TIME_ONLY: return "Time only";
    default: return "Unknown";
  }
}

static const char *quality_name(uint8_t q) {
  switch (q) {
    case GNSS_QUALITY_INVALID: return "No fix";
    case GNSS_QUALITY_GPS: return "GNSS";
    case GNSS_QUALITY_DGPS: return "DGNSS";
    case GNSS_QUALITY_PPS: return "PPS";
    case GNSS_QUALITY_RTK_FIXED: return "RTK fixed";
    case GNSS_QUALITY_RTK_FLOAT: return "RTK float";
    case GNSS_QUALITY_ESTIMATED: return "Estimated";
    case GNSS_QUALITY_MANUAL: return "Manual";
    case GNSS_QUALITY_SIMULATION: return "Simulation";
    default: return "Unknown";
  }
}

static const char *antenna_name(uint8_t a) {
  switch (a) {
    case GNSS_ANT_INIT: return "Initialising";
    case GNSS_ANT_UNKNOWN: return "Unknown";
    case GNSS_ANT_OK: return "OK";
    case GNSS_ANT_SHORT: return "Short";
    case GNSS_ANT_OPEN: return "Open";
    default: return "";
  }
}

static const char *corr_name(uint8_t c) {
  switch (c) {
    case GNSS_CORR_DISABLED: return "Disabled";
    case GNSS_CORR_CONNECTING: return "Connecting";
    case GNSS_CORR_CONNECTED: return "Connected";
    case GNSS_CORR_RETRYING: return "Retrying";
    default: return "Unknown";
  }
}

/* The six constellations every receiver here can report, in display order. */
static const uint8_t SHOWN[] = {GNSS_GPS, GNSS_GLONASS, GNSS_GALILEO, GNSS_BEIDOU, GNSS_QZSS, GNSS_SBAS};
#define N_SHOWN (sizeof(SHOWN) / sizeof(SHOWN[0]))

/* --- tele/<topic>/GNSS ------------------------------------------------------------ */

static void dop(jw_t *w, const char *key, uint16_t v) { jw_fixed(w, key, v, 2, v != 0xffff); }

size_t gnss_json_state(char *out, size_t out_len, const gnss_state_t *s, const gnss_extra_t *x) {
  jw_t w;
  jw_init(&w, out, out_len);
  jw_open(&w, 0, "{");

  if (s->time_valid && s->date_valid) {
    char t[32];
    snprintf(t, sizeof(t), "%04u-%02u-%02uT%02u:%02u:%02uZ", (unsigned)s->year, (unsigned)s->month, (unsigned)s->day,
             (unsigned)s->hour, (unsigned)s->minute, (unsigned)s->second);
    jw_text(&w, "Time", t);
  } else {
    jw_null(&w, "Time");
  }
  jw_text(&w, "Fix", fix_name(s->fix_type));
  jw_int(&w, "FixType", s->fix_type);
  jw_text(&w, "Quality", quality_name(s->quality));
  jw_int(&w, "QualityId", s->quality);

  bool pos = s->position_valid;
  jw_fixed(&w, "Lat", s->lat_e7, 7, pos);
  jw_fixed(&w, "Lon", s->lon_e7, 7, pos);
  jw_fixed(&w, "AltMSL", s->alt_msl_mm, 3, pos && s->alt_valid);
  jw_fixed(&w, "AltEllipsoid", s->alt_ellipsoid_mm, 3, pos && s->ellipsoid_valid);
  jw_fixed(&w, "GeoidSep", s->geoid_sep_mm, 3, s->geoid_valid);
  jw_fixed(&w, "HAcc", s->h_acc_mm, 3, s->acc_valid && pos);
  jw_fixed(&w, "VAcc", s->v_acc_mm, 3, s->acc_valid && pos);
  jw_fixed(&w, "SAcc", s->speed_acc_mm_s, 3, s->motion_valid && s->speed_acc_mm_s);
  jw_fixed(&w, "TAcc", s->time_accuracy_ns, 0, s->time_accuracy_ns != 0);
  jw_fixed(&w, "Speed", s->speed_mm_s, 3, s->motion_valid);
  jw_fixed(&w, "Course", s->course_e5, 5, s->motion_valid);
  dop(&w, "HDOP", s->hdop);
  dop(&w, "VDOP", s->vdop);
  dop(&w, "PDOP", s->pdop);
  dop(&w, "GDOP", s->gdop);
  dop(&w, "TDOP", s->tdop);

  jw_int(&w, "SatsUsed", s->n_sats ? gnss_count_sats(s, GNSS_UNKNOWN, true) : s->sats_used_reported);
  jw_int(&w, "SatsInView", gnss_count_sats(s, GNSS_UNKNOWN, false));
  jw_open(&w, "Constellations", "{");
  for (size_t i = 0; i < N_SHOWN; i++) {
    jw_open(&w, gnss_constellation_name(SHOWN[i]), "{");
    jw_int(&w, "InView", gnss_count_sats(s, SHOWN[i], false));
    jw_int(&w, "Used", gnss_count_sats(s, SHOWN[i], true));
    jw_close(&w, "}");
  }
  jw_close(&w, "}");
  uint8_t lo, avg, hi;
  bool cno = gnss_cno_stats(s, &lo, &avg, &hi) > 0;
  jw_open(&w, "CNo", "{");
  jw_fixed(&w, "Min", lo, 0, cno);
  jw_fixed(&w, "Avg", avg, 0, cno);
  jw_fixed(&w, "Max", hi, 0, cno);
  jw_close(&w, "}");

  jw_fixed(&w, "DiffAge", s->diff_age_ds, 1, s->diff_age_ds >= 0);
  jw_fixed(&w, "DiffStation", s->diff_station, 0, s->diff_station >= 0);

  const gnss_rtcm_t *r = x->rtcm;
  jw_open(&w, "Corrections", "{");
  jw_text(&w, "State", corr_name(x->corr_state));
  jw_text(&w, "Mount", x->corr_mount);
  jw_text(&w, "Error", x->corr_error);
  jw_int(&w, "Bytes", r ? r->bytes : 0);
  jw_int(&w, "Frames", r ? r->frames : 0);
  jw_int(&w, "BadCRC", r ? r->bad_crc : 0);
  jw_fixed(&w, "Age", x->corr_age_s, 0, x->corr_age_s >= 0);
  char types[GNSS_RTCM_TYPES * 6] = "";
  if (r) {
    for (uint8_t i = 0; i < r->n_types; i++) {
      size_t n = strlen(types);
      snprintf(types + n, sizeof(types) - n, "%s%u", i ? "," : "", (unsigned)r->types[i]);
    }
  }
  jw_text(&w, "Types", types);
  jw_fixed(&w, "Station", r ? r->station : 0, 0, r && r->station_valid);
  jw_int(&w, "Used", s->rtcm_used);
  jw_int(&w, "Failed", s->rtcm_failed);
  jw_close(&w, "}");

  jw_open(&w, "Receiver", "{");
  jw_text(&w, "Model", s->model);
  jw_text(&w, "Firmware", s->sw_version);
  jw_text(&w, "Hardware", s->hw_version);
  jw_text(&w, "Protocol", s->protocol);
  jw_fixed(&w, "Baud", x->baud, 0, x->baud != 0);
  jw_text(&w, "Antenna", antenna_name(s->antenna));
  jw_fixed(&w, "Jamming", s->jamming, 0, s->rf_valid);
  jw_fixed(&w, "Noise", s->noise, 0, s->rf_valid);
  jw_fixed(&w, "QErr", s->qerr_ps, 0, s->qerr_valid);
  jw_close(&w, "}");

  jw_open(&w, "PPS", "{");
  jw_bool(&w, "Present", x->pps_present);
  jw_int(&w, "Count", x->pps_count);
  jw_close(&w, "}");

  jw_open(&w, "Health", "{");
  jw_int(&w, "NMEA", s->nmea_ok);
  jw_int(&w, "NMEABad", s->nmea_bad);
  jw_int(&w, "UBX", s->ubx_ok);
  jw_int(&w, "UBXBad", s->ubx_bad);
  jw_fixed(&w, "LastData", x->data_age_s, 0, x->data_age_s >= 0);
  jw_close(&w, "}");

  jw_close(&w, "}");
  return jw_done(&w);
}

/* --- tele/<topic>/GNSS_SATS --------------------------------------------------------- */

static void sat_id(char *out, size_t len, uint8_t gnss, uint8_t svid) {
  snprintf(out, len, "%c%02u", gnss_constellation_letter(gnss), (unsigned)svid);
}

size_t gnss_json_sats(char *out, size_t out_len, const gnss_state_t *s) {
  jw_t w;
  jw_init(&w, out, out_len);
  jw_open(&w, 0, "{");
  jw_int(&w, "InView", gnss_count_sats(s, GNSS_UNKNOWN, false));
  jw_int(&w, "Used", gnss_count_sats(s, GNSS_UNKNOWN, true));
  /* [id, constellation, elevation, azimuth, C/N0, used]; null for unknown */
  jw_open(&w, "Sats", "[");
  for (uint8_t i = 0; i < s->n_sats; i++) {
    const gnss_sat_t *t = &s->sats[i];
    char id[8];
    sat_id(id, sizeof(id), t->gnss, t->svid);
    jw_open(&w, 0, "[");
    jw_text(&w, 0, id);
    jw_text(&w, 0, gnss_constellation_name(t->gnss));
    jw_fixed(&w, 0, t->elev, 0, t->elev >= -90);
    jw_fixed(&w, 0, t->azim, 0, t->azim >= 0);
    jw_fixed(&w, 0, t->cno, 0, t->cno > 0);
    jw_bool(&w, 0, t->used);
    jw_close(&w, "]");
  }
  jw_close(&w, "]");
  /* C/N0 by satellite ID, for the per-satellite entities' templates */
  jw_open(&w, "CNo", "{");
  for (uint8_t i = 0; i < s->n_sats; i++) {
    char id[8];
    sat_id(id, sizeof(id), s->sats[i].gnss, s->sats[i].svid);
    jw_fixed(&w, id, s->sats[i].cno, 0, s->sats[i].cno > 0);
  }
  jw_close(&w, "}");
  jw_close(&w, "}");
  return jw_done(&w);
}

/* --- Home Assistant discovery --------------------------------------------------------- */

enum { T_GNSS, T_SATS, T_TRACKER };

typedef struct {
  const char *id;        /* entity suffix: unique_id = <uid>_<id> */
  const char *comp;      /* sensor, binary_sensor, device_tracker */
  const char *name;
  const char *path;      /* value_json.<path> */
  const char *unit;
  const char *dev_cla;
  const char *stat_cla;
  const char *icon;
  uint8_t diag;          /* entity_category: diagnostic */
  uint8_t topic;
} entity_t;

#define MEAS "measurement"
#define TOTAL "total_increasing"

static const entity_t ENTITIES[] = {
  {"location", "device_tracker", "Location", 0, 0, 0, 0, "mdi:crosshairs-gps", 0, T_TRACKER},
  {"lat", "sensor", "Latitude", "Lat", "\xc2\xb0", 0, MEAS, "mdi:latitude", 0, T_GNSS},
  {"lon", "sensor", "Longitude", "Lon", "\xc2\xb0", 0, MEAS, "mdi:longitude", 0, T_GNSS},
  {"alt", "sensor", "Altitude", "AltMSL", "m", "distance", MEAS, "mdi:altimeter", 0, T_GNSS},
  {"alt_ellipsoid", "sensor", "Height above ellipsoid", "AltEllipsoid", "m", "distance", MEAS, "mdi:altimeter", 1, T_GNSS},
  {"geoid_sep", "sensor", "Geoid separation", "GeoidSep", "m", "distance", MEAS, "mdi:earth", 1, T_GNSS},
  {"fix", "sensor", "Fix", "Fix", 0, 0, 0, "mdi:crosshairs-gps", 0, T_GNSS},
  {"quality", "sensor", "Fix quality", "Quality", 0, 0, 0, "mdi:crosshairs-question", 0, T_GNSS},
  {"h_acc", "sensor", "Horizontal accuracy", "HAcc", "m", "distance", MEAS, "mdi:map-marker-radius", 0, T_GNSS},
  {"v_acc", "sensor", "Vertical accuracy", "VAcc", "m", "distance", MEAS, "mdi:arrow-expand-vertical", 0, T_GNSS},
  {"s_acc", "sensor", "Speed accuracy", "SAcc", "m/s", "speed", MEAS, "mdi:speedometer", 1, T_GNSS},
  {"t_acc", "sensor", "Time accuracy", "TAcc", "ns", 0, MEAS, "mdi:timer-outline", 1, T_GNSS},
  {"hdop", "sensor", "HDOP", "HDOP", 0, 0, MEAS, "mdi:chart-bell-curve", 0, T_GNSS},
  {"vdop", "sensor", "VDOP", "VDOP", 0, 0, MEAS, "mdi:chart-bell-curve", 1, T_GNSS},
  {"pdop", "sensor", "PDOP", "PDOP", 0, 0, MEAS, "mdi:chart-bell-curve", 1, T_GNSS},
  {"gdop", "sensor", "GDOP", "GDOP", 0, 0, MEAS, "mdi:chart-bell-curve", 1, T_GNSS},
  {"tdop", "sensor", "TDOP", "TDOP", 0, 0, MEAS, "mdi:chart-bell-curve", 1, T_GNSS},
  {"speed", "sensor", "Speed", "Speed", "m/s", "speed", MEAS, "mdi:speedometer", 0, T_GNSS},
  {"course", "sensor", "Course", "Course", "\xc2\xb0", 0, 0, "mdi:compass", 0, T_GNSS},
  {"time", "sensor", "GNSS time", "Time", 0, "timestamp", 0, "mdi:clock-outline", 0, T_GNSS},
  {"pps", "binary_sensor", "PPS", "PPS.Present", 0, 0, 0, "mdi:pulse", 1, T_GNSS},
  {"pps_count", "sensor", "PPS pulses", "PPS.Count", 0, 0, TOTAL, "mdi:pulse", 1, T_GNSS},
  {"sats_used", "sensor", "Satellites used", "SatsUsed", 0, 0, MEAS, "mdi:satellite-variant", 0, T_GNSS},
  {"sats_in_view", "sensor", "Satellites in view", "SatsInView", 0, 0, MEAS, "mdi:satellite-variant", 0, T_GNSS},
  {"gps_in_view", "sensor", "GPS satellites in view", "Constellations.GPS.InView", 0, 0, MEAS, "mdi:satellite-variant", 0, T_GNSS},
  {"gps_used", "sensor", "GPS satellites used", "Constellations.GPS.Used", 0, 0, MEAS, "mdi:satellite-variant", 0, T_GNSS},
  {"glonass_in_view", "sensor", "GLONASS satellites in view", "Constellations.GLONASS.InView", 0, 0, MEAS, "mdi:satellite-variant", 0, T_GNSS},
  {"glonass_used", "sensor", "GLONASS satellites used", "Constellations.GLONASS.Used", 0, 0, MEAS, "mdi:satellite-variant", 0, T_GNSS},
  {"galileo_in_view", "sensor", "Galileo satellites in view", "Constellations.Galileo.InView", 0, 0, MEAS, "mdi:satellite-variant", 0, T_GNSS},
  {"galileo_used", "sensor", "Galileo satellites used", "Constellations.Galileo.Used", 0, 0, MEAS, "mdi:satellite-variant", 0, T_GNSS},
  {"beidou_in_view", "sensor", "BeiDou satellites in view", "Constellations.BeiDou.InView", 0, 0, MEAS, "mdi:satellite-variant", 0, T_GNSS},
  {"beidou_used", "sensor", "BeiDou satellites used", "Constellations.BeiDou.Used", 0, 0, MEAS, "mdi:satellite-variant", 0, T_GNSS},
  {"qzss_in_view", "sensor", "QZSS satellites in view", "Constellations.QZSS.InView", 0, 0, MEAS, "mdi:satellite-variant", 0, T_GNSS},
  {"qzss_used", "sensor", "QZSS satellites used", "Constellations.QZSS.Used", 0, 0, MEAS, "mdi:satellite-variant", 0, T_GNSS},
  {"sbas_in_view", "sensor", "SBAS satellites in view", "Constellations.SBAS.InView", 0, 0, MEAS, "mdi:satellite-variant", 0, T_GNSS},
  {"sbas_used", "sensor", "SBAS satellites used", "Constellations.SBAS.Used", 0, 0, MEAS, "mdi:satellite-variant", 0, T_GNSS},
  {"cno_min", "sensor", "Signal C/N0 minimum", "CNo.Min", "dB-Hz", 0, MEAS, "mdi:signal", 0, T_GNSS},
  {"cno_avg", "sensor", "Signal C/N0 average", "CNo.Avg", "dB-Hz", 0, MEAS, "mdi:signal", 0, T_GNSS},
  {"cno_max", "sensor", "Signal C/N0 maximum", "CNo.Max", "dB-Hz", 0, MEAS, "mdi:signal", 0, T_GNSS},
  {"satellites", "sensor", "Satellites", "InView", 0, 0, MEAS, "mdi:satellite-uplink", 0, T_SATS},
  {"corr_state", "sensor", "Corrections", "Corrections.State", 0, 0, 0, "mdi:transit-connection-variant", 0, T_GNSS},
  {"corr_mount", "sensor", "Corrections mountpoint", "Corrections.Mount", 0, 0, 0, "mdi:transit-connection-variant", 1, T_GNSS},
  {"corr_error", "sensor", "Corrections error", "Corrections.Error", 0, 0, 0, "mdi:alert-circle-outline", 1, T_GNSS},
  {"corr_bytes", "sensor", "Corrections received", "Corrections.Bytes", "B", "data_size", TOTAL, "mdi:download", 1, T_GNSS},
  {"corr_frames", "sensor", "RTCM frames received", "Corrections.Frames", 0, 0, TOTAL, "mdi:download", 1, T_GNSS},
  {"corr_bad_crc", "sensor", "RTCM CRC errors", "Corrections.BadCRC", 0, 0, TOTAL, "mdi:alert", 1, T_GNSS},
  {"corr_age", "sensor", "Corrections age", "Corrections.Age", "s", "duration", MEAS, "mdi:timer-sand", 0, T_GNSS},
  {"corr_types", "sensor", "RTCM message types", "Corrections.Types", 0, 0, 0, "mdi:format-list-numbered", 1, T_GNSS},
  {"corr_station", "sensor", "Corrections base station", "Corrections.Station", 0, 0, 0, "mdi:radio-tower", 1, T_GNSS},
  {"diff_age", "sensor", "Differential age", "DiffAge", "s", "duration", MEAS, "mdi:timer-sand", 0, T_GNSS},
  {"diff_station", "sensor", "Differential station", "DiffStation", 0, 0, 0, "mdi:radio-tower", 1, T_GNSS},
  {"rtcm_used", "sensor", "RTCM messages used", "Corrections.Used", 0, 0, TOTAL, "mdi:check", 1, T_GNSS},
  {"rtcm_failed", "sensor", "RTCM messages rejected", "Corrections.Failed", 0, 0, TOTAL, "mdi:close", 1, T_GNSS},
  {"model", "sensor", "Receiver model", "Receiver.Model", 0, 0, 0, "mdi:chip", 1, T_GNSS},
  {"firmware", "sensor", "Receiver firmware", "Receiver.Firmware", 0, 0, 0, "mdi:chip", 1, T_GNSS},
  {"protocol", "sensor", "Receiver protocol version", "Receiver.Protocol", 0, 0, 0, "mdi:chip", 1, T_GNSS},
  {"baud", "sensor", "Receiver UART speed", "Receiver.Baud", "Bd", 0, 0, "mdi:serial-port", 1, T_GNSS},
  {"antenna", "sensor", "Antenna", "Receiver.Antenna", 0, 0, 0, "mdi:antenna", 0, T_GNSS},
  {"jamming", "sensor", "Jamming indicator", "Receiver.Jamming", 0, 0, MEAS, "mdi:signal-off", 1, T_GNSS},
  {"noise", "sensor", "Noise level", "Receiver.Noise", 0, 0, MEAS, "mdi:waveform", 1, T_GNSS},
  {"qerr", "sensor", "Timepulse quantization error", "Receiver.QErr", "ps", 0, MEAS, "mdi:pulse", 1, T_GNSS},
  {"nmea", "sensor", "NMEA sentences", "Health.NMEA", 0, 0, TOTAL, "mdi:counter", 1, T_GNSS},
  {"nmea_bad", "sensor", "NMEA sentences bad", "Health.NMEABad", 0, 0, TOTAL, "mdi:counter", 1, T_GNSS},
  {"ubx", "sensor", "UBX messages", "Health.UBX", 0, 0, TOTAL, "mdi:counter", 1, T_GNSS},
  {"ubx_bad", "sensor", "UBX messages bad", "Health.UBXBad", 0, 0, TOTAL, "mdi:counter", 1, T_GNSS},
  {"last_data", "sensor", "Since last receiver data", "Health.LastData", "s", "duration", MEAS, "mdi:timer-outline", 1, T_GNSS},
};
#define N_ENTITIES ((int)(sizeof(ENTITIES) / sizeof(ENTITIES[0])))

int gnss_hass_count(void) { return N_ENTITIES; }

static void device(jw_t *w, const gnss_hass_device_t *dev) {
  jw_open(w, "dev", "{");
  jw_open(w, "ids", "[");
  jw_text(w, 0, dev->uid);
  jw_close(w, "]");
  jw_text(w, "name", dev->name);
  jw_text(w, "mf", "esp32-to-gps");
  jw_text(w, "mdl", dev->model && *dev->model ? dev->model : "GNSS receiver");
  jw_text(w, "sw", dev->sw);
  if (dev->url && *dev->url) { jw_text(w, "cu", dev->url); }
  jw_close(w, "}");
}

static void availability(jw_t *w, const gnss_hass_device_t *dev) {
  jw_text(w, "avty_t", dev->lwt);
  jw_text(w, "pl_avail", "Online");
  jw_text(w, "pl_not_avail", "Offline");
}

size_t gnss_hass_config(int index, const gnss_hass_device_t *dev, char *topic, size_t topic_len,
                        char *payload, size_t payload_len) {
  if (index < 0 || index >= N_ENTITIES) { return 0; }
  const entity_t *e = &ENTITIES[index];
  int n = snprintf(topic, topic_len, "homeassistant/%s/%s/%s/config", e->comp, dev->uid, e->id);
  if (n < 0 || (size_t)n >= topic_len) { return 0; }

  jw_t w;
  jw_init(&w, payload, payload_len);
  jw_open(&w, 0, "{");
  jw_text(&w, "name", e->name);
  char buf[160];
  snprintf(buf, sizeof(buf), "%s_%s", dev->uid, e->id);
  jw_text(&w, "uniq_id", buf);

  if (e->topic == T_TRACKER) {
    snprintf(buf, sizeof(buf), "%sGNSS", dev->topic);
    jw_text(&w, "json_attr_t", buf);
    jw_text(&w, "json_attr_tpl",
            "{% if value_json.Lat is not none %}{{ {'latitude': value_json.Lat, 'longitude': value_json.Lon, "
            "'gps_accuracy': value_json.HAcc} | tojson }}{% else %}{}{% endif %}");
    jw_text(&w, "src_type", "gps");
  } else {
    snprintf(buf, sizeof(buf), "%s%s", dev->topic, e->topic == T_SATS ? "GNSS_SATS" : "GNSS");
    jw_text(&w, "stat_t", buf);
    if (strcmp(e->comp, "binary_sensor") == 0) {
      snprintf(buf, sizeof(buf), "{{ 'ON' if value_json.%s else 'OFF' }}", e->path);
    } else {
      snprintf(buf, sizeof(buf), "{{ value_json.%s }}", e->path);
    }
    jw_text(&w, "val_tpl", buf);
    if (e->topic == T_SATS) {
      snprintf(buf, sizeof(buf), "%sGNSS_SATS", dev->topic);
      jw_text(&w, "json_attr_t", buf);
    }
    if (e->unit) { jw_text(&w, "unit_of_meas", e->unit); }
    if (e->dev_cla) { jw_text(&w, "dev_cla", e->dev_cla); }
    if (e->stat_cla) { jw_text(&w, "stat_cla", e->stat_cla); }
  }
  if (e->icon) { jw_text(&w, "ic", e->icon); }
  if (e->diag) { jw_text(&w, "ent_cat", "diagnostic"); }
  availability(&w, dev);
  device(&w, dev);
  jw_close(&w, "}");
  return jw_done(&w);
}

size_t gnss_hass_sat_config(uint8_t gnss, uint8_t svid, bool remove, const gnss_hass_device_t *dev,
                            char *topic, size_t topic_len, char *payload, size_t payload_len) {
  char id[8];
  sat_id(id, sizeof(id), gnss, svid);
  int n = snprintf(topic, topic_len, "homeassistant/sensor/%s/sat_%s/config", dev->uid, id);
  if (n < 0 || (size_t)n >= topic_len || payload_len == 0) { return 0; }
  if (remove) {
    payload[0] = 0;
    return 0;  /* an empty retained payload removes the entity */
  }
  jw_t w;
  jw_init(&w, payload, payload_len);
  jw_open(&w, 0, "{");
  char buf[160];
  snprintf(buf, sizeof(buf), "%s %u C/N0", gnss_constellation_name(gnss), (unsigned)svid);
  jw_text(&w, "name", buf);
  snprintf(buf, sizeof(buf), "%s_sat_%s", dev->uid, id);
  jw_text(&w, "uniq_id", buf);
  snprintf(buf, sizeof(buf), "%sGNSS_SATS", dev->topic);
  jw_text(&w, "stat_t", buf);
  snprintf(buf, sizeof(buf), "{{ value_json.CNo.%s | default(None) }}", id);
  jw_text(&w, "val_tpl", buf);
  jw_text(&w, "unit_of_meas", "dB-Hz");
  jw_text(&w, "stat_cla", MEAS);
  jw_text(&w, "ic", "mdi:satellite-variant");
  jw_text(&w, "ent_cat", "diagnostic");
  jw_int(&w, "exp_aft", 120);
  availability(&w, dev);
  device(&w, dev);
  jw_close(&w, "}");
  return jw_done(&w);
}
