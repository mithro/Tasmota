/*
  gnss_nmea.c - decode NMEA 0183 sentences into the receiver state

  Numbers are parsed as fixed point: the ESP32-C3 has no floating-point unit.

  SPDX-License-Identifier: GPL-3.0-or-later
*/

#include "gnss_nmea.h"

#include <string.h>

#define MAX_FIELDS 40

typedef struct {
  char buf[GNSS_TEXT_LEN * 6];
  const char *f[MAX_FIELDS];
  int n;
} fields_t;

/* Split at commas, keeping empty fields. f[0] is the address ("GPGGA"). */
static void split(fields_t *fs, const char *sentence) {
  if (*sentence == '$') { sentence++; }
  size_t len = strlen(sentence);
  if (len >= sizeof(fs->buf)) { len = sizeof(fs->buf) - 1; }
  memcpy(fs->buf, sentence, len);
  fs->buf[len] = 0;
  fs->n = 0;
  char *p = fs->buf;
  fs->f[fs->n++] = p;
  for (; *p; p++) {
    if (*p == ',') {
      *p = 0;
      if (fs->n < MAX_FIELDS) { fs->f[fs->n++] = p + 1; }
    }
  }
}

static const char *field(const fields_t *fs, int i) {
  return i < fs->n ? fs->f[i] : "";
}

/* "12.345" -> 12345 for decimals = 3. Returns false for an empty or
 * malformed field. Extra decimal places are truncated. */
static bool fixed(const char *p, int decimals, int64_t *out) {
  bool neg = false, digits = false;
  int64_t v = 0;
  if (*p == '-') { neg = true; p++; } else if (*p == '+') { p++; }
  for (; *p >= '0' && *p <= '9'; p++) { v = v * 10 + (*p - '0'); digits = true; }
  int d = 0;
  if (*p == '.') {
    for (p++; *p >= '0' && *p <= '9'; p++) {
      if (d < decimals) { v = v * 10 + (*p - '0'); d++; }
      digits = true;
    }
  }
  if (*p || !digits) { return false; }
  for (; d < decimals; d++) { v *= 10; }
  *out = neg ? -v : v;
  return true;
}

static bool integer(const char *p, int *out) {
  int64_t v;
  if (!fixed(p, 0, &v) || v < -2147483647 || v > 2147483647) { return false; }
  *out = (int)v;
  return true;
}

/* NMEA ddmm.mmmm / dddmm.mmmm plus hemisphere -> degrees x 1e7. */
static bool latlon(const char *v, const char *hemi, int32_t *out) {
  int64_t raw;  /* ddmm.mmmmmmm x 1e7 */
  if (!fixed(v, 7, &raw) || raw < 0) { return false; }
  int64_t deg = raw / 1000000000;           /* the digits before the minutes */
  int64_t min_e7 = raw - deg * 1000000000;  /* minutes x 1e7 */
  int64_t e7 = deg * 10000000 + (min_e7 + 30) / 60;
  if (hemi[0] == 'S' || hemi[0] == 'W') { e7 = -e7; }
  else if (hemi[0] != 'N' && hemi[0] != 'E') { return false; }
  *out = (int32_t)e7;
  return true;
}

/* hhmmss.ss -> time of day in centiseconds, and the fields in the state. */
static bool utc_time(gnss_state_t *s, const char *v) {
  int64_t t;
  if (strlen(v) < 6 || !fixed(v, 2, &t)) { return false; }
  int hh = (int)(t / 1000000), mm = (int)(t / 10000 % 100), ss = (int)(t / 100 % 100), cs = (int)(t % 100);
  if (hh > 23 || mm > 59 || ss > 60) { return false; }
  s->hour = (uint8_t)hh;
  s->minute = (uint8_t)mm;
  s->second = (uint8_t)ss;
  s->millisecond = (uint16_t)(cs * 10);
  s->time_valid = true;
  gnss_epoch_time(s, (uint32_t)(((hh * 60 + mm) * 60 + ss) * 100 + cs));
  return true;
}

static uint8_t talker_gnss(const char *addr) {
  if (addr[0] == 'G') {
    switch (addr[1]) {
      case 'P': return GNSS_GPS;
      case 'L': return GNSS_GLONASS;
      case 'A': return GNSS_GALILEO;
      case 'B': return GNSS_BEIDOU;
      case 'Q': return GNSS_QZSS;
      case 'I': return GNSS_NAVIC;
      default: return GNSS_UNKNOWN;  /* GN: combined */
    }
  }
  if (addr[0] == 'B' && addr[1] == 'D') { return GNSS_BEIDOU; }
  if (addr[0] == 'Q' && addr[1] == 'Z') { return GNSS_QZSS; }
  return GNSS_UNKNOWN;
}

bool gnss_nmea_satellite(uint8_t talker, uint8_t system_id, int prn, uint8_t *gnss, uint8_t *svid) {
  uint8_t g = talker;
  switch (system_id) {  /* NMEA 4.10 system ID, where given, names the constellation */
    case 1: g = GNSS_GPS; break;
    case 2: g = GNSS_GLONASS; break;
    case 3: g = GNSS_GALILEO; break;
    case 4: g = GNSS_BEIDOU; break;
    case 5: g = GNSS_QZSS; break;
    case 6: g = GNSS_NAVIC; break;
    default: break;
  }
  if (prn <= 0) { return false; }
  /* Numbering ranges shared by the GP and GN talkers (NMEA 4.0 extended,
   * as u-blox and others number them): SBAS 33-64 -> PRN 120-151,
   * GLONASS 65-96, QZSS 193-202, Galileo 301-336, BeiDou 401-437. */
  if (g == GNSS_GPS || g == GNSS_UNKNOWN) {
    if (prn <= 32) { g = GNSS_GPS; }
    else if (prn <= 64) { *gnss = GNSS_SBAS; *svid = (uint8_t)(prn + 87); return true; }
    else if (prn <= 96) { g = GNSS_GLONASS; prn -= 64; }
    else if (prn >= 193 && prn <= 202) { g = GNSS_QZSS; prn -= 192; }
    else if (prn >= 301 && prn <= 336) { g = GNSS_GALILEO; prn -= 300; }
    else if (prn >= 401 && prn <= 437) { g = GNSS_BEIDOU; prn -= 400; }
    else { return false; }
  } else if (g == GNSS_GLONASS && prn > 64 && prn <= 96) {
    prn -= 64;  /* GL talker numbers its satellites 65-96 */
  } else if (g == GNSS_QZSS && prn >= 193 && prn <= 202) {
    prn -= 192;
  }
  if (prn > 255) { return false; }
  *gnss = g;
  *svid = (uint8_t)prn;
  return true;
}

static void gga(gnss_state_t *s, const fields_t *fs) {
  int q = 0, n;
  int64_t v;
  utc_time(s, field(fs, 1));
  if (integer(field(fs, 6), &q)) { s->quality = (uint8_t)q; }
  int32_t lat, lon;
  if (q > 0 && latlon(field(fs, 2), field(fs, 3), &lat) && latlon(field(fs, 4), field(fs, 5), &lon)) {
    s->lat_e7 = lat;
    s->lon_e7 = lon;
    s->position_valid = true;
  } else if (q == 0) {
    s->position_valid = false;
  }
  if (integer(field(fs, 7), &n)) { s->sats_used_reported = (uint8_t)n; }
  if (fixed(field(fs, 8), 2, &v)) { s->hdop = (uint16_t)(v > 0xfffe ? 0xfffe : v); }
  if (fixed(field(fs, 9), 3, &v)) { s->alt_msl_mm = (int32_t)v; s->alt_valid = q > 0; }
  if (fixed(field(fs, 11), 3, &v)) {
    s->geoid_sep_mm = (int32_t)v;
    s->geoid_valid = true;
    if (s->alt_valid) {
      s->alt_ellipsoid_mm = s->alt_msl_mm + s->geoid_sep_mm;
      s->ellipsoid_valid = true;
    }
  }
  s->diff_age_ds = fixed(field(fs, 13), 1, &v) ? (int32_t)v : -1;
  s->diff_station = integer(field(fs, 14), &n) ? n : -1;
}

static void rmc(gnss_state_t *s, const fields_t *fs) {
  int64_t v;
  utc_time(s, field(fs, 1));
  bool active = field(fs, 2)[0] == 'A';
  int32_t lat, lon;
  if (active && latlon(field(fs, 3), field(fs, 4), &lat) && latlon(field(fs, 5), field(fs, 6), &lon)) {
    s->lat_e7 = lat;
    s->lon_e7 = lon;
    s->position_valid = true;
  }
  if (active && fixed(field(fs, 7), 3, &v)) {  /* knots x 1000 -> mm/s: 1 kn = 514.444 mm/s */
    s->speed_mm_s = (uint32_t)((v * 514444 + 500000) / 1000000);
    s->motion_valid = true;
  }
  if (active && fixed(field(fs, 8), 5, &v)) { s->course_e5 = (int32_t)v; }
  /* The date only of a valid fix: before one the LC29H(AA) sends its
   * placeholder 060180 (2080-01-06), which must never reach the clock. */
  const char *d = field(fs, 9);
  int date;
  if (active && strlen(d) == 6 && integer(d, &date)) {
    s->day = (uint8_t)(date / 10000);
    s->month = (uint8_t)(date / 100 % 100);
    s->year = (uint16_t)(2000 + date % 100);
    s->date_valid = s->month >= 1 && s->month <= 12 && s->day >= 1;
  }
}

static void gsa(gnss_state_t *s, const fields_t *fs, uint8_t talker) {
  int mode, sys = 0;
  int64_t v;
  if (integer(field(fs, 2), &mode)) {
    s->fix_type = (uint8_t)(mode == 3 ? GNSS_FIX_3D : mode == 2 ? GNSS_FIX_2D : GNSS_FIX_NONE);
  }
  if (fs->n > 18) { integer(field(fs, 18), &sys); }
  if (s->gsa_epoch != s->epoch) {  /* the first GSA of the epoch: forget who was used */
    for (uint8_t i = 0; i < s->n_sats; i++) { s->sats[i].used = false; }
    s->gsa_epoch = s->epoch;
  }
  for (int i = 3; i <= 14; i++) {
    int prn;
    uint8_t g, id;
    if (integer(field(fs, i), &prn) && gnss_nmea_satellite(talker, (uint8_t)sys, prn, &g, &id)) {
      gnss_sat_t *sat = gnss_sat(s, g, id);
      if (sat) { sat->used = true; }
    }
  }
  if (fixed(field(fs, 15), 2, &v)) { s->pdop = (uint16_t)(v > 0xfffe ? 0xfffe : v); }
  if (fixed(field(fs, 16), 2, &v)) { s->hdop = (uint16_t)(v > 0xfffe ? 0xfffe : v); }
  if (fixed(field(fs, 17), 2, &v)) { s->vdop = (uint16_t)(v > 0xfffe ? 0xfffe : v); }
}

static void gsv(gnss_state_t *s, const fields_t *fs, uint8_t talker) {
  /* $xxGSV,total,num,in-view, 4 x (prn,elev,azim,cno) [,signal id] */
  for (int i = 4; i + 3 < fs->n; i += 4) {
    int prn, elev, azim, cno;
    uint8_t g, id;
    if (!integer(field(fs, i), &prn) || !gnss_nmea_satellite(talker, 0, prn, &g, &id)) { continue; }
    gnss_sat_t *sat = gnss_sat(s, g, id);
    if (!sat) { continue; }
    if (integer(field(fs, i + 1), &elev) && elev >= -90 && elev <= 90) { sat->elev = (int8_t)elev; }
    if (integer(field(fs, i + 2), &azim) && azim >= 0 && azim < 360) { sat->azim = (int16_t)azim; }
    if (integer(field(fs, i + 3), &cno) && cno >= 0 && cno < 100) { gnss_sat_cno(s, sat, (uint8_t)cno); }
    else { gnss_sat_cno(s, sat, 0); }
  }
}

static void vtg(gnss_state_t *s, const fields_t *fs) {
  int64_t v;
  if (fixed(field(fs, 1), 5, &v)) { s->course_e5 = (int32_t)v; }
  if (fixed(field(fs, 7), 3, &v)) {  /* km/h x 1000 -> mm/s */
    s->speed_mm_s = (uint32_t)((v * 1000 + 1800) / 3600);
    s->motion_valid = true;
  }
}

static void gll(gnss_state_t *s, const fields_t *fs) {
  utc_time(s, field(fs, 5));
  int32_t lat, lon;
  if (field(fs, 6)[0] == 'A' && latlon(field(fs, 1), field(fs, 2), &lat) && latlon(field(fs, 3), field(fs, 4), &lon)) {
    s->lat_e7 = lat;
    s->lon_e7 = lon;
    s->position_valid = true;
  }
}

static void zda(gnss_state_t *s, const fields_t *fs) {
  int d, m, y;
  if (!utc_time(s, field(fs, 1))) { return; }
  if (integer(field(fs, 2), &d) && integer(field(fs, 3), &m) && integer(field(fs, 4), &y) &&
      d >= 1 && d <= 31 && m >= 1 && m <= 12 && y >= 1980) {
    s->day = (uint8_t)d;
    s->month = (uint8_t)m;
    s->year = (uint16_t)y;
    s->date_valid = true;
  }
}

static uint32_t isqrt64(uint64_t v) {
  uint64_t r = 0, bit = (uint64_t)1 << 62;
  while (bit > v) { bit >>= 2; }
  while (bit) {
    if (v >= r + bit) { v -= r + bit; r = (r >> 1) + bit; } else { r >>= 1; }
    bit >>= 2;
  }
  return (uint32_t)r;
}

static void gst(gnss_state_t *s, const fields_t *fs) {
  /* $xxGST,time,rms,major,minor,orient,lat-sd,lon-sd,alt-sd (metres) */
  int64_t la, lo, al;
  if (fixed(field(fs, 6), 3, &la) && fixed(field(fs, 7), 3, &lo) && fixed(field(fs, 8), 3, &al) &&
      la >= 0 && lo >= 0 && al >= 0) {
    s->h_acc_mm = isqrt64((uint64_t)(la * la + lo * lo));
    s->v_acc_mm = (uint32_t)al;
    s->acc_valid = true;
  }
}

static void txt(gnss_state_t *s, const fields_t *fs) {
  /* u-blox antenna supervisor: $GPTXT,01,01,02,ANTSTATUS=OK */
  const char *t = field(fs, 4);
  if (strncmp(t, "ANTSTATUS=", 10) == 0) {
    t += 10;
    s->antenna = (uint8_t)(strcmp(t, "OK") == 0 ? GNSS_ANT_OK : strcmp(t, "SHORT") == 0 ? GNSS_ANT_SHORT
                           : strcmp(t, "OPEN") == 0 ? GNSS_ANT_OPEN : strcmp(t, "INIT") == 0 ? GNSS_ANT_INIT
                           : GNSS_ANT_UNKNOWN);
  }
}

static void copy_text(char *dst, const char *src, size_t len) {
  size_t n = strlen(src);
  if (n >= GNSS_TEXT_LEN) { n = GNSS_TEXT_LEN - 1; }
  if (len && len < n) { n = len; }
  memcpy(dst, src, n);
  dst[n] = 0;
}

static void pqtmverno(gnss_state_t *s, const fields_t *fs) {
  /* $PQTMVERNO,LC29HAANR11A05S,2025/09/12,10:21:16: product code, then build. */
  const char *v = field(fs, 1);
  s->module = GNSS_MODULE_QUECTEL_LC29H;
  copy_text(s->sw_version, v, 0);
  if (strncmp(v, "LC29H", 5) == 0 && strlen(v) >= 7) {
    /* LC29H + two-letter variant: "LC29H(AA)" */
    char m[GNSS_TEXT_LEN] = "LC29H(";
    m[6] = v[5];
    m[7] = v[6];
    m[8] = ')';
    m[9] = 0;
    copy_text(s->model, m, 0);
  } else {
    copy_text(s->model, v, 0);
  }
}

static void pair021(gnss_state_t *s, const fields_t *fs) {
  /* $PAIR021,AG3335M_V3.2.2.AG3335_20250912,...: chip before the first '_' */
  const char *v = field(fs, 1);
  const char *u = strchr(v, '_');
  copy_text(s->hw_version, v, u ? (size_t)(u - v) : 0);
}

static void pair_ack(gnss_state_t *s, const fields_t *fs) {
  /* $PAIR001,<command>,<result>: result 0 is success */
  int cmd, result;
  if (!integer(field(fs, 1), &cmd) || !integer(field(fs, 2), &result)) { return; }
  if (result == 0) {
    s->ack++;
    s->last_ack_cls = 0;
    s->last_ack_id = (uint8_t)cmd;
  } else if (result != 1) {  /* 1 = still processing */
    s->nak++;
    s->last_nak_cls = 0;
    s->last_nak_id = (uint8_t)cmd;
  }
}

gnss_nmea_kind_t gnss_nmea_parse(gnss_state_t *s, const char *sentence) {
  fields_t fs;
  split(&fs, sentence);
  const char *addr = fs.f[0];
  s->nmea_ok++;
  if (addr[0] == 'P') {
    if (strcmp(addr, "PQTMVERNO") == 0) { pqtmverno(s, &fs); return GNSS_NMEA_PQTMVERNO; }
    if (strcmp(addr, "PAIR001") == 0) { pair_ack(s, &fs); return GNSS_NMEA_PAIR_ACK; }
    if (strcmp(addr, "PAIR021") == 0) { pair021(s, &fs); return GNSS_NMEA_PAIR021; }
    return GNSS_NMEA_OTHER;
  }
  if (strlen(addr) != 5) { return GNSS_NMEA_OTHER; }
  uint8_t talker = talker_gnss(addr);
  const char *kind = addr + 2;
  if (strcmp(kind, "GGA") == 0) { gga(s, &fs); return GNSS_NMEA_GGA; }
  if (strcmp(kind, "RMC") == 0) { rmc(s, &fs); return GNSS_NMEA_RMC; }
  if (strcmp(kind, "GSA") == 0) { gsa(s, &fs, talker); return GNSS_NMEA_GSA; }
  if (strcmp(kind, "GSV") == 0) { gsv(s, &fs, talker); return GNSS_NMEA_GSV; }
  if (strcmp(kind, "VTG") == 0) { vtg(s, &fs); return GNSS_NMEA_VTG; }
  if (strcmp(kind, "GLL") == 0) { gll(s, &fs); return GNSS_NMEA_GLL; }
  if (strcmp(kind, "ZDA") == 0) { zda(s, &fs); return GNSS_NMEA_ZDA; }
  if (strcmp(kind, "GST") == 0) { gst(s, &fs); return GNSS_NMEA_GST; }
  if (strcmp(kind, "TXT") == 0) { txt(s, &fs); return GNSS_NMEA_TXT; }
  return GNSS_NMEA_OTHER;
}
