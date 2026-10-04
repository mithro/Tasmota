/*
  gnss_ubx.c - decode and build u-blox UBX messages

  SPDX-License-Identifier: GPL-3.0-or-later
*/

#include "gnss_ubx.h"

#include <string.h>

#define CLS_NAV 0x01
#define CLS_RXM 0x02
#define CLS_ACK 0x05
#define CLS_CFG 0x06
#define CLS_MON 0x0A
#define CLS_TIM 0x0D

static uint16_t u2(const uint8_t *p) { return (uint16_t)(p[0] | p[1] << 8); }
static uint32_t u4(const uint8_t *p) { return (uint32_t)p[0] | (uint32_t)p[1] << 8 | (uint32_t)p[2] << 16 | (uint32_t)p[3] << 24; }
static int16_t i2(const uint8_t *p) { return (int16_t)u2(p); }
static int32_t i4(const uint8_t *p) { return (int32_t)u4(p); }

static void text(char *dst, const uint8_t *src, size_t max) {
  size_t n = 0;
  if (max > GNSS_TEXT_LEN - 1) { max = GNSS_TEXT_LEN - 1; }
  while (n < max && src[n] >= 0x20 && src[n] < 0x7f) { dst[n] = (char)src[n]; n++; }
  while (n && dst[n - 1] == ' ') { n--; }
  dst[n] = 0;
}

static void nav_pvt(gnss_state_t *s, const uint8_t *p) {
  uint8_t valid = p[11], flags = p[21];
  if (valid & 0x02) {  /* validTime: UTC time of day is good */
    int32_t nano = i4(p + 16);
    int32_t cs = ((p[8] * 60 + p[9]) * 60 + p[10]) * 100 + (nano >= 0 ? (nano + 5000000) : (nano - 5000000)) / 10000000;
    if (cs < 0) { cs += 24 * 60 * 60 * 100; }
    gnss_epoch_time(s, (uint32_t)cs);
    s->hour = p[8];
    s->minute = p[9];
    s->second = p[10];
    s->millisecond = (uint16_t)(nano > 0 ? nano / 1000000 : 0);
    s->time_valid = true;
  }
  if (valid & 0x01) {  /* validDate */
    s->year = u2(p + 4);
    s->month = p[6];
    s->day = p[7];
    s->date_valid = true;
  }
  s->time_accuracy_ns = u4(p + 12);
  s->fix_type = p[20];
  s->diff_soln = (flags & 0x02) != 0;
  s->carr_soln = (uint8_t)(flags >> 6);
  s->sats_used_reported = p[23];
  bool ok = (flags & 0x01) && (s->fix_type == GNSS_FIX_2D || s->fix_type == GNSS_FIX_3D || s->fix_type == GNSS_FIX_GNSS_DR);
  s->position_valid = ok;
  if (ok) {
    s->lon_e7 = i4(p + 24);
    s->lat_e7 = i4(p + 28);
    s->alt_ellipsoid_mm = i4(p + 32);
    s->alt_msl_mm = i4(p + 36);
    s->ellipsoid_valid = s->alt_valid = s->fix_type != GNSS_FIX_2D;
    s->geoid_sep_mm = s->alt_ellipsoid_mm - s->alt_msl_mm;
    s->geoid_valid = s->alt_valid;
  }
  s->h_acc_mm = u4(p + 40);
  s->v_acc_mm = u4(p + 44);
  s->acc_valid = true;
  int32_t gspeed = i4(p + 60);
  s->speed_mm_s = gspeed > 0 ? (uint32_t)gspeed : 0;
  s->course_e5 = i4(p + 64);
  s->speed_acc_mm_s = u4(p + 68);
  s->motion_valid = ok;
  s->pdop = u2(p + 76);
  /* NAV-PVT's own quality, in GGA terms, for receivers that send no GGA. */
  if (!ok) { s->quality = GNSS_QUALITY_INVALID; }
  else if (s->carr_soln == 2) { s->quality = GNSS_QUALITY_RTK_FIXED; }
  else if (s->carr_soln == 1) { s->quality = GNSS_QUALITY_RTK_FLOAT; }
  else if (s->diff_soln) { s->quality = GNSS_QUALITY_DGPS; }
  else if (s->fix_type == GNSS_FIX_GNSS_DR || s->fix_type == GNSS_FIX_DEAD_RECKONING) { s->quality = GNSS_QUALITY_ESTIMATED; }
  else { s->quality = GNSS_QUALITY_GPS; }
}

static void nav_dop(gnss_state_t *s, const uint8_t *p) {
  s->gdop = u2(p + 4);
  s->pdop = u2(p + 6);
  s->tdop = u2(p + 8);
  s->vdop = u2(p + 10);
  s->hdop = u2(p + 12);
}

static gnss_ubx_kind_t nav_sat(gnss_state_t *s, const uint8_t *p, uint16_t len) {
  uint8_t n = p[5];
  if (len < 8 + 12 * (uint32_t)n) { return GNSS_UBX_SHORT; }
  for (uint8_t i = 0; i < n; i++) {
    const uint8_t *b = p + 8 + 12 * i;
    if (b[0] >= GNSS_N_CONSTELLATIONS) { continue; }
    gnss_sat_t *sat = gnss_sat(s, b[0], b[1]);
    if (!sat) { break; }
    int16_t az = i2(b + 4);
    int8_t el = (int8_t)b[3];
    sat->elev = (el >= -90 && el <= 90) ? el : -91;
    sat->azim = (int16_t)((az >= 0 && az <= 359) ? az : -1);
    gnss_sat_cno(s, sat, b[2]);
    sat->used = (u4(b + 8) & 0x08) != 0;
  }
  s->gsa_epoch = s->epoch;  /* the used flags are this epoch's */
  return GNSS_UBX_NAV_SAT;
}

bool gnss_ubx_svid(uint8_t svid, uint8_t *gnss, uint8_t *sat) {
  if (svid >= 1 && svid <= 32) { *gnss = GNSS_GPS; *sat = svid; }
  else if (svid >= 33 && svid <= 64) { *gnss = GNSS_BEIDOU; *sat = (uint8_t)(svid - 27); }
  else if (svid >= 65 && svid <= 96) { *gnss = GNSS_GLONASS; *sat = (uint8_t)(svid - 64); }
  else if (svid >= 120 && svid <= 158) { *gnss = GNSS_SBAS; *sat = svid; }
  else if (svid >= 159 && svid <= 163) { *gnss = GNSS_BEIDOU; *sat = (uint8_t)(svid - 158); }
  else if (svid >= 173 && svid <= 182) { *gnss = GNSS_IMES; *sat = (uint8_t)(svid - 172); }
  else if (svid >= 193 && svid <= 202) { *gnss = GNSS_QZSS; *sat = (uint8_t)(svid - 192); }
  else if (svid >= 211 && svid <= 246) { *gnss = GNSS_GALILEO; *sat = (uint8_t)(svid - 210); }
  else { return false; }
  return true;
}

static gnss_ubx_kind_t nav_svinfo(gnss_state_t *s, const uint8_t *p, uint16_t len) {
  uint8_t n = p[4];
  if (len < 8 + 12 * (uint32_t)n) { return GNSS_UBX_SHORT; }
  for (uint8_t i = 0; i < n; i++) {
    const uint8_t *b = p + 8 + 12 * i;
    uint8_t gnss, id;
    if (!gnss_ubx_svid(b[1], &gnss, &id)) { continue; }
    gnss_sat_t *sat = gnss_sat(s, gnss, id);
    if (!sat) { break; }
    int8_t el = (int8_t)b[5];
    int16_t az = i2(b + 6);
    sat->elev = (el >= -90 && el <= 90) ? el : -91;
    sat->azim = (int16_t)((az >= 0 && az <= 359) ? az : -1);
    gnss_sat_cno(s, sat, b[4]);
    sat->used = (b[2] & 0x01) != 0;
  }
  s->gsa_epoch = s->epoch;
  return GNSS_UBX_NAV_SVINFO;
}

/* MON-VER: identify the receiver generation and model. */
static void mon_ver(gnss_state_t *s, const uint8_t *p, uint16_t len) {
  text(s->sw_version, p, 30);
  text(s->hw_version, p + 30, 10);
  if (strcmp(s->hw_version, "00070000") == 0) { s->module = GNSS_MODULE_UBLOX7; }
  else if (strcmp(s->hw_version, "00080000") == 0) { s->module = GNSS_MODULE_UBLOX_M8; }
  else if (strcmp(s->hw_version, "000A0000") == 0) { s->module = GNSS_MODULE_UBLOX_M10; }
  else { s->module = GNSS_MODULE_UBLOX_OTHER; }
  for (uint16_t off = 40; off + 30 <= len; off += 30) {
    char ext[GNSS_TEXT_LEN];
    text(ext, p + off, 30);
    if (strncmp(ext, "MOD=", 4) == 0) {
      memmove(s->model, ext + 4, strlen(ext + 4) + 1);
    } else if (strncmp(ext, "PROTVER", 7) == 0 && (ext[7] == '=' || ext[7] == ' ')) {
      memmove(s->protocol, ext + 8, strlen(ext + 8) + 1);  /* "PROTVER=34.10", or "PROTVER 14.00" on a u-blox 7 */
    } else if (strncmp(ext, "FWVER=", 6) == 0) {
      memmove(s->sw_version, ext + 6, strlen(ext + 6) + 1);  /* the firmware, more useful than the ROM base */
    }
  }
  if (!s->model[0]) {  /* no MOD= extension: name the generation */
    const char *m = s->module == GNSS_MODULE_UBLOX7 ? "u-blox 7" : s->module == GNSS_MODULE_UBLOX_M8 ? "u-blox M8"
                  : s->module == GNSS_MODULE_UBLOX_M10 ? "u-blox M10" : "u-blox";
    memcpy(s->model, m, strlen(m) + 1);
  }
}

static gnss_antenna_t antenna(uint8_t a) {
  return a <= GNSS_ANT_OPEN ? (gnss_antenna_t)a : GNSS_ANT_UNKNOWN;
}

static void mon_hw(gnss_state_t *s, const uint8_t *p) {
  s->noise = u2(p + 16);
  s->antenna = (uint8_t)antenna(p[20]);
  s->jamming = p[45];
  s->rf_valid = true;
}

static gnss_ubx_kind_t mon_rf(gnss_state_t *s, const uint8_t *p, uint16_t len) {
  if (p[1] == 0) { return GNSS_UBX_MON_RF; }
  if (len < 4 + 24) { return GNSS_UBX_SHORT; }
  const uint8_t *b = p + 4;  /* the first RF block: L1 on every receiver here */
  s->antenna = (uint8_t)antenna(b[2]);
  s->noise = u2(b + 12);
  s->jamming = b[16];
  s->rf_valid = true;
  return GNSS_UBX_MON_RF;
}

static void rxm_rtcm(gnss_state_t *s, const uint8_t *p) {
  uint8_t flags = p[1];
  s->rtcm_last_type = u2(p + 6);
  if (flags & 0x01) { s->rtcm_failed++; }
  else if (((flags >> 1) & 0x03) == 2) { s->rtcm_used++; }
}

static void tim_tp(gnss_state_t *s, const uint8_t *p) {
  s->qerr_ps = i4(p + 8);
  s->qerr_valid = (p[14] & 0x10) == 0;  /* qErrInvalid */
}

gnss_ubx_kind_t gnss_ubx_parse(gnss_state_t *s, uint8_t cls, uint8_t id, const uint8_t *p, uint16_t len) {
  s->ubx_ok++;
  switch (cls << 8 | id) {
    case CLS_NAV << 8 | 0x07:
      if (len < 84) { return GNSS_UBX_SHORT; }  /* 84 on a u-blox 7, 92 later */
      nav_pvt(s, p);
      return GNSS_UBX_NAV_PVT;
    case CLS_NAV << 8 | 0x04:
      if (len < 18) { return GNSS_UBX_SHORT; }
      nav_dop(s, p);
      return GNSS_UBX_NAV_DOP;
    case CLS_NAV << 8 | 0x35:
      return len < 8 ? GNSS_UBX_SHORT : nav_sat(s, p, len);
    case CLS_NAV << 8 | 0x30:
      return len < 8 ? GNSS_UBX_SHORT : nav_svinfo(s, p, len);
    case CLS_MON << 8 | 0x04:
      if (len < 40) { return GNSS_UBX_SHORT; }
      mon_ver(s, p, len);
      return GNSS_UBX_MON_VER;
    case CLS_MON << 8 | 0x09:
      if (len < 60) { return GNSS_UBX_SHORT; }
      mon_hw(s, p);
      return GNSS_UBX_MON_HW;
    case CLS_MON << 8 | 0x38:
      return len < 4 ? GNSS_UBX_SHORT : mon_rf(s, p, len);
    case CLS_RXM << 8 | 0x32:
      if (len < 8) { return GNSS_UBX_SHORT; }
      rxm_rtcm(s, p);
      return GNSS_UBX_RXM_RTCM;
    case CLS_TIM << 8 | 0x01:
      if (len < 16) { return GNSS_UBX_SHORT; }
      tim_tp(s, p);
      return GNSS_UBX_TIM_TP;
    case CLS_ACK << 8 | 0x01:
    case CLS_ACK << 8 | 0x00:
      if (len < 2) { return GNSS_UBX_SHORT; }
      if (id == 0x01) {
        s->ack++;
        s->last_ack_cls = p[0];
        s->last_ack_id = p[1];
        return GNSS_UBX_ACK;
      }
      s->nak++;
      s->last_nak_cls = p[0];
      s->last_nak_id = p[1];
      return GNSS_UBX_NAK;
    default:
      return GNSS_UBX_OTHER;
  }
}

size_t gnss_ubx_frame(uint8_t *out, size_t out_len, uint8_t cls, uint8_t id, const uint8_t *payload, uint16_t len) {
  size_t total = (size_t)len + 8;
  if (out_len < total) { return 0; }
  out[0] = 0xB5;
  out[1] = 0x62;
  out[2] = cls;
  out[3] = id;
  out[4] = (uint8_t)(len & 0xff);
  out[5] = (uint8_t)(len >> 8);
  if (len) { memmove(out + 6, payload, len); }
  uint8_t a = 0, b = 0;
  for (size_t i = 2; i < 6 + (size_t)len; i++) {
    a = (uint8_t)(a + out[i]);
    b = (uint8_t)(b + a);
  }
  out[6 + len] = a;
  out[7 + len] = b;
  return total;
}

size_t gnss_ubx_poll(uint8_t *out, size_t out_len, uint8_t cls, uint8_t id) {
  return gnss_ubx_frame(out, out_len, cls, id, 0, 0);
}

size_t gnss_ubx_cfg_msg(uint8_t *out, size_t out_len, uint8_t cls, uint8_t id, uint8_t rate) {
  uint8_t p[3] = {cls, id, rate};
  return gnss_ubx_frame(out, out_len, CLS_CFG, 0x01, p, sizeof(p));
}

size_t gnss_ubx_cfg_valset(uint8_t *out, size_t out_len, uint8_t layers,
                           const uint32_t *keys, const uint64_t *values, uint8_t n) {
  uint8_t p[4 + 64 * 12];
  size_t len = 4;
  p[0] = 0;  /* version */
  p[1] = layers;
  p[2] = p[3] = 0;
  for (uint8_t i = 0; i < n; i++) {
    /* Bits 28-30 of the key give the value size: 1 = one bit (stored in a
     * byte), 2 = 1 byte, 3 = 2, 4 = 4, 5 = 8 bytes. */
    uint8_t size_code = (uint8_t)((keys[i] >> 28) & 0x07);
    size_t size = size_code == 1 || size_code == 2 ? 1 : size_code == 3 ? 2 : size_code == 4 ? 4 : size_code == 5 ? 8 : 0;
    if (!size || len + 4 + size > sizeof(p)) { return 0; }
    for (int k = 0; k < 4; k++) { p[len++] = (uint8_t)(keys[i] >> (8 * k)); }
    for (size_t k = 0; k < size; k++) { p[len++] = (uint8_t)(values[i] >> (8 * k)); }
  }
  return gnss_ubx_frame(out, out_len, CLS_CFG, 0x8A, p, (uint16_t)len);
}
