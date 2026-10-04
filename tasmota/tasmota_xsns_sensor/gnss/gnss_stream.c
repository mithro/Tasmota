/*
  gnss_stream.c - split a receiver's serial output into NMEA sentences and UBX frames

  SPDX-License-Identifier: GPL-3.0-or-later
*/

#include "gnss_stream.h"

enum {
  M_IDLE,
  M_NMEA,        /* collecting '$' .. '*' */
  M_NMEA_CK1,    /* first checksum digit */
  M_NMEA_CK2,    /* second checksum digit */
  M_NMEA_END,    /* waiting for the line end */
  M_UBX_SYNC2,   /* had 0xB5, want 0x62 */
  M_UBX_HEAD,    /* class, id, length */
  M_UBX_BODY,    /* payload */
  M_UBX_CKA,
  M_UBX_CKB,
};

void gnss_stream_init(gnss_stream_t *st) {
  st->mode = M_IDLE;
  st->len = 0;
}

static int hexval(uint8_t c) {
  if (c >= '0' && c <= '9') { return c - '0'; }
  if (c >= 'A' && c <= 'F') { return c - 'A' + 10; }
  if (c >= 'a' && c <= 'f') { return c - 'a' + 10; }
  return -1;
}

static void ubx_sum(gnss_stream_t *st, uint8_t b) {
  st->ck_a = (uint8_t)(st->ck_a + b);
  st->ck_b = (uint8_t)(st->ck_b + st->ck_a);
}

/* A byte that can start a frame, from any state that has just given up. */
static gnss_frame_t start(gnss_stream_t *st, uint8_t b) {
  st->len = 0;
  if (b == '$') {
    st->mode = M_NMEA;
    st->nmea_ck = 0;
    st->buf[st->len++] = b;
  } else if (b == 0xB5) {
    st->mode = M_UBX_SYNC2;
  } else {
    st->mode = M_IDLE;
  }
  return GNSS_FRAME_NONE;
}

/* Abandon a frame part-way through: report it as bad, and let the byte that
 * interrupted it start the next one. */
static gnss_frame_t abandon(gnss_stream_t *st, uint8_t b, gnss_frame_t bad) {
  start(st, b);
  return bad;
}

gnss_frame_t gnss_stream_byte(gnss_stream_t *st, uint8_t b) {
  switch (st->mode) {
    case M_IDLE:
      return start(st, b);

    case M_NMEA:
      if (b == '*') {
        st->buf[st->len] = 0;
        st->mode = M_NMEA_CK1;
        return GNSS_FRAME_NONE;
      }
      if (b < 0x20 || b > 0x7e || st->len >= GNSS_NMEA_MAX - 1) {
        return abandon(st, b, GNSS_FRAME_BAD_NMEA);
      }
      if (b == '$') {  /* a new sentence before this one ended */
        return abandon(st, b, GNSS_FRAME_BAD_NMEA);
      }
      st->nmea_ck ^= b;
      st->buf[st->len++] = b;
      return GNSS_FRAME_NONE;

    case M_NMEA_CK1:
    case M_NMEA_CK2:
      if (hexval(b) < 0) { return abandon(st, b, GNSS_FRAME_BAD_NMEA); }
      st->hex[st->mode == M_NMEA_CK1 ? 0 : 1] = b;
      st->mode = (uint8_t)(st->mode == M_NMEA_CK1 ? M_NMEA_CK2 : M_NMEA_END);
      return GNSS_FRAME_NONE;

    case M_NMEA_END:
      if (b != '\r' && b != '\n') { return abandon(st, b, GNSS_FRAME_BAD_NMEA); }
      st->mode = M_IDLE;
      if ((uint8_t)(hexval(st->hex[0]) << 4 | hexval(st->hex[1])) != st->nmea_ck) {
        return GNSS_FRAME_BAD_NMEA;
      }
      return GNSS_FRAME_NMEA;

    case M_UBX_SYNC2:
      if (b != 0x62) { return start(st, b); }
      st->mode = M_UBX_HEAD;
      st->ck_a = st->ck_b = 0;
      return GNSS_FRAME_NONE;

    case M_UBX_HEAD:
      ubx_sum(st, b);
      st->buf[st->len++] = b;
      if (st->len == 4) {
        st->ubx_len = (uint16_t)(st->buf[2] | st->buf[3] << 8);
        if (st->ubx_len > GNSS_UBX_MAX) {
          st->mode = M_IDLE;
          return GNSS_FRAME_BAD_UBX;
        }
        st->mode = st->ubx_len ? M_UBX_BODY : M_UBX_CKA;
      }
      return GNSS_FRAME_NONE;

    case M_UBX_BODY:
      ubx_sum(st, b);
      st->buf[st->len++] = b;
      if (st->len == 4 + st->ubx_len) { st->mode = M_UBX_CKA; }
      return GNSS_FRAME_NONE;

    case M_UBX_CKA:
      if (b != st->ck_a) {
        st->mode = M_IDLE;
        return GNSS_FRAME_BAD_UBX;
      }
      st->mode = M_UBX_CKB;
      return GNSS_FRAME_NONE;

    case M_UBX_CKB:
      st->mode = M_IDLE;
      return b == st->ck_b ? GNSS_FRAME_UBX : GNSS_FRAME_BAD_UBX;
  }
  return start(st, b);
}
