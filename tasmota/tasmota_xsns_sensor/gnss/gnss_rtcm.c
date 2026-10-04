/*
  gnss_rtcm.c - watch the RTCM correction stream, and the NTRIP request and reply

  SPDX-License-Identifier: GPL-3.0-or-later
*/

#include "gnss_rtcm.h"

#include <stdio.h>
#include <string.h>

uint32_t gnss_crc24q(const uint8_t *data, size_t len) {
  uint32_t crc = 0;
  for (size_t i = 0; i < len; i++) {
    crc ^= (uint32_t)data[i] << 16;
    for (int b = 0; b < 8; b++) {
      crc <<= 1;
      if (crc & 0x1000000) { crc ^= 0x1864CFB; }
    }
  }
  return crc & 0xFFFFFF;
}

void gnss_rtcm_init(gnss_rtcm_t *r) {
  memset(r, 0, sizeof(*r));
}

static void count_type(gnss_rtcm_t *r, uint16_t type) {
  r->last_type = type;
  for (uint8_t i = 0; i < r->n_types; i++) {
    if (r->types[i] == type) { r->counts[i]++; return; }
  }
  if (r->n_types < GNSS_RTCM_TYPES) {
    r->types[r->n_types] = type;
    r->counts[r->n_types++] = 1;
  }
}

/* A complete frame is in r->buf: check it and count it. */
static bool frame_done(gnss_rtcm_t *r) {
  size_t body = (size_t)r->need - 3;
  uint32_t crc = (uint32_t)r->buf[body] << 16 | (uint32_t)r->buf[body + 1] << 8 | r->buf[body + 2];
  if (gnss_crc24q(r->buf, body) != crc) {
    r->bad_crc++;
    return false;
  }
  r->frames++;
  if (r->need >= 3 + 2 + 3) {  /* the 12-bit message type opens the payload */
    uint16_t type = (uint16_t)(r->buf[3] << 4 | r->buf[4] >> 4);
    count_type(r, type);
    if ((type == 1005 || type == 1006) && r->need >= 3 + 3 + 3) {
      r->station = (uint16_t)((r->buf[4] & 0x0F) << 8 | r->buf[5]);
      r->station_valid = true;
    }
  }
  return true;
}

uint32_t gnss_rtcm_feed(gnss_rtcm_t *r, const uint8_t *data, size_t len) {
  uint32_t done = 0;
  r->bytes += (uint32_t)len;
  for (size_t i = 0; i < len; i++) {
    uint8_t b = data[i];
    if (r->len == 0) {
      if (b != 0xD3) { continue; }  /* RTCM 2.3, or noise between frames */
      r->buf[r->len++] = b;
      continue;
    }
    r->buf[r->len++] = b;
    if (r->len == 3) {
      /* 6 reserved bits, which must be zero, then a 10-bit length */
      if (r->buf[1] & 0xFC) { r->len = 0; continue; }
      r->need = (uint16_t)(3 + ((r->buf[1] & 0x03) << 8 | r->buf[2]) + 3);
    }
    if (r->len >= 3 && r->len == r->need) {
      if (frame_done(r)) { done++; }
      r->len = 0;
    }
  }
  return done;
}

size_t gnss_ntrip_request(char *out, size_t out_len, const char *host, uint16_t port, const char *mount,
                          const char *user, const char *password) {
  static const char b64[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  char auth[160] = "";
  if (user && *user) {
    char plain[96];
    int n = snprintf(plain, sizeof(plain), "%s:%s", user, password ? password : "");
    if (n < 0 || (size_t)n >= sizeof(plain)) { return 0; }
    char enc[132];
    size_t o = 0;
    for (int i = 0; i < n; i += 3) {
      uint32_t v = (uint32_t)(uint8_t)plain[i] << 16;
      if (i + 1 < n) { v |= (uint32_t)(uint8_t)plain[i + 1] << 8; }
      if (i + 2 < n) { v |= (uint8_t)plain[i + 2]; }
      enc[o++] = b64[v >> 18 & 63];
      enc[o++] = b64[v >> 12 & 63];
      enc[o++] = i + 1 < n ? b64[v >> 6 & 63] : '=';
      enc[o++] = i + 2 < n ? b64[v & 63] : '=';
    }
    enc[o] = 0;
    snprintf(auth, sizeof(auth), "Authorization: Basic %s\r\n", enc);
  }
  int n = snprintf(out, out_len,
                   "GET /%s HTTP/1.1\r\n"
                   "Host: %s:%u\r\n"
                   "Ntrip-Version: Ntrip/2.0\r\n"
                   "User-Agent: NTRIP esp32-to-gps/1.0\r\n"
                   "%s"
                   "Connection: close\r\n"
                   "\r\n",
                   mount, host, (unsigned)port, auth);
  return (n < 0 || (size_t)n >= out_len) ? 0 : (size_t)n;
}

static bool starts(const char *data, size_t len, const char *prefix) {
  size_t n = strlen(prefix);
  return len >= n && memcmp(data, prefix, n) == 0;
}

static const char *find(const char *data, size_t len, const char *needle) {
  size_t n = strlen(needle);
  for (size_t i = 0; i + n <= len; i++) {
    if (memcmp(data + i, needle, n) == 0) { return data + i; }
  }
  return 0;
}

static bool header_has(const char *data, size_t len, const char *name, const char *value) {
  /* case-insensitive header name and value, anywhere in the header block */
  size_t nn = strlen(name), nv = strlen(value);
  for (size_t i = 0; i + nn <= len; i++) {
    size_t k = 0;
    while (k < nn && (data[i + k] | 0x20) == (name[k] | 0x20)) { k++; }
    if (k < nn) { continue; }
    for (size_t j = i + nn; j + nv <= len && data[j] != '\r'; j++) {
      size_t m = 0;
      while (m < nv && (data[j + m] | 0x20) == (value[m] | 0x20)) { m++; }
      if (m == nv) { return true; }
    }
  }
  return false;
}

gnss_ntrip_reply_t gnss_ntrip_reply(const char *data, size_t len, size_t *header_len, bool *chunked) {
  *header_len = 0;
  *chunked = false;
  if (starts(data, len, "ICY 200 OK")) {
    /* NTRIP 1: the status line, then data; some casters add a blank line. */
    const char *eol = find(data, len, "\r\n");
    if (!eol) { return GNSS_NTRIP_INCOMPLETE; }
    size_t h = (size_t)(eol - data) + 2;
    /* RTCM data never starts with CR, so a blank line here is the caster's. */
    if (len >= h + 2 && data[h] == '\r' && data[h + 1] == '\n') { h += 2; }
    *header_len = h;
    return GNSS_NTRIP_OK;
  }
  if (starts(data, len, "SOURCETABLE 200")) { return GNSS_NTRIP_SOURCETABLE; }
  if (len < 12) { return GNSS_NTRIP_INCOMPLETE; }
  if (starts(data, len, "HTTP/1.")) {
    const char *end = find(data, len, "\r\n\r\n");
    if (!end) { return len > 2048 ? GNSS_NTRIP_ERROR : GNSS_NTRIP_INCOMPLETE; }
    size_t h = (size_t)(end - data) + 4;
    if (memcmp(data + 9, "401", 3) == 0) { return GNSS_NTRIP_UNAUTHORIZED; }
    if (memcmp(data + 9, "200", 3) != 0) { return GNSS_NTRIP_ERROR; }
    if (header_has(data, h, "Content-Type:", "gnss/sourcetable")) { return GNSS_NTRIP_SOURCETABLE; }
    *chunked = header_has(data, h, "Transfer-Encoding:", "chunked");
    *header_len = h;
    return GNSS_NTRIP_OK;
  }
  return GNSS_NTRIP_ERROR;
}

enum { DC_SIZE, DC_SIZE_EXT, DC_SIZE_LF, DC_DATA, DC_DATA_CR, DC_DATA_LF };

void gnss_dechunk_init(gnss_dechunk_t *d, bool declared) {
  memset(d, 0, sizeof(*d));
  d->active = declared;
  d->decided = declared;
  d->mode = DC_SIZE;
}

static int hexdigit(uint8_t c) {
  if (c >= '0' && c <= '9') { return c - '0'; }
  if (c >= 'a' && c <= 'f') { return c - 'a' + 10; }
  if (c >= 'A' && c <= 'F') { return c - 'A' + 10; }
  return -1;
}

/* Does the stream open with "<hex>" CR LF? 1 yes, 0 no, -1 too short to tell. */
static int looks_chunked(const uint8_t *p, size_t len) {
  size_t i = 0;
  while (i < len && i < 8 && hexdigit(p[i]) >= 0) { i++; }
  if (i == 0) { return 0; }
  if (i + 2 > len) { return len >= 10 ? 0 : -1; }
  return p[i] == 13 && p[i + 1] == 10;
}

size_t gnss_dechunk(gnss_dechunk_t *d, uint8_t *data, size_t len) {
  if (!d->decided) {
    int v = looks_chunked(data, len);
    if (v < 0) { return 0; }  /* the caller keeps these bytes and offers them again with more */
    d->active = v == 1;
    d->decided = true;
  }
  if (!d->active) { return len; }
  size_t o = 0;
  for (size_t i = 0; i < len; i++) {
    uint8_t b = data[i];
    switch (d->mode) {
      case DC_SIZE: {
        int h = hexdigit(b);
        if (h >= 0 && d->hex_digits < 8) { d->remaining = d->remaining << 4 | (uint32_t)h; d->hex_digits++; }
        else if (b == ';' || b == ' ') { d->mode = DC_SIZE_EXT; }
        else if (b == 13 && d->hex_digits) { d->mode = DC_SIZE_LF; }
        else { d->active = false; data[o++] = b; }  /* not chunk framing after all */
        break;
      }
      case DC_SIZE_EXT:
        if (b == 13) { d->mode = DC_SIZE_LF; }
        break;
      case DC_SIZE_LF:
        d->hex_digits = 0;
        d->chunks++;
        d->mode = d->remaining ? DC_DATA : DC_DATA_CR;  /* a zero-size chunk ends the body */
        break;
      case DC_DATA:
        data[o++] = b;
        if (--d->remaining == 0) { d->mode = DC_DATA_CR; }
        break;
      case DC_DATA_CR:
        d->mode = DC_DATA_LF;
        break;
      case DC_DATA_LF:
        d->mode = DC_SIZE;
        break;
    }
    if (!d->active) {  /* pass everything after a framing error through untouched */
      for (i++; i < len; i++) { data[o++] = data[i]; }
      break;
    }
  }
  return o;
}
