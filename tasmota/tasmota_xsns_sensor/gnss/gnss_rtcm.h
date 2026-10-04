/*
  gnss_rtcm.h - watch the RTCM correction stream on its way to the receiver,
  and the NTRIP request and reply around it

  The bytes themselves go to the receiver unchanged. RTCM 3 frames are
  checked (CRC-24Q) and counted by message type so Home Assistant can show
  what is arriving; RTCM 2.3 is only counted.

  SPDX-License-Identifier: GPL-3.0-or-later
*/

#ifndef GNSS_RTCM_H
#define GNSS_RTCM_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define GNSS_RTCM_TYPES 16  /* distinct RTCM 3 message types remembered */

typedef struct {
  uint32_t bytes;       /* everything received from the caster */
  uint32_t frames;      /* RTCM 3 frames with a good CRC */
  uint32_t bad_crc;     /* RTCM 3 frames with a bad CRC */
  uint16_t last_type;   /* the latest good frame's message type */
  uint16_t station;     /* reference station ID from the latest 1005/1006 */
  bool station_valid;
  uint8_t n_types;
  uint16_t types[GNSS_RTCM_TYPES];   /* message types seen, in order of first sight */
  uint32_t counts[GNSS_RTCM_TYPES];  /* frames of each */
  /* framing state */
  uint16_t len, need;
  uint8_t buf[1029];   /* 3-byte header + up to 1023 payload + 3-byte CRC */
} gnss_rtcm_t;

void gnss_rtcm_init(gnss_rtcm_t *r);

/* Watch a block of bytes going to the receiver. Returns how many complete
 * good RTCM 3 frames it finished. */
uint32_t gnss_rtcm_feed(gnss_rtcm_t *r, const uint8_t *data, size_t len);

/* CRC-24Q, as RTCM 3 and SBAS use it. */
uint32_t gnss_crc24q(const uint8_t *data, size_t len);

/* The NTRIP request for a mountpoint. Version 2, which the ten64 proxy and
 * most casters answer; user and password may be NULL or empty for none.
 * Returns the length written, or 0 when out is too small. */
size_t gnss_ntrip_request(char *out, size_t out_len, const char *host, uint16_t port, const char *mount,
                          const char *user, const char *password);

typedef enum {
  GNSS_NTRIP_INCOMPLETE = 0,  /* need more bytes */
  GNSS_NTRIP_OK,              /* streaming starts after the header */
  GNSS_NTRIP_SOURCETABLE,     /* the mountpoint is unknown or down */
  GNSS_NTRIP_UNAUTHORIZED,
  GNSS_NTRIP_ERROR,           /* any other reply */
} gnss_ntrip_reply_t;

/* Look at the start of the caster's reply. On GNSS_NTRIP_OK, *header_len is
 * the number of bytes before the correction data begins, and *chunked says
 * whether the reply declared HTTP chunked transfer encoding. Accepts NTRIP 1
 * ("ICY 200 OK") and NTRIP 2 ("HTTP/1.x 200 OK"). */
gnss_ntrip_reply_t gnss_ntrip_reply(const char *data, size_t len, size_t *header_len, bool *chunked);

/* Removes HTTP chunked transfer framing from the correction stream.
 *
 * Besides casters that declare it, the ten64 proxy (ntrip-rtcm3-to-rtcm2p3
 * 0.1.0.post39) passes the upstream caster's chunk framing through on its
 * RTCM 3 mountpoint without declaring it, so it is also switched on when the
 * first bytes of the stream are a chunk header. */
typedef struct {
  uint8_t mode;       /* internal */
  bool active;        /* stripping chunk framing */
  bool decided;       /* the start of the stream has been looked at */
  uint32_t remaining; /* data bytes left in the current chunk */
  uint8_t hex_digits;
  uint32_t chunks;    /* chunk headers removed */
} gnss_dechunk_t;

/* declared: the reply said "Transfer-Encoding: chunked". Otherwise the
 * start of the stream decides. */
void gnss_dechunk_init(gnss_dechunk_t *d, bool declared);

/* Strip framing in place. Returns the new length of data.
 *
 * Until it has decided, it needs the first 10 bytes of the stream (fewer if
 * they settle it). Given less, it returns 0 and leaves data untouched: call
 * again with those bytes and more. RTCM 3 starts with 0xD3 and RTCM 2 bytes
 * are 0x40-0x7F, so neither can look like a chunk header ending in CR LF. */
size_t gnss_dechunk(gnss_dechunk_t *d, uint8_t *data, size_t len);

#ifdef __cplusplus
}
#endif

#endif /* GNSS_RTCM_H */
