/*
  gnss_stream.h - split a receiver's serial output into NMEA sentences and UBX frames

  Receivers mix NMEA text and UBX binary on one UART. The framer takes one
  byte at a time and reports each complete frame whose checksum is right.

  SPDX-License-Identifier: GPL-3.0-or-later
*/

#ifndef GNSS_STREAM_H
#define GNSS_STREAM_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define GNSS_NMEA_MAX 160   /* longer than the 82 the standard allows: Quectel replies run longer */
#define GNSS_UBX_MAX 1024   /* NAV-SAT for 64 satellites is 8 + 12 x 64 = 776 bytes of payload */

typedef enum {
  GNSS_FRAME_NONE = 0,
  GNSS_FRAME_NMEA,  /* frame = the sentence from '$' to before '*', NUL-terminated */
  GNSS_FRAME_UBX,   /* frame = class, id, length (LE), payload */
  GNSS_FRAME_BAD_NMEA,
  GNSS_FRAME_BAD_UBX,
} gnss_frame_t;

typedef struct {
  uint8_t mode;        /* internal framing state */
  uint16_t len;        /* bytes in buf */
  uint16_t ubx_len;    /* payload length from the UBX header */
  uint8_t ck_a, ck_b;  /* running UBX checksum */
  uint8_t nmea_ck;     /* running NMEA checksum */
  uint8_t hex[2];      /* NMEA checksum digits */
  uint8_t buf[GNSS_UBX_MAX + 4];
} gnss_stream_t;

void gnss_stream_init(gnss_stream_t *st);

/* Feed one byte. When it completes a frame, returns its type: the frame is
 * in st->buf (st->len bytes) until the next call. Bad frames are reported
 * too, so the caller can count them. */
gnss_frame_t gnss_stream_byte(gnss_stream_t *st, uint8_t b);

/* A UBX frame in st->buf is: class, id, payload length (2 bytes, little
 * endian), then st->ubx_len bytes of payload starting at st->buf + 4. */

#ifdef __cplusplus
}
#endif

#endif /* GNSS_STREAM_H */
