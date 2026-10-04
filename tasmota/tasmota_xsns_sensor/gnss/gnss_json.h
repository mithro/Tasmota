/*
  gnss_json.h - the receiver state as JSON, and its Home Assistant discovery

  tele/<topic>/GNSS       the full state (gnss_json_state)
  tele/<topic>/GNSS_SATS  the satellite table (gnss_json_sats)
  homeassistant/<component>/<uid>/<entity>/config   discovery, retained

  Unknown values are JSON null: Home Assistant shows those entities as
  unknown rather than as a misleading zero.

  SPDX-License-Identifier: GPL-3.0-or-later
*/

#ifndef GNSS_JSON_H
#define GNSS_JSON_H

#include <stddef.h>
#include <stdint.h>

#include "gnss_rtcm.h"
#include "gnss_state.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
  GNSS_CORR_DISABLED = 0,
  GNSS_CORR_CONNECTING,
  GNSS_CORR_CONNECTED,
  GNSS_CORR_RETRYING,  /* the last attempt failed; waiting to try again */
} gnss_corr_state_t;

/* What the driver knows beyond the receiver's own output. */
typedef struct {
  uint8_t corr_state;          /* gnss_corr_state_t */
  const char *corr_mount;      /* "" when none */
  const char *corr_error;      /* why the last attempt failed, "" when it did not */
  const gnss_rtcm_t *rtcm;
  int32_t corr_age_s;          /* since the last good RTCM 3 frame, or the last byte for RTCM 2; -1 never */
  uint32_t baud;
  bool pps_present;            /* a pulse in the last 2 s */
  uint32_t pps_count;
  int32_t data_age_s;          /* since the last good frame from the receiver; -1 never */
} gnss_extra_t;

/* Write JSON into out. Each returns the length, or 0 when out is too small. */
size_t gnss_json_state(char *out, size_t out_len, const gnss_state_t *s, const gnss_extra_t *x);
size_t gnss_json_sats(char *out, size_t out_len, const gnss_state_t *s);

/* The Home Assistant device all the entities belong to. */
typedef struct {
  const char *uid;      /* unique per ESP32: "gps_" + MAC */
  const char *name;     /* Tasmota DeviceName */
  const char *topic;    /* full topic prefix for state: "tele/<topic>/" */
  const char *lwt;      /* availability topic: "tele/<topic>/LWT" */
  const char *model;    /* receiver model, "" when not yet known */
  const char *sw;       /* firmware version string for the device */
  const char *url;      /* configuration_url: the ESP32's web page, "" for none */
} gnss_hass_device_t;

/* Number of fixed entities. */
int gnss_hass_count(void);

/* Topic and payload of fixed entity number index. Returns the payload
 * length, or 0 when a buffer is too small or index is out of range. */
size_t gnss_hass_config(int index, const gnss_hass_device_t *dev, char *topic, size_t topic_len,
                        char *payload, size_t payload_len);

/* Topic and payload for one satellite's C/N0 entity (the per-satellite
 * entities, off by default). An empty payload (removal) when remove is set. */
size_t gnss_hass_sat_config(uint8_t gnss, uint8_t svid, bool remove, const gnss_hass_device_t *dev,
                            char *topic, size_t topic_len, char *payload, size_t payload_len);

#ifdef __cplusplus
}
#endif

#endif /* GNSS_JSON_H */
