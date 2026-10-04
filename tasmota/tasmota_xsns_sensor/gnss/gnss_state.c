/*
  gnss_state.c - the receiver state and its satellite table

  SPDX-License-Identifier: GPL-3.0-or-later
*/

#include "gnss_state.h"

#include <string.h>

void gnss_state_init(gnss_state_t *s) {
  memset(s, 0, sizeof(*s));
  s->gdop = s->pdop = s->hdop = s->vdop = s->tdop = 0xffff;
  s->diff_age_ds = -1;
  s->diff_station = -1;
  s->antenna = GNSS_ANT_NOT_REPORTED;
  s->epoch = 1;
}

void gnss_new_epoch(gnss_state_t *s) {
  s->epoch++;
  gnss_prune_sats(s);
}

gnss_sat_t *gnss_sat(gnss_state_t *s, uint8_t gnss, uint8_t svid) {
  for (uint8_t i = 0; i < s->n_sats; i++) {
    if (s->sats[i].gnss == gnss && s->sats[i].svid == svid) {
      s->sats[i].epoch = s->epoch;
      return &s->sats[i];
    }
  }
  if (s->n_sats >= GNSS_MAX_SATS) {
    gnss_prune_sats(s);
    if (s->n_sats >= GNSS_MAX_SATS) { return 0; }
  }
  gnss_sat_t *sat = &s->sats[s->n_sats++];
  memset(sat, 0, sizeof(*sat));
  sat->gnss = gnss;
  sat->svid = svid;
  sat->elev = -91;
  sat->azim = -1;
  sat->epoch = s->epoch;
  return sat;
}

void gnss_prune_sats(gnss_state_t *s) {
  uint8_t keep = 0;
  for (uint8_t i = 0; i < s->n_sats; i++) {
    if (s->sats[i].epoch + 1 >= s->epoch) {
      s->sats[keep++] = s->sats[i];
    }
  }
  s->n_sats = keep;
}

uint8_t gnss_count_sats(const gnss_state_t *s, uint8_t gnss, bool used_only) {
  uint8_t n = 0;
  for (uint8_t i = 0; i < s->n_sats; i++) {
    const gnss_sat_t *sat = &s->sats[i];
    if (gnss != GNSS_UNKNOWN && sat->gnss != gnss) { continue; }
    if (used_only && !sat->used) { continue; }
    n++;
  }
  return n;
}

uint8_t gnss_cno_stats(const gnss_state_t *s, uint8_t *min, uint8_t *avg, uint8_t *max) {
  uint8_t n = 0, lo = 255, hi = 0;
  uint32_t sum = 0;
  for (uint8_t i = 0; i < s->n_sats; i++) {
    const gnss_sat_t *sat = &s->sats[i];
    if (!sat->used || sat->cno == 0) { continue; }
    n++;
    sum += sat->cno;
    if (sat->cno < lo) { lo = sat->cno; }
    if (sat->cno > hi) { hi = sat->cno; }
  }
  *min = n ? lo : 0;
  *max = hi;
  *avg = n ? (uint8_t)((sum + n / 2) / n) : 0;
  return n;
}

const char *gnss_constellation_name(uint8_t gnss) {
  switch (gnss) {
    case GNSS_GPS: return "GPS";
    case GNSS_SBAS: return "SBAS";
    case GNSS_GALILEO: return "Galileo";
    case GNSS_BEIDOU: return "BeiDou";
    case GNSS_IMES: return "IMES";
    case GNSS_QZSS: return "QZSS";
    case GNSS_GLONASS: return "GLONASS";
    case GNSS_NAVIC: return "NavIC";
    default: return "Unknown";
  }
}

char gnss_constellation_letter(uint8_t gnss) {
  switch (gnss) {
    case GNSS_GPS: return 'G';
    case GNSS_SBAS: return 'S';
    case GNSS_GALILEO: return 'E';
    case GNSS_BEIDOU: return 'C';
    case GNSS_IMES: return 'M';
    case GNSS_QZSS: return 'J';
    case GNSS_GLONASS: return 'R';
    case GNSS_NAVIC: return 'I';
    default: return '?';
  }
}
