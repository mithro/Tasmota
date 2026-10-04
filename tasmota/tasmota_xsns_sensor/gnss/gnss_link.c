/*
  gnss_link.c - bring the link to the receiver up, and keep it up

  SPDX-License-Identifier: GPL-3.0-or-later
*/

#include "gnss_link.h"

#include <stdio.h>
#include <string.h>

#include "gnss_module.h"

static const uint32_t ALL_BAUDS[] = {9600, 38400, 115200};

static void clear(gnss_link_action_t *a) {
  a->baud = 0;
  a->send_len = 0;
  a->baud_after = 0;
  a->save = false;
  a->log[0] = 0;
}

/* Begin listening at bauds[baud_index]. */
static void listen_at(gnss_link_t *l, uint32_t now, gnss_link_action_t *a) {
  l->baud = l->bauds[l->baud_index];
  l->phase_ms = now;
  l->got_frame = false;
  a->baud = l->baud;
}

static void set_bauds(gnss_link_t *l, uint8_t phase) {
  l->phase = phase;
  l->baud_index = 0;
  if (l->cfg.fixed_baud) {
    l->bauds[0] = l->cfg.fixed_baud;
    l->n_bauds = 1;
  } else if (phase == GNSS_LINK_LISTEN) {
    l->n_bauds = gnss_module_boot_bauds(l->module, l->cfg.known_baud, l->bauds, 4);
  } else {
    l->n_bauds = (uint8_t)(sizeof(ALL_BAUDS) / sizeof(ALL_BAUDS[0]));
    memcpy(l->bauds, ALL_BAUDS, sizeof(ALL_BAUDS));
  }
  l->baud = 0;  /* the first step opens the UART */
}

static uint8_t assumed_module(const gnss_link_cfg_t *cfg) {
  return cfg->fixed_module ? cfg->fixed_module : cfg->known_module;
}

void gnss_link_start(gnss_link_t *l, const gnss_link_cfg_t *cfg, gnss_state_t *state, uint32_t now) {
  gnss_link_cfg_t c = *cfg;
  memset(l, 0, sizeof(*l));
  l->cfg = c;
  l->module = assumed_module(&c);
  l->phase_ms = now;
  l->last_frame_ms = now;
  state->module = GNSS_MODULE_UNKNOWN;
  set_bauds(l, l->module ? GNSS_LINK_LISTEN : GNSS_LINK_SEARCH);
}

void gnss_link_forget(gnss_link_t *l, gnss_state_t *state, uint32_t now, gnss_link_action_t *a) {
  gnss_link_cfg_t c = l->cfg;
  clear(a);
  if (c.known_module || c.known_baud) {
    c.known_module = 0;
    c.known_baud = 0;
    a->save = true;
  }
  gnss_link_start(l, &c, state, now);
}

void gnss_link_frame(gnss_link_t *l, uint32_t now) {
  l->got_frame = true;
  l->last_frame_ms = now;
}

static void start_configure(gnss_link_t *l, uint32_t now) {
  l->phase = GNSS_LINK_CONFIGURE;
  l->step = 0;
  l->phase_ms = now - GNSS_LINK_GAP_MS;  /* the first command goes at once */
}

/* Remember where the receiver was found. */
static void remember(gnss_link_t *l, uint8_t module, gnss_link_action_t *a) {
  if (l->cfg.fixed_module) { module = l->cfg.known_module; }  /* a fixed module is not "remembered" */
  if (l->cfg.known_module != module || l->cfg.known_baud != l->baud) {
    l->cfg.known_module = module;
    l->cfg.known_baud = l->baud;
    a->save = true;
  }
}

static bool listening(gnss_link_t *l, gnss_state_t *state, uint32_t now, gnss_link_action_t *a) {
  if (l->baud == 0) {           /* first step: open the UART */
    listen_at(l, now, a);
    return true;
  }
  if (l->got_frame) {
    if (l->phase == GNSS_LINK_LISTEN || l->cfg.fixed_module) {
      /* a known receiver: configure it, no probing */
      remember(l, l->module, a);
      snprintf(a->log, sizeof(a->log), "GPS: %s receiver at %u baud, %s", gnss_module_key(l->module),
               (unsigned)l->baud, l->cfg.fixed_module ? "set by GpsModule" : "as last time");
      start_configure(l, now);
    } else {
      l->phase = GNSS_LINK_PROBE;
      l->step = 0;
      state->module = GNSS_MODULE_UNKNOWN;
      a->send_len = gnss_module_probe(0, a->send, sizeof(a->send));
      l->phase_ms = now;
    }
    return true;
  }
  if (now - l->phase_ms < GNSS_LINK_LISTEN_MS) { return false; }
  /* nothing at this speed: the next one, and after the last of a known
   * receiver's speeds, every speed */
  if (++l->baud_index >= l->n_bauds) {
    if (l->phase == GNSS_LINK_LISTEN && !l->cfg.fixed_baud) {
      snprintf(a->log, sizeof(a->log), "GPS: no %s receiver at its usual speeds, searching",
               gnss_module_key(l->module));
      set_bauds(l, GNSS_LINK_SEARCH);
    }
    l->baud_index = 0;
  }
  listen_at(l, now, a);
  return true;
}

static bool probing(gnss_link_t *l, gnss_state_t *state, uint32_t now, gnss_link_action_t *a) {
  if (state->module != GNSS_MODULE_UNKNOWN) {
    l->module = state->module;
    l->identified = true;
    remember(l, l->module, a);
    snprintf(a->log, sizeof(a->log), "GPS: identified %s (%s) at %u baud", state->model[0] ? state->model : "receiver",
             gnss_module_key(l->module), (unsigned)l->baud);
    start_configure(l, now);
    return true;
  }
  if (now - l->phase_ms < GNSS_LINK_PROBE_MS) { return false; }
  l->step++;
  a->send_len = gnss_module_probe(l->step, a->send, sizeof(a->send));
  l->phase_ms = now;
  if (!a->send_len) {
    snprintf(a->log, sizeof(a->log), "GPS: receiver not identified; reading its NMEA as it is");
    l->phase = GNSS_LINK_RUNNING;
  }
  return true;
}

static bool configuring(gnss_link_t *l, uint32_t now, gnss_link_action_t *a) {
  if (now - l->phase_ms < GNSS_LINK_GAP_MS) { return false; }
  uint32_t switch_baud = 0;
  a->send_len = gnss_module_config(l->module, l->step, a->send, sizeof(a->send), &switch_baud);
  if (!a->send_len) {
    l->phase = GNSS_LINK_RUNNING;
    snprintf(a->log, sizeof(a->log), "GPS: %s configured", gnss_module_key(l->module));
    return true;
  }
  if (switch_baud && switch_baud != l->baud && !l->cfg.fixed_baud) {
    a->baud_after = switch_baud;
    l->baud = switch_baud;
  }
  l->step++;
  l->phase_ms = now;
  return true;
}

static bool running(gnss_link_t *l, gnss_state_t *state, uint32_t now, gnss_link_action_t *a) {
  uint8_t reported = state->module;
  if (reported != GNSS_MODULE_UNKNOWN && reported != l->module && l->phase == GNSS_LINK_RUNNING) {
    if (l->cfg.fixed_module) {
      if (!l->mismatch_logged) {
        snprintf(a->log, sizeof(a->log), "GPS: GpsModule says %s, but the receiver reports %s",
                 gnss_module_key(l->module), gnss_module_key(reported));
        l->mismatch_logged = true;
        return true;
      }
    } else {
      snprintf(a->log, sizeof(a->log), "GPS: receiver is %s, not %s: configuring it as that",
               gnss_module_key(reported), gnss_module_key(l->module));
      l->module = reported;
      remember(l, reported, a);
      start_configure(l, now);
      return true;
    }
  }
  if (now - l->last_frame_ms > GNSS_LINK_SILENT_MS) {
    gnss_link_start(l, &l->cfg, state, now);
    clear(a);
    snprintf(a->log, sizeof(a->log), "GPS: receiver silent, looking for it again");
    return true;
  }
  return false;
}

bool gnss_link_step(gnss_link_t *l, gnss_state_t *state, uint32_t now, gnss_link_action_t *a) {
  clear(a);
  switch (l->phase) {
    case GNSS_LINK_LISTEN:
    case GNSS_LINK_SEARCH: return listening(l, state, now, a);
    case GNSS_LINK_PROBE: return probing(l, state, now, a);
    case GNSS_LINK_CONFIGURE:
      if (now - l->last_frame_ms > GNSS_LINK_SILENT_MS) { return running(l, state, now, a); }
      return configuring(l, now, a);
    default: return running(l, state, now, a);
  }
}
