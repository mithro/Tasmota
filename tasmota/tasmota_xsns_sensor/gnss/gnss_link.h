/*
  gnss_link.h - bring the link to the receiver up, and keep it up

  Decides which UART speed to listen at, whether the receiver must be
  identified, and what to send it, from what is saved about it:

  * A receiver already known (fixed with GpsModule, or remembered from an
    earlier identification) is listened for at its likely speeds and, as
    soon as valid data arrives, configured. No speed search, no probe.
  * An unknown receiver is searched for at every speed, identified, and
    remembered: the caller saves known_module / known_baud when
    action.save is set.
  * If a remembered receiver's own identity reply shows a different one,
    the new one is remembered and configured. A fixed receiver is kept, and
    the mismatch logged once.
  * A receiver silent for 10 s is looked for again, the same way.

  Configuration still goes to the receiver at every start: it is written to
  RAM only (the MAX-M10S has no flash), so it is gone after a power cut.

  The caller does the I/O: it calls gnss_link_frame() for every valid frame
  and gnss_link_step() often, and carries out the action each step returns.

  SPDX-License-Identifier: GPL-3.0-or-later
*/

#ifndef GNSS_LINK_H
#define GNSS_LINK_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "gnss_state.h"

#ifdef __cplusplus
extern "C" {
#endif

#define GNSS_LINK_LISTEN_MS 2500   /* time at each speed */
#define GNSS_LINK_PROBE_MS 1500    /* time for an identification reply */
#define GNSS_LINK_GAP_MS 150       /* between configuration commands */
#define GNSS_LINK_SILENT_MS 10000  /* no valid frame this long: look again */

typedef enum {
  GNSS_LINK_LISTEN = 0,   /* listening for a known receiver at its likely speeds */
  GNSS_LINK_SEARCH,       /* listening at every speed */
  GNSS_LINK_PROBE,        /* asking an unknown receiver what it is */
  GNSS_LINK_CONFIGURE,    /* sending the configuration */
  GNSS_LINK_RUNNING,
} gnss_link_phase_t;

/* What is saved about the receiver. */
typedef struct {
  uint8_t fixed_module;   /* GpsModule: 0 = automatic */
  uint32_t fixed_baud;    /* GpsBaud: 0 = automatic */
  uint8_t known_module;   /* remembered from the last identification */
  uint32_t known_baud;    /* the speed it was found at */
} gnss_link_cfg_t;

typedef struct {
  uint32_t baud;          /* reopen the UART at this speed first; 0 = no change */
  uint8_t send[256];      /* then send these bytes */
  size_t send_len;
  uint32_t baud_after;    /* then, once they have left the UART, reopen at this speed; 0 = no change */
  bool save;              /* cfg.known_module / known_baud changed: save the settings */
  char log[96];           /* a line for the log, "" for none */
} gnss_link_action_t;

typedef struct {
  gnss_link_cfg_t cfg;
  uint8_t phase;          /* gnss_link_phase_t */
  uint8_t module;         /* the receiver being configured / run */
  uint32_t baud;          /* the UART speed now */
  uint32_t bauds[4];      /* speeds to listen at */
  uint8_t n_bauds, baud_index;
  uint32_t phase_ms;      /* when the current phase, speed or step began */
  uint8_t step;           /* probe step or configuration command */
  bool got_frame;         /* a valid frame since the current speed began */
  uint32_t last_frame_ms;
  bool mismatch_logged;
  bool identified;        /* the module was identified, not assumed */
} gnss_link_t;

/* Start (or restart) bringing the link up. state->module is cleared. */
void gnss_link_start(gnss_link_t *l, const gnss_link_cfg_t *cfg, gnss_state_t *state, uint32_t now_ms);

/* Forget the remembered receiver and identify it from scratch. Sets save. */
void gnss_link_forget(gnss_link_t *l, gnss_state_t *state, uint32_t now_ms, gnss_link_action_t *a);

/* A valid frame arrived from the receiver. */
void gnss_link_frame(gnss_link_t *l, uint32_t now_ms);

/* Advance. Returns true when *a holds something to do. */
bool gnss_link_step(gnss_link_t *l, gnss_state_t *state, uint32_t now_ms, gnss_link_action_t *a);

#ifdef __cplusplus
}
#endif

#endif /* GNSS_LINK_H */
