/*
  gnss_module.h - identify the receiver and configure it

  Identification: a UBX MON-VER poll, then the Quectel version queries; the
  first reply decides (decoded by gnss_ubx / gnss_nmea into gnss_state_t.module).

  Configuration goes to RAM only and is sent again after every
  identification: the MAX-M10S has no flash, so nothing may depend on saved
  settings.

  SPDX-License-Identifier: GPL-3.0-or-later
*/

#ifndef GNSS_MODULE_H
#define GNSS_MODULE_H

#include <stddef.h>
#include <stdint.h>

#include "gnss_state.h"

#ifdef __cplusplus
extern "C" {
#endif

/* The UART speed the driver runs each receiver at. */
uint32_t gnss_module_baud(uint8_t module);

/* Identification probes, step 0 and 1. Returns the bytes to send, or 0
 * past the last step. */
size_t gnss_module_probe(uint8_t step, uint8_t *out, size_t out_len);

/* Configuration command number `index` for a receiver. Returns its length,
 * or 0 after the last one. When *switch_baud is set on return, send the
 * command, wait for it to leave the UART, then reopen the UART at that
 * speed before sending the next. */
size_t gnss_module_config(uint8_t module, uint8_t index, uint8_t *out, size_t out_len, uint32_t *switch_baud);

/* The ten64 proxy mountpoint each receiver takes corrections from. */
const char *gnss_module_mount(uint8_t module);

/* "$" + body + "*" + checksum + CR LF. Returns the length, or 0 when out is
 * too small. */
size_t gnss_nmea_command(char *out, size_t out_len, const char *body);

#ifdef __cplusplus
}
#endif

#endif /* GNSS_MODULE_H */
