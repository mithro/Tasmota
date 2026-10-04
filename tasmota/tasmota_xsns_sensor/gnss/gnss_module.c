/*
  gnss_module.c - identify the receiver and configure it

  SPDX-License-Identifier: GPL-3.0-or-later
*/

#include "gnss_module.h"

#include <stdio.h>
#include <string.h>

#include "gnss_ubx.h"

#define BAUD_UBLOX_LEGACY 38400  /* u-blox 7 / M8: raised from 9600 so the UBX messages fit */
#define BAUD_M10 38400           /* the MAX-M10S default */
#define BAUD_LC29H 115200        /* the LC29H default */

uint32_t gnss_module_baud(uint8_t module) {
  switch (module) {
    case GNSS_MODULE_UBLOX7:
    case GNSS_MODULE_UBLOX_M8: return BAUD_UBLOX_LEGACY;
    case GNSS_MODULE_UBLOX_M10: return BAUD_M10;
    case GNSS_MODULE_QUECTEL_LC29H: return BAUD_LC29H;
    default: return 0;  /* unknown: stay at whatever speed found it */
  }
}

size_t gnss_nmea_command(char *out, size_t out_len, const char *body) {
  uint8_t ck = 0;
  for (const char *p = body; *p; p++) { ck ^= (uint8_t)*p; }
  int n = snprintf(out, out_len, "$%s*%02X\r\n", body, ck);
  return (n < 0 || (size_t)n >= out_len) ? 0 : (size_t)n;
}

size_t gnss_module_probe(uint8_t step, uint8_t *out, size_t out_len) {
  switch (step) {
    case 0: return gnss_ubx_poll(out, out_len, 0x0A, 0x04);  /* MON-VER */
    case 1: return gnss_nmea_command((char *)out, out_len, "PQTMVERNO");
    default: return 0;
  }
}

/* --- u-blox 7 and M8: legacy messages ------------------------------------ */

/* CFG-PRT for UART1 (portID 1): 8N1 at the given speed. */
static size_t cfg_prt(uint8_t *out, size_t out_len, uint32_t baud, uint16_t in_proto) {
  uint8_t p[20] = {0};
  p[0] = 1;                                   /* UART1 */
  p[4] = 0xC0; p[5] = 0x08;                   /* mode 0x000008C0: 8 data bits, no parity, 1 stop bit */
  for (int k = 0; k < 4; k++) { p[8 + k] = (uint8_t)(baud >> (8 * k)); }
  p[12] = (uint8_t)in_proto; p[13] = (uint8_t)(in_proto >> 8);
  p[14] = 0x03;                               /* out: UBX + NMEA */
  return gnss_ubx_frame(out, out_len, 0x06, 0x00, p, sizeof(p));
}

typedef struct { uint8_t cls, id, rate; } msg_rate_t;

/* Shared by the u-blox 7 and M8. NMEA GSA, GSV, GLL and VTG are turned off:
 * UBX carries the same information more completely. GGA stays for the
 * differential age and station, RMC for the date. */
static const msg_rate_t LEGACY_COMMON[] = {
  {0x01, 0x07, 1},   /* NAV-PVT */
  {0x01, 0x04, 1},   /* NAV-DOP */
  {0x0A, 0x09, 5},   /* MON-HW, every 5 solutions */
  {0xF0, 0x00, 1},   /* GGA */
  {0xF0, 0x04, 1},   /* RMC */
  {0xF0, 0x01, 0},   /* GLL */
  {0xF0, 0x02, 0},   /* GSA */
  {0xF0, 0x03, 0},   /* GSV */
  {0xF0, 0x05, 0},   /* VTG */
  {0x01, 0x02, 0},   /* NAV-POSLLH, NAV-STATUS, NAV-TIMEUTC, NAV-VELNED: what */
  {0x01, 0x03, 0},   /*   upstream xsns_60 enables; this firmware uses NAV-PVT */
  {0x01, 0x21, 0},
  {0x01, 0x12, 0},
};
static const msg_rate_t UBLOX7_EXTRA[] = {
  {0x01, 0x30, 1},   /* NAV-SVINFO: the u-blox 7 has no NAV-SAT */
};
static const msg_rate_t M8_EXTRA[] = {
  {0x01, 0x35, 1},   /* NAV-SAT */
  {0x02, 0x32, 1},   /* RXM-RTCM: which corrections were used */
  {0x0D, 0x01, 1},   /* TIM-TP: the LEA-M8T's timepulse quantization error */
};
#define N(a) (sizeof(a) / sizeof((a)[0]))

static size_t legacy(uint8_t module, uint8_t index, uint8_t *out, size_t out_len, uint32_t *switch_baud) {
  if (index == 0) {
    /* in: UBX + NMEA + RTCM 2 (bit 2), and RTCM 3 (bit 5) on the M8 */
    *switch_baud = BAUD_UBLOX_LEGACY;
    return cfg_prt(out, out_len, BAUD_UBLOX_LEGACY, module == GNSS_MODULE_UBLOX_M8 ? 0x0027 : 0x0007);
  }
  index--;
  if (index < N(LEGACY_COMMON)) {
    const msg_rate_t *m = &LEGACY_COMMON[index];
    return gnss_ubx_cfg_msg(out, out_len, m->cls, m->id, m->rate);
  }
  index = (uint8_t)(index - N(LEGACY_COMMON));
  const msg_rate_t *extra = module == GNSS_MODULE_UBLOX7 ? UBLOX7_EXTRA : M8_EXTRA;
  size_t n_extra = module == GNSS_MODULE_UBLOX7 ? N(UBLOX7_EXTRA) : N(M8_EXTRA);
  if (index < n_extra) { return gnss_ubx_cfg_msg(out, out_len, extra[index].cls, extra[index].id, extra[index].rate); }
  index = (uint8_t)(index - n_extra);
  if (index == 0) { return gnss_ubx_poll(out, out_len, 0x0A, 0x04); }  /* MON-VER again, for the record */
  return 0;
}

/* --- M10: key/value configuration ---------------------------------------- */

/* Key IDs from the u-blox M10 interface description; every one is checked
 * against pyubx2 by the host tests, and those also used by ten64's
 * ten64-gps-stationary were proven on its MAX-M10S. */
static const uint32_t M10_KEYS[] = {
  0x10730001, 0x10730002, 0x10730004,  /* UART1INPROT: UBX, NMEA, RTCM3X */
  0x10740001, 0x10740002,              /* UART1OUTPROT: UBX, NMEA */
  0x20910007, 0x20910039, 0x20910016,  /* MSGOUT UART1: NAV_PVT, NAV_DOP, NAV_SAT */
  0x2091035a, 0x20910269,              /* MON_RF, RXM_RTCM */
  0x209100bb, 0x209100ac,              /* NMEA GGA, RMC */
  0x209100c0, 0x209100c5, 0x209100ca,  /* NMEA GSA, GSV, GLL */
  0x209100b1, 0x209100d9,              /* NMEA VTG, ZDA */
  0x20910346, 0x20910025, 0x2091003e,  /* NAV_SIG, NAV_POSECEF, NAV_VELECEF: off */
  0x20910048, 0x20910066, 0x20910160,  /* NAV_TIMEGPS, NAV_CLOCK, NAV_EOE: off */
};
static const uint64_t M10_VALUES[] = {
  1, 1, 1,
  1, 1,
  1, 1, 1,
  5, 1,
  1, 1,
  0, 0, 0,
  0, 0,
  0, 0, 0,
  0, 0, 0,
};

static size_t m10(uint8_t index, uint8_t *out, size_t out_len) {
  switch (index) {
    case 0: return gnss_ubx_cfg_valset(out, out_len, 0x01 /* RAM */, M10_KEYS, M10_VALUES, (uint8_t)N(M10_KEYS));
    default: return 0;
  }
}

/* --- Quectel LC29H --------------------------------------------------------- */

static const char *const LC29H_COMMANDS[] = {
  "PAIR021",       /* chip and firmware version */
  "PAIR062,8,1",   /* GST every fix: position error estimates. Confirmed on the LC29H(AA) by
                      Quectel's forum (not supported on the DA variant). */
};

static size_t lc29h(uint8_t index, uint8_t *out, size_t out_len) {
  if (index >= N(LC29H_COMMANDS)) { return 0; }
  return gnss_nmea_command((char *)out, out_len, LC29H_COMMANDS[index]);
}

size_t gnss_module_config(uint8_t module, uint8_t index, uint8_t *out, size_t out_len, uint32_t *switch_baud) {
  *switch_baud = 0;
  switch (module) {
    case GNSS_MODULE_UBLOX7:
    case GNSS_MODULE_UBLOX_M8: return legacy(module, index, out, out_len, switch_baud);
    case GNSS_MODULE_UBLOX_M10: return m10(index, out, out_len);
    case GNSS_MODULE_QUECTEL_LC29H: return lc29h(index, out, out_len);
    default: return 0;
  }
}

const char *gnss_module_mount(uint8_t module) {
  switch (module) {
    case GNSS_MODULE_UBLOX7: return "ADDE_RTCM23";  /* takes RTCM 2.3 only */
    case GNSS_MODULE_UBLOX_M8:
    case GNSS_MODULE_UBLOX_M10:
    case GNSS_MODULE_QUECTEL_LC29H: return "ADDE_RTCM3";
    default: return "";
  }
}
