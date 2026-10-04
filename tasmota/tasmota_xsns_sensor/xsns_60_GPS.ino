/*
  xsns_60_GPS.ino - GPS UBLOX support for Tasmota

  Copyright (C) 2021  Theo Arends, Christian Baars and Adrian Scillato

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifdef USE_GPS
/*********************************************************************************************\
  --------------------------------------------------------------------------------------------
  Version Date      Action    Description
  --------------------------------------------------------------------------------------------

  1.0.0.0 20261004  extend    - esp32-to-gps (github.com/mithro/esp32-to-gps): NMEA and UBX through
                                the pure-C core in gnss/, receiver identification and configuration
                                (u-blox 7, M8, M10, Quectel LC29H), NTRIP corrections, PPS input,
                                full state over MQTT with Home Assistant discovery, Gps* commands.
  ---
  0.9.3.0 20200214  integrate - fix set lat/lon via commandd 13, V-Port now works parallel
  ---
  0.9.2.0 20200110  integrate - Added UART-over-TCP/IP-bridge (virtual serial port). Minor tweaks.
  ---
  0.9.1.0 20191216  integrate - Added pin specifications from Tasmota WEB UI. Minor tweaks.
  ---
  0.9.0.0 20190817  started   - further development by Christian Baars  - https://github.com/Staars/Sonoff-Tasmota
                    forked    - from arendst/tasmota                    - https://github.com/arendst/Sonoff-Tasmota
                    base      - code base from arendst and              - https://www.youtube.com/watch?v=TwhCX0c8Xe0

## GPS driver

Reads the receiver through the pure-C core in tasmota_xsns_sensor/gnss/, which frames and decodes
NMEA and UBX, identifies the receiver and builds its configuration. The core is compiled and tested
on a host by github.com/mithro/esp32-to-gps.

Supported receivers: u-blox 7, u-blox M8 (including the LEA-M8T), u-blox M10 (MAX-M10S) and the
Quectel LC29H. Any other receiver that speaks NMEA is read, but not configured.

## Features:
- position, time, motion, accuracy, DOPs, every satellite with its signal, receiver identity and RF
- sets system time automatically and Settings->latitude and Settings->longitude via command
- RTCM corrections from an NTRIP caster, passed to the receiver
- one pulse per second input (GPIO "GPS PPS")
- full state on tele/<topic>/GNSS and tele/<topic>/GNSS_SATS, with Home Assistant discovery
- can log postion data with timestamp to flash with a small memory footprint of only 12 Bytes per record
- constructs a GPX-file for download of this data
- Web-UI
- simplified NTP-server and UART-over-TCP/IP-bridge (virtual serial port)

## Usage:
The serial pins are GPS_RX and GPS_TX, and optionally GPS PPS. The GPS_RX variant no longer sets the
speed: the driver searches 9600, 38400 and 115200 baud unless GpsBaud fixes it.

## Commands:

+ GpsBaud <speed>          fix the receiver's UART speed; 0 searches (default)
+ GpsNtrip 0               corrections off
+ GpsNtrip 1               corrections from the ten64 proxy, mountpoint chosen by receiver (default)
+ GpsNtrip <host>:<port>/<mount>[ <user> <password>]   corrections from a caster of your choice
+ GpsPeriod <seconds>      how often tele/<topic>/GNSS is published (default 10)
+ GpsSatEntities 0|1       one Home Assistant entity per satellite (default 0)
+ GpsHass 0|1              Home Assistant discovery (default 1)
+ GpsReinit                identify and configure the receiver again
+ GpsStatus                the full state, as published on tele/<topic>/GNSS

+ sensor60 0 .. 15, 1001 .. 1065   as before (flash log, noise filter, NTP server, virtual port, rate)

## Rules examples for SSD1306 32x128


rule1 on tele-GPS#lat do DisplayText [s1p21c1l01f1]LAT: %value% endon on tele-GPS#lon do DisplayText [s1p21c1l2]LON: %value% endon on switch1#state==3 do sensor60 4 endon on switch1#state==2 do sensor60 6 endon

rule2  on tele-GPS#int>9 do DisplayText [f0c9l4]I%value%  endon  on tele-GPS#int<10 do DisplayText [f0c9l4]I0%value%  endon on tele-GPS#fil==1 do DisplayText [f0c18l4]F endon on tele-GPS#fil==0 do DisplayText [f0c18l4]N endon

rule3 on tele-FLOG#sec do DisplayText  [f0c1l4]SAV:%value% endon on tele-FLOG#rec==1 do DisplayText [f0c1l4]REC: endon on tele-FLOG#mode do DisplayText [f0c14l4]M%value% endon

\*********************************************************************************************/

#define XSNS_60        60

#include "NTPServer.h"
#include "NTPPacket.h"

#include "tasmota_xsns_sensor/gnss/gnss_state.h"
#include "tasmota_xsns_sensor/gnss/gnss_stream.h"
#include "tasmota_xsns_sensor/gnss/gnss_nmea.h"
#include "tasmota_xsns_sensor/gnss/gnss_ubx.h"
#include "tasmota_xsns_sensor/gnss/gnss_rtcm.h"
#include "tasmota_xsns_sensor/gnss/gnss_module.h"
#include "tasmota_xsns_sensor/gnss/gnss_json.h"

/*********************************************************************************************\
 * constants
\*********************************************************************************************/

#define D_CMND_UBX "UBX"

const char S_JSON_UBX_COMMAND_NVALUE[] PROGMEM = "{\"" D_CMND_UBX "%s\":%d}";

const char kUBXTypes[] PROGMEM = "UBX";

#define UBX_LAT_LON_THRESHOLD 100 // filter out some noise of local drift

#define UBX_SERIAL_BUFFER_SIZE 2048   // the LC29H sends ~1.2 kB per fix at 115200 baud
#define UBX_TCP_PORT           1234
#define NTP_MILLIS_OFFSET      50              // estimated latency in milliseconds

#define GNSS_CFG_FILE          "/gnss.cfg"
#define GNSS_CFG_MAGIC         0x47505331      // "GPS1"
#define GNSS_NTRIP_HOST        "10.1.10.1"     // the ten64 proxy, as the IoT VLAN reaches it
#define GNSS_NTRIP_PORT        2101
#define GNSS_SEARCH_MS         2500            // time at each speed while searching
#define GNSS_PROBE_MS          1500            // time to wait for an identification reply
#define GNSS_CMD_GAP_MS        150             // between configuration commands
#define GNSS_SILENT_MS         10000           // no valid frame this long: search again
#define GNSS_MAX_ANNOUNCED     128             // per-satellite entities remembered

static const uint32_t kGnssBauds[] = {9600, 38400, 115200};

/********************************************************************************************\
| *globals
\*********************************************************************************************/

struct UBX_t {
  int32_t lat;
  int32_t lon;

  struct entry_t {
    int32_t lat;               // raw sensor value
    int32_t lon;               // raw sensor value
    uint32_t time;             // local time from system (maybe provided by the sensor)
  };

  union {
    entry_t values;
    uint8_t bytes[sizeof(entry_t)];
  } rec_buffer;

  struct CFG_RATE {
    uint8_t cls;               // 0x06
    uint8_t id;                // 0x08
    uint16_t len;              // 6 bytes
    uint16_t measRate;         // in every ms -> 1 Hz = 1000 ms; 10 Hz = 100 ms -> x = 1000 ms / Hz
    uint16_t navRate;          //  x measurements for 1 navigation event
    uint16_t timeRef;          //  align to time system: 0= UTC, 1 = GPS, 2 = GLONASS, ...
  };

  struct {
    uint16_t log_interval;     // in tenth of seconds
    int32_t timeOffset;        // roughly computed offset millis() - iTOW
  } state;

  struct {
    uint32_t init:1;
    uint32_t filter_noise:1;
    uint32_t send_when_new:1;  // no teleinterval
    uint32_t send_UI_only:1;
    uint32_t runningNTP:1;
    uint32_t forceUTCupdate:1;
    uint32_t runningVPort:1;
  } mode;

  uint32_t utc_time;

  uint8_t TCPbuf[256];
  size_t TCPbufSize;
} UBX;

enum GnssPhase { GNSS_SEARCH, GNSS_PROBE, GNSS_CONFIGURE, GNSS_RUNNING };

struct GnssSettings {
  uint32_t magic;
  uint32_t baud;               // 0 = search
  uint16_t period;             // seconds between tele/<topic>/GNSS
  uint8_t sat_entities;
  uint8_t hass;
  uint8_t ntrip;               // 0 off, 1 ten64 proxy, 2 custom
  char host[64];
  uint16_t port;
  char mount[32];
  char user[32];
  char password[32];
};

struct GNSS_t {
  gnss_state_t st;
  gnss_stream_t stream;
  gnss_rtcm_t rtcm;
  gnss_dechunk_t dechunk;
  GnssSettings cfg;

  uint8_t phase;
  uint8_t baud_index;
  uint32_t baud;
  uint32_t phase_ms;           // when the current phase / step started
  uint8_t step;                // probe step or configuration index
  uint32_t last_frame_ms;
  bool got_frame;              // a valid frame since the phase started
  uint32_t last_epoch;

  // NTRIP
  WiFiClient *ntrip;
  uint8_t corr_state;
  char corr_mount[33];
  char corr_error[48];
  uint32_t next_try_ms;
  uint32_t backoff_s;
  bool header_done;
  char hdr[512];
  uint16_t hdr_len;
  uint8_t pending[16];         // the first stream bytes, while the de-chunker decides
  uint8_t pending_len;
  uint32_t last_rtcm_ms;       // last good RTCM 3 frame, or any byte for RTCM 2
  uint32_t last_frames;

  // PPS
  int pps_pin;

  // publishing
  uint32_t last_pub_ms;
  uint32_t last_sats_ms;
  int hass_index;              // next discovery entity to publish; -1 when done
  uint8_t hass_model_published;
  uint16_t announced[GNSS_MAX_ANNOUNCED];   // per-satellite entities published: gnss << 8 | svid
  uint8_t n_announced;
} *Gnss = nullptr;

static volatile uint32_t gnss_pps_count = 0;
static volatile uint32_t gnss_pps_ms = 0;

#ifdef USE_FLOG
FLOG *Flog = nullptr;
#endif  // USE_FLOG
TasmotaSerial *UBXSerial;

NtpServer timeServer(PortUdp);

WiFiServer vPortServer(UBX_TCP_PORT);
WiFiClient vPortClient;

/*********************************************************************************************\
 * settings
\*********************************************************************************************/

void GnssDefaults(void) {
  memset(&Gnss->cfg, 0, sizeof(Gnss->cfg));
  Gnss->cfg.magic = GNSS_CFG_MAGIC;
  Gnss->cfg.period = 10;
  Gnss->cfg.hass = 1;
  Gnss->cfg.ntrip = 1;
  strlcpy(Gnss->cfg.host, GNSS_NTRIP_HOST, sizeof(Gnss->cfg.host));
  Gnss->cfg.port = GNSS_NTRIP_PORT;
}

void GnssLoadSettings(void) {
  GnssDefaults();
#ifdef USE_UFILESYS
  GnssSettings loaded;
  if (TfsLoadFile(GNSS_CFG_FILE, (uint8_t*)&loaded, sizeof(loaded)) && loaded.magic == GNSS_CFG_MAGIC) {
    Gnss->cfg = loaded;
    Gnss->cfg.host[sizeof(Gnss->cfg.host) - 1] = 0;
    Gnss->cfg.mount[sizeof(Gnss->cfg.mount) - 1] = 0;
    Gnss->cfg.user[sizeof(Gnss->cfg.user) - 1] = 0;
    Gnss->cfg.password[sizeof(Gnss->cfg.password) - 1] = 0;
    if (Gnss->cfg.period < 1) { Gnss->cfg.period = 10; }
  }
#endif  // USE_UFILESYS
}

void GnssSaveSettings(void) {
#ifdef USE_UFILESYS
  TfsSaveFile(GNSS_CFG_FILE, (const uint8_t*)&Gnss->cfg, sizeof(Gnss->cfg));
#endif  // USE_UFILESYS
}

/*********************************************************************************************\
 * receiver: search, identify, configure
\*********************************************************************************************/

void GnssWrite(const uint8_t *data, size_t len) {
  UBXSerial->write(data, len);
}

void GnssSetBaud(uint32_t baud) {
  UBXSerial->flush();
  UBXSerial->begin(baud);
  Gnss->baud = baud;
  AddLog(LOG_LEVEL_DEBUG, PSTR("GPS: UART at %u baud"), baud);
}

void GnssStartSearch(void) {
  gnss_stream_init(&Gnss->stream);
  Gnss->st.module = GNSS_MODULE_UNKNOWN;
  Gnss->st.model[0] = 0;
  Gnss->phase = GNSS_SEARCH;
  Gnss->phase_ms = millis();
  Gnss->got_frame = false;
  if (Gnss->cfg.baud) {
    GnssSetBaud(Gnss->cfg.baud);
  } else {
    GnssSetBaud(kGnssBauds[Gnss->baud_index % (sizeof(kGnssBauds) / sizeof(kGnssBauds[0]))]);
  }
}

void GnssSendProbe(void) {
  uint8_t out[32];
  size_t n = gnss_module_probe(Gnss->step, out, sizeof(out));
  if (n) { GnssWrite(out, n); }
  Gnss->phase_ms = millis();
}

void GnssIdentified(void) {
  AddLog(LOG_LEVEL_INFO, PSTR("GPS: %s (%s, %s) at %u baud"), Gnss->st.model, Gnss->st.sw_version,
         Gnss->st.hw_version, Gnss->baud);
  Gnss->phase = GNSS_CONFIGURE;
  Gnss->step = 0;
  Gnss->phase_ms = millis() - GNSS_CMD_GAP_MS;
  Gnss->hass_index = 0;         // republish discovery: the device model is now known
}

void GnssPhaseStep(void) {
  uint32_t now = millis();
  switch (Gnss->phase) {
    case GNSS_SEARCH:
      if (Gnss->got_frame) {
        Gnss->phase = GNSS_PROBE;
        Gnss->step = 0;
        GnssSendProbe();
      } else if (now - Gnss->phase_ms > GNSS_SEARCH_MS) {
        Gnss->baud_index++;
        GnssStartSearch();
      }
      break;
    case GNSS_PROBE:
      if (Gnss->st.module != GNSS_MODULE_UNKNOWN) {
        GnssIdentified();
      } else if (now - Gnss->phase_ms > GNSS_PROBE_MS) {
        Gnss->step++;
        uint8_t out[32];
        if (gnss_module_probe(Gnss->step, out, sizeof(out))) {
          GnssSendProbe();
        } else {
          AddLog(LOG_LEVEL_INFO, PSTR("GPS: receiver not identified; reading its NMEA as it is"));
          Gnss->phase = GNSS_RUNNING;
        }
      }
      break;
    case GNSS_CONFIGURE:
      if (now - Gnss->phase_ms >= GNSS_CMD_GAP_MS) {
        uint8_t out[256];
        uint32_t switch_baud = 0;
        size_t n = gnss_module_config(Gnss->st.module, Gnss->step, out, sizeof(out), &switch_baud);
        if (!n) {
          Gnss->phase = GNSS_RUNNING;
          AddLog(LOG_LEVEL_INFO, PSTR("GPS: configured"));
          break;
        }
        GnssWrite(out, n);
        if (switch_baud && switch_baud != Gnss->baud && !Gnss->cfg.baud) {
          UBXSerial->flush();
          delay(20);              // let the receiver act on the last byte
          GnssSetBaud(switch_baud);
        }
        Gnss->step++;
        Gnss->phase_ms = now;
      }
      break;
    case GNSS_RUNNING:
      if (now - Gnss->last_frame_ms > GNSS_SILENT_MS) {
        AddLog(LOG_LEVEL_INFO, PSTR("GPS: receiver silent, searching again"));
        GnssStartSearch();
      }
      break;
  }
}

/*********************************************************************************************\
 * PPS
\*********************************************************************************************/

void IRAM_ATTR GnssPpsIsr(void) {
  gnss_pps_count++;
  gnss_pps_ms = millis();
}

bool GnssPpsPresent(void) {
  return gnss_pps_count && (millis() - gnss_pps_ms < 2000);
}

/*********************************************************************************************\
 * NTRIP corrections
\*********************************************************************************************/

void GnssNtripStop(uint8_t state, const char *error) {
  if (Gnss->ntrip) {
    Gnss->ntrip->stop();
    delete Gnss->ntrip;
    Gnss->ntrip = nullptr;
  }
  Gnss->corr_state = state;
  strlcpy(Gnss->corr_error, error, sizeof(Gnss->corr_error));
  if (state == GNSS_CORR_RETRYING) {
    Gnss->backoff_s = Gnss->backoff_s ? Gnss->backoff_s * 2 : 5;
    if (Gnss->backoff_s > 300) { Gnss->backoff_s = 300; }
    Gnss->next_try_ms = millis() + Gnss->backoff_s * 1000;
    AddLog(LOG_LEVEL_INFO, PSTR("GPS: corrections: %s, retry in %u s"), error, Gnss->backoff_s);
  }
}

const char *GnssNtripMount(void) {
  if (Gnss->cfg.ntrip == 2) { return Gnss->cfg.mount; }
  return gnss_module_mount(Gnss->st.module);
}

void GnssNtripConnect(void) {
  const char *mount = GnssNtripMount();
  if (!*mount) { return; }        // receiver not identified yet: no mountpoint to ask for
  strlcpy(Gnss->corr_mount, mount, sizeof(Gnss->corr_mount));
  Gnss->corr_state = GNSS_CORR_CONNECTING;
  Gnss->ntrip = new WiFiClient();
  if (!Gnss->ntrip->connect(Gnss->cfg.host, Gnss->cfg.port, 2000)) {
    GnssNtripStop(GNSS_CORR_RETRYING, "cannot connect");
    return;
  }
  char req[384];
  size_t n = gnss_ntrip_request(req, sizeof(req), Gnss->cfg.host, Gnss->cfg.port, mount,
                                Gnss->cfg.ntrip == 2 ? Gnss->cfg.user : nullptr,
                                Gnss->cfg.ntrip == 2 ? Gnss->cfg.password : nullptr);
  if (!n) {
    GnssNtripStop(GNSS_CORR_RETRYING, "request too long");
    return;
  }
  Gnss->ntrip->write((const uint8_t*)req, n);
  Gnss->header_done = false;
  Gnss->hdr_len = 0;
  Gnss->pending_len = 0;
  Gnss->next_try_ms = millis() + 10000;   // the caster has 10 s to answer
  AddLog(LOG_LEVEL_INFO, PSTR("GPS: corrections: requesting %s:%u/%s"), Gnss->cfg.host, Gnss->cfg.port, mount);
}

/* Correction bytes, header and chunk framing removed: to the receiver. */
void GnssCorrections(uint8_t *data, size_t len) {
  if (!len) { return; }
  uint32_t frames = gnss_rtcm_feed(&Gnss->rtcm, data, len);
  // RTCM 3 is alive when frames check out; a stream that has never framed (RTCM 2.3) on any data.
  if (frames || Gnss->rtcm.frames == 0) { Gnss->last_rtcm_ms = millis(); }
  if (Gnss->phase == GNSS_RUNNING) { GnssWrite(data, len); }
}

/* Stream bytes after the header: hold the first few until the de-chunker can decide. */
void GnssStreamBytes(uint8_t *data, size_t len) {
  if (!Gnss->dechunk.decided) {
    size_t room = sizeof(Gnss->pending) - Gnss->pending_len;
    size_t take = len < room ? len : room;
    memcpy(Gnss->pending + Gnss->pending_len, data, take);
    Gnss->pending_len += take;
    data += take;
    len -= take;
    size_t n = gnss_dechunk(&Gnss->dechunk, Gnss->pending, Gnss->pending_len);
    if (!Gnss->dechunk.decided) { return; }
    if (Gnss->dechunk.active) {
      AddLog(LOG_LEVEL_DEBUG, PSTR("GPS: corrections: stripping HTTP chunk framing"));
    }
    GnssCorrections(Gnss->pending, n);
    Gnss->pending_len = 0;
  }
  GnssCorrections(data, gnss_dechunk(&Gnss->dechunk, data, len));
}

void GnssNtripLoop(void) {
  if (!Gnss->ntrip) { return; }
  if (!Gnss->ntrip->connected() && !Gnss->ntrip->available()) {
    GnssNtripStop(GNSS_CORR_RETRYING, "disconnected");
    return;
  }
  uint8_t buf[512];
  int avail = Gnss->ntrip->available();
  if (avail <= 0) {
    if (!Gnss->header_done && TimeReached(Gnss->next_try_ms)) {
      GnssNtripStop(GNSS_CORR_RETRYING, "no reply");
    }
    return;
  }
  int n = Gnss->ntrip->read(buf, avail < (int)sizeof(buf) ? avail : sizeof(buf));
  if (n <= 0) { return; }
  if (Gnss->header_done) {
    GnssStreamBytes(buf, n);
    return;
  }
  size_t room = sizeof(Gnss->hdr) - Gnss->hdr_len;
  size_t take = (size_t)n < room ? n : room;
  memcpy(Gnss->hdr + Gnss->hdr_len, buf, take);
  Gnss->hdr_len += take;
  size_t header_len;
  bool chunked;
  switch (gnss_ntrip_reply(Gnss->hdr, Gnss->hdr_len, &header_len, &chunked)) {
    case GNSS_NTRIP_INCOMPLETE:
      if (Gnss->hdr_len == sizeof(Gnss->hdr)) { GnssNtripStop(GNSS_CORR_RETRYING, "reply header too long"); }
      return;
    case GNSS_NTRIP_SOURCETABLE:
      GnssNtripStop(GNSS_CORR_RETRYING, "mountpoint unknown or down");
      return;
    case GNSS_NTRIP_UNAUTHORIZED:
      GnssNtripStop(GNSS_CORR_RETRYING, "login refused");
      return;
    case GNSS_NTRIP_ERROR:
      GnssNtripStop(GNSS_CORR_RETRYING, "caster error");
      return;
    case GNSS_NTRIP_OK:
      break;
  }
  Gnss->header_done = true;
  Gnss->corr_state = GNSS_CORR_CONNECTED;
  Gnss->corr_error[0] = 0;
  Gnss->backoff_s = 0;
  gnss_dechunk_init(&Gnss->dechunk, chunked);
  AddLog(LOG_LEVEL_INFO, PSTR("GPS: corrections: streaming %s"), Gnss->corr_mount);
  // whatever followed the header, in this read and in the header buffer
  size_t after = Gnss->hdr_len - header_len;
  uint8_t rest[sizeof(Gnss->hdr)];
  memcpy(rest, Gnss->hdr + header_len, after);
  GnssStreamBytes(rest, after);
  if (take < (size_t)n) { GnssStreamBytes(buf + take, n - take); }
}

void GnssNtripEverySecond(void) {
  bool wanted = Gnss->cfg.ntrip != 0 && Gnss->phase == GNSS_RUNNING && *GnssNtripMount() &&
                !TasmotaGlobal.global_state.network_down;
  if (!wanted) {
    if (Gnss->ntrip || Gnss->corr_state != GNSS_CORR_DISABLED) {
      GnssNtripStop(GNSS_CORR_DISABLED, "");
    }
    return;
  }
  if (!Gnss->ntrip && (Gnss->corr_state != GNSS_CORR_RETRYING || TimeReached(Gnss->next_try_ms))) {
    GnssNtripConnect();
  }
  // A stream that stops delivering corrections is reconnected.
  if (Gnss->ntrip && Gnss->header_done && Gnss->last_rtcm_ms && millis() - Gnss->last_rtcm_ms > 60000) {
    GnssNtripStop(GNSS_CORR_RETRYING, "no corrections for 60 s");
  }
}

/*********************************************************************************************\
 * frames from the receiver
\*********************************************************************************************/

/* The Arduino build declares every .ino function at the top of the merged
 * file, before the gnss_*.h types exist, so functions here take only types
 * declared that early: an integer frame type, void pointers for structs. */
void GnssFrame(uint32_t t) {
  switch (t) {
    case GNSS_FRAME_NMEA:
      gnss_nmea_parse(&Gnss->st, (const char*)Gnss->stream.buf);
      break;
    case GNSS_FRAME_UBX:
      gnss_ubx_parse(&Gnss->st, Gnss->stream.buf[0], Gnss->stream.buf[1], Gnss->stream.buf + 4, Gnss->stream.ubx_len);
      break;
    case GNSS_FRAME_BAD_NMEA:
      Gnss->st.nmea_bad++;
      return;
    case GNSS_FRAME_BAD_UBX:
      Gnss->st.ubx_bad++;
      return;
    default:
      return;
  }
  Gnss->last_frame_ms = millis();
  Gnss->got_frame = true;
}

void UBXSelectMode(uint16_t mode);

/* Once per navigation epoch: the legacy features fed from the new state. */
void GnssNewEpoch(void) {
  gnss_state_t *s = &Gnss->st;
  UBX.state.timeOffset = millis();      // the NTP server's reference for this second
  if (s->position_valid && s->fix_type >= GNSS_FIX_2D) {
    bool moved = true;
    if (UBX.mode.filter_noise) {
      moved = abs(s->lat_e7 - UBX.rec_buffer.values.lat) >= UBX_LAT_LON_THRESHOLD &&
              abs(s->lon_e7 - UBX.rec_buffer.values.lon) >= UBX_LAT_LON_THRESHOLD;
    }
    if (moved) {
      UBX.rec_buffer.values.lat = s->lat_e7;
      UBX.rec_buffer.values.lon = s->lon_e7;
      if (UBX.mode.send_when_new) { MqttPublishTeleperiodSensor(); }
    }
  }
  if (s->time_valid && s->date_valid && s->year >= 2023) {
    bool resync = (Rtc.utc_time > UBX.utc_time);  // Sync local time every hour
    if (Rtc.user_time_entry == false || UBX.mode.forceUTCupdate || UBX.mode.runningNTP || resync) {
      TIME_T gpsTime;
      gpsTime.year = s->year - 1970;
      gpsTime.month = s->month;
      gpsTime.day_of_month = s->day;
      gpsTime.hour = s->hour;
      gpsTime.minute = s->minute;
      gpsTime.second = s->second;
      UBX.rec_buffer.values.time = MakeTime(gpsTime);
      if (UBX.mode.forceUTCupdate || (Rtc.user_time_entry == false) || resync) {
        UBX.utc_time = UBX.rec_buffer.values.time + 3600;
        Rtc.utc_time = UBX.rec_buffer.values.time;
        RtcSync("GPS");
      }
      Rtc.user_time_entry = true;
    }
  }
}

/* Called from FUNC_LOOP: drain the UART as fast as it fills. */
void GnssLoop(void) {
  uint32_t n = 0;
  while (UBXSerial->available() && n < 1024) {
    uint8_t c = UBXSerial->read();
    if (UBX.mode.runningVPort && UBX.TCPbufSize < sizeof(UBX.TCPbuf)) {
      UBX.TCPbuf[UBX.TCPbufSize++] = c;   // the virtual serial port gets every byte
    }
    gnss_frame_t t = gnss_stream_byte(&Gnss->stream, c);
    if (t != GNSS_FRAME_NONE) { GnssFrame(t); }
    n++;
  }
  if (Gnss->st.epoch != Gnss->last_epoch) {
    Gnss->last_epoch = Gnss->st.epoch;
    GnssNewEpoch();
  }
  GnssPhaseStep();
  GnssNtripLoop();
}

/*********************************************************************************************\
 * MQTT and Home Assistant
\*********************************************************************************************/

void GnssExtra(void *xp) {
  gnss_extra_t *x = (gnss_extra_t*)xp;
  memset(x, 0, sizeof(*x));
  x->corr_state = Gnss->corr_state;
  x->corr_mount = Gnss->corr_state == GNSS_CORR_DISABLED ? "" : Gnss->corr_mount;
  x->corr_error = Gnss->corr_error;
  x->rtcm = &Gnss->rtcm;
  x->corr_age_s = Gnss->last_rtcm_ms ? (int32_t)((millis() - Gnss->last_rtcm_ms) / 1000) : -1;
  x->baud = Gnss->baud;
  x->pps_present = GnssPpsPresent();
  x->pps_count = gnss_pps_count;
  x->data_age_s = Gnss->last_frame_ms ? (int32_t)((millis() - Gnss->last_frame_ms) / 1000) : -1;
}

void GnssPublishState(void) {
  char *buf = (char*)malloc(2048);
  if (!buf) { return; }
  gnss_extra_t x;
  GnssExtra(&x);
  if (gnss_json_state(buf, 2048, &Gnss->st, &x)) {
    MqttPublishPayloadPrefixTopic_P(TELE, PSTR("GNSS"), buf);
  }
  free(buf);
}

void GnssPublishSats(void) {
  char *buf = (char*)malloc(4096);
  if (!buf) { return; }
  if (gnss_json_sats(buf, 4096, &Gnss->st)) {
    MqttPublishPayloadPrefixTopic_P(TELE, PSTR("GNSS_SATS"), buf);
  }
  free(buf);
}

struct GnssDevice {
  char uid[24];
  char topic[TOPSZ];
  char lwt[TOPSZ];
  char sw[48];
  char url[32];
  gnss_hass_device_t dev;
};

void GnssDeviceInfo(void *dp) {
  GnssDevice *d = (GnssDevice*)dp;
  snprintf_P(d->uid, sizeof(d->uid), PSTR("gps_%s"), NetworkUniqueId().c_str());
  GetTopic_P(d->topic, TELE, TasmotaGlobal.mqtt_topic, "");
  GetTopic_P(d->lwt, TELE, TasmotaGlobal.mqtt_topic, S_LWT);
  snprintf_P(d->sw, sizeof(d->sw), PSTR("%s (%s)"), TasmotaGlobal.version, TasmotaGlobal.image_name);
  snprintf_P(d->url, sizeof(d->url), PSTR("http://%s/"), NetworkAddress().toString().c_str());
  d->dev.uid = d->uid;
  d->dev.name = SettingsText(SET_DEVICENAME);
  d->dev.topic = d->topic;
  d->dev.lwt = d->lwt;
  d->dev.model = Gnss->st.model;
  d->dev.sw = d->sw;
  d->dev.url = d->url;
}

/* A few discovery messages at a time, so MQTT is never flooded. */
void GnssHassStep(void) {
  if (!Gnss->cfg.hass || Gnss->hass_index < 0 || !MqttIsConnected()) { return; }
  GnssDevice *d = (GnssDevice*)malloc(sizeof(GnssDevice));
  char *payload = (char*)malloc(1024);
  if (d && payload) {
    GnssDeviceInfo(d);
    char topic[160];
    for (int i = 0; i < 3 && Gnss->hass_index >= 0; i++) {
      if (gnss_hass_config(Gnss->hass_index, &d->dev, topic, sizeof(topic), payload, 1024)) {
        MqttPublishPayload(topic, payload, 0, true);
      }
      if (++Gnss->hass_index >= gnss_hass_count()) { Gnss->hass_index = -1; }
    }
  }
  free(payload);
  free(d);
}

int GnssAnnounced(uint16_t key) {
  for (uint8_t i = 0; i < Gnss->n_announced; i++) {
    if (Gnss->announced[i] == key) { return i; }
  }
  return -1;
}

/* Per-satellite entities: announce new satellites, a few per second; remove them all when switched off. */
void GnssSatEntitiesStep(void) {
  if (!MqttIsConnected()) { return; }
  bool want = Gnss->cfg.hass && Gnss->cfg.sat_entities;
  if (!want && !Gnss->n_announced) { return; }
  GnssDevice *d = (GnssDevice*)malloc(sizeof(GnssDevice));
  char *payload = (char*)malloc(1024);
  if (d && payload) {
    GnssDeviceInfo(d);
    char topic[160];
    uint8_t sent = 0;
    if (!want) {
      while (Gnss->n_announced && sent < 4) {
        uint16_t key = Gnss->announced[--Gnss->n_announced];
        gnss_hass_sat_config(key >> 8, key & 0xff, true, &d->dev, topic, sizeof(topic), payload, 1024);
        MqttPublishPayload(topic, "", 0, true);
        sent++;
      }
    } else {
      for (uint8_t i = 0; i < Gnss->st.n_sats && sent < 4 && Gnss->n_announced < GNSS_MAX_ANNOUNCED; i++) {
        uint16_t key = Gnss->st.sats[i].gnss << 8 | Gnss->st.sats[i].svid;
        if (GnssAnnounced(key) >= 0) { continue; }
        if (gnss_hass_sat_config(key >> 8, key & 0xff, false, &d->dev, topic, sizeof(topic), payload, 1024)) {
          MqttPublishPayload(topic, payload, 0, true);
          Gnss->announced[Gnss->n_announced++] = key;
          sent++;
        }
      }
    }
  }
  free(payload);
  free(d);
}

void GnssEverySecond(void) {
  GnssNtripEverySecond();
  if (!MqttIsConnected()) { return; }
  uint32_t now = millis();
  if (now - Gnss->last_pub_ms >= (uint32_t)Gnss->cfg.period * 1000) {
    Gnss->last_pub_ms = now;
    GnssPublishState();
  }
  if (now - Gnss->last_sats_ms >= 30000) {
    Gnss->last_sats_ms = now;
    GnssPublishSats();
  }
  GnssSatEntitiesStep();
}

/*********************************************************************************************\
 * Gps* commands
\*********************************************************************************************/

const char kGnssCommands[] PROGMEM = "Gps|"
  "Baud|Ntrip|Period|SatEntities|Hass|Reinit|Status";

void (* const GnssCommand[])(void) PROGMEM = {
  &CmndGpsBaud, &CmndGpsNtrip, &CmndGpsPeriod, &CmndGpsSatEntities, &CmndGpsHass, &CmndGpsReinit, &CmndGpsStatus };

void CmndGpsBaud(void) {
  if (XdrvMailbox.data_len > 0) {
    uint32_t baud = XdrvMailbox.payload;
    if (baud == 0 || baud == 9600 || baud == 19200 || baud == 38400 || baud == 57600 || baud == 115200 || baud == 230400) {
      Gnss->cfg.baud = baud;
      GnssSaveSettings();
      GnssStartSearch();
    }
  }
  ResponseCmndNumber(Gnss->cfg.baud);
}

void CmndGpsNtrip(void) {
  if (XdrvMailbox.data_len > 0) {
    if (XdrvMailbox.data_len == 1 && (XdrvMailbox.payload == 0 || XdrvMailbox.payload == 1)) {
      Gnss->cfg.ntrip = XdrvMailbox.payload;
      strlcpy(Gnss->cfg.host, GNSS_NTRIP_HOST, sizeof(Gnss->cfg.host));
      Gnss->cfg.port = GNSS_NTRIP_PORT;
      Gnss->cfg.mount[0] = Gnss->cfg.user[0] = Gnss->cfg.password[0] = 0;
    } else {
      // <host>:<port>/<mount>[ <user> <password>]
      char arg[160];
      strlcpy(arg, XdrvMailbox.data, sizeof(arg));
      char *user = strchr(arg, ' ');
      char *password = nullptr;
      if (user) { *user++ = 0; password = strchr(user, ' '); if (password) { *password++ = 0; } }
      char *mount = strchr(arg, '/');
      char *port = strchr(arg, ':');
      if (!mount || !port || port > mount) {
        ResponseCmndChar_P(PSTR("Use <host>:<port>/<mount>[ <user> <password>]"));
        return;
      }
      *mount++ = 0;
      *port++ = 0;
      Gnss->cfg.ntrip = 2;
      strlcpy(Gnss->cfg.host, arg, sizeof(Gnss->cfg.host));
      Gnss->cfg.port = atoi(port);
      strlcpy(Gnss->cfg.mount, mount, sizeof(Gnss->cfg.mount));
      strlcpy(Gnss->cfg.user, user ? user : "", sizeof(Gnss->cfg.user));
      strlcpy(Gnss->cfg.password, password ? password : "", sizeof(Gnss->cfg.password));
    }
    GnssSaveSettings();
    GnssNtripStop(GNSS_CORR_DISABLED, "");   // reconnects next second with the new settings
  }
  if (Gnss->cfg.ntrip == 0) {
    ResponseCmndChar_P(PSTR("0"));
  } else {
    char out[128];
    snprintf_P(out, sizeof(out), PSTR("%s:%u/%s"), Gnss->cfg.host, Gnss->cfg.port, GnssNtripMount());
    ResponseCmndChar(out);
  }
}

void CmndGpsPeriod(void) {
  if (XdrvMailbox.data_len > 0 && XdrvMailbox.payload >= 1 && XdrvMailbox.payload <= 3600) {
    Gnss->cfg.period = XdrvMailbox.payload;
    GnssSaveSettings();
  }
  ResponseCmndNumber(Gnss->cfg.period);
}

void CmndGpsSatEntities(void) {
  if (XdrvMailbox.data_len > 0 && XdrvMailbox.payload <= 1) {
    Gnss->cfg.sat_entities = XdrvMailbox.payload;
    GnssSaveSettings();
  }
  ResponseCmndStateText(Gnss->cfg.sat_entities);
}

void CmndGpsHass(void) {
  if (XdrvMailbox.data_len > 0 && XdrvMailbox.payload <= 1) {
    Gnss->cfg.hass = XdrvMailbox.payload;
    GnssSaveSettings();
    Gnss->hass_index = 0;
  }
  ResponseCmndStateText(Gnss->cfg.hass);
}

void CmndGpsReinit(void) {
  GnssStartSearch();
  ResponseCmndDone();
}

void CmndGpsStatus(void) {
  char *buf = (char*)malloc(2048);
  if (!buf) { return; }
  gnss_extra_t x;
  GnssExtra(&x);
  if (gnss_json_state(buf, 2048, &Gnss->st, &x)) {
    Response_P(PSTR("{\"%s\":%s}"), XdrvMailbox.command, buf);
  }
  free(buf);
}

/*********************************************************************************************\
 * set-up
\*********************************************************************************************/

void UBXDetect(void) {
  UBX.mode.init = 0;
  if (!(PinUsed(GPIO_GPS_RX, GPIO_ANY) && PinUsed(GPIO_GPS_TX))) { return; }

  Gnss = new GNSS_t();
  if (!Gnss) { return; }
  memset(Gnss, 0, sizeof(GNSS_t));
  gnss_state_init(&Gnss->st);
  gnss_rtcm_init(&Gnss->rtcm);
  Gnss->hass_index = 0;
  Gnss->pps_pin = -1;
  GnssLoadSettings();

  UBXSerial = new TasmotaSerial(Pin(GPIO_GPS_RX, GPIO_ANY), Pin(GPIO_GPS_TX), 1, 0, UBX_SERIAL_BUFFER_SIZE);
  if (!UBXSerial->begin(Gnss->cfg.baud ? Gnss->cfg.baud : kGnssBauds[0])) { return; }

  if (UBXSerial->hardwareSerial()) {
    ClaimSerial();
  }
#ifdef ESP32
  AddLog(LOG_LEVEL_DEBUG, PSTR("GPS: Serial UART%d"), UBXSerial->getUart());
#endif

  if (PinUsed(GPIO_GPS_PPS)) {
    Gnss->pps_pin = Pin(GPIO_GPS_PPS);
    pinMode(Gnss->pps_pin, INPUT);
    attachInterrupt(Gnss->pps_pin, GnssPpsIsr, RISING);
  }

  GnssStartSearch();
  UBX.mode.init = 1;

#ifdef USE_FLOG
  if (!Flog) {
    Flog = new FLOG;            // init Flash Log
    Flog->init();
  }
#endif  // USE_FLOG

  UBX.state.log_interval = 10;  // 1 second
  UBX.mode.send_UI_only = true; // send UI data ...
}

/********************************************************************************************\
| * callback functions for the download
\*********************************************************************************************/

#ifdef USE_FLOG
void UBXsendHeader(void)
{
  Webserver->setContentLength(CONTENT_LENGTH_UNKNOWN);
  Webserver->sendHeader(F("Content-Disposition"), F("attachment; filename=TASMOTA.gpx"));
  WSSend(200, CT_APP_STREAM, F(
    "<?xml version=\"1.0\" encoding=\"UTF-8\" standalone=\"no\" ?>\r\n"
    "<GPX version=\"1.1\" creator=\"TASMOTA\" xmlns=\"http://www.topografix.com/GPX/1/1\" \r\n"
    "xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\"\r\n"
    "xsi:schemaLocation=\"http://www.topografix.com/GPX/1/1 http://www.topografix.com/GPX/1/1/gpx.xsd\">\r\n"
    "<trk>\r\n<trkseg>\r\n"));
}

void UBXsendRecord(uint8_t *buf)
{
	char record[100];
	char stime[32];
	UBX_t::entry_t *entry = (UBX_t::entry_t*)buf;
	snprintf_P(stime, sizeof(stime), GetDT(entry->time).c_str());
	char lat[FLOATSZ];
	char lon[FLOATSZ];
	dtostrfd((double)entry->lat/10000000.0f,7,lat);
	dtostrfd((double)entry->lon/10000000.0f,7,lon);
	snprintf_P(record, sizeof(record),PSTR("<trkpt\n\t lat=\"%s\" lon=\"%s\">\n\t<time>%s</time>\n</trkpt>\n"),lat ,lon, stime);
	Webserver->sendContent_P(record);
}

void UBXsendFooter(void)
{
  Webserver->sendContent(F("</trkseg>\n</trk>\n</GPX>"));
  Webserver->sendContent("");
  Rtc.user_time_entry = false; // we have blocked the main loop and want a new valid time
}

/********************************************************************************************/

void UBXsendFile(void)
{
  if (!HttpCheckPriviledgedAccess()) { return; }
  Flog->startDownload(sizeof(UBX.rec_buffer),UBXsendHeader,UBXsendRecord,UBXsendFooter);
}
#endif  // USE_FLOG

/********************************************************************************************/

/* Legacy UBX CFG-RATE (u-blox 7, M8): measurement interval in seconds. */
void UBXSetRate(uint16_t interval)
{
  uint32_t measRate = (1000*(uint32_t)interval); //seconds to milliseconds
  if (measRate > 0xffff) {
    measRate = 0xffff; // max. 65535 ms interval
  }
  uint8_t payload[6] = {(uint8_t)measRate, (uint8_t)(measRate >> 8), 1, 0, 1, 0};  // navRate 1, timeRef GPS
  uint8_t out[16];
  size_t n = gnss_ubx_frame(out, sizeof(out), 0x06, 0x08, payload, sizeof(payload));
  GnssWrite(out, n);
  DEBUG_SENSOR_LOG(PSTR("UBX: requested interval: %u seconds measRate: %u ms"), interval, measRate);
  UBX.state.log_interval = 10*interval;
}

void UBXSelectMode(uint16_t mode)
{
  DEBUG_SENSOR_LOG(PSTR("UBX: set mode to %u"),mode);
  switch(mode){
#ifdef USE_FLOG
    case 0:
      Flog->mode = 0; // write once to all available sectors, then stop
      break;
    case 1:
      Flog->mode = 1; // write to all available sectors, then restart and overwrite the older ones
      break;
    case 2:
      UBX.mode.filter_noise = true;  // filter out horizontal drift noise, TODO: find useful values
      break;
    case 3:
      UBX.mode.filter_noise = false;
      break;
    case 4:
      Flog->startRecording(true);
      AddLog(LOG_LEVEL_INFO, PSTR("UBX: start recording - appending"));
      break;
    case 5:
      Flog->startRecording(false);
      AddLog(LOG_LEVEL_INFO, PSTR("UBX: start recording - new log"));
      break;
    case 6:
      if(Flog->recording == true){
        Flog->stopRecording();
      }
      AddLog(LOG_LEVEL_INFO, PSTR("UBX: stop recording"));
      break;
#endif  // USE_FLOG
    case 7:
      UBX.mode.send_when_new = 1; // send mqtt on new postion + TELE -> consider to set TELE to a very high value
      break;
    case 8:
      UBX.mode.send_when_new = 0; // only TELE
      break;
    case 9:
      if (!TasmotaGlobal.global_state.network_down && timeServer.beginListening()) {
        UBX.mode.runningNTP = true;
      }
      break;
    case 10:
      UBX.mode.runningNTP = false;
      break;
    case 11:
      UBX.mode.forceUTCupdate = true;
      break;
    case 12:
      UBX.mode.forceUTCupdate = false;
      break;
    case 13:
      Settings->latitude = UBX.rec_buffer.values.lat/10;
      Settings->longitude = UBX.rec_buffer.values.lon/10;
      break;
    case 14:
      vPortServer.begin();
      UBX.mode.runningVPort = 1;
      break;
    case 15:
      // vPortServer.stop(); // seems not to work reliably
      UBX.mode.runningVPort = 0;
      break;
    default:
      if (mode>1000 && mode <1066) {
        UBXSetRate(mode-1000); // set interval between measurements in seconds from 1 to 65
      }
      break;
  }
  UBX.mode.send_UI_only = true;
  MqttPublishTeleperiodSensor();
}

/********************************************************************************************/

void UBXLoop50msec(void)
{
  // handle virtual serial port
  if (UBX.mode.runningVPort){
    if(!vPortClient.connected()) {
      vPortClient = vPortServer.available();
    }
    while(vPortClient.available()) {
      byte _newByte = vPortClient.read();
      UBXSerial->write(_newByte);
    }

    if (UBX.TCPbufSize!=0){
      vPortClient.write((char*)UBX.TCPbuf, UBX.TCPbufSize);
      UBX.TCPbufSize = 0;
    }
  }
  // handle NTP-server
  if(!TasmotaGlobal.global_state.network_down && UBX.mode.runningNTP){
    timeServer.processOneRequest(UBX.rec_buffer.values.time, UBX.state.timeOffset - NTP_MILLIS_OFFSET);
  }
}

void UBXLoop(void)
{
  static uint16_t counter; //count up every 100 msec

#ifdef USE_FLOG
  if (counter>UBX.state.log_interval) {
    if (Flog->recording && Gnss->st.position_valid) {
      UBX.rec_buffer.values.time = Rtc.local_time;
      Flog->addToBuffer(UBX.rec_buffer.bytes, sizeof(UBX.rec_buffer.bytes));
      counter = 0;
    }
  }
#endif  // USE_FLOG

  counter++;
}

/********************************************************************************************/
// normaly in i18n.h

#ifdef USE_WEBSERVER
// {s} = <tr><th>, {m} = </th><td>, {e} = </td></tr>

#ifdef USE_FLOG
#ifdef DEBUG_TASMOTA_SENSOR
const char HTTP_SNS_FLOGVER[] PROGMEM = "{s}FLOG with %u sectors:{m}%u bytes{e}"
                                        "{s}FLOG next sector for REC:{m} %u {e}"
                                        "{s}%u sector(s) with data at sector:{m}%u{e}";
const char HTTP_SNS_FLOGREC[] PROGMEM = "{s}RECORDING (bytes in buffer){m}%u{e}";
#endif  // DEBUG_TASMOTA_SENSOR

const char HTTP_SNS_FLOG[] PROGMEM = "{s}GPS Logging{m}%s{e}";
const char kFLOGstate[] PROGMEM = "Ready|Recording";
const char HTTP_BTN_FLOG_DL[] PROGMEM = "<button><a href='/UBX'>Download GPX-File</a></button>";
#endif  // USE_FLOG

const char HTTP_SNS_NTPSERVER[] PROGMEM = "{s}GPS NTP server{m}Active{e}";

const char HTTP_SNS_GPS[] PROGMEM = "{s}GPS " D_SAT_FIX "{m}%s{e}"
                                    "{s}GPS " D_LATITUDE "{m}%s{e}"
                                    "{s}GPS " D_LONGITUDE "{m}%s{e}"
                                    "{s}GPS " D_HORIZONTAL_ACCURACY "{m}%3_f " D_UNIT_METER "{e}"
                                    "{s}GPS " D_ALTITUDE "{m}%3_f " D_UNIT_METER "{e}"
                                    "{s}GPS " D_VERTICAL_ACCURACY "{m}%3_f " D_UNIT_METER "{e}";
#ifdef USE_GPS_VELOCITY
const char HTTP_SNS_GPS2[] PROGMEM = "{s}GPS " D_SPEED "{m}%2_f " D_UNIT_KILOMETER_PER_HOUR "{e}"
                                     "{s}GPS " D_SPEED_ACCURACY "{m}%2_f " D_UNIT_KILOMETER_PER_HOUR "{e}"
                                     "{s}GPS " D_HEADING "{m}%1_f{e}";
#endif  // USE_GPS_VELOCITY
const char HTTP_SNS_GNSS[] PROGMEM = "{s}GPS receiver{m}%s{e}"
                                     "{s}GPS satellites used / in view{m}%u / %u{e}"
                                     "{s}GPS corrections{m}%s %s{e}";

#ifdef USE_GPS_MAPS
const char UBX_GOOGLE_MAPS[] ="<iframe width='100%%' src='https://maps.google.com/maps?width=&amp;height=&amp;hl=en&amp;q=%s %s+(Tasmota)&amp;ie=UTF8&amp;t=&amp;z=10&amp;iwloc=B&amp;output=embed' frameborder='0' scrolling='no' marginheight='0' marginwidth='0'></iframe>";
#endif  // USE_GPS_MAPS

#endif  // USE_WEBSERVER

const char kGPSFix[] PROGMEM = D_SAT_FIX_NO_FIX "|" D_SAT_FIX_DEAD_RECK "|" D_SAT_FIX_2D "|" D_SAT_FIX_3D "|" D_SAT_FIX_GPS_DEAD "|" D_SAT_FIX_TIME;
const char kGnssCorr[] PROGMEM = "Off|Connecting|Connected|Retrying";

/********************************************************************************************/

void UBXShow(bool json) {
  gnss_state_t *s = &Gnss->st;
  char fix[32];
  GetTextIndexed(fix, sizeof(fix), s->fix_type <= 5 ? s->fix_type : 0, kGPSFix);
  char lat[FLOATSZ];
  dtostrfd((double)UBX.rec_buffer.values.lat / 10000000.0f, 7, lat);  // degrees
  char lon[FLOATSZ];
  dtostrfd((double)UBX.rec_buffer.values.lon / 10000000.0f, 7, lon);  // degrees
  float hAcc = (float)s->h_acc_mm / 1000.0f;                          // mm -> meters
  float alt = (float)s->alt_msl_mm / 1000.0f;                         // mm -> meters
  float vAcc = (float)s->v_acc_mm / 1000.0f;                          // mm -> meters
#ifdef USE_GPS_VELOCITY
  float spd = (float)s->speed_mm_s / 277.778f;                        // mm/s -> km/h
  float sAcc = (float)s->speed_acc_mm_s / 277.778f;                   // mm/s -> km/h
  float hdng = (float)s->course_e5 / 100000.0f;                       // degrees
#endif  // USE_GPS_VELOCITY

  if (json) {
    ResponseAppend_P(PSTR(",\"GPS\":{"));
    if (UBX.mode.send_UI_only) {
      uint32_t i = UBX.state.log_interval / 10;
      ResponseAppend_P(PSTR("\"Fil\":%u,\"Int\":%u}"), UBX.mode.filter_noise, i);
    } else {
      ResponseAppend_P(PSTR("\"Lat\":%s,\"Lon\":%s,\"Alt\":%3_f,\"hAcc\":%3_f,\"vAcc\":%3_f,\"Fix\":\"%s\",\"Sats\":%u"),
        lat, lon, &alt, &hAcc, &vAcc, fix, gnss_count_sats(s, GNSS_UNKNOWN, true));
#ifdef USE_GPS_VELOCITY
      ResponseAppend_P(PSTR(",\"Spd\":%2_f,\"Hdng\":%1_f,\"sAcc\":%2_f"), &spd, &hdng, &sAcc);
#endif  // USE_GPS_VELOCITY
      ResponseAppend_P(PSTR("}"));
    }
#ifdef USE_FLOG
    ResponseAppend_P(PSTR(",\"FLOG\":{\"Rec\":%u,\"Mode\":%u,\"Sec\":%u}"), Flog->recording, Flog->mode, Flog->sectors_left);
#endif  // USE_FLOG
    UBX.mode.send_UI_only = false;
#ifdef USE_WEBSERVER
  } else {
    WSContentSend_PD(HTTP_SNS_GPS, fix, lat, lon, &hAcc, &alt, &vAcc);
#ifdef USE_GPS_VELOCITY
    WSContentSend_PD(HTTP_SNS_GPS2, &spd, &sAcc, &hdng);
#endif  // USE_GPS_VELOCITY
    char corr[16];
    GetTextIndexed(corr, sizeof(corr), Gnss->corr_state, kGnssCorr);
    WSContentSend_PD(HTTP_SNS_GNSS, s->model[0] ? s->model : "searching",
                     gnss_count_sats(s, GNSS_UNKNOWN, true), gnss_count_sats(s, GNSS_UNKNOWN, false),
                     corr, Gnss->corr_state == GNSS_CORR_DISABLED ? "" : Gnss->corr_mount);

#ifdef USE_GPS_MAPS
    int32_t lat_diff = UBX.rec_buffer.values.lat - UBX.lat;
    int32_t lon_diff = UBX.rec_buffer.values.lon - UBX.lon;
    if ((lat_diff > 1000) || (lon_diff > 1000)) {
      UBX.lat = UBX.rec_buffer.values.lat;
      UBX.lon = UBX.rec_buffer.values.lon;
      WSContentSend_P(UBX_GOOGLE_MAPS, lat, lon);
    }
#endif  // USE_GPS_MAPS

#ifdef USE_FLOG
    WSContentSeparator(0);
#ifdef DEBUG_TASMOTA_SENSOR
    WSContentSend_PD(HTTP_SNS_FLOGVER, Flog->num_sectors, Flog->size, Flog->current_sector, Flog->sectors_left, Flog->sector.header.physical_start_sector);
    if (Flog->recording) {
      WSContentSend_PD(HTTP_SNS_FLOGREC, Flog->sector.header.buf_pointer - 8);
    }
#endif  // DEBUG_TASMOTA_SENSOR
    if (Flog->ready) {
      char flog_state[32];
      WSContentSend_P(HTTP_SNS_FLOG, GetTextIndexed(flog_state, sizeof(flog_state), Flog->recording, kFLOGstate));
    }
    if (!Flog->recording && Flog->found_saved_data) {
      WSContentSend_P(HTTP_BTN_FLOG_DL);
    }
#endif  // USE_FLOG

    if (UBX.mode.runningNTP) {
      WSContentSeparator(0);
      WSContentSend_P(HTTP_SNS_NTPSERVER);
    }

#endif  // USE_WEBSERVER
  }
}

/*********************************************************************************************\
 * check the UBX commands
\*********************************************************************************************/

bool UBXCmd(void)
{
  bool serviced = true;
  if (XdrvMailbox.data_len > 0) {
    UBXSelectMode(XdrvMailbox.payload);
    Response_P(S_JSON_UBX_COMMAND_NVALUE, XdrvMailbox.command, XdrvMailbox.payload);
  }
  return serviced;
}

/*********************************************************************************************\
 * Interface
\*********************************************************************************************/

bool Xsns60(uint32_t function)
{
  bool result = false;

  if (FUNC_INIT == function) {
    UBXDetect();
  }

  if (UBX.mode.init) {
    switch (function) {
      case FUNC_LOOP:
#ifdef USE_FLOG
        if (!Flog->running_download)
#endif  // USE_FLOG
        {
          GnssLoop();
        }
        break;
      case FUNC_COMMAND_SENSOR:
        if (XSNS_60 == XdrvMailbox.index) {
          result = UBXCmd();
        }
        break;
      case FUNC_COMMAND:
        result = DecodeCommand(kGnssCommands, GnssCommand);
        break;
      case FUNC_EVERY_50_MSECOND:
        UBXLoop50msec(); // handles virtual serial port and NTP server
        break;
      case FUNC_EVERY_100_MSECOND:
        UBXLoop();       // flash log
        GnssHassStep();
        break;
      case FUNC_EVERY_SECOND:
        GnssEverySecond();
        break;
      case FUNC_MQTT_INIT:
        Gnss->hass_index = 0;      // the broker may have lost retained discovery
        Gnss->n_announced = 0;
        break;
#ifdef USE_FLOG
      case FUNC_WEB_ADD_HANDLER:
        WebServer_on(PSTR("/UBX"), UBXsendFile);
        break;
#endif  // USE_FLOG
      case FUNC_JSON_APPEND:
        UBXShow(1);
        break;
#ifdef USE_WEBSERVER
      case FUNC_WEB_SENSOR:
#ifdef USE_FLOG
        if (!Flog->running_download)
#endif  // USE_FLOG
        {
          UBXShow(0);
        }
        break;
#endif  // USE_WEBSERVER
    }
  }
  return result;
}

#endif  // USE_GPS
