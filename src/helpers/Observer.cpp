#include "Observer.h"
#include <math.h>
#include <string.h>

#ifdef WITH_NET_BRIDGE
  #include <helpers/bridges/MqttPublisher.h>
#endif

#ifdef WITH_BACKHAUL_PERIPHERAL
  #include <MeshCore.h>                       // PUB_KEY_SIZE, MAX_TRANS_UNIT
  #include <Mesh.h>                            // mesh::RTCClock
  #include <helpers/nrf52/BleConsole.h>
  #include <helpers/bridges/BackhaulFrame.h>
  #include <helpers/ByteRing.h>               // generalized offline cache
  #include <helpers/console/CLIExtensions.h>  // cliext::g_packet_dump_enabled (`log on|off`)
#endif

namespace observer {

#ifdef WITH_NET_BRIDGE
// ---- net-bridge sink: publish locally (observers[0] = self) ------------------------
// The hooks delegate to the existing local publisher unchanged; the central's relayed
// path (BleNusRelay → MqttPub.publishObservation) shares the SAME publish internals.
void onRawRx(const uint8_t* raw, int len, float snr, float rssi) { MqttPub.onRawRx(raw, len, snr, rssi); }
void onPacketRx(mesh::Packet* pkt, float score)                  { MqttPub.onPacketRx(pkt, score); }
void onPacketTx(mesh::Packet* pkt)                               { MqttPub.onPacketTx(pkt); }

#elif defined(WITH_BACKHAUL_PERIPHERAL)
// ---- peripheral sink: frame the observation over the BLE NUS backhaul ---------------
static mesh::RTCClock* s_rtc = nullptr;
static uint8_t  s_pubkey[PUB_KEY_SIZE];
static char     s_name[32];
static char     s_model[16];      // self-described hardware, framed in IDENTITY so the
static char     s_firmware[16];   // central can fill this relay observer's status JSON
static char     s_radio[12];      // (model/firmware/radio) instead of leaving it blank.
static bool     s_have_id = false;

// Staged RX radio bytes for the imminent onPacketRx (same packet, fired right after).
static uint8_t  s_raw[MAX_TRANS_UNIT];
static int      s_raw_len = 0;
static float    s_snr = 0, s_rssi = 0;
static bool     s_staged = false;

// Offline observation cache. The mast ALWAYS pushes encoded OBSERVATION frames here and a
// drain step flushes them to the backhaul whenever the link is up — so a dropout simply
// fills the ring (drop-oldest) and reconnect replays them in order, each keeping the
// observe-time epoch baked into its ObsHeader. The RAK3401 has ample free RAM (~13% used),
// so the arena is generous.
#ifndef OBS_CACHE_BYTES
#define OBS_CACHE_BYTES 49152
#endif
static uint8_t  s_cache_arena[OBS_CACHE_BYTES];
static ByteRing s_cache;
static bool     s_cache_init = false;

// IDENTITY cadence (prior-art aligned): the central may not have enabled NUS notifications
// the instant the link comes up (CCCD write lands a few connection events later), so a
// single connect-edge announce is lost. Burst a few IDENTITY frames right after connect to
// converge fast, then settle to the 300 s status cadence (agessaman mqtt_status_interval).
static bool     s_was_connected = false;
static uint32_t s_id_next = 0;
static uint8_t  s_id_burst = 0;
static uint32_t s_status_next = 0;
static const uint8_t  ID_BURST_COUNT  = 4;
static const uint32_t ID_BURST_MS     = 4000;     // 4 announces over ~16 s after connect
static const uint32_t ANNOUNCE_PERIOD_MS = 300000;  // steady-state IDENTITY + STATUS

// Demux of central→mast frames out of the inbound NUS/CLI stream.
static backhaul::Parser s_in;

// Remote-admin handler (set by main): runs a console command, fills the reply buffer.
static ConsoleHandler s_console = nullptr;

static void cacheObservation(bool is_tx, const uint8_t* wire, int wire_len,
                             float snr, float rssi, float score) {
  if (!cliext::g_packet_dump_enabled) return;   // `log off` silences both feeds
  if (wire_len <= 0 || wire_len > MAX_TRANS_UNIT) return;
  if (!s_cache_init) { s_cache.init(s_cache_arena, sizeof(s_cache_arena)); s_cache_init = true; }

  backhaul::ObsHeader h;
  memset(&h, 0, sizeof(h));
  h.is_tx = is_tx ? 1 : 0;
  h.epoch = s_rtc ? s_rtc->getCurrentTime() : 0;   // observe-time stamp (real if synced)
  h.snr = snr; h.rssi = rssi; h.score = score;
  h.wire_len = (uint8_t)wire_len;

  uint8_t payload[sizeof(backhaul::ObsHeader) + MAX_TRANS_UNIT];
  memcpy(payload, &h, sizeof(h));
  memcpy(payload + sizeof(h), wire, wire_len);

  uint8_t frame[5 + sizeof(payload)];
  size_t n = backhaul::encode(frame, sizeof(frame), backhaul::FRAME_OBSERVATION,
                              payload, (uint16_t)(sizeof(h) + wire_len));
  if (n) s_cache.push(frame, (uint16_t)n);   // drain happens in loop() when the link is up
}

void onRawRx(const uint8_t* raw, int len, float snr, float rssi) {
  if (len > 0 && len <= (int)sizeof(s_raw)) {
    memcpy(s_raw, raw, len);
    s_raw_len = len; s_snr = snr; s_rssi = rssi; s_staged = true;
  } else {
    s_staged = false;
  }
}

void onPacketRx(mesh::Packet* pkt, float score) {
  if (s_staged) {
    cacheObservation(false, s_raw, s_raw_len, s_snr, s_rssi, score);
  } else if (pkt) {                               // no staged radio bytes — serialize the packet
    uint8_t wbuf[MAX_TRANS_UNIT];
    uint8_t wlen = pkt->writeTo(wbuf);
    cacheObservation(false, wbuf, wlen, 0, 0, score);
  }
  s_staged = false;
}

void onPacketTx(mesh::Packet* pkt) {
  if (!pkt) return;
  uint8_t wbuf[MAX_TRANS_UNIT];
  uint8_t wlen = pkt->writeTo(wbuf);
  cacheObservation(true, wbuf, wlen, 0, 0, NAN);
}

static void copyField(char* dst, size_t cap, const char* src) {
  if (!src) { dst[0] = 0; return; }
  strncpy(dst, src, cap - 1); dst[cap - 1] = 0;
}

void begin(mesh::RTCClock* rtc, const uint8_t* pubkey, const char* name,
           const char* model, const char* firmware, const char* radio) {
  s_rtc = rtc;
  if (pubkey) memcpy(s_pubkey, pubkey, PUB_KEY_SIZE);
  copyField(s_name,     sizeof(s_name),     name);
  copyField(s_model,    sizeof(s_model),    model);
  copyField(s_firmware, sizeof(s_firmware), firmware);
  copyField(s_radio,    sizeof(s_radio),    radio);
  s_have_id = (pubkey != nullptr);
  s_cache.init(s_cache_arena, sizeof(s_cache_arena)); s_cache_init = true;
}

static void sendIdentity() {
  if (!s_have_id) return;
  // pubkey[32] + NUL-separated fields: name\0model\0firmware\0radio (see BackhaulFrame.h).
  uint8_t payload[PUB_KEY_SIZE + sizeof(s_name) + sizeof(s_model)
                  + sizeof(s_firmware) + sizeof(s_radio) + 3];
  size_t off = PUB_KEY_SIZE;
  memcpy(payload, s_pubkey, PUB_KEY_SIZE);
  const char* fields[] = { s_name, s_model, s_firmware, s_radio };
  for (int i = 0; i < 4; i++) {
    if (i) payload[off++] = 0;                       // NUL separator between fields
    size_t l = strlen(fields[i]);
    memcpy(payload + off, fields[i], l); off += l;
  }
  uint8_t frame[5 + sizeof(payload)];
  size_t n = backhaul::encode(frame, sizeof(frame), backhaul::FRAME_IDENTITY,
                              payload, (uint16_t)off);
  if (n) BleConsole.write(frame, n);
}

static void sendStatus() {
  backhaul::StatusBody sb;
  sb.uptime_secs = millis() / 1000;
  uint8_t frame[5 + sizeof(sb)];
  size_t n = backhaul::encode(frame, sizeof(frame), backhaul::FRAME_STATUS,
                              (const uint8_t*)&sb, (uint16_t)sizeof(sb));
  if (n) BleConsole.write(frame, n);
}

// Flush cached observations to the backhaul, oldest first, while the link is up.
static void drainCache() {
  uint8_t rec[5 + sizeof(backhaul::ObsHeader) + MAX_TRANS_UNIT];
  while (!s_cache.empty() && BleConsole.connected()) {
    uint16_t n = s_cache.front(rec, sizeof(rec));
    if (!n) { s_cache.pop(); continue; }   // oversize/corrupt guard
    BleConsole.write(rec, n);
    s_cache.pop();
  }
}

void loop() {
  bool now_conn = BleConsole.connected();
  uint32_t now = millis();
  if (now_conn && !s_was_connected) {            // central just attached → schedule announces
    s_id_next = now; s_id_burst = ID_BURST_COUNT;
    s_status_next = now + 2000;
  }
  if (now_conn) {
    if ((long)(now - s_id_next) >= 0) {
      sendIdentity();
      if (s_id_burst) { s_id_burst--; s_id_next = now + ID_BURST_MS; }
      else            { s_id_next = now + ANNOUNCE_PERIOD_MS; }
    }
    if ((long)(now - s_status_next) >= 0) {
      sendStatus();
      s_status_next = now + ANNOUNCE_PERIOD_MS;
    }
    drainCache();
  }
  s_was_connected = now_conn;
}

void setConsoleHandler(ConsoleHandler fn) { s_console = fn; }

bool feedBackhaulByte(uint8_t b) {
  backhaul::Parser::Result r = s_in.feed(b);
  if (r == backhaul::Parser::PASS) return false;          // ordinary CLI text → caller handles
  if (r == backhaul::Parser::FRAME) {
    if (s_in.type() == backhaul::FRAME_TIME && s_in.len() >= sizeof(backhaul::TimeBody) && s_rtc) {
      backhaul::TimeBody tb; memcpy(&tb, s_in.payload(), sizeof(tb));
      if (tb.epoch >= backhaul::EPOCH_SANE_MIN) s_rtc->setCurrentTime(tb.epoch);  // backhaul time sync
    } else if (s_in.type() == backhaul::FRAME_CONSOLE && s_console) {
      // Remote-admin: run the command line, frame the reply back to the central.
      char cmd[160];
      uint16_t l = s_in.len();
      if (l >= sizeof(cmd)) l = sizeof(cmd) - 1;
      memcpy(cmd, s_in.payload(), l); cmd[l] = 0;
      char reply[160]; reply[0] = 0;
      s_console(cmd, reply, sizeof(reply));
      if (reply[0]) {
        uint8_t frame[5 + sizeof(reply)];
        size_t fn = backhaul::encode(frame, sizeof(frame), backhaul::FRAME_CONSOLE,
                                     (const uint8_t*)reply, (uint16_t)strlen(reply));
        if (fn) BleConsole.write(frame, fn);
      }
    }
  }
  return true;   // EAT or FRAME — consumed as part of a structured frame
}

#else
// ---- no sink compiled in -----------------------------------------------------------
void onRawRx(const uint8_t*, int, float, float) {}
void onPacketRx(mesh::Packet*, float) {}
void onPacketTx(mesh::Packet*) {}
#endif

}  // namespace observer
