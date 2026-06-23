#include "MqttPublisher.h"

#ifdef WITH_NET_BRIDGE

#include <string.h>
#include <math.h>
#include <malloc.h>
#include <helpers/bridges/EthernetTcpConsole.h>   // EthConsole.isReady() — shared W5100S lease
#include <helpers/bridges/BackhaulFrame.h>         // EPOCH_SANE_MIN (clock-synced threshold)

// Per-slot reconnect backoff ladder. Index advances on each failed connect, resets to
// 0 on success. Staying at the top rung for BREAKER_MAX_FAILS consecutive attempts
// trips the circuit breaker (stops the storm until `mqtt reset` / reconfigure).
static const uint32_t BACKOFF_MS[] = { 5000, 15000, 30000, 60000, 120000 };
static const uint8_t  BACKOFF_MAX       = (sizeof(BACKOFF_MS) / sizeof(BACKOFF_MS[0])) - 1;
static const uint8_t  BREAKER_MAX_FAILS = 3;

#define MQTT_KEEPALIVE_S  30

MqttPublisher MqttPub;

void MqttPublisher::setContext(mesh::RTCClock* rtc, const char* origin, const char* origin_id,
                               const char* model, const char* firmware, const char* radio,
                               const char* client_version) {
  _rtc = rtc; _origin = origin; _origin_id = origin_id;
  _model = model; _firmware = firmware; _radio = radio; _client_version = client_version;
}

// Current UTC epoch from this node's NTP-synced clock; remembers the last plausible value
// as a fallback floor (used when an observer's clock is unsynced).
uint32_t MqttPublisher::nowEpoch() {
  uint32_t e = _rtc ? _rtc->getCurrentTime() : 0;
  if (e >= backhaul::EPOCH_SANE_MIN) _last_epoch = e;
  return e;
}

ObserverJson::Ctx MqttPublisher::makeCtx() {
  ObserverJson::Ctx c;
  c.origin = _origin; c.origin_id = _origin_id; c.model = _model;
  c.firmware = _firmware; c.radio = _radio; c.client_version = _client_version;
  c.epoch = nowEpoch();
  c.uptime_secs = (int)(millis() / 1000);
  c.internal_heap = -1;   // mallinfo fordblks is bogus on this core — omit (see M2 notes)
  c.queue_len = (int)_q.count();
  return c;
}

// Per-observer context: self build strings + the observe-time `epoch` (the observer's own
// moment, per spec — NOT this node's publish time), with origin/origin_id from the observer
// table. Relay observers (idx != 0) have no model/firmware/radio of their own → blanked.
ObserverJson::Ctx MqttPublisher::makeCtxFor(uint8_t obsIdx, uint32_t epoch) {
  ObserverJson::Ctx c = makeCtx();
  c.epoch = epoch;
  if (obsIdx >= OBSERVERS_MAX) return c;
  const cliext::Observer& o = cliext::config().observers[obsIdx];
  if (o.name[0])       c.origin    = o.name;
  if (o.pubkey_hex[0]) c.origin_id = o.pubkey_hex;
  if (obsIdx != 0) { c.model = ""; c.firmware = ""; c.radio = ""; }
  return c;
}

bool MqttPublisher::observerEnabled(uint8_t obsIdx) {
  if (obsIdx >= OBSERVERS_MAX) return false;
  const cliext::Observer& o = cliext::config().observers[obsIdx];
  return o.mqtt_enabled && o.pubkey_hex[0];
}

// Topic = <slot prefix>/<observer pubkey>/<suffix>, i.e. meshcore/{iata}/{device}/{type}.
void MqttPublisher::buildTopic(char* out, size_t cap, int slot_i, uint8_t obsIdx,
                               const char* suffix) {
  const char* prefix = cliext::mqttSlot(slot_i).topic;
  const char* pubkey = cliext::config().observers[obsIdx].pubkey_hex;
  snprintf(out, cap, "%s/%s/%s", prefix, pubkey, suffix);
}

// ---- config ----------------------------------------------------------------------

void MqttPublisher::applyConfig() {
  for (int i = 0; i < MQTT_SLOTS; i++) {
    Slot& sl = _slots[i];
    const cliext::MqttSlot& c = cliext::mqttSlot(i);

    sl.active = c.enabled && c.host[0] && c.port && c.topic[0];

    snprintf(sl.client_id, sizeof(sl.client_id), "mc-obs-%d", i + 1);

    sl.backoff = 0; sl.max_fails = 0; sl.tripped = false; sl.next_attempt = 0;

    if (!sl.active) continue;
    if (!sl.began) {                     // bind the Client + allocate buffers once
      sl.mqtt.begin(sl.net);
      sl.mqtt.setKeepAlive(MQTT_KEEPALIVE_S);
      sl.began = true;
    }
    sl.mqtt.setHost(c.host, c.port);
  }
}

void MqttPublisher::begin() {
  _q.init(_q_arena, sizeof(_q_arena));
  applyConfig();
  struct mallinfo mi = mallinfo();
  Serial.printf("MQTT: %d slots, offline cache %d B, heap free ~%d B\n",
                MQTT_SLOTS, (int)sizeof(_q_arena), (int)mi.fordblks);
}

void MqttPublisher::reconfigure() {
  for (int i = 0; i < MQTT_SLOTS; i++) {
    if (_slots[i].began && _slots[i].mqtt.connected()) {
      // Graceful DISCONNECT does NOT trigger the broker LWT, so publish offline status
      // ourselves before tearing down; an active slot republishes online on reconnect.
      publishStatusToSlot(i, 0, false);
      _slots[i].mqtt.disconnect();
    }
  }
  applyConfig();
}

// ---- connection lifecycle --------------------------------------------------------

void MqttPublisher::publishStatusToSlot(int i, uint8_t obsIdx, bool online) {
  Slot& sl = _slots[i];
  if (!sl.began || !sl.mqtt.connected()) return;
  if (!observerEnabled(obsIdx)) return;
  char json[MQTT_MSG_MAX], topic[176];
  ObserverJson::Ctx ctx = makeCtxFor(obsIdx, nowEpoch());
  size_t n = ObserverJson::buildStatus(json, sizeof(json), ctx, online);
  if (n) { buildTopic(topic, sizeof(topic), i, obsIdx, "status"); sl.mqtt.publish(topic, json, true, 0); }   // retained
}

bool MqttPublisher::serviceConnect(int i, uint32_t now) {
  Slot& sl = _slots[i];
  if (!sl.active || sl.tripped) return false;
  if (sl.next_attempt && (long)(now - sl.next_attempt) < 0) return false;
  sl.next_attempt = now + BACKOFF_MS[sl.backoff];

  const cliext::MqttSlot& c = cliext::mqttSlot(i);
  const char* user = c.user[0] ? c.user : nullptr;
  const char* pass = c.pass[0] ? c.pass : nullptr;

  // LWT = the SELF observer's retained offline status JSON (the broker fires it if THIS
  // node drops). willbuf only needs to live across connect() (lwmqtt serializes the will
  // into the CONNECT packet synchronously). Relay observers' offline is published
  // app-level on backhaul link loss (one LWT per connection covers only self).
  char willbuf[MQTT_MSG_MAX], will_topic[176];
  ObserverJson::Ctx ctx = makeCtxFor(0, nowEpoch());
  if (observerEnabled(0) && ObserverJson::buildStatus(willbuf, sizeof(willbuf), ctx, false)) {
    buildTopic(will_topic, sizeof(will_topic), i, 0, "status");
    sl.mqtt.setWill(will_topic, willbuf, true, 0);
  }

  if (sl.mqtt.connect(sl.client_id, user, pass)) {
    sl.backoff = 0; sl.max_fails = 0;
    publishStatusToSlot(i, 0, true);        // retained online status (self)
    return true;
  }
  if (sl.backoff < BACKOFF_MAX)            sl.backoff++;
  else if (++sl.max_fails >= BREAKER_MAX_FAILS) sl.tripped = true;
  return false;
}

void MqttPublisher::loop() {
  if (!EthConsole.isReady()) return;
  uint32_t now = millis();

  for (int i = 0; i < MQTT_SLOTS; i++)
    if (_slots[i].active && _slots[i].began && _slots[i].mqtt.connected())
      _slots[i].mqtt.loop();

  // One connect attempt per loop (lwmqtt connect() blocks) — round-robin for fairness.
  for (int n = 0; n < MQTT_SLOTS; n++) {
    int i = (_attempt_cursor + n) % MQTT_SLOTS;
    Slot& sl = _slots[i];
    if (sl.active && sl.began && !sl.tripped && !sl.mqtt.connected()) {
      serviceConnect(i, now);
      _attempt_cursor = (i + 1) % MQTT_SLOTS;
      break;
    }
  }

  if (anyConnected()) {
    drainQueue();
    maybePeriodicStatus(now);
  }
  // No time-based expiry: the offline cache is bounded by capacity (drop-oldest) and every
  // message carries its observe-time stamp, so a long broker outage keeps the freshest
  // correctly-timestamped messages instead of discarding the lot.
}

void MqttPublisher::maybePeriodicStatus(uint32_t now) {
  if (!cliext::config().msg_status) return;
  uint32_t iv = (uint32_t)cliext::config().status_interval_s * 1000UL;
  if (_last_status_ms != 0 && (long)(now - _last_status_ms) < (long)iv) return;
  _last_status_ms = now;
  // Periodic status for self (observers[0]); relay observers' status is event-driven
  // from their backhaul link state (publishObserverStatus), not polled here.
  for (int i = 0; i < MQTT_SLOTS; i++)
    if (_slots[i].active && _slots[i].began && _slots[i].mqtt.connected())
      publishStatusToSlot(i, 0, true);
}

// ---- publish + queue -------------------------------------------------------------

bool MqttPublisher::anyConnected() {
  for (int i = 0; i < MQTT_SLOTS; i++)
    if (_slots[i].active && _slots[i].began && _slots[i].mqtt.connected()) return true;
  return false;
}

void MqttPublisher::publishLive(uint8_t obsIdx, MsgType type, const char* json) {
  if (!observerEnabled(obsIdx)) return;
  const char* suffix = (type == MSG_RAW) ? "raw" : "packets";
  char topic[176];
  for (int i = 0; i < MQTT_SLOTS; i++) {
    Slot& sl = _slots[i];
    if (!sl.active || !sl.began || !sl.mqtt.connected()) continue;
    buildTopic(topic, sizeof(topic), i, obsIdx, suffix);
    sl.mqtt.publish(topic, json);
  }
}

void MqttPublisher::enqueue(uint8_t obsIdx, MsgType type, const char* json) {
  // Record = [observer:1][type:1][json...]; ByteRing drops oldest to fit.
  uint8_t rec[2 + MQTT_MSG_MAX];
  size_t jlen = strnlen(json, MQTT_MSG_MAX - 1);
  rec[0] = obsIdx; rec[1] = (uint8_t)type;
  memcpy(rec + 2, json, jlen);
  _q.push(rec, (uint16_t)(2 + jlen));
}

void MqttPublisher::drainQueue() {
  uint8_t rec[2 + MQTT_MSG_MAX];
  char json[MQTT_MSG_MAX];
  while (!_q.empty()) {
    uint16_t n = _q.front(rec, sizeof(rec));
    if (!n || n < 2) { _q.pop(); continue; }
    uint16_t jlen = n - 2;
    if (jlen >= sizeof(json)) jlen = sizeof(json) - 1;
    memcpy(json, rec + 2, jlen); json[jlen] = 0;
    publishLive(rec[0], (MsgType)rec[1], json);   // topic rebuilt per observer
    _q.pop();
  }
}

// Shared packet→JSON→publish path for BOTH the local and the relayed feed. obsIdx selects
// the observer identity/topic; pkt may be a live packet (local) or one reconstructed from
// framed wire bytes (relayed); `epoch` is the observe-time stamp. The builder is identical.
void MqttPublisher::emitPacket(uint8_t obsIdx, mesh::Packet* pkt, bool is_tx,
                               const uint8_t* raw, int raw_len, float snr, float rssi, float score,
                               uint32_t epoch) {
  if (!pkt || !observerEnabled(obsIdx)) return;
  const cliext::Config& cfg = cliext::config();
  if (!cfg.msg_packets) return;
  if (is_tx) {
    if (cfg.msg_tx == 0) return;
    if (cfg.msg_tx == 2 && pkt->getPayloadType() != PAYLOAD_TYPE_ADVERT) return;  // advert-only
  } else if (!cfg.msg_rx) {
    return;
  }
  char json[MQTT_MSG_MAX];
  ObserverJson::Ctx ctx = makeCtxFor(obsIdx, epoch);
  if (ObserverJson::buildPacket(json, sizeof(json), ctx, pkt, is_tx, raw, raw_len, snr, rssi, score)) {
    if (anyConnected()) publishLive(obsIdx, MSG_PACKETS, json);
    else                enqueue(obsIdx, MSG_PACKETS, json);
  }
}

// ---- local feed hooks (self = observers[0]) --------------------------------------

void MqttPublisher::onRawRx(const uint8_t* raw, int len, float snr, float rssi) {
  // Stage for the imminent onPacketRx (same packet, fired right after by the dispatcher).
  if (len > 0 && len <= (int)sizeof(_staged_raw)) {
    memcpy(_staged_raw, raw, len);
    _staged_len = len; _staged_snr = snr; _staged_rssi = rssi; _staged_valid = true;
  } else {
    _staged_valid = false;
  }

  if (!cliext::config().msg_raw || !observerEnabled(0)) return;
  char json[MQTT_MSG_MAX];
  ObserverJson::Ctx ctx = makeCtxFor(0, nowEpoch());
  if (ObserverJson::buildRaw(json, sizeof(json), ctx, raw, len)) {
    if (anyConnected()) publishLive(0, MSG_RAW, json);
    else                enqueue(0, MSG_RAW, json);
  }
}

void MqttPublisher::onPacketRx(mesh::Packet* pkt, float score) {
  const uint8_t* raw = _staged_valid ? _staged_raw : nullptr;
  int   rawlen = _staged_valid ? _staged_len : 0;
  float snr    = _staged_valid ? _staged_snr : 0;
  float rssi   = _staged_valid ? _staged_rssi : 0;
  _staged_valid = false;
  emitPacket(0, pkt, false, raw, rawlen, snr, rssi, score, nowEpoch());   // self: stamp at observe
}

void MqttPublisher::onPacketTx(mesh::Packet* pkt) {
  emitPacket(0, pkt, true, nullptr, 0, 0, 0, NAN, nowEpoch());
}

// ---- relayed feed (a remote observer over the backhaul) --------------------------

void MqttPublisher::publishObservation(uint8_t obsIdx, bool is_tx, const uint8_t* wire,
                                       int wire_len, float snr, float rssi, float score,
                                       uint32_t epoch) {
  mesh::Packet pkt;
  if (!pkt.readFrom(wire, (uint8_t)wire_len)) return;   // reconstruct from the framed bytes
  // Stamp with the observer's observe-time epoch; if its clock was unsynced (implausible),
  // fall back to this node's NTP clock so the timestamp is at least sane.
  uint32_t e = (epoch >= backhaul::EPOCH_SANE_MIN) ? epoch : nowEpoch();
  // RX: the framed wire bytes ARE the on-air radio bytes → reuse them for the `raw` hex
  // + length, exactly as the local RX path does. TX: builder serializes the packet.
  const uint8_t* raw = is_tx ? nullptr : wire;
  int   raw_len2     = is_tx ? 0       : wire_len;
  emitPacket(obsIdx, &pkt, is_tx, raw, raw_len2, snr, rssi, score, e);
}

void MqttPublisher::publishObserverStatus(uint8_t obsIdx, bool online) {
  if (!observerEnabled(obsIdx)) return;
  for (int i = 0; i < MQTT_SLOTS; i++)
    if (_slots[i].active && _slots[i].began && _slots[i].mqtt.connected())
      publishStatusToSlot(i, obsIdx, online);
}

// ---- CLI introspection -----------------------------------------------------------

bool MqttPublisher::slotConnected(int i) {
  if (i < 0 || i >= MQTT_SLOTS) return false;
  return _slots[i].active && _slots[i].began && _slots[i].mqtt.connected();
}
bool MqttPublisher::slotTripped(int i) const {
  if (i < 0 || i >= MQTT_SLOTS) return false;
  return _slots[i].tripped;
}
void MqttPublisher::resetSlot(int i) {
  for (int k = 0; k < MQTT_SLOTS; k++) {
    if (i >= 0 && k != i) continue;
    _slots[k].backoff = 0; _slots[k].max_fails = 0;
    _slots[k].tripped = false; _slots[k].next_attempt = 0;
  }
}

#endif  // WITH_NET_BRIDGE
