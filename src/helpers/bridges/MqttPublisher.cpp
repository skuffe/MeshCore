#include "MqttPublisher.h"

#ifdef WITH_NET_BRIDGE

#include <string.h>
#include <math.h>
#include <malloc.h>
#include <helpers/bridges/EthernetTcpConsole.h>   // EthConsole.isReady() — shared W5100S lease

// Per-slot reconnect backoff ladder. Index advances on each failed connect, resets to
// 0 on success. Staying at the top rung for BREAKER_MAX_FAILS consecutive attempts
// trips the circuit breaker (stops the storm until `mqtt reset` / reconfigure).
static const uint32_t BACKOFF_MS[] = { 5000, 15000, 30000, 60000, 120000 };
static const uint8_t  BACKOFF_MAX       = (sizeof(BACKOFF_MS) / sizeof(BACKOFF_MS[0])) - 1;
static const uint8_t  BREAKER_MAX_FAILS = 3;

// Drop the whole offline queue after this long with no broker connected.
static const uint32_t QUEUE_STALE_MS = 300000UL;   // 5 min
#define MQTT_KEEPALIVE_S  30

MqttPublisher MqttPub;

void MqttPublisher::setContext(mesh::RTCClock* rtc, const char* origin, const char* origin_id,
                               const char* model, const char* firmware, const char* radio,
                               const char* client_version) {
  _rtc = rtc; _origin = origin; _origin_id = origin_id;
  _model = model; _firmware = firmware; _radio = radio; _client_version = client_version;
}

ObserverJson::Ctx MqttPublisher::makeCtx() {
  ObserverJson::Ctx c;
  c.origin = _origin; c.origin_id = _origin_id; c.model = _model;
  c.firmware = _firmware; c.radio = _radio; c.client_version = _client_version;
  c.epoch = _rtc ? _rtc->getCurrentTime() : 0;
  c.uptime_secs = (int)(millis() / 1000);
  struct mallinfo mi = mallinfo();
  c.internal_heap = (int)mi.fordblks;
  c.queue_len = _q_count;
  return c;
}

// ---- config ----------------------------------------------------------------------

void MqttPublisher::applyConfig() {
  for (int i = 0; i < MQTT_SLOTS; i++) {
    Slot& sl = _slots[i];
    const cliext::MqttSlot& c = cliext::mqttSlot(i);

    sl.active = c.enabled && c.host[0] && c.port && c.topic[0];

    snprintf(sl.topic_packets, sizeof(sl.topic_packets), "%s/packets", c.topic);
    snprintf(sl.topic_raw,     sizeof(sl.topic_raw),     "%s/raw",     c.topic);
    snprintf(sl.topic_status,  sizeof(sl.topic_status),  "%s/status",  c.topic);
    snprintf(sl.client_id,     sizeof(sl.client_id),     "mc-obs-%d",  i + 1);

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
  applyConfig();
  struct mallinfo mi = mallinfo();
  Serial.printf("MQTT: %d slots, queue %d x %d B (%d B), heap free ~%d B\n",
                MQTT_SLOTS, (int)QUEUE_ENTRIES, (int)sizeof(QEntry),
                (int)(QUEUE_ENTRIES * (int)sizeof(QEntry)), (int)mi.fordblks);
}

void MqttPublisher::reconfigure() {
  for (int i = 0; i < MQTT_SLOTS; i++) {
    if (_slots[i].began && _slots[i].mqtt.connected()) {
      // Graceful DISCONNECT does NOT trigger the broker LWT, so publish offline status
      // ourselves before tearing down; an active slot republishes online on reconnect.
      publishStatusToSlot(i, false);
      _slots[i].mqtt.disconnect();
    }
  }
  applyConfig();
}

// ---- connection lifecycle --------------------------------------------------------

void MqttPublisher::publishStatusToSlot(int i, bool online) {
  Slot& sl = _slots[i];
  if (!sl.began || !sl.mqtt.connected()) return;
  char json[MQTT_MSG_MAX];
  ObserverJson::Ctx ctx = makeCtx();
  size_t n = ObserverJson::buildStatus(json, sizeof(json), ctx, online);
  if (n) sl.mqtt.publish(sl.topic_status, json, true, 0);   // retained
}

bool MqttPublisher::serviceConnect(int i, uint32_t now) {
  Slot& sl = _slots[i];
  if (!sl.active || sl.tripped) return false;
  if (sl.next_attempt && (long)(now - sl.next_attempt) < 0) return false;
  sl.next_attempt = now + BACKOFF_MS[sl.backoff];

  const cliext::MqttSlot& c = cliext::mqttSlot(i);
  const char* user = c.user[0] ? c.user : nullptr;
  const char* pass = c.pass[0] ? c.pass : nullptr;

  // LWT = retained offline status JSON. willbuf only needs to live across connect()
  // (lwmqtt serializes the will into the CONNECT packet synchronously).
  char willbuf[MQTT_MSG_MAX];
  ObserverJson::Ctx ctx = makeCtx();
  if (ObserverJson::buildStatus(willbuf, sizeof(willbuf), ctx, false))
    sl.mqtt.setWill(sl.topic_status, willbuf, true, 0);

  if (sl.mqtt.connect(sl.client_id, user, pass)) {
    sl.backoff = 0; sl.max_fails = 0;
    publishStatusToSlot(i, true);        // retained online status
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
    _disconnected_since = 0;
    drainQueue();
    maybePeriodicStatus(now);
  } else if (_q_count) {
    if (_disconnected_since == 0) _disconnected_since = now;
    else if ((long)(now - _disconnected_since) > (long)QUEUE_STALE_MS)
      _q_head = _q_tail = _q_count = 0;
  }
}

void MqttPublisher::maybePeriodicStatus(uint32_t now) {
  if (!cliext::config().msg_status) return;
  uint32_t iv = (uint32_t)cliext::config().status_interval_s * 1000UL;
  if (_last_status_ms != 0 && (long)(now - _last_status_ms) < (long)iv) return;
  _last_status_ms = now;
  for (int i = 0; i < MQTT_SLOTS; i++)
    if (_slots[i].active && _slots[i].began && _slots[i].mqtt.connected())
      publishStatusToSlot(i, true);
}

// ---- publish + queue -------------------------------------------------------------

bool MqttPublisher::anyConnected() {
  for (int i = 0; i < MQTT_SLOTS; i++)
    if (_slots[i].active && _slots[i].began && _slots[i].mqtt.connected()) return true;
  return false;
}

void MqttPublisher::publishLive(const char*, MsgType type, const char* json) {
  for (int i = 0; i < MQTT_SLOTS; i++) {
    Slot& sl = _slots[i];
    if (!sl.active || !sl.began || !sl.mqtt.connected()) continue;
    const char* topic = (type == MSG_RAW) ? sl.topic_raw : sl.topic_packets;
    sl.mqtt.publish(topic, json);
  }
}

void MqttPublisher::enqueue(MsgType type, const char* json) {
  if (_q_count == QUEUE_ENTRIES) {            // full → drop oldest
    _q_head = (_q_head + 1) % QUEUE_ENTRIES;
    _q_count--;
  }
  _queue[_q_tail].type = (uint8_t)type;
  strncpy(_queue[_q_tail].json, json, MQTT_MSG_MAX - 1);
  _queue[_q_tail].json[MQTT_MSG_MAX - 1] = 0;
  _q_tail = (_q_tail + 1) % QUEUE_ENTRIES;
  _q_count++;
}

void MqttPublisher::drainQueue() {
  while (_q_count) {
    QEntry& e = _queue[_q_head];
    publishLive(nullptr, (MsgType)e.type, e.json);
    _q_head = (_q_head + 1) % QUEUE_ENTRIES;
    _q_count--;
  }
}

// ---- feed hooks ------------------------------------------------------------------

void MqttPublisher::onRawRx(const uint8_t* raw, int len, float snr, float rssi) {
  // Stage for the imminent onPacketRx (same packet, fired right after by the dispatcher).
  if (len > 0 && len <= (int)sizeof(_staged_raw)) {
    memcpy(_staged_raw, raw, len);
    _staged_len = len; _staged_snr = snr; _staged_rssi = rssi; _staged_valid = true;
  } else {
    _staged_valid = false;
  }

  if (!cliext::config().msg_raw) return;
  char json[MQTT_MSG_MAX];
  ObserverJson::Ctx ctx = makeCtx();
  if (ObserverJson::buildRaw(json, sizeof(json), ctx, raw, len)) {
    if (anyConnected()) publishLive(nullptr, MSG_RAW, json);
    else                enqueue(MSG_RAW, json);
  }
}

void MqttPublisher::onPacketRx(mesh::Packet* pkt, float score) {
  const cliext::Config& cfg = cliext::config();
  if (!cfg.msg_packets || !cfg.msg_rx) { _staged_valid = false; return; }

  const uint8_t* raw = _staged_valid ? _staged_raw : nullptr;
  int   rawlen = _staged_valid ? _staged_len : 0;
  float snr    = _staged_valid ? _staged_snr : 0;
  float rssi   = _staged_valid ? _staged_rssi : 0;
  _staged_valid = false;

  char json[MQTT_MSG_MAX];
  ObserverJson::Ctx ctx = makeCtx();
  if (ObserverJson::buildPacket(json, sizeof(json), ctx, pkt, false, raw, rawlen, snr, rssi, score)) {
    if (anyConnected()) publishLive(nullptr, MSG_PACKETS, json);
    else                enqueue(MSG_PACKETS, json);
  }
}

void MqttPublisher::onPacketTx(mesh::Packet* pkt) {
  const cliext::Config& cfg = cliext::config();
  if (cfg.msg_tx == 0 || !cfg.msg_packets) return;
  if (cfg.msg_tx == 2 && pkt->getPayloadType() != PAYLOAD_TYPE_ADVERT) return;  // advert-only

  char json[MQTT_MSG_MAX];
  ObserverJson::Ctx ctx = makeCtx();
  if (ObserverJson::buildPacket(json, sizeof(json), ctx, pkt, true, nullptr, 0, 0, 0, NAN)) {
    if (anyConnected()) publishLive(nullptr, MSG_PACKETS, json);
    else                enqueue(MSG_PACKETS, json);
  }
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
