#include "MqttPublisher.h"

#ifdef WITH_NET_BRIDGE

#include <string.h>
#include <malloc.h>
#include <helpers/bridges/EthernetTcpConsole.h>   // EthConsole.isReady() — shared W5100S lease

// Per-slot reconnect backoff ladder. Index advances on each failed connect, resets to
// 0 on success. Staying at the top rung for BREAKER_MAX_FAILS consecutive attempts
// trips the circuit breaker (stops the storm until `mqtt reset` / reconfigure).
static const uint32_t BACKOFF_MS[] = { 5000, 15000, 30000, 60000, 120000 };
static const uint8_t  BACKOFF_MAX     = (sizeof(BACKOFF_MS) / sizeof(BACKOFF_MS[0])) - 1;
static const uint8_t  BREAKER_MAX_FAILS = 3;

// Drop the whole offline queue after this long with no broker connected — stale telemetry
// is worse than none, and it caps the catch-up burst on a long outage.
static const uint32_t QUEUE_STALE_MS = 300000UL;   // 5 min

#define MQTT_KEEPALIVE_S  30

MqttPublisher MqttPub;

// ---- config ----------------------------------------------------------------------

void MqttPublisher::applyConfig() {
  for (int i = 0; i < MQTT_SLOTS; i++) {
    Slot& sl = _slots[i];
    const cliext::MqttSlot& c = cliext::mqttSlot(i);

    // A slot publishes only when fully provisioned: enabled + host + port + base topic.
    sl.active = c.enabled && c.host[0] && c.port && c.topic[0];

    snprintf(sl.topic_raw,    sizeof(sl.topic_raw),    "%s/raw",    c.topic);
    snprintf(sl.topic_status, sizeof(sl.topic_status), "%s/status", c.topic);
    snprintf(sl.client_id,    sizeof(sl.client_id),    "mc-obs-%d", i + 1);

    sl.backoff = 0; sl.max_fails = 0; sl.tripped = false; sl.next_attempt = 0;

    if (!sl.active) continue;
    if (!sl.began) {                     // bind the Client + allocate buffers once
      sl.mqtt.begin(sl.net);
      sl.mqtt.setKeepAlive(MQTT_KEEPALIVE_S);
      sl.began = true;
    }
    sl.mqtt.setHost(c.host, c.port);
    sl.mqtt.setWill(sl.topic_status, "offline", true, 0);   // before connect
  }
}

void MqttPublisher::begin() {
  applyConfig();

  // Surface the queue sizing vs. live free heap so headroom is visible on boot.
  struct mallinfo mi = mallinfo();
  Serial.printf("MQTT: %d slots, queue %d x %d B (%d B), heap free ~%d B\n",
                MQTT_SLOTS, (int)QUEUE_ENTRIES, (int)MQTT_LINE_MAX,
                (int)(QUEUE_ENTRIES * MQTT_LINE_MAX), (int)mi.fordblks);
}

void MqttPublisher::reconfigure() {
  for (int i = 0; i < MQTT_SLOTS; i++) {
    if (_slots[i].began && _slots[i].mqtt.connected()) {
      // Graceful DISCONNECT does NOT trigger the broker's LWT, so the retained status
      // would stay "online" after an `enabled off` / re-host. Publish offline ourselves
      // first; an active slot republishes "online" on its next reconnect.
      _slots[i].mqtt.publish(_slots[i].topic_status, "offline", true, 0);
      _slots[i].mqtt.disconnect();
    }
  }
  applyConfig();
}

// ---- connection lifecycle --------------------------------------------------------

bool MqttPublisher::serviceConnect(int i, uint32_t now) {
  Slot& sl = _slots[i];
  if (!sl.active || sl.tripped) return false;
  if (sl.next_attempt && (long)(now - sl.next_attempt) < 0) return false;

  sl.next_attempt = now + BACKOFF_MS[sl.backoff];

  const cliext::MqttSlot& c = cliext::mqttSlot(i);
  const char* user = c.user[0] ? c.user : nullptr;
  const char* pass = c.pass[0] ? c.pass : nullptr;

  if (sl.mqtt.connect(sl.client_id, user, pass)) {
    sl.backoff = 0; sl.max_fails = 0;
    sl.mqtt.publish(sl.topic_status, "online", true, 0);   // retained
    return true;
  }

  // Failed: climb the backoff ladder; trip the breaker after sustained max-rung failure.
  if (sl.backoff < BACKOFF_MAX) {
    sl.backoff++;
  } else if (++sl.max_fails >= BREAKER_MAX_FAILS) {
    sl.tripped = true;
  }
  return false;
}

void MqttPublisher::loop() {
  if (!EthConsole.isReady()) return;          // shared W5100S not up yet
  uint32_t now = millis();

  // Service connected slots; remember whether any reconnect candidate exists.
  for (int i = 0; i < MQTT_SLOTS; i++) {
    if (_slots[i].active && _slots[i].began && _slots[i].mqtt.connected())
      _slots[i].mqtt.loop();
  }

  // At most one connect attempt per loop (lwmqtt connect() blocks on the handshake) —
  // round-robin so one dead broker can't starve the others or the mesh loop.
  for (int n = 0; n < MQTT_SLOTS; n++) {
    int i = (_attempt_cursor + n) % MQTT_SLOTS;
    Slot& sl = _slots[i];
    if (sl.active && sl.began && !sl.tripped && !sl.mqtt.connected()) {
      serviceConnect(i, now);
      _attempt_cursor = (i + 1) % MQTT_SLOTS;
      break;
    }
  }

  // Queue maintenance: drain to live brokers, or age out a prolonged outage.
  if (anyConnected()) {
    _disconnected_since = 0;
    drainQueue();
  } else if (_q_count) {
    if (_disconnected_since == 0) {
      _disconnected_since = now;
    } else if ((long)(now - _disconnected_since) > (long)QUEUE_STALE_MS) {
      _q_head = _q_tail = _q_count = 0;        // flush stale telemetry
    }
  }
}

// ---- queue + publish -------------------------------------------------------------

bool MqttPublisher::anyConnected() {
  for (int i = 0; i < MQTT_SLOTS; i++)
    if (_slots[i].active && _slots[i].began && _slots[i].mqtt.connected()) return true;
  return false;
}

void MqttPublisher::publishLine(const char* line) {
  for (int i = 0; i < MQTT_SLOTS; i++)
    if (_slots[i].active && _slots[i].began && _slots[i].mqtt.connected())
      _slots[i].mqtt.publish(_slots[i].topic_raw, line);
}

void MqttPublisher::enqueue(const char* line) {
  if (_q_count == QUEUE_ENTRIES) {            // full → drop oldest
    _q_head = (_q_head + 1) % QUEUE_ENTRIES;
    _q_count--;
  }
  strncpy(_queue[_q_tail], line, MQTT_LINE_MAX - 1);
  _queue[_q_tail][MQTT_LINE_MAX - 1] = 0;
  _q_tail = (_q_tail + 1) % QUEUE_ENTRIES;
  _q_count++;
}

void MqttPublisher::drainQueue() {
  while (_q_count) {
    publishLine(_queue[_q_head]);
    _q_head = (_q_head + 1) % QUEUE_ENTRIES;
    _q_count--;
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

// ---- Stream sink -----------------------------------------------------------------

size_t MqttPublisher::write(uint8_t c) {
  if (c == '\r') return 1;                    // ignore CR; lines terminate on '\n'
  if (c == '\n') {
    if (_len > 0) {
      _line[_len] = 0;
      if (anyConnected()) publishLine(_line);  // live → publish to connected brokers
      else                enqueue(_line);       // none up → spill to the ring buffer
      _len = 0;
    }
    return 1;
  }
  if (_len < sizeof(_line) - 1) _line[_len++] = (char)c;
  // Overlong line (no '\n' yet): keep the head, drop the tail until the next newline
  // resets us — bounded, never overruns.
  return 1;
}

size_t MqttPublisher::write(const uint8_t* buf, size_t size) {
  for (size_t i = 0; i < size; i++) write(buf[i]);
  return size;
}

#endif  // WITH_NET_BRIDGE
