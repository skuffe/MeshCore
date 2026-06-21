#pragma once

#ifdef WITH_NET_BRIDGE

#include <Arduino.h>
#include <RAK13800_W5100S.h>
#include <MQTT.h>                              // 256dpi/arduino-mqtt (lwmqtt)
#include <helpers/console/CliextConfig.h>      // MqttSlot, MQTT_SLOTS, mqttSlot()

/**
 * Native on-node MQTT publisher for the observer packet feed (Phase B1).
 *
 * Subclasses `Stream` (write-only — reads are inert no-ops) so the SAME
 * `meshconsole::logRx/logTx` formatter (which takes a `Stream&`) that feeds the TCP
 * console (helpers/PacketLogConsole.h) can write byte-identical lines here — the
 * MyMesh logRx/logTx/logRxRaw hooks just tee a second call to MqttPub under
 * `#ifdef WITH_NET_BRIDGE`. Bytes accumulate in a line buffer; each completed line
 * ('\n') is published to `<base>/raw` on every connected broker, where `<base>` is
 * that slot's operator-set `topic` (NO presets ship — deliberate). A retained
 * `<base>/status` carries `online`, with `offline` as the MQTT Last Will.
 *
 * Multi-broker: up to MQTT_SLOTS independent targets, each its own EthernetClient +
 * MQTTClient + reconnect state. Resilience per slot: exponential backoff and a
 * circuit breaker that stops reconnect storms after sustained failure (re-armed via
 * `mqtt reset`). When NO slot is connected, completed lines spill into a fixed-size
 * ring buffer and flush on reconnect (drop-oldest on overflow, staleness-flush after
 * QUEUE_STALE_MS).
 *
 * Milestone 1 (this block): plain MQTT 1883 + user/pass over the W5100S
 * `EthernetClient`. TLS (8883) is deferred to B1 milestone 3 (SSLClient/BearSSL):
 * the per-slot `Client&` handed to MQTTClient::begin() is the swap point. Milestone 2
 * will publish analyzer-spec JSON to `<base>/packets`; today's raw lines mirror the
 * `<base>/raw` feed the Go mcbridge already emits.
 *
 * Shares the W5100S the console brought up — every broker op gates on
 * EthConsole.isReady(). The feed is gated upstream on cliext::g_packet_dump_enabled
 * (`log on|off`), so MQTT and the TCP console silence together.
 */

// Per-line cap for the queue + line buffer; a packet-log line is ~150 chars.
#ifndef MQTT_LINE_MAX
#define MQTT_LINE_MAX 192
#endif
// Offline ring-buffer RAM budget. Entries = budget / MQTT_LINE_MAX (~42 @ 8 KB).
// Tune for the platform (nRF52840: 256 KB − SoftDevice ~64 KB − W5100S/BLE); begin()
// logs free heap so headroom is visible. Fixed static storage — no heap churn.
#ifndef MQTT_QUEUE_BYTES
#define MQTT_QUEUE_BYTES 8192
#endif

class MqttPublisher : public Stream {
public:
  static const int QUEUE_ENTRIES = MQTT_QUEUE_BYTES / MQTT_LINE_MAX;

  // Bind each slot's MQTTClient to its EthernetClient (once) and apply config. No-op
  // for unprovisioned slots. Call at boot, after EthConsole.begin().
  void begin();

  // Service every slot (keepalive + one throttled reconnect attempt) and drain the
  // offline queue to connected brokers. Call every main loop().
  void loop();

  // Re-read cliext config into the live slots (after `set mqtt*`). Drops connections
  // so changed credentials/host take effect, and re-arms any tripped breaker.
  void reconfigure();

  // CLI introspection / control (used by cliext `get mqtt*` / `mqtt reset`).
  bool slotConnected(int i);
  bool slotTripped(int i) const;
  void resetSlot(int i);          // i < 0 → all slots
  int  queueDepth() const { return _q_count; }

  // Stream write side: accumulate into a line buffer, publish/queue on newline.
  size_t write(uint8_t c) override;
  size_t write(const uint8_t* buf, size_t size) override;

  // Stream read side: write-only sink, reads are inert.
  int available() override { return 0; }
  int read() override      { return -1; }
  int peek() override      { return -1; }
  void flush() override    {}

private:
  struct Slot {
    EthernetClient net;
    MQTTClient     mqtt{512};      // 512 B buffers (topic + ~150-char line)
    bool     active   = false;     // enabled + provisioned snapshot
    uint8_t  backoff  = 0;         // index into BACKOFF_MS
    uint8_t  max_fails = 0;        // consecutive failures while at max backoff
    bool     tripped  = false;     // circuit breaker open
    uint32_t next_attempt = 0;     // millis() gate for the next connect
    bool     began    = false;     // MQTTClient::begin() done (buffers allocated)
    char     topic_raw[MQTT_LINE_MAX];
    char     topic_status[MQTT_LINE_MAX];
    char     client_id[20];
  };

  void applyConfig();                          // (re)load cliext slots into _slots
  bool serviceConnect(int i, uint32_t now);    // one throttled connect attempt
  void publishLine(const char* line);          // to all connected slots
  bool anyConnected();
  void drainQueue();
  void enqueue(const char* line);

  Slot     _slots[MQTT_SLOTS];
  uint8_t  _attempt_cursor = 0;                // round-robins reconnect attempts

  char     _line[MQTT_LINE_MAX];
  size_t   _len = 0;

  char     _queue[QUEUE_ENTRIES][MQTT_LINE_MAX];
  uint16_t _q_head = 0, _q_tail = 0, _q_count = 0;
  uint32_t _disconnected_since = 0;            // for staleness flush (0 = connected)
};

extern MqttPublisher MqttPub;

#endif  // WITH_NET_BRIDGE
