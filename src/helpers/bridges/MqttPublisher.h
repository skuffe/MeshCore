#pragma once

#ifdef WITH_NET_BRIDGE

#include <Arduino.h>
#include <RAK13800_W5100S.h>
#include <MQTT.h>                              // 256dpi/arduino-mqtt (lwmqtt)
#include <Mesh.h>
#include <helpers/console/CliextConfig.h>      // MqttSlot, MQTT_SLOTS, mqttSlot()
#include <helpers/bridges/ObserverJson.h>      // analyzer-spec JSON builders

/**
 * Native on-node MQTT publisher for the observer (Phase B1).
 *
 * Publishes the analyzer-spec JSON (docs/analyzer-spec.md, ported from agessaman) to
 * up to MQTT_SLOTS brokers in parallel from the relay gateway (Sortsnak /
 * WITH_NET_BRIDGE):
 *   <base>/packets  per-packet JSON   (RX gated on msg_rx, TX on msg_tx mode)
 *   <base>/raw      raw-radio JSON    (gated on msg_raw — heavy, default off)
 *   <base>/status   retained status JSON, periodic + on-connect; offline as the LWT
 * <base> is each slot's operator-set `topic` (no presets ship). The MyMesh
 * logRx/logTx/logRxRaw hooks drive onRawRx/onPacketRx/onPacketTx; JSON is built from
 * the packet (not by parsing the console line), so the TCP console feed is untouched.
 *
 * Transport: lwmqtt over the W5100S EthernetClient, one MQTTClient per slot. M3 swaps
 * EthernetClient → SSLClient (TLS 8883) per slot. Resilience: per-slot exponential
 * backoff + circuit breaker (re-armed via `mqtt reset`). When no slot is connected,
 * serialized messages spill to a fixed-size ring buffer and flush on reconnect
 * (drop-oldest, staleness flush). Gated upstream on cliext::g_packet_dump_enabled
 * (`log on|off`); shares EthConsole's W5100S lease (gates on isReady()).
 */

// Max serialized message (JSON) — a packet with a full ~255-byte raw hex (510 chars)
// plus fields + path. Also the per-slot MQTTClient buffer floor (topic + payload).
#ifndef MQTT_MSG_MAX
#define MQTT_MSG_MAX 1024
#endif
// Offline ring-buffer RAM budget. Entries = budget / sizeof(QEntry) (~16 @ 16 KB).
// Fixed static storage — no heap churn (agessaman lesson). begin() logs free heap.
#ifndef MQTT_QUEUE_BYTES
#define MQTT_QUEUE_BYTES 16384
#endif

class MqttPublisher {
public:
  enum MsgType : uint8_t { MSG_PACKETS = 0, MSG_RAW = 1 };

  // Identity/clock context for JSON. Pointers are borrowed and must stay valid (origin
  // = prefs node_name; the rest are build-time strings / a static pubkey buffer).
  void setContext(mesh::RTCClock* rtc, const char* origin, const char* origin_id,
                  const char* model, const char* firmware, const char* radio,
                  const char* client_version);

  void begin();          // bind clients + apply config; call after EthConsole.begin()
  void loop();           // keepalive, throttled reconnect, queue drain, periodic status
  void reconfigure();    // re-read cliext config (after `set mqtt*`), re-arm breakers

  // Feed hooks (from MyMesh, all under WITH_NET_BRIDGE):
  void onRawRx(const uint8_t* raw, int len, float snr, float rssi); // logRxRaw
  void onPacketRx(mesh::Packet* pkt, float score);                  // logRx
  void onPacketTx(mesh::Packet* pkt);                               // logTx

  // CLI (cliext `get mqtt*` / `mqtt reset`).
  bool slotConnected(int i);
  bool slotTripped(int i) const;
  void resetSlot(int i);          // i < 0 → all
  int  queueDepth() const { return _q_count; }

private:
  struct Slot {
    EthernetClient net;
    MQTTClient     mqtt{MQTT_MSG_MAX + 256};   // hold topic + a full JSON payload
    bool     active   = false;
    uint8_t  backoff  = 0;
    uint8_t  max_fails = 0;
    bool     tripped  = false;
    uint32_t next_attempt = 0;
    bool     began    = false;
    char     topic_packets[96];
    char     topic_raw[96];
    char     topic_status[96];
    char     client_id[20];
  };

  struct QEntry { uint8_t type; char json[MQTT_MSG_MAX]; };

  ObserverJson::Ctx makeCtx();
  void applyConfig();
  bool serviceConnect(int i, uint32_t now);    // one throttled connect attempt
  void publishStatusToSlot(int i, bool online);
  void publishLive(const char* topic_suffix, MsgType type, const char* json);
  bool anyConnected();
  void drainQueue();
  void enqueue(MsgType type, const char* json);
  void maybePeriodicStatus(uint32_t now);

  mesh::RTCClock* _rtc = nullptr;
  const char* _origin = "";
  const char* _origin_id = "";
  const char* _model = "";
  const char* _firmware = "";
  const char* _radio = "";
  const char* _client_version = "";

  Slot     _slots[MQTT_SLOTS];
  uint8_t  _attempt_cursor = 0;
  uint32_t _last_status_ms = 0;

  // RX raw staging: onRawRx() stashes the radio bytes + metrics, onPacketRx() consumes
  // them for the packet JSON's raw/SNR/RSSI (mirrors agessaman storeRawRadioData).
  uint8_t  _staged_raw[256];
  int      _staged_len = 0;
  float    _staged_snr = 0, _staged_rssi = 0;
  bool     _staged_valid = false;

  static const int QUEUE_ENTRIES = MQTT_QUEUE_BYTES / (int)sizeof(QEntry);
  QEntry   _queue[QUEUE_ENTRIES];
  uint16_t _q_head = 0, _q_tail = 0, _q_count = 0;
  uint32_t _disconnected_since = 0;
};

extern MqttPublisher MqttPub;

#endif  // WITH_NET_BRIDGE
