#pragma once

#ifdef WITH_NET_BRIDGE

#include <Arduino.h>
#include <RAK13800_W5100S.h>
#include <MQTT.h>                              // 256dpi/arduino-mqtt (lwmqtt)
#include <Mesh.h>
#include <helpers/console/CliextConfig.h>      // MqttSlot, MQTT_SLOTS, mqttSlot()
#include <helpers/bridges/ObserverJson.h>      // analyzer-spec JSON builders
#include <helpers/ByteRing.h>                  // generalized offline message cache

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

  // Hardware (model/firmware/radio) for a relayed observer (obsIdx != 0), self-described by
  // the peripheral over the backhaul (FRAME_IDENTITY). Cached in RAM, refreshed each IDENTITY
  // announce; used by makeCtxFor() so a relay observer's status JSON carries real hw instead
  // of empty strings. obsIdx 0 (local) is sourced from setContext() and ignored here.
  void setObserverHw(uint8_t obsIdx, const char* model, const char* firmware, const char* radio);

  // Record a relayed observer's own uptime, reported over the backhaul (FRAME_STATUS). The
  // status JSON then carries the peripheral's uptime (extrapolated between frames), not this
  // central's. obsIdx 0 (local) uses millis() directly and is ignored here.
  void setObserverUptime(uint8_t obsIdx, uint32_t uptime_secs);

  void begin();          // bind clients + apply config; call after EthConsole.begin()
  void loop();           // keepalive, throttled reconnect, queue drain, periodic status
  void reconfigure();    // re-read cliext config (after `set mqtt*`), re-arm breakers

  // Local feed hooks (self = observers[0]; driven via helpers/Observer.h):
  void onRawRx(const uint8_t* raw, int len, float snr, float rssi); // logRxRaw
  void onPacketRx(mesh::Packet* pkt, float score);                  // logRx
  void onPacketTx(mesh::Packet* pkt);                               // logTx

  // Relayed feed (from BleNusRelay's backhaul demux): a remote observer's observation,
  // reconstructed from its framed wire bytes and published under observers[obsIdx]. `epoch`
  // is the observer's observe-time stamp from the frame — used verbatim if plausible, else
  // (mast clock unsynced) falls back to this node's NTP clock. Shares the exact local
  // publish path — only the observer identity/topic/timestamp source differ.
  void publishObservation(uint8_t obsIdx, bool is_tx, const uint8_t* wire, int wire_len,
                          float snr, float rssi, float score, uint32_t epoch);
  // Retained per-observer status (online/offline), published on backhaul link changes.
  void publishObserverStatus(uint8_t obsIdx, bool online);

  // CLI (cliext `get mqtt*` / `mqtt reset`).
  bool slotConnected(int i);
  bool slotTripped(int i) const;
  void resetSlot(int i);          // i < 0 → all
  int  queueDepth() const { return (int)_q.count(); }

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
    char     client_id[20];
  };

  uint32_t nowEpoch();   // this node's UTC epoch (NTP), tracking the last plausible value
  ObserverJson::Ctx makeCtx();
  ObserverJson::Ctx makeCtxFor(uint8_t obsIdx, uint32_t epoch);  // observer identity + observe time
  bool observerEnabled(uint8_t obsIdx);
  void buildTopic(char* out, size_t cap, int slot_i, uint8_t obsIdx, const char* suffix);
  // Build the spec JSON for a packet observation (stamped at observe time = `epoch`) and
  // publish/enqueue it for one observer.
  void emitPacket(uint8_t obsIdx, mesh::Packet* pkt, bool is_tx,
                  const uint8_t* raw, int raw_len, float snr, float rssi, float score, uint32_t epoch);
  void applyConfig();
  bool serviceConnect(int i, uint32_t now);    // one throttled connect attempt
  void publishStatusToSlot(int i, uint8_t obsIdx, bool online);
  void publishLive(uint8_t obsIdx, MsgType type, const char* json);
  bool anyConnected();
  void drainQueue();
  void enqueue(uint8_t obsIdx, MsgType type, const char* json);
  void maybePeriodicStatus(uint32_t now);

  mesh::RTCClock* _rtc = nullptr;
  const char* _origin = "";
  const char* _origin_id = "";
  const char* _model = "";
  const char* _firmware = "";
  const char* _radio = "";
  const char* _client_version = "";

  // Per-relay-observer hardware, self-described over the backhaul (see setObserverHw). Index 0
  // (local) is unused — it comes from _model/_firmware/_radio above. "" until an IDENTITY fills it.
  char _obs_model[OBSERVERS_MAX][16] = {};
  char _obs_firmware[OBSERVERS_MAX][16] = {};
  char _obs_radio[OBSERVERS_MAX][12] = {};
  // Per-relay-observer uptime (FRAME_STATUS), with the local millis() at receipt so makeCtxFor
  // extrapolates a live value between frames. _at == 0 → no STATUS seen yet (uptime omitted).
  uint32_t _obs_uptime[OBSERVERS_MAX] = {};
  uint32_t _obs_uptime_at[OBSERVERS_MAX] = {};

  Slot     _slots[MQTT_SLOTS];
  uint8_t  _attempt_cursor = 0;
  uint32_t _last_status_ms = 0;
  uint32_t _last_epoch = 0;   // last plausible UTC epoch seen (NTP or backhaul) — fallback floor

  // RX raw staging: onRawRx() stashes the radio bytes + metrics, onPacketRx() consumes
  // them for the packet JSON's raw/SNR/RSSI (mirrors agessaman storeRawRadioData).
  uint8_t  _staged_raw[256];
  int      _staged_len = 0;
  float    _staged_snr = 0, _staged_rssi = 0;
  bool     _staged_valid = false;

  // Offline message cache (generalized ByteRing): variable-length records
  // [observer:1][type:1][json...]. Drop-oldest, no time expiry — JSON is stamped at observe
  // time so a late-drained message keeps its true timestamp. Drains FIFO on broker reconnect.
  uint8_t  _q_arena[MQTT_QUEUE_BYTES];
  ByteRing _q;
};

extern MqttPublisher MqttPub;

#endif  // WITH_NET_BRIDGE
