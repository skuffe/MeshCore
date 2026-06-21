#pragma once

#ifdef WITH_NET_BRIDGE

#include <Arduino.h>
#include <Mesh.h>

// Analyzer-spec MQTT JSON builders (Phase B1 M2). A trimmed nRF52 port of agessaman's
// helpers/MQTTMessageBuilder.cpp — field names, ordering, string-typed numerics, hex
// casing (raw/hash UPPER, path lower), UTC time/date formats and the
// "%Y-%m-%dT%H:%M:%S.000000+00:00" timestamp are kept byte-compatible so the same
// analyzer/meshview consumers parse it. See docs/analyzer-spec.md. ESP32 specifics
// (PSRAM/JWT/NTP/FreeRTOS) are dropped; time comes from the mesh RTC (epoch seconds),
// sub-second is always .000000 (the node has no wall-clock sub-second source).
namespace ObserverJson {

// Caller-owned context shared by every message. Strings are borrowed pointers that must
// outlive the build call (the MqttPublisher keeps stable copies). Stats use -1 to omit.
struct Ctx {
  const char* origin;          // node name
  const char* origin_id;       // 64-hex public key (UPPER)
  const char* model;           // board model
  const char* firmware;        // firmware version
  const char* radio;           // radio descriptor
  const char* client_version;  // "<name>/<version>"
  uint32_t    epoch;           // current UTC epoch seconds (mesh RTC)
  int         uptime_secs;     // stats; -1 omits
  int         internal_heap;
  int         queue_len;
};

// status JSON → <base>/status (retained). `online` selects status:"online"/"offline".
// Returns serialized length (0 on overflow).
size_t buildStatus(char* buf, size_t cap, const Ctx& c, bool online);

// packets JSON → <base>/packets. For RX pass the raw radio bytes (raw/raw_len, incl
// headers) + snr/rssi/score; for TX pass raw=nullptr (uses Packet::writeTo) and is_tx.
size_t buildPacket(char* buf, size_t cap, const Ctx& c, mesh::Packet* pkt, bool is_tx,
                   const uint8_t* raw, int raw_len, float snr, float rssi, float score);

// raw JSON → <base>/raw (type:"RAW", data = radio bytes hex incl headers).
size_t buildRaw(char* buf, size_t cap, const Ctx& c, const uint8_t* raw, int raw_len);

}  // namespace ObserverJson

#endif  // WITH_NET_BRIDGE
