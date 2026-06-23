#pragma once

#include <Mesh.h>

// ONE observation-emission mechanism for both node roles. The room_server (Sortsnak)
// and the repeater_ble_gw (mast) logRx/logTx/logRxRaw hooks both build the same
// observation and hand it here; the only difference is the sink, chosen at compile time
// by capability:
//
//   WITH_NET_BRIDGE          → publish locally via MqttPub as observers[0] (self).
//   WITH_BACKHAUL_PERIPHERAL → frame the observation over the BLE NUS backhaul; the
//                              central decodes it and runs the SAME publish path under
//                              the mast's observer identity. "Relayed" is purely transport.
//
// A node is one or the other (the central is NET_BRIDGE + BACKHAUL_CENTRAL, never
// PERIPHERAL), so the routing is mutually exclusive. Neither flag → no-ops (the feed
// still rides the existing text console untouched).
namespace observer {

// RX raw radio bytes (+ metrics) for the imminent packet. Staged for onPacketRx so the
// observation/JSON carries the exact on-air `raw` hex, SNR, RSSI (mirrors the local
// MqttPublisher staging).
void onRawRx(const uint8_t* raw, int len, float snr, float rssi);

// A received / transmitted packet observation.
void onPacketRx(mesh::Packet* pkt, float score);
void onPacketTx(mesh::Packet* pkt);

#ifdef WITH_BACKHAUL_PERIPHERAL
// Bind the node's RTC (for observe-time stamping + applying backhaul time pushes) and its
// identity (pubkey + name + hardware: model/firmware/radio, framed to the central so it
// auto-populates this relay observer's status JSON). Call once at boot.
void begin(mesh::RTCClock* rtc, const uint8_t* pubkey, const char* name,
           const char* model, const char* firmware, const char* radio);

// Drive the backhaul side effects: announce IDENTITY (burst after connect, then periodic),
// periodic STATUS, and drain the offline observation cache to the link when up. Call from
// the mast main loop.
void loop();

// Feed one byte received on the backhaul (NUS) console. Returns true if the byte was part
// of a structured frame (FRAME_TIME clock push or a FRAME_CONSOLE remote-admin command)
// and was consumed; false if it is ordinary console text the caller should hand to the CLI.
// Lets the mast demux central→mast frames out of its inbound CLI stream.
bool feedBackhaulByte(uint8_t b);

// Remote-admin handler: runs a console command line and writes the reply into `reply`
// (capacity `cap`). The mast sets this to its CLI dispatcher so a central can run commands
// over the backhaul (FRAME_CONSOLE) — the replacement for the dropped :5001 passthrough.
typedef void (*ConsoleHandler)(const char* cmd, char* reply, size_t cap);
void setConsoleHandler(ConsoleHandler fn);
#endif

}  // namespace observer
