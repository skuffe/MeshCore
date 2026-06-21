#pragma once

// Console packet-log formatter — emits the RX/TX summary lines that upstream
// prints from Dispatcher.cpp's `#if MESH_PACKET_LOGGING` block, but from the
// `logRx()`/`logTx()` virtual hooks instead. This keeps src/Dispatcher.cpp 100%
// pristine (no fork edit there → no rebase conflict on its log format strings);
// our observer roles just override the hooks and call these helpers.
//
// The line format is copied VERBATIM from upstream Dispatcher.cpp so the wire
// format the mcbridge parses is byte-identical. The hooks receive the same
// (pkt, len, score) the upstream block had, and `air_time` is recomputed the
// same way (`radio.getEstAirtimeFor(len)`, len == pkt->getRawLength()).

#include <Arduino.h>
#include <Mesh.h>

namespace meshconsole {

inline bool _isAddrPair(uint8_t type) {
  return type == PAYLOAD_TYPE_PATH || type == PAYLOAD_TYPE_REQ
      || type == PAYLOAD_TYPE_RESPONSE || type == PAYLOAD_TYPE_TXT_MSG;
}

// Mirrors the upstream RX log line (Dispatcher::checkRecv).
inline void logRx(Stream& out, const char* datetime, mesh::Radio& radio,
                  mesh::Packet* pkt, int len, float score) {
  uint32_t air_time = radio.getEstAirtimeFor(len);
  out.print(datetime);
  out.printf(": RX, len=%d (type=%d, route=%s, payload_len=%d) SNR=%d RSSI=%d score=%d time=%d",
             len, pkt->getPayloadType(), pkt->isRouteDirect() ? "D" : "F", pkt->payload_len,
             (int)pkt->getSNR(), (int)radio.getLastRSSI(), (int)(score * 1000), air_time);

  static uint8_t packet_hash[MAX_HASH_SIZE];
  pkt->calculatePacketHash(packet_hash);
  out.print(" hash=");
  mesh::Utils::printHex(out, packet_hash, MAX_HASH_SIZE);

  if (_isAddrPair(pkt->getPayloadType())) {
    out.printf(" [%02X -> %02X]\n", (uint32_t)pkt->payload[1], (uint32_t)pkt->payload[0]);
  } else {
    out.printf("\n");
  }
}

// Mirrors the upstream TX log line (Dispatcher::checkSend).
inline void logTx(Stream& out, const char* datetime, mesh::Packet* pkt, int len) {
  out.print(datetime);
  out.printf(": TX, len=%d (type=%d, route=%s, payload_len=%d)",
             len, pkt->getPayloadType(), pkt->isRouteDirect() ? "D" : "F", pkt->payload_len);
  if (_isAddrPair(pkt->getPayloadType())) {
    out.printf(" [%02X -> %02X]\n", (uint32_t)pkt->payload[1], (uint32_t)pkt->payload[0]);
  } else {
    out.printf("\n");
  }
}

}  // namespace meshconsole
