#pragma once

#include <Arduino.h>
#include <string.h>

// In-band structured backhaul framing, shared by both ends of the BLE NUS link
// (the mast peripheral that emits frames and the central relay that demuxes them).
//
// The NUS pipe already carries the human-readable mast console (CLI replies + the
// MESH_PACKET_LOGGING text feed, forwarded to TCP :5001). We multiplex structured
// records onto the SAME pipe by prefixing each frame with a control byte (0x1E, the
// ASCII Record Separator) that never appears in the printable console stream — so a
// byte-stream demux can split the two: printable bytes pass through to the text
// console, 0x1E-led frames are decoded here. We own the mast firmware, so this carries
// the full lossless observation (raw radio bytes + metrics) instead of regex-parsing
// the lossy text line (which has integer-only SNR and no full path).
//
// Wire format:  [0x1E][type:1][len_lo][len_hi][payload: len bytes][checksum:1]
//   checksum = XOR of type, both length bytes, and every payload byte.
//   len is little-endian; payloads stay well under 64 KB (an observation is <300 B).
namespace backhaul {

static const uint8_t FRAME_MARK = 0x1E;

enum FrameType : uint8_t {
  FRAME_OBSERVATION = 1,   // a packet observation: ObsHeader + wire bytes (see below)
  FRAME_IDENTITY    = 2,   // pubkey[32] + name (UTF-8, not NUL-terminated) — node identity
  FRAME_STATUS      = 3,   // StatusBody — periodic node stats
  FRAME_TIME        = 4,   // TimeBody — central→mast UTC epoch push (backhaul time sync)
};

// Epochs below this (2023-11-14) are treated as "clock not yet synced" — the central then
// falls back to its own NTP clock when stamping, and the mast ignores an implausible push.
static const uint32_t EPOCH_SANE_MIN = 1700000000UL;

// FRAME_OBSERVATION payload = this fixed header followed by `wire_len` packet bytes.
// `wire` is the on-air radio bytes for an RX (so the central reproduces the exact `raw`
// hex + length) or Packet::writeTo() output for a TX. The central reconstructs a
// mesh::Packet via readFrom(wire) and runs the SAME ObserverJson::buildPacket() the
// local path uses — one builder, identical spec output. snr/rssi/score are floats
// (both ends are little-endian ARM); score is NAN for TX / when unscored.
struct __attribute__((packed)) ObsHeader {
  uint8_t  is_tx;
  uint8_t  _pad;
  uint32_t epoch;      // observer's RTC at OBSERVE time (the spec's "observer moment"); the
                       // central stamps the JSON with this, not its own publish-time clock,
                       // so a cached/late observation keeps its true time. < EPOCH_SANE_MIN
                       // (mast clock unsynced) → central falls back to its own NTP clock.
  float    snr;
  float    rssi;
  float    score;
  uint8_t  wire_len;
};

// FRAME_STATUS payload.
struct __attribute__((packed)) StatusBody {
  uint32_t uptime_secs;
};

// FRAME_TIME payload — central pushes its NTP-synced UTC epoch to the mast over the
// backhaul so the non-network node can stamp observations with real time (enableable
// service). The mast applies it via RTCClock::setCurrentTime().
struct __attribute__((packed)) TimeBody {
  uint32_t epoch;
};

// Encode a frame into `out` (cap bytes). Returns total bytes written, or 0 on overflow.
inline size_t encode(uint8_t* out, size_t cap, uint8_t type,
                     const uint8_t* payload, uint16_t len) {
  size_t total = 5 + (size_t)len;   // mark+type+len2 + payload + cksum
  if (cap < total) return 0;
  out[0] = FRAME_MARK;
  out[1] = type;
  out[2] = (uint8_t)(len & 0xFF);
  out[3] = (uint8_t)(len >> 8);
  uint8_t ck = out[1] ^ out[2] ^ out[3];
  for (uint16_t i = 0; i < len; i++) { out[4 + i] = payload[i]; ck ^= payload[i]; }
  out[4 + len] = ck;
  return total;
}

// Byte-fed demux for the central. Feed each NUS byte; PASS bytes belong to the text
// console (forward them), EAT bytes are consumed inside a (possible) frame, and FRAME
// signals a complete, checksum-verified frame is available via type()/payload()/len().
// A stray 0x1E in text would start a false frame; the console feed is plain ASCII so
// this does not occur, and a checksum mismatch simply drops the bogus frame's bytes.
class Parser {
public:
  enum Result { PASS, EAT, FRAME };

  static const uint16_t MAX_PAYLOAD = 320;   // ObsHeader(15) + up to 255 wire bytes + slack

  Result feed(uint8_t b) {
    switch (_state) {
      case S_SEEK:
        if (b == FRAME_MARK) { _state = S_TYPE; return EAT; }
        return PASS;
      case S_TYPE:    _type = b; _ck = b; _state = S_LEN_LO; return EAT;
      case S_LEN_LO:  _len = b; _ck ^= b; _state = S_LEN_HI; return EAT;
      case S_LEN_HI:
        _len |= (uint16_t)b << 8; _ck ^= b; _got = 0;
        if (_len > MAX_PAYLOAD) { _state = S_SEEK; return EAT; }   // bogus → resync
        _state = _len ? S_PAYLOAD : S_CKSUM;
        return EAT;
      case S_PAYLOAD:
        _buf[_got++] = b; _ck ^= b;
        if (_got >= _len) _state = S_CKSUM;
        return EAT;
      case S_CKSUM:
        _state = S_SEEK;
        return (b == _ck) ? FRAME : EAT;   // bad checksum → drop frame
    }
    _state = S_SEEK;
    return EAT;
  }

  uint8_t        type() const    { return _type; }
  const uint8_t* payload() const { return _buf; }
  uint16_t       len() const     { return _len; }

private:
  enum State { S_SEEK, S_TYPE, S_LEN_LO, S_LEN_HI, S_PAYLOAD, S_CKSUM };
  State    _state = S_SEEK;
  uint8_t  _type = 0, _ck = 0;
  uint16_t _len = 0, _got = 0;
  uint8_t  _buf[MAX_PAYLOAD];
};

}  // namespace backhaul
