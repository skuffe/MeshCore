#pragma once

#include <Arduino.h>
#include <string.h>

// Generalized fixed-capacity, drop-oldest ring buffer of variable-length byte records,
// over caller-provided static storage (no heap churn). The observer role uses it to ride
// out link outages: the mast caches encoded observation frames while the BLE backhaul is
// down, and the central caches serialized MQTT messages while the broker is down — one
// primitive, two independent rings (one per outage point).
//
// Records are length-prefixed in the arena ([u16 len][bytes], wrapping). When a push does
// not fit, the OLDEST records are dropped until it does — so during a long outage the ring
// always retains the freshest data. Timestamps are baked into each record at observe time
// (never the drain time), so a record keeps its correct time no matter how late it flushes;
// that is why there is no time-based expiry here — bounding is purely by capacity.
class ByteRing {
public:
  void init(uint8_t* buf, size_t cap) { _buf = buf; _cap = cap; _head = _tail = _used = _count = 0; }
  size_t count() const { return _count; }
  size_t used()  const { return _used; }
  size_t capacity() const { return _cap; }
  bool   empty() const { return _count == 0; }

  // Append a record, dropping oldest records as needed to fit. Returns false only if the
  // record is larger than the entire ring (it can never be stored).
  bool push(const uint8_t* rec, uint16_t len) {
    size_t need = (size_t)len + 2;
    if (need > _cap) return false;
    while (_cap - _used < need && _count > 0) pop();
    uint8_t hdr[2] = { (uint8_t)(len & 0xFF), (uint8_t)(len >> 8) };
    writeRaw(hdr, 2);
    writeRaw(rec, len);
    _used += need; _count++;
    return true;
  }

  // Copy the oldest record into out; returns its length, or 0 if empty or it exceeds outcap.
  uint16_t front(uint8_t* out, uint16_t outcap) const {
    if (!_count) return 0;
    uint8_t hdr[2];
    readRaw(_head, hdr, 2);
    uint16_t len = (uint16_t)hdr[0] | ((uint16_t)hdr[1] << 8);
    if (len > outcap) return 0;
    readRaw((_head + 2) % _cap, out, len);
    return len;
  }

  // Discard the oldest record.
  void pop() {
    if (!_count) return;
    uint8_t hdr[2];
    readRaw(_head, hdr, 2);
    uint16_t len = (uint16_t)hdr[0] | ((uint16_t)hdr[1] << 8);
    size_t rec = (size_t)len + 2;
    _head = (_head + rec) % _cap;
    _used -= rec; _count--;
  }

private:
  void writeRaw(const uint8_t* src, size_t n) {
    size_t first = _cap - _tail; if (first > n) first = n;
    memcpy(_buf + _tail, src, first);
    if (n > first) memcpy(_buf, src + first, n - first);
    _tail = (_tail + n) % _cap;
  }
  void readRaw(size_t pos, uint8_t* dst, size_t n) const {
    size_t first = _cap - pos; if (first > n) first = n;
    memcpy(dst, _buf + pos, first);
    if (n > first) memcpy(dst + first, _buf, n - first);
  }

  uint8_t* _buf = nullptr;
  size_t   _cap = 0, _head = 0, _tail = 0, _used = 0, _count = 0;
};
