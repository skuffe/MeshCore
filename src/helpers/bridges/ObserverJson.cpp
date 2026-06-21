#include "ObserverJson.h"

#ifdef WITH_NET_BRIDGE

#include <ArduinoJson.h>
#include <time.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

namespace ObserverJson {

static void bytesToHexUpper(const uint8_t* data, size_t len, char* hex, size_t cap) {
  if (cap < len * 2 + 1) { if (cap) hex[0] = 0; return; }
  for (size_t i = 0; i < len; i++) snprintf(hex + i * 2, 3, "%02X", data[i]);
  hex[len * 2] = 0;
}

// ISO-8601 UTC with explicit offset + zero sub-second, matching Python
// datetime.now(timezone.utc).isoformat() — the analyzer-spec `timestamp` everywhere.
static void isoTimestamp(uint32_t epoch, char* buf, size_t cap) {
  time_t now = (time_t)epoch;
  struct tm* g = gmtime(&now);
  if (g) {
    size_t n = strftime(buf, cap, "%Y-%m-%dT%H:%M:%S", g);
    if (n > 0 && (size_t)snprintf(buf + n, cap - n, ".000000+00:00") < cap - n) return;
  }
  strncpy(buf, "1970-01-01T00:00:00.000000+00:00", cap - 1);
  buf[cap - 1] = 0;
}

static void addStats(JsonObject& root, const Ctx& c) {
  if (c.uptime_secs < 0 && c.internal_heap < 0 && c.queue_len < 0) return;
  JsonObject st = root["stats"].to<JsonObject>();
  if (c.uptime_secs   >= 0) st["uptime_secs"]   = c.uptime_secs;
  if (c.queue_len     >= 0) st["queue_len"]     = c.queue_len;
  if (c.internal_heap >= 0) st["internal_heap"] = c.internal_heap;
}

size_t buildStatus(char* buf, size_t cap, const Ctx& c, bool online) {
  char ts[40];
  isoTimestamp(c.epoch, ts, sizeof(ts));

  JsonDocument doc;
  JsonObject root = doc.to<JsonObject>();
  root["status"]           = online ? "online" : "offline";
  root["timestamp"]        = ts;
  root["origin"]           = c.origin;
  root["origin_id"]        = c.origin_id;
  root["model"]            = c.model;
  root["firmware_version"] = c.firmware;
  root["radio"]            = c.radio;
  root["client_version"]   = c.client_version;
  addStats(root, c);

  size_t n = serializeJson(doc, buf, cap);
  return (n > 0 && n < cap) ? n : 0;
}

size_t buildPacket(char* buf, size_t cap, const Ctx& c, mesh::Packet* pkt, bool is_tx,
                   const uint8_t* raw, int raw_len, float snr, float rssi, float score) {
  if (!pkt) return 0;

  char ts[40], time_str[16], date_str[16];
  isoTimestamp(c.epoch, ts, sizeof(ts));
  time_t now = (time_t)c.epoch;
  struct tm* g = gmtime(&now);
  if (g) {
    strftime(time_str, sizeof(time_str), "%H:%M:%S", g);
    strftime(date_str, sizeof(date_str), "%d/%m/%Y", g);
  } else { strcpy(time_str, "00:00:00"); strcpy(date_str, "01/01/1970"); }

  // raw hex: RX uses the captured radio bytes (incl headers); TX serializes the packet.
  char raw_hex[2 * 256 + 1];
  int  len_field;
  if (raw && raw_len > 0) {
    bytesToHexUpper(raw, (size_t)raw_len, raw_hex, sizeof(raw_hex));
    len_field = raw_len;
  } else {
    uint8_t wbuf[256];
    uint8_t wlen = pkt->writeTo(wbuf);
    bytesToHexUpper(wbuf, wlen, raw_hex, sizeof(raw_hex));
    len_field = pkt->getRawLength();
  }

  char hash_str[2 * MAX_HASH_SIZE + 1];
  uint8_t hash[MAX_HASH_SIZE];
  pkt->calculatePacketHash(hash);
  bytesToHexUpper(hash, MAX_HASH_SIZE, hash_str, sizeof(hash_str));

  // Numeric packet fields are serialized as STRINGS (analyzer-spec).
  char len_s[12], type_s[12], plen_s[12], snr_s[12], rssi_s[12], score_s[12];
  snprintf(len_s,  sizeof(len_s),  "%d", len_field);
  snprintf(type_s, sizeof(type_s), "%d", pkt->getPayloadType());
  snprintf(plen_s, sizeof(plen_s), "%d", pkt->payload_len);
  const char* route = pkt->isRouteDirect() ? "D" : "F";

  JsonDocument doc;
  JsonObject root = doc.to<JsonObject>();
  root["timestamp"]   = ts;
  root["hash"]        = hash_str;
  root["origin"]      = c.origin;
  root["type"]        = "PACKET";
  root["direction"]   = is_tx ? "tx" : "rx";
  root["time"]        = time_str;
  root["date"]        = date_str;
  root["len"]         = len_s;
  root["packet_type"] = type_s;
  root["route"]       = route;
  root["payload_len"] = plen_s;
  root["raw"]         = raw_hex;
  root["origin_id"]   = c.origin_id;
  if (!is_tx) {                                  // SNR/RSSI/score only on RX
    snprintf(snr_s,  sizeof(snr_s),  "%.1f", snr);
    snprintf(rssi_s, sizeof(rssi_s), "%d",   (int)rssi);
    root["SNR"]  = snr_s;
    root["RSSI"] = rssi_s;
    if (!isnan(score)) {
      snprintf(score_s, sizeof(score_s), "%d", (int)(score * 1000));
      root["score"] = score_s;
    }
  }
  // Routing path (direct packets that carry hops): array of lowercase hex hop tokens.
  if (pkt->isRouteDirect() && pkt->getPathHashCount() > 0) {
    int hops = pkt->getPathHashCount();
    int hsz  = pkt->getPathHashSize();
    JsonArray pa = root["path"].to<JsonArray>();
    char hop[2 * 4 + 1];
    for (int i = 0; i < hops; i++) {
      size_t pos = 0;
      for (int b = 0; b < hsz && b < 4; b++) {
        size_t idx = (size_t)i * hsz + b;
        if (idx >= MAX_PATH_SIZE) break;
        snprintf(hop + pos, 3, "%02x", pkt->path[idx]); pos += 2;
      }
      hop[pos] = 0;
      pa.add((const char*)hop);
    }
  }

  size_t n = serializeJson(doc, buf, cap);
  return (n > 0 && n < cap) ? n : 0;
}

size_t buildRaw(char* buf, size_t cap, const Ctx& c, const uint8_t* raw, int raw_len) {
  if (!raw || raw_len <= 0) return 0;
  char ts[40];
  isoTimestamp(c.epoch, ts, sizeof(ts));
  char raw_hex[2 * 256 + 1];
  bytesToHexUpper(raw, (size_t)raw_len, raw_hex, sizeof(raw_hex));

  JsonDocument doc;
  JsonObject root = doc.to<JsonObject>();
  root["origin"]    = c.origin;
  root["origin_id"] = c.origin_id;
  root["timestamp"] = ts;
  root["type"]      = "RAW";
  root["data"]      = raw_hex;

  size_t n = serializeJson(doc, buf, cap);
  return (n > 0 && n < cap) ? n : 0;
}

}  // namespace ObserverJson

#endif  // WITH_NET_BRIDGE
