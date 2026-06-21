#pragma once

#include <Arduino.h>
#include <helpers/IdentityStore.h>   // brings in the FILESYSTEM macro + LittleFS File API

// cliext-owned durable config store. Parallel to upstream's NodePrefs/`/com_prefs`,
// but kept in a SEPARATE file (`/cliext_cfg`) so upstream `CommonCLI.h` stays byte-
// pristine (extending NodePrefs would diverge it and break cheap rebases). Holds the
// cliext runtime options that must survive reboots AND reflashes — the LittleFS
// region is preserved across DFU, which is also why node identity survives a flash.
//
// On-disk layout: a small `Header` (magic + version + body length) followed by the
// `Config` body. The body is APPEND-ONLY: never reorder, resize, or delete an
// existing field — only append new fields at the end and bump CONFIG_VERSION. Load
// reads min(on-disk body length, this build's struct size), so an older file loads
// into newer firmware (missing tail fields keep their reset() defaults) and a newer
// file loads into older firmware (extra trailing fields are ignored). A factory
// reset / FS format wipes it — acceptable, same contract as node identity.
namespace cliext {

// Number of independent MQTT broker targets the node can publish to in parallel
// (Phase B1). Each costs one W5100S socket; budget against EthConsole + BleRelay
// (~4 sockets) within the chip's 8. Override per-env via `-D MQTT_SLOTS=N`.
#ifndef MQTT_SLOTS
#define MQTT_SLOTS 3
#endif

// One MQTT broker target. All empty/zero == "unset": NO presets ship in firmware
// (deliberate operator requirement). Provisioned via `set mqtt<N>.<field>`.
struct MqttSlot {
  char     host[64];
  uint16_t port;
  char     user[32];
  char     pass[64];
  char     topic[80];      // base topic; the publisher appends /raw, /status, ...
  uint8_t  tls;            // 0/1 — secure transport (8883); off until B1 TLS lands
  uint8_t  enabled;        // 0/1 — per-slot publish enable
};

struct Config {
  // Runtime packet-observation toggle (`log on|off`), made durable. Tri-state:
  //   -1 = unset  → fall back to the WITH_OBSERVER build seed at boot
  //    0 = off    → last `log off`
  //    1 = on     → last `log on`
  int8_t   packet_dump;

  // LEGACY single-broker fields (B1 pre-provisioning, pre-multi-slot). Retained at
  // this offset for the on-disk cross-load contract — do NOT remove or reorder. New
  // firmware MIGRATES these into slots[0] at load (configBegin) and then sources all
  // runtime access from slots[]; these stay only so an older build can still read its
  // own broker config out of a file this build wrote.
  char     mqtt_host[64];
  uint16_t mqtt_port;
  char     mqtt_user[32];
  char     mqtt_pass[64];
  char     mqtt_topic[80];
  uint8_t  mqtt_tls;
  uint8_t  mqtt_enabled;

  // Multi-broker slots, APPENDED after the legacy block (append-only contract).
  // slots[0] supersedes the legacy fields above; slots[1..] are net-new.
  MqttSlot slots[MQTT_SLOTS];
};

// Live broker slot i (0..MQTT_SLOTS-1). Runtime access goes through here, never the
// legacy mqtt_* fields (which are migration-source only).
MqttSlot& mqttSlot(int i);
inline int mqttSlotCount() { return MQTT_SLOTS; }

// Reset the in-RAM config to defaults ("unset" everywhere). Called by configBegin
// before a load; also the post-condition when no config file exists yet.
void    configReset();

// Bind the filesystem and load `/cliext_cfg` if present (else keep defaults). Call
// once at boot, after the filesystem is up, before any consumer reads the config.
void    configBegin(FILESYSTEM* fs);

// Persist the current in-RAM config to `/cliext_cfg`. Returns false on FS error or
// if the store was never bound. Call after any mutation that must be durable.
bool    configSave();

// The live in-RAM config (valid after configReset/configBegin).
Config& config();

}  // namespace cliext
