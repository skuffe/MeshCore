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

struct Config {
  // Runtime packet-observation toggle (`log on|off`), made durable. Tri-state:
  //   -1 = unset  → fall back to the WITH_OBSERVER build seed at boot
  //    0 = off    → last `log off`
  //    1 = on     → last `log on`
  int8_t   packet_dump;

  // Native MQTT publish (Phase B1). Defaults are empty/zero == "unset": NO broker
  // presets ship in firmware (deliberate operator requirement). These can be
  // pre-provisioned via `set mqtt.*` before the B1 publisher block lands.
  char     mqtt_host[64];
  uint16_t mqtt_port;
  char     mqtt_user[32];
  char     mqtt_pass[64];
  char     mqtt_topic[80];
  uint8_t  mqtt_tls;       // 0/1 — secure transport (8883); off until B1 TLS lands
  uint8_t  mqtt_enabled;   // 0/1 — master enable for the on-node publisher
};

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
