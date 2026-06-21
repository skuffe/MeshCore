#include "CliextConfig.h"
#include <string.h>

namespace cliext {

static const char*    CONFIG_PATH    = "/cliext_cfg";
static const uint32_t CONFIG_MAGIC   = 0x43584C43;  // 'CLXC'
// v2: appended Config::slots[MQTT_SLOTS] after the legacy mqtt_* block. The append-only
// loader (read min(body_len, sizeof)) means a v1 file loads here with slots zeroed (then
// migrated from the legacy fields), and a v2 file loads into a v1 build with the slots
// tail simply ignored — no version gate needed, the bump is documentation.
static const uint16_t CONFIG_VERSION = 2;

// Fixed-size on-disk preamble. `body_len` records how many Config bytes were written
// so a newer build (larger struct) knows how much of an older, shorter file is real.
struct Header {
  uint32_t magic;
  uint16_t version;
  uint16_t body_len;
};

static FILESYSTEM* _fs = nullptr;
static Config      _cfg;

void configReset() {
  memset(&_cfg, 0, sizeof(_cfg));
  _cfg.packet_dump = -1;   // unset → WITH_OBSERVER build seed wins at boot
  // all mqtt_*/slots left zero/empty == unset (no presets)
}

Config& config() { return _cfg; }

MqttSlot& mqttSlot(int i) {
  if (i < 0) i = 0;
  if (i >= MQTT_SLOTS) i = MQTT_SLOTS - 1;
  return _cfg.slots[i];
}

// One-time migration of the pre-multi-slot legacy mqtt_* fields into slots[0]. Runs at
// load when slots[0] is still empty but a legacy broker was provisioned — so a node that
// had `set mqtt.host ...` under old firmware keeps its broker after this upgrade.
static void migrateLegacyToSlot0() {
  if (_cfg.slots[0].host[0] == 0 && _cfg.mqtt_host[0] != 0) {
    MqttSlot& s = _cfg.slots[0];
    strncpy(s.host,  _cfg.mqtt_host,  sizeof(s.host) - 1);
    strncpy(s.user,  _cfg.mqtt_user,  sizeof(s.user) - 1);
    strncpy(s.pass,  _cfg.mqtt_pass,  sizeof(s.pass) - 1);
    strncpy(s.topic, _cfg.mqtt_topic, sizeof(s.topic) - 1);
    s.port    = _cfg.mqtt_port;
    s.tls     = _cfg.mqtt_tls;
    s.enabled = _cfg.mqtt_enabled;
  }
}

// Force NUL-termination + bound the on/off bytes of one slot after a raw disk read.
static void sanitiseSlot(MqttSlot& s) {
  s.host[sizeof(s.host) - 1]   = 0;
  s.user[sizeof(s.user) - 1]   = 0;
  s.pass[sizeof(s.pass) - 1]   = 0;
  s.topic[sizeof(s.topic) - 1] = 0;
  if (s.tls > 1)     s.tls = 1;
  if (s.enabled > 1) s.enabled = 1;
}

void configBegin(FILESYSTEM* fs) {
  _fs = fs;
  configReset();
  if (!_fs) return;

#if defined(RP2040_PLATFORM)
  File f = _fs->open(CONFIG_PATH, "r");
#else
  File f = _fs->open(CONFIG_PATH);
#endif
  if (!f) return;   // no config yet — defaults stand

  Header h;
  if (f.read((uint8_t*)&h, sizeof(h)) == (int)sizeof(h) && h.magic == CONFIG_MAGIC) {
    // Append-only compatibility: read only the overlap between what the file holds
    // and what this build's struct knows. Older file → tail fields keep defaults;
    // newer file → extra trailing fields are skipped.
    uint16_t n = h.body_len;
    if (n > sizeof(_cfg)) n = sizeof(_cfg);
    f.read((uint8_t*)&_cfg, n);

    // Defensive sanitise — a corrupt/garbage file must not wedge runtime state.
    if (_cfg.packet_dump < -1 || _cfg.packet_dump > 1) _cfg.packet_dump = -1;
    _cfg.mqtt_host[sizeof(_cfg.mqtt_host) - 1]   = 0;
    _cfg.mqtt_user[sizeof(_cfg.mqtt_user) - 1]   = 0;
    _cfg.mqtt_pass[sizeof(_cfg.mqtt_pass) - 1]   = 0;
    _cfg.mqtt_topic[sizeof(_cfg.mqtt_topic) - 1] = 0;
    if (_cfg.mqtt_tls > 1)     _cfg.mqtt_tls = 1;
    if (_cfg.mqtt_enabled > 1) _cfg.mqtt_enabled = 1;
    for (int i = 0; i < MQTT_SLOTS; i++) sanitiseSlot(_cfg.slots[i]);
    migrateLegacyToSlot0();
  }
  f.close();
}

bool configSave() {
  if (!_fs) return false;

#if defined(NRF52_PLATFORM) || defined(STM32_PLATFORM)
  _fs->remove(CONFIG_PATH);
  File f = _fs->open(CONFIG_PATH, FILE_O_WRITE);
#elif defined(RP2040_PLATFORM)
  File f = _fs->open(CONFIG_PATH, "w");
#else
  File f = _fs->open(CONFIG_PATH, "w", true);
#endif
  if (!f) return false;

  Header h = { CONFIG_MAGIC, CONFIG_VERSION, (uint16_t)sizeof(_cfg) };
  f.write((uint8_t*)&h, sizeof(h));
  f.write((uint8_t*)&_cfg, sizeof(_cfg));
  f.close();
  return true;
}

}  // namespace cliext
