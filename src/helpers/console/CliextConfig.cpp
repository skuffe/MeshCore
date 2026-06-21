#include "CliextConfig.h"
#include <string.h>

namespace cliext {

static const char*    CONFIG_PATH    = "/cliext_cfg";
static const uint32_t CONFIG_MAGIC   = 0x43584C43;  // 'CLXC'
static const uint16_t CONFIG_VERSION = 1;

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
  // all mqtt_* left zero/empty == unset (no presets)
}

Config& config() { return _cfg; }

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
