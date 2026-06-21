#include "CLIExtensions.h"
#include "CliextConfig.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

// The local-console reply buffer both gateway apps pass to handleCommand() is
// `char reply[160]`; bound every formatted write to that so a long `get`/`set`
// response can never overrun it.
#define CLIEXT_REPLY_CAP 160

#ifdef WITH_RAK13800_ETHERNET
  #include <helpers/bridges/EthernetTcpConsole.h>
  #include <RAK13800_W5100S.h>
#endif
#ifdef WITH_BACKHAUL_CENTRAL
  #include <helpers/bridges/BleNusRelay.h>
#endif
#ifdef WITH_BACKHAUL_PERIPHERAL
  #include <helpers/nrf52/BleConsole.h>
#endif

namespace cliext {

#ifdef WITH_OBSERVER
  // The observer role boots with the packet dump ON. A persisted `log off` in
  // /cliext_cfg (loaded by begin()) overrides this at startup.
  bool g_packet_dump_enabled = true;
#endif

#ifdef WITH_RAK13800_ETHERNET
// Deferred reboot into the bootloader's TCP DFU mode (0 = inactive). Lets the
// command reply drain to the client before the chip resets.
static uint32_t _tcpota_reboot_at = 0;
#endif

void begin(FILESYSTEM* fs) {
  configBegin(fs);   // load /cliext_cfg (or defaults)
#ifdef WITH_OBSERVER
  // Once the operator has set `log on|off`, that persisted choice overrides the
  // WITH_OBSERVER build seed; an unset (-1) store leaves the seed in place.
  if (config().packet_dump >= 0) {
    g_packet_dump_enabled = (config().packet_dump == 1);
  }
#endif
}

#ifdef WITH_NET_BRIDGE
// Bounded copy into a fixed-size config string field, always NUL-terminated.
// Only the network-bridge (MQTT) build uses these helpers.
static void setStr(char* dst, size_t cap, const char* src) {
  strncpy(dst, src, cap - 1);
  dst[cap - 1] = 0;
}

// Parse an on/off (also 1/0, true/false) token. Returns false if unrecognised.
static bool parseOnOff(const char* v, uint8_t* out) {
  if (strcmp(v, "on") == 0 || strcmp(v, "1") == 0 || strcmp(v, "true") == 0)  { *out = 1; return true; }
  if (strcmp(v, "off") == 0 || strcmp(v, "0") == 0 || strcmp(v, "false") == 0) { *out = 0; return true; }
  return false;
}
#endif  // WITH_NET_BRIDGE

bool handleCommand(const char* command, char* reply) {
  (void)command; (void)reply;

#ifdef WITH_RAK13800_ETHERNET
  if (strcmp(command, "start tcpota") == 0) {
    char ip_str[16];
    if (EthConsole.prepareTcpDfuHandoff(4444, ip_str)) {
      sprintf(reply, "TCP DFU: rebooting, send image to %s:4444", ip_str);
      _tcpota_reboot_at = millis() + 700;   // let the reply drain first
    } else {
      strcpy(reply, "Err - ethernet not up");
    }
    return true;
  }
  if (strcmp(command, "eth") == 0) {
    if (!EthConsole.isReady()) {
      strcpy(reply, "eth: down (acquiring)");
    } else {
      IPAddress ip = Ethernet.localIP();
      sprintf(reply, "eth: up ip=%d.%d.%d.%d link=%s",
              ip[0], ip[1], ip[2], ip[3],
              Ethernet.linkStatus() == LinkON ? "on" : "off");
    }
    return true;
  }
#endif

#if defined(WITH_BACKHAUL_CENTRAL) || defined(WITH_BACKHAUL_PERIPHERAL)
  if (strcmp(command, "backhaul") == 0) {
  #if defined(WITH_BACKHAUL_CENTRAL)
    // Central side (Sortsnak): the relay's link up to the mast NUS peripheral.
    if (BleRelay.linkUp()) {
      sprintf(reply, "backhaul: up rssi=%ddBm", (int)BleRelay.rssi());
    } else {
      strcpy(reply, "backhaul: down (scanning)");
    }
  #elif defined(WITH_BACKHAUL_PERIPHERAL)
    // Peripheral side (mast): whether a central holds the NUS console slot.
    if (BleConsole.connected()) {
      sprintf(reply, "backhaul: central connected rssi=%ddBm", (int)BleConsole.rssi());
    } else {
      strcpy(reply, "backhaul: no central (advertising)");
    }
  #endif
    return true;
  }
#endif

#ifdef WITH_OBSERVER
  if (strcmp(command, "log on") == 0) {
    g_packet_dump_enabled = true;
    config().packet_dump = 1;   // persist across reboot/reflash
    configSave();
    strcpy(reply, "log: on");
    return true;
  }
  if (strcmp(command, "log off") == 0) {
    g_packet_dump_enabled = false;
    config().packet_dump = 0;
    configSave();
    strcpy(reply, "log: off");
    return true;
  }
  if (strcmp(command, "log") == 0 || strcmp(command, "log status") == 0) {
    sprintf(reply, "log: %s", g_packet_dump_enabled ? "on" : "off");
    return true;
  }
#endif

  // Durable config get/set for our own namespace. Gated on WITH_NET_BRIDGE: MQTT only
  // makes sense on a node that bridges the observer feed to a network (the relay
  // gateway, Sortsnak). The mast has no IP transport, so exposing mqtt config there is
  // just dead provisioning. The store struct stays build-agnostic; only the commands
  // gate. CRITICAL: upstream CommonCLI owns the generic `get <var>` / `set <var> <val>`
  // over the same admin path — so we ONLY claim keys under `mqtt.` (and the exact `get
  // mqtt` dump) and return false for everything else, letting CommonCLI handle its own
  // config. Never claim a bare `set `/`get ` or upstream breaks.
#ifdef WITH_NET_BRIDGE
  if (strcmp(command, "get mqtt") == 0) {
    Config& c = config();
    snprintf(reply, CLIEXT_REPLY_CAP,
             "mqtt: en=%s host=%s port=%u user=%s topic=%s tls=%s pass=%s",
             c.mqtt_enabled ? "on" : "off",
             c.mqtt_host[0] ? c.mqtt_host : "-", (unsigned)c.mqtt_port,
             c.mqtt_user[0] ? c.mqtt_user : "-",
             c.mqtt_topic[0] ? c.mqtt_topic : "-",
             c.mqtt_tls ? "on" : "off",
             c.mqtt_pass[0] ? "set" : "unset");
    return true;
  }
  if (strncmp(command, "set mqtt.", 9) == 0) {
    const char* args = command + 4;          // points at "mqtt.<field> <value>"
    const char* sp = strchr(args, ' ');
    if (!sp || sp == args) {
      strcpy(reply, "set: usage 'set mqtt.<field> <value>'");
      return true;
    }
    char key[24];
    size_t klen = (size_t)(sp - args);
    if (klen >= sizeof(key)) klen = sizeof(key) - 1;
    memcpy(key, args, klen);
    key[klen] = 0;
    const char* val = sp + 1;
    while (*val == ' ') val++;   // skip extra separators before the value

    Config& c = config();
    bool ok = true;
    if (strcmp(key, "mqtt.host") == 0)       setStr(c.mqtt_host,  sizeof(c.mqtt_host),  val);
    else if (strcmp(key, "mqtt.user") == 0)  setStr(c.mqtt_user,  sizeof(c.mqtt_user),  val);
    else if (strcmp(key, "mqtt.pass") == 0)  setStr(c.mqtt_pass,  sizeof(c.mqtt_pass),  val);
    else if (strcmp(key, "mqtt.topic") == 0) setStr(c.mqtt_topic, sizeof(c.mqtt_topic), val);
    else if (strcmp(key, "mqtt.port") == 0)  c.mqtt_port = (uint16_t)atoi(val);
    else if (strcmp(key, "mqtt.tls") == 0)     ok = parseOnOff(val, &c.mqtt_tls);
    else if (strcmp(key, "mqtt.enabled") == 0) ok = parseOnOff(val, &c.mqtt_enabled);
    else { snprintf(reply, CLIEXT_REPLY_CAP, "set: unknown mqtt key '%s'", key); return true; }

    if (!ok) { snprintf(reply, CLIEXT_REPLY_CAP, "set: bad value for %s (on|off)", key); return true; }
    if (configSave()) snprintf(reply, CLIEXT_REPLY_CAP, "ok: %s", key);
    else              strcpy(reply, "set: save failed (fs)");
    return true;
  }
#endif  // WITH_NET_BRIDGE

  return false;   // not an extension command — caller falls through to CommonCLI
}

void loop() {
#ifdef WITH_RAK13800_ETHERNET
  if (_tcpota_reboot_at && (long)(millis() - _tcpota_reboot_at) >= 0) {
    NRF_POWER->GPREGRET = DFU_MAGIC_TCP_RESET;
    NVIC_SystemReset();
  }
#endif
}

}  // namespace cliext
