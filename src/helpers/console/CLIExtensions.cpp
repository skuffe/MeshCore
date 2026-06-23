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
#ifdef WITH_NET_BRIDGE
  #include <helpers/bridges/MqttPublisher.h>   // slot conn/breaker state + `mqtt reset`
  #include <helpers/bridges/NtpClient.h>       // `get ntp` sync state
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

  // Durable MULTI-BROKER config + control for our own namespace. Gated on WITH_NET_BRIDGE:
  // MQTT only makes sense on a node that bridges the observer feed to a network (the relay
  // gateway, Sortsnak). The mast has no IP transport, so exposing mqtt config there is just
  // dead provisioning. CRITICAL: upstream CommonCLI owns the generic `get <var>` / `set
  // <var> <val>` over the same admin path — so we ONLY claim keys under `mqtt`/`mqtt<N>.`
  // (and the exact `get mqtt[N]` / `mqtt reset` verbs) and return false otherwise, letting
  // CommonCLI handle its own config. Never claim a bare `set `/`get ` or upstream breaks.
#ifdef WITH_NET_BRIDGE
  // `get mqtt`  → one-line summary across all slots (conn/breaker state + queue depth).
  if (strcmp(command, "get mqtt") == 0) {
    int n = snprintf(reply, CLIEXT_REPLY_CAP, "mqtt: q=%d", MqttPub.queueDepth());
    for (int i = 0; i < mqttSlotCount() && n < CLIEXT_REPLY_CAP; i++) {
      const MqttSlot& s = mqttSlot(i);
      const char* st = !s.enabled ? "off"
                     : MqttPub.slotConnected(i) ? "up"
                     : MqttPub.slotTripped(i)   ? "trip"
                     : "down";
      n += snprintf(reply + n, CLIEXT_REPLY_CAP - n, " %d:%s", i + 1, st);
    }
    return true;
  }
  // `get mqtt<N>` → full detail for one slot (1-based). Bounded to the reply cap.
  if (strncmp(command, "get mqtt", 8) == 0 && command[8] >= '1' && command[8] <= '9' && command[9] == 0) {
    int idx = command[8] - '1';
    if (idx >= mqttSlotCount()) { snprintf(reply, CLIEXT_REPLY_CAP, "mqtt: only %d slots", mqttSlotCount()); return true; }
    const MqttSlot& s = mqttSlot(idx);
    const char* conn = !s.enabled ? "off"
                     : MqttPub.slotConnected(idx) ? "up"
                     : MqttPub.slotTripped(idx)   ? "trip"
                     : "down";
    snprintf(reply, CLIEXT_REPLY_CAP,
             "mqtt%d en=%s conn=%s host=%s:%u topic=%s tls=%s user=%s pass=%s",
             idx + 1, s.enabled ? "on" : "off", conn,
             s.host[0] ? s.host : "-", (unsigned)s.port,
             s.topic[0] ? s.topic : "-", s.tls ? "on" : "off",
             s.user[0] ? s.user : "-", s.pass[0] ? "set" : "unset");
    return true;
  }
  // `get mqtt msg` → global message-type toggles (apply across all brokers).
  if (strcmp(command, "get mqtt msg") == 0) {
    Config& g = config();
    const char* tx = g.msg_tx == 0 ? "off" : g.msg_tx == 1 ? "all" : "advert";
    snprintf(reply, CLIEXT_REPLY_CAP,
             "msg: status=%s packets=%s raw=%s rx=%s tx=%s interval=%us",
             g.msg_status ? "on" : "off", g.msg_packets ? "on" : "off",
             g.msg_raw ? "on" : "off", g.msg_rx ? "on" : "off", tx,
             (unsigned)g.status_interval_s);
    return true;
  }
  // `observer list` → the observer table: index, source (self/relay), name (or pubkey
  // prefix), and per-observer MQTT publish toggle. Index 0 is this node; 1.. are
  // backhaul observers auto-populated from their IDENTITY frames.
  if (strcmp(command, "observer list") == 0) {
    int n = 0;
    for (int i = 0; i < OBSERVERS_MAX && n < CLIEXT_REPLY_CAP; i++) {
      const Observer& o = config().observers[i];
      if (!o.pubkey_hex[0]) continue;
      char id8[9]; strncpy(id8, o.pubkey_hex, 8); id8[8] = 0;
      n += snprintf(reply + n, CLIEXT_REPLY_CAP - n, "%s%d:%s %s mqtt=%s",
                    n ? " | " : "", i, o.source == OBS_LOCAL ? "self" : "relay",
                    o.name[0] ? o.name : id8, o.mqtt_enabled ? "on" : "off");
    }
    if (!n) strcpy(reply, "observer: none");
    return true;
  }
  // `set observer.<n>.<name|mqtt> <value>` — rename an observer or toggle its publishing.
  if (strncmp(command, "set observer.", 13) == 0) {
    const char* rest = command + 13;          // "<n>.<field> <value>"
    char* endp;
    long n = strtol(rest, &endp, 10);
    if (endp == rest || *endp != '.') { strcpy(reply, "set: usage 'set observer.<n>.<name|mqtt> <value>'"); return true; }
    if (n < 0 || n >= OBSERVERS_MAX)  { snprintf(reply, CLIEXT_REPLY_CAP, "set: observer 0..%d", OBSERVERS_MAX - 1); return true; }
    const char* fieldpart = endp + 1;         // "<field> <value>"
    const char* sp = strchr(fieldpart, ' ');
    if (!sp || sp == fieldpart) { strcpy(reply, "set: usage 'set observer.<n>.<name|mqtt> <value>'"); return true; }
    char field[16];
    size_t fl = (size_t)(sp - fieldpart);
    if (fl >= sizeof(field)) fl = sizeof(field) - 1;
    memcpy(field, fieldpart, fl); field[fl] = 0;
    const char* val = sp + 1;
    while (*val == ' ') val++;

    Observer& o = config().observers[n];
    if (strcmp(field, "name") == 0)        setStr(o.name, sizeof(o.name), val);
    else if (strcmp(field, "mqtt") == 0) { if (!parseOnOff(val, &o.mqtt_enabled)) { strcpy(reply, "set: observer mqtt on|off"); return true; } }
    else { snprintf(reply, CLIEXT_REPLY_CAP, "set: unknown observer field '%s'", field); return true; }

    if (configSave()) snprintf(reply, CLIEXT_REPLY_CAP, "ok: observer.%ld.%s", n, field);
    else              strcpy(reply, "set: save failed (fs)");
    return true;
  }
  // NTP time-sync config + state (M2). `set ntp.server <host>` / `set ntp.enabled on|off`,
  // `get ntp`. Kept on the bridge node (WITH_NET_BRIDGE) — it owns the IP path.
  if (strcmp(command, "get ntp") == 0) {
    Config& g = config();
    snprintf(reply, CLIEXT_REPLY_CAP, "ntp: en=%s server=%s synced=%s stage=%s bh_timesync=%s",
             g.ntp_enabled ? "on" : "off",
             g.ntp_server[0] ? g.ntp_server : "pool.ntp.org(default)",
             Ntp.synced() ? "yes" : "no", Ntp.stage(),
             g.backhaul_timesync ? "on" : "off");
    return true;
  }
  // Backhaul time-sync service toggle (central pushes NTP UTC to the mast over the backhaul).
  if (strcmp(command, "set backhaul.timesync on") == 0 || strcmp(command, "set backhaul.timesync off") == 0) {
    config().backhaul_timesync = (command[23] == 'n') ? 1 : 0;   // "...o[n]" vs "...o[f]f"
    if (configSave()) snprintf(reply, CLIEXT_REPLY_CAP, "ok: backhaul.timesync %s", config().backhaul_timesync ? "on" : "off");
    else              strcpy(reply, "set: save failed (fs)");
    return true;
  }
  if (strncmp(command, "set ntp.", 8) == 0) {
    const char* args = command + 4;          // "ntp.<field> <value>"
    const char* sp = strchr(args, ' ');
    if (!sp || sp == args) { strcpy(reply, "set: usage 'set ntp.<server|enabled> <value>'"); return true; }
    char key[24];
    size_t klen = (size_t)(sp - args);
    if (klen >= sizeof(key)) klen = sizeof(key) - 1;
    memcpy(key, args, klen); key[klen] = 0;
    const char* val = sp + 1;
    while (*val == ' ') val++;

    Config& g = config();
    if (strcmp(key, "ntp.server") == 0)       setStr(g.ntp_server, sizeof(g.ntp_server), val);
    else if (strcmp(key, "ntp.enabled") == 0) { if (!parseOnOff(val, &g.ntp_enabled)) { strcpy(reply, "set: ntp.enabled on|off"); return true; } }
    else { snprintf(reply, CLIEXT_REPLY_CAP, "set: unknown ntp key '%s'", key); return true; }

    if (configSave()) snprintf(reply, CLIEXT_REPLY_CAP, "ok: %s (reboot to apply server)", key);
    else              strcpy(reply, "set: save failed (fs)");
    return true;
  }
  // `mqtt reset [N]` → clear a tripped circuit breaker and re-arm reconnect (all, or one).
  if (strcmp(command, "mqtt reset") == 0) {
    MqttPub.resetSlot(-1);
    strcpy(reply, "mqtt: all slots re-armed");
    return true;
  }
  if (strncmp(command, "mqtt reset ", 11) == 0) {
    int idx = atoi(command + 11) - 1;
    if (idx < 0 || idx >= mqttSlotCount()) { snprintf(reply, CLIEXT_REPLY_CAP, "mqtt: bad slot (1..%d)", mqttSlotCount()); return true; }
    MqttPub.resetSlot(idx);
    snprintf(reply, CLIEXT_REPLY_CAP, "mqtt%d: re-armed", idx + 1);
    return true;
  }
  // `set mqtt[N].<field> <value>` — N=1..MQTT_SLOTS; bare `mqtt.` aliases slot 1.
  if (strncmp(command, "set mqtt", 8) == 0 &&
      (command[8] == '.' || (command[8] >= '1' && command[8] <= '9'))) {
    const char* args = command + 4;          // "mqtt<N>.<field> <value>" | "mqtt.<field> ..."
    const char* sp = strchr(args, ' ');
    if (!sp || sp == args) {
      strcpy(reply, "set: usage 'set mqtt[N].<field> <value>'");
      return true;
    }
    char key[24];
    size_t klen = (size_t)(sp - args);
    if (klen >= sizeof(key)) klen = sizeof(key) - 1;
    memcpy(key, args, klen);
    key[klen] = 0;
    const char* val = sp + 1;
    while (*val == ' ') val++;   // skip extra separators before the value

    // Parse the slot index out of the key prefix: "mqtt." or "mqtt1." → 0, "mqtt2." → 1...
    const char* p = key + 4;     // past "mqtt"
    int idx = 0;
    bool had_digit = false;
    if (*p >= '1' && *p <= '9') { idx = *p - '1'; p++; had_digit = true; }
    if (*p != '.') { snprintf(reply, CLIEXT_REPLY_CAP, "set: bad mqtt key '%s'", key); return true; }
    const char* field = p + 1;

    // Bare `set mqtt.<toggle>` (no slot digit) sets a GLOBAL message-type toggle. These
    // names are disjoint from the slot fields below, so they never shadow `set mqtt.host`
    // etc (which alias slot 1). Indexed `set mqtt<N>.<toggle>` falls through to the slot.
    if (!had_digit) {
      Config& g = config();
      bool gok = true, ghandled = true;
      if      (strcmp(field, "status")  == 0) gok = parseOnOff(val, &g.msg_status);
      else if (strcmp(field, "packets") == 0) gok = parseOnOff(val, &g.msg_packets);
      else if (strcmp(field, "raw")     == 0) gok = parseOnOff(val, &g.msg_raw);
      else if (strcmp(field, "rx")      == 0) gok = parseOnOff(val, &g.msg_rx);
      else if (strcmp(field, "tx")      == 0) {
        if      (strcmp(val, "off") == 0 || strcmp(val, "0") == 0)    g.msg_tx = 0;
        else if (strcmp(val, "all") == 0 || strcmp(val, "1") == 0)    g.msg_tx = 1;
        else if (strcmp(val, "advert") == 0 || strcmp(val, "2") == 0) g.msg_tx = 2;
        else gok = false;
      }
      else if (strcmp(field, "interval") == 0) { int iv = atoi(val); g.status_interval_s = (uint16_t)(iv < 10 ? 10 : iv); }
      else ghandled = false;

      if (ghandled) {
        if (!gok) { snprintf(reply, CLIEXT_REPLY_CAP, "set: bad value for mqtt.%s", field); return true; }
        if (!configSave()) { strcpy(reply, "set: save failed (fs)"); return true; }
        MqttPub.reconfigure();
        snprintf(reply, CLIEXT_REPLY_CAP, "ok: mqtt.%s", field);
        return true;
      }
      // not a global toggle → fall through to slot-1 field handling
    }

    if (idx >= mqttSlotCount()) { snprintf(reply, CLIEXT_REPLY_CAP, "set: only %d slots", mqttSlotCount()); return true; }

    MqttSlot& s = mqttSlot(idx);
    bool ok = true;
    if (strcmp(field, "host") == 0)       setStr(s.host,  sizeof(s.host),  val);
    else if (strcmp(field, "user") == 0)  setStr(s.user,  sizeof(s.user),  val);
    else if (strcmp(field, "pass") == 0)  setStr(s.pass,  sizeof(s.pass),  val);
    else if (strcmp(field, "topic") == 0) setStr(s.topic, sizeof(s.topic), val);
    else if (strcmp(field, "port") == 0)  s.port = (uint16_t)atoi(val);
    else if (strcmp(field, "tls") == 0)     ok = parseOnOff(val, &s.tls);
    else if (strcmp(field, "enabled") == 0) ok = parseOnOff(val, &s.enabled);
    else { snprintf(reply, CLIEXT_REPLY_CAP, "set: unknown mqtt field '%s'", field); return true; }

    if (!ok) { snprintf(reply, CLIEXT_REPLY_CAP, "set: bad value for %s (on|off)", field); return true; }
    if (!configSave()) { strcpy(reply, "set: save failed (fs)"); return true; }
    // Re-apply config to the live publisher so edits take effect without a reboot.
    MqttPub.reconfigure();
    snprintf(reply, CLIEXT_REPLY_CAP, "ok: mqtt%d.%s", idx + 1, field);
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
