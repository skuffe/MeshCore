#include "CLIExtensions.h"
#include "CliextConfig.h"
#include <string.h>
#include <strings.h>   // strcasecmp / strncasecmp (node-name matching)
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

// Upstream CommonCLI command listing for `help`. Go-generated (tools/genclihelp.go) from
// the pristine CommonCLI.cpp + tools/cli_help.tsv; committed so a bare `pio run` compiles.
// Defines COMMONCLI_CMDS(X) — the upstream half of the `help` table.
#include <generated/commoncli_help.h>

namespace cliext {

// ── `help` command registry ────────────────────────────────────────────────
// Single source of truth for OUR (cliext) commands, self-documenting via the
// `help` command. Pure-preprocessor X-macro: each X(token, group, description).
// Sub-lists are #ifdef-gated on the same WITH_* capability flags that gate the
// command implementations below, so a build's `help` lists exactly the commands it
// actually carries — no runtime guard column, no hand-maintained list to rot.
// The upstream half comes from the generated COMMONCLI_CMDS(X) (above).
#define CLIEXT_CMDS_ALWAYS(X) \
  X("help",                   "SYSTEM",  "this command list (alias '?')")

#ifdef WITH_BACKHAUL_CENTRAL
  #define CLIEXT_CMDS_NODE(X) \
    X("node list",            "NODE",    "list all nodes (self first)") \
    X("node <name|id> <cmd>", "NODE",    "run <cmd> on that node (self/peripheral)")
#else
  #define CLIEXT_CMDS_NODE(X)
#endif

#if defined(WITH_RAK13800_ETHERNET) || defined(WITH_BACKHAUL_PERIPHERAL)
  #define CLIEXT_CMDS_DFU(X) \
    X("start dfu",            "NODE",    "enter this node's native DFU")
#else
  #define CLIEXT_CMDS_DFU(X)
#endif

#ifdef WITH_OBSERVER
  #define CLIEXT_CMDS_OBSERVE(X) \
    X("feed on|off|status",   "OBSERVE", "observation feed toggle")
#else
  #define CLIEXT_CMDS_OBSERVE(X)
#endif

#ifdef WITH_RAK13800_ETHERNET
  #define CLIEXT_CMDS_ETH(X) \
    X("eth",                  "NET",     "ethernet link status")
#else
  #define CLIEXT_CMDS_ETH(X)
#endif

#if defined(WITH_BACKHAUL_CENTRAL) || defined(WITH_BACKHAUL_PERIPHERAL)
  #define CLIEXT_CMDS_BACKHAUL(X) \
    X("backhaul",             "NET",     "backhaul link status")
#else
  #define CLIEXT_CMDS_BACKHAUL(X)
#endif

#ifdef WITH_NET_BRIDGE
  #define CLIEXT_CMDS_NET(X) \
    X("get ntp",              "NET",     "NTP / clock status") \
    X("set ntp.<k> <v>",      "NET",     "ntp.server | ntp.enabled") \
    X("get mqtt [msg|<N>]",   "NET",     "MQTT status (all | toggles | slot N)") \
    X("mqtt reset [<N>]",     "NET",     "re-arm tripped broker slot(s)") \
    X("set mqtt[N].<k> <v>",  "NET",     "broker config / msg-type toggles") \
    X("set observer.<n>.<k>", "OBSERVE", "rename a node / toggle its mqtt publish") \
    X("set backhaul.timesync","NET",     "on|off — push UTC to peripherals")
#else
  #define CLIEXT_CMDS_NET(X)
#endif

#define CLIEXT_CMDS(X) \
  CLIEXT_CMDS_ALWAYS(X) \
  CLIEXT_CMDS_NODE(X) \
  CLIEXT_CMDS_DFU(X) \
  CLIEXT_CMDS_OBSERVE(X) \
  CLIEXT_CMDS_ETH(X) \
  CLIEXT_CMDS_BACKHAUL(X) \
  CLIEXT_CMDS_NET(X)

#ifdef WITH_OBSERVER
  // The observer role boots with the packet dump ON. A persisted `feed off` in
  // /cliext_cfg (loaded by begin()) overrides this at startup.
  bool g_packet_dump_enabled = true;
#endif

#ifdef WITH_RAK13800_ETHERNET
// Deferred reboot into the bootloader's TCP DFU mode (0 = inactive). Lets the
// `start dfu` reply drain to the client before the chip resets.
static uint32_t _dfu_reboot_at = 0;
#endif

// Local-exec hook for self-addressed `node <self> <cmd>` (uniform node addressing).
static LocalExecFn s_local_exec = nullptr;
void setLocalExec(LocalExecFn fn) { s_local_exec = fn; }

// Active console stream (set by each app to its CONSOLE: EthConsole on the central's
// :5000, BleConsole/Serial on the mast). `help` streams its multi-line table here rather
// than into the 160-byte reply — which also makes `node <edge> help` work: the edge
// streams the listing to its NUS as plain text, and the central forwards that stray
// (non-frame) text to :5000 (BleNusRelay PASS path).
static Stream* s_console = nullptr;
void setConsole(Stream* s) { s_console = s; }

// Paced line emit for multi-line console output (`help`). A large reply (the help table is
// ~2.5 KB) streamed flat over the BLE backhaul can outrun the consumer: the central drains
// its NUS RX FIFO only once per loop(), so a flat blast can overflow it AND starve the
// interleaved observation frames sharing the pipe. We cap in-flight bytes — after roughly a
// notification window's worth, flush + yield + a short pause so the link drains before more
// is queued. This bounds occupancy to well under BLE_RELAY_RX_FIFO regardless of the reply's
// total length, so output can never overflow (the enlarged FIFO is then pure headroom), and
// concurrent observation frames keep flowing. Direct consoles (TCP/serial) have their own
// flow control; the brief pacing there is harmless on an operator-invoked command.
static uint16_t s_emit_pending = 0;
static void emitLine(const char* line) {
  if (!s_console) return;
  s_console->println(line);
  s_emit_pending += (uint16_t)strlen(line) + 2;   // + CRLF
  if (s_emit_pending >= 192) {                     // ~one BLE notification window
    s_console->flush();
    yield();                                       // let the SoftDevice flush notifications
    delay(4);                                       // and the central drain its RX FIFO
    s_emit_pending = 0;
  }
}

// Native-DFU hook for `start dfu` on nodes whose native DFU isn't the ethernet TCP path
// (i.e. the BLE-bootloader edge). The central's ethernet DFU is handled inline below; a
// peripheral registers a handler that triggers its bootloader BLE DFU. Unset → "no DFU".
static DfuFn s_dfu = nullptr;
void setDfuHandler(DfuFn fn) { s_dfu = fn; }

void begin(FILESYSTEM* fs) {
  configBegin(fs);   // load /cliext_cfg (or defaults)
#ifdef WITH_OBSERVER
  // Once the operator has set `feed on|off`, that persisted choice overrides the
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

  // `help` / `?` — print the command set this build actually carries (native CLIEXT_CMDS
  // registry + the Go-generated upstream COMMONCLI_CMDS), streamed to the console so it
  // isn't bound by the 160-byte reply. Available on every node; routes through uniform
  // addressing (`node <x> help` self-documents node x's own compiled-in commands).
  if (strcmp(command, "help") == 0 || strcmp(command, "?") == 0) {
    if (!s_console) { strcpy(reply, "help: console unavailable"); return true; }
    s_emit_pending = 0;
    emitLine("commands on this node:");
    #define X(tok, grp, desc) { char ln[120]; \
      snprintf(ln, sizeof ln, "  %-8s %-24s %s", grp, tok, desc); emitLine(ln); }
    CLIEXT_CMDS(X)
    emitLine("  -- upstream (CommonCLI) --");
    COMMONCLI_CMDS(X)
    #undef X
    s_console->flush();
    reply[0] = 0;   // already streamed (paced); nothing for the caller to print
    return true;
  }

#ifdef WITH_RAK13800_ETHERNET
  // Central's native DFU: reboot into the (forked) bootloader's TCP DFU receiver on :4444
  // (was `start tcpota`; renamed to the uniform `start dfu` — each node does its native DFU).
  if (strcmp(command, "start dfu") == 0) {
    char ip_str[16];
    if (EthConsole.prepareTcpDfuHandoff(4444, ip_str)) {
      sprintf(reply, "TCP DFU: rebooting, send image to %s:4444", ip_str);
      _dfu_reboot_at = millis() + 700;   // let the reply drain first
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

#if defined(WITH_BACKHAUL_PERIPHERAL) && !defined(WITH_RAK13800_ETHERNET)
  // Edge node's native DFU: hand off to the bootloader's BLE DFU (Adafruit BLEDfu). The
  // app can't reach the board directly from here, so the role main registers a handler
  // (setDfuHandler) that runs upstream's `start ota` → startOTAUpdate() → re-advertise.
  // Reached locally (mast serial) or via `node <edge> start dfu` over the backhaul.
  if (strcmp(command, "start dfu") == 0) {
    if (s_dfu) s_dfu(reply, CLIEXT_REPLY_CAP);
    else       strcpy(reply, "dfu: handler unset");
    return true;
  }
#endif

#ifdef WITH_BACKHAUL_CENTRAL
  // Uniform node addressing (replaces the retired :5001 passthrough). Every node is equal:
  // `node list` enumerates them all (self first, then backhaul peripherals); `node <name|id>
  // <cmd>` runs <cmd> on that node. Self → runs locally, reply inline. A peripheral → relays
  // as a FRAME_CONSOLE, reply arrives asynchronously tagged "[mast] ..." on this :5000 console.
  // (Auth-gating the remote-admin surface is phase B3.) `nodeMatches` below is the shared
  // self(0)/relay(1..) enumerator.
  // LOCKED, parseable format — one node per line, whitespace-delimited, fixed column order:
  //   <name> <id8> <role> <online> [<rssi>dBm]
  // self first (slot 0), then backhaul peripherals. role = self|relay; online = online|offline
  // (self is always online); rssi only on an online relay. The TUI splits on whitespace, so this
  // FORMAT IS A CONTRACT — keep the column order/tokens stable. Streamed (like `help`) so it isn't
  // bound by the 160 B reply, and paced so a long list can't overflow the backhaul.
  if (strcmp(command, "node list") == 0) {
    if (!s_console) { strcpy(reply, "node list: console unavailable"); return true; }
    s_emit_pending = 0;
    int active = BleRelay.activeObserverIdx();   // live peripheral's slot, or -1
    int shown = 0;
    for (int i = 0; i < OBSERVERS_MAX; i++) {
      const Observer& o = config().observers[i];
      if (!o.pubkey_hex[0]) continue;
      if (i != 0 && o.source != OBS_RELAY) continue;   // slot 0 = self; others must be relays
      char id8[9]; strncpy(id8, o.pubkey_hex, 8); id8[8] = 0;
      bool online = (i == 0) || (i == active);
      char line[80];
      if (i != 0 && online)
        snprintf(line, sizeof line, "%-18s %-8s %-5s %-7s %ddBm",
                 o.name[0] ? o.name : id8, id8, "relay", "online", (int)BleRelay.rssi());
      else
        snprintf(line, sizeof line, "%-18s %-8s %-5s %s",
                 o.name[0] ? o.name : id8, id8, i == 0 ? "self" : "relay",
                 online ? "online" : "offline");
      emitLine(line);
      shown++;
    }
    if (!shown) emitLine("(no nodes)");
    s_console->flush();
    reply[0] = 0;
    return true;
  }
  if (strncmp(command, "node ", 5) == 0) {
    const char* rest = command + 5;            // "<name|id> <cmd>"
    const char* sp = strchr(rest, ' ');
    if (!sp || sp == rest) { strcpy(reply, "node: usage 'node <name|id> <cmd>'"); return true; }
    char target[40];
    size_t tl = (size_t)(sp - rest);
    if (tl >= sizeof(target)) tl = sizeof(target) - 1;
    memcpy(target, rest, tl); target[tl] = 0;
    const char* fwd = sp + 1;
    while (*fwd == ' ') fwd++;
    // Match any node by name (case-insensitive) or pubkey prefix: slot 0 = self, 1.. = relays.
    int found = -1;
    for (int i = 0; i < OBSERVERS_MAX; i++) {
      const Observer& o = config().observers[i];
      if (!o.pubkey_hex[0]) continue;
      if (i != 0 && o.source != OBS_RELAY) continue;
      if ((o.name[0] && strcasecmp(o.name, target) == 0) ||
          strncasecmp(o.pubkey_hex, target, strlen(target)) == 0) { found = i; break; }
    }
    if (found < 0) { snprintf(reply, CLIEXT_REPLY_CAP, "node: '%s' not found (see 'node list')", target); return true; }
    if (!*fwd)     { strcpy(reply, "node: empty command"); return true; }
    if (found == 0) {                          // self → run locally, reply inline
      static bool in_self;                     // guard against `node self node self ...`
      if (!s_local_exec || in_self) { strcpy(reply, "node: self exec unavailable"); return true; }
      in_self = true;
      s_local_exec(fwd, reply, CLIEXT_REPLY_CAP);
      in_self = false;
      return true;
    }
    if (!BleRelay.linkUp()) { strcpy(reply, "node: backhaul down"); return true; }
    BleRelay.sendConsole(fwd);
    snprintf(reply, CLIEXT_REPLY_CAP, "node %s: sent (reply async)",
             config().observers[found].name[0] ? config().observers[found].name : target);
    return true;
  }
#endif

#ifdef WITH_OBSERVER
  // Observation-feed toggle. Renamed from `log on|off|status` → `feed …` so it no longer
  // collides with upstream CommonCLI's flash packet-log (`log start|stop|erase`), which now
  // reaches CommonCLI unshadowed. The persisted config field stays `packet_dump`.
  if (strcmp(command, "feed on") == 0) {
    g_packet_dump_enabled = true;
    config().packet_dump = 1;   // persist across reboot/reflash
    configSave();
    strcpy(reply, "feed: on");
    return true;
  }
  if (strcmp(command, "feed off") == 0) {
    g_packet_dump_enabled = false;
    config().packet_dump = 0;
    configSave();
    strcpy(reply, "feed: off");
    return true;
  }
  if (strcmp(command, "feed") == 0 || strcmp(command, "feed status") == 0) {
    sprintf(reply, "feed: %s", g_packet_dump_enabled ? "on" : "off");
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
  // (`observer list` retired — folded into `node list`, the single node enumerator.
  // Per-observer mqtt state is still set via `set observer.<n>.mqtt`.)
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
  if (_dfu_reboot_at && (long)(millis() - _dfu_reboot_at) >= 0) {
    NRF_POWER->GPREGRET = DFU_MAGIC_TCP_RESET;
    NVIC_SystemReset();
  }
#endif
}

}  // namespace cliext
