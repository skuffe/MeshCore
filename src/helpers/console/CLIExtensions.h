#pragma once

#include <Arduino.h>
#include <helpers/IdentityStore.h>   // FILESYSTEM macro for begin()

// Feature-gated CLI command registry — a generic extension point that complements
// upstream's CommonCLI. Each role-specific command lives here once and is compiled
// in ONLY when its capability flag is set (WITH_OBSERVER, WITH_NET_BRIDGE,
// WITH_RAK13800_ETHERNET, WITH_BACKHAUL_PERIPHERAL/CENTRAL — all derived from the
// node's ROLE_* in helpers/ObserverRoles.h), so a build neither carries nor exposes a
// command for a feature it lacks. Apps call cliext::handleCommand() from their own
// handleCommand() just before falling through to CommonCLI, and cliext::loop() from their loop().
//
// This is the same pattern as the original `start tcpota` hook, generalised so both
// the room_server (relay gateway) and repeater_ble_gw (mast observer) roles share one source.
namespace cliext {

// Bind the durable config store (loads `/cliext_cfg`) and seed runtime state from
// it. Call once at boot, after the filesystem is up and before consumers read any
// toggle. Safe to call with the same `fs` the app already uses for NodePrefs.
void begin(FILESYSTEM* fs);

// Try to handle `command`. Returns true if a (compiled-in) extension command
// matched — `reply` is then filled with the response (may be empty). Returns false
// if no extension matched, so the caller continues to CommonCLI.
bool handleCommand(const char* command, char* reply);

// Service deferred extension actions (currently the `start tcpota` reboot). Cheap;
// call once per main loop from every app that includes this module.
void loop();

// Local-exec hook for uniform node addressing. `node <name|id> <cmd>` treats every node
// equally — the central and each backhaul peripheral. When the target resolves to THIS
// node (self), the command runs locally and its reply returns inline; the app wires this
// to its own full dispatch (cliext + CommonCLI). A peripheral target relays over the
// backhaul as before. Without this hook, self-addressed commands are rejected.
typedef void (*LocalExecFn)(const char* cmd, char* reply, size_t cap);
void setLocalExec(LocalExecFn fn);

#ifdef WITH_OBSERVER
// Runtime observation toggle. Gates the meshconsole::log* emission in the app's
// logRx/logTx hooks, so the packet feed can be silenced/enabled live via `log
// on|off` without a reflash. Defaults on for the observer role; overridden at boot
// by a persisted `log off` in /cliext_cfg.
extern bool g_packet_dump_enabled;
#endif

}  // namespace cliext
