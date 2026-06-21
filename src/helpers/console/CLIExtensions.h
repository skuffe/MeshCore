#pragma once

#include <Arduino.h>

// Feature-gated CLI command registry — a generic extension point that complements
// upstream's CommonCLI. Each role-specific command lives here once and is compiled
// in ONLY when its capability flag is set (WITH_RAK13800_ETHERNET, WITH_BLE_*,
// PACKET_LOG_STREAM), so a build neither carries nor exposes a command for a
// feature it lacks. Apps call cliext::handleCommand() from their own handleCommand()
// just before falling through to CommonCLI, and cliext::loop() from their loop().
//
// This is the same pattern as the original `start tcpota` hook, generalised so both
// the room_server (eth gateway) and repeater_ble_gw (mast) roles share one source.
namespace cliext {

// Try to handle `command`. Returns true if a (compiled-in) extension command
// matched — `reply` is then filled with the response (may be empty). Returns false
// if no extension matched, so the caller continues to CommonCLI.
bool handleCommand(const char* command, char* reply);

// Service deferred extension actions (currently the `start tcpota` reboot). Cheap;
// call once per main loop from every app that includes this module.
void loop();

#ifdef PACKET_LOG_STREAM
// Runtime observation toggle. Gates the meshconsole::log* emission in the app's
// logRx/logTx hooks, so the packet feed can be silenced/enabled live via `log
// on|off` without a reflash. Seeded at boot from WITH_PACKET_OBSERVER.
extern bool g_packet_dump_enabled;
#endif

}  // namespace cliext
