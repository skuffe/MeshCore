#pragma once
#ifdef WITH_BACKHAUL_CENTRAL
#include <Arduino.h>

// Central-side legacy (SDK11) BLE-DFU client for B-OTA. The central has ONE BLE central slot,
// shared with BleNusRelay; for a flash the relay is suspended and this owns the radio. It scans
// for the edge bootloader's legacy DFU service (0x1530), connects, and discovers the control-
// point (0x1531) + packet (0x1532) characteristics, then (M2b) drives the legacy state machine:
//   START → INIT(.dat) → RECEIVE(.bin) → VALIDATE → ACTIVATE.
//
// M2a (current): handoff + scan/connect/discover + report, then back off WITHOUT flashing — safe,
// because the bootloader self-times-out back to the app when no DFU activity follows. M2b adds the
// flashing state machine fed by DfuRelay's FRAME_DFU image.
//
// Pumped from the central loop() (non-blocking); DfuRelay narrates state() transitions to the host
// as DFU_STATUS and gates the flash on READY.
namespace bledfu {

enum State {
  IDLE,         // not running
  TRIGGERING,   // sent `start dfu` to the edge; waiting for its NUS link to drop (reboot to DFU)
  SCANNING,     // hunting the bootloader's 0x1530 DFU advert
  CONNECTING,   // connect() issued; waiting for the link
  DISCOVERING,  // connected; discovering the DFU service + characteristics
  READY,        // discovered — reached the bootloader (M2a end state; M2b: START/INIT done next)
  FAILED,       // gave up — see status(); radio handed back to the relay
  DONE,         // flash complete (M2b)
};

void        begin();                 // one-time char/UUID setup (call once, after Bluefruit is up)
void        start(const char* target); // kick a session: send `start dfu`, suspend relay, scan
void        loop();                  // pump the state machine — call every central loop()
void        abort(const char* why);  // tear down + hand the radio back to BleNusRelay
State       state();
const char* status();                // latest human status line (narrated as DFU_STATUS)
bool        statusChanged();         // true once after each status() change (edge-trigger)

}  // namespace bledfu
#endif
