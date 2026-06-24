#pragma once
#ifdef WITH_BACKHAUL_CENTRAL
#include <Arduino.h>

// Central-side legacy (SDK11) BLE-DFU client for B-OTA. The central has ONE BLE central slot,
// shared with BleNusRelay; for a flash the relay is suspended and this owns the radio. It drives
// the Adafruit 2-stage buttonless DFU of an edge node:
//   STAGE A — connect to the re-advertised app, discover its BLEDfu 0x1530 service, write the
//             buttonless trigger (START_DFU) → the app resets into the bootloader.
//   STAGE B — re-scan for the bootloader ("AdaDFU" / 0x1530), reconnect, discover.
//   STAGE C — run the legacy DFU state machine fed by DfuRelay's FRAME_DFU image:
//             START → INIT(.dat) → RECEIVE(.bin) → VALIDATE → ACTIVATE.
//
// Pumped from the central loop() (non-blocking except the brief per-chunk BLE write). DfuRelay
// gates the host stream on RECEIVING, feeds chunks via feedChunk(), and finishes via commit().
namespace bledfu {

enum State {
  IDLE,
  // stage A — reach the app's DFU service + trigger buttonless
  A_TRIGGER, A_SCAN, A_CONNECT, A_DISCOVER, A_BUTTONLESS,
  // stage B — reconnect to the bootloader
  B_SCAN, B_CONNECT, B_DISCOVER,
  // stage C — legacy DFU
  C_START, W_START, C_INIT, W_INIT, C_RECV_BEGIN,
  RECEIVING,                 // streaming firmware; DfuRelay feeds chunks here
  W_RECV, C_VALIDATE, W_VALIDATE, C_ACTIVATE,
  DONE, FAILED,
};

// Kick a session. init/init_len = the .dat init packet (from the BEGIN frame); image_len = total
// firmware bytes to stream. The image itself arrives later via feedChunk().
void        start(const char* target, const uint8_t* init, uint16_t init_len, uint32_t image_len);
void        loop();                   // pump the state machine — call every central loop()
bool        feedChunk(const uint8_t* data, uint16_t len); // write a firmware chunk to the edge (RECEIVING)
void        commit();                 // all firmware sent → finish (final RECEIVE resp → VALIDATE → ACTIVATE)
void        abort(const char* why);   // tear down + hand the radio back to BleNusRelay
State       state();
const char* status();                 // latest human status line (narrated as DFU_STATUS)
bool        statusChanged();          // true once after each status() change (edge-trigger)

}  // namespace bledfu
#endif
