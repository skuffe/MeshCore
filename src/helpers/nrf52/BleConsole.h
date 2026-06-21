#pragma once

#ifdef WITH_BACKHAUL_PERIPHERAL

#include <Arduino.h>
#include <bluefruit.h>

// TX power for the *application* BLE stack (the NUS console). The nRF52840
// radio supports up to +8 dBm; higher power buys a more stable link to the
// central relay at the cost of current draw. Override with -D BLE_TX_POWER=<n>
// (accepted: -40,-20,-16,-12,-8,-4,0,+2..+8).
//
// NOTE: this only sets the running firmware's TX power. The BLE *DFU* link runs
// in the bootloader, which has its own (often lower) TX power — see
// docs/remote-observer-backhaul.md for the separate bootloader-DFU-TX lever.
#ifndef BLE_TX_POWER
  #define BLE_TX_POWER 8
#endif

// Advertised BLE name; the central relay matches on this. Override per node.
#ifndef BLE_CONSOLE_NAME
  #define BLE_CONSOLE_NAME "MeshCore-RPT"
#endif

// Set true once begin() has brought BLE up. Read by NRF52Board::startOTAUpdate
// to skip a second Bluefruit.begin() (which would fail with INVALID_STATE).
extern bool g_ble_console_up;

// Mirrors the serial console (CLI + MESH_PACKET_LOGGING output) onto a BLE
// Nordic-UART (NUS) peripheral characteristic, so a co-located BLE-central
// bridge can consume packet logs and issue CLI commands wirelessly instead of
// over USB/ethernet. Writes fan out to USB Serial + the connected NUS client;
// reads drain Serial first, then NUS. Single central at a time.
//
// The DFU service (bledfu) is registered at boot (dormant) so the stock
// bootloader's native BLE DFU keeps working — `start ota` then only needs to
// free the single peripheral slot and re-advertise (see prepareForDfu()).
class BleConsoleStream : public Stream {
public:
  void begin();
  void loop();

  // Backhaul link telemetry (consumed by cliext's `backhaul` command on the mast
  // side). connected() is true while a central holds the NUS slot; rssi() returns
  // the last monitored connection RSSI in dBm (0 when no central is connected).
  bool   connected() const { return Bluefruit.connected() > 0; }
  int8_t rssi() const;

  // Drop any connected central and re-advertise so a DFU client can take the
  // single peripheral slot. Fills reply with the BLE MAC. Called from
  // NRF52Board::startOTAUpdate under WITH_BACKHAUL_PERIPHERAL.
  void prepareForDfu(char* reply);

  size_t write(uint8_t c) override;
  size_t write(const uint8_t* buf, size_t size) override;
  int available() override;
  int read() override;
  int peek() override;
  void flush() override;

private:
  // Heap-allocated in begin(), NOT direct members: these ctors malloc internal
  // FIFOs/buffers, which faults during C++ static-init on this build (the global
  // BleConsole is constructed before the heap/SoftDevice are ready → the node
  // crashes dead-silent before Serial). Construct them after main(). (Same gotcha
  // BleNusRelay hit with BLEClientUart.)
  BLEUart* _bleuart = nullptr;
  BLEDfu*  _bledfu = nullptr;
  unsigned long _last_adv_check = 0;

  // Peripheral connect callback: arms RSSI monitoring on the central's link and
  // records its handle so rssi() can read it back.
  static void onConnect(uint16_t conn_handle);
  static volatile uint16_t _conn_handle;
};

extern BleConsoleStream BleConsole;

#endif
