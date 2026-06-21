#pragma once

#ifdef WITH_BLE_CENTRAL_RELAY

#include <Arduino.h>
#include <bluefruit.h>
#include <SPI.h>
#include <RAK13800_W5100S.h>

// TCP port the relayed remote-NUS feed is served on (a second mcbridge consumes
// it, distinct from EthConsole's :5000).
#ifndef BLE_RELAY_PORT
  #define BLE_RELAY_PORT 5001
#endif

// TX power for the central radio. nRF52840 supports up to +8 dBm; higher power
// improves link stability to the (possibly distant) peripheral. -D BLE_TX_POWER.
#ifndef BLE_TX_POWER
  #define BLE_TX_POWER 8
#endif

// Substring the target peripheral's advertised name must contain (matched in
// addition to the NUS service-UUID filter). Override to target a specific node.
#ifndef BLE_RELAY_TARGET_NAME
  #define BLE_RELAY_TARGET_NAME "MeshCore-RPT"
#endif

// BLE central that connects to a remote MeshCore node's NUS console peripheral
// (e.g. the mast repeater running BleConsole) and bridges it bidirectionally to
// a TCP listener on BLE_RELAY_PORT, reusing the W5100S ethernet that
// EthernetTcpConsole already brought up (its own DHCP/socket management is left
// untouched — this only opens a second listening socket).
class BleNusRelay {
public:
  void begin();   // lightweight: just records the singleton. Safe in setup().
  void loop();    // lazily brings up BLE + the listener once ethernet is up, then pumps

  // Backhaul link telemetry (consumed by cliext's `backhaul` command). linkUp()
  // is true once the NUS pipe is discovered and notifications are enabled; rssi()
  // returns the last monitored connection RSSI in dBm (0 when down).
  bool   linkUp() const { return _linkUp; }
  int8_t rssi() const;

  // Static BLE callbacks (route to the singleton).
  static void onScan(ble_gap_evt_adv_report_t* report);
  static void onConnect(uint16_t conn_handle);
  static void onDisconnect(uint16_t conn_handle, uint8_t reason);

private:
  void startBle();   // Bluefruit central init — deferred out of setup() (see .cpp)

  // Heap-allocated in startBle(), NOT a static member: BLEClientUart's ctor
  // mallocs an RX FIFO, which faults during C++ static-init on this build (the
  // node crashes before Serial even starts). Construct it after main() instead.
  BLEClientUart* _clientUart = nullptr;
  EthernetServer _server{BLE_RELAY_PORT};
  EthernetClient _client;
  bool           _started = false;
  bool           _eth_was_ready = false;   // edge-detect EthConsole self-heal
  volatile bool  _linkUp = false;
  volatile uint16_t _conn_handle = BLE_CONN_HANDLE_INVALID;  // for RSSI readback
};

extern BleNusRelay BleRelay;

#endif
