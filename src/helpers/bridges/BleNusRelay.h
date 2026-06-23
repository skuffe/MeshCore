#pragma once

#ifdef WITH_BACKHAUL_CENTRAL

#include <Arduino.h>
#include <bluefruit.h>
#include <SPI.h>
#include <RAK13800_W5100S.h>
#include <helpers/bridges/BackhaulFrame.h>   // structured-observation demux off the NUS feed

namespace mesh { class RTCClock; }   // for the backhaul time-push (NTP epoch → mast)

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
// (e.g. the mast repeater running BleConsole). The remote's observations arrive as
// structured backhaul frames and are republished to MQTT under its observer identity;
// remote-admin console commands are carried as FRAME_CONSOLE in both directions (the
// reply is printed to EthConsole's :5000). No separate TCP port — :5001 is retired.
class BleNusRelay {
public:
  void begin();   // lightweight: just records the singleton. Safe in setup().
  void loop();    // lazily brings up BLE once ethernet is up, then pumps the NUS feed

  // Send a console command line to the remote peripheral (FRAME_CONSOLE). The reply
  // arrives asynchronously and is printed to EthConsole (:5000). Used by `node <name> <cmd>`.
  void sendConsole(const char* cmd);

  // Bind the node's NTP-synced RTC so the relay can push UTC to the mast over the backhaul
  // (FRAME_TIME), gated on the backhaul_timesync config toggle. Call once at boot.
  void setRtc(mesh::RTCClock* rtc) { _rtc = rtc; }

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
  void dispatchFrame();   // hand a complete backhaul frame to the MQTT publisher

  // Structured-backhaul demux: the mast multiplexes 0x1E-prefixed observation/identity/
  // status/console frames onto the NUS pipe. _parser splits them; frames are decoded and
  // observations published under the mast's observer identity, console replies printed to
  // :5000. _mast_obs_idx is the mast's slot in the observer table (resolved from its
  // IDENTITY frame; -1 until then, so observations before identity are dropped).
  // _mast_online edge-detects link state to publish the mast's online/offline.
  backhaul::Parser _parser;
  int              _mast_obs_idx = -1;
  bool             _mast_online  = false;
  mesh::RTCClock*  _rtc = nullptr;       // for FRAME_TIME pushes to the mast
  uint32_t         _next_time_push = 0;  // throttle the backhaul time-sync send

  // Heap-allocated in startBle(), NOT a static member: BLEClientUart's ctor
  // mallocs an RX FIFO, which faults during C++ static-init on this build (the
  // node crashes before Serial even starts). Construct it after main() instead.
  BLEClientUart* _clientUart = nullptr;
  bool           _started = false;
  volatile bool  _linkUp = false;
  volatile uint16_t _conn_handle = BLE_CONN_HANDLE_INVALID;  // for RSSI readback
};

extern BleNusRelay BleRelay;

#endif
