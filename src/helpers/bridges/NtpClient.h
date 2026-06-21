#pragma once

#ifdef WITH_NET_BRIDGE

#include <Arduino.h>
#include <RAK13800_W5100S.h>
#include <Mesh.h>

// Simple SNTP client for the network node (Phase B1 M2). The RAK4631 has no battery-
// backed RTC and its mesh clock drifts / boots years off, which would poison every
// analyzer-spec JSON `timestamp`. This keeps the mesh RTCClock in UTC: an initial sync
// once ethernet is up, then a periodic refresh. Non-blocking — a tiny state machine over
// EthernetUDP (request → await reply with timeout) so it never stalls the mesh loop
// (beyond the one-shot DNS resolve, which is cached). Reads server/enable live from
// cliext config (`set ntp.*`). Gated WITH_NET_BRIDGE (bridge node only).
#ifndef NTP_DEFAULT_SERVER
#define NTP_DEFAULT_SERVER "pool.ntp.org"
#endif

class NtpClient {
public:
  void begin(mesh::RTCClock* rtc);   // bind the clock; call at boot
  void loop();                       // drive the state machine; call every loop()

  bool synced() const { return _synced; }
  const char* stage() const { return _stage; }   // last attempt outcome (for `get ntp`)

private:
  enum State { IDLE, WAITING };

  void sendRequest(uint32_t now);
  bool resolveServer();              // cached DNS; false on failure

  mesh::RTCClock* _rtc = nullptr;
  EthernetUDP _udp;                  // opened per sync and stop()'d after — never held
  IPAddress _server_ip;
  bool      _have_ip = false;

  State    _state = IDLE;
  bool     _synced = false;
  const char* _stage = "idle";       // diagnostics: dns-fail|no-socket|sent|timeout|synced
  uint32_t _next_sync = 0;           // millis() of the next attempt (0 = asap)
  uint32_t _sent_at = 0;
};

extern NtpClient Ntp;

#endif  // WITH_NET_BRIDGE
