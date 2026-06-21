#pragma once

#ifdef WITH_RAK13800_ETHERNET

#include <Arduino.h>
#include <SPI.h>
#include <RAK13800_W5100S.h>

/**
 * Mirrors the serial console (CLI plus MESH_PACKET_LOGGING output) on a TCP
 * socket served via a RAK13800 W5100S ethernet module, so a bridge daemon can
 * consume packet logs and issue CLI commands over the network instead of USB.
 *
 * Writes fan out to USB Serial and the connected TCP client; reads drain USB
 * Serial first, then the TCP client. Single client at a time — a new
 * connection displaces the previous one.
 */
// Retained-RAM handoff to the bootloader's TCP DFU mode (see dfu_tcp.h in
// the tcp-dfu bootloader fork). Address and layout must stay in sync.
#define DFU_TCP_HANDOFF_ADDR   0x20007F60UL
#define DFU_TCP_HANDOFF_MAGIC  0x54435041UL  // "TCPA"
#define DFU_MAGIC_TCP_RESET    0x7E

struct DfuTcpHandoff {
  uint32_t magic;
  uint8_t  ip[4];
  uint8_t  gw[4];
  uint8_t  mask[4];
  uint16_t port;
  uint16_t reserved;
};

class EthernetTcpConsole : public Stream {
public:
  void begin();   // hard-reset the W5100S and prepare for DHCP
  void loop();    // service DHCP/link and accept clients; call every loop()

  // True once the W5100S is initialised and a DHCP lease is held (i.e. it is
  // safe to issue SPI/socket calls to the chip). Other consumers that reuse
  // this ethernet (e.g. BleNusRelay) MUST gate on this before touching the
  // W5100S — the driver's SPI peripheral isn't begun until the first lease.
  bool isReady() const { return _ready; }

  // Stash the live lease for the bootloader's TCP DFU receiver.
  // Returns false while ethernet is down.
  bool prepareTcpDfuHandoff(uint16_t port, char* ip_str);

  size_t write(uint8_t c) override;
  size_t write(const uint8_t* buf, size_t size) override;
  int available() override;
  int read() override;
  int peek() override;
  void flush() override;

private:
  void serviceLink();

  EthernetClient _client;
  unsigned long _next_dhcp_attempt = 0;
  uint32_t _next_health_check = 0;   // throttles the once-ready link/lease probe
  uint32_t _attempts = 0;
  bool _ready = false;        // DHCP lease held, server listening
};

extern EthernetTcpConsole EthConsole;

#endif
