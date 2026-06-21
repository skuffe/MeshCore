#ifdef WITH_RAK13800_ETHERNET

#include "EthernetTcpConsole.h"

// RAK13800 on the WisBlock IO slot of the RAK19007 base board, wired to the
// header SPI. The default `SPI` instance (SPIM3) is rebound to the SX1262 by
// CustomSX1262::std_init, and TWIM0/1 are taken by Wire, so the W5100S gets
// its own SPIM2 instance. Pin assignments match Meshtastic's rak4631_eth_gw
// variant for this same hardware.
#ifndef ETH_PIN_RESET
  #define ETH_PIN_RESET 21
#endif
#ifndef ETH_PIN_CS
  #define ETH_PIN_CS 26
#endif
#define ETH_PIN_MISO 29
#define ETH_PIN_MOSI 30
#define ETH_PIN_SCK   3

#ifndef ETH_TCP_PORT
  #define ETH_TCP_PORT 5000
#endif

// DHCP timeouts: keep attempts short — Ethernet.begin() blocks the mesh loop
// for up to this long when the cable is unplugged.
#define ETH_DHCP_TIMEOUT_MS    8000UL
#define ETH_DHCP_RESPONSE_MS   4000UL
#define ETH_DHCP_RETRY_MS     30000UL
// Let the supply rail and switch-port link negotiation settle before the
// first SPI contact — a cold PoE bring-up glitches early chip reads.
#define ETH_FIRST_ATTEMPT_DELAY_MS 4000UL
// How often, once up, to probe link/lease health. Lease timers are second-grained
// (DhcpClass::checkLease only acts on >=1s elapsed), so a few seconds is plenty
// and keeps maintain()'s (potentially blocking) renew off the hot mesh loop.
#define ETH_HEALTH_CHECK_MS 3000UL

static SPIClass eth_spi(NRF_SPIM2, ETH_PIN_MISO, ETH_PIN_SCK, ETH_PIN_MOSI);
static EthernetServer server(ETH_TCP_PORT);

EthernetTcpConsole EthConsole;

// stable, locally administered MAC derived from the nRF52 device ID
static void getMacAddress(uint8_t mac[6]) {
  uint32_t lo = NRF_FICR->DEVICEID[0];
  uint32_t hi = NRF_FICR->DEVICEID[1];
  mac[0] = 0x02;   // locally administered, unicast
  mac[1] = (hi >> 8) & 0xFF;
  mac[2] = hi & 0xFF;
  mac[3] = (lo >> 16) & 0xFF;
  mac[4] = (lo >> 8) & 0xFF;
  mac[5] = lo & 0xFF;
}

static void resetChip() {
  pinMode(ETH_PIN_RESET, OUTPUT);   // hard-reset the W5100S
  digitalWrite(ETH_PIN_RESET, LOW);
  delay(100);
  digitalWrite(ETH_PIN_RESET, HIGH);
  delay(100);
}

void EthernetTcpConsole::begin() {
  resetChip();
  Ethernet.init(eth_spi, ETH_PIN_CS);
  _next_dhcp_attempt = millis() + ETH_FIRST_ATTEMPT_DELAY_MS;
}

void EthernetTcpConsole::serviceLink() {
  if (_ready) {
    // Throttled health supervisor. maintain() renews at T1 / rebinds at T2, but
    // its return code (and link/lease state) is otherwise ignored by callers —
    // a transient DHCP hiccup leaves the W5100S holding an expired IP forever
    // (observed: hours-up node loses the network, needs a power cycle). Detect
    // that, tear down, and re-acquire from a fresh hardware reset.
    if ((long)(millis() - _next_health_check) < 0) return;
    _next_health_check = millis() + ETH_HEALTH_CHECK_MS;

    int rc = Ethernet.maintain();   // renew/rebind the DHCP lease as needed
    // rc 1 = DHCP_CHECK_RENEW_FAIL, 3 = DHCP_CHECK_REBIND_FAIL (Dhcp.h — not
    // included here, so compared by value). 0/2/4 = nothing/renew-ok/rebind-ok.
    bool lost = (rc == 1 || rc == 3)
             || (Ethernet.linkStatus() == LinkOFF)            // cable/PHY dropped
             || (Ethernet.localIP() == IPAddress(0, 0, 0, 0)); // lease gone
    if (lost) {
      Serial.println("Ethernet: link/lease lost — re-acquiring");
      if (_client) _client.stop();
      _ready = false;
      _next_dhcp_attempt = millis();    // resume the acquire loop (waits for link)
    }
    return;
  }
  if ((long)(millis() - _next_dhcp_attempt) < 0) return;

  // Wait for PHY link before the (blocking) DHCP attempt. Crucially do NOT
  // hardware-reset the chip in this retry loop: resetChip() restarts PHY
  // auto-negotiation, and linkStatus() read immediately after a reset always
  // returns LinkOFF — re-resetting every cycle means the PHY never settles
  // (deadlock; observed as "link never comes back" on cable replug). A cable
  // event needs no reset — the PHY re-negotiates on its own, and the chip
  // config (done once at boot via the one-shot W5100.init()) is still intact.
  if (Ethernet.linkStatus() == LinkOFF) {
    _next_dhcp_attempt = millis() + 2000;
    return;
  }

  _attempts++;
  uint8_t mac[6];
  getMacAddress(mac);
  if (Ethernet.begin(mac, ETH_DHCP_TIMEOUT_MS, ETH_DHCP_RESPONSE_MS) == 1) {
    server.begin();
    _ready = true;
    _next_health_check = millis() + ETH_HEALTH_CHECK_MS;
    Serial.print("Ethernet up, IP: ");
    Serial.println(Ethernet.localIP());
  } else {
    Serial.println("Ethernet: DHCP failed, will retry");
    _next_dhcp_attempt = millis() + ETH_DHCP_RETRY_MS;
  }
}

void EthernetTcpConsole::loop() {
  serviceLink();
  if (!_ready) return;

  EthernetClient incoming = server.accept();
  if (incoming) {
    if (_client) _client.stop();   // new connection displaces the old one
    _client = incoming;
  }
}

bool EthernetTcpConsole::prepareTcpDfuHandoff(uint16_t port, char* ip_str) {
  if (!_ready) return false;

  IPAddress ip = Ethernet.localIP();
  IPAddress gw = Ethernet.gatewayIP();
  IPAddress mask = Ethernet.subnetMask();

  DfuTcpHandoff* h = (DfuTcpHandoff*) DFU_TCP_HANDOFF_ADDR;
  for (int i = 0; i < 4; i++) {
    h->ip[i] = ip[i];
    h->gw[i] = gw[i];
    h->mask[i] = mask[i];
  }
  h->port = port;
  h->reserved = 0;
  h->magic = DFU_TCP_HANDOFF_MAGIC;

  sprintf(ip_str, "%d.%d.%d.%d", ip[0], ip[1], ip[2], ip[3]);
  return true;
}

size_t EthernetTcpConsole::write(uint8_t c) {
  Serial.write(c);
  if (_ready && _client && _client.connected()) _client.write(c);
  return 1;
}

size_t EthernetTcpConsole::write(const uint8_t* buf, size_t size) {
  Serial.write(buf, size);
  if (_ready && _client && _client.connected()) _client.write(buf, size);
  return size;
}

int EthernetTcpConsole::available() {
  int n = Serial.available();
  if (n > 0) return n;
  if (_ready && _client && _client.connected()) return _client.available();
  return 0;
}

int EthernetTcpConsole::read() {
  if (Serial.available()) return Serial.read();
  if (_ready && _client && _client.connected() && _client.available()) return _client.read();
  return -1;
}

int EthernetTcpConsole::peek() {
  if (Serial.available()) return Serial.peek();
  if (_ready && _client && _client.connected() && _client.available()) return _client.peek();
  return -1;
}

void EthernetTcpConsole::flush() {
  Serial.flush();
}

#endif
