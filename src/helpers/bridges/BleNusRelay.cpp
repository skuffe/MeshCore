#ifdef WITH_BACKHAUL_CENTRAL

#include "BleNusRelay.h"
#include <helpers/bridges/EthernetTcpConsole.h>   // gate BLE bringup on EthConsole.isReady()
#include <string.h>

BleNusRelay BleRelay;
static BleNusRelay* s_instance = nullptr;

void BleNusRelay::begin() {
  // Deliberately does NOT touch BLE. BLE central init is deferred to loop() and
  // gated on ethernet being up — so if Bluefruit faults, it does so AFTER the
  // node is reachable (DHCP done, :5000 serving), keeping it OTA-recoverable.
  // (A fault here in setup() bricks the node: setup never returns → no DHCP.)
  s_instance = this;
}

void BleNusRelay::startBle() {
  Serial.println("BleRelay: bringing up BLE central...");
  Bluefruit.begin(1, 1);              // 1 peripheral (unused) + 1 central
  Bluefruit.setTxPower(BLE_TX_POWER);
  Bluefruit.setName("MeshCore-Relay");
  Serial.println("BleRelay: Bluefruit up");

  _clientUart = new BLEClientUart();  // heap-alloc now (see header) — never at static-init
  _clientUart->begin();

  Bluefruit.Central.setConnectCallback(onConnect);
  Bluefruit.Central.setDisconnectCallback(onDisconnect);

  Bluefruit.Scanner.setRxCallback(onScan);
  Bluefruit.Scanner.restartOnDisconnect(true);
  Bluefruit.Scanner.setInterval(160, 80);          // units: 0.625ms
  // NOTE: do NOT filterUuid(BLEUART_UUID_SERVICE) here. The mast advertises the
  // NUS UUID in the PRIMARY advert but its name in the SCAN RESPONSE — and the
  // UUID filter only passes the primary report, which carries no name, so the
  // name match in onScan would always fail (the node never connects). Instead we
  // match on the name (delivered by active scan's scan-response) and guarantee
  // the NUS service post-connect via _clientUart->discover() (onConnect drops the
  // link if the service isn't present).
  Bluefruit.Scanner.useActiveScan(true);           // needed to receive scan responses (the name)
  Bluefruit.Scanner.start(0);                       // 0 = scan forever
  Serial.println("BleRelay: scanning for NUS peripheral");
}

void BleNusRelay::onScan(ble_gap_evt_adv_report_t* report) {
  // Match the configured node name (carried in the scan response). We don't
  // pre-filter by UUID (see startBle) — the NUS service is confirmed after
  // connecting, in onConnect via discover().
  char name[32] = {0};
  uint8_t len = Bluefruit.Scanner.parseReportByType(
      report, BLE_GAP_AD_TYPE_COMPLETE_LOCAL_NAME, (uint8_t*)name, sizeof(name) - 1);
  if (len == 0) {
    len = Bluefruit.Scanner.parseReportByType(
        report, BLE_GAP_AD_TYPE_SHORT_LOCAL_NAME, (uint8_t*)name, sizeof(name) - 1);
  }

  if (strstr(name, BLE_RELAY_TARGET_NAME) != nullptr) {
    Bluefruit.Central.connect(report);
  } else {
    Bluefruit.Scanner.resume();   // keep looking
  }
}

void BleNusRelay::onConnect(uint16_t conn_handle) {
  if (!s_instance || !s_instance->_clientUart) return;
  if (s_instance->_clientUart->discover(conn_handle)) {
    s_instance->_clientUart->enableTXD();   // subscribe to NUS notifications
    s_instance->_conn_handle = conn_handle;
    BLEConnection* conn = Bluefruit.Connection(conn_handle);
    if (conn) conn->monitorRssi();          // enable RSSI readback for `backhaul`
    s_instance->_linkUp = true;
  } else {
    Bluefruit.disconnect(conn_handle);      // not the node we want
  }
}

void BleNusRelay::onDisconnect(uint16_t conn_handle, uint8_t reason) {
  (void)conn_handle;
  (void)reason;
  if (s_instance) {
    s_instance->_linkUp = false;
    s_instance->_conn_handle = BLE_CONN_HANDLE_INVALID;
  }
  // Scanner.restartOnDisconnect(true) resumes scanning automatically.
}

int8_t BleNusRelay::rssi() const {
  if (!_linkUp || _conn_handle == BLE_CONN_HANDLE_INVALID) return 0;
  BLEConnection* conn = Bluefruit.Connection(_conn_handle);
  return conn ? conn->getRssi() : 0;
}

void BleNusRelay::loop() {
  // One-time lazy init, gated on EthConsole reporting ready. EthConsole owns the
  // W5100S: it doesn't begin() the chip's SPI peripheral until its first DHCP
  // lease, so we must NOT touch Ethernet (not even localIP()) before isReady() —
  // an SPI transaction on the un-begun SPIM hard-wedges the MCU. Gating here also
  // means BLE only comes up after :5000 is serving, keeping boot OTA-recoverable.
  if (!_started) {
    if (!EthConsole.isReady()) return;
    startBle();
    _server.begin();
    _started = true;
    _eth_was_ready = true;
  }

  // Re-arm the :5001 listener after an ethernet self-heal. On link/lease loss
  // EthConsole resets the W5100S, wiping every socket — including our listening
  // socket — so re-open it on the isReady() false->true edge. BLE is unaffected
  // (independent radio), so it is left up.
  bool eth_ready = EthConsole.isReady();
  if (eth_ready && !_eth_was_ready) {
    if (_client) _client.stop();
    _server.begin();
  }
  _eth_was_ready = eth_ready;

  // While EthConsole is down (resetting the chip + re-acquiring DHCP) the W5100S
  // sockets are invalid — do NOT touch them, or we stomp its recovery. Our :5001
  // listener is re-armed on the isReady() edge above once it is back.
  if (!eth_ready) return;

  // Accept / refresh the single TCP client.
  EthernetClient incoming = _server.accept();
  if (incoming) {
    if (_client) _client.stop();   // new connection displaces the old one
    _client = incoming;
  }

  if (!_linkUp || !_clientUart) return;

  uint8_t buf[128];
  int n;

  // NUS -> TCP (packet log + CLI replies from the remote node).
  while ((n = _clientUart->read(buf, sizeof(buf))) > 0) {
    if (_client && _client.connected()) _client.write(buf, n);
  }

  // TCP -> NUS (CLI commands to the remote node).
  if (_client && _client.connected()) {
    while ((n = _client.read(buf, sizeof(buf))) > 0) {
      _clientUart->write(buf, n);
    }
  }
}

#endif
