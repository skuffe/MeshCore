#ifdef WITH_BACKHAUL_PERIPHERAL

#include "BleConsole.h"

// Advertising interval (units: 0.625ms) and watchdog cadence.
#define BLE_ADV_INTERVAL_MIN  32     // 20ms
#define BLE_ADV_INTERVAL_MAX  244    // 152.5ms
#define BLE_ADV_FAST_TIMEOUT  30     // seconds
#define BLE_ADV_WATCHDOG_MS   10000UL

bool g_ble_console_up = false;
BleConsoleStream BleConsole;
volatile uint16_t BleConsoleStream::_conn_handle = BLE_CONN_HANDLE_INVALID;

void BleConsoleStream::begin() {
  // Heap-alloc the BLE objects now (after main()) — see header: their ctors
  // malloc and fault during static-init.
  _bleuart = new BLEUart();
  _bledfu  = new BLEDfu();

  Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);
  Bluefruit.begin(1, 0);                 // 1 peripheral, 0 central
  Bluefruit.setTxPower(BLE_TX_POWER);
  Bluefruit.setName(BLE_CONSOLE_NAME);

  // NUS console — left open (no MITM/pairing) so the central relay connects
  // freely on a private mesh. Tighten with setPermission(SECMODE_ENC_WITH_MITM)
  // if over-the-air exposure is a concern.
  _bleuart->begin();

  // Register DFU now (dormant). Keeps the bootloader's BLE DFU reachable
  // without a second Bluefruit.begin() at `start ota` time.
  _bledfu->begin();

  // Arm RSSI monitoring whenever a central connects (read back by rssi()).
  Bluefruit.Periph.setConnectCallback(onConnect);

  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addService(*_bleuart);
  Bluefruit.ScanResponse.addName();
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(BLE_ADV_INTERVAL_MIN, BLE_ADV_INTERVAL_MAX);
  Bluefruit.Advertising.setFastTimeout(BLE_ADV_FAST_TIMEOUT);
  Bluefruit.Advertising.start(0);        // 0 = advertise forever

  g_ble_console_up = true;
}

void BleConsoleStream::loop() {
  // Advertising watchdog: if disconnected and somehow not advertising, restart.
  if (Bluefruit.connected() == 0) {
    unsigned long now = millis();
    if (now - _last_adv_check >= BLE_ADV_WATCHDOG_MS) {
      _last_adv_check = now;
      if (!Bluefruit.Advertising.isRunning()) Bluefruit.Advertising.start(0);
    }
  }
}

void BleConsoleStream::onConnect(uint16_t conn_handle) {
  _conn_handle = conn_handle;
  BLEConnection* conn = Bluefruit.Connection(conn_handle);
  if (conn) conn->monitorRssi();
}

int8_t BleConsoleStream::rssi() const {
  if (Bluefruit.connected() == 0 || _conn_handle == BLE_CONN_HANDLE_INVALID) return 0;
  BLEConnection* conn = Bluefruit.Connection(_conn_handle);
  return conn ? conn->getRssi() : 0;
}

void BleConsoleStream::prepareForDfu(char* reply) {
  // Free the single peripheral slot so a DFU client can connect.
  for (uint8_t h = 0; h < BLE_MAX_CONNECTION; h++) {
    BLEConnection* conn = Bluefruit.Connection(h);
    if (conn && conn->connected()) conn->disconnect();
  }
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.start(0);

  uint8_t mac[6];
  Bluefruit.getAddr(mac);
  sprintf(reply, "OK - mac: %02X:%02X:%02X:%02X:%02X:%02X",
          mac[5], mac[4], mac[3], mac[2], mac[1], mac[0]);
}

size_t BleConsoleStream::write(uint8_t c) {
  Serial.write(c);
  if (_bleuart && Bluefruit.connected() > 0) _bleuart->write(c);
  return 1;
}

size_t BleConsoleStream::write(const uint8_t* buf, size_t size) {
  Serial.write(buf, size);
  // Send the WHOLE buffer over NUS. BLEUart::write can short-write when its TX FIFO is
  // full; one call could truncate a large frame (a max-size ~270 B observation spans many
  // 20 B notifications) → the central's checksum fails and the frame is dropped, while
  // tiny IDENTITY/STATUS frames slip through. Loop until all bytes are queued, yielding so
  // the SoftDevice can flush notifications; the guard bounds it if the link stalls.
  if (_bleuart && Bluefruit.connected() > 0) {
    size_t off = 0;
    uint32_t guard = 0;
    while (off < size && guard++ < 2000) {
      size_t w = _bleuart->write(buf + off, size - off);
      off += w;
      if (off < size) yield();   // let the BLE stack drain queued notifications
    }
  }
  return size;
}

int BleConsoleStream::available() {
  int n = Serial.available();
  if (n > 0) return n;
  return _bleuart ? _bleuart->available() : 0;
}

int BleConsoleStream::read() {
  if (Serial.available()) return Serial.read();
  if (_bleuart && _bleuart->available()) return _bleuart->read();
  return -1;
}

int BleConsoleStream::peek() {
  if (Serial.available()) return Serial.peek();
  if (_bleuart && _bleuart->available()) return _bleuart->peek();
  return -1;
}

void BleConsoleStream::flush() {
  Serial.flush();
}

#endif
