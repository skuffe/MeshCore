#ifdef WITH_BACKHAUL_CENTRAL

#include "BleNusRelay.h"
#include <helpers/bridges/EthernetTcpConsole.h>   // gate BLE bringup on EthConsole.isReady()
#include <string.h>
#ifdef WITH_NET_BRIDGE
  #include <MeshCore.h>                            // PUB_KEY_SIZE
  #include <Mesh.h>                                // mesh::RTCClock (backhaul time push)
  #include <helpers/bridges/MqttPublisher.h>       // publish the relayed observer feed
  #include <helpers/console/CliextConfig.h>        // observer table (auto-populate from IDENTITY)
#endif

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

  // Generous RX FIFO (default is only 256 B). The notify callback drops bytes when the
  // FIFO is full, and we only drain it once per loop() pass — so a large burst from the
  // peripheral (e.g. a ~2.5 KB `node <edge> help` dump) overflows the default and clips
  // the tail. 4 KB comfortably bridges the gap between drains for any single console reply.
  _clientUart = new BLEClientUart(BLE_RELAY_RX_FIFO);  // heap-alloc now (see header) — never at static-init
  _clientUart->begin();

  Bluefruit.Central.setConnectCallback(onConnect);
  Bluefruit.Central.setDisconnectCallback(onDisconnect);

  Bluefruit.Scanner.setRxCallback(onScan);
  Bluefruit.Scanner.restartOnDisconnect(true);
  Bluefruit.Scanner.setInterval(160, 80);          // units: 0.625ms
  // NOTE: do NOT filterUuid(BLEUART_UUID_SERVICE) here. The peripheral advertises the
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
  // lease, so we must NOT touch Ethernet before isReady() — an SPI transaction on
  // the un-begun SPIM hard-wedges the MCU. Gating here also means BLE only comes up
  // after :5000 is serving, keeping boot OTA-recoverable. (We no longer open any
  // socket of our own — admin + feed both ride EthConsole's :5000 now — so there is
  // nothing to re-arm across an EthConsole self-heal.)
  if (!_started) {
    if (!EthConsole.isReady()) return;
    startBle();
    _started = true;
  }

#ifdef WITH_NET_BRIDGE
  // Backhaul link dropped → flag the peripheral observer offline (retained), once; re-arm the
  // time push so the next link-up syncs the peripheral's clock promptly.
  if (!_linkUp && _peer_online) {
    if (_peer_obs_idx >= 0) MqttPub.publishObserverStatus((uint8_t)_peer_obs_idx, false);
    _peer_online = false;
  }
  if (!_linkUp) _next_time_push = 0;
#endif

  if (!_linkUp || !_clientUart) return;

#ifdef WITH_NET_BRIDGE
  // Backhaul time sync: push this node's NTP-synced UTC to the peripheral (FRAME_TIME) so it
  // can stamp observations with real time. Gated on the config toggle; fires promptly on
  // link-up then every 5 min. The peripheral ignores an implausible epoch.
  if (_rtc && cliext::config().backhaul_timesync) {
    uint32_t now_ms = millis();
    if ((long)(now_ms - _next_time_push) >= 0) {
      uint32_t epoch = _rtc->getCurrentTime();
      if (epoch >= backhaul::EPOCH_SANE_MIN) {
        backhaul::TimeBody tb; tb.epoch = epoch;
        uint8_t frame[5 + sizeof(tb)];
        size_t fn = backhaul::encode(frame, sizeof(frame), backhaul::FRAME_TIME,
                                     (const uint8_t*)&tb, (uint16_t)sizeof(tb));
        if (fn) _clientUart->write(frame, fn);
        _next_time_push = now_ms + 300000;   // 5 min
      }
    }
  }
#endif

  // NUS -> demux. The peripheral emits only structured frames now (observations/identity/
  // status, and FRAME_CONSOLE admin replies); any stray PASS text is forwarded to :5000.
  uint8_t buf[128];
  int n;
  while ((n = _clientUart->read(buf, sizeof(buf))) > 0) {
    for (int k = 0; k < n; k++) {
#ifdef WITH_NET_BRIDGE
      backhaul::Parser::Result r = _parser.feed(buf[k]);
      if (r == backhaul::Parser::PASS) {
        EthConsole.write(buf[k]);          // stray peripheral text → :5000
      } else if (r == backhaul::Parser::FRAME) {
        dispatchFrame();
      }
#else
      EthConsole.write(buf[k]);
#endif
    }
  }
}

// Send a console command line to the remote peripheral (FRAME_CONSOLE). The reply comes
// back asynchronously as a FRAME_CONSOLE and dispatchFrame() prints it to :5000.
void BleNusRelay::sendConsole(const char* cmd) {
  if (!_linkUp || !_clientUart || !cmd) return;
  size_t len = strlen(cmd);
  if (len > backhaul::Parser::MAX_PAYLOAD - 1) len = backhaul::Parser::MAX_PAYLOAD - 1;
  uint8_t frame[5 + backhaul::Parser::MAX_PAYLOAD];
  size_t fn = backhaul::encode(frame, sizeof(frame), backhaul::FRAME_CONSOLE,
                               (const uint8_t*)cmd, (uint16_t)len);
  if (fn) _clientUart->write(frame, fn);
}

#ifdef WITH_NET_BRIDGE
// Decode one complete backhaul frame from the remote peripheral observer and route it to
// the MQTT publisher under the peripheral's observer identity (auto-resolved from IDENTITY).
void BleNusRelay::dispatchFrame() {
  const uint8_t* p = _parser.payload();
  uint16_t       len = _parser.len();

  switch (_parser.type()) {
    case backhaul::FRAME_IDENTITY: {
      if (len < PUB_KEY_SIZE) break;
      char pubhex[2 * PUB_KEY_SIZE + 1];
      for (int i = 0; i < PUB_KEY_SIZE; i++) sprintf(pubhex + i * 2, "%02X", p[i]);
      // Tail = NUL-separated fields name\0model\0firmware\0radio (BackhaulFrame.h). Copy with
      // a trailing NUL so each field is a valid C-string; trailing fields may be absent (an
      // old, pre-hw peripheral sends just the name with no NULs → model/firmware/radio stay empty).
      char tail[80];
      size_t tlen = len - PUB_KEY_SIZE;
      if (tlen >= sizeof(tail)) tlen = sizeof(tail) - 1;
      memcpy(tail, p + PUB_KEY_SIZE, tlen); tail[tlen] = 0;
      const char* fields[4] = { tail, "", "", "" };   // name, model, firmware, radio
      for (size_t i = 0, f = 1; i < tlen && f < 4; i++) {
        if (tail[i] == 0) fields[f++] = tail + i + 1;  // next field starts after each NUL
      }
      const char* name = fields[0];

      int idx = cliext::observerUpsert(pubhex, name[0] ? name : nullptr, cliext::OBS_RELAY);
      if (idx >= 0) {
        MqttPub.setObserverHw((uint8_t)idx, fields[1], fields[2], fields[3]);
        if (idx != _peer_obs_idx) cliext::configSave();   // newly added observer → persist
        _peer_obs_idx = idx;
        if (!_peer_online) { MqttPub.publishObserverStatus((uint8_t)idx, true); _peer_online = true; }
      }
      break;
    }
    case backhaul::FRAME_OBSERVATION: {
      if (_peer_obs_idx < 0 || len < sizeof(backhaul::ObsHeader)) break;   // need identity first
      backhaul::ObsHeader h;
      memcpy(&h, p, sizeof(h));
      const uint8_t* wire = p + sizeof(h);
      if (sizeof(h) + h.wire_len > len) break;
      MqttPub.publishObservation((uint8_t)_peer_obs_idx, h.is_tx, wire, h.wire_len,
                                 h.snr, h.rssi, h.score, h.epoch);
      break;
    }
    case backhaul::FRAME_STATUS: {
      // Mast STATUS frame: record the peripheral's own uptime, then (re)publish its retained
      // online status so the JSON carries that uptime instead of this central's millis().
      if (_peer_obs_idx < 0 || len < sizeof(backhaul::StatusBody)) break;
      backhaul::StatusBody sb;
      memcpy(&sb, p, sizeof(sb));
      MqttPub.setObserverUptime((uint8_t)_peer_obs_idx, sb.uptime_secs);
      MqttPub.publishObserverStatus((uint8_t)_peer_obs_idx, true);
      _peer_online = true;
      break;
    }
    case backhaul::FRAME_CONSOLE: {
      // Remote-admin reply from the peripheral → print to the :5000 console (replaces :5001),
      // tagged with the peripheral's CONFIGURED node name (pubkey-prefix fallback) from the
      // observer table — never a hardcoded "mast". Same identity `node list` shows.
      char tag[40];
      if (_peer_obs_idx >= 0 && cliext::config().observers[_peer_obs_idx].name[0]) {
        snprintf(tag, sizeof tag, "[%s] ", cliext::config().observers[_peer_obs_idx].name);
      } else if (_peer_obs_idx >= 0) {
        char id8[9]; strncpy(id8, cliext::config().observers[_peer_obs_idx].pubkey_hex, 8); id8[8] = 0;
        snprintf(tag, sizeof tag, "[%s] ", id8);
      } else {
        strcpy(tag, "[node] ");
      }
      EthConsole.print(tag);
      EthConsole.write(p, len);
      EthConsole.println();
      break;
    }
    default: break;
  }
}
#endif

#endif
