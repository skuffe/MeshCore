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

  // Free the W5100S socket the moment the peer disconnects (don't hold it until a new
  // client is accepted) — a lingering CLOSE_WAIT socket starves the 4-socket chip and
  // gets both :5000 and :5001 refused. Same fix as EthernetTcpConsole.
  if (_client && !_client.connected()) _client.stop();

  // Accept / refresh the single TCP client.
  EthernetClient incoming = _server.accept();
  if (incoming) {
    if (_client) _client.stop();   // new connection displaces the old one
    _client = incoming;
  }

#ifdef WITH_NET_BRIDGE
  // Backhaul link dropped → flag the mast observer offline (retained), once; re-arm the
  // time push so the next link-up syncs the mast's clock promptly.
  if (!_linkUp && _mast_online) {
    if (_mast_obs_idx >= 0) MqttPub.publishObserverStatus((uint8_t)_mast_obs_idx, false);
    _mast_online = false;
  }
  if (!_linkUp) _next_time_push = 0;
#endif

  if (!_linkUp || !_clientUart) return;

#ifdef WITH_NET_BRIDGE
  // Backhaul time sync: push this node's NTP-synced UTC to the mast (FRAME_TIME) so it can
  // stamp observations with real time. Gated on the config toggle; fires promptly on
  // link-up then every 5 min. The mast ignores an implausible epoch.
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

  uint8_t buf[128];
  int n;

  // NUS -> TCP (packet log + CLI replies from the remote node), with structured-frame
  // demux: 0x1E-led frames are decoded (observations/identity/status) and stripped;
  // everything else is plain console text, forwarded to the :5001 client. Text is
  // coalesced into `txt` so we are not writing the TCP socket a byte at a time.
  uint8_t txt[128];
  int     txt_n = 0;
  while ((n = _clientUart->read(buf, sizeof(buf))) > 0) {
    for (int k = 0; k < n; k++) {
#ifdef WITH_NET_BRIDGE
      backhaul::Parser::Result r = _parser.feed(buf[k]);
      if (r == backhaul::Parser::PASS) {
        txt[txt_n++] = buf[k];
        if (txt_n == (int)sizeof(txt)) { if (_client && _client.connected()) _client.write(txt, txt_n); txt_n = 0; }
      } else if (r == backhaul::Parser::FRAME) {
        dispatchFrame();
      }
#else
      txt[txt_n++] = buf[k];
      if (txt_n == (int)sizeof(txt)) { if (_client && _client.connected()) _client.write(txt, txt_n); txt_n = 0; }
#endif
    }
    if (txt_n && _client && _client.connected()) { _client.write(txt, txt_n); txt_n = 0; }
  }

  // TCP -> NUS (CLI commands to the remote node).
  if (_client && _client.connected()) {
    while ((n = _client.read(buf, sizeof(buf))) > 0) {
      _clientUart->write(buf, n);
    }
  }
}

#ifdef WITH_NET_BRIDGE
// Decode one complete backhaul frame from the remote (mast) observer and route it to
// the MQTT publisher under the mast's observer identity (auto-resolved from IDENTITY).
void BleNusRelay::dispatchFrame() {
  const uint8_t* p = _parser.payload();
  uint16_t       len = _parser.len();

  switch (_parser.type()) {
    case backhaul::FRAME_IDENTITY: {
      if (len < PUB_KEY_SIZE) break;
      char pubhex[2 * PUB_KEY_SIZE + 1];
      for (int i = 0; i < PUB_KEY_SIZE; i++) sprintf(pubhex + i * 2, "%02X", p[i]);
      char name[32];
      size_t nlen = len - PUB_KEY_SIZE;
      if (nlen >= sizeof(name)) nlen = sizeof(name) - 1;
      memcpy(name, p + PUB_KEY_SIZE, nlen); name[nlen] = 0;

      int idx = cliext::observerUpsert(pubhex, nlen ? name : nullptr, cliext::OBS_RELAY);
      if (idx >= 0) {
        if (idx != _mast_obs_idx) cliext::configSave();   // newly added observer → persist
        _mast_obs_idx = idx;
        if (!_mast_online) { MqttPub.publishObserverStatus((uint8_t)idx, true); _mast_online = true; }
      }
      break;
    }
    case backhaul::FRAME_OBSERVATION: {
      if (_mast_obs_idx < 0 || len < sizeof(backhaul::ObsHeader)) break;   // need identity first
      backhaul::ObsHeader h;
      memcpy(&h, p, sizeof(h));
      const uint8_t* wire = p + sizeof(h);
      if (sizeof(h) + h.wire_len > len) break;
      MqttPub.publishObservation((uint8_t)_mast_obs_idx, h.is_tx, wire, h.wire_len,
                                 h.snr, h.rssi, h.score, h.epoch);
      break;
    }
    case backhaul::FRAME_STATUS: {
      // Mast STATUS frame (uptime). Online status is already published on the IDENTITY/
      // link edge; a STATUS frame simply reaffirms the link is live. Stats enrichment
      // (carrying uptime into the status JSON) is a follow-on.
      if (_mast_obs_idx >= 0 && !_mast_online) {
        MqttPub.publishObserverStatus((uint8_t)_mast_obs_idx, true);
        _mast_online = true;
      }
      break;
    }
    default: break;
  }
}
#endif

#endif
