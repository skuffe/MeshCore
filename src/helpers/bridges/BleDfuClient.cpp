#include "BleDfuClient.h"
#ifdef WITH_BACKHAUL_CENTRAL
#include <bluefruit.h>
#include <string.h>
#include <stdio.h>
#include "BleNusRelay.h"

namespace bledfu {
namespace {

// Legacy SDK11 DFU 128-bit UUIDs (base 00001530-1212-EFDE-1523-785FEABCD123), little-endian for
// Bluefruit's BLEUuid(uint8_t[16]). Bytes [12]/[13] carry the 0x15xx 16-bit part (30/31/32).
const uint8_t UUID_SVC[16]  = {0x23,0xD1,0xBC,0xEA,0x5F,0x78,0x23,0x15,0xDE,0xEF,0x12,0x12,0x30,0x15,0x00,0x00};
const uint8_t UUID_CTRL[16] = {0x23,0xD1,0xBC,0xEA,0x5F,0x78,0x23,0x15,0xDE,0xEF,0x12,0x12,0x31,0x15,0x00,0x00};
const uint8_t UUID_PKT[16]  = {0x23,0xD1,0xBC,0xEA,0x5F,0x78,0x23,0x15,0xDE,0xEF,0x12,0x12,0x32,0x15,0x00,0x00};

// Stage A connects to the re-advertised app (name match — its 0x1530 is GATT-only); stage B to
// the bootloader (advertises "AdaDFU" + 0x1530). The user confirmed `start ota` flashes via nRF
// Connect, i.e. this exact buttonless flow.
const char* BOOTLOADER_DFU_NAME = "AdaDFU";
const char* APP_DFU_NAME        = BLE_RELAY_TARGET_NAME;

// Legacy DFU control-point opcodes / procedures (SDK11 ble_dfu).
enum {
  OP_START    = 0x01,  // + update-mode byte; sizes follow on the packet char
  OP_INIT     = 0x02,  // + 0x00 receive / 0x01 complete; .dat on the packet char
  OP_RECEIVE  = 0x03,  // then stream firmware on the packet char
  OP_VALIDATE = 0x04,
  OP_ACTIVATE = 0x05,
  RESP        = 0x10,  // notification: [0x10, proc, status]
  RESP_OK     = 0x01,
  MODE_APP    = 0x04,  // update-mode: application image
};

// Heap-allocated on first start() (a BLEClientCharacteristic ctor mallocs — unsafe at static init).
BLEClientService*        s_svc  = nullptr;
BLEClientCharacteristic* s_ctrl = nullptr;
BLEClientCharacteristic* s_pkt  = nullptr;
bool      s_inited = false;

State     s_state = IDLE;
uint32_t  s_since = 0;
char      s_status[72] = "";
bool      s_status_changed = false;
volatile uint16_t s_conn = BLE_CONN_HANDLE_INVALID;
volatile bool     s_connected_evt = false;
volatile bool     s_disconnected_evt = false;
char      s_advname[24] = "";
char      s_seen[80] = "";
bool      s_stage_b = false;            // onScan: false → match the app, true → the bootloader
uint8_t   s_bconn_tries = 0;            // bootloader connect attempts (retry the first races)

uint8_t   s_init[300];                  // the .dat init packet from BEGIN
uint16_t  s_init_len = 0;
uint32_t  s_image_len = 0;
uint32_t  s_received = 0;

// Control-point notification state (the bootloader replies [0x10, proc, status]).
volatile bool    s_resp_got = false;
volatile uint8_t s_resp_proc = 0, s_resp_status = 0;

void setState(State s) { s_state = s; s_since = millis(); }
void setStatus(const char* m) {
  strncpy(s_status, m, sizeof s_status - 1);
  s_status[sizeof s_status - 1] = 0;
  s_status_changed = true;
}

void noteSeen(const char* name) {
  if (!name[0] || strstr(s_seen, name)) return;
  size_t l = strlen(s_seen);
  snprintf(s_seen + l, sizeof s_seen - l, "%s%s", l ? "," : "", name);
}

void ctrlNotifyCb(BLEClientCharacteristic*, uint8_t* data, uint16_t len) {
  if (len >= 3 && data[0] == RESP) {
    s_resp_proc = data[1];
    s_resp_status = data[2];
    s_resp_got = true;
  }
}

void onScan(ble_gap_evt_adv_report_t* report) {
  s_advname[0] = 0;
  uint8_t n = Bluefruit.Scanner.parseReportByType(report, BLE_GAP_AD_TYPE_COMPLETE_LOCAL_NAME,
                                                  (uint8_t*)s_advname, sizeof s_advname - 1);
  if (n == 0)
    Bluefruit.Scanner.parseReportByType(report, BLE_GAP_AD_TYPE_SHORT_LOCAL_NAME,
                                        (uint8_t*)s_advname, sizeof s_advname - 1);
  bool match;
  if (s_stage_b)   // bootloader: "AdaDFU" / 0x1530 in the advert
    match = strstr(s_advname, BOOTLOADER_DFU_NAME) != nullptr ||
            Bluefruit.Scanner.checkReportForUuid(report, BLEUuid(UUID_SVC));
  else             // app: re-advertised under its normal name (0x1530 is GATT-only)
    match = strstr(s_advname, APP_DFU_NAME) != nullptr;
  if (match) {
    setState(s_stage_b ? B_CONNECT : A_CONNECT);
    Bluefruit.Central.connect(report);
  } else {
    noteSeen(s_advname);
    Bluefruit.Scanner.resume();
  }
}

void onConnect(uint16_t conn)    { s_conn = conn; s_connected_evt = true; }
void onDisconnect(uint16_t, uint8_t) { s_conn = BLE_CONN_HANDLE_INVALID; s_disconnected_evt = true; }

void takeoverScanner() {
  Bluefruit.Central.setConnectCallback(onConnect);
  Bluefruit.Central.setDisconnectCallback(onDisconnect);
  Bluefruit.Scanner.setRxCallback(onScan);
  Bluefruit.Scanner.restartOnDisconnect(false);
  Bluefruit.Scanner.useActiveScan(true);
  Bluefruit.Scanner.setInterval(160, 80);
  Bluefruit.Scanner.start(0);
}

// Discover the 0x1530 DFU service + CP/packet chars on the current connection; enable CP notify.
bool discoverDfu() {
  if (!s_svc->discover(s_conn)) return false;
  if (!s_ctrl->discover())      return false;
  if (!s_pkt->discover())       return false;
  s_ctrl->enableNotify();
  return true;
}

// Control-point write = WRITE_REQ (engages the bootloader/app authorize + ack). Packet char =
// WRITE_CMD (no response). cpWrite returns true on the GATT write-ack.
bool cpWrite(const uint8_t* d, uint16_t n) { return s_ctrl->write_resp(d, n) == n; }

// Write firmware to the packet char in 20-byte units (≤ the 23-byte BLE minimum MTU − 3, and a
// multiple of 4 — keeps every write_cmd one word-aligned packet for the legacy bootloader). Paces
// on the SoftDevice write-cmd TX buffers: write() returns short when they're exhausted, so yield()
// to drain and retry. Blocks briefly per chunk (DFU pauses everything else — documented).
bool pktWrite(const uint8_t* data, uint16_t len) {
  uint16_t off = 0;
  while (off < len) {
    uint16_t n = (uint16_t)(len - off);
    if (n > 20) n = 20;
    uint32_t guard = 0;
    while (s_pkt->write(data + off, n) < n) {
      yield();
      if (++guard > 200000) return false;   // link stalled
    }
    off += n;
  }
  return true;
}

// Wait helper for the C_* states: returns 1 on the expected response, -1 on bad/timeout, 0 pending.
int awaitResp(uint8_t proc, uint32_t timeout) {
  if (s_resp_got) {
    s_resp_got = false;
    return (s_resp_proc == proc && s_resp_status == RESP_OK) ? 1 : -1;
  }
  return (millis() - s_since > timeout) ? -1 : 0;
}

}  // namespace

void start(const char* target, const uint8_t* init, uint16_t init_len, uint32_t image_len) {
  (void)target;
  if (!s_inited) {
    s_svc  = new BLEClientService(BLEUuid(UUID_SVC));
    s_ctrl = new BLEClientCharacteristic(BLEUuid(UUID_CTRL));
    s_pkt  = new BLEClientCharacteristic(BLEUuid(UUID_PKT));
    s_svc->begin();
    s_ctrl->setNotifyCallback(ctrlNotifyCb);
    s_ctrl->begin();
    s_pkt->begin();
    s_inited = true;
  }
  if (init_len > sizeof s_init) init_len = sizeof s_init;
  memcpy(s_init, init, init_len);
  s_init_len = init_len;
  s_image_len = image_len;
  s_received = 0;
  s_advname[0] = s_seen[0] = 0;
  s_stage_b = false;
  s_bconn_tries = 0;
  s_resp_got = s_connected_evt = s_disconnected_evt = false;
  // Command the edge into DFU over the live NUS, then take the radio (the edge drops the link).
  BleRelay.sendConsole("start dfu");
  BleRelay.suspendForDfu();
  setStatus("commanded edge into DFU; waiting for its link to drop");
  setState(A_TRIGGER);
}

void loop() {
  uint32_t now = millis();
  switch (s_state) {

    // ---- stage A: reach the app's DFU service, write the buttonless trigger ----
    case A_TRIGGER:
      if (!BleRelay.linkUp() && now - s_since > 800) {
        setStatus("scanning for the re-advertised edge app");
        s_stage_b = false; takeoverScanner(); setState(A_SCAN);
      } else if (now - s_since > 8000) abort("edge did not drop its NUS link");
      break;
    case A_SCAN:
      if (now - s_since > 25000) { char m[80]; snprintf(m,sizeof m,"app not seen; saw:[%.50s]", s_seen); abort(m); }
      break;
    case A_CONNECT:
      if (s_connected_evt) { s_connected_evt = false;
        if (discoverDfu()) { setStatus("connected to app; discovered 0x1530 — triggering buttonless DFU"); setState(A_BUTTONLESS); }
        else abort("app: 0x1530 DFU service not found");
      } else if (now - s_since > 8000) abort("connect to app timed out");
      break;
    case A_BUTTONLESS: {
      uint8_t trig = OP_START;            // BLEDfu app: data[0]==START_DFU → reset into bootloader
      cpWrite(&trig, 1);                  // app disconnects right after (may race the ack — fine)
      s_disconnected_evt = false;
      setStatus("buttonless trigger sent; waiting for edge to reboot to bootloader");
      setState(B_SCAN);                   // wait for the bootloader's AdaDFU advert
      s_since = millis();
      break;
    }

    // ---- stage B: reconnect to the bootloader ----
    case B_SCAN:
      if (!s_stage_b) {
        // Wait for the app to actually drop (buttonless reset frees the single central slot —
        // connecting before it's released fails) AND a grace for the bootloader to boot+advertise.
        if (s_disconnected_evt && now - s_since > 1500) {
          s_stage_b = true; s_seen[0] = 0; s_bconn_tries = 0;
          takeoverScanner();
          setStatus("scanning for edge bootloader (AdaDFU / 0x1530)");
          s_since = millis();
        } else if (now - s_since > 12000) {
          abort("edge did not reset to bootloader after the trigger");
        }
      } else if (now - s_since > 30000) {
        char m[80]; snprintf(m,sizeof m,"AdaDFU not seen; saw:[%.50s]", s_seen); abort(m);
      }
      break;
    case B_CONNECT:
      if (s_connected_evt) { s_connected_evt = false;
        if (discoverDfu()) { setStatus("connected to bootloader; starting legacy DFU"); setState(C_START); }
        else abort("bootloader: 0x1530 DFU service not found");
      } else if (now - s_since > 6000) {
        // Connect didn't take — the bootloader may still be advertising. Retry a few times before
        // giving up (first attempt after a fresh reset can race the slot release).
        if (++s_bconn_tries <= 4) {
          setStatus("bootloader connect retry");
          Bluefruit.Scanner.start(0);
          setState(B_SCAN); s_stage_b = true; s_since = millis();
        } else {
          abort("connect to bootloader timed out (4 tries)");
        }
      }
      break;

    // ---- stage C: legacy DFU ----
    case C_START: {
      uint8_t cmd[2] = { OP_START, MODE_APP };
      cpWrite(cmd, 2);
      uint8_t sizes[12] = {0};            // sd=0, bl=0, app=image_len (LE)
      memcpy(sizes + 8, &s_image_len, 4);
      if (!pktWrite(sizes, 12)) { abort("START: size write failed"); break; }
      setStatus("DFU START + image sizes sent"); setState(W_START);
      break;
    }
    case W_START: { int r = awaitResp(OP_START, 6000);
      if (r == 1) setState(C_INIT); else if (r < 0) abort("START rejected/timeout"); break; }

    case C_INIT: {
      uint8_t rx[2] = { OP_INIT, 0x00 };  // receive init
      cpWrite(rx, 2);
      if (!pktWrite(s_init, s_init_len)) { abort("INIT: .dat write failed"); break; }
      uint8_t done[2] = { OP_INIT, 0x01 }; // init complete
      cpWrite(done, 2);
      setStatus("DFU INIT (.dat) sent"); setState(W_INIT);
      break;
    }
    case W_INIT: { int r = awaitResp(OP_INIT, 6000);
      if (r == 1) setState(C_RECV_BEGIN); else if (r < 0) abort("INIT rejected/timeout"); break; }

    case C_RECV_BEGIN: {
      uint8_t cmd = OP_RECEIVE; cpWrite(&cmd, 1);
      setStatus("streaming firmware to edge"); setState(RECEIVING);
      break;
    }

    case RECEIVING:
      break;   // DfuRelay feeds chunks via feedChunk(); commit() advances to W_RECV

    case W_RECV: { int r = awaitResp(OP_RECEIVE, 30000);  // bootloader replies once app_len received
      if (r == 1) setState(C_VALIDATE); else if (r < 0) abort("RECEIVE incomplete/timeout"); break; }

    case C_VALIDATE: { uint8_t cmd = OP_VALIDATE; cpWrite(&cmd, 1);
      setStatus("validating image"); setState(W_VALIDATE); break; }
    case W_VALIDATE: { int r = awaitResp(OP_VALIDATE, 10000);
      if (r == 1) setState(C_ACTIVATE); else if (r < 0) abort("VALIDATE failed (image bad)"); break; }

    case C_ACTIVATE: {
      uint8_t cmd = OP_ACTIVATE; cpWrite(&cmd, 1);   // edge resets into the new app
      setStatus("activated — edge rebooting into new firmware");
      // SUCCESS path must also hand the radio back (only abort() did before → central stuck
      // suspended, no scanning, backhaul never reconnects). The edge is rebooting; resume the NUS
      // relay so it re-links to the edge's app.
      if (s_conn != BLE_CONN_HANDLE_INVALID) Bluefruit.disconnect(s_conn);
      s_conn = BLE_CONN_HANDLE_INVALID;
      Bluefruit.Scanner.stop();
      BleRelay.resumeAfterDfu();
      setState(DONE);
      break;
    }

    default: break;   // IDLE / DONE / FAILED terminal
  }
}

bool feedChunk(const uint8_t* data, uint16_t len) {
  if (s_state != RECEIVING) return false;
  if (!pktWrite(data, len)) { abort("firmware write to edge failed"); return false; }
  s_received += len;
  return true;
}

void commit() {
  if (s_state == RECEIVING) { setStatus("all firmware sent; waiting for edge to confirm"); setState(W_RECV); }
}

void abort(const char* why) {
  if (why && *why) setStatus(why);
  if (s_conn != BLE_CONN_HANDLE_INVALID) Bluefruit.disconnect(s_conn);
  s_conn = BLE_CONN_HANDLE_INVALID;
  Bluefruit.Scanner.stop();
  BleRelay.resumeAfterDfu();
  setState(FAILED);
}

State       state()         { return s_state; }
const char* status()        { return s_status; }
bool        statusChanged() { bool c = s_status_changed; s_status_changed = false; return c; }

}  // namespace bledfu
#endif
