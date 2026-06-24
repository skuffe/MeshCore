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

// Heap-allocated on first start() — a BLEClientCharacteristic ctor mallocs, which faults during
// C++ static-init on this build (same hazard as BleNusRelay's BLEClientUart).
BLEClientService*        s_svc  = nullptr;
BLEClientCharacteristic* s_ctrl = nullptr;
BLEClientCharacteristic* s_pkt  = nullptr;
bool      s_inited = false;

State     s_state = IDLE;
uint32_t  s_since = 0;            // millis at last state change (drives timeouts)
char      s_status[72] = "";
bool      s_status_changed = false;
volatile uint16_t s_conn = BLE_CONN_HANDLE_INVALID;
volatile bool     s_connected_evt = false;   // set by onConnect, consumed in loop()
char      s_advname[24] = "";
char      s_seen[80] = "";        // distinct advert names seen while scanning (diagnostics)

// Two advert identities we may connect to, mirroring what nRF Connect does for Adafruit OTA:
//  - STAGE 1 (buttonless): the running app, re-advertised by `start dfu`/prepareForDfu under its
//    normal name (BLE_RELAY_TARGET_NAME, "MeshCore-RPT"). Its Adafruit BLEDfu 0x1530 service sits
//    in the GATT (registered at boot) but is NOT in the advert — so we match by name, then write
//    the buttonless trigger to make it reset into the bootloader.
//  - STAGE 2 (bootloader): after that reset, the bootloader advertises "AdaDFU" + 0x1530 for the
//    actual legacy DFU.
const char* BOOTLOADER_DFU_NAME = "AdaDFU";
const char* APP_DFU_NAME        = BLE_RELAY_TARGET_NAME;

void noteSeen(const char* name) {
  if (!name[0] || strstr(s_seen, name)) return;
  size_t l = strlen(s_seen);
  snprintf(s_seen + l, sizeof s_seen - l, "%s%s", l ? "," : "", name);
}

void setState(State s) { s_state = s; s_since = millis(); }
void setStatus(const char* m) {
  strncpy(s_status, m, sizeof s_status - 1);
  s_status[sizeof s_status - 1] = 0;
  s_status_changed = true;
}

void ctrlNotifyCb(BLEClientCharacteristic*, uint8_t*, uint16_t) { /* M2b: CP response opcodes */ }

void onScan(ble_gap_evt_adv_report_t* report) {
  // Match the bootloader by its 0x1530 DFU service UUID OR the confirmed "AdaDFU" advert name.
  s_advname[0] = 0;
  uint8_t n = Bluefruit.Scanner.parseReportByType(report, BLE_GAP_AD_TYPE_COMPLETE_LOCAL_NAME,
                                                  (uint8_t*)s_advname, sizeof s_advname - 1);
  if (n == 0)
    Bluefruit.Scanner.parseReportByType(report, BLE_GAP_AD_TYPE_SHORT_LOCAL_NAME,
                                        (uint8_t*)s_advname, sizeof s_advname - 1);
  // Match the re-advertised app (stage 1, by name — its 0x1530 is GATT-only, not advertised) OR
  // the bootloader (stage 2: "AdaDFU" / 0x1530 in the advert).
  bool match = strstr(s_advname, APP_DFU_NAME) != nullptr ||
               strstr(s_advname, BOOTLOADER_DFU_NAME) != nullptr ||
               Bluefruit.Scanner.checkReportForUuid(report, BLEUuid(UUID_SVC));
  if (match) {
    setState(CONNECTING);
    Bluefruit.Central.connect(report);
  } else {
    noteSeen(s_advname);          // record for the timeout diagnostic
    Bluefruit.Scanner.resume();   // keep hunting
  }
}

void onConnect(uint16_t conn) {
  s_conn = conn;
  s_connected_evt = true;   // discovery runs in loop() to keep the SoftDevice callback light
}

void onDisconnect(uint16_t conn, uint8_t reason) {
  (void)conn; (void)reason;
  s_conn = BLE_CONN_HANDLE_INVALID;
  // Expected on abort (M2a) and after ACTIVATE (M2b: edge reset into the new app).
}

void takeoverScanner() {
  Bluefruit.Central.setConnectCallback(onConnect);
  Bluefruit.Central.setDisconnectCallback(onDisconnect);
  Bluefruit.Scanner.setRxCallback(onScan);
  Bluefruit.Scanner.restartOnDisconnect(false);
  Bluefruit.Scanner.useActiveScan(true);
  Bluefruit.Scanner.setInterval(160, 80);
  Bluefruit.Scanner.start(0);
}

void doDiscover() {
  setState(DISCOVERING);
  if (!s_svc->discover(s_conn)) { abort("DFU service 0x1530 not found on edge"); return; }
  if (!s_ctrl->discover())     { abort("DFU control-point char (0x1531) not found"); return; }
  if (!s_pkt->discover())      { abort("DFU packet char (0x1532) not found"); return; }
  s_ctrl->enableNotify();
  char m[72];
  snprintf(m, sizeof m, "connected to '%.16s'; discovered DFU svc 0x1530 + CP/pkt",
           s_advname[0] ? s_advname : "?");
  setStatus(m);
  setState(READY);   // M2a end — reached the DFU service; M2b: buttonless trigger → reconnect → flash
}

}  // namespace

void begin() { /* lazy-init in start(): Bluefruit must be up first */ }

void start(const char* target) {
  (void)target;   // single-link today: the one suspended peripheral is the target
  if (!s_inited) {
    s_svc  = new BLEClientService(BLEUuid(UUID_SVC));
    s_ctrl = new BLEClientCharacteristic(BLEUuid(UUID_CTRL));
    s_pkt  = new BLEClientCharacteristic(BLEUuid(UUID_PKT));
    s_svc->begin();
    s_ctrl->setNotifyCallback(ctrlNotifyCb);
    s_ctrl->begin();   // associates with the most-recently-begun client service (s_svc)
    s_pkt->begin();
    s_inited = true;
  }
  s_advname[0] = 0;
  s_seen[0] = 0;
  // Command the edge into its bootloader DFU over the live NUS, then take the radio. The edge
  // reboots on its own (dropping the link); suspendForDfu just stops the relay reclaiming it.
  BleRelay.sendConsole("start dfu");
  BleRelay.suspendForDfu();
  setStatus("commanded edge into DFU; waiting for its link to drop");
  setState(TRIGGERING);
}

void loop() {
  uint32_t now = millis();
  switch (s_state) {
    case TRIGGERING:
      if (!BleRelay.linkUp() && now - s_since > 800) {
        setStatus("scanning for edge DFU advert (app re-advertised by start dfu)");
        takeoverScanner();
        setState(SCANNING);
      } else if (now - s_since > 8000) {
        abort("edge did not enter DFU (NUS link never dropped)");
      }
      break;
    case SCANNING:
      if (now - s_since > 25000) {
        char m[80];
        snprintf(m, sizeof m, "no DFU advert in 25 s; saw: [%.56s]", s_seen[0] ? s_seen : "nothing");
        abort(m);
      }
      break;
    case CONNECTING:
      if (s_connected_evt) { s_connected_evt = false; doDiscover(); }
      else if (now - s_since > 8000) abort("connect to edge bootloader timed out");
      break;
    default:
      break;   // DISCOVERING resolves synchronously; READY/FAILED/DONE are terminal
  }
}

void abort(const char* why) {
  if (why && *why) setStatus(why);
  if (s_conn != BLE_CONN_HANDLE_INVALID) Bluefruit.disconnect(s_conn);
  s_conn = BLE_CONN_HANDLE_INVALID;
  Bluefruit.Scanner.stop();
  BleRelay.resumeAfterDfu();   // hand the radio back to the NUS relay
  setState(FAILED);
}

State       state()         { return s_state; }
const char* status()        { return s_status; }
bool        statusChanged() { bool c = s_status_changed; s_status_changed = false; return c; }

}  // namespace bledfu
#endif
