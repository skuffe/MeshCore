// Fork-vendored copy of examples/simple_repeater/main.cpp.
//
// We keep our BLE-console additions here instead of editing the upstream
// example so `make rebase` never conflicts on this file. Everything else in the
// role (MyMesh.cpp, UITask.cpp, RateLimiter.h) is still compiled from
// examples/simple_repeater/ and continues to track upstream — only main.cpp is
// vendored. The env adds `-I examples/simple_repeater` so the includes below
// resolve. If upstream changes simple_repeater/main.cpp, re-sync this copy.
//
// Delta vs. upstream: WITH_BACKHAUL_PERIPHERAL -> CONSOLE macro + BleConsole begin/loop
// + CLI routed through CONSOLE (see docs/remote-observer-backhaul.md §4).

#include <Arduino.h>   // needed for PlatformIO
#include <Mesh.h>

#include "MyMesh.h"
#include <helpers/console/CLIExtensions.h>   // feature-gated extension commands

#ifdef WITH_BACKHAUL_PERIPHERAL
  #include <helpers/nrf52/BleConsole.h>
  #include <helpers/Observer.h>   // unified emission — frames observations over the backhaul
  #define CONSOLE BleConsole   // CLI + packet logs mirrored on a BLE NUS peripheral
#else
  #define CONSOLE Serial
#endif

#ifdef DISPLAY_CLASS
  #include "UITask.h"
  static UITask ui_task(display);
#endif

StdRNG fast_rng;
SimpleMeshTables tables;

#ifdef PACKET_LOG_STREAM
  #include <helpers/PacketLogConsole.h>
  // Re-emit the packet-log console feed from the virtual hooks to
  // PACKET_LOG_STREAM (the mast's BLE NUS, =BleConsole). Upstream Dispatcher
  // prints these RX/TX summary lines to Serial only and the simple_repeater
  // MyMesh leaves the hooks as no-ops; doing it here keeps src/Dispatcher.cpp and
  // examples/simple_repeater/ pristine while still shipping the feed over BLE.
  class ConsoleLoggingMesh : public MyMesh {
  public:
    using MyMesh::MyMesh;

    // Route the console CLI (which on the mast is the BLE NUS backhaul) through the
    // feature-gated extension commands before the upstream CommonCLI. simple_repeater's
    // MyMesh::handleCommand stays pristine; the mast gains backhaul/log/eth/tcpota here.
    void handleCommand(uint32_t sender_timestamp, char* command, char* reply) override {
      // Extension commands serve BOTH the local/backhaul console (sender_timestamp
      // == 0, the mast's CONSOLE = BLE NUS) and the over-the-air admin path:
      // onPeerDataRecv only routes CLI text here for clients that pass isAdmin(),
      // so companion remote-management can run backhaul/log/get-set too. cliext's
      // set/get fall through to upstream CommonCLI for keys it doesn't own.
      if (cliext::handleCommand(command, reply)) return;
      MyMesh::handleCommand(sender_timestamp, command, reply);
    }
  protected:
    // logRxRaw stages the on-air radio bytes + metrics so the imminent logRx frames the
    // exact `raw`/SNR/RSSI (same staging contract as the net-bridge node).
    void logRxRaw(float snr, float rssi, const uint8_t raw[], int len) override {
      MyMesh::logRxRaw(snr, rssi, raw, len);
#ifdef WITH_BACKHAUL_PERIPHERAL
      observer::onRawRx(raw, len, snr, rssi);
#endif
    }
    void logRx(mesh::Packet* pkt, int len, float score) override {
      MyMesh::logRx(pkt, len, score);
#ifndef WITH_BACKHAUL_PERIPHERAL
      // Human text feed. On the backhaul mast this is dropped: the feed now rides the
      // structured OBSERVATION frames (→ MQTT), and the old :5001 text sink is gone — so
      // emitting text here would just be wasted BLE bandwidth contending with the frames.
      if (cliext::g_packet_dump_enabled)   // runtime observation toggle (`log on|off`)
        meshconsole::logRx(PACKET_LOG_STREAM, getLogDateTime(), *_radio, pkt, len, score);
#else
      observer::onPacketRx(pkt, score);    // frame to the central (gated on `log on|off`)
#endif
    }
    void logTx(mesh::Packet* pkt, int len) override {
      MyMesh::logTx(pkt, len);
#ifndef WITH_BACKHAUL_PERIPHERAL
      if (cliext::g_packet_dump_enabled)
        meshconsole::logTx(PACKET_LOG_STREAM, getLogDateTime(), pkt, len);
#else
      observer::onPacketTx(pkt);
#endif
    }
  };
  ConsoleLoggingMesh the_mesh(board, radio_driver, *new ArduinoMillis(), fast_rng, rtc_clock, tables);
#else
  MyMesh the_mesh(board, radio_driver, *new ArduinoMillis(), fast_rng, rtc_clock, tables);
#endif

#ifdef WITH_BACKHAUL_PERIPHERAL
// Remote-admin: a central runs commands over the backhaul (FRAME_CONSOLE). Route them
// through the same dispatcher as the local console and return the reply. Replaces the old
// :5001 NUS-passthrough; sender_timestamp 0 = the local/admin path (isAdmin-gated upstream).
static void mastConsoleExec(const char* cmd, char* reply, size_t cap) {
  (void)cap;
  char buf[160];
  strncpy(buf, cmd, sizeof(buf) - 1); buf[sizeof(buf) - 1] = 0;
  reply[0] = 0;
  the_mesh.handleCommand(0, buf, reply);
}
#endif

void halt() {
  while (1) ;
}

static char command[160];

// For power saving
unsigned long POWERSAVING_FIRSTSLEEP_SECS = 120; // The first sleep (if enabled) from boot

#if defined(PIN_USER_BTN) && defined(_SEEED_SENSECAP_SOLAR_H_)
static unsigned long userBtnDownAt = 0;
#define USER_BTN_HOLD_OFF_MILLIS 1500
#endif

void setup() {
  Serial.begin(115200);
  delay(1000);

  board.begin();

#if defined(MESH_DEBUG) && defined(NRF52_PLATFORM)
  // give some extra time for serial to settle so
  // boot debug messages can be seen on terminal
  delay(5000);
#endif

#ifdef DISPLAY_CLASS
  if (display.begin()) {
    display.startFrame();
    display.setCursor(0, 0);
    display.print("Please wait...");
    display.endFrame();
  }
#endif

  if (!radio_init()) {
    MESH_DEBUG_PRINTLN("Radio init failed!");
    halt();
  }

  fast_rng.begin(radio_driver.getRngSeed());

  FILESYSTEM* fs;
#if defined(NRF52_PLATFORM) || defined(STM32_PLATFORM)
  InternalFS.begin();
  fs = &InternalFS;
  IdentityStore store(InternalFS, "");
#elif defined(ESP32)
  SPIFFS.begin(true);
  fs = &SPIFFS;
  IdentityStore store(SPIFFS, "/identity");
#elif defined(RP2040_PLATFORM)
  LittleFS.begin();
  fs = &LittleFS;
  IdentityStore store(LittleFS, "/identity");
  store.begin();
#else
  #error "need to define filesystem"
#endif
  if (!store.load("_main", the_mesh.self_id)) {
    MESH_DEBUG_PRINTLN("Generating new keypair");
    the_mesh.self_id = radio_new_identity();   // create new random identity
    int count = 0;
    while (count < 10 && (the_mesh.self_id.pub_key[0] == 0x00 || the_mesh.self_id.pub_key[0] == 0xFF)) {  // reserved id hashes
      the_mesh.self_id = radio_new_identity(); count++;
    }
    store.save("_main", the_mesh.self_id);
  }

  Serial.print("Repeater ID: ");
  mesh::Utils::printHex(Serial, the_mesh.self_id.pub_key, PUB_KEY_SIZE); Serial.println();

  command[0] = 0;

  sensors.begin();

  the_mesh.begin(fs);

  cliext::begin(fs);   // load /cliext_cfg + seed durable runtime toggles (log on|off)

#ifdef DISPLAY_CLASS
  ui_task.begin(the_mesh.getNodePrefs(), FIRMWARE_BUILD_DATE, FIRMWARE_VERSION);
#endif

  // send out initial zero hop Advertisement to the mesh
#if ENABLE_ADVERT_ON_BOOT == 1
  the_mesh.sendSelfAdvertisement(16000, false);
#endif

#ifdef WITH_BACKHAUL_PERIPHERAL
  BleConsole.begin();   // NUS console + dormant DFU service
  // Bind RTC + identity. IDENTITY is framed to the central on connect (auto-populates this
  // relay observer); the RTC stamps observations at observe time and receives backhaul time
  // pushes (FRAME_TIME) so this non-network node can keep real UTC.
  observer::begin(the_mesh.getRTCClock(), the_mesh.self_id.pub_key, the_mesh.getNodeName(),
                  "RAK3401", FIRMWARE_VERSION, "SX1262");
  observer::setConsoleHandler(mastConsoleExec);   // remote-admin over the backhaul (FRAME_CONSOLE)
#endif

  board.onBootComplete();
}

void loop() {
#ifdef WITH_BACKHAUL_PERIPHERAL
  BleConsole.loop();
  observer::loop();   // send IDENTITY/STATUS on the backhaul link edge + periodic status
#endif

  int len = strlen(command);
  while (CONSOLE.available() && len < sizeof(command)-1) {
    char c = CONSOLE.read();
#ifdef WITH_BACKHAUL_PERIPHERAL
    // Demux central→mast structured frames (e.g. FRAME_TIME clock pushes) out of the
    // inbound NUS stream before the byte reaches the CLI command buffer.
    if (observer::feedBackhaulByte((uint8_t)c)) continue;
#endif
    if (c != '\n') {
      command[len++] = c;
      command[len] = 0;
      Serial.print(c);   // echo to USB only
    }
    if (c == '\r') break;
  }
  if (len == sizeof(command)-1) {  // command buffer full
    command[sizeof(command)-1] = '\r';
  }

  if (len > 0 && command[len - 1] == '\r') {  // received complete line
    Serial.print('\n');
    command[len - 1] = 0;  // replace newline with C string null terminator
    char reply[160];
    the_mesh.handleCommand(0, command, reply);  // NOTE: there is no sender_timestamp via serial!
    if (reply[0]) {
      CONSOLE.print("  -> "); CONSOLE.println(reply);
    }

    command[0] = 0;  // reset command buffer
  }

#if defined(PIN_USER_BTN) && defined(_SEEED_SENSECAP_SOLAR_H_)
  // Hold the user button to power off the SenseCAP Solar repeater.
  int btnState = digitalRead(PIN_USER_BTN);
  if (btnState == LOW) {
    if (userBtnDownAt == 0) {
      userBtnDownAt = millis();
    } else if ((unsigned long)(millis() - userBtnDownAt) >= USER_BTN_HOLD_OFF_MILLIS) {
      Serial.println("Powering off...");
      board.powerOff();  // does not return
    }
  } else {
    userBtnDownAt = 0;
  }
#endif

  cliext::loop();   // services deferred extension actions (tcpota reboot)
  the_mesh.loop();
  sensors.loop();
#ifdef DISPLAY_CLASS
  ui_task.loop();
#endif
  rtc_clock.tick();

  if (the_mesh.getNodePrefs()->powersaving_enabled && !the_mesh.hasPendingWork()) {
#if defined(NRF52_PLATFORM)
    board.sleep(0); // nrf ignores seconds param, sleeps whenever possible
#else
    if (the_mesh.millisHasNowPassed(POWERSAVING_FIRSTSLEEP_SECS * 1000)) { // To check if it is time to sleep
      board.sleep(30); // Sleep. Wake up after a while or when receiving a LoRa packet
    }
#endif
  }
}
