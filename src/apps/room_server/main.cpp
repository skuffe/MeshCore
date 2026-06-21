// Fork-vendored copy of examples/simple_room_server/main.cpp.
//
// Shared by both RAK_4631_room_server_eth_gw (ethernet only) and
// RAK_4631_room_server_eth_gw_blerelay (ethernet + relay) — additions are
// #ifdef'd on WITH_RAK13800_ETHERNET / WITH_BACKHAUL_CENTRAL. Kept here instead
// of editing the upstream example so `make rebase` never conflicts on it.
// The WHOLE room-server role app is vendored here (main + MyMesh + UITask) — the
// env compiles src/apps/room_server/ and excludes examples/simple_room_server/,
// which stays 100% pristine. If upstream changes the example, re-sync these
// copies — `make vendor-diff` (see docs/remote-observer-backhaul.md §5.1).
//
// Delta vs. upstream: WITH_RAK13800_ETHERNET -> EthConsole; WITH_BACKHAUL_CENTRAL
// -> BleRelay begin/loop (see docs/remote-observer-backhaul.md §4.1).

#include <Arduino.h>   // needed for PlatformIO
#include <Mesh.h>

#include "MyMesh.h"
#include <helpers/console/CLIExtensions.h>   // cliext::begin() — durable config store

#ifdef WITH_RAK13800_ETHERNET
  #include <helpers/bridges/EthernetTcpConsole.h>
  #define CONSOLE EthConsole   // CLI mirrored on TCP port 5000
#else
  #define CONSOLE Serial
#endif

#ifdef WITH_BACKHAUL_CENTRAL
  #include <helpers/bridges/BleNusRelay.h>   // relays the mast's BLE NUS to TCP :5001
#endif

#ifdef WITH_NET_BRIDGE
  #include <helpers/bridges/MqttPublisher.h>   // native on-node MQTT publish (Phase B1)
  #include <helpers/bridges/NtpClient.h>       // keep the mesh RTC in UTC for JSON timestamps
#endif

#ifdef DISPLAY_CLASS
  #include "UITask.h"
  static UITask ui_task(display);
#endif

StdRNG fast_rng;
SimpleMeshTables tables;
MyMesh the_mesh(board, radio_driver, *new ArduinoMillis(), fast_rng, rtc_clock, tables);

void halt() {
  while (1) ;
}

static char command[MAX_POST_TEXT_LEN+1];

void setup() {
  Serial.begin(115200);
  delay(1000);

  board.begin();

#ifdef DISPLAY_CLASS
  if (display.begin()) {
    display.startFrame();
    display.setCursor(0, 0);
    display.print("Please wait...");
    display.endFrame();
  }
#endif

  if (!radio_init()) { halt(); }

  fast_rng.begin(radio_driver.getRngSeed());

  FILESYSTEM* fs;
#if defined(NRF52_PLATFORM)
  InternalFS.begin();
  fs = &InternalFS;
  IdentityStore store(InternalFS, "");
#elif defined(RP2040_PLATFORM)
  LittleFS.begin();
  fs = &LittleFS;
  IdentityStore store(LittleFS, "/identity");
  store.begin();
#elif defined(ESP32)
  SPIFFS.begin(true);
  fs = &SPIFFS;
  IdentityStore store(SPIFFS, "/identity");
#else
  #error "need to define filesystem"
#endif
  if (!store.load("_main", the_mesh.self_id)) {
    the_mesh.self_id = radio_new_identity();   // create new random identity
    int count = 0;
    while (count < 10 && (the_mesh.self_id.pub_key[0] == 0x00 || the_mesh.self_id.pub_key[0] == 0xFF)) {  // reserved id hashes
      the_mesh.self_id = radio_new_identity(); count++;
    }
    store.save("_main", the_mesh.self_id);
  }

  Serial.print("Room ID: ");
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

#ifdef WITH_RAK13800_ETHERNET
  EthConsole.begin();
#endif

#ifdef WITH_BACKHAUL_CENTRAL
  BleRelay.begin();   // BLE central → mast NUS, bridged to TCP :5001
#endif

#ifdef WITH_NET_BRIDGE
  // Identity/clock context for the analyzer-spec JSON (origin_id = 64-hex pubkey).
  static char obs_pubkey[2 * PUB_KEY_SIZE + 1];
  for (int i = 0; i < PUB_KEY_SIZE; i++) sprintf(obs_pubkey + i * 2, "%02X", the_mesh.self_id.pub_key[i]);
  MqttPub.setContext(the_mesh.getRTCClock(), the_mesh.getNodeName(), obs_pubkey,
                     "RAK4631", FIRMWARE_VERSION, "SX1262",
                     "wismesh-observer/" FIRMWARE_VERSION);
  MqttPub.begin();   // inert unless mqtt.enabled + provisioned (see `set mqtt.*`)
  Ntp.begin(the_mesh.getRTCClock());   // sync UTC once ethernet is up, then hourly
#endif

  board.onBootComplete();
}

void loop() {
#ifdef WITH_RAK13800_ETHERNET
  EthConsole.loop();
#endif

#ifdef WITH_BACKHAUL_CENTRAL
  BleRelay.loop();
#endif

#ifdef WITH_NET_BRIDGE
  MqttPub.loop();   // MQTT keepalive + throttled reconnect
  Ntp.loop();       // periodic UTC time sync
#endif

  int len = strlen(command);
  while (CONSOLE.available() && len < sizeof(command)-1) {
    char c = CONSOLE.read();
    if (c == '\n') {
      // treat bare LF as Enter (nc/unix line endings); ignore the LF of a
      // CRLF pair and empty lines
      if (len > 0 && command[len - 1] != '\r') {
        command[len++] = '\r';
        command[len] = 0;
      }
    } else {
      command[len++] = c;
      command[len] = 0;
    }
    Serial.print(c);   // echo to USB serial only — TCP clients see their own typing locally
  }
  if (len == sizeof(command)-1) {  // command buffer full
    command[sizeof(command)-1] = '\r';
  }

  if (len > 0 && command[len - 1] == '\r') {  // received complete line
    command[len - 1] = 0;  // replace newline with C string null terminator
    char reply[160];
    the_mesh.handleCommand(0, command, reply);  // NOTE: there is no sender_timestamp via serial!
    if (reply[0]) {
      CONSOLE.print("  -> "); CONSOLE.println(reply);
    }

    command[0] = 0;  // reset command buffer
  }

  the_mesh.loop();
  sensors.loop();
#ifdef DISPLAY_CLASS
  ui_task.loop();
#endif
  rtc_clock.tick();
}
