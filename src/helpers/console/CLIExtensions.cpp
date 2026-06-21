#include "CLIExtensions.h"
#include <string.h>
#include <stdio.h>

#ifdef WITH_RAK13800_ETHERNET
  #include <helpers/bridges/EthernetTcpConsole.h>
  #include <RAK13800_W5100S.h>
#endif
#ifdef WITH_BLE_CENTRAL_RELAY
  #include <helpers/bridges/BleNusRelay.h>
#endif
#ifdef WITH_BLE_CONSOLE
  #include <helpers/nrf52/BleConsole.h>
#endif

namespace cliext {

#ifdef PACKET_LOG_STREAM
  #ifdef WITH_PACKET_OBSERVER
    bool g_packet_dump_enabled = true;
  #else
    bool g_packet_dump_enabled = false;
  #endif
#endif

#ifdef WITH_RAK13800_ETHERNET
// Deferred reboot into the bootloader's TCP DFU mode (0 = inactive). Lets the
// command reply drain to the client before the chip resets.
static uint32_t _tcpota_reboot_at = 0;
#endif

bool handleCommand(const char* command, char* reply) {
  (void)command; (void)reply;

#ifdef WITH_RAK13800_ETHERNET
  if (strcmp(command, "start tcpota") == 0) {
    char ip_str[16];
    if (EthConsole.prepareTcpDfuHandoff(4444, ip_str)) {
      sprintf(reply, "TCP DFU: rebooting, send image to %s:4444", ip_str);
      _tcpota_reboot_at = millis() + 700;   // let the reply drain first
    } else {
      strcpy(reply, "Err - ethernet not up");
    }
    return true;
  }
  if (strcmp(command, "eth") == 0) {
    if (!EthConsole.isReady()) {
      strcpy(reply, "eth: down (acquiring)");
    } else {
      IPAddress ip = Ethernet.localIP();
      sprintf(reply, "eth: up ip=%d.%d.%d.%d link=%s",
              ip[0], ip[1], ip[2], ip[3],
              Ethernet.linkStatus() == LinkON ? "on" : "off");
    }
    return true;
  }
#endif

#if defined(WITH_BLE_CENTRAL_RELAY) || defined(WITH_BLE_CONSOLE)
  if (strcmp(command, "backhaul") == 0) {
  #if defined(WITH_BLE_CENTRAL_RELAY)
    // Central side (Sortsnak): the relay's link up to the mast NUS peripheral.
    if (BleRelay.linkUp()) {
      sprintf(reply, "backhaul: up rssi=%ddBm", (int)BleRelay.rssi());
    } else {
      strcpy(reply, "backhaul: down (scanning)");
    }
  #elif defined(WITH_BLE_CONSOLE)
    // Peripheral side (mast): whether a central holds the NUS console slot.
    if (BleConsole.connected()) {
      sprintf(reply, "backhaul: central connected rssi=%ddBm", (int)BleConsole.rssi());
    } else {
      strcpy(reply, "backhaul: no central (advertising)");
    }
  #endif
    return true;
  }
#endif

#ifdef PACKET_LOG_STREAM
  if (strcmp(command, "log on") == 0) {
    g_packet_dump_enabled = true;
    strcpy(reply, "log: on");
    return true;
  }
  if (strcmp(command, "log off") == 0) {
    g_packet_dump_enabled = false;
    strcpy(reply, "log: off");
    return true;
  }
  if (strcmp(command, "log") == 0 || strcmp(command, "log status") == 0) {
    sprintf(reply, "log: %s", g_packet_dump_enabled ? "on" : "off");
    return true;
  }
#endif

  return false;   // not an extension command — caller falls through to CommonCLI
}

void loop() {
#ifdef WITH_RAK13800_ETHERNET
  if (_tcpota_reboot_at && (long)(millis() - _tcpota_reboot_at) >= 0) {
    NRF_POWER->GPREGRET = DFU_MAGIC_TCP_RESET;
    NVIC_SystemReset();
  }
#endif
}

}  // namespace cliext
