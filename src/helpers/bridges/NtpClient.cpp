#include "NtpClient.h"

#ifdef WITH_NET_BRIDGE

#include <string.h>
#include <Dns.h>
#include <helpers/console/CliextConfig.h>
#include <helpers/bridges/EthernetTcpConsole.h>   // EthConsole.isReady() — shared W5100S lease

static const uint16_t NTP_PORT        = 123;
static const uint16_t NTP_LOCAL_PORT  = 8888;
static const uint32_t NTP_RESYNC_MS   = 3600000UL;  // refresh hourly once synced
static const uint32_t NTP_RETRY_MS    = 60000UL;    // retry a minute after a failure
static const uint32_t NTP_RESP_TO_MS  = 2000UL;     // await reply this long
// Seconds between 1900-01-01 (NTP epoch) and 1970-01-01 (UNIX epoch).
static const uint32_t NTP_UNIX_DELTA  = 2208988800UL;

NtpClient Ntp;

void NtpClient::begin(mesh::RTCClock* rtc) {
  _rtc = rtc;
  _next_sync = 0;   // first sync as soon as ethernet is ready
}

static const char* ntpServer() {
  const char* s = cliext::config().ntp_server;
  return s[0] ? s : NTP_DEFAULT_SERVER;
}

bool NtpClient::resolveServer() {
  if (_have_ip) return true;
  // 4.5 s timeout: a first DNS query (cache cold) regularly needs >2 s, and a too-short
  // timeout here silently never resolves → never syncs. Blocks the loop once, then the
  // result is cached for the session.
  DNSClient dns;
  dns.begin(Ethernet.dnsServerIP());
  if (dns.getHostByName(ntpServer(), _server_ip, 4500) == 1) { _have_ip = true; return true; }
  return false;
}

void NtpClient::sendRequest(uint32_t now) {
  if (!resolveServer()) { _stage = "dns-fail"; _next_sync = now + NTP_RETRY_MS; return; }

  // Open the UDP socket only for this exchange. The W5100S has just 4 sockets; holding
  // one persistently here starves the console/relay/MQTT — so we stop() it again the
  // moment the reply lands or times out (see loop()). begin() returns 0 when no socket
  // is free; back off and retry rather than spin.
  if (_udp.begin(NTP_LOCAL_PORT) == 0) { _stage = "no-socket"; _next_sync = now + NTP_RETRY_MS; return; }

  uint8_t pkt[48];
  memset(pkt, 0, sizeof(pkt));
  pkt[0] = 0xE3;   // LI=3 (unsync), VN=4, Mode=3 (client)
  pkt[1] = 0;      // stratum
  pkt[2] = 6;      // poll interval
  pkt[3] = 0xEC;   // precision

  if (_udp.beginPacket(_server_ip, NTP_PORT) && _udp.write(pkt, sizeof(pkt)) == sizeof(pkt)
      && _udp.endPacket()) {
    _state = WAITING;
    _stage = "sent";
    _sent_at = now;
  } else {
    _udp.stop();                      // release the socket
    _stage = "send-fail";
    _have_ip = false;                 // force a re-resolve next time
    _next_sync = now + NTP_RETRY_MS;
  }
}

void NtpClient::loop() {
  if (!_rtc) return;
  if (!cliext::config().ntp_enabled) return;
  if (!EthConsole.isReady()) return;          // shares the console's W5100S lease

  uint32_t now = millis();

  if (_state == IDLE) {
    if (_next_sync == 0 || (long)(now - _next_sync) >= 0) sendRequest(now);
    return;
  }

  // WAITING for the reply.
  int sz = _udp.parsePacket();
  if (sz >= 48) {
    uint8_t buf[48];
    _udp.read(buf, sizeof(buf));
    _udp.stop();                              // release the socket immediately
    // Transmit timestamp seconds: big-endian 32-bit at offset 40.
    uint32_t ntp_secs = ((uint32_t)buf[40] << 24) | ((uint32_t)buf[41] << 16)
                      | ((uint32_t)buf[42] << 8)  |  (uint32_t)buf[43];
    if (ntp_secs > NTP_UNIX_DELTA) {
      uint32_t epoch = ntp_secs - NTP_UNIX_DELTA;
      _rtc->setCurrentTime(epoch);
      _synced = true;
      _stage = "synced";
      Serial.printf("NTP: synced epoch=%lu (%s)\n", (unsigned long)epoch, ntpServer());
    }
    _state = IDLE;
    _next_sync = now + NTP_RESYNC_MS;
  } else if ((long)(now - _sent_at) > (long)NTP_RESP_TO_MS) {
    _udp.stop();                              // timed out — release the socket, retry soon
    _stage = "timeout";
    _state = IDLE;
    _next_sync = now + NTP_RETRY_MS;
  }
}

#endif  // WITH_NET_BRIDGE
