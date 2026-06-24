#include "DfuRelay.h"
#ifdef WITH_BACKHAUL_CENTRAL
#include <string.h>
#include <stdio.h>
#include "BackhaulFrame.h"
#include "BleDfuClient.h"   // M2: the legacy BLE-DFU client this relay feeds

// M1: host↔central FRAME_DFU transport with windowed flow control; the BLE-DFU client is a
// stub (the `(M2: …)` comments mark where the real edge-side actions go). The state machine:
//   DFU_BEGIN  → reset counters, reply DFU_READY{window}
//   DFU_DATA   → if in-order, count bytes; reply DFU_ACK{highest contiguous seq}; periodic PROGRESS
//   DFU_COMMIT → received==image_len ? DFU_DONE : DFU_ERROR
//   DFU_ABORT  → drop the session
namespace dfurelay {
namespace {

Stream*          s_con = nullptr;
void           (*s_hold)(bool) = nullptr;   // hold the :5000 socket for the flash duration
backhaul::Parser s_parser;          // demuxes 0x1E frames out of the :5000 input
bool             s_active = false;
uint32_t         s_image_len = 0;   // total firmware bytes the host promised in BEGIN
uint32_t         s_received = 0;    // firmware bytes accepted so far
uint16_t         s_next_seq = 0;    // next in-order DFU_DATA seq we expect
uint16_t         s_since_progress = 0;
bool             s_sent_ready = false;  // DFU_READY emitted once the edge reaches RECEIVING
char             s_target[25];      // edge node addressed by the active session

const uint16_t WINDOW = 16;         // max DFU_DATA the host may keep in flight before an ACK

// Emit one central→host FRAME_DFU frame (op + small body). Bodies here are ≤4 B.
void sendOp(uint8_t op, const uint8_t* body, uint16_t blen) {
  uint8_t payload[8];
  if (blen + 1u > sizeof payload) return;
  payload[0] = op;
  memcpy(payload + 1, body, blen);
  uint8_t frame[5 + 8];
  size_t n = backhaul::encode(frame, sizeof frame, backhaul::FRAME_DFU, payload, (uint16_t)(1 + blen));
  if (n && s_con) s_con->write(frame, n);
}
void sendOp0(uint8_t op) { sendOp(op, nullptr, 0); }

void setHeld(bool h) { if (s_hold) s_hold(h); }

// Emit an ASCII frame (DFU_STATUS narration or DFU_ERROR reason) — op + text, no NUL.
void sendText(uint8_t op, const char* msg) {
  uint8_t payload[80];
  payload[0] = op;
  uint16_t mlen = 0;
  while (msg[mlen] && mlen + 1u < sizeof payload) { payload[1 + mlen] = (uint8_t)msg[mlen]; mlen++; }
  uint8_t frame[5 + 80];
  size_t n = backhaul::encode(frame, sizeof frame, backhaul::FRAME_DFU, payload, (uint16_t)(1 + mlen));
  if (n && s_con) s_con->write(frame, n);
}
void sendStatus(const char* msg) { sendText(backhaul::DFU_STATUS, msg); }

void sendErr(const char* msg) {
  sendText(backhaul::DFU_ERROR, msg);
  s_active = false;
  setHeld(false);
}

void handle(const uint8_t* p, uint16_t len) {
  if (len < 1) return;
  switch (p[0]) {

    case backhaul::DFU_BEGIN: {
      if (len < 1 + sizeof(backhaul::DfuBeginBody)) { sendErr("BEGIN too short"); return; }
      backhaul::DfuBeginBody b;
      memcpy(&b, p + 1, sizeof b);
      const uint8_t* init = p + 1 + sizeof b;             // .dat init packet (sent to the edge)
      uint16_t init_len = b.init_len;
      if (1 + sizeof b + init_len > len) { sendErr("BEGIN init_len overruns frame"); return; }
      s_image_len = b.image_len;
      s_received = 0;
      s_next_seq = 0;
      s_since_progress = 0;
      s_sent_ready = false;
      memcpy(s_target, b.target, 24);
      s_target[24] = 0;
      s_active = true;
      setHeld(true);   // own :5000 for the flash — no displacing client aborts it
      {
        char st[64];
        snprintf(st, sizeof st, "B-OTA: %.20s, %lu B — entering edge DFU",
                 s_target, (unsigned long)b.image_len);
        sendStatus(st);
      }
      // Async: hand the radio to the BLE-DFU client (buttonless → bootloader → legacy DFU). It
      // gets the init(.dat) + image size now; DFU_READY is sent (in loop) once the edge is primed
      // and reaches RECEIVING; the firmware then streams in via DFU_DATA → feedChunk.
      bledfu::start(s_target, init, init_len, b.image_len);
      break;
    }

    case backhaul::DFU_DATA: {
      if (!s_active) return;
      if (len < 1 + sizeof(backhaul::DfuDataBody)) return;
      uint16_t seq;
      memcpy(&seq, p + 1, 2);
      uint16_t chunk = len - 1 - (uint16_t)sizeof(backhaul::DfuDataBody);
      if (seq == s_next_seq) {
        // Write the chunk to the edge's DFU packet characteristic; pktWrite paces on the
        // SoftDevice TX buffers (real backpressure), so the ACK below means it's on the wire.
        const uint8_t* d = p + 1 + (uint16_t)sizeof(backhaul::DfuDataBody);
        if (!bledfu::feedChunk(d, chunk)) return;   // write failed → bledfu FAILED; loop() reports it
        s_received += chunk;
        s_next_seq++;
      }
      // ACK the highest contiguous seq accepted (host resends from here on a gap/dup).
      uint16_t ack = (uint16_t)(s_next_seq - 1);
      uint8_t ab[2] = { (uint8_t)(ack & 0xFF), (uint8_t)(ack >> 8) };
      sendOp(backhaul::DFU_ACK, ab, 2);
      if (++s_since_progress >= 8) {   // narrate remote byte progress to the host every ~2 KB
        s_since_progress = 0;
        uint8_t pb[4];
        memcpy(pb, &s_received, 4);
        sendOp(backhaul::DFU_PROGRESS, pb, 4);
      }
      break;
    }

    case backhaul::DFU_COMMIT:
      if (!s_active) return;
      if (s_received != s_image_len) { sendErr("size mismatch"); break; }
      // All firmware is on the wire to the edge. Finish the legacy DFU (final RECEIVE response →
      // VALIDATE → ACTIVATE); loop() emits DFU_DONE when bledfu reaches DONE. Stays active.
      bledfu::commit();
      break;

    case backhaul::DFU_ABORT:
      bledfu::abort("host aborted");   // tear down BLE + hand the radio back; edge keeps old app
      s_active = false;
      setHeld(false);
      break;
  }
}

}  // namespace

void begin(Stream* console) { s_con = console; }
void setHoldHandler(void (*fn)(bool)) { s_hold = fn; }

void loop() {
  if (!s_active) return;
  bledfu::loop();
  if (bledfu::statusChanged()) sendStatus(bledfu::status());   // narrate edge state to the host
  switch (bledfu::state()) {
    case bledfu::RECEIVING:
      // Edge bootloader is primed (START + INIT done) — let the host start streaming the image.
      if (!s_sent_ready) {
        s_sent_ready = true;
        uint8_t w[2] = { (uint8_t)(WINDOW & 0xFF), (uint8_t)(WINDOW >> 8) };
        sendOp(backhaul::DFU_READY, w, 2);
      }
      break;
    case bledfu::DONE:
      sendOp0(backhaul::DFU_DONE);   // edge validated + rebooting into the new firmware
      s_active = false;
      setHeld(false);
      break;
    case bledfu::FAILED:
      sendErr(bledfu::status());     // bledfu already handed the radio back; sendErr clears s_active/hold
      break;
    default:
      break;
  }
}

bool feedByte(uint8_t b) {
  backhaul::Parser::Result r = s_parser.feed(b);
  if (r == backhaul::Parser::PASS) return false;               // ordinary CLI text
  if (r == backhaul::Parser::FRAME && s_parser.type() == backhaul::FRAME_DFU) {
    handle(s_parser.payload(), s_parser.len());
  }
  return true;                                                 // EAT/FRAME → not CLI text
}

}  // namespace dfurelay
#endif  // WITH_BACKHAUL_CENTRAL
