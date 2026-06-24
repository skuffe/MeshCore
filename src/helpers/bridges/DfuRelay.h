#pragma once
#include <Arduino.h>

// DfuRelay — the central side of B-OTA (relay-flash an edge node). The host (mcflash -mode
// backhaul) streams a firmware image as FRAME_DFU frames on the :5000 console; this demuxes
// those frames off the console byte stream (the same 0x1E-parser pattern the NUS backhaul
// uses) and drives a legacy Nordic BLE-DFU client to the edge bootloader.
//
// M1 (current): transport + flow control only — the BLE-DFU client is a STUB that validates
// framing/size and ACKs, so the host↔central path is HW-verifiable without any BLE. M2 wires
// the real BleDfuClient into the marked hooks (resolve target, scan/connect, push to the edge).
//
// Gated WITH_BACKHAUL_CENTRAL (only the relay gateway runs it).
namespace dfurelay {

// Bind the console the DFU frames arrive on / replies go out on (EthConsole :5000).
void begin(Stream* console);

// Optional: a hook toggled true for the duration of a DFU session so the transport can hold its
// socket against displacement (EthConsole.hold) — a stray client must not abort a flash.
void setHoldHandler(void (*fn)(bool));

// Feed one inbound console byte. Returns true if the byte was consumed as part of a FRAME_DFU
// frame (so the caller must NOT treat it as CLI text), false if it belongs to the text console.
bool feedByte(uint8_t b);

// Pump the async relay-flash state machine (BLE-DFU client progress, status narration). Call
// every central loop(); cheap no-op when no flash is in progress.
void loop();

}  // namespace dfurelay
