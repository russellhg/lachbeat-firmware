// lb_bus.h - LatchBeat module bus constants and helpers (shared by leader and follower)
//
// THESE ARE AI SUGGESTIONS. AI-assisted DRAFT written by Claude, Oct 29, 2026.
// NOT COMPILED, NOT TESTED. Keep this file IDENTICAL in leader_esp32/ and
// follower_promini/ (Arduino sketches can't share a header from a parent folder).
//
// Addressing (see Houston's "AI ASSISTED - DRAFT - Pinout v0.2 suggested fixes.md"):
//   - 0x30        boot address: every unassigned module listens here
//   - 0x10..0x27  session addresses handed out by the leader
//   - Avoided: I2C reserved 0x00-0x07 and 0x78-0x7F (UM10204 Table 4),
//              SMBus 0x08-0x0C, 0x28, 0x2C, 0x2D, 0x37, 0x61 (SMBus 3.3.1 App. C)

#pragma once
#include <stdint.h>

namespace lb {

// ---- addresses --------------------------------------------------------------
const uint8_t BOOT_ADDR     = 0x30;
const uint8_t SESSION_FIRST = 0x10;
const uint8_t SESSION_LAST  = 0x27;   // 24 addresses
const uint8_t MAX_MODULES   = SESSION_LAST - SESSION_FIRST + 1;

// ---- boot frame (module -> leader, read from BOOT_ADDR) ---------------------
// [0] 'L' magic  [1] module type  [2] fw version  [3..10] UID (8 bytes)  [11] CRC-8 of bytes 0..10
const uint8_t UID_LEN        = 8;
const uint8_t BOOT_FRAME_LEN = 12;
const uint8_t MAGIC          = 'L';

// ---- commands written to BOOT_ADDR (every unassigned module receives them) --
const uint8_t CMD_ASSIGN  = 0xA1;  // payload: UID[8], new_addr, CRC-8 of (CMD..new_addr)
const uint8_t CMD_BACKOFF = 0xB0;  // no payload: go quiet, release ALERT#, retry after a random delay

// ---- registers at a session address -----------------------------------------
// Leader writes 1 byte (register pointer), then reads.
const uint8_t REG_IDENT     = 0x00;  // read 12 bytes: same layout as the boot frame
const uint8_t REG_EVENTS    = 0x10;  // read 4 bytes: [seq] [buttons bitmask] [pot hi] [pot lo]
const uint8_t REG_UNASSIGN  = 0x7E;  // write: drop back to unassigned (leader-initiated reset)
const uint8_t EVENTS_LEN    = 4;

// ---- module types (placeholder list) -----------------------------------------
const uint8_t TYPE_DEMO_BUTTON = 0x01;
const uint8_t TYPE_DEMO_POT    = 0x02;
const uint8_t TYPE_ENCODER     = 0x03;
const uint8_t TYPE_FADER       = 0x04;

// ---- CRC-8, polynomial 0x07 (CRC-8/SMBUS style), init 0x00 ------------------
inline uint8_t crc8(const uint8_t *data, uint8_t len) {
  uint8_t crc = 0x00;
  for (uint8_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t b = 0; b < 8; b++) {
      crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
    }
  }
  return crc;
}

inline bool uidEqual(const uint8_t *a, const uint8_t *b) {
  for (uint8_t i = 0; i < UID_LEN; i++) {
    if (a[i] != b[i]) return false;
  }
  return true;
}

}  // namespace lb