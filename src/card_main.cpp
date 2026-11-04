// card_main.cpp - Card & leader firmware
//
// Targeting ESP-32S (breadboard prototype) or ESP32-S3 (V1 PCB)
// Includes:
//   Wireless BLE-MIDI
//   Live GPIO Pin-Finder ('d') & I2C Bus Scanner ('i') for instant breadboard verification
//   Non-blocking 1 kHz (1 ms) scan loop & 8-bit shift-register debouncing 
//   Active-Note Tracking Table: flushes exact Note-Offs on scale change or unplug
//   Non-blocking MPU-6050 (0x68) tilt-to-MIDI CC#1 with EMA filter
//   Full LatchBeat I2C hot-plug discovery (0x30) & follower polling
#include <Arduino.h>
#include <Wire.h>
#include <NimBLEDevice.h>
#include "pin_map.h"
#include "lb_bus.h"

//BLE-MIDI Standard UUIDs (MMA Specification v1.0)
#define BLE_MIDI_SERVICE_UUID "03B80E5A-EDE8-4B33-A751-6CE34EC4C700"
#define BLE_MIDI_CHAR_UUID    "7772E5DB-3868-4112-A1A9-F2669D106BF3"

NimBLEServer*         pBleServer   = nullptr;
NimBLECharacteristic* pMidiChar    = nullptr;
bool                  bleConnected = false;

class BleMidiServerCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer* /*pServer*/) override { bleConnected = true; }
  void onDisconnect(NimBLEServer* pServer) override {
    bleConnected = false;
    pServer->getAdvertising()->start();
  }
};

// Runtime toggleable Serial mode: false = human-readable monitor text, true = raw 3-byte MIDI
bool serialBinaryMidiMode = false;
bool pinDiagnosticMode    = false;

//Scales/Note Tracking
const uint8_t NUM_SCALES = 4;
const char* SCALE_NAMES[NUM_SCALES] = {
  "C Major", "A Natural Minor", "C Major Pentatonic", "Chromatic"
};
const uint8_t SCALE_INTERVALS[NUM_SCALES][pins::NUM_CARD_KEYS] = {
  { 0, 2, 4, 5, 7, 9, 11, 12 },   // Major
  { 0, 2, 3, 5, 7, 8, 10, 12 },   // Natural Minor
  { 0, 2, 4, 7, 9, 12, 14, 16 },  // Major Pentatonic
  { 0, 1, 2, 3, 4, 5, 6, 7 }      // Chromatic
};

uint8_t currentScale  = 0;
uint8_t baseMidiNote  = 60;       // Middle C (C4 = 60)
const uint8_t MIDI_CH = 0;        // Channel 1 (0-indexed)

uint8_t keyHistory[pins::NUM_CARD_KEYS];
bool    keyPressed[pins::NUM_CARD_KEYS];
uint8_t activeKeyNote[pins::NUM_CARD_KEYS];  // Exact MIDI note currently held (0xFF = none)
uint8_t modeBtnHistory = 0xFF;
bool    modeBtnPressed = false;

// Safe ESP32 GPIOs to monitor during live 'd' Pin-Finder diagnostic mode
const uint8_t DIAG_PIN_COUNT = 16;
const int DIAG_PINS[DIAG_PIN_COUNT] = {
  0, 2, 4, 5, 12, 13, 14, 15, 16, 17, 18, 19, 23, 25, 26, 27
};
uint8_t diagPinLast[DIAG_PIN_COUNT];

//MPU-6050 (0x68) state
const uint8_t MPU6050_ADDR = 0x68;
bool    mpuPresent         = false;
int32_t emaAccelX          = 0;
int32_t emaAccelY          = 0;
uint8_t lastModWheelCC     = 64;

//Follower module table
struct Module {
  bool    known;
  bool    present;
  uint8_t uid[lb::UID_LEN];
  uint8_t type;
  uint8_t lastSeq;
  uint8_t lastButtons;
  uint8_t lastPotCC;
  uint8_t misses;
  uint8_t heldNote;   // 0xFF if no note held
};
Module modules[lb::MAX_MODULES];

const uint32_t I2C_HZ       = 100000;
const uint32_t DISCOVER_MS  = 250;
const uint32_t HEARTBEAT_MS = 50;
const uint32_t IMU_POLL_MS  = 20;     // 50 Hz IMU update
const uint8_t  MISS_LIMIT   = 3;
const uint8_t  ERR_LIMIT    = 5;

uint32_t lastScanUs    = 0;
uint32_t lastDiscover  = 0;
uint32_t lastHeartbeat = 0;
uint32_t lastImuPoll   = 0;
uint8_t  busErrors     = 0;
uint32_t collisions    = 0;

inline uint8_t addrOf(uint8_t idx) { return lb::SESSION_FIRST + idx; }

//BLE-MIDI and serial output functions
void initBleMidi() {
  NimBLEDevice::init("LatchBeat MIDI");
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);
  pBleServer = NimBLEDevice::createServer();
  pBleServer->setCallbacks(new BleMidiServerCallbacks());

  NimBLEService* pService = pBleServer->createService(BLE_MIDI_SERVICE_UUID);
  pMidiChar = pService->createCharacteristic(
    BLE_MIDI_CHAR_UUID,
    NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE |
    NIMBLE_PROPERTY::NOTIFY | NIMBLE_PROPERTY::WRITE_NR
  );
  pService->start();

  NimBLEAdvertising* pAdv = NimBLEDevice::getAdvertising();
  pAdv->addServiceUUID(BLE_MIDI_SERVICE_UUID);
  pAdv->setScanResponse(true);
  pAdv->start();
}

void sendBleMidiPacket(uint8_t status, uint8_t data1, uint8_t data2) {
  if (!bleConnected || pMidiChar == nullptr) return;
  uint16_t t = (uint16_t)(millis() & 0x1FFF);
  uint8_t blePkt[5] = {
    (uint8_t)(0x80 | ((t >> 7) & 0x3F)),  // BLE-MIDI Header byte
    (uint8_t)(0x80 | (t & 0x7F)),         // BLE-MIDI Timestamp byte
    status,
    (uint8_t)(data1 & 0x7F),
    (uint8_t)(data2 & 0x7F)
  };
  pMidiChar->setValue(blePkt, 5);
  pMidiChar->notify();
}

void sendMidiNoteOn(uint8_t ch, uint8_t note, uint8_t vel) {
  uint8_t status = (uint8_t)(0x90 | (ch & 0x0F));
  sendBleMidiPacket(status, note, vel);
  if (serialBinaryMidiMode) {
    uint8_t pkt[3] = { status, (uint8_t)(note & 0x7F), (uint8_t)(vel & 0x7F) };
    Serial.write(pkt, 3);
  } else {
    Serial.printf("[MIDI] NoteOn  ch=%u note=%u vel=%u (BLE=%s)\n",
                  ch + 1, note, vel, bleConnected ? "yes" : "no");
  }
}

void sendMidiNoteOff(uint8_t ch, uint8_t note) {
  uint8_t status = (uint8_t)(0x80 | (ch & 0x0F));
  sendBleMidiPacket(status, note, 0);
  if (serialBinaryMidiMode) {
    uint8_t pkt[3] = { status, (uint8_t)(note & 0x7F), 0x00 };
    Serial.write(pkt, 3);
  } else {
    Serial.printf("[MIDI] NoteOff ch=%u note=%u\n", ch + 1, note);
  }
}

void sendMidiCC(uint8_t ch, uint8_t cc, uint8_t val) {
  uint8_t status = (uint8_t)(0xB0 | (ch & 0x0F));
  sendBleMidiPacket(status, cc, val);
  if (serialBinaryMidiMode) {
    uint8_t pkt[3] = { status, (uint8_t)(cc & 0x7F), (uint8_t)(val & 0x7F) };
    Serial.write(pkt, 3);
  } else {
    Serial.printf("[MIDI] CC      ch=%u cc=%u val=%u\n", ch + 1, cc, val);
  }
}

//Flushes all active notes before changing scale/octave or on Panic
void flushAllCardNotes() {
  for (uint8_t i = 0; i < pins::NUM_CARD_KEYS; i++) {
    if (keyPressed[i] && activeKeyNote[i] != 0xFF) {
      sendMidiNoteOff(MIDI_CH, activeKeyNote[i]);
      activeKeyNote[i] = 0xFF;
      keyPressed[i] = false;
    }
  }
  sendMidiCC(MIDI_CH, 123, 0);  // Standard MIDI CC#123 All Notes Off
}

void cycleScaleMode() {
  flushAllCardNotes();
  currentScale = (currentScale + 1) % NUM_SCALES;
  if (!serialBinaryMidiMode) {
    Serial.printf("[MODE] Scale switched to: %s (All active notes flushed cleanly)\n",
                  SCALE_NAMES[currentScale]);
  }
}

// Non-Blocking 1 kHz Card Key Scanner (H-04)
void scanCardKeys1kHz() {
  if (pinDiagnosticMode) {
    for (uint8_t i = 0; i < DIAG_PIN_COUNT; i++) {
      uint8_t v = (uint8_t)digitalRead(DIAG_PINS[i]);
      if (v != diagPinLast[i]) {
        diagPinLast[i] = v;
        Serial.printf("[PIN FINDER] GPIO %d -> %s\n",
                      DIAG_PINS[i], (v == LOW) ? "PRESSED (LOW)" : "RELEASED (HIGH)");
      }
    }
    return;
  }

  for (uint8_t i = 0; i < pins::NUM_CARD_KEYS; i++) {
    uint8_t raw = (uint8_t)digitalRead(pins::CARD_KEY_PINS[i]);
    keyHistory[i] = (uint8_t)((keyHistory[i] << 1) | (raw & 0x01));

    //Falling edge settled for 4 consecutive ms (0b11110000 = 0xF0) -> Press
    if (keyHistory[i] == 0xF0 && !keyPressed[i]) {
      keyPressed[i] = true;
      uint8_t note = baseMidiNote + SCALE_INTERVALS[currentScale][i];
      activeKeyNote[i] = note;
      sendMidiNoteOn(MIDI_CH, note, 100);

      // Simultaneous Key 0 + Key 7 chord triggers Scale Mode cycle
      if (keyPressed[0] && keyPressed[pins::NUM_CARD_KEYS - 1]) {
        cycleScaleMode();
      }
    }
    //Rising edge settled for 4 consecutive ms (0b00001111 = 0x0F) -> Release
    else if (keyHistory[i] == 0x0F && keyPressed[i]) {
      keyPressed[i] = false;
      if (activeKeyNote[i] != 0xFF) {
        sendMidiNoteOff(MIDI_CH, activeKeyNote[i]);
        activeKeyNote[i] = 0xFF;
      }
    }
  }

  //Optional P19 Mode button
  uint8_t rawMode = (uint8_t)digitalRead(pins::PIN_MODE_BTN);
  modeBtnHistory = (uint8_t)((modeBtnHistory << 1) | (rawMode & 0x01));
  if (modeBtnHistory == 0xF0 && !modeBtnPressed) {
    modeBtnPressed = true;
    cycleScaleMode();
  } else if (modeBtnHistory == 0x0F && modeBtnPressed) {
    modeBtnPressed = false;
  }
}

//Non-blocking tilt reader
void initMpu6050() {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0x00);  // Wake up MPU-6050
  if (Wire.endTransmission(true) == 0) {
    mpuPresent = true;
    if (!serialBinaryMidiMode) Serial.println("[IMU] MPU-6050 detected at 0x68");
  } else {
    if (!serialBinaryMidiMode) Serial.println("[IMU] MPU-6050 not found at 0x68 (press 'i' to scan I2C)");
  }
}

void pollMpu6050() {
  if (!mpuPresent || pinDiagnosticMode) return;
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x3B);  // ACCEL_XOUT_H
  if (Wire.endTransmission(false) != 0) return;
  if (Wire.requestFrom(MPU6050_ADDR, (uint8_t)4) != 4) return;

  int16_t ax = (int16_t)((Wire.read() << 8) | Wire.read());
  int16_t ay = (int16_t)((Wire.read() << 8) | Wire.read());

  emaAccelX = (ax + 3 * emaAccelX) >> 2;
  emaAccelY = (ay + 3 * emaAccelY) >> 2;

  int32_t mapped = 64 + (emaAccelX / 256);
  if (mapped < 0) mapped = 0;
  if (mapped > 127) mapped = 127;
  uint8_t ccVal = (uint8_t)mapped;

  if (abs((int)ccVal - (int)lastModWheelCC) >= 2) {
    lastModWheelCC = ccVal;
    sendMidiCC(MIDI_CH, 1, ccVal);
  }
}

//I2C Bus Scanner
void runI2cScanner() {
  Serial.println("=== Scanning I2C Bus (SDA=P21, SCL=P22) ===");
  uint8_t found = 0;
  for (uint8_t addr = 0x08; addr <= 0x77; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission(true) == 0) {
      Serial.printf("  -> Found I2C device at 0x%02X", addr);
      if (addr == 0x68 || addr == 0x69) Serial.print(" (MPU-6050 IMU)");
      else if (addr == lb::BOOT_ADDR)   Serial.print(" (LatchBeat Unassigned Follower)");
      else if (addr >= lb::SESSION_FIRST && addr <= lb::SESSION_LAST) Serial.print(" (LatchBeat Assigned Follower)");
      Serial.println();
      found++;
    }
  }
  if (found == 0) Serial.println("  -> No I2C devices responded. Check VCC/GND/SDA/SCL wires.");
  Serial.println("===========================================");
}

//I2C Bus Helpers
void busBegin() {
  Wire.begin(pins::PIN_SDA, pins::PIN_SCL, I2C_HZ);
  Wire.setTimeOut(20);
}

bool writeBytes(uint8_t addr, const uint8_t *data, uint8_t len) {
  Wire.beginTransmission(addr);
  Wire.write(data, len);
  uint8_t err = Wire.endTransmission(true);
  if (err != 0) { busErrors++; return false; }
  busErrors = 0;
  return true;
}

uint8_t readBytes(uint8_t addr, uint8_t *out, uint8_t len) {
  (void)Wire.requestFrom(addr, len);
  uint8_t i = 0;
  while (Wire.available() && i < len) out[i++] = (uint8_t)Wire.read();
  while (Wire.available()) Wire.read();
  return i;
}

bool readReg(uint8_t addr, uint8_t reg, uint8_t *out, uint8_t len) {
  if (!writeBytes(addr, &reg, 1)) return false;
  return readBytes(addr, out, len) == len;
}

void markAllModulesAbsent() {
  for (uint8_t i = 0; i < lb::MAX_MODULES; i++) {
    if (modules[i].present && modules[i].heldNote != 0xFF) {
      sendMidiNoteOff(MIDI_CH, modules[i].heldNote);
    }
    modules[i].present  = false;
    modules[i].heldNote = 0xFF;
  }
}

void busClear() {
  Wire.end();
  pinMode(pins::PIN_SDA, INPUT_PULLUP);
  pinMode(pins::PIN_SCL, OUTPUT_OPEN_DRAIN);
  for (int i = 0; i < 9 && digitalRead(pins::PIN_SDA) == LOW; i++) {
    digitalWrite(pins::PIN_SCL, LOW);  delayMicroseconds(5);
    digitalWrite(pins::PIN_SCL, HIGH); delayMicroseconds(5);
  }
  pinMode(pins::PIN_SDA, OUTPUT_OPEN_DRAIN);
  digitalWrite(pins::PIN_SDA, LOW);  delayMicroseconds(5);
  digitalWrite(pins::PIN_SCL, HIGH); delayMicroseconds(5);
  digitalWrite(pins::PIN_SDA, HIGH); delayMicroseconds(5);
  pinMode(pins::PIN_SDA, INPUT_PULLUP);
  busBegin();
  busErrors = 0;
}

int findByUid(const uint8_t *uid) {
  for (uint8_t i = 0; i < lb::MAX_MODULES; i++)
    if (modules[i].known && lb::uidEqual(modules[i].uid, uid)) return i;
  return -1;
}

int firstFreeSlot() {
  for (uint8_t i = 0; i < lb::MAX_MODULES; i++) if (!modules[i].known) return i;
  for (uint8_t i = 0; i < lb::MAX_MODULES; i++) if (!modules[i].present) return i;
  return -1;
}

bool discoverOnce() {
  uint8_t f[lb::BOOT_FRAME_LEN];
  uint8_t n = readBytes(lb::BOOT_ADDR, f, lb::BOOT_FRAME_LEN);
  if (n == 0) return false;

  bool good = (n == lb::BOOT_FRAME_LEN) && (f[0] == lb::MAGIC) && (lb::crc8(f, 11) == f[11]);
  if (!good) {
    collisions++;
    uint8_t cmd = lb::CMD_BACKOFF;
    writeBytes(lb::BOOT_ADDR, &cmd, 1);
    return false;
  }

  const uint8_t *uid = &f[3];
  int idx = findByUid(uid);
  if (idx < 0) idx = firstFreeSlot();
  if (idx < 0) return false;
  uint8_t addr = addrOf(idx);

  uint8_t cmd[1 + lb::UID_LEN + 2];
  cmd[0] = lb::CMD_ASSIGN;
  memcpy(&cmd[1], uid, lb::UID_LEN);
  cmd[1 + lb::UID_LEN] = addr;
  cmd[2 + lb::UID_LEN] = lb::crc8(cmd, 1 + lb::UID_LEN + 1);
  if (!writeBytes(lb::BOOT_ADDR, cmd, sizeof(cmd))) return false;

  delay(10);
  uint8_t ident[lb::BOOT_FRAME_LEN];
  if (!readReg(addr, lb::REG_IDENT, ident, lb::BOOT_FRAME_LEN) || !lb::uidEqual(&ident[3], uid)) {
    uint8_t r = lb::REG_UNASSIGN;
    writeBytes(addr, &r, 1);
    return false;
  }

  Module &m   = modules[idx];
  bool replug = m.known;
  m.known     = true;
  m.present   = true;
  memcpy(m.uid, uid, lb::UID_LEN);
  m.type      = f[1];
  m.misses    = 0;
  m.lastSeq   = 0;
  m.heldNote  = 0xFF;
  m.lastPotCC = 0xFF;
  if (!serialBinaryMidiMode) {
    Serial.printf("[BUS] Module type %u %s at 0x%02X\n",
                  m.type, replug ? "re-plugged" : "assigned", addr);
  }
  return true;
}

void readModule(uint8_t idx) {
  Module &m = modules[idx];
  uint8_t addr = addrOf(idx);
  uint8_t ev[lb::EVENTS_LEN];
  if (!readReg(addr, lb::REG_EVENTS, ev, lb::EVENTS_LEN)) {
    if (++m.misses >= MISS_LIMIT) {
      m.present = false;
      if (m.heldNote != 0xFF) {
        sendMidiNoteOff(MIDI_CH, m.heldNote);
        m.heldNote = 0xFF;
      }
      if (!serialBinaryMidiMode) Serial.printf("[BUS] Module at 0x%02X removed\n", addr);
    }
    return;
  }
  m.misses = 0;
  if (ev[0] == m.lastSeq) return;
  m.lastSeq = ev[0];

  bool pressed = ev[1] & 0x01;
  uint8_t modNote = 72 + idx;
  if (pressed && m.heldNote == 0xFF) {
    m.heldNote = modNote;
    sendMidiNoteOn(MIDI_CH, modNote, 100);
  } else if (!pressed && m.heldNote != 0xFF) {
    sendMidiNoteOff(MIDI_CH, m.heldNote);
    m.heldNote = 0xFF;
  }

  uint16_t rawVal = ((uint16_t)ev[2] << 8) | ev[3];
  if (m.type == lb::TYPE_DEMO_POT || m.type == lb::TYPE_ENCODER) {
    uint8_t ccVal = (uint8_t)(rawVal >> 3);  // 10-bit (0..1023) -> 7-bit MIDI CC (0..127)
    if (ccVal > 127) ccVal = 127;
    if (ccVal != m.lastPotCC) {
      m.lastPotCC = ccVal;
      sendMidiCC(MIDI_CH, 16 + idx, ccVal);
    }
  }
}

void listModules() {
  Serial.println("addr  present  type  uid");
  for (uint8_t i = 0; i < lb::MAX_MODULES; i++) {
    if (!modules[i].known) continue;
    Serial.printf("0x%02X  %s      %u     ", addrOf(i), modules[i].present ? "yes" : "no ", modules[i].type);
    for (uint8_t b = 0; b < lb::UID_LEN; b++) Serial.printf("%02X", modules[i].uid[b]);
    Serial.println();
  }
  Serial.printf("collisions so far: %lu\n", (unsigned long)collisions);
}

void simultaneousReset() {
  Serial.println("[BUS] Resetting all followers at once");
  pinMode(pins::PIN_SIM_RESET, OUTPUT_OPEN_DRAIN);
  digitalWrite(pins::PIN_SIM_RESET, LOW);
  delay(10);
  digitalWrite(pins::PIN_SIM_RESET, HIGH);
  pinMode(pins::PIN_SIM_RESET, INPUT);
  markAllModulesAbsent();
}

// ---------------- Setup & Main Loop ----------------
void setup() {
  Serial.begin(115200);
  delay(200);

  for (uint8_t i = 0; i < pins::NUM_CARD_KEYS; i++) {
    pinMode(pins::CARD_KEY_PINS[i], INPUT_PULLUP);
    keyHistory[i]    = 0xFF;
    keyPressed[i]    = false;
    activeKeyNote[i] = 0xFF;
  }
  pinMode(pins::PIN_MODE_BTN, INPUT_PULLUP);
  pinMode(pins::PIN_ALERT, INPUT_PULLUP);
  pinMode(pins::PIN_MOD_PWR_EN, OUTPUT);
  digitalWrite(pins::PIN_MOD_PWR_EN, HIGH);
  pinMode(pins::PIN_SIM_RESET, INPUT);

  memset(modules, 0, sizeof(modules));
  for (uint8_t i = 0; i < lb::MAX_MODULES; i++) modules[i].heldNote = 0xFF;

  busBegin();
  initMpu6050();
  initBleMidi();
  lastScanUs = micros();

  Serial.println("=== Team H03 LatchBeat MIDI Card + Leader Ready ===");
  Serial.println("BLE-MIDI Advertising as: 'LatchBeat MIDI'");
  Serial.println("Serial Commands:");
  Serial.println("  'd' = Toggle Live GPIO Pin-Finder (press any button to see its GPIO #)");
  Serial.println("  'i' = Scan I2C Bus (checks MPU-6050 at 0x68 & Followers at 0x30/0x10..0x27)");
  Serial.println("  'm' = Cycle Musical Scale (flushes active notes first)");
  Serial.println("  'p' = Panic All-Notes-Off");
  Serial.println("  'b' = Toggle Binary Serial MIDI Mode (for Hairless MIDI Bridge)");
  Serial.println("  'l' = List LatchBeat Follower Modules | 's' = Simultaneous Follower Reset");
}

void loop() {
  //Deterministic 1 kHz (1000 us) non-blocking Card key scan (H-04)
  uint32_t nowUs = micros();
  if ((uint32_t)(nowUs - lastScanUs) >= 1000) {
    lastScanUs += 1000;
    scanCardKeys1kHz();
  }

  // Non-blocking 50 Hz MPU-6050 tilt poll (H-04)
  uint32_t nowMs = millis();
  if ((nowMs - lastImuPoll) >= IMU_POLL_MS) {
    lastImuPoll = nowMs;
    pollMpu6050();
  }

  //Non-blocking LatchBeat follower discovery & event polling (H-02 / H-03)
  bool alert = (digitalRead(pins::PIN_ALERT) == LOW);
  if (alert || (nowMs - lastDiscover) >= DISCOVER_MS) {
    lastDiscover = nowMs;
    for (uint8_t tries = 0; tries < 4 && discoverOnce(); tries++) {}
  }
  if (alert || (nowMs - lastHeartbeat) >= HEARTBEAT_MS) {
    lastHeartbeat = nowMs;
    for (uint8_t i = 0; i < lb::MAX_MODULES; i++) {
      if (modules[i].present) readModule(i);
    }
  }

  if (busErrors >= ERR_LIMIT && digitalRead(pins::PIN_SDA) == LOW) busClear();

  // Control Commands & Serial Diagnostic
  if (Serial.available()) {
    char c = (char)Serial.read();
    if (c == 'm') cycleScaleMode();
    else if (c == 'p') flushAllCardNotes();
    else if (c == 'i') runI2cScanner();
    else if (c == 'l') listModules();
    else if (c == 's') simultaneousReset();
    else if (c == 'b') {
      serialBinaryMidiMode = !serialBinaryMidiMode;
      if (!serialBinaryMidiMode) Serial.println("\n[MODE] Switched to Human-Readable Serial Monitor");
    } else if (c == 'd') {
      pinDiagnosticMode = !pinDiagnosticMode;
      if (pinDiagnosticMode) {
        for (uint8_t i = 0; i < DIAG_PIN_COUNT; i++) {
          pinMode(DIAG_PINS[i], INPUT_PULLUP);
          diagPinLast[i] = (uint8_t)digitalRead(DIAG_PINS[i]);
        }
        Serial.println("[PIN FINDER ON] Press any breadboard button to see its exact ESP32 GPIO number ('d' again to exit)");
      } else {
        Serial.println("[PIN FINDER OFF] Returning to normal MIDI key scanning");
      }
    }
  }
}
