// follower_promini.cpp - Latchbeat Follower Firmware 
//
// Target: Arduino Pro Mini (ATmega328P, 3.3V, 8 MHz)
// Supports all following follower types via MODULE_TYPE
//   lb::TYPE_DEMO_BUTTON (0x01): Pushbutton on D3 to GND
//   lb::TYPE_DEMO_POT    (0x02): 10k Potentiometer on A0 (0..3.3V)
//   lb::TYPE_ENCODER     (0x03): Quadrature Rotary Encoder on D4 (CLK), D5 (DT), D6 (SW)

#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>
#include "pin_map.h"
#include "lb_bus.h"

// Select Module before flashing
const uint8_t MODULE_TYPE = lb::TYPE_DEMO_BUTTON;  // Change to TYPE_DEMO_POT or TYPE_ENCODER
const uint8_t FW_VERSION  = 0x01;

const uint8_t EEPROM_UID_ADDR    = 0;
const uint8_t EEPROM_MARKER_ADDR = 8;
const uint8_t EEPROM_MARKER      = 0x5A;

enum State : uint8_t { ST_WAIT_START, ST_UNASSIGNED, ST_BACKOFF, ST_ASSIGNED };
volatile State state = ST_WAIT_START;

uint8_t uid[lb::UID_LEN];
uint8_t bootFrame[lb::BOOT_FRAME_LEN];
uint8_t myAddr = lb::BOOT_ADDR;

volatile uint8_t  regPtr            = lb::REG_EVENTS;
volatile uint8_t  pendingAddr       = 0;
volatile bool     backoffRequested  = false;
volatile bool     unassignRequested = false;
volatile bool     eventsRead        = false;

volatile uint8_t  evSeq     = 0;
volatile uint8_t  evButtons = 0;
volatile uint16_t evValue   = 512;  // 0..1023 for pot or scaled encoder position
bool eventPending           = false;

unsigned long stateTimer = 0;
unsigned long waitMs     = 0;

void alertAssert()  { digitalWrite(pins::PIN_ALERT, LOW); pinMode(pins::PIN_ALERT, OUTPUT); }
void alertRelease() { pinMode(pins::PIN_ALERT, INPUT); }

void loadOrCreateUid() {
  if (EEPROM.read(EEPROM_MARKER_ADDR) == EEPROM_MARKER) {
    for (uint8_t i = 0; i < lb::UID_LEN; i++) uid[i] = EEPROM.read(EEPROM_UID_ADDR + i);
    return;
  }
  uint32_t seed = micros();
  for (uint8_t i = 0; i < 32; i++) {
    seed = (seed << 3) ^ (seed >> 29) ^ (uint32_t)analogRead(A1) ^ (uint32_t)analogRead(A2) ^ micros();
  }
  randomSeed(seed);
  for (uint8_t i = 0; i < lb::UID_LEN; i++) {
    uid[i] = (uint8_t)random(0, 256);
    EEPROM.update(EEPROM_UID_ADDR + i, uid[i]);
  }
  EEPROM.update(EEPROM_MARKER_ADDR, EEPROM_MARKER);
}

void buildBootFrame() {
  bootFrame[0] = lb::MAGIC;
  bootFrame[1] = MODULE_TYPE;
  bootFrame[2] = FW_VERSION;
  for (uint8_t i = 0; i < lb::UID_LEN; i++) bootFrame[3 + i] = uid[i];
  bootFrame[11] = lb::crc8(bootFrame, 11);
}

unsigned long randomDelayMs(unsigned long lo, unsigned long hi) {
  return (unsigned long)random((long)lo, (long)hi);
}

void onReceiveHandler(int howMany);
void onRequestHandler();

void startI2C(uint8_t addr) {
  Wire.begin(addr);
  Wire.onReceive(onReceiveHandler);
  Wire.onRequest(onRequestHandler);
  digitalWrite(SDA, LOW);
  digitalWrite(SCL, LOW);
  myAddr = addr;
}

void stopI2C() { Wire.end(); }

void onReceiveHandler(int howMany) {
  if (howMany <= 0) return;
  uint8_t buf[16];
  uint8_t n = 0;
  while (Wire.available() && n < sizeof(buf)) buf[n++] = (uint8_t)Wire.read();
  while (Wire.available()) Wire.read();

  if (state == ST_UNASSIGNED) {
    if (buf[0] == lb::CMD_BACKOFF) {
      backoffRequested = true;
    } else if (buf[0] == lb::CMD_ASSIGN && n == (1 + lb::UID_LEN + 1 + 1)) {
      uint8_t crc = lb::crc8(buf, 1 + lb::UID_LEN + 1);
      uint8_t newAddr = buf[1 + lb::UID_LEN];
      bool mine = lb::uidEqual(&buf[1], uid);
      if (crc == buf[1 + lb::UID_LEN + 1] && mine &&
          newAddr >= lb::SESSION_FIRST && newAddr <= lb::SESSION_LAST) {
        pendingAddr = newAddr;
      }
    }
  } else if (state == ST_ASSIGNED) {
    regPtr = buf[0];
    if (buf[0] == lb::REG_UNASSIGN) unassignRequested = true;
  }
}

void onRequestHandler() {
  if (state == ST_UNASSIGNED) {
    Wire.write(bootFrame, lb::BOOT_FRAME_LEN);
    return;
  }
  if (regPtr == lb::REG_IDENT) {
    Wire.write(bootFrame, lb::BOOT_FRAME_LEN);
  } else {
    uint8_t ev[lb::EVENTS_LEN] = {
      evSeq, evButtons, (uint8_t)(evValue >> 8), (uint8_t)(evValue & 0xFF)
    };
    Wire.write(ev, lb::EVENTS_LEN);
    eventsRead = true;
  }
}

void scanInputs() {
  static uint8_t stableBtn = 0, lastRawBtn = 0;
  static unsigned long lastBtnChange = 0;
  static uint16_t lastPot = 0;
  static uint8_t lastEncA = HIGH;
  unsigned long now = millis();
  bool changed = false;

  // Scan pushbutton (D3 or encoder push-switch D6)
  uint8_t rawBtn = ((digitalRead(pins::PIN_BUTTON) == LOW) ||
                    (digitalRead(pins::PIN_ENC_SW) == LOW)) ? 0x01 : 0x00;
  if (rawBtn != lastRawBtn) { lastRawBtn = rawBtn; lastBtnChange = now; }
  if ((now - lastBtnChange) >= 5 && rawBtn != stableBtn) {
    stableBtn = rawBtn;
    changed = true;
  }

  // Scan analog pot (if TYPE_DEMO_POT)
  if (MODULE_TYPE == lb::TYPE_DEMO_POT) {
    uint16_t pot = (uint16_t)analogRead(pins::PIN_POT);
    if (pot > lastPot + 8 || pot + 8 < lastPot) {
      lastPot = pot;
      changed = true;
    }
  }

  // Scan quadrature rotary encoder (if TYPE_ENCODER)
  if (MODULE_TYPE == lb::TYPE_ENCODER) {
    uint8_t encA = (uint8_t)digitalRead(pins::PIN_ENC_A);
    if (encA != lastEncA && encA == LOW) {
      int16_t step = (digitalRead(pins::PIN_ENC_B) != encA) ? 16 : -16;
      int16_t nextVal = (int16_t)lastPot + step;
      if (nextVal < 0) nextVal = 0;
      if (nextVal > 1023) nextVal = 1023;
      lastPot = (uint16_t)nextVal;
      changed = true;
    }
    lastEncA = encA;
  }

  if (changed) {
    noInterrupts();
    evButtons = stableBtn;
    evValue   = lastPot;
    evSeq++;
    interrupts();
    eventPending = true;
  }
}

void setup() {
  pinMode(pins::PIN_BUTTON, INPUT_PULLUP);
  pinMode(pins::PIN_ENC_A, INPUT_PULLUP);
  pinMode(pins::PIN_ENC_B, INPUT_PULLUP);
  pinMode(pins::PIN_ENC_SW, INPUT_PULLUP);
  pinMode(pins::PIN_LED, OUTPUT);
  alertRelease();
  loadOrCreateUid();
  buildBootFrame();

  uint32_t s = 0;
  for (uint8_t i = 0; i < lb::UID_LEN; i++) s = (s * 31u) ^ uid[i];
  randomSeed(s ^ micros());

  state = ST_WAIT_START;
  stateTimer = millis();
  waitMs = randomDelayMs(10, 200);
}

void loop() {
  unsigned long now = millis();
  switch (state) {
    case ST_WAIT_START:
    case ST_BACKOFF:
      if (now - stateTimer >= waitMs) {
        startI2C(lb::BOOT_ADDR);
        state = ST_UNASSIGNED;
        alertAssert();
        digitalWrite(pins::PIN_LED, LOW);
      }
      break;

    case ST_UNASSIGNED:
      if (backoffRequested) {
        backoffRequested = false;
        stopI2C();
        alertRelease();
        state = ST_BACKOFF;
        stateTimer = now;
        waitMs = randomDelayMs(20, 300);
      } else if (pendingAddr != 0) {
        uint8_t a = pendingAddr;
        pendingAddr = 0;
        stopI2C();
        state = ST_ASSIGNED;
        startI2C(a);
        alertRelease();
        digitalWrite(pins::PIN_LED, HIGH);
      }
      break;

    case ST_ASSIGNED:
      if (unassignRequested) {
        unassignRequested = false;
        stopI2C();
        state = ST_BACKOFF;
        stateTimer = now;
        waitMs = randomDelayMs(20, 300);
        digitalWrite(pins::PIN_LED, LOW);
        break;
      }
      if (eventsRead)   { eventsRead = false; alertRelease(); }
      scanInputs();
      if (eventPending) { alertAssert(); eventPending = false; }
      break;
  }
}
