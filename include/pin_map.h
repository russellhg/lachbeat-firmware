// pin_map.h - Latchbeat Pin Map for unified hardware (H-01)
//
// Centralizes pin assignments across:
// Breadboard (ESP-32)
// PCB Leader (ESP32-S3, Pinout v0.2)
// Follower Modules (Arduino Pro Mini ATmega328P 3.3V 8MHz for H-03 breadboard)

#pragma once
#include <stdint.h>

namespace pins {

#if defined(ARDUINO_AVR_PRO) || defined(AVR_ATmega328P)
// FOLLOWER MODULE PIN MAP (Arduino PM 3.3V/8MHz H-03)
const uint8_t PIN_I2C_SDA   = A4;   // Off-grid inner hole A4 on PM
const uint8_t PIN_I2C_SCL   = A5;   // Off-grid inner hole A5 on PM
const uint8_t PIN_ALERT     = 2;    // D2: Open-drain active-low interrupt to leader
const uint8_t PIN_BUTTON    = 3;    // D3: PB/Gateron switch to GND
const uint8_t PIN_ENC_A     = 4;    // D4: Rotary encoder CLK (quadrature A) to GND
const uint8_t PIN_ENC_B     = 5;    // D5: Rotary encoder DT  (quadrature B) to GND
const uint8_t PIN_ENC_SW    = 6;    // D6: Rotary encoder push-switch to GND
const uint8_t PIN_POT       = A0;   // A0: 10k potentiometer wiper
const uint8_t PIN_LED       = 13;   // D13: Onboard status LED (HIGH = assigned)

#elif CONFIG_IDF_TARGET_ESP32S3
// 2. V1 CUSTOM PCB LEADER PIN MAP (ESP32-S3 - Pinout)
// Native USB-MIDI (Internal PHY)
const int PIN_USB_DM        = 19;   // USB_D-
const int PIN_USB_DP        = 20;   // USB_D+

// Shared Module I2C Bus/Hot-Plug Control
const int PIN_SDA           = 8;    // I2C0_SDA (2.2k - 4.7k pull-up to 3V3_MOD)
const int PIN_SCL           = 9;    // I2C0_SCL (2.2k - 4.7k pull-up to 3V3_MOD)
const int PIN_ALERT         = 15;   // Active-low open-drain interrupt from followers
const int PIN_DETECT        = 16;   // Module presence detect (pulled low on mate)
const int PIN_MOD_PWR_EN    = 21;   // Enables AP22802 / TPS2553 3V3_MOD load switch
const int PIN_SIM_RESET     = 4;    // Demo simultaneous follower reset line

// Onboard Card Internal I2C1 Bus (MPU-6050 / LSM6DS3 IMU)
const int PIN_IMU_SDA       = 47;   // Dedicated internal IMU SDA
const int PIN_IMU_SCL       = 48;   // Dedicated internal IMU SCL

// 8 Keys
const uint8_t NUM_CARD_KEYS = 8;
const int CARD_KEY_PINS[NUM_CARD_KEYS] = { 1, 2, 3, 5, 6, 7, 14, 17 };
const PIN_MODE_BTN = 0;

// LEDS & battery sense
const int PIN_RGB_LED       = 38;   // WS2812 / SK6812 data line
const int PIN_VBAT_ADC      = 18;   // 100k/100k voltage divider 

#else
// 1. BREADBOARD PROTOTYPE PIN MAP
// Shared I2C Bus (MPU-6050 at 0x68 + LatchBeat Followers at 0x30 / 0x10..0x27)
// Note: Blue MPU-6050 breakout PCB provides onboard 4.7k pull-up resistors.
const int PIN_SDA           = 21;   // P21 (Grey wire to MPU-6050 SDA)
const int PIN_SCL           = 22;   // P22 (Purple wire to MPU-6050 SCL)
const int PIN_IMU_SDA       = 21;   // Shares I2C0 on classic ESP-32S breadboard
const int PIN_IMU_SCL       = 22;

// Bus Control Pins (Moved off P4 to avoid button ribbon conflict!)
const int PIN_ALERT         = 23;   // P23 (Supports internal pull-up; or P34 w/ ext 10k)
const int PIN_MOD_PWR_EN    = 25;   // P25 (Right-side header, free of button ribbon)
const int PIN_SIM_RESET     = 26;   // P26 (Moved from P4 so P4 button works cleanly)

// 8 Buttons (Rainbow Ribbon to Left Header)
// Active-LOW: Buttons connect pin to blue GND rail when pressed.
// Note: P15, P2, P0 are boot-strapping pins; don't hold down for reset
const uint8_t NUM_CARD_KEYS = 8;
const int CARD_KEY_PINS[NUM_CARD_KEYS] = {
  15,   // Key 0 (Root / C)   - P15
  2,    // Key 1 (2nd / D)    - P2
  0,    // Key 2 (3rd / E)    - P0
  4,    // Key 3 (4th / F)    - P4
  16,   // Key 4 (5th / G)    - P16
  17,   // Key 5 (6th / A)    - P17
  5,    // Key 6 (7th / B)    - P5
  18    // Key 7 (Octave / C) - P18 
};
const int PIN_MODE_BTN      = 19;   // Placeholder/optional ribbon wire for Scale/Mode shift

#endif

}  // namespace pins
