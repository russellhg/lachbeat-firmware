// pin_map.h - Team H03 (LatchBeat / MusicCard) Unified Hardware Pin Map (Task H-01)
//
// Centralizes all pin assignments across:
//   1. Breadboard / 2025 Card Prototype (HiLetgo ESP-32S / ESP-WROOM-32)
//   2. V1 Custom PCB Leader (ESP32-S3-WROOM-1 / ESP32-S3-MINI-1, 12-Pin Bus)
//   3. Follower Modules (Arduino Pro Mini ATmega328P 3.3V 8MHz for H-03 breadboard)

#pragma once
#include <stdint.h>

namespace pins {

#if defined(ARDUINO_AVR_PRO) || defined(__AVR_ATmega328P__)
// ============================================================================
// 3. FOLLOWER MODULE PIN MAP (Arduino Pro Mini 3.3V / 8MHz - Task H-03)
// ============================================================================
const uint8_t PIN_I2C_SDA   = A4;   // Off-grid inner hole A4 on Pro Mini (Pin 3)
const uint8_t PIN_I2C_SCL   = A5;   // Off-grid inner hole A5 on Pro Mini (Pin 4)
const uint8_t PIN_ALERT     = 2;    // D2: Open-drain active-low interrupt (Pin 5)
const uint8_t PIN_BUTTON    = 3;    // D3: Pushbutton / Gateron switch to GND
const uint8_t PIN_ENC_A     = 4;    // D4: Rotary encoder CLK (quadrature A) to GND
const uint8_t PIN_ENC_B     = 5;    // D5: Rotary encoder DT  (quadrature B) to GND
const uint8_t PIN_ENC_SW    = 6;    // D6: Rotary encoder push-switch to GND
const uint8_t PIN_POT       = A0;   // A0: 10k potentiometer wiper (0..3.3V)
const uint8_t PIN_LED       = 13;   // D13: Onboard status LED (HIGH = assigned)

#elif CONFIG_IDF_TARGET_ESP32S3
// ============================================================================
// 2. V1 CUSTOM PCB LEADER PIN MAP (ESP32-S3 - 12-Pin Module Bus)
// ============================================================================
// Native USB-MIDI (Internal PHY)
const int PIN_USB_DM        = 19;   // USB_D-
const int PIN_USB_DP        = 20;   // USB_D+

// 12-Pin Connector — Core Bus (Pins 1..6: Used by all LatchBeat I2C Followers)
// Pin 1 = GND, Pin 2 = 3V3_MOD (via AP22802 / TPS2553 load switch)
const int PIN_SDA           = 8;    // Pin 3: I2C0_SDA (2.2k - 4.7k pull-up to 3V3_MOD)
const int PIN_SCL           = 9;    // Pin 4: I2C0_SCL (2.2k - 4.7k pull-up to 3V3_MOD)
const int PIN_ALERT         = 15;   // Pin 5: Active-low open-drain interrupt
const int PIN_DETECT        = 16;   // Pin 6: Module presence detect (pulled low on mate)
const int PIN_MOD_PWR_EN    = 21;   // Enables AP22802 / TPS2553 3V3_MOD load switch
const int PIN_SIM_RESET     = 4;    // Demo simultaneous follower reset line

// 12-Pin Connector — Expansion Lanes (Pins 7..12: Reserved for V2 Screen/Audio/ADC)
const int PIN_EXP_SCK       = 12;   // Pin 7:  SPI2_SCK  / I2S1_BCK
const int PIN_EXP_MOSI      = 11;   // Pin 8:  SPI2_MOSI / I2S1_DOUT
const int PIN_EXP_MISO      = 13;   // Pin 9:  SPI2_MISO / I2S1_DIN
const int PIN_EXP_CS        = 10;   // Pin 10: SPI2_CS0  / I2S1_WS
const int PIN_MOD_ADC0      = 5;    // Pin 11: ADC1_CH4  / TOUCH5
const int PIN_MOD_ADC1      = 6;    // Pin 12: ADC1_CH5  / TOUCH6 (or 5V_VBUS on Dock)

// Onboard Card Internal I2C1 Bus (MPU-6050 / LSM6DS3 IMU)
const int PIN_IMU_SDA       = 47;   // Dedicated internal IMU SDA
const int PIN_IMU_SCL       = 48;   // Dedicated internal IMU SCL

// 8 Scale-Locked Card Keys (Active-Low or Native Touch) + Mode Button
const uint8_t NUM_CARD_KEYS = 8;
const int CARD_KEY_PINS[NUM_CARD_KEYS] = { 1, 2, 3, 5, 6, 7, 14, 17 };
const int PIN_MODE_BTN      = 0;    // GPIO0: Scale/Mode shift button on ESP32-S3

// Addressable RGB LEDs & Battery Sense
const int PIN_RGB_LED       = 38;   // WS2812 / SK6812 data line (RMT DMA)
const int PIN_VBAT_ADC      = 18;   // 100k/100k voltage divider from LiPo VBAT

#else
// ============================================================================
// 1. BREADBOARD & 2025 CARD PROTOTYPE PIN MAP (HiLetgo ESP-32S / ESP-WROOM-32)
//    Matches exact rainbow ribbon & MPU-6050 wiring in photos (1000025728..747)
// ============================================================================
// Shared I2C Bus (MPU-6050 at 0x68 + LatchBeat Followers at 0x30 / 0x10..0x27)
const int PIN_SDA           = 21;   // P21 (Grey wire to MPU-6050 SDA)
const int PIN_SCL           = 22;   // P22 (Purple wire to MPU-6050 SCL)
const int PIN_IMU_SDA       = 21;   // Shares I2C0 on classic ESP-32S breadboard
const int PIN_IMU_SCL       = 22;

// LatchBeat Bus Control Pins (Moved off P4 to avoid button ribbon conflict!)
const int PIN_ALERT         = 23;   // P23 (Supports internal pull-up; or P34 w/ ext 10k)
const int PIN_MOD_PWR_EN    = 25;   // P25 (Right-side header, free of button ribbon)
const int PIN_SIM_RESET     = 26;   // P26 (Moved from P4 so P4 button works cleanly)

// 8 Breadboard / Card Tactile Buttons (Wired via Rainbow Ribbon to Left Header)
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
const int PIN_MODE_BTN      = 19;   // P19: Optional 9th ribbon wire for Scale/Mode shift

#endif

}  // namespace pins
