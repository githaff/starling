#include <Arduino.h>
#include <Wire.h>
#include "Adafruit_TinyUSB.h"

#define SDP_ADDR 0x25  // 7-bit I2C address

// ---- Sensirion CRC-8 (poly 0x31, init 0xFF) ----
static uint8_t sdp_crc8(uint8_t b1, uint8_t b2) {
  uint8_t crc = 0xFF;
  crc ^= b1;
  for (int i=0;i<8;i++) crc = (crc & 0x80) ? (crc << 1) ^ 0x31 : (crc << 1);
  crc ^= b2;
  for (int i=0;i<8;i++) crc = (crc & 0x80) ? (crc << 1) ^ 0x31 : (crc << 1);
  return crc;
}

// ---- Write a 16-bit command; repeated_start=false sends STOP, true keeps bus for next read ----
static bool sdp_write_cmd(uint16_t cmd, bool repeated_start=false) {
  Wire.beginTransmission(SDP_ADDR);
  Wire.write((uint8_t)(cmd >> 8));
  Wire.write((uint8_t)(cmd & 0xFF));
  uint8_t err = Wire.endTransmission(!repeated_start);  // false -> repeated START
  return err == 0;
}

// ---- Read N 16-bit words, each followed by a CRC byte ----
static int sdp_read_words(uint8_t n_words, uint16_t* out) {
  const uint8_t need = n_words * 3;
  uint8_t got = Wire.requestFrom(SDP_ADDR, need);
  if (got != need) return 0;

  int ok = 0;
  for (uint8_t i=0; i<n_words; ++i) {
    uint8_t msb = Wire.read();
    uint8_t lsb = Wire.read();
    uint8_t crc = Wire.read();
    if (sdp_crc8(msb, lsb) != crc) {
      // drain remaining bytes to keep Wire state clean
      while (Wire.available()) (void)Wire.read();
      return ok;
    }
    out[i] = ((uint16_t)msb << 8) | lsb;
    ok++;
  }
  return ok;
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);

  // USB CDC (you already have this working)
  TinyUSBDevice.begin();
  while (!TinyUSBDevice.mounted()) { delay(10); }
  Serial.begin(115200);
  while (!Serial) { /* block until DTR */ }

  // Optional UART (if you also want Serial1 logs)
  Serial1.setPins(D7, D6);
  Serial1.begin(115200);

  // I2C
  Wire.begin();              // XIAO nRF52840: D4=SDA, D5=SCL
  Wire.setClock(400000);

  Serial.println("Hello, world!");

  // --- Product ID sequence you’re using: 0x367C then read via 0xE102 ---
  // Enter/prepare command mode:
  if (!sdp_write_cmd(0x367C)) {
    Serial.println("FAIL: 0x367C");
  }

  // Send the "read product ID" command with a repeated START, then read 6 words (18 bytes)
  if (!sdp_write_cmd(0xE102, true)) {
    Serial.println("FAIL: 0xE102");
  } else {
    uint16_t w[6];
    int n = sdp_read_words(6, w);   // 6 words = 12 data bytes + 6 CRC = 18 bytes total
    Serial.print("Product ID words ("); Serial.print(n); Serial.println("):");
    for (int i=0;i<n;i++) {
      Serial.print("  w"); Serial.print(i); Serial.print(" = 0x");
      if (w[i] < 0x1000) Serial.print('0');
      if (w[i] < 0x100)  Serial.print('0');
      if (w[i] < 0x10)   Serial.print('0');
      Serial.println(w[i], HEX);
    }
  }

  // --- Start continuous differential-pressure measurement (your 0x361E) ---
  if (!sdp_write_cmd(0x361E)) {
    Serial.println("FAIL: start cont meas (0x361E)");
  } else {
    Serial.println("Continuous measurement started (0x361E)");
  }

  // Give first sample time to appear
  delay(10);
}

void loop() {
  // Blink so we know firmware runs
  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
//  delay(100);

  // Many SDP8xx continuous modes return 3 words: dp, temperature/scale, status
  uint16_t words[3];
  int n = sdp_read_words(3, words);
  if (n == 3) {
    int16_t dp_raw = (int16_t)words[0];  // signed 16-bit
    // For typical diff-pressure modes, scale is 1 Pa/LSB (confirm in your command’s table)
    float dp_pa = (float)dp_raw;

//    Serial.print("dp_raw="); Serial.print(dp_raw);
//    Serial.print("  dp[Pa]="); Serial.print(dp_pa);
//    Serial.print("  w1=0x"); Serial.print(words[1], HEX);
//    Serial.print("  w2=0x"); Serial.println(words[2], HEX);

//    Serial.println(dp_pa);
//    Serial.flush();

    if (dp_pa > 100.0) {
      Serial.println(dp_pa);
      Serial.flush();
    }

  } else if (n == 0) {
    Serial.println("read: short (no data)");  // sensor not ready / bus issue
  } else {
    Serial.print("read: partial ("); Serial.print(n); Serial.println(" words)");
  }

  delay(20);  // polling rate; tune per datasheet update rate
}




//#include <Arduino.h>
//#include <Wire.h>
//#include "Adafruit_TinyUSB.h"
//
//#define I2C_ADDR 0x25
//
//
//void sendCommand(uint16_t cmd) {
//  Wire.beginTransmission(I2C_ADDR);
//
//  // high byte first
//  Wire.write((cmd >> 8) & 0xFF);
//  Wire.write(cmd & 0xFF);
//
//  uint8_t err = Wire.endTransmission();  // actually sends STOP
//  if (err != 0) {
//    Serial.println("FAIL TO SEND I2C COMMAND");
//    // handle error: 0 = success
//  }
//}
//
//void setup() {
//  pinMode(LED_BUILTIN, OUTPUT);   // LED_BUILTIN resolves to D13
//
//  Wire.begin();
//  Wire.setClock(400000);
//
//  TinyUSBDevice.begin();
//  while (!TinyUSBDevice.mounted()) { delay(10); }
//
//  Serial.begin(115200);
//  while (!Serial) { ; }
//  Serial.println("Hello, world!");
//  Serial.flush();
//
////  Serial1.setPins(D7, D6);
////  Serial1.begin(115200);
////  Serial1.println("Hello, world! Serial1");
////  Serial1.flush();
//
//  sendCommand(0x367c);
//  sendCommand(0xe102);
//
//  // 2) request 18 bytes back
//  uint8_t count = Wire.requestFrom(I2C_ADDR, (uint8_t)18);
//  if (count != 18) {
//    Serial.print("I2C read short: got ");
//    Serial.println(count);
//    return;
//  }
//
//    // 3) read and print
//  Serial.print("Sensor product ID: ");
//  while (Wire.available()) {
//    uint8_t b = Wire.read();
//    Serial.print("0x");
//    Serial.print(b, HEX);
//    Serial.print(" ");
//  }
//  Serial.println();
//
//  // Continuous measurement
//  // Maybe makes sense to get 'average till read' instead.
//  sendCommand(0x361e);
//
//}
//
//int16_t readRaw() {
//  Wire.requestFrom(I2C_ADDR, (uint8_t)9);  // 2 data bytes + 1 CRC
//  if (Wire.available() < 9) {
//    Serial.println("I2C read short");
//    return 0;
//  }
//
//  uint8_t msb = Wire.read();
//  uint8_t lsb = Wire.read();
//  uint8_t crc = Wire.read();  // unused for now
//
//  uint8_t msb_t = Wire.read();
//  uint8_t lsb_t = Wire.read();
//  uint8_t crc_t = Wire.read();  // unused for now
//
//  uint8_t msb_s = Wire.read();
//  uint8_t lsb_s = Wire.read();
//  uint8_t crc_s = Wire.read();  // unused for now
//
//  int16_t raw = (int16_t)((msb_t << 8) | lsb_t);
//  return raw;
//}
//
//void loop() {
//  static int x = 0;
//  digitalWrite(LED_BUILTIN, HIGH);  // turn LED on
//  delay(100);
//  digitalWrite(LED_BUILTIN, LOW);   // turn LED off
//  delay(100);
//  Serial.print("Looped ");
//  Serial.println(x++);
//  Serial.flush();
//
//  Serial1.print("Serial ");
//  Serial1.println(x);
//  Serial1.flush();
//
////int16_t p = readPressureRaw();
////float pressure_hPa = (float)p;
////
////Serial.println(p);
////Serial.println(pressure_hPa);
//
//  uint8_t count = Wire.requestFrom(I2C_ADDR, (uint8_t)3);
//  Serial.print(count);
//  Serial.print(" : ");
//
//  Serial.print("Sensor product ID: ");
//  while (Wire.available()) {
//    uint8_t b = Wire.read();
//    Serial.print("0x");
//    if (b < 0x10) Serial1.print("0");
//    Serial.print(b, HEX);
//    Serial.print(" ");
//  }
//  Serial.println();
//}
//