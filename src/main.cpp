#include <Arduino.h>
#include <Wire.h>
#include "Adafruit_TinyUSB.h"
#include "Adafruit_MPR121.h"

/* --------- Config --------- */
#define SDP_ADDR 0x25    /* 7-bit I2C address */
#define I2C_HZ 400000

#define CMD_ENTER_MODE 0x367C
#define CMD_READ_PRODUCTID 0xE102
#define CMD_START_CONT 0x361E

#define SERIAL_BAUD 115200

#define HEARTBEAT_MS 100
#define READ_PERIOD_MS 20
#define PRINT_THRESHOLD_PA 100.0f

#define AWAIT_SERIAL

#ifndef _BV
#define _BV(bit) (1 << (bit)) 
#endif

// You can have up to 4 on one i2c bus but one is enough for testing!
Adafruit_MPR121 cap = Adafruit_MPR121();

// Keeps track of the last pins touched
// so we know when buttons are 'released'
uint16_t lasttouched = 0;
uint16_t currtouched = 0;


/* --------- Sensirion CRC-8 (poly 0x31, init 0xFF) --------- */
static uint8_t sdp_crc8(uint8_t msb, uint8_t lsb)
{
	uint8_t crc = 0xFF;
	int i;

	crc ^= msb;
	for (i = 0; i < 8; i++)
		crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x31) : (uint8_t)(crc << 1);

	crc ^= lsb;
	for (i = 0; i < 8; i++)
		crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x31) : (uint8_t)(crc << 1);

	return crc;
}

/* --------- I2C helpers --------- */
static bool sdp_write_cmd(uint16_t cmd, bool repeated_start)
{
	Wire.beginTransmission(SDP_ADDR);
	Wire.write((uint8_t)(cmd >> 8));
	Wire.write((uint8_t)(cmd & 0xFF));
	/* endTransmission(stop = !repeated_start) */
	return Wire.endTransmission(!repeated_start) == 0;
}

static bool sdp_read_words(uint8_t n_words, uint16_t *out)
{
	const uint8_t need = n_words * 3;
	uint8_t msb, lsb, crc;
	uint8_t i;

	if (Wire.requestFrom(SDP_ADDR, need) != need)
		return false;

	for (i = 0; i < n_words; i++) {
		msb = Wire.read();
		lsb = Wire.read();
		crc = Wire.read();

		if (sdp_crc8(msb, lsb) != crc) {
			while (Wire.available())
				(void)Wire.read();
			return false;
		}
		out[i] = ((uint16_t)msb << 8) | lsb;
	}

	return true;
}

/* --------- utils --------- */
static void heartbeat_tick(void)
{
	static uint32_t t0;
	uint32_t now = millis();

	if (now - t0 >= HEARTBEAT_MS) {
		t0 = now;
		digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
	}
}

static void print_hex_words(const uint16_t *w, size_t n)
{
	size_t i;

	for (i = 0; i < n; i++) {
		Serial.print("  w");
		Serial.print((int)i);
		Serial.print(" = 0x");
		if (w[i] < 0x1000) Serial.print('0');
		if (w[i] < 0x0100) Serial.print('0');
		if (w[i] < 0x0010) Serial.print('0');
		Serial.println(w[i], HEX);
	}
}

//static bool mpr121_write_u8(uint8_t reg, uint8_t val)
//{
//    Wire.beginTransmission(MPR121_ADDR);
//    Wire.write(reg);
//    Wire.write(val);
//    return Wire.endTransmission(true) == 0; /* send STOP */
//}
//
//
//static bool mpr121_read_bytes(uint8_t reg, uint8_t *buf, uint8_t n)
//{
//    /* set register pointer, then repeated START to read */
//    Wire.beginTransmission(MPR121_ADDR);
//    Wire.write(reg);
//    if (Wire.endTransmission(false) != 0) /* hold bus */
//        return false;
//
//    if (Wire.requestFrom(MPR121_ADDR, n) != n)
//        return false;
//
//    for (uint8_t i = 0; i < n; i++)
//        buf[i] = Wire.read();
//    return true;
//}
//
//static void mpr121_probe_print(void)
//{
//    uint8_t v[4];
//
//    /* Soft reset (write 0x63 to 0x80), per datasheet */
//    if (!mpr121_write_u8(MPR121_SOFTRESET, 0x63)) {
//        Serial.println("MPR121: soft reset write FAILED");
//        return;
//    }
//    delay(1); /* tiny settle; typical libs use ~1ms */
//
//    /* Read a small ID-ish block: CFG1, CFG2, FILTER_GLOBAL, ECR */
//    if (!mpr121_read_bytes(MPR121_CONFIG1, v, 4)) {
//        Serial.println("MPR121: register read FAILED");
//        return;
//    }
//
//    Serial.print("MPR121 regs @0x5B..0x5E: ");
//    for (int i = 0; i < 4; i++) {
//        Serial.print("0x");
//        if (v[i] < 0x10) Serial.print('0');
//        Serial.print(v[i], HEX);
//        if (i != 3) Serial.print(' ');
//    }
//    Serial.println();
//
//    if (v[3] == 0x00) {
//        Serial.println("MPR121 present: ECR=0x00 after reset (expected).");
//    } else {
//        Serial.print("MPR121 ECR unexpected: 0x");
//        if (v[3] < 0x10) Serial.print('0');
//        Serial.println(v[3], HEX);
//    }
//}

void setup(void)
{
	int i;
	uint16_t w[6];

	pinMode(LED_BUILTIN, OUTPUT);
	digitalWrite(LED_BUILTIN, HIGH);

	for (int i = 0; i < 10; i++) {
		digitalWrite(LED_BUILTIN, HIGH);
		delay(50);
		digitalWrite(LED_BUILTIN, LOW);
		delay(50);
	}
	digitalWrite(LED_BUILTIN, HIGH);

	/* USB + Serial */
	TinyUSBDevice.begin();
	while (!TinyUSBDevice.mounted()) {
		delay(10);
	}
	Serial.begin(SERIAL_BAUD);

#ifdef AWAIT_SERIAL
	/* Wait indefinitely for DTR (Serial) */
	while (!Serial) {
		delay(50);
		digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
	}
#endif

	/* I2C */
//	Wire.begin();             /* XIAO nRF52840: D4=SDA, D5=SCL */
//	Wire.setClock(I2C_HZ);
//
//	Serial.println("SDP init...");

//	/* Read Product ID: enter mode -> issue read cmd (repeated START) -> read 6 words */
//	if (!sdp_write_cmd(CMD_ENTER_MODE, false)) {
//		Serial.println("FAIL: enter mode (0x367C)");
//	} else if (!sdp_write_cmd(CMD_READ_PRODUCTID, true)) {
//		Serial.println("FAIL: read product ID cmd (0xE102)");
//	} else if (sdp_read_words(6, w)) {
//		Serial.println("Product ID words:");
//		print_hex_words(w, 6);
//	} else {
//		Serial.println("FAIL: product ID read/CRC");
//	}
//
//	/* Start continuous measurement */
//	if (sdp_write_cmd(CMD_START_CONT, false))
//		Serial.println("Continuous measurement started (0x361E)");
//	else
//		Serial.println("FAIL: start continuous (0x361E)");

	Serial.println("MPR121 probe init...");
//	mpr121_probe_print();

  if (!cap.begin(0x5A)) {
    Serial.println("MPR121 not found, check wiring?");
    while (1);
  }
  Serial.println("MPR121 found!");


	delay(10);                /* let first sample appear */
}

void loop(void)
{
//	static uint32_t t_last;
//	uint32_t now = millis();
//	uint16_t words[3];
//	int16_t dp_raw;
//	float dp_pa;

//	heartbeat_tick();

	// Get the currently touched pads
	currtouched = cap.touched();

	for (uint8_t i = 0; i < 12; i++) {
		// it if *is* touched and *wasnt* touched before, alert!
		if ((currtouched & _BV(i)) && !(lasttouched & _BV(i)) ) {
			Serial.print(i);
			Serial.println(" touched");
		}
		// if it *was* touched and now *isnt*, alert!
		if (!(currtouched & _BV(i)) && (lasttouched & _BV(i)) ) {
			Serial.print(i);
			Serial.println(" released");
		}
	}

	// reset our state
	lasttouched = currtouched;

	// debugging info, what
//	Serial.print("\t\t\t\t\t\t\t\t\t\t\t\t\t 0x");
//	Serial.println(cap.touched(), HEX);
	Serial.print("Filt: ");
	for (uint8_t i = 0; i < 12; i++) {
		Serial.print(cap.filteredData(i));
		Serial.print(" ");
	}
	Serial.println();
//	Serial.print("Base: ");
//	for (uint8_t i = 0; i < 12; i++) {
//		Serial.print(cap.baselineData(i));
//		Serial.print(" ");
//	}
//	Serial.println();
  
	delay(50);

//	if (now - t_last < READ_PERIOD_MS)
//		return;
//	t_last = now;
//
//	/* Typical frame: dp, aux (temp/scale), status */
//	if (!sdp_read_words(3, words))
//		return;

//	dp_raw = (int16_t)words[0];
//	/* Many modes are 1 Pa/LSB; verify in datasheet for your command. */
//	dp_pa = (float)dp_raw;
//
//	if (dp_pa > PRINT_THRESHOLD_PA) {
//		Serial.println(dp_pa);
//		Serial.flush();
//	}
}