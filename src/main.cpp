//#define TX  // Comment this out for RX device

#include <Arduino.h>
#include <Wire.h>
#include "Adafruit_TinyUSB.h"
#include "Adafruit_MPR121.h"

#include "nrf.h"
#include "nrf_gpio.h"
#include "nrfx_clock.h"
//#include "nrf_esb.h"
#include <nrf_to_nrf.h>

nrf_to_nrf radio;

// Packet structure for TX->RX communication
struct SensorPacket {
	float pressure;      // Pressure value in Pa
	uint16_t buttons;    // Bitmask of pressed buttons (12 bits used)
};

/* --------- Config --------- */
#define SERIAL_BAUD 115200

//#define AWAIT_SERIAL

#ifndef _BV
#define _BV(bit) (1 << (bit)) 
#endif

#ifdef TX
// TX-only: I2C sensors
#define SDP_ADDR 0x25    /* 7-bit I2C address */
#define I2C_HZ 400000

#define CMD_ENTER_MODE 0x367C
#define CMD_READ_PRODUCTID 0xE102
#define CMD_START_CONT 0x361E

#define HEARTBEAT_MS 100
#define READ_PERIOD_MS 20
#define PRINT_THRESHOLD_PA 100.0f

// You can have up to 4 on one i2c bus but one is enough for testing!
Adafruit_MPR121 cap = Adafruit_MPR121();

// Keeps track of the last pins touched
// so we know when buttons are 'released'
uint16_t lasttouched = 0;
uint16_t currtouched = 0;
#endif

#ifdef TX
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
#endif  // TX-only I2C functions


void radio_init()
{
	uint8_t addr[6] = "SENSR";

	radio.begin();
	radio.setPALevel(NRF_PA_LOW);
	radio.setDataRate(NRF_2MBPS);
	radio.setChannel(80);
	radio.setCRCLength(NRF_CRC_8);
	radio.setAutoAck(false);
	radio.enableDynamicPayloads();  // Enable dynamic payloads
	
#ifdef TX
	radio.openWritingPipe(addr);
	radio.stopListening();
	Serial.println("Radio: TX mode");
#else
	radio.openReadingPipe(1, addr);
	radio.startListening();
	Serial.println("Radio: RX mode");
#endif
	
	radio.printDetails();
}

#ifdef TX
#define LED_STATUS D6
#else
#define LED_STATUS LED_BUILTIN
#endif

void setup(void)
{
#ifdef TX
	uint16_t w[6];
#endif

	pinMode(LED_STATUS, OUTPUT);
	digitalWrite(LED_STATUS, HIGH);

	for (int i = 0; i < 10; i++) {
		digitalWrite(LED_STATUS, HIGH);
		delay(50);
		digitalWrite(LED_STATUS, LOW);
		delay(50);
	}
	digitalWrite(LED_STATUS, HIGH);

//	/* USB + Serial */
//	TinyUSBDevice.begin();
//	while (!TinyUSBDevice.mounted()) {
//		delay(10);
//	}

	Serial.begin(SERIAL_BAUD);
#ifdef AWAIT_SERIAL
	/* Wait indefinitely for DTR (Serial) */
	while (!Serial) {
		delay(50);
		digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
	}
#else
	delay(2000);
	for (int i = 0; i < 10; i++) {
		digitalWrite(LED_STATUS, HIGH);
		delay(50);
		digitalWrite(LED_STATUS, LOW);
		delay(50);
	}
	digitalWrite(LED_STATUS, HIGH);
#endif

#ifdef TX
	/* I2C - only needed for TX */
	Wire.begin();             /* XIAO nRF52840: D4=SDA, D5=SCL */
	Wire.setClock(I2C_HZ);

	Serial.println("SDP init...");

	/* Read Product ID: enter mode -> issue read cmd (repeated START) -> read 6 words */
	if (!sdp_write_cmd(CMD_ENTER_MODE, false)) {
		Serial.println("FAIL: enter mode (0x367C)");
	} else if (!sdp_write_cmd(CMD_READ_PRODUCTID, true)) {
		Serial.println("FAIL: read product ID cmd (0xE102)");
	} else if (sdp_read_words(6, w)) {
		Serial.println("Product ID words:");
		print_hex_words(w, 6);
	} else {
		Serial.println("FAIL: product ID read/CRC");
	}

	/* Start continuous measurement */
	if (sdp_write_cmd(CMD_START_CONT, false))
		Serial.println("Continuous measurement started (0x361E)");
	else
		Serial.println("FAIL: start continuous (0x361E)");

	Serial.println("MPR121 probe init...");

	if (!cap.begin(0x5A)) {
		Serial.println("MPR121 not found, check wiring?");
		while (1);
	}
	Serial.println("MPR121 found!");

	delay(10);                /* let first sample appear */
#endif

	Serial.println("Initializing radio...");
	radio_init();
	Serial.println("Initialization complete");
}

#ifdef TX
// ==================== TX IMPLEMENTATION ====================
void loop(void)
{
	static uint32_t t_last = 0;
	uint32_t now = millis();
	
	// Send packet every 1000ms (1 second)
	if (now - t_last < 1000)
		return;
	t_last = now;
	
	SensorPacket packet;
	uint16_t words[3];
	
	// Read pressure sensor
	if (sdp_read_words(3, words)) {
		int16_t dp_raw = (int16_t)words[0];
		packet.pressure = (float)dp_raw;  // 1 Pa/LSB
	} else {
		packet.pressure = 0.0f;  // Error reading
	}
	
	// Read capacitive touch sensors
	packet.buttons = 0;
	for (uint8_t i = 0; i < 12; i++) {
		uint16_t filtered = cap.filteredData(i);
		if (filtered < 8) {  // Button pressed when < 8
			packet.buttons |= (1 << i);
		}
	}
	
	// Blink LED when sending
	digitalWrite(LED_STATUS, HIGH);
	delay(200);
	
	// Send packet
	bool ok = radio.write(&packet, sizeof(packet));
	
	digitalWrite(LED_STATUS, LOW);
	
	if (ok) {
		Serial.print("TX: P=");
		Serial.print(packet.pressure, 1);
		Serial.print(" Pa, Buttons=0x");
		Serial.println(packet.buttons, HEX);
	} else {
		Serial.println("TX failed!");
	}
}

#else
// ==================== RX IMPLEMENTATION ====================
void loop(void)
{
	static uint32_t last_check = 0;
	
	// Debug: print every 5 seconds that we're checking
	if (millis() - last_check > 5000) {
		uint8_t pipe;
		Serial.print("RX: Checking... available(pipe)=");
		Serial.println(radio.available(&pipe));
		last_check = millis();
	}
	
	if (!radio.available())
		return;
	
	SensorPacket packet;
	radio.read(&packet, sizeof(packet));
	
	// Print received data
	Serial.print("RX: Pressure=");
	Serial.print(packet.pressure, 1);
	Serial.print(" Pa, Buttons=0x");
	Serial.print(packet.buttons, HEX);
	Serial.print(" [");
	
	// Print which buttons are pressed
	bool first = true;
	for (uint8_t i = 0; i < 12; i++) {
		if (packet.buttons & (1 << i)) {
			if (!first) Serial.print(",");
			Serial.print(i);
			first = false;
		}
	}
	Serial.println("]");

	digitalWrite(LED_STATUS, HIGH);
	delay(100);
	digitalWrite(LED_STATUS, LOW);

}
#endif