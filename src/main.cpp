#include <Arduino.h>
#include <Wire.h>
#include "Adafruit_TinyUSB.h"
#include "Adafruit_MPR121.h"

#include "nrf.h"
#include "nrf_gpio.h"
#include "nrfx_clock.h"
#include <nrf_to_nrf.h>

nrf_to_nrf radio;

// Packet structure for TX->RX communication
struct SensorPacket {
	int16_t pressure;      // Pressure value in Pa
	uint16_t buttons;    // Bitmask of pressed buttons (12 bits used)
};

/* --------- Config --------- */
#define SERIAL_BAUD 115200

//#define AWAIT_SERIAL

//#ifndef _BV
//#define _BV(bit) (1 << (bit)) 
//#endif

#define TX  // Comment this out for RX device


#ifdef TX
// TX-only: I2C sensors
#define SDP_ADDR 0x25    /* 7-bit I2C address */
#define I2C_HZ 400000

#define CMD_ENTER_MODE 0x367C
#define CMD_READ_PRODUCTID 0xE102
#define CMD_START_CONT 0x361E

#define PRESSURE_THRESHOLD 1000

// Button wiring index
uint8_t button_idx_mapping[] = {
	11, 10, 9, 7, 6, 8, 5, 3, 4, 2, 0, 1
};
// Measured stable sensor values at normal conditions
uint8_t button_idle[] = {
	14, 15, 17, 16, 16, 16, 16, 16, 15, 16, 14, 14
};
#define BUTTON_IDLE_TOLERANCE 2

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

/* Read pressure in Pa
 * Returns 0 if below threshold, -1 on error
 */
static int16_t read_pressure()
{
	int16_t pressure;
	uint16_t words[3];

	if (sdp_read_words(3, words)) {
		pressure = words[0];
		if (pressure < PRESSURE_THRESHOLD)
			pressure = 0;
	} else {
		pressure = -1;
	}

	return pressure;
}

static uint16_t read_buttons()
{
	uint16_t buttons = 0;
	uint16_t state;

	for (uint8_t i = 0; i < 12; i++) {
		state = cap.filteredData(button_idx_mapping[i]);
		if ((state < button_idle[i] - BUTTON_IDLE_TOLERANCE) ||
			(state > button_idle[i] + BUTTON_IDLE_TOLERANCE))
			buttons |= 1 << i;
	}

	return buttons;
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
	radio.enableDynamicPayloads();
	
#ifdef TX
	radio.openWritingPipe(addr);
	radio.stopListening();
#else
	radio.openReadingPipe(1, addr);
	radio.startListening();
#endif
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
	Wire.begin();
	Wire.setClock(I2C_HZ);

	/* Initialize SDP pressure sensor */
	if (!sdp_write_cmd(CMD_ENTER_MODE, false) ||
	    !sdp_write_cmd(CMD_READ_PRODUCTID, true) ||
	    !sdp_read_words(6, w)) {
		Serial.println("SDP sensor init failed!");
	}
	
	if (!sdp_write_cmd(CMD_START_CONT, false)) {
		Serial.println("SDP continuous mode failed!");
	}

	/* Initialize MPR121 capacitive touch sensor */
	if (!cap.begin(0x5A)) {
		Serial.println("MPR121 not found!");
		while (1);
	}

	delay(10);
#endif

	radio_init();
	Serial.println("Ready");
}

#ifdef TX
// ==================== TX IMPLEMENTATION ====================
void loop(void)
{
	static uint32_t t_last = 0;
	uint32_t now = millis();
	
	// Send packets every 50ms
	if (now - t_last < 50)
		return;
	t_last = now;
	
	SensorPacket packet;
	packet.pressure = read_pressure();
	if (packet.pressure == 0)
		return;
	packet.buttons = read_buttons();
	
	// Send packet
	bool ok = radio.write(&packet, sizeof(packet));
}

#else
// ==================== RX IMPLEMENTATION ====================
void loop(void)
{
	if (!radio.available())
		return;
	
	SensorPacket packet;
	radio.read(&packet, sizeof(packet));
	
	// Print received data
	Serial.print("P=");
	Serial.print(packet.pressure);
	Serial.print(" B=0x");
	Serial.print(packet.buttons, HEX);
	
	// Print button list
	Serial.print(" [");
	bool first = true;
	for (uint8_t i = 0; i < 12; i++) {
		if (packet.buttons & (1 << i)) {
			if (!first) Serial.print(",");
			Serial.print(i);
			first = false;
		}
	}
	Serial.println("]");
	
	// Brief LED blink on packet received
	digitalWrite(LED_STATUS, HIGH);
	delayMicroseconds(100);
	digitalWrite(LED_STATUS, LOW);
}
#endif