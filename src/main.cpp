#define TX  // Comment this out for RX device

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
	int16_t pressure;
	uint8_t note;
};

/* --------- Config --------- */
#define SERIAL_BAUD 115200

//#define AWAIT_SERIAL
#define DEBUG

// Debug macros - only compile when DEBUG is defined
#ifdef DEBUG
	#define dbg(x) Serial.print(x)
	#define dbgln(x) Serial.println(x)
#else
	#define dbg(x)
	#define dbgln(x)
#endif

//#ifndef _BV
//#define _BV(bit) (1 << (bit)) 
//#endif


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
#define BUTTONS_TO_NOTE(b11, b10, b9, b8, b7, b6, b5, b4, b3, b2, b1, b0) \
	((b11 << 11) | (b10 << 10) | (b9 << 9) | (b8 << 8) | (b7 << 7) | (b6 << 6) | (b5 << 5) | (b4 << 4) | (b3 << 3) | (b2 << 2) | (b1 << 1) | (b0))
#define NOTE_BUTTONS_A5  (BUTTONS_TO_NOTE(1,1,1,1,1,1,1,1,1,1,1,1))
#define NOTE_BUTTONS_Bb5 (BUTTONS_TO_NOTE(1,1,1,1,1,1,1,1,0,1,1,1))
#define NOTE_BUTTONS_B5  (BUTTONS_TO_NOTE(1,1,1,0,1,1,1,1,1,1,1,1))
#define NOTE_BUTTONS_C6  (BUTTONS_TO_NOTE(1,1,1,0,1,1,1,1,0,1,1,1))
#define NOTE_BUTTONS_Db6 (BUTTONS_TO_NOTE(1,1,1,0,1,1,1,1,1,1,1,0))
#define NOTE_BUTTONS_D6  (BUTTONS_TO_NOTE(1,1,1,0,1,1,1,1,0,1,1,0))
#define NOTE_BUTTONS_Eb6 (BUTTONS_TO_NOTE(1,1,1,0,1,1,1,1,1,1,0,0))
#define NOTE_BUTTONS_E6  (BUTTONS_TO_NOTE(1,1,1,0,1,1,1,1,0,1,0,0))
#define NOTE_BUTTONS_F6  (BUTTONS_TO_NOTE(1,1,1,0,1,1,1,1,0,0,0,0))
#define NOTE_BUTTONS_Gb6 (BUTTONS_TO_NOTE(1,1,1,0,1,1,1,0,0,0,1,0))
#define NOTE_BUTTONS_G6  (BUTTONS_TO_NOTE(1,1,1,0,1,1,1,0,0,0,0,0))
#define NOTE_BUTTONS_Ab6 (BUTTONS_TO_NOTE(1,1,1,0,1,0,1,0,0,0,1,0))
#define NOTE_BUTTONS_A6  (BUTTONS_TO_NOTE(1,1,1,0,1,0,1,0,0,0,0,0))
#define NOTE_BUTTONS_Bb6 (BUTTONS_TO_NOTE(1,1,1,0,0,0,1,0,0,0,1,0))
#define NOTE_BUTTONS_B6  (BUTTONS_TO_NOTE(1,1,1,0,0,0,1,0,0,0,0,0))
#define NOTE_BUTTONS_C7  (BUTTONS_TO_NOTE(1,1,0,0,0,0,1,0,0,0,0,0))
#define NOTE_BUTTONS_Db7 (BUTTONS_TO_NOTE(0,1,0,0,0,0,1,0,0,0,1,0))
#define NOTE_BUTTONS_D7  (BUTTONS_TO_NOTE(0,1,0,0,0,0,1,0,0,0,0,0))
#define NOTE_BUTTONS_Eb7 (BUTTONS_TO_NOTE(0,0,0,0,0,0,1,0,0,0,1,0))
#define NOTE_BUTTONS_E7  (BUTTONS_TO_NOTE(0,0,0,0,0,0,1,0,0,0,0,0))
#define NOTE_BUTTONS_F7  (BUTTONS_TO_NOTE(0,0,0,0,0,0,0,0,0,0,0,0))

enum NoteMidi {
	NOTE_MIDI_INVAL = 0,
	NOTE_MIDI_A5   = 69,
	NOTE_MIDI_Bb5  = 70,
	NOTE_MIDI_B5   = 71,
	NOTE_MIDI_C6   = 72,
	NOTE_MIDI_Db6  = 73,
	NOTE_MIDI_D6   = 74,
	NOTE_MIDI_Eb6  = 75,
	NOTE_MIDI_E6   = 76,
	NOTE_MIDI_F6   = 77,
	NOTE_MIDI_Gb6  = 78,
	NOTE_MIDI_G6   = 79,
	NOTE_MIDI_Ab6  = 80,
	NOTE_MIDI_A6   = 81,
	NOTE_MIDI_Bb6  = 82,
	NOTE_MIDI_B6   = 83,
	NOTE_MIDI_C7   = 84,
	NOTE_MIDI_Db7  = 85,
	NOTE_MIDI_D7   = 86,
	NOTE_MIDI_Eb7  = 87,
	NOTE_MIDI_E7   = 88,
	NOTE_MIDI_F7   = 89
};

#define EON_COUNT_MAX 5


#ifdef TX

// You can have up to 4 on one i2c bus but one is enough for testing!
Adafruit_MPR121 cap = Adafruit_MPR121();

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

uint8_t button_to_note(uint16_t buttons)
{
	static uint8_t last_good_note = NOTE_MIDI_INVAL;
	uint8_t note;

	switch (buttons) {
		case NOTE_BUTTONS_A5:  note = NOTE_MIDI_A5;
		case NOTE_BUTTONS_Bb5: note = NOTE_MIDI_Bb5;
		case NOTE_BUTTONS_B5:  note = NOTE_MIDI_B5;
		case NOTE_BUTTONS_C6:  note = NOTE_MIDI_C6;
		case NOTE_BUTTONS_Db6: note = NOTE_MIDI_Db6;
		case NOTE_BUTTONS_D6:  note = NOTE_MIDI_D6;
		case NOTE_BUTTONS_Eb6: note = NOTE_MIDI_Eb6;
		case NOTE_BUTTONS_E6:  note = NOTE_MIDI_E6;
		case NOTE_BUTTONS_F6:  note = NOTE_MIDI_F6;
		case NOTE_BUTTONS_Gb6: note = NOTE_MIDI_Gb6;
		case NOTE_BUTTONS_G6:  note = NOTE_MIDI_G6;
		case NOTE_BUTTONS_Ab6: note = NOTE_MIDI_Ab6;
		case NOTE_BUTTONS_A6:  note = NOTE_MIDI_A6;
		case NOTE_BUTTONS_Bb6: note = NOTE_MIDI_Bb6;
		case NOTE_BUTTONS_B6:  note = NOTE_MIDI_B6;
		case NOTE_BUTTONS_C7:  note = NOTE_MIDI_C7;
		case NOTE_BUTTONS_Db7: note = NOTE_MIDI_Db7;
		case NOTE_BUTTONS_D7:  note = NOTE_MIDI_D7;
		case NOTE_BUTTONS_Eb7: note = NOTE_MIDI_Eb7;
		case NOTE_BUTTONS_E7:  note = NOTE_MIDI_E7;
		case NOTE_BUTTONS_F7:  note = NOTE_MIDI_F7;
		default:               note = NOTE_MIDI_INVAL;
	}

	if (note == NOTE_MIDI_INVAL) {
		note = last_good_note;
	} else {
		last_good_note = note;
	}

	return note;
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
	// When pressure is off - send empty packet 5 times to ensure end of note
	static uint8_t eon_count = EON_COUNT_MAX;
	SensorPacket packet;
	uint16_t buttons;
	
	packet.pressure = read_pressure();
	if (packet.pressure == 0) {
		if (eon_count > 0) {
			eon_count--;
			packet.note = NOTE_MIDI_INVAL;
		} else {
			return;
		}
	} else {
		eon_count = EON_COUNT_MAX;
		buttons = read_buttons();
		packet.note = button_to_note(buttons);
	}
	
	radio.write(&packet, sizeof(packet));
	
	// Debug output
	dbg("TX: P=");
	dbg(packet.pressure);
	dbg(" Note=");
	if (packet.note == NOTE_MIDI_INVAL) {
		dbgln("END");
	} else {
		dbgln(packet.note);
	}

	delay(100);
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
	Serial.print(" Note=");
	
	if (packet.note == NOTE_MIDI_INVAL) {
		Serial.print("END");
	} else {
		Serial.print(packet.note);
	}
	
	Serial.println();
	
	// Brief LED blink on packet received
	digitalWrite(LED_STATUS, HIGH);
	delay(50);
	digitalWrite(LED_STATUS, LOW);
}
#endif