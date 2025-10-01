#if 1

#define TX



#ifdef TX

#include <Arduino.h>
#include "nrf_to_nrf.h"

#define LED_STATUS	D6

static nrf_to_nrf	radio;
static uint8_t		address[][6] = { "1Node", "2Node" };
static bool		radio_number = 0;	/* 0 → TX to "1Node" */
static float		payload = 0.0f;

void setup(void)
{
	Serial.begin(115200);
	delay(3000);	

	pinMode(LED_STATUS, OUTPUT);
	delay(1000);	
	digitalWrite(LED_STATUS, LOW);

	for (int i = 0; i < 10; i++) {
		digitalWrite(LED_STATUS, HIGH);
		delay(50);
		digitalWrite(LED_STATUS, LOW);
		delay(50);
	}
	digitalWrite(LED_STATUS, HIGH);

	if (!radio.begin()) {
		Serial.println("radio hardware is not responding!!");
		for (;;)
			;
	}

	Serial.println("RF24/examples/GettingStarted");
	Serial.print("radioNumber = ");
	Serial.println((int)radio_number);

	radio.begin();
	radio.setPALevel(NRF_PA_LOW);		/* same as example */
	radio.setDataRate(NRF_2MBPS);
	radio.setChannel(80);
	radio.setCRCLength(NRF_CRC_DISABLED);	/* disable CRC */
	radio.setAutoAck(true);		/* enable ACK with payloads */
	radio.enableAckPayload();
	radio.enableDynamicPayloads();		/* enable dynamic payloads */

	radio.openWritingPipe(address[0]);     /* TX writes to "1Node" */
	radio.stopListening();
//	radio.startListening();
}

void loop(void)
{
	float ack_payload = 0.0f;

	digitalWrite(LED_STATUS, HIGH);

	unsigned long t0 = micros();
	bool ok = radio.write(&payload, sizeof(float));	/* waits for ACK */
	
	// Check for ACK payload after successful transmission
	if (ok && radio.available()) {
		radio.read(&ack_payload, sizeof(ack_payload));
	}
	unsigned long t1 = micros();

	if (ok) {
		Serial.print("Transmission successful! Time to transmit = ");
		Serial.print(t1 - t0);
		Serial.print(" us. Sent: ");
		Serial.print(payload, 2);
		Serial.print(" Received: ");
		Serial.println(ack_payload, 2);
		payload += 0.01f;

		delay(500);
		digitalWrite(LED_STATUS, LOW);
	} else {
		Serial.println("Transmission failed or timed out");
		delay(100);
		digitalWrite(LED_STATUS, LOW);
		delay(100);
		digitalWrite(LED_STATUS, HIGH);
		delay(100);
		digitalWrite(LED_STATUS, LOW);
	}

	delay(1000);	/* same pacing as example */
}

#else
#include <Arduino.h>
#include "nrf_to_nrf.h"

#define LED_STATUS	LED_BUILTIN

static nrf_to_nrf	radio;
static uint8_t		address[][6] = { "1Node", "2Node" };
static bool		radio_number = 1;	/* 1 → listens for "1Node" on pipe 1 */
static float		payload = 0.0f;

void setup(void)
{
	Serial.begin(115200);
	delay(2000);	

	pinMode(LED_STATUS, OUTPUT);
	delay(1000);	
	digitalWrite(LED_STATUS, LOW);

	for (int i = 0; i < 10; i++) {
		digitalWrite(LED_STATUS, HIGH);
		delay(50);
		digitalWrite(LED_STATUS, LOW);
		delay(50);
	}
	digitalWrite(LED_STATUS, HIGH);

	if (!radio.begin()) {
		Serial.println("radio hardware is not responding!!");
		for (;;)
			;
	}

	Serial.println("RF24/examples/GettingStarted");
	Serial.print("radioNumber = ");
	Serial.println((int)radio_number);

	radio.begin();
	radio.setPALevel(NRF_PA_LOW);		/* same as example */
	radio.setDataRate(NRF_2MBPS);
	radio.setChannel(80);
	radio.setCRCLength(NRF_CRC_DISABLED);	/* disable CRC */
	radio.setAutoAck(true);		/* enable ACK with payloads */
	radio.enableAckPayload();
	radio.enableDynamicPayloads();		/* enable dynamic payloads */

	radio.openReadingPipe(1, address[0]); /* RX listens on "1Node" */
	float reply_val = 108.0f;
	radio.writeAckPayload(1, &reply_val, sizeof(reply_val));
	radio.startListening();				/* RX mode */
}

void loop(void)
{
	uint8_t pipe;
	float ack_payload;

	if (!radio.available(&pipe))
		return;

	radio.read(&payload, sizeof(payload));
	ack_payload = payload + 88.0f;
	
	// Pre-load the NEXT ACK payload for the next transmission
	float next_ack = ack_payload + 1.0f;  // or any other value
	radio.writeAckPayload(pipe, &next_ack, sizeof(next_ack));

	digitalWrite(LED_STATUS, HIGH);
	delay(300);
	digitalWrite(LED_STATUS, LOW);

	Serial.print("Received ");
	Serial.print(payload, 2);
	Serial.print(" on pipe ");
	Serial.print(pipe);
	Serial.print(", sent ACK: ");
	Serial.println(ack_payload, 2);
}
#endif



///*
// * See documentation at https://nRF24.github.io/RF24
// * See License information at root directory of this library
// * Author: Brendan Doherty (2bndy5)
// */
//
///**
// * A simple example of sending data from 1 nRF24L01 transceiver to another.
// *
// * This example was written to be used on 2 devices acting as "nodes".
// * Use the Serial Monitor to change each node's behavior.
// */
//#include "nrf_to_nrf.h"
//
////#define LED_STATUS LED_BUILTIN
//#define LED_STATUS D6
//
//
//// instantiate an object for the nRF24L01 transceiver
//nrf_to_nrf radio;  // using pin 7 for the CE pin, and pin 8 for the CSN pin
//
//// Let these addresses be used for the pair
//uint8_t address[][6] = { "1Node", "2Node" };
//// It is very helpful to think of an address as a path instead of as
//// an identifying device destination
//
//// to use different addresses on a pair of radios, we need a variable to
//// uniquely identify which address this radio will use to transmit
////bool radioNumber = 1;  // 0 uses address[0] to transmit, 1 uses address[1] to transmit
//
//// Used to control whether this node is sending or receiving
////bool role = true;  // true = TX role, false = RX role
//bool role = false;  // true = TX role, false = RX role
//
//// For this example, we'll be using a payload containing
//// a single float number that will be incremented
//// on every successful transmission
//float payload = 0.0;
//
//void setup() {
//
//  pinMode(LED_STATUS, OUTPUT);
//  digitalWrite(LED_STATUS, HIGH);
//
//  Serial.begin(115200);
//  while (!Serial) {
//    // some boards need to wait to ensure access to serial over USB
//  }
//  delay(3000);
//
//  // initialize the transceiver on the SPI bus
//  if (!radio.begin()) {
//    Serial.println(F("radio hardware is not responding!!"));
//    while (1) {}  // hold in infinite loop
//  }
//
//  // print example's introductory prompt
//  Serial.println(F("RF24/examples/GettingStarted"));
//
//  // To set the radioNumber via the Serial monitor on startup
////  Serial.println(F("Which radio is this? Enter '0' or '1'. Defaults to '0'"));
////  while (!Serial.available()) {
////    // wait for user input
////  }
////  char input = Serial.parseInt();
////  radioNumber = input == 1;
//  bool radioNumber = 1;
//  Serial.print(F("radioNumber = "));
//  Serial.println((int)radioNumber);
//
//  // role variable is hardcoded to RX behavior, inform the user of this
////  Serial.println(F("*** PRESS 'T' to begin transmitting to the other node"));
//
//  // Set the PA Level low to try preventing power supply related problems
//  // because these examples are likely run with nodes in close proximity to
//  // each other.
//  radio.setPALevel(NRF_PA_LOW);  // RF24_PA_MAX is default.
//
//  // save on transmission time by setting the radio to only transmit the
//  // number of bytes we need to transmit a float
//  radio.setPayloadSize(sizeof(payload));  // float datatype occupies 4 bytes
//
//  // set the TX address of the RX node into the TX pipe
//  radio.openWritingPipe(address[radioNumber]);  // always uses pipe 0
//
//  // set the RX address of the TX node into a RX pipe
//  radio.openReadingPipe(1, address[!radioNumber]);  // using pipe 1
//
//  // additional setup specific to the node's role
//  if (role) {
//    radio.stopListening();  // put radio in TX mode
//  } else {
//    radio.startListening();  // put radio in RX mode
//  }
//
//  // For debugging info
//  // printf_begin();             // needed only once for printing details
//  // radio.printDetails();       // (smaller) function that prints raw register values
//  // radio.printPrettyDetails(); // (larger) function that prints human readable data
//
//}  // setup
//
//void loop() {
//
//  if (role) {
//    // This device is a TX node
//
//    unsigned long start_timer = micros();                // start the timer
//    bool report = radio.write(&payload, sizeof(float));  // transmit & save the report
//    unsigned long end_timer = micros();                  // end the timer
//
//    if (report) {
//      Serial.print(F("Transmission successful! "));  // payload was delivered
//      Serial.print(F("Time to transmit = "));
//      Serial.print(end_timer - start_timer);  // print the timer result
//      Serial.print(F(" us. Sent: "));
//      Serial.println(payload);  // print payload sent
//      payload += 0.01;          // increment float payload
//    } else {
//      Serial.println(F("Transmission failed or timed out"));  // payload was not delivered
//	  digitalWrite(LED_STATUS, HIGH);
//      delay(100);
//	  digitalWrite(LED_STATUS, LOW);
//    }
//
//    // to make this example readable in the serial monitor
//    delay(1000);  // slow transmissions down by 1 second
//
//  } else {
//    // This device is a RX node
//
//    uint8_t pipe;
//    if (radio.available(&pipe)) {              // is there a payload? get the pipe number that recieved it
//      uint8_t bytes = radio.getPayloadSize();  // get the size of the payload
//      radio.read(&payload, bytes);             // fetch payload from FIFO
//      Serial.print(F("Received "));
//      Serial.print(bytes);  // print the size of the payload
//      Serial.print(F(" bytes on pipe "));
//      Serial.print(pipe);  // print the pipe number
//      Serial.print(F(": "));
//      Serial.println(payload);  // print the payload's value
//    }
//  }  // role
//
//  if (Serial.available()) {
//    // change the role via the serial monitor
//
//    char c = toupper(Serial.read());
//    if (c == 'T' && !role) {
//      // Become the TX node
//
//      role = true;
//      Serial.println(F("*** CHANGING TO TRANSMIT ROLE -- PRESS 'R' TO SWITCH BACK"));
//      radio.stopListening();
//
//    } else if (c == 'R' && role) {
//      // Become the RX node
//
//      role = false;
//      Serial.println(F("*** CHANGING TO RECEIVE ROLE -- PRESS 'T' TO SWITCH BACK"));
//      radio.startListening();
//    }
//  }
//
//}  // loop
//

#else /////////////////////////////////////////////////////////////////////////////////////////


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


void esb_tx_init()
{
	radio.begin();
	radio.stopListening();
	radio.setPALevel(NRF_PA_MAX);
	radio.setDataRate(NRF_2MBPS);
	radio.setChannel(80);                 // ~2480 MHz
	radio.setAutoAck(false);              // one-way, no ACK
	radio.setRetries(0,0);
	radio.setAddressWidth(5);
	uint8_t addr[5] = {'E','S','B','0','0'};
	radio.setCRCLength(NRF_CRC_16);
	radio.disableDynamicPayloads();
	radio.setPayloadSize(8);
	radio.openWritingPipe(addr);

	radio.printDetails();
}

void esb_send8(uint8_t *d)
{
	radio.writeFast(d, 8, true);                // fire-and-forget, lowest latency
}

#define LED_STATUS D6

void setup(void)
{
	int i;
	uint16_t w[6];

	pinMode(LED_STATUS, OUTPUT);
	digitalWrite(LED_STATUS, HIGH);

	for (int i = 0; i < 10; i++) {
		digitalWrite(LED_STATUS, HIGH);
		delay(50);
		digitalWrite(LED_STATUS, LOW);
		delay(50);
	}
	digitalWrite(LED_STATUS, HIGH);

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

	Serial.println("Initializing ESB TX...");
	esb_tx_init();
	Serial.println("Initialization complete");

	delay(10);                /* let first sample appear */
}

void loop(void)
{
	static uint32_t t_last;
	uint32_t now = millis();
	uint16_t words[3];
	int16_t dp_raw;
	float dp_pa;

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
	Serial.print("Filt: ");
	for (uint8_t i = 0; i < 12; i++) {
		Serial.print(cap.filteredData(i));
		Serial.print(" ");
	}
//	Serial.print("Base: ");
//	for (uint8_t i = 0; i < 12; i++) {
//		Serial.print(cap.baselineData(i));
//		Serial.print(" ");
//	}
//	Serial.println();
  

	if (now - t_last < READ_PERIOD_MS)
		return;
	t_last = now;

	/* Typical frame: dp, aux (temp/scale), status */
	if (!sdp_read_words(3, words))
		return;

	dp_raw = (int16_t)words[0];
	/* Many modes are 1 Pa/LSB; verify in datasheet for your command. */
	dp_pa = (float)dp_raw;

//	if (dp_pa > PRINT_THRESHOLD_PA) {
		Serial.print(dp_pa);
//	}

	uint8_t buf[8] = { 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07 };
	buf[0] = dp_pa / 32768.0f * 127.0f;   // scale to int8 range
	dp_raw = (int16_t)words[0];

	Serial.print(" -> ");
	Serial.print(buf[0]);
	
	Serial.println();


	esb_send8(buf);

	Serial.println();
	Serial.flush();

	delay(50);
}
#endif