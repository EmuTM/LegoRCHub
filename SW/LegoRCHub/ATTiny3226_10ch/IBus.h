#ifndef _IBUS_h
#define _IBUS_h

#define IBUS_BAUD_RATE 115200
#define IBUS_UART_SETTINGS SERIAL_8N1
#define IBUS_PACKET_SIZE 32
#define IBUS_CHANNEL_COUNT 14
#define IBUS_MIN 1000
#define IBUS_MID 1500
#define IBUS_MID_TOLERANCE 5
#define IBUS_MAX 2000

#include "Arduino.h"

class IBus
{
public:
	volatile uint16_t channels[IBUS_CHANNEL_COUNT];

#ifdef DEBUG
	IBus(HardwareSerial* hardwareSerial, Print* print);
#else
	IBus(HardwareSerial* hardwareSerial);
#endif
	void begin(void);
	bool loop(void);

private:
	HardwareSerial* hardwareSerial;
	uint8_t ibusPacket[IBUS_PACKET_SIZE];
	uint8_t nextByteInPacket = 0;

	bool validateChecksum(uint8_t* packet);
	void decodeChannels(uint8_t* packet, uint16_t* channels);

#ifdef DEBUG
	Print* print;
#endif
};

#endif

