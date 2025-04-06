#include "IBus.h"

#ifdef DEBUG
IBus::IBus(HardwareSerial* hardwareSerial, Print* print)
{
	this->hardwareSerial = hardwareSerial;
	this->print = print;
}
#else
IBus::IBus(HardwareSerial* hardwareSerial)
{
	this->hardwareSerial = hardwareSerial;
}
#endif

void IBus::begin(void)
{
	this->hardwareSerial->begin(IBUS_BAUD_RATE, IBUS_UART_SETTINGS);

	while (!(*this->hardwareSerial));
}
bool IBus::loop(void)
{
	if (this->hardwareSerial->available())
	{
		uint8_t byte = this->hardwareSerial->read();

		//first byte must be 0x20
		if (byte == 0x20)
		{
			//store the byte and set the index for the next byte
			this->ibusPacket[0] = byte;
			this->nextByteInPacket = 1;
		}
		//store the rest of the packet
		else if (this->nextByteInPacket > 0)
		{
			this->ibusPacket[this->nextByteInPacket++] = byte;
		}

		//when a full packet is received...
		if (this->nextByteInPacket == IBUS_PACKET_SIZE)
		{
			//validate packet start
			if (this->ibusPacket[0] == 0x20 && this->ibusPacket[1] == 0x40)
			{
				//then process the packet
				if (this->validateChecksum(this->ibusPacket))
				{
					this->decodeChannels(this->ibusPacket, channels);

					return true;
				}
			}

			//reset the index for the next packet
			this->nextByteInPacket = 0;
		}
	}

	return false;
}

bool IBus::validateChecksum(uint8_t* packet)
{
	uint16_t  calculatedChecksum = 0xffff;
	for (int a = 0; a < IBUS_PACKET_SIZE - 2; a++)
	{
		calculatedChecksum -= packet[a];
	}

	//little-endian
	uint16_t receivedChecksum = (packet[IBUS_PACKET_SIZE - 2] | (packet[IBUS_PACKET_SIZE - 1] << 8));


#ifdef DEBUG
	if (calculatedChecksum != receivedChecksum)
	{
		this->print->print("Calc:");
		this->print->print(calculatedChecksum, HEX);
		this->print->print(" ");
		this->print->print("Rec:");
		this->print->print(receivedChecksum, HEX);
		this->print->println();

		this->print->print("PCK:");
		for (int i = 0; i < IBUS_PACKET_SIZE; i++)
		{
			this->print->print(this->ibusPacket[i], HEX);
			this->print->print(" ");
		}
		this->print->println();
	}
#endif

	return calculatedChecksum == receivedChecksum;
}
void IBus::decodeChannels(uint8_t* packet, uint16_t* channels)
{
	for (int a = 0; a < IBUS_CHANNEL_COUNT; a++)
	{
		//little-endian
		channels[a] = packet[2 + a * 2] | (packet[3 + a * 2] << 8);

		if (channels[a] < IBUS_MIN)
		{
			channels[a] = IBUS_MIN;
		}
		else if (channels[a] > IBUS_MAX)
		{
			channels[a] = IBUS_MAX;
		}
		//allow for zero with some tolerance
		else if (channels[a] > (IBUS_MID - IBUS_MID_TOLERANCE)
			&& channels[a] < (IBUS_MID + IBUS_MID_TOLERANCE))
		{
			channels[a] = IBUS_MID;
		}

	}

#ifdef DEBUG
		for (int a = 0; a < IBUS_CHANNEL_COUNT; a++)
		{
			this->print->print(channels[a]);
			this->print->print(";");
		}
		this->print->println();
#endif
}
