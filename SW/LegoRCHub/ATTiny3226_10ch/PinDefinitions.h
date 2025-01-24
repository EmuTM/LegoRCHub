#ifndef _PINDEFINITIONS_h
#define _PINDEFINITIONS_h

#include "PWMPin.h"


#define PWM_OFF 255
#define RESOLUTION 255


volatile pwmPin_t paPins[]
{
	{0x00, 0}, //PA0 UPDI //this is in the array for practical reasons, so the indexes always match
	{digitalPinToBitMask(PIN_PA1), PWM_OFF},
	{digitalPinToBitMask(PIN_PA2), PWM_OFF},
	{digitalPinToBitMask(PIN_PA3), PWM_OFF},
	{digitalPinToBitMask(PIN_PA4), PWM_OFF},
	{digitalPinToBitMask(PIN_PA5), PWM_OFF},
	{digitalPinToBitMask(PIN_PA6), PWM_OFF},
	{digitalPinToBitMask(PIN_PA7), PWM_OFF}
};
const uint8_t paPinsCount = sizeof(paPins) / sizeof(paPins[0]);

volatile pwmPin_t pbPins[] =
{
	{digitalPinToBitMask(PIN_PB0), PWM_OFF},
	{digitalPinToBitMask(PIN_PB1), PWM_OFF},
	{digitalPinToBitMask(PIN_PB2), PWM_OFF},
	{0x00, 0}, //PB3 RX //this is in the array for practical reasons, so the indexes always match
	{digitalPinToBitMask(PIN_PB4), PWM_OFF},
	{digitalPinToBitMask(PIN_PB5), PWM_OFF}
};
const uint8_t pbPinsCount = sizeof(pbPins) / sizeof(pbPins[0]);

const uint8_t paPinsBitmask = pwmPin_t::getPinsBitmask(paPins, paPinsCount);
const uint8_t pbPinsBitmask = pwmPin_t::getPinsBitmask(pbPins, pbPinsCount);

pwmPin_t* pwmChannels[12]
{
	&paPins[4], //ch0_up
	&paPins[3], //ch0_down

	&paPins[5], //ch1_up
	&paPins[2], //ch1_down

	&paPins[6], //ch2_up
	&paPins[1], //ch2_down

	&pbPins[0], //ch3_up
	&pbPins[1], //ch3_down

	&pbPins[2], //ch4_up
	&paPins[7], //ch4_down

	&pbPins[5], //ch5_up
	&pbPins[4]  //ch5_down
};


#include <Servo_megaTinyCore.h>

#define SERVO_MIN 0
#define SERVO_MAX 180

const Servo servoChannels[]
{
	Servo(),
	Servo(),
	Servo(),
	Servo()
};

uint8_t iBusMap[IBUS_CHANNEL_COUNT]
{
	1,
	1,
	1,
	1,
	1,
	1,
	1,
	1,
	1,
	1,
	1,
	1,
	1,
	1

	//0,
	//1,
	//2,
	//3,
	//4,
	//5,
	//6,
	//7,
	//8,
	//9,
	//10,
	//11,
	//12,
	//13
};



#endif
