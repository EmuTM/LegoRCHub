#ifndef _PINDEFINITIONS_h
#define _PINDEFINITIONS_h


typedef struct PWMPin
{
	uint8_t bitMask;
	uint8_t dutyCycle;
	PORT_t* port;

	static inline uint8_t getPinsBitmask(PWMPin* pins, const uint8_t pinCount)
	{
		uint8_t val = 0;

		for (uint8_t a = 0; a < pinCount; a++)
		{
			val |= pins[a].bitMask;
		}

		return val;
	}
} pwmPin_t;


//use this for sinking current pwm pins
//#define PWM_INVERTED

//IMPORTANT NOTE: this resolution is interdependant on timer A settings and should not be changed per se
#define RESOLUTION 31

//max pwm = slow decay (brake); 0 = fast decay (coast)
#define PWM_OFF 0

//IMPORTANT NOTE: maps pwm channels to pins that drive H bridges
//THIS IS PCB DESIGN DEPENDANT!
pwmPin_t pwmChannels[6 * 2]
{
	{digitalPinToBitMask(PIN_PA5), PWM_OFF, &PORTA}, //ch0_up (ch1_2)
	{digitalPinToBitMask(PIN_PA4), PWM_OFF, &PORTA}, //ch0_down (ch1_3)

	{digitalPinToBitMask(PIN_PB5), PWM_OFF, &PORTB}, //ch1_up (ch2_2)
	{digitalPinToBitMask(PIN_PB4), PWM_OFF, &PORTB}, //ch1_down (ch2_3)

	{digitalPinToBitMask(PIN_PA7), PWM_OFF, &PORTA}, //ch2_up (ch3_2)
	{digitalPinToBitMask(PIN_PA6), PWM_OFF, &PORTA}, //ch2_down (ch3_3)

	{digitalPinToBitMask(PIN_PA3), PWM_OFF, &PORTA}, //ch3_up (ch4_2)
	{digitalPinToBitMask(PIN_PA2), PWM_OFF, &PORTA}, //ch3_down (ch4_3)

	{digitalPinToBitMask(PIN_PB1), PWM_OFF, &PORTB}, //ch4_up (ch5_2)
	{digitalPinToBitMask(PIN_PB2), PWM_OFF, &PORTB}, //ch4_down (ch5_3)

	{digitalPinToBitMask(PIN_PA1), PWM_OFF, &PORTA}, //ch5_up (ch6_2)
	{digitalPinToBitMask(PIN_PB0), PWM_OFF, &PORTB}  //ch5_down (ch6_3)
};
const uint8_t pwmChannelsCount = sizeof(pwmChannels) / sizeof(pwmChannels[0]);

#include <Servo_megaTinyCore.h>
typedef struct ServoPin
{
	uint8_t pin;
	Servo servo;

	static inline void attachAll(ServoPin* pins, const uint8_t pinCount)
	{
		for (int a = 0; a < pinCount; a++)
		{
			pins[a].servo.attach(pins[a].pin);
		}
	}

} servoPin_t;

#define SERVO_MIN 0
#define SERVO_MAX 180

//IMPORTANT NOTE: maps servo channels to pins that drive servos
//THIS IS PCB DESIGN DEPENDANT!
#ifdef DEBUG
const servoPin_t servoChannels[3]
{
	//{PIN_PC3, Servo()}, //pin reserved for debug!
	{PIN_PC0, Servo()}, //ch8
	{PIN_PC2, Servo()}, //ch9
	{PIN_PC1, Servo()} //ch10
};
#else
const servoPin_t servoChannels[4]
{
	{PIN_PC3, Servo()}, //ch7
	{PIN_PC0, Servo()}, //ch8
	{PIN_PC2, Servo()}, //ch9
	{PIN_PC1, Servo()} //ch10
};
#endif
const uint8_t servoChannelsCount = sizeof(servoChannels) / sizeof(servoChannels[0]);

#include "IBus.h"

//USE THIS TO MAP IBUS SIGNALS TO OUTPUT CHANNELS
//number in here define the pwm/servo channel (channels 0-5 are pwm; channels 6-9 are servos) with respect to iBus channel
uint8_t iBusMap[IBUS_CHANNEL_COUNT]
{
	0,
	1,
	2,
	3,
	4,
	5,
	6,
	7,
	8,
	9,


	10, //do not use - no pin available
	11, //do not use - no pin available
	12, //do not use - no pin available
	13  //do not use - no pin available
};

#endif
