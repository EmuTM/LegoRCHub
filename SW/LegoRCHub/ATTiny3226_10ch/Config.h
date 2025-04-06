#ifndef _CONFIG_h
#define _CONFIG_h


typedef struct PWMPin
{
	PORT_t* port;
	uint8_t bitMask;
	uint8_t dutyCycle;
} pwmPin_t;


//IMPORTANT NOTE: this resolution is interdependant on timer A settings and should not be changed per se
#define PWM_RESOLUTION 31

#define MIN_PWM 7
#define MAX_PWM 32 //+1 for constant on

//use this for braking effect; both pins 1 when idle
//#define PWM_SLOW_DECAY


//IMPORTANT NOTE: maps pwm channels to pins that drive H bridges
//THIS IS PCB DESIGN DEPENDANT!
pwmPin_t pwmPins[6 * 2]
{
	{&PORTA, digitalPinToBitMask(PIN_PA5)}, //ch0_up (ch1_2)
	{&PORTA, digitalPinToBitMask(PIN_PA4)}, //ch0_down (ch1_3)

	{&PORTB, digitalPinToBitMask(PIN_PB5)}, //ch1_up (ch2_2)
	{&PORTB, digitalPinToBitMask(PIN_PB4)}, //ch1_down (ch2_3)

	{&PORTA, digitalPinToBitMask(PIN_PA7)}, //ch2_up (ch3_2)
	{&PORTA, digitalPinToBitMask(PIN_PA6)}, //ch2_down (ch3_3)

	{&PORTA, digitalPinToBitMask(PIN_PA3)}, //ch3_up (ch4_2)
	{&PORTA, digitalPinToBitMask(PIN_PA2)}, //ch3_down (ch4_3)

	{&PORTB, digitalPinToBitMask(PIN_PB1)}, //ch4_up (ch5_2)
	{&PORTB, digitalPinToBitMask(PIN_PB2)}, //ch4_down (ch5_3)

	{&PORTA, digitalPinToBitMask(PIN_PA1)}, //ch5_up (ch6_2)
	{&PORTB, digitalPinToBitMask(PIN_PB0)}  //ch5_down (ch6_3)
};
const uint8_t pwmPinsCount = sizeof(pwmPins) / sizeof(pwmPins[0]);

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
const servoPin_t servoPins[3]
{
	//{PIN_PC3, Servo()}, //pin reserved for debug!
	{PIN_PC0, Servo()}, //ch8
	{PIN_PC2, Servo()}, //ch9
	{PIN_PC1, Servo()} //ch10
};
#else
const servoPin_t servoPins[4]
{
	{PIN_PC3, Servo()}, //ch7
	{PIN_PC0, Servo()}, //ch8
	{PIN_PC2, Servo()}, //ch9
	{PIN_PC1, Servo()} //ch10
};
#endif
const uint8_t servoPinsCount = sizeof(servoPins) / sizeof(servoPins[0]);

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
