#ifndef _PWMPIN_h
#define _PWMPIN_h

typedef struct PWMPin
{
	uint8_t bitMask;
	uint8_t dutyCycle;

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

#endif
