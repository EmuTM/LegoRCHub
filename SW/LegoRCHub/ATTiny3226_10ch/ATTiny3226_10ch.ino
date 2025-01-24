//#define DEBUG



#ifdef DEBUG
#include <SoftwareSerial.h>
SoftwareSerial mySerial(-1 /*rx not supported*/, PIN_PA1 /*any pin except UPDI*/);
#endif


#include "IBus.h"
#ifdef DEBUG
IBus iBus(&Serial, &mySerial);
#else
IBus iBus(&Serial/*, &mySerial*/);
#endif


#include "PWMPin.h"
#include "PinDefinitions.h"
#include <Servo_megaTinyCore.h>


void setup()
{
#ifdef DEBUG
	mySerial.begin(9600);
	mySerial.println("DEBUG");
#endif

	noInterrupts();

	iBus.begin();

	//disable TX in the USART peripheral
	USART0.CTRLB &= ~USART_TXEN_bm;

	//configure the following pins as output
	PORTA.DIRSET = paPinsBitmask;
	PORTB.DIRSET = pbPinsBitmask;

	//set timer A
	{
		//timer_frequency_Hz = processor_clock_frequency_Hz / prescaler_value
		//interrupt_frequency_Hz = timer_frequency_Hz / (TCA0.SINGLE.PER[timer period] + 1)
		//pwm_frequency_Hz = timer_frequency_Hz / resolution

		//for a pwm resolution of 255, this gives around 150 Hz pwm signals
		TCA0.SINGLE.CTRLA = TCA_SINGLE_CLKSEL_enum::TCA_SINGLE_CLKSEL_DIV8_gc | TCA_SINGLE_ENABLE_bm;
		TCA0.SINGLE.PER = 31;
		TCA0.SINGLE.INTCTRL = TCA_SINGLE_OVF_bm; //enable overflow interrupt
	}

	//set servos
	{
		servoChannels[0].attach(PIN_PC0);
		servoChannels[1].attach(PIN_PC1);
		servoChannels[2].attach(PIN_PC2);
		servoChannels[3].attach(PIN_PC3);
	}

	interrupts();
}

void loop()
{
	if (iBus.loop())
	{

#ifdef DEBUG
		for (int a = 0; a < IBUS_CHANNEL_COUNT; a++)
		{
			mySerial.print(iBus.channels[iBusMap[a]]);
			mySerial.print(";");
		}
		mySerial.println();
#endif

		//first 12 (6*2) channels for pwm pins
		for (int a = 0; a < 6; a++)
		{
			uint16_t val = iBus.channels[iBusMap[a]];

			if (val > IBUS_MID)
			{
#ifdef DEBUG
				mySerial.print("CH ");
				mySerial.print(a);
				mySerial.print(" UP:");
				mySerial.println(map(val, IBUS_MID, IBUS_MAX, 0, RESOLUTION));
#endif

				pwmChannels[2 * a]->dutyCycle = map(val, IBUS_MID, IBUS_MAX, 0, RESOLUTION);
				pwmChannels[(2 * a) + 1]->dutyCycle = PWM_OFF;
			}
			else if (val < IBUS_MID)
			{
#ifdef DEBUG
				mySerial.print("CH ");
				mySerial.print(a);
				mySerial.print(" DOWN:");
				mySerial.println(map(val, IBUS_MID, IBUS_MIN, 0, RESOLUTION));
#endif

				pwmChannels[2 * a]->dutyCycle = PWM_OFF;
				pwmChannels[(2 * a) + 1]->dutyCycle = map(val, IBUS_MID, IBUS_MIN, 0, RESOLUTION);
			}
			else
			{
#ifdef DEBUG
				mySerial.print("CH ");
				mySerial.print(a);
				mySerial.println("MID");
#endif

				pwmChannels[2 * a]->dutyCycle = pwmChannels[(2 * a) + 1]->dutyCycle = PWM_OFF;
			}
		}


#ifdef DEBUG
		for (int a = 0; a < sizeof(pwmChannels) / sizeof(pwmChannels[0]); a += 2)
		{
			mySerial.print("CH ");
			mySerial.print(a / 2);
			mySerial.print(" : ");
			mySerial.print(pwmChannels[a]->dutyCycle);
			mySerial.print(" / ");
			mySerial.print(pwmChannels[a + 1]->dutyCycle);
			mySerial.println();
		}
#endif

		//next 4 channels go to servo pins
		for (int a = 0; a < 4; a++)
		{
			uint8_t val = iBus.channels[iBusMap[a + 6]];
			servoChannels[a].write(map(val, IBUS_MIN, IBUS_MAX, SERVO_MIN, SERVO_MAX));

		}
	}
}


//isr for sw pwm
ISR(TCA0_OVF_vect)
{
	static uint8_t currentPwm = 0;


	for (uint8_t a = 0; a < paPinsCount; a++)
	{
		//if (currentPwm < paPins[a].dutyCycle)
		//{
		//	PORTA.OUTCLR = paPins[a].bitMask; //set pin low
		//}
		//else
		//{
		//	PORTA.OUTSET = paPins[a].bitMask; //set pin hi
		//}

		if (currentPwm < paPins[a].dutyCycle)
		{
			PORTA.OUTSET = paPins[a].bitMask; //set pin high
		}
		else
		{
			PORTA.OUTCLR = paPins[a].bitMask; //set pin low
		}
	}


	for (uint8_t a = 0; a < pbPinsCount; a++)
	{
		//if (currentPwm < pbPins[a].dutyCycle)
		//{
		//	PORTB.OUTCLR = pbPins[a].bitMask; //set pin low
		//}
		//else
		//{
		//	PORTB.OUTSET = pbPins[a].bitMask; //set pin high
		//}

		if (currentPwm < pbPins[a].dutyCycle)
		{
			PORTB.OUTSET = pbPins[a].bitMask; //set pin high
		}
		else
		{
			PORTB.OUTCLR = pbPins[a].bitMask; //set pin low
		}
	}


	//increase or reset the counter at the end of the period
	if (currentPwm++ >= RESOLUTION)
	{
		currentPwm = 0;
	}


	//clear the interrupt flag
	TCA0.SINGLE.INTFLAGS = TCA_SINGLE_OVF_bm;
}
