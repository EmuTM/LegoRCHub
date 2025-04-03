#define VERSION 4


//#define DEBUG
#ifdef DEBUG
#define SW_SERIAL_BAUD_RATE 9600

//enable this to measure PWM frequency
//#define DEBUG_ISR_FREQ
#endif


#ifdef DEBUG
#include <SoftwareSerial.h>

#endif
#include "IBus.h"
#include "PinDefinitions.h"
#include <Servo_megaTinyCore.h>


#ifdef DEBUG
SoftwareSerial mySerial(-1 /*rx not supported*/, PIN_PC3 /*any pin except UPDI*/);
IBus iBus(&Serial, &mySerial);

#ifdef DEBUG_ISR_FREQ
volatile uint32_t isrCounter = 0; //counter for ISR calls
volatile uint32_t lastMillis = 0; //stores last millis for frequency calculation
#endif
#else
IBus iBus(&Serial);
#endif


//state
volatile pwmPin_t* activePWMChannels[6]
{
	&pwmChannels[0],
	&pwmChannels[2],
	&pwmChannels[4],
	&pwmChannels[6],
	&pwmChannels[8],
	&pwmChannels[10]
};

volatile uint16_t previousVals[10] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
//state


void setup()
{
#ifdef DEBUG
	delay(3000); //this is for visual convenience when using port monitors
	mySerial.begin(SW_SERIAL_BAUD_RATE);
	mySerial.print("DEBUG (v");
	mySerial.print(VERSION);
	mySerial.println(")");
#endif

	noInterrupts();

	iBus.begin();

	//disable TX in the USART peripheral, so the tx pin is available (for pwm)
	USART0.CTRLB &= ~USART_TXEN_bm;

	//configure all pins on PORTA and PORTB as output (for pwm)
	PORTA.DIRSET = 0xfe; //PA0 is UPDI
	PORTB.DIRSET = 0x37; //PB3 is RX

	//set ISR on timer A
	{
		//timer_frequency_Hz = processor_clock_frequency_Hz / prescaler_value
		//interrupt_frequency_Hz = timer_frequency_Hz / (TCA0.SINGLE.PER[timer period] + 1)
		//pwm_frequency_Hz = timer_frequency_Hz / resolution

		//3kHz with PWM_RESOLUTION being 31
		TCA0.SINGLE.CTRLA = TCA_SINGLE_CLKSEL_enum::TCA_SINGLE_CLKSEL_DIV16_gc | TCA_SINGLE_ENABLE_bm;
		TCA0.SINGLE.PER = 12;
		TCA0.SINGLE.INTCTRL = TCA_SINGLE_OVF_bm; //enable overflow interrupt
	}

	//attach servos
	ServoPin::attachAll(servoChannels, servoChannelsCount);

	interrupts();


}

void loop()
{
#ifdef DEBUG_ISR_FREQ
	noInterrupts(); //disable interrupts to safely access shared variables
	uint32_t count = isrCounter;
	uint32_t lastUpdate = lastMillis;
	interrupts(); //re-enable interrupts

	//calculate the ISR frequency once every second
	if (millis() - lastUpdate >= 1000)
	{
		noInterrupts();

		//update the lastMillis timestamp
		lastMillis = millis();
		//reset the ISR counter
		isrCounter = 0;

		interrupts();

		mySerial.print("ISR Frequency: ");
		mySerial.print(count);
		mySerial.println(" Hz");
	}
#else
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

			if (val != previousVals[a])
			{
				previousVals[a] = val;

				if (val > IBUS_MID)
				{
#ifdef DEBUG
					mySerial.print("CH ");
					mySerial.print(a);
					mySerial.print(" UP:");
					mySerial.println(map(val, IBUS_MID, IBUS_MAX, 0, PWM_RESOLUTION));
#endif

					activePWMChannels[a] = &pwmChannels[2 * a];
					pwmChannels[2 * a].dutyCycle = map(val, IBUS_MID, IBUS_MAX, 0, PWM_RESOLUTION);
					pwmChannels[(2 * a) + 1].dutyCycle = PWM_OFF;
				}
				else if (val < IBUS_MID)
				{
#ifdef DEBUG
					mySerial.print("CH ");
					mySerial.print(a);
					mySerial.print(" DOWN:");
					mySerial.println(map(val, IBUS_MID, IBUS_MIN, 0, PWM_RESOLUTION));
#endif

					activePWMChannels[a] = &pwmChannels[(2 * a) + 1];
					pwmChannels[2 * a].dutyCycle = PWM_OFF;
					pwmChannels[(2 * a) + 1].dutyCycle = map(val, IBUS_MID, IBUS_MIN, 0, PWM_RESOLUTION);
				}
				else
				{
#ifdef DEBUG
					mySerial.print("CH ");
					mySerial.print(a);
					mySerial.println("MID");
#endif

					pwmChannels[2 * a].dutyCycle = pwmChannels[(2 * a) + 1].dutyCycle = PWM_OFF;
				}
			}
		}


#ifdef DEBUG
		for (int a = 0; a < pwmChannelsCount; a += 2)
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
		for (int a = 0; a < servoChannelsCount; a++)
		{
			uint16_t val = iBus.channels[iBusMap[a + 6]];

			if (val != previousVals[a + 6])
			{
				previousVals[a + 6] = val;
				servoChannels[a].servo.write(map(val, IBUS_MIN, IBUS_MAX, SERVO_MIN, SERVO_MAX));
			}
		}
	}
#endif
}

//SW PWM ISR
ISR(TCA0_OVF_vect)
{
	static uint8_t currentPwm = 0;

	{
#ifdef PWM_INVERTED
		if (currentPwm > activePWMChannels[0]->dutyCycle)
#else
		if (currentPwm < activePWMChannels[0]->dutyCycle)
#endif
		{
			activePWMChannels[0]->port->OUTSET = activePWMChannels[0]->bitMask; //set pin high
		}
		else
		{
			activePWMChannels[0]->port->OUTCLR = activePWMChannels[0]->bitMask; //set pin low
		}
	}
	{
#ifdef PWM_INVERTED
		if (currentPwm > activePWMChannels[1]->dutyCycle)
#else
		if (currentPwm < activePWMChannels[1]->dutyCycle)
#endif
		{
			activePWMChannels[1]->port->OUTSET = activePWMChannels[1]->bitMask; //set pin high
		}
		else
		{
			activePWMChannels[1]->port->OUTCLR = activePWMChannels[1]->bitMask; //set pin low
		}
	}
	{
#ifdef PWM_INVERTED
		if (currentPwm > activePWMChannels[2]->dutyCycle)
#else
		if (currentPwm < activePWMChannels[2]->dutyCycle)
#endif
		{
			activePWMChannels[2]->port->OUTSET = activePWMChannels[2]->bitMask; //set pin high
		}
		else
		{
			activePWMChannels[2]->port->OUTCLR = activePWMChannels[2]->bitMask; //set pin low
		}
	}
	{
#ifdef PWM_INVERTED
		if (currentPwm > activePWMChannels[3]->dutyCycle)
#else
		if (currentPwm < activePWMChannels[3]->dutyCycle)
#endif
		{
			activePWMChannels[3]->port->OUTSET = activePWMChannels[3]->bitMask; //set pin high
		}
		else
		{
			activePWMChannels[3]->port->OUTCLR = activePWMChannels[3]->bitMask; //set pin low
		}
	}
	{
#ifdef PWM_INVERTED
		if (currentPwm > activePWMChannels[4]->dutyCycle)
#else
		if (currentPwm < activePWMChannels[4]->dutyCycle)
#endif
		{
			activePWMChannels[4]->port->OUTSET = activePWMChannels[4]->bitMask; //set pin high
		}
		else
		{
			activePWMChannels[4]->port->OUTCLR = activePWMChannels[4]->bitMask; //set pin low
		}
	}
	{
#ifdef PWM_INVERTED
		if (currentPwm > activePWMChannels[5]->dutyCycle)
#else
		if (currentPwm < activePWMChannels[5]->dutyCycle)
#endif
		{
			activePWMChannels[5]->port->OUTSET = activePWMChannels[5]->bitMask; //set pin high
		}
		else
		{
			activePWMChannels[5]->port->OUTCLR = activePWMChannels[5]->bitMask; //set pin low
		}
	}

	//increases or resets currentPwm; this is a trickery, from chatgpt, like modulo but faster
	currentPwm = (currentPwm + 1) & PWM_RESOLUTION;

	//clear the interrupt flag
	TCA0.SINGLE.INTFLAGS = TCA_SINGLE_OVF_bm;
}
