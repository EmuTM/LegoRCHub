#define VERSION 6


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
#include "Config.h"
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
volatile pwmPin_t* drivenPins[6]
{
	&pwmPins[0],
	&pwmPins[2],
	&pwmPins[4],
	&pwmPins[6],
	&pwmPins[8],
	&pwmPins[10]
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
	ServoPin::attachAll(servoPins, servoPinsCount);

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

		//first 6 channels are pwm pins (6 x H-bridge; 2 pins per bridge, alternately driven)
		for (int a = 0; a < 6; a++)
		{
			int _2a = 2 * a;
			int _2a1 = _2a + 1;

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
					mySerial.println(map(val, IBUS_MID, IBUS_MAX, MIN_PWM, MAX_PWM));
#endif

					//make sure to switch the proper pin in the drivenPins
					if (drivenPins[a] != &pwmPins[_2a])
					{
						drivenPins[a] = &pwmPins[_2a];
					}

					//set the duty cycle on the driven pin...
					drivenPins[a]->dutyCycle = map(val, IBUS_MID, IBUS_MAX, MIN_PWM, MAX_PWM);

					//...and DRIVE the other pin
					pwmPins[_2a1].port->OUTCLR = pwmPins[_2a1].bitMask;
				}
				else if (val < IBUS_MID)
				{
#ifdef DEBUG
					mySerial.print("CH ");
					mySerial.print(a);
					mySerial.print(" DOWN:");
					mySerial.println(map(val, IBUS_MID, IBUS_MIN, MIN_PWM, MAX_PWM));
#endif

					//make sure to switch the proper pin in the drivenPins
					if (drivenPins[a] != &pwmPins[_2a1])
					{
						drivenPins[a] = &pwmPins[_2a1];
					}

					//set the duty cycle on the driven pin...
					drivenPins[a]->dutyCycle = map(val, IBUS_MID, IBUS_MIN, MIN_PWM, MAX_PWM);

					//...and DRIVE the other pin
					pwmPins[_2a].port->OUTCLR = pwmPins[_2a].bitMask;
				}
				else
				{
#ifdef DEBUG
					mySerial.print("CH ");
					mySerial.print(a);
					mySerial.println("MID");
#endif

					//both pins must be OFF; set the duty cycle, but also drive the pins
#ifdef PWM_SLOW_DECAY
					drivenPins[a]->dutyCycle = MAX_PWM;

					pwmPins[_2a].port->OUTSET = pwmPins[_2a].bitMask;
					pwmPins[_2a1].port->OUTSET = pwmPins[_2a1].bitMask;
#else
					drivenPins[a]->dutyCycle = 0;

					pwmPins[_2a].port->OUTCLR = pwmPins[_2a].bitMask;
					pwmPins[_2a1].port->OUTCLR = pwmPins[_2a1].bitMask;
#endif
				}
			}
		}


#ifdef DEBUG
		for (int a = 0; a < pwmPinsCount; a += 2)
		{
			mySerial.print("CH ");
			mySerial.print(a / 2);
			mySerial.print(" : ");
			mySerial.print(pwmPins[a]->dutyCycle);
			mySerial.print(" / ");
			mySerial.print(pwmPins[a + 1]->dutyCycle);
			mySerial.println();
		}
#endif

		//next 4 channels go to servo pins
		for (int a = 0; a < servoPinsCount; a++)
		{
			uint16_t val = iBus.channels[iBusMap[a + 6]];

			if (val != previousVals[a + 6])
			{
				previousVals[a + 6] = val;
				servoPins[a].servo.write(map(val, IBUS_MIN, IBUS_MAX, SERVO_MIN, SERVO_MAX));
			}
		}
	}
#endif
}

//SW PWM ISR
ISR(TCA0_OVF_vect)
{
	static uint8_t currentPwm = 0;


	if (currentPwm < drivenPins[0]->dutyCycle)
	{
		drivenPins[0]->port->OUTSET = drivenPins[0]->bitMask; //set pin high
	}
	else
	{
		drivenPins[0]->port->OUTCLR = drivenPins[0]->bitMask; //set pin low
	}

	if (currentPwm < drivenPins[1]->dutyCycle)
	{
		drivenPins[1]->port->OUTSET = drivenPins[1]->bitMask; //set pin high
	}
	else
	{
		drivenPins[1]->port->OUTCLR = drivenPins[1]->bitMask; //set pin low
	}

	if (currentPwm < drivenPins[2]->dutyCycle)
	{
		drivenPins[2]->port->OUTSET = drivenPins[2]->bitMask; //set pin high
	}
	else
	{
		drivenPins[2]->port->OUTCLR = drivenPins[2]->bitMask; //set pin low
	}

	if (currentPwm < drivenPins[3]->dutyCycle)
	{
		drivenPins[3]->port->OUTSET = drivenPins[3]->bitMask; //set pin high
	}
	else
	{
		drivenPins[3]->port->OUTCLR = drivenPins[3]->bitMask; //set pin low
	}

	if (currentPwm < drivenPins[4]->dutyCycle)
	{
		drivenPins[4]->port->OUTSET = drivenPins[4]->bitMask; //set pin high
	}
	else
	{
		drivenPins[4]->port->OUTCLR = drivenPins[4]->bitMask; //set pin low
	}

	if (currentPwm < drivenPins[5]->dutyCycle)
	{
		drivenPins[5]->port->OUTSET = drivenPins[5]->bitMask; //set pin high
	}
	else
	{
		drivenPins[5]->port->OUTCLR = drivenPins[5]->bitMask; //set pin low
	}

	//increases or resets currentPwm; this is a trickery, from chatgpt, like modulo but faster
	currentPwm = (currentPwm + 1) & PWM_RESOLUTION;

	//clear the interrupt flag
	TCA0.SINGLE.INTFLAGS = TCA_SINGLE_OVF_bm;
}
