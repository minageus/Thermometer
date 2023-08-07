/*
 * Thermometer.c
 *
 * Created: 12/07/2023 09:32:54
 * Author : dminagias
 */ 

/*
We will write a program for an ATmega328p chip that measures the temperature. 
The circuit has the chip an din one of its pins there is a voltage divider.
The voltage divider consists of one known resistance (Rref) and an NTC, more
specifically the Semitec 103AT-4-70374 NTC, that can read temperatures from
-15oC to 90oC. We will do the following implementation: the ATmega328p will
read the voltage in the vout pin. Knowing the fact that we have to do with a
voltage divider, we are going to find the resistance of the NTC in a specific
temperature. Given the Steinhart-Hart formula for the NTCs and all the 
specifications of the NTC, ie its beta, our code will simply calculate the 
temperature.
*/



#include <avr/io.h>
#define F_CPU 16000000UL	//always before the <util/delay.h> header
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdbool.h>  //only for the up variable, in there is not so much space
#include "Serial.h"

//we will do the variable int and use 0 and1 instead
//#define _BV(bit) (1<<bit) //for some reason the compiler
//states that it is redefined in sfr_refs.h, that's why I put it in comments
#define Rref 10000	//The known resistance is (Rref) 10kΩ
#define T0 298.15	//25oC in Kelvin scale
#define beta 3435	//ranges from 3000 to 4000, characteristic of each NTC
#define BYTE uint8_t
#define LED_GREEN_ON()	{PORTD &= ~_BV(2);}
#define LED_GREEN_OFF()	{PORTD |= _BV(2);}
#define ButtonIsPressed()  ((PIND & _BV(3)) == 0)
#define LED_RED_ON()		{PORTD &= ~_BV(4);}
#define LED_RED_OFF()		{PORTD |= _BV(4);}
#define THERMO_ON()		{PORTD &= ~_BV(5);}
#define THERMO_OFF()	{PORTD |= _BV(5);}
#define EEPROM_ADDRESS_START_HEATER 0
#define EEPROM_ADDRESS_STOP_HEATER 1
#define EEPROM_ADDRESS_FAN_IDLE 2
#define EEPROM_ADDRESS_FAN_WORKING 3
#define EEPROM_ADDRESS_DELAY 4


// DEBUG DEFINES
// #define DEBUG__REPORT_INITIAL_SETTINGS
// #define DEBUG__REPORT_TEMPERATURE
// #define DEBUG__SERIAL_RECEPTION

// LED state machine
enum LEDState {
	NORMAL_HEAT_OFF_GREEN_ON,
	NORMAL_HEAT_OFF_GREEN_OFF,
	NORMAL_HEAT_ON_GREEN_ON,
	NORMAL_HEAT_ON_GREEN_OFF,
	TEST_GREEN_ON,
	TEST_GREEN_OFF,
	ALARM_NTC_RED_ON,
	ALARM_NTC_RED_OFF,
	ALARM_OVERCURRENT_RED_ON1,
	ALARM_OVERCURRENT_RED_OFF1,
	ALARM_OVERCURRENT_RED_ON2,
	ALARM_OVERCURRENT_RED_OFF2
};

// Automation state machine
enum AutomationState {
	WAIT_FOR_LOW_TEMP,
	LOW_TEMP_FOUND,
	LOW_TEMP_FAN_START,
	LOW_TEMP_HEATER_ON,
	
	ALARM_LOCKOUT_NTC,
	ALARM_LOCKOUT_OVERCURRENT,
	
	TEST_BEGIN,
	TEST_FAN_START,
	TEST_HEATER_ON,
};

typedef enum _eLedTaskPhase_t
{
	LTP_Null = 0,
	LTP_NormalOperation_Begin,
	LTP_NormalOperation_On,
	LTP_NormalOperation_Off,
	LTP_TestMode_Begin,
	LTP_TestMode_Flash1_On,
	LTP_TestMode_Flash1_Off,
	LTP_TestMode_Flash2_On,
	LTP_TestMode_Flash2_Off,
} eLedTaskPhase_t;



typedef enum Task_button
{
	button_Released,
	button_Pressed_check,
	button_Pressed_100ms,
	button_Pressed_1000ms,
} Task_button;

typedef enum Automation
{
	Aut_Heat_Temp,
	Aut_Heat_Predelay,
	Aut_Heat_Heat,
	Aut_Test_Predelay_Heat,
} Automation;

typedef enum Errors
{
	Aut_Error_NTC,
	Aut_Error_Current,
} Errors;


unsigned long previousTempTime = 0;
float temperature_Sum = 0.0f;
uint16_t temperature_NoOfMeasurements = 0;
uint16_t temperature_MeasurementsWithError = 0;
uint16_t temperature_MeasurementsValid = 0;
float temperature_LastAverage = 25.0f;
bool temperature_MeasurementError = false;

// * Serial reception variables
uint8_t recv_InputChar;
char recv_LinearInputBuffer[30];
int recv_LinearInputBufferDataSize = 0;

// * Debug print buffer
char debugPrintBuffer[40];

enum AutomationState automation_CurrentState = WAIT_FOR_LOW_TEMP;
uint32_t automation_Tick = 0;


Task_button ButtonTask = 0;
uint32_t button_Tick = 0;
bool button_LongPressDetected = false;


enum LEDState led_State = NORMAL_HEAT_OFF_GREEN_ON;
uint32_t led_Tick = 0;


////////////////////////////////////////////////////////////////////////////////
// Type definitions.
////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////
// Enums.


// Variables for the LED task
eLedTaskPhase_t g_eLedTask_Phase = LTP_Null;
uint32_t g_lLedTask_Tick;



uint32_t g_tTimer2Unserved1msecEventsCounter;

float vin=5;
float vout;
unsigned int value;
float buffer = 0;
float RT = 0;
char temperature_show[20];// for the sprintf

static const uint8_t A = 126;	//totally random
static const uint8_t C = 234;	//totally random
static const uint8_t rand_max = 13;

uint32_t g_lTickValue = 0;
uint32_t pwm = 160;




void InitADC()
{
	// Select Vref=AVcc
	ADMUX |= (1<<REFS0);
	//set prescaller to 128 and enable ADC
	ADCSRA |= (1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0)|(1<<ADEN);
}
uint16_t ReadADC(uint8_t ADCchannel)
{
	//select ADC channel with safety mask
	ADMUX = (ADMUX & 0xF0) | (ADCchannel & 0x0F);
	//single conversion mode
	ADCSRA |= (1<<ADSC);
	// wait until ADC conversion is complete
	while( ADCSRA & (1<<ADSC) );
	return ADC;
}


//Initialize the frequence at 20KHz
void setup()
{
	TCCR1A = _BV(COM1A1) | (0<< COM1A0) | (0 << COM1B1) | (0 << COM1B0) | _BV(WGM11) | (0 << WGM10);
	ICR1 = 799;
	TCNT1 = 0;
	OCR1B = 0;
	OCR1A = 200;
	TIMSK1 = (0 << ICIE1) | (0 << OCIE1B) | (0 << OCIE1A) | (0 << TOIE1) ;
	DDRB |= _BV(PORTB1);
	TCCR1B = (0 << ICNC1) | (0 << ICES1) | (1 << WGM13) | (1 << WGM12) | (0 << CS12) | (0 << CS11) | (1 << CS10);
}

// Ξεκινάει τον Timer/Counter1 ο οποίος χρησιμοποιείται για γενικές περιοδικές
// διαδικασίες. Θα τον θέσουμε για περίοδο 10,000 msec. Είναι υποχρεωτικό η περίοδος
// να διαιρεί ακριβώς τα 100 msec γιατί την χρησιμοποιούμε και για μέτρηση χρόνου.
void TimerCounter2Start()
{
	BYTE l_tStatusRegister;
	
	l_tStatusRegister = SREG;
	cli();
	// Ο timer θα πρέπει να είναι ήδη σταματημένος, αλλά σταμάτα τον ξανά.
	// Ξεκινάμε από τον καταχωρητή TCCR1B που σταματάει τον timer και μετά
	// καθαρίζουμε τους υπόλοιπους καταχωρητές.
	TCCR2B = 0x00;
	TCCR2A = 0x00;
	// Απενεργοποιούμε όλα του τα interrupts.
	TIMSK2 = 0x00;
	// Καθαρίζουμε τυχούσες σημαίες interrupt.
	TIFR2 = _BV(OCF2B) | _BV(OCF2A) | _BV(TOV2);
	
	// Αρχικοποιούμε τις μεταβλητές τις σχετικές με τον timer.
	g_tTimer2Unserved1msecEventsCounter = 0;
	
	// Θα δουλέψουμε σε clear timer on compare match mode, prescaler = 128,
	// χρησιμοποιώντας το Output Compare A (υποχρεωτικό στο CTC mode).
	// Η συχνότητα που επιτυγχάνουμε είναι F = Fclk / (prescaler * (1 + OCR2A)).
	// Με κρύσταλλο 16000000 MHz, τον OCR1A = 127 και prescaler 128, παίρνουμε
	// συχνότητα 1000,000 Hz που δίνει περίοδο 1,000 msec.
	// Πριν ενεργοποιήσουμε τους timers, θέτουμε τους καταχωρητές μέτρησης στην τιμή 0
	// οπότε, έχουμε κάμποσο (!) χρόνο μετά την ενεργοποίηση μέχρι να συμβεί interrupt.
	TCNT2 = 0;
	OCR2A = 127;
	// Θέσε τον Timer/Counter1 σε CTC mode, prescaler = 128.
	// Στους καταχωρητές TCCR1A, TCCR1C δεν χρειάζεται να γράψουμε τίποτε,
	// οι προηγούμενοι μηδενισμοί βάζουν την τιμή που πρέπει να έχουν.
	TCCR2A = _BV(WGM21);
	TCCR2B = _BV(CS22) |_BV(CS20);
	// Και ενεργοποιούμε το Timer/Counter2 Output Compare Match Α interrupt.
	TIMSK2 |= _BV(OCIE2A);
	SREG = l_tStatusRegister;
}


// Interrupt service για το Timer/Counter1 Output Compare A Match interrupt.
// Συμβαίνει κάθε 10,0000 msec.
ISR(TIMER2_COMPA_vect)
{
	g_lTickValue++;
}

 
uint32_t TickCounterGetValue(void)
{
	BYTE l_tStatusRegister;
	uint32_t l_lReturnValue;
	
	l_tStatusRegister = SREG;
	cli();

	l_lReturnValue = g_lTickValue;

	// Επαναφέρουμε τα interrupts εκεί που ήτανε.
	SREG = l_tStatusRegister;
	
	return l_lReturnValue;
}



uint8_t EepromReadByte(uint16_t p_rAddress)
{
	// Πρώτα περίμενε μέχρι να είναι ελεύθερη η διαδικασία write της EEPROM.
	while ((EECR & _BV(EEPE)) != 0);
	// Δώσε τη διεύθυνση, διάβασε και επέστρεψε.
	EEARH = p_rAddress / 256;
	EEARL = p_rAddress % 256;
	EECR |= _BV(EERE);
	return EEDR;
}


// Γράφει την τιμή p_tData στη διεύθυνση p_rAddress της EEPROM.
// Για να έχουμε ταχύτητα και να αποφεύγουμε φθορά της EEPROM, αν τα υπάρχοντα
// περιεχόμενα είναι ίδια με την τιμή που θέλουμε να γράψουμε, απλά επιστρέφουμε.
// Αν είναι να γράψει, φρεσκάρει και το watchdog.
void EepromWriteByte(uint16_t p_rAddress, uint8_t p_tData)
{
	irqflags_t l_tIrqFlags;

	// Περίμενε να ελευθερωθεί η διαδικασία εγγραφής.
	while ((EECR & _BV(EEPE)) != 0);
	// Περίμενε και τη διαδικασία self programming της flash.
	while ((SPMCSR & _BV(SELFPRGEN)) != 0);
	// Διάβασε τα περιεχόμενα και αν είναι ίδια με την τιμή που θέλουμε να γράψουμε
	// επέστρεψε.
	EEARH = p_rAddress / 256;
	EEARL = p_rAddress % 256;
	EECR |= _BV(EERE);
	if (EEDR == p_tData)
	{
		return;
	}
	// Πρέπει να κάνουμε εγγραφή.
	wdt_reset();
	// Προετοίμασε τους καταχωρητές address και data της EEPROM.
	EEARH = p_rAddress / 256;
	EEARL = p_rAddress % 256;
	EEDR = p_tData;
	// Μηδένισε τα bits EEPM1, EEPM0 του EECR ώστε το σβήσιμο και η εγγραφή της
	// θέσης μνήμης της EEPROM να γίνουν σε μία λειτουργία.
	EECR &= ~(_BV(EEPM1) | _BV(EEPM0));
	// Κράτα την κατάσταση του Status Register (που έχει την κατάσταση των
	// interrupts) απενεργοποιώντας ταυτόχρονα τα interrupts.
	l_tIrqFlags = cpu_irq_save();
	EECR |= _BV(EEMPE);
	EECR |= _BV(EEPE);
	// Επανάφερε την κατάσταση των interrupts.
	cpu_irq_restore(l_tIrqFlags);
	// Περίμενε να ελευθερωθεί η διαδικασία εγγραφής.
	while ((EECR & _BV(EEPE)) != 0);
}

void  EepromWriteInteger(uint16_t p_rAddress, uint16_t p_tData)
{
	EepromWriteByte(p_rAddress, (uint8_t)(p_tData));
	EepromWriteByte(p_rAddress + 1, (uint8_t)(p_tData >> 8));
}

uint16_t EepromReadInteger(uint16_t p_rAddress)
{
	return ((uint16_t) EepromReadByte(p_rAddress)) | (((uint16_t)EepromReadByte(p_rAddress + 1)) << 8);
}

// Helper function declarations
void setFanPWM(int percentage);
void setHeaterPWM(int percentage);
void startHeater();
void stopHeater();


float readCurrent(void)
{
	InitADC();
	float rawCurr;
	float potval=(float)(Rref/5*ReadADC(0)/1024)/(1-(ReadADC(0)/(1024*5)));
	potval = (float) ReadADC(0) / potval;
	rawCurr = potval;
	return rawCurr;
}




int start_heater, stop_heater, fan_speed_idle, fan_speed_working, delay;
// Function to handle LED blinking based on the state
void LedTask()
{
	unsigned long currentTime = TickCounterGetValue();
	
	if (automation_CurrentState == WAIT_FOR_LOW_TEMP &&
	!(led_State >= NORMAL_HEAT_OFF_GREEN_ON || led_State <= NORMAL_HEAT_OFF_GREEN_OFF))
	{
		led_State = NORMAL_HEAT_OFF_GREEN_ON;
		led_Tick = currentTime;
	}
	
	if ((automation_CurrentState >= LOW_TEMP_FOUND && automation_CurrentState <= LOW_TEMP_HEATER_ON) &&
		!(led_State >= NORMAL_HEAT_ON_GREEN_ON || led_State <= NORMAL_HEAT_ON_GREEN_OFF))
	{
		led_State = NORMAL_HEAT_ON_GREEN_ON;
		led_Tick = currentTime;
	}
	
	if ((automation_CurrentState >= TEST_BEGIN && automation_CurrentState <= TEST_HEATER_ON) &&
		!(led_State >= TEST_GREEN_ON || led_State <= TEST_GREEN_OFF))
	{
		led_State = TEST_GREEN_ON;
		led_Tick = currentTime;
	}
	
	if (automation_CurrentState == ALARM_LOCKOUT_NTC &&
		!(led_State >= ALARM_NTC_RED_ON || led_State <= ALARM_NTC_RED_OFF))
	{
		led_State = ALARM_NTC_RED_ON;
		led_Tick = currentTime;
	}

	if (automation_CurrentState == ALARM_LOCKOUT_OVERCURRENT &&
		!(led_State >= ALARM_OVERCURRENT_RED_ON1 || led_State <= ALARM_OVERCURRENT_RED_OFF2))
	{
		led_State = ALARM_OVERCURRENT_RED_ON1;
		led_Tick = currentTime;
	}


	switch (led_State)
	{
		case NORMAL_HEAT_OFF_GREEN_ON:
			LED_RED_OFF();
			LED_GREEN_ON();
			if (currentTime - led_Tick >= 200)
			{
				led_Tick = currentTime;
				led_State = NORMAL_HEAT_OFF_GREEN_OFF;
			}
			break;
		case NORMAL_HEAT_OFF_GREEN_OFF:
			LED_RED_OFF();
			LED_GREEN_OFF();
			if (currentTime - led_Tick >= 800)
			{
				led_Tick = currentTime;
				led_State = NORMAL_HEAT_OFF_GREEN_ON;
			}
			break;
	
	
		case NORMAL_HEAT_ON_GREEN_ON:
			LED_RED_OFF();
			LED_GREEN_ON();
			if (currentTime - led_Tick >= 800)
			{
				led_Tick = currentTime;
				led_State = NORMAL_HEAT_ON_GREEN_OFF;
			}
			break;
		case NORMAL_HEAT_ON_GREEN_OFF:
			LED_RED_OFF();
			LED_GREEN_OFF();
			if (currentTime - led_Tick >= 200)
			{
				led_Tick = currentTime;
				led_State = NORMAL_HEAT_ON_GREEN_ON;
			}
			break;


		case TEST_GREEN_ON:
			LED_RED_OFF();
			LED_GREEN_ON();
			if(currentTime - led_Tick >= 250)
			{
				led_Tick = currentTime;
				led_State = TEST_GREEN_OFF;
			}
			break;
		case TEST_GREEN_OFF:
			LED_RED_OFF();
			LED_GREEN_OFF();
			if(currentTime - led_Tick >= 250)
			{
				led_Tick = currentTime;
				led_State = TEST_GREEN_ON;
			}
			break;
	
		
		case ALARM_NTC_RED_ON:
			LED_GREEN_OFF();
			LED_RED_ON();
			if(currentTime - led_Tick >= 200)
			{
				led_Tick = currentTime;
				led_State = ALARM_NTC_RED_OFF;
			}
			break;
		case ALARM_NTC_RED_OFF:
			LED_GREEN_OFF();
			LED_RED_OFF();
			if(currentTime - led_Tick >= 800)
			{
				led_Tick = currentTime;
				led_State = ALARM_NTC_RED_ON;
			}
			break;
	
		case ALARM_OVERCURRENT_RED_ON1:
			LED_GREEN_OFF();
			LED_RED_ON();
			if(currentTime - led_Tick >= 200)
			{
				led_Tick = currentTime;
				led_State = ALARM_OVERCURRENT_RED_OFF1;
			}
			break;
		case ALARM_OVERCURRENT_RED_OFF1:
			LED_GREEN_OFF();
			LED_RED_OFF();
			if(currentTime - led_Tick >= 200)
			{
				led_Tick = currentTime;
				led_State = ALARM_OVERCURRENT_RED_ON2;
			}
			break;
		case ALARM_OVERCURRENT_RED_ON2:
			LED_GREEN_OFF();
			LED_RED_ON();
			if(currentTime - led_Tick >= 200)
			{
				led_Tick = currentTime;
				led_State = ALARM_OVERCURRENT_RED_OFF2;
			}
			break;
		case ALARM_OVERCURRENT_RED_OFF2:
			LED_GREEN_OFF();
			LED_RED_OFF();
			if(currentTime - led_Tick >= 800)
			{
				led_Tick = currentTime;
				led_State = ALARM_OVERCURRENT_RED_ON1;
			}
			break;
		
		default:
			break;
	}
}

// Function to handle automation (heater and fan control)
void AutomationTask() {
	
	uint32_t currentTime = TickCounterGetValue();


	if (temperature_MeasurementError && automation_CurrentState >= WAIT_FOR_LOW_TEMP && automation_CurrentState <= LOW_TEMP_HEATER_ON)
	{
		automation_CurrentState = ALARM_LOCKOUT_NTC;
	}
	
	if (button_LongPressDetected)
	{
		if (automation_CurrentState == WAIT_FOR_LOW_TEMP)
		{
			automation_CurrentState = TEST_BEGIN;
		}
		button_LongPressDetected = false;
	}

	switch (automation_CurrentState) {
		case WAIT_FOR_LOW_TEMP:
			// Check for temperature thresholds and control the heater and fan accordingly
			if (temperature_LastAverage < start_heater)
			{
				setFanPWM(fan_speed_working);
				automation_CurrentState = LOW_TEMP_FOUND;
			}
			break;
			
		case LOW_TEMP_FOUND:
			automation_Tick = currentTime;
			automation_CurrentState = LOW_TEMP_FAN_START;
			break;
			
		case LOW_TEMP_FAN_START:
			if (currentTime - automation_Tick >= (uint32_t) delay)
			{
				startHeater();
				automation_CurrentState = LOW_TEMP_HEATER_ON;
			}
			break;
			
		case LOW_TEMP_HEATER_ON:
			if (temperature_LastAverage >= stop_heater)
			{
				setFanPWM(fan_speed_idle);
				stopHeater();
				automation_CurrentState = WAIT_FOR_LOW_TEMP;
			}
			break;
	
		case ALARM_LOCKOUT_NTC:
			break;
			
		case ALARM_LOCKOUT_OVERCURRENT:
			break;

		case TEST_BEGIN:
			setFanPWM(fan_speed_working);
			automation_Tick = currentTime;
			automation_CurrentState = TEST_FAN_START;
			break;

		case TEST_FAN_START:
			if (currentTime - automation_Tick >= (uint32_t) delay)
			{
				setFanPWM(fan_speed_working);
				startHeater();
				automation_Tick = currentTime;
				automation_CurrentState = TEST_HEATER_ON;
			}
			break;
			
		case TEST_HEATER_ON:
			if (currentTime - automation_Tick >= 300000UL)
			{
				automation_CurrentState = WAIT_FOR_LOW_TEMP;
				setFanPWM(fan_speed_idle);
				stopHeater();
			}
			break;
	}
}


void setFanPWM(int percentage) {
	OCR1A = (uint16_t)( (799 * (uint32_t) percentage) / 100);
}


void startHeater() {
	THERMO_ON();
}

void stopHeater() {
	THERMO_OFF();
}

void TemperatureMeasurementTask(void)
{
	if (temperature_MeasurementError)
	{
		return;
	}
	
	if (TickCounterGetValue() - previousTempTime >= 50)
	{
		previousTempTime = TickCounterGetValue();

		// Every 50 msec.
		float raw = (float) ReadADC(0);
		float R2 = (raw * Rref) / (1024.0f - raw);
		buffer = log(R2 / Rref);
	
		float temperatura;
		temperatura = (1 / ((buffer / beta) + (1 / T0)));
		temperatura = temperatura - 273.15;

		if (temperatura < -50.0f || temperatura >= 90.0f)
		{
			temperature_MeasurementsWithError++;
		}
		else
		{
			// Valid temperature
			temperature_MeasurementsValid++;
			temperature_Sum += temperatura;
		}
		
		temperature_NoOfMeasurements++;
		if (temperature_NoOfMeasurements >= 40)
		{
			if (temperature_MeasurementsWithError >= 30)
			{
				temperature_MeasurementError = true;
				temperature_LastAverage = 25.0f;
			}
			else
			{
				temperature_MeasurementError = false;
				temperature_LastAverage = temperature_Sum / temperature_MeasurementsValid;				
			}

#ifdef DEBUG__REPORT_TEMPERATURE
			sprintf(debugPrintBuffer, "Temperature is %d oC, auto is %d\r\n", (int16_t) temperature_LastAverage, automation_CurrentState);
			SerialWriteData((uint8_t*)debugPrintBuffer, strlen(debugPrintBuffer));
#endif
			
			// Init for next loop of 40 measurements.
			temperature_Sum = 0.0f;
			temperature_NoOfMeasurements = 0;
			temperature_MeasurementsWithError = 0;
			temperature_MeasurementsValid = 0;
		}
	}
}


void Button_Task(void)
{
	//bool ButtonIsPressed = !button_Released;
	bool b_continue = true;
	while(b_continue)
	{
		b_continue = false;
		
		switch (ButtonTask)
		{
			case button_Released:
				if(ButtonIsPressed())
				{
					button_Tick = TickCounterGetValue();
					ButtonTask = button_Pressed_check;
				}
				break;
			case button_Pressed_check:
				if(!ButtonIsPressed())
				{
					ButtonTask = button_Released;
					break;
				}
				if(TickCounterGetValue() - button_Tick >=100)
				{
					ButtonTask = button_Pressed_100ms;
				}
				break;
			case button_Pressed_100ms:
				if(!ButtonIsPressed())
				{
					ButtonTask = button_Released;
					break;
				}
				if(TickCounterGetValue() - button_Tick >= 1000)
				{
					button_LongPressDetected = true;
					ButtonTask = button_Pressed_1000ms;
				}
				break;
					
				
			case button_Pressed_1000ms:
				if(!ButtonIsPressed())
				{
					ButtonTask = button_Released;
				}
				break;
		}
	}
}

uint8_t written_serial_data[20];
uint8_t read_serial_data[20];
int eeprom_memory_address = 0;

FILE serial_buffer = FDEV_SETUP_STREAM((void *)SerialWriteByte, NULL, _FDEV_SETUP_WRITE);
int serialData_to_write;
int main(void)
{
	InitADC();
	//USART0Init();
	SerialInitialize(51);
	stdout = &serial_buffer;
	
	
	//stdout=&usart0_str;
	THERMO_OFF();
	
	DDRD |= _BV(DDB2);
	DDRD &= ~_BV(DDB3);
	PORTD |= _BV(3);
	DDRD |= _BV(DDB4);
	DDRD |= _BV(DDB5);
		
	DDRC &= ~_BV(DDC0);
	
	start_heater = EepromReadByte(EEPROM_ADDRESS_START_HEATER);
	stop_heater = EepromReadByte(EEPROM_ADDRESS_STOP_HEATER);
	fan_speed_idle = EepromReadByte(EEPROM_ADDRESS_FAN_IDLE);
	fan_speed_working = EepromReadByte(EEPROM_ADDRESS_FAN_WORKING);
	delay = EepromReadByte(EEPROM_ADDRESS_DELAY);
	
	//PWM
	setup();
	TimerCounter2Start();
	sei();
	// bool ledaki = true;

	start_heater = EepromReadByte(EEPROM_ADDRESS_START_HEATER);
	stop_heater = EepromReadByte(EEPROM_ADDRESS_STOP_HEATER);
	fan_speed_idle = EepromReadByte(EEPROM_ADDRESS_FAN_IDLE);
	fan_speed_working = EepromReadByte(EEPROM_ADDRESS_FAN_WORKING);
	delay = EepromReadInteger(EEPROM_ADDRESS_DELAY);
	if(start_heater >=3 && start_heater<=40 && stop_heater>=3 && stop_heater<=40 && start_heater<(3+stop_heater) &&
	fan_speed_idle>=15 && fan_speed_idle<=100 && fan_speed_working >=15 && fan_speed_working<=100 && fan_speed_idle < (fan_speed_working +40) &&
	delay>=1000 && delay<=15000)
	{
#ifdef DEBUG__REPORT_INITIAL_SETTINGS
		sprintf(debugPrintBuffer, "RD %d %d %d %d %d\r\n", start_heater, stop_heater, fan_speed_idle, fan_speed_working, delay);
		SerialWriteData((uint8_t *)debugPrintBuffer, strlen(debugPrintBuffer));
#endif
	}
	else
	{
		start_heater = 5;
		stop_heater = 15;
		fan_speed_idle = 20;
		fan_speed_working = 80;
		delay = 3000;
		EepromWriteByte(EEPROM_ADDRESS_START_HEATER, start_heater);
		EepromWriteByte(EEPROM_ADDRESS_STOP_HEATER , stop_heater);
		EepromWriteByte(EEPROM_ADDRESS_FAN_IDLE ,fan_speed_idle);
		EepromWriteByte(EEPROM_ADDRESS_FAN_WORKING ,fan_speed_working);
		EepromWriteInteger(EEPROM_ADDRESS_DELAY ,delay);
#ifdef DEBUG__REPORT_INITIAL_SETTINGS
		sprintf(debugPrintBuffer, "WC %d %d %d %d %d\r\n", 5, 15, 20, 80, 3000);
		SerialWriteData((uint8_t *)debugPrintBuffer, strlen(recv_LinearInputBuffer));
#endif
	}
	
	while (1)
    {
		// Καλούμε συνέχεια όλες τις "tasks" οι οποίες πρέπει να έχουν πολύ μικρό χρόνο εκτέλεσης.			
		TemperatureMeasurementTask();
		AutomationTask();
		LedTask();
		Button_Task();


		if (SerialReadData(&recv_InputChar))
		{
			if ((recv_InputChar < ' ' || recv_InputChar >= 0x7f) && recv_InputChar != 0x0d)
			{
				recv_LinearInputBufferDataSize = 0;
#ifdef DEBUG__SERIAL_RECEPTION
				sprintf(debugPrintBuffer, "CharRecv %.2x %d\r\n", recv_InputChar, recv_LinearInputBufferDataSize);
				SerialWriteData((uint8_t*)debugPrintBuffer, strlen(debugPrintBuffer));
#endif				
			}
			else
			{
				recv_LinearInputBuffer[recv_LinearInputBufferDataSize] = (char) recv_InputChar;
				if(recv_LinearInputBufferDataSize < 29)
				{
					recv_LinearInputBufferDataSize++;
				}
			
	#ifdef DEBUG__SERIAL_RECEPTION
				sprintf(debugPrintBuffer, "CharRecv %.2x %d\r\n", recv_InputChar, recv_LinearInputBufferDataSize);
				SerialWriteData((uint8_t*)debugPrintBuffer, strlen(debugPrintBuffer));
	#endif
			
				if(recv_InputChar == '\r')
				{												
					if(strncmp("rc\r", recv_LinearInputBuffer, 3) == 0)
					{
						char wc[30];
						sprintf(wc, "rc %d %d %d %d %d\r\n", start_heater, stop_heater, fan_speed_idle, fan_speed_working, delay);
						SerialWriteData((uint8_t*) wc, strlen(wc));
					}
					else if(strncmp("wc ", recv_LinearInputBuffer, 3) == 0)
					{
						int read1= -1, read2 = -1, read3 = -1, read4 = -1, read5 = -1;					
										
						int scanned =  sscanf(recv_LinearInputBuffer+3, "%d %d %d %d %d", &read1, &read2, &read3, &read4, &read5);					
													
						if(scanned == 5 && read1>=3 && read1<=40 &&  read2>=3 && read2<=40 && read3>15 && read3<100
						&& read4>=15 && read4<=100 && read1< (read2 +3) && read3 < (read4+40) && read5 >=1000 && read5 <=15000)
						{
							start_heater = read1;
							stop_heater = read2;
							fan_speed_idle = read3;
							fan_speed_working = read4;
							delay = read5;
							EepromWriteByte(EEPROM_ADDRESS_START_HEATER, start_heater);
							EepromWriteByte(EEPROM_ADDRESS_STOP_HEATER, stop_heater);
							EepromWriteByte(EEPROM_ADDRESS_FAN_IDLE, fan_speed_idle);
							EepromWriteByte(EEPROM_ADDRESS_FAN_WORKING, fan_speed_working);
							EepromWriteInteger(EEPROM_ADDRESS_DELAY, delay);
						
							char wc[30];
							sprintf(wc, "wc %d %d %d %d %d\r\n", start_heater, stop_heater, fan_speed_idle, fan_speed_working, delay);
							SerialWriteData((uint8_t *)wc, strlen(wc));	
						}
						else
						{
							SerialWriteData((uint8_t *)"err\r\n", 5);
						}
					}
					else
					{
						SerialWriteData((uint8_t *)"err\r\n", 5);
					}
					recv_LinearInputBufferDataSize = 0;
				}
			}
		}
	}
	return 0;
}