#include "Arduino.h"

#define NUM_SENSORES 17
#define PINS0 PD4
#define PINS1 PD5

// Div clock MASKS
#define MASK_DIV2 0x01 // Also 0x00
#define MASK_DIV4 0x02
#define MASK_DIV8 0x03
#define MASK_DIV16 0x04
#define MASK_DIV32 0x05
#define MASK_DIV64 0x06
#define MASK_DIV128 0x07

// GET BIT MASKS
#define MASK_GET_BIT0 0x01
#define MASK_GET_BIT1 0x02
#define MASK_GET_BIT2 0x04
#define MASK_GET_BIT3 0x08

// CLEAR BIT MASKS
#define MASK_CLEAR_BIT0 0x00
#define MASK_CLEAR_BIT1 0x01
#define MASK_CLEAR_BIT2 0x02
#define MASK_CLEAR_BIT3 0x03

#define MASK_CLEAR_MUXBITS 0x0F

//Variable declaration
int sensor[NUM_SENSORES] = {0};
bool s[2] = {0};
uint8_t m = 0;

//Auxiliary variables
long a = 0,b = 0, c = 0 , d = 0;

void setup(){
	cli();
	Serial.begin(9600);
	Serial.println("Iniciating...");
//  External MUX configuration for first sensor reading
	DDRD |= _BV(PINS0) | _BV(PINS1);	// pinMode(...)
	PORTD = 0;
	// digitalWrite(PINS0, s[0]);
	// digitalWrite(PINS1, s[1]);

// ADC registers configuration
	ADMUX |= _BV(REFS0) ;	// Set bit to configure reference voltage

	ADCSRA = 0;	// Both registers cleared
	ADCSRB = 0;

	ADCSRA |= _BV(ADEN) | _BV(ADATE) | _BV(ADIE);	// Enabled ADC. Free running mode activated. Interruptions activated
	ADCSRA |= MASK_DIV128;	// Velocity set up to 500 kHz --> Div = 32 --> F = 16M/32 = 500 kHz
	ADCSRA |= _BV(ADSC);	// Start Conversion
	sei();
}

void loop(){
	b = millis();
	if(b - a > 1000){
		a = millis();
		for(int i=0; i<NUM_SENSORES; i++){
			Serial.print("Sensor[");
			Serial.print(i);
			Serial.print("] = ");
			Serial.print(sensor[i]);
			Serial.print("\n");
		}
		Serial.print("\n");
	}
}

// ADC interruption
ISR(ADC_vect){
	//c = micros();
	sensor[m++] = ADCL | (ADCH << 8);	// Value assignment
	if(m > (NUM_SENSORES - 1)) m = 0;

	ADMUX &= ~MASK_CLEAR_MUXBITS;	// ADMUX register 4 last bit are cleared
	ADMUX |= m & (MASK_GET_BIT1 | MASK_GET_BIT0);
	// The last two values of m are copied to ADMUX:
	// 1.	   0000 0001 --> MASK_GET_BIT0
	//	OR 0000 0010 --> MASK_GET_BIT1
	// 	  -----------
	// 2.	   0000 0011 --> (MASK_GET_BIT0 | MASK_GET_BIT1)
	//   AND XXXX XXXX --> m
	//	  -----------
	//	   0000 00XX --> 2 last values isolated and ready to be copied

	if (m ==0 || m == 4 || m == 8 || m == 12) {
		PORTD = 0;	// PORTD register is cleared
		s[0] = m & MASK_GET_BIT2;	// Bit nº2 of m is copied to S0
		s[1] = m & MASK_GET_BIT3; 	// Bit nº3 of m is copied to S1
		// How do the copy works? (f.e: S0 demonstration)
		// 1. m = XXXX XXXX --> bit 2 is the value desired for S0
		// 2.     XXXX XXXX --> m
		//	AND 0000 0100 --> MASK_GET_BIT2
		//       -----------
		//	    0000 0X00 --> S0 ('0' or '1' the only possible values)
		PORTD |= (s[0] << PINS0) | (s[1] << PINS1);	// PORTD is updated
	}

	//d = micros();
	//Serial.print("Tiempo --> ");
	//Serial.println(d-c);
	ADCSRA &= ~ _BV(ADIF);	// Flag cleared to start a new conversion

}
