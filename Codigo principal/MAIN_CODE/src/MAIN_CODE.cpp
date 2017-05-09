
/* << CONFIGURATION OF ADC >>
	ADMUX REGISTER -> ADC Multiplexer Selection Register
		------------------------------------------------------------
		|   7   |   6   |   5   |  4   |  3   |  2   |  1   |  0   |
		|-------|-------|-------|------|------|------|------|------|
		| REFS1 | REFS0 | ADLAR | MUX4 | MUX3 | MUX2 | MUX1 | MUX0 |
		------------------------------------------------------------
			REFS1:0 --> Voltage reference adjust. Set both to '0'
			ADLAR --> Left Adjust on the result of conversion if '1'
			MUX4:0 --> Channel (0 to 7, DEC) and gain selection (8 to 31,DEC)
	ADCSRA REGISTER -> ADC Control and Status Register A
		-------------------------------------------------------------
		|  7   |  6   |   5   |  4   |  3   |   2   |   1   |   0   |
		|------|------|-------|------|------|-------|-------|-------|
		| ADEN | ADSC | ADFR* | ADIF | ADIE | ADPS2 | ADPS1 | ADPS0 |
		-------------------------------------------------------------
			ADEN --> ADC Enable
			ADCS --> ADC Start Conversion. Set to '1' before starting conversion. It is cleared to '0' by hardware when finishes. It takes 13 clock ticks or 25 if it's the first conversion
			ADFR --> ACD AutoTrigger. When is '1' Free Running Mode is activated.
			* --> In some micros it is called ADATE. When is '1' enables SFIOR register to control more functions than ADFR bit can do
			ADIF --> ADC Interrupt Flag. Is set to '1' by hardware when conversion is done and registers are updated
			ADIE --> ADC Interrupt Enable
			ADPS2:0 --> ADC Prescaler Select Bits. Division factor goes from '2' up to '128'. To set prescaler '2' the three bits must be '0'
	ADCL/ADCH REGISTERS -> ADC Data Registers
		-------------------------------------------------------
		|  15  |  14  |  13  |  12  |  11  |  10  |  9  |  8  | << ADCH
		|------|------|------|------|------|------|-----|-----|
		|  7   |  6   |  5   |  4   |  3   |  2   |  1  |  0  | << ADCL
		-------------------------------------------------------
			--> These two registers store the result of the ADC conversion. The total size of the registers is 16 bits (8 bits each) and the resolution of the result is 10 bits, so 6 bits are not used.
			--> By changing ADLAR bit in ADMUX register the bits of the result can be displaced to MSB or LSB
	SFIOR --> Special Function I/O  Register
		-----------------------------------------------------------
		|   7   |   6   |   5   |  4  |  3   |  2  |  1   |   0   |
		|-------|-------|-------|-----|------|-----|------|-------|
		| ADTS2 | ADTS1 | ADTS0 |  -  | ACME | PUD | PSR2 | PSR10 |
		-----------------------------------------------------------
			ADTS2:0 --> Only if ADATE exists and it value is '1', these bits determine the trigger source of ADC conversor. Free Running Mode is active when these bits are '0'
	*/

/* << TRUTH TABLE: IR SENSOR >>
--------------------------
| S1 | S0 | ADMUX |--> m |
%%%%%%%%%%%%%%%%%%%%%%%%%%
| 0  | 0  | 0 | 0 |--> 0 |
--------------------------
| 0  | 0  | 0 | 1 |--> 1 |
--------------------------
| 0  | 0  | 1 | 0 |--> 2 |
--------------------------
| 0  | 0  | 1 | 1 |--> 3 |
%%%%%%%%%%%%%%%%%%%%%%%%%%
| 0  | 1  | 0 | 0 |--> 4 |
--------------------------
| 0  | 1  | 0 | 1 |--> 5 |
--------------------------
| 0  | 1  | 1 | 0 |--> 6 |
--------------------------
| 0  | 1  | 1 | 1 |--> 7 |
%%%%%%%%%%%%%%%%%%%%%%%%%%
| 1  | 0  | 0 | 0 |--> 8 |
--------------------------
| 1  | 0  | 0 | 1 |--> 9 |
--------------------------
| 1  | 0  | 1 | 0 |--> 10|
--------------------------
| 1  | 0  | 1 | 1 |--> 11|
%%%%%%%%%%%%%%%%%%%%%%%%%%
| 1  | 1  | 0 | 0 |--> 12|
--------------------------
| 1  | 1  | 0 | 1 |--> 13|
--------------------------
| 1  | 1  | 1 | 0 |--> 14|
--------------------------
| 1  | 1  | 1 | 1 |--> 15|
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
| X  | X  | 1 | 0 | 0 |--> 16|
------------------------------
*/

/* << TRUTH TABLE: ADMUX >>
-------------------------
| ADMUX | MUX | --> ADC |
%%%%%%%%%%%%%%%%%%%%%%%%%
|  0 0  |  1X | --> A0  |
-------------------------
|  0 1  |  1Y | --> A1  |
-------------------------
|  1 0  |  2X | --> A2  |
-------------------------
|  1 1  |  2Y | --> A3  |
-------------------------
| 1 0 0 |  -  | --> A4  |
-------------------------

*/

/* << PWM REGISTERS >>
	_____________	   _____________
	|		|	   |		   |
	|    '1'	|   '0'  |	   	   |
______|		|________|           |_
	<----------><-------->
		%on	   %off
	<-------------------->
		     T
     While 't' is on %on interval, the bit value in the register will be set
     to '1'. Otherwise it will be cleared. Each bit of the registers is refered
     to one motor only, that's why 3 bytes (17 bits used) are needed.

     PWMS --> PWM State register: keeps the cycle possition of the first 8 DC
     		  motors.
*/

#include "Arduino.h"
#include <SPI.h>

#define NUM_SENSORES 17
#define NUM_MOTORES 17
#define MAX_DISCRETIZATION 50
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
// Sensors:
volatile uint16_t sensor[NUM_SENSORES] = {0};	// To keep sensor value
volatile bool s[2] = {0};	// To keep external Mux state
volatile uint8_t m = 0;	// To keep sensor info. See "TRUTH TABLE: IR SENSOR"
// Actuators:
volatile uint8_t t = 0;	// To keep discretization counter
volatile uint8_t vel_motor[NUM_MOTORES] = {0};	// To keep motor velocity
volatile uint32_t PWMS = 0;	// Register 'PWM State'
uint8_t out_vel[5] = {0};

//Auxiliary variables
long a = 0,b = 0, c = 0 , d = 0;

void setup(){
	Serial.begin(9600);
	Serial.println("Iniciating...");
	cli();	// Global interruptions disabled

// SPI: Digital pin configuration
	DDRB |= _BV(PB2);	// set digital pin 10 as OUTPUT

// SPI: transaction configuration
	SPI.begin();	//SPI communication is initiated
	SPI.beginTransaction(SPISettings(F_CPU,MSBFIRST,SPI_MODE0));
		//SPI parameters are set according hardware specifications
	SPSR |= _BV(SPI2X);	// Doubles speed transmission

// External MUX configuration for first sensor reading
	DDRD |= _BV(PINS0) | _BV(PINS1);	// pinMode(...)
	PORTD = 0;	// digitalWrite(PIN0-PIN7 , LOW);

// ADC registers configuration
	ADMUX |= _BV(REFS0);	// Set bit to configure reference voltage

      DIDR0 |= _BV(ADC4D) | _BV(ADC3D) | _BV(ADC2D) | _BV(ADC1D) | _BV(ADC0D);
      // Disable digital features

	ADCSRA = 0;	// Both registers cleared
	ADCSRB = 0;

	ADCSRA |= _BV(ADEN) | _BV(ADIE);
      // Enabled ADC. Free running mode activated. Interruptions activated
	ADCSRA |= MASK_DIV32;
      // Velocity set up to 500 kHz --> Div = 32 --> F = 16M/32 = 500 kHz
	ADCSRA |= _BV(ADSC);	// Start Conversion

	sei();	// Global interruptions enabled
}

void loop(){

	if (Serial.available() > 0) {
		out_vel[0] = Serial.parseInt();
	}

	PORTB &= ~_BV(PB2);	// Latch pin on Slave cleared
	SPI.transfer(out_vel, sizeof(out_vel));	// Data transmission via SPI
	// HOW DOES IT WORK?
	// - The function transfers each byte separately. Most signficant byte
	//   (byte number 4) goes to the first Shift Register. Least significant
	//   byte (byte number 0) goes to the last Shift Register of the cascade.
	PORTB |= _BV(PB2);	// Latch pin on Slave set

	Serial.print("out_vel = ");
	Serial.println(out_vel[0],BIN);

/* << Sensor test code >>
	b = millis();
	if(b - a > 1000){
		a = millis();
            cli();
		for(int i=0; i<NUM_SENSORES; i++){
			Serial.print("Sensor[");
			Serial.print(i);
			Serial.print("] = ");
			Serial.print(sensor[i]);
			Serial.print("\n");
		}
		Serial.print("\n");
            sei();
	}
*/

}

// ADC interruption
ISR(ADC_vect){

// VALUE ASSIGNMENT
	sensor[m] = ADCL | (ADCH << 8);	// Value assignment
      if(++m == (NUM_SENSORES)) m = 0;    // m counter update

// INTERNAL MULTIPLEXER CONFIGURATION. See "TRUTH TABLE: IR SENSOR / ADMUX"
	ADMUX &= ~MASK_CLEAR_MUXBITS;	// ADMUX register 4 last bits are cleared
	ADMUX |= m & (MASK_GET_BIT1 | MASK_GET_BIT0);	// Last two m bits are copied to ADMUX
	// HOW DOES IT WORK?
	// 1.	   0000 0001 --> MASK_GET_BIT0
	//	OR 0000 0010 --> MASK_GET_BIT1
	// 	  -----------
	// 2.	   0000 0011 --> (MASK_GET_BIT0 | MASK_GET_BIT1)
	//   AND XXXX XXXX --> m
	//	  -----------
	//	   0000 00XX --> 2 last values isolated and ready to be copied

      // The last sensor has direct conection to ADC4 (no external mux)
      if(m == 16){
            ADMUX &= ~MASK_CLEAR_MUXBITS;	// ADMUX register 4 last bit are cleared
            ADMUX |= _BV(MUX2) ;	// ADC4 is selected
      }

// EXTERNAL MULTIPLEXER CONFIGURATION. See "TRUTH TABLE: IR SENSOR"
	if (!m % 4) {
		PORTD = 0;	// PORTD register is cleared
		s[0] = m & MASK_GET_BIT2;	// Bit nº2 of m is copied to S0
		s[1] = m & MASK_GET_BIT3; 	// Bit nº3 of m is copied to S1
		// HOW DOES IT WORK? (f.e: S0 demonstration)
		// 1. m = XXXX XXXX --> bit 2 is the value desired for S0
		// 2.     XXXX XXXX --> m
		//	AND 0000 0100 --> MASK_GET_BIT2
		//       -----------
		//	    0000 0X00 --> S0 ('0' or '1' the only possible values)
		PORTD |= (s[0] << PINS0) | (s[1] << PINS1);	// PORTD is updated
	}

// PWM REGULATION
	// PWMS Register actualization
	(vel_motor[0] < t) ? (PWMS |= _BV(0)) : (PWMS &= ~_BV(0)) ;
	(vel_motor[1] < t) ? (PWMS |= _BV(1)) : (PWMS &= ~_BV(1)) ;
	(vel_motor[2] < t) ? (PWMS |= _BV(2)) : (PWMS &= ~_BV(2)) ;
	(vel_motor[3] < t) ? (PWMS |= _BV(3)) : (PWMS &= ~_BV(3)) ;
	(vel_motor[4] < t) ? (PWMS |= _BV(4)) : (PWMS &= ~_BV(4)) ;
	(vel_motor[5] < t) ? (PWMS |= _BV(5)) : (PWMS &= ~_BV(5)) ;
	(vel_motor[6] < t) ? (PWMS |= _BV(6)) : (PWMS &= ~_BV(6)) ;
	(vel_motor[7] < t) ? (PWMS |= _BV(7)) : (PWMS &= ~_BV(7)) ;
	(vel_motor[8] < t) ? (PWMS |= _BV(8)) : (PWMS &= ~_BV(8)) ;
	(vel_motor[9] < t) ? (PWMS |= _BV(9)) : (PWMS &= ~_BV(9)) ;
	(vel_motor[10] < t) ? (PWMS |= _BV(10)) : (PWMS &= ~_BV(10)) ;
	(vel_motor[11] < t) ? (PWMS |= _BV(11)) : (PWMS &= ~_BV(11)) ;
	(vel_motor[12] < t) ? (PWMS |= _BV(12)) : (PWMS &= ~_BV(12)) ;
	(vel_motor[13] < t) ? (PWMS |= _BV(13)) : (PWMS &= ~_BV(13)) ;
	(vel_motor[14] < t) ? (PWMS |= _BV(14)) : (PWMS &= ~_BV(14)) ;
	(vel_motor[15] < t) ? (PWMS |= _BV(15)) : (PWMS &= ~_BV(15)) ;
	(vel_motor[16] < t) ? (PWMS |= _BV(16)) : (PWMS &= ~_BV(16)) ;
	// HOW DOES IT WORK?
	// 1. velMotor[i] keeps the moment where the duty cycle falls from '1'
	//	to '0'. Variable 't' represents the time.
	// 2. While the time doesn't reach velMotor[i], that motor will be ON and
	//	bit numer 'i' of the register will be set to '1'.
	// 3. When time reaches velMotor[i], bit 'i' will be cleared to turn off
	//	the motor.
	// 4. When the time 't' reaches the period, the process restarts.

	// Discretization counter actualization
	if(++t > MAX_DISCRETIZATION) t = 0;

// ADC RESTART
	ADCSRA |= _BV(ADSC);	// Start next conversion
}
