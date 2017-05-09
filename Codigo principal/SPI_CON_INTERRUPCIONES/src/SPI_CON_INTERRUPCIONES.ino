#include <Arduino.h>

#include <SPI.h>	// SPI communication
#include <avr/io.h>
#include <avr/interrupt.h>

// Data is divided in bytes where motors are controlled by 2 bits each. Each motor has been called with a letter (D,C,B,A) in MSB order.
//
// BYTE--> XX XX XX XX (each pair of bits corresponds to one motor)
// MOTOR->  D  C  B  A
// 	XX = 01 or 10 --> rotation clockwise/counter-clockwise respectively
// 	XX = 00 or 11 --> stop

#define MOTOR_A_RIGHT 0x01 	//set bit 0
#define MOTOR_A_LEFT 0x02 	//set bit 1
#define MOTOR_B_RIGHT 0x04 	//set bit 2
#define MOTOR_B_LEFT 0x08 	//set bit 3
#define MOTOR_C_RIGHT 0x10 	//set bit 4
#define MOTOR_C_LEFT 0x20 	//set bit 5
#define MOTOR_D_RIGHT 0x40 	//set bit 6
#define MOTOR_D_LEFT 0x80 	//set bit 7
#define MOTOR_A_STOP 0x03 	//set bit 0 y 1
#define MOTOR_B_STOP 0x0C 	//set bit 2 y 3
#define MOTOR_C_STOP 0x30 	//set bit 4 y 5
#define MOTOR_D_STOP 0xC0 	//set bit 6 y 7

//Prescaler clock for ADC in octal base
#define ADC_CLOCK_DIV2 0x0
#define ADC_CLOCK_DIV4 0x2
#define ADC_CLOCK_DIV8 0x3
#define ADC_CLOCK_DIV16 0x4
#define ADC_CLOCK_DIV32 0x5
#define ADC_CLOCK_DIV64 0x6
#define ADC_CLOCK_DIV128 0x7

int latchPin = 10 ;	// SS (SlaveSelect) pin used to latch the output on 74HC595
int pinMux_A = 3;	// Selection pin of the demultiplexer
int pinMux_B = 4;
volatile int n = 0;	// Auxiliary variable
long tic = 0;
long toc = 0;
volatile uint8_t out_buffer[5] = {};
volatile uint16_t value_ADC = 0;

// Initial parameters are set in this function:
void setup(){
	Serial.begin(9600);	//Serial port opened
	pinMode(latchPin, OUTPUT);	//pinMode for latchPin is defined
	pinMode(pinMux_A, OUTPUT);	//PinMode for demultiplexer outputs
	pinMode(pinMux_B, OUTPUT);
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

	ADCSRA |= _BV(ADC_CLOCK_DIV2);	//Prescaler selection
	digitalWrite(pinMux_A, HIGH);
	digitalWrite(pinMux_B, HIGH);
	Serial.print("ADMUX = ");
	Serial.print(ADMUX,BIN);
	Serial.print("\n");

	Serial.print("ADCSRA = ");
	Serial.print(ADCSRA,BIN);
	Serial.print("\n");


	out_buffer[0] |= MOTOR_A_RIGHT | MOTOR_B_STOP;
	out_buffer[2] |= MOTOR_A_RIGHT | MOTOR_C_STOP;

	SPI.begin();	//SPI communication is initiated
	SPI.beginTransaction(SPISettings(F_CPU,MSBFIRST,SPI_MODE0));	//SPI parameters are set according hardware specifications
	SPSR |= _BV(SPI2X);
	Serial.print("\nSPCR = ");
	Serial.print(SPCR,BIN);
	Serial.print("\nSPSR = ");
	Serial.print(SPSR,BIN);

}

void loop(){
	// Serial.print("\nBUFFER (BIN): ");
	// for(int i=0; i<5; i++){
	//     Serial.print(buf[i], BIN);
	//     Serial.print(" ");
	// }

	// Serial.print("\nBUFFER (HEX): ");
	// for(int i=0; i<5; i++){
	//     Serial.print(buf[i],HEX);
	//     Serial.print(" ");
	// }
	// Serial.print("\n");


	digitalWrite(latchPin, LOW) ;	//latchpin low to begin data transmission
	tic = micros();
	// Metodo 1:
	SPI.transfer(out_buffer, sizeof(out_buffer));
	// Metodo 2:
	// for(int i=0; i<5; i++){
	//     SPI.transfer(buf[i]);
	// }
	// Metodo 3:
	// SPI.transfer(buf[0]);
	// SPI.transfer(buf[1]);
	// SPI.transfer(buf[2]);
	// SPI.transfer(buf[3]);
	// SPI.transfer(buf[4]);
	toc = micros();
	digitalWrite(latchPin, HIGH) ;	//latchpin high to end data transmission


	// Serial.print("\nTiempo de ejecución:");
	// Serial.print(toc - tic);
	// Serial.print(" us\n");

	delay(2000);

}

// ADC Interruption
ISR(ADC_vect){
	value_ADC = ADC;	//assignation of the result of ADC stored in ADCH and ADCL registers
	n++;
}
