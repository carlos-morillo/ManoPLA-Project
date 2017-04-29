
#define NUM_SENSORES 17

//Variable declaration
int sensor[NUM_SENSORES] = {0};
bool S[2] = {0};
uint8_t m = 0;
int pinS0 = 4, pinS1 = 5;

//Auxiliary variables
long a = 0,b = 0;

void setup(){
	Serial.begin(9600);
//  External MUX configuration for first sensor reading
	digitalWrite(pinS0, S[0]);
	digitalWrite(pinS1, S[1]);

// ADC registers configuration
	ADMUX |= _BV(REFS0);	// Set bit to configure reference voltage

	ADCSRA = 0;	// Both registers cleared
	ADCSRB = 0;

	ADCSRA |= _BV(ADEN) | _BV(ADATE) | _BV(ADIE) | _BV(ADPS2) | _BV(ADPS0);
	// Enabled ADC
	// Free running mode activated
	// Interruptions activated
	// Velocity set up to 500 kHz --> Div = 32 --> F = 16M/32 = 500 kHz
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
	sensor[m++] = ADCL | (ADCH << 8);

}