int F_pin = A1;     // Force                   
int P1_pin = A2;    // Pressure 1
int P2_pin = A3;    // Pressure 2
int Pos_pin = A4;   // Position
bool read;  // Start reading or not

void setup()
{

    pinMode(F_pin, INPUT); //initialize INPUTs
    pinMode(__,OUTPUT)     //initialize OUTPUT to computer?
    Serial.begin(9600);          //  setup serial

}

// TIMER 1 for interrupt frequency 2000 Hz:
cli(); // stop interrupts
TCCR1A = 0; // set entire TCCR1A register to 0
TCCR1B = 0; // same for TCCR1B
TCNT1  = 0; // initialize counter value to 0
// set compare match register for 2000 Hz increments
OCR1A = 7999; // = 16000000 / (1 * 2000) - 1 (must be <65536)
// turn on CTC mode
TCCR1B |= (1 << WGM12);
// Set CS12, CS11 and CS10 bits for 1 prescaler
TCCR1B |= (0 << CS12) | (0 << CS11) | (1 << CS10);
// enable timer compare interrupt
TIMSK1 |= (1 << OCIE1A);
sei(); // allow interrupts

// the loop function runs over and over again until power down or reset
ISR(TIMER1_COMPA_vect)
{
	read = true;
}

void loop()
{

	if (read)
	{

        int F_data=analogRead(F_pin);
        Serial.print(“Force reading=“);
        Serial.println(F_data);

    }


}