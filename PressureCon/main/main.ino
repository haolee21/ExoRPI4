// This script records the pressure, piston position, force, and valve conditions during random impulses
// 1. an external force will randomly apply to the force sensor
// 2. sensors record the data with 1000Hz sampling rate
// 3. After 10 sec, recording, arduino sends the data back to pc, waiting for reset
// 4. Timer0 controls the valves
//    Timer1 record the data
// analog input pins
int F_in = A1;     // Force                   
int P1_in = A2;    // Pressure 1
int P2_in = A3;    // Pressure 2
int Pos_in = A0;   // Position

// digital input pins
int Val1_in=13;
int Val2_in=14;

// output pins
int Val1_out=51;
int Val2_out=52;
// PWM control
unsigned val1_on_duty;
unsigned val2_on_duty;
const unsigned TIME_UNIT=100;

// record arrays
const unsigned DATALEN = 1000*10;

int *force_mem = new int(DATALEN);

int *p1_mem = new int(DATALEN);
int *p2_mem= new int(DATALEN);
int *pos_mem = new int(DATALEN);
int *val1_mem= new int(DATALEN);
int *val2_mem=new int(DATALEN);

unsigned int recIdx; //index of the array




bool fullTestFlag; //we only run once after pressing the reset buttom
bool sendDataFlag; //when true, send all the data back

void setup()
{

    recIdx=0;
    fullTestFlag=true;
    sendDataFlag=false;

    val1_on_duty=5;
    val2_on_duty=0;
    //clear the memory
    memset(force_mem,0,DATALEN);
    memset(p1_mem,0,DATALEN);
    memset(p2_mem,0,DATALEN);
    memset(pos_mem,0,DATALEN);
    memset(val1_mem,0,DATALEN);
    memset(val2_mem,0,DATALEN);


    //config output pins (analog read pins does not need to be configured)
    pinMode(Val1_out,OUTPUT);
    pinMode(Val2_out,OUTPUT);  

    digitalWrite(Val1_out,LOW);
    digitalWrite(Val2_out,LOW);
    
    pinMode(Val1_in,INPUT);
    pinMode(Val2_in,INPUT);
    

    Serial.begin(115200);          //  setup serial


    //Timer setup
    // TIMER 1 for interrupt frequency 1000 Hz
    cli(); // stop interrupts
    TCCR1A = 0; // set entire TCCR1A register to 0
    TCCR1B = 0; // same for TCCR1B
    TCNT1  = 0; // initialize counter value to 0
    // set compare match register for 1000 Hz increments
    OCR1A = 15999; // = 16000000 / (1 * 1000) - 1 (must be <65536)
    // turn on CTC mode
    TCCR1B |= (1 << WGM12);
    // Set CS12, CS11 and CS10 bits for 1 prescaler
    TCCR1B |= (0 << CS12) | (0 << CS11) | (1 << CS10);
    // enable timer compare interrupt
    TIMSK1 |= (1 << OCIE1A);
    sei(); // allow interrupts

    // TIMER 0 for interrupt frequency 100.16025641025641 Hz:
    cli(); // stop interrupts
    TCCR0A = 0; // set entire TCCR0A register to 0
    TCCR0B = 0; // same for TCCR0B
    TCNT0  = 0; // initialize counter value to 0
    // set compare match register for 100.16025641025641 Hz increments
    OCR0A = 155; // = 16000000 / (1024 * 100.16025641025641) - 1 (must be <256)
    // turn on CTC mode
    TCCR0B |= (1 << WGM01);
    // Set CS02, CS01 and CS00 bits for 1024 prescaler
    TCCR0B |= (1 << CS02) | (0 << CS01) | (1 << CS00);
    // enable timer compare interrupt
    TIMSK0 |= (1 << OCIE0A);
    sei(); // allow interrupts//

}




// the loop function runs over and over again until power down or reset
ISR(TIMER1_COMPA_vect)
{
    if(recIdx<DATALEN){
      force_mem[recIdx]=analogRead(F_in);
      p1_mem[recIdx]=analogRead(P1_in);
      p2_mem[recIdx]=analogRead(P2_in);
      pos_mem[recIdx]=analogRead(Pos_in);
      val1_mem[recIdx]=digitalRead(Val1_in);
      val2_mem[recIdx++]=digitalRead(Val2_in);
        
    }
    else{
      sendDataFlag=true;
      
    }
	
}




ISR(TIMER0_COMPA_vect){
   if(fullTestFlag){

    int long_pin,short_pin; //pin with higher duty and lower
    int high_duty,low_duty;
    if(val1_on_duty>=val2_on_duty){
      long_pin=Val1_out;
      short_pin=Val2_out;
      high_duty=val1_on_duty;
      low_duty=val2_on_duty;
    }
    else{
      long_pin=Val2_out;
      short_pin=Val1_out;
      high_duty=val2_on_duty;
      low_duty=val1_on_duty;
      
    }
    digitalWrite(long_pin,HIGH);
    delayMicroseconds(high_duty-low_duty);
    digitalWrite(short_pin,HIGH);
    delayMicroseconds(val2_on_duty);

    digitalWrite(long_pin,LOW);
    digitalWrite(short_pin,LOW);
    
    
   }
}
void Send16BitIntArray(int* array,unsigned len){
  for(unsigned i=0;i<len;i++){
    Serial.write((array[i]>>8)&0xFF); //write the higher byte
    Serial.write((array[i]&0xFF)); //write the lower byte
    
  }
  Serial.write('\t');
  
}

void SendData(){
  //TODO: finish the serial sending 
  fullTestFlag=false; //prevent the main loop from sending any data back
  Send16BitIntArray(force_mem,DATALEN);
  
  Send16BitIntArray(force_mem,DATALEN);
  Send16BitIntArray(p1_mem,DATALEN);
  Send16BitIntArray(p2_mem,DATALEN);
  Send16BitIntArray(pos_mem,DATALEN);
  Send16BitIntArray(val1_mem,DATALEN);
  Send16BitIntArray(val2_mem,DATALEN);
  Serial.println("Done\n");
  Serial.write('\n');
  
  
  
}

void loop()
{

	if (fullTestFlag)
	{
  
    if(sendDataFlag){
      SendData();   
    }


        

  }


}
