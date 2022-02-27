IntervalTimer senTimer;
IntervalTimer actTimer;
// This script records the pressure, piston position, force, and valve conditions during random impulses
// 1. an external force will randomly apply to the force sensor
// 2. sensors record the data with 1000Hz sampling rate
// 3. After 10 sec, recording, arduino sends the data back to pc, waiting for reset
// 4. Timer0 controls the valves
//    Timer1 record the data
// analog input pins
const int F_in = A1;     // Force                   
const int P1_in = A2;    // Pressure 1
const int P2_in = A3;    // Pressure 2
const int Pos_in = A0;   // Position

const int LED = LED_BUILTIN;

// output pins
const int Val1_out=7;
const int Val2_out=8;

// valve condition
volatile bool val1_flag;
volatile bool val2_flag;
// PWM control
volatile unsigned val1_on_duty;
volatile unsigned val2_on_duty;
const unsigned TIME_UNIT=500;

// record arrays
const unsigned DATALEN = 1000*20;
const unsigned CLEAR_BUF_LEN=4000;
unsigned buf_clear_count;


volatile int16_t force_mem[DATALEN];
volatile int16_t p1_mem[DATALEN];
volatile int16_t p2_mem[DATALEN];
volatile int16_t pos_mem[DATALEN];
volatile bool val1_mem[DATALEN];
volatile bool val2_mem[DATALEN];
volatile char extraBuf[DATALEN*2];

volatile unsigned recIdx; //index of the array




volatile bool fullTestFlag; //we only run once after pressing the reset buttom
volatile bool sendDataFlag; //when true, send all the data back

void setup()
{

    recIdx=0;
    fullTestFlag=true;
    sendDataFlag=false;
    buf_clear_count = 0;
    val1_on_duty=0;
    val2_on_duty=0;
    val1_flag=false;
    val2_flag=false;
    //clear the memory
    volatileArrayInit(force_mem,0,sizeof(int16_t)*DATALEN);
    volatileArrayInit(p1_mem,0,sizeof(int16_t)*DATALEN);
    volatileArrayInit(p2_mem,0,sizeof(int16_t)*DATALEN);
    volatileArrayInit(pos_mem,0,sizeof(int16_t)*DATALEN);
    volatileArrayInit(val1_mem,false,sizeof(bool)*DATALEN);
    volatileArrayInit(val2_mem,false,sizeof(bool)*DATALEN);



    //config output pins (analog read pins does not need to be configured)
    pinMode(Val1_out,OUTPUT);
    pinMode(Val2_out,OUTPUT);  
    pinMode(LED,OUTPUT);
    digitalWrite(Val1_out,LOW);
    digitalWrite(Val2_out,LOW);
    digitalWrite(LED,HIGH);


    Serial.begin(1000000);          //  setup serial


    senTimer.begin(SenUpdate,1000);
    actTimer.begin(ActUpdate,50000);

}
void volatileArrayInit(volatile void* dataArray,char value, size_t len){
  volatile char *p = (char*)dataArray;
  while(len-->0){
    *p++=value;
  }
}
void SenUpdate()
{
    if(recIdx<DATALEN){
      force_mem[recIdx]=analogRead(F_in);
      p1_mem[recIdx]=analogRead(P1_in);
      p2_mem[recIdx]=analogRead(P2_in);
      pos_mem[recIdx]=analogRead(Pos_in);
      val1_mem[recIdx]=val1_flag;
      val2_mem[recIdx++]=val2_flag;
        
    }
    else{
      sendDataFlag=true;
      
    }
  
}
void ActUpdate(){
   if(fullTestFlag){

    int long_pin,short_pin; //pin with higher duty and lower
    int high_duty,low_duty;
    volatile bool *highVal_flag, *lowVal_flag;
    if(val1_on_duty>=val2_on_duty){
      long_pin=Val1_out;
      short_pin=Val2_out;
      high_duty=val1_on_duty;
      low_duty=val2_on_duty;
      highVal_flag=&val1_flag;
      lowVal_flag=&val2_flag;
    }
    else{
      long_pin=Val2_out;
      short_pin=Val1_out;
      high_duty=val2_on_duty;
      low_duty=val1_on_duty;
      highVal_flag=&val2_flag;
      lowVal_flag=&val1_flag;
      
    }
    digitalWrite(long_pin,HIGH);
    *highVal_flag=true;
    delayMicroseconds((high_duty-low_duty)*TIME_UNIT);
    digitalWrite(short_pin,HIGH);
    *lowVal_flag=true;
    delayMicroseconds(low_duty*TIME_UNIT);

    digitalWrite(long_pin,LOW);
    digitalWrite(short_pin,LOW);
    val1_flag=false;
    val2_flag=false;
    
   }
}



// the loop function runs over and over again until power down or reset









void Send16BitIntArray(volatile int16_t* dataArray,unsigned len){
  unsigned remain = len;
  unsigned idx=0;
  while(remain>0){
    long cap = (long)Serial.availableForWrite()/(sizeof(int16_t)+2);
    long left = (long)remain-cap;
    if(left<0){
      for(unsigned i=0;i<remain;i++){
        Serial.print(dataArray[i+idx]);
        Serial.print(',');        
      }
      remain = 0;
    }
    else{
      remain = (unsigned)left;
      for(unsigned i=0;i<cap;i++){
        Serial.print(dataArray[i+idx]);
        Serial.print(',');
      }
      idx+=cap;
    }
  }
  delayMicroseconds(100);
  Serial.print('\t');
  Serial.flush();
  
}
void Send16BitIntArray(volatile bool* dataArray,unsigned len){
  unsigned remain = len;
  unsigned idx=0;
  while(remain>0){
    long cap = (long)Serial.availableForWrite()/(sizeof(bool)+2);
    long left = (long)remain-cap;
    if(left<0){
      for(unsigned i=0;i<remain;i++){
        Serial.print(dataArray[i+idx]);
        Serial.print(',');        
      }
      remain = 0;
    }
    else{
      remain = (unsigned)left;
      for(unsigned i=0;i<cap;i++){
        Serial.print(dataArray[i+idx]);
        Serial.print(',');
      }
      idx+=cap;
    }
  }
  delayMicroseconds(100);
  Serial.print('\t');
  Serial.flush();
  
  
}
//void Send8BitsArray(volatile bool* dataArray, unsigned len){
//  for(unsigned i=0;i<len;i++){
//    Serial.print(dataArray[i]);
//    Serial.write((int)dataArray[i]);
//    Serial.write((int)dataArray[i]);
//  }
//  Serial.print('@');
//}

void SendData(){
  //TODO: finish the serial sending 
  fullTestFlag=false; //prevent the main loop from sending any data back
  Send16BitIntArray(p1_mem,DATALEN);
  Serial.flush();
  Send16BitIntArray(p2_mem,DATALEN);
  Serial.flush();
  Send16BitIntArray(force_mem,DATALEN);
  Serial.flush();
  Send16BitIntArray(pos_mem,DATALEN);
  Serial.flush();
  Send16BitIntArray(val1_mem,DATALEN);
  Serial.flush();
  Send16BitIntArray(val2_mem,DATALEN);
  Serial.flush();


  Serial.print('\n');

  digitalWrite(LED,LOW);
  
  
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
