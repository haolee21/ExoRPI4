// The I2C slave library is from https://github.com/Richard-Gemmell/teensy4_i2c

#include <Arduino.h>
#include "i2c_driver.h"
#include "imx_rt1060_i2c_driver.h"
#include "PWM_valve.h"
#include "Valve.h"
#include "TeensyCommon.h"
// Blink the LED to make sure the Teensy hasn't hung
IntervalTimer blink_timer;
IntervalTimer pwm_timer;
unsigned long previousMicro;//micros only last for less than 2 hrs




volatile bool led_high = false;
void blink_isr();

const uint16_t slave_address = 0x002D;
I2CSlave& slave = Slave;
void after_receive(int size);

const int duty_unit = 500; //in micro second, the valve PWM period is 200 Hz
const unsigned long period_time =duty_unit*100;

const int num_pwm = PWM_VAL_NUM;
const int num_val = SW_VAL_NUM;

int pwm_idx=0;//this is for record the valve id, not the pin id (e.g., pwm1 is pin 8)
int val_idx=0;



// create two lists for pwm_valves, pwm_list is the original list, pwm_sort_list is after sorting
PWM_valve pwm_list[num_pwm] = {
  PWM_valve(pwm_idx++,TEENSY_PWM0, duty_unit),
  PWM_valve(pwm_idx++,TEENSY_PWM1, duty_unit),
  PWM_valve(pwm_idx++,TEENSY_PWM2, duty_unit),
  PWM_valve(pwm_idx++,TEENSY_PWM3, duty_unit),
  PWM_valve(pwm_idx++,TEENSY_PWM4, duty_unit),
  PWM_valve(pwm_idx++,TEENSY_PWM5, duty_unit),
};
PWM_valve pwm_sort_list[num_pwm];

// normal valve list
Valve valve_list[num_val] = {
    Valve(TEENSY_SW0),
    Valve(TEENSY_SW1),
    Valve(TEENSY_SW2),
    Valve(TEENSY_SW3),
    Valve(TEENSY_SW4),
    Valve(TEENSY_SW5),
    Valve(TEENSY_SW6),
    Valve(TEENSY_SW7)};

// Double receive buffers to hold data from master.
const size_t slave_rx_buffer_size = num_pwm+num_val;
uint8_t slave_rx_buffer[slave_rx_buffer_size] = {};
uint8_t slave_rx_buffer_2[slave_rx_buffer_size] = {};
volatile size_t slave_bytes_received = 0; //when a variable is not used in the main thread, we need to declared it as volatile to avoid it got deleted when optimized
                                          // it only get modified in IRS

void log_message_received();




// Called by the I2C interrupt service routine.
// This method must be as fast as possible.
// Do not perform IO in it.
void after_receive(int size) {
  // This is the only time we can guarantee that the
  // receive buffer is not changing.
  // Copy the content so we can handle it in the main loop.
  if (!slave_bytes_received) {
    memcpy(slave_rx_buffer_2, slave_rx_buffer, size);
    slave_bytes_received = size;
  }
  // else ignore this message because the main loop hasn't
  // handled the previous one yet.


}

void log_message_received() {
  if (slave.has_error()) {
    if (slave.error() == I2CError::buffer_overflow) {
      Serial.println("App: Buffer Overflow. (Master sent too many bytes.)");
    } else {
      Serial.println("App: Unexpected error");
    }
  }
//  Serial.print("App: Slave received ");
//  Serial.print(slave_bytes_received);
//  Serial.print(" bytes: ");
//  for(size_t i=0; i<slave_bytes_received; i++) {
//     Serial.print(slave_rx_buffer_2[i]);
//     Serial.print(", ");
//  }
//  Serial.println();

}

void blink_isr() {
  led_high = !led_high;
  digitalWrite(LED_BUILTIN, led_high);
}
void heapify(PWM_valve arr[], int n, int i)
{
  int largest = i; // Initialize largest as root
  int l = 2 * i + 1; // left = 2*i + 1
  int r = 2 * i + 2; // right = 2*i + 2

  // If left child is larger than root
  if (l < n && arr[l] > arr[largest])
    largest = l;

  // If right child is larger than largest so far
  if (r < n && arr[r] > arr[largest])
    largest = r;

  // If largest is not root
  if (largest != i)
  {
    std::swap(arr[i], arr[largest]);

    // Recursively heapify the affected sub-tree
    heapify(arr, n, largest);
  }
}
void heapSort(PWM_valve arr[], int n)
{
  // Build heap (rearrange array)
  for (int i = n / 2 - 1; i >= 0; i--)
    heapify(arr, n, i);

  // One by one extract an element from heap
  for (int i = n - 1; i > 0; i--)
  {
    // Move current root to end
    std::swap(arr[0], arr[i]);

    // call max heapify on the reduced heap
    heapify(arr, i, 0);
  }
}
void decode_msg()
{
//  Serial.print("set duty:");
  for (int i = 0; i < num_pwm; i++) {
    pwm_list[i].setDuty((int)slave_rx_buffer_2[i]);

  }
  for (int i = num_pwm; i < num_pwm + num_val;i++){
    if((bool)slave_rx_buffer_2[i]){
      valve_list[i-num_pwm].on();
    }
    else{
      valve_list[i-num_pwm].off();
    }
    
  }


  // memcpy(pwm_sort_list, pwm_list, sizeof(PWM_valve) * num_pwm);
  // heapSort(pwm_sort_list, num_pwm);
}
// reference from https://www.geeksforgeeks.org/heap-sort/


void PWM_valve_on_off()
{

  int time_diff = (int)(micros()-previousMicro);

  for(int i =0;i<num_pwm;i++){
    if(pwm_list[i].GetDuty()*duty_unit>=time_diff){
      // Serial.println("on");
      pwm_list[i].on();
    }
    else if(pwm_list[i].GetDuty()*duty_unit<time_diff){
      pwm_list[i].off();
      // Serial.println("off");
    }
  }


  // for (int i = 0; i < num_pwm; i++)
  // {
  //   pwm_sort_list[i].on();
  // }
  // int pre_duty = 0;
  // for (int i = 0; i < num_pwm; i++)
  // {
  //   int cur_sleep_t = pwm_sort_list[i].cal_off_t(pre_duty);
  //   delayMicroseconds(duty_unit*cur_sleep_t);
  //   pwm_sort_list[i].off();

  // }

  if(time_diff >= period_time){
    previousMicro = micros();
  
  }
  
}
void setup() {
  // Turn the LED on
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, true);

  // Create a timer to blink the LED
  blink_timer.begin(blink_isr, 500000);

  //create pwm_valve list
  memcpy(pwm_sort_list, pwm_list, sizeof(PWM_valve) * num_pwm);

  previousMicro = micros();
  pwm_timer.begin(PWM_valve_on_off, duty_unit  ); //will check the valve on/off every 1 unit of duty time

  // Configure I2C Slave
  slave.after_receive(after_receive);
  slave.set_receive_buffer(slave_rx_buffer, slave_rx_buffer_size);

  // Enable the slave
  slave.listen(slave_address);

  // Enable the serial port for debugging
  Serial.begin(115200);
  Serial.println("Started");



}

void loop() {
  if (slave_bytes_received) {
    // Handle the message.
    log_message_received();

    // Clear slave_bytes_received to signal that we're ready for another message
    slave_bytes_received = false;
    //decode the message
    decode_msg();
    
  }


  // We could receive multiple message while we're asleep.
  // This example is modelling an application where it's Ok
  // to drop messages.
  // delayMicroseconds(50);
}