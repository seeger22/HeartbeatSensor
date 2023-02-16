#include <Arduino.h>
#include <Wire.h>
#include <time.h>
#include <math.h>

/*
------------------------------------------------
EMBEDDED CHALLENGE SP22
NEW YORK UNIVERSITY TANDON SCHOOL OF ENGINEERING
JIAKAI "SEEGER" ZOU
------------------------------------------------

COMPONENTS:
- Adafruit Playground Classic (8MHz)
- Honeywell MPRLS0300YG00001BB Pressure Sensor (0 - 300mmHg)
- Standard Blood Pressure Cuff
- Silicone tubing

Demonstration video avaliable.
*/


// device specific variables
#define PSENSOR_ADDR 0x18
#define Output_max 3774874
#define Output_min 419430
#define P_max 300

/* Seeger - 05/15/22:
1. A timer is used for measuring time elapsed from the start of the oscillations till the end. The data is used to calculate heart rate.
  - I have also experimented with using the same timer to replace the delay functions, but there are inconsistencies in its readings (I believe the timer is slightly faster), and 
  it messes with the sensor readings in readPressure().
2. A large part of my time was spent on developing a GUI with GTK. However this is ultimately not implemented due to the same timing issues I experienced above. It required further
synchronization.
  - Another issue is that, the nature of this project is meant to be purely ran on the microcontroller. This implementation somewhat goes against that idea.
3. Note that from the demonstration video, a pull up resistor is not used in this case, due to the specific schematic of our microcontroller (Adafruit Playground Classic). Or else
the sensor will require an external pull up resistor on other boards.
5. The biggest problem I encountered during this project was to measure the amount of oscillations happening during the deflation, and calculating the heart rate through it.
  - I noticed that measuring data on a high interval, given the inconsistent experiment environment, resulted in a high number of minimum oscillations.
  - The method I took was to find both the number of local minimums and maximums and take the average of them to find the heart rate.

Overall, the project was successful in its ability to communicate with the pressor sensor via I2C, converting and calculating the pressure measurements, finding the deflation rate
and inform the user, and calculate the systolic, diastolic bp values and the heart rate using the measured values and algorithms. The project is not, however, practical as a consumer
product as it has not been tested to be accurate nor precise. Further examinations is required.
*/

void setup() {
  Wire.begin();
  Serial.begin(9600);
  
  // Timer configuration
  TCCR3A = 0x00; // disconnect pins, simple timer, count up to OCR3A
  TCCR3B = 0b00001011; // 64 prescalar
  TCCR3C = 0x00;
  OCR3A = 125; // overflow every 1 ms
  OCR3B = 0x00;
  TIMSK3 = 0b00000010; // enable overflow interrupt
}

uint32_t global_timer_count; // in ms

ISR(TIMER3_COMPA_vect){
  global_timer_count++;
}

float readPressure(){
  uint32_t curr_reading; // variable to store the 24 bit data count
  float curr_pressure; // final return pressure

  uint32_t status=255;
  uint32_t ReturnByte1=255;
  uint32_t ReturnByte2=255;
  uint32_t ReturnByte3=255; // split bits

  Wire.beginTransmission(PSENSOR_ADDR);
  Wire.write(0xAA); // command
  Wire.write(0x00);
  Wire.write(0x00);
  Wire.endTransmission(false);
  delay(5); // the use of timer here seems to cause timing issues, not getting valid status

  Wire.requestFrom(PSENSOR_ADDR,4,true);
  status = Wire.read();
  ReturnByte1 = Wire.read();
  ReturnByte2 = Wire.read();
  ReturnByte3 = Wire.read();

  if (status != (1<<6)) {
    return -1.0; // error
  }
  // Preparing data and calculations
  curr_reading = (ReturnByte1 << 16) + (ReturnByte2 << 8) + (ReturnByte3);

  if (curr_reading > Output_min){
    curr_reading -= Output_min;
  }
  else {
    curr_reading = Output_min - curr_reading; // to compensate the off measurements in the beginning
  }
  curr_pressure = curr_reading*300/3355444.0; // transfer formula
  return(curr_pressure);
}

void Measure() {

  // declarations
  global_timer_count = 0;
  // pressure related
  float curr_pressure;
  float prev_pressure = 0.0;
  float target_pressure; // the apex
  float curr_dev;
  float time_dev = 0.0; // used to calculate the deflation rate
  float total_dev = 0.0; // used to calculate global maximum increase (apex)
  float max_dev = 0.0; // the apex (deviation)

  // time related
  int HRate; // heart rate
  int min_count = 0, max_count = 0; // minimum count and maximum count
  int osc_count = 0; // oscillation count
  int local_time_count; // local to this function, different from global_timer_count

  // boolean flags
  bool curr_state;
  bool prev_state = false; // decreasing at start
  bool increasing = true;
  bool decreasing = false;
  
  // INFLATING UP TO 150mmHg
  curr_pressure = readPressure();
  while(curr_pressure < 150.0){
    prev_pressure = curr_pressure;
    curr_pressure = readPressure();
    Serial.print("Keep inflating\tCurrent = ");
    Serial.println(curr_pressure);
  }
  delay(100); // delay to offset the inconsistency when the cuff first reaches 150mmHg

  // DELFATING TO 30mmHg
  Serial.println("**********Start Deflating**********");
  global_timer_count = 0; // reset for start time

  while(curr_pressure > 30.0){
    curr_pressure = readPressure();

    if (curr_pressure < prev_pressure) { curr_state = decreasing; }
    if (curr_pressure > prev_pressure) { curr_state = increasing; }

    curr_dev = fabs(curr_pressure - prev_pressure);
    time_dev += curr_dev;
    if (local_time_count!=0 && local_time_count%20==0){
      if (time_dev > 5) { Serial.println("TOO FAST"); }
      else if (time_dev < 3) { Serial.println("TOO SLOW"); }
      else { Serial.println("GOOD PACE"); }
      time_dev = 0;
    }

    if (curr_state && prev_state) { total_dev += curr_dev; } // if still increasing, add up dev
    else if (curr_state && !prev_state) { // minimum: reset for next possible maximum
      total_dev = 0.0;
      min_count++;
    }
    else if (!curr_state && !prev_state) { total_dev = 0.0; } // decreasing: reset for next possible maximum
    else if (!curr_state && prev_state) {
      if (max_dev < total_dev) {// global maximum found
        max_dev = total_dev;
        target_pressure = curr_pressure;
      }
      max_count++;
      total_dev = 0.0; // reset for next possible maximum
    }

    // update prev values
    prev_pressure = curr_pressure;
    prev_state = curr_state;
    local_time_count++;
    delay(50); // take measurements every 50ms, can be changed (need to also change the remainder calculation at line 151)
    global_timer_count += 50;
  }

  // calculate and output values
  osc_count = (min_count + max_count)/2;

  HRate = (osc_count*60000)/(global_timer_count);
  Serial.print("Heart Rate = ");
  Serial.println(HRate);

  float Systolic = 0.5*target_pressure;
  float Diastolic = 0.7*target_pressure;

  Serial.print("Systolic = ");
  Serial.println(Systolic);
  Serial.print("Diastolic = ");
  Serial.println(Diastolic);

  delay(10000);
}

void loop() {
  Measure();
  exit(0);
}