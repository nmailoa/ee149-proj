#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#define LED (0)
#define FORCE_SENSOR (1)
#define BUTTON (3)

Adafruit_BNO055 bno = Adafruit_BNO055();

uint8_t sys, gyroCal, accelCal, magCal = 0;
long time = 0;
boolean button = 0;
boolean sync = 0;
int do_loop = 0;
boolean debug = 0;
byte count = 0;
byte cur_count = 0;
char buff[60] = {0};
int reading_no = 0;
int ax = 0;
int ay = 0;
int az = 0;
int ex = 0;
int ey = 0;
int ez = 0;
boolean force = 0;

imu::Vector<3> accel;
imu::Vector<3> euler;


// Timer ISR to trigger reading
ISR(TIMER1_COMPA_vect){
  do_loop = 1;
  count++;
}


void setup(void){
  // Set to red for calibration
  Bean.setLed(255,0,0);
  
  sync = 0;
  do_loop = 0;
  reading_no = 0;
  count = 0;
  
  pinMode(LED, OUTPUT);
  pinMode(BUTTON, INPUT);
  pinMode(FORCE_SENSOR, INPUT);

  cli();          // disable global interrupts
  TCCR1A = 0;     // set entire TCCR1A register to 0
  TCCR1B = 0;     // same for TCCR1B

  // set compare match register to desired timer count:
  OCR1A = 117;    // 15ms period

  // turn on CTC mode:
  TCCR1B |= (1 << WGM12);

  // Set CS10 and CS12 bits for 1024 prescaler:
  TCCR1B |= (1 << CS10);
  TCCR1B |= (1 << CS12);

  // enable timer compare interrupt:
  TIMSK1 |= (1 << OCIE1A);
  sei();          // enable global interrupts
  
  // Start serial communication
  Serial.begin(57600);
  Serial.println("Orientation Sensor Raw Data Test"); Serial.println("");

  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  delay(1000);

  bno.setExtCrystalUse(true);
  
}


void loop(void){
  if (do_loop){
    // Get data from IMU, force sensor and button
    cur_count = count;
    bno.getCalibration(&sys, &gyroCal, &accelCal, &magCal);
    accel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
    euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    force = digitalRead(FORCE_SENSOR);
    button = digitalRead(BUTTON) | (force << 1);

    // If just got synced
    if (sync == 0 && gyroCal == 3 && accelCal == 3 && magCal == 3){
      buff[0] = char(1 << 7);
      buff[1] = char(sys << 6 | accelCal << 4 | gyroCal << 2 | magCal);
      buff[11] = char(cur_count);
      
      Serial.write((const unsigned char*)buff, 12);
      Serial.flush();
      Bean.setLed(246, 255, 0);
      reading_no = 0;
      
      // Wait for confirmation from PC
      while (Serial.read() != '1');

      // Wait for button press
      Bean.setLed(0,0,255);
      while (digitalRead(BUTTON) != 1);
      while (digitalRead(BUTTON) != 0);
      Bean.setLed(0,255,0);
      
      sync = 1;
    }
    else {
      // If not synced yet
      if (sync == 0){
        // Set calibration packet in buffer
        buff[12*reading_no] = char(1 << 7);
        buff[12*reading_no+1] = char(sys << 6 | accelCal << 4 | gyroCal << 2 | magCal);
        buff[12*reading_no+11] = char(cur_count);

        // Turn on LED based on calibration state
        if (gyroCal == 3 && magCal == 3){
          if (accelCal == 0) Bean.setLed(0,0,0);
          if (accelCal == 1) Bean.setLed(73,243,243);
          if (accelCal == 2) Bean.setLed(242,189,73);
        }
          
      }
      else {
        // Format data for packeting
        ax = int(accel.x()*100) & 0xfff;
        ay = int(accel.y()*100) & 0xfff;
        az = int(accel.z()*100) & 0xfff;

        ex = int(euler.x()*100);
        ey = int(euler.y()*100);
        ez = int(euler.z()*100);
        
        // Set data packet in buffer
        buff[12*reading_no] = char(1 << 7 | button << 4 | ax >> 8);
        buff[12*reading_no+1] = char(ax & 0xff);
        buff[12*reading_no+2] = char(ay >> 4);
        buff[12*reading_no+3] = char((ay & 0xf) << 4 | (az >> 8) & 0xf);
        buff[12*reading_no+4] = char(az & 0xff);
        buff[12*reading_no+5] = char(ex >> 8);
        buff[12*reading_no+6] = char(ex & 0xff);
        buff[12*reading_no+7] = char(ey >> 8);
        buff[12*reading_no+8] = char(ey & 0xff);
        buff[12*reading_no+9] = char(ez >> 8);
        buff[12*reading_no+10] = char(ez & 0xff);
        buff[12*reading_no+11] = char(cur_count);
        
        // User feedback for force sensor on LED Sequin
        if(force) digitalWrite(LED, HIGH);
        else digitalWrite(LED, LOW);
      }

      reading_no++;
      // If buffer filled with 5 readings, send to PC
      if (reading_no > 4) {
        Serial.write((const unsigned char*)buff, 60);
        reading_no = 0;
      }
    }
    // Reset ISR signal
    do_loop = 0;
  }
}
