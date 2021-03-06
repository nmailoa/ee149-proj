#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

/* This driver reads raw data from the BNO055

   Connections
   ===========
   Connect SCL to analog 5
   Connect SDA to analog 4
   Connect VDD to 3.3V DC
   Connect GROUND to common ground

   History
   =======
   2015/MAR/03  - First release (KTOWN)
*/

/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (10)

#define LED (0)
#define FORCE_SENSOR (1)
#define BUTTON (3)

int calibrated = 0;

unsigned int accel_offset_x = 65516;
unsigned int accel_offset_y = 65509;
unsigned int accel_offset_z = 2;
unsigned int mag_offset_x = 65529;
unsigned int mag_offset_y = 0;
unsigned int mag_offset_z = 0;
unsigned int gyro_offset_x = 65258;
unsigned int gyro_offset_y = 116;
unsigned int gyro_offset_z = 65087;
unsigned int accel_radius = 1000;
unsigned int mag_radius = 570;

Adafruit_BNO055 bno = Adafruit_BNO055();


uint8_t sys, gyroCal, accelCal, magCal = 0;
long time = 0;
boolean button = 0;
boolean sync = 0;
int do_loop = 0;
boolean debug = 0;
byte count = 0;
byte cur_count = 0;

char buff[60] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
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

ISR(TIMER1_COMPA_vect)
{
//digitalWrite(5, debug);
do_loop = 1;
count++;
//if (debug) debug = 0;
//else debug = 1;
}

/**************************************************************************/
/*
    Arduino setup function (automatically called at startup)
*/
/**************************************************************************/
void setup(void)
{
  Bean.setLed(255,0,0);

  
  sync = 0;
  do_loop = 0;
  reading_no = 0;
  count = 0;
  
  pinMode(LED, OUTPUT);
  pinMode(BUTTON, INPUT);
  pinMode(FORCE_SENSOR, INPUT);

  //pinMode(12, OUTPUT);
  cli();          // disable global interrupts
  TCCR1A = 0;     // set entire TCCR1A register to 0
  TCCR1B = 0;     // same for TCCR1B

  // set compare match register to desired timer count:
  OCR1A = 117;   // 15ms period

  // turn on CTC mode:
  TCCR1B |= (1 << WGM12);

  // Set CS10 and CS12 bits for 1024 prescaler:
  TCCR1B |= (1 << CS10);
  TCCR1B |= (1 << CS12);

  // enable timer compare interrupt:
  TIMSK1 |= (1 << OCIE1A);
  sei();          // enable global interrupts
  
  
  Serial.begin(57600);
  Serial.println("Orientation Sensor Raw Data Test"); Serial.println("");
  Serial.println(do_loop);

  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  delay(1000);

  bno.setExtCrystalUse(true);

  //Serial.println("Calibration status values: 0=uncalibrated, 3=fully calibrated");
  //sendCalibration();
  
}




/**************************************************************************/
/*
    Arduino loop function, called once 'setup' is complete (your own code
    should go here)
*/
/**************************************************************************/
void loop(void)
{
  // Possible vector values can be:
  // - VECTOR_ACCELEROMETER - m/s^2
  // - VECTOR_MAGNETOMETER  - uT
  // - VECTOR_GYROSCOPE     - rad/s
  // - VECTOR_EULER         - degrees
  // - VECTOR_LINEARACCEL   - m/s^2
  // - VECTOR_GRAVITY       - m/s^2
  
  /* Display calibration status for each sensor. */
  
  if (do_loop){
    cur_count = count;
    bno.getCalibration(&sys, &gyroCal, &accelCal, &magCal);
    accel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
    euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    force = digitalRead(FORCE_SENSOR);
    button = digitalRead(BUTTON) | (force << 1);

    if (sync == 0 && gyroCal == 3 && accelCal == 3 && magCal == 3){
      buff[0] = char(1 << 7);
      buff[1] = char(sys << 6 | accelCal << 4 | gyroCal << 2 | magCal);
      buff[11] = char(cur_count);
      
      Serial.write((const unsigned char*)buff, 12);
      Serial.flush();
      Bean.setLed(246, 255, 0);
      reading_no = 0;
      
      while (Serial.read() != '1');

      Bean.setLed(0,0,255);
      while (digitalRead(BUTTON) != 1);
      while (digitalRead(BUTTON) != 0);
      Bean.setLed(0,255,0);
      
      sync = 1;
    }
    else {
      if (sync == 0){
        buff[12*reading_no] = char(1 << 7);
        buff[12*reading_no+1] = char(sys << 6 | accelCal << 4 | gyroCal << 2 | magCal);
        buff[12*reading_no+11] = char(cur_count);
        if (gyroCal == 3 && magCal == 3){
          if (accelCal == 0) Bean.setLed(0,0,0);
          if (accelCal == 1) Bean.setLed(73,243,243);
          if (accelCal == 2) Bean.setLed(242,189,73);
        }
          
      }
      else {
        
        ax = int(accel.x()*100) & 0xfff;
        ay = int(accel.y()*100) & 0xfff;
        az = int(accel.z()*100) & 0xfff;

        ex = int(euler.x()*100);
        ey = int(euler.y()*100);
        ez = int(euler.z()*100);
        
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
        
        if(force) digitalWrite(LED, HIGH);
        else digitalWrite(LED, LOW);

        //digitalWrite(4, LOW);
      }
      reading_no++;
      if (reading_no > 4) {
        Serial.write((const unsigned char*)buff, 60);
        reading_no = 0;
      }
      
    }
    
    do_loop = 0;
  }
}

void saveCalibration(){
  bno.setMode(Adafruit_BNO055::OPERATION_MODE_CONFIG);
  
  accel_offset_x = bno.read8(Adafruit_BNO055::ACCEL_OFFSET_X_LSB_ADDR);
  accel_offset_x |= (bno.read8(Adafruit_BNO055::ACCEL_OFFSET_X_MSB_ADDR) << 8);
  accel_offset_y = bno.read8(Adafruit_BNO055::ACCEL_OFFSET_Y_LSB_ADDR);
  accel_offset_y |= (bno.read8(Adafruit_BNO055::ACCEL_OFFSET_Y_MSB_ADDR) << 8);
  accel_offset_z = bno.read8(Adafruit_BNO055::ACCEL_OFFSET_Z_LSB_ADDR);
  accel_offset_z |= (bno.read8(Adafruit_BNO055::ACCEL_OFFSET_Z_MSB_ADDR) << 8);
  
  mag_offset_x = bno.read8(Adafruit_BNO055::MAG_OFFSET_X_LSB_ADDR);
  mag_offset_x |= (bno.read8(Adafruit_BNO055::MAG_OFFSET_X_MSB_ADDR) << 8);
  mag_offset_y = bno.read8(Adafruit_BNO055::MAG_OFFSET_Y_LSB_ADDR);
  mag_offset_y |= (bno.read8(Adafruit_BNO055::MAG_OFFSET_Y_MSB_ADDR) << 8);
  mag_offset_z = bno.read8(Adafruit_BNO055::MAG_OFFSET_Z_LSB_ADDR);
  mag_offset_z |= (bno.read8(Adafruit_BNO055::MAG_OFFSET_Z_MSB_ADDR) << 8);
  
  gyro_offset_x = bno.read8(Adafruit_BNO055::GYRO_OFFSET_X_LSB_ADDR);
  gyro_offset_x |= (bno.read8(Adafruit_BNO055::GYRO_OFFSET_X_MSB_ADDR) << 8);
  gyro_offset_y = bno.read8(Adafruit_BNO055::GYRO_OFFSET_Y_LSB_ADDR);
  gyro_offset_y |= (bno.read8(Adafruit_BNO055::GYRO_OFFSET_Y_MSB_ADDR) << 8);
  gyro_offset_z = bno.read8(Adafruit_BNO055::GYRO_OFFSET_Z_LSB_ADDR);
  gyro_offset_z |= (bno.read8(Adafruit_BNO055::GYRO_OFFSET_Z_MSB_ADDR) << 8);
  
  accel_radius = bno.read8(Adafruit_BNO055::ACCEL_RADIUS_LSB_ADDR);
  accel_radius |= (bno.read8(Adafruit_BNO055::ACCEL_RADIUS_MSB_ADDR) << 8);
  mag_radius = bno.read8(Adafruit_BNO055::MAG_RADIUS_LSB_ADDR);
  mag_radius |= (bno.read8(Adafruit_BNO055::MAG_RADIUS_MSB_ADDR) << 8);
  
  bno.setMode(Adafruit_BNO055::OPERATION_MODE_NDOF);
}

void sendCalibration(){
  Serial.println("Sending calibration");
  bno.setMode(Adafruit_BNO055::OPERATION_MODE_CONFIG);
  
  bno.write8(Adafruit_BNO055::ACCEL_OFFSET_X_LSB_ADDR, accel_offset_x & 0xFF);
  bno.write8(Adafruit_BNO055::ACCEL_OFFSET_X_MSB_ADDR, (accel_offset_x >> 8) & 0xFF);
  bno.write8(Adafruit_BNO055::ACCEL_OFFSET_Y_LSB_ADDR, accel_offset_y & 0xFF);
  bno.write8(Adafruit_BNO055::ACCEL_OFFSET_Y_MSB_ADDR, (accel_offset_y >> 8) & 0xFF);
  bno.write8(Adafruit_BNO055::ACCEL_OFFSET_Z_LSB_ADDR, accel_offset_z & 0xFF);
  bno.write8(Adafruit_BNO055::ACCEL_OFFSET_Z_MSB_ADDR, (accel_offset_z >> 8) & 0xFF);

  bno.write8(Adafruit_BNO055::MAG_OFFSET_X_LSB_ADDR, mag_offset_x & 0xFF);
  bno.write8(Adafruit_BNO055::MAG_OFFSET_X_MSB_ADDR, (mag_offset_x >> 8) & 0xFF);
  bno.write8(Adafruit_BNO055::MAG_OFFSET_Y_LSB_ADDR, mag_offset_y & 0xFF);
  bno.write8(Adafruit_BNO055::MAG_OFFSET_Y_MSB_ADDR, (mag_offset_y >> 8) & 0xFF);
  bno.write8(Adafruit_BNO055::MAG_OFFSET_Z_LSB_ADDR, mag_offset_z & 0xFF);
  bno.write8(Adafruit_BNO055::MAG_OFFSET_Z_MSB_ADDR, (mag_offset_z >> 8) & 0xFF);

  bno.write8(Adafruit_BNO055::GYRO_OFFSET_X_LSB_ADDR, gyro_offset_x & 0xFF);
  bno.write8(Adafruit_BNO055::GYRO_OFFSET_X_MSB_ADDR, (gyro_offset_x >> 8) & 0xFF);
  bno.write8(Adafruit_BNO055::GYRO_OFFSET_Y_LSB_ADDR, gyro_offset_y & 0xFF);
  bno.write8(Adafruit_BNO055::GYRO_OFFSET_Y_MSB_ADDR, (gyro_offset_y >> 8) & 0xFF);
  bno.write8(Adafruit_BNO055::GYRO_OFFSET_Z_LSB_ADDR, gyro_offset_z & 0xFF);
  bno.write8(Adafruit_BNO055::GYRO_OFFSET_Z_MSB_ADDR, (gyro_offset_z >> 8) & 0xFF);  

  bno.write8(Adafruit_BNO055::ACCEL_RADIUS_LSB_ADDR, accel_radius & 0xFF);
  bno.write8(Adafruit_BNO055::ACCEL_RADIUS_MSB_ADDR, (accel_radius >> 8) & 0xFF);
  bno.write8(Adafruit_BNO055::MAG_RADIUS_LSB_ADDR, mag_radius & 0xFF);
  bno.write8(Adafruit_BNO055::MAG_RADIUS_MSB_ADDR, (mag_radius >> 8) & 0xFF);
  
  bno.setMode(Adafruit_BNO055::OPERATION_MODE_NDOF);
}
