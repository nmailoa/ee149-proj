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

/**************************************************************************/
/*
    Arduino setup function (automatically called at startup)
*/
/**************************************************************************/
void setup(void)
{
  Serial.begin(9600);
  Serial.println("Orientation Sensor Raw Data Test"); Serial.println("");

  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  delay(1000);

  /* Display the current temperature */
  int8_t temp = bno.getTemp();
  Serial.print("Current Temperature: ");
  Serial.print(temp);
  Serial.println(" C");
  Serial.println("");

  bno.setExtCrystalUse(true);

  Serial.println("Calibration status values: 0=uncalibrated, 3=fully calibrated");
  sendCalibration();
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
  //imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);

  /* Display the floating point data */
  Serial.print("");
  Serial.print(accel.x());
  Serial.print(",");
  Serial.print(accel.y());
  Serial.print(",");
  Serial.print(accel.z());
  Serial.print(",");

  /*
  // Quaternion data
  imu::Quaternion quat = bno.getQuat();
  Serial.print("qW: ");
  Serial.print(quat.w(), 4);
  Serial.print(" qX: ");
  Serial.print(quat.y(), 4);
  Serial.print(" qY: ");
  Serial.print(quat.x(), 4);
  Serial.print(" qZ: ");
  Serial.print(quat.z(), 4);
  Serial.print("\t\t");
  */

  /* Display calibration status for each sensor. */
  uint8_t system, gyroCal, accelCal, magCal = 0;
  bno.getCalibration(&system, &gyroCal, &accelCal, &magCal);
  Serial.print(system, DEC);
  Serial.print(accelCal, DEC);
  Serial.print(gyroCal, DEC);
  Serial.print(magCal, DEC);
  Serial.print("\n"); 
  
  
  /*if (gyroCal == 3 && accelCal == 3 && magCal == 3 && calibrated == 0){
    saveCalibration();
    calibrated = 1;
    Serial.print(accel_offset_x);
    Serial.print(",");
    Serial.print(accel_offset_y);
    Serial.print(",");
    Serial.print(accel_offset_z);
    Serial.print(",");
    Serial.print(gyro_offset_x);
    Serial.print(",");
    Serial.print(gyro_offset_y);
    Serial.print(",");
    Serial.print(gyro_offset_z);
    Serial.print(",");
    Serial.print(mag_offset_x);
    Serial.print(",");
    Serial.print(mag_offset_y);
    Serial.print(",");
    Serial.print(mag_offset_z);
    Serial.print(",");
    Serial.print(accel_radius);
    Serial.print(",");
    Serial.print(mag_radius);
    Serial.print("\n");
  } */
  
  /*if (gyroCal < 2 || accelCal < 2) {
    sendCalibration();
  }*/

  delay(BNO055_SAMPLERATE_DELAY_MS);
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
