/* Program to write a preset calibration profile to the IMU, remap the sensor's axes, and
 *  then print its output over serial.
 *  
 *  This program was based on Adafruit's example code.
 *  
 *  By Galen Savidge, 4/6/2019
 *  https://galensavidge.com/
 */

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

Adafruit_BNO055 bno = Adafruit_BNO055();

void setup() {
  Serial.begin(9600);
  Serial.println("BNO055 Test");
  Serial.println("");
  
  /* Initialise the sensor */
  if(!bno.begin(Adafruit_BNO055::OPERATION_MODE_NDOF))
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  bno.setAxisRemap(Adafruit_BNO055::REMAP_CONFIG_P3);
  bno.setAxisSign(Adafruit_BNO055::REMAP_SIGN_P3);
  
  /* Write calibration profile */
  adafruit_bno055_offsets_t offsets;
  offsets.accel_offset_x = -48;
  offsets.accel_offset_y = -1;
  offsets.accel_offset_z = -29;
  
  offsets.mag_offset_x = 95;
  offsets.mag_offset_y = 141;
  offsets.mag_offset_z = 120;
  
  offsets.gyro_offset_x = 0;
  offsets.gyro_offset_y = 0;
  offsets.gyro_offset_z = 0;
  
  offsets.accel_radius = 1000;
  offsets.mag_radius = 610;


  bno.setSensorOffsets(offsets);

  /* Configure sensor */
  bno.setExtCrystalUse(true);
  
  delay(10);
}

void loop(void) 
{
  /* Get a new sensor event */ 
  sensors_event_t event; 
  bno.getEvent(&event);
  
  /* Display the floating point data */
  Serial.print("X: ");
  Serial.print(event.orientation.x, 4);
  Serial.print("\tY: ");
  Serial.print(event.orientation.y, 4);
  Serial.print("\tZ: ");
  Serial.print(event.orientation.z, 4);
  Serial.println("");
  
  delay(100);
}
