/* Script to calibrate the IMU and print the values in the calibration registers.
 *  
 *  This program was based on Adafruit's example code.
 *  
 *  By Galen Savidge, 4/6/2019
 *  https://galensavidge.com/
 */

#include <stdlib.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

Adafruit_BNO055 bno = Adafruit_BNO055();

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.println("BNO055 Calibration");
  Serial.println("");

  /* Initialise the sensor */
  if(!bno.begin(Adafruit_BNO055::OPERATION_MODE_NDOF))
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  delay(1000);

  bno.setAxisRemap(Adafruit_BNO055::REMAP_CONFIG_P3);
  bno.setExtCrystalUse(true);

  // Wait until sensor is fully calibrated
  uint8_t system, gyro, accel, mag;
  char str[50];
  do {
    bno.getCalibration(&system, &gyro, &accel, &mag);
    sprintf(str, "System: %d\tGyro: %d\tAccel: %d\tMag: %d\n", system, gyro, accel, mag);
    Serial.print(str);
  } while(!bno.isFullyCalibrated());
  
  // Print offsets
  adafruit_bno055_offsets_t offsets;
  bno.getSensorOffsets(offsets);
  Serial.print("offsets.accel_offset_x = "); Serial.print(offsets.accel_offset_x); Serial.println(";");
  Serial.print("offsets.accel_offset_y = "); Serial.print(offsets.accel_offset_y); Serial.println(";");
  Serial.print("offsets.accel_offset_z = "); Serial.print(offsets.accel_offset_z); Serial.println(";");
  Serial.println();
  Serial.print("offsets.mag_offset_x = "); Serial.print(offsets.mag_offset_x); Serial.println(";");
  Serial.print("offsets.mag_offset_y = "); Serial.print(offsets.mag_offset_y); Serial.println(";");
  Serial.print("offsets.mag_offset_z = "); Serial.print(offsets.mag_offset_z); Serial.println(";");
  Serial.println();
  Serial.print("offsets.gyro_offset_x = "); Serial.print(offsets.gyro_offset_x); Serial.println(";");
  Serial.print("offsets.gyro_offset_y = "); Serial.print(offsets.gyro_offset_y); Serial.println(";");
  Serial.print("offsets.gyro_offset_z = "); Serial.print(offsets.gyro_offset_z); Serial.println(";");
  Serial.println();
  Serial.print("offsets.accel_radius = "); Serial.print(offsets.accel_radius); Serial.println(";");
  Serial.print("offsets.mag_radius = "); Serial.print(offsets.mag_radius); Serial.println(";");
}

void loop() {
  // put your main code here, to run repeatedly:
  
}
