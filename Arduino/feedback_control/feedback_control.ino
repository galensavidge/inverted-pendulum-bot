/*
 * Program to control the linear position of an inverted pendulum bot. This is a direct
 * implementation of the feedback controller in inverted_pendulum_sim.m.
 * 
 * Created by Galen Savidge, 4/6/2019
 */

// ----- LIBRARIES -----
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <math.h>
#include <Servo.h>

// ----- CONSTANTS -----
// Program configuration
#define MAX_SPEED 0.5 // m/s

// Bot characteristics
#define WHEEL_RADIUS 0.035 // 3.5cm
#define R 0.07 // Wheel center of rotation to bot center of mass, about 7cm
#define BALANCED_PITCH 84 // Degrees

// Pins
#define LEFT_SERVO_PIN 10
#define RIGHT_SERVO_PIN 9
#define BATTERY_PIN A0

// Servo characteristics
#define SERVO_ZERO_PULSE 1500
#define SERVO_MAX_PULSE 100 // Servo pulse range is SERVO_ZERO_PULSE +/- this value
#define SERVO_MAX_SPEED 18.85 // rad/s

// Position controller
#define K_POS 0.2
#define KP_POS (1.5*K_POS)
#define KD_POS (1*K_POS)

// Pitch controller
#define K_PITCH 100 //25
#define KP_PITCH (1*K_PITCH)
#define KI_PITCH 0 //(0.02*K_PITCH)
#define KD_PITCH (0.01*K_PITCH)

// Battery voltage sensing
#define ADC_RESOLUTION_BITS 10
#define ADC_REFERENCE_VOLTAGE 1.1
#define BATT_SCALAR 11
#define DIODE_DROP 0.22
#define MIN_BATT_VOLTAGE 3.8

#define V_W_HIST_LENGTH 10

// ----- GLOBAL VARIABLES -----
Adafruit_BNO055 bno = Adafruit_BNO055();
Servo left_servo, right_servo;

float com_position = 0; // Meters, linear position of center of mass
float pos_desired = 1;
float last_pos_desired = pos_desired;
float last_pos_err = com_position - pos_desired;

float last_pitch_desired = 0;
float last_pitch_err = 0;
float pitch_integrator = 0;

float v_W = 0; // Desired speed of the point in the middle of the wheel axis of rotation

uint32_t last_time_us;

// ----- HELPER FUNCTIONS -----
float deg2rad(float x) {
  float r = x*M_PI/180.0;
  return r;
}

float getBattVoltage(int adc_reading) {
  return (float)BATT_SCALAR*adc_reading*ADC_REFERENCE_VOLTAGE/(1<<ADC_RESOLUTION_BITS) + DIODE_DROP;
}

void init_IMU() {
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

// Speeds are in rad/s
void setWheelSpeeds(float left, float right) {
  // Cap wheel speeds
  if (left > SERVO_MAX_SPEED) left = SERVO_MAX_SPEED;
  else if (left < -SERVO_MAX_SPEED) left = -SERVO_MAX_SPEED;
  if (right > SERVO_MAX_SPEED) right = SERVO_MAX_SPEED;
  else if (right < -SERVO_MAX_SPEED) right = -SERVO_MAX_SPEED;

  left_servo.writeMicroseconds(wheelSpeedToPulse(-left));
  right_servo.writeMicroseconds(wheelSpeedToPulse(right));
}

// Speed is in rad/s
int wheelSpeedToPulse(float speed) {
  return SERVO_ZERO_PULSE + (int)speed*SERVO_MAX_PULSE/SERVO_MAX_SPEED;
}

// Returns the sign of x or 0 if x == 0
float sign(float x) {
  if (x > 0) return 1;
  if (x < 0) return -1;
  return 0;
}

void setup() {
  Serial.begin(115200);
  Serial.println("Inverted pendulum feedback control program by Galen Savidge");
  Serial.println("");
  
  init_IMU();

  left_servo.attach(LEFT_SERVO_PIN);
  right_servo.attach(RIGHT_SERVO_PIN);

  analogReference(INTERNAL);

  // imu_filter = newMovingAvgFilter(1, 10);
  
  last_time_us = micros();
  delay(1);
}

void loop() {
  // Check battery voltage
  float batt_voltage = getBattVoltage(analogRead(BATTERY_PIN));
  if(batt_voltage < MIN_BATT_VOLTAGE) {
    Serial.println("Battery low, shutting down...");
    setWheelSpeeds(0, 0);
    while(1);
  }
  
  // SENSING
  // Read orientation and angular rate
  sensors_event_t event; 
  bno.getEvent(&event);
  float pitch = deg2rad(event.orientation.z - BALANCED_PITCH);
  if(pitch > M_PI) pitch -= M_PI;
  float yaw = deg2rad(event.orientation.x);
  float pitch_dot = deg2rad(event.gyro.z);
  float yaw_dot = deg2rad(event.gyro.x);

  // Moving average pitch
  
//  Serial.print("Yaw: ");
//  Serial.print(event.orientation.x, 4);
//  Serial.print("Pitch: ");
//  Serial.print(event.orientation.z, 4);
  
  // Measure time
  uint32_t new_time_us = micros();
  float dt = ((unsigned)(new_time_us - last_time_us))/1000000.0; // Convert to seconds
  
  // POSITION CONTROLLER
  // Stop changes in deisred position from breaking the controller
  float delta_pos_desired = pos_desired - last_pos_desired;
  last_pos_desired = pos_desired;
  last_pos_err = last_pos_err - delta_pos_desired;

  // Run PD controller
  float pos_err = com_position - pos_desired; // Find error
  float pos_err_p = sign(pos_err)*min(abs(pos_err), MAX_SPEED*KD_POS/KP_POS);
  float c_pos = KP_POS*pos_err_p + KD_POS*(pos_err - last_pos_err)/dt;
  
  last_pos_err = pos_err;
  //float pitch_desired = c_pos;
  float pitch_desired = 0;

  // PITCH CONTROLLER
  // Stop changes in deisred pitch from breaking the controller
  float delta_pitch_desired = pitch_desired - last_pitch_desired;
  last_pitch_desired = pitch_desired;
  last_pitch_err = last_pitch_err - delta_pitch_desired;

  // Run PID controller
  float pitch_err = pitch - pitch_desired; // Find error
  pitch_integrator = pitch_integrator + KI_PITCH*pitch_err*dt;
  float c_pitch = KP_PITCH*pitch_err + KD_PITCH*(pitch_err - last_pitch_err)/dt + pitch_integrator;
  
  last_pitch_err = pitch_err;

  // SET MOTOR SPEEDS
  //float v_W_dot = -c_pitch;
  //float v_W = v_W + v_W_dot*dt;
  // float w_W = -(v_W/WHEEL_RADIUS + pitch_dot);
  float w_W = c_pitch;
  Serial.println(w_W);
  float left_speed = w_W;
  float right_speed = w_W;
  
  setWheelSpeeds(left_speed, right_speed);

  delay(5);
}
