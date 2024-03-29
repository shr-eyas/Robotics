// Code for PID control on a Self Balancing Robot
#include <MPU6050_tockn.h>
#include <Wire.h>

MPU6050 mpu6050(Wire);

const int MLD = 25; // Connect to GPIO25 Left Direction
const int MRD = 14; // Connect to GPIO14 Right Direction

// Position variables 
float angleX = 0;
float desiredX = 0;

// PID constants
float kp = 5.31; 
float ki = 0.00001; 
float kd = 0.00002;   

// Variables for PID control
float error = 0;
float previousError = 0.0;
float proportional;
float derivative;
float integral = 0.0;

// Motor driver output
float pwmVal = 0;
float driverOP = 70;

// PWM variables 
const int MLP = 0;
const int MRP = 1;
const int freq = 5000;
const int resolution = 8;

// Math variables
volatile float interval = 0.0;
volatile float preInterval = 0.0;

void setup() {

  // Serial monitor setup
  Serial.begin(115200);

  // MPU setup
  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);

  // Motor PWM setup
  ledcSetup(MLP, freq, resolution);
  ledcSetup(MRP, freq, resolution);
  pinMode(MRD, OUTPUT);
  pinMode(MLD, OUTPUT);
  ledcAttachPin(33, MLP); // Connect to GPIO27 Right PWM
  ledcAttachPin(27, MRP); // Connect to GPIO33 Left PWM
}

void loop() {

  mpu6050.update();

  angleX = mpu6050.getAngleX();
  error = desiredX - angleX;

  // PID components
  proportional = kp * error;
  integral += ki * error;
  derivative = kd * (error - previousError);
  previousError = error;

  pwmVal = proportional + integral + derivative;

  // Motor driver output
  driverOP = abs(pwmVal);
  if (driverOP > 100) {
    driverOP = 100;
  }

  // PWM control
  if (error <= 0 && error > -40) {
    digitalWrite(MLD, 0);
    digitalWrite(MRD, 0);
    ledcWrite(MLP, driverOP);
    ledcWrite(MRP, driverOP);
  } else if (error > 0 && error < 40) {
    digitalWrite(MLD, 1);
    digitalWrite(MRD, 1);
    ledcWrite(MLP, driverOP);
    ledcWrite(MRP, driverOP);
  } else if (angleX > 40) {
    digitalWrite(MLD, 0);
    digitalWrite(MRD, 0);
    ledcWrite(MLP, 0);
    ledcWrite(MRP, 0);
  } else if (angleX < -40) {
    digitalWrite(MLD, 0);
    digitalWrite(MRD, 0);
    ledcWrite(MLP, 0);
    ledcWrite(MRP, 0);
  }

  // Serial print for debugging
  Serial.print(angleX);
  Serial.print(" ");
  Serial.print(error);
  Serial.print(" ");
  Serial.print(pwmVal);
  Serial.print(" ");
  Serial.println(driverOP);
}
