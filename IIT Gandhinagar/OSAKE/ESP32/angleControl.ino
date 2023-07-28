#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

const int MLD = 25; // Connect to GPIO25 Left Direction
const int MRD = 14; // Connect to GPIO14 Right Direction
#define ENCODER_A1 4  // Pin for Output A of Left Encoder 
#define ENCODER_B1 15 // Pin for Output B of Left Encoder 
#define ENCODER_A2 18 // Pin for Output A of Right Encoder 
#define ENCODER_B2 19 // Pin for Output B of Right Encoder 

// Global variable for storing the positions of Encoder
volatile int leftEncoderVal = 0;  
volatile int rightEncoderVal = 0; 

// Desired position 
float desiredY = 0;

// PID constants
float Kp = 40; 
float Ki = 0; 
float Kd = 0.1; 

// Variables for PID control
float errorY = 0;
float prevErrorY = 0.0;
float proportional;
float derivative;
float integral = 0.0;

// Motor driver output
float pwmVal = 0;
float driverOP = 70;
float PIDVal = 0;

// MPU calibration offsets
float offsetX = 0;
float offsetY = 0;
float offsetZ = 0;

// PWM variables 
const int MLP = 0;  
const int MRP = 1;  
const int freq = 5000;
const int resolution = 8;

// Physics variables
float vxi = 0.0;
float vyi = 0.0;
float vzi = 0.0;
float off_vxi = 0.0;
float off_vyi = 0.0;
float off_vzi = 0.0;
volatile float angleGyroX = 0.0;
volatile float angleGyroY = 0.0;
volatile float angleGyroZ = 0.0;
volatile float angleAccX = 0.0;
volatile float angleAccY = 0.0;
volatile float angleX = 0.0;
volatile float angleY = 0.0;
volatile float angleZ = 0.0;

// Math variables
volatile float interval = 0.0;
volatile float preInterval = 0.0;

Adafruit_MPU6050 mpu;

void encoder_left() {

  int A = digitalRead(ENCODER_A1);
  int B = digitalRead(ENCODER_B1);

  if ((A == HIGH) != (B == LOW)) {
    leftEncoderVal--;
  } else {
    leftEncoderVal++;
  }
}

void encoder_right() {

  int A = digitalRead(ENCODER_A2);
  int B = digitalRead(ENCODER_B2);

  if ((A == HIGH) != (B == LOW)) {
    rightEncoderVal--;
  } else {
    rightEncoderVal++;
  }
}

void setup(void) 
{
  // Serial monitor setup
  Serial.begin(115200);

  // Inbuilt LED setup
  pinMode(2, OUTPUT);

  // Encoder setup
  pinMode(ENCODER_A1, INPUT_PULLUP);
  pinMode(ENCODER_B1, INPUT_PULLUP);
  pinMode(ENCODER_A2, INPUT_PULLUP);
  pinMode(ENCODER_B2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A1), encoder_left, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A2), encoder_right, CHANGE);

  // MPU setup
  while (!Serial)
  delay(10); 
  Serial.println("Adafruit MPU6050 test!");
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_94_HZ);
  delay(100);

  // Motor PWM setup
  ledcSetup(MLP, freq, resolution);
  ledcSetup(MRP, freq, resolution);
  pinMode(MRD, OUTPUT); 
  pinMode(MLD, OUTPUT); 
  ledcAttachPin(33, MLP);  // Connect to GPIO27 Right PWM
  ledcAttachPin(27, MRP);  // Connect to GPIO33 Left PWM 

  // Call function to set desired angleY
  setDesired();
}

void setDesired() {
  digitalWrite(2, HIGH);
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  const int numSamples = 1000; 
  float sumAngleAccY = 0.0;
  for (int i = 0; i < numSamples; i++) {
    mpu.getEvent(&a, &g, &temp);
    float angleAccY = atan2(a.acceleration.x, sqrt(a.acceleration.z * a.acceleration.z + a.acceleration.y * a.acceleration.y)) * -180.0 / PI;
    sumAngleAccY += angleAccY;
    delay(10); 
  }
  desiredY = sumAngleAccY / numSamples;
  digitalWrite(2, LOW);
}


void loop() 
{
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Angular velocity 
  vxi = g.gyro.x - offsetX; 
  vyi = g.gyro.y - offsetY; 
  vzi = g.gyro.z - offsetZ;

  // Offsets calculation
  off_vxi = g.gyro.x - offsetX - vxi;  
  off_vyi = g.gyro.y - offsetY - vyi;
  off_vzi = g.gyro.z - offsetZ - vzi;

  // Angle from acceleration data
  angleAccX = atan2(a.acceleration.y, sqrt(a.acceleration.z * a.acceleration.z + a.acceleration.x * a.acceleration.x)) * 180.0 / PI;  
  angleAccY = atan2(a.acceleration.x, sqrt(a.acceleration.z * a.acceleration.z + a.acceleration.y * a.acceleration.y)) * -180.0 / PI;

  // Integrationg angular velocity to get change in angle
  interval = (millis() - preInterval) * 0.001; 
  angleGyroX = angleGyroX + off_vxi * interval;
  angleGyroY = angleGyroY + off_vyi * interval;
  angleGyroZ = angleGyroZ + off_vzi * interval;

  // Sensor fusion
  const float gyroCoef = 0.95;
  const float accCoef = 0.05;
  angleX = (gyroCoef * angleX + off_vxi * interval) + (accCoef * angleAccX); 
  angleY = (gyroCoef * angleY + off_vyi * interval) + (accCoef * angleAccY); 
  angleZ = angleGyroZ;

  preInterval = millis();

  // Measure desiredY by keeping the robot erect and measuring angleY value
  errorY = desiredY - angleY;

  // PID components 
  proportional = Kp * (errorY);
  integral = integral + Ki * (errorY) * interval;
  derivative = Kd * (errorY - prevErrorY) / interval;

  // Update prevErrorY for the next iteration
  prevErrorY = errorY;

  // Calculate the PWM value 
  pwmVal = proportional + integral + derivative;

  // Motor driver output
  driverOP = abs(pwmVal);
  if (driverOP>70){
    driverOP = 70;
  }

  PWMControl();

  //Serial print for debugging
  Serial.print(desiredY);
  Serial.print(" ");
  Serial.print(angleY);
  Serial.print(" ");
  Serial.print(errorY);
  Serial.print(" ");
  Serial.println(driverOP);  
}

void PWMControl() {

  if (errorY<0) {
    digitalWrite(MLD, 0);
    digitalWrite(MRD, 0);
    float KP = 0;
    float rotationError = leftEncoderVal - rightEncoderVal;
    float PIDVal = KP*rotationError;
    ledcWrite(MLP, driverOP-PIDVal);
    ledcWrite(MRP, driverOP+PIDVal);
  }
  else if (errorY>0) {
    digitalWrite(MLD, 1);
    digitalWrite(MRD, 1);
    float KP = 0;
    float rotationError = leftEncoderVal - rightEncoderVal;
    float PIDVal = KP*rotationError;
    ledcWrite(MLP, driverOP-PIDVal);
    ledcWrite(MRP, driverOP+PIDVal);
  }

}
