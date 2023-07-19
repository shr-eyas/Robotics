#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

const int MLD = 25; // Connect to GPIO25 Left Direction
const int MRD = 14; // Connect to GPIO14 Right Direction

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

void setup(void) 
{
  // Serial monitor setup
  Serial.begin(115200);

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
  Serial.print("Accelerometer range set to: ");
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  mpu.setFilterBandwidth(MPU6050_BAND_94_HZ);
  Serial.print("Filter bandwidth set to: ");
  Serial.println("");
  delay(100);

  // Motor PWM setup
  ledcSetup(MLP, freq, resolution);
  ledcSetup(MRP, freq, resolution);
  pinMode(MRD, OUTPUT); 
  pinMode(MLD, OUTPUT); 
  ledcAttachPin(27, MLP);  // Connect to GPIO27 Right PWM
  ledcAttachPin(33, MRP);  // Connect to GPIO33 Left PWM 
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
  const float gyroCoef = 0.85;
  const float accCoef = 0.15;
  angleX = (gyroCoef * angleGyroX) + (accCoef * angleAccX); 
  angleY = (gyroCoef * angleGyroY) + (accCoef * angleAccY); 
  angleZ = angleGyroZ;

  preInterval = millis();
  Serial.println(angleY);
  

}
