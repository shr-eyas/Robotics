#include <math.h>

// Motor driver connections
const int dirPinR = 6;
const int pwmPinR = 7;

// Encoder connections
#define ENC1A 18
#define ENC1B 19

// Target position
float targetR = 10;

// PD constants
float KpR = 10;

int dirR;
float posR = 0;
float posL = 0;

float runTime = 5;

const float correctionFactor = 1;
const int encoderCountsPerRevolution = 4096;

void readEncoder() {

  int b = digitalRead(ENC1B);
  if (b > 0) {
    posR--;
  } else {
    posR++;
  }
}

void setup() {

  Serial.begin(9600);

  pinMode(ENC1A, INPUT);
  pinMode(ENC1B, INPUT);
  pinMode(dirPinR, OUTPUT);
  pinMode(pwmPinR, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(ENC1A), readEncoder, RISING);
}

float timer(float runT)
{
  static unsigned long startTime = millis();  
  unsigned long currentTime = millis();  
  unsigned long elapsedTime = currentTime - startTime;  
  if (elapsedTime < runT*1000) {  
    return elapsedTime;
  }
}

void trajectoryPlanner(float* targetRVal) {

  float time = timer(runTime);
  float t = time / 1000;

  float real_part = pow(t, 3) * (5.0 / 2.0) - pow(t, 2) * (15.0 / 2.0) + 10.0;
  float imaginary_part = -pow(t, 3) * (5.0 / 2.0);
  float q1 = atan2(imaginary_part, real_part);

  *targetRVal = q1*(180/3.14);
}

void loop(){

  float angleR = (posR*360.0*correctionFactor)/encoderCountsPerRevolution; 
  
  float targetRV;
  trajectoryPlanner(&targetRV);

  float errorR = targetRV - angleR;

  // P control
  float pwmR = KpR*errorR;

  if (pwmR > 0){
    dirR = 1;
  } else{
    dirR = 0;
  }

  float powerR = abs(pwmR);

  if (powerR>150){
    powerR = 150;
  }

  digitalWrite(dirPinR,dirR);
  analogWrite(pwmPinR,powerR);

  Serial.print(angleR);
  Serial.print(" ");
  Serial.print(errorR);
  Serial.print(" ");
  Serial.println(pwmR);
}
