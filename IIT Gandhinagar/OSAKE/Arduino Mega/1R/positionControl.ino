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
float KdR = 0;

int dirR;
float posR = 0;
float posL = 0;

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

void loop(){

  float angleR = (posR*360.0*correctionFactor)/encoderCountsPerRevolution; 

  float errorR = targetR - angleR;

  // PD control
  long currT = micros();
  long prevT = 0;   
  float eprev = 0;
  long et = currT - prevT;
  float deltaT = ((float)(currT - prevT))/1.0e6;
  prevT = currT;  
  float dedtR = (-errorR + eprev)/(deltaT);
  float pwmR = KpR*errorR + KdR*dedtR;

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

