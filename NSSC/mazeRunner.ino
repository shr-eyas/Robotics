#include <NewPing.h>

// Sonar pins
const int trigPinR = 2;
const int echoPinR = 3;
const int trigPinF = 4;
const int echoPinF = 5;
const int trigPinL = 6;
const int echoPinL = 7;

#define maxDistance 200 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.

NewPing sonarL(trigPinL, echoPinL, maxDistance); 
NewPing sonarF(trigPinF, echoPinF, maxDistance); 
NewPing sonarR(trigPinR, echoPinR, maxDistance); 

// Motor pins
const int ENC1A = 18;
const int ENC1B = 19;
const int ENC2A = 20;
const int ENC2B = 21;
const int M1A = 9;
const int M1B = 8;
const int M2A = 10;
const int M2B = 11;

// Encoder variables
const int encoderCountsPerRevolution = 680;
double posR, posL;
double angleR, angleL;
double lastProportional, PIDValue;

// Sonar Variables
long durationR, durationF, durationL;
double distanceR, distanceF, distanceL;

bool L, R, F, S;

// PID constants
double Kp = 3;
double Ki = 0;
double Kd = 0;

double baseSpeed = 100;
double thresholdF = 20;
double thresholdT = 20;
double turnAngleL = 204;
double turnAngleR = 100;
double delayBT = 700;
double delayAT = 500;

//int i=0;

void setup() 
{
  pinMode(trigPinR, OUTPUT); 
  pinMode(echoPinR, INPUT); 
  pinMode(trigPinF, OUTPUT); 
  pinMode(echoPinF, INPUT); 
  pinMode(trigPinL, OUTPUT); 
  pinMode(echoPinL, INPUT); 

  pinMode(ENC1A, INPUT);
  pinMode(ENC1B, INPUT);
  pinMode(ENC2A, INPUT);
  pinMode(ENC2B, INPUT);

  pinMode(M1A, OUTPUT);
  pinMode(M1B, OUTPUT);
  pinMode(M2A, OUTPUT);
  pinMode(M2B, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(ENC1A), readEncoder1, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC2A), readEncoder2, RISING);

  Serial.begin(9600); 
}

void readEncoder1() 
{
  int b = digitalRead(ENC1B);
  if (b > 0) {
    posR--;
  } else {
    posR++;
  }
}

void readEncoder2() 
{
  int b = digitalRead(ENC2B);
  if (b > 0) {
    posL--;
  } else {
    posL++;
  }
}

float calculateAngle(int encoderPosition) 
{
  // Calculate the angle based on encoder position, CPR, and gear ratio

  float angle = (encoderPosition * 360.0) / encoderCountsPerRevolution;
  return angle;
}

void loop() 
{
  distance();

  double requiredR = (distanceR + distanceL)/2;
  double error = requiredR-distanceR;
  double proportional = error; 
  double derivative = proportional - lastProportional;
  lastProportional = proportional;
  PIDValue = Kp*proportional + Kd*derivative;
  
  // Serial.print("R:");
  // Serial.print(distanceR);
  // Serial.print(" F:");
  // Serial.print(distanceF);
  // Serial.print(" L:");
  // Serial.print(distanceL);
  // Serial.print(" ");
  // Serial.print(angleL);
  // Serial.print(" ");
  // Serial.println(angleR);


  setFlag();
  // Serial.print(L);
  // Serial.print(F);
  // Serial.println(R);

  if(L)
  {
    analogWrite(M1A, baseSpeed);
    digitalWrite(M1B, 0);
    analogWrite(M2A, baseSpeed);
    digitalWrite(M2B, 0);
    delay(delayBT);

    posR = 0;
    posL = 0;
    angleR = 0;
    angleL = 0;

    while(abs(angleR)<=turnAngleL)
    {
      analogWrite(M1A, 120);
      digitalWrite(M1B, 0);
      analogWrite(M2A, 120);
      digitalWrite(M2B, 1);
      angleR = calculateAngle(posR); 
      Serial.println("Left");
    }
    analogWrite(M1A, 0);
    digitalWrite(M1B, 0);
    analogWrite(M2A, 0);
    digitalWrite(M2B, 0);
    setFlag();

    analogWrite(M1A, baseSpeed);
    digitalWrite(M1B, 0);
    analogWrite(M2A, baseSpeed);
    digitalWrite(M2B, 0);
    delay(delayAT);
    
  }

  else if(F)
  { 
    Serial.println("Front");
    analogWrite(M1A, baseSpeed+PIDValue);
    digitalWrite(M1B, 0);
    analogWrite(M2A, baseSpeed-PIDValue);
    digitalWrite(M2B, 0);
    setFlag();
  }

  else if(R)
  {
    analogWrite(M1A, baseSpeed);
    digitalWrite(M1B, 0);
    analogWrite(M2A, baseSpeed);
    digitalWrite(M2B, 0);
    delay(delayBT);

    posR = 0;
    posL = 0;
    angleR = 0;
    angleL = 0;

    while(abs(angleL)<=turnAngleR)
    {
      analogWrite(M1A, 120);
      digitalWrite(M1B, 1);
      analogWrite(M2A, 120);
      digitalWrite(M2B, 0);
      angleL = calculateAngle(posL); 
      Serial.println("Right");
    }
    analogWrite(M1A, 0);
    digitalWrite(M1B, 0);
    analogWrite(M2A, 0);
    digitalWrite(M2B, 0);
    setFlag();

    analogWrite(M1A, baseSpeed);
    digitalWrite(M1B, 0);
    analogWrite(M2A, baseSpeed);
    digitalWrite(M2B, 0);
    delay(delayAT);
  }

  else
  {
    Serial.println("Stop");
    analogWrite(M1A, 0);
    digitalWrite(M1B, 0);
    analogWrite(M2A, 0);
    digitalWrite(M2B, 0);
    setFlag();
  }
}

void setFlag() 
{
  distance();

  if (distanceF < thresholdF) {
    F = false;
  } else {
    F = true;
  }

  if (distanceR < thresholdT) {
    R = false;
  } else {
    R = true;
  }

  if (distanceL < thresholdT) {
    L = false;
  } else {
    L = true;
  }
}

void distance()
{

  distanceR = sonarR.ping_cm();
  distanceF = sonarF.ping_cm();
  distanceL = sonarL.ping_cm();

  if (distanceR==0)
  {
    distanceR = 200;
  }
  
  if (distanceF==0)
  {
    distanceF = 200;
  }

  if (distanceL==0)
  {
    distanceL = 200;
  }

  return (distanceR, distanceF, distanceL);
}
