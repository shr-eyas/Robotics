#include <Arduino.h>

// PWM Channels
const int MLA = 0;  
const int MLB = 1;  
const int MRA = 2;  
const int MRB = 3; 

// Encoder Inputs
const int ER = 23;
const int EL = 22;

double posR = 0;
double posL = 0;

double error, proportional, derivative, integral, PIDValue;
double lastProportional = 0;

double Kp = 11;
double Kd = 4;
double Ki = 0;

double motorBaseSpeed = 150;

void setup(){

  ledcSetup(MLA, 5000, 8);
  ledcSetup(MLB, 5000, 8);
  ledcSetup(MRA, 5000, 8);
  ledcSetup(MRB, 5000, 8);

  ledcAttachPin(16, MLA);
  ledcAttachPin(17, MLB);
  ledcAttachPin(18, MRA);
  ledcAttachPin(19, MRB);

  pinMode(ER, INPUT);
  pinMode(EL, INPUT);

  Serial.begin(115200);
}
 
void loop(){
  
  ledcWrite(MLA, int(motorBaseSpeed+PIDValue));
  ledcWrite(MLB, 0); 
  ledcWrite(MRA, int(motorBaseSpeed-PIDValue));
  ledcWrite(MRB, 0); 

  int prevR = digitalRead(ER);
  int prevL = digitalRead(EL);
  delay(10);
  int currR = digitalRead(ER);
  int currL = digitalRead(EL);

  if (currR != prevR) {
    posR = posR + 1;
  }

  if (currL != prevL) {
    posL = posL + 1;
  }

  error = posR - posL;

  proportional = error; 
  derivative = proportional - lastProportional;
  integral = integral + proportional;
  lastProportional = proportional;
  PIDValue = Kp*proportional + Ki*integral + Kd*derivative;

  Serial.print(error);
  Serial.print(" ");
  Serial.print(posL);
  Serial.print(" ");
  Serial.println(posR);

}


