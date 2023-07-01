#include <math.h>

const int dirPinL = 4;
const int pwmPinL = 5;
const int dirPinR = 6;
const int pwmPinR = 7;

#define ENC1A 18
#define ENC1B 19
#define ENC2A 2
#define ENC2B 3

const int encoderCountsPerRevolution = 2048 * 2; // Number of encoder counts per motor shaft revolution
int dirR,dirL;

float posR = 0;
float posL = 0;

int speedR = 0;
int speedL = 0;

float KpR = 10;
float KdR = 0;

float KpL = 3;
float KdL = 0;

float runTime = 5;
float L1 = 105;
float L2 = 105;

void setup() {

  Serial.begin(9600);

  pinMode(ENC1A, INPUT);
  pinMode(ENC1B, INPUT);
  pinMode(ENC2A, INPUT);
  pinMode(ENC2B, INPUT);

  pinMode(dirPinR, OUTPUT);
  pinMode(pwmPinR, OUTPUT);
  pinMode(dirPinL, OUTPUT);
  pinMode(pwmPinL, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(ENC1A), readEncoder1, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC2A), readEncoder2, RISING);
}

void loop() {

  float targetRV, targetLV;
  trajectoryPlanner(&targetRV, &targetLV);
  motorPositionControl(targetRV,targetLV);
  
  Serial.print(targetLV);
  Serial.print(" ");
  Serial.println(targetRV);

}

// Function to generate joint positions as a function of time
void trajectoryPlanner(float* targetRVal, float* targetLVal )   
{

  float time = timer(runTime);
  float t = time / 1000;

  float pi = 3.1415926;  
  float q1 = ((pi*sign((42*pow(t,3))/25 - (63*pow(t,2))/5 + 210)*(sign((42*pow(t,3))/25 - (63*pow(t,2))/5 + 210) - 1))/2 - atan2(210*sqrt(1 - pow(((42*pow(t,3))/25 - (63*pow(t,2))/5 + 210),2)/44100), pow(((42*pow(t,3))/25 - (63*pow(t,2))/5 + 210),2)/ sqrt((pow( ((42*pow(t,3))/25 - (63*pow(t,2))/5 + 210),2)))));
  float q2 = (pi - atan2( sqrt(1 - pow(( pow(((42*pow(t,3))/25 - (63*pow(t,2))/5 + 210),2)/22050 - 1),2)) , 1 - pow(((42*pow(t,3))/25 - (63*pow(t,2))/5 + 210),2)/22050));

  *targetRVal = q1*(180/3.14);
  *targetLVal = q2*(180/3.14);

}

double sign(float x) {
  if (x < 0) {
    return -1.0;
  } else if (x > 0) {
    return 1.0;
  } else {
    return 0.0;
  }
}

void motorPositionControl(float targetR,float targetL)
{
  float angleR = calculateAngle(posR); 
  float angleL = calculateAngle(posL); 

  Serial.print(angleL);
  Serial.println(angleR);


  float errorR = targetR - angleR;
  float errorL = targetL - angleL;

  long currT = micros();
  long prevT = 0;
  float eprev = 0;
  long et = currT - prevT;
  float deltaT = ((float)(currT - prevT))/1.0e6;
  prevT = currT;  

  float dedtR = (-errorR + eprev)/(deltaT);
  float dedtL = (-errorL + eprev)/(deltaT);

  float pwmR = KpR*errorR + KdR*dedtR;
  float pwmL = KpL*errorL + KdL*dedtL;

  if (pwmR > 0){
    dirR = 0;
  } else{
    dirR = 1;
  }

  if (pwmL > 0){
    dirL = 0;
  } else{
    dirL = 1;
  }

  float powerR = abs(pwmR);
  float powerL = abs(pwmL);

  if (powerR>150){
    powerR = 150;
  }

  if (powerL>150){
    powerL = 150;
  }

  digitalWrite(dirPinR,dirR);
  analogWrite(pwmPinR,powerR);
  digitalWrite(dirPinL,dirL);
  analogWrite(pwmPinL,powerL);

  // Print values to check PID
  // Serial.print(angleL);
  // Serial.print(" ");
  // Serial.print(powerL);
  // Serial.print(" ");
  // Serial.print(dirL);
  // Serial.print(" ");
  // Serial.print(angleR);
  // Serial.print(" ");
  // Serial.print(powerR);
  // Serial.print(" ");
  // Serial.println(dirR);
  // delay(50);

}

// Function to return X and Y as a function of time
// void endEffector(float* x, float* y) 
// {
//   float time = timer(runTime);
//   float t = time / 1000;
//   *x = 25.2*pow(t,2) - 3.36*pow(t,3);
//   *y = 0;
// }

// Funtion to return time in milliseconds variable 'runT' being run time 
float timer(float runT)
{
  static unsigned long startTime = millis();  
  unsigned long currentTime = millis();  
  unsigned long elapsedTime = currentTime - startTime;  
  if (elapsedTime < runT*1000) {  
    return elapsedTime;
  }
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
