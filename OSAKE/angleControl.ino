// Code for controlling joint angles of OSAKE

const int dirPinL = 4;
const int pwmPinL = 5;
const int dirPinR = 6;
const int pwmPinR = 7;

#define ENC1A 18
#define ENC1B 19
#define ENC2A 2
#define ENC2B 3

const int encoderCountsPerRevolution = 2048*2; 
int dirR,dirL;

float posR = 0;
float posL = 0;

int speedR = 0;
int speedL = 0;

// Target values 
float targetR = 0;
float targetL = 0;

// PD constants 
float KpR = 1;
float KpL = 3;
float KdR = 0;
float KdL = 5;

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

  float angleR = calculateAngle(posR); 
  float angleL = calculateAngle(posL); 

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
  
  Serial.print(pwmL);
  Serial.print(" ");
  Serial.print(powerL);
  Serial.print(" ");
  Serial.println(dirL);

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
  float angle = (encoderPosition * 360.0) / encoderCountsPerRevolution;
  return angle;
}
