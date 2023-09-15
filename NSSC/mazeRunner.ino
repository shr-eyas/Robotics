const int trigPinR = 2;
const int echoPinR = 3;
const int trigPinF = 4;
const int echoPinF = 5;
const int trigPinL = 6;
const int echoPinL = 7;
const int dirPinR = 8;
const int dirPinL = 9;
const int pwrPinR = 10;
const int pwrPinL = 11;

const double baseSpeed = 100;

long durationR, durationF, durationL;
double distanceR, distanceF, distanceL, requiredR, thresholdDistance;

// PID variables
double error, proportional, derivative, PIDValue;
double lastProportional = 0;

// PID constants
double Kp = 11;
double Kd = 4;
double Ki = 0;

// Encoder variables
double encoderR, encoderL;

void setup() {

  pinMode(trigPinR, OUTPUT); 
  pinMode(echoPinR, INPUT); 
  pinMode(trigPinF, OUTPUT); 
  pinMode(echoPinF, INPUT); 
  pinMode(trigPinL, OUTPUT); 
  pinMode(echoPinL, INPUT); 

  Serial.begin(9600); 
}

void loop() {
  
  digitalWrite(trigPinR, LOW);
  digitalWrite(trigPinF, LOW);
  digitalWrite(trigPinL, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPinR, HIGH);
  digitalWrite(trigPinF, HIGH);
  digitalWrite(trigPinL, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPinR, LOW);
  digitalWrite(trigPinL, LOW);
  digitalWrite(trigPinF, LOW);

  durationR = pulseIn(echoPinR, HIGH);
  durationF = pulseIn(echoPinF, HIGH);
  durationL = pulseIn(echoPinL, HIGH); 

  distanceR = durationR*0.034/2;
  distanceF = durationF*0.034/2;
  distanceL = durationL*0.034/2;

  requiredR = (distanceR + distanceL)/2;
  error = requiredR-distanceR;

  proportional = error; 
  derivative = proportional - lastProportional;
  lastProportional = proportional;
  PIDValue = Kp*proportional + Kd*derivative;

  if (distanceF>thresholdDistance) {
    digitalWrite(dirPinR, HIGH);
    analogWrite(pwrPinR, baseSpeed+PIDValue);
    digitalWrite(dirPinL, HIGH);
    analogWrite(pwrPinL, baseSpeed-PIDValue);

  }
  else {
    while(encoderR<500 && encoderL<500)
    digitalWrite(dirPinR, HIGH);
    analogWrite(pwrPinR, baseSpeed);
    digitalWrite(dirPinL, LOW);
    analogWrite(pwrPinL, baseSpeed);
  }

}
