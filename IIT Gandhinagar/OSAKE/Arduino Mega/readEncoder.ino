#define ENC1A 18
#define ENC1B 19
#define ENC2A 2
#define ENC2B 3

float posR = 0;
float posL = 0;

const int encoderCountsPerRevolution = 2048 * 2;

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

void setup() {

  Serial.begin(9600);

  pinMode(ENC1A, INPUT);
  pinMode(ENC1B, INPUT);
  pinMode(ENC2A, INPUT);
  pinMode(ENC2B, INPUT);

  attachInterrupt(digitalPinToInterrupt(ENC1A), readEncoder1, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC2A), readEncoder2, RISING);
}

void loop(){

  float angleR = calculateAngle(posR); 
  float angleL = calculateAngle(posL); 
  Serial.print(angleL);
  Serial.print(" ");
  Serial.println(angleR);
}
