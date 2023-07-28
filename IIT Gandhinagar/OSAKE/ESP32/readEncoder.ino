// Encoder pins
#define RIGHT_ENCODER_A 13 
#define RIGHT_ENCODER_B 12 
#define LEFT_ENCODER_A 14 
#define LEFT_ENCODER_B 15

volatile int rightEncoderVal = 0; 
volatile int leftEncoderVal = 0;  

void rightEncoder() {
  int A = digitalRead(RIGHT_ENCODER_A);
  int B = digitalRead(RIGHT_ENCODER_B);
  if ((A == HIGH) != (B == LOW)) {
    rightEncoderVal--;
  } else {
    rightEncoderVal++;
  }
}

void leftEncoder() {
  int A = digitalRead(LEFT_ENCODER_A);
  int B = digitalRead(LEFT_ENCODER_B);
  if ((A == HIGH) != (B == LOW)) {
    leftEncoderVal--;
  } else {
    leftEncoderVal++;
  }
}

void setup() {
  Serial.begin(115200); 

  pinMode(RIGHT_ENCODER_A, INPUT_PULLUP);
  pinMode(RIGHT_ENCODER_B, INPUT_PULLUP);
  pinMode(LEFT_ENCODER_A, INPUT_PULLUP);
  pinMode(LEFT_ENCODER_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_A), rightEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_A), leftEncoder, CHANGE);
}

void loop() {
  volatile int thetaLeft = (leftEncoderVal*360/(2*4096));
  volatile int thetaRight = (rightEncoderVal*360/(2*4096))*1.02857;

  // Serial printing the angles
  Serial.print(thetaLeft);
  Serial.print(" ");
  Serial.println(thetaRight);

}
