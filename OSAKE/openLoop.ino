const int dir1 = 4;
const int pwm1 = 5;
const int dir2 = 6;
const int pwm2 = 7;

int speed = 50;

void setup() {
  // put your setup code here, to run once:
  pinMode(dir1,OUTPUT);
  pinMode(pwm1,OUTPUT);
  pinMode(dir2,OUTPUT);
  pinMode(pwm2,OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(dir1,0);
  analogWrite(pwm1,speed);
  analogWrite(dir2,0);
  analogWrite(pwm2,speed);

}
