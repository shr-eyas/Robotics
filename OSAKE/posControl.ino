// position control for joint 1 using PD control (target variable named "target1")

const int dir1 = 4;
const int pwm1 = 5;
const int dir2 = 6;
const int pwm2 = 7;
#define ENC1A 18
#define ENC1B 19
#define ENC2A 2
#define ENC2B 3

int pos1 = 0;
int pos2 = 0;

int speed = 0;
float Kp1 = 0;
float Kd = 0;
int target1 = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  pinMode(ENC1A,INPUT);
  pinMode(ENC1B,INPUT);
  pinMode(ENC2A,INPUT);
  pinMode(ENC2B,INPUT);

  pinMode(dir1,OUTPUT);
  pinMode(pwm1,OUTPUT);
  pinMode(dir2,OUTPUT);
  pinMode(pwm2,OUTPUT);

  attachInterrupt(digitalPinToInterrupt(ENC1A),readEncoder1,RISING);
  attachInterrupt(digitalPinToInterrupt(ENC2A),readEncoder2,RISING);

}

void loop() {

  long currT = micros();
  long prevT = 0;
  float eprev = 0;
  long et = currT - prevT;
  float deltaT = ((float)(currT - prevT))/1.0e6;
  prevT = currT;  

  int error1 =  pos1 - target1;

  float dedt1 = (-error1 + eprev)/(deltaT);
  
  float u1 = Kp1*error1 + Kd*dedt1;

  float pwr1 = fabs(u1);
  if(pwr1 > 255){
    pwr1 = 255;
  }
  int dirVal1 = 1;
  if (u1<0){
    dirVal1 = 0;
  }


  Serial.print(error1);
  Serial.print(" ");
  Serial.print(u1);
  Serial.print(" ");
  Serial.print(dirVal1);
  Serial.print(" ");
  Serial.println(pwr1);

  digitalWrite(dir1,dirVal1);
  analogWrite(pwm1,pwr1);
}

void readEncoder1(){
  int b = digitalRead(ENC1B);
  if(b>0){
    pos1++;
  }
  else{
    pos1--;
  }
}

void readEncoder2(){
  int b = digitalRead(ENC2B);
  if(b>0){
    pos2++;
  }
  else{
    pos2--;
  }
}
