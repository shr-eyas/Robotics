const int dir1 = 4;
const int pwm1 = 5;
const int dir2 = 6;
const int pwm2 = 7;
#define ENC1A 18
#define ENC1B 19
#define ENC2A 2
#define ENC2B 3

int posR = 0;
int posL = 0;

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

  Serial.print(posR);
  Serial.print(" ");
  Serial.println(posL);
  }


void readEncoder1(){
  int b = digitalRead(ENC1B);
  if(b>0){
    posR++;
  }
  else{
    posR--;
  }
}

void readEncoder2(){
  int b = digitalRead(ENC2B);
  if(b>0){
    posL++;
  }
  else{
    posL--;
  }
}
