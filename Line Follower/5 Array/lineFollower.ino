int motorBaseSpeed = 255;  
int motorTurnSpeed = 255;

//hit and trail zone starts
int Kp = 110;
int Ki = 0;
int Kd = 50;

//hit and trail zone ends

const int motorRPin1 = 2;
const int motorRPin2 = 5;
const int motorREnable = 11;
const int motorLPin1 = 12;
const int motorLPin2 = 13;  
const int motorLEnable = 3;

const int irPins[5] = {6, 7, 8, 9, 10};
int irVal[5];
int err, proportional, derivative, integral, PIDValue;
int lastProportional = 0;

void setup() 
{
  pinMode(motorRPin1, OUTPUT);
  pinMode(motorRPin2, OUTPUT);
  pinMode(motorREnable, OUTPUT);
  pinMode(motorLPin1, OUTPUT);
  pinMode(motorLPin2, OUTPUT);
  pinMode(motorLEnable, OUTPUT);
  for (int i = 0; i < 5; i++) 
  {
    pinMode(irPins[i], INPUT);
  }  
}


void loop() 
{
  scan();
  run();
}

//taking input from IR sensor and storing it in an array 
void scan()
{
  for(int i=0; i<5; i++)
  {  
    irVal[i]=digitalRead(irPins[i]);
  }
}


void run()
{
  if      (irVal[0]==1 && irVal[1]==1 && irVal[2]==1 && irVal[3]==1 && irVal[4]==1) {err=0; reverse();}    //all white
  else if (irVal[0]==0 && irVal[1]==0 && irVal[2]==0 && irVal[3]==0 && irVal[4]==0) {err=0; motorLF();}    //all black

  else if (irVal[0]==1 && irVal[1]==0 && irVal[2]==1 && irVal[3]==1 && irVal[4]==1) {err = -2; motorLF();}  //01000
  else if (irVal[0]==1 && irVal[1]==0 && irVal[2]==0 && irVal[3]==1 && irVal[4]==1) {err = -1; motorLF();}  //01100
  else if (irVal[0]==1 && irVal[1]==1 && irVal[2]==0 && irVal[3]==1 && irVal[4]==1) {err = 0; motorLF();}   //00100
  else if (irVal[0]==1 && irVal[1]==1 && irVal[2]==0 && irVal[3]==0 && irVal[4]==1) {err = 1; motorLF();}   //00110
  else if (irVal[0]==1 && irVal[1]==1 && irVal[2]==1 && irVal[3]==0 && irVal[4]==1) {err = 2; motorLF();}   //00010

  else if (irVal[0]==0) {err = 0; leftTurn();}   //left
  else if (irVal[4]==0) {err = 0; rightTurn();}   //right
  else {err= 0; motorLF();}
}


void motorLF() // line follow
{  
  proportional = err; //1 
  derivative = proportional - lastProportional;
  integral = integral + proportional;
  lastProportional = proportional;
  PIDValue = Kp*proportional + Ki*integral + Kd*derivative;

  analogWrite(motorREnable, motorBaseSpeed+PIDValue);  // -ve error:speed increases:turn towards left 
  digitalWrite(motorRPin1, HIGH); 
  digitalWrite(motorRPin2, LOW);

  analogWrite(motorLEnable, motorBaseSpeed-PIDValue); // -ve error:speed decreases:turn towards left
  digitalWrite(motorLPin1, HIGH);
  digitalWrite(motorLPin2, LOW);
}

void reverse() // line follow
{  
  while(irVal[1]==1 && irVal[2]==1 && irVal[3]==1)
  {
  analogWrite(motorREnable, motorTurnSpeed);  // -ve error:speed increases:turn towards left 
  digitalWrite(motorRPin1, LOW); 
  digitalWrite(motorRPin2, HIGH);

  analogWrite(motorLEnable, motorTurnSpeed); // -ve error:speed decreases:turn towards left
  digitalWrite(motorLPin1, LOW);
  digitalWrite(motorLPin2, HIGH);
  scan();
  }
  
}


void motorStop()
{
  analogWrite(motorREnable, 0);
  digitalWrite(motorRPin1, HIGH);
  digitalWrite(motorRPin2, LOW);

  analogWrite(motorLEnable, 0);
  digitalWrite(motorLPin1, HIGH);
  digitalWrite(motorLPin2, LOW);  
  scan();
}

void leftTurn() 
{
   while(irVal[2]==1&&irVal[3]==1)
  {
    analogWrite(motorREnable, motorTurnSpeed);
    digitalWrite(motorRPin1, HIGH);
    digitalWrite(motorRPin2, LOW);

    analogWrite(motorLEnable, motorTurnSpeed);
    digitalWrite(motorLPin1, LOW);
    digitalWrite(motorLPin2, HIGH); 
    scan();
  }
}

void rightTurn() 
{
 
  // delay(goAheadTime); 
  motorStop();  
 // delay(500);
  while(irVal[2]==1&&irVal[1]==1)
  {
    analogWrite(motorREnable, motorTurnSpeed);
    digitalWrite(motorRPin1, LOW);
    digitalWrite(motorRPin2, HIGH);

    analogWrite(motorLEnable, motorTurnSpeed);
    digitalWrite(motorLPin1, HIGH);
    digitalWrite(motorLPin2, LOW); 
    scan();
  }
}


void motorUTurn() 
{
  //motorLF();
  //delay(goAheadTime); 
  motorStop();  
  delay(500);
  while(irVal[2]==1)
  {
    analogWrite(motorREnable, motorTurnSpeed);
    digitalWrite(motorRPin1, LOW);
    digitalWrite(motorRPin2, HIGH);

    analogWrite(motorLEnable, motorTurnSpeed);
    digitalWrite(motorLPin1, HIGH);
    digitalWrite(motorLPin2, LOW);
  }
}
