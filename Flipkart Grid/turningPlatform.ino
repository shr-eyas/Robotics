#define centralAxis 11 //MG996R continuous servo motor
#define electro 13 //electromagnet on the turning platform
#define flipAxis_1 10 //in1 and in2 pins of L298N motor driver to control the 10 rpm dc motor
#define flipAxis_2 9

void setup() {
  pinMode(centralAxis, OUTPUT);
  pinMode(flipAxis_1, OUTPUT);
  pinMode(flipAxis_2, OUTPUT);
  pinMode(electro, OUTPUT);


  digitalWrite(flipAxis_1, LOW);
  digitalWrite(flipAxis_2, LOW);
  analogWrite(centralAxis, 1560);
  digitalWrite(electro, LOW);
}

void loop() {
  recieveData();
  float angularTurn;
  boolean flip;
  // recieve angular turn needed and whether to flip or not from image processing code
  // put your main code here, to run repeatedly:
  electroOn();
  turn(angularTurn);
  if(flip){
    flipPlatform();
    electroOff();
    flipBack();
  }
  else{
    electroOff();
    dropPackage();
  }
}

void turn(float deg){
  //servo turns 180 degrees clockwise at a delay of 800 when PWM is 1590
  //180/800 = 0.225
  float del = deg/0.225;
  analogWrite(centralAxis, 1590);
  delay(del);
  analogWrite(centralAxis, 1560);
  delay(200); // 200 ms pause before next command
}

void flipPlatform(){
  digitalWrite(flipAxis_1, HIGH);
  digitalWrite(flipAxis_2, LOW);
  delay(0); // delay needed for 10 rpm motor to turn 180 degrees
  digitalWrite(flipAxis_1, LOW);
  digitalWrite(flipAxis_2, LOW);
  delay(200); //200 ms delay before next command 
}

void flipBack(){
  digitalWrite(flipAxis_1, LOW);
  digitalWrite(flipAxis_2, HIGH);
  delay(0); // delay needed for 10 rpm motor to turn 180 degrees
  digitalWrite(flipAxis_1, LOW);
  digitalWrite(flipAxis_2, LOW);
  delay(200); //200 ms delay before next command 
}

void dropPackage(){
  digitalWrite(flipAxis_1, HIGH);
  digitalWrite(flipAxis_2, LOW);
  delay(0); // delay needed to turn platform 60 degrees
  digitalWrite(flipAxis_1, LOW);
  digitalWrite(flipAxis_2, LOW);
  delay(1500); // delay of 1.5 secs to let the package drop
  digitalWrite(flipAxis_1, LOW);
  digitalWrite(flipAxis_2, HIGH);
  delay(0); // same delay as before to turn platform back to original state
  digitalWrite(flipAxis_1, LOW);
  digitalWrite(flipAxis_2, LOW);
  delay(200); //200 ms delay before next command 
}

void electroOn(){
  digitalWrite(electro, HIGH);
}

void electroOff(){
  digitalWrite(electro, LOW);
}

void recieveData(){

}
