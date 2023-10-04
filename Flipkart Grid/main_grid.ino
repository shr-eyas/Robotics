#define STEP_PIN_Y 8
#define DIR_PIN_Y 2
#define ENABLE_PIN_Y 3

#define STEP_PIN_X 9
#define DIR_PIN_X 4
#define ENABLE_PIN_X 5

#define servoPin 10

#define zElectro 14
#define lim_X_Y 15
#define limZ 16

const int coToDist = 0;
const int servoNeutral = 1560;  // Adjust this value to set the servo's neutral position
const int servoUp = 1530;
const int servoDown = 1590;
const int turningX = 0;
const int turningY = 0;
int servoDirection = 1;
int x;
int y;


//200 steps=40mm
void setup() {

  // Set up the motor control pins as outputs
  pinMode(STEP_PIN_Y, OUTPUT);
  pinMode(DIR_PIN_Y, OUTPUT);
  pinMode(ENABLE_PIN_Y, OUTPUT);
  pinMode(STEP_PIN_X, OUTPUT);
  pinMode(DIR_PIN_X, OUTPUT);
  pinMode(ENABLE_PIN_X, OUTPUT);
  pinMode(servoPin, OUTPUT);
  digitalWrite(servoPin, LOW);
  pinMode(servoPin, OUTPUT);
  pinMode(limZ, INPUT_PULLUP);
  Serial.begin(9600);
}

void loop() {
  x, y = recieveCoordinates();
  move(x, y);
  int currentMillis = millis();
  zDown(currentMillis);
  turnElectroOn();
  //zUp(); (called from zDown)
  goToTurning();
  currentMillis = millis();
  zDown(currentMillis);
  turnElectroOff();
  //zUp(); (called from zDown)
  setToHome();

  // put your main code here, to run repeatedly:

}

void move(int x, int y){
  // Set the desired number of steps and direction
  int numberOfSteps_x = x * coToDist; // Change this to the desired number of steps
  int numberOfSteps_y = y * coToDist;
  boolean direction_y = HIGH; // Motor Y = HIGH for ACW, LOW for CW (from base of motor)
  boolean direction_x = HIGH; // Motor X = HIGH for ACW, LOW for CW (from base of motor)
  
  // Enable the motor (active low)
  digitalWrite(ENABLE_PIN_Y, LOW);
  digitalWrite(ENABLE_PIN_X, LOW);

  // Set the direction
  digitalWrite(DIR_PIN_Y, direction_y);
  digitalWrite(DIR_PIN_X, direction_x);
  
  // Pulse the step pin to move the motor
  for (int i = 0; i < numberOfSteps_x; i++) {
    digitalWrite(STEP_PIN_X, HIGH);
    delayMicroseconds(500); // Adjust this delay for your desired speed
    digitalWrite(STEP_PIN_X, LOW);
    delayMicroseconds(500); // Adjust this delay for your desired speed
  }

  for (int i = 0; i < numberOfSteps_y; i++) {
    digitalWrite(STEP_PIN_Y, HIGH);
    delayMicroseconds(500); // Adjust this delay for your desired speed
    digitalWrite(STEP_PIN_Y, LOW);
    delayMicroseconds(500); // Adjust this delay for your desired speed
  }

  digitalWrite(ENABLE_PIN_X, HIGH);
  digitalWrite(ENABLE_PIN_Y, HIGH);
}

void turnElectroOn(){
  digitalWrite(zElectro, HIGH);
}

// void zMove(){
//   if (digitalRead(limZ) == LOW) {
//     // Limit switch is pressed, stop the servo
//     digitalWrite(servoPin, LOW);
//     delay(500);  // Add a delay to avoid rapid switch state changes
//     turnElectroOn();
//     servoDirection = (-1 * servoDirection);
//     int servoSpeed = 30; // Adjust this value for desired speed
//     int servoPosition = servoNeutral + (servoDirection * servoSpeed);
    
//     servoPosition = constrain(servoPosition, 1520, 1600);
    
//     analogWrite(servoPin, servoPosition);
//   } else {
//     //add millis function to store time taken in going down
//     // Limit switch is not pressed, continue rotating the servo
//     int servoSpeed = 30; // Adjust this value for desired speed
//     int servoPosition = servoNeutral + (servoDirection * servoSpeed);
    
//     // Ensure servoPosition is within valid range
//     servoPosition = constrain(servoPosition, 1520, 1600);
    
//     analogWrite(servoPin, servoPosition);
//   }
// }

void zDown(int currentMillis){
  
  while (digitalRead(limZ) == HIGH) {
    // Limit switch is not pressed, continue rotating the servo
    analogWrite(servoPin, servoDown);
  }
    int previousMillis = currentMillis;
    currentMillis = millis();
    int delayReq = currentMillis - previousMillis;
    zUp(delayReq);
    // Limit switch is pressed, stop the servo and bring it back up
}

void zUp(int delayReq){
  digitalWrite(servoPin, LOW);
  delay(100);  // Add a delay to avoid rapid switch state changes
  
  analogWrite(servoPin, servoUp);
  delay(delayReq);
}

void goToTurning(){
  int xDist = turningX - x;
  int yDist = turningY - y;
  move(xDist, yDist);
}

void turnElectroOff(){
  digitalWrite(zElectro, LOW);
}

void setToHome(){
  int numberOfSteps_x = 0; // steps required from turning platform to home
  int numberOfSteps_y = 0;
  boolean direction_y = LOW; // Motor Y = HIGH for ACW, LOW for CW (from base of motor)
  boolean direction_x = LOW; // Motor X = HIGH for ACW, LOW for CW (from base of motor)
  
  // Enable the motor (active low)
  digitalWrite(ENABLE_PIN_Y, LOW);
  digitalWrite(ENABLE_PIN_X, LOW);

  // Set the direction
  digitalWrite(DIR_PIN_Y, direction_y);
  digitalWrite(DIR_PIN_X, direction_x);
  
  // Pulse the step pin to move the motor
  for (int i = 0; i < numberOfSteps_x; i++) {
    digitalWrite(STEP_PIN_X, HIGH);
    delayMicroseconds(500); // Adjust this delay for your desired speed
    digitalWrite(STEP_PIN_X, LOW);
    delayMicroseconds(500); // Adjust this delay for your desired speed
  }

  for (int i = 0; i < numberOfSteps_y; i++) {
    digitalWrite(STEP_PIN_Y, HIGH);
    delayMicroseconds(500); // Adjust this delay for your desired speed
    digitalWrite(STEP_PIN_Y, LOW);
    delayMicroseconds(500); // Adjust this delay for your desired speed
  }

  digitalWrite(ENABLE_PIN_X, HIGH);
  digitalWrite(ENABLE_PIN_Y, HIGH);
}

int recieveCoordinates(){
}
