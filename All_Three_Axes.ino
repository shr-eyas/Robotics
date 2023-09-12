// Define the connections to the DRV8825 driver
#define STEP_PIN_Y 8
#define DIR_PIN_Y 2
#define ENABLE_PIN_Y 3

#define STEP_PIN_X 9
#define DIR_PIN_X 4
#define ENABLE_PIN_X 5

#define servoPin 10

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
  // 2595 for end to end (X Axis)
  // 3500 for end to end (Y Axis)
  // 0.2 mm/step
  // 5 step/mm

  // Disable the motor (active low)
  digitalWrite(ENABLE_PIN_Y, HIGH);
  digitalWrite(ENABLE_PIN_X, HIGH);
  // Set the desired number of steps and direction
  int numberOfSteps = 1250; // Change this to the desired number of steps
  boolean direction_y = LOW; // Motor Y = HIGH for ACW, LOW for CW (from base of motor)
  boolean direction_x = HIGH; // Motor X = HIGH for ACW, LOW for CW (from base of motor)
  
  // Enable the motor (active low)
  digitalWrite(ENABLE_PIN_Y, LOW);
  digitalWrite(ENABLE_PIN_X, LOW);

  // Set the direction
  digitalWrite(DIR_PIN_Y, direction_y);
  digitalWrite(DIR_PIN_X, direction_x);
  
  // Pulse the step pin to move the motor
  for (int i = 0; i < numberOfSteps; i++) {
    digitalWrite(STEP_PIN_X, HIGH);
    digitalWrite(STEP_PIN_Y, HIGH);
    delayMicroseconds(500); // Adjust this delay for your desired speed
    digitalWrite(STEP_PIN_X, LOW);
    digitalWrite(STEP_PIN_Y, LOW);
    delayMicroseconds(500); // Adjust this delay for your desired speed
  }
  digitalWrite(ENABLE_PIN_X, HIGH);
  digitalWrite(ENABLE_PIN_Y, HIGH);
  
  delay(1000);

    analogWrite(servoPin, 1530);
    delay(2000);
    analogWrite(servoPin, 1560);
    delay(1000);

  // direction = LOW;
  // digitalWrite(DIR_PIN, direction);
  // digitalWrite(ENABLE_PIN, LOW);
  // for (int i = 0; i < numberOfSteps; i++) {
  //   digitalWrite(STEP_PIN, HIGH);
  //   delayMicroseconds(500); // Adjust this delay for your desired speed
  //   digitalWrite(STEP_PIN, LOW);
  //   delayMicroseconds(500); // Adjust this delay for your desired speed
  // }
  // Disable the motor (active low)
  // digitalWrite(ENABLE_PIN, HIGH);
  
  // delay(1000); // Wait for a second before the next movement
}
// HIGH direction is clockwise seeing from base of stepper

void loop() {
  // // Set the desired number of steps and direction
  // int numberOfSteps = 1600; // Change this to the desired number of steps
  // boolean direction = HIGH; // HIGH for one direction, LOW for the other
  
  // // Enable the motor (active low)
  // digitalWrite(ENABLE_PIN, LOW);
  
  // // Set the direction
  // digitalWrite(DIR_PIN, direction);
  
  // // Pulse the step pin to move the motor
  // for (int i = 0; i < numberOfSteps; i++) {
  //   digitalWrite(STEP_PIN, HIGH);
  //   delayMicroseconds(500); // Adjust this delay for your desired speed
  //   digitalWrite(STEP_PIN, LOW);
  //   delayMicroseconds(500); // Adjust this delay for your desired speed
  // }
  // digitalWrite(ENABLE_PIN, HIGH);
  
  // delay(1000);

  // direction = LOW;
  // digitalWrite(DIR_PIN, direction);
  // digitalWrite(ENABLE_PIN, LOW);
  // // for (int i = 0; i < numberOfSteps; i++) {
  // //   digitalWrite(STEP_PIN, HIGH);
  // //   delayMicroseconds(500); // Adjust this delay for your desired speed
  // //   digitalWrite(STEP_PIN, LOW);
  // //   delayMicroseconds(500); // Adjust this delay for your desired speed
  // // }
  // // Disable the motor (active low)
  // digitalWrite(ENABLE_PIN, HIGH);
  
  // delay(1000); // Wait for a second before the next movement
}
