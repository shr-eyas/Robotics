// Define the connections to the DRV8825 driver
#define STEP_PIN 8
#define DIR_PIN 9
#define ENABLE_PIN 10
//200 steps=40mm
void setup() {
  // Set up the motor control pins as outputs
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(ENABLE_PIN, OUTPUT);
  
  // Disable the motor (active low)
  digitalWrite(ENABLE_PIN, HIGH);
  // Set the desired number of steps and direction
  int numberOfSteps = 2500; // Change this to the desired number of steps
  boolean direction = HIGH; // HIGH for one direction, LOW for the other
  
  // Enable the motor (active low)
  digitalWrite(ENABLE_PIN, LOW);
  
  // Set the direction
  digitalWrite(DIR_PIN, direction);
  
  // Pulse the step pin to move the motor
  for (int i = 0; i < numberOfSteps; i++) {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(500); // Adjust this delay for your desired speed
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(500); // Adjust this delay for your desired speed
  }
  digitalWrite(ENABLE_PIN, HIGH);
  
  delay(1000);

  direction = LOW;
  digitalWrite(DIR_PIN, direction);
  digitalWrite(ENABLE_PIN, LOW);
  // for (int i = 0; i < numberOfSteps; i++) {
  //   digitalWrite(STEP_PIN, HIGH);
  //   delayMicroseconds(500); // Adjust this delay for your desired speed
  //   digitalWrite(STEP_PIN, LOW);
  //   delayMicroseconds(500); // Adjust this delay for your desired speed
  // }
  // Disable the motor (active low)
  digitalWrite(ENABLE_PIN, HIGH);
  
  delay(1000); // Wait for a second before the next movement
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
