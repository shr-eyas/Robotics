const int MLD = 25; // Connect to GPIO25 Left Direction
const int MRD = 14; // Connect to GPIO14 Right Direction

// PWM variables 
const int MLP = 0;
const int MRP = 1;
const int freq = 5000;
const int resolution = 8;

int speed = 50;

void setup() {
  // Motor PWM setup
  ledcSetup(MLP, freq, resolution);
  ledcSetup(MRP, freq, resolution);
  pinMode(MRD, OUTPUT);
  pinMode(MLD, OUTPUT);
  ledcAttachPin(33, MLP); // Connect to GPIO27 Right PWM
  ledcAttachPin(27, MRP); // Connect to GPIO33 Left PWM
}

void loop() {
    // Open loop control
    digitalWrite(MLD, 0);
    digitalWrite(MRD, 0);
    ledcWrite(MLP, speed);
    ledcWrite(MRP, speed);
}
