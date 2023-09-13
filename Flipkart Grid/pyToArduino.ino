void setup() {
  Serial.begin(115200);
  pinMode(13, OUTPUT);
}

void loop() {

  while (Serial.available()==0) {}

  //if (Serial.available()) {

    // Read the incoming string
    String input = Serial.readStringUntil('/n');
    // String input="200v300";
    int d = input.indexOf("v");

    int x = input.substring(0, d).toInt();
    int y = input.substring(d+1).toInt();

    if ((x+y) == 500) {
      digitalWrite(13, HIGH);
    }
  //}
}
