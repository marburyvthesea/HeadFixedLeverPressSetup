// void setup() {
//   Serial.begin(115200);
// }

// void loop() {
//   Serial.println("Arduino is running");
//   delay(1000);
// }

const int noPin = 2;   // switch NO terminal
const int ncPin = 3;   // switch NC terminal

void setup() {
  pinMode(noPin, INPUT_PULLUP);
  pinMode(ncPin, INPUT_PULLUP);

  Serial.begin(115200);
  delay(500);
  Serial.println("Lever switch test starting...");
}

void loop() {
  bool noClosed = (digitalRead(noPin) == LOW);
  bool ncClosed = (digitalRead(ncPin) == LOW);

  if (noClosed && !ncClosed) {
    Serial.println("DEPRESSED");
  }
  else if (!noClosed && ncClosed) {
    Serial.println("UNDEPRESSED");
  }
  else {
    Serial.println("TRANSITION / BOUNCE / WIRING CHECK");
  }

  delay(50);
}
