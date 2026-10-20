const int lever1NoPin = 2;
const int lever1NcPin = 3;

const int lever2NoPin = 9;
const int lever2NcPin = 10;

// Connect each lever's COM terminal to Arduino GND.

void setup() {
  pinMode(lever1NoPin, INPUT_PULLUP);
  pinMode(lever1NcPin, INPUT_PULLUP);
  pinMode(lever2NoPin, INPUT_PULLUP);
  pinMode(lever2NcPin, INPUT_PULLUP);

  Serial.begin(115200);
  delay(500);
  Serial.println("Two-lever switch test starting...");
}

const char* getLeverState(int noPin, int ncPin) {
  bool noClosed = (digitalRead(noPin) == LOW);
  bool ncClosed = (digitalRead(ncPin) == LOW);

  if (noClosed && !ncClosed) {
    return "DEPRESSED";
  } else if (!noClosed && ncClosed) {
    return "UNDEPRESSED";
  } else {
    return "TRANSITION / BOUNCE / WIRING CHECK";
  }
}

void loop() {
  const char* lever1State = getLeverState(lever1NoPin, lever1NcPin);
  const char* lever2State = getLeverState(lever2NoPin, lever2NcPin);

  Serial.print("LEVER 1: ");
  Serial.print(lever1State);
  Serial.print(", LEVER 2: ");
  Serial.println(lever2State);

  delay(200);  // Print both states five times per second.
}