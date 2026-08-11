const byte noPin = 2;       // lever NO
const byte ncPin = 3;       // lever NC
const byte buzzerPin = 5;   // buzzer output
const byte solenoidPin = 7;   // solenoid driver output (relay/MOSFET gate)

// USER SETTINGS
const double X_seconds = 1;      // wait time before each trial starts
const unsigned long Y_seconds = 60;      // response window
const unsigned long debounceMs = 10;
const unsigned int buzzerFreq = 4000;
const unsigned long buzzerMs = 500;
const unsigned long solenoidMs = 67;
const unsigned int pressesRequiredForWater = 3;
unsigned int consecutivePressStreak = 0;

//SOLENOID STATE
bool solenoidActive = false;
unsigned long solenoidOffMs = 0;
bool waterDelivered = false;

// STATE MACHINE
enum SystemState {
  IDLE,
  WAIT_BETWEEN_TRIALS,
  TRIAL_ACTIVE,
  WAIT_FOR_MATLAB
};

SystemState systemState = WAIT_BETWEEN_TRIALS;

// LEVER STATE
enum LeverState {
  LEVER_UNPRESSED,
  LEVER_PRESSED,
  LEVER_INVALID
};

LeverState lastRawState = LEVER_INVALID;
LeverState stableState = LEVER_INVALID;
unsigned long lastRawChangeTime = 0;

// TRIAL VARIABLES
unsigned long nextTrialStartMs = 0;
unsigned long trialEndMs = 0;
unsigned long trialNumber = 0;
unsigned long pressCount = 0;
// For VI
bool sawPressThisTrial = false;

// SERIAL COMMAND BUFFER
String cmdBuffer = "";

// Read raw lever state from SPDT switch
LeverState readRawLeverState() {
  bool noClosed = (digitalRead(noPin) == LOW); // LOW means connected to COM/GND
  bool ncClosed = (digitalRead(ncPin) == LOW);

  if (noClosed && !ncClosed) {
    return LEVER_PRESSED;
  } else if (!noClosed && ncClosed) {
    return LEVER_UNPRESSED;
  } else {
    return LEVER_INVALID;
  }
}

// Debounce lever and count new presses only during active trial
void updateLever(unsigned long nowMs) {
  LeverState rawState = readRawLeverState();

  if (rawState != lastRawState) {
    lastRawState = rawState;
    lastRawChangeTime = nowMs;
  }

  if ((nowMs - lastRawChangeTime) >= debounceMs && rawState != stableState) {
    LeverState previousStableState = stableState;
    stableState = rawState;

    if (systemState == TRIAL_ACTIVE &&
        stableState == LEVER_PRESSED &&
        previousStableState != LEVER_PRESSED) {
      pressCount++;
      sawPressThisTrial = true;
    }
  }
}

void startTrial(unsigned long nowMs) {
  trialNumber++;
  pressCount = 0;
  sawPressThisTrial = false;
  waterDelivered = false;
  trialEndMs = nowMs + Y_seconds * 1000UL;
  systemState = TRIAL_ACTIVE;

  tone(buzzerPin, buzzerFreq, buzzerMs); // sets off buzzer

  Serial.print("TRIAL_START,");
  Serial.println(trialNumber);
}

void endTrial(unsigned long nowMs) {
  systemState = WAIT_FOR_MATLAB;

  // Fixed interval trials
  if (sawPressThisTrial) {
    triggerSolenoid(nowMs);
    waterDelivered = true;
  }

  // triggerSolenoid(nowMs);
  // waterDelivered = true;

  // Variable interval trials
  // if (sawPressThisTrial) {
  //   consecutivePressStreak++;
  //   //Serial.println(consecutivePressStreak++); /// for debugging
  // } else {
  //   consecutivePressStreak = 0;
  // }
  // bool waterDelivered = false;
  // if (consecutivePressStreak >= pressesRequiredForWater) {
  //   triggerSolenoid(nowMs);
  //   waterDelivered = true;
  //   consecutivePressStreak = 0;   // reset the streak after rewarding
  // }

  Serial.print("TRIAL_END,");
  Serial.print(trialNumber);
  Serial.print(",");
  Serial.print(pressCount);
  Serial.print(",");
  Serial.print(sawPressThisTrial ? 1 : 0);
  Serial.print(",");
  Serial.println(waterDelivered ? 1 : 0); 
}

void handleCommand(String cmd, unsigned long nowMs) {
  cmd.trim();

  if (cmd.equalsIgnoreCase("CONTINUE")) {
    if (systemState == WAIT_FOR_MATLAB) {
      nextTrialStartMs = nowMs + X_seconds * 1000UL;
      systemState = WAIT_BETWEEN_TRIALS;

      Serial.print("ACK_CONTINUE,");
      Serial.println(trialNumber);
    }
  } else if (cmd.equalsIgnoreCase("START")) {
    trialNumber = 0;
    pressCount = 0;
    sawPressThisTrial = false;  // for VI
    consecutivePressStreak = 0; // for VI
    nextTrialStartMs = nowMs;
    systemState = WAIT_BETWEEN_TRIALS;
    Serial.println("ACK_START");
  } else if (cmd.equalsIgnoreCase("STOP")) {
    systemState = WAIT_FOR_MATLAB;
    solenoidActive = false;
    Serial.println("ACK_STOP");
  }
}

void readSerialCommands(unsigned long nowMs) {
  while (Serial.available() > 0) {
    char c = Serial.read();

    if (c == '\n') {
      handleCommand(cmdBuffer, nowMs);
      cmdBuffer = "";
    } else if (c != '\r') {
      cmdBuffer += c;
    }
  }
}

void triggerSolenoid(unsigned long nowMs) {
  digitalWrite(solenoidPin, HIGH); // trigger solenoid pin, deliver water
  solenoidActive = true;
  solenoidOffMs = nowMs + solenoidMs;
  //Serial.println("water on");
}

void setup() {
  pinMode(noPin, INPUT_PULLUP);
  pinMode(ncPin, INPUT_PULLUP);
  pinMode(buzzerPin, OUTPUT);
  pinMode(solenoidPin, OUTPUT);      
  digitalWrite(solenoidPin, LOW);    //ensure closed at boot

  Serial.begin(115200);
  delay(1000);

  lastRawState = readRawLeverState();
  stableState = lastRawState;
  lastRawChangeTime = millis();

  nextTrialStartMs = millis() + X_seconds * 1000UL;

  Serial.println("READY");
}

void loop() {
  unsigned long nowMs = millis();

  readSerialCommands(nowMs);
  updateLever(nowMs);

  if (systemState == WAIT_BETWEEN_TRIALS && nowMs >= nextTrialStartMs) {
    startTrial(nowMs);
  }

  if (systemState == TRIAL_ACTIVE && nowMs >= trialEndMs) {
    endTrial(nowMs);
  }
  if (solenoidActive && nowMs >= solenoidOffMs) {
    digitalWrite(solenoidPin, LOW); // deactivate solenoid pin, close water
    solenoidActive = false;
    //Serial.println("water off");
  }
}