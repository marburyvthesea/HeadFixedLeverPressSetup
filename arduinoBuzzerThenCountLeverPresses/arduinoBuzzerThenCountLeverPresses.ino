const byte lever1NoPin = 2;    // lever 1 NO
const byte lever1NcPin = 3;    // lever 1 NC
const byte lever2NoPin = 9;    // lever 2 NO
const byte lever2NcPin = 10;   // lever 2 NC
const byte buzzerPin = 5;      // buzzer output
const byte solenoidPin = 7;    // solenoid driver output

// USER SETTINGS
const unsigned long X_seconds = 1;      // wait time before each trial starts
const unsigned long Y_seconds = 5;      // response window
const unsigned long debounceMs = 10;
const unsigned int buzzerFreq = 4000;
const unsigned long buzzerMs = 500;
const unsigned long solenoidMs = 220;
const unsigned int pressesRequiredForWater = 3;  // consecutive trials w/ a press needed for water

// STATE MACHINE
enum SystemState {
  IDLE,
  WAIT_BETWEEN_TRIALS,
  TRIAL_ACTIVE,
  WAIT_FOR_MATLAB
};
SystemState systemState = IDLE;

// LEVER STATE
enum LeverState {
  LEVER_UNPRESSED,
  LEVER_PRESSED,
  LEVER_INVALID
};

struct Lever {
  byte noPin;
  byte ncPin;
  LeverState lastRawState;
  LeverState stableState;
  unsigned long lastRawChangeTime;
  unsigned long pressCountThisTrial;
};

Lever lever1 = { lever1NoPin, lever1NcPin, LEVER_INVALID, LEVER_INVALID, 0, 0 };
Lever lever2 = { lever2NoPin, lever2NcPin, LEVER_INVALID, LEVER_INVALID, 0, 0 };

// SOLENOID STATE
bool solenoidActive = false;
unsigned long solenoidOffMs = 0;
unsigned int consecutivePressStreak = 0;

// TRIAL VARIABLES
unsigned long nextTrialStartMs = 0;
unsigned long trialEndMs = 0;
unsigned long trialNumber = 0;
unsigned long pressCount = 0;        // total presses this trial, both levers combined
bool sawPressThisTrial = false;

// SERIAL COMMAND BUFFER
String cmdBuffer = "";

LeverState readRawLeverState(Lever &lev) {
  bool noClosed = (digitalRead(lev.noPin) == LOW);
  bool ncClosed = (digitalRead(lev.ncPin) == LOW);

  if (noClosed && !ncClosed) {
    return LEVER_PRESSED;
  } else if (!noClosed && ncClosed) {
    return LEVER_UNPRESSED;
  } else {
    return LEVER_INVALID;
  }
}

// Debounce one lever; counts a fresh press only during an active trial
void updateSingleLever(Lever &lev, unsigned long nowMs) {
  LeverState rawState = readRawLeverState(lev);

  if (rawState != lev.lastRawState) {
    lev.lastRawState = rawState;
    lev.lastRawChangeTime = nowMs;
  }

  if ((nowMs - lev.lastRawChangeTime) >= debounceMs && rawState != lev.stableState) {
    LeverState previousStableState = lev.stableState;
    lev.stableState = rawState;

    if (systemState == TRIAL_ACTIVE &&
        lev.stableState == LEVER_PRESSED &&
        previousStableState != LEVER_PRESSED) {
      lev.pressCountThisTrial++;
      pressCount++;
      sawPressThisTrial = true;
    }
  }
}

void updateLevers(unsigned long nowMs) {
  //updateSingleLever(lever1, nowMs);
  updateSingleLever(lever2, nowMs);
}

void triggerSolenoid(unsigned long nowMs) {
  digitalWrite(solenoidPin, HIGH);
  solenoidActive = true;
  solenoidOffMs = nowMs + solenoidMs;
}

void startTrial(unsigned long nowMs) {
  trialNumber++;
  pressCount = 0;
  sawPressThisTrial = false;
  lever1.pressCountThisTrial = 0;
  lever2.pressCountThisTrial = 0;
  trialEndMs = nowMs + Y_seconds * 1000UL;
  systemState = TRIAL_ACTIVE;

  tone(buzzerPin, buzzerFreq, buzzerMs);

  Serial.print("TRIAL_START,");
  Serial.println(trialNumber);
}

void endTrial(unsigned long nowMs) {
  systemState = WAIT_FOR_MATLAB;

  bool waterDelivered = false;
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
  Serial.print(waterDelivered ? 1 : 0);
  Serial.print(",");
  Serial.print(lever1.pressCountThisTrial);
  Serial.print(",");
  Serial.println(lever2.pressCountThisTrial);
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
    sawPressThisTrial = false;
    consecutivePressStreak = 0;
    nextTrialStartMs = nowMs;
    systemState = WAIT_BETWEEN_TRIALS;
    Serial.println("ACK_START");
  } else if (cmd.equalsIgnoreCase("STOP")) {
    systemState = WAIT_FOR_MATLAB;
    solenoidActive = false;
    digitalWrite(solenoidPin, LOW);
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

void setup() {
  pinMode(lever1.noPin, INPUT_PULLUP);
  pinMode(lever1.ncPin, INPUT_PULLUP);
  pinMode(lever2.noPin, INPUT_PULLUP);
  pinMode(lever2.ncPin, INPUT_PULLUP);
  pinMode(buzzerPin, OUTPUT);
  pinMode(solenoidPin, OUTPUT);
  digitalWrite(solenoidPin, LOW);

  Serial.begin(115200);
  delay(1000);

  unsigned long nowMs = millis();
  lever1.lastRawState = readRawLeverState(lever1);
  lever1.stableState = lever1.lastRawState;
  lever1.lastRawChangeTime = nowMs;

  lever2.lastRawState = readRawLeverState(lever2);
  lever2.stableState = lever2.lastRawState;
  lever2.lastRawChangeTime = nowMs;

  Serial.println("READY");
}

void loop() {
  unsigned long nowMs = millis();

  readSerialCommands(nowMs);
  updateLevers(nowMs);

  if (systemState == WAIT_BETWEEN_TRIALS && nowMs >= nextTrialStartMs) {
    startTrial(nowMs);
  }

  if (systemState == TRIAL_ACTIVE && nowMs >= trialEndMs) {
    endTrial(nowMs);
  }

  if (solenoidActive && nowMs >= solenoidOffMs) {
    digitalWrite(solenoidPin, LOW);
    solenoidActive = false;
  }
}

