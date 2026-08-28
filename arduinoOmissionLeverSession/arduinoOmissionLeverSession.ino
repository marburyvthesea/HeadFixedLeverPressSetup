/*
  arduinoOmissionLeverSession.ino

  Omission-training firmware for runOmissionLeverSessionVideoRecord.m.

  Reversed contingency:
    - The buzzer sounds at the start of each omission interval.
    - No valid lever press for the full interval earns one valve pulse.
    - The first valid press immediately cancels the interval, gives no water,
      and starts the reset delay.  After that delay the buzzer signals a new
      interval.  Therefore pressing delays water.

  Wiring and default timings match arduinoBuzzerThenCountLeverPresses.ino:
    Lever 1 NO/NC: pins 2/3       Lever 2 NO/NC: pins 9/10
    Buzzer: pin 5                 Solenoid driver: pin 7
    Buzzer: 4000 Hz, 500 ms       Valve pulse: 220 ms

  MATLAB serial protocol (115200 baud, LF terminator):
    MATLAB -> OMISSION_CONFIG,intervalMs,resetDelayMs,itiMs,debounceMs,
              buzzerFreqHz,buzzerDurationMs,valvePulseMs
    Arduino -> ACK_OMISSION_CONFIG
    MATLAB -> START_OMISSION
    Arduino -> ACK_START_OMISSION
    Arduino -> INTERVAL_START,n
    Arduino -> LEVER_PRESS,n,leverNumber
    Arduino -> INTERVAL_RESET,n
    Arduino -> INTERVAL_END,n,RESET,pressCount
      OR
    Arduino -> REWARD,n
    Arduino -> INTERVAL_END,n,REWARD,0
    MATLAB -> STOP
    Arduino -> ACK_STOP
*/

// --------------------- Hardware pins ---------------------
const byte lever1NoPin = 2;
const byte lever1NcPin = 3;
const byte lever2NoPin = 9;
const byte lever2NcPin = 10;
const byte buzzerPin = 5;
const byte solenoidPin = 7;

// Set false for a lever that is not installed.  The original sketch was
// configured to monitor only lever 2; setting this true enables lever 1 too.
const bool monitorLever1 = true;
const bool monitorLever2 = true;

// --------------------- Default settings -------------------
// MATLAB's OMISSION_CONFIG replaces these values at the beginning of a run.
unsigned long omissionIntervalMs = 20000UL;
unsigned long resetDelayMs = 1000UL;
unsigned long interTrialDelayMs = 1000UL;
unsigned long debounceMs = 10UL;
unsigned int buzzerFreqHz = 4000;
unsigned long buzzerDurationMs = 500UL;
unsigned long valvePulseMs = 220UL;

enum LeverState { LEVER_UNPRESSED, LEVER_PRESSED, LEVER_INVALID };

struct Lever {
  byte noPin;
  byte ncPin;
  LeverState lastRawState;
  LeverState stableState;
  unsigned long lastRawChangeMs;
};

Lever lever1 = {lever1NoPin, lever1NcPin, LEVER_INVALID, LEVER_INVALID, 0};
Lever lever2 = {lever2NoPin, lever2NcPin, LEVER_INVALID, LEVER_INVALID, 0};

enum SystemState { IDLE, OMIT_INTERVAL_ACTIVE, WAIT_AFTER_RESET, WAIT_AFTER_REWARD };
SystemState systemState = IDLE;

unsigned long intervalNumber = 0;
unsigned long intervalEndMs = 0;
unsigned long nextIntervalStartMs = 0;
unsigned long pressCountThisInterval = 0;

bool solenoidActive = false;
unsigned long solenoidOffMs = 0;
String commandBuffer = "";

bool timeReached(unsigned long nowMs, unsigned long deadlineMs) {
  // Works correctly when millis() wraps around.
  return (long)(nowMs - deadlineMs) >= 0;
}

LeverState readRawLeverState(const Lever &lever) {
  const bool noClosed = (digitalRead(lever.noPin) == LOW);
  const bool ncClosed = (digitalRead(lever.ncPin) == LOW);
  if (noClosed && !ncClosed) return LEVER_PRESSED;
  if (!noClosed && ncClosed) return LEVER_UNPRESSED;
  return LEVER_INVALID;  // Both open or both closed: wiring/fault state.
}

void sendIntervalStart() {
  intervalNumber++;
  pressCountThisInterval = 0;
  intervalEndMs = millis() + omissionIntervalMs;
  systemState = OMIT_INTERVAL_ACTIVE;

  tone(buzzerPin, buzzerFreqHz, buzzerDurationMs);
  Serial.print("INTERVAL_START,");
  Serial.println(intervalNumber);
}

void triggerSolenoid(unsigned long nowMs) {
  digitalWrite(solenoidPin, HIGH);
  solenoidActive = true;
  solenoidOffMs = nowMs + valvePulseMs;
}

void cancelIntervalForPress(byte leverNumber, unsigned long nowMs) {
  // The first press ends the active omission interval immediately.
  pressCountThisInterval++;
  Serial.print("LEVER_PRESS,");
  Serial.print(intervalNumber);
  Serial.print(",");
  Serial.println(leverNumber);

  Serial.print("INTERVAL_RESET,");
  Serial.println(intervalNumber);
  Serial.print("INTERVAL_END,");
  Serial.print(intervalNumber);
  Serial.print(",RESET,");
  Serial.println(pressCountThisInterval);

  nextIntervalStartMs = nowMs + resetDelayMs;
  systemState = WAIT_AFTER_RESET;
}

void rewardCompletedInterval(unsigned long nowMs) {
  triggerSolenoid(nowMs);
  Serial.print("REWARD,");
  Serial.println(intervalNumber);
  Serial.print("INTERVAL_END,");
  Serial.print(intervalNumber);
  Serial.println(",REWARD,0");

  nextIntervalStartMs = nowMs + interTrialDelayMs;
  systemState = WAIT_AFTER_REWARD;
}

// Returns true only for a new, debounced press transition.
bool updateLever(Lever &lever, unsigned long nowMs) {
  LeverState rawState = readRawLeverState(lever);
  if (rawState != lever.lastRawState) {
    lever.lastRawState = rawState;
    lever.lastRawChangeMs = nowMs;
  }

  if ((nowMs - lever.lastRawChangeMs) >= debounceMs && rawState != lever.stableState) {
    LeverState previousStableState = lever.stableState;
    lever.stableState = rawState;
    return lever.stableState == LEVER_PRESSED && previousStableState != LEVER_PRESSED;
  }
  return false;
}

void updateLevers(unsigned long nowMs) {
  // Always update state so a held-down lever cannot be counted repeatedly.
  bool lever1NewPress = monitorLever1 && updateLever(lever1, nowMs);
  bool lever2NewPress = monitorLever2 && updateLever(lever2, nowMs);

  if (systemState != OMIT_INTERVAL_ACTIVE) return;

  // One physical press is enough to cancel the interval.  If both transition
  // in the same loop, report lever 1 first for deterministic logging.
  if (lever1NewPress) {
    cancelIntervalForPress(1, nowMs);
  } else if (lever2NewPress) {
    cancelIntervalForPress(2, nowMs);
  }
}

String fieldAt(const String &line, int fieldIndex) {
  int startIndex = 0;
  for (int index = 0; index < fieldIndex; index++) {
    startIndex = line.indexOf(',', startIndex);
    if (startIndex < 0) return "";
    startIndex++;
  }
  int endIndex = line.indexOf(',', startIndex);
  if (endIndex < 0) endIndex = line.length();
  return line.substring(startIndex, endIndex);
}

bool configureOmission(const String &command) {
  // Field 0 is OMISSION_CONFIG, fields 1..7 are positive integer settings.
  unsigned long values[7];
  for (int i = 0; i < 7; i++) {
    String field = fieldAt(command, i + 1);
    if (field.length() == 0) return false;
    long parsed = field.toInt();
    if (parsed <= 0) return false;
    values[i] = (unsigned long) parsed;
  }
  if (values[4] > 65535UL) return false;  // tone() frequency is unsigned int.

  omissionIntervalMs = values[0];
  resetDelayMs = values[1];
  interTrialDelayMs = values[2];
  debounceMs = values[3];
  buzzerFreqHz = (unsigned int) values[4];
  buzzerDurationMs = values[5];
  valvePulseMs = values[6];
  return true;
}

void handleCommand(String command, unsigned long nowMs) {
  command.trim();
  if (command.startsWith("OMISSION_CONFIG,")) {
    if (configureOmission(command)) {
      Serial.println("ACK_OMISSION_CONFIG");
    } else {
      Serial.println("ERROR_BAD_OMISSION_CONFIG");
    }
  } else if (command.equalsIgnoreCase("START_OMISSION")) {
    intervalNumber = 0;
    pressCountThisInterval = 0;
    noTone(buzzerPin);
    digitalWrite(solenoidPin, LOW);
    solenoidActive = false;
    systemState = WAIT_AFTER_REWARD;
    nextIntervalStartMs = nowMs;
    Serial.println("ACK_START_OMISSION");
  } else if (command.equalsIgnoreCase("STOP")) {
    systemState = IDLE;
    noTone(buzzerPin);
    digitalWrite(solenoidPin, LOW);
    solenoidActive = false;
    Serial.println("ACK_STOP");
  } else if (command.length() > 0) {
    Serial.print("ERROR_UNKNOWN_COMMAND,");
    Serial.println(command);
  }
}

void readSerialCommands(unsigned long nowMs) {
  while (Serial.available() > 0) {
    char character = (char) Serial.read();
    if (character == '\n') {
      handleCommand(commandBuffer, nowMs);
      commandBuffer = "";
    } else if (character != '\r') {
      commandBuffer += character;
    }
  }
}

void initializeLever(Lever &lever, unsigned long nowMs) {
  lever.lastRawState = readRawLeverState(lever);
  lever.stableState = lever.lastRawState;
  lever.lastRawChangeMs = nowMs;
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
  initializeLever(lever1, nowMs);
  initializeLever(lever2, nowMs);
  Serial.println("READY");
}

void loop() {
  unsigned long nowMs = millis();
  readSerialCommands(nowMs);
  updateLevers(nowMs);

  if ((systemState == WAIT_AFTER_RESET || systemState == WAIT_AFTER_REWARD) &&
      timeReached(nowMs, nextIntervalStartMs)) {
    sendIntervalStart();
  }

  if (systemState == OMIT_INTERVAL_ACTIVE && timeReached(nowMs, intervalEndMs)) {
    rewardCompletedInterval(nowMs);
  }

  if (solenoidActive && timeReached(nowMs, solenoidOffMs)) {
    digitalWrite(solenoidPin, LOW);
    solenoidActive = false;
  }
}
