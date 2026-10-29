/*
  leverTaskControllerWindow - UNO R4 WiFi, Serial 115200, newline commands.
  Lever 1: NO=2 NC=3; lever 2: NO=9 NC=10. Both COM -> GND.
  Buzzer=5. Pin 7 -> active-HIGH external solenoid driver.
  NEVER connect the 24 V valve directly to an Arduino pin.

  Configure while stopped:
    SET_TASK,IMMEDIATE / TRIAL / WINDOW -> ACK_TASK,<task>
    SET_ACTIVE,BOTH / LEVER1 / LEVER2 -> ACK_ACTIVE,<selection>
    SET_TRIAL,<windowMs>,<pauseMs>,<buzzerMs>,<frequencyHz>
      -> ACK_TRIAL,<same four values>
    SET_WINDOW,<startMinMs>,<startMaxMs>,<lengthMinMs>,<lengthMaxMs>,<seed>
      -> ACK_WINDOW,<same five values>. Send AFTER SET_TRIAL.
    Bounds: startMax+lengthMax <= trial duration, lengthMin >= 1 ms.
    Seed: integer 1..2147483646. WINDOW settings required before START.
  START -> ACK_START. STOP -> ACK_STOP after current pulse finishes.
  CONTINUE -> ACK_CONTINUE,<last trial number> (TRIAL mode only).

  IMMEDIATE: PRESS,<number>,<leverId>,<waterCommanded>,<pressDetectionMs>
  TRIAL: TRIAL_START,<number>,<startMs>
    TRIAL_END,<number>,<total>,<sawActive>,<water>,<lever1>,<lever2>,
              <activeCount>,<startMs>,<endMs>
    TRIAL_ABORT: same fields, but water=0 and an incomplete window.
  WINDOW: WINDOW_START,<number>,<startMs>,<windowStartOffsetMs>,<windowEndOffsetMs>
    WINDOW_END / WINDOW_ABORT: the same 10 TRIAL fields, followed by:
      <windowStartOffsetMs>,<windowEndOffsetMs>,
      <earlyL1>,<earlyL2>,<windowL1>,<windowL2>,<lateL1>,<lateL2>
    activeCount/sawActive in WINDOW mean SELECTED-lever presses INSIDE
    [windowStart,windowEnd). Both levers are counted across the full trial.
    Start and length are drawn independently, uniformly in integer ms.
    There is only the trial-onset buzzer; no extra cue at window onset.
  Water=1 means valve commanded on, NOT measured flow.

  A completed trial earns ONE reward if activeCount > 0.
  The pause starts after reward completion, or after an unrewarded trial.
  MATLAB sends CONTINUE to enable the next trial.
  STOP aborts incomplete trials without reward. Pending immediate presses
  are logged with water=0 on STOP. Held levers do not repeat.
*/
#include <Arduino.h>
#include <string.h>
#include <stdint.h>

const byte buzzerPin = 5;
const byte solenoidPin = 7;
const unsigned long debounceMs = 10;
const unsigned long solenoidMs = 220;
const unsigned long valveClosedGapMs = 50;

enum TaskMode { IMMEDIATE, TRIAL, WINDOW };
enum TrialState { NO_TRIAL, TRIAL_RUNNING, WAIT_REWARD, TRIAL_GAP };
enum LeverState { RELEASED, PRESSED, INVALID };

struct Lever {
  byte noPin, ncPin, id;
  LeverState raw;
  unsigned long changedAt;
  bool armed;
};
struct PressEvent {
  unsigned long number, detectedAt;
  byte leverId;
  bool eligible;
};

Lever lever1 = {2, 3, 1, INVALID, 0, false};
Lever lever2 = {9, 10, 2, INVALID, 0, false};
TaskMode taskMode = IMMEDIATE;
TrialState trialState = NO_TRIAL;
byte activeLever = 0; // 0=BOTH, 1=LEVER1, 2=LEVER2
bool sessionActive = false, stopPending = false;
bool valveActive = false, valveGapActive = false;
unsigned long valveStartedAt = 0, valveClosedAt = 0;
unsigned long trialWindowMs = 15000, trialPauseMs = 1000;
unsigned long buzzerMs = 500, buzzerHz = 4000;
unsigned long trialNumber = 0, trialStartedAt = 0, trialGapStartedAt = 0;
unsigned long trialLever1 = 0, trialLever2 = 0, trialActivePresses = 0;
bool continueRequested = false;
unsigned long windowStartMinMs = 1000, windowStartMaxMs = 8000;
unsigned long windowLengthMinMs = 2000, windowLengthMaxMs = 5000;
unsigned long windowSeed = 1;
uint32_t windowRngState = 1;
unsigned long windowStartOffsetMs = 0, windowEndOffsetMs = 0;
unsigned long earlyL1 = 0, earlyL2 = 0, windowL1 = 0, windowL2 = 0;
unsigned long lateL1 = 0, lateL2 = 0;
bool windowConfigured = false;

// xorshift32 has a nonzero state and generates 1..UINT32_MAX.
// Rejection sampling avoids modulo bias for integer-millisecond ranges.
unsigned long drawWindowValue(unsigned long low, unsigned long high) {
  uint32_t span = (uint32_t)(high - low + 1UL);
  uint32_t limit = UINT32_MAX - (UINT32_MAX % span);
  uint32_t value;
  do {
    windowRngState ^= windowRngState << 13;
    windowRngState ^= windowRngState >> 17;
    windowRngState ^= windowRngState << 5;
    value = windowRngState - 1U;
  } while (value >= limit);
  return low + value % span;
}

const byte queueCapacity = 32;
PressEvent pressQueue[queueCapacity];
byte queueHead = 0, queueTail = 0, queueCount = 0;
unsigned long pressNumber = 0;
char commandBuffer[80];
byte commandLength = 0;
bool commandOverflow = false;

bool isActiveLever(byte id) {
  return activeLever == 0 || activeLever == id;
}

void startWater() {
  digitalWrite(solenoidPin, HIGH);
  valveStartedAt = millis();
  valveActive = true;
  valveGapActive = false;
}

void serviceWater(unsigned long nowMs) {
  if (valveActive && nowMs - valveStartedAt >= solenoidMs) {
    digitalWrite(solenoidPin, LOW);
    valveActive = false;
    valveClosedAt = nowMs;
    valveGapActive = true;
  }
}

void printTrialResult(bool completed, bool water, unsigned long nowMs) {
  if (taskMode == WINDOW)
    Serial.print(completed ? F("WINDOW_END,") : F("WINDOW_ABORT,"));
  else
    Serial.print(completed ? F("TRIAL_END,") : F("TRIAL_ABORT,"));
  Serial.print(trialNumber);
  Serial.print(','); Serial.print(trialLever1 + trialLever2);
  Serial.print(','); Serial.print(trialActivePresses > 0 ? 1 : 0);
  Serial.print(','); Serial.print(water ? 1 : 0);
  Serial.print(','); Serial.print(trialLever1);
  Serial.print(','); Serial.print(trialLever2);
  Serial.print(','); Serial.print(trialActivePresses);
  Serial.print(','); Serial.print(trialStartedAt);
  Serial.print(','); Serial.print(nowMs);
  if (taskMode == WINDOW) {
    Serial.print(','); Serial.print(windowStartOffsetMs);
    Serial.print(','); Serial.print(windowEndOffsetMs);
    Serial.print(','); Serial.print(earlyL1);
    Serial.print(','); Serial.print(earlyL2);
    Serial.print(','); Serial.print(windowL1);
    Serial.print(','); Serial.print(windowL2);
    Serial.print(','); Serial.print(lateL1);
    Serial.print(','); Serial.print(lateL2);
  }
  Serial.println();
}

void finishStopIfReady() {
  if (stopPending && !valveActive && queueCount == 0) {
    stopPending = false;
    Serial.println(F("ACK_STOP"));
  }
}

void requestStop() {
  sessionActive = false;
  noTone(buzzerPin);
  if (trialState == TRIAL_RUNNING) {
    printTrialResult(false, false, millis());
  }
  trialState = NO_TRIAL;
  continueRequested = false;
  stopPending = true;
  finishStopIfReady();
}

void beginTrial(unsigned long nowMs) {
  trialNumber++;
  trialLever1 = trialLever2 = trialActivePresses = 0;
  earlyL1 = earlyL2 = windowL1 = windowL2 = lateL1 = lateL2 = 0;
  if (taskMode == WINDOW) {
    windowStartOffsetMs = drawWindowValue(windowStartMinMs, windowStartMaxMs);
    windowEndOffsetMs = windowStartOffsetMs +
        drawWindowValue(windowLengthMinMs, windowLengthMaxMs);
  }
  trialStartedAt = nowMs;
  trialState = TRIAL_RUNNING;
  continueRequested = false;
  if (buzzerMs > 0) tone(buzzerPin, (unsigned int)buzzerHz, buzzerMs);
  Serial.print(taskMode == WINDOW ? F("WINDOW_START,") : F("TRIAL_START,"));
  Serial.print(trialNumber);
  Serial.print(','); Serial.print(trialStartedAt);
  if (taskMode == WINDOW) {
    Serial.print(','); Serial.print(windowStartOffsetMs);
    Serial.print(','); Serial.print(windowEndOffsetMs);
  }
  Serial.println();
}

void serviceTrial(unsigned long nowMs) {
  if (!sessionActive || taskMode == IMMEDIATE) return;
  if (trialState == TRIAL_RUNNING && nowMs - trialStartedAt >= trialWindowMs) {
    noTone(buzzerPin);
    bool water = trialActivePresses > 0;
    if (water) startWater();
    trialState = WAIT_REWARD;
    printTrialResult(true, water, nowMs);
  }
  if (trialState == WAIT_REWARD && !valveActive) {
    trialGapStartedAt = nowMs;
    trialState = TRIAL_GAP;
  }
  if (trialState == TRIAL_GAP && continueRequested &&
      nowMs - trialGapStartedAt >= trialPauseMs) {
    beginTrial(nowMs);
  }
}

void handlePress(byte id, unsigned long nowMs) {
  if (!sessionActive) return;
  if (taskMode == TRIAL || taskMode == WINDOW) {
    if (trialState != TRIAL_RUNNING) return;
    if (id == 1) trialLever1++; else trialLever2++;
    bool inWindow = true;
    if (taskMode == WINDOW) {
      unsigned long elapsed = nowMs - trialStartedAt;
      inWindow = elapsed >= windowStartOffsetMs && elapsed < windowEndOffsetMs;
      if (elapsed < windowStartOffsetMs) {
        if (id == 1) earlyL1++; else earlyL2++;
      } else if (inWindow) {
        if (id == 1) windowL1++; else windowL2++;
      } else {
        if (id == 1) lateL1++; else lateL2++;
      }
    }
    if (isActiveLever(id) && inWindow) trialActivePresses++;
    return;
  }
  if (queueCount >= queueCapacity) {
    Serial.println(F("ERROR,PRESS_QUEUE_FULL,SESSION_STOPPED"));
    requestStop();
    return;
  }
  PressEvent &event = pressQueue[queueTail];
  event.number = ++pressNumber;
  event.detectedAt = nowMs;
  event.leverId = id;
  event.eligible = isActiveLever(id);
  queueTail = (queueTail + 1) % queueCapacity;
  queueCount++;
}

void updateLever(Lever &lever, unsigned long nowMs) {
  bool noClosed = digitalRead(lever.noPin) == LOW;
  bool ncClosed = digitalRead(lever.ncPin) == LOW;
  LeverState raw = noClosed && !ncClosed ? PRESSED :
                   (!noClosed && ncClosed ? RELEASED : INVALID);
  if (raw != lever.raw) {
    lever.raw = raw;
    lever.changedAt = nowMs;
  }
  if (nowMs - lever.changedAt < debounceMs) return;
  if (raw == RELEASED) {
    lever.armed = true;
  } else if (raw == PRESSED && lever.armed) {
    lever.armed = false;
    handlePress(lever.id, nowMs);
  }
}

void servicePressQueue(unsigned long nowMs) {
  if (queueCount == 0) return;
  PressEvent event = pressQueue[queueHead];
  bool water = sessionActive && taskMode == IMMEDIATE && event.eligible;
  if (stopPending && valveActive) return;
  if (water && (valveActive ||
      (valveGapActive && nowMs - valveClosedAt < valveClosedGapMs))) return;
  queueHead = (queueHead + 1) % queueCapacity;
  queueCount--;
  if (water) startWater();
  Serial.print(F("PRESS,")); Serial.print(event.number);
  Serial.print(','); Serial.print(event.leverId);
  Serial.print(','); Serial.print(water ? 1 : 0);
  Serial.print(','); Serial.println(event.detectedAt);
}

bool parseTrialSettings(const char *text, unsigned long values[4]) {
  for (byte i = 0; i < 4; i++) {
    if (*text < '0' || *text > '9') return false;
    unsigned long n = 0;
    while (*text >= '0' && *text <= '9') {
      n = n * 10UL + (*text++ - '0');
      if (n > 3600000UL) return false;
    }
    values[i] = n;
    if (i < 3) { if (*text++ != ',') return false; }
    else if (*text != '\0') return false;
  }
  return values[0] >= 1 && values[2] <= values[0] &&
         values[3] >= 31 && values[3] <= 20000;
}

bool parseWindowSettings(const char *text, unsigned long values[5]) {
  const unsigned long maxValue = 2147483646UL;
  for (byte i = 0; i < 5; i++) {
    if (*text < '0' || *text > '9') return false;
    unsigned long n = 0;
    while (*text >= '0' && *text <= '9') {
      unsigned long digit = *text++ - '0';
      if (n > (maxValue - digit) / 10UL) return false;
      n = n * 10UL + digit;
    }
    values[i] = n;
    if (i < 4) { if (*text++ != ',') return false; }
    else if (*text != '\0') return false;
  }
  return values[0] <= values[1] && values[2] >= 1 &&
         values[2] <= values[3] && values[1] <= trialWindowMs &&
         values[3] <= trialWindowMs - values[1] && values[4] >= 1;
}

void handleCommand() {
  commandBuffer[commandLength] = '\0';
  if (strncmp(commandBuffer, "SET_", 4) == 0 && (sessionActive || stopPending)) {
    Serial.println(F("ERROR,STOP_BEFORE_CONFIGURING"));
    return;
  }
  if (strncmp(commandBuffer, "SET_TASK,", 9) == 0) {
    const char *value = commandBuffer + 9;
    if (strcmp(value, "IMMEDIATE") == 0) taskMode = IMMEDIATE;
    else if (strcmp(value, "TRIAL") == 0) taskMode = TRIAL;
    else if (strcmp(value, "WINDOW") == 0) taskMode = WINDOW;
    else { Serial.println(F("ERROR,INVALID_TASK")); return; }
    Serial.print(F("ACK_TASK,")); Serial.println(value);
  } else if (strncmp(commandBuffer, "SET_ACTIVE,", 11) == 0) {
    const char *value = commandBuffer + 11;
    if (strcmp(value, "BOTH") == 0) activeLever = 0;
    else if (strcmp(value, "LEVER1") == 0) activeLever = 1;
    else if (strcmp(value, "LEVER2") == 0) activeLever = 2;
    else { Serial.println(F("ERROR,INVALID_ACTIVE_MODE")); return; }
    Serial.print(F("ACK_ACTIVE,")); Serial.println(value);
  } else if (strncmp(commandBuffer, "SET_TRIAL,", 10) == 0) {
    unsigned long values[4];
    if (!parseTrialSettings(commandBuffer + 10, values)) {
      Serial.println(F("ERROR,INVALID_TRIAL_SETTINGS")); return;
    }
    trialWindowMs = values[0]; trialPauseMs = values[1];
    windowConfigured = false; // Revalidate window bounds after any duration change.
    buzzerMs = values[2]; buzzerHz = values[3];
    Serial.print(F("ACK_TRIAL,"));
    for (byte i = 0; i < 4; i++) {
      Serial.print(values[i]);
      if (i < 3) Serial.print(','); else Serial.println();
    }
  } else if (strncmp(commandBuffer, "SET_WINDOW,", 11) == 0) {
    unsigned long values[5];
    if (!parseWindowSettings(commandBuffer + 11, values)) {
      Serial.println(F("ERROR,INVALID_WINDOW_SETTINGS")); return;
    }
    windowStartMinMs = values[0]; windowStartMaxMs = values[1];
    windowLengthMinMs = values[2]; windowLengthMaxMs = values[3];
    windowSeed = values[4];
    windowConfigured = true;
    Serial.print(F("ACK_WINDOW,"));
    for (byte i = 0; i < 5; i++) {
      Serial.print(values[i]);
      if (i < 4) Serial.print(','); else Serial.println();
    }
  } else if (strcmp(commandBuffer, "START") == 0) {
    if (stopPending) { Serial.println(F("ERROR,STOP_IN_PROGRESS")); return; }
    if (taskMode == WINDOW && !windowConfigured) {
      Serial.println(F("ERROR,CONFIGURE_WINDOW_BEFORE_START")); return;
    }
    bool newSession = !sessionActive;
    if (newSession) {
      pressNumber = trialNumber = 0;
      windowRngState = (uint32_t)windowSeed;
      queueHead = queueTail = queueCount = 0;
      lever1.armed = lever2.armed = false;
      trialState = NO_TRIAL;
      continueRequested = false;
      sessionActive = true;
    }
    Serial.println(F("ACK_START"));
    if (newSession && taskMode != IMMEDIATE) beginTrial(millis());
  } else if (strcmp(commandBuffer, "CONTINUE") == 0) {
    if (sessionActive && taskMode != IMMEDIATE &&
        (trialState == WAIT_REWARD || trialState == TRIAL_GAP)) {
      continueRequested = true;
      Serial.print(F("ACK_CONTINUE,")); Serial.println(trialNumber);
    } else Serial.println(F("ERROR,CONTINUE_NOT_EXPECTED"));
  } else if (strcmp(commandBuffer, "STOP") == 0) {
    requestStop();
  } else if (commandLength > 0) Serial.println(F("ERROR,UNKNOWN_COMMAND"));
}

void readCommands() {
  byte bytesRead = 0;
  while (Serial.available() > 0 && bytesRead++ < 32) {
    char c = Serial.read();
    if (c == '\n') {
      if (commandOverflow) Serial.println(F("ERROR,COMMAND_TOO_LONG"));
      else handleCommand();
      commandLength = 0; commandOverflow = false;
    } else if (c != '\r') {
      if (commandLength < sizeof(commandBuffer) - 1) commandBuffer[commandLength++] = c;
      else commandOverflow = true;
    }
  }
}

void setup() {
  digitalWrite(solenoidPin, LOW);
  pinMode(solenoidPin, OUTPUT);
  pinMode(buzzerPin, OUTPUT);
  pinMode(lever1.noPin, INPUT_PULLUP); pinMode(lever1.ncPin, INPUT_PULLUP);
  pinMode(lever2.noPin, INPUT_PULLUP); pinMode(lever2.ncPin, INPUT_PULLUP);
  Serial.begin(115200);
  Serial.println(F("READY"));
}

void loop() {
  serviceWater(millis());
  readCommands();
  unsigned long nowMs = millis();
  // Close a window BEFORE processing presses at or past its deadline.
  serviceTrial(nowMs);
  updateLever(lever1, nowMs); updateLever(lever2, nowMs);
  serviceWater(millis());
  servicePressQueue(millis());
  finishStopIfReady();
}
