/*
  leverTaskControllerWindow - UNO R4 WiFi, Serial 115200, newline commands.
  Lever 1: NO=2 NC=3; lever 2: NO=9 NC=10. Both COM -> GND.
  Buzzer=5. Pin 7 -> active-HIGH external solenoid driver.
  NEVER connect the 24 V valve directly to an Arduino pin.

  Configure while stopped:
    SET_TASK,IMMEDIATE / TRIAL / WINDOW / OMISSION -> ACK_TASK,<task>
    SET_ACTIVE,BOTH / LEVER1 / LEVER2 -> ACK_ACTIVE,<selection>
    SET_PRESS_LOG,ON / OFF -> ACK_PRESS_LOG,<ON/OFF> (stopped only)
      Opt-in TRIAL/WINDOW press timing; default OFF and reset OFF by STOP.
    SET_TRIAL,<windowMs>,<pauseMs>,<buzzerMs>,<frequencyHz>
      -> ACK_TRIAL,<same four values>
    SET_WINDOW,<startMinMs>,<startMaxMs>,<lengthMinMs>,<lengthMaxMs>,<seed>
      -> ACK_WINDOW,<same five values>. Send AFTER SET_TRIAL.
    Bounds: startMax+lengthMax <= trial duration, lengthMin >= 1 ms.
    Seed: integer 1..2147483646. WINDOW settings required before START.
    SET_OMISSION,<intervalMs>,<resetDelayMs>,<rewardPauseMs>,<buzzerMs>,<frequencyHz>
      -> ACK_OMISSION,<same five values>. OMISSION settings required before START.
  START -> ACK_START. STOP -> ACK_STOP after current pulse finishes.
  CONTINUE -> ACK_CONTINUE,<last trial number> (TRIAL/WINDOW modes only).

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
  With SET_PRESS_LOG,ON, TRIAL/WINDOW additionally send on each fresh
  debounced session press (both levers, including intertrial gaps):
    PRESS_EVENT,<trial>,<leverId>,<ArduinoMs>,<elapsedMs>,<inTrial>,
                <selected>,<qualifying>,<phase>
    In-trial elapsedMs is since trial/buzzer onset; phases are TRIAL or
    EARLY/WINDOW/LATE. Window start is inclusive and end is exclusive.
    Intertrial presses use trial=0, elapsedMs=-1, inTrial=0, qualifying=0,
    phase=INTERTRIAL. selected still reports SET_ACTIVE membership.
    Timestamps are millis() at debounced detection, not MATLAB receipt time.
    Event logging does not change counts, reward eligibility, or old records.
    IMMEDIATE/OMISSION never send PRESS_EVENT; older scripts leave it OFF.
  OMISSION: INTERVAL_START,<number>,<startMs>
    Selected-lever press before the deadline:
      LEVER_PRESS,<number>,<leverId>,<pressDetectionMs>
      INTERVAL_RESET,<number>
      INTERVAL_END,<number>,RESET,<totalPressCount>
    No selected-lever press for the full interval:
      REWARD,<number>
      INTERVAL_END,<number>,REWARD,<totalPressCount>
    STOP during an interval:
      INTERVAL_END,<number>,ABORT,<totalPressCount>
    OMISSION intervals restart automatically after the configured reset or
    reward pause; MATLAB does not send CONTINUE in this mode. Both levers are
    counted, but only SET_ACTIVE-selected lever presses reset the interval.
  Transport: complete lines enter a 4096-byte FIFO (512 bytes reserved for
  stop/error records), then drain in bounded chunks only when USB TX has room.
  This keeps control, pulse closure and STOP handling running if MATLAB stalls.
  Finite overflow stops the session and eventually sends:
    ERROR,TX_OVERFLOW,<droppedProtocolLines>,SESSION_STOPPED
  Previously queued lines stay ordered; no partial record is silently dropped.
  Zero lost lines can indicate stopped immediate-mode backpressure before loss.
  New commands are ignored during fault recovery until ERROR/ACK_STOP drain.
  Water=1 means valve commanded on, NOT measured flow.

  In TRIAL/WINDOW modes, a completed trial earns ONE reward if activeCount > 0.
  Their pause starts after reward completion, or after an unrewarded trial;
  MATLAB sends CONTINUE to enable the next trial.
  STOP aborts incomplete trials without reward. Pending immediate presses
  are logged with water=0 on STOP. Held levers do not repeat.
*/
#include <Arduino.h>
#include <string.h>
#include <stdint.h>
#include <limits.h>

const byte buzzerPin = 5;
const byte solenoidPin = 7;
const unsigned long debounceMs = 10;
const unsigned long solenoidMs = 350;
const unsigned long valveClosedGapMs = 50;

enum TaskMode { IMMEDIATE, TRIAL, WINDOW, OMISSION };
enum TrialState {
  NO_TRIAL,
  TRIAL_RUNNING,
  WAIT_REWARD,
  TRIAL_GAP,
  OMISSION_RESET_GAP,
  OMISSION_REWARD_GAP
};
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
bool pressLogEnabled = false;
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
unsigned long omissionIntervalMs = 20000;
unsigned long omissionResetDelayMs = 1000;
unsigned long omissionRewardPauseMs = 1000;
unsigned long omissionBuzzerMs = 500;
unsigned long omissionBuzzerHz = 4000;
bool omissionConfigured = false;

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

// Each protocol line is staged before entering the FIFO, so overflow can never
// leave half a record interleaved with the next one. SerialUSB::write on the
// UNO R4 can wait indefinitely when its TX FIFO is full; only serviceTx below
// may call it, and only for the capacity reported by availableForWrite().
class ProtocolOutput : public Print {
 public:
  static const uint16_t capacity = 4096;
  static const uint16_t stopReserve = 512;
  static const uint16_t maxLine = 256;
  bool emergency = false;
  bool faultPending = false;
  unsigned long droppedLines = 0;

  size_t write(uint8_t value) override {
    if (lineLength < maxLine) line[lineLength++] = value;
    else lineTooLong = true;
    if (value == '\n') {
      uint16_t limit = emergency ? capacity : capacity - stopReserve;
      if (lineTooLong || lineLength > limit || used > limit - lineLength) {
        noteOverflow(true);
      } else {
        for (uint16_t i = 0; i < lineLength; i++) {
          bytes[tail] = line[i];
          tail = (tail + 1) % capacity;
        }
        used += lineLength;
      }
      lineLength = 0;
      lineTooLong = false;
    }
    return 1; // Formatting never waits for the transport or retries a dropped line.
  }
  using Print::write;

  bool roomFor(uint16_t length, bool reserved = false) const {
    uint16_t limit = reserved ? capacity : capacity - stopReserve;
    return length <= limit && used <= limit - length;
  }
  void noteOverflow(bool lostLine) {
    faultPending = true;
    if (lostLine && droppedLines != ULONG_MAX) droppedLines++;
  }
  uint16_t queuedBytes() const { return used; }
  void service() {
    if (used == 0) return;
    int available = Serial.availableForWrite();
    if (available <= 0) return;
    uint16_t count = used;
    if (count > 64) count = 64; // Bound transport work per control-loop iteration.
    if (count > capacity - head) count = capacity - head;
    if (count > (unsigned int)available) count = (uint16_t)available;
    size_t written = Serial.write(bytes + head, count);
    if (written > count) written = count;
    head = (head + written) % capacity;
    used -= written; // A disconnected/short writer keeps every unsent byte queued.
  }
  void reset() {
    head = tail = used = lineLength = 0;
    lineTooLong = emergency = faultPending = false;
    droppedLines = 0;
  }
 private:
  uint8_t bytes[capacity];
  uint8_t line[maxLine];
  uint16_t head = 0, tail = 0, used = 0, lineLength = 0;
  bool lineTooLong = false;
};
ProtocolOutput Protocol;
bool txFaultLatched = false, txOverflowReportPending = false;

void serviceTx() { Protocol.service(); }

bool isActiveLever(byte id) {
  return activeLever == 0 || activeLever == id;
}

void printPressEvent(byte id, unsigned long nowMs) {
  if (!pressLogEnabled || (taskMode != TRIAL && taskMode != WINDOW)) return;
  bool inTrial = trialState == TRIAL_RUNNING;
  unsigned long elapsedMs = inTrial ? nowMs - trialStartedAt : 0;
  bool selected = isActiveLever(id);
  bool qualifying = inTrial && selected;
  const char *phase = inTrial ? "TRIAL" : "INTERTRIAL";
  if (inTrial && taskMode == WINDOW) {
    if (elapsedMs < windowStartOffsetMs) phase = "EARLY";
    else if (elapsedMs < windowEndOffsetMs) phase = "WINDOW";
    else phase = "LATE";
    qualifying = selected && elapsedMs >= windowStartOffsetMs &&
                 elapsedMs < windowEndOffsetMs;
  }
  Protocol.print(F("PRESS_EVENT,")); Protocol.print(inTrial ? trialNumber : 0);
  Protocol.print(','); Protocol.print(id);
  Protocol.print(','); Protocol.print(nowMs);
  Protocol.print(',');
  if (inTrial) Protocol.print(elapsedMs); else Protocol.print(-1);
  Protocol.print(','); Protocol.print(inTrial ? 1 : 0);
  Protocol.print(','); Protocol.print(selected ? 1 : 0);
  Protocol.print(','); Protocol.print(qualifying ? 1 : 0);
  Protocol.print(','); Protocol.println(phase);
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
    Protocol.print(completed ? F("WINDOW_END,") : F("WINDOW_ABORT,"));
  else
    Protocol.print(completed ? F("TRIAL_END,") : F("TRIAL_ABORT,"));
  Protocol.print(trialNumber);
  Protocol.print(','); Protocol.print(trialLever1 + trialLever2);
  Protocol.print(','); Protocol.print(trialActivePresses > 0 ? 1 : 0);
  Protocol.print(','); Protocol.print(water ? 1 : 0);
  Protocol.print(','); Protocol.print(trialLever1);
  Protocol.print(','); Protocol.print(trialLever2);
  Protocol.print(','); Protocol.print(trialActivePresses);
  Protocol.print(','); Protocol.print(trialStartedAt);
  Protocol.print(','); Protocol.print(nowMs);
  if (taskMode == WINDOW) {
    Protocol.print(','); Protocol.print(windowStartOffsetMs);
    Protocol.print(','); Protocol.print(windowEndOffsetMs);
    Protocol.print(','); Protocol.print(earlyL1);
    Protocol.print(','); Protocol.print(earlyL2);
    Protocol.print(','); Protocol.print(windowL1);
    Protocol.print(','); Protocol.print(windowL2);
    Protocol.print(','); Protocol.print(lateL1);
    Protocol.print(','); Protocol.print(lateL2);
  }
  Protocol.println();
}

void finishStopIfReady() {
  if (stopPending && !valveActive && queueCount == 0 &&
      !Protocol.faultPending && !txOverflowReportPending && Protocol.roomFor(16, true)) {
    bool previous = Protocol.emergency;
    Protocol.emergency = true;
    Protocol.println(F("ACK_STOP"));
    Protocol.emergency = previous;
    stopPending = false;
  }
}

void requestStop() {
  sessionActive = false;
  pressLogEnabled = false;
  noTone(buzzerPin);
  bool previous = Protocol.emergency;
  Protocol.emergency = true;
  if (trialState == TRIAL_RUNNING) {
    if (taskMode == OMISSION) {
      Protocol.print(F("INTERVAL_END,"));
      Protocol.print(trialNumber);
      Protocol.print(F(",ABORT,"));
      Protocol.println(trialLever1 + trialLever2);
    } else {
      printTrialResult(false, false, millis());
    }
  }
  Protocol.emergency = previous;
  trialState = NO_TRIAL;
  continueRequested = false;
  stopPending = true;
  finishStopIfReady();
}

void serviceTxFault() {
  if (Protocol.faultPending) {
    Protocol.faultPending = false;
    if (!txFaultLatched) {
      txFaultLatched = true;
      txOverflowReportPending = true;
      // Stop control state immediately; do not shorten an already-started pulse.
      requestStop();
    }
    // requestStop's abort record can itself fail if repeated commands consumed
    // the reserve; keep the loss count and report it once transport recovers.
    Protocol.faultPending = false;
  }
  if (txOverflowReportPending && Protocol.roomFor(96, true)) {
    bool previous = Protocol.emergency;
    Protocol.emergency = true;
    Protocol.print(F("ERROR,TX_OVERFLOW,"));
    Protocol.print(Protocol.droppedLines);
    Protocol.println(F(",SESSION_STOPPED"));
    Protocol.emergency = previous;
    txOverflowReportPending = false;
    finishStopIfReady();
  }
  if (txFaultLatched && !txOverflowReportPending && !stopPending &&
      queueCount == 0 && Protocol.queuedBytes() == 0) {
    txFaultLatched = false;
    Protocol.droppedLines = 0;
  }
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
  unsigned long cueMs = taskMode == OMISSION ? omissionBuzzerMs : buzzerMs;
  unsigned long cueHz = taskMode == OMISSION ? omissionBuzzerHz : buzzerHz;
  if (cueMs > 0) tone(buzzerPin, (unsigned int)cueHz, cueMs);
  if (taskMode == OMISSION) Protocol.print(F("INTERVAL_START,"));
  else Protocol.print(taskMode == WINDOW ? F("WINDOW_START,") : F("TRIAL_START,"));
  Protocol.print(trialNumber);
  Protocol.print(','); Protocol.print(trialStartedAt);
  if (taskMode == WINDOW) {
    Protocol.print(','); Protocol.print(windowStartOffsetMs);
    Protocol.print(','); Protocol.print(windowEndOffsetMs);
  }
  Protocol.println();
}

void serviceTrial(unsigned long nowMs) {
  if (!sessionActive || taskMode == IMMEDIATE) return;
  if (taskMode == OMISSION) {
    if (trialState == TRIAL_RUNNING &&
        nowMs - trialStartedAt >= omissionIntervalMs) {
      noTone(buzzerPin);
      startWater();
      trialState = WAIT_REWARD;
      Protocol.print(F("REWARD,"));
      Protocol.println(trialNumber);
      Protocol.print(F("INTERVAL_END,"));
      Protocol.print(trialNumber);
      Protocol.print(F(",REWARD,"));
      Protocol.println(trialLever1 + trialLever2);
    }
    if (trialState == WAIT_REWARD && !valveActive) {
      trialGapStartedAt = nowMs;
      trialState = OMISSION_REWARD_GAP;
    }
    if (trialState == OMISSION_REWARD_GAP &&
        nowMs - trialGapStartedAt >= omissionRewardPauseMs) {
      beginTrial(nowMs);
    }
    if (trialState == OMISSION_RESET_GAP &&
        nowMs - trialGapStartedAt >= omissionResetDelayMs) {
      beginTrial(nowMs);
    }
    return;
  }
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
  printPressEvent(id, nowMs);
  if (taskMode == TRIAL || taskMode == WINDOW || taskMode == OMISSION) {
    if (trialState != TRIAL_RUNNING) return;
    if (id == 1) trialLever1++; else trialLever2++;
    if (taskMode == OMISSION) {
      if (!isActiveLever(id)) return;
      trialActivePresses++;
      noTone(buzzerPin);
      Protocol.print(F("LEVER_PRESS,"));
      Protocol.print(trialNumber);
      Protocol.print(','); Protocol.print(id);
      Protocol.print(','); Protocol.println(nowMs);
      Protocol.print(F("INTERVAL_RESET,"));
      Protocol.println(trialNumber);
      Protocol.print(F("INTERVAL_END,"));
      Protocol.print(trialNumber);
      Protocol.print(F(",RESET,"));
      Protocol.println(trialLever1 + trialLever2);
      trialGapStartedAt = nowMs;
      trialState = OMISSION_RESET_GAP;
      return;
    }
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
    Protocol.println(F("ERROR,PRESS_QUEUE_FULL,SESSION_STOPPED"));
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
  // Pending immediate presses remain immutable until their full PRESS line fits.
  // Under backpressure stop earning rewards, then drain canceled records later.
  if (!Protocol.roomFor(80)) {
    if (sessionActive) Protocol.noteOverflow(false);
    return;
  }
  PressEvent event = pressQueue[queueHead];
  bool water = sessionActive && taskMode == IMMEDIATE && event.eligible;
  if (stopPending && valveActive) return;
  if (water && (valveActive ||
      (valveGapActive && nowMs - valveClosedAt < valveClosedGapMs))) return;
  queueHead = (queueHead + 1) % queueCapacity;
  queueCount--;
  if (water) startWater();
  Protocol.print(F("PRESS,")); Protocol.print(event.number);
  Protocol.print(','); Protocol.print(event.leverId);
  Protocol.print(','); Protocol.print(water ? 1 : 0);
  Protocol.print(','); Protocol.println(event.detectedAt);
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

bool parseOmissionSettings(const char *text, unsigned long values[5]) {
  for (byte i = 0; i < 5; i++) {
    if (*text < '0' || *text > '9') return false;
    unsigned long n = 0;
    while (*text >= '0' && *text <= '9') {
      unsigned long digit = *text++ - '0';
      if (n > (3600000UL - digit) / 10UL) return false;
      n = n * 10UL + digit;
    }
    values[i] = n;
    if (i < 4) { if (*text++ != ',') return false; }
    else if (*text != '\0') return false;
  }
  return values[0] >= 1 && values[1] <= 3600000UL &&
         values[2] <= 3600000UL && values[3] <= values[0] &&
         values[4] >= 31 && values[4] <= 20000;
}

void handleCommand() {
  commandBuffer[commandLength] = '\0';
  // A fault already stops the session and queues ERROR/ACK_STOP. Do not let
  // retries start/configure a task until those records have drained in order.
  if (txFaultLatched) return;
  if (strncmp(commandBuffer, "SET_", 4) == 0 && (sessionActive || stopPending)) {
    Protocol.println(F("ERROR,STOP_BEFORE_CONFIGURING"));
    return;
  }
  if (strncmp(commandBuffer, "SET_PRESS_LOG,", 14) == 0) {
    const char *value = commandBuffer + 14;
    if (strcmp(value, "ON") == 0) pressLogEnabled = true;
    else if (strcmp(value, "OFF") == 0) pressLogEnabled = false;
    else { Protocol.println(F("ERROR,INVALID_PRESS_LOG")); return; }
    Protocol.print(F("ACK_PRESS_LOG,")); Protocol.println(value);
  } else if (strncmp(commandBuffer, "SET_TASK,", 9) == 0) {
    const char *value = commandBuffer + 9;
    if (strcmp(value, "IMMEDIATE") == 0) taskMode = IMMEDIATE;
    else if (strcmp(value, "TRIAL") == 0) taskMode = TRIAL;
    else if (strcmp(value, "WINDOW") == 0) taskMode = WINDOW;
    else if (strcmp(value, "OMISSION") == 0) taskMode = OMISSION;
    else { Protocol.println(F("ERROR,INVALID_TASK")); return; }
    Protocol.print(F("ACK_TASK,")); Protocol.println(value);
  } else if (strncmp(commandBuffer, "SET_ACTIVE,", 11) == 0) {
    const char *value = commandBuffer + 11;
    if (strcmp(value, "BOTH") == 0) activeLever = 0;
    else if (strcmp(value, "LEVER1") == 0) activeLever = 1;
    else if (strcmp(value, "LEVER2") == 0) activeLever = 2;
    else { Protocol.println(F("ERROR,INVALID_ACTIVE_MODE")); return; }
    Protocol.print(F("ACK_ACTIVE,")); Protocol.println(value);
  } else if (strncmp(commandBuffer, "SET_TRIAL,", 10) == 0) {
    unsigned long values[4];
    if (!parseTrialSettings(commandBuffer + 10, values)) {
      Protocol.println(F("ERROR,INVALID_TRIAL_SETTINGS")); return;
    }
    trialWindowMs = values[0]; trialPauseMs = values[1];
    windowConfigured = false; // Revalidate window bounds after any duration change.
    buzzerMs = values[2]; buzzerHz = values[3];
    Protocol.print(F("ACK_TRIAL,"));
    for (byte i = 0; i < 4; i++) {
      Protocol.print(values[i]);
      if (i < 3) Protocol.print(','); else Protocol.println();
    }
  } else if (strncmp(commandBuffer, "SET_WINDOW,", 11) == 0) {
    unsigned long values[5];
    if (!parseWindowSettings(commandBuffer + 11, values)) {
      Protocol.println(F("ERROR,INVALID_WINDOW_SETTINGS")); return;
    }
    windowStartMinMs = values[0]; windowStartMaxMs = values[1];
    windowLengthMinMs = values[2]; windowLengthMaxMs = values[3];
    windowSeed = values[4];
    windowConfigured = true;
    Protocol.print(F("ACK_WINDOW,"));
    for (byte i = 0; i < 5; i++) {
      Protocol.print(values[i]);
      if (i < 4) Protocol.print(','); else Protocol.println();
    }
  } else if (strncmp(commandBuffer, "SET_OMISSION,", 13) == 0) {
    unsigned long values[5];
    if (!parseOmissionSettings(commandBuffer + 13, values)) {
      Protocol.println(F("ERROR,INVALID_OMISSION_SETTINGS")); return;
    }
    omissionIntervalMs = values[0];
    omissionResetDelayMs = values[1];
    omissionRewardPauseMs = values[2];
    omissionBuzzerMs = values[3];
    omissionBuzzerHz = values[4];
    omissionConfigured = true;
    Protocol.print(F("ACK_OMISSION,"));
    for (byte i = 0; i < 5; i++) {
      Protocol.print(values[i]);
      if (i < 4) Protocol.print(','); else Protocol.println();
    }
  } else if (strcmp(commandBuffer, "START") == 0) {
    if (stopPending) { Protocol.println(F("ERROR,STOP_IN_PROGRESS")); return; }
    if (taskMode == WINDOW && !windowConfigured) {
      Protocol.println(F("ERROR,CONFIGURE_WINDOW_BEFORE_START")); return;
    }
    if (taskMode == OMISSION && !omissionConfigured) {
      Protocol.println(F("ERROR,CONFIGURE_OMISSION_BEFORE_START")); return;
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
    Protocol.println(F("ACK_START"));
    if (newSession && taskMode != IMMEDIATE) beginTrial(millis());
  } else if (strcmp(commandBuffer, "CONTINUE") == 0) {
    if (sessionActive && (taskMode == TRIAL || taskMode == WINDOW) &&
        (trialState == WAIT_REWARD || trialState == TRIAL_GAP)) {
      continueRequested = true;
      Protocol.print(F("ACK_CONTINUE,")); Protocol.println(trialNumber);
    } else Protocol.println(F("ERROR,CONTINUE_NOT_EXPECTED"));
  } else if (strcmp(commandBuffer, "STOP") == 0) {
    requestStop();
  } else if (commandLength > 0) Protocol.println(F("ERROR,UNKNOWN_COMMAND"));
}

void readCommands() {
  byte bytesRead = 0;
  while (Serial.available() > 0 && bytesRead++ < 32) {
    char c = Serial.read();
    if (c == '\n') {
      if (commandOverflow) Protocol.println(F("ERROR,COMMAND_TOO_LONG"));
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
  Protocol.println(F("READY"));
}

void loop() {
  serviceWater(millis());
  serviceTx();
  serviceTxFault();
  readCommands();
  serviceTxFault();
  unsigned long nowMs = millis();
  // Close a window BEFORE processing presses at or past its deadline.
  serviceTrial(nowMs);
  serviceTxFault();
  updateLever(lever1, nowMs);
  serviceTxFault();
  updateLever(lever2, nowMs);
  serviceTxFault();
  serviceWater(millis());
  servicePressQueue(millis());
  serviceTxFault();
  finishStopIfReady();
  serviceTx();
}
