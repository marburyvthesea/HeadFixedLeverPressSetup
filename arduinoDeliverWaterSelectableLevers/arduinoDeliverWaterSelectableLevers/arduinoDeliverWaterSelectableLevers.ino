/*
  deliverWaterSelectableLevers

  Lever 1: NO -> 2, NC -> 3, COM -> GND
  Lever 2: NO -> 9, NC -> 10, COM -> GND
  Pin 7 -> active-HIGH solenoid DRIVER input.
  Never connect the 24 V valve coil directly to an Arduino pin.

  Serial: 115200 baud, newline-terminated commands.
    SET_ACTIVE,BOTH / SET_ACTIVE,LEVER1 / SET_ACTIVE,LEVER2
      -> ACK_ACTIVE,<mode> (configure while stopped, before START)
    START -> ACK_START (repeated START does not reset a running session)
    STOP  -> ACK_STOP after the current water pulse has finished
    Boot  -> READY

  Both levers are logged:
    PRESS,<pressNumber>,<leverId>,<water>,<pressDetectionMillis>
  The 1 means the valve was commanded on, not measured water delivery.
  The 0 means no water command (inactive lever or canceled queued reward).
  Overlapping presses wait in a FIFO for separate pulses.
  STOP finishes an already started pulse, then logs waiting presses with
  water=0, without rewarding them, before ACK_STOP.
  Default mode is BOTH. The selected mode stays fixed until STOP.
*/

const byte lever1NoPin = 2;
const byte lever1NcPin = 3;
const byte lever2NoPin = 9;
const byte lever2NcPin = 10;
const byte solenoidPin = 7;

const unsigned long debounceMs = 10;
const unsigned long solenoidMs = 220;
const unsigned long valveClosedGapMs = 50;
const byte valveOnLevel = HIGH;
const byte valveOffLevel = LOW;

enum LeverState { RELEASED, PRESSED, INVALID };

struct Lever {
  byte noPin;
  byte ncPin;
  byte id;
  LeverState raw;
  LeverState stable;
  unsigned long rawChangedAt;
  bool armed;
};

Lever lever1 = {lever1NoPin, lever1NcPin, 1, INVALID, INVALID, 0, false};
Lever lever2 = {lever2NoPin, lever2NcPin, 2, INVALID, INVALID, 0, false};

struct PressEvent {
  unsigned long number;
  unsigned long detectedAt;
  byte leverId;
  bool eligible;
};

const byte queueCapacity = 32;
PressEvent rewardQueue[queueCapacity];
byte queueHead = 0;
byte queueTail = 0;
byte queueCount = 0;

bool sessionActive = false;
bool stopPending = false;
bool valveActive = false;
bool gapActive = false;
unsigned long valveStartedAt = 0;
unsigned long valveClosedAt = 0;
unsigned long pressNumber = 0;
byte activeLever = 0; // 0 = BOTH, 1 = LEVER1, 2 = LEVER2

char commandBuffer[24];
byte commandLength = 0;
bool commandOverflow = false;

LeverState readLever(const Lever &lever) {
  bool noClosed = digitalRead(lever.noPin) == LOW;
  bool ncClosed = digitalRead(lever.ncPin) == LOW;
  if (noClosed && !ncClosed) return PRESSED;
  if (!noClosed && ncClosed) return RELEASED;
  return INVALID;
}

void clearQueue() {
  queueHead = queueTail = queueCount = 0;
}

void finishStopIfReady() {
  if (stopPending && !valveActive && queueCount == 0) {
    stopPending = false;
    Serial.println(F("ACK_STOP"));
  }
}

void requestStop() {
  sessionActive = false;
  stopPending = true;
  finishStopIfReady();
}

void enqueuePress(byte leverId, unsigned long nowMs) {
  if (queueCount >= queueCapacity) {
    // Fail visibly rather than silently dropping rewards and continuing.
    Serial.println(F("ERROR,REWARD_QUEUE_FULL,SESSION_STOPPED"));
    requestStop();
    return;
  }

  PressEvent &event = rewardQueue[queueTail];
  event.number = ++pressNumber;
  event.detectedAt = nowMs;
  event.leverId = leverId;
  event.eligible = (activeLever == 0 || activeLever == leverId);
  queueTail = (queueTail + 1) % queueCapacity;
  queueCount++;
}

void updateLever(Lever &lever, unsigned long nowMs) {
  LeverState state = readLever(lever);
  if (state != lever.raw) {
    lever.raw = state;
    lever.rawChangedAt = nowMs;
  }

  if (nowMs - lever.rawChangedAt < debounceMs) return;
  lever.stable = state;

  if (state == RELEASED) {
    lever.armed = true;
  } else if (state == PRESSED && lever.armed) {
    // Only a debounced release can rearm this lever.
    // INVALID contact states cannot create repeated rewards while held.
    lever.armed = false;
    if (sessionActive) enqueuePress(lever.id, nowMs);
  }
}

void serviceValve(unsigned long nowMs) {
  if (valveActive && nowMs - valveStartedAt >= solenoidMs) {
    digitalWrite(solenoidPin, valveOffLevel);
    valveActive = false;
    valveClosedAt = nowMs;
    gapActive = true;
  }

  finishStopIfReady();
  if (queueCount == 0) return;

  PressEvent event = rewardQueue[queueHead];
  bool giveWater = sessionActive && event.eligible;
  // During STOP, finish the current pulse before draining canceled events.
  if (stopPending && valveActive) return;
  if (giveWater) {
    if (valveActive) return;
    if (gapActive && nowMs - valveClosedAt < valveClosedGapMs) return;
  }
  queueHead = (queueHead + 1) % queueCapacity;
  queueCount--;

  if (giveWater) {
    digitalWrite(solenoidPin, valveOnLevel);
    valveStartedAt = millis();
    valveActive = true;
    gapActive = false;
  }

  Serial.print(F("PRESS,"));
  Serial.print(event.number);
  Serial.print(',');
  Serial.print(event.leverId);
  Serial.print(',');
  Serial.print(giveWater ? 1 : 0);
  Serial.print(',');
  Serial.println(event.detectedAt);
  finishStopIfReady();
}

void handleCommand() {
  commandBuffer[commandLength] = '\0';
  if (strncmp(commandBuffer, "SET_ACTIVE,", 11) == 0) {
    if (sessionActive || stopPending) {
      Serial.println(F("ERROR,STOP_BEFORE_SETTING_MODE"));
      return;
    }
    const char *mode = commandBuffer + 11;
    if (strcmp(mode, "BOTH") == 0) {
      activeLever = 0;
    } else if (strcmp(mode, "LEVER1") == 0) {
      activeLever = 1;
    } else if (strcmp(mode, "LEVER2") == 0) {
      activeLever = 2;
    } else {
      Serial.println(F("ERROR,INVALID_ACTIVE_MODE"));
      return;
    }
    Serial.print(F("ACK_ACTIVE,"));
    Serial.println(mode);
  } else if (strcmp(commandBuffer, "START") == 0) {
    if (stopPending) {
      Serial.println(F("ERROR,STOP_IN_PROGRESS"));
      return;
    }
    if (!sessionActive) {
      clearQueue();
      pressNumber = 0;
      // Require a stable release after START before accepting a press.
      lever1.armed = false;
      lever2.armed = false;
      sessionActive = true;
    }
    Serial.println(F("ACK_START"));
  } else if (strcmp(commandBuffer, "STOP") == 0) {
    requestStop();
  } else if (commandLength != 0) {
    Serial.println(F("ERROR,UNKNOWN_COMMAND"));
  }
}

void readCommands() {
  // Bound work per loop so serial input cannot starve valve timing.
  byte bytesRead = 0;
  while (Serial.available() > 0 && bytesRead < 32) {
    char c = Serial.read();
    bytesRead++;
    if (c == '\n') {
      if (commandOverflow) {
        Serial.println(F("ERROR,COMMAND_TOO_LONG"));
      } else {
        handleCommand();
      }
      commandLength = 0;
      commandOverflow = false;
    } else if (c != '\r') {
      if (commandLength < sizeof(commandBuffer) - 1) {
        commandBuffer[commandLength++] = c;
      } else {
        commandOverflow = true;
      }
    }
  }
}

void setup() {
  digitalWrite(solenoidPin, valveOffLevel);
  pinMode(solenoidPin, OUTPUT);
  pinMode(lever1NoPin, INPUT_PULLUP);
  pinMode(lever1NcPin, INPUT_PULLUP);
  pinMode(lever2NoPin, INPUT_PULLUP);
  pinMode(lever2NcPin, INPUT_PULLUP);
  Serial.begin(115200);
  Serial.println(F("READY"));
}

void loop() {
  readCommands();
  unsigned long nowMs = millis();
  updateLever(lever1, nowMs);
  updateLever(lever2, nowMs);
  serviceValve(millis());
}
