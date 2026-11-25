# Experimental trial-buffered press logging

This is an alternative to live press streaming and the general USB transmit
queue in commit `57be043`. It uses the earlier direct command/acknowledgment
handshake and removes timestamp transmission from the timed trial itself.
It is isolated on `experiment/buffered-trial-logging`; main is not replaced.

## How it works

1. Arduino starts the trial and sounds the buzzer. Both levers' new debounced
   in-trial presses are stored as timestamped records in RAM. There is no USB
   output during this timed BATCH trial or its reward pulse.
2. At trial end, the usual eligibility rule determines the one reward command.
   Readout remains deferred until the valve pulse has finished.
3. MATLAB requests short replies, one at a time. Each press-data reply contains
   up to four retained timestamps and stays below 64 bytes including CRLF.
   The next request is sent only after the previous reply has been received.
4. MATLAB validates the result and writes/finalizes a MAT checkpoint BEFORE
   acknowledging the records. The Arduino will not start another trial before
   acknowledgment and CONTINUE.

This protects the trial and valve timing from per-press USB transmission. A
USB/driver failure can still prevent communication during idle/readout; the
code cannot guarantee reconnection or fix a cable/driver fault. RAM data stays
available until acknowledged, unless the board resets or loses power.

## Keep your working checkout intact

Your main checkout has local changes in the shared .ino and window script.
Test this branch in another folder instead of resetting or merging over them:

```sh
git fetch origin
git worktree add -b test-buffered ../LeverPress-buffered origin/experiment/buffered-trial-logging
```

Change MATLAB's Current Folder to the NEW `LeverPress-buffered` directory.
Upload `leverTaskControllerWindow/leverTaskControllerWindow.ino` from that new
folder, then use its scripts. Do not mix scripts/firmware from different versions.
Copy only your rig/subject settings into the new scripts; leave the old source,
local changes and stash intact. If a local branch named test-buffered already
exists, choose another new local branch name for the worktree.

## Run

Open either `runRandomActiveWindowLeverSession.m` or
`runBuzzerLeverSessionVideoRecord_unified.m`. Click through USER SETTINGS,
VIDEO SETTINGS, then RUN SESSION. Keep `runBufferedLeverSession.m` and
`decodeBufferedLeverTrial.m` alongside the scripts/on the MATLAB path.
The helper owns serial/camera cleanup; there is no separate GUI or separate
Arduino sketch per task.

COM port, session/window durations, lever selection, buzzer, output identifiers,
and camera defaults follow the existing scripts. Change those for your rig.
Video recording and live preview are independently optional; set BOTH false to
run without a camera. Close Serial Monitor before MATLAB connects.

The sketch preserves the 325 ms solenoid pulse in the firmware you pasted. The
runner reads the actual configured pulse duration back from the board. Check
that this matches your calibration; it does not infer measured water flow.

`saveResults=true` is required: BATCH mode does not acknowledge and overwrite
RAM records unless their MAT checkpoint has been finalized successfully.

## What changes operationally

- Capture covers **in-trial presses only**, both selected and unselected levers,
  including early/late presses in random-window sessions. Intertrial presses
  remain irrelevant to reward and are not timestamped in this alternative.
- The buffer holds 1,024 presses PER TRIAL in 4 KiB. It is not a whole-session limit.
  If it fills, the incomplete trial is aborted without reward, the session
  stops, and retained records/device counts explicitly report dropped times.
  Never interpret an overflowed trial as complete data.
- Readout/checkpoint time may extend the gap between trials. The configured
  pause remains a minimum; the timed trial and buzzer/reward rules stay on
  Arduino. Actual onset/end times remain in the dataset.
- TrialStart/TrialEnd host wall-clock fields are NaT because no live onset/end
  messages are streamed. HostReadoutAt and HostReceivedAt refer to later data
  retrieval, NOT precise behavioral events or video-frame synchronization.
  Use ArduinoStartMs/ArduinoEndMs and trial-relative times for behavior.
- The new command is SET_PRESS_LOG,BATCH. Live SET_PRESS_LOG,ON is rejected.
  Legacy task modes with logging OFF, IMMEDIATE and OMISSION retain their
  existing protocols and behaviors. The USB-silent guarantee applies to the
  new BATCH TRIAL/WINDOW path, not all legacy modes.

## Saved data and recovery

A timestamped session directory contains an optional video, serial.log, per-trial
MAT checkpoints, and final trial/press CSVs. The checkpoint file retains the
existing results and pressEvents tables plus session/settings, pressEventInfo,
raw trialBatches and per-trial verification information. It is updated after
each trial, before the device is acknowledged.

pressEvents includes Trial, LeverID, raw ArduinoMs, TrialElapsedMs,
TrialTimeSeconds, SessionTimeSeconds, InTrial, SelectedLever, Qualifying,
Phase, and HostReceivedAt. Clock-wrap handling and early/window/late semantics
remain explicit. Qualifying is eligibility, not a reward delivered per press.
Aborted trials receive no new reward; an already-started pulse finishes.

A failed fetch, validation or save leaves the snapshot unacknowledged on the
board. On reconnect, the runner first issues STOP and recovers that retained
snapshot to a separate recovered_device_trial file, saves it, then acknowledges
it before configuring a new session. Recovered RAM cannot identify the prior
subject or session: its payload is explicitly marked as unknown-prior-session
provenance. Do not assign it to the current subject just because it resides in
that output folder. Power loss/reset destroys the volatile retained snapshot.

Ctrl+C attempts STOP through function-scoped cleanup. Earlier checkpoints stay
on disk; the last unacknowledged snapshot may be recovered on reconnect. A hard
process/board/OS/storage failure can still prevent cleanup or data persistence.

## Protocol summary

Each request normally has exactly one reply. STOP can cancel a deferred GET_BATCH.

| Request | Reply |
| --- | --- |
| SET_PRESS_LOG,BATCH | ACK_PRESS_LOG,BATCH,1024,pulseMs |
| GET_BATCH | BATCH_EMPTY or BATCH_READY,trial,stored,dropped,completed,water,acknowledged |
| GET_META,trial | META,trial,startMs,endMs |
| GET_COUNTS,trial | COUNTS,trial,lever1,lever2,qualifying |
| GET_WINDOW,trial | WINDOW,trial,startOffset,endOffset |
| GET_CONTEXT,trial | CONTEXT,trial,mode,activeLevers,pulseMs |
| GET_PRESS,trial,firstIndex | PRESS_DATA,trial,firstIndex,word1[,word2,word3,word4] |
| ACK_BATCH,trial | ACK_BATCH,trial |

Indices start at 1. Each packed word is (elapsedMs<<1)|(leverId-1).
GET_BATCH during an active trial waits silently for hardware to become quiet.
A request immediately after CONTINUE targets the NEXT trial, even during the
remaining pause. Read and acknowledgment retries are idempotent. Configuration,
a new session and CONTINUE cannot overwrite an unacknowledged snapshot.

## Verification status

Host C++ tests passed, exercising the actual sketch with mocked GPIO/clock/serial:
startup ACKs, zero USB output during timed trials/pulses, the supplied
325 ms pulse, STOP/deferred pulls, four-word pages, read/ACK retries, retention,
overflow, count consistency, clock wrap and legacy task regressions. The mock
uses CRLF and verifies the maximum press-page reply is 60 bytes.

```sh
g++ -std=c++17 -Wall -Wextra -Werror -I tests tests/batch_controller_test.cpp -o /tmp/batch_controller_test
/tmp/batch_controller_test
```

The MATLAB path was independently reviewed. Decoder tests are included:

```matlab
addpath('tests');
test_buffered_decoder
```

MATLAB execution, Arduino IDE/board compilation and a physical USB/camera test
were unavailable here. Those MATLAB tests have not been run by this agent.
Start with a short bench session and inspect results/pressEvents/pressEventInfo
before collecting experimental data. This is an experimental alternative,
not a claim that the rig's original USB failure has been reproduced or cured.
