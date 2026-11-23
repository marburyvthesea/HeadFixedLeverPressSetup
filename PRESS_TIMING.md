# Individual press timestamps

Replace `runRandomActiveWindowLeverSession.m` and
`runBuzzerLeverSessionVideoRecord_unified.m` with these versions and upload the
included `leverTaskControllerWindow/leverTaskControllerWindow.ino` once. Both
scripts remain standalone, section-by-section MATLAB scripts. Your COM port,
subject names, camera and folder settings still need to match the rig.

The shared sketch retains the current IMMEDIATE, TRIAL, WINDOW and OMISSION
protocols and reward rules, including the existing 350 ms reward pulse. Older
scripts can still use it. Individual press logging is opt-in and is disabled
by STOP. Each updated script requires an acknowledgment for SET_PRESS_LOG,ON
before starting, so older firmware cannot silently produce an empty press log.

## MAT output

Each existing `.mat` export now contains:

- `results`: the existing per-trial summary table, with its existing fields.
- `pressEvents`: one row per new debounced press of either lever while the
  session is active, including unselected-lever and between-trial presses.
- `pressEventInfo`: timing definitions and verification flags.

The same events are also exported beside the MAT file as `<base>_presses.csv`.
Timing is measured by Arduino at press detection, at 1 ms clock resolution,
using the existing 10 ms debounce. A lever held down is not repeated. The
recorded timestamp is the debounced detection time, not the initial switch
contact/bounce time. Arduino timing does not depend on MATLAB polling latency.

| pressEvents column | Meaning |
| --- | --- |
| Trial | Trial number, or 0 for a between-trial press |
| LeverID | 1 or 2, regardless of which lever is selected for reward |
| ArduinoMs | Raw Arduino millis() clock at detection, milliseconds since boot (wraps at 2^32) |
| TrialElapsedMs | Detection time minus trial/buzzer onset, or -1 between trials |
| TrialTimeSeconds | TrialElapsedMs / 1000, or NaN between trials |
| SessionTimeSeconds | Seconds since the first trial/buzzer onset, including pauses |
| InTrial | True inside a trial; false during the reward pulse or intertrial pause |
| SelectedLever | True if this lever is selected by activeLevers |
| Qualifying | True if this press counts toward that trial's reward rule |
| Phase | TRIAL, EARLY, WINDOW, LATE, or INTERTRIAL |
| HostReceivedAt | MATLAB receipt datetime; not the timestamp used for behavioral timing |

In full-trial mode, all in-trial presses have Phase=TRIAL; selected-lever presses
qualify. In random-window mode, only selected-lever presses in Phase=WINDOW
qualify. Window start is inclusive and end is exclusive. Between-trial presses
are recorded but never counted toward reward or included in trial counts.
Presses detected at/past a trial deadline are processed after trial closure and
are labeled INTERTRIAL. STOP ends event recording. Qualifying does not mean an
individual reward was delivered: these tasks deliver at most one reward at
completed trial end, and aborted trials are not rewarded.

`SessionTimeSeconds` uses modulo arithmetic to handle an Arduino clock rollover
within a session shorter than 2^32 ms (about 49.7 days). `ArduinoMs` is retained
unchanged for analysis. HostReceivedAt and the video are not hardware synchronized;
do not interpret host receipt time as an exact video-frame alignment.

Example MATLAB analysis:

```matlab
S = load('your_session.mat');
events = S.pressEvents;
% Times in seconds after buzzer onset for lever 2 in trial 3:
t = events.TrialTimeSeconds(events.Trial==3 & events.LeverID==2);
% Only eligible presses within random active windows:
eligible = events(events.Qualifying,:);
```

`pressEventInfo.MessagesValid` reports malformed event/start records.
`ClockOriginValid` reports whether the first trial Arduino timestamp was received.
`TrialCountsMatch` compares the event log against both per-lever and qualifying
counts for completed and aborted trials. `StopAcknowledged` indicates whether
STOP was confirmed. These checks cannot verify completeness of between-trial
presses because those are excluded from trial summaries. A warning is emitted
when verification is incomplete. Received event rows remain available for review.

## Protocol

Configure while stopped:

```
SET_PRESS_LOG,ON  -> ACK_PRESS_LOG,ON
SET_PRESS_LOG,OFF -> ACK_PRESS_LOG,OFF
```

In TRIAL/WINDOW sessions with logging enabled, each new press emits nine fields:

```
PRESS_EVENT,<trial>,<lever>,<ArduinoMs>,<elapsedMs>,<inTrial>,<selected>,<qualifying>,<phase>
```

Events appear in detection order ahead of their corresponding trial summary.
TRIAL_START/END/ABORT and WINDOW_START/END/ABORT keep their original formats.
No additional press events are emitted in IMMEDIATE or OMISSION mode, preserving
those scripts' protocols. STOP resets logging off before the next session.

## Verification and limits

Host checks compile and execute the actual sketch with mocked clock, GPIO and
serial. They cover phases, both levers, selected/qualifying flags, held/debounced
presses, exact window/trial boundaries, STOP/abort, clock wrap, opt-in behavior,
and existing immediate/omission reward behavior. Run from this folder:

```sh
g++ -std=c++17 -Wall -Wextra -Werror -I tests tests/press_event_controller_test.cpp -o /tmp/press_event_controller_test
/tmp/press_event_controller_test
```

Firmware behavior checks passed and the MATLAB data flow was statically reviewed.
MATLAB and physical board/camera execution were not available here. Run Verify
in Arduino IDE and a short rig session, then check S.pressEvents and
S.pressEventInfo before collecting data. Main-loop serial errors now attempt STOP and save parsed data, with diagnostics;
Ctrl+C, startup errors or export failure can still bypass normal saving. See
SERIAL_STALL_FIX.md for transport buffering, progress timeouts and verification.
These scripts do not promise crash-proof persistence.
