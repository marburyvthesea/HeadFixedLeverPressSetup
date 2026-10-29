# Head-fixed lever press tasks

The controller in [leverTaskControllerWindow/leverTaskControllerWindow.ino](leverTaskControllerWindow/leverTaskControllerWindow.ino) supports three task modes. Upload it once, then choose the task by running its MATLAB script.

| Task | MATLAB script | Reward rule |
| --- | --- | --- |
| Immediate | `deliverWaterImmediatelyAfterEveryPress_selectLevers_unified.m` | One pulse per selected-lever press; overlapping deliveries are queued. |
| Full trial | `runBuzzerLeverSessionVideoRecord_unified.m` | One pulse at trial end if a selected lever was pressed during the trial. |
| Random active window | `runRandomActiveWindowLeverSession.m` | One pulse at trial end if a selected lever was pressed inside the randomized window. |

The extended controller retains the existing IMMEDIATE and TRIAL command and result formats. Older sketches remain in the repository for reference.

## Wiring

- Lever 1: NO to digital 2, NC to digital 3, COM to GND.
- Lever 2: NO to digital 9, NC to digital 10, COM to GND.
- Buzzer output: digital 5.
- Active-HIGH solenoid-driver input: digital 7.
- Valve pulse: 220 ms. The 24 V valve requires its external power supply and driver; do not connect its coil directly to an Arduino GPIO.

## Random active-window session

Open `runRandomActiveWindowLeverSession.m`. Check the COM port, camera ID, output folder, and subject/file identifiers before starting.

The main settings are:

```matlab
activeLevers = "LEVER2";                 % "LEVER1", "LEVER2", or "BOTH"
trialSeconds = 15;
windowStartRangeSeconds = [1 8];
activeWindowLengthRangeSeconds = [2 5]; % [3 3] for fixed 3-second duration
interTrialSeconds = 1;
buzzerSeconds = 0.5;
maxWaterDeliveries = 100;               % Inf disables this cap
randomSeed = [];                        % Auto-select and save a seed
```

The latest start plus the longest window must fit inside the trial. Window onset and length are independently sampled in integer milliseconds. A fixed seed and the same window settings reproduce the offset sequence.

Run the setup sections in order. Running the **START SESSION** section continues through acquisition, the main loop, cleanup, and saving. Close Arduino Serial Monitor before MATLAB opens the serial port.

Only fresh, debounced presses of a selected lever in the interval **start inclusive, end exclusive** qualify. A lever held down before the window does not become a new qualifying press when the window opens. There is a buzzer at trial onset but no additional window-onset cue. All in-trial presses are counted, including inactive-lever, early, and late presses. Between-trial presses are ignored.

A qualifying trial receives one reward at the end of the full trial, not at window end. The pause begins after reward completion, or at the end of an unrewarded trial. The session stops at its time limit or reward cap. An interrupted trial is saved as incomplete without reward; an already-started water pulse is allowed to finish.

## Saved data

The random-window script saves MAT, CSV, text metadata, and Excel results beside the video. Data includes actual window offsets, per-lever early/window/late counts, qualifying counts, reward commands, completion status, and Arduino timestamps. Metadata includes the selected lever, configured timing bounds, random seed, and stop reason. `WaterDelivered=1` records a valve command, not measured water flow.

## Validation status

The protocol and MATLAB section flow have been reviewed. Automated compilation in the development sandbox was blocked by a Windows DLL relocation error launching the installed ARM compiler. Run **Verify** in Arduino IDE for the UNO R4 WiFi before uploading. Hardware behavior and camera acquisition have not been tested by the authoring agent.
