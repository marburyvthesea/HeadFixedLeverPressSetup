# Experimental buffered lever sessions

This branch provides a trial-buffered alternative for individual press timing.
It does not replace main. See [BATCH_LOGGING.md](BATCH_LOGGING.md) for protocol,
installation, limits, recovery behavior and verification status.

The Arduino stores both levers' in-trial press times in RAM and sends no USB
messages during a timed BATCH trial or its reward pulse. MATLAB retrieves small
pages after the pulse ends, validates/saves a MAT checkpoint, then acknowledges
the data before requesting the next trial. The same unified .ino still supports
IMMEDIATE, TRIAL, WINDOW and OMISSION; BATCH capture applies to TRIAL/WINDOW.

Run either `runBuzzerLeverSessionVideoRecord_unified.m` or
`runRandomActiveWindowLeverSession.m` with `runBufferedLeverSession.m` and
`decodeBufferedLeverTrial.m` alongside/on the MATLAB path. Upload the matching
`leverTaskControllerWindow/leverTaskControllerWindow.ino` from this branch.
Click through USER SETTINGS, VIDEO SETTINGS and RUN SESSION.

Important differences: up to 1,024 in-trial presses per trial; no intertrial
press timestamps; readout can extend the intertrial gap. The supplied firmware
retains the 325 ms pulse from the uploaded rig sketch. MAT files retain the
per-press timestamps, trial summaries, raw batches and explicit loss/status data.

Use a separate Git worktree to keep your working scripts and calibration edits
intact. No reset, stash pop, or replacement of main is needed:

```sh
git fetch origin
git worktree add -b test-buffered ../LeverPress-buffered origin/experiment/buffered-trial-logging
```

Host firmware tests pass. MATLAB decoding/transport code was reviewed and tests
are included, but MATLAB and physical board/camera execution were unavailable.
Run Arduino IDE Verify and a short bench session before using this version for
data collection. No claim is made that the earlier rig's USB fault is cured.
