# Serial-stall hardening

This update addresses two code paths that can contribute to the reported symptom:
video keeps recording, but the behavioral trial stops progressing. That symptom
alone does not establish whether the rig experienced USB backpressure, a USB
reset/disconnection, or another host/board fault.

## Findings

The press-timing update sends a line for every debounced press, from inside the
Arduino control loop. On the UNO R4 native USB implementation, Serial.write can
wait when the transmit FIFO is full. While waiting there, the sketch cannot
advance trials, read STOP, or run its software-controlled valve-close check.
Existing trial-summary prints could also block. Typical lever traffic is small,
so this is a backpressure susceptibility rather than proof of bandwidth overload.

Core source reviewed: https://github.com/arduino/ArduinoCore-renesas/blob/main/cores/arduino/usb/SerialUSB.cpp

The scripts also called readline whenever any byte was available. A partial line
can make that call wait for its terminator. Both tasks require MATLAB to read a
trial-end record before sending CONTINUE, so a communication stall can look like
a behavioral task that has simply stopped between trials.

## Changes

- All Arduino protocol output uses a 4096-byte FIFO with atomic complete lines,
  including events, summaries, ACKs and errors. The normal loop sends at most
  64 bytes per service call and never requests more than availableForWrite.
  Partial/zero writes leave unsent bytes queued. Timing, lever sampling and
  STOP handling continue when the host stops reading.
- Finite overflow stops the session and reports
  `ERROR,TX_OVERFLOW,<droppedProtocolLines>,SESSION_STOPPED` when transport
  progresses again. There is reserved space for stop/error records. Earlier
  queued records retain their order; loss is explicit rather than silent.
  Already-started valve pulses are allowed to finish at the existing 350 ms.
- Both MATLAB scripts retain received fragments and process only complete LF
  records. Setup and acquisition share that buffer, preserving records that
  arrive in the same transfer as ACK_START. Short reads are not solved by
  discarding partial lines or merely reducing serial timeout.
- A progress watchdog includes the configured trial duration/pause plus
  `serialStallGraceSeconds` (default 10 seconds). A long press-free trial is
  normal. A missing trial-end or next-start record beyond the expected time
  causes a warning, a STOP attempt, and saving of parsed data. It processes
  queued complete records first and yields to video/UI work regularly.
- Main-loop transport exceptions also attempt STOP and continue to saving.
  This does not guarantee a final aborted-trial summary if the port failed.
  Existing press-event verification flags reveal unmatched/incomplete trials.

Task modes, reward rules, 350 ms pulse, press timestamps and existing result
fields are preserved. The unified setup still uses one .ino. No reconnect or
automatic CONTINUE retry silently starts/resumes a trial after a fault.

## Install and check

1. End the current session and close MATLAB's serial connection.
2. Replace both MATLAB scripts and upload the included
   `leverTaskControllerWindow/leverTaskControllerWindow.ino` once.
3. Keep your local COM port, subject IDs, data paths, camera and trial settings.
4. Run Arduino IDE Verify and a short rig check before collecting data.

The MAT file now includes `serialDiagnostics` and `serialStallGraceSeconds`.
Useful fields are `StallDetected`, `StallReason`, `MessageAtStall`,
`TransportAtStall`, `TransportError`, and `LastMessage`/`LastTransport`.
Transport diagnostics report received byte/line counts, cached bytes, maximum
cache size, whether a complete line remains, and time since bytes/lines arrived.

If the problem recurs, retain the warning text and saved serialDiagnostics.
A READY message during a running session indicates a controller reset; a
TX_OVERFLOW message identifies output backpressure/queue exhaustion. A plain
overdue-progress warning alone does not prove the underlying hardware cause.

## Verification

Host firmware tests passed for zero transmit capacity, a disconnected writer,
partial writes, recovery/FIFO order, queue overflow, responsive STOP, and valve
closure on the existing deadline. Existing task/press-log regression checks pass.

```sh
g++ -std=c++17 -Wall -Wextra -Werror -I tests tests/press_event_controller_test.cpp -o /tmp/press_event_controller_test
/tmp/press_event_controller_test
```

A MATLAB reader regression test is included. It extracts each script's actual
reader and exercises fragmented lines, combined ACK_START/start records, cached
records, resets and receive-backlog bounds, without hardware or a camera:

```matlab
addpath('tests');
test_serial_reader
```

MATLAB runtime, Arduino board compilation and a physical USB/camera test were
not available here. The MATLAB paths were reviewed statically; the reader test
has not been executed here. Ctrl+C, startup errors or failure during saving can
still bypass normal exports. This patch is not a claim that the rig's observed
root cause was reproduced.
