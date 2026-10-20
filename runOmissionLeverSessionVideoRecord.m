%% runOmissionLeverSessionVideoRecord.m
% Omission training with video recording.
%
% REVERSED CONTINGENCY
% A water reward is delivered only after the mouse completes an entire
% omissionIntervalSec without a lever press.  A press immediately cancels
% that interval, delivers NO water, waits resetDelaySec, then starts a new
% cued interval.  Thus pressing delays (rather than earns) water.
%
% IMPORTANT -- this controller requires an omission-capable Arduino sketch.
% The repository's original arduinoBuzzerThenCountLeverPresses.ino implements
% a fixed-interval press-to-reward contingency and only understands START,
% CONTINUE, and STOP.  It cannot implement immediate resets from MATLAB
% alone.  Flash firmware that implements the serial contract listed below
% before running this file; the original buzzer/valve pin assignments and
% timing constants are reproduced in USER SETTINGS for that firmware.
%
% Required Arduino serial contract (one LF-terminated line per message):
%   MATLAB -> OMISSION_CONFIG,<intervalMs>,<resetDelayMs>,<itiMs>,...
%   MATLAB -> START_OMISSION
%   Arduino -> ACK_OMISSION_CONFIG
%   Arduino -> ACK_START_OMISSION
%   Arduino -> INTERVAL_START,<intervalNumber>
%   Arduino -> LEVER_PRESS,<intervalNumber>,<leverNumber>
%   Arduino -> INTERVAL_RESET,<intervalNumber>
%   Arduino -> REWARD,<intervalNumber>
%   Arduino -> INTERVAL_END,<intervalNumber>,REWARD|RESET,<pressCount>
%   MATLAB -> STOP
%   Arduino -> ACK_STOP
%
% The Arduino must: sound the buzzer at every INTERVAL_START; monitor both
% levers with the same debounce handling as the original sketch; pulse the
% valve at interval end only if no press occurred; and ignore presses during
% the reset/inter-trial delay.  It must produce INTERVAL_RESET immediately
% after the first valid press so MATLAB can log the cancellation precisely.

function runOmissionLeverSessionVideoRecord
% The function form lets the nested event logger share the session state.
clc;

%% ===================== USER SETTINGS =====================
% Serial/session settings
portName           = "COM4";  % Change to the Arduino port.
baudRate           = 115200;  % Must match Serial.begin().
sessionDurationMin = 30;
omissionIntervalSec = 20;     % Required press-free duration for one reward.
resetDelaySec      = 1;       % Brief delay after a press before the next cue.
interTrialDelaySec = 1;       % Brief delay after a reward before the next cue.
readyTimeout       = 10;      % Seconds to wait for the Arduino after connect.
ackTimeout         = 5;       % Seconds to wait for command acknowledgements.
saveResults        = true;
outFileBase        = "m0710_02_omission_lever_session";

% Preserve the original Arduino hardware constants.  These are sent to the
% omission firmware in OMISSION_CONFIG; keep them matched to the firmware.
lever1NoPin        = 2;       %#ok<NASGU> % Documentation for firmware setup.
lever1NcPin        = 3;       %#ok<NASGU>
lever2NoPin        = 9;       %#ok<NASGU>
lever2NcPin        = 10;      %#ok<NASGU>
buzzerPin          = 5;       %#ok<NASGU>
solenoidPin        = 7;       %#ok<NASGU>
debounceMs         = 10;
buzzerFreqHz       = 4000;
buzzerDurationMs   = 500;
valvePulseMs       = 220;

% Video settings preserved from runBuzzerLeverSessionVideoRecord.m.
videoFolder        = 'D:\Data\JJM\HeadFixedLeverPress\08-31-2026';
mouseIDPrefix      = 'm0710_02_omission';
videoAdaptor       = 'winvideo';
videoDeviceID      = 2;
videoFormat        = 'Y800_640x480';
cameraFrameRate    = '120.0005';
savedVideoFrameRate = 113;

%% ===================== CONNECT =====================
if exist('s', 'var')
    try delete(s); catch, end
    clear s
end

s = serialport(portName, baudRate);
configureTerminator(s, "LF");
pause(2);                         % Allow the Arduino bootloader to finish.
flush(s);

fprintf("Connecting to Arduino...\n");
if ~waitForLine(s, "READY", readyTimeout)
    % Some existing sketches print READY only once during startup.  Continue
    % to the explicit omission configuration handshake if it was missed.
    warning("Did not receive READY; attempting omission configuration.");
end

configCommand = sprintf("OMISSION_CONFIG,%d,%d,%d,%d,%d,%d,%d", ...
    round(omissionIntervalSec * 1000), round(resetDelaySec * 1000), ...
    round(interTrialDelaySec * 1000), debounceMs, buzzerFreqHz, ...
    buzzerDurationMs, valvePulseMs);
writeline(s, configCommand);
if ~waitForLine(s, "ACK_OMISSION_CONFIG", ackTimeout)
    delete(s);
    error(["The Arduino did not acknowledge OMISSION_CONFIG. Flash the " + ...
        "matching omission-capable firmware before running this script."]);
end

%% ===================== VIDEO RECORDING PARAMETERS =====================
timestamp = datestr(now, 'mm_dd_yy_HH_MM_SS');
mouseID = strcat(mouseIDPrefix, '_', timestamp);
newDirectory = fullfile(videoFolder, mouseID);
if ~exist(newDirectory, 'dir')
    mkdir(newDirectory);
end
fileName = fullfile(newDirectory, strcat(mouseID, '_T.avi'));

vid = videoinput(videoAdaptor, videoDeviceID, videoFormat);
src = getselectedsource(vid);
src.FrameRate = cameraFrameRate;
vw = VideoWriter(fileName, 'Motion JPEG AVI');
vw.Quality = 75;
vw.FrameRate = savedVideoFrameRate;
vid.LoggingMode = 'disk';
vid.DiskLogger = vw;
vid.ReturnedColorSpace = 'grayscale';
vid.FramesPerTrigger = Inf;

%% ===================== START SESSION =====================
open(vw);
preview(vid);
triggerconfig(vid, 'immediate');
start(vid);
disp('Video recording started.');

writeline(s, "START_OMISSION");
if ~waitForLine(s, "ACK_START_OMISSION", ackTimeout)
    stopAndClose(vid, vw, s);
    error("Arduino did not acknowledge START_OMISSION.");
end

sessionStart = datetime('now');
sessionEnd = sessionStart + minutes(sessionDurationMin);
fprintf("Omission session started at %s. Ends at %s (%.1f min).\n", ...
    datestr(sessionStart), datestr(sessionEnd), sessionDurationMin);

% One row per completed interval attempt; a press makes Outcome="RESET".
intervalNumber = [];
intervalStartWallClock = datetime.empty(0, 1);
intervalEndWallClock = datetime.empty(0, 1);
pressCount = [];
firstPressWallClock = datetime.empty(0, 1);
firstPressLever = [];
outcome = strings(0, 1);
waterDelivered = [];

% Event log contains every interval start, press, reset, reward, and end.
eventTime = datetime.empty(0, 1);
eventType = strings(0, 1);
eventInterval = [];
eventDetail = strings(0, 1);

currentInterval = NaN;
currentIntervalStart = NaT;
currentPressCount = 0;
currentFirstPress = NaT;
currentFirstPressLever = NaN;
running = true;

%% ===================== MAIN LOOP =====================
while running
    if datetime('now') >= sessionEnd
        fprintf("Session time budget reached. Sending STOP.\n");
        writeline(s, "STOP");
        waitForLine(s, "ACK_STOP", ackTimeout);
        break;
    end

    if s.NumBytesAvailable == 0
        pause(0.01);
        continue;
    end

    line = strtrim(readline(s));
    if line == ""
        continue;
    end
    tokens = strsplit(line, ",");
    nowTime = datetime('now');

    switch tokens{1}
        case "INTERVAL_START"
            currentInterval = str2double(tokens{2});
            currentIntervalStart = nowTime;
            currentPressCount = 0;
            currentFirstPress = NaT;
            currentFirstPressLever = NaN;
            addEvent("INTERVAL_START", currentInterval, "buzzer sounded");
            fprintf("[%s] Omission interval %d started (%.1f s press-free required).\n", ...
                datestr(nowTime, 'HH:MM:SS'), currentInterval, omissionIntervalSec);

        case "LEVER_PRESS"
            loggedInterval = str2double(tokens{2});
            leverNumber = NaN;
            if numel(tokens) >= 3
                leverNumber = str2double(tokens{3});
            end
            currentPressCount = currentPressCount + 1;
            if isnat(currentFirstPress)
                currentFirstPress = nowTime;
                currentFirstPressLever = leverNumber;
            end
            addEvent("LEVER_PRESS", loggedInterval, sprintf("lever=%g", leverNumber));
            fprintf("[%s] Lever %g press: interval %d cancelled; no reward.\n", ...
                datestr(nowTime, 'HH:MM:SS'), leverNumber, loggedInterval);

        case "INTERVAL_RESET"
            addEvent("INTERVAL_RESET", str2double(tokens{2}), ...
                sprintf("resetDelaySec=%.3f", resetDelaySec));

        case "REWARD"
            addEvent("REWARD", str2double(tokens{2}), ...
                sprintf("valvePulseMs=%d", valvePulseMs));
            fprintf("[%s] Water reward delivered.\n", datestr(nowTime, 'HH:MM:SS'));

        case "INTERVAL_END"
            endedInterval = str2double(tokens{2});
            endedOutcome = string(tokens{3});
            endedPressCount = currentPressCount;
            if numel(tokens) >= 4
                endedPressCount = str2double(tokens{4});
            end
            rewardThisInterval = double(endedOutcome == "REWARD");

            intervalNumber(end+1, 1) = endedInterval; %#ok<SAGROW>
            intervalStartWallClock(end+1, 1) = currentIntervalStart;
            intervalEndWallClock(end+1, 1) = nowTime;
            pressCount(end+1, 1) = endedPressCount;
            firstPressWallClock(end+1, 1) = currentFirstPress;
            firstPressLever(end+1, 1) = currentFirstPressLever;
            outcome(end+1, 1) = endedOutcome;
            waterDelivered(end+1, 1) = rewardThisInterval;
            addEvent("INTERVAL_END", endedInterval, ...
                sprintf("outcome=%s; presses=%d", endedOutcome, endedPressCount));

        case {"ACK_OMISSION_CONFIG", "ACK_START_OMISSION", "ACK_STOP"}
            % Handshake acknowledgements are normally consumed by waitForLine.

        otherwise
            fprintf("[Arduino] %s\n", line);
    end
end

%% ===================== CLEAN UP =====================
stopAndClose(vid, vw, s);
disp(['Video saved as ', fileName]);

results = table(intervalNumber, intervalStartWallClock, intervalEndWallClock, ...
    pressCount, firstPressWallClock, firstPressLever, outcome, waterDelivered, ...
    'VariableNames', {'Interval', 'IntervalStart', 'IntervalEnd', 'PressCount', ...
    'FirstPressTime', 'FirstPressLever', 'Outcome', 'WaterDelivered'});
eventLog = table(eventTime, eventType, eventInterval, eventDetail, ...
    'VariableNames', {'Timestamp', 'Event', 'Interval', 'Detail'});

fprintf("\nSession complete: %d intervals, %d water rewards, %d resets.\n", ...
    height(results), sum(results.WaterDelivered), sum(results.Outcome == "RESET"));
disp(results);

if saveResults
    saveTimestamp = datestr(now, 'yyyy-mm-dd_HHMMSS');
    resultsCsvName = sprintf('%s_%s_intervals.csv', outFileBase, saveTimestamp);
    eventsCsvName = sprintf('%s_%s_events.csv', outFileBase, saveTimestamp);
    matName = sprintf('%s_%s.mat', outFileBase, saveTimestamp);
    writetable(results, resultsCsvName);
    writetable(eventLog, eventsCsvName);
    save(matName, 'results', 'eventLog', 'sessionStart', 'sessionEnd', ...
        'sessionDurationMin', 'omissionIntervalSec', 'resetDelaySec', ...
        'interTrialDelaySec', 'debounceMs', 'buzzerFreqHz', ...
        'buzzerDurationMs', 'valvePulseMs', 'fileName');
    fprintf("Results saved to %s, %s, and %s\n", ...
        resultsCsvName, eventsCsvName, matName);
end

%% ===================== LOCAL FUNCTIONS =====================
function found = waitForLine(serialObj, expectedPrefix, timeoutSec)
% Wait up to timeoutSec for a line beginning with expectedPrefix.
    found = false;
    t0 = tic;
    while toc(t0) < timeoutSec
        if serialObj.NumBytesAvailable > 0
            line = strtrim(readline(serialObj));
            if startsWith(line, expectedPrefix)
                found = true;
                return;
            elseif line ~= ""
                fprintf("[Arduino] %s\n", line);
            end
        else
            pause(0.01);
        end
    end
end

function addEvent(type, interval, detail)
% Append an event without losing a timestamp for immediate reset/reward lines.
    eventTime(end+1, 1) = datetime('now'); %#ok<AGROW>
    eventType(end+1, 1) = string(type); %#ok<AGROW>
    eventInterval(end+1, 1) = interval; %#ok<AGROW>
    eventDetail(end+1, 1) = string(detail); %#ok<AGROW>
end

function stopAndClose(videoObj, writerObj, serialObj)
% Preserve the original script's video and serial shutdown sequence.
    try writeline(serialObj, "STOP"); catch, end
    try stop(videoObj); catch, end
    try closepreview(videoObj); catch, end
    try close(writerObj); catch, end
    try delete(videoObj); catch, end
    try delete(serialObj); catch, end
    fprintf("Video and serial resources closed.\n");
end

end
