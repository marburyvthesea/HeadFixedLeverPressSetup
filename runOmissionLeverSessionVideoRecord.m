%% runOmissionLeverSessionVideoRecord.m
% Omission training with video recording.
%
% REVERSED CONTINGENCY
% A water reward is delivered only after the mouse completes an entire
% omissionIntervalSec without a lever press.  A press immediately cancels
% that interval, delivers NO water, waits resetDelaySec, then starts a new
% cued interval.  Thus pressing delays (rather than earns) water.
%
% Requires the unified leverTaskControllerWindow.ino firmware, which remains
% backward-compatible with its IMMEDIATE, TRIAL, and WINDOW modes. This
% script selects its additional OMISSION mode before starting the session.
%
% Required Arduino serial contract (one LF-terminated line per message):
%   MATLAB -> SET_TASK,OMISSION
%   MATLAB -> SET_ACTIVE,BOTH|LEVER1|LEVER2
%   MATLAB -> SET_OMISSION,<intervalMs>,<resetMs>,<itiMs>,<buzzerMs>,<Hz>
%   Arduino -> matching ACK_TASK, ACK_ACTIVE, and ACK_OMISSION lines
%   MATLAB -> START
%   Arduino -> ACK_START
%   Arduino -> INTERVAL_START,<intervalNumber>,<ArduinoMs>
%   Arduino -> LEVER_PRESS,<intervalNumber>,<leverNumber>,<ArduinoMs>
%   Arduino -> INTERVAL_RESET,<intervalNumber>
%   Arduino -> REWARD,<intervalNumber>
%   Arduino -> INTERVAL_END,<intervalNumber>,REWARD|RESET,<pressCount>
%   MATLAB -> STOP
%   Arduino -> ACK_STOP
%
% The Arduino handles timing and the reversed contingency locally, including
% immediate cancellation, valve timing, and automatic post-reset/reward gaps.

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
ackTimeout         = 5;       % Seconds to wait for command acknowledgements.
saveResults        = true;
outFileBase        = "m0710_02_omission_lever_session";
activeLevers       = "BOTH";  % "BOTH", "LEVER1", or "LEVER2" reset the timer.

% Hardware constants documented here must match the unified Arduino firmware.
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
maxRetries = 10;
connected = false;
for attempt = 1:maxRetries
    flush(s);
    writeline(s, "STOP");       % Verify connection with rewards disabled.
    if waitForLine(s, "ACK_STOP", 1.5)
        connected = true;
        break;
    end
    fprintf("No response on attempt %d/%d, retrying...\n", attempt, maxRetries);
end
if ~connected
    delete(s);
    error("Could not get ACK_STOP from Arduino after %d attempts. Check port/wiring.", maxRetries);
end
fprintf("Arduino connected; rewards remain disabled until START.\n");

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
% Validate and configure the unified firmware while it is stopped.
activeLevers = upper(strtrim(string(activeLevers)));
if ~isscalar(activeLevers) || ~ismember(activeLevers, ["BOTH","LEVER1","LEVER2"])
    error('activeLevers must be "BOTH", "LEVER1", or "LEVER2".');
end
validateattributes(sessionDurationMin, {'numeric'}, {'scalar','finite','positive'});
validateattributes(omissionIntervalSec, {'numeric'}, {'scalar','finite','>=',0.001,'<=',3600});
validateattributes(resetDelaySec, {'numeric'}, {'scalar','finite','>=',0,'<=',3600});
validateattributes(interTrialDelaySec, {'numeric'}, {'scalar','finite','>=',0,'<=',3600});
validateattributes(buzzerDurationMs, {'numeric'}, {'scalar','integer','>=',0});
validateattributes(buzzerFreqHz, {'numeric'}, {'scalar','integer','>=',31,'<=',20000});
if buzzerDurationMs > round(1000 * omissionIntervalSec)
    error('buzzerDurationMs cannot exceed the omission interval.');
end
omissionSettingsMs = [round(1000 * omissionIntervalSec), ...
    round(1000 * resetDelaySec), round(1000 * interTrialDelaySec)];
omissionValues = sprintf('%d,%d,%d,%d,%d', omissionSettingsMs(1), ...
    omissionSettingsMs(2), omissionSettingsMs(3), buzzerDurationMs, buzzerFreqHz);
commands = ["SET_TASK,OMISSION", "SET_ACTIVE," + activeLevers, ...
    "SET_OMISSION," + omissionValues];
expectedAcks = ["ACK_TASK,OMISSION", "ACK_ACTIVE," + activeLevers, ...
    "ACK_OMISSION," + omissionValues];
for configIndex = 1:numel(commands)
    writeline(s, commands(configIndex));
    if ~waitForLine(s, expectedAcks(configIndex), ackTimeout)
        delete(s);
        error("Configuration not confirmed. Upload the updated leverTaskControllerWindow.ino.");
    end
end
sessionActiveLevers = activeLevers;
omissionIntervalSec = omissionSettingsMs(1) / 1000;
resetDelaySec = omissionSettingsMs(2) / 1000;
interTrialDelaySec = omissionSettingsMs(3) / 1000;

open(vw);
preview(vid);
triggerconfig(vid, 'immediate');
start(vid);
disp('Video recording started.');

writeline(s, "START");
if ~waitForLine(s, "ACK_START", ackTimeout)
    stopAndClose(vid, vw, s, true);
    error("Arduino did not acknowledge START.");
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
stopRequested = false;
stopAcknowledged = false;
stopTimer = [];

%% ===================== MAIN LOOP =====================
while running
    if ~stopRequested && datetime('now') >= sessionEnd
        fprintf("Session time budget reached. Sending STOP.\n");
        writeline(s, "STOP");
        stopRequested = true;
        stopTimer = tic;
    end
    if stopRequested && toc(stopTimer) >= ackTimeout
        warning("STOP acknowledgment timed out; saving received interval records.");
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

        case "ACK_STOP"
            stopAcknowledged = true;
            running = false;

        case {"ACK_TASK", "ACK_ACTIVE", "ACK_OMISSION", "ACK_START"}
            % Handshake acknowledgements are normally consumed by waitForLine.

        otherwise
            fprintf("[Arduino] %s\n", line);
    end
end

%% ===================== CLEAN UP =====================
stopAndClose(vid, vw, s, ~stopAcknowledged);
disp(['Video saved as ', fileName]);

results = table(intervalNumber, intervalStartWallClock, intervalEndWallClock, ...
    pressCount, firstPressWallClock, firstPressLever, outcome, waterDelivered, ...
    'VariableNames', {'Interval', 'IntervalStart', 'IntervalEnd', 'PressCount', ...
    'FirstPressTime', 'FirstPressLever', 'Outcome', 'WaterDelivered'});
results.ActiveLevers = repmat(sessionActiveLevers, height(results), 1);
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
        'buzzerDurationMs', 'valvePulseMs', 'sessionActiveLevers', ...
        'stopAcknowledged', 'fileName');
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

function stopAndClose(videoObj, writerObj, serialObj, sendStop)
% Preserve the original script's video and serial shutdown sequence.
    if sendStop
        try writeline(serialObj, "STOP"); catch, end
    end
    try stop(videoObj); catch, end
    try closepreview(videoObj); catch, end
    try close(writerObj); catch, end
    try delete(videoObj); catch, end
    try delete(serialObj); catch, end
    fprintf("Video and serial resources closed.\n");
end

end
