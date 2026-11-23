%% runRandomActiveWindowLeverSession.m
%
% Requires leverTaskControllerWindow.ino (one firmware update).
% Setup sections can be run separately. Run START SESSION once: it includes
% configuration, acquisition, the main loop, cleanup and automatic saving.
%
% Each trial starts with a buzzer. The selected lever earns ONE reward at
% trial end only if freshly pressed within [WindowStartMs, WindowEndMs).
% Window start and length are drawn independently in integer milliseconds.
% Both levers are counted before, during and after the window.
% Holding a lever before the window does not count as a new in-window press.
% Between-trial presses are logged but do not count or earn reward. STOP aborts incomplete trials without reward.
% MAT output also includes pressEvents and pressEventInfo.
% WaterDelivered is a valve command, not measured water flow.
% If manually interrupting, send writeline(s, "STOP") while s is still open.
%
clear; clc;

%% ===================== USER SETTINGS =====================
portName        = "COM4";   % <-- change to your Arduino's port (e.g. "COM3" or "/dev/tty.usbmodem14101")
baudRate        = 115200;   % must match Serial.begin() in the sketch
sessionMinutes  = 40;       % total session duration (minutes)
readyTimeout    = 10;       % seconds to wait for "READY" after connecting
ackTimeout      = 5;        % seconds to wait for ACKs
serialStallGraceSeconds = 10; % Extra time beyond expected trial/pause before STOP+save
saveResults     = true;
outFileBase     = "m4850_VI_80";
activeLevers    = "LEVER2"; % "BOTH", "LEVER1", or "LEVER2"
trialSeconds    = 15;       % Total trial duration: e.g. 15 or 30 seconds % VI 80 
windowStartRangeSeconds = [0 3];    % Random onset after trial/buzzer onset % VI 80 is [0 3], VI 60 is [0 6]
activeWindowLengthRangeSeconds = [12 12]; % Random length; [3 3] means fixed 3 seconds % VI 80 is [12 12], VI 60 is [9 9]
randomSeed = [];                    % [] chooses/saves a new seed; integer repeats a schedule
maxWaterDeliveries = 100;           % Inf disables reward cap; session time limit still applies
interTrialSeconds = 1;      % Pause AFTER reward completion (or an unrewarded trial)
buzzerSeconds   = 0.5;      % Tone at trial start; use trialSeconds for a continuous cue
buzzerFrequencyHz = 4000;

%% ===================== CONNECT =====================
% Clear any stale connection from a previous run (e.g. if the script
% errored out or was stopped before reaching delete(s)) -- a leftover
% handle can leave the port open and cause connection issues.
if exist('s','var')
    try delete(s); catch, end
    clear s
end

s = serialport(portName, baudRate, 'Timeout', ackTimeout);
configureTerminator(s, "LF");
pause(2);      % let the bootloader / setup()'s delay(1000) finish
flush(s);

fprintf("Connecting to Arduino...\n");

maxRetries = 10;
connected = false;
for attempt = 1:maxRetries
    flush(s);
    readAvailableSerialLine(s,true);
    writeline(s, "STOP");
    if waitForLine(s, "ACK_STOP", 1.5)   % short timeout per attempt
        connected = true;
        break;
    end
    fprintf("No response on attempt %d/%d, retrying...\n", attempt, maxRetries);
end

if ~connected
    delete(s);
    error("Could not get ACK_STOP from Arduino after %d attempts. Check port/wiring.", maxRetries);
end

fprintf("Arduino connected; rewards remain disabled until START SESSION.\n");
%% Video Recording Parameters
videoFolder = 'D:\Data\JJM\HeadFixedLeverPress';         %folder for all mouse trial videos
timestamp = datestr(now, 'mm_dd_yy_HH_MM_SS');
mouseID = 'm4850_VI_80';   %mouse ID
mouseID = strcat(mouseID, '_', timestamp);         % Update mouseID with timestamp
newDirectory = fullfile(videoFolder, mouseID);
mkdir(newDirectory);                                %creates new directory with mouseID as folder name
fileName = fullfile(newDirectory, [mouseID '_T.avi']);
%establish video settings
vid = videoinput('winvideo', 1, 'Y800_640x480'); % Change to 'gentl'/'winvideo' if preferred
src = getselectedsource(vid);
src.FrameRate = '120.0005'; % Set the desired frame rate
% log frames directly to disk in the background
vw = VideoWriter(fileName, 'Motion JPEG AVI');
vw.Quality = 75;
vw.FrameRate = 113;
vid.LoggingMode = 'disk';
vid.DiskLogger = vw;
vid.ReturnedColorSpace = 'grayscale' ;
vid.FramesPerTrigger = Inf;

%% ===================== START SESSION =====================
% Everything from here through saving runs as a single MATLAB section.
% Validate and send all settings while Arduino is stopped.
activeLevers = upper(strtrim(string(activeLevers)));
if ~isscalar(activeLevers) || ~ismember(activeLevers, ["BOTH","LEVER1","LEVER2"])
    error('activeLevers must be "BOTH", "LEVER1", or "LEVER2".');
end
validateattributes(sessionMinutes, {'numeric'}, {'scalar','finite','positive'});
validateattributes(serialStallGraceSeconds, {'numeric'}, {'scalar','finite','positive'});
validateattributes(trialSeconds, {'numeric'}, {'scalar','finite','>=',0.001,'<=',3600});
validateattributes(interTrialSeconds, {'numeric'}, {'scalar','finite','>=',0,'<=',3600});
validateattributes(buzzerSeconds, {'numeric'}, {'scalar','finite','>=',0,'<=',trialSeconds});
validateattributes(buzzerFrequencyHz, {'numeric'}, {'scalar','integer','>=',31,'<=',20000});
validateattributes(windowStartRangeSeconds, {'numeric'}, {'row','numel',2,'finite','nonnegative'});
validateattributes(activeWindowLengthRangeSeconds, {'numeric'}, {'row','numel',2,'finite','>=',0.001});
validateattributes(maxWaterDeliveries, {'numeric'}, {'scalar','positive'});
if ~(isinf(maxWaterDeliveries) || maxWaterDeliveries == fix(maxWaterDeliveries))
    error('maxWaterDeliveries must be a positive integer or Inf.');
end
if windowStartRangeSeconds(1) > windowStartRangeSeconds(2) || ...
        activeWindowLengthRangeSeconds(1) > activeWindowLengthRangeSeconds(2)
    error('Window ranges must be [minimum maximum].');
end
windowStartRangeMs = round(1000 * windowStartRangeSeconds);
windowLengthRangeMs = round(1000 * activeWindowLengthRangeSeconds);
if windowStartRangeMs(2) + windowLengthRangeMs(2) > round(1000 * trialSeconds)
    error('Latest window start + longest window length must fit inside trialSeconds.');
end
if isempty(randomSeed)
    previousRng = rng;
    rng('shuffle');
    sessionRandomSeed = randi(2147483646);
    rng(previousRng);
else
    validateattributes(randomSeed, {'numeric'}, {'scalar','integer','>=',1,'<=',2147483646});
    sessionRandomSeed = double(randomSeed);
end
trialSettingsMs = round(1000 * [trialSeconds, interTrialSeconds, buzzerSeconds]);
trialValues = sprintf('%d,%d,%d,%d', trialSettingsMs(1), trialSettingsMs(2), ...
    trialSettingsMs(3), buzzerFrequencyHz);
windowValues = sprintf('%d,%d,%d,%d,%d', windowStartRangeMs(1), windowStartRangeMs(2), ...
    windowLengthRangeMs(1), windowLengthRangeMs(2), sessionRandomSeed);
commands = ["SET_TASK,WINDOW", "SET_ACTIVE," + activeLevers, ...
    "SET_TRIAL," + trialValues, "SET_WINDOW," + windowValues];
expectedAcks = ["ACK_TASK,WINDOW", "ACK_ACTIVE," + activeLevers, ...
    "ACK_TRIAL," + trialValues, "ACK_WINDOW," + windowValues];
commands(end+1) = "SET_PRESS_LOG,ON";
expectedAcks(end+1) = "ACK_PRESS_LOG,ON";
for configIndex = 1:numel(commands)
    writeline(s, commands(configIndex));
    if ~waitForLine(s, expectedAcks(configIndex), ackTimeout)
        error("Configuration not confirmed. Upload leverTaskControllerWindow.ino and rerun CONNECT.");
    end
end
sessionTaskMode = "WINDOW";
sessionActiveLevers = activeLevers;
% Save the acknowledged (millisecond-rounded) settings, not later workspace edits.
sessionTrialSeconds = trialSettingsMs(1) / 1000;
sessionInterTrialSeconds = trialSettingsMs(2) / 1000;
sessionBuzzerSeconds = trialSettingsMs(3) / 1000;
sessionBuzzerFrequencyHz = buzzerFrequencyHz;
sessionWindowStartRangeSeconds = windowStartRangeMs / 1000;
sessionWindowLengthRangeSeconds = windowLengthRangeMs / 1000;
sessionMaxWaterDeliveries = maxWaterDeliveries;
fprintf("WINDOW mode: active=%s; trial=%.3f s; onset=[%.3f %.3f] s; length=[%.3f %.3f] s; seed=%d.\n", ...
    sessionActiveLevers, sessionTrialSeconds, sessionWindowStartRangeSeconds(1), ...
    sessionWindowStartRangeSeconds(2), sessionWindowLengthRangeSeconds(1), ...
    sessionWindowLengthRangeSeconds(2), sessionRandomSeed);

% --- run ---
open(vw);                % open the writer before start
preview(vid);            % non-blocking live view
triggerconfig(vid,'immediate');
start(vid);              % begins acquiring & writing to disk in background
disp('Video recording started.');


readAvailableSerialLine(s,true);
writeline(s, "START");
if ~waitForLine(s, "ACK_START", ackTimeout)
    writeline(s, "STOP");
    waitForLine(s, "ACK_STOP", ackTimeout);
    stop(vid); closepreview(vid);
    try close(vw); catch, end
    delete(vid); delete(s);
    error("Did not receive ACK_START. Session aborted.");
end

sessionStart = datetime('now');
sessionEnd   = sessionStart + minutes(sessionMinutes);
fprintf("Session started at %s. Will run until %s (%.1f min).\n", ...
    datestr(sessionStart), datestr(sessionEnd), sessionMinutes);

% Individual press records, including both levers and intertrial presses.
pressEventData = zeros(0,7);
pressEventPhase = strings(0,1);
pressEventHostTime = datetime.empty(0,1);
pressEventMessagesValid = true;
firstTrialArduinoMs = NaN;

% One row per completed or aborted trial.
trialNum = zeros(0,1);
pressCount = zeros(0,1);
sawPress = zeros(0,1);
waterDelivered = zeros(0,1);
lever1Presses = zeros(0,1);
lever2Presses = zeros(0,1);
activePresses = zeros(0,1);
trialCompleted = false(0,1);
trialStartArduinoMs = zeros(0,1);
trialEndArduinoMs = zeros(0,1);
windowStartMsLog = zeros(0,1);
windowEndMsLog = zeros(0,1);
earlyLever1Presses = zeros(0,1);
earlyLever2Presses = zeros(0,1);
windowLever1Presses = zeros(0,1);
windowLever2Presses = zeros(0,1);
lateLever1Presses = zeros(0,1);
lateLever2Presses = zeros(0,1);
trialStartWallClock = datetime.empty(0,1);
trialEndWallClock   = datetime.empty(0,1);

running = true;
stopRequested = false;
stopAcknowledged = false;
stopTimer = [];
stopReason = "";
currentTrialStartTime = NaT;
trialProgressClock = [];
nextTrialClock = tic; % Waiting for first *_START; ACK_START was already read.
uiClock = tic;
serialDiagnostics = struct('StallDetected',false,'TransportError',"", ...
    'StallReason',"",'MessageAtStall',"",'TransportAtStall',struct(), ...
    'LastMessage',"ACK_START",'LastTransport',struct());

% ===================== MAIN LOOP =====================
try
    while running
        if toc(uiClock)>=0.05
            drawnow limitrate;
            uiClock = tic;
        end
        if ~stopRequested && datetime('now') >= sessionEnd
            stopReason = "Session time limit";
            fprintf("Session time budget reached. Sending STOP.\n");
            writeline(s, "STOP");
            stopRequested = true;
            stopTimer = tic;
        end
        if ~stopRequested && sum(waterDelivered) >= sessionMaxWaterDeliveries
            stopReason = "Water delivery cap";
            fprintf("Water delivery cap reached. Sending STOP.\n");
            writeline(s, "STOP");
            stopRequested = true;
            stopTimer = tic;
        end
        if stopRequested && toc(stopTimer) >= ackTimeout
            warning("STOP acknowledgment timed out; saving received trial records.");
            break;
        end
        [line,transport] = readAvailableSerialLine(s);
        serialDiagnostics.LastTransport = transport;
        if line ~= ""
            serialDiagnostics.LastMessage = line;
            tokens = strsplit(line, ",");

            switch tokens{1}
                case "PRESS_EVENT"
                    try
                        [eventData,eventPhase] = parsePressEvent(line,sessionTaskMode,sessionActiveLevers);
                        pressEventData(end+1,:) = eventData;
                        pressEventPhase(end+1,1) = eventPhase;
                        pressEventHostTime(end+1,1) = datetime('now');
                    catch eventError
                        pressEventMessagesValid = false;
                        warning("Invalid press event: %s (%s)",char(line),eventError.message);
                        if ~stopRequested
                            writeline(s,"STOP"); stopRequested = true; stopTimer = tic;
                        end
                    end

                case "WINDOW_START"
                    if numel(tokens) ~= 5 || any(~isfinite(str2double(tokens(2:5)))) || ...
                            any(str2double(tokens(2:5))<0) || ...
                            any(str2double(tokens(2:5))~=fix(str2double(tokens(2:5)))) || ...
                            str2double(tokens{2})<1 || str2double(tokens{3})>=2^32
                        pressEventMessagesValid = false;
                        warning("Malformed window-start record: %s", char(line));
                        if ~stopRequested
                            stopReason = "Malformed window-start record";
                            writeline(s, "STOP");
                            stopRequested = true;
                            stopTimer = tic;
                        end
                        continue;
                    end
                    if isnan(firstTrialArduinoMs)
                        firstTrialArduinoMs = str2double(tokens{3});
                    end
                    trialProgressClock = tic; nextTrialClock = [];
                    currentTrialStartTime = datetime('now');
                    fprintf("[%s] Trial %s started: window %.3f to %.3f s; active=%s.\n", ...
                        datestr(currentTrialStartTime,'HH:MM:SS'), tokens{2}, ...
                        str2double(tokens{4})/1000, str2double(tokens{5})/1000, sessionActiveLevers);

                case {"WINDOW_END", "WINDOW_ABORT"}
                    if numel(tokens) ~= 18
                        stopReason = "Malformed window-result record";
                        pressEventMessagesValid = false;
                        warning("Malformed trial record: %s", char(line));
                        if ~stopRequested
                            writeline(s, "STOP");
                            stopRequested = true;
                            stopTimer = tic;
                        end
                        continue;
                    end
                    trialProgressClock = [];
                    tNum = str2double(tokens{2});
                    nPress = str2double(tokens{3});
                    saw = str2double(tokens{4}); % At least one selected-lever press INSIDE window
                    water = str2double(tokens{5});
                    completed = string(tokens{1}) == "WINDOW_END";
                    tEnd = datetime('now');

                    trialNum(end+1,1) = tNum;
                    pressCount(end+1,1) = nPress;
                    sawPress(end+1,1) = saw;
                    waterDelivered(end+1,1) = water;
                    lever1Presses(end+1,1) = str2double(tokens{6});
                    lever2Presses(end+1,1) = str2double(tokens{7});
                    activePresses(end+1,1) = str2double(tokens{8});
                    trialStartArduinoMs(end+1,1) = str2double(tokens{9});
                    trialEndArduinoMs(end+1,1) = str2double(tokens{10});
                    windowStartMsLog(end+1,1) = str2double(tokens{11});
                    windowEndMsLog(end+1,1) = str2double(tokens{12});
                    earlyLever1Presses(end+1,1) = str2double(tokens{13});
                    earlyLever2Presses(end+1,1) = str2double(tokens{14});
                    windowLever1Presses(end+1,1) = str2double(tokens{15});
                    windowLever2Presses(end+1,1) = str2double(tokens{16});
                    lateLever1Presses(end+1,1) = str2double(tokens{17});
                    lateLever2Presses(end+1,1) = str2double(tokens{18});
                    trialCompleted(end+1,1) = completed;
                    trialStartWallClock(end+1,1) = currentTrialStartTime;
                    trialEndWallClock(end+1,1) = tEnd;

                    fprintf("Trial %d: lever1=%d, lever2=%d, qualifying=%d, water=%d, completed=%d.\n", ...
                        tNum, lever1Presses(end), lever2Presses(end), activePresses(end), water, completed);
                    currentTrialStartTime = NaT;

                    if completed && ~stopRequested
                        if datetime('now') >= sessionEnd || sum(waterDelivered) >= sessionMaxWaterDeliveries
                            if datetime('now') >= sessionEnd
                                stopReason = "Session time limit";
                            else
                                stopReason = "Water delivery cap";
                            end
                            writeline(s, "STOP");
                            stopRequested = true;
                            stopTimer = tic;
                        else
                            writeline(s, "CONTINUE");
                            nextTrialClock = tic;
                            % Read ACK/events here in MAIN LOOP, so no records are discarded.
                        end
                    end

                case "ACK_CONTINUE"
                    % Arduino starts the next trial after reward and configured pause.

                case "ACK_STOP"
                    stopAcknowledged = true;
                    running = false;

                case {"ERROR", "READY"}
                    serialDiagnostics.TransportError = line;
                    pressEventMessagesValid = false;
                    stopReason = "Arduino error/reset: " + line;
                    warning("Arduino error/reset: %s", char(line));
                    if ~stopRequested
                        writeline(s, "STOP");
                        stopRequested = true;
                        stopTimer = tic;
                    end

                otherwise
                    fprintf("[Arduino] %s\n", line);
            end
        end
        % Process any complete record above before checking progress deadlines.
        % A long press-free trial is normal. Only declare a stall AFTER the
        % expected trial end or expected next onset plus the configured grace.
        lateEnd = ~isempty(trialProgressClock) && ...
            toc(trialProgressClock)>sessionTrialSeconds+serialStallGraceSeconds;
        lateStart = ~isempty(nextTrialClock) && ...
            toc(nextTrialClock)>sessionInterTrialSeconds+0.350+serialStallGraceSeconds;
        if running && ~stopRequested && ~transport.HasCompleteLine && (lateEnd || lateStart)
            serialDiagnostics.StallDetected = true;
            if lateEnd, reason = "Trial-end record overdue";
            else, reason = "Next trial-start record overdue"; end
            serialDiagnostics.StallReason = reason;
            serialDiagnostics.MessageAtStall = serialDiagnostics.LastMessage;
            serialDiagnostics.TransportAtStall = transport;
            stopReason = reason;
            warning('LeverTask:SerialStall', ...
                '%s. Last message: %s; %.1f s since bytes; buffered=%d. Sending STOP and saving.', ...
                reason,serialDiagnostics.LastMessage,transport.SecondsSinceByte,transport.BufferedBytes);
            writeline(s,"STOP"); stopRequested = true; stopTimer = tic;
        end
        if line=="", pause(0.005); end
    end
catch transportError
    serialDiagnostics.TransportError = string(transportError.message);
    stopReason = "Serial/session error: " + serialDiagnostics.TransportError;
    pressEventMessagesValid = false;
    warning('LeverTask:TransportError','%s. Saving received data.',transportError.message);
    try writeline(s,"STOP"); catch, end
end
actualSessionEnd = datetime('now');
actualMinutes = seconds(actualSessionEnd - sessionStart) / 60;
disp('experiment over')
% Close acquisition before saving; a video-cleanup error must not lose data.
try
    stop(vid);
    diskTimer = tic;
    while vid.DiskLoggerFrameCount < vid.FramesAcquired && toc(diskTimer) < 15
        pause(0.05);
    end
    if vid.DiskLoggerFrameCount < vid.FramesAcquired
        warning("Video disk logging did not finish within 15 seconds.");
    end
catch videoError
    warning("Video cleanup issue: %s", videoError.message);
end
try closepreview(vid); catch, end
try close(vw); catch, end
try delete(vid); catch, end
clear vid
disp(['Video saved as ', fileName]);
% ===================== CLEAN UP =====================
try delete(s); catch, end
fprintf("Serial connection closed.\n");

results = table(trialNum, trialStartWallClock, trialEndWallClock, pressCount, ...
    sawPress, waterDelivered, lever1Presses, lever2Presses, activePresses, ...
    trialCompleted, trialStartArduinoMs, trialEndArduinoMs, ...
    'VariableNames', {'Trial','TrialStart','TrialEnd','PressCount','SawQualifyingPress', ...
    'WaterDelivered','Lever1Presses','Lever2Presses','QualifyingPresses','Completed', ...
    'ArduinoStartMs','ArduinoEndMs'});
results.ActiveLevers = repmat(sessionActiveLevers, height(results), 1);
results.WindowStartMs = windowStartMsLog;
results.WindowEndMs = windowEndMsLog;
results.WindowLengthMs = windowEndMsLog - windowStartMsLog;
results.EarlyLever1Presses = earlyLever1Presses;
results.EarlyLever2Presses = earlyLever2Presses;
results.WindowLever1Presses = windowLever1Presses;
results.WindowLever2Presses = windowLever2Presses;
results.LateLever1Presses = lateLever1Presses;
results.LateLever2Presses = lateLever2Presses;
results.NonQualifyingPresses = results.PressCount - results.QualifyingPresses;

[pressEvents,pressEventInfo] = makePressEventTable(pressEventData,pressEventPhase, ...
    pressEventHostTime,firstTrialArduinoMs,trialNum,lever1Presses,lever2Presses, ...
    activePresses,stopAcknowledged,pressEventMessagesValid);

fprintf("\nSession complete: %d completed, %d aborted trials in %.2f minutes.\n", ...
    sum(trialCompleted), sum(~trialCompleted), actualMinutes);
disp(results);
%
if saveResults
    timestamp = datestr(now, 'yyyy-mm-dd_HHMMSS');
    sessionOutputBase = string(fullfile(newDirectory, sprintf('%s_%s', outFileBase, timestamp)));
    csvName = sessionOutputBase + ".csv";
    matName = sessionOutputBase + ".mat";
    save(matName, 'results', 'pressEvents', 'pressEventInfo', 'serialDiagnostics', 'serialStallGraceSeconds', 'sessionStart', 'sessionEnd', 'sessionMinutes', ...
        'actualSessionEnd', 'actualMinutes', 'stopAcknowledged', 'sessionTaskMode', ...
        'sessionActiveLevers', 'sessionTrialSeconds', 'sessionInterTrialSeconds', ...
        'sessionBuzzerSeconds', 'sessionBuzzerFrequencyHz', 'sessionWindowStartRangeSeconds', ...
        'sessionWindowLengthRangeSeconds', 'sessionRandomSeed', 'sessionMaxWaterDeliveries', 'stopReason');
    writetable(results, csvName);
    writetable(pressEvents,sessionOutputBase + "_presses.csv");
    txtName = sessionOutputBase + ".txt";
    fid = fopen(txtName, 'w');
    if fid == -1
        error("Could not write metadata: %s", txtName);
    end
    fprintf(fid, "Task mode: %s\nActive levers: %s\n", sessionTaskMode, sessionActiveLevers);
    fprintf(fid, "Session start: %s\nSession end: %s\n", datestr(sessionStart), datestr(actualSessionEnd));
    fprintf(fid, "Actual session duration: %.2f minutes\n", actualMinutes);
    fprintf(fid, "Trial window: %.3f seconds\n", sessionTrialSeconds);
    fprintf(fid, "Pause after reward completion/unrewarded trial: %.3f seconds\n", sessionInterTrialSeconds);
    fprintf(fid, "Buzzer: %.3f seconds at %d Hz\n", sessionBuzzerSeconds, sessionBuzzerFrequencyHz);
    fprintf(fid, "Window start bounds: [%.3f %.3f] seconds after trial/buzzer onset\n", sessionWindowStartRangeSeconds);
    fprintf(fid, "Window length bounds: [%.3f %.3f] seconds\n", sessionWindowLengthRangeSeconds);
    fprintf(fid, "Randomization: independent uniform integer milliseconds; xorshift32 with rejection sampling\n");
    fprintf(fid, "Random seed: %d\nReward cap: %g\nStop reason: %s\n", ...
        sessionRandomSeed, sessionMaxWaterDeliveries, stopReason);
    fprintf(fid, "Window rule: start inclusive, end exclusive; timing uses debounced press detection.\n");
    fprintf(fid, "Only a new press of the selected lever inside the window qualifies.\n");
    fprintf(fid, "Reward rule: one 350 ms pulse at completed TRIAL END if QualifyingPresses > 0.\n");
    fprintf(fid, "No extra cue at active-window onset. Counts cover both levers across the full trial.\n");
    fprintf(fid, "Individual press log: %d records; in-trial counts match=%d; valid messages=%d.\n", ...
        height(pressEvents),pressEventInfo.TrialCountsMatch,pressEventInfo.MessagesValid);
    fprintf(fid, "Press timing: Arduino debounced detection at 1 ms resolution, 10 ms debounce.\n");
    fprintf(fid, "TrialTimeSeconds is relative to trial/buzzer onset; ITI uses Trial=0 and NaN.\n");
    fprintf(fid, "SessionTimeSeconds is relative to first trial/buzzer onset; HostReceivedAt is receipt time.\n");
    fprintf(fid, "WaterDelivered is a valve command, not measured water flow.\n");
    fprintf(fid, "Incomplete trials receive no reward. Between-trial presses are logged but do not count or earn reward.\n");
    fprintf(fid, "Completed trials: %d\nAborted trials: %d\n", sum(trialCompleted), sum(~trialCompleted));
    fprintf(fid, "Lever 1 presses: %d\nLever 2 presses: %d\nQualifying presses: %d\n", ...
        sum(lever1Presses), sum(lever2Presses), sum(activePresses));
    fprintf(fid, "Serial stall detected: %d\nTransport error: %s\nLast serial message: %s\n", ...
        serialDiagnostics.StallDetected,serialDiagnostics.TransportError,serialDiagnostics.LastMessage);
    fprintf(fid, "Reward commands: %d\nSTOP acknowledged: %d\n", sum(waterDelivered), stopAcknowledged);
    fclose(fid);
    % Text/MAT/CSV are already saved even if optional Excel export fails.
    summaryTable = table(["Completed trials"; "Aborted trials"; "Reward commands"; ...
        "Lever 1 presses"; "Lever 2 presses"; "Qualifying presses"; "Nonqualifying presses"], ...
        [sum(trialCompleted); sum(~trialCompleted); sum(waterDelivered); ...
        sum(lever1Presses); sum(lever2Presses); sum(activePresses); ...
        sum(results.NonQualifyingPresses)], 'VariableNames', {'Metric','Value'});
    try
        writetable(results, sessionOutputBase + ".xlsx", 'Sheet', 'Trials');
        writetable(summaryTable, sessionOutputBase + ".xlsx", 'Sheet', 'Summary');
    catch exportError
        warning("Excel export failed; MAT, CSV and TXT are saved: %s", exportError.message);
    end
    fprintf("Session data and metadata saved in %s\n", newDirectory);
end

% %% ===================== PLOT =====================
% figure;
% bar(results.Trial, results.PressCount);
% xlabel('Trial number');
% ylabel('Lever presses');
% title(sprintf('Lever presses per trial (%.0f min session)', sessionMinutes));
% grid on;

%% ===================== HELPER FUNCTION =====================
function found = waitForLine(s, expectedPrefix, timeoutSec)
% Waits for an exact acknowledgment; used only during setup.
% Trial events are read in MAIN LOOP.
    found = false;
    t0 = tic;
    while toc(t0) < timeoutSec
        line = readAvailableSerialLine(s);
        if line ~= ""
            if line == expectedPrefix
                found = true;
                return;
            elseif line ~= ""
                fprintf("[Arduino] %s\n", line);
            end
        else
            pause(0.02);
        end
    end
end


function [data,phase] = parsePressEvent(line,taskMode,activeLevers)
% PRESS_EVENT,trial,lever,ArduinoMs,elapsedMs,inTrial,selected,qualifying,phase
    tokens = split(string(line),",");
    assert(numel(tokens)==9 && tokens(1)=="PRESS_EVENT",'Expected nine press-event fields.');
    data = str2double(tokens(2:8))';
    assert(all(isfinite(data)) && all(data==fix(data)),'Press-event values must be finite integers.');
    assert(data(1)>=0 && ismember(data(2),[1 2]) && data(3)>=0 && data(3)<2^32, ...
        'Invalid trial, lever or Arduino clock.');
    assert(all(ismember(data(5:7),[0 1])),'Invalid press-event flags.');
    phase = tokens(9);
    selected = activeLevers=="BOTH" || activeLevers=="LEVER"+string(data(2));
    assert(data(6)==selected,'Press-event lever selection disagrees with session settings.');
    if data(5)==0
        assert(data(1)==0 && data(4)==-1 && data(7)==0 && phase=="INTERTRIAL", ...
            'Invalid intertrial event.');
    else
        assert(data(1)>=1 && data(4)>=0 && data(4)<2^32,'Invalid trial-relative timing.');
        if taskMode=="TRIAL"
            assert(phase=="TRIAL",'Invalid full-trial phase.');
        else
            assert(taskMode=="WINDOW" && ismember(phase,["EARLY","WINDOW","LATE"]), ...
                'Invalid random-window phase.');
        end
        qualifying = selected && ismember(phase,["TRIAL","WINDOW"]);
        assert(data(7)==qualifying,'Invalid qualifying-press flag.');
    end
end

function [events,info] = makePressEventTable(data,phase,hostTime,firstMs, ...
        trials,lever1Counts,lever2Counts,qualifyingCounts,acknowledged,messagesValid)
% Keep raw Arduino clock values; derive relative seconds without host latency.
    trialTime = data(:,4)/1000;
    trialTime(data(:,5)==0) = NaN;
    if isfinite(firstMs)
        sessionTime = mod(data(:,3)-firstMs,2^32)/1000;
    else
        sessionTime = NaN(size(data,1),1);
    end
    events = table(data(:,1),data(:,2),data(:,3),data(:,4),trialTime,sessionTime, ...
        logical(data(:,5)),logical(data(:,6)),logical(data(:,7)),phase,hostTime, ...
        'VariableNames',{'Trial','LeverID','ArduinoMs','TrialElapsedMs', ...
        'TrialTimeSeconds','SessionTimeSeconds','InTrial','SelectedLever', ...
        'Qualifying','Phase','HostReceivedAt'});
    % Summaries contain only in-trial counts, so ITI completeness cannot be
    % inferred from this check. These flags expose malformed/missing trial data.
    countsMatch = all(ismember(events.Trial(events.InTrial),trials));
    for k = 1:numel(trials)
        selectedRows = events.InTrial & events.Trial==trials(k);
        countsMatch = countsMatch && sum(selectedRows & events.LeverID==1)==lever1Counts(k) && ...
            sum(selectedRows & events.LeverID==2)==lever2Counts(k) && ...
            sum(selectedRows & events.Qualifying)==qualifyingCounts(k);
    end
    info = struct('ProtocolVersion',1,'Enabled',true,'MessagesValid',messagesValid, ...
        'TrialCountsMatch',countsMatch,'StopAcknowledged',acknowledged, ...
        'ClockOriginValid',isfinite(firstMs),'SessionStartArduinoMs',firstMs,'TimestampResolutionMs',1,'DebounceMs',10, ...
        'SessionClockOrigin',"First trial/buzzer onset", ...
        'IntertrialConvention',"Trial=0, TrialElapsedMs=-1, TrialTimeSeconds=NaN", ...
        'TimingMeaning',"Arduino time of a new debounced press; HostReceivedAt is receipt time only");
    if ~countsMatch || ~messagesValid || ~acknowledged || ~isfinite(firstMs)
        warning('PressTiming:IncompleteLog', ...
            'Press-event verification incomplete. Inspect pressEventInfo and the trial summaries.');
    end
end

function [line,diagnostics] = readAvailableSerialLine(s,reset)
% Never wait for a terminator. Retain fragments and any complete queued lines.
% This function is shared by setup handshakes and the session loop, so bytes
% received behind ACK_START remain available even when the device buffer is empty.
    persistent rx byteCount lineCount lastByteClock lastLineClock highWater
    if nargin<2, reset = false; end
    if reset || isempty(byteCount)
        rx = ''; byteCount = 0; lineCount = 0; highWater = 0;
        lastByteClock = tic; lastLineClock = tic;
    end
    line = "";
    available = s.NumBytesAvailable;
    if available>0
        bytes = read(s,available,'uint8');
        rx = [rx char(bytes(:)')];
        byteCount = byteCount + numel(bytes); lastByteClock = tic;
    end
    highWater = max(highWater,numel(rx));
    assert(numel(rx)<=1048576,'Serial receive backlog exceeds 1 MiB.');
    pos = find(rx==char(10),1);
    if ~isempty(pos)
        line = string(strtrim(rx(1:pos-1))); rx = rx(pos+1:end);
        lineCount = lineCount + 1; lastLineClock = tic;
    end
    diagnostics = struct('BytesReceived',byteCount,'LinesReceived',lineCount, ...
        'BufferedBytes',numel(rx),'MaxBufferedBytes',highWater,'HasCompleteLine',any(rx==char(10)), ...
        'SecondsSinceByte',toc(lastByteClock),'SecondsSinceLine',toc(lastLineClock));
end
