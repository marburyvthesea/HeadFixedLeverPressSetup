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
% Between-trial presses are ignored. STOP aborts incomplete trials without reward.
% WaterDelivered is a valve command, not measured water flow.
% If manually interrupting, send writeline(s, "STOP") while s is still open.
%
clear; clc;

%% ===================== USER SETTINGS =====================
portName        = "COM4";   % <-- change to your Arduino's port (e.g. "COM3" or "/dev/tty.usbmodem14101")
baudRate        = 115200;   % must match Serial.begin() in the sketch
sessionMinutes  = 30;       % total session duration (minutes)
readyTimeout    = 10;       % seconds to wait for "READY" after connecting
ackTimeout      = 5;        % seconds to wait for ACKs
saveResults     = true;
outFileBase     = "m6880_random_window";
activeLevers    = "LEVER2"; % "BOTH", "LEVER1", or "LEVER2"
trialSeconds    = 15;       % Total trial duration: e.g. 15 or 30 seconds
windowStartRangeSeconds = [1 8];    % Random onset after trial/buzzer onset
activeWindowLengthRangeSeconds = [2 5]; % Random length; [3 3] means fixed 3 seconds
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

s = serialport(portName, baudRate);
configureTerminator(s, "LF");
pause(2);      % let the bootloader / setup()'s delay(1000) finish
flush(s);

fprintf("Connecting to Arduino...\n");

maxRetries = 10;
connected = false;
for attempt = 1:maxRetries
    flush(s);
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
mouseID = 'm6880_random_window';   %mouse ID
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

% ===================== MAIN LOOP =====================
while running
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
    if s.NumBytesAvailable == 0
        pause(0.02);
        continue;
    end

    line = strtrim(readline(s));
    if line == ""
        continue;
    end
    tokens = strsplit(line, ",");

    switch tokens{1}
        case "WINDOW_START"
            if numel(tokens) ~= 5
                warning("Malformed window-start record: %s", char(line));
                if ~stopRequested
                    stopReason = "Malformed window-start record";
                    writeline(s, "STOP");
                    stopRequested = true;
                    stopTimer = tic;
                end
                continue;
            end
            currentTrialStartTime = datetime('now');
            fprintf("[%s] Trial %s started: window %.3f to %.3f s; active=%s.\n", ...
                datestr(currentTrialStartTime,'HH:MM:SS'), tokens{2}, ...
                str2double(tokens{4})/1000, str2double(tokens{5})/1000, sessionActiveLevers);

        case {"WINDOW_END", "WINDOW_ABORT"}
            if numel(tokens) ~= 18
                stopReason = "Malformed window-result record";
                warning("Malformed trial record: %s", char(line));
                if ~stopRequested
                    writeline(s, "STOP");
                    stopRequested = true;
                    stopTimer = tic;
                end
                continue;
            end
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
                    % Read ACK/events here in MAIN LOOP, so no records are discarded.
                end
            end

        case "ACK_CONTINUE"
            % Arduino starts the next trial after reward and configured pause.

        case "ACK_STOP"
            stopAcknowledged = true;
            running = false;

        case {"ERROR", "READY"}
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

fprintf("\nSession complete: %d completed, %d aborted trials in %.2f minutes.\n", ...
    sum(trialCompleted), sum(~trialCompleted), actualMinutes);
disp(results);
%
if saveResults
    timestamp = datestr(now, 'yyyy-mm-dd_HHMMSS');
    sessionOutputBase = string(fullfile(newDirectory, sprintf('%s_%s', outFileBase, timestamp)));
    csvName = sessionOutputBase + ".csv";
    matName = sessionOutputBase + ".mat";
    save(matName, 'results', 'sessionStart', 'sessionEnd', 'sessionMinutes', ...
        'actualSessionEnd', 'actualMinutes', 'stopAcknowledged', 'sessionTaskMode', ...
        'sessionActiveLevers', 'sessionTrialSeconds', 'sessionInterTrialSeconds', ...
        'sessionBuzzerSeconds', 'sessionBuzzerFrequencyHz', 'sessionWindowStartRangeSeconds', ...
        'sessionWindowLengthRangeSeconds', 'sessionRandomSeed', 'sessionMaxWaterDeliveries', 'stopReason');
    writetable(results, csvName);
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
    fprintf(fid, "Reward rule: one 220 ms pulse at completed TRIAL END if QualifyingPresses > 0.\n");
    fprintf(fid, "No extra cue at active-window onset. Counts cover both levers across the full trial.\n");
    fprintf(fid, "WaterDelivered is a valve command, not measured water flow.\n");
    fprintf(fid, "Incomplete trials receive no reward. Between-trial presses are ignored.\n");
    fprintf(fid, "Completed trials: %d\nAborted trials: %d\n", sum(trialCompleted), sum(~trialCompleted));
    fprintf(fid, "Lever 1 presses: %d\nLever 2 presses: %d\nQualifying presses: %d\n", ...
        sum(lever1Presses), sum(lever2Presses), sum(activePresses));
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
        if s.NumBytesAvailable > 0
            line = strtrim(readline(s));
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
