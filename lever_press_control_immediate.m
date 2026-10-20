function results = lever_press_control_immediate()
% Run with deliverWaterImmediatelyAfterEveryPress.ino.
% PRESS fields: number, lever ID, valve-on command, detection millis.
% ArduinoMs is time since Arduino boot, not time since session start.
% WallClock is MATLAB receipt time, not the physical press time.

%% USER SETTINGS
portName = "COM4";
baudRate = 115200;
sessionMinutes = 1;
maxPresses = 100; % Host limit; a strict reward cap requires Arduino support.
ackTimeout = 5;
saveResults = true;
outFileBase = 'hardwareTest';

recordVideo = true;
videoFolder = 'D:\Data\JJM\HeadFixedLeverPress';
mouseID = '10202026_test';
cameraDeviceID = 2;
cameraFormat = 'Y800_640x480';
cameraFrameRate = '120.0005';
videoPlaybackFrameRate = 113; % Preserved from original; affects playback speed.

%% INITIALIZE
s = [];
vid = [];
vw = [];
writerOpened = false;
sessionMayBeActive = false;
stopAcknowledged = false;
sessionStart = NaT;
sessionEnd = NaT;
sessionTimer = [];
actualDurationSeconds = 0;
stopReason = "";
failureMessage = "";
pressNum = zeros(0,1);
leverUsed = zeros(0,1);
waterDelivered = zeros(0,1);
pressTimeArduinoMs = zeros(0,1);
pressWallClock = datetime.empty(0,1);
results = table();
cleanupGuard = onCleanup(@cleanupResources); %#ok<NASGU>

timestamp = datestr(now, 'yyyy-mm-dd_HHMMSS');
newDirectory = fullfile(videoFolder, [mouseID '_' timestamp]);
[ok, message] = mkdir(newDirectory);
if ~ok
    error("Could not create output folder: %s", message);
end
fileName = fullfile(newDirectory, [mouseID '_' timestamp '_T.avi']);

try
    %% CONNECT AND ENSURE THE VALVE SESSION IS STOPPED
    s = serialport(portName, baudRate, 'Timeout', 1);
    configureTerminator(s, "LF");
    pause(2); % Allow reset on serial connection.
    flush(s);
    writeline(s, "STOP");
    awaitAck("ACK_STOP", ackTimeout);

    %% PREPARE AND START VIDEO BEFORE ENABLING REWARDS
    if recordVideo
        vid = videoinput('winvideo', cameraDeviceID, cameraFormat);
        src = getselectedsource(vid);
        src.FrameRate = cameraFrameRate;
        vw = VideoWriter(fileName, 'Motion JPEG AVI');
        vw.Quality = 75;
        vw.FrameRate = videoPlaybackFrameRate;
        vid.LoggingMode = 'disk';
        vid.DiskLogger = vw;
        vid.ReturnedColorSpace = 'grayscale';
        vid.FramesPerTrigger = Inf;
        triggerconfig(vid, 'immediate');
        open(vw);
        writerOpened = true;
        preview(vid);
        start(vid);
        fprintf("Video recording started.\n");
    end

    %% START ONCE
    sessionStart = datetime('now');
    sessionTimer = tic;
    sessionMayBeActive = true; % Also clean up if ACK is lost.
    writeline(s, "START");
    awaitAck("ACK_START", ackTimeout);
    fprintf("Session started: %.1f minutes or %d recorded rewards.\n", ...
        sessionMinutes, maxPresses);

    %% READ EVENTS, INCLUDING EVENTS ARRIVING WHILE STOPPING
    stopRequested = false;
    stopTimer = [];
    while true
        if ~stopRequested
            if toc(sessionTimer) >= sessionMinutes * 60
                stopReason = "Duration reached";
            elseif numel(pressNum) >= maxPresses
                stopReason = "Press limit reached";
            end
            if stopReason ~= ""
                writeline(s, "STOP");
                stopRequested = true;
                stopTimer = tic;
                fprintf("%s. Waiting for the active reward to finish...\n", ...
                    stopReason);
            end
        end

        if stopRequested && toc(stopTimer) >= ackTimeout
            error("Timed out waiting for ACK_STOP.");
        end
        if s.NumBytesAvailable == 0
            pause(0.005);
            continue;
        end

        line = strtrim(readline(s));
        if startsWith(line, "PRESS,")
            recordPress(line);
        elseif line == "ACK_STOP"
            stopAcknowledged = true;
            sessionMayBeActive = false;
            if ~stopRequested
                stopReason = "Arduino stopped";
            end
            break;
        elseif startsWith(line, "ERROR,")
            warning("Arduino: %s", char(line));
            failureMessage = line;
            if ~stopRequested
                stopReason = "Arduino error";
                writeline(s, "STOP");
                stopRequested = true;
                stopTimer = tic;
            end
        elseif line == "READY"
            error("Arduino reset during the session.");
        elseif line ~= ""
            fprintf("[Arduino] %s\n", line);
        end
    end
catch ME
    failureMessage = string(ME.message);
    stopReason = "Error";
    warning("Session interrupted: %s", ME.message);
end

% On errors, this attempts STOP and collects pending PRESS records.
% onCleanup also invokes it if execution is interrupted by Ctrl+C.
cleanupResources();
sessionEnd = datetime('now');
if ~isempty(sessionTimer)
    actualDurationSeconds = toc(sessionTimer);
end

%% SAVE PRESS DATA AND SUMMARY
results = table(pressNum, pressWallClock, pressTimeArduinoMs, ...
    waterDelivered, leverUsed, 'VariableNames', ...
    {'PressNum','WallClock','ArduinoMs','WaterDelivered','LeverUsed'});

lever1Presses = sum(leverUsed == 1);
lever2Presses = sum(leverUsed == 2);
lever1Water = sum(leverUsed == 1 & waterDelivered == 1);
lever2Water = sum(leverUsed == 2 & waterDelivered == 1);
summaryTable = table(["Lever 1"; "Lever 2"; "Total"], ...
    [lever1Presses; lever2Presses; height(results)], ...
    [lever1Water; lever2Water; sum(waterDelivered == 1)], ...
    'VariableNames', {'Lever','PressCount','WaterDeliveredCount'});

if saveResults
    outputBase = fullfile(newDirectory, [outFileBase '_' timestamp]);
    % Save MAT first, so data survives a later export failure.
    save([outputBase '.mat'], 'results', 'summaryTable', 'sessionStart', ...
        'sessionEnd', 'actualDurationSeconds', 'sessionMinutes', ...
        'maxPresses', 'stopReason', 'failureMessage', 'stopAcknowledged', ...
        'cameraFrameRate', 'videoPlaybackFrameRate');
    writetable(results, [outputBase '.csv']);
    writetable(results, [outputBase '.xlsx'], 'Sheet', 'RawPresses');
    writetable(summaryTable, [outputBase '.xlsx'], 'Sheet', 'Summary');
    writetable(summaryTable, [outputBase '_summary.txt'], 'Delimiter', '\t');
    fprintf("Results saved in: %s\n", newDirectory);
end

fprintf("Session finished: %d recorded rewards in %.2f minutes.\n", ...
    height(results), actualDurationSeconds / 60);
disp(summaryTable);
if failureMessage ~= ""
    fprintf("Session issue: %s\n", failureMessage);
end
if recordVideo && writerOpened
    fprintf("Video path: %s\n", fileName);
end

%% NESTED HELPERS
    function recordPress(line)
        tokens = split(line, ",");
        if numel(tokens) ~= 5
            error("Malformed PRESS message: %s", line);
        end
        values = str2double(tokens(2:5));
        if any(~isfinite(values)) || any(values ~= fix(values)) || ...
                values(1) < 1 || ~ismember(values(2), [1 2]) || ...
                ~ismember(values(3), [0 1]) || values(4) < 0
            error("Invalid PRESS values: %s", line);
        end
        if ~isempty(pressNum) && values(1) <= pressNum(end)
            error("Press numbers repeated or reset: %s", line);
        end
        if values(1) ~= numel(pressNum) + 1
            warning("Gap in press numbering: received %d.", values(1));
        end
        pressNum(end+1,1) = values(1);
        leverUsed(end+1,1) = values(2);
        waterDelivered(end+1,1) = values(3);
        pressTimeArduinoMs(end+1,1) = values(4);
        pressWallClock(end+1,1) = datetime('now');
        fprintf("Press %d | Lever %d | Valve command %d | Arduino %d ms\n", ...
            values(1), values(2), values(3), values(4));
    end

    function awaitAck(expected, timeoutSeconds)
        timer = tic;
        while toc(timer) < timeoutSeconds
            if s.NumBytesAvailable == 0
                pause(0.005);
                continue;
            end
            line = strtrim(readline(s));
            if line == expected
                return;
            elseif startsWith(line, "PRESS,") && sessionMayBeActive
                recordPress(line);
            elseif startsWith(line, "ERROR,")
                error("Arduino reported: %s", line);
            elseif line ~= ""
                fprintf("[Arduino] %s\n", line);
            end
        end
        error("Arduino did not send %s within %.1f seconds.", ...
            expected, timeoutSeconds);
    end

    function cleanupResources()
        if ~isempty(s) && sessionMayBeActive
            try
                writeline(s, "STOP");
                awaitAck("ACK_STOP", ackTimeout);
                stopAcknowledged = true;
            catch cleanupError
                warning("Could not confirm Arduino STOP: %s", cleanupError.message);
            end
            sessionMayBeActive = false;
        end
        if ~isempty(vid)
            try
                stop(vid);
                % Disk logging can finish after acquisition stops.
                diskTimer = tic;
                while vid.DiskLoggerFrameCount < vid.FramesAcquired && toc(diskTimer) < 15
                    pause(0.05);
                end
                if vid.DiskLoggerFrameCount < vid.FramesAcquired
                    warning("Video disk logging did not finish within 15 seconds.");
                end
            catch videoError
                warning("Video cleanup: %s", videoError.message);
            end
            try closepreview(vid); catch, end
            try delete(vid); catch, end
            vid = [];
        end
        if ~isempty(vw) && writerOpened
            try close(vw); catch, end
            vw = [];
        end
        if ~isempty(s)
            try delete(s); catch, end
            s = [];
        end
    end
end
