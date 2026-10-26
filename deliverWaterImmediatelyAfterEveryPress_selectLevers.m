% %% check
% ver
%% deliverWaterImmediatelyAfterEveryPress_selectLevers.m
%
% Controls deliverWaterSelectableLevers.ino over serial.
% Run sections in order. CONNECT leaves rewards disabled; START SESSION
% enables rewards. Run MAIN LOOP promptly after START SESSION.
%
% Protocol implemented (must match the Arduino sketch):
%   Arduino -> "READY"                          on boot
%   MATLAB  -> "SET_ACTIVE,BOTH" / "SET_ACTIVE,LEVER1" / "SET_ACTIVE,LEVER2"
%   Arduino -> "ACK_ACTIVE,<mode>"              confirms session mode
%   MATLAB  -> "START"                          begin the session
%   Arduino -> "ACK_START"
%   Arduino -> "PRESS,<pressNum>,<leverId>,<water>,<ArduinoMs>"
%              ArduinoMs is press-detection time since boot.
%              water=1 means valve commanded on; water=0 means no command
%              (inactive lever, or a pending reward canceled by STOP).
%   MATLAB  -> "STOP"                           end the session
%   Arduino -> "ACK_STOP"
%
% Stop after sessionMinutes or maxPresses recorded presses, either lever.
% The host press limit is not an exact hardware cutoff.
% If manually interrupting MAIN LOOP, send writeline(s, "STOP").
%


clear; clc;

%% ===================== USER SETTINGS =====================
portName        = "COM4";   % <-- change to your Arduino's port (e.g. "COM3" or "/dev/tty.usbmodem14101")
baudRate        = 115200;   % must match Serial.begin() in the sketch
sessionMinutes  = 30;       % total session duration (minutes)
readyTimeout    = 10;       % seconds to wait for "READY" after connecting
ackTimeout      = 5;        % seconds to wait for ACKs
saveResults     = true;
outFileBase     = "m7287";
maxPresses = 100; % Counts active-lever presses only.
activeLevers = "LEVER1"; % "BOTH", "LEVER1 - closer to computer", or "LEVER2"; choose before START SESSION.

%% Video Recording Parameters
videoFolder = 'D:\Data\JJM\HeadFixedLeverPress';         %folder for all mouse trial videos
timestamp = datestr(now, 'mm_dd_yy_HH_MM_SS'); 
mouseID = 'm7287';   %mouse ID
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
    writeline(s, "STOP"); % Verify connection without enabling rewards.
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

fprintf("Arduino connected; rewards disabled until START SESSION.\n");

%% ===================== START SESSION =====================
% Set mode while stopped. Acknowledgment is required before recording/start.
activeLevers = upper(strtrim(string(activeLevers)));
if ~isscalar(activeLevers) || ~ismember(activeLevers, ["BOTH", "LEVER1", "LEVER2"])
    error('activeLevers must be "BOTH", "LEVER1", or "LEVER2".');
end
writeline(s, "SET_ACTIVE," + activeLevers);
if ~waitForLine(s, "ACK_ACTIVE," + activeLevers, ackTimeout)
    error("Mode not confirmed. Upload deliverWaterSelectableLevers.ino and rerun CONNECT.");
end
sessionActiveLevers = activeLevers; % Snapshot the acknowledged mode for metadata.
fprintf("Active levers for this session: %s\n", sessionActiveLevers);

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
    stop(vid);
    closepreview(vid);
    try close(vw); catch, end
    delete(vid);
    delete(s);
    error("Did not receive ACK_START. Session aborted.");
end

sessionStart = datetime('now');
sessionEnd   = sessionStart + minutes(sessionMinutes);
fprintf("Session started at %s. Will run until %s (%.1f min).\n", ...
    datestr(sessionStart), datestr(sessionEnd), sessionMinutes);

% ===================== MAIN LOOP =====================
% One row per logged press, rewarded or not. Run once per session.
pressNum = zeros(0,1);
leverUsed = zeros(0,1);
waterDelivered = zeros(0,1);
pressTimeArduinoMs = zeros(0,1);
pressWallClock = datetime.empty(0,1);

running = true;
stopRequested = false;
stopAcknowledged = false;
stopTimer = [];
while running
    if ~stopRequested && datetime('now') >= sessionEnd
        fprintf("Session time budget reached. Sending STOP.\n");
        writeline(s, "STOP");
        stopRequested = true;
        stopTimer = tic;
    end

    switch sessionActiveLevers
        case "LEVER1"
            activePressCount = sum(leverUsed == 1);
        case "LEVER2"
            activePressCount = sum(leverUsed == 2);
        case "BOTH"
            activePressCount = numel(leverUsed);
    end

    if ~stopRequested && activePressCount >= maxPresses
        fprintf("Active-lever press limit (%d) reached. Sending STOP.\n", maxPresses);
        writeline(s, "STOP");
        stopRequested = true;
        stopTimer = tic;
    end

    % Keep parsing PRESS records until the active pulse finishes.
    if stopRequested && toc(stopTimer) >= ackTimeout
        warning("Did not receive ACK_STOP. Check Arduino; saving received data.");
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
        case "PRESS"
            if numel(tokens) ~= 5
                warning("Ignoring malformed PRESS message: %s", char(line));
                continue;
            end
            pNum      = str2double(tokens{2});
            leverId   = str2double(tokens{3});   % NEW
            water     = str2double(tokens{4});   % shifted from {3}
            msTime    = str2double(tokens{5});   % shifted from {4}
            wallClock = datetime('now');
        
            pressNum(end+1,1)           = pNum;
            leverUsed(end+1,1)          = leverId;   % NEW accumulator
            waterDelivered(end+1,1)     = water;
            pressTimeArduinoMs(end+1,1) = msTime;
            pressWallClock(end+1,1)     = wallClock;
        
            fprintf("[%s] Press %d (lever %d) -> water=%d\n", ...
                datestr(wallClock,'HH:MM:SS'), pNum, leverId, water);

        case "ACK_STOP"
            stopAcknowledged = true;
            running = false;

        case {"ERROR", "READY"}
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
results = table(pressNum, pressWallClock, pressTimeArduinoMs, waterDelivered, leverUsed, ...
    'VariableNames', {'PressNum','WallClock','ArduinoMs','WaterDelivered', 'LeverUsed'});
results.ActiveLevers = repmat(sessionActiveLevers, height(results), 1);
disp('experiment over')

%
stop(vid);               % stops acquisition; file is already on disk
closepreview(vid);
try
    close(vw);           % close writer explicitly (good hygiene)
catch
end
delete(vid);
clear vid
disp(['Video saved as ', fileName]);
% ===================== CLEAN UP =====================
delete(s);
fprintf("Serial connection closed.\n");

% results = table(trialNum, trialStartWallClock, trialEndWallClock, pressCount, sawPress, waterDelivered, ...
%    'VariableNames', {'Trial','TrialStart','TrialEnd','PressCount','SawPress', 'WaterDelivered'});

fprintf("\nSession complete: %d recorded presses in %.2f minutes; mode=%s.\n", ...
    height(results), actualMinutes, sessionActiveLevers);
disp(results);
% ===================== SAVE RESULTS =====================
if saveResults
    timestamp = datestr(now, 'yyyy-mm-dd_HHMMSS');
    % Save all session data and metadata alongside the video.
    sessionOutputBase = fullfile(newDirectory, sprintf('%s_%s', outFileBase, timestamp));

    % Existing CSV/MAT exports
    csvName = sessionOutputBase + ".csv";
    matName = sessionOutputBase + ".mat";
    writetable(results, csvName);
    save(matName, 'results', 'sessionStart', 'sessionEnd', 'sessionMinutes', ...
        'actualSessionEnd', 'actualMinutes', 'stopAcknowledged', 'sessionActiveLevers');
    fprintf("Results saved to %s and %s\n", csvName, matName);

    % ---- Lever press summary stats ----
    totalPresses = height(results);
    if totalPresses > 0
        lever1Presses = sum(results.LeverUsed == 1);
        lever2Presses = sum(results.LeverUsed == 2);
        lever1Water   = sum(results.LeverUsed == 1 & results.WaterDelivered == 1);
        lever2Water   = sum(results.LeverUsed == 2 & results.WaterDelivered == 1);
        totalWater    = sum(results.WaterDelivered == 1);
    else
        lever1Presses = 0; lever2Presses = 0;
        lever1Water = 0; lever2Water = 0; totalWater = 0;
    end

    % ---- Excel export: raw data + summary on separate sheets ----
    xlsxName = sessionOutputBase + ".xlsx";
    writetable(results, xlsxName, 'Sheet', 'RawPresses');

    summaryTable = table( ...
        ["Lever 1"; "Lever 2"; "Total"], ...
        [lever1Presses; lever2Presses; totalPresses], ...
        [lever1Water; lever2Water; totalWater], ...
        'VariableNames', {'Lever','PressCount','WaterDeliveredCount'});
    writetable(summaryTable, xlsxName, 'Sheet', 'Summary');
    fprintf("Excel file saved to %s\n", xlsxName);

    % ---- Plain text summary ----
    txtName = sessionOutputBase + ".txt";
    fid = fopen(txtName, 'w');
    if fid == -1
        error("Could not open session metadata file: %s", txtName);
    end
    fprintf(fid, "Lever Press Session Summary\n");
    fprintf(fid, "============================\n");
    fprintf(fid, "Session start: %s\n", datestr(sessionStart));
    fprintf(fid, "Session end:   %s\n", datestr(actualSessionEnd));
    fprintf(fid, "Duration:      %.2f minutes\n\n", actualMinutes);
    fprintf(fid, "Active levers: %s (fixed for this session)\n", sessionActiveLevers);
    fprintf(fid, "Lever 1 active: %d\n", ismember(sessionActiveLevers, ["BOTH", "LEVER1"]));
    fprintf(fid, "Lever 2 active: %d\n", ismember(sessionActiveLevers, ["BOTH", "LEVER2"]));
    fprintf(fid, "Press limit: %d active-lever presses only; active mode: %s\n", ...
    maxPresses, sessionActiveLevers);
    fprintf(fid, "WaterDelivered=1 means valve commanded on, not measured flow.\n");
    fprintf(fid, "WaterDelivered=0 means inactive press or queued reward canceled by STOP.\n\n");
    fprintf(fid, "Total presses (both levers): %d\n", totalPresses);
    fprintf(fid, "  Lever 1 presses: %d (water delivered: %d)\n", lever1Presses, lever1Water);
    fprintf(fid, "  Lever 2 presses: %d (water delivered: %d)\n", lever2Presses, lever2Water);
    fprintf(fid, "Total water deliveries: %d\n", totalWater);
    fclose(fid);
    fprintf("Text summary saved to %s\n", txtName);
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
% Waits up to timeoutSec for an exact acknowledgment line.
% Prints any other lines seen along the way.
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
