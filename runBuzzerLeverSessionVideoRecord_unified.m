% %% check
% ver
%% runBuzzerLeverSessionVideoRecord_unified.m
%
% Controls leverTaskController.ino in TRIAL mode.
% Run sections in order; run MAIN LOOP promptly after START SESSION.
% Each completed window earns ONE reward if the active lever was pressed.
% Both levers are counted. Intertrial presses are not counted or rewarded.
%
% Protocol implemented (must match the Arduino sketch):
%   Arduino -> "READY"                          on boot
%   MATLAB  -> "START"                          begin the session
%   Arduino -> "ACK_START"
%   Arduino -> "TRIAL_START,<trialNum>"         when a trial's tone/response window begins
%   Arduino -> "TRIAL_END,<trial>,<total>,<sawActive>,<water>,<lever1>,<lever2>,<activeCount>,<startMs>,<endMs>"
%   Arduino -> "TRIAL_ABORT,..." with the same fields for an incomplete trial
%   MATLAB configures SET_TASK, SET_ACTIVE and SET_TRIAL before START.
%   MATLAB  -> "CONTINUE"                       tell Arduino to schedule the next trial
%   Arduino -> "ACK_CONTINUE,<trialNum>"
%   MATLAB  -> "STOP"                           end the session
%   Arduino -> "ACK_STOP"
%
% Stopping rule: the session runs for a FIXED TOTAL DURATION
% (sessionMinutes below). MATLAB keeps sending CONTINUE after every
% TRIAL_END until the time budget is used up, then sends STOP.
% STOP aborts an unfinished trial without reward; partial counts are saved.
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
outFileBase     = "lever_press_session";
activeLevers    = "LEVER2"; % "BOTH", "LEVER1", or "LEVER2"
trialSeconds    = 15;       % Response window: e.g. 15 or 30 seconds
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
mouseID = 'm2_t1';   %mouse ID
mouseID = strcat(mouseID, '_', timestamp);         % Update mouseID with timestamp
newDirectory = fullfile(videoFolder, mouseID); 
mkdir(newDirectory);                                %creates new directory with mouseID as folder name
fileName = fullfile(newDirectory, [mouseID '_T.avi']);
%establish video settings
vid = videoinput('winvideo', 2, 'Y800_640x480'); % Change to 'gentl'/'winvideo' if preferred
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
% Validate and send all settings while Arduino is stopped.
activeLevers = upper(strtrim(string(activeLevers)));
if ~isscalar(activeLevers) || ~ismember(activeLevers, ["BOTH","LEVER1","LEVER2"])
    error('activeLevers must be "BOTH", "LEVER1", or "LEVER2".');
end
validateattributes(trialSeconds, {'numeric'}, {'scalar','finite','>=',0.001,'<=',3600});
validateattributes(interTrialSeconds, {'numeric'}, {'scalar','finite','>=',0,'<=',3600});
validateattributes(buzzerSeconds, {'numeric'}, {'scalar','finite','>=',0,'<=',trialSeconds});
validateattributes(buzzerFrequencyHz, {'numeric'}, {'scalar','integer','>=',31,'<=',20000});
trialSettingsMs = round(1000 * [trialSeconds, interTrialSeconds, buzzerSeconds]);
trialValues = sprintf('%d,%d,%d,%d', trialSettingsMs(1), trialSettingsMs(2), ...
    trialSettingsMs(3), buzzerFrequencyHz);
commands = ["SET_TASK,TRIAL", "SET_ACTIVE," + activeLevers, "SET_TRIAL," + trialValues];
expectedAcks = ["ACK_TASK,TRIAL", "ACK_ACTIVE," + activeLevers, "ACK_TRIAL," + trialValues];
for configIndex = 1:numel(commands)
    writeline(s, commands(configIndex));
    if ~waitForLine(s, expectedAcks(configIndex), ackTimeout)
        error("Configuration not confirmed. Upload leverTaskController.ino and rerun CONNECT.");
    end
end
sessionTaskMode = "TRIAL";
sessionActiveLevers = activeLevers;
% Save the acknowledged (millisecond-rounded) settings, not later workspace edits.
sessionTrialSeconds = trialSettingsMs(1) / 1000;
sessionInterTrialSeconds = trialSettingsMs(2) / 1000;
sessionBuzzerSeconds = trialSettingsMs(3) / 1000;
sessionBuzzerFrequencyHz = buzzerFrequencyHz;
fprintf("TRIAL mode: active=%s; window=%.3f s; pause=%.3f s; buzzer=%.3f s.\n", ...
    sessionActiveLevers, sessionTrialSeconds, sessionInterTrialSeconds, sessionBuzzerSeconds);

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
trialStartWallClock = datetime.empty(0,1);
trialEndWallClock   = datetime.empty(0,1);

running = true;
stopRequested = false;
stopAcknowledged = false;
stopTimer = [];
currentTrialStartTime = NaT;

%% ===================== MAIN LOOP =====================
while running
    if ~stopRequested && datetime('now') >= sessionEnd
        fprintf("Session time budget reached. Sending STOP.\n");
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
        case "TRIAL_START"
            currentTrialStartTime = datetime('now');
            fprintf("[%s] Trial %s started.\n", ...
                datestr(currentTrialStartTime,'HH:MM:SS'), tokens{2});

        case {"TRIAL_END", "TRIAL_ABORT"}
            if numel(tokens) ~= 10
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
            saw = str2double(tokens{4}); % At least one ACTIVE-lever press
            water = str2double(tokens{5});
            completed = string(tokens{1}) == "TRIAL_END";
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
            trialCompleted(end+1,1) = completed;
            trialStartWallClock(end+1,1) = currentTrialStartTime;
            trialEndWallClock(end+1,1) = tEnd;

            fprintf("Trial %d: lever1=%d, lever2=%d, active=%d, water=%d, completed=%d.\n", ...
                tNum, lever1Presses(end), lever2Presses(end), activePresses(end), water, completed);
            currentTrialStartTime = NaT;

            if completed && ~stopRequested
                if datetime('now') >= sessionEnd
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
stop(vid);               % stops acquisition; file is already on disk
closepreview(vid);
try
    close(vw);           % close writer explicitly (good hygiene)
catch
end
delete(vid);
clear vid
disp(['Video saved as ', fileName]);
%% ===================== CLEAN UP =====================
delete(s);
fprintf("Serial connection closed.\n");

results = table(trialNum, trialStartWallClock, trialEndWallClock, pressCount, ...
    sawPress, waterDelivered, lever1Presses, lever2Presses, activePresses, ...
    trialCompleted, trialStartArduinoMs, trialEndArduinoMs, ...
    'VariableNames', {'Trial','TrialStart','TrialEnd','PressCount','SawActivePress', ...
    'WaterDelivered','Lever1Presses','Lever2Presses','ActivePresses','Completed', ...
    'ArduinoStartMs','ArduinoEndMs'});
results.ActiveLevers = repmat(sessionActiveLevers, height(results), 1);

fprintf("\nSession complete: %d completed, %d aborted trials in %.2f minutes.\n", ...
    sum(trialCompleted), sum(~trialCompleted), actualMinutes);
disp(results);
%%
if saveResults
    timestamp = datestr(now, 'yyyy-mm-dd_HHMMSS');
    sessionOutputBase = string(fullfile(newDirectory, sprintf('%s_%s', outFileBase, timestamp)));
    csvName = sessionOutputBase + ".csv";
    matName = sessionOutputBase + ".mat";
    save(matName, 'results', 'sessionStart', 'sessionEnd', 'sessionMinutes', ...
        'actualSessionEnd', 'actualMinutes', 'stopAcknowledged', 'sessionTaskMode', ...
        'sessionActiveLevers', 'sessionTrialSeconds', 'sessionInterTrialSeconds', ...
        'sessionBuzzerSeconds', 'sessionBuzzerFrequencyHz');
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
    fprintf(fid, "Reward rule: one 220 ms valve pulse at completed trial end if ActivePresses > 0.\n");
    fprintf(fid, "WaterDelivered is a valve command, not measured water flow.\n");
    fprintf(fid, "Incomplete trials receive no reward. Between-trial presses are ignored.\n");
    fprintf(fid, "Completed trials: %d\nAborted trials: %d\n", sum(trialCompleted), sum(~trialCompleted));
    fprintf(fid, "Lever 1 presses: %d\nLever 2 presses: %d\nActive presses: %d\n", ...
        sum(lever1Presses), sum(lever2Presses), sum(activePresses));
    fprintf(fid, "Reward commands: %d\nSTOP acknowledged: %d\n", sum(waterDelivered), stopAcknowledged);
    fclose(fid);
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
