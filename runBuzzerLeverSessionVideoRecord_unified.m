% %% check
% ver
%% runBuzzerLeverSessionVideoRecord_unified.m
%
% Controls leverTaskControllerWindow.ino in TRIAL mode.
% Setup sections can be run separately. START SESSION continues through saving.
% Each completed window earns ONE reward if the active lever was pressed.
% Both levers are counted. Intertrial presses are logged separately and never rewarded.
%
% Each MAT file also contains pressEvents and pressEventInfo.
% Firmware press logging must be acknowledged before START.
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
outFileBase     = "m5650_FI_15";
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
mouseID = 'm5650_FI_15';   %mouse ID
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
commands(end+1) = "SET_PRESS_LOG,ON";
expectedAcks(end+1) = "ACK_PRESS_LOG,ON";
for configIndex = 1:numel(commands)
    writeline(s, commands(configIndex));
    if ~waitForLine(s, expectedAcks(configIndex), ackTimeout)
        error("Configuration not confirmed. Upload leverTaskControllerWindow.ino and rerun CONNECT.");
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
trialStartWallClock = datetime.empty(0,1);
trialEndWallClock   = datetime.empty(0,1);

running = true;
stopRequested = false;
stopAcknowledged = false;
stopTimer = [];
currentTrialStartTime = NaT;

% ===================== MAIN LOOP =====================
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

        case "TRIAL_START"
            if numel(tokens)~=3 || any(~isfinite(str2double(tokens(2:3)))) || ...
                    any(str2double(tokens(2:3))<0) || ...
                    any(str2double(tokens(2:3))~=fix(str2double(tokens(2:3)))) || ...
                    str2double(tokens{2})<1 || str2double(tokens{3})>=2^32
                pressEventMessagesValid = false;
                warning("Malformed trial-start record: %s",char(line));
                if ~stopRequested
                    writeline(s,"STOP"); stopRequested = true; stopTimer = tic;
                end
                continue;
            end
            if isnan(firstTrialArduinoMs)
                firstTrialArduinoMs = str2double(tokens{3});
            end
            currentTrialStartTime = datetime('now');
            fprintf("[%s] Trial %s started.\n", ...
                datestr(currentTrialStartTime,'HH:MM:SS'), tokens{2});

        case {"TRIAL_END", "TRIAL_ABORT"}
            if numel(tokens) ~= 10
                pressEventMessagesValid = false;
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
            pressEventMessagesValid = false;
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
% Drain disk logging before saving; video cleanup errors do not discard events.
try
    stop(vid); diskTimer = tic;
    while vid.DiskLoggerFrameCount < vid.FramesAcquired && toc(diskTimer)<15
        pause(0.05);
    end
catch videoError
    warning("Video cleanup issue: %s",videoError.message);
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
    'VariableNames', {'Trial','TrialStart','TrialEnd','PressCount','SawActivePress', ...
    'WaterDelivered','Lever1Presses','Lever2Presses','ActivePresses','Completed', ...
    'ArduinoStartMs','ArduinoEndMs'});
results.ActiveLevers = repmat(sessionActiveLevers, height(results), 1);

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
    save(matName, 'results', 'pressEvents', 'pressEventInfo', 'sessionStart', 'sessionEnd', 'sessionMinutes', ...
        'actualSessionEnd', 'actualMinutes', 'stopAcknowledged', 'sessionTaskMode', ...
        'sessionActiveLevers', 'sessionTrialSeconds', 'sessionInterTrialSeconds', ...
        'sessionBuzzerSeconds', 'sessionBuzzerFrequencyHz');
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
    fprintf(fid, "Reward rule: one 350 ms valve pulse at completed trial end if ActivePresses > 0.\n");
    fprintf(fid, "Individual press log: %d records; in-trial counts match=%d; valid messages=%d.\n", ...
        height(pressEvents),pressEventInfo.TrialCountsMatch,pressEventInfo.MessagesValid);
    fprintf(fid, "Press timing: Arduino debounced detection at 1 ms resolution, 10 ms debounce.\n");
    fprintf(fid, "TrialTimeSeconds is relative to trial/buzzer onset; ITI uses Trial=0 and NaN.\n");
    fprintf(fid, "SessionTimeSeconds is relative to first trial/buzzer onset; HostReceivedAt is receipt time.\n");
    fprintf(fid, "WaterDelivered is a valve command, not measured water flow.\n");
    fprintf(fid, "Incomplete trials receive no reward. Between-trial presses are logged but do not count or earn reward.\n");
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
