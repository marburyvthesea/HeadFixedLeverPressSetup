% %% check
% ver
%% lever_press_control.m
%
% Controls the Arduino lever-press/buzzer sketch over serial.
%
% Protocol implemented (must match the Arduino sketch):
%   Arduino -> "READY"                          on boot
%   MATLAB  -> "START"                          begin the session
%   Arduino -> "ACK_START"
%   Arduino -> "TRIAL_START,<trialNum>"         when a trial's tone/response window begins
%   Arduino -> "TRIAL_END,<trialNum>,<pressCount>,<sawPress>"
%   MATLAB  -> "CONTINUE"                       tell Arduino to schedule the next trial
%   Arduino -> "ACK_CONTINUE,<trialNum>"
%   MATLAB  -> "STOP"                           end the session
%   Arduino -> "ACK_STOP"
%
% Stopping rule: the session runs for a FIXED TOTAL DURATION
% (sessionMinutes below). MATLAB keeps sending CONTINUE after every
% TRIAL_END until the time budget is used up, then sends STOP.
%


clear; clc;

%% ===================== USER SETTINGS =====================
portName        = "COM4";   % <-- change to your Arduino's port (e.g. "COM3" or "/dev/tty.usbmodem14101")
baudRate        = 115200;   % must match Serial.begin() in the sketch
sessionMinutes  = 5;       % total session duration (minutes)
readyTimeout    = 10;       % seconds to wait for "READY" after connecting
ackTimeout      = 5;        % seconds to wait for ACKs
saveResults     = true;
outFileBase     = "m0710_1_Devaluation_session";
maxWaterDeliveries = 100;

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
    writeline(s, "START");
    if waitForLine(s, "ACK_START", 1.5)   % short timeout per attempt
        connected = true;
        break;
    end
    fprintf("No response on attempt %d/%d, retrying...\n", attempt, maxRetries);
end

if ~connected
    delete(s);
    error("Could not get ACK_START from Arduino after %d attempts. Check port/wiring.", maxRetries);
end

fprintf("Arduino connected and session started.\n");
%% Video Recording Parameters
videoFolder = 'D:\Data\JJM\HeadFixedLeverPress';         %folder for all mouse trial videos
timestamp = datestr(now, 'mm_dd_yy_HH_MM_SS'); 
mouseID = 'm07102026_01';   %mouse ID
mouseID = strcat(mouseID, '_', timestamp);         % Update mouseID with timestamp
newDirectory = strcat(videoFolder, mouseID); 
mkdir(newDirectory);                                %creates new directory with mouseID as folder name
fileName = strcat(newDirectory, "\", mouseID, "_T", '.avi');
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
% --- run ---
open(vw);                % open the writer before start
preview(vid);            % non-blocking live view
triggerconfig(vid,'immediate');
start(vid);              % begins acquiring & writing to disk in background
disp('Video recording started.');


writeline(s, "START");
if ~waitForLine(s, "ACK_START", ackTimeout)
    warning("Did not receive ACK_START -- continuing anyway, but check the connection.");
end

sessionStart = datetime('now');
sessionEnd   = sessionStart + minutes(sessionMinutes);
fprintf("Session started at %s. Will run until %s (%.1f min).\n", ...
    datestr(sessionStart), datestr(sessionEnd), sessionMinutes);

% Results accumulators: one entry per completed trial
trialNum   = [];
pressCount = [];
sawPress = [];
waterDelivered = [];
lever1PressCount = [];   % NEW
lever2PressCount = [];   % NEW
trialStartWallClock = datetime.empty(0,1);
trialEndWallClock   = datetime.empty(0,1);
running = true;
currentTrialStartTime = NaT;

%% ===================== MAIN LOOP =====================
while running

    % Check time budget
    if datetime('now') >= sessionEnd
        fprintf("Session time budget reached. Sending STOP.\n");
        writeline(s, "STOP");
        waitForLine(s, "ACK_STOP", ackTimeout);
        running = false;
        break;
    end

    % Check water delivery limit
    if sum(waterDelivered) >= maxWaterDeliveries
        fprintf("Water delivery limit (%d) reached. Sending STOP.\n", maxWaterDeliveries);
        writeline(s, "STOP");
        waitForLine(s, "ACK_STOP", ackTimeout);
        running = false;
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
            fprintf("[%s] Trial %s started.\n", datestr(currentTrialStartTime,'HH:MM:SS'), tokens{2});

        case "TRIAL_END"
            tNum = str2double(tokens{2});
            nPress = str2double(tokens{3});
            saw = str2double(tokens{4});
            water = str2double(tokens{5});
            lever1Cnt = str2double(tokens{6});
            lever2Cnt = str2double(tokens{7});
            tEnd = datetime('now');
        
            trialNum(end+1,1)   = tNum;
            pressCount(end+1,1) = nPress;
            sawPress(end+1,1)   = saw;
            waterDelivered(end+1,1) = water;
            lever1PressCount(end+1,1) = lever1Cnt;
            lever2PressCount(end+1,1) = lever2Cnt;
            trialStartWallClock(end+1,1) = currentTrialStartTime;
            trialEndWallClock(end+1,1) = tEnd;
        
            fprintf("[%s] Trial %d ended: %d press(es) [L1=%d, L2=%d], sawPress=%d, water=%d.\n", ...
                datestr(tEnd,'HH:MM:SS'), tNum, nPress, lever1Cnt, lever2Cnt, saw, water);
            if water == 1
                fprintf(" Water delivered. Total so far: %d/%d\n", sum(waterDelivered), maxWaterDeliveries);
            else
                fprintf(" No water delivered.\n");
            end
        
            % Decide whether to continue or stop: time budget OR water limit OR neither
            if datetime('now') >= sessionEnd
                fprintf("Session time budget reached. Sending STOP.\n");
                writeline(s, "STOP");
                waitForLine(s, "ACK_STOP", ackTimeout);
                running = false;
            elseif sum(waterDelivered) >= maxWaterDeliveries
                fprintf("Water delivery limit (%d) reached. Sending STOP.\n", maxWaterDeliveries);
                writeline(s, "STOP");
                waitForLine(s, "ACK_STOP", ackTimeout);
                running = false;
            else
                writeline(s, "CONTINUE");
                waitForLine(s, "ACK_CONTINUE", ackTimeout);
            end

        case "ACK_CONTINUE"
            % already handled via waitForLine above in most cases; ignore if seen again

        otherwise
            % Unrecognized line - print for visibility (e.g. debug prints)
            fprintf("[Arduino] %s\n", line);
    end
end
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

results = table(trialNum, trialStartWallClock, trialEndWallClock, pressCount, sawPress, waterDelivered, ...
    lever1PressCount, lever2PressCount, ...
    'VariableNames', {'Trial','TrialStart','TrialEnd','PressCount','SawPress','WaterDelivered', ...
    'Lever1PressCount','Lever2PressCount'});

fprintf("\nSession complete: %d trials in %.1f minutes.\n", height(results), sessionMinutes);
disp(results);
%%
if saveResults
     timestamp = datestr(now, 'yyyy-mm-dd_HHMMSS');
     csvName = sprintf('%s_%s.csv', outFileBase, timestamp);
     matName = sprintf('%s_%s.mat', outFileBase, timestamp);
     writetable(results, csvName);
     save(matName, 'results', 'sessionStart', 'sessionEnd', 'sessionMinutes');
     fprintf("Results saved to %s and %s\n", csvName, matName);

     % ---- Summary stats ----
     totalTrials  = height(results);
     trialsWithPress = sum(results.SawPress == 1);
     totalWater   = sum(results.WaterDelivered);
     lever1Total  = sum(results.Lever1PressCount);
     lever2Total  = sum(results.Lever2PressCount);

     % ---- Water delivery timing detail ----
     waterTrials = results(results.WaterDelivered == 1, {'Trial','TrialStart','TrialEnd','Lever1PressCount','Lever2PressCount'});

     % ---- Excel export: raw trials + summary + water timing on separate sheets ----
     xlsxName = sprintf('%s_%s.xlsx', outFileBase, timestamp);
     writetable(results, xlsxName, 'Sheet', 'RawTrials');

     summaryTable = table( ...
         ["Total trials"; "Trials with any press"; "Total water deliveries"; ...
          "Lever 1 total presses"; "Lever 2 total presses"], ...
         [totalTrials; trialsWithPress; totalWater; lever1Total; lever2Total], ...
         'VariableNames', {'Metric','Value'});
     writetable(summaryTable, xlsxName, 'Sheet', 'Summary');

     writetable(waterTrials, xlsxName, 'Sheet', 'WaterDeliveryTimes');
     fprintf("Excel file saved to %s\n", xlsxName);

     % % ---- Plain text summary ----
     % txtName = sprintf('%s_%s.txt', outFileBase, timestamp);
     % fid = fopen(txtName, 'w');
     % fprintf(fid, "Lever Press Session Summary\n");
     % fprintf(fid, "============================\n");
     % fprintf(fid, "Session start: %s\n", datestr(sessionStart));
     % fprintf(fid, "Session end:   %s\n", datestr(sessionEnd));
     % fprintf(fid, "Duration:      %.1f minutes\n\n", sessionMinutes);
     % fprintf(fid, "Total trials: %d\n", totalTrials);
     % fprintf(fid, "Trials with any press: %d\n", trialsWithPress);
     % fprintf(fid, "Lever 1 total presses: %d\n", lever1Total);
     % fprintf(fid, "Lever 2 total presses: %d\n", lever2Total);
     % fprintf(fid, "Total water deliveries: %d/%d\n\n", totalWater, maxWaterDeliveries);
     % fprintf(fid, "Water delivery times:\n");
     % for i = 1:height(waterTrials)
     %     fprintf(fid, "  Trial %d at %s (L1=%d, L2=%d presses that trial)\n", ...
     %         waterTrials.Trial(i), datestr(waterTrials.TrialEnd(i),'HH:MM:SS'), ...
     %         waterTrials.Lever1PressCount(i), waterTrials.Lever2PressCount(i));
     % end
     % fclose(fid);
     % fprintf("Text summary saved to %s\n", txtName);
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
% Waits up to timeoutSec for a line from the serial port that starts
% with expectedPrefix. Prints any other lines seen along the way.
    found = false;
    t0 = tic;
    while toc(t0) < timeoutSec
        if s.NumBytesAvailable > 0
            line = strtrim(readline(s));
            if startsWith(line, expectedPrefix)
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