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
sessionMinutes  = 1;       % total session duration (minutes)
readyTimeout    = 10;       % seconds to wait for "READY" after connecting
ackTimeout      = 5;        % seconds to wait for ACKs
saveResults     = true;
outFileBase     = "test_FI_15";

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
mouseID = 'test_FI_15';   %mouse ID
mouseID = strcat(mouseID, '_', timestamp);         % Update mouseID with timestamp
newDirectory = strcat(videoFolder, mouseID); 
mkdir(newDirectory);                                %creates new directory with mouseID as folder name
fileName = strcat(newDirectory, "\", mouseID, "_T", '.avi');
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
trialStartWallClock = datetime.empty(0,1);
trialEndWallClock   = datetime.empty(0,1);

running = true;
currentTrialStartTime = NaT;

%% ===================== MAIN LOOP =====================
while running

    % Check time budget before/while waiting on Arduino
    if datetime('now') >= sessionEnd
        fprintf("Session time budget reached. Sending STOP.\n");
        writeline(s, "STOP");
        waitForLine(s, "ACK_STOP", ackTimeout); % best effort
        running = false;
        break;
    end

    if s.NumBytesAvailable == 0
        pause(0.02);           % small idle wait, avoid busy-spinning
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
            tEnd = datetime('now');

            trialNum(end+1,1)   = tNum;   %#ok<*SAGROW>
            pressCount(end+1,1) = nPress;
            sawPress(end+1,1)   = saw;
            waterDelivered(end+1,1) = water;
            trialStartWallClock(end+1,1) = currentTrialStartTime;
            trialEndWallClock(end+1,1) = tEnd;

            fprintf("[%s] Trial %d ended: %d press(es), sawPress=%d, water=%d.\n", ...
                datestr(tEnd,'HH:MM:SS'), tNum, nPress, saw, water);
            if water == 1
                fprintf(" Water delivered.\n");
                %fprintf('%d\n', saw);
            else
                fprintf(" No water delivered.\n");
                %fprintf('0\n');
            end
            %fprintf('%s\n', water);

            % Decide whether to continue or stop based on remaining time
            if datetime('now') >= sessionEnd
                fprintf("Session time budget reached. Sending STOP.\n");
                writeline(s, "STOP");
                waitForLine(s, "ACK_STOP", ackTimeout); % best effort
                running = false;
            else
                writeline(s, "CONTINUE");
                % Not strictly necessary to block on the ACK, but useful for logging
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
    'VariableNames', {'Trial','TrialStart','TrialEnd','PressCount','SawPress', 'WaterDelivered'});

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