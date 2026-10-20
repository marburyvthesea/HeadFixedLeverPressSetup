%% lever_press_control_VI.m
%
% Variable-interval lever press session with two levers, buzzer cue,
% and video recording. Each trial has a randomized ACTIVE WINDOW
% (random start AND random end within the trial) -- presses inside
% that window qualify for reward; presses outside it (before or after)
% do not.
%
% Session ends when EITHER the time budget OR the water delivery cap
% is reached, whichever comes first.
%
% Protocol (must match the Arduino sketch):
%   Arduino -> "READY"
%   MATLAB  -> "START"
%   Arduino -> "ACK_START"
%   Arduino -> "TRIAL_START,<trialNum>,<windowStartOffsetMs>,<windowEndOffsetMs>"
%   Arduino -> "TRIAL_END,<trialNum>,<pressCount>,<sawPress>,<waterDelivered>,<lever1Cnt>,<lever2Cnt>,<earlyCnt>,<lateCnt>"
%   MATLAB  -> "CONTINUE"
%   Arduino -> "ACK_CONTINUE,<trialNum>"
%   MATLAB  -> "STOP"
%   Arduino -> "ACK_STOP"

clear; clc;

%% ===================== USER SETTINGS =====================
portName            = "COM4";
baudRate            = 115200;
sessionMinutes      = 30;      % total session duration (minutes)
maxWaterDeliveries  = 100;     % stop early once this many rewards given
ackTimeout          = 5;
saveResults         = true;
outFileBase         = "m0708_2_VI_session";

%% ===================== CONNECT =====================
if exist('s','var')
    try
        delete(s);
    catch
    end
    clear s
end

s = serialport(portName, baudRate);
configureTerminator(s, "LF");
pause(2);
flush(s);

fprintf("Connecting to Arduino...\n");

maxRetries = 10;
connected = false;
for attempt = 1:maxRetries
    flush(s);
    writeline(s, "START");
    if waitForLine(s, "ACK_START", 1.5)
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

%% ===================== VIDEO RECORDING SETUP =====================
videoFolder = 'D:\Data\JJM\HeadFixedLeverPress';
timestamp = datestr(now, 'mm_dd_yy_HH_MM_SS');
mouseID = 'm07082026_01';
mouseID = strcat(mouseID, '_', timestamp);
newDirectory = strcat(videoFolder, mouseID);
mkdir(newDirectory);
fileName = strcat(newDirectory, "\", mouseID, "_T", '.avi');

vid = videoinput('winvideo', 2, 'Y800_640x480');
src = getselectedsource(vid);
src.FrameRate = '120.0005';
vw = VideoWriter(fileName, 'Motion JPEG AVI');
vw.Quality = 75;
vw.FrameRate = 113;
vid.LoggingMode = 'disk';
vid.DiskLogger = vw;
vid.ReturnedColorSpace = 'grayscale';
vid.FramesPerTrigger = Inf;

%% ===================== START SESSION =====================
open(vw);
preview(vid);
triggerconfig(vid,'immediate');
start(vid);
disp('Video recording started.');

sessionStart = datetime('now');
sessionEnd   = sessionStart + minutes(sessionMinutes);
fprintf("Session started at %s. Will run until %s (%.1f min) or %d water deliveries.\n", ...
    datestr(sessionStart), datestr(sessionEnd), sessionMinutes, maxWaterDeliveries);

% Results accumulators: one entry per completed trial
trialNum   = [];
pressCount = [];
sawPress = [];
waterDelivered = [];
lever1PressCount = [];
lever2PressCount = [];
earlyPressCount = [];
latePressCount = [];
windowStartMsLog = [];
windowEndMsLog   = [];
trialStartWallClock = datetime.empty(0,1);
trialEndWallClock   = datetime.empty(0,1);

running = true;
currentTrialStartTime = NaT;
currentWindowStartMs = NaN;
currentWindowEndMs = NaN;

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

    % Check water delivery cap
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
            currentWindowStartMs = str2double(tokens{3});
            currentWindowEndMs   = str2double(tokens{4});
            fprintf("[%s] Trial %s started. Active window: %.1fs - %.1fs into trial.\n", ...
                datestr(currentTrialStartTime,'HH:MM:SS'), tokens{2}, ...
                currentWindowStartMs/1000, currentWindowEndMs/1000);

        case "TRIAL_END"
            tNum      = str2double(tokens{2});
            nPress    = str2double(tokens{3});
            saw       = str2double(tokens{4});
            water     = str2double(tokens{5});
            lever1Cnt = str2double(tokens{6});
            lever2Cnt = str2double(tokens{7});
            earlyCnt  = str2double(tokens{8});
            lateCnt   = str2double(tokens{9});
            tEnd      = datetime('now');

            trialNum(end+1,1)   = tNum;   %#ok<*SAGROW>
            pressCount(end+1,1) = nPress;
            sawPress(end+1,1)   = saw;
            waterDelivered(end+1,1) = water;
            lever1PressCount(end+1,1) = lever1Cnt;
            lever2PressCount(end+1,1) = lever2Cnt;
            earlyPressCount(end+1,1) = earlyCnt;
            latePressCount(end+1,1) = lateCnt;
            windowStartMsLog(end+1,1) = currentWindowStartMs;
            windowEndMsLog(end+1,1) = currentWindowEndMs;
            trialStartWallClock(end+1,1) = currentTrialStartTime;
            trialEndWallClock(end+1,1) = tEnd;

            fprintf("[%s] Trial %d ended: %d press(es) [L1=%d, L2=%d], nonQualifying=%d, qualifying=%d, water=%d.\n", ...
                datestr(tEnd,'HH:MM:SS'), tNum, nPress, lever1Cnt, lever2Cnt, earlyCnt, lateCnt, water);
            if water == 1
                fprintf(" Water delivered. Total so far: %d/%d\n", sum(waterDelivered), maxWaterDeliveries);
            else
                fprintf(" No water delivered.\n");
            end

            % Decide whether to continue or stop
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
            % handled via waitForLine above; ignore if seen again

        otherwise
            fprintf("[Arduino] %s\n", line);
    end
end

disp('Experiment over.')
stop(vid);
closepreview(vid);
try
    close(vw);
catch
end
delete(vid);
clear vid
disp(['Video saved as ', fileName]);

%% ===================== CLEAN UP =====================
delete(s);
fprintf("Serial connection closed.\n");

results = table(trialNum, trialStartWallClock, trialEndWallClock, windowStartMsLog, windowEndMsLog, ...
    pressCount, sawPress, waterDelivered, lever1PressCount, lever2PressCount, ...
    earlyPressCount, latePressCount, ...
    'VariableNames', {'Trial','TrialStart','TrialEnd','WindowStartMs','WindowEndMs', ...
    'PressCount','SawPress','WaterDelivered','Lever1PressCount','Lever2PressCount', ...
    'NonQualifyingPressCount','QualifyingPressCount'});

fprintf("\nSession complete: %d trials in %.1f minutes, %d water deliveries.\n", ...
    height(results), sessionMinutes, sum(results.WaterDelivered));
disp(results);

if saveResults
    timestamp = datestr(now, 'yyyy-mm-dd_HHMMSS');

    % Save results into the same per-mouse folder as the video
    csvName  = fullfile(newDirectory, sprintf('%s_%s.csv', outFileBase, timestamp));
    matName  = fullfile(newDirectory, sprintf('%s_%s.mat', outFileBase, timestamp));
    xlsxName = fullfile(newDirectory, sprintf('%s_%s.xlsx', outFileBase, timestamp));

    writetable(results, csvName);
    save(matName, 'results', 'sessionStart', 'sessionEnd', 'sessionMinutes', 'maxWaterDeliveries');
    fprintf("Results saved to %s and %s\n", csvName, matName);

    % ---- Excel export: just trial count and water delivery count ----
    totalTrials = height(results);
    totalWater  = sum(results.WaterDelivered);

    summaryTable = table( ...
        ["Total trials"; "Total water deliveries"], ...
        [totalTrials; totalWater], ...
        'VariableNames', {'Metric','Value'});
    writetable(summaryTable, xlsxName, 'Sheet', 'Summary');
    fprintf("Excel file saved to %s\n", xlsxName);

    % % ---- Plain text summary ----
    % fid = fopen(txtName, 'w');
    % fprintf(fid, "Lever Press Session Summary (Variable Interval)\n");
    % fprintf(fid, "=================================================\n");
    % fprintf(fid, "Session start: %s\n", datestr(sessionStart));
    % fprintf(fid, "Session end:   %s\n", datestr(sessionEnd));
    % fprintf(fid, "Duration:      %.1f minutes\n\n", sessionMinutes);
    % fprintf(fid, "Total trials: %d\n", totalTrials);
    % fprintf(fid, "Trials with any press: %d\n", trialsWithAnyPress);
    % fprintf(fid, "Lever 1 total presses: %d\n", lever1Total);
    % fprintf(fid, "Lever 2 total presses: %d\n", lever2Total);
    % fprintf(fid, "Non-qualifying presses: %d\n", nonQualTotal);
    % fprintf(fid, "Qualifying presses: %d\n", qualTotal);
    % fprintf(fid, "Total water deliveries: %d/%d\n\n", totalWater, maxWaterDeliveries);
    % fprintf(fid, "Water delivery times:\n");
    % for i = 1:height(waterTrials)
    %     fprintf(fid, "  Trial %d at %s (window %.1fs-%.1fs, L1=%d, L2=%d)\n", ...
    %         waterTrials.Trial(i), datestr(waterTrials.TrialEnd(i),'HH:MM:SS'), ...
    %         waterTrials.WindowStartMs(i)/1000, waterTrials.WindowEndMs(i)/1000, ...
    %         waterTrials.Lever1PressCount(i), waterTrials.Lever2PressCount(i));
    % end
    % fclose(fid);
    % fprintf("Text summary saved to %s\n", txtName);
end

%% ===================== HELPER FUNCTION =====================
function found = waitForLine(s, expectedPrefix, timeoutSec)
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