%% runRandomActiveWindowLeverSession.m
% Trial-buffered alternative. Upload the MATCHING unified .ino from this branch.
% Click through USER SETTINGS, VIDEO SETTINGS, then RUN SESSION.
% The board stores in-trial press times in RAM and sends no USB messages during
% the timed trial/reward pulse. Data is pulled and checkpointed before CONTINUE.
% Keep runBufferedLeverSession.m and decodeBufferedLeverTrial.m on the MATLAB path.
% Intertrial presses are NOT timestamped. Readout may extend the minimum pause.

clear; clc;

%% ===================== USER SETTINGS =====================
portName        = "COM4";   % <-- change to your Arduino's port (e.g. "COM3" or "/dev/tty.usbmodem14101")
baudRate        = 115200;   % must match Serial.begin() in the sketch
sessionMinutes  = 40;       % total session duration (minutes)
readyTimeout    = 10;       % total seconds allowed for the STOP connection handshake
ackTimeout      = 5;        % seconds to wait for ACKs
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


% BATCH requires durable MAT checkpoints before acknowledging device data.
serialStallGraceSeconds = 10;

%% ===================== VIDEO SETTINGS =====================
videoFolder = 'D:\Data\JJM\HeadFixedLeverPress';         %folder for all mouse trial videos
mouseID = 'm4850_VI_80';   %mouse ID
recordVideo = true;
showVideoPreview = true;
videoAdaptor = "winvideo";
videoDevice = 1;
videoFormat = "Y800_640x480";
cameraFrameRate = "120.0005";
savedVideoFrameRate = 113;

%% ===================== RUN SESSION =====================
% One continuous call connects, configures, records, fetches, saves and stops.
settings = struct('taskMode',"WINDOW",'portName',string(portName),'baudRate',baudRate, ...
    'sessionMinutes',sessionMinutes,'readyTimeout',readyTimeout,'ackTimeout',ackTimeout, ...
    'serialStallGraceSeconds',serialStallGraceSeconds,'saveResults',saveResults, ...
    'outFileBase',string(outFileBase),'activeLevers',string(activeLevers), ...
    'trialSeconds',trialSeconds,'interTrialSeconds',interTrialSeconds, ...
    'buzzerSeconds',buzzerSeconds,'buzzerFrequencyHz',buzzerFrequencyHz, ...
    'videoFolder',string(videoFolder),'mouseID',string(mouseID),'recordVideo',recordVideo, ...
    'showVideoPreview',showVideoPreview,'videoAdaptor',videoAdaptor,'videoDevice',videoDevice, ...
    'videoFormat',videoFormat,'cameraFrameRate',cameraFrameRate,'savedVideoFrameRate',savedVideoFrameRate);
settings.windowStartRangeSeconds = windowStartRangeSeconds;
settings.activeWindowLengthRangeSeconds = activeWindowLengthRangeSeconds;
settings.randomSeed = randomSeed;
settings.maxWaterDeliveries = maxWaterDeliveries;
[results,pressEvents,session] = runBufferedLeverSession(settings);
