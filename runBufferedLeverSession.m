function [results,pressEvents,session] = runBufferedLeverSession(settings)
% BATCH transport: one requested reply at a time, only after hardware is quiet.
% Do not mix this runner with the live-stream or generic TX-queue firmware.
settings = validateSettings(settings);
s = []; vid = []; writer = []; rx = ''; serialLog = -1;
results = table; pressEvents = table; trialBatches = struct([]); trialInfo = struct([]);
firstTrialArduinoMs = NaN; sessionClock = []; stopAcknowledged = false;
readyBeforeStop = []; logBytes = 0; logLines = 0;
stamp = string(datetime('now','Format','yyyyMMdd_HHmmss_SSS'));
folder = fullfile(settings.videoFolder,settings.mouseID+"_"+stamp);
[ok,message] = mkdir(folder); assert(ok,'Cannot create session folder: %s',message);
matFile = fullfile(folder,settings.outFileBase+"_batch.mat");
session = struct('TaskMode',settings.taskMode,'Settings',settings,'OutputFolder',folder, ...
    'MatFile',matFile,'Start',NaT,'End',NaT,'Error',"",'StopReason',"", ...
    'StopAcknowledged',false,'DeviceCapacity',NaN,'DeviceSolenoidMs',NaN, ...
    'RecoveredFiles',strings(0,1),'AcknowledgedTrials',zeros(0,1),'SerialBytes',0,'SerialLines',0);
cleanup = onCleanup(@release);
try
    serialLog = fopen(fullfile(folder,'serial.log'),'w');
    assert(serialLog>=0,'Cannot create serial log.');
    s = serialport(settings.portName,settings.baudRate,'Timeout',settings.ackTimeout);
    configureTerminator(s,"LF");
    try setDTR(s,true); catch dtrError
        warning('LeverBatch:DTR','Could not set DTR: %s',dtrError.message);
    end
    pause(2); flush(s); rx = '';
    fprintf("Connecting to Arduino (STOP only)...\n");
    connectClock = tic; connected = false;
    while toc(connectClock)<settings.readyTimeout
        writeline(s,"STOP");
        try
            waitReply("ACK_STOP",min(1.5,settings.ackTimeout),true);
            connected = true; break;
        catch connectError
            if ~strcmp(connectError.identifier,'LeverBatch:Timeout'), rethrow(connectError); end
        end
    end
    assert(connected,'No ACK_STOP. Check current COM port, Serial Monitor and matching firmware.');
    stopAcknowledged = true;
    % Never overwrite a previous unacknowledged device trial on reconnect.
    pending = request("GET_BATCH","BATCH",settings.ackTimeout);
    if pending(1)=="BATCH_READY"
        header = parseHeader(pending);
        if ~header.Acknowledged
            recovered = fetchBatch(header);
            [recoveredResults,recoveredPressEvents,recoveredInfo] = ...
                decodeBufferedLeverTrial(recovered,recovered.ArduinoStartMs);
            recoveryFile = fullfile(folder,sprintf('recovered_device_trial_%d.mat',header.Trial));
            recoveredCheckpoint = struct('snapshot',recovered,'results',recoveredResults, ...
                'pressEvents',recoveredPressEvents,'pressEventInfo',recoveredInfo, ...
                'Provenance',"Retained board RAM from a prior session; subject identity unknown");
            durableSave(recoveryFile,recoveredCheckpoint);
            acknowledge(header.Trial); % Only after recovery MAT is on disk.
            session.RecoveredFiles(end+1,1) = string(recoveryFile);
            fprintf('Recovered retained device trial to %s\n',recoveryFile);
        end
    elseif pending(1)~="BATCH_EMPTY"
        error('Unexpected initial batch response.');
    else
        assert(numel(pending)==1,'Malformed BATCH_EMPTY response.');
    end
    request("SET_TASK,"+settings.taskMode,"ACK_TASK",settings.ackTimeout, ...
        ["ACK_TASK",settings.taskMode]);
    request("SET_ACTIVE,"+settings.activeLevers,"ACK_ACTIVE",settings.ackTimeout, ...
        ["ACK_ACTIVE",settings.activeLevers]);
    trialMs = round(1000*[settings.trialSeconds settings.interTrialSeconds settings.buzzerSeconds]);
    values = sprintf('%d,%d,%d,%d',trialMs(1),trialMs(2),trialMs(3),settings.buzzerFrequencyHz);
    request("SET_TRIAL,"+values,"ACK_TRIAL",settings.ackTimeout,split("ACK_TRIAL,"+values,","));
    if settings.taskMode=="WINDOW"
        values = sprintf('%d,%d,%d,%d,%d',settings.windowStartRangeMs(1), ...
            settings.windowStartRangeMs(2),settings.windowLengthRangeMs(1), ...
            settings.windowLengthRangeMs(2),settings.sessionRandomSeed);
        request("SET_WINDOW,"+values,"ACK_WINDOW",settings.ackTimeout,split("ACK_WINDOW,"+values,","));
    end
    enabled = request("SET_PRESS_LOG,BATCH","ACK_PRESS_LOG",settings.ackTimeout);
    assert(numel(enabled)==4 && enabled(2)=="BATCH",'Matching BATCH firmware is required.');
    values = unsignedValues(enabled(3:4));
    assert(values(1)==1024 && values(2)>0,'Unexpected device buffer capacity or valve pulse.');
    session.DeviceCapacity = values(1); session.DeviceSolenoidMs = values(2);
    if settings.recordVideo || settings.showVideoPreview
        vid = videoinput(char(settings.videoAdaptor),settings.videoDevice,char(settings.videoFormat));
        source = getselectedsource(vid);
        if strlength(settings.cameraFrameRate)>0, source.FrameRate = char(settings.cameraFrameRate); end
        vid.ReturnedColorSpace = 'grayscale';
        if settings.recordVideo
            writer = VideoWriter(fullfile(folder,settings.mouseID+"_T.avi"),'Motion JPEG AVI');
            writer.FrameRate = settings.savedVideoFrameRate; writer.Quality = 75;
            vid.LoggingMode = 'disk'; vid.DiskLogger = writer;
            vid.FramesPerTrigger = Inf; triggerconfig(vid,'immediate'); start(vid);
        end
        if settings.showVideoPreview, preview(vid); end
    end
    session.Start = datetime('now'); sessionClock = tic;
    stopAcknowledged = false;
    request("START","ACK_START",settings.ackTimeout,"ACK_START");
    expectedTrial = 1;
    while true
        fprintf('Trial %d requested; board is capturing silently.\n',expectedTrial);
        [header,stopped] = awaitBatch(expectedTrial);
        if isempty(header), break; end
        snapshot = fetchBatch(header);
        assert(string(snapshot.TaskMode)==settings.taskMode && ...
            string(snapshot.ActiveLevers)==settings.activeLevers,'Unexpected device task configuration.');
        assert(snapshot.SolenoidMs==session.DeviceSolenoidMs,'Device pulse changed after configuration.');
        plannedTrialMs = round(1000*settings.trialSeconds);
        if settings.taskMode=="WINDOW"
            actualLength = snapshot.WindowEndMs-snapshot.WindowStartMs;
            assert(snapshot.WindowStartMs>=settings.windowStartRangeMs(1) && ...
                snapshot.WindowStartMs<=settings.windowStartRangeMs(2) && ...
                actualLength>=settings.windowLengthRangeMs(1) && ...
                actualLength<=settings.windowLengthRangeMs(2) && ...
                snapshot.WindowEndMs<=plannedTrialMs,'Window metadata disagrees with acknowledged settings.');
        else
            assert(snapshot.WindowStartMs==0 && snapshot.WindowEndMs==plannedTrialMs, ...
                'Full-trial window disagrees with acknowledged settings.');
        end
        if snapshot.Completed
            assert(mod(snapshot.ArduinoEndMs-snapshot.ArduinoStartMs,2^32)>=plannedTrialMs, ...
                'Completed trial ended before its configured duration.');
        end
        if isnan(firstTrialArduinoMs), firstTrialArduinoMs = snapshot.ArduinoStartMs; end
        [row,events,info] = decodeBufferedLeverTrial(snapshot,firstTrialArduinoMs);
        if isempty(results), results = row; else, results = [results;row]; end
        if isempty(pressEvents), pressEvents = events; else, pressEvents = [pressEvents;events]; end
        trialBatches(end+1,1) = snapshot;
        trialInfo(end+1,1) = info;
        % This write is mandatory. A failure leaves RAM data unacknowledged.
        checkpoint();
        acknowledge(snapshot.Trial);
        session.AcknowledgedTrials(end+1,1) = snapshot.Trial;
        fprintf('Trial %d: presses=%d, eligible=%d, reward=%d, saved=%d, dropped=%d.\n', ...
            snapshot.Trial,row.PressCount,row.ActivePresses,row.WaterDelivered, ...
            row.CapturedPresses,row.DroppedPresses);
        if snapshot.DroppedPresses>0
            session.StopReason = "Device press-buffer overflow";
            stopController(); break;
        end
        if stopped || ~snapshot.Completed
            if strlength(session.StopReason)==0, session.StopReason = "Trial aborted"; end
            stopController(); break;
        end
        if expired()
            session.StopReason = "Session time limit"; stopController(); break;
        end
        if sum(results.WaterDelivered)>=settings.maxWaterDeliveries
            session.StopReason = "Water delivery cap"; stopController(); break;
        end
        request("CONTINUE","ACK_CONTINUE",settings.ackTimeout,["ACK_CONTINUE",string(snapshot.Trial)]);
        expectedTrial = snapshot.Trial+1;
    end
catch err
    session.Error = string(err.message);
    if strlength(session.StopReason)==0, session.StopReason = "Error"; end
    warning('LeverBatch:SessionError','%s. Device data is not silently discarded.',err.message);
end
release();
session.End = datetime('now'); session.StopAcknowledged = stopAcknowledged;
session.SerialBytes = logBytes; session.SerialLines = logLines;
try
    checkpoint();
    writetable(results,fullfile(folder,settings.outFileBase+"_trials.csv"));
    writetable(pressEvents,fullfile(folder,settings.outFileBase+"_presses.csv"));
catch exportError
    session.Error = session.Error+" Export: "+string(exportError.message);
    warning('LeverBatch:Export','Export failed: %s. Do not clear retained device data.',exportError.message);
end
fprintf('Session output: %s\n',folder);
if strlength(session.Error)>0, fprintf('Session error: %s\n',session.Error); end

    function yes = expired()
        yes = ~isempty(sessionClock) && toc(sessionClock)>=settings.sessionMinutes*60;
    end
    function [header,stopped] = awaitBatch(expected)
        header = []; stopped = false;
        writeline(s,"GET_BATCH");
        timer = tic; stopTimer = [];
        budget = settings.trialSeconds+settings.interTrialSeconds+ ...
            session.DeviceSolenoidMs/1000+settings.serialStallGraceSeconds;
        while true
            drawnow limitrate;
            if isempty(stopTimer) && expired()
                session.StopReason = "Session time limit";
                writeline(s,"STOP"); stopTimer = tic; stopped = true;
            end
            if isempty(stopTimer) && toc(timer)>budget
                error('LeverBatch:Timeout','No completed batch by expected trial deadline.');
            end
            if ~isempty(stopTimer) && toc(stopTimer)>settings.ackTimeout
                error('LeverBatch:Timeout','STOP was not acknowledged.');
            end
            line = receive();
            if line=="", pause(0.001); continue; end
            tokens = split(line,",");
            if tokens(1)=="BATCH_READY"
                candidate = parseHeader(tokens);
                if isempty(stopTimer)
                    assert(candidate.Trial==expected,'Unexpected batch trial number.');
                    header = candidate; return;
                end
                readyBeforeStop = candidate;
            elseif tokens(1)=="ACK_STOP"
                stopAcknowledged = true; stopped = true;
                tokens = request("GET_BATCH","BATCH",settings.ackTimeout);
                if tokens(1)=="BATCH_READY"
                    candidate = parseHeader(tokens);
                    if candidate.Trial==expected
                        header = candidate;
                    else
                        assert(candidate.Acknowledged && candidate.Trial<expected, ...
                            'Unexpected retained trial after STOP.');
                    end
                elseif tokens(1)~="BATCH_EMPTY"
                    error('Unexpected batch state after STOP.');
                else
                    assert(numel(tokens)==1,'Malformed BATCH_EMPTY response.');
                end
                return;
            else
                checkControllerError(tokens);
            end
        end
    end
    function snapshot = fetchBatch(header)
        id = string(header.Trial);
        meta = request("GET_META,"+id,"META",settings.ackTimeout);
        assert(numel(meta)==4,'Malformed META.'); m = unsignedValues(meta(2:end));
        assert(m(1)==header.Trial && all(m(2:3)<2^32),'Mismatched META trial/clock.');
        counts = request("GET_COUNTS,"+id,"COUNTS",settings.ackTimeout);
        assert(numel(counts)==5,'Malformed COUNTS.'); c = unsignedValues(counts(2:end));
        assert(c(1)==header.Trial,'Mismatched COUNTS trial.');
        window = request("GET_WINDOW,"+id,"WINDOW",settings.ackTimeout);
        assert(numel(window)==4,'Malformed WINDOW.'); w = unsignedValues(window(2:end));
        assert(w(1)==header.Trial,'Mismatched WINDOW trial.');
        context = request("GET_CONTEXT,"+id,"CONTEXT",settings.ackTimeout);
        assert(numel(context)==5 && str2double(context(2))==header.Trial,'Malformed CONTEXT.');
        pulse = unsignedValues(context(5));
        words = zeros(header.StoredPresses,1); first = 1;
        while first<=header.StoredPresses
            frame = request("GET_PRESS,"+id+","+string(first),"PRESS_DATA",settings.ackTimeout);
            assert(numel(frame)>=4 && numel(frame)<=7,'Malformed press-data page.');
            v = unsignedValues(frame(2:end));
            assert(v(1)==header.Trial && v(2)==first,'Mismatched press-data page.');
            count = numel(v)-2;
            assert(count==min(4,header.StoredPresses-first+1),'Unexpected press-data page size.');
            words(first:first+count-1) = v(3:end); first = first+count;
        end
        snapshot = struct('Trial',header.Trial,'StoredPresses',header.StoredPresses, ...
            'DroppedPresses',header.DroppedPresses,'Completed',header.Completed, ...
            'WaterDelivered',header.WaterDelivered,'Acknowledged',header.Acknowledged, ...
            'ArduinoStartMs',m(2),'ArduinoEndMs',m(3),'Lever1Presses',c(2), ...
            'Lever2Presses',c(3),'ActivePresses',c(4),'WindowStartMs',w(2), ...
            'WindowEndMs',w(3),'TaskMode',context(3),'ActiveLevers',context(4), ...
            'SolenoidMs',pulse,'PackedPresses',words,'HostReadoutAt',datetime('now'));
    end
    function acknowledge(trial)
        request("ACK_BATCH,"+string(trial),"ACK_BATCH",settings.ackTimeout,["ACK_BATCH",string(trial)]);
    end
    function tokens = request(cmd,prefix,timeout,exact)
        if nargin<4, exact = strings(0,1); end
        writeline(s,cmd);
        tokens = waitReply(prefix,timeout,false);
        if ~isempty(exact)
            assert(isequal(tokens(:),string(exact(:))),'Acknowledgment does not match requested settings.');
        end
    end
    function tokens = waitReply(prefix,timeout,allowReady)
        timer = tic;
        while toc(timer)<timeout
            line = receive();
            if line=="", pause(0.001); continue; end
            tokens = split(line,",");
            matches = tokens(1)==prefix || (prefix=="BATCH" && ismember(tokens(1),["BATCH_EMPTY","BATCH_READY"]));
            if matches, return; end
            if allowReady && tokens(1)=="READY", continue; end
            checkControllerError(tokens);
            % Harmless late startup/STOP replies can follow a connection retry.
            assert(ismember(tokens(1),["ACK_STOP","READY"]),'Unexpected reply while waiting for %s: %s',prefix,line);
            if tokens(1)=="READY" && ~allowReady, error('Controller reset during command.'); end
        end
        error('LeverBatch:Timeout','No reply for %s. Use the matching BATCH firmware.',prefix);
    end
    function line = receive()
        available = s.NumBytesAvailable;
        if available>0
            bytes = read(s,available,'uint8'); rx = [rx char(bytes(:)')];
            logBytes = logBytes+numel(bytes);
        end
        assert(numel(rx)<=65536,'Serial receive backlog is invalid.');
        line = ""; pos = find(rx==char(10),1);
        if isempty(pos), return; end
        line = string(strtrim(rx(1:pos-1))); rx = rx(pos+1:end); logLines = logLines+1;
        if serialLog>=0 && line~="", fprintf(serialLog,'%s\t%s\n',char(datetime('now')),char(line)); end
    end
    function stopController()
        if isempty(s) || stopAcknowledged, return; end
        writeline(s,"STOP"); timer = tic;
        while toc(timer)<settings.ackTimeout
            line = receive();
            if line=="", pause(0.001); continue; end
            tokens = split(line,",");
            if tokens(1)=="ACK_STOP", stopAcknowledged = true; return; end
            if tokens(1)=="BATCH_READY", readyBeforeStop = parseHeader(tokens); end
        end
        warning('LeverBatch:StopTimeout','STOP acknowledgment was not received.');
    end
    function checkpoint()
        droppedTotal = 0;
        if ~isempty(trialInfo), droppedTotal = sum([trialInfo.DroppedPresses]); end
        pressEventInfo = struct('ProtocolVersion',2,'Transport',"BATCH",'Enabled',true, ...
            'MessagesValid',strlength(session.Error)==0,'ClockOriginValid',isfinite(firstTrialArduinoMs), ...
            'TimestampResolutionMs',1,'DebounceMs',10, ...
            'CaptureScope',"In-trial presses only; no intertrial timestamps", ...
            'CapacityPerTrial',session.DeviceCapacity,'DeviceSolenoidMs',session.DeviceSolenoidMs, ...
            'TrialCountsMatch',isempty(trialInfo) || all([trialInfo.TrialCountsMatch]), ...
            'EventsComplete',isempty(trialInfo) || all([trialInfo.EventsComplete]), ...
            'DroppedPresses',droppedTotal,'StopAcknowledged',stopAcknowledged, ...
            'SessionStartArduinoMs',firstTrialArduinoMs,'SessionClockOrigin',"First trial/buzzer onset", ...
            'IntertrialConvention',"Not captured in BATCH mode", ...
            'HostTimeMeaning',"HostReceivedAt/HostReadoutAt are batch retrieval times; trial host times are NaT");
        session.SerialBytes = logBytes; session.SerialLines = logLines;
        payload = struct('results',results,'pressEvents',pressEvents,'pressEventInfo',pressEventInfo, ...
            'session',session,'settings',settings,'trialBatches',trialBatches,'trialInfo',trialInfo, ...
            'sessionStart',session.Start,'actualSessionEnd',session.End,'sessionMinutes',settings.sessionMinutes, ...
            'sessionTaskMode',settings.taskMode,'sessionActiveLevers',settings.activeLevers, ...
            'sessionTrialSeconds',settings.trialSeconds,'sessionInterTrialSeconds',settings.interTrialSeconds, ...
            'sessionBuzzerSeconds',settings.buzzerSeconds,'sessionBuzzerFrequencyHz',settings.buzzerFrequencyHz, ...
            'stopAcknowledged',stopAcknowledged,'stopReason',session.StopReason);
        if settings.taskMode=="WINDOW"
            payload.sessionWindowStartRangeSeconds = settings.windowStartRangeMs/1000;
            payload.sessionWindowLengthRangeSeconds = settings.windowLengthRangeMs/1000;
            payload.sessionRandomSeed = settings.sessionRandomSeed;
            payload.sessionMaxWaterDeliveries = settings.maxWaterDeliveries;
        end
        durableSave(matFile,payload);
    end
    function release()
        if ~isempty(s)
            try stopController(); catch, end
            try delete(s); catch, end
            s = [];
        end
        if ~isempty(vid)
            try closepreview(vid); catch, end
            try
                stop(vid); timer = tic;
                while settings.recordVideo && vid.DiskLoggerFrameCount<vid.FramesAcquired && toc(timer)<15
                    pause(0.05);
                end
            catch
            end
            try delete(vid); catch, end
            vid = [];
        end
        if ~isempty(writer), try close(writer); catch, end; writer = []; end
        if serialLog>=0, fclose(serialLog); serialLog = -1; end
    end
end

function header = parseHeader(tokens)
    assert(numel(tokens)==7 && tokens(1)=="BATCH_READY",'Malformed BATCH_READY.');
    v = unsignedValues(tokens(2:end));
    assert(v(1)>=1 && v(2)<=1024 && all(ismember(v(4:6),[0 1])),'Invalid batch header.');
    header = struct('Trial',v(1),'StoredPresses',v(2),'DroppedPresses',v(3), ...
        'Completed',v(4),'WaterDelivered',v(5),'Acknowledged',v(6));
end
function values = unsignedValues(tokens)
    values = str2double(tokens(:));
    assert(all(isfinite(values)) && all(values>=0) && all(values==fix(values)) && all(values<2^32), ...
        'Invalid numeric protocol fields.');
end
function checkControllerError(tokens)
    if tokens(1)=="ERROR", error('LeverBatch:Controller','Arduino: %s',join(tokens,",")); end
    if tokens(1)=="READY", error('LeverBatch:Reset','Controller reset; retained RAM data may be lost.'); end
end
function durableSave(filename,payload)
    temporary = string(filename)+".pending.mat";
    save(temporary,'-struct','payload','-v7');
    [ok,message] = movefile(temporary,filename,'f');
    assert(ok,'Cannot finalize checkpoint: %s',message);
end
function c = validateSettings(c)
    c.taskMode = string(c.taskMode); c.activeLevers = upper(strtrim(string(c.activeLevers)));
    assert(ismember(c.taskMode,["TRIAL","WINDOW"]),'Batch mode supports TRIAL and WINDOW.');
    assert(isscalar(c.activeLevers) && ismember(c.activeLevers,["LEVER1","LEVER2","BOTH"]),'Invalid active levers.');
    assert(c.saveResults,'Batch mode requires saveResults=true for checkpoint-before-ACK.');
    assert(c.baudRate==115200,'Firmware baud rate must be 115200.');
    for name = ["sessionMinutes","readyTimeout","ackTimeout","serialStallGraceSeconds"]
        validateattributes(c.(name),{'numeric'},{'scalar','finite','positive'});
    end
    validateattributes(c.trialSeconds,{'numeric'},{'scalar','finite','>=',0.001,'<=',3600});
    validateattributes(c.interTrialSeconds,{'numeric'},{'scalar','finite','>=',0,'<=',3600});
    validateattributes(c.buzzerSeconds,{'numeric'},{'scalar','finite','>=',0,'<=',c.trialSeconds});
    validateattributes(c.buzzerFrequencyHz,{'numeric'},{'scalar','integer','>=',31,'<=',20000});
    validateattributes(c.maxWaterDeliveries,{'numeric'},{'scalar','positive'});
    assert(~isnan(c.maxWaterDeliveries) && (isinf(c.maxWaterDeliveries) || ...
        c.maxWaterDeliveries==fix(c.maxWaterDeliveries)),'Invalid reward cap.');
    assert(~isempty(regexp(char(c.outFileBase),'^[A-Za-z0-9_-]+$','once')),'Invalid output file base.');
    assert(~isempty(regexp(char(c.mouseID),'^[A-Za-z0-9_-]+$','once')),'Invalid mouse ID.');
    for name = ["trialSeconds","interTrialSeconds","buzzerSeconds"]
        c.(name) = round(1000*c.(name))/1000;
    end
    if c.recordVideo || c.showVideoPreview
        validateattributes(c.videoDevice,{'numeric'},{'scalar','integer','positive'});
        validateattributes(c.savedVideoFrameRate,{'numeric'},{'scalar','finite','positive'});
    end
    if c.taskMode=="WINDOW"
        validateattributes(c.windowStartRangeSeconds,{'numeric'},{'row','numel',2,'finite','nonnegative'});
        validateattributes(c.activeWindowLengthRangeSeconds,{'numeric'},{'row','numel',2,'finite','>=',0.001});
        c.windowStartRangeMs = round(1000*c.windowStartRangeSeconds);
        c.windowLengthRangeMs = round(1000*c.activeWindowLengthRangeSeconds);
        assert(diff(c.windowStartRangeMs)>=0 && diff(c.windowLengthRangeMs)>=0 && ...
            c.windowStartRangeMs(2)+c.windowLengthRangeMs(2)<=1000*c.trialSeconds, ...
            'Latest window start + longest window length must fit within trialSeconds.');
        if isempty(c.randomSeed)
            old = rng; rng('shuffle'); c.sessionRandomSeed = randi(2147483646); rng(old);
        else
            validateattributes(c.randomSeed,{'numeric'},{'scalar','integer','>=',1,'<=',2147483646});
            c.sessionRandomSeed = double(c.randomSeed);
        end
    end
end
