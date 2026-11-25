function test_buffered_runner
% Run actual session helper against a temporary serialport factory; no hardware.
% MATLAB only: addpath('tests'); test_buffered_runner
% The factory is materialized in a temporary directory, never placed on the
% repository MATLAB path, so adding tests to the path cannot shadow serialport.
    checkRun("");
    checkRun("GET_COUNTS");
    fprintf('Actual buffered runner accumulation/checkpoint/error-cleanup checks passed.\n');
end
function checkRun(failureCommand)
    global LEVER_BATCH_TEST_CONTROLLER LEVER_BATCH_TEST_LOG
    root = fileparts(fileparts(mfilename('fullpath')));
    fixture = tempname; mkdir(fixture);
    output = fullfile(fixture,'output'); mkdir(output);
    oldPath = path;
    restore = onCleanup(@() releaseFixture(oldPath,fixture));
    factory = fullfile(fixture,'serialport.m');
    fid = fopen(factory,'w'); assert(fid>=0);
    fprintf(fid,'%s\n', ...
        'function object = serialport(port,varargin)', ...
        'global LEVER_BATCH_TEST_CONTROLLER', ...
        'assert(string(port)=="MOCK_PORT","Test factory refuses physical ports.");', ...
        'object = LEVER_BATCH_TEST_CONTROLLER;', ...
        'end');
    fclose(fid);
    addpath(root); addpath(fullfile(root,'tests')); addpath(fixture,'-begin');
    clear serialport
    assert(strcmp(which('serialport'),factory),'Temporary mock factory did not take precedence.');
    LEVER_BATCH_TEST_LOG = struct('Commands',strings(0,1),'StopCommands',0, ...
        'SerialDeleted',false,'CheckpointTrials',zeros(0,1),'CheckpointRows',zeros(0,1), ...
        'CheckpointPressRows',zeros(0,1),'AcknowledgedTrials',zeros(0,1));
    LEVER_BATCH_TEST_CONTROLLER = MockBufferedController(output,failureCommand);
    settings = struct('taskMode',"TRIAL",'portName',"MOCK_PORT",'baudRate',115200, ...
        'sessionMinutes',1,'readyTimeout',2,'ackTimeout',1,'serialStallGraceSeconds',1, ...
        'saveResults',true,'outFileBase',"test",'activeLevers',"LEVER2", ...
        'trialSeconds',1,'interTrialSeconds',0,'buzzerSeconds',0,'buzzerFrequencyHz',4000, ...
        'videoFolder',string(output),'mouseID',"mock",'recordVideo',false, ...
        'showVideoPreview',false,'videoAdaptor',"unused",'videoDevice',1, ...
        'videoFormat',"unused",'cameraFrameRate',"",'savedVideoFrameRate',113, ...
        'maxWaterDeliveries',2);
    lastwarn('');
    transcript = evalc('[results,events,session] = runBufferedLeverSession(settings);');
    [warningText,~] = lastwarn;
    forbidden = ["already-destroyed","already destroyed", ...
        "error while executing handle class delete method", "error while executing cleanup"];
    assert(~any(contains(lower(string(transcript)),forbidden)), ...
        'Cleanup accessed an already-destroyed workspace.');
    assert(~any(contains(lower(string(warningText)),forbidden)));
    assert(LEVER_BATCH_TEST_LOG.SerialDeleted,'Session serial object was not released.');
    assert(LEVER_BATCH_TEST_LOG.StopCommands>=2,'Session did not stop during cleanup.');
    if failureCommand==""
        assert(strlength(session.Error)==0,'Actual runner failed: %s',session.Error);
        assert(height(results)==2 && height(events)==4);
        assert(isequal(LEVER_BATCH_TEST_LOG.AcknowledgedTrials,[1;2]));
        assert(isequal(LEVER_BATCH_TEST_LOG.CheckpointRows,[1;2]));
        assert(isequal(LEVER_BATCH_TEST_LOG.CheckpointPressRows,[2;4]));
        saved = load(session.MatFile);
        assert(numel(saved.trialBatches)==2 && numel(saved.trialInfo)==2);
        assert(saved.stopAcknowledged && saved.session.StopAcknowledged);
        fprintf('Two-trial actual-runner test passed.\n');
    else
        assert(contains(session.Error,"INJECTED_READOUT_FAILURE"),'Injected error was not preserved.');
        assert(isempty(LEVER_BATCH_TEST_LOG.AcknowledgedTrials), ...
            'Malformed/partial trial data was acknowledged.');
        assert(~any(startsWith(LEVER_BATCH_TEST_LOG.Commands,"CONTINUE")), ...
            'Runner continued after a failed readout.');
        fprintf('Injected partial-readout/no-ACK/cleanup test passed.\n');
    end
end
function releaseFixture(oldPath,fixture)
    global LEVER_BATCH_TEST_CONTROLLER LEVER_BATCH_TEST_LOG
    try
        if ~isempty(LEVER_BATCH_TEST_CONTROLLER) && isvalid(LEVER_BATCH_TEST_CONTROLLER)
            delete(LEVER_BATCH_TEST_CONTROLLER);
        end
    catch
    end
    LEVER_BATCH_TEST_CONTROLLER = [];
    LEVER_BATCH_TEST_LOG = [];
    path(oldPath); clear serialport
    if isfolder(fixture), rmdir(fixture,'s'); end
end
