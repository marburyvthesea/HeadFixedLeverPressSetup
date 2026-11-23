function test_serial_reader
% Run in MATLAB: addpath('tests'); test_serial_reader
% Extract the exact local function from each standalone script, avoiding a
% copied implementation in this test. No board, camera or toolbox required.
    root = fileparts(fileparts(mfilename('fullpath')));
    files = {'runRandomActiveWindowLeverSession.m','runBuzzerLeverSessionVideoRecord_unified.m'};
    for k=1:numel(files), checkScript(root,files{k}); end
    fprintf('Both serial readers passed fragmented/batched/reset/backlog checks.\n');
end
function checkScript(root,name)
    source = fileread(fullfile(root,name));
    marker = 'function [line,diagnostics] = readAvailableSerialLine';
    start = strfind(source,marker); assert(isscalar(start));
    fixture = tempname; mkdir(fixture);
    oldPath = path;
    cleanup = onCleanup(@() releaseFixture(oldPath,fixture));
    fid = fopen(fullfile(fixture,'readAvailableSerialLine.m'),'w');
    assert(fid>=0); fwrite(fid,source(start:end)); fclose(fid);
    addpath(fixture); addpath(fullfile(root,'tests'));
    clear readAvailableSerialLine
    s = MockSerialInput;
    [line,d] = readAvailableSerialLine(s,true);
    assert(line=="" && d.BytesReceived==0 && s.ReadCalls==0);
    s.append('PRESS_EVENT,1,2,');
    [line,d] = readAvailableSerialLine(s);
    assert(line=="" && d.BufferedBytes>0 && ~d.HasCompleteLine);
    calls = s.ReadCalls;
    for k=1:5
        line = readAvailableSerialLine(s); assert(line=="");
    end
    assert(s.ReadCalls==calls); % Empty/partial data must not cause another read.
    s.append(sprintf('100,20,1,1,1,TRIAL\r\nACK_STOP\n'));
    [line,d] = readAvailableSerialLine(s);
    assert(line=="PRESS_EVENT,1,2,100,20,1,1,1,TRIAL" && d.HasCompleteLine);
    calls = s.ReadCalls;
    [line,d] = readAvailableSerialLine(s);
    assert(line=="ACK_STOP" && s.ReadCalls==calls && d.BufferedBytes==0);
    % Startup ACK and trial-start in the SAME transfer cannot lose the start.
    readAvailableSerialLine(s,true);
    s.append(sprintf('ACK_START\nTRIAL_START,1,42\n'));
    assert(readAvailableSerialLine(s)=="ACK_START");
    assert(readAvailableSerialLine(s)=="TRIAL_START,1,42");
    % Restart drops an incomplete prior record instead of joining sessions.
    s.append('unfinished'); assert(readAvailableSerialLine(s)=="");
    [line,d] = readAvailableSerialLine(s,true);
    assert(line=="" && d.BufferedBytes==0 && d.LinesReceived==0);
    % Preserve arbitrary one-byte fragmentation and output ordering.
    text = sprintf('ONE\nTWO\n'); actual = strings(0,1);
    for k=1:numel(text)
        s.append(text(k)); line = readAvailableSerialLine(s);
        if line~="", actual(end+1,1)=line; end
    end
    assert(isequal(actual,["ONE";"TWO"]));
    s.append(repmat('x',1,1048577));
    failed = false;
    try readAvailableSerialLine(s); catch, failed=true; end
    assert(failed,'Receive backlog must be bounded.');
    fprintf('%s passed.\n',name);
end
function releaseFixture(oldPath,fixture)
    clear readAvailableSerialLine
    path(oldPath);
    if isfolder(fixture), rmdir(fixture,'s'); end
end
