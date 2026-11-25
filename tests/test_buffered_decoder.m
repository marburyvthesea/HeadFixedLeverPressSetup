function test_buffered_decoder
% MATLAB-only data checks; no board, camera or toolbox required.
root = fileparts(fileparts(mfilename('fullpath')));
oldPath = path; restore = onCleanup(@() path(oldPath)); addpath(root);
s = snapshot("WINDOW",[0;401;1000;1999]); % L1@0, L2@200, L1@500, L2@999
s.WindowStartMs = 200; s.WindowEndMs = 500;
s.Lever1Presses = 2; s.Lever2Presses = 2; s.ActivePresses = 1; s.WaterDelivered = 1;
[r,e,info] = decodeBufferedLeverTrial(s,1000);
assert(isequal(e.LeverID,[1;2;1;2]));
assert(isequal(e.TrialElapsedMs,[0;200;500;999]));
assert(isequal(e.Phase,["EARLY";"WINDOW";"LATE";"LATE"]));
assert(isequal(e.Qualifying,[false;true;false;false]));
assert(all(isnat([r.TrialStart r.TrialEnd])) && r.Completed && info.EventsComplete);
assert(isequal(e.ArduinoMs,[1000;1200;1500;1999]));
% Maximum retained batch; both levers selected, all in full-trial scope.
words = 2*(0:1023)' + mod((0:1023)',2);
s = snapshot("TRIAL",words); s.ArduinoEndMs = 3000;
s.ActiveLevers = "BOTH"; s.Lever1Presses = 512; s.Lever2Presses = 512;
s.ActivePresses = 1024; s.WaterDelivered = 1;
[r,e,info] = decodeBufferedLeverTrial(s);
assert(height(e)==1024 && all(e.Qualifying) && all(e.InTrial));
assert(r.CapturedPresses==1024 && info.TrialCountsMatch);
% Raw millis wraps; relative event times must remain correct.
s = snapshot("TRIAL",[0;401]); s.ArduinoStartMs = 2^32-50; s.ArduinoEndMs = 950;
s.ActiveLevers = "BOTH"; s.Lever1Presses = 1; s.Lever2Presses = 1;
s.ActivePresses = 2; s.WaterDelivered = 1;
[~,e] = decodeBufferedLeverTrial(s,s.ArduinoStartMs);
assert(isequal(e.ArduinoMs,[2^32-50;150]));
assert(isequal(e.SessionTimeSeconds,[0;0.2]));
% A press-free abort is distinct from a successfully completed trial.
s = snapshot("TRIAL",zeros(0,1)); s.Completed = 0; s.ArduinoEndMs = s.ArduinoStartMs;
[r,e,info] = decodeBufferedLeverTrial(s);
assert(~r.Completed && r.WaterDelivered==0 && height(e)==0 && info.EventsComplete);
% Explicit overflow retains known timestamps and raw device counts.
s = snapshot("TRIAL",2*(0:1023)'); s.ArduinoEndMs = 2024;
s.Completed = 0; s.DroppedPresses = 1; s.ActiveLevers = "LEVER1";
s.Lever1Presses = 1025; s.ActivePresses = 1025;
[r,e,info] = decodeBufferedLeverTrial(s);
assert(height(e)==1024 && r.PressCount==1025 && ~info.EventsComplete && r.DroppedPresses==1);
% Count corruption without a declared loss must not be acknowledged by caller.
s = snapshot("TRIAL",0); s.Lever1Presses = 2;
expectFailure(@() decodeBufferedLeverTrial(s));
% An event at the completed trial deadline is outside the interval.
s = snapshot("TRIAL",2000); s.Lever1Presses = 1;
expectFailure(@() decodeBufferedLeverTrial(s));
fprintf('Buffered press decoder checks passed.\n');
end
function s = snapshot(mode,words)
s = struct('Trial',1,'StoredPresses',numel(words),'DroppedPresses',0, ...
    'Completed',1,'WaterDelivered',0,'Acknowledged',0,'ArduinoStartMs',1000, ...
    'ArduinoEndMs',2000,'Lever1Presses',0,'Lever2Presses',0,'ActivePresses',0, ...
    'WindowStartMs',0,'WindowEndMs',1000,'TaskMode',mode,'ActiveLevers',"LEVER2", ...
    'SolenoidMs',325,'PackedPresses',words,'HostReadoutAt',datetime('now'));
end
function expectFailure(call)
failed = false; try call(); catch, failed = true; end
assert(failed,'Invalid snapshot should fail validation before ACK.');
end
