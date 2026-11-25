classdef MockBufferedController < handle
    % STOP/config/BATCH-only board fixture. It never opens a physical port.
    properties
        Rx = uint8([])
        TaskMode = "TRIAL"
        ActiveLevers = "LEVER2"
        TrialMs = 1000
        PulseMs = 325
        Trial = 0
        HasBatch = false
        BatchAcknowledged = false
        FailureCommand = ""
        CheckpointRoot = ""
        Timeout = 1
        Port = "MOCK_PORT"
        UserData = []
    end
    properties (Dependent)
        NumBytesAvailable
    end
    methods
        function obj = MockBufferedController(root,failureCommand)
            obj.CheckpointRoot = string(root);
            if nargin>1, obj.FailureCommand = string(failureCommand); end
        end
        function count = get.NumBytesAvailable(obj)
            count = numel(obj.Rx);
        end
        function configureTerminator(~,~)
        end
        function setDTR(~,~)
        end
        function flush(obj,varargin)
            obj.Rx = uint8([]);
        end
        function bytes = read(obj,count,kind)
            assert(strcmp(kind,'uint8') && count<=numel(obj.Rx));
            bytes = obj.Rx(1:count); obj.Rx(1:count) = [];
        end
        function write(obj,bytes,kind)
            assert(strcmp(kind,'uint8'));
            obj.writeline(string(strtrim(char(bytes(:)'))));
        end
        function writeline(obj,line)
            global LEVER_BATCH_TEST_LOG
            line = string(line); fields = split(line,","); kind = fields(1);
            LEVER_BATCH_TEST_LOG.Commands(end+1,1) = line;
            if strlength(obj.FailureCommand)>0 && kind==obj.FailureCommand
                obj.reply("ERROR,INJECTED_READOUT_FAILURE"); return;
            end
            switch kind
                case "STOP"
                    LEVER_BATCH_TEST_LOG.StopCommands = LEVER_BATCH_TEST_LOG.StopCommands+1;
                    obj.reply("ACK_STOP");
                case "GET_BATCH"
                    if ~obj.HasBatch
                        obj.reply("BATCH_EMPTY");
                    else
                        obj.reply(sprintf('BATCH_READY,%d,2,0,1,1,%d',obj.Trial,obj.BatchAcknowledged));
                    end
                case "SET_TASK"
                    obj.TaskMode = fields(2); obj.reply("ACK_TASK,"+fields(2));
                case "SET_ACTIVE"
                    obj.ActiveLevers = fields(2); obj.reply("ACK_ACTIVE,"+fields(2));
                case "SET_TRIAL"
                    obj.TrialMs = str2double(fields(2));
                    obj.reply("ACK_TRIAL,"+join(fields(2:end),","));
                case "SET_WINDOW"
                    obj.reply("ACK_WINDOW,"+join(fields(2:end),","));
                case "SET_PRESS_LOG"
                    assert(fields(2)=="BATCH");
                    obj.reply(sprintf('ACK_PRESS_LOG,BATCH,1024,%d',obj.PulseMs));
                case "START"
                    obj.Trial = 1; obj.HasBatch = true; obj.BatchAcknowledged = false;
                    obj.reply("ACK_START");
                case "CONTINUE"
                    assert(obj.BatchAcknowledged,'CONTINUE preceded durable checkpoint and ACK.');
                    last = obj.Trial; obj.Trial = obj.Trial+1; obj.BatchAcknowledged = false;
                    obj.reply("ACK_CONTINUE,"+string(last));
                case "GET_META"
                    obj.checkTrial(fields);
                    start = (obj.Trial-1)*2000+1000;
                    obj.reply(sprintf('META,%d,%d,%d',obj.Trial,start,start+obj.TrialMs));
                case "GET_COUNTS"
                    obj.checkTrial(fields); obj.reply(sprintf('COUNTS,%d,1,1,1',obj.Trial));
                case "GET_WINDOW"
                    obj.checkTrial(fields); obj.reply(sprintf('WINDOW,%d,0,%d',obj.Trial,obj.TrialMs));
                case "GET_CONTEXT"
                    obj.checkTrial(fields);
                    obj.reply(sprintf('CONTEXT,%d,%s,%s,%d',obj.Trial,obj.TaskMode,obj.ActiveLevers,obj.PulseMs));
                case "GET_PRESS"
                    obj.checkTrial(fields); assert(str2double(fields(3))==1);
                    obj.reply(sprintf('PRESS_DATA,%d,1,0,201',obj.Trial));
                case "ACK_BATCH"
                    obj.checkTrial(fields);
                    % Exercise the actual runner's durable-save-before-ACK contract.
                    files = dir(fullfile(obj.CheckpointRoot,'**','*_batch.mat'));
                    assert(numel(files)==1,'ACK_BATCH sent without one finalized MAT checkpoint.');
                    saved = load(fullfile(files(1).folder,files(1).name));
                    assert(height(saved.results)==obj.Trial,'Trial structs did not accumulate before ACK.');
                    assert(height(saved.pressEvents)==2*obj.Trial,'Press rows were not checkpointed before ACK.');
                    assert(numel(saved.trialBatches)==obj.Trial && numel(saved.trialInfo)==obj.Trial, ...
                        'Snapshot/info structures were not checkpointed.');
                    assert(numel(saved.session.AcknowledgedTrials)==obj.Trial-1, ...
                        'Checkpoint must be finalized before this trial is acknowledged.');
                    LEVER_BATCH_TEST_LOG.CheckpointTrials(end+1,1) = obj.Trial;
                    LEVER_BATCH_TEST_LOG.CheckpointRows(end+1,1) = height(saved.results);
                    LEVER_BATCH_TEST_LOG.CheckpointPressRows(end+1,1) = height(saved.pressEvents);
                    LEVER_BATCH_TEST_LOG.AcknowledgedTrials(end+1,1) = obj.Trial;
                    obj.BatchAcknowledged = true;
                    obj.reply("ACK_BATCH,"+string(obj.Trial));
                otherwise
                    error('Unexpected command in test fixture: %s',line);
            end
        end
        function delete(~)
            global LEVER_BATCH_TEST_LOG
            if isstruct(LEVER_BATCH_TEST_LOG) && isfield(LEVER_BATCH_TEST_LOG,'SerialDeleted')
                LEVER_BATCH_TEST_LOG.SerialDeleted = true;
            end
        end
    end
    methods (Access=private)
        function reply(obj,line)
            obj.Rx = [obj.Rx uint8(char(string(line))) uint8(13) uint8(10)];
        end
        function checkTrial(obj,fields)
            assert(obj.HasBatch && str2double(fields(2))==obj.Trial,'Wrong requested device trial.');
        end
    end
end
