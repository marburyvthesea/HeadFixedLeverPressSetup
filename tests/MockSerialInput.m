classdef MockSerialInput < handle
    % Byte source for exercising the actual scripts' nonblocking LF reader.
    properties
        Bytes = uint8([])
        ReadCalls = 0
    end
    properties (Dependent)
        NumBytesAvailable
    end
    methods
        function value = get.NumBytesAvailable(obj)
            value = numel(obj.Bytes);
        end
        function append(obj,text)
            obj.Bytes = [obj.Bytes uint8(char(text))];
        end
        function value = read(obj,count,kind)
            assert(strcmp(kind,'uint8'));
            assert(count<=numel(obj.Bytes),'Reader requested unavailable bytes.');
            obj.ReadCalls = obj.ReadCalls+1;
            value = obj.Bytes(1:count); obj.Bytes(1:count) = [];
        end
    end
end
