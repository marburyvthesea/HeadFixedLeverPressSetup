%% test
serialportlist('available')
%% arduino parameters
port = 'COM4'; 
baudRate = 115200; 
arduinoSerial = serialport(port, baudRate);



%%
flush(arduinoSerial);

rawData = strtrim(readline(arduinoSerial));
%%
flush(arduinoSerial);
