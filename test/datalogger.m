%
% Generic Data Logger Routine Symposium
% @Author: Seb Blanchet
% 

%% Init
clear variables
clc
close all

%% Constants
params.mins = 10;
params.fhz = 2;
params.Tmax = 200;
params.Tnom = 25;
params.Bcoeff = 3950;
params.R0 = 100e3;
params.Vcc = 5;
params.sec = 60 * params.mins;
params.Tsamp = 1 / params.fhz;
params.maxval = 150;
params.minval = 15;
params.IsensGainApV = 10;
params.IsensOffsetA = 25;
% params.id = 'Temperature $T$ [$^o$C]';
params.id = 'Current $I$ [A]';

% Make serial connection with device
a = arduino;
% a = arduino('/dev/cu.usbmodem143401');

port = 'A5'

dataLog(a, port, @currentSensA, params)