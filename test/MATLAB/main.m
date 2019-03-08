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
params.fhz = 5;
params.Tmax = 200;
params.Tnom = 25;
params.Bcoeff = 3950;
params.R0 = 100e3;
params.Vcc = 5;
params.sec = 60 * params.mins;
params.Tsamp = 1 / params.fhz;

% params.maxval = 150; % Temp
% params.minval = 25;  % Temp
% params.id = 'Temperature $T$ [$^o$C]';  % Temp

params.maxval = 10; % Isens
params.minval = -10;  % Isens
params.m = 10; % Isens
params.b = -25; % Isens
params.id = 'Current $I$ [A]'; % Isens

% params.maxval = 6; % Joystick
% params.minval = -1; % Joystick
% params.m = 1; % Joystick
% params.b = 0; % Joystick
% params.id = 'Voltage $V$ [V]'; % Joystick

% Make serial connection with device
% a = arduino;

a = arduino;
port = 'A3';

dataLog(a, port, @ymxb, params)