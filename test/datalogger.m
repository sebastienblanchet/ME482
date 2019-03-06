%
% Generic Data Logger for Symposium
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
params.therm1 = 'A0';
params.therm2 = 'A1';
params.Tnom = 25;
params.Bcoeff = 3950;
params.R0 = 100e3;
params.Vcc = 5;
params.name = datestr(now,'dd_mm_yy_HHMMSS');
params.sec = 60 * params.mins;
params.Tsamp = 1 / params.fhz;

% Make serial connection with device
a = arduino;
% a = arduino('/dev/cu.usbmodem143401');

%% Acquire and display live data
figure(1)
hold on 

h1 = animatedline;
h2 = animatedline;
ax = gca;
ax.YGrid = 'on';
ax.YLim = [15 150];

stop = false;

startTime = datetime('now');

while ~stop

    % Read current voltage value    
    V1 = readVoltage(a, params.therm1);
    V2 = readVoltage(a, params.therm2);

    % Calculate temperature
    Tplate1 = thermistorTemp(V1, params)
    Tplate2 = thermistorTemp(V2, params)
       
    T1 = [T1, Tplate1];
    T2 = [T2, Tplate2];

    % Get current time
    t =  datetime('now') - startTime;
    % Add points to animation
    addpoints(h1, datenum(t),T1)
    addpoints(h2, datenum(t),T2)

    % Update axes
    ax.XLim = datenum([t-seconds(15) t]);
    datetick('x','keeplimits')
    drawnow

    % Check stop condition
    stop = readDigitalPin(a,'D12');
end
