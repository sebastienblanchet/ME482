%
% @Author: Seb Blanchet
% @Date:   2019-01-31 10:49:18
% @Last Modified by:   Seb Blanchet
% @Last Modified time: 2019-01-31 10:49:20
% 

% Init
clear variables
clc
close all

% Define constants
params.mins = 10;
params.fhz = 2;
params.Tmax = 200;
params.therm1 = 'A0';
params.therm2 = 'A1';
params.Tnom = 25;
params.Bcoeff = 3950;
params.R0 = 100e3;
params.Vcc = 5;
params.name = 'nosubstance';

% Caculated constants
params.sec = 60 * params.mins;
params.Tsamp = 1 / params.fhz;

% Make serial connection with device
a = arduino('/dev/cu.usbmodem143401');

% Number of sample
interv = params.sec * params.fhz;

% Init plotting variable
T1 = 0;
T2 = 0;

% Setup while loop for real time plotting
figure(1)
hold on

while numel(T1) < ( interv + 1)

    % Get analog readings
    V1 = readVoltage(a, params.therm1);
    V2 = readVoltage(a, params.therm2);
    
    % Calculate temperature
    Tplate1 = thermistorTemp(V1, params)
    Tplate2 = thermistorTemp(V2, params)
       
    T1 = [T1, Tplate1];
    T2 = [T2, Tplate2];

    % Plot
    plot(T1)
    plot(T2)
    axis([0, interv, 0, params.Tmax]);
    title('Temperature Variation','Interpreter','latex');
    xlabel('Samples $s$','Interpreter','latex');
    ylabel('Temperature $T$ [$^o$C]','Interpreter','latex');
    grid on;
    drawnow;
 
    %Update the graph
    pause on
    pause(params.Tsamp);
    pause off

end

t = 0 : params.Tsamp : params.sec;

figure(2)
hold on
plot(t, T1)
plot(t, T2)
title('Temperature Variation','Interpreter','latex');
xlabel('Time $s$ [s]','Interpreter','latex');
ylabel('Temperature $T$ [$^o$C]','Interpreter','latex');
legend('Plate 1', 'Plate 2')
grid on;

% Save out
plotstr = strcat('data\test_', params.name);   
plotstr = char(plotstr);
print(2,'-djpeg',plotstr);
save(strcat('test_', params.name, '.mat'), 't' , 'T1', 'T2');

