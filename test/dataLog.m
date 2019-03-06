%% Plot realtime data and export results to 
function dataLog(a, port, fxn, params, test)

    % Generate timestamp
    timeStamp = datestr(now,'dd_mm_yy_HHMMSS');

    %% Acquire and display live data
    figure(1)
    hold on 

    % Create animated line and define axis
    h = animatedline;
    ax = gca;
    ax.YGrid = 'on';
    ax.YLim = [params.minval params.maxval];
    title(strcat('Data ', timeStamp) ,'Interpreter','latex');
    xlabel('Time','Interpreter','latex');
    ylabel(params.id,'Interpreter','latex');

    % Define stop condition
    stop = false;

    startTime = datetime('now');

    % Real time data log
    while ~stop

        % Read current voltage value   
        voltage = readVoltage(a, port);

        % Transform voltage based on passed in function
        data = @fxn(voltage, params)
        dataPlot = [dataPlot, data];

        % Get current time
        t =  datetime('now') - startTime;
        
        % Add points to animation
        addpoints(h, datenum(t),dataPlot)

        % Update axes
        ax.XLim = datenum([t-startTime t]);
        datetick('x','keeplimits')
        drawnow

        % Check stop condition
        stop = readDigitalPin(a,'D12');

    end

    % Create plot for export
    figure(2)
    hold on
    plot(t, dataPlot)
    title(strcat('Data ', timeStamp) ,'Interpreter','latex');
    xlabel('Time','Interpreter','latex');
    ylabel(params.id,'Interpreter','latex');
    grid on;

    % Save plot
    plotstr = strcat('plots\test_', timeStamp);   
    plotstr = char(plotstr);
    print(2,'-djpeg',plotstr);

    % Save .mat data
    save(strcat('test_', timeStamp, '.mat'), 't' , 'dataPlot');

end
