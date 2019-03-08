%% Plot realtime data and export results to 
function dataLog2(fxn, params)

    % Generate timestamp
    timeStamp = datestr(now,'dd_mm_yy_HHMMSS');
    params.mins = minutes(params.mins);

    %% Acquire and display live data
    figure(1)
    hold on 

    % Create animated line and define axis
    h = animatedline;
    ax = gca;
    ax.YGrid = 'on';
    ylim([params.minval params.maxval])
    title(strcat('Data', {' '}, timeStamp) ,'Interpreter','none');
    xlabel('Time','Interpreter','latex');
    ylabel((params.id),'Interpreter','latex');

    % Define stop condition
    stop = false;

    interv = params.sec * params.fhz;
    dataPlot = [];
    
    startTime = datetime('now');
    t =  datetime('now') - startTime;
    
    % Real time data log
    while t < params.mins

        % Read current voltage value   
        voltage = 2.75 + rand / 100;

        % Transform voltage based on passed in function
        data = fxn(voltage, params);
        dataPlot = [dataPlot, data];

        % Get current time
        t =  datetime('now') - startTime;
        
        % Add points to animation
        addpoints(h, datenum(t), data)

        % Update axes
        % ax.XLim = datenum([(t-seconds(15)) t]);
        datetick('x','keeplimits')
        drawnow

    end

    % Create plot for export
    figure(2)
    hold on
    tplot = linspace(0, seconds(t), numel(dataPlot));
    plot(tplot, dataPlot)
    xlim([0 seconds(t)])
    ylim([params.minval params.maxval])
    title(strcat('Data', {' '}, timeStamp) ,'Interpreter','none');
    xlabel('Time $t$ [s]','Interpreter','latex');
    ylabel(params.id,'Interpreter','latex');
    grid on;

    % Save plot
    plotstr = strcat('plots/test_', timeStamp);   
    plotstr = char(plotstr);
    print(2,'-djpeg',plotstr);

    % Save .mat data
    save(strcat('data/test_', timeStamp, '.mat'), 'tplot' , 'dataPlot');

end
