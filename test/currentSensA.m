%% Calculate the current of based on voltage reading
function IoutA = currentSensA(voltageV, params)

    IoutA = (params.IsensGainApV * voltageV) - params.IsensOffsetA;

end
