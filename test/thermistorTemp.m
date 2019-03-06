%% Calculate the temperature of a thermistor from a voltage reading
function ToutC = thermistorTemp(V, params)

    Vrat = V / params.Vcc;
    Rtherm = (Vrat .* params.R0) / (1 - Vrat);
    ToutC =  (((1 / params.Bcoeff) * log(Rtherm / params.R0) + 1.0 / (params.Tnom + 273.15)) ^ -1) - 273.15;

end
