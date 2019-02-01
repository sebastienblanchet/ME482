function Tout = thermistorTemp(V, params)

    % Vo = R / (R + 10K) * Vcc
    % \frac{1}{T} = \frac{1}{T_0} + \frac{1}{B}\ln \left(\frac{R}{R_0}\right)
    Vrat = V / params.Vcc;
    Rtherm = (Vrat .* params.R0) / (1 - Vrat);
    Tout =  (((1 / params.Bcoeff) * log(Rtherm / params.R0) + 1.0 / (params.Tnom + 273.15)) ^ -1) - 273.15;

end
