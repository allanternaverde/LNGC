function M_0 = Mean_0(e, upsilon)
    % retorna a anomalia media para uma anomalia verdadeira nu_0
    %% anomalia excentrica
    nu_0 = upsilon;
    sinE_0 = sqrt(1-e^2)*sin(nu_0)/(1+e*cos(nu_0));
    cosE_0 = (e+cos(nu_0))/(1+e*cos(nu_0));
    E_0 = atan2(sinE_0,cosE_0);

    %% anomalia media 
    M_0 = E_0 - e*sinE_0;
end