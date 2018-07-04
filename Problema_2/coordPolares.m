function [r, nu] = coordPolares(a,e)
    % a função retorna o vetor de posição do veículo espacial em
    % coordenadas polares (r, nu) no plano orbital
    % anomalia verdadeira
    nu = [0:0.01:2*pi];
    % parâmetro da cônica
    p = a*(1-e^2);
    % calculo de r
    for i=1:length(nu)
        r(i) =  p/(1 + e*cos(nu(i)));
    end
end