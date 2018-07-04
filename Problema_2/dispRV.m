function [] = dispRV(R,V)
    % vetor de posicao r 
    disp('Vetor posição');
    s=sprintf('r = % +.3f I % +.3f J % +.3f K',R(1),R(2),R(3));
    disp(s)

    % vetor de velocidade v 
    disp('Vetor velocidade')
    s=sprintf('v= % +.3f I % +.3f J     % +.3f K',R(1),R(2),R(3));
    disp(s)
    disp(' ');
end