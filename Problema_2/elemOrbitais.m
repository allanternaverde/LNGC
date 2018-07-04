function [a,e,i,Omega,omega,upsilon] = elemOrbitais(mu,r,v)
% os vetores posicao e velocidade devem ser passados em [m] e [m/s]
% a: semieixo maior
% e: excentricidade
% i: inclinacao do plano orbital
% Omega: longitude do nodo ascendente
% omega: argumento do perigeu
% upsilon: anomalia verdadeira

% vetor de momento angular especifico 
C = cross(r,v);

% versor que aponta na direcao de K
K = [0 0 1];
% versor que aponta na direcao de I
I = [1 0 0];

% vetor que aponta na direcao do nodo ascentende
N = cross(K,C);

% energia total especifica 
E = norm(v)^2/2 -mu/norm(r);

% semieixo maior 
a = - mu/(2*E);

% excentricidade e
e = sqrt(1 + 2*(norm(C)^2)*E/mu^2);

% inclinacao do plano orbital
% i = acos(dot(K,C)/norm(C))*180/pi;
i = acos(dot(K,C)/norm(C));

% longitude do nodo ascentende
Omega = acos(dot(N,I)/norm(N));
if N(2) < 0
    Omega = 2*pi - Omega;
end
%Omega = Omega*180/pi;

% vetor excentricidade
epsilon = cross(v,C) -mu*r/norm(r);

% argumento do perigeu
omega = acos(dot(epsilon,N)/(norm(epsilon)*norm(N)));
if epsilon(3) < 0
    omega = 2*pi- omega;
end
%omega = omega*180/pi;

% anomalia verdadeira 
upsilon = acos(dot(epsilon,r)/(norm(epsilon)*norm(r)));
if dot(r,v) < 0
    upsilon =2*pi-upsilon;
end
%upsilon = upsilon*180/pi;