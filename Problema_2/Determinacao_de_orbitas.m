%% limpa aerea de trabalho
clear all;
close all;
clc;
format long;

%% Problema_2 - Determinação de elementos orbitais e traçado de solo
% parametro gravitacional para orbita terrestre
mu = 3.986e5; % [km³/s²]

% vetores de posição e velocidade
R = [205.081; 5393.556; -5866.674]; % [km]
V = [-5.518; 6.72; 2.901]; % [km/s]
dispRV(R,V);

% calculo dos elementos orbitais
[a,e,I,Omega,omega,upsilon] = elemOrbitais(mu,R,V);
dispEO(a,e,I,Omega,omega,upsilon);

%% plot-2d da orbita
plotPlanoOrbital(a,e);

%% plot-3d da orbital
plotOrbita(a,e,I,Omega,omega)

%% plot do tracado de solo
nOrbits = 4; 
rotacao = 1;
plotTracadoSolo(a,e,I,Omega,omega,mu,nOrbits,rotacao)

%% TLE elements
TLE = tle(a,e,I,Omega,omega,upsilon,mu);
disp('Two Line Elements:')
disp(TLE);

%% plot da orbita com animação
nVezes = 10;
plotOrbitaAnimado(a,e,I,Omega,omega,mu,nVezes)


