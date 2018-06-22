% limpa framework
clear all;
close all;
clc;

digits

%% parametro gravitacional da órbita terrestre
mu = 3.986e5; %[km^3/s^2]

%% vetore de posição e velocidade inerciais
R_ = [205.081 5393.556 5866.674]; %[km]
V_ = [-5.518 6.72 2.901]; %[km/s]

%% energia total específica
E = norm(V_)^2/2-mu/norm(R_);

%% semieixo maior
a = -mu/(2*E)
