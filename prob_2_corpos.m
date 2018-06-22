%% limpa framework
clear all;
close all;
clc;

%% vetores de velocidade e posição iniciais
r_0 = [205.081; 5393.556; -5866.674]; % [km]
v_0 = [-5.518; 6.72; 2.901]; % [km/s]

% parametro gravitacional par uma orbita Terrestr u = G * m_Terra
u = 3.986e5; %[km^3/s^2]
% energia total específica orbital
E = norm(v_0)^2/2 - u/norm(r_0); %[kJ/kg]
% semieixo maior da órbita
a = -u/(2*E); % [km]
% período da órbita
T = 2*pi*sqrt(a^3/u); %[s]

%% vetor de condições iniciais para integração numérica
X_0 = [r_0; v_0];

%% Sistema de equações para o problema dos dois corpos
doisCorpos = @(t, X) [zeros(3,3), eye(3); -(u/norm(X(1:3,1))^3)*eye(3), zeros(3,3)]*X;

%% numero de órbitas a serem propagadas 
nOrbitas = 3;

%% integração numérica utilizando ODE45
% ajuste da tolerância relativa 
options = odeset('RelTol',1e-9); 
% integração 
[t X] = ode45(doisCorpos, [0 nOrbitas*T], X_0, options);

%% figura 1 - plot 3D animado da órbita
% nAnimacoes escolhe o numero de animacoes 
nAnimacoes = 1;
if nAnimacoes > nOrbitas
    nAnimacoes = nOrbitas;
end

fig = figure;
% plot das orbitas
plot3(X(:,1),X(:,2),X(:,3),'LineWidth',2,'Color','red');
hold on;
% posicao inicial
p = plot3(X(1,1),X(1,2),X(1,3),'o','MarkerFaceColor','blue');
% vetor de posicao inicial
vel = quiver3(X(1,1),X(1,2),X(1,3),X(1,4),X(1,5),X(1,6),'AutoScaleFactor',2000,'LineWidth',2,'Color','blue','MaxHeadSize',2);

% plot do elipsóide terrestre utilizando o modelo GRS80
earth = referenceEllipsoid('GRS80','km');
[x, y, z] = ellipsoid(0,0,0,earth.SemimajorAxis, earth.SemimajorAxis, earth.SemiminorAxis);
globe = surf(x, y, -z, 'FaceColor', 'none', 'EdgeColor', 0.5*[1 1 1]);
image_file = 'https://noperation.files.wordpress.com/2012/11/world32k.jpg';
cdata = imread(image_file);
set(globe, 'FaceColor', 'texturemap', 'CData', cdata,'EdgeColor', 'none');

% unidades dos eixos e opções gráficas
xlabel('[km]');
ylabel('[km]');
zlabel('[km]');
axis equal;
axis manual;
grid;

% animação da posição e vetor velocidade
n = 13;
for k = 1:n:int32(((length(X(:,1)))/nOrbitas)*nAnimacoes)
    set(p, 'XData', X(k,1));
    set(p, 'YData', X(k,2));
    set(p, 'ZData', X(k,3));
    set(vel, 'XData', X(k,1));
    set(vel, 'yData', X(k,2));
    set(vel, 'ZData', X(k,3));
    set(vel, 'UData', X(k,4));
    set(vel, 'VData', X(k,5));
    set(vel, 'WData', X(k,6));
     if k > 1
         delay = (t(k)-t(k-n))/10000;
         pause(delay);
     end
    drawnow;
end
hold off;

%% figura 2 - plano orbital e vetores de velocidade
fig2 = figure;
% plot das orbitas
plot3(X(:,1),X(:,2),X(:,3),'LineWidth',2,'Color','red');
hold on;

% plot do elipsóide terrestre utilizando o modelo GRS80
earth = referenceEllipsoid('GRS80','km');
[x, y, z] = ellipsoid(0,0,0,earth.SemimajorAxis, earth.SemimajorAxis, earth.SemiminorAxis);
globe = surf(x, y, -z, 'FaceColor', 'none', 'EdgeColor', 0.5*[1 1 1]);
image_file = 'https://noperation.files.wordpress.com/2012/11/world32k.jpg';
cdata = imread(image_file);
set(globe, 'FaceColor', 'texturemap', 'CData', cdata,'EdgeColor', 'none');

% plot vetores de velocidade no plano orbital
fim = int32(length(X(:,1))/nOrbitas);
for i=1:40:fim
    quiver3(X(i,1),X(i,2),X(i,3),X(i,4),X(i,5),X(i,6),'AutoScaleFactor',1200,'LineWidth',2,'Color','blue','MaxHeadSize',3);
end

% unidades dos eixos e opções do gráfico
xlabel('[km]');
ylabel('[km]');
zlabel('[km]');
view(cross(r_0,v_0));
axis equal;
grid;

%% plot no tempo dos vetores de posição, velocidade e momento angular especifico em módulo
for i=1:length(X(:,1))
    R_ = [X(i,1);X(i,2);X(i,3)];
    V_ = [X(i,4);X(i,5);X(i,6)];
    R(i) = norm(R_);
    V(i) = norm(V_);
    H(i) = norm(cross(R_,V_));
    E(i) = V(i)^2/2-u/R(i);
end

figure;
ax1 = subplot(3,1,1);
plot(ax1,t,R);
title(ax1,'Posição em módulo');
ylabel(ax1,'|r| [km]');
grid minor;

ax2 = subplot(3,1,2);
plot(ax2,t,V);
title(ax2,'Velocidade em módulo');
ylabel('|v| [km/s]');
grid minor;

ax3 = subplot(3,1,3);
plot(ax3,t,E);
ylim([-8.03 -7.94]);
title(ax3, 'Energia total específica');
ylabel(ax3, 'E [kJ/kg]');
grid minor;

%% altitudes e velocidades no apogeu e perigeu
disp('altitude no perigeu');
disp([num2str(min(R)), ' km']);

disp('altitude no apogeu');
disp([num2str(max(R)- 6357),' km']);

disp('velocidade no perigeu');
disp([num2str(max(V)), ' km/s']);

disp('velocidade no apogeu');
disp([num2str(min(V)), ' km/s']);
