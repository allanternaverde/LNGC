% limpa framework
clear all;
close all;
clc;

% vetores iniciais
% posicao inicial
r_0 = [205.081; 5393.556; -5866.674]; % [km]
%r_0 = [10016.34; -17012.52; 7899.28];
% velocidade inicial
v_0 = [-5.518; 6.72; 2.901]; % [km/s]
%v_0 = [2.5; -1.05; 3.88];

% vetor de condições iniciais
X_0 = [r_0; v_0];

% parametro gravitacional par uma orbita Terrestr u = G * m_Terra
u = 3.986e5;
% energia total específica orbital
E = norm(v_0)^2/2 - u/norm(r_0);
% semieixo maior da órbita
a = -u/(2*E);
% período da órbita
T = 2*pi*sqrt(a^3/u);

% Sistema de equações para o problema dos dois corpos
doisCorpos = @(t, X) [zeros(3,3), eye(3); -(u/norm(X(1:3,1))^3)*eye(3), zeros(3,3)]*X;

% ajuste da tolerância relativa do integrador
options = odeset('RelTol',1e-9);

% numero de órbitas a serem propagadas 
nOrbitas = 1;

% integração numérica das equações doisCorpos considerando as condicoes
% iniciais X_0, para integração é utilizado um tempo múltiplo do período
% orbital
[t X] = ode45(doisCorpos, [0 nOrbitas*T], X_0, options);

%% figura 1 - plot 3D animado da órbita
fig = figure;
plot3(X(:,1),X(:,2),X(:,3),'LineWidth',2,'Color','red');
hold on;
p = plot3(X(1,1),X(1,2),X(1,3),'o','MarkerFaceColor','blue');
vel = quiver3(X(1,1),X(1,2),X(1,3),X(1,4),X(1,5),X(1,6),'AutoScaleFactor',2000,'LineWidth',2,'Color','blue','MaxHeadSize',2);

% plot do elipsóide terrestre utilizando o modelo GRS80
earth = referenceEllipsoid('GRS80','km');
[x, y, z] = ellipsoid(0,0,0,earth.SemimajorAxis, earth.SemimajorAxis, earth.SemiminorAxis);
globe = surf(x, y, -z, 'FaceColor', 'none', 'EdgeColor', 0.5*[1 1 1]);
image_file = 'https://noperation.files.wordpress.com/2012/11/world32k.jpg';
cdata = imread(image_file);
set(globe, 'FaceColor', 'texturemap', 'CData', cdata,'EdgeColor', 'none');

% unidades de opções do gráfico
xlabel('[km]');
ylabel('[km]');
zlabel('[km]');
axis equal;
ax = gca;
xlim(ax.XLim*1.1);
ylim(ax.YLim*1.1);
zlim([ax.ZLim(1)*2 ax.ZLim(2)]);
axis manual;
grid;

n = 13;
for k = 1:n:length(X(:,1))
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

% unidades de opções do gráfico
xlabel('[km]');
ylabel('[km]');
zlabel('[km]');
view(cross(r_0,v_0));
axis equal;
grid;
