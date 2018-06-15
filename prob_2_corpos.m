clear all;
close all;
clc;
% vetores iniciais
r_0 = [205.081; 5393.556; -5866.674]; % [km]
v_0 = [-5.518; 6.72; 2.901]; % [km/s]
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

% Sistema de equações para o sistema dos dois corpos
doisCorpos = @(t, X) [zeros(3,3), eye(3); -(u/norm(X(1:3,1))^3)*eye(3), zeros(3,3)]*X;

% o tempo de integração, será feito igual a um período orbinal
options = odeset('RelTol',1e-9);
[t X] = ode45(doisCorpos, [0 10*T], X_0, options);


% posição e velocidade de primeira orbita propagada
r_i = [ X(1,1) X(1,2) X(1,3)];
v_i = [ X(1,4) X(1,5) X(1,6)];
% posição e velocidade de primeira orbita propagada
r_f = [ X(end,1) X(end,2) X(end,3)];
v_f = [ X(end,4) X(end,5) X(end,6)];
% energia total específica orbital para primeira orbita
E_i = norm(v_i)^2/2 - u/norm(r_i);
% energia total específica orbital para ultima orbita
E_f = norm(v_f)^2/2 - u/norm(r_f);
% semieixo maior da primeira órbita
a_i = -u/(2*E_i);
% semieixo maior da ultima órbita
a_f = -u/(2*E_f);
% plot dos vetores de posição

disp(['discrepancia = ',num2str(a_i-a_f),' [km]']);
plot3(X(:,1),X(:,2),X(:,3),'LineWidth',2,'Color','red');
hold;

%view(cross(r_0,v_0));

% % o tempo de integração, será feito igual a um período orbinal
% [t X] = ode15s(doisCorpos, [0 T], X_0);
% 
% % plot dos vetores de posição
% plot3(X(:,1),X(:,2),X(:,3),'b');
% 
% % o tempo de integração, será feito igual a um período orbinal
% [t X] = ode23s(doisCorpos, [0 T], X_0);
% 
% % plot dos vetores de posição
% plot3(X(:,1),X(:,2),X(:,3),'r');

% % plot do modelo terrestre
% r_e = 6378.137; % raio equatorial
% r_p = 6356.752; % raio polar
% [x, y, z] = referenceEllipsoid(0,0,0,r_e,r_e,r_p,30);
% surf(x, y, z);
%grs80 = referenceEllipsoid('GRS80','km');


% % PLOTAGEM DA TERRA NO MODELO DE ELIPSOIDE 'GRS80'
% grs80 = referenceEllipsoid('GRS80','km');
% figure('Renderer','opengl');
% ax = axesm('globe','Geoid',grs80,'Grid','on', ...
%     'GLineWidth',1,'GLineStyle','-',...
%     'Gcolor',[0.9 0.9 0.1],'Galtitude',100);
% ax.Position = [0 0 1 1];
% view(3)
% load topo
% geoshow(topo,topolegend,'DisplayType','texturemap')
% demcmap(topo)
% land = shaperead('landareas','UseGeoCoords',true);
% plotm([land.Lat],[land.Lon],'Color','black')
% rivers = shaperead('worldrivers','UseGeoCoords',true);
% plotm([rivers.Lat],[rivers.Lon],'Color','blue')


xlabel('[km]');
ylabel('[km]');
zlabel('[km]');
axis equal;
grid;


% lat = atan(X(:,3)./sqrt(X(:,1).^2 + X(:,2).^2))*180/pi;
% for i=1:length(X(:,1))
%     lon(i,1) = acos(X(i,1)/sqrt(X(i,1)^2 + X(i,2)^2));
%     if X(i,2) < 0
%         lon(i,1) = -lon(i,1);
%     end
%     lon(i,1) = lon(i,1)*180/pi;
% end
%plot(x,y);
%grid minor;
%plot(lon,lat,'.');

grs80 = referenceEllipsoid('GRS80','km');
[lat,lon,h] = ecef2geodetic(grs80,X(:,1),X(:,2),X(:,3));
load geoid
figure
axesm eckert4; 
framem; gridm;
axis off;
geoshow(geoid, geoidrefvec, 'DisplayType', 'texturemap');
hcb = colorbar('southoutside');
set(get(hcb,'Xlabel'),'String','EGM96 Geoid Heights in Meters.')
geoshow('landareas.shp', 'FaceColor', [0.5 1.0 0.5]);
geoshow(lat,lon,'LineWidth',2 ,'Color','red');


