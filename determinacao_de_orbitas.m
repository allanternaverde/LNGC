clear all;
close all;
clc;
format long;
%% Problema_2 - Determinação de elementos orbitais
% parametro gravitacional para orbita terrestre
mu = 3.986e5; % [km³/s²]
% vetores de posição e velocidade
R = [205.081 5393.556 -5866.674]; % [km]
V = [-5.518 6.72 2.901]; % [km/s]

% calculo dos elementos orbitais
[a,e,i,Omega,omega,upsilon] = elemOrbitais(mu,R,V);

%% equação da orbita em coordenadas polares 
% parâmetro da cônica
p = a*(1-e^2);
% anomalia verdadeira
nu = [0:0.1:2*pi+0.1];
% calcula de r
for i=1:length(nu)
    r(i) =  p/(1 + e*cos(nu(i)));
end


%% plot-2d da orbita
figure();
hold on;
grid;
grid minor;
axis equal;
[rx,ry] = pol2cart(nu, r);
plot(rx,ry);

r_T = 6371; 
[x,y] = pol2cart(nu, r_T);
plot(x,y);

%% plot3d da orbita
% rotacoes
% Omega = -Omega;
% i = -i;
% omega = -omega;
rotz1= [cos(omega) sin(omega) 0; -sin(omega) cos(omega) 0; 0 0 1];
rotx2=[1 0 0;0 cos(i) sin(i);0 -sin(i) cos(i)];
rotz3=[cos(Omega) sin(Omega) 0;-sin(Omega) cos(Omega) 0; 0 0 1];

R_pol = [rx;ry;zeros(1,length(rx))];

R_xyz = rotz3*rotx2*rotz1*R_pol;
figure()
axis equal;
hold on;
grid;
grid minor;
plot3(R_xyz(1,:),R_xyz(2,:),R_xyz(3,:));

% plot do elipsóide terrestre utilizando o modelo GRS80
earth = referenceEllipsoid('GRS80','km');
[x, y, z] = ellipsoid(0,0,0,earth.SemimajorAxis, earth.SemimajorAxis, earth.SemiminorAxis);
globe = surf(x, y, -z, 'FaceColor', 'none', 'EdgeColor', 0.5*[1 1 1]);
image_file = 'https://noperation.files.wordpress.com/2012/11/world32k.jpg';
cdata = imread(image_file);
set(globe, 'FaceColor', 'texturemap', 'CData', cdata,'EdgeColor', 'none');
view(-cross(R,V));


lat = atan(R_xyz(3,:)./sqrt(R_xyz(1,:).^2 + R_xyz(2,:).^2))*180/pi;
lat = lat';
for i=1:length(R_xyz(1,:))
    lon(i,1) = acos(R_xyz(1,i)/sqrt(R_xyz(1,i)^2 + R_xyz(2,i)^2));
    if R_xyz(2,i) < 0
        lon(i,1) = -lon(i,1);
    end
    lon(i,1) = lon(i,1)*180/pi;
end

grs80 = referenceEllipsoid('GRS80','km');
%[lat,lon,h] = ecef2geodetic(grs80,R_xyz(1,:),R_xyz(2,:),R_xyz(3,:));
load geoid
figure();
hold on;
axesm('MapProjection','mercator'); 
framem; gridm;
axis off;
geoshow(geoid, geoidrefvec, 'DisplayType', 'texturemap');
hcb = colorbar('southoutside');
%set(get(hcb,'Xlabel'),'traçado de solo na projeção de Mercator')
geoshow('landareas.shp', 'FaceColor', [0.5 1.0 0.5]);
geoshow(lat,lon,'LineWidth',2 ,'Color','red');
