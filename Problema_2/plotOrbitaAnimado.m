function [] = plotOrbitaAnimado(a,e,I,Omega,omega,mu,nvezes)
    % período orbital
    P = 2*pi*sqrt(a^3/mu);
    % semieixo menor
    b = a*sqrt(1-e^2);
    
    % vetores em coordenadas polares
    [r, nu] = coordPolares(a,e);
    % conversão para coordenadas cartesianas
    [rx,ry] = pol2cart(nu,r);

    % rotacoes que levam do plano orbital para o sistema Oxyz fixado na Terra
    rotz1= [cos(-omega) sin(-omega) 0; -sin(-omega) cos(-omega) 0; 0 0 1];
    rotx2=[1 0 0;0 cos(-I) sin(-I);0 -sin(-I) cos(-I)];
    rotz3=[cos(-Omega) sin(-Omega) 0;-sin(-Omega) cos(-Omega) 0; 0 0 1];

    R_planoOrbital = [rx;ry;zeros(1,length(rx))];
    R_xyz = rotz3*rotx2*rotz1*R_planoOrbital;
    % plot 3D animado da órbita
    fig = figure();
    plot3(R_xyz(1,:),R_xyz(2,:),R_xyz(3,:),'LineWidth',2,'Color','red');
    hold on;
    p = plot3(R_xyz(1,1),R_xyz(2,1),R_xyz(3,1),'o','MarkerFaceColor','blue');
    %vel = quiver3(X(1,1),X(1,2),X(1,3),X(1,4),X(1,5),X(1,6),'AutoScaleFactor',2000,'LineWidth',2,'Color','blue','MaxHeadSize',2);

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

    % plot animado do satelite e vetor velocidade
    delay = 1/60;
    nvezes = 10;
    % fator de aceleraça com relacao a velocidade real do movimento
    for n = 1: nvezes
        for k=1:length(R_xyz(1,:))
            set(p, 'XData', R_xyz(1,k));
            set(p, 'YData', R_xyz(2,k));
            set(p, 'ZData', R_xyz(3,k));
    %             set(vel, 'XData', X(k,1));
    %             set(vel, 'yData', X(k,2));
    %             set(vel, 'ZData', X(k,3));
    %             set(vel, 'UData', X(k,4));
    %             set(vel, 'VData', X(k,5));
    %             set(vel, 'WData', X(k,6));
            if k>1
                dt = (nu(k)-nu(k-1))*r(k)^2/(2*pi*a*b);
                pause(dt);
            end
            %pause(delay);
            drawnow;
        end
    end
hold off;