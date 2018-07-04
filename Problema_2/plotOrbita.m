function [] = plotOrbita(a,e,I,Omega,omega)
    % vetores em coordenadas polares
    [r, nu] = coordPolares(a,e);

    % conversão para coordenadas cartesianas
    [rx,ry] = pol2cart(nu, r);

    %% plot3d da orbita
    % rotacoes que levam do plano orbital para o sistema Oxyz fixado na Terra
    rotz1= [cos(-omega) sin(-omega) 0; -sin(-omega) cos(-omega) 0; 0 0 1];
    rotx2=[1 0 0;0 cos(-I) sin(-I);0 -sin(-I) cos(-I)];
    rotz3=[cos(-Omega) sin(-Omega) 0;-sin(-Omega) cos(-Omega) 0; 0 0 1];

    R_planoOrbital = [rx;ry;zeros(1,length(rx))];
    R_xyz = rotz3*rotx2*rotz1*R_planoOrbital;

    figure()
    axis equal;
    hold on;
    grid;
    grid minor;
    %quiver3(zeros(1,length(R_xyz(1,:))),zeros(1,length(R_xyz(1,:))),zeros(1,length(R_xyz(1,:))),R_xyz(1,:),R_xyz(2,:),R_xyz(3,:));
    plot3(R_xyz(1,:),R_xyz(2,:),R_xyz(3,:),'LineWidth',2,'Color','red');
    xlabel('[km]');
    ylabel('[km]');
    zlabel('[km]');
    
    %% plot do elipsóide terrestre utilizando o modelo GRS80
    earth = referenceEllipsoid('GRS80','km');
    [x, y, z] = ellipsoid(0,0,0,earth.SemimajorAxis, earth.SemimajorAxis, earth.SemiminorAxis);
    globe = surf(x, y, -z, 'FaceColor', 'none', 'EdgeColor', 0.5*[1 1 1]);
    image_file = 'https://noperation.files.wordpress.com/2012/11/world32k.jpg';
    cdata = imread(image_file);
    set(globe, 'FaceColor', 'texturemap', 'CData', cdata,'EdgeColor', 'none');
    view([1 1 1]);
end