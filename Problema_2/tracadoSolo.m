function [] = plotTracadoSolo(a,e,I,Omega,omega)
    % vetores em coordenadas polares
    [r, nu] = coordPolares(a,e);
    % conversão para coordenadas cartesianas
    [rx,ry] = pol2cart(nu, r);
    
    % rotacoes que levam do plano orbital para o sistema Oxyz fixado na Terra
    rotz1= [cos(-omega) sin(-omega) 0; -sin(-omega) cos(-omega) 0; 0 0 1];
    rotx2=[1 0 0;0 cos(-I) sin(-I);0 -sin(-I) cos(-I)];
    rotz3=[cos(-Omega) sin(-Omega) 0;-sin(-Omega) cos(-Omega) 0; 0 0 1];

    R_planoOrbital = [rx;ry;zeros(1,length(rx))];
    R_xyz = rotz3*rotx2*rotz1*R_planoOrbital;
    
    %% traçado de solo
    % delta N - 
    dN_ = 0;
    T = 0;
    lat_ = atan(R_xyz(3,:)./sqrt(R_xyz(1,:).^2 + R_xyz(2,:).^2))*180/pi;
    lat = [];
    lon = [];
    nOrbs = 2;
    %lon = atan2(R_xyz(2,:),R_xyz(1,:))*180/pi-dN;
    for n=1:nOrbs
        for i=1:length(R_xyz(1,:))
              if i > 1
                dt = P*(nu(i)-nu(i-1))*r(i)^2/(2*pi*a*b);
                dN_ = dN_ + (15/3600)*dt;
                T = T + dt;
              end
              lon_(i) = atan2(R_xyz(2,i),R_xyz(1,i))*180/pi- dN_;
        end
        lat = [lat;lat_'];
        lon = [lon;lon_'];
    end


    grs80 = referenceEllipsoid('GRS80','km');
    %[lat,lon,h] = ecef2geodetic(grs80,R_xyz(1,:),R_xyz(2,:),R_xyz(3,:));
    load geoid
    figure();
    hold on;
    axesm('mercator','Frame','on','Grid','on','MeridianLabel','on', 'ParallelLabel','on','FontSize',7); 
    %framem; 
    %grid minor; 
    axis off;
    geoshow(geoid, geoidrefvec, 'DisplayType', 'texturemap');
    geoshow('landareas.shp', 'FaceColor', [0.5 1.0 0.5]);
    %geoshow([lat;lat],[lon;lon+lon(end)],'LineWidth',2 ,'Color','red');
    geoshow([lat],[lon],'LineWidth',2 ,'Color','red');
    %axis equal;
    %xlabel('Longitude');
    %ylabel('Latitude');
end