function [] = plotPlanoOrbital(a, e)
    % plot 2d da orbita no plano orbital, recebe como argumentos os vetores em
    % coordenadas polares
    
    % vetores da orbita em coordenadas polares no plano orbital
    [r, nu] = coordPolares(a,e);
    
    figure(1);
    hold on;
    grid;
    grid minor;
    axis equal;
    [rx,ry] = pol2cart(nu, r);
    plot(rx,ry,'LineWidth',2,'Color','red');

    r_T = 6371; 
    [x,y] = pol2cart(nu, r_T);
    plot(x,y,'--','Color','k');

    scale = 1e4/2;
    quiver(0,0,scale,0);
    quiver(0,0,0,scale);

    xlim([-4.5e4,1e4]);
    xlabel('[km]');
    ylabel('[km]');
end