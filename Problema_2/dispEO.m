function [] = dispEO(a,e,I,Omega,omega,upsilon)
    disp('Elementos orbitais');
    disp(['a=',num2str(a)]);
    disp(['e=',num2str(e)]);
    disp(['i=',num2str(i*180/pi), '°']);
    disp(['Omega=',num2str(Omega*180/pi), '°']);
    disp(['omega=',num2str(omega*180/pi), '°']);
    disp(['upsilon=',num2str(upsilon*180/pi), '°']);
    disp(' ');
end