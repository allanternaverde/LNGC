% vetores iniciais
r_0 = [205.081; 5393.556; -5866.674]; % [km]
v_0 = [-5.518; 6.72; 2.901]; % [km/s]
% vetor de condições iniciais
X_0 = [r_0; v_0];

% parametro gravitacional par uma orbita Terrestr u = G * m_Terra
u = 3.986e5;

% energia total específica orbital
E = norm(V0)^2/2 - u/norm(R0)
% semieixo maior da órbita
a = -u/(2*E);
% período da órbita
T = 2*pi*sqrt(a^3/u);

% Sistema de equações para o sistema dos dois corpos
doisCorpos = @(t, X) [zeros(3,3), eye(3); -(u/norm(X(1:3,1))^3)*eye(3), zeros(3,3)]*X;

% o tempo de integração, será feito igual a um período orbinal
[t X] = ode45(doisCorpos, [0 T], X0);

% plot dos vetores de posição
plot3(X(:,1),X(:,2),X(:,3),'g');
hold;

% o tempo de integração, será feito igual a um período orbinal
[t X] = ode15s(doisCorpos, [0 T], X0);

% plot dos vetores de posição
plot3(X(:,1),X(:,2),X(:,3),'b');

% o tempo de integração, será feito igual a um período orbinal
[t X] = ode23s(doisCorpos, [0 T], X0);

% plot dos vetores de posição
plot3(X(:,1),X(:,2),X(:,3),'r');

% plot do modelo terrestre
r_e = 6378.137; % raio equatorial
r_p = 6356.752; % raio polar
[x, y, z] = ellipsoid(0,0,0,r_e,r_e,r_p,30);
surf(x, y, z);

xlabel('[km]');
ylabel('[km]');
zlabel('[km]');
axis equal;