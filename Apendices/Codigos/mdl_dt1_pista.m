function [mdl_out] = mdl_dt1_pista(x, v, i, rm) 
% constantes
Kt = 0.119;     % Constante de torque do motor [N/A]
rr = 0.254;     % Raio da roda [m]
ro = 1.22;      % Densidade do ar [kg/m3]
Af = 0.26;      % Área frontal [m2]
Cd = 0.164;     % Coef. de arrasto aerodinâmico []
G  = 9.81;      % Aceleração da gravidade [m/s2]
mi = 0.0024;    % Coef. de resistência ao rolamento dos pneus []
Jr = 0.015;     % Momento de inercia da roda[kg.m2]
Jm = 0.0625e-3; % Momento do inercia do motor [kg.m2]
mv = 18;        % Massa do veiculo [kg]
mp = 50;        % Massa do piloto [kg]
ef = 0.95;      % Eficiência da roda de atrito (transmissão) []

% aproximação com 8 cos da inclinação da pista
theta = atan(0.012293*cos(0.004363*x-0.228758)+...
    0.003041*cos(0.000236*x+0.399758)+...
    0.005818*cos(0.008726*x-2.193381)+...
    0.002926*cos(0.000246*x+3.490690)+...
    0.003958*cos(0.017449*x-3.058604)+...
    0.005365*cos(0.026184*x+0.579933)+...
    0.004361*cos(0.021814*x+1.627015)+...
    0.005539*cos(0.030552*x-1.400496));

eta = rr/rm;                        % relação de transmissão
mr = 3*(Jr/rr^2) + Jm*(eta^2/rr^2); % massa eq. rotacional 
M = mv + mp + mr;

Ft = ((Kt*i)*ef)/rm;               % força de tração
Fa = (ro*Af*Cd*v^2)/2;             % arrasto aerodinâmico
Fr = (mv + mp)*G*mi*cos(theta);    % resistência ao rolamento
Fg = (mv + mp)*G*sin(theta);       % componente do peso

% derivada dos estados
x_dot = v;
v_dot = (Ft - Fa - Fr - Fg)/M;

mdl_out = [x_dot; v_dot; Ft; Fa; Fr; Fg; theta];
end