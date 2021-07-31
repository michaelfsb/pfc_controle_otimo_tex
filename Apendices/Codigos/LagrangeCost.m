function [Cost] = lagrange_cost(v, i, phi)
Kv = 0.119;     % Constante de tensão induzida no motor [V/(rad/s)]
Ra = 0.07;      % Resistência elétrica do motor [V/A]
rr = 0.254;     % Raio da roda [m]

u = i*Ra + Kv*(v*phi/rr);

Cost = u*i;
end
