function [Cost] = lagrange_cost(v, i, rm)
Kv = 0.119;     % Constante de tensão induzinada no motor [V/(rad/s)]
Ra = 0.07;      % Resistência elétrica do motor [V/A]

u = i*Ra + Kv*(v/rm);

Cost = u*i;
end
