function [Iup] = i_max(v, i, phi)
Kv = 0.119;     % Constante de tensão induzida no motor [V/(rad/s)]
Ra = 0.07;      % Resistência elétrica do motor [V/A]
Vbat = 42;      % Tensão da bateria [V]
rr = 0.254;     % Raio da roda [m]

I_MAX = (Vbat-Kv*(v*phi/rr))/Ra;

Iup = i - I_MAX;
end