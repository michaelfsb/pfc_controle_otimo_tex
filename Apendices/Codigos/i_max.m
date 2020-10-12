function [Iup] = i_max(v, i, rm)
Kv = 0.119;     % Constante de tensão induzida no motor [V/(rad/s)]
Ra = 0.07;      % Resistência elétrica do motor [V/A]
Vbat = 42;      % Tensão da bateria [V]

I_MAX = (Vbat-Kv*(v/rm))/Ra;

Iup = i - I_MAX;
end