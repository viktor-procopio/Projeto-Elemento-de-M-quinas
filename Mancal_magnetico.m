clear; close all; clc;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Projeto Elementos de Máquinas                                           %
% Fase 2: Projeto de Mancais (Mancal Magnético)                           %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% 1. Parâmetros de entrada
A       = 3e-4;          % Área da seção transversal da bobina [m^2]
x       = [0, 0.1e-3, 0.25e-3, 0.5e-3]; % Deslocamento do eixo [m]
mhi_0   = 4*pi*1e-7;     % Permeabilidade magnética do vácuo [T.m/A]
i0      = 4;             % Corrente de Bias [A]
ix      = 4;             % Corrente de controle [A] (assumindo o máximo para teste)
i1      = i0 + ix;       % Corrente na bobina 1 [A]
i2      = i0 - ix;       % Corrente na bobina 2 [A]
N       = 200;           % Número de espiras da bobina
g0      = 1e-3;          % Entreferro [m]

% 2. Constantes do Mancal
Km      = (mhi_0 * A * N^2) / 4; 
Ki      = 4 * Km * (i0 / g0^2);
Kx      = 4 * Km * (i0^2 / g0^3);

% 3. Cálculos de Força (Vetorizado - dispensa o laço for)
% Força magnética não linear
F_nl = Km * ( (i1^2 ./ (g0 - x).^2) - (i2^2 ./ (g0 + x).^2) );

% Força magnética linearizada
F_l = Ki * ix + Kx * x;

% 4. Exibição dos resultados no console
disp('Deslocamento x [mm] | Força Não Linear [N] | Força Linearizada [N]');
for i = 1:length(x)
    fprintf('%.2f                | %6.2f               | %6.2f\n', x(i)*1000, F_nl(i), F_l(i));
end
