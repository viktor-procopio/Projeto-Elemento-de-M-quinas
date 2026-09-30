clear; close all; clc;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Projeto Elementos de Máquinas                                           %
% Fase 2: Projeto de Mancais (Mancal Magnético)                           %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Parâmetros de entrada
B       = 0;
A       = 0;
mhi_0   = 4*pi*1e-7;     % Permeabilidade magnética do vácuo
ix      = 4;
i0      = 4;             % Corrente de Bias [A]
i1      = i0 + ix;       % Corrente elétrica
i2      = i0 - ix;
i_max   = i0 + ix;       % Corrente máxima [A]
N       = 0;             % Número de espiras
g       = 0;             % Entreferro
m       = 0;             % Massa do sistema


% Equações

F = (B^2 * A) / mhi_0; % Força magnética
B_0 = (mhi_0 * i * N)/2*g; % Densidade de fluxo magnético