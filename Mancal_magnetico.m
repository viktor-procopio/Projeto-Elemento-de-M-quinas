clear; close all; clc;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Projeto Elementos de Máquinas                                           %
% Fase 2: Projeto de Mancais (Mancal Magnético)                           %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% 1. Parâmetros Comuns aos Dois Mancais
mhi_0 = 4*pi*1e-7; % Permeabilidade magnética do vácuo [T.m/A]
g0    = 1e-3;      % Entreferro constante para todos os eixos [m]

% 2. Demandas de Força (Condição 1)
Fmax_vert_MG1 = abs(-17.9569 + -4.9049);
Fmax_rad_MG1  = abs(-790.4523 + -215.9106);
Fmax_vert_MG2 = abs(1.0126e+04 + 2.7659e+03);
Fmax_rad_MG2  = abs(-659.8267 + -180.2304);

% =========================================================================
% 3. PARÂMETROS ASSIMÉTRICOS POR EIXO E MANCAL
% =========================================================================
% --- Mancal 1 (MG1) - Eixo Vertical (Carga Mínima: ~23 N) ---
A_MG1_v  = 0.5e-3;  N_MG1_v  = 50;   i0_MG1_v = 1.5; ix_MG1_v = 1.5;
i1_MG1_v = i0_MG1_v + ix_MG1_v;  i2_MG1_v = i0_MG1_v - ix_MG1_v;

% --- Mancal 1 (MG1) - Eixo Radial (Carga Média: ~1.000 N) ---
A_MG1_r  = 1.5e-3;  N_MG1_r  = 120;  i0_MG1_r = 3;   ix_MG1_r = 3;
i1_MG1_r = i0_MG1_r + ix_MG1_r;  i2_MG1_r = i0_MG1_r - ix_MG1_r;

% --- Mancal 2 (MG2) - Eixo Vertical (Carga Crítica: ~12.890 N) ---
A_MG2_v  = 3.0e-3;  N_MG2_v  = 200;  i0_MG2_v = 4;   ix_MG2_v = 4;
i1_MG2_v = i0_MG2_v + ix_MG2_v;  i2_MG2_v = i0_MG2_v - ix_MG2_v;

% --- Mancal 2 (MG2) - Eixo Radial (Carga Leve: ~840 N) ---
A_MG2_r  = 1.5e-3;  N_MG2_r  = 110;  i0_MG2_r = 3;   ix_MG2_r = 3;
i1_MG2_r = i0_MG2_r + ix_MG2_r;  i2_MG2_r = i0_MG2_r - ix_MG2_r;

% =========================================================================
% 4. CÁLCULO DE CAPACIDADE E FATOR DE SEGURANÇA
% =========================================================================
% Constantes Magnéticas (Km)
Km_MG1_v = (mhi_0 * A_MG1_v * N_MG1_v^2) / 4;
Km_MG1_r = (mhi_0 * A_MG1_r * N_MG1_r^2) / 4;
Km_MG2_v = (mhi_0 * A_MG2_v * N_MG2_v^2) / 4;
Km_MG2_r = (mhi_0 * A_MG2_r * N_MG2_r^2) / 4;

% Capacidade Máxima de Força (x = 0)
Fcap_MG1_v = Km_MG1_v * ( (i1_MG1_v^2 / g0^2) - (i2_MG1_v^2 / g0^2) );
Fcap_MG1_r = Km_MG1_r * ( (i1_MG1_r^2 / g0^2) - (i2_MG1_r^2 / g0^2) );
Fcap_MG2_v = Km_MG2_v * ( (i1_MG2_v^2 / g0^2) - (i2_MG2_v^2 / g0^2) );
Fcap_MG2_r = Km_MG2_r * ( (i1_MG2_r^2 / g0^2) - (i2_MG2_r^2 / g0^2) );

% Fatores de Segurança
FS_MG1_v = Fcap_MG1_v / Fmax_vert_MG1;
FS_MG1_r = Fcap_MG1_r / Fmax_rad_MG1;
FS_MG2_v = Fcap_MG2_v / Fmax_vert_MG2;
FS_MG2_r = Fcap_MG2_r / Fmax_rad_MG2;

% =========================================================================
% 5. EXIBIÇÃO
% =========================================================================
disp('=======================================================================');
disp('             ANÁLISE DE FATOR DE SEGURANÇA (Alvo: 2.0 a 3.0)           ');
disp('=======================================================================');
fprintf('MG1 Vertical: %7.2f  |  (Demanda: %7.1f N | Capacidade: %7.1f N)\n', FS_MG1_v, Fmax_vert_MG1, Fcap_MG1_v);
fprintf('MG1 Radial:   %7.2f  |  (Demanda: %7.1f N | Capacidade: %7.1f N)\n', FS_MG1_r, Fmax_rad_MG1, Fcap_MG1_r);
fprintf('MG2 Vertical: %7.2f  |  (Demanda: %7.1f N | Capacidade: %7.1f N)\n', FS_MG2_v, Fmax_vert_MG2, Fcap_MG2_v);
fprintf('MG2 Radial:   %7.2f  |  (Demanda: %7.1f N | Capacidade: %7.1f N)\n', FS_MG2_r, Fmax_rad_MG2, Fcap_MG2_r);
disp('-----------------------------------------------------------------------');
