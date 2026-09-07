clear; close all; clc;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Projeto Elementos de Máquinas                                           %
% Fase 1: Projeto de Eixo de Saída                                        %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% --- DIMENSÕES DE PROJETO ---
D1 = 225e-3; % [m] G1
D2 = 250e-3; % [m] G2
Dp = 150e-3; % [m] polia
phi = 25;    % [°] Ângulo de Pressão
vp2 = 2.36;  % [m/s] v na condição 2
rfp = 5;     % Razão de força da Polia

% Comprimentos
L1 = 95e-3; L2 = 125e-3; L3 = 150e-3; L4 = 120e-3; L5 = 100e-3; L6 = 60e-3;
l_d1 = 19e-3; l_d2 = 23e-3; l_d3 = 16e-3; l_d4 = 19e-3; l_d5 = 25e-3;

fs_e = 3.5; % fator de segurança mínimo do eixo
P = 12.5 * 0.746; % [kW] Potência do motor
def_max = 90e-6; % [micrômetro] Deflexão máxima

% --- CÁLCULO DE ESFORÇOS ---
omega_p = vp2 / (Dp / 2); 
T_G2 = (P * 1000) / omega_p; 
Fp_n = T_G2 / (Dp / 2); 
Fp_s = 1.5 * Fp_n; 
Fg2_tang = T_G2 / (D2 / 2); 
Fg2_radial = Fg2_tang * tand(phi); 

% Reações
A = (l_d4 + L4 + L3 + L2 + l_d1); 
B = Fg2_tang*(l_d1 + L2 + L3 + l_d3) + Fp_s*(l_d5 + L5 + L4 + L3 + L2 + l_d1); 
Rmg2_vertical = B / A;
Rmg1_vertical = Fg2_tang - Rmg2_vertical + Fp_s;
Rmg2_radial = -Fg2_radial*(l_d1 + L2 + L3 + l_d3) / A;
Rmg1_radial = -Fg2_radial - Rmg2_radial; 

% --- DIAGRAMAS DE ESFORÇO ---
x_m1 = 0;                                     % Mancal 1
x_g2 = l_d1 + L2 + L3 + l_d3;                 % Engrenagem G2
x_m2 = l_d1 + L2 + L3 + L4 + l_d4;            % Mancal 2
x_p  = l_d1 + L2 + L3 + L4 + L5 + l_d5;       % Polia
L_total = L1 + L2 + L3 + L4 + L5 + L6;        

sing = @(x, a, n) (x >= a) .* ((x - a).^n);
x = linspace(-L1, L_total, 2000); 

V_z = Rmg1_vertical * sing(x, x_m1, 0) - Fg2_tang * sing(x, x_g2, 0) + Rmg2_vertical * sing(x, x_m2, 0) - Fp_s * sing(x, x_p, 0);
V_y = Rmg1_radial * sing(x, x_m1, 0) + Fg2_radial * sing(x, x_g2, 0) + Rmg2_radial * sing(x, x_m2, 0);
M_z = Rmg1_vertical * sing(x, x_m1, 1) - Fg2_tang * sing(x, x_g2, 1) + Rmg2_vertical * sing(x, x_m2, 1) - Fp_s * sing(x, x_p, 1);
M_y = Rmg1_radial * sing(x, x_m1, 1) + Fg2_radial * sing(x, x_g2, 1) + Rmg2_radial * sing(x, x_m2, 1);
M_res = sqrt(M_z.^2 + M_y.^2);

% Vetor de Torque (Ativo apenas entre G2 e Polia)
T_vec = zeros(size(x));
T_vec(x >= x_g2 & x <= x_p) = T_G2;

% --- OTIMIZAÇÃO DOS DIÂMETROS À FADIGA ---
Sut = 627e6; % SAE 1045
Sut_MPa = Sut / 1e6;
ka = 4.51 * (Sut_MPa)^(-0.265); 
Se_linha = ka * (0.5 * Sut);    

% Fatiamento do eixo para extrair os esforços críticos de cada seção
b0 = -L1; b1 = 0; b2 = b1 + L2; b3 = b2 + L3; b4 = b3 + L4; b5 = b4 + L5; b6 = b5 + L6;
idx{1} = (x >= b0 & x <= b1);
idx{2} = (x > b1  & x <= b2);
idx{3} = (x > b2  & x <= b3);
idx{4} = (x > b3  & x <= b4);
idx{5} = (x > b4  & x <= b5);
idx{6} = (x > b5  & x <= b6);

M_sec = zeros(1,6); T_sec = zeros(1,6); d_min = zeros(1,6);
for i = 1:6
    M_sec(i) = max(M_res(idx{i}));
    T_sec(i) = max(T_vec(idx{i}));
    d_min(i) = calcular_diametro_fadiga(M_sec(i), 0, 0, T_sec(i), Sut, Se_linha, fs_e);
end

% --- RESTRIÇÕES GEOMÉTRICAS (DEGRAUS) ---
degrau = 0.005; 
D1_final = max(d_min(1), d_min(5));
D5_final = D1_final;
D6_final = d_min(6);
if D5_final < (D6_final + degrau)
    D5_final = D6_final + degrau; D1_final = D5_final;
end
D4_final = max(d_min(4), D5_final + degrau);
D2_final = max(d_min(2), D1_final + degrau);
D3_final = max([d_min(3), D4_final + degrau, D2_final + degrau]);

fprintf('=== DIÂMETROS INICIAIS (FADIGA) ===\n');
fprintf('D1 = D5 = %.2f mm\n', D1_final * 1000);
fprintf('D2 = %.2f mm\n', D2_final * 1000);
fprintf('D3 = %.2f mm\n', D3_final * 1000);
fprintf('D4 = %.2f mm\n', D4_final * 1000);
fprintf('D6 = %.2f mm\n', D6_final * 1000);

% --- ANÁLISE DE DEFLEXÃO (LINHA ELÁSTICA INICIAL) ---
E = 200e9; 
D_vetor = zeros(size(x));
D_vetor(x <= b1) = D1_final;
D_vetor(x > b1 & x <= b2) = D2_final;
D_vetor(x > b2 & x <= b3) = D3_final;
D_vetor(x > b3 & x <= b4) = D4_final;
D_vetor(x > b4 & x <= b5) = D5_final;
D_vetor(x > b5) = D6_final; 

I_vetor = (pi .* D_vetor.^4) ./ 64;
curvatura = M_res ./ (E .* I_vetor);
theta_bruto = cumtrapz(x, curvatura);
y_bruto = cumtrapz(x, theta_bruto);

% Índices dos mancais para zerar a deflexão neles
idx_A = find(x >= x_m1, 1);
idx_B = find(x >= x_m2, 1);

C1 = - (y_bruto(idx_B) - y_bruto(idx_A)) / (x(idx_B) - x(idx_A));
C2 = - y_bruto(idx_A) - C1 * x(idx_A);
y_final = y_bruto + C1 .* x + C2;

deflexao_maxima = max(abs(y_final)) * 1e6; 
fprintf('\n=== ANÁLISE DE RIGIDEZ INICIAL ===\n');
fprintf('Deflexão máxima calculada: %.2f µm\n', deflexao_maxima);

figure('Name', 'Linha Elástica do Eixo');
plot(x, y_final * 1000, 'r', 'LineWidth', 2);
title('Linha Elástica (Deflexão Transversal)');
xlabel('Posição x (m)'); ylabel('Deflexão (mm)'); grid on;
hold on; plot([x_m1, x_m2], [0, 0], 'k^', 'MarkerSize', 10, 'MarkerFaceColor', 'k');

% --- CORREÇÃO ÚNICA DE RIGIDEZ ---
if deflexao_maxima > def_max * 1e6
    fprintf('\n=== RECALCULANDO DIÂMETROS PARA RIGIDEZ ===\n');
    
    % Fator com margem de 3% (1.03) para absorver perdas por degraus fixos
    Kd = 1.03 * (deflexao_maxima / (def_max * 1e6))^(1/4);
    fprintf('Fator de escala aplicado: %.3f\n\n', Kd);
    
    d_min_rig = d_min .* Kd;
    
    D1_final = max(d_min_rig(1), d_min_rig(5));
    D5_final = D1_final;
    D6_final = d_min_rig(6);
    if D5_final < (D6_final + degrau)
        D5_final = D6_final + degrau; D1_final = D5_final;
    end
    D4_final = max(d_min_rig(4), D5_final + degrau);
    D2_final = max(d_min_rig(2), D1_final + degrau);
    D3_final = max([d_min_rig(3), D4_final + degrau, D2_final + degrau]);
    
    fprintf('Novos Diâmetros Teóricos Corrigidos:\n');
    fprintf('D1 = D5 = %.2f mm\n', D1_final * 1000);
    fprintf('D2 = %.2f mm\n', D2_final * 1000);
    fprintf('D3 = %.2f mm\n', D3_final * 1000);
    fprintf('D4 = %.2f mm\n', D4_final * 1000);
    fprintf('D6 = %.2f mm\n', D6_final * 1000);
    
    % Recálculo com novos diâmetros
    D_vetor_rig = zeros(size(x));
    D_vetor_rig(x <= b1) = D1_final;
    D_vetor_rig(x > b1 & x <= b2) = D2_final;
    D_vetor_rig(x > b2 & x <= b3) = D3_final;
    D_vetor_rig(x > b3 & x <= b4) = D4_final;
    D_vetor_rig(x > b4 & x <= b5) = D5_final;
    D_vetor_rig(x > b5) = D6_final; 
    
    I_vetor_rig = (pi .* D_vetor_rig.^4) ./ 64;
    curvatura_rig = M_res ./ (E .* I_vetor_rig);
    
    theta_bruto_rig = cumtrapz(x, curvatura_rig);
    y_bruto_rig = cumtrapz(x, theta_bruto_rig);
    
    C1_rig = - (y_bruto_rig(idx_B) - y_bruto_rig(idx_A)) / (x(idx_B) - x(idx_A));
    C2_rig = - y_bruto_rig(idx_A) - C1_rig * x(idx_A);
    y_final_rig = y_bruto_rig + C1_rig .* x + C2_rig;
    
    deflexao_maxima_rig = max(abs(y_final_rig)) * 1e6; 
    
    fprintf('\n=== VERIFICAÇÃO DE RIGIDEZ FINAL ===\n');
    fprintf('Nova deflexão máxima calculada: %.2f µm\n', deflexao_maxima_rig);
    
    if deflexao_maxima_rig <= def_max * 1e6
        fprintf('SUCESSO: Eixo operando dentro da deflexão segura!\n');
    else
        fprintf('ALERTA: Deflexão ainda excede o limite.\n');
    end
    
    plot(x, y_final_rig * 1000, 'b--', 'LineWidth', 2);
    legend('Deflexão Inicial (Falha)', 'Mancais', 'Deflexão Corrigida (Segura)');
end

% =========================================================================
% FUNÇÃO LOCAL: CÁLCULO ITERATIVO À FADIGA
% =========================================================================
function d_opt = calcular_diametro_fadiga(Ma, Mm, Ta, Tm, Sut, Se_linha, fs)
    tol = 1e-4; erro = 1; d_opt = 20e-3;
    Kf = 2.0; Kfs = 1.5; 
    
    while erro > tol
        d_old = d_opt;
        if d_opt >= 2.79e-3 && d_opt <= 51e-3
            kb = 1.24 * (d_opt * 1000)^(-0.107);
        else
            kb = 1.51 * (d_opt * 1000)^(-0.157);
        end
        Se = Se_linha * kb; 
        
        termo_alt = sqrt((Kf * Ma)^2 + 0.75 * (Kfs * Ta)^2) / Se;
        termo_med = sqrt((Kf * Mm)^2 + 0.75 * (Kfs * Tm)^2) / Sut;
        
        d_opt = ((16 * fs / pi) * (termo_alt + termo_med))^(1/3);
        erro = abs(d_opt - d_old);
    end
end