clear; clc; close all;

%% ============================================================
%  CINEMÁTICA INVERSA DO UR5e COM SOMENTE JUNTAS 2 E 3 LIVRES
%
%  Entrada:
%     p_des = [x; y; z] em metros no referencial fixo da base {0}
%
%  Saídas:
%     - gráfico 3D do robô
%     - posição desejada
%     - posição final real
%     - erro absoluto e relativo
%     - ângulos finais das juntas
%
%  Convenção DH padrão:
%  A_i = Rot_z(theta_i)*Trans_z(d_i)*Trans_x(a_i)*Rot_x(alpha_i)
%% ============================================================

%% Entrada: ponto desejado no referencial da base {0}
% Altere aqui.
p_des = [0.32;
        -0.2329;
         0.35];

%% Configuração inicial
% Apenas q2 e q3 serão modificadas.
q0 = [0;
     -pi/2;
      0;
     -pi/2;
      0;
      0];

%% Juntas livres e juntas travadas
free_idx = [2 3];
fixed_idx = [1 4 5 6];

q_fixed = q0;

%% Parâmetros DH UR5e/UR7e
a = [0, -0.425, -0.3922, 0, 0, 0];

d = [0.1625, 0, 0, 0.1333, 0.0997, 0.0996];

alpha = [pi/2, 0, 0, pi/2, -pi/2, 0];

n = 6;

%% Parâmetros numéricos da IK
maxIter = 1000;
tol_abs = 1e-5;           % tolerância absoluta [m]
lambda = 1e-2;            % amortecimento
ganho = 0.8;              % ganho da atualização
maxStep = 0.08;           % passo máximo por iteração [rad]

q = q0;

erro_hist = zeros(maxIter,1);
q_hist = zeros(n,maxIter);

%% ============================================================
%  LOOP DE CINEMÁTICA INVERSA
%% ============================================================

for k = 1:maxIter

    % Garante que as juntas travadas não se movam
    q(fixed_idx) = q_fixed(fixed_idx);

    [T0, T_ee] = fkine_ur5e(q, a, d, alpha);

    p_atual = T_ee(1:3,4);

    erro = p_des - p_atual;
    erro_norm = norm(erro);

    erro_hist(k) = erro_norm;
    q_hist(:,k) = q;

    if erro_norm < tol_abs
        break;
    end

    % Jacobiana linear completa
    Jv = jacobiana_posicao(T0);

    % Usa apenas as colunas das juntas 2 e 3
    J_free = Jv(:,free_idx);

    % Pseudoinversa amortecida para sistema 3x2
    % Resolve no sentido de mínimos quadrados.
    dq_free = (J_free.'*J_free + lambda^2*eye(length(free_idx))) \ (J_free.'*erro);

    dq_free = ganho*dq_free;

    % Limita o passo
    if norm(dq_free) > maxStep
        dq_free = maxStep*dq_free/norm(dq_free);
    end

    % Atualiza apenas juntas 2 e 3
    q(free_idx) = q(free_idx) + dq_free;

    % Mantém ângulos em faixa conveniente
    q = wrapToPi_local(q);
end

%% Resultado final
q_sol = q;
q_sol(fixed_idx) = q_fixed(fixed_idx);

[T0_sol, T_ee_sol] = fkine_ur5e(q_sol, a, d, alpha);

p_final = T_ee_sol(1:3,4);
R_final = T_ee_sol(1:3,1:3);

erro_abs = norm(p_des - p_final);
erro_rel = erro_abs / max(norm(p_des), eps);

erro_hist = erro_hist(1:k);
q_hist = q_hist(:,1:k);

%% ============================================================
%  RESULTADOS NO COMMAND WINDOW
%% ============================================================

fprintf('\n============================================================\n');
fprintf('IK UR5e COM SOMENTE JUNTAS 2 E 3 LIVRES\n');
fprintf('============================================================\n');

fprintf('\nJuntas livres:\n');
disp(free_idx);

fprintf('Juntas travadas:\n');
disp(fixed_idx);

fprintf('\nPonto desejado p_des [m]:\n');
disp(p_des);

fprintf('Posição final real p_final [m]:\n');
disp(p_final);

fprintf('Erro absoluto final [m]: %.10f\n', erro_abs);
fprintf('Erro relativo final [-]: %.10f\n', erro_rel);
fprintf('Erro relativo final [%%]: %.6f %%\n', 100*erro_rel);
fprintf('Iterações executadas: %d\n', k);

fprintf('\nÂngulos finais q_sol [rad]:\n');
disp(q_sol);

fprintf('Ângulos finais q_sol [graus]:\n');
disp(rad2deg(q_sol));

fprintf('\nJuntas efetivamente modificadas:\n');
fprintf('q2 = %.8f rad = %.4f graus\n', q_sol(2), rad2deg(q_sol(2)));
fprintf('q3 = %.8f rad = %.4f graus\n', q_sol(3), rad2deg(q_sol(3)));

fprintf('\nJuntas mantidas fixas:\n');
fprintf('q1 = %.8f rad = %.4f graus\n', q_sol(1), rad2deg(q_sol(1)));
fprintf('q4 = %.8f rad = %.4f graus\n', q_sol(4), rad2deg(q_sol(4)));
fprintf('q5 = %.8f rad = %.4f graus\n', q_sol(5), rad2deg(q_sol(5)));
fprintf('q6 = %.8f rad = %.4f graus\n', q_sol(6), rad2deg(q_sol(6)));

fprintf('\nMatriz homogênea final T_0_6:\n');
disp(T_ee_sol);

if erro_abs > 5e-3
    warning(['Erro final maior que 5 mm. Isso pode significar que o ponto não pertence ', ...
             'ao espaço alcançável com apenas q2 e q3 livres.']);
end

%% ============================================================
%  GRÁFICO DE CONVERGÊNCIA
%% ============================================================

figure('Color','w');
plot(erro_hist, 'LineWidth', 2);
grid on;
xlabel('Iteração');
ylabel('Erro absoluto [m]');
title('Convergência da IK com apenas juntas 2 e 3 livres');

%% ============================================================
%  PLOT 3D DO ROBÔ
%% ============================================================

figure('Color','w');
hold on; grid on; axis equal;

xlabel('X_0 [m]');
ylabel('Y_0 [m]');
zlabel('Z_0 [m]');
title('UR5e - IK com juntas 2 e 3 livres');

view(135,25);

%% Origens dos referenciais
origens = zeros(3,n+1);

for i = 0:n
    origens(:,i+1) = T0_sol{i+1}(1:3,4);
end

%% Elos entre origens
plot3(origens(1,:), origens(2,:), origens(3,:), ...
      'k-o', 'LineWidth', 2, 'MarkerSize', 5);

%% Ponto desejado
plot3(p_des(1), p_des(2), p_des(3), ...
      'rp', 'MarkerSize', 14, 'MarkerFaceColor', 'r');

text(p_des(1), p_des(2), p_des(3), ...
     '  Ponto desejado', ...
     'FontSize', 10, 'FontWeight', 'bold');

%% Ponto final real
plot3(p_final(1), p_final(2), p_final(3), ...
      'bo', 'MarkerSize', 9, 'MarkerFaceColor', 'b');

text(p_final(1), p_final(2), p_final(3), ...
     '  TCP final real', ...
     'FontSize', 10, 'FontWeight', 'bold');

%% Linha de erro entre ponto final e ponto desejado
plot3([p_final(1) p_des(1)], ...
      [p_final(2) p_des(2)], ...
      [p_final(3) p_des(3)], ...
      'm--', 'LineWidth', 2);

%% Referenciais DH
escala_frame = 0.10;

for i = 0:n
    T_i = T0_sol{i+1};

    origem_i = T_i(1:3,4);
    R_i = T_i(1:3,1:3);

    draw_frame(origem_i, R_i, escala_frame, sprintf('{%d}', i));
end

%% Eixos reais de rotação das juntas
% No DH padrão, a junta i gira em torno de z_{i-1}.
escala_eixo_junta = 0.14;

for i = 1:n
    origem_junta = T0_sol{i}(1:3,4);
    eixo_junta = T0_sol{i}(1:3,3);

    if ismember(i, free_idx)
        estilo_linha = '-';
        largura = 3.2;
    else
        estilo_linha = '--';
        largura = 1.6;
    end

    quiver3(origem_junta(1), origem_junta(2), origem_junta(3), ...
            escala_eixo_junta*eixo_junta(1), ...
            escala_eixo_junta*eixo_junta(2), ...
            escala_eixo_junta*eixo_junta(3), ...
            'm', 'LineWidth', largura, 'MaxHeadSize', 0.8, ...
            'LineStyle', estilo_linha);

    if ismember(i, free_idx)
        label = sprintf('  J%d livre', i);
    else
        label = sprintf('  J%d fixa', i);
    end

    text(origem_junta(1), origem_junta(2), origem_junta(3), ...
         label, ...
         'FontSize', 10, 'FontWeight', 'bold');
end

%% Mostrar trajetória do TCP durante a iteração
traj_tcp = zeros(3,k);

for kk = 1:k
    [~, T_temp] = fkine_ur5e(q_hist(:,kk), a, d, alpha);
    traj_tcp(:,kk) = T_temp(1:3,4);
end

plot3(traj_tcp(1,:), traj_tcp(2,:), traj_tcp(3,:), ...
      'c-', 'LineWidth', 1.5);

%% Ajuste visual
lim = 1.0;
xlim([-lim lim]);
ylim([-lim lim]);
zlim([0 lim]);

legend({'Elos entre origens', ...
        'Ponto desejado', ...
        'TCP final real', ...
        'Erro final', ...
        'Trajetória iterativa do TCP'}, ...
        'Location', 'bestoutside');

%% Caixa de texto com resumo no gráfico
resumo = sprintf(['Erro abs = %.5f m\n', ...
                  'Erro rel = %.4f %%\n', ...
                  'q2 = %.2f°\n', ...
                  'q3 = %.2f°'], ...
                  erro_abs, 100*erro_rel, ...
                  rad2deg(q_sol(2)), rad2deg(q_sol(3)));

annotation('textbox', [0.15 0.72 0.22 0.16], ...
           'String', resumo, ...
           'FitBoxToText', 'on', ...
           'BackgroundColor', 'w', ...
           'EdgeColor', 'k');

fprintf('\nObservação importante:\n');
fprintf('Com apenas q2 e q3 livres, o manipulador tem somente 2 DOF ativos.\n');
fprintf('Logo, a posição desejada em 3D pode não ser exatamente alcançável.\n');
fprintf('O código retorna a melhor solução no sentido de mínimos quadrados.\n');

%% ============================================================
%  FUNÇÕES LOCAIS
%% ============================================================

function [T0, T_ee] = fkine_ur5e(q, a, d, alpha)

    n = 6;

    T0 = cell(n+1,1);
    T0{1} = eye(4);

    for i = 1:n
        A_i = dh_standard_num(a(i), alpha(i), d(i), q(i));
        T0{i+1} = T0{i} * A_i;
    end

    T_ee = T0{n+1};

end

function A = dh_standard_num(a, alpha, d, theta)

    A = [ cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta);
          sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
          0,           sin(alpha),             cos(alpha),            d;
          0,           0,                      0,                     1 ];

end

function Jv = jacobiana_posicao(T0)

    n = 6;

    p_ee = T0{n+1}(1:3,4);

    Jv = zeros(3,n);

    for j = 1:n
        z_jmenos1 = T0{j}(1:3,3);
        o_jmenos1 = T0{j}(1:3,4);

        Jv(:,j) = cross(z_jmenos1, p_ee - o_jmenos1);
    end

end

function draw_frame(origem, R, escala, nome)

    x_axis = R(:,1);
    y_axis = R(:,2);
    z_axis = R(:,3);

    % Eixo x - vermelho
    quiver3(origem(1), origem(2), origem(3), ...
            escala*x_axis(1), escala*x_axis(2), escala*x_axis(3), ...
            'r', 'LineWidth', 1.8, 'MaxHeadSize', 0.6);

    % Eixo y - verde
    quiver3(origem(1), origem(2), origem(3), ...
            escala*y_axis(1), escala*y_axis(2), escala*y_axis(3), ...
            'g', 'LineWidth', 1.8, 'MaxHeadSize', 0.6);

    % Eixo z - azul
    quiver3(origem(1), origem(2), origem(3), ...
            escala*z_axis(1), escala*z_axis(2), escala*z_axis(3), ...
            'b', 'LineWidth', 1.8, 'MaxHeadSize', 0.6);

    text(origem(1), origem(2), origem(3), ...
         ['  ', nome], ...
         'FontSize', 10, 'FontWeight', 'bold');

end

function q = wrapToPi_local(q)

    q = mod(q + pi, 2*pi) - pi;

end