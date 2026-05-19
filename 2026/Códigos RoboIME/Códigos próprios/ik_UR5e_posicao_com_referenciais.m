clear; clc; close all;

%% ============================================================
%  CINEMÁTICA INVERSA NUMÉRICA DO UR5e/UR7e
%  Entrada: ponto desejado p_des = [x; y; z] no referencial fixo da base {0}
%  Saída:   q_sol = [q1; q2; q3; q4; q5; q6]
%
%  Convenção DH padrão:
%  A_i = Rot_z(theta_i)*Trans_z(d_i)*Trans_x(a_i)*Rot_x(alpha_i)
%% ============================================================

%% Ponto desejado no referencial fixo da base {0}
% Unidades em metros.
% Altere aqui.
p_des = [0.35;
        -0.25;
         0.45];

%% Chute inicial das juntas
% Essa postura influencia a solução encontrada.
% Use uma postura próxima da que você espera fisicamente.
q0 = [0;
     -pi/2;
      0;
     -pi/2;
      0;
      0];

%% Parâmetros DH UR5e/UR7e
a = [0, -0.425, -0.3922, 0, 0, 0];

d = [0.1625, 0, 0, 0.1333, 0.0997, 0.0996];

alpha = [pi/2, 0, 0, pi/2, -pi/2, 0];

n = 6;

%% Parâmetros do método numérico
maxIter = 1000;
tol = 1e-5;              % tolerância de posição [m]
lambda = 1e-2;           % amortecimento
ganho = 0.8;             % ganho da atualização
maxStep = 0.12;          % passo máximo por iteração [rad]

q = q0;

erro_hist = zeros(maxIter,1);

%% ============================================================
%  LOOP DE CINEMÁTICA INVERSA
%% ============================================================

for k = 1:maxIter

    [T0, T_ee] = fkine_ur5e(q, a, d, alpha);

    p_atual = T_ee(1:3,4);

    erro = p_des - p_atual;
    erro_norm = norm(erro);

    erro_hist(k) = erro_norm;

    if erro_norm < tol
        break;
    end

    Jv = jacobiana_posicao(T0);

    % Pseudoinversa amortecida:
    % dq = Jv' * inv(Jv*Jv' + lambda^2 I) * erro
    dq = Jv.' * ((Jv*Jv.' + lambda^2*eye(3)) \ erro);

    dq = ganho*dq;

    % Limita passo para evitar saltos grandes
    if norm(dq) > maxStep
        dq = maxStep * dq / norm(dq);
    end

    q = q + dq;

    % Mantém ângulos em faixa conveniente
    q = wrapToPi_local(q);
end

q_sol = q;

[T0_sol, T_ee_sol] = fkine_ur5e(q_sol, a, d, alpha);
p_final = T_ee_sol(1:3,4);
R_final = T_ee_sol(1:3,1:3);

erro_final = norm(p_des - p_final);

erro_hist = erro_hist(1:k);

%% ============================================================
%  RESULTADOS NO COMMAND WINDOW
%% ============================================================

fprintf('\n============================================\n');
fprintf('RESULTADO DA CINEMÁTICA INVERSA\n');
fprintf('============================================\n');

fprintf('\nPonto desejado p_des [m]:\n');
disp(p_des);

fprintf('Ponto atingido p_final [m]:\n');
disp(p_final);

fprintf('Erro final [m]: %.8f\n', erro_final);
fprintf('Iterações: %d\n', k);

fprintf('\nq_sol [rad]:\n');
disp(q_sol);

fprintf('q_sol [graus]:\n');
disp(rad2deg(q_sol));

fprintf('\nMatriz homogênea final T_0_6:\n');
disp(T_ee_sol);

if erro_final > 5e-3
    warning('A solução final ficou com erro maior que 5 mm. Verifique se o ponto é alcançável ou altere q0.');
end

%% ============================================================
%  GRÁFICO DO ERRO
%% ============================================================

figure('Color','w');
plot(erro_hist, 'LineWidth', 2);
grid on;
xlabel('Iteração');
ylabel('Erro de posição [m]');
title('Convergência da cinemática inversa');

%% ============================================================
%  PLOT 3D DO ROBÔ, REFERENCIAIS E JUNTAS
%% ============================================================

figure('Color','w');
hold on; grid on; axis equal;

xlabel('X_0 [m]');
ylabel('Y_0 [m]');
zlabel('Z_0 [m]');
title('UR5e/UR7e - solução de cinemática inversa e referenciais DH');

view(135,25);

%% Origens dos referenciais
origens = zeros(3,n+1);

for i = 0:n
    origens(:,i+1) = T0_sol{i+1}(1:3,4);
end

%% Linha estrutural entre origens
plot3(origens(1,:), origens(2,:), origens(3,:), ...
      'k-o', 'LineWidth', 2, 'MarkerSize', 5);

%% Ponto desejado
plot3(p_des(1), p_des(2), p_des(3), ...
      'rp', 'MarkerSize', 14, 'MarkerFaceColor', 'r');

text(p_des(1), p_des(2), p_des(3), ...
     '  Ponto desejado', ...
     'FontSize', 10, 'FontWeight', 'bold');

%% Ponto atingido
plot3(p_final(1), p_final(2), p_final(3), ...
      'bo', 'MarkerSize', 9, 'MarkerFaceColor', 'b');

text(p_final(1), p_final(2), p_final(3), ...
     '  TCP atingido', ...
     'FontSize', 10, 'FontWeight', 'bold');

%% Referencial fixo da base e referenciais das juntas
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

    quiver3(origem_junta(1), origem_junta(2), origem_junta(3), ...
            escala_eixo_junta*eixo_junta(1), ...
            escala_eixo_junta*eixo_junta(2), ...
            escala_eixo_junta*eixo_junta(3), ...
            'm', 'LineWidth', 2.5, 'MaxHeadSize', 0.8);

    text(origem_junta(1), origem_junta(2), origem_junta(3), ...
         sprintf('  J%d', i), ...
         'FontSize', 10, 'FontWeight', 'bold');
end

%% Ajuste visual
lim = 0.9;
xlim([-lim lim]);
ylim([-lim lim]);
zlim([0 lim]);

legend({'Elos entre origens', ...
        'Ponto desejado', ...
        'TCP atingido', ...
        'Eixos das juntas'}, ...
        'Location', 'bestoutside');

fprintf('\nObservação:\n');
fprintf('A junta i gira em torno do eixo z_{i-1}.\n');
fprintf('Logo: J1 -> z0, J2 -> z1, J3 -> z2, J4 -> z3, J5 -> z4, J6 -> z5.\n');

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