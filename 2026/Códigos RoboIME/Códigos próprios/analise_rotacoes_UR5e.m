clear; clc; close all;

%% ============================================================
%  VISUALIZAÇÃO DOS REFERENCIAIS DH DO UR5e/UR7e
%  Baseado no modelo DH padrão:
%  A_i = Rot_z(theta_i)*Trans_z(d_i)*Trans_x(a_i)*Rot_x(alpha_i)
%% ============================================================

%% Configuração articular para visualização
% Altere aqui os ângulos das juntas.
% Exemplo em radianos:
q_plot = [0; -pi/2; 0; -pi/2; 0; 0];

% Se quiser usar tudo zerado, use:
% q_plot = zeros(6,1);

%% Parâmetros DH UR5e/UR7e
a = [0, -0.425, -0.3922, 0, 0, 0];

d = [0.1625, 0, 0, 0.1333, 0.0997, 0.0996];

alpha = [pi/2, 0, 0, pi/2, -pi/2, 0];

n = 6;

%% Cálculo das transformações homogêneas
A = cell(n,1);
T0 = cell(n+1,1);

T0{1} = eye(4);   % Referencial {0}, base

for i = 1:n
    A{i} = dh_standard_num(a(i), alpha(i), d(i), q_plot(i));
    T0{i+1} = T0{i} * A{i};   % T_0^i
end

%% Mostrar matrizes no Command Window
fprintf('\n============================================\n');
fprintf('REFERENCIAIS DH DO UR5e/UR7e\n');
fprintf('============================================\n');

for i = 0:n
    fprintf('\nT_0_%d = \n', i);
    disp(T0{i+1});
end

%% Plot 3D
figure('Color','w');
hold on; grid on; axis equal;
xlabel('X_0 [m]');
ylabel('Y_0 [m]');
zlabel('Z_0 [m]');
title('Referenciais DH do UR5e/UR7e');

view(135, 25);

%% Escala dos eixos desenhados
escala_frame = 0.12;

%% Plotar referenciais
for i = 0:n
    T = T0{i+1};
    origem = T(1:3,4);
    R = T(1:3,1:3);

    draw_frame(origem, R, escala_frame, sprintf('{%d}', i));
end

%% Plotar elos ligando as origens dos referenciais
origens = zeros(3,n+1);

for i = 0:n
    origens(:,i+1) = T0{i+1}(1:3,4);
end

plot3(origens(1,:), origens(2,:), origens(3,:), ...
      'k-o', 'LineWidth', 2, 'MarkerSize', 5);

%% Marcar eixos reais das juntas
% Em DH padrão, a junta i gira em torno de z_{i-1}.
for i = 1:n
    origem_junta = T0{i}(1:3,4);
    eixo_junta   = T0{i}(1:3,3);

    quiver3(origem_junta(1), origem_junta(2), origem_junta(3), ...
            0.08*eixo_junta(1), 0.08*eixo_junta(2), 0.08*eixo_junta(3), ...
            'LineWidth', 2, 'MaxHeadSize', 0.8);

    text(origem_junta(1), origem_junta(2), origem_junta(3), ...
         sprintf('  J%d', i), ...
         'FontSize', 10, 'FontWeight', 'bold');
end

legend({'x_i', 'y_i', 'z_i', 'elos'}, 'Location', 'bestoutside');

fprintf('\nObservação:\n');
fprintf('No DH padrão, a junta i gira em torno do eixo z_{i-1}.\n');
fprintf('Portanto, a Junta 1 gira em z_0, a Junta 2 em z_1, ..., a Junta 6 em z_5.\n');

%% ============================================================
% Função local: matriz DH padrão numérica
%% ============================================================
function A = dh_standard_num(a, alpha, d, theta)

A = [ cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta);
      sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
      0,           sin(alpha),             cos(alpha),            d;
      0,           0,                      0,                     1 ];

end

%% ============================================================
% Função local: desenhar referencial
%% ============================================================
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

% Nome do referencial
text(origem(1), origem(2), origem(3), ...
     ['  ', nome], ...
     'FontSize', 10, 'FontWeight', 'bold');

end