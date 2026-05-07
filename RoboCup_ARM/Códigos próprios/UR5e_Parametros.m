clear; clc;
tic;

%% ============================================================
%  MODELO SIMBÓLICO UR5e/UR7e POR DENAVIT-HARTENBERG PADRÃO
%  Gera:
%     T      -> cinemática direta base-flange
%     Jacobi -> Jacobiana geométrica do flange/TCP nominal
%     M      -> matriz de inércia
%     C      -> matriz de Coriolis/centrífuga
%     G      -> vetor gravitacional
%  Salva:
%     UR5e.mat
%% ============================================================

%% Controle de custo simbólico
gerarCoriolis = true;   % Se estiver demorando muito, mude para false.
salvarExtras  = true;

%% Variáveis simbólicas
syms q_1 q_2 q_3 q_4 q_5 q_6 real
syms dq_1 dq_2 dq_3 dq_4 dq_5 dq_6 real
syms g real

q  = [q_1;  q_2;  q_3;  q_4;  q_5;  q_6];
dq = [dq_1; dq_2; dq_3; dq_4; dq_5; dq_6];

n = 6;

%% Parâmetros DH padrão do UR5e/UR7e
% A_i = Rot_z(theta_i)*Trans_z(d_i)*Trans_x(a_i)*Rot_x(alpha_i)

a = [0, -0.425, -0.3922, 0, 0, 0];

d = [0.1625, 0, 0, 0.1333, 0.0997, 0.0996];

alpha = [pi/2, 0, 0, pi/2, -pi/2, 0];

%% Parâmetros dinâmicos
m = [3.761, 8.058, 2.846, 1.37, 1.3, 0.365];

pc = [ 0,        0.2125, 0.15,     0,       0,        0;
      -0.02561, 0,      0,       -0.0018,  0.0018,   0;
       0.00193, 0.11336,0.0265,   0.01634, 0.01634, -0.001159];

Icm = cell(n,1);
for i = 1:n
    Icm{i} = sym(zeros(3,3));
end

Icm{6} = sym([0, 0, 0;
              0, 0, 0;
              0, 0, 0.0002]);

%% Transformações homogêneas DH
A  = cell(n,1);
T0 = cell(n+1,1);

T0{1} = sym(eye(4));   % T_0^0

for i = 1:n
    A{i} = dh_standard(a(i), alpha(i), d(i), q(i));
    T0{i+1} = T0{i} * A{i};   % T_0^i
end

T = T0{n+1};           % T_0^6
R = T(1:3,1:3);
p = T(1:3,4);

%% Eixos z e origens de cada junta no sistema da base
z = cell(n,1);
o = cell(n,1);

for j = 1:n
    z{j} = T0{j}(1:3,3);   % z_{j-1}
    o{j} = T0{j}(1:3,4);   % o_{j-1}
end

%% Jacobiana geométrica do flange/TCP nominal
Jv = sym(zeros(3,n));
Jw = sym(zeros(3,n));

for j = 1:n
    Jv(:,j) = cross(z{j}, p - o{j});
    Jw(:,j) = z{j};
end

Jacobi = [Jv; Jw];

%% Posições dos centros de massa e Jacobianas dos centros de massa
pc0 = sym(zeros(3,n));
Jv_c = cell(n,1);
Jw_c = cell(n,1);
R0   = cell(n,1);

for i = 1:n
    R0{i} = T0{i+1}(1:3,1:3);

    temp = T0{i+1} * [pc(:,i); 1];
    pc0(:,i) = temp(1:3);

    Jv_c{i} = sym(zeros(3,n));
    Jw_c{i} = sym(zeros(3,n));

    for j = 1:n
        if j <= i
            Jv_c{i}(:,j) = cross(z{j}, pc0(:,i) - o{j});
            Jw_c{i}(:,j) = z{j};
        end
    end
end

%% Matriz de inércia M(q)
M = sym(zeros(n,n));

for i = 1:n
    M = M ...
      + Jv_c{i}.' * m(i) * eye(3) * Jv_c{i} ...
      + Jw_c{i}.' * R0{i} * Icm{i} * R0{i}.' * Jw_c{i};
end

%% Matriz de Coriolis/Centrífuga C(q,dq)
C = sym(zeros(n,n));

if gerarCoriolis
    for k = 1:n
        for s = 1:n
            termo = sym(0);
            for r = 1:n
                termo = termo + 0.5 * ...
                    ( diff(M(k,s), q(r)) ...
                    + diff(M(k,r), q(s)) ...
                    - diff(M(r,s), q(k)) ) * dq(r);
            end
            C(k,s) = termo;
        end
    end
end

%% Vetor gravitacional G(q)
% Mantive o mesmo sentido usado no código anterior: gravidade em -Z.
% Se sua simulação estiver invertida, troque para grav0 = [0; 0; g].
grav0 = [0; 0; -g];

P = sym(0);
for i = 1:n
    P = P + m(i) * grav0.' * pc0(:,i);
end

G = jacobian(P, q).';

%% Variáveis auxiliares úteis
p_ee = p;
R_ee = R;

Jacobi_com6 = [Jv_c{6}; Jw_c{6}];

params.a = a;
params.d = d;
params.alpha = alpha;
params.m = m;
params.pc = pc;
params.Icm = Icm;

%% Salvar modelo
if salvarExtras
    save('UR5e.mat', ...
        'T', 'Jacobi', 'M', 'C', 'G', ...
        'q', 'dq', 'g', ...
        'p_ee', 'R_ee', ...
        'pc0', 'Jv_c', 'Jw_c', ...
        'Jacobi_com6', ...
        'params');
else
    save('UR5e.mat', 'T', 'Jacobi', 'M', 'C', 'G');
end

fprintf('\nModelo UR5e salvo em UR5e.mat\n');
fprintf('Tempo total: %.2f s\n', toc);

%% ============================================================
% Função local: DH padrão
%% ============================================================
function A = dh_standard(a, alpha, d, theta)

A = [ cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta);
      sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
      0,           sin(alpha),             cos(alpha),            d;
      0,           0,                      0,                     1 ];

A = sym(A);

end