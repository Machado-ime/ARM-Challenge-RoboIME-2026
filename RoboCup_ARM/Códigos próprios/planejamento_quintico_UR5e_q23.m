clear; clc; close all;

%% ============================================================
%  PLANEJAMENTO QUÍNTICO DE TRAJETÓRIA - UR5e
%  Modelo reduzido: somente juntas 2 e 3 ativas
%
%  Entrada:
%     p_des = [x; y; z] no referencial fixo da base {0}
%
%  Saídas:
%     - cinemática inversa reduzida
%     - trajetória quíntica q(t), dq(t), ddq(t)
%     - torque estimado
%     - potência mecânica
%     - energia acumulada
%     - parâmetros para seleção preliminar de motores
%% ============================================================

%% ============================================================
%  ENTRADA DO PROBLEMA
%% ============================================================

% Posição desejada do TCP em relação à base {0} [m]
p_des = [0.35;
        -0.2329;
         0.40];

% Configuração inicial do manipulador [rad]
q0 = [0;
     -pi/2;
      0;
     -pi/2;
      0;
      0];

% Juntas livres
free_idx = [2 3];

% Juntas travadas
fixed_idx = [1 4 5 6];

% Tempos de operação a comparar [s]
Tf_list = [2 5 10];

% Número de pontos da trajetória
Npts = 401;

% Gravidade [m/s^2]
gval = 9.81;

% Carga pontual no TCP [kg]
% Use 0 se quiser analisar sem carga.
m_payload = 5.0;

% Vetor gravidade: z positivo para cima
grav0 = [0; 0; -gval];

%% Parâmetros para estimativa no eixo do motor
% Nred = relação de redução.
% Se ainda não sabe a redução, mantenha Nred = 1 para obter valores na junta.
Nred = [1; 1];

% Eficiência mecânica estimada da transmissão
eta = [0.75; 0.75];

% Fator de segurança para pré-seleção de motores
FS = 1.5;

%% Controle de custo computacional
usar_dinamica_elos = true;

% Coriolis por diferença finita deixa o código mais lento.
% Para primeira análise de motor, deixe false.
usar_coriolis = false;

%% ============================================================
%  PARÂMETROS DH E DINÂMICOS DO MODELO ATUAL
%% ============================================================

params.n = 6;

params.a = [0, -0.425, -0.3922, 0, 0, 0];

params.d = [0.1625, 0, 0, 0.1333, 0.0997, 0.0996];

params.alpha = [pi/2, 0, 0, pi/2, -pi/2, 0];

params.m = [3.761, 8.058, 2.846, 1.37, 1.3, 0.365];

params.pc = [ 0,        0.2125, 0.15,     0,       0,        0;
             -0.02561, 0,      0,       -0.0018,  0.0018,   0;
              0.00193, 0.11336,0.0265,   0.01634, 0.01634, -0.001159];

params.Icm = zeros(3,3,6);

params.Icm(:,:,6) = [0, 0, 0;
                     0, 0, 0;
                     0, 0, 0.0002];

%% ============================================================
%  CINEMÁTICA INVERSA REDUZIDA
%% ============================================================

maxIter = 1000;
tol_abs = 1e-5;
lambda = 1e-2;
ganho = 0.8;
maxStep = 0.08;

q = q0;
erro_hist = zeros(maxIter,1);

for k = 1:maxIter

    q(fixed_idx) = q0(fixed_idx);

    [T0, T_ee] = fkine_ur5e(q, params);
    p_atual = T_ee(1:3,4);

    erro = p_des - p_atual;
    erro_norm = norm(erro);
    erro_hist(k) = erro_norm;

    if erro_norm < tol_abs
        break;
    end

    Jv = jacobiana_posicao(T0);
    J_free = Jv(:,free_idx);

    dq_free = (J_free.'*J_free + lambda^2*eye(length(free_idx))) \ (J_free.'*erro);

    dq_free = ganho*dq_free;

    if norm(dq_free) > maxStep
        dq_free = maxStep*dq_free/norm(dq_free);
    end

    q(free_idx) = q(free_idx) + dq_free;
    q = wrapToPi_local(q);
end

q_goal = q;
q_goal(fixed_idx) = q0(fixed_idx);

[T0_goal, T_goal] = fkine_ur5e(q_goal, params);
p_final_ik = T_goal(1:3,4);

erro_abs_ik = norm(p_des - p_final_ik);
erro_rel_ik = erro_abs_ik / max(norm(p_des), eps);

fprintf('\n============================================================\n');
fprintf('RESULTADO DA CINEMÁTICA INVERSA REDUZIDA\n');
fprintf('============================================================\n');

fprintf('\nPonto desejado p_des [m]:\n');
disp(p_des);

fprintf('Posição final real obtida pela IK [m]:\n');
disp(p_final_ik);

fprintf('Erro absoluto final da IK [m]: %.10f\n', erro_abs_ik);
fprintf('Erro relativo final da IK [-]: %.10f\n', erro_rel_ik);
fprintf('Erro relativo final da IK [%%]: %.6f %%\n', 100*erro_rel_ik);
fprintf('Iterações da IK: %d\n', k);

fprintf('\nq0 [rad]:\n');
disp(q0);

fprintf('q_goal [rad]:\n');
disp(q_goal);

fprintf('q_goal [graus]:\n');
disp(rad2deg(q_goal));

fprintf('\nJuntas ativas:\n');
fprintf('q2_goal = %.8f rad = %.4f graus\n', q_goal(2), rad2deg(q_goal(2)));
fprintf('q3_goal = %.8f rad = %.4f graus\n', q_goal(3), rad2deg(q_goal(3)));

if erro_abs_ik > 5e-3
    warning(['Erro final da IK maior que 5 mm. Com apenas q2 e q3 livres, ', ...
             'nem todo ponto 3D é exatamente alcançável.']);
end

%% ============================================================
%  GERAÇÃO DAS TRAJETÓRIAS QUÍNTICAS
%% ============================================================

resultados = struct();
linhas_tabela = {};

for r = 1:length(Tf_list)

    Tf = Tf_list(r);

    t = linspace(0, Tf, Npts);
    tau = t/Tf;

    s    = 10*tau.^3 - 15*tau.^4 + 6*tau.^5;
    ds   = 30*tau.^2 - 60*tau.^3 + 30*tau.^4;
    dds  = 60*tau - 180*tau.^2 + 120*tau.^3;

    Q   = zeros(6,Npts);
    Qd  = zeros(6,Npts);
    Qdd = zeros(6,Npts);

    delta_q = q_goal - q0;

    for i = 1:Npts
        Q(:,i)   = q0 + s(i)*delta_q;
        Qd(:,i)  = (ds(i)/Tf)*delta_q;
        Qdd(:,i) = (dds(i)/Tf^2)*delta_q;

        Q(fixed_idx,i)   = q0(fixed_idx);
        Qd(fixed_idx,i)  = 0;
        Qdd(fixed_idx,i) = 0;
    end

    %% Cinemática ao longo da trajetória
    p_tcp = zeros(3,Npts);
    Jv_all = zeros(3,6,Npts);

    for i = 1:Npts
        [T0_i, T_ee_i] = fkine_ur5e(Q(:,i), params);
        p_tcp(:,i) = T_ee_i(1:3,4);
        Jv_all(:,:,i) = jacobiana_posicao(T0_i);
    end

    %% Aceleração cartesiana aproximada do TCP
    v_tcp = zeros(3,Npts);
    a_tcp = zeros(3,Npts);

    for ax = 1:3
        v_tcp(ax,:) = gradient(p_tcp(ax,:), t);
        a_tcp(ax,:) = gradient(v_tcp(ax,:), t);
    end

    %% Torque, potência e energia
    Tau_robo = zeros(6,Npts);
    Tau_payload = zeros(6,Npts);
    Tau_total = zeros(6,Npts);
    P_joint = zeros(6,Npts);

    for i = 1:Npts

        q_i   = Q(:,i);
        dq_i  = Qd(:,i);
        ddq_i = Qdd(:,i);

        if usar_dinamica_elos
            Tau_robo(:,i) = invdyn_numeric(q_i, dq_i, ddq_i, params, gval, usar_coriolis);
        else
            Tau_robo(:,i) = zeros(6,1);
        end

        if m_payload > 0
            % Força que o robô precisa aplicar para sustentar e acelerar a carga:
            % F = m*(a_tcp - gravidade)
            F_payload = m_payload*(a_tcp(:,i) - grav0);

            Tau_payload(:,i) = Jv_all(:,:,i).' * F_payload;
        end

        Tau_total(:,i) = Tau_robo(:,i) + Tau_payload(:,i);

        P_joint(:,i) = Tau_total(:,i).*dq_i;
    end

    P_total_abs = sum(abs(P_joint(free_idx,:)),1);
    E_total = cumtrapz(t, P_total_abs);

    %% Parâmetros para seleção de motores
    tau_active = Tau_total(free_idx,:);
    qd_active = Qd(free_idx,:);
    qdd_active = Qdd(free_idx,:);
    P_active = P_joint(free_idx,:);

    tau_peak = max(abs(tau_active),[],2);
    tau_rms = sqrt(trapz(t, tau_active.^2, 2)/Tf);

    omega_peak = max(abs(qd_active),[],2);
    alpha_peak = max(abs(qdd_active),[],2);

    P_peak = max(abs(P_active),[],2);
    E_joint = trapz(t, abs(P_active), 2);

    tau_motor_peak = FS*tau_peak./(eta.*Nred);
    tau_motor_rms = FS*tau_rms./(eta.*Nred);
    omega_motor_peak = Nred.*omega_peak;

    P_motor_peak_est = FS*P_peak./eta;

    for jj = 1:length(free_idx)
        junta = free_idx(jj);

        linhas_tabela(end+1,:) = {Tf, junta, ...
            tau_peak(jj), tau_rms(jj), ...
            omega_peak(jj), alpha_peak(jj), ...
            P_peak(jj), E_joint(jj), ...
            tau_motor_peak(jj), tau_motor_rms(jj), ...
            omega_motor_peak(jj), P_motor_peak_est(jj)};
    end

    %% Armazenar resultados
    resultados(r).Tf = Tf;
    resultados(r).t = t;
    resultados(r).Q = Q;
    resultados(r).Qd = Qd;
    resultados(r).Qdd = Qdd;
    resultados(r).p_tcp = p_tcp;
    resultados(r).Tau_robo = Tau_robo;
    resultados(r).Tau_payload = Tau_payload;
    resultados(r).Tau_total = Tau_total;
    resultados(r).P_joint = P_joint;
    resultados(r).P_total_abs = P_total_abs;
    resultados(r).E_total = E_total;
end

%% ============================================================
%  TABELA RESUMO PARA SELEÇÃO PRELIMINAR DE MOTORES
%% ============================================================

ResumoMotores = cell2table(linhas_tabela, ...
    'VariableNames', {'Tf_s', 'Junta', ...
    'TorquePico_Junta_Nm', 'TorqueRMS_Junta_Nm', ...
    'VelPico_Junta_rad_s', 'AcelPico_Junta_rad_s2', ...
    'PotPico_Junta_W', 'Energia_Junta_J', ...
    'TorquePico_Motor_comFS_Nm', 'TorqueRMS_Motor_comFS_Nm', ...
    'VelPico_Motor_rad_s', 'PotPico_Motor_comFS_W'});

fprintf('\n============================================================\n');
fprintf('RESUMO PARA SELEÇÃO PRELIMINAR DE MOTORES\n');
fprintf('============================================================\n');
disp(ResumoMotores);

fprintf('\nObservações de dimensionamento:\n');
fprintf('- Torque de pico: verifica esforço instantâneo máximo.\n');
fprintf('- Torque RMS: mais relacionado a regime contínuo e aquecimento.\n');
fprintf('- Velocidade máxima: verifica se motor/redutor atinge a rotação necessária.\n');
fprintf('- Potência de pico: verifica demanda mecânica instantânea.\n');
fprintf('- Energia: indica esforço mecânico acumulado da trajetória.\n');
fprintf('- Valores no eixo do motor dependem de Nred e eta definidos no início.\n');

%% ============================================================
%  GRÁFICOS: POSIÇÃO, VELOCIDADE E ACELERAÇÃO
%% ============================================================

cores = lines(length(Tf_list));

figure('Color','w');
for jj = 1:length(free_idx)
    subplot(2,1,jj); hold on; grid on;
    junta = free_idx(jj);

    for r = 1:length(Tf_list)
        plot(resultados(r).t, resultados(r).Q(junta,:), ...
             'LineWidth', 2, 'Color', cores(r,:));
    end

    xlabel('Tempo [s]');
    ylabel(sprintf('q_%d [rad]', junta));
    title(sprintf('Posição angular - Junta %d', junta));
    legend(compose('Tf = %g s', Tf_list), 'Location', 'best');
end

figure('Color','w');
for jj = 1:length(free_idx)
    subplot(2,1,jj); hold on; grid on;
    junta = free_idx(jj);

    for r = 1:length(Tf_list)
        plot(resultados(r).t, resultados(r).Qd(junta,:), ...
             'LineWidth', 2, 'Color', cores(r,:));
    end

    xlabel('Tempo [s]');
    ylabel(sprintf('dq_%d [rad/s]', junta));
    title(sprintf('Velocidade angular - Junta %d', junta));
    legend(compose('Tf = %g s', Tf_list), 'Location', 'best');
end

figure('Color','w');
for jj = 1:length(free_idx)
    subplot(2,1,jj); hold on; grid on;
    junta = free_idx(jj);

    for r = 1:length(Tf_list)
        plot(resultados(r).t, resultados(r).Qdd(junta,:), ...
             'LineWidth', 2, 'Color', cores(r,:));
    end

    xlabel('Tempo [s]');
    ylabel(sprintf('ddq_%d [rad/s²]', junta));
    title(sprintf('Aceleração angular - Junta %d', junta));
    legend(compose('Tf = %g s', Tf_list), 'Location', 'best');
end

%% ============================================================
%  GRÁFICOS: TORQUE, POTÊNCIA E ENERGIA
%% ============================================================

figure('Color','w');
for jj = 1:length(free_idx)
    subplot(2,1,jj); hold on; grid on;
    junta = free_idx(jj);

    for r = 1:length(Tf_list)
        plot(resultados(r).t, resultados(r).Tau_total(junta,:), ...
             'LineWidth', 2, 'Color', cores(r,:));
    end

    xlabel('Tempo [s]');
    ylabel(sprintf('\\tau_%d [N.m]', junta));
    title(sprintf('Torque estimado - Junta %d', junta));
    legend(compose('Tf = %g s', Tf_list), 'Location', 'best');
end

figure('Color','w');
for jj = 1:length(free_idx)
    subplot(2,1,jj); hold on; grid on;
    junta = free_idx(jj);

    for r = 1:length(Tf_list)
        plot(resultados(r).t, resultados(r).P_joint(junta,:), ...
             'LineWidth', 2, 'Color', cores(r,:));
    end

    xlabel('Tempo [s]');
    ylabel(sprintf('P_%d [W]', junta));
    title(sprintf('Potência mecânica - Junta %d', junta));
    legend(compose('Tf = %g s', Tf_list), 'Location', 'best');
end

figure('Color','w');

subplot(2,1,1); hold on; grid on;
for r = 1:length(Tf_list)
    plot(resultados(r).t, resultados(r).P_total_abs, ...
         'LineWidth', 2, 'Color', cores(r,:));
end
xlabel('Tempo [s]');
ylabel('Potência total demandada [W]');
title('Potência total demandada nas juntas ativas');
legend(compose('Tf = %g s', Tf_list), 'Location', 'best');

subplot(2,1,2); hold on; grid on;
for r = 1:length(Tf_list)
    plot(resultados(r).t, resultados(r).E_total, ...
         'LineWidth', 2, 'Color', cores(r,:));
end
xlabel('Tempo [s]');
ylabel('Energia acumulada [J]');
title('Energia mecânica acumulada nas juntas ativas');
legend(compose('Tf = %g s', Tf_list), 'Location', 'best');

%% ============================================================
%  GRÁFICO 3D DO MOVIMENTO PARA UM TEMPO ESCOLHIDO
%% ============================================================

idx_plot = min(2, length(Tf_list));   % normalmente Tf = 5 s
Q_plot = resultados(idx_plot).Q;
p_plot = resultados(idx_plot).p_tcp;
Tf_plot = resultados(idx_plot).Tf;

q_final_plot = Q_plot(:,end);
[T0_final, ~] = fkine_ur5e(q_final_plot, params);

figure('Color','w');
hold on; grid on; axis equal;
xlabel('X_0 [m]');
ylabel('Y_0 [m]');
zlabel('Z_0 [m]');
title(sprintf('Trajetória 3D do TCP e referenciais finais - Tf = %.1f s', Tf_plot));
view(135,25);

% Trajetória cartesiana do TCP
plot3(p_plot(1,:), p_plot(2,:), p_plot(3,:), ...
      'c-', 'LineWidth', 2);

% Ponto desejado e ponto final atingido
plot3(p_des(1), p_des(2), p_des(3), ...
      'rp', 'MarkerSize', 14, 'MarkerFaceColor', 'r');

plot3(p_final_ik(1), p_final_ik(2), p_final_ik(3), ...
      'bo', 'MarkerSize', 9, 'MarkerFaceColor', 'b');

% Origens dos referenciais finais
origens = zeros(3,7);
for i = 0:6
    origens(:,i+1) = T0_final{i+1}(1:3,4);
end

plot3(origens(1,:), origens(2,:), origens(3,:), ...
      'k-o', 'LineWidth', 2, 'MarkerSize', 5);

% Referenciais finais
escala_frame = 0.10;
for i = 0:6
    Ti = T0_final{i+1};
    draw_frame(Ti(1:3,4), Ti(1:3,1:3), escala_frame, sprintf('{%d}', i));
end

% Eixos das juntas
escala_eixo_junta = 0.14;
for i = 1:6
    origem_junta = T0_final{i}(1:3,4);
    eixo_junta = T0_final{i}(1:3,3);

    if ismember(i, free_idx)
        largura = 3.0;
        label = sprintf(' J%d livre', i);
    else
        largura = 1.4;
        label = sprintf(' J%d fixa', i);
    end

    quiver3(origem_junta(1), origem_junta(2), origem_junta(3), ...
            escala_eixo_junta*eixo_junta(1), ...
            escala_eixo_junta*eixo_junta(2), ...
            escala_eixo_junta*eixo_junta(3), ...
            'm', 'LineWidth', largura, 'MaxHeadSize', 0.8);

    text(origem_junta(1), origem_junta(2), origem_junta(3), ...
         label, 'FontSize', 10, 'FontWeight', 'bold');
end

legend({'Trajetória do TCP', 'Ponto desejado', 'TCP final', 'Elos'}, ...
       'Location', 'bestoutside');

lim = 1.0;
xlim([-lim lim]);
ylim([-lim lim]);
zlim([0 lim]);

%% ============================================================
%  FUNÇÕES LOCAIS
%% ============================================================

function [T0, T_ee] = fkine_ur5e(q, params)

    n = params.n;

    T0 = cell(n+1,1);
    T0{1} = eye(4);

    for i = 1:n
        A_i = dh_standard_num(params.a(i), params.alpha(i), params.d(i), q(i));
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

function tau = invdyn_numeric(q, dq, ddq, params, gval, usar_coriolis)

    M = mass_matrix_numeric(q, params);
    G = gravity_torque_required_numeric(q, params, gval);

    if usar_coriolis
        h = coriolis_vector_numeric(q, dq, params);
    else
        h = zeros(6,1);
    end

    tau = M*ddq + h + G;

end

function M = mass_matrix_numeric(q, params)

    n = params.n;

    [T0, ~] = fkine_ur5e(q, params);

    M = zeros(n,n);

    for i = 1:n

        Ti = T0{i+1};
        Ri = Ti(1:3,1:3);

        pc_local = params.pc(:,i);
        pc_h = Ti*[pc_local; 1];
        pc0 = pc_h(1:3);

        Jv = zeros(3,n);
        Jw = zeros(3,n);

        for j = 1:i
            z_jmenos1 = T0{j}(1:3,3);
            o_jmenos1 = T0{j}(1:3,4);

            Jv(:,j) = cross(z_jmenos1, pc0 - o_jmenos1);
            Jw(:,j) = z_jmenos1;
        end

        mi = params.m(i);
        Ii = params.Icm(:,:,i);

        M = M + mi*(Jv.'*Jv) + Jw.'*Ri*Ii*Ri.'*Jw;
    end

end

function G = gravity_torque_required_numeric(q, params, gval)

    n = params.n;

    [T0, ~] = fkine_ur5e(q, params);

    G = zeros(n,1);

    % Força que o atuador deve equilibrar para sustentar cada elo.
    % z positivo para cima.
    for i = 1:n

        Ti = T0{i+1};
        pc_local = params.pc(:,i);
        pc_h = Ti*[pc_local; 1];
        pc0 = pc_h(1:3);

        Jv = zeros(3,n);

        for j = 1:i
            z_jmenos1 = T0{j}(1:3,3);
            o_jmenos1 = T0{j}(1:3,4);

            Jv(:,j) = cross(z_jmenos1, pc0 - o_jmenos1);
        end

        F_sustentacao = [0; 0; params.m(i)*gval];

        G = G + Jv.'*F_sustentacao;
    end

end

function h = coriolis_vector_numeric(q, dq, params)

    n = params.n;
    hstep = 1e-6;

    dM = zeros(n,n,n);

    for r = 1:n
        qp = q;
        qm = q;

        qp(r) = qp(r) + hstep;
        qm(r) = qm(r) - hstep;

        Mp = mass_matrix_numeric(qp, params);
        Mm = mass_matrix_numeric(qm, params);

        dM(:,:,r) = (Mp - Mm)/(2*hstep);
    end

    h = zeros(n,1);

    for k = 1:n
        soma = 0;

        for s = 1:n
            for r = 1:n
                gamma = 0.5*(dM(k,s,r) + dM(k,r,s) - dM(r,s,k));
                soma = soma + gamma*dq(s)*dq(r);
            end
        end

        h(k) = soma;
    end

end

function draw_frame(origem, R, escala, nome)

    x_axis = R(:,1);
    y_axis = R(:,2);
    z_axis = R(:,3);

    quiver3(origem(1), origem(2), origem(3), ...
            escala*x_axis(1), escala*x_axis(2), escala*x_axis(3), ...
            'r', 'LineWidth', 1.8, 'MaxHeadSize', 0.6);

    quiver3(origem(1), origem(2), origem(3), ...
            escala*y_axis(1), escala*y_axis(2), escala*y_axis(3), ...
            'g', 'LineWidth', 1.8, 'MaxHeadSize', 0.6);

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