%GERAR_WORKSPACE_TRAJETORIA  Gera a matriz de ângulos quíntica para o Simulink.
%
%  Execute este script NO MATLAB antes de iniciar a simulação.
%  Ele popula o workspace com as variáveis necessárias para os blocos
%  "From Workspace" e "Constant" do seu modelo Simulink.
%
%  ── ENTRADAS (edite aqui) ────────────────────────────────────────────────

% Configuração inicial das 6 juntas [rad]
q0 = [0;
     -pi/2;
      0;
     -pi/2;
      0;
      0];

% Configuração final das 6 juntas [rad]
% Substitua pelos valores desejados ou resultado da cinemática inversa.
qf = [0;
     -pi/4;
     -pi/3;
     -pi/2;
      0;
      0];

% Duração total da trajetória [s]
Tf = 5.0;

% Número de pontos amostrados (define a resolução temporal)
% Para simulações em tempo real, use Npts = Tf / passo_fixo_do_solver + 1
% Ex.: Tf = 5 s, passo = 0.01 s  →  Npts = 501
Npts = 501;

%  ── GERAÇÃO DA TRAJETÓRIA ────────────────────────────────────────────────

[t_vec, Q, Qd, Qdd] = quintica_matriz(q0, qf, Tf, Npts);

%  ── MONTAGEM DO OBJETO PARA "From Workspace" ─────────────────────────────
%
%  O bloco "From Workspace" do Simulink aceita uma struct com os campos:
%    .time             — vetor coluna de tempos  [N×1]
%    .signals.values   — matriz de dados         [N×n_juntas]
%    .signals.dimensions — número de colunas     (= 6)
%
%  A interpolação entre amostras é feita automaticamente pelo bloco.

% Posição das juntas  (6 sinais)
simin_q.time               = t_vec(:);          % [N×1]
simin_q.signals.values     = Q.';               % [N×6]
simin_q.signals.dimensions = 6;

% Velocidade das juntas  (6 sinais)
simin_qd.time               = t_vec(:);
simin_qd.signals.values     = Qd.';
simin_qd.signals.dimensions = 6;

% Aceleração das juntas  (6 sinais)
simin_qdd.time               = t_vec(:);
simin_qdd.signals.values     = Qdd.';
simin_qdd.signals.dimensions = 6;

%  ── COMO CONFIGURAR O BLOCO "From Workspace" ────────────────────────────
%
%  Para cada bloco:
%    • "Data"        : simin_q   (ou simin_qd, simin_qdd)
%    • "Sample time" : -1  (herda do modelo) ou o passo da simulação
%    • "Interpolate" : marcar "on"
%    • "Form output after final data value": "Holding final value"

%  ── SAÍDA NO WORKSPACE ───────────────────────────────────────────────────

fprintf('\n==============================================================\n');
fprintf('  TRAJETÓRIA QUÍNTICA GERADA\n');
fprintf('==============================================================\n');
fprintf('  q0 [rad]:  %s\n', mat2str(q0.',4));
fprintf('  qf [rad]:  %s\n', mat2str(qf.',4));
fprintf('  Tf [s]:    %.3f\n', Tf);
fprintf('  Npts:      %d\n', Npts);
fprintf('  Passo dt:  %.5f s\n', Tf/(Npts-1));
fprintf('\n  Variáveis geradas no workspace:\n');
fprintf('    t_vec    — vetor de tempo      [%d×1]\n', length(t_vec));
fprintf('    Q        — posição             [6×%d]\n', Npts);
fprintf('    Qd       — velocidade          [6×%d]\n', Npts);
fprintf('    Qdd      — aceleração          [6×%d]\n', Npts);
fprintf('    simin_q  — struct p/ From Workspace (posição)\n');
fprintf('    simin_qd — struct p/ From Workspace (velocidade)\n');
fprintf('    simin_qdd— struct p/ From Workspace (aceleração)\n');
fprintf('    q0, qf, Tf — parâmetros para blocos Constant\n');
fprintf('==============================================================\n\n');

% Mostra o perfil quíntico
figure('Color','w','Name','Trajetória quíntica gerada');
junta_nomes = {'q_1','q_2','q_3','q_4','q_5','q_6'};
cores = lines(6);

subplot(3,1,1); hold on; grid on;
for j = 1:6
    plot(t_vec, Q(j,:), 'LineWidth', 1.8, 'Color', cores(j,:));
end
xlabel('Tempo [s]');
ylabel('Posição [rad]');
title('Posição angular das juntas');
legend(junta_nomes, 'Location','best','NumColumns',3);

subplot(3,1,2); hold on; grid on;
for j = 1:6
    plot(t_vec, Qd(j,:), 'LineWidth', 1.8, 'Color', cores(j,:));
end
xlabel('Tempo [s]');
ylabel('Velocidade [rad/s]');
title('Velocidade angular das juntas');
legend(junta_nomes, 'Location','best','NumColumns',3);

subplot(3,1,3); hold on; grid on;
for j = 1:6
    plot(t_vec, Qdd(j,:), 'LineWidth', 1.8, 'Color', cores(j,:));
end
xlabel('Tempo [s]');
ylabel('Aceleração [rad/s²]');
title('Aceleração angular das juntas');
legend(junta_nomes, 'Location','best','NumColumns',3);

%  ══════════════════════════════════════════════════════════════════════════
%  FUNÇÃO LOCAL — núcleo do cálculo quíntico
%  ══════════════════════════════════════════════════════════════════════════

function [t_vec, Q, Qd, Qdd] = quintica_matriz(q0, qf, Tf, Npts)
%QUINTICA_MATRIZ  Gera as matrizes de posição, velocidade e aceleração
%                 para um perfil quíntico com repouso nas duas extremidades.
%
%  Entradas:
%    q0   — configuração inicial  [6×1] rad
%    qf   — configuração final    [6×1] rad
%    Tf   — duração da trajetória [s]
%    Npts — número de pontos amostrados
%
%  Saídas:
%    t_vec — vetor de tempo [1×Npts]
%    Q     — posição        [6×Npts] rad
%    Qd    — velocidade     [6×Npts] rad/s
%    Qdd   — aceleração     [6×Npts] rad/s²

    % Vetor de tempo e parâmetro normalizado
    t_vec = linspace(0, Tf, Npts);        % [1×Npts]
    tau   = t_vec / Tf;                   % τ ∈ [0,1]

    % Polinômios quínticos (escalares ao longo do tempo)
    s   =   10*tau.^3 -  15*tau.^4 +  6*tau.^5;   % posição normalizada
    ds  =   30*tau.^2 -  60*tau.^3 + 30*tau.^4;   % d(s)/d(tau)
    dds =   60*tau    - 180*tau.^2 + 120*tau.^3;  % d²(s)/d(tau²)

    % Diferença de configuração
    delta_q = qf(:) - q0(:);    % [6×1]

    % Expansão vetorial: cada linha é uma junta, cada coluna é um instante
    Q   = q0(:) + delta_q * s;             % [6×Npts]
    Qd  = delta_q * (ds  / Tf);            % [6×Npts]
    Qdd = delta_q * (dds / Tf^2);          % [6×Npts]

end
