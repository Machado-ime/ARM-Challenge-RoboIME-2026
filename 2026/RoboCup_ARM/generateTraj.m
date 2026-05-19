function currentConfig = generateTraj(pos_final, q_atual, T_mov, peso_pos, peso_ori)

    persistent alpha;
    persistent primeiro_tick;

    T_sim = 0.01;
    N     = max(round(T_mov / T_sim), 2);

    % GARANTE FORMATOS CORRETOS
    pos_final = pos_final(:);
    q_atual   = q_atual(:);

    if isempty(alpha),         alpha = 0;         end
    if isempty(primeiro_tick)
        primeiro_tick = false;
        currentConfig = q_atual;
        return;
    end

    T_atual   = cinematica_direta_ur5e(q_atual);
    pos_atual = T_atual(1:3, 4);

    if norm(pos_final - pos_atual) < 1e-3
        alpha         = 0;
        currentConfig = q_atual;
        return;
    end

    pos_alvo = pos_atual + alpha * (pos_final - pos_atual);

    [currentConfig, ~] = cinematica_inversa_generica(pos_alvo, q_atual, peso_pos, peso_ori);

    % GARANTE SAÍDA 6x1
    currentConfig = currentConfig(:);

    if alpha < 1
        alpha = min(alpha + 1/(N - 1), 1);
    end

end