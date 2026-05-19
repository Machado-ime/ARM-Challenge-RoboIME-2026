function [idx,status] = EvalFun(pack,centers3D,objects,bboxes,labels,pack2)
% Projeta centros 3D na imagem usando matriz intrínseca e marca pontos
% Função de avaliação
    
    shrinkFactor = pack2{1};
    threshold = pack2{2};
    zMesa = pack2{3};

    Slabels = string(labels);
    idxSelecionados = ismember(Slabels, {'Can', 'Bottle','Marker'});
    status = categorical(repmat("-", numel(objects), 1));  % default

    for k = 1:numel(objects)
        if ~idxSelecionados(k)
            continue;
        end

        % Filtra ruído do objeto
        objFiltrado = noiseFilter(objects{k}, 10, 0.01);

        % Ignora objetos muito pequenos após o filtro
        if objFiltrado.Count < 50
            status(k) = "Ignored";
            continue;
        end

        % Detecta plano superior do objeto
        [zTopo, ~] = supPlane(objFiltrado, 0.05, 5000);

        % Se falhou em detectar plano (retornou NaN), pula
        if isnan(zTopo)
            status(k) = "Indefinido";
            continue;
        end

        % Classificação baseada na altura
        if (zMesa - zTopo) > threshold
            status(k) = "Up";
        else
            status(k) = "Down";
        end
    end

    colorImage = pack{2};
    %imageSize = [480 640];

    focalLength = [604.3037 602.5643]; %Como foi calibrado
    principalPoint = [321.5102 235.7259]; %Como foi calibrado

    % Extrai parâmetros
    fx = focalLength(1);
    fy = focalLength(2);
    cx = principalPoint(1);
    cy = principalPoint(2);

    % Projeta cada ponto 3D para pixel (u,v)
    X = centers3D(:,1);
    Y = centers3D(:,2);
    Z = centers3D(:,3);

    % Evita divisão por zero
    valid = Z > 0;
    u = zeros(size(Z));
    v = zeros(size(Z));
    u(valid) = fx .* X(valid) ./ Z(valid) + cx;
    v(valid) = fy .* Y(valid) ./ Z(valid) + cy;

    % Mostra imagem
    figure;
    imshow(colorImage);
    hold on;
    
    idx = {};
    j = 1;
    % shrinkFactor Vai de 0 a 1 (0 tamanho normal, 1 degenera)
    M = ones(numel(labels));
    M = [M(:,1) M(:,1) M(:,1) M(:,1)];
    M(:,1) = bboxes(:,3)*shrinkFactor/2;
    M(:,2) = bboxes(:,4)*shrinkFactor/2;
    M(:,3) = -bboxes(:,3)*shrinkFactor;
    M(:,4) = -bboxes(:,4)*shrinkFactor;
    
    deltaPix = 10;
    bboxes1 = bboxes + M;
    for k = 1:length(Z)
        s = 0;
        if (u(k) < bboxes1(k,1)) || (u(k) > bboxes1(k,1)+bboxes1(k,3)) || ...
            (v(k) < bboxes1(k,2)) || (v(k) > bboxes1(k,2)+bboxes1(k,4))
            idx{j} = k;
            j = j+1;
            s = 1;
        end
        % if (bboxes(k,1) < 10) || (bboxes(k,1) + bboxes(k,3) > 640 - deltaPix) || ...
        %    (bboxes(k,2) < 10) || (bboxes(k,2) + bboxes(k,4) > 480 - deltaPix) & (s == 0)
        %     idx{j} = k;
        %     j = j+1;
        % end
    end

    % Plota só pontos válidos
    for k = 1:length(Z)
        if ~valid(k)
            continue
        end
        plot(u(k), v(k), 'ro', 'MarkerSize', 10, 'LineWidth', 2);
        txt = sprintf("%d. %s - %s",k, string(labels(k)), status(k));
        txt = strrep(txt, "_", " ");
        text(u(k)+5, v(k), txt, 'Color','yellow', 'FontSize',10, 'FontWeight','bold');
    end

    title("Center estimative");
    hold off;

end