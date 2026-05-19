function [objetosComPlano, objetosSemPlano, Labels] = objextract(ptCloud, pack, classesPermitidas)
    
colorImage = pack{2};
Detector = pack{1};

[bboxes, scores, labels] = detect(Detector, colorImage);

numObjetos = size(bboxes, 1);
disp(numObjetos)

%Threshold de overlap
[bboxes, scores, selectedIdx] = selectStrongestBbox(bboxes, scores,'OverlapThreshold', 0.5);
labels = labels(selectedIdx);

% Filtrar apenas classes desejadas
isPermitido = ismember(string(labels), classesPermitidas);

% Aplicar o filtro de classe
bboxes = bboxes(isPermitido, :);
scores = scores(isPermitido);
labels = labels(isPermitido);

% Lista de labels únicos
labelsUnicos = unique(labels);

% Inicializar vetor de índices válidos
indicesValidos = [];

for i = 1:numel(labelsUnicos)
    labelAtual = labelsUnicos(i);
    
    idx = find(labels == labelAtual);        % Índices com esse label
    scoresLabel = scores(idx);
    
    [~, idxMax] = max(scoresLabel);          % Índice local com maior score
    indicesValidos(end+1) = idx(idxMax);     % Índice global no vetor original
end

% Criar novos vetores diretamente a partir dos índices válidos
bboxes = bboxes(indicesValidos, :);
scores = scores(indicesValidos);
Labels = labels(indicesValidos);

numObjetos = size(bboxes, 1);
disp(numObjetos)

% Formatando saída final para compatibilidade
labelsWithScores = string(Labels) + ": " + string(scores);
imTested = insertObjectAnnotation(colorImage, "rectangle", bboxes, labelsWithScores);
imshow(imTested);

%% Recorte
    [rows, cols, ~] = size(ptCloud.Location);
    xyzVec = reshape(ptCloud.Location, [], 3);
    rgbVec = reshape(ptCloud.Color, [], 3);

    objetosComPlano = {};
    objetosSemPlano = {};

    for i = 1:size(bboxes, 1)
        bbox = round(bboxes(i, :));  % [x, y, w, h]
        x1 = max(1, min(bbox(1), cols));
        x2 = max(1, min(bbox(1) + bbox(3) - 1, cols));
        y1 = max(1, min(bbox(2), rows));
        y2 = max(1, min(bbox(2) + bbox(4) - 1, rows));

        mask = false(rows, cols);
        mask(y1:y2, x1:x2) = true;
        linearIdx = find(mask);

        xyz = xyzVec(linearIdx, :);
        rgb = rgbVec(linearIdx, :);

        % Remove NaNs
        validIdx = ~any(isnan(xyz), 2);
        pt = pointCloud(xyz(validIdx, :), 'Color', rgb(validIdx, :));

        % Armazena o ponto (sem remover plano)
        objetosComPlano{end+1} = pt;
        objetosSemPlano{end+1} = pt;

    end
end