function [ptCloud,objPlane,objSem,objLab,idxDegenerados] = PcGenRSD435(pack)
    
    scoreThresh = 0.6; %Threshold de detecção
    
    Detector = pack{1};
    colorImage = pack{2};
    depthImage = pack{3};
    
    %ptCloud = pcfromdepth(imcut,depthScaleFactor, intrinsics, ...
    %ColorImage=colorImage, DepthRange=[0 maxCameraDepth]);
    %depthScaleFactor = 100; Reescala a nuvem de pontos
    %maxCameraDepth   = 100;   
    imageSize = [480 640]; %Para a IntelRealSense D435
    focalLength = [604.3037 602.5643]; %Como foi calibrado
    principalPoint = [321.5102 235.7259]; %Como foi calibrado

    intrinsics = cameraIntrinsics(focalLength,principalPoint,imageSize);

    %Geração da PtCloud
    img = depthImage + 0;
    [imgH, imgW, ~] = size(img);
    imgM = imresize(img,1.596);
    roi = [155,143, imgW, imgH];
    imcut = imcrop(imgM, roi);

    imcut = im2gray(imcut);
    [rows, cols, ~] = size(colorImage);
    imcut = imresize(imcut, [rows cols]);

    ptCloud = pcfromdepth(imcut,90,intrinsics, ...
    ColorImage=colorImage, DepthRange=[0 100]);

    %Faz filtragem dos pontos NaN da nuvem
    loc = ptCloud.Location;
    col = ptCloud.Color;    
    maskValida = ~any(isnan(loc), 3);
    locFiltered = reshape(loc, [], 3);
    colFiltered = reshape(col, [], 3);
    locFiltered = locFiltered(maskValida(:), :);
    colFiltered = colFiltered(maskValida(:), :);
    ptCloud = pointCloud(locFiltered, 'Color', colFiltered);

    %Calcula onde é o centro da nuvem e força métrica de 1.0
    P = (ptCloud.Location);
    P = reshape(P, [], 3);
    mediaXYZ = mean(P, 1);
    depthScaleFactor = mediaXYZ(3);
    ptCloud = pcfromdepth(imcut,90*depthScaleFactor, intrinsics, ...
    ColorImage=colorImage, DepthRange=[0 100]);

    % pcshow(ptCloud);
    % title("Nuvem da cena");

    %Detecção e extração de objetos
    [bboxes, scores, labels] = detect(Detector, colorImage);

    [bboxes, scores, selectedIdx] = selectStrongestBbox(bboxes, scores, ...
    'OverlapThreshold', 0.5);
    labels = labels(selectedIdx);

    idxValidos = scores >= scoreThresh;
    bboxes = bboxes(idxValidos, :);
    scores = scores(idxValidos);
    labels = labels(idxValidos);

    [rows, cols, ~] = size(ptCloud.Location);
    xyzVec = reshape(ptCloud.Location, [], 3);
    rgbVec = reshape(ptCloud.Color, [], 3);

    objPlane = {};
    objSem = {};
    objLab = {};

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
        objPlane{end+1} = pt;
        objSem{end+1} = pt;

        % Armazena o rótulo correspondente
        objLab{end+1} = labels(i);
    end

    % Parâmetros para descartar objetos degenerados
    minPontos = 100;          % mínimo de pontos aceitável
    minStd = 0.005;           % mínimo de dispersão em cada eixo
    minSpread = 0.01;         % mínima extensão espacial total
    
    % Novo conjunto de objetos válidos
    objPlaneFiltr = {};
    objSemFiltr = {};
    objLabFiltr = {};
    
    idxDegenerados = [];  % Índices dos objetos degenerados
    
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
    
        % Dados válidos para análise
        xyzVal = xyz(validIdx, :);
    
        % Critérios para degeneração
        if size(xyzVal,1) < minPontos || ...
           any(std(xyzVal,0,1) < minStd) || ...
           any((max(xyzVal) - min(xyzVal)) < minSpread)
            idxDegenerados(end+1) = i;  % marca índice como degenerado
            continue;  % pula esse objeto
        end
    
        % Adiciona objeto válido
        objPlaneFiltr{end+1} = pt;
        objSemFiltr{end+1} = pt;
        objLabFiltr{end+1} = labels(i);
    end
    
    % Opcional: atualizar os objetos filtrados (ou não)
    % objPlane = objPlaneFiltr;
    % objSem = objSemFiltr;
    % objLab = objLabFiltr;
    
    % Retorna também os índices degenerados
    % Modifique a assinatura da função para:
    % function [ptCloud,objPlane,objSem,objLab, idxDegenerados] = PcGenRSD435(pack)

    % for k = 1:numel(objSem)
    %     figure;
    %     hold on;
    %     pcshow(objSem{k});
    %     txt = sprintf("Objeto %d - %s", k, string(objLab(k)));
    %     txt = strrep(txt, "_", " ");
    %     title(txt);
    %     hold off;
    % end

end