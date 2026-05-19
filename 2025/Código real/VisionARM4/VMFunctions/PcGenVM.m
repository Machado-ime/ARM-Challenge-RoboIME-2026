function [ptCloud,objPlane,objSem,objLab] = PcGenVM(pack)
    
    scoreThresh = 0.6; %Threshold de detecção
    Detector = pack{1};
    colorImage = pack{2};
    depthImage = pack{3};
    %ptCloud = pcfromdepth(imcut,depthScaleFactor, intrinsics, ...
    %ColorImage=colorImage, DepthRange=[0 maxCameraDepth]);
    % depthScaleFactor = 100; Reescala a nuvem de pontos
    % maxCameraDepth   = 10;   
    imageSize = [480 640]; %Para a IntelRealSense D435
    focalLength = [604.3037 602.5643]; %Como foi calibrado
    principalPoint = [321.5102 235.7259]; %Como foi calibrado

    intrinsics = cameraIntrinsics(focalLength,principalPoint,imageSize);

    %Geração da PtCloud

    ptCloud = pcfromdepth(depthImage,100,intrinsics, ...
    ColorImage=colorImage, DepthRange=[0 10]);
    
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
end