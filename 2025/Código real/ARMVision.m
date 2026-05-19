function [displacement,idx,sts,labels,scores] = ARMVision(colorImage,depthImage,Detector,realHeight)

    depthImage = depthImage(:,:,3);

    colorImage = imresize(colorImage, [480 640]);
    depthImage = imresize(depthImage, [480 640]);

    pack = cell(1,3);  %Célula que vai transportar Detector, colorImage e depthImage.        
    pack{1} = Detector;    
    pack{2} = colorImage;              
    pack{3} = depthImage; 

    [ptCloud,~,objSem,objLab] = PcGenRSD435(pack);

    ptCloudF = noiseFilter(ptCloud,100,0.1);
    [zMesa, ~] = denseplane(ptCloudF,0.1);

    corrFactor = realHeight/zMesa;

    Ccenters = {};
    centers3D = zeros(numel(objSem), 3);

    for k = 1:numel(objSem)
    obj = noiseFilter(objSem{k},10,0.01); 
    [~, pcPlano] = supPlane(obj, 0.05, 3000);
    Ccenters{k} = mean(pcPlano.Location)*[1 0; 0 1; 0 0];
    centers3D(k, :) = mean(pcPlano.Location);
    end

    [bboxes, scores, labels] = detect(pack{1}, pack{2});

    [bboxes, scores, selectedIdx] = selectStrongestBbox(bboxes, scores, ...
        'OverlapThreshold', 0.5);
    labels = labels(selectedIdx);
    
    idxValidos = scores >= 0.6;
    bboxes = bboxes(idxValidos, :);
    scores = scores(idxValidos);
    labels = labels(idxValidos);

    centers = reshape(cell2mat(Ccenters),numel(cell2mat(Ccenters))/2,2);
    displacement = corrFactor*centers;

    RealThreshold = 0.082;
    PcThreshold = RealThreshold/corrFactor;
    pack2 = cell(1,3);  %Célula que vai transportar Detector, colorImage e depthImage.        
    pack2{1} = 0.5; %ShrinkFactor   
    pack2{2} = PcThreshold;              
    pack2{3} = zMesa; 
    
    %Roda a função de avaliação
    [idx,sts] = EvalFun(pack,centers3D,objSem,bboxes,objLab,pack2);

end