function [displacement,idx,sts] = VMVision(colorImage,depthImage,Detector,realHeight)
    
    %RealThreshold: Medida de limite de altura entre objeto em pé e objeto
    %deitado
    %realHeight: Altura real entre câmera e objeto
    %idx: Índice dos objetos os quais podem ser descartados
    %sts: Diz se o objeto está em pé ou deitado
    
    colorImage = imresize(colorImage, [480 640]);
    depthImage = imresize(depthImage, [480 640]);

    pack = cell(1,3);  %Célula que vai transportar Detector, colorImage e depthImage.        
    pack{1} = Detector;    
    pack{2} = colorImage;              
    pack{3} = depthImage; 

    [ptCloud,objCom,objSem,objLab] = PcGenVM(pack);

    ptCloudF = noiseFilter(ptCloud,8,0.01);
    [zMesa, ~] = denseplane(ptCloudF, 0.06);
    corrFactor = realHeight/zMesa; %Fator de correção da ptCloud para a realidade
    
    
    centers3DNF = zeros(numel(objSem), 3);
    CcentersNF = {};

    for k = 1:numel(objSem)
        obj = objSem{k};
        [zPlano, pcPlano] = supPlane(obj, 0.05, 3000);
        CcentersNF{k} = mean(pcPlano.Location)*[1 0; 0 1; 0 0];
        centers3DNF(k, :) = mean(pcPlano.Location);
    end

    centersNF = reshape(cell2mat(CcentersNF),numel(cell2mat(CcentersNF))/2,2);
    displacement = corrFactor*centersNF;

    RealThreshold = 0.082; 
    PcThreshold = RealThreshold/corrFactor;
    pack2 = cell(1,3);  %Célula que vai transportar Detector, colorImage e depthImage.        
    pack2{1} = 0.5; %ShrinkFactor   
    pack2{2} = PcThreshold;              
    pack2{3} = zMesa; 
    
    [bboxes, scores, labels] = detect(Detector, colorImage);

    [bboxes, scores, selectedIdx] = selectStrongestBbox(bboxes, scores, ...
    'OverlapThreshold', 0.5);
    labels = labels(selectedIdx);
    
    scoreThresh = 0.6;
    idxValidos = scores >= scoreThresh;
    bboxes = bboxes(idxValidos, :);
    scores = scores(idxValidos);
    labels = labels(idxValidos);

    [idx,sts] = EvalFun(pack,centers3DNF,objSem,bboxes,objLab,pack2);

end
