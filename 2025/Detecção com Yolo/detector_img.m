function [tipo_Objeto, pos_objeto] = detector_img(pos_garra, pcSub)
%%
% === Captura da nuvem de pontos do sensor ===
pcMsg = receive(pcSub, 3);

% Extrai XYZ
xyz = rosReadXYZ(pcMsg, "PreserveStructureOnRead", true);
xyzVec = reshape(xyz, [], 3);

% Extrai RGB (opcional)
try
    rgb = rosReadRGB(pcMsg, "PreserveStructureOnRead", true);
    rgbVec = reshape(rgb, [], 3);
    hasColor = true;
catch
    hasColor = false;
end

% Remove NaNs
valid = ~any(isnan(xyzVec), 2);
xyzClean = xyzVec(valid, :);
xyzClean(:,3) = -xyzClean(:,3);  % Inverte o eixo Z para alinhar com câmera

% Cria ptCloud da cena
if hasColor
    rgbClean = uint8(255 * rgbVec(valid, :));
    ptCena = pointCloud(xyzClean, "Color", rgbClean);
else
    ptCena = pointCloud(xyzClean);
end

% Mostrar cena
figure;
pcshow(ptCena, VerticalAxis="Y", VerticalAxisDir="Up");
title("Nuvem de Pontos da Cena");
xlabel("X (m)"); ylabel("Y (m)"); zlabel("Z (m)");

%% === Visualização no plano XY com projeção ortográfica ===

figure;
ax = pcshow(ptCena);
view(2);                                      % vista de cima (plano XY)
axis equal;                                   % escalas iguais em X e Y
set(gca, 'Projection', 'orthographic');      % remove perspectiva :contentReference[oaicite:1]{index=1}
xlabel('X (m)'); ylabel('Y (m)');
title('Projeção 2D da Nuvem de Pontos (XY)');

% Captura da imagem da projeção
F = getframe(gca);
I = frame2im(F);

% Mostra a imagem capturada
figure; imshow(I);
title('Imagem extraída da projeção XY da nuvem de pontos');

%% === Carregador do detector === 
addpath('C:\Users\roboime\Downloads\');
load('C:\Users\roboime\Downloads\Color_Detector.mat','colorDetector'); 

imTest = I;
[dbox, dscore, dlabel] = detect(colorDetector, imTest);

% === Filtrar detecções com score >= 0.6 ===
minScore = 0.6;
validIdx = dscore >= minScore;
dbox = dbox(validIdx, :);
dscore = dscore(validIdx);
dlabel = dlabel(validIdx);

% === Verificar se ainda há detecções ===
if isempty(dbox)
    disp("Nenhum objeto detectado com score acima de 60%.");
    return;
end

% === Mostrar detecções válidas ===
labelsWithScores = string(dlabel) + ": " + string(dscore);
imTested = insertObjectAnnotation(imTest, "rectangle", dbox, labelsWithScores);
imshow(imTested);
pause(2);

% === Objeto com maior score ===
[~, idxMax] = max(dscore);
tipo_Objeto = string(dlabel(idxMax));
fprintf("Objeto com maior score: %s (%.2f%% de certeza)\n", tipo_Objeto, dscore(idxMax)*100);



%% === Extração da posição real ===

% === 2. Obter limites reais da nuvem no plano XY ===
xLimits = ptCena.XLimits;  % [xmin xmax]
yLimits = ptCena.YLimits;  % [ymin ymax]

% === 3. Obter dimensões da imagem renderizada ===
[imgHeight, imgWidth, ~] = size(I);

% === 4. Calcular escala (metros por pixel) ===
scaleX = diff(xLimits) / imgWidth;
scaleY = diff(yLimits) / imgHeight;

% === 5. Converter bbox da imagem (em pixels) para coordenadas reais ===
xPixel = bboxMax(1);  % coluna (esquerda)
yPixel = bboxMax(2);  % linha (topo)
wPixel = bboxMax(3);
hPixel = bboxMax(4);

% Canto superior esquerdo em metros
xReal = xLimits(1) + xPixel * scaleX;
yReal = yLimits(2) - yPixel * scaleY;  % eixo Y invertido na imagem

% Centro do bounding box (em metros)
centerX = xReal + (wPixel * scaleX) / 2;
centerY = yReal - (hPixel * scaleY) / 2;

% === 6. Exibir resultados ===
fprintf("Objeto com maior confiança:\n");
fprintf("Classe: %s\n", string(dlabel(idxMax)));
fprintf("Score: %.2f%%\n", 100 * dscore(idxMax));
fprintf("Centro (X,Y) em metros: (%.3f m, %.3f m)\n", centerX, centerY);

% (Opcional) plotar na imagem
hold on;
plot(centerX, centerY, 'r+', 'MarkerSize', 12, 'LineWidth', 2);

% === soma vetorial ===
pos_objeto(1) = pos_garra(1) + centerX;
pos_objeto(2) = pos_garra(2) + centerY;
disp(pos_objeto);
end