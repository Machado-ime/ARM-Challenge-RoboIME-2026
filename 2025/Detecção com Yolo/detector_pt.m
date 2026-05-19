function [tipo_Objeto, pos_objeto] = detector_pt(pos_garra, pcSub)
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
pcshow(ptCena);
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

% Índice do maior score
bboxMax = round(dbox(idxMax, :));  % [x, y, w, h] em pixels, arredondado 

%% === selecionar objeto no 2D === 
% === 3. Máscara de pontos dentro do retângulo detectado ===
xLeft   = bboxMax(1);                 
yTop    = bboxMax(2);                 
xRight  = xLeft + bboxMax(3);         
yBottom = yTop  + bboxMax(4);   

% Redesenha imagem 2D de projeção com pontos mapeados
pcXY = ptCena.Location(:,1:2);  % [X Y] dos pontos (em metros)
pcColor = ptCena.Color;

% Limites da nuvem e tamanho da imagem
xLimits = ptCena.XLimits;
yLimits = ptCena.YLimits;
[imgHeight, imgWidth, ~] = size(I);

% Escalas (m/pixel)
scaleX = diff(xLimits) / imgWidth;
scaleY = diff(yLimits) / imgHeight;

% Converter coordenadas reais → pixel
xPix = round((pcXY(:,1) - xLimits(1)) / scaleX);
yPix = round((yLimits(2) - pcXY(:,2)) / scaleY);  % Y invertido

% Eliminar pontos fora da imagem
inFrame = xPix >= 1 & xPix <= imgWidth & yPix >= 1 & yPix <= imgHeight;
xPix = xPix(inFrame);
yPix = yPix(inFrame);
xyzSub = ptCena.Location(inFrame, :);
if ~isempty(pcColor)
    rgbSub = ptCena.Color(inFrame, :);
end

% === Visualizar imagem com bbox antes do corte ===
IcomBBox = insertShape(I, 'Rectangle', bboxMax, ...
    'Color', 'red', 'LineWidth', 2);
imshow(IcomBBox);
%% === selecionar objeto no 3D ===
% === 1. P0 e P1 em pixels (imagem)
P0 = [xLeft, yTop];
P1 = [xRight, yBottom];

% === 2. Converter para coordenadas reais da nuvem (XY)
xLimits = ptCena.XLimits;
yLimits = ptCena.YLimits;
[imgHeight, imgWidth, ~] = size(I);

scaleX = diff(xLimits) / imgWidth;
scaleY = diff(yLimits) / imgHeight;

% Converter pixel → métrico
P0_XY = [P0(1) * scaleX + xLimits(1), yLimits(2) - P0(2) * scaleY];
P1_XY = [P1(1) * scaleX + xLimits(1), yLimits(2) - P1(2) * scaleY];

% === 3. Definir os 4 cantos do retângulo 2D (XY)
cornersXY = [ ...
    P0_XY;
    P1_XY(1) P0_XY(2);
    P1_XY;
    P0_XY(1) P1_XY(2)
];  % sentido anti-horário

% === 4. Estimar limites Z
zMin = min(ptCena.Location(:,3));
zMax = max(ptCena.Location(:,3));

% === 5. Construir os 8 vértices 3D
verts = [];
for i = 1:4
    verts = [verts; cornersXY(i,:), zMin];  % base
end
for i = 1:4
    verts = [verts; cornersXY(i,:), zMax];  % topo
end

% === 6. Conectar as arestas do cubo
edges = [
    1 2; 2 3; 3 4; 4 1;     % base
    5 6; 6 7; 7 8; 8 5;     % topo
    1 5; 2 6; 3 7; 4 8      % colunas
];

% === 7. Plotar a nuvem com o retângulo 3D
figure;
pcshow(ptCena, VerticalAxis="Y", VerticalAxisDir="Up");
hold on;
for e = 1:size(edges,1)
    p1 = verts(edges(e,1),:);
    p2 = verts(edges(e,2),:);
    plot3([p1(1) p2(1)], [p1(2) p2(2)], [p1(3) p2(3)], 'r-', 'LineWidth', 2);
end
title("Nuvem de pontos com retângulo manual");
xlabel("X (m)"); ylabel("Y (m)"); zlabel("Z (m)");
hold off;

%%

% filtro de planos
ptFiltrada = ptCena;
planes = 0;
while true
    [planeModel, inlierIdx, ~] = pcfitplane(ptFiltrada, 0.01);
    if numel(inlierIdx) < 50000  % Ignorar planos pequenos
        break
    end
    ptFiltrada = select(ptFiltrada, setdiff(1:ptFiltrada.Count, inlierIdx));
    planes = planes + 1;
end

fprintf("Removidos %d planos principais.\n", planes);


faces = {
    verts([1 2 3 4], :);  % base inferior
    verts([5 6 7 8], :);  % base superior
    verts([1 2 6 5], :);  % face X-min/X-max lado
    verts([2 3 7 6], :);  % face Y-min/Y-max lado
    verts([3 4 8 7], :);  % outro lado
    verts([4 1 5 8], :);  % outro lado
};

% === função para checar ponto dentro do cubo ===
xyz = ptCena.Location;
inside = true(ptCena.Count, 1);

for i = 1:numel(faces)
    V = faces{i};
    n = cross(V(2,:) - V(1,:), V(3,:) - V(1,:));
    n = n / norm(n);
    P0f = V(1,:);
    d = sum((xyz - P0f) .* n, 2);

    
    % Determinar sentido: para as duas bases, queremos d >=0 ou <=0?
    % Base inferior (i==1): normal aponta para cima, ponto dentro deve estar acima → d >= 0
    % Base superior (i==2): normal aponta para cima, ponto dentro deve estar abaixo do topo → d <= 0
    if i == 1
        inside = inside & (d >= 0);
    elseif i == 2
        inside = inside & (d <= 0);
    else
        inside = inside & (abs(d) <= 1e-6);  % face lateral, dentro é dentro das duas semiespaços
    end
end

% === selecionar e visualizar nuvem recortada ===
ptRecorte = select(ptCena, inside);
figure;
pcshow(ptRecorte, VerticalAxis="Y", VerticalAxisDir="Up");
title("Recorte da Nuvem: Dentro do cubo manual");
xlabel("X (m)"); ylabel("Y (m)"); zlabel("Z (m)");
%% === Importar modelo ===

switch lower(tipo_Objeto)
    case "can"
        gm = importGeometry("C:\Users\roboime\Desktop\Main\Modelos_garra\can.stl");
    case "bottle"
        gm = importGeometry("C:\Users\roboime\Desktop\Main\Modelos_garra\bottle.stl");
    otherwise
        disp("Objeto não reconhecido.");
end

pdemModel = createpde();
pdemModel.Geometry = gm;
generateMesh(pdemModel, 'Hmax', 0.1);  %Geração da malha refinada

% === Extrair nós da malha e converter para metros (modelo vem em mm)
nodes = pdemModel.Mesh.Nodes';     % Transpor de 3xN para N×3
nodes = nodes * 0.034;              % Corrigir escala (mm → m → fator de 10x menor)

% Criar nuvem de pontos do modelo
ptCloudModel = pointCloud(nodes);

% Mostrar modelo corretamente
figure;
pcshow(ptCloudModel, VerticalAxis="Y", VerticalAxisDir="Up");
title("Nuvem de Pontos do Modelo");
xlabel("X (m)"); ylabel("Y (m)"); zlabel("Z (m)");
%% === ICP ===

% Carregar as nuvens de pontos (exemplos .pcd, .ply, .mat, etc.)
fixed  = ptCloudModel;   % referência
moving = ptRecorte;  % a ser alinhada

% Pré-processamento
gridSize = 0.05;

% ICP refinado sem extrapolate
[tform, movingReg, rmse] = pcregistericp(...
    moving, fixed, ...
    Metric="pointToPlane", ...
    MaxIterations=1000, ...
    InlierRatio=0.8, ...
    Tolerance=[0.001, 0.5], ...
    Verbose=true ...
);

% === Inverter a transformação ===
tformInv = invert(tform);  % inverte a matriz de transformação

% === Aplicar a transformação inversa no modelo ===
modelAligned = pctransform(fixed, tformInv);  % agora o modelo se alinha à cena

% === Visualizar ===
figure;
pcshow(ptCena, VerticalAxis="Y", VerticalAxisDir="Up");
hold on;
pcshow(modelAligned, 'MarkerSize', 50);
title("Modelo Alinhado com ptRecorte Fixo");
xlabel("X (m)"); ylabel("Y (m)"); zlabel("Z (m)");
legend('ptRecorte (Cena Detecção)','ptCloudModel Alinhado');
view(3); grid on;

centerModel = mean(modelAligned.Location, 1);
%%
% === soma vetorial ===
pos_objeto = pos_garra + centerModel;
disp(pos_objeto);
end