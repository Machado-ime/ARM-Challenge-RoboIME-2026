% 1. Importar o STL
stlvp = stlread('C:\Users\ROBOIME5\Desktop\ARM-Challenge-RoboIME-2026\RoboCup_ARM\Modelos_garra\cubo.stl');
V = stlvp.Points;
F = stlvp.ConnectivityList;

% 2. Definir quantos pontos você quer no total
numPoints = 50000; % Aumente aqui se quiser ainda mais pontos

% 3. Gerar pontos aleatórios na superfície dos triângulos
numFaces = size(F, 1);
areas = zeros(numFaces, 1);

% Calcular a área de cada triângulo para distribuir os pontos proporcionalmente
for i = 1:numFaces
    A = V(F(i,1), :);
    B = V(F(i,2), :);
    C = V(F(i,3), :);
    areas(i) = 0.5 * norm(cross(B-A, C-A));
end

% Sortear quais faces receberão pontos baseando-se na área
faceIndices = discretize(rand(numPoints, 1), [0; cumsum(areas)/sum(areas)]);

% Gerar os pontos usando coordenadas baricêntricas
r1 = rand(numPoints, 1);
r2 = rand(numPoints, 1);
mask = (r1 + r2) > 1;
r1(mask) = 1 - r1(mask);
r2(mask) = 1 - r2(mask);
r3 = 1 - r1 - r2;

P = zeros(numPoints, 3);
for i = 1:numPoints
    idx = faceIndices(i);
    P(i,:) = r1(i)*V(F(idx,1),:) + r2(i)*V(F(idx,2),:) + r3(i)*V(F(idx,3),:);
end

% 4. Aplicar sua escala de 0.034
nodes = P * 0.034;

% 5. Criar e Visualizar a Nuvem
ptCloudModel = pointCloud(single(nodes));

figure('Color', 'w');
pcshow(ptCloudModel, 'MarkerSize', 30);
title(sprintf('Nuvem de Pontos com %d pontos', numPoints));
xlabel('X'); ylabel('Y'); zlabel('Z');

% 6. Salvar
pcwrite(ptCloudModel, 'cubo_denso.ply', 'Encoding', 'ascii');
fprintf('Sucesso! Arquivo "cubo_denso.ply" gerado com %d pontos.\n', numPoints);