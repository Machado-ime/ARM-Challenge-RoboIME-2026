%% GERA NUVEM DE PONTOS RGB-D CORRETA (COM INTRÍNSECOS REAIS)

%% 1) Último frame
depth = out.depth.signals.values(:,:,end);      % HxW (m)
rgb   = out.rgb.signals.values(:,:,:,end);      % HxWx3

if ~isa(rgb,'uint8')
    rgb = im2uint8(rgb);
end

[H,W] = size(depth);

%% 2) Intrínsecos REAIS (do bloco Simulation 3D Camera)
fx = 1109;
fy = 1109;
cx = 640;
cy = 360;

%% 3) Grade de pixels
[u,v] = meshgrid(1:W,1:H);

Z = depth;
X = (u - cx) .* Z / fx;
Y = (v - cy) .* Z / fy;

%% 4) Máscara válida
mask = Z > 0 & isfinite(Z);

X = X(mask);
Y = Y(mask);
Z = Z(mask);

rgbPts = reshape(rgb,[],3);
rgbPts = rgbPts(mask(:),:);

%% 5) Nuvem no frame da câmera
ptsCam = [X Y Z];
ptCloudCam = pointCloud(ptsCam,'Color',rgbPts);

%% 6) Visualização CORRETA
figure;
pcshow(ptCloudCam);
axis equal;
grid on;
xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');
title('Nuvem de Pontos RGB-D (Frame da Câmera)');
view(3);