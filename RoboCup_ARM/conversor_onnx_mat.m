%% ── 1. Importa o modelo ONNX ──────────────────────────────
onnxFile = "best.onnx";   % ajuste o caminho

net = importNetworkFromONNX(onnxFile, ...
    "InputDataFormats",  "BCSS", ...   % Batch, Canais, Altura, Largura
    "OutputDataFormats", "BC");        % saída: Batch, Classes

%  Se a linha acima falhar (MATLAB < R2023b), use:
%  net = importONNXNetwork(onnxFile, "InputDataFormats", "BCSS");

disp(net)   % confere a arquitetura

%% ── 2. Testa com uma imagem ────────────────────────────────
img  = imread("teste.jpg");
img  = imresize(img, [640 640]);
img  = im2single(img);               % normaliza para [0,1]
img  = permute(img, [1 2 3]);        % HxWxC já está correto

pred = predict(net, img);
disp(pred)

%% ── 3. Salva como .mat ─────────────────────────────────────
save("detector.mat", "net", "-v7.3");   % -v7.3 suporta objetos grandes
disp("✅ Salvo em detector.mat")

%% ── 4. Recarregar depois ───────────────────────────────────
% loaded = load("detector.mat");
% net    = loaded.net;