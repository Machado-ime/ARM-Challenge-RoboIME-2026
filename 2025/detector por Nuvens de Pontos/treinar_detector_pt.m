% === Carregar Labels e Dados ===
labelFile = 'G:\Drives compartilhados\ARM challenge - 2025\NP rotulados - Can 1\export_can_1.mat';
labelData = load(labelFile);
gTruth = labelData.gTruth;  % groundTruthLidar

% Converter em datastore para treinamento
[pcds, bxds] = lidarObjectDetectorTrainingData(gTruth);
trainingData = combine(pcds, bxds);

% === Definir Configuração ===
classes = ["Can"];
anchorBoxes = {
    [0.07 0.07 0.12 0.06  0;     % orientação 0°
     0.07 0.07 0.12 0.06  pi/4]     % orientação 90°
};

% Especifique o alcance do point cloud (ajuste conforme seu sensor)
pcRange = [-10, 10, -10, 10, -2, 2];  % [xmin,xmax, ymin,ymax, zmin,zmax]
voxelSize = [0.1, 0.1];

detector = pointPillarsObjectDetector(pcRange, classes, anchorBoxes, ...
    VoxelSize=voxelSize);

% === Escolher GPU ou CPU ===
if canUseGPU
    execEnv = "gpu";
else
    execEnv = "cpu";
end

% === Opções de Treinamento ===
options = trainingOptions("adam", ...
    MaxEpochs=30, ...
    MiniBatchSize=4, ...
    InitialLearnRate=0.001, ...
    BatchNormalizationStatistics="moving", ...
    ResetInputNormalization=false, ...
    ExecutionEnvironment=execEnv, ...
    Plots="training-progress", ...
    CheckpointPath="C:\Users\pecci\OneDrive\Desktop\Checkpoints");


% === Treinamento ===
[detector, info] = trainPointPillarsObjectDetector(trainingData, detector, options);

% === Salvar Detector ===
save("C:\Users\pecci\OneDrive\Desktop\Detectores\Detector_teste.mat", "detector", "info");
