% 1. Adicionar o repositório ao path (se ainda não fez)
addpath('C:\caminho\para\Pretrained-YOLOv8-Network-For-Object-Detection')

% 2. Importar o modelo
net = importYOLOv8Model('C:\Users\ROBOIME5\Desktop\yolo\treino_arm\v1\weights\best.onnx');

% 3. Definir classes na mesma ordem do treino
classNames = {'blue_cube','bottle','can','green_cube', ...
              'marker','purple_cube','red_cube','spam'};

% 4. Criar o detector
det = yolov8ObjectDetector(net, classNames);

% 5. Salvar como .mat
save('detector_yolov8.mat', 'det');
disp('Salvo com sucesso!');