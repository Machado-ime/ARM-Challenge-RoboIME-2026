function obstacles = Obstaculos()
    modelMsg = receive(modelSub, 10); % Espera até 3 segundos
    idx = find(strcmp(modelMsg.Name, 'robot'));  % substitua 'robot' pelo nome correto
    pos = [modelMsg.Pose(idx).Position.X, modelMsg.Pose(idx).Position.Y, modelMsg.Pose(idx).Position.Z];
    ori = [modelMsg.Pose(idx).Orientation.X, modelMsg.Pose(idx).Orientation.Y, ...
           modelMsg.Pose(idx).Orientation.Z, modelMsg.Pose(idx).Orientation.W];
    tformBase = trvec2tform(pos) * quat2tform(ori)
    % Extrair nomes dos modelos, posições e orientações
    modelNames = modelMsg.Name;
    positions = modelMsg.Pose;
    
    % Criar lista para armazenar obstáculos
    obstacles = {};
    
    % Loop para processar cada modelo
    for idx = 1:length(modelNames)
        name = modelNames{idx};

    % Filtrar: ignorar o robô UR5e e a base
    if contains (name, 'base') || contains(name, 'ground_plane')
        continue; % Pula esses
    end
    if contains(name, 'box')
        pos = [positions(idx).Position.X, positions(idx).Position.Y, positions(idx).Position.Z];
        ori = [positions(idx).Orientation.X, positions(idx).Orientation.Y, ...
               positions(idx).Orientation.Z, positions(idx).Orientation.W];
        dimx= 0.24;
        dimy = 0.16; 
        dimz = 0.11;
        obstacle = collisionBox(dimx, dimy, dimz);
        tform = trvec2tform(pos)*quat2tform(ori);
        obstacle.Pose = tform;
        obstacles{end + 1} = obstacle;
        continue;
    end
    if contains(name, 'block')
        pos = [positions(idx).Position.X, positions(idx).Position.Y, positions(idx).Position.Z];
        ori = [positions(idx).Orientation.X, positions(idx).Orientation.Y, ...
               positions(idx).Orientation.Z, positions(idx).Orientation.W];
        dimxa= 0.025;
        dimya = 0.025; 
        dimza = 0.025;
        obstacle = collisionBox(dimxa, dimya, dimza);
        tform = trvec2tform(pos)*quat2tform(ori);
        obstacle.Pose = tform;
        obstacles{end + 1} = obstacle;
        continue;
    end
    if contains(name, 'robot')
        pos = [modelMsg.Pose(idx).Position.X, modelMsg.Pose(idx).Position.Y, modelMsg.Pose(idx).Position.Z];
        ori = [modelMsg.Pose(idx).Orientation.X, modelMsg.Pose(idx).Orientation.Y, ...
               modelMsg.Pose(idx).Orientation.Z, modelMsg.Pose(idx).Orientation.W];
        tformBase = trvec2tform(pos) * quat2tform(ori);
        quat2tform(ori);
    end
    % Pega a posição e orientação
    pos = [positions(idx).Position.X, positions(idx).Position.Y, positions(idx).Position.Z];
    ori = [positions(idx).Orientation.X, positions(idx).Orientation.Y, ...
           positions(idx).Orientation.Z, positions(idx).Orientation.W];

    % Defina um tamanho padrão para os obstáculos (ou ajuste conforme seus modelos)
    raio = 0.033;
    altura = 3*0.033;
    obstacle = collisionCylinder(raio, altura);

    % Define a pose do obstáculo
    tform = trvec2tform(pos)*quat2tform(ori);
    obstacle.Pose = tform;

    % Armazena o obstáculo
    obstacles{end + 1} = obstacle;
end
disp(['Total de obstáculos detectados: ', num2str(length(obstacles))]);