function obstacles = Obstaculos(modelSub)
    modelMsg = receive(modelSub, 10); % Espera até 3 segundos

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
        pos = [-positions(idx).Position.Y, positions(idx).Position.X + 0.1,  positions(idx).Position.Z-.75];
        ori = [positions(idx).Orientation.X, positions(idx).Orientation.Y, ...
               positions(idx).Orientation.Z, positions(idx).Orientation.W];
        dimx= 0.11;
        dimy = 0.11; 
        dimz = 0.11;
        obstacle = collisionBox(dimx, dimy, dimz);
        tform = trvec2tform(pos)*quat2tform(ori);
        obstacle.Pose = tform;
        obstacles{end + 1} = obstacle;
        continue;
    end
    if contains(name, 'block')
       pos = [-positions(idx).Position.Y, positions(idx).Position.X + 0.1,  positions(idx).Position.Z-.75];
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
           pos = [-positions(idx).Position.Y, positions(idx).Position.X + 0.1,  positions(idx).Position.Z-.75];
        ori = [modelMsg.Pose(idx).Orientation.X, modelMsg.Pose(idx).Orientation.Y, ...
               modelMsg.Pose(idx).Orientation.Z, modelMsg.Pose(idx).Orientation.W];
        tformBase = trvec2tform(pos) * quat2tform(ori);
        quat2tform(ori);
        continue
    end

    if contains(name, 'Bottle')
        pos = [-positions(idx).Position.Y, positions(idx).Position.X + 0.1,  positions(idx).Position.Z-.75];
        ori = [modelMsg.Pose(idx).Orientation.X, modelMsg.Pose(idx).Orientation.Y, ...
               modelMsg.Pose(idx).Orientation.Z, modelMsg.Pose(idx).Orientation.W];
        raio = 0.028;
        altura = 1.5*3*0.038;
        obstacle = collisionCylinder(raio, altura);
    
        % Define a pose do obstáculo
        tform = trvec2tform(pos)*quat2tform(ori);
        obstacle.Pose = tform;
    
        % Armazena o obstáculo
        obstacles{end + 1} = obstacle;
        continue
    end

    % Pega a posição e orientação
    pos = [-positions(idx).Position.Y, positions(idx).Position.X + 0.1,  positions(idx).Position.Z-.75];
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