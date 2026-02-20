%% ACUMULADOR DE NUVEM DE PONTOS RGB-D (SEM POSE)
% Funciona com a simulação RODANDO

% Intrínsecos (Simulation 3D Camera)
fx = 1109;
fy = 1109;
cx = 640;
cy = 360;

globalCloud = [];   % [X Y Z R G B]

disp('Coletando nuvem de pontos... Ctrl+C para parar');

while true
    % Aguarda novos dados
    if ~isfield(out,'depth') || isempty(out.depth.signals.values)
        pause(0.05);
        continue;
    end

    % Último frame disponível
    depth = out.depth.signals.values(:,:,end);
    rgb   = out.rgb.signals.values(:,:,:,end);

    if ~isa(rgb,'uint8')
        rgb = im2uint8(rgb);
    end

    [H,W] = size(depth);
    [u,v] = meshgrid(1:W,1:H);

    Z = depth;
    mask = Z > 0 & isfinite(Z);

    X = (u - cx) .* Z / fx;
    Y = (v - cy) .* Z / fy;

    X = X(mask);
    Y = Y(mask);
    Z = Z(mask);

    rgbPts = reshape(rgb,[],3);
    rgbPts = rgbPts(mask(:),:);

    pts = [X Y Z double(rgbPts)];

    % Acumula
    globalCloud = [globalCloud; pts];

    fprintf('Pontos acumulados: %d\n', size(globalCloud,1));

    pause(0.1); % controla taxa de aquisição
end