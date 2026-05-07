function  [tipo, pos_objeto, estado, angulo] = detector_pt(pos_garra, rgb, depth)
    %% Entradas:
    %   pos_garra — [1x3] posição atual da garra em metros
    %   rgb       — [HxWx3] uint8 imagem colorida da câmera
    %   depth     — [HxW]   double imagem de profundidade em metros
    
    % Saídas seguras
    tipo       = 0;   % ← adicionar esta linha
    estado     = 0;
    angulo     = 0.0;
    pos_objeto = pos_garra;
    
    % ── Saídas seguras ───────────────────────────────────────────────────────
    pos_objeto  = pos_garra;
    
    % ── Intrínsecos da câmera ────────────────────────────────────────────────
    fx = 1109;  fy = 1109;
    cx = 640;   cy = 360;
    
    %% === 1. Construir nuvem de pontos a partir de rgb + depth ================
    if ~isa(rgb, 'uint8')
        rgb = im2uint8(rgb);
    end
    
    [H, W] = size(depth);
    [u, v] = meshgrid(1:W, 1:H);
    
    Z = double(depth);
    X = (u - cx) .* Z / fx;
    Y = (v - cy) .* Z / fy;
    
    rgbPts    = reshape(rgb, [], 3);
    mask_full = Z > 0 & isfinite(Z);
    
    ptCena = pointCloud([X(mask_full), Y(mask_full), Z(mask_full)], ...
                        'Color', rgbPts(mask_full(:), :));
    
    % figure;
    % pcshow(ptCena, VerticalAxis='Y', VerticalAxisDir='Up');
    % title('Nuvem Completa'); xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
    
    %% === 2. Detecção direta na imagem RGB ====================================
    persistent colorDetector;
    
    if isempty(colorDetector)
        modelPath     = 'C:\Users\ROBOIME5\Desktop\ARM-Challenge-RoboIME-2026\RoboCup_ARM\meuDetectorTreinado.mat';
        data          = load(modelPath);
        colorDetector = data.detector;   % ← era 'colorDetector', o correto é 'detector'
        disp('[detector_pt] Modelo carregado.');
    end
    
    [dbox, dscore, dlabel] = detect(colorDetector, rgb);
    % Filtrar score >= 0.6
    validIdx = dscore >= 0.6;
    dbox     = dbox(validIdx, :);
    dscore   = dscore(validIdx);
    dlabel   = dlabel(validIdx);
    
    if isempty(dbox)
        disp('Nenhum objeto detectado com score acima de 60%.');
        return;
    end
    
    % Mostrar detecções
    labelsWithScores = string(dlabel) + ': ' + string(dscore);
    imTested = insertObjectAnnotation(rgb, 'rectangle', dbox, labelsWithScores);
    figure; imshow(imTested);
    title('Detecções na imagem RGB');
    pause(2);
    
    % Objeto com maior score
    [~, idxMax]  = max(dscore);
    tipo_Objeto  = string(dlabel(idxMax));
    bboxMax      = round(dbox(idxMax, :));  % [x, y, w, h] em pixels
    fprintf('Objeto com maior score: %s (%.2f%%)\n', tipo_Objeto, dscore(idxMax)*100);
    
    
    %tipo
    switch lower(tipo_Objeto)
        case 'can'
            tipo = 1;
        case 'bottle'
            tipo = 2;
        case 'spam'
            tipo = 3;
        case 'marker'
            tipo = 4;
        case {'green_cube', 'purple_cube'}
            tipo = 5;
        case {'blue_cube', 'red_cube'}
            tipo = 6;
        otherwise
            tipo = 0; % desconhecido
    end
    
    %% === 3. Converter bbox de pixels → metros via depth =====================
    xLeft   = bboxMax(1);
    yTop    = bboxMax(2);
    xRight  = xLeft + bboxMax(3);
    yBottom = yTop  + bboxMax(4);
    
    % Recorta região da bbox no depth
    xLeft_c  = max(1, xLeft);
    xRight_c = min(W, xRight);
    yTop_c   = max(1, yTop);
    yBot_c   = min(H, yBottom);
    
    % Máscara 2D da bbox no espaço da imagem
    mask_bbox = false(H, W);
    mask_bbox(yTop_c:yBot_c, xLeft_c:xRight_c) = true;
    mask_obj  = mask_full & mask_bbox;
    
    % Cantos da bbox convertidos para metros usando Z médio da região
    if any(mask_obj(:))
        Z_medio = mean(Z(mask_obj));
    else
        Z_medio = mean(Z(mask_full));
    end
    
    P0_XY = [(xLeft  - cx) * Z_medio / fx,  (yTop    - cy) * Z_medio / fy];
    P1_XY = [(xRight - cx) * Z_medio / fx,  (yBottom - cy) * Z_medio / fy];
    
    cornersXY = [
        P0_XY(1), P0_XY(2);
        P1_XY(1), P0_XY(2);
        P1_XY(1), P1_XY(2);
        P0_XY(1), P1_XY(2)
    ];
    
    zMin = min(ptCena.Location(:,3));
    zMax = max(ptCena.Location(:,3));
    
    % 8 vértices 3D
    verts = [];
    for i = 1:4
        verts = [verts; cornersXY(i,:), zMin];
    end
    for i = 1:4
        verts = [verts; cornersXY(i,:), zMax];
    end
    
    edges = [1 2;2 3;3 4;4 1; 5 6;6 7;7 8;8 5; 1 5;2 6;3 7;4 8];
    
    % % Nuvem + caixa 3D
    % figure;
    % pcshow(ptCena, VerticalAxis='Y', VerticalAxisDir='Up');
    % hold on;
    % for e = 1:size(edges,1)
    %     p1 = verts(edges(e,1),:);
    %     p2 = verts(edges(e,2),:);
    %     plot3([p1(1) p2(1)],[p1(2) p2(2)],[p1(3) p2(3)],'r-','LineWidth',2);
    % end
    % title('Nuvem com Região de Detecção');
    % xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
    % hold off;

    %% === 4. Recorte 3D dentro da caixa ======================================
    xyz_all = ptCena.Location;
    
    inside = xyz_all(:,1) >= min(cornersXY(:,1)) & ...
             xyz_all(:,1) <= max(cornersXY(:,1)) & ...
             xyz_all(:,2) >= min(cornersXY(:,2)) & ...
             xyz_all(:,2) <= max(cornersXY(:,2)) & ...
             xyz_all(:,3) >= zMin                & ...
             xyz_all(:,3) <= zMax;
    
    ptRecorte = select(ptCena, inside);
    
    % figure;
    % pcshow(ptRecorte, VerticalAxis='Y', VerticalAxisDir='Up');
    % title('Recorte da Nuvem: Objeto Detectado');
    % xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
    
    %% === 5. Filtro de planos =================================================
    ptFiltrada = ptRecorte;
    % planes = 0;
    % while true
    %     [~, inlierIdx, ~] = pcfitplane(ptFiltrada, 0.01);
    %     if numel(inlierIdx) < 1000
    %         break;
    %     end
    %     ptFiltrada = select(ptFiltrada, setdiff(1:ptFiltrada.Count, inlierIdx));
    %     planes = planes + 1;
    % end
    %fprintf('Removidos %d planos.\n', planes);
    
    %% === 6. Estimativa de pose ===============================================
    if tipo == 5 || tipo == 6
        % ── CUBOS: sem ICP, posição por centroide + ângulo por PCA ──────────

        xyzObj       = ptFiltrada.Location;
        offset_cam   = [0.05, 0, 0];
        xyzObj_corr  = [-xyzObj(:,2), -xyzObj(:,1), -xyzObj(:,3)];
        xyzObj_corr  = xyzObj_corr + pos_garra + offset_cam;

        %% === 6.5. Nuvem da cena corrigida ===================================
        xyzCena      = ptCena.Location;
        colCena      = ptCena.Color;
        xyzCena_corr = [-xyzCena(:,2), -xyzCena(:,1), -xyzCena(:,3)];
        xyzCena_corr = xyzCena_corr + pos_garra + offset_cam;
        ptCena_corr  = pointCloud(xyzCena_corr, 'Color', colCena);
        ptObj_corr   = pointCloud(xyzObj_corr, 'Color', ptFiltrada.Color);

        figure;
        pcshow(ptCena_corr, VerticalAxis='Y', VerticalAxisDir='Up');
        hold on;
        pcshow(ptObj_corr, 'MarkerSize', 50);
        title('Nuvem no Frame do Mundo – Cena + Cubo');
        xlabel('X\_mundo (m)'); ylabel('Y\_mundo (m)'); zlabel('Z\_mundo (m)');
        legend('Cena','Cubo');
        hold off;

        %% === 7. Estado + Ângulo via PCA =====================================
        extV = max(xyzObj_corr(:,2)) - min(xyzObj_corr(:,2));
        extH = max([max(xyzObj_corr(:,1))-min(xyzObj_corr(:,1)), ...
                    max(xyzObj_corr(:,3))-min(xyzObj_corr(:,3))]);

        if extV >= 0.8 * extH
            estado = 1;
            fprintf('Estado: EM PÉ\n');
        else
            estado = 0;
            fprintf('Estado: DEITADO\n');
        end

        pts2D  = [xyzObj_corr(:,1), xyzObj_corr(:,3)];
        pts2D  = pts2D - mean(pts2D, 1);
        [V, ~] = eig((pts2D' * pts2D) / (size(pts2D,1)-1));
        eixo   = V(:, end);
        angulo = atan2(eixo(2), eixo(1)) + pi/2;
        fprintf('Ângulo (PCA): %.2f°\n', rad2deg(angulo));

        %% === 8. Posição final ===============================================
        pos_objeto = mean(xyzObj_corr, 1);
        fprintf('Posição do objeto (mundo): X=%.3f  Y=%.3f  Z=%.3f\n', ...
                pos_objeto(1), pos_objeto(2), pos_objeto(3));

    else
        % ── DEMAIS OBJETOS: fluxo original com ICP ───────────────────────────

        switch lower(tipo_Objeto)
        case 'can'
            gm = importGeometry('C:\Users\ROBOIME5\Desktop\ARM-Challenge-RoboIME-2026\RoboCup_ARM\Modelos_garra\can.stl');
            pdemModel = createpde();
            pdemModel.Geometry = gm;
            generateMesh(pdemModel, 'Hmax', 0.1);
            nodes = pdemModel.Mesh.Nodes' * 0.034;
            ptCloudModel = pointCloud(nodes);

        case 'bottle'
            gm = importGeometry('C:\Users\ROBOIME5\Desktop\ARM-Challenge-RoboIME-2026\RoboCup_ARM\Modelos_garra\bottle.stl');
            pdemModel = createpde();
            pdemModel.Geometry = gm;
            generateMesh(pdemModel, 'Hmax', 0.1);
            nodes = pdemModel.Mesh.Nodes' * 0.034;
            ptCloudModel = pointCloud(nodes);

        case 'spam'
            ptCloudModel = pcread('C:\Users\ROBOIME5\Desktop\ARM-Challenge-RoboIME-2026\RoboCup_ARM\Modelos_garra\spam.ply');

        case 'marker'
            ptCloudModel = pcread('C:\Users\ROBOIME5\Desktop\ARM-Challenge-RoboIME-2026\RoboCup_ARM\Modelos_garra\marker.ply');

        otherwise
            disp('Objeto não reconhecido para ICP.');
            return;
        end

        [tform, ~, ~] = pcregistericp( ...
            ptFiltrada, ptCloudModel, ...
            Metric='pointToPlane', ...
            MaxIterations=1000, ...
            InlierRatio=0.8, ...
            Tolerance=[0.001, 0.5]);

        tformInv     = invert(tform);
        modelAligned = pctransform(ptCloudModel, tformInv);

        %% === 6.5. Correcção de orientação ===================================
        xyzCena = ptCena.Location;
        colCena = ptCena.Color;
        offset_cam = [0.05, 0, 0];

        xyzCena_corr = [-xyzCena(:,2), -xyzCena(:,1), -xyzCena(:,3)];
        xyzCena_corr = xyzCena_corr + pos_garra + offset_cam;
        ptCena_corr  = pointCloud(xyzCena_corr, 'Color', colCena);

        xyzModel      = modelAligned.Location;
        xyzModel_corr = [-xyzModel(:,2), -xyzModel(:,1), -xyzModel(:,3)];
        xyzModel_corr = xyzModel_corr + pos_garra + offset_cam;
        ptModel_corr  = pointCloud(xyzModel_corr);

        figure;
        pcshow(ptCena_corr, VerticalAxis='Y', VerticalAxisDir='Up');
        hold on;
        pcshow(ptModel_corr, 'MarkerSize', 50);
        title('Nuvem no Frame do Mundo – Cena + Modelo Alinhado');
        xlabel('X\_mundo (m)'); ylabel('Y\_mundo (m)'); zlabel('Z\_mundo (m)');
        legend('Cena','Modelo Alinhado');
        hold off;

        %% === 7. Estado + Roll, Pitch, Yaw ===================================
        try
            R = tformInv.R;
        catch
            R = tformInv.T(1:3,1:3);
        end

        roll  = atan2d(R(3,2), R(3,3));
        pitch = atan2d(-R(3,1), sqrt(R(3,2)^2 + R(3,3)^2));
        yaw   = atan2d(R(2,1), R(1,1));

        limiar = 45;
        if abs(pitch) < limiar
            estado = 1;
            fprintf('Estado: EM PÉ\n');
        else
            estado = 0;
            fprintf('Estado: DEITADO\n');
        end

        if estado == 1
            angulo = yaw + pi/2;    % em pé  → usa yaw
        else
            angulo = roll + pi/2;   % deitado → usa roll
        end

        fprintf('Roll: %.2f° | Pitch: %.2f° | Yaw: %.2f°\n', roll, pitch, yaw);

        %% === 8. Posição final ===============================================
        centerModel_corr = mean(xyzModel_corr, 1);
        pos_objeto(1) = centerModel_corr(1);
        pos_objeto(2) = centerModel_corr(2);
        pos_objeto(3) = centerModel_corr(3);

        fprintf('Posição do objeto (mundo): X=%.3f  Y=%.3f  Z=%.3f\n', ...
                pos_objeto(1), pos_objeto(2), pos_objeto(3));
    end