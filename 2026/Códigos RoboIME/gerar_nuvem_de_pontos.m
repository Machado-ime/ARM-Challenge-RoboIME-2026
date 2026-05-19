function gerar_nuvem_de_pontos(depth, rgb, dbox, margem_m)

persistent hFig_full hFig_crop hFig_rect;
if isempty(hFig_full) || ~isvalid(hFig_full)
    hFig_full = figure('Name', 'Nuvem Completa',         'NumberTitle', 'off', 'Position', [100  100 700 500]);
end
if isempty(hFig_crop) || ~isvalid(hFig_crop)
    hFig_crop = figure('Name', 'Nuvem Detectada',        'NumberTitle', 'off', 'Position', [820  100 700 500]);
end
if isempty(hFig_rect) || ~isvalid(hFig_rect)
    hFig_rect = figure('Name', 'Nuvem + Região de Corte','NumberTitle', 'off', 'Position', [460  620 700 500]);
end

if nargin < 4 || isempty(margem_m)
    margem_m = 0.05;
end
if ~isa(rgb, 'uint8')
    rgb = im2uint8(rgb);
end

[H, W] = size(depth);
fx = 1109;  fy = 1109;
cx = 640;   cy = 360;

[u, v] = meshgrid(1:W, 1:H);
Z = double(depth);
X = (u - cx) .* Z / fx;
Y = (v - cy) .* Z / fy;

rgbPts    = reshape(rgb, [], 3);
mask_full = Z > 0 & isfinite(Z);

% ════════════════════════════════════════════════════════════════════════
% FIGURA 1 — Nuvem completa (pcshow isolado, sem hold)
% ════════════════════════════════════════════════════════════════════════
if any(mask_full(:))
    ptCloudFull = pointCloud([X(mask_full), Y(mask_full), Z(mask_full)], ...
                             'Color', rgbPts(mask_full(:), :));
    set(0, 'CurrentFigure', hFig_full);
    pcshow(ptCloudFull);
    axis equal; grid on;
    xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
    title('Nuvem Completa');
    view(3);
end

if ~isempty(dbox) && any(dbox ~= 0)

    X1 = min(dbox(1), dbox(3)) - margem_m;
    X2 = max(dbox(1), dbox(3)) + margem_m;
    Y1 = min(dbox(2), dbox(4)) - margem_m;
    Y2 = max(dbox(2), dbox(4)) + margem_m;

    mask_crop = mask_full & X >= X1 & X <= X2 & Y >= Y1 & Y <= Y2;

    % ════════════════════════════════════════════════════════════════════
    % FIGURA 2 — Nuvem detectada (pcshow isolado)
    % ════════════════════════════════════════════════════════════════════
    if any(mask_crop(:))
        ptCloudDetected = pointCloud([X(mask_crop), Y(mask_crop), Z(mask_crop)], ...
                                     'Color', rgbPts(mask_crop(:), :));
        set(0, 'CurrentFigure', hFig_crop);
        pcshow(ptCloudDetected);
        axis equal; grid on;
        xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
        title('Nuvem Detectada');
        view(3);
    end

    % ════════════════════════════════════════════════════════════════════
    % FIGURA 3 — Retângulo PRIMEIRO, depois scatter3 por cima
    % ════════════════════════════════════════════════════════════════════
    pts_z_crop = Z(mask_crop);
    if ~isempty(pts_z_crop)
        Z_topo = mean(pts_z_crop);
    else
        Z_topo = 1.0;
    end

    rx = [X1, X2, X2, X1, X1];
    ry = [Y1, Y1, Y2, Y2, Y1];
    cx_ = [X1, X2, X2, X1];
    cy_ = [Y1, Y1, Y2, Y2];

    set(0, 'CurrentFigure', hFig_rect);
    cla;
    set(hFig_rect, 'Color', 'k');       % fundo preto igual ao pcshow
    ax3 = gca;
    set(ax3, 'Color', 'k', ...
             'XColor', 'w', 'YColor', 'w', 'ZColor', 'w');

    % 1º — retângulo wireframe (base + tampa + pilares)
    hold on;
    plot3(ax3, rx, ry, zeros(1,5),         'r-',  'LineWidth', 2.5);   % base
    plot3(ax3, rx, ry, ones(1,5)*Z_topo,   'r:',  'LineWidth', 1.5);   % tampa
    for k = 1:4
        plot3(ax3, [cx_(k) cx_(k)], [cy_(k) cy_(k)], [0 Z_topo], ...
              'r--', 'LineWidth', 1.2);                                  % pilares
    end

    % 2º — nuvem completa via scatter3 (não reseta axes)
    if any(mask_full(:))
        Xf = X(mask_full);
        Yf = Y(mask_full);
        Zf = Z(mask_full);
        Cf = double(rgbPts(mask_full(:), :)) / 255;

        % Subsample para manter fluidez (máx 40 000 pontos)
        n_pts = numel(Xf);
        if n_pts > 40000
            idx_s = randperm(n_pts, 40000);
            Xf = Xf(idx_s); Yf = Yf(idx_s);
            Zf = Zf(idx_s); Cf = Cf(idx_s, :);
        end

        scatter3(ax3, Xf, Yf, Zf, 1, Cf, 'filled');
    end

    hold off;

    axis(ax3, 'equal'); grid(ax3, 'on');
    xlabel(ax3, 'X (m)'); ylabel(ax3, 'Y (m)'); zlabel(ax3, 'Z (m)');
    title(ax3, sprintf('Nuvem + Região de Corte  ±%.2f m', margem_m));
    view(ax3, 3);
end

drawnow limitrate;
end