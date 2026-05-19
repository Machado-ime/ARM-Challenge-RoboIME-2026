function ptCloudFiltrada = noiseFilter(ptCloud, minPontos, voxelSize)

    pts = ptCloud.Location;
    pts = reshape(pts, [], 3);
    pts = pts(~any(isnan(pts), 2), :);
    dist = sqrt(sum(pts.^2, 2));
    pts = pts(dist >= 0.4, :);

    gridCoord = floor(pts / voxelSize);
    [voxelKeys, ~, idxVoxel] = unique(gridCoord, 'rows');
    contagem = accumarray(idxVoxel, 1);
    voxelsValidos = contagem >= minPontos;
    mask = voxelsValidos(idxVoxel);
    ptsFiltrados = pts(mask, :);
    ptCloudFiltrada = pointCloud(ptsFiltrados);
end