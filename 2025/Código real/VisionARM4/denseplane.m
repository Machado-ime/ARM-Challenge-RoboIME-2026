function [zPlano, ptPlano] = denseplane(ptCloud, larguraZ)
    pts = ptCloud.Location;
    pts = reshape(pts, [], 3);
    pts = pts(~any(isnan(pts), 2), :);  

    z = pts(:,3);
    zMin = min(z);
    zMax = max(z);
    edges = zMin:larguraZ:zMax;

    [counts, ~, binIdx] = histcounts(z, edges);
    [~, maxBin] = max(counts);
    zLow = edges(maxBin);
    zHigh = edges(maxBin+1);

    inFaixa = z >= zLow & z <= zHigh;
    planoPts = pts(inFaixa, :);

    color = [];
    if ~isempty(ptCloud.Color)
        cor = reshape(ptCloud.Color, [], 3);
        corPlano = cor(inFaixa, :);
        color = corPlano;
    end

    if isempty(color)
        ptPlano = pointCloud(planoPts);
    else
        ptPlano = pointCloud(planoPts, "Color", color);
    end

    zPlano = mean(planoPts(:,3));
end
