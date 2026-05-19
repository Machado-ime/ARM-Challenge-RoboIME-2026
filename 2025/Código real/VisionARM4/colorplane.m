function [ptFiltrada, corMedia] = colorplane(ptCloud, ptPlano, tolerancia)

    if isempty(ptCloud.Color) || isempty(ptPlano.Color)
        error("Ambas as pointClouds devem conter dados de cor.");
    end

    corPlano = reshape(ptPlano.Color, [], 3);
    corTotal = reshape(ptCloud.Color, [], 3);

    corMedia = mean(double(corPlano), 1);

    distCor = sqrt(sum((double(corTotal) - corMedia).^2, 2));

    idxMantidos = find(distCor > tolerancia);
    ptFiltrada = select(ptCloud, idxMantidos);
end