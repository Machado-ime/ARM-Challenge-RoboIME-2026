function [zPlano, ptPlano] = supPlane(ptCloud, dzInicial, minPontosInicial)
    pts = ptCloud.Location;
    pts = reshape(pts, [], 3);
    pts = pts(~any(isnan(pts),2), :);

    dzMin = 0.00001;
    minPontosMin = 10;

    dz = dzInicial;
    minPontos = minPontosInicial;

    zPlano = NaN;
    ptPlano = pointCloud(zeros(0,3));

    while dz >= dzMin && minPontos >= minPontosMin
        zMin = min(pts(:,3));
        zMax = max(pts(:,3));
        numDivZ = floor((zMax - zMin)/dz);

        if numDivZ < 1
            dz = dz / 3; 
            continue;
        end

        edges = linspace(zMin, zMax, numDivZ+1);

        for i = 1:numDivZ
            inBin = pts(:,3) >= edges(i) & pts(:,3) < edges(i+1);
            if sum(inBin) >= minPontos
                ptsPlano = pts(inBin, :);
                zPlano = mean(ptsPlano(:,3));
                ptPlano = pointCloud(ptsPlano);
                return;
            end
        end

        if minPontos > minPontosMin
            minPontos = minPontos / 2;
        elseif dz > dzMin
            dz = dz / 2;
        else
            break; % Nada mais a ajustar
        end
    end
end

% function [zPlano, ptPlano] = supPlane(ptCloud, dz, minPontos)
%     pts = ptCloud.Location;
%     pts = reshape(pts, [], 3);
%     pts = pts(~any(isnan(pts),2), :);
% 
%     zMin = min(pts(:,3));
%     zMax = max(pts(:,3));
% 
%     numDivZ = floor((zMax - zMin)/dz);
%     if numDivZ < 1
%         error("Valor de dz muito grande para a extensão do plano Z.");
%         if dz > 0.00001
%             [zPlano, ptPlano] = supPlane(ptCloud, dz/3, minPontos);
%         end
%         return;
%     end
% 
%     edges = linspace(zMin, zMax, numDivZ+1);
% 
%     for i = 1:numDivZ
%         inBin = pts(:,3) >= edges(i) & pts(:,3) < edges(i+1);
%         if sum(inBin) >= minPontos
%             ptsPlano = pts(inBin, :);
%             zPlano = mean(ptsPlano(:,3));
%             ptPlano = pointCloud(ptsPlano);
%             return;
%         end
%     end
% 
%     zPlano = NaN;
%     ptPlano = pointCloud(zeros(0,3));
%     %warning("Nenhum plano encontrado com pelo menos %d pontos.", minPontos);
%     if minPontos > 100
%         [zPlano, ptPlano] = supPlane(ptCloud, dz, minPontos/2);
%     end
% end
