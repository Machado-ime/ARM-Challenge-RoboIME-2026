function salvarImagemScript(imagemRGB)
    pastaDestino = 'C:\Users\ROBOIME5\Desktop\Dataset - ARM 2026'; % Altere para sua pasta
    dataHoraAtual = datestr(now, 'yyyymmdd_HHMMSS');
    nomeArquivo = sprintf('imagem_%s.png', dataHoraAtual);

    if ~exist(pastaDestino, 'dir')
        mkdir(pastaDestino);
    end
    caminhoCompleto = fullfile(pastaDestino, nomeArquivo);
    imwrite(imagemRGB, caminhoCompleto);
end