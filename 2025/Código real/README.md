# Código Principal

Pipeline completo de percepção e controle para a **fase real** da competição. Integra a câmera RealSense D415, detecção YOLO 2D, processamento de nuvem de pontos e envio de comandos ao braço UR5e via ROS.

A versão mais recente e organizada do sistema de visão está na subpasta [`VisionARM4/`](VisionARM4/).

---

## Fluxo do pipeline

```
realsenseSubscriberSO_ARM  →  PcGenRSD435  →  noiseFilter  →  denseplane
      (câmera)                 (nuvem de         (filtra        (detecta
                                pontos)           ruído)        plano mesa)
                                   │
                                   ▼
                              ARMVision.m   ←   YOLO detector
                            (orquestrador)
                                   │
                              supPlane + EvalFun
                            (extrai objetos, associa
                             detecções 2D com 3D)
                                   │
                                   ▼
                            displacement (posição 3D
                            do objeto em relação ao braço)
```

---

## Arquivos principais

### `ARMVision.m`
Função central do sistema. Recebe imagem colorida, imagem de profundidade e o detector YOLO; devolve a posição 3D dos objetos detectados e metadados das detecções.

```matlab
[displacement, idx, sts, labels, scores] = ARMVision(colorImage, depthImage, Detector, realHeight)
% colorImage  — imagem RGB 480×640
% depthImage  — imagem de profundidade 480×640
% Detector    — objeto detector YOLO treinado
% realHeight  — altura real da mesa (metros), usada como fator de escala
```

### `realsenseSubscriberSO_ARM.m`
System Object MATLAB que gerencia o streaming da câmera RealSense D415. A cada chamada de `step()` devolve:
- `colorImage` — imagem RGB (480×640×3, uint8)
- `orderedPointCloud` — nuvem de pontos alinhada à imagem de cor (480×640×3, double)

Requer o **Intel RealSense SDK 2.0** instalado em `C:\Program Files (x86)\Intel RealSense SDK 2.0\matlab`.

### `PcGenRSD435.m`
Gera a nuvem de pontos 3D a partir dos dados da câmera e executa a detecção YOLO sobre a imagem colorida. Segmenta a nuvem por objeto detectado.

### `noiseFilter.m`
Filtra pontos espúrios da nuvem usando remoção de *outliers* por raio. Recebe a nuvem, o número mínimo de vizinhos e o raio de busca.

### `denseplane.m`
Detecta o plano dominante da cena (a mesa) na nuvem de pontos usando RANSAC. Retorna a altura `z` do plano, usada como referência para calcular o fator de correção de escala.

### `supPlane.m`
Isola os pontos acima do plano da mesa, separando os objetos de interesse da superfície.

### `objextract.m`
Extrai e segmenta objetos individuais a partir da nuvem filtrada.

### `EvalFun.m`
Associa as detecções 2D do YOLO com os clusters 3D da nuvem de pontos, resolvendo a correspondência entre o que foi visto na imagem e o que foi medido em profundidade.

### `coletor_imagens.m`
Utilitário para capturar e salvar pares de imagens (cor + profundidade) da câmera. Usado para coletar dados de treinamento do detector.

---

## Scripts de operação (`*.mlx`)

Scripts executados durante a operação real do braço:

| Script | Descrição |
|---|---|
| `main.mlx` | Script principal de execução da tarefa completa |
| `pegar.mlx` | Sequência de movimentos para pegar um objeto |
| `posicionar.mlx` | Leva o objeto para a posição de destino |
| `pesar.mlx` | Rotina de pesagem do objeto |
| `medir.mlx` | Rotina de medição |
| `GARRA_REAL.mlx` | Controle manual da garra na bancada real |
| `calcular_estado.mlx` | Lê e exibe o estado atual das juntas |
| `calcular_pose.mlx` | Calcula a pose atual da ferramenta |
| `Calcular_rot.mlx` | Calcula a rotação do efetuador |
| `testar_detector.mlx` | Testa o detector YOLO sobre imagens capturadas |
| `teste_azul.mlx` / `teste_vermelho.mlx` | Testes de detecção por cor específica |
| `extrairObjetosDeDeteccoes.mlx` | Extrai regiões de interesse a partir das detecções |

---

## Subpasta `VisionARM4/`

Versão refatorada e mais organizada do pipeline de visão. É a versão de referência do sistema — os arquivos na raiz desta pasta são versões de desenvolvimento intermediárias.

```
VisionARM4/
├── ARMVision.m               # Versão atual do orquestrador
├── realsenseSubscriberSO_ARM.m
├── PcGenRSD435.m
├── denseplane.m
├── colorplane.m              # Detecção de plano por cor (adicional)
├── supPlane.m
├── noiseFilter.m
├── EvalFun.m
├── VMFunctions/
│   ├── VMVision.m            # Módulo de visão alternativo
│   └── PcGenVM.m             # Geração de nuvem para VM
└── Arquivos/
    ├── ARM_2025_Calibration.mat   # Parâmetros de calibração da câmera
    ├── LabelsARM.mat              # Classes dos objetos
    ├── gTruthMerged.mat           # Ground truth de treinamento
    └── gTruthMergedFinal.mat      # Ground truth final consolidado
```

---

## Dependências

- MATLAB com:
  - Computer Vision Toolbox
  - Image Processing Toolbox
  - Robotics System Toolbox (para comunicação ROS)
- Intel RealSense SDK 2.0 (wrapper MATLAB)
- ROS conectado ao controlador do UR5e
- Detector YOLO treinado (`.mat`) — ver pasta [`Detecção com Yolo/`](../Detecção%20com%20Yolo/)
