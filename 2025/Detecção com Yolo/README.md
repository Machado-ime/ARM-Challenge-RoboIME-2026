# Detecção com YOLO

Notebooks e scripts para treinamento, validação e uso do detector **YOLO 2D** aplicado à detecção de objetos na arena do ARM Challenge.

O detector treinado nesta pasta é consumido pelo pipeline de visão em [`../Código real/`](../Código%20real/).

---

## Arquivos

### `YOLO.mlx`
Notebook principal de treinamento do detector YOLO no MATLAB. Cobre o processo completo: carregamento do dataset, configuração da rede, treinamento e avaliação do modelo resultante.

### `ObjectDetectionwithDeepLearning.mlx`
Estudo de detecção de objetos com deep learning usando o Computer Vision Toolbox do MATLAB. Serve como base conceitual e experimentação com diferentes arquiteturas antes de consolidar o uso do YOLO.

### `Computer_Vision.mlx`
Experimentos gerais de visão computacional aplicados ao problema da competição: pré-processamento de imagens, augmentation de dados e análise de resultados de detecção.

### `Gazebo_YOLO_usage.mlx`
Demonstra o uso do detector YOLO integrado ao ambiente Gazebo, para validação do detector antes de partir para o hardware real. Conecta o YOLO com o stream de imagens do simulador via ROS.

---

## Subpasta `Detector Real 1/`

Testes de validação do detector com imagens capturadas diretamente da câmera RealSense em bancada real.

| Arquivo | Descrição |
|---|---|
| `PointCloud_Real_Troubleshooting.mlx` | Diagnóstico de problemas na integração entre o detector YOLO e a nuvem de pontos em cenas reais |
| `18.png` / `18_d.png` | Par: imagem original + imagem com detecções sobrepostas |
| `19.png` / `19_d.png` | Idem |
| `20.png` / `20_d.png` | Idem |
| `21.jpg` / `21_d.jpg` | Idem |

Os arquivos com sufixo `_d` são as imagens com as *bounding boxes* desenhadas, usadas para inspeção visual da qualidade das detecções.

---

## Como o detector é usado no sistema

O detector treinado é salvo como arquivo `.mat` e carregado no pipeline principal:

```matlab
% No pipeline de visão (ARMVision.m):
[bboxes, scores, labels] = detect(Detector, colorImage);

% Apenas detecções com confiança >= 0.6 são aceitas:
idxValidos = scores >= 0.6;
```

O YOLO fornece as coordenadas 2D (bounding boxes) dos objetos na imagem. Essas coordenadas são então combinadas com a nuvem de pontos 3D para obter a posição real dos objetos no espaço — essa fusão é feita pela função `EvalFun.m` no `Código real/`.

---

## Dependências

- MATLAB **Computer Vision Toolbox** (treinamento e inferência YOLO)
- **Deep Learning Toolbox**
- ROS (para `Gazebo_YOLO_usage.mlx`)
- Dataset de imagens rotuladas (ground truth em [`../Código real/VisionARM4/Arquivos/`](../Código%20real/VisionARM4/Arquivos/))
