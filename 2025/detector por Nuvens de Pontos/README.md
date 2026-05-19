# Detector por Nuvens de Pontos

> **Esta abordagem foi abandonada.** Os arquivos estão preservados apenas para referência histórica, caso a equipe queira retomar a linha de pesquisa no futuro.

---

## O que foi tentado

A ideia era detectar objetos diretamente na nuvem de pontos 3D, sem depender de uma câmera colorida ou de detecção 2D. O algoritmo escolhido foi o **PointPillars**, uma rede neural para detecção de objetos em nuvens de pontos espaças, originalmente desenvolvida para LiDAR automotivo.

O processo planejado era:

```
Nuvem de pontos (RealSense)
        │
        ▼
Rotulagem manual (lidarLabelingSession)
        │
        ▼
Treinamento PointPillars (treinar_detector_pt.m)
        │
        ▼
Inferência (detector_pretreinado.m)
```

---

## Por que foi abandonado

A abordagem não chegou a funcionar de forma satisfatória. Os principais problemas identificados foram:

- A nuvem de pontos da RealSense D415 é **muito ruidosa a curta distância**, dificultando a separação entre objetos próximos e a superfície.
- O PointPillars foi originado para cenários LiDAR com **objetos grandes e bem separados** (carros, pedestres) — em uma bancada com objetos pequenos como latas e caixas, a adaptação mostrou-se problemática.
- O esforço de rotulagem manual de nuvens de pontos é alto e os dados coletados foram insuficientes para treinar um detector confiável.

A solução adotada foi uma abordagem **híbrida**: detecção 2D com YOLO na imagem colorida, combinada com alinhamento da nuvem de pontos para obter a posição 3D — implementada em [`../Código real/`](../Código%20real/).

---

## Arquivos

| Arquivo | Descrição |
|---|---|
| `treinar_detector_pt.m` | Script de treinamento do PointPillars. Carrega o ground truth, configura ancoragens e opções de treino, e salva o modelo resultante. |
| `detector_pretreinado.m` | Carrega e testa um detector PointPillars pré-treinado sobre nuvens de pontos capturadas. |
| `baixar_detector_pré-treinado.m` | Script para download de um modelo PointPillars pré-treinado disponibilizado pelo MATLAB como ponto de partida. |
| `coletor_2.m` | Captura nuvens de pontos da câmera e as salva para rotulagem e treinamento. |
| `lidarLabelingSession.mat` | Sessão de rotulagem manual de nuvens de pontos feita com o **Lidar Labeler** do MATLAB. |
| `teste_pt_rotulada.mat` | Nuvem de pontos com anotações de bounding boxes 3D, usada para avaliar o detector. |
| `coletor_2.asv` | Backup automático do MATLAB Editor (pode ser ignorado). |
| `treinar_detector_pt.asv` | Backup automático do MATLAB Editor (pode ser ignorado). |

---

## Dependências (para referência)

- MATLAB **Lidar Toolbox** (`pointPillarsObjectDetector`, `trainPointPillarsObjectDetector`)
- **Deep Learning Toolbox**
- GPU recomendada para treinamento
