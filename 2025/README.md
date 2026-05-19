# ARM Challenge 2025 — RoboIME

Este repositório contém o código desenvolvido pela equipe **RoboIME** para o **ARM Challenge**, competição da [RoboCup](https://www.robocup.org/) voltada para manipulação autônoma com braços robóticos.

---

## Sobre a competição

O ARM Challenge é dividido em duas fases:

| Fase | Descrição |
|---|---|
| **Simulação** | O braço robótico opera em ambiente virtual (Gazebo). A equipe submete código que detecta e manipula objetos no simulador. |
| **Real** | O braço físico **Universal Robots UR5e** opera em bancada real, utilizando câmera **Intel RealSense D415** para percepção do ambiente. |

Em ambas as fases o objetivo é o mesmo: detectar objetos sobre uma superfície, calcular sua posição 3D e comandar o braço para pegá-los e reposicioná-los de forma autônoma.

O hardware utilizado na fase real:
- **Braço:** Universal Robots UR5e (6 graus de liberdade)
- **Câmera:** Intel RealSense D415 (cor + profundidade, 640×480 @ 15 fps)
- **Comunicação:** ROS (*Robot Operating System*)
- **Linguagem:** MATLAB

---

## Estrutura do repositório

- [`Cinemática Inversa`](Cinemática%20Inversa) — Modelo rígido do UR5e e funções que calculam os ângulos das 6 juntas a partir de uma posição 3D desejada. Utilizado tanto na fase real quanto na simulada.
- [`Código real`](Código%20real) — Pipeline completo para a fase real: leitura da câmera RealSense, geração de nuvem de pontos, detecção YOLO, localização 3D dos objetos e envio de comandos ao braço via ROS. A versão mais recente está na subpasta `VisionARM4/`.
- [`Códigos simulação`](Códigos%20simulação) — Código submetido nas rodadas da fase simulada do RoboCup. Contém as funções de empacotamento de mensagens ROS, controle de garra e movimentação do braço no ambiente Gazebo.
- [`Detecção com Yolo`](Detecção%20com%20Yolo) — Notebooks e scripts para treinamento, validação e uso do detector YOLO 2D. Inclui experimentos com Gazebo e testes com imagens reais da câmera RealSense.
- [`detector por Nuvens de Pontos`](detector%20por%20Nuvens%20de%20Pontos) — Primeira abordagem tentada: detectar objetos diretamente na nuvem de pontos 3D com PointPillars. Abandonada — arquivos mantidos para referência futura.

---

## Fluxo geral do sistema (fase real)

```
Câmera RealSense D415
        │
        ▼
  ARMVision.m  ──────────────────────────────────────────┐
  (Código real/)                                           │
        │                                                 │
   ┌────┴────┐                                            │
   │         │                                            │
   ▼         ▼                                            │
YOLO 2D   Nuvem de Pontos                                 │
(detecção) (profundidade)                                 │
   │         │                                            │
   └────┬────┘                                            │
        │  localização 3D do objeto                       │
        ▼                                                 │
Cinemática Inversa (cinematicainversa.m)  ◄───────────────┘
        │  ângulos das 6 juntas
        ▼
   mover_para.m  →  ROS  →  UR5e
```

## Dependências

- MATLAB com os toolboxes:
  - Robotics System Toolbox
  - Computer Vision Toolbox
  - Image Processing Toolbox
- Intel RealSense SDK 2.0 (wrapper MATLAB)
- ROS (conectado ao controlador do UR5e)
