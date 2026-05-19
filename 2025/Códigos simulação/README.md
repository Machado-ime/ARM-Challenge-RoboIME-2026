# Códigos de Simulação

Código submetido nas rodadas da **fase simulada** do ARM Challenge (RoboCup). O braço opera em ambiente virtual **Gazebo** e os comandos são enviados via ROS, com a mesma interface de mensagens usada no hardware real.

---

## Subpastas

### `RoboCup Arm_Challenge RoboIME/` — Submissão principal

Código da submissão oficial da equipe RoboIME para a fase simulada.

| Arquivo | Descrição |
|---|---|
| `packTrajGoal.m` | Empacota uma configuração de juntas em uma mensagem ROS `FollowJointTrajectoryGoal` pronta para envio |
| `packGripGoal.m` | Empacota um comando de abertura/fechamento de garra em mensagem ROS |
| `abrir_garra.m` | Abre ou fecha a garra com o valor de abertura especificado (0 = aberto, 1 = fechado) |
| `ajuste.m` | Realiza ajuste fino de posição após o movimento principal |
| `rotelem.m` | Rotaciona um elemento — usado para reposicionar objetos em orientações específicas |
| `garra_automatica.mlx` | Script de controle automático da garra durante a execução da tarefa |
| `garra_manual.mlx` | Interface para controle manual da garra durante testes |
| `área cinza.mp4` | Vídeo de demonstração da execução na área cinza da arena simulada |

---

### `RoboIME Second Submission - cópia/` — Segunda submissão (backup)

Cópia dos arquivos utilizados na segunda submissão da equipe. Os arquivos têm o prefixo "Cópia de" pois foram preservados manualmente como backup antes de ajustes.

Contém as mesmas funções da submissão principal, acrescido de:

| Arquivo | Descrição |
|---|---|
| `Cópia de Obstaculos.m` | Lógica de desvio de obstáculos na arena |
| `Cópia de lixo.m` | Script auxiliar de testes (rascunho) |

---

## Como as mensagens ROS funcionam

O controle do braço na simulação segue o mesmo protocolo da fase real. O fluxo é:

```
cinematicainversa(pos)           → ângulos das 6 juntas
        │
        ▼
packTrajGoal(config, trajGoal)   → monta a mensagem ROS de trajetória
        │
        ▼
sendGoal(trajAct, trajGoal)      → envia ao action server do controlador
```

Para a garra:
```
packGripGoal(abertura, gripGoal) → monta a mensagem de garra
        │
        ▼
abrir_garra(abertura, ...)       → envia o comando
```

---

## Diferenças em relação à fase real

| Aspecto | Simulação | Real |
|---|---|---|
| Ambiente | Gazebo | Bancada física |
| Câmera | Simulada no Gazebo | RealSense D415 |
| Detecção | Posição fornecida pelo simulador ou YOLO no Gazebo | ARMVision + YOLO + nuvem de pontos |
| Comunicação | ROS (mesmo protocolo) | ROS (mesmo protocolo) |
| Garra | Simulada | Garra pneumática real |

---

## Dependências

- MATLAB com Robotics System Toolbox
- ROS conectado ao ambiente Gazebo com o UR5e simulado
- Funções de cinemática inversa em [`../Cinemática Inversa/`](../Cinemática%20Inversa/)
