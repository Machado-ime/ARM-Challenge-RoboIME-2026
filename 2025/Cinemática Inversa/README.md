# Cinemática Inversa

Este módulo contém o modelo rígido do braço **Universal Robots UR5e** e as funções responsáveis por calcular os ângulos das juntas a partir de uma posição 3D desejada no espaço (*cinemática inversa*).

É utilizado tanto na fase real quanto na fase simulada da competição.

---

## O que é cinemática inversa

Dado um ponto no espaço `(x, y, z)` onde se deseja posicionar a ferramenta do braço, a cinemática inversa calcula quais devem ser os ângulos `θ1 ... θ6` das seis juntas do UR5e para que a ferramenta chegue àquela posição.

O MATLAB resolve isso numericamente via o solver `inverseKinematics` do Robotics System Toolbox, usando o modelo de corpo rígido do robô.

---

## Arquivos

### `rigidbodytree_UR5e.m`
Carrega o modelo do UR5e (`loadrobot("universalUR5e")`), aplica modificações nas juntas para restringir o espaço de busca do solver e executa um exemplo de cinemática inversa.

Modificações aplicadas ao modelo padrão:
- `wrist_2_link` fixada (reduz os graus de liberdade para simplificar o problema)
- Limites de posição redefinidos para as juntas `shoulder`, `upper_arm`, `forearm` e `wrist_1`

### `cinematicainversa.m`
Função principal de IK. Recebe a posição desejada, um chute inicial dos ângulos e o modelo do robô; devolve a configuração das juntas que leva a ferramenta até o ponto.

```matlab
config = cinematicainversa(pos, UR5e, chute)
% pos   — vetor [x y z] em metros
% UR5e  — modelo RigidBodyTree
% chute — ângulos iniciais [θ1 θ2 θ3 θ4] para guiar o solver
% config — ângulos resolvidos das juntas
```

A orientação da ferramenta é fixada em `[0 π 0]` (garra apontando para baixo).

### `cinematica_inversa_generica.m`
Versão genérica do solver, sem as restrições específicas do `cinematicainversa.m`. Útil para testes e exploração de configurações alternativas.

### `mover_para.m`
Envia a configuração calculada ao braço físico via ROS. Empacota os ângulos em uma mensagem `trajectory_msgs/JointTrajectoryPoint` e a envia ao action server do controlador.

```matlab
mover_para(ang, trajGoal, trajAct)
% ang      — vetor com os 6 ângulos das juntas (em radianos)
% trajGoal — mensagem ROS de goal de trajetória (pré-criada)
% trajAct  — action client ROS conectado ao controlador
```

O tempo de execução da trajetória é fixo em **5 segundos**.

### `UR5e_alterada_Multibody.slx`
Modelo Simulink Multibody do UR5e com as mesmas alterações de juntas aplicadas no script `rigidbodytree_UR5e.m`. Usado para visualização e validação da cinemática em ambiente de simulação.

---

## Pasta `UR5e/`

Contém arquivos de referência do hardware:

| Arquivo | Descrição |
|---|---|
| `UR5e_User_Manual_ru_Global.pdf` | Manual do usuário do UR5e |
| `eSeries_UR5e.STEP` | Modelo CAD 3D do robô (formato STEP) |

---

## Ordem das juntas

O UR5e tem 6 juntas. A ordem usada pelo controlador ROS difere da ordem padrão do modelo MATLAB, por isso `mover_para.m` faz a reorganização:

```matlab
config = ang([3 2 1 4 5 6]);
% Ordem ROS: elbow → shoulder_lift → shoulder_pan → wrist_1 → wrist_2 → wrist_3
```

---

## Dependências

- MATLAB **Robotics System Toolbox** (`inverseKinematics`, `loadrobot`, `rigidBodyTree`)
- ROS conectado ao controlador do UR5e (apenas para `mover_para.m`)
