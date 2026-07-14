# CLAUDE.md — ros2_ws/src

Contexto do workspace para o Claude Code. Lido automaticamente em qualquer máquina onde este repositório for clonado (PC de desenvolvimento, Raspberry Pi do robô físico, ou máquina de outros colaboradores do projeto).

---

## O que é este projeto

Workspace ROS2 Humble de estudo e desenvolvimento para o robô **ROSMASTER X3** da Yahboom, com foco em:

- Simulação no **Gazebo Fortress** (o fabricante não oferece isso de fábrica)
- Navegação autônoma (Nav2 + SLAM) com comportamento inspirado em cão de vigilância
- Documentação em **português**, como base para um futuro curso de ROS2 para desenvolvedores lusófonos

O pacote autoral e âncora do projeto é o **robodog2**. Os pacotes `yahboomcar_*` são o código original da Yahboom, mantidos aqui como referência de hardware real.

---

## Estrutura de repositórios git (IMPORTANTE)

Este workspace usa **dois repositórios git separados**:

| Pasta | Repositório | Conteúdo |
|---|---|---|
| `src/` (aqui) | `github.com/acflemos/ros2_ws` | Pacotes `yahboomcar_*` (referência Yahboom) + utilitários |
| `src/robodog2/` | `github.com/acflemos/robodog2` | Pacote autoral — **git próprio**, gitignorado deste repo |

`robodog2/` está listado no `.gitignore` deste repo de propósito — não é submodule, é um clone independente dentro da mesma pasta `src/`.

### Como clonar (evita confusão de nomes)

O conteúdo deste repositório É a pasta `src/` de um workspace ROS2 (sem `build/`, `install/`, `log/`). Um `git clone` simples cria uma pasta chamada `ros2_ws/`, que não é a estrutura esperada. Clone sempre assim:

```bash
mkdir -p ~/ros2_ws && cd ~/ros2_ws
git clone https://github.com/acflemos/ros2_ws.git src
git clone https://github.com/acflemos/robodog2.git src/robodog2
```

---

## Regras de trabalho (críticas — não violar)

1. **Nunca modificar `robodog2/` a partir de uma sessão aberta em `src/`.** Alterações no robodog2 exigem uma sessão do Claude Code aberta diretamente na pasta `robodog2/` (que tem seu próprio `CLAUDE.md` com arquitetura detalhada), com branch adequada criada antes de qualquer mudança.

2. **Nunca editar arquivos-fonte dentro de pacotes `yahboomcar_*`** (ex: `Mcnamu_driver_X3.py`, `base_node_X3.cpp`) — mesmo para corrigir bugs reais encontrados no hardware físico. Motivo: o usuário quer poder comparar o código original do fabricante com as modificações feitas, para fins didáticos. Ajustes de comportamento (parâmetros, frame_id, tópicos) devem ser feitos via **argumentos de launch** sempre que possível. Se for necessário mudar lógica/código-fonte de verdade, criar um pacote novo com prefixo `robodog2_` (ex: `robodog2_bringup`), copiar apenas o(s) arquivo(s) afetado(s) para lá, e apontar os launch files para o pacote novo.

3. **`~/codigo_referencia/` (pasta irmã, fora deste workspace) é somente leitura.** Permitido adicionar comentários e READMEs; nunca modificar código existente.

4. Novos pacotes autorais derivados de código Rosmaster/NVIDIA usam sempre o prefixo `robodog2_`.

5. **Fluxo git deste repositório (`ros2_ws`)**:
   - Documentação pura (README, este `CLAUDE.md`, comentários) → pode commitar direto em `main`
   - Qualquer mudança de código real (pacotes novos, launch files, scripts) → sempre branch + PR, nunca direto em `main` — mesma regra do `robodog2`

---

## Ambiente físico do robô

- Hardware: **ROSMASTER X3** (Yahboom), rodas mecanum
- Computador de bordo: **Raspberry Pi 4B, 8 GB RAM**
- ROS2 Humble rodando **dentro de um container Docker** na Pi
- LiDAR: **RPLIDAR A1** — driver `sllidar_ros2`, porta `/dev/ttyUSB0`, publica `/scan`
- IMU: MPU6050 — fusão Madgwick + EKF (`robot_localization`)
- Driver do Arduino: `Mcnamu_driver_X3` (pacote `yahboomcar_bringup`) via `Rosmaster_Lib`

---

## Estado atual (2026-07-14)

- Stack de bringup para hardware real foi criado no `robodog2` (PR #22, branch `bringup_robo_real`, já mergeada em `main`):
  - `rbd_robo_hardware_launch.py` — roda **no robô** (container): `yahboomcar_bringup_X3_launch.py` (driver Arduino, base_node, IMU, EKF, robot_state_publisher) + `sllidar_launch.py` (RPLIDAR A1)
  - `rbd_bringup.launch.py` — Nav2 (AMCL+DWB+BT Navigator) + RViz2 opcional + `rbd_navega` opcional. Pode rodar no robô ou remotamente no PC via rede ROS2 (mesmo `ROS_DOMAIN_ID`)
  - `params/rbd_dwa_nav_params_real.yaml` — Nav2 com `use_sim_time: false`
  - Ver `robodog2/CLAUDE.md` para arquitetura completa e estratégia de pose inicial

### 🐛 Bug conhecido a corrigir antes do teste físico

Em `robodog2/launch/rbd_robo_hardware_launch.py`, o include do `sllidar_launch.py` **não sobrescreve o parâmetro `frame_id`**. O `sllidar_ros2` publica `/scan` por padrão com `frame_id="laser"`, mas o URDF real (`yahboomcar_description/urdf/yahboomcar_X3.urdf`) usa `laser_link`. Sem a correção, o TF do LiDAR não bate com o resto da árvore (RViz não mostra o `/scan`, Nav2 não monta o costmap a partir do laser). Corrigir assim:

```python
rplidar = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory('sllidar_ros2'), 'launch', 'sllidar_launch.py')
    ),
    launch_arguments={
        'frame_id': 'laser_link',
    }.items()
)
```

### Próximo passo imediato

Depurar `rbd2_robo_hardware` no robô físico (dentro do container): aplicar a correção acima, confirmar que `/scan` e `/odom` são publicados e que a árvore TF está consistente (sem erros de frame ausente) antes de avançar para Nav2.

---

## Fluxo de trabalho esperado no robô físico

Claude Code deve rodar **no host da Raspberry Pi** (fora do container), usando `docker exec` para comandos ROS2 dentro dele — ver `robodog2/CLAUDE.md` para o fluxo completo (`colcon build`, aliases `rbd2_*`, ordem dos terminais).

---

## Referências

- `README.md` (este diretório) — descrição detalhada de todos os pacotes `yahboomcar_*`, dependências entre eles e convenções
- `robodog2/CLAUDE.md` — arquitetura completa do pacote principal, launch files, convenções de código, histórico de PRs e estado do projeto
