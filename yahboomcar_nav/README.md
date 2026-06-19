# yahboomcar_nav

Pacote ROS2 de navegação autónoma e SLAM para o ROSMaster (X1/X3/R2).
Contém launch files para mapeamento (gmapping/Cartographer/RTAB-Map),
navegação (Nav2 com DWB ou TEB) e visualização (RViz2).

---

## Estrutura de ficheiros

```
yahboomcar_nav/
├── launch/
│   ├── laser_bringup_launch.py      ← bringup hardware + LiDAR (entrada para tudo)
│   ├── map_gmapping_launch.py       ← dispatcher gmapping por tipo de LiDAR
│   ├── map_gmapping_a1_launch.py    ← SLAM gmapping com RPLIDAR A1
│   ├── map_gmapping_4ros_s2_launch.py ← SLAM gmapping com YDLIDAR X4 / RPLIDAR S2 (+scan_filter)
│   ├── map_cartographer_launch.py   ← SLAM Cartographer (hardware + LiDAR + Cartographer)
│   ├── cartographer_launch.py       ← apenas o nó Cartographer (sem bringup)
│   ├── occupancy_grid_launch.py     ← helper do Cartographer: renderiza /map
│   ├── map_rtabmap_launch.py        ← SLAM RTAB-Map com câmera RGB-D + LiDAR
│   ├── rtabmap_sync_launch.py       ← nó RTAB-Map SLAM/localização (modo SLAM por padrão)
│   ├── rtabmap_localization_launch.py ← nó RTAB-Map (modo localização por padrão)
│   ├── navigation_dwa_launch.py     ← Nav2 com DWB (planeador local)
│   ├── navigation_teb_launch.py     ← Nav2 com TEB (planeador local)
│   ├── navigation_rtabmap_launch.py ← Nav2 com localização RTAB-Map (sem AMCL)
│   ├── rtabmap_nav_launch.py        ← Nav2 bringup para uso com RTAB-Map
│   ├── save_map_launch.py           ← salva mapa para disco (BUG: caminho hacky)
│   ├── display_map_launch.py        ← RViz2 para visualizar mapeamento
│   ├── display_nav_launch.py        ← RViz2 para visualizar navegação
│   ├── display_rtabmap_map_launch.py ← RViz2 para RTAB-Map (mapeamento)
│   └── display_rtabmap_nav_launch.py ← RViz2 para RTAB-Map (navegação)
├── yahboomcar_nav/
│   ├── __init__.py
│   └── scan_filter.py               ← downsampler /scan → /downsampled_scan (para gmapping)
├── params/
│   ├── dwa_nav_params.yaml          ← Nav2 completo com DWB (BUG: AMCL diferencial)
│   ├── teb_nav_params.yaml          ← Nav2 completo com TEB (mesmos bugs + typo)
│   ├── rtabmap_nav_params.yaml      ← Nav2 sem AMCL (para uso com RTAB-Map)
│   └── lds_2d.lua                   ← configuração Cartographer 2D
├── maps/
│   └── yahboomcar.yaml              ← mapa de exemplo (resolução 0.05m)
└── rviz/
    ├── map.rviz, nav.rviz, rtabmap_map.rviz, rtabmap_nav.rviz
```

---

## Fluxo de trabalho típico

### 1. Mapeamento com Cartographer (recomendado)

```bash
# Terminal 1: hardware + LiDAR + SLAM
export ROBOT_TYPE=x3
export RPLIDAR_TYPE=4ROS
ros2 launch yahboomcar_nav map_cartographer_launch.py

# Terminal 2: visualização
ros2 launch yahboomcar_nav display_map_launch.py

# Terminal 3: salvar mapa quando concluído
ros2 launch yahboomcar_nav save_map_launch.py map_path:=/home/user/ros2_ws/src/yahboomcar_nav/maps/yahboomcar
```

### 2. Navegação com Nav2 (DWB)

```bash
# Terminal 1: hardware + LiDAR (já deve estar ativo, ou iniciar separadamente)
ros2 launch yahboomcar_nav laser_bringup_launch.py

# Terminal 2: Nav2 (AMCL + DWB + NavFn)
ros2 launch yahboomcar_nav navigation_dwa_launch.py

# Terminal 3: RViz2 (para enviar goals e monitorar)
ros2 launch yahboomcar_nav display_nav_launch.py
```

### 3. Mapeamento com RTAB-Map (requer câmera RGB-D)

```bash
ros2 launch yahboomcar_nav map_rtabmap_launch.py
ros2 launch yahboomcar_nav display_rtabmap_map_launch.py
```

---

## Descrição dos ficheiros

### scan_filter.py

Nó que reduz a densidade do LaserScan para compatibilidade com gmapping.

| Parâmetro | Valor | Descrição |
|-----------|-------|-----------|
| `self.multiple` | 2 | Mantém 1 de cada 2 pontos (50% de downsampling) |

- Subscreve: `/scan` — LaserScan original do driver do LiDAR
- Publica: `/downsampled_scan` — LaserScan com metade dos pontos
- Necessário apenas para YDLIDAR X4 e RPLIDAR S2 com gmapping

### dwa_nav_params.yaml — Parâmetros principais

| Parâmetro | Valor | Descrição |
|-----------|-------|-----------|
| `robot_model_type` | `"differential"` | **BUG**: deve ser `"omni"` para X3 mecanum |
| `max_vel_x` | 0.26 m/s | Velocidade linear máxima |
| `max_vel_y` | 0.0 | **BUG**: deve ser > 0 para holonómico |
| `max_vel_theta` | 1.0 rad/s | Velocidade angular máxima |
| `robot_radius` | 0.1 m | Raio do robô (círculo de colisão) |
| `inflation_radius` | 0.1 m | Margem de segurança no costmap |
| `planner` | NavFn (Dijkstra) | Planeador global |
| `controller` | DWB | Planeador local |

### lds_2d.lua — Parâmetros Cartographer

| Parâmetro | Valor | Descrição |
|-----------|-------|-----------|
| `use_imu_data` | false | Sem IMU — odometria é suficiente |
| `use_odometry` | true | Usa /odom para aided odometry |
| `max_range` | 8 m | Alcance máximo do LiDAR |
| `use_online_correlative_scan_matching` | true | Melhor qualidade, mais CPU |
| `min_score` | 0.7 | Threshold conservador para loop closure |

---

## Bugs conhecidos

| Bug | Ficheiro | Impacto |
|-----|---------|---------|
| `robot_model_type: "differential"` em vez de `"omni"` | `dwa_nav_params.yaml`, `teb_nav_params.yaml` | AMCL subestima movimento lateral do X3 — localização degradada com movimento sideways |
| `max_vel_y: 0.0`, `vy_samples: 0`, `acc_lim_y: 0.0` | `dwa_nav_params.yaml`, `teb_nav_params.yaml` | Movimento lateral mecanum completamente desabilitado no DWB |
| `'ontroller_frequency: 20.0'` — typo, falta 'c' | `teb_nav_params.yaml` | Parâmetro ignorado silenciosamente; frequência do controller_server indefinida |
| Caminho `../../../../src/yahboomcar_nav` | `save_map_launch.py` | Só funciona se o workspace estiver em `~/ros2_ws`; usar `map_path:=` explícito |
| TF frame `"laser"` vs URDF `"laser_link"` | `laser_bringup_launch.py` | TF estático publicado para frame errado — possível inconsistência na transformação LiDAR |

---

## Comparação de métodos de SLAM

| Método | Pacote | Requisitos | Qualidade | Uso no robodog2 |
|--------|--------|-----------|-----------|----------------|
| gmapping | `slam_gmapping` | LiDAR 2D | Baixa (obsoleto) | Não recomendado |
| Cartographer | `cartographer_ros` | LiDAR 2D + odom | Alta | Sim — reutilizar `lds_2d.lua` |
| slam_toolbox | `slam_toolbox` | LiDAR 2D | Alta (padrão Nav2) | Sim — preferível ao Cartographer |
| RTAB-Map | `rtabmap_ros` | RGB-D + LiDAR 2D | Muito alta | Se houver câmera de profundidade |

---

## Comparação de planeadores locais Nav2

| Planeador | Plugin | Requisitos | Adequado para X3 |
|-----------|--------|-----------|-----------------|
| DWB | `dwb_core::DWBLocalPlanner` | Nenhum extra | Sim (mas configurado como diferencial — BUG) |
| TEB | `teb_local_planner::TebLocalPlannerROS` | `ros-humble-teb-local-planner` | Sim (mesmo BUG) |

---

## Dependências externas necessárias

| Pacote | Instalação |
|--------|-----------|
| `nav2_bringup` | `sudo apt install ros-humble-nav2-bringup` |
| `cartographer_ros` | `sudo apt install ros-humble-cartographer-ros` |
| `slam_gmapping` | Não disponível diretamente em Humble (compilar da fonte) |
| `slam_toolbox` | `sudo apt install ros-humble-slam-toolbox` (recomendado em vez de gmapping) |
| `rtabmap_ros` | `sudo apt install ros-humble-rtabmap-ros` |
| `teb_local_planner` | `sudo apt install ros-humble-teb-local-planner` |

---

## Relação com o projeto robodog2

### Ficheiros reutilizáveis diretamente:
- `navigation_dwa_launch.py` — template principal de navegação (corrigir bugs de AMCL/velocidade)
- `lds_2d.lua` — configuração Cartographer reutilizável em simulação (passar `use_sim_time:=true`)
- `display_nav_launch.py` — RViz2 de navegação

### Correções obrigatórias antes de usar com robodog2:
1. `dwa_nav_params.yaml`: mudar `robot_model_type: "omni"` e habilitar `max_vel_y`, `vy_samples`
2. `laser_bringup_launch.py`: unificar frame de TF de `"laser"` para `"laser_link"` (consistente com URDF)
3. `save_map_launch.py`: usar caminho absoluto em vez do hack `../../../../src/`

### O que criar de novo para robodog2:
- Launch de navegação que integra Gazebo + Nav2 (substituindo `laser_bringup_launch.py`)
- Configuração de `slam_toolbox` (mais moderna que gmapping e sem limite de pontos)
