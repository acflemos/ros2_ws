# yahboomcar_laser

Pacote ROS2 de aplicações autónomas baseadas em LiDAR para o ROSMaster (X3/R2).
Contém três modos de operação independentes:
- **Avoidance** — desvio reactivo de obstáculos (10 casos baseados em setores)
- **Tracker** — seguimento de objeto mais próximo com controlo PID
- **Warning** — aviso sonoro + rotação para enfrentar obstáculos

---

## Estrutura de ficheiros

```
yahboomcar_laser/
├── launch/
│   ├── laser_Avoidance_a1_X3.launch.py  ← bringup + A1 + desvio X3
│   ├── laser_Tracker_a1_X3.launch.py    ← bringup + A1 + seguimento X3
│   └── laser_Warning_a1_X3.launch.py    ← bringup + A1 + aviso X3
└── yahboomcar_laser/
    ├── common.py                         ← SinglePID + importações partilhadas
    ├── laser_Avoidance_a1_X3.py         ← desvio X3 com RPLIDAR A1 (BUG: self.Moving)
    ├── laser_Avoidance_4ROS.py          ← desvio X3 com YDLIDAR X4 (sem entry point)
    ├── laser_Avoidance_a1_R2.py         ← desvio R2 com RPLIDAR A1
    ├── laser_Avoidance_4ROS_R2.py       ← desvio R2 com YDLIDAR X4
    ├── laser_Tracker_a1_X3.py           ← seguimento X3 com RPLIDAR A1
    ├── laser_Tracker_4ROS.py            ← seguimento X3 com YDLIDAR X4 (BUG: self.laserAngle)
    ├── laser_Tracker_a1_R2.py           ← seguimento R2 com RPLIDAR A1
    ├── laser_Tracker_4ROS_R2.py         ← seguimento R2 com YDLIDAR X4 (versão corrigida)
    └── laser_Warning_a1_X3.py           ← aviso X3 com RPLIDAR A1
```

---

## Convenção de ângulos (CRÍTICO)

O ângulo zero do LiDAR aponta para onde o sensor está apontado fisicamente.
O modo de montagem determina qual setor corresponde à frente do robô:

| LiDAR | Montagem no X3 | Frente do robô | Flancos |
|-------|---------------|----------------|---------|
| RPLIDAR A1 | Para trás (yaw=π no TF) | `abs(angle) > 160°` | ~140-160° |
| YDLIDAR X4 | Para a frente | `abs(angle) < 10°` | 10-50° |

Esta convenção explica por que `a1_X3` e `4ROS` têm código de zonas completamente diferentes,
apesar de implementarem a mesma lógica de desvio.

---

## Como usar

```bash
# Desvio de obstáculos com RPLIDAR A1 (X3)
ros2 launch yahboomcar_laser laser_Avoidance_a1_X3.launch.py

# Ativar/desativar desvio em runtime
ros2 param set /laser_Avoidance_a1 Switch false  # false = desvio ativo
ros2 param set /laser_Avoidance_a1 ResponseDist 0.4  # distância de reação

# Seguimento de objeto (manter 0.55m de distância)
ros2 launch yahboomcar_laser laser_Tracker_a1_X3.launch.py
ros2 param set /laser_Tracker_a1 Switch false  # false = seguimento ativo

# Aviso sonoro quando obstáculo < 0.55m
ros2 launch yahboomcar_laser laser_Warning_a1_X3.launch.py
```

---

## Parâmetros comuns (todos os nós)

| Parâmetro | Padrão | Descrição |
|-----------|--------|-----------|
| `Switch` | `False` | `False` = operação ativa; `True` = parado (cede ao joystick) |
| `linear` | 0.5 m/s | Velocidade linear de movimento |
| `angular` | 1.0 rad/s | Velocidade angular de rotação |
| `LaserAngle` | 40° | Largura de cada setor lateral monitorado |
| `ResponseDist` | 0.55m | Distância de reação (desvio/seguimento/aviso) |

---

## Modos de operação

### Avoidance (desvio)
10 casos baseados em combinações de `front_warning > 10`, `Left_warning > 10`, `Right_warning > 10`:
- Cada warning conta pontos LiDAR a menos de `ResponseDist × 1.5m` no setor correspondente
- Threshold: 10 pontos por setor
- Usa `sleep()` após cada decisão (bloqueante — pode causar atraso na reação)

### Tracker (seguimento)
- Zona de prioridade (`priorityAngle=30°`): objetos na frente têm prioridade
- `lin_pid` (P=2, D=2): regula velocidade linear para manter `ResponseDist`
- `ang_pid` (P=3, D=5): regula angular para alinhar com o objeto
- Deadzone: `ang_pid < 0.02 → angular.z = 0`

### Warning (aviso)
- Encontra o objeto mais próximo no setor frontal/lateral
- Se `minDist <= ResponseDist` → `/Buzzer = True`
- Gira para alinhar com o objeto (mesmo PID do Tracker)
- Não avança nem recua (`linear.x = 0` sempre)

---

## Bugs conhecidos

| Bug | Ficheiro | Impacto |
|-----|---------|---------|
| `self.Moving` não inicializado em `__init__` | `laser_Avoidance_a1_X3.py` | `AttributeError` se `/JoyState` chegar antes de `/scan` |
| `self.laserAngle` (minúsculo) em vez de `self.LaserAngle` | `laser_Tracker_4ROS.py` | `AttributeError` no primeiro LaserScan recebido — nó crasha |
| `laser_Avoidance_4ROS.py` e `laser_Tracker_4ROS.py` não registados em `setup.py` | `setup.py` | Não podem ser executados via `ros2 run` |
| Todos os launch files hardcoded para RPLIDAR A1 (`sllidar_launch.py`) | `launch/*.launch.py` | Para usar YDLIDAR X4, trocar para `ydlidar_ros2_driver` manualmente |
| Uso de `sleep()` no callback de LaserScan (bloqueante) | `laser_Avoidance_*.py` | Pode causar atrasos e perda de mensagens durante o sono |

---

## Relação com o projeto robodog2

### Relevância:
- **Avoidance** — útil como comportamento reativo de segurança (parar/desviar antes do Nav2)
- **Tracker** — demonstração de como usar LaserScan para seguimento; reutilizável como node autónomo
- **Warning** — pode ser integrado como safety monitor no robodog2

### Em simulação (Gazebo):
- Os nós podem ser testados diretamente em simulação — subscreve `/scan` que o plugin Gazebo publica
- Nenhuma modificação necessária exceto verificar a convenção de ângulos (depende da montagem do LiDAR no URDF)
- O URDF do X3 usa `yaw=π` para o LiDAR → usar variantes `a1` (abs(angle) > 160° para frente)

### O que corrigir antes de usar com robodog2:
1. `laser_Avoidance_a1_X3.py`: adicionar `self.Moving = False` em `__init__`
2. `laser_Tracker_4ROS.py`: corrigir `self.laserAngle` → `self.LaserAngle`
3. Substituir `sleep()` bloqueante por um timer ou lógica baseada em eventos
