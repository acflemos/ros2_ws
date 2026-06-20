# yahboomcar_bringup — Arranque e drivers do ROSMASTER X3/R2/X1 (ROS2 Humble)

Pacote ROS2 originado do **ROSMASTER X3** (Yahboom). Contém os drivers de hardware, launch files de bringup completo, nós de patrulha autónoma e utilitários de calibração para operação do robô físico.

Este pacote é a **referência de hardware** para o robodog2 — equivalente ao código que comunica com o Arduino via biblioteca proprietária `Rosmaster_Lib`.

---

## Estrutura de ficheiros

```
yahboomcar_bringup/
├── launch/
│   ├── yahboomcar_bringup_X3_launch.py  ← Bringup completo X3 (mecanum) — REFERÊNCIA PRINCIPAL
│   ├── yahboomcar_bringup_R2_launch.py  ← Bringup completo R2 (Ackermann)
│   └── yahboomcar_bringup_X1_launch.py  ← Bringup completo X1 (mecanum menor)
├── param/
│   └── imu_filter_param.yaml            ← Parâmetros Madgwick (use_mag=false, publish_tf=false)
├── rviz/
│   └── yahboomcar.rviz                  ← Configuração RViz2 para hardware real
└── yahboomcar_bringup/
    ├── Mcnamu_driver_X3.py              ← Driver hardware X3 (mecanum 4WD) — PEÇA CENTRAL
    ├── Ackman_driver_R2.py              ← Driver hardware R2 (Ackermann)
    ├── Mcnamu_driver_x1.py              ← Driver hardware X1 (variante menor)
    ├── patrol_a1_X3.py                  ← Patrulha autónoma X3 (com JoyState + LaserScan)
    ├── patrol_a1_R2.py                  ← Patrulha autónoma R2 (mesma estrutura, defaults R2)
    ├── patrol_4ROS.py                   ← Patrulha X3 simplificada (sem JoyState)
    ├── patrol_4ROS_R2.py               ← Patrulha R2 com JoyState (versão 4ROS)
    ├── calibrate_linear_X3.py           ← Calibração de escala da odometria linear X3
    ├── calibrate_angular_X3.py          ← Calibração de escala da odometria angular X3
    ├── calibrate_linear_R2.py           ← Calibração linear R2 (tem bug de tipo no parâmetro direction)
    ├── calibrate_angular_R2.py          ← Calibração angular R2 (fator padrão: 0.65)
    └── transform_utils.py               ← Utilitários TF2: quat_to_angle, normalize_angle
```

---

## Launch files

| Arquivo | Modelo | Uso |
|---|---|---|
| `yahboomcar_bringup_X3_launch.py` | X3 (mecanum 4WD) | Hardware real X3 — referência para rbd2_bringup |
| `yahboomcar_bringup_R2_launch.py` | R2 (Ackermann) | Hardware real R2 |
| `yahboomcar_bringup_X1_launch.py` | X1 (mecanum menor) | Hardware real X1 |

### Cadeia de nós do bringup X3

```
[Hardware Arduino] ←→ USB série (Rosmaster_Lib)
         │
         ▼
  Mcnamu_driver_X3 ──┬─► /imu/data_raw  (Imu, 10Hz) ──► imu_filter_madgwick ──► /imu/data ──►┐
                     ├─► /imu/mag        (MagneticField, 10Hz, não usado pelo Madgwick)          │
                     ├─► /vel_raw        (Twist) ──────────► base_node_X3 ──► /odom ────────────►│
                     ├─► /joint_states   (JointState) ──► robot_state_publisher (TF juntas)      │
                     ├─► /voltage        (Float32)                                               │
                     └─► /edition        (Float32)                                               │
                                                                                                  ▼
  /cmd_vel ◄── yahboom_joy_X3 ◄── joystick                               robot_localization EKF
  /JoyState ──► patrol_a1_X3 (pausa quando joystick ativo)                    └─► /odometry/filtered
                                                                                └─► TF: odom → base_footprint

  Nav2 / teleop ──► /cmd_vel
```

### Argumentos dos launch files

| Argumento | Default | Descrição |
|---|---|---|
| `gui` | false | Ativar joint_state_publisher_gui (sliders) |
| `model` | (URDF do pacote description) | Caminho absoluto para o URDF |
| `rvizconfig` | (rviz do pacote description) | Configuração RViz2 (nó comentado por padrão) |
| `pub_odom_tf` | false | Se true, base_node publica TF odom→base_footprint (em vez do EKF) |

---

## Drivers de hardware

| Arquivo | Modelo | car_type | /vel_raw.linear.y | state.position |
|---|---|---|---|---|
| `Mcnamu_driver_X3.py` | X3 mecanum | 1 | vy em m/s (lateral) | NÃO publicado |
| `Ackman_driver_R2.py` | R2 Ackermann | 5 | vy×1000 (millirad) | Publicado com ângulo de esterçamento |
| `Mcnamu_driver_x1.py` | X1 mecanum | 4 | vy×1000 (como R2) | Publicado com ângulo |

### Tópicos ROS2 (comuns a todos os drivers)

| Tópico | Direção | Tipo | Frequência | Descrição |
|---|---|---|---|---|
| `/cmd_vel` | Subscrito | `Twist` | — | Comandos de velocidade (de Nav2, joystick, etc.) |
| `/RGBLight` | Subscrito | `Int32` | — | Efeito de LEDs RGB (0..X) |
| `/Buzzer` | Subscrito | `Bool` | — | Liga/desliga buzzer do Arduino |
| `/imu/data_raw` | Publicado | `Imu` | 10Hz | Acelerómetro + giroscópio (sem orientação) |
| `/imu/mag` | Publicado | `MagneticField` | 10Hz | Magnetómetro (não usado pelo Madgwick) |
| `/vel_raw` | Publicado | `Twist` | 10Hz | Velocidade dos encoders (entrada do base_node) |
| `/joint_states` | Publicado | `JointState` | 10Hz | Estado das juntas (para TF/RViz) |
| `/voltage` | Publicado | `Float32` | 10Hz | Tensão da bateria (V) |
| `/edition` | Publicado | `Float32` | 10Hz | Versão do firmware Arduino |

### Parâmetros dos drivers

| Parâmetro | X3 default | R2 default | Descrição |
|---|---|---|---|
| `car_type` | 'X3' | 'R2' | Tipo de robô |
| `imu_link` | 'imu_link' | 'imu_link' | Frame do IMU |
| `Prefix` | '' | '' | Prefixo para nomes dos joints |
| `xlinear_limit` | 1.0 | 1.0 | Limite velocidade linear X (m/s) |
| `ylinear_limit` | 1.0 | 1.0 | Limite velocidade linear Y (m/s) |
| `angular_limit` | 5.0 | 1.0 | Limite velocidade angular (rad/s) |
| `nav_use_rotvel` | N/A | False | Usar vel. angular de navegação (não implementado) |

---

## Parâmetros IMU (imu_filter_param.yaml)

```yaml
imu_filter_madgwick:
    fixed_frame: "base_link"      # frame base do URDF
    use_mag: false                 # magnetómetro desativado (interferências magnéticas)
    publish_tf: false              # TF publicado pelo EKF, não pelo filtro
    world_frame: "enu"             # convenção ENU (East-North-Up, padrão ROS2)
    orientation_stddev: 0.05       # desvio padrão da orientação estimada (rad)
```

---

## Nós de patrulha autónoma

| Arquivo | Para | Comandos | JoyState | LaserAngle ref. |
|---|---|---|---|---|
| `patrol_a1_X3.py` | X3 | Square (90°), Triangle, Circle, LengthTest | Sim | >(180-LaserAngle)° (frente X3) |
| `patrol_a1_R2.py` | R2 | Square (180°=ida/volta), Circle, LengthTest | Sim | >(180-LaserAngle)° |
| `patrol_4ROS.py` | X3 | Square (180°=ida/volta), Circle, LengthTest | Não | <LaserAngle° |
| `patrol_4ROS_R2.py` | R2 | Square (180°), Circle, LengthTest | Sim | <LaserAngle° |

Todos os nós de patrulha requerem TF `odom → base_footprint` ativo.

---

## Nós de calibração

| Arquivo | Calibra | Parâmetro ajustado | Modelo |
|---|---|---|---|
| `calibrate_linear_X3.py` | Odometria linear | `odom_linear_scale_correction` (default: 1.0) | X3 |
| `calibrate_angular_X3.py` | Odometria angular | `odom_angular_scale_correction` (default: 0.75) | X3 |
| `calibrate_linear_R2.py` | Odometria linear | `odom_linear_scale_correction` (default: 1.0) | R2 |
| `calibrate_angular_R2.py` | Odometria angular | `odom_angular_scale_correction` (default: 0.65) | R2 |

**Procedimento de calibração linear:**
```bash
ros2 run yahboomcar_bringup calibrate_linear_X3
ros2 param set /calibrate_linear start_test true
# observar onde para → ajustar:
ros2 param set /calibrate_linear odom_linear_scale_correction 1.05
```

---

## Bugs conhecidos

1. **Mcnamu_driver_X3 — nomes errados em /joint_states**: publica nomes do R2 (`front_left_steer_joint`, etc.) em vez dos nomes mecanum X3 (`front_left_joint`, etc.). Não afeta o hardware (base_node ignora), mas pode causar avisos no robot_state_publisher se os nomes não baterem com o URDF.

2. **Mcnamu_driver_X3 — sem animação de rodas**: não publica `state.position` → rodas ficam paradas no RViz2. O R2 e X1 têm animação (posição aleatória quando em movimento).

3. **calibrate_linear_R2 — tipo errado no parâmetro `direction`**: declarado como `bool` mas lido como `double_value` no `on_timer`. Causa erro de tipo em runtime no ROS2 Humble. Para o robodog2, usar sempre `calibrate_linear_X3`.

4. **patrol_a1_X3 — Square tem 9 passos em vez de 8**: o quadrado de 4 lados deveria ter 8 passos (4×avança + 4×vira), mas tem 9 (erro de indexação). Funciona na prática mas termina depois de 4,5 lados.

---

## Uso no robodog2

### O que reutilizar diretamente

| Componente | Pacote | Notas |
|---|---|---|
| `imu_filter_madgwick` | `imu_filter_madgwick` | Pacote ROS2 padrão — reutilizar config YAML diretamente |
| `robot_localization` (EKF) | `robot_localization` | Pacote ROS2 padrão — adaptar o YAML de configuração |
| `robot_state_publisher` | ROS2 base | Reutilizar — apenas mudar o URDF |
| Estrutura geral do launch | — | Replicar a cadeia de nós do X3 launch |

### O que adaptar

| Original | Para o robodog2 | Adaptação necessária |
|---|---|---|
| `Mcnamu_driver_X3.py` | `rbd2_driver.py` | Verificar compatibilidade da `Rosmaster_Lib`, corrigir nomes dos joints, adicionar gestão de erros série |
| `base_node_X3` | `rbd2_base_node` | Verificar se os parâmetros cinemáticos (raio das rodas, distância entre eixos) correspondem ao hardware robodog2 |
| `yahboomcar_bringup_X3_launch.py` | `rbd2_bringup_launch.py` | Mudar URDF, nomes dos executáveis, remover yahboom_joy (ou adaptar) |
| `calibrate_linear_X3.py` | reutilizar diretamente | Apenas mudar nome do nó no main() |
| `calibrate_angular_X3.py` | reutilizar diretamente | Ajustar `odom_angular_scale_correction` padrão |

### O que NÃO usar no robodog2

| Componente | Motivo |
|---|---|
| `patrol_a1_X3.py` (em produção) | robodog2 usa Nav2 (mais robusto, com mapa) |
| `patrol_4ROS.py` | idem — Nav2 substitui completamente |
| `Ackman_driver_R2.py` | robodog2 usa X3 mecanum, não Ackermann |
| `Mcnamu_driver_x1.py` | robodog2 usa X3, não X1 |
| `calibrate_linear_R2.py` | tem bug de tipo; usar versão X3 |

### Checklist para criar o rbd2_bringup

- [ ] Confirmar que a `Rosmaster_Lib` funciona na Raspberry Pi (ou Jetson Nano) do robodog2
- [ ] Verificar versão do firmware Arduino e compatibilidade com `set_car_type(1)`
- [ ] Criar `rbd2_driver.py` baseado em `Mcnamu_driver_X3.py`:
  - Corrigir nomes dos joints em `/joint_states` para bater com o URDF do robodog2
  - Adicionar publicação de `state.position` para animação de rodas no RViz2
  - Adicionar logging via `self.get_logger()` em vez de `print()`
- [ ] Criar `rbd2_bringup_launch.py` baseado em `yahboomcar_bringup_X3_launch.py`
- [ ] Copiar `imu_filter_param.yaml` para o rbd2_bringup (sem alterações)
- [ ] Calibrar `odom_linear_scale_correction` e `odom_angular_scale_correction` no hardware real
- [ ] Verificar `ekf_x1_x3_launch.py` em `robot_localization` e adaptar o YAML para os frames do robodog2
- [ ] Testar a cadeia completa: driver → base_node → imu_filter → EKF → Nav2

### Sequência de arranque em hardware real

```bash
# Terminal 1: bringup principal
ros2 launch yahboomcar_bringup yahboomcar_bringup_X3_launch.py

# Verificar tópicos disponíveis:
ros2 topic list | grep -E "imu|vel|odom|joint"

# Verificar TF:
ros2 run tf2_tools view_frames.py

# Calibração (antes de usar Nav2):
ros2 run yahboomcar_bringup calibrate_linear_X3
ros2 param set /calibrate_linear start_test true

ros2 run yahboomcar_bringup calibrate_angular_X3
ros2 param set /calibrate_angular start_test true
```
