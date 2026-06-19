# yahboomcar_bringup — Arranque e drivers do ROSMASTER X3/R2 (ROS2)

Pacote ROS2 originado do **ROSMASTER X3** (Yahboom). Contém os drivers de hardware, launch files de bringup completo e nós utilitários para operação do robô físico e patrulha autónoma.

Este pacote é a **referência ROS2** para a integração de hardware (Arduino ↔ ROS2) — equivalente ao código do robodog1 que comunicava com o Arduino via ROS1.

---

## Estrutura

```
yahboomcar_bringup/
├── launch/                    ← Launch files de bringup completo
├── param/                     ← Parâmetros dos filtros
├── rviz/                      ← Configuração RViz2 para hardware real
└── yahboomcar_bringup/        ← Código Python dos nós
    ├── Mcnamu_driver_X3.py    ← Driver hardware X3 (mecanum)
    ├── Ackman_driver_R2.py    ← Driver hardware R2 (Ackermann)
    ├── Mcnamu_driver_x1.py    ← Driver hardware X1 (variante menor)
    ├── patrol_a1_X3.py        ← Patrulha autónoma X3
    ├── patrol_a1_R2.py        ← Patrulha autónoma R2
    ├── patrol_4ROS.py         ← Patrulha por waypoints X3
    ├── patrol_4ROS_R2.py      ← Patrulha por waypoints R2
    ├── calibrate_linear_X3.py ← Calibração de velocidade linear X3
    ├── calibrate_angular_X3.py← Calibração de velocidade angular X3
    ├── calibrate_linear_R2.py ← Calibração linear R2
    ├── calibrate_angular_R2.py← Calibração angular R2
    └── transform_utils.py     ← Utilitários TF2
```

---

## Launch files

| Arquivo | Descrição |
|---|---|
| [launch/yahboomcar_bringup_X3_launch.py](launch/yahboomcar_bringup_X3_launch.py) | Bringup completo do **X3 (mecanum)**. Lança driver, base_node, IMU filter, EKF e joystick. |
| [launch/yahboomcar_bringup_R2_launch.py](launch/yahboomcar_bringup_R2_launch.py) | Bringup completo do **R2 (Ackermann)**. Mesma estrutura com driver e base_node R2. |
| [launch/yahboomcar_bringup_X1_launch.py](launch/yahboomcar_bringup_X1_launch.py) | Bringup do **X1** (variante menor). |

### Cadeia de nós do bringup X3

```
Hardware Arduino
      │ (USB série / Rosmaster_Lib)
      ▼
Mcnamu_driver_X3 ──── /imu/data_raw ──► imu_filter_madgwick ──► /imu/data ──►┐
      │               /imu/mag                                                 │
      │               /vel_raw ──► base_node_X3 ──► /odom ──────────────────►ekf_node
      │               /joint_states ──► robot_state_publisher (TF)             │
      ▼                                                                         ▼
/cmd_vel ◄─── yahboom_joy_X3                                        /odometry/filtered
      ▲                                                                         │
      └──────────────────────── Nav2 / teleop ─────────────────────────────────┘
```

---

## Drivers de hardware

| Arquivo | Variante | car_type |
|---|---|---|
| [Mcnamu_driver_X3.py](yahboomcar_bringup/Mcnamu_driver_X3.py) | X3 (mecanum 4WD) | 1 |
| [Ackman_driver_R2.py](yahboomcar_bringup/Ackman_driver_R2.py) | R2 (Ackermann) | 5 |
| [Mcnamu_driver_x1.py](yahboomcar_bringup/Mcnamu_driver_x1.py) | X1 (2WD) | — |

### Tópicos ROS2 (X3 e R2)

| Tópico | Direção | Tipo | Descrição |
|---|---|---|---|
| `/cmd_vel` | Subscrito | `Twist` | Comandos de velocidade |
| `/RGBLight` | Subscrito | `Int32` | Efeito de LEDs RGB |
| `/Buzzer` | Subscrito | `Bool` | Buzzer |
| `/imu/data_raw` | Publicado | `Imu` | Dados brutos do IMU (10Hz) |
| `/imu/mag` | Publicado | `MagneticField` | Magnetómetro (10Hz) |
| `/vel_raw` | Publicado | `Twist` | Velocidade lida dos encoders |
| `/joint_states` | Publicado | `JointState` | Estado das rodas (para TF/RViz) |
| `/voltage` | Publicado | `Float32` | Tensão da bateria (V) |
| `/edition` | Publicado | `Float32` | Versão do firmware Arduino |

---

## Parâmetros de configuração IMU

**[param/imu_filter_param.yaml](param/imu_filter_param.yaml)**

```yaml
imu_filter_madgwick:
    fixed_frame: "base_link"
    use_mag: false          # magnetómetro desativado (evita interferências magnéticas)
    publish_tf: false        # TF publicado pelo EKF, não pelo filtro
    world_frame: "enu"       # convenção ENU (East-North-Up)
    orientation_stddev: 0.05 # desvio padrão da orientação estimada
```

---

## Nós de patrulha autónoma

| Arquivo | Descrição |
|---|---|
| [patrol_a1_X3.py](yahboomcar_bringup/patrol_a1_X3.py) | Patrulha reactiva com LaserScan + TF2. Parâmetros: velocidade, tolerância, frame de referência. |
| [patrol_4ROS.py](yahboomcar_bringup/patrol_4ROS.py) | Patrulha por waypoints pré-definidos (sequência de poses). |
| [calibrate_linear_X3.py](yahboomcar_bringup/calibrate_linear_X3.py) | Calibração de fator de escala da velocidade linear. |
| [calibrate_angular_X3.py](yahboomcar_bringup/calibrate_angular_X3.py) | Calibração de fator de escala da velocidade angular. |

---

## Uso no robodog2 (próximos passos)

- [ ] Adaptar `Mcnamu_driver_X3.py` para o hardware real do robodog2 (verificar se usa o mesmo Arduino/Rosmaster_Lib)
- [ ] Criar launch file `robodog2_bringup.launch.py` baseado em `yahboomcar_bringup_X3_launch.py` com URDF do robodog2
- [ ] Verificar se `yahboomcar_base_node` (executável `base_node_X3`) é compatível ou precisa ser adaptado
- [ ] O EKF (`robot_localization`) e o IMU filter (`imu_filter_madgwick`) são pacotes ROS2 padrão — reutilizar diretamente
- [ ] Os nós de patrulha (`patrol_a1_X3.py`) podem ser reutilizados diretamente no robodog2
- [ ] Verificar compatibilidade da `Rosmaster_Lib` com o hardware do robodog2
