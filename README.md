# ros2_ws/src — Workspace de Desenvolvimento robodog2

Este README é a **memória compartilhada** do projeto: explica o propósito de cada pasta, as regras de trabalho, as convenções adotadas e o estado atual. Destina-se ao autor, ao assistente de IA (Claude) e a futuros leitores do projeto.

---

## ⚠️ Como clonar este repositório

O nome do repositório no GitHub é `ros2_ws`, mas o seu **conteúdo é a pasta `src/` de um workspace ROS2** — o repo não inclui `build/`, `install/` nem `log/` (ver `.gitignore`). Um `git clone` simples cria uma pasta chamada `ros2_ws/`, o que **não** é a estrutura esperada por um workspace ROS2 (que precisa de `ros2_ws/src/`).

Para clonar corretamente, informe o nome da pasta de destino como `src`:

```bash
mkdir -p ~/ros2_ws && cd ~/ros2_ws
git clone https://github.com/acflemos/ros2_ws.git src
git clone https://github.com/acflemos/robodog2.git src/robodog2
```

Isso evita ter que mover manualmente os pacotes depois do clone.

---

## Visão geral

Este workspace é o ambiente de estudo e desenvolvimento para o robô **ROSMASTER X3** da Yahboom, com foco em:

- **Simulação no Gazebo Fortress** (algo que o próprio fabricante não fornece)
- **Navegação autônoma** com Nav2, SLAM e comportamento inspirado em cão de vigilância
- **Documentação em português**, como base para um futuro projeto educativo de ROS2 para desenvolvedores lusófonos

O pacote principal e âncora do projeto é o **robodog2**, já publicado no GitHub. Os pacotes `yahboomcar_*` são o código original da Yahboom, estudados e documentados aqui como base para as futuras evoluções do robodog2 no hardware real.

---

## Estrutura de pastas

```
ros2_ws/src/
│
├── robodog2/                    # ★ PACOTE ÂNCORA — estável, publicado no GitHub
│                                #   ⚠️  NÃO modificar a partir deste contexto (src/)
│                                #   Modificações: abrir VSCode direto em robodog2/
│                                #                 criar branch adequada antes de alterar
│
├── yahboomcar_bringup/          # ✅ Estudado — Driver hardware + bringup completo
├── yahboomcar_description/      # URDF/geometria do X3 e R2 (meshes, xacro)
├── yahboomcar_description_x1/   # URDF/geometria do X1 (variante menor)
├── yahboomcar_laser/            # ✅ Estudado — Comportamentos autônomos LiDAR
├── yahboomcar_nav/              # SLAM e Nav2 para hardware real
├── yahboomcar_msgs/             # Mensagens ROS2 customizadas do fabricante
├── yahboomcar_ctrl/             # Teleoperação por joystick e teclado
├── yahboomcar_base_node/        # Nó C++ de odometria (encoder → /odom)
├── yahboomcar_astra/            # Visão por câmera Astra RGB-D (seguimento de cor)
├── yahboomcar_visual/           # Visão geral: QR, AR, câmera, laser→imagem
├── yahboomcar_linefollow/       # Seguimento de linha colorida + LiDAR
├── yahboomcar_mediapipe/        # Poses humanas / mãos / rostos com MediaPipe
├── yahboomcar_voice_ctrl/       # Controle por voz dos comportamentos principais
├── laserscan_to_point_pulisher/ # Converte LaserScan → PointCloud2
├── robot_pose_publisher/        # Publica pose do robô lida do TF (Python)
├── yahboom_app_save_map/        # Salva mapa via app mobile Yahboom
├── yahboom_web_savmap_interfaces/ # Interfaces ROS2 (srv) para salvar mapa via web
│
├── bkp_codigo_problematico/     # ⛔ Pacotes que não compilam (COLCON_IGNORE)
│   ├── yahboomcar_KCFTracker/   #    Rastreador KCF C++ — OpenCV4 incompatível
│   ├── yahboomcar_slam/         #    SLAM 3D com PCL — pcl_ros não instalado
│   ├── yahboomcar_point/        #    Nuvem de pontos PCL — pcl_conversions ausente
│   └── robot_pose_publisher_ros2/ #  Publica pose TF — API tf2_ros incompatível
│
└── robodog2_*/                  # NOVOS PACOTES AUTORAIS (a criar)
                                 # Prefixo robodog2_ para separar do código Yahboom
```

### Pasta irmã: `~/codigo_referencia/`

```
codigo_referencia/
├── Rosmaster-x3/                # Código-fonte completo do fabricante Yahboom
│   └── src/                     # Comentado em português (maio/2026)
├── robodog1/                    # Versão ROS1 Noetic — CONGELADA, apenas referência
└── ...zips de backup
```

> **Regra:** `codigo_referencia/` é **somente leitura**. Permitido apenas adicionar comentários e criar READMEs. Nunca modificar código existente.

---

## Descrição detalhada dos pacotes

### Pacotes de hardware e inicialização

| Pacote | O que faz | Relação com robodog2 |
|---|---|---|
| `yahboomcar_bringup` ✅ | Driver do Arduino (Rosmaster_Lib → /cmd_vel, /imu, /odom). Launch file que inicia toda a cadeia de nós do X3 real. Inclui calibração e patrulha simples. | **Base do rbd2_bringup**: o `Mcnamu_driver_X3.py` é o equivalente hardware do plugin Gazebo. O launch file é o modelo para `rbd2_bringup_launch.py`. |
| `yahboomcar_base_node` | Nó C++ que lê `/vel_raw` (encoders) e calcula `/odom` com integração de velocidade. | Equivalente ao `OdometryPublisher` do robodog2. Pode ser reutilizado diretamente no hardware real. |
| `yahboomcar_description` | URDF/Xacro do X3 e R2 com meshes STL reais (para RViz2 no hardware). | O robodog2 tem seu próprio URDF de simulação. O URDF real do X3 pode complementar. |
| `yahboomcar_description_x1` | URDF/Xacro do X1 (variante menor do X3). | Referência — o robodog2 usa o X3. |

### Cadeia completa do bringup X3 (hardware real)

```
Hardware Arduino
      │ (USB série / Rosmaster_Lib)
      ▼
Mcnamu_driver_X3 ──► /imu/data_raw ──► imu_filter_madgwick ──► /imu/data ──►┐
      │              /vel_raw ──► base_node_X3 ──► /odom ───────────────────►ekf_node
      │              /joint_states ──► robot_state_publisher (TF)             │
      ▼                                                                        ▼
/cmd_vel ◄── yahboom_joy_X3 / Nav2 / rbd_navega                  /odometry/filtered
```

### Pacotes de navegação e LiDAR

| Pacote | O que faz | Relação com robodog2 |
|---|---|---|
| `yahboomcar_laser` ✅ | Três modos autônomos por LiDAR: Avoidance (10 casos), Tracker (PID), Warning (buzzer). Convenção de ângulos A1 vs YDLIDAR documentada. | Avoidance pode ser safety monitor do robodog2. Tracker demonstra uso do LaserScan para seguimento. |
| `yahboomcar_nav` | Configurações Nav2 + SLAM (slam_toolbox) para hardware real. Params de DWB, costmap, AMCL. | Referência para comparar com os params do robodog2 (`rbd_dwa_nav_params.yaml`). |

### Pacotes de teleoperação

| Pacote | O que faz | Relação com robodog2 |
|---|---|---|
| `yahboomcar_ctrl` | Joystick físico Yahboom (detecta Jetson via user=root) e teclado. Publica `/cmd_vel` e `/JoyState`. | `/JoyState` é usado pelos nós de patrulha para ceder controle ao operador. Útil no rbd2_bringup. |

### Pacotes de mensagens customizadas

| Pacote | O que faz | Quem usa |
|---|---|---|
| `yahboomcar_msgs` | Define tipos customizados: `Target`, `TargetArray`, `Position`, `PointArray`, `ImageMsg`. | `yahboomcar_astra`, `yahboomcar_mediapipe`, `yahboomcar_voice_ctrl` |
| `yahboom_web_savmap_interfaces` | Serviço ROS2 `WebSaveMap.srv` para salvar mapa via interface web. | `yahboom_app_save_map` |

### Pacotes de visão computacional

| Pacote | O que faz | Relação com robodog2 |
|---|---|---|
| `yahboomcar_astra` | Seguimento de objetos por cor HSV com câmera Astra RGB-D. Calibração de cores interativa. | Referência para visão com câmera — **futuro com Jetson Nano**. |
| `yahboomcar_visual` | Câmera Astra (RGB, depth, flip), projeção laser→imagem, AR markers, detecção de objetos (SSD MobileNet). | Base para visão de máquina do robodog2 na Jetson. |
| `yahboomcar_linefollow` | Seguimento de faixa colorida no chão + evitação de obstáculos por LiDAR. PID de visão + PID laser. | Demonstração de fusão visão+LiDAR. |
| `yahboomcar_mediapipe` | Pose humana, mãos, rosto via MediaPipe. Publica landmarks em ROS2. Requer GPU/Jetson para tempo real. | **Futuro com Jetson Nano**: reconhecimento de gestos para interação com o robodog2. |
| `yahboomcar_voice_ctrl` | Versões dos comportamentos (seguimento de cor, linha) com camada de controle por voz. | Referência para comando por voz no robodog2. |

### Utilitários

| Pacote | O que faz |
|---|---|
| `laserscan_to_point_pulisher` | Converte `/scan` (LaserScan 2D) em `/pointcloud` (PointCloud2). |
| `robot_pose_publisher` | Publica a pose do robô (`/robot_pose`) lendo do TF (`map→base_footprint`). |
| `yahboom_app_save_map` | Salva o mapa atual via chamada do app mobile Yahboom. |

---

## Dependências entre pacotes

```
yahboomcar_msgs  ◄── yahboomcar_astra
                 ◄── yahboomcar_mediapipe
                 ◄── yahboomcar_voice_ctrl

yahboom_web_savmap_interfaces  ◄── yahboom_app_save_map

yahboomcar_bringup (runtime) ──► yahboomcar_base_node (base_node_X3)
                              ──► imu_filter_madgwick  (pacote ROS2 padrão)
                              ──► robot_localization   (pacote ROS2 padrão — EKF)
                              ──► robot_state_publisher (pacote ROS2 padrão)
```

Todos os pacotes dependem de `rclpy`/`rclcpp` e mensagens ROS2 padrão (`std_msgs`, `geometry_msgs`, `sensor_msgs`). As dependências internas entre pacotes `yahboomcar_*` são mínimas — cada pacote é relativamente autônomo.

---

## Pacotes que não compilam (bkp_codigo_problematico/)

| Pacote | Motivo | Quando resolver |
|---|---|---|
| `yahboomcar_KCFTracker` | OpenCV4 incompatível (`IplImage` removido) + `image_transport` ausente | Ao implementar visão com Jetson Nano |
| `yahboomcar_slam` | `pcl_ros` não instalado | Ao explorar SLAM 3D com câmera depth |
| `yahboomcar_point` | `pcl_conversions` ausente | Junto com `yahboomcar_slam` |
| `robot_pose_publisher_ros2` | API `tf2_ros::Buffer` incompatível com Humble | Baixa prioridade — usar versão Python (`robot_pose_publisher`) |

Para reativar, instalar dependências e mover de volta para `src/`.

---

## Convenção de nomenclatura

| Prefixo | Origem | Exemplo |
|---|---|---|
| `yahboomcar_` | Fabricante Yahboom (original, não modificar) | `yahboomcar_bringup` |
| `robodog2_` | Autoral — derivado/inspirado do código Yahboom | `robodog2_laser`, `robodog2_vision` |

---

## Regras de trabalho

### robodog2 — pacote âncora
- **Nunca modificar** `robodog2/` quando trabalhando a partir deste contexto (`src/`)
- Modificações ao robodog2 → abrir Claude **dentro da pasta `robodog2/`** no VSCode
- Sempre criar **branch adequada** antes de qualquer alteração no robodog2

### codigo_referencia
- Apenas leitura e estudo — nunca alterar código existente
- Pode adicionar comentários em português e READMEs

### Novos pacotes `robodog2_*`
- Esta pasta `src/` é o sandbox: experimentos, testes, tentativas de integração
- Quando um pacote `robodog2_*` estiver maduro → publicar no GitHub ao lado do robodog2

---

## Estado do estudo (branch: estudo_inicial)

| Pacote | README | Comentários PT-BR | Contexto robodog2 |
|---|---|---|---|
| `yahboomcar_bringup` | ✅ | ✅ | ✅ |
| `yahboomcar_laser` | ✅ | ✅ | ✅ |
| `yahboomcar_description` | ✅ | ✅ | ✅ |
| `yahboomcar_nav` | ✅ | ✅ | ✅ |
| `yahboomcar_msgs` | ✅ | ✅ | ✅ |
| `yahboomcar_ctrl` | ✅ | ✅ | ✅ |
| `yahboomcar_base_node` | ✅ | ✅ | ✅ |
| `yahboomcar_astra` | ✅ | ✅ | ✅ |
| `yahboomcar_visual` | ✅ | ✅ | ✅ |
| `yahboomcar_linefollow` | ✅ | ✅ | ✅ |
| `yahboomcar_mediapipe` | ✅ | ✅ | ✅ |
| `yahboomcar_voice_ctrl` | ✅ | ✅ | ✅ |
| utilitários (4 pacotes) | ✅ | ✅ | ✅ |

---

## Por que este projeto é relevante para a comunidade

O fabricante Yahboom **não fornece suporte a Gazebo** para o ROSMASTER X3. Os pacotes `yahboomcar_*` são 100% voltados para hardware real. O **robodog2 é provavelmente o único projeto público** que coloca o ROSMASTER X3 a funcionar em simulação no **Gazebo Fortress (ROS2 Humble)**, com:

- URDF de simulação com plugins Fortress (velocidade, odometria, LiDAR)
- Bridge ROS-Gazebo configurado
- Mundos SDF convertidos de Gazebo Classic para Fortress
- Nav2 + SLAM calibrado para o X3
- Navegação autônoma com comportamento de patrulha

---

## Referências

- [robodog2 no GitHub](https://github.com/acflemos/robodog2) — pacote principal
- [robodog1](https://github.com/acflemos/robodog1) — versão ROS1, congelada
- [ROSMASTER X3 — Yahboom](https://github.com/YahboomTechnology/ROSMASTERX3)
- [Nav2](https://navigation.ros.org/)
- [Gazebo Fortress](https://gazebosim.org/docs/fortress)
- [ROS2 Humble](https://docs.ros.org/en/humble/)
