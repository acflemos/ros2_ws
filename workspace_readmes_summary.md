# Síntese e Memória do Workspace — ROSMASTER X3 & robodog2

Este documento consolida o conhecimento extraído de todos os arquivos `README.md` do workspace `ros2_ws/src`. Ele serve como memória técnica e guia de integração para as próximas fases do projeto **robodog2** no hardware real.

---

## 1. Visão Geral do Ecossistema

O workspace é composto por duas grandes vertentes:
1. **robodog2 (Pacote Âncora)**: Solução autoral e estável focada na simulação no **Gazebo Fortress** sob **ROS2 Humble**. Implementa navegação autônoma por comportamento (patrulha por pesos baseada em instintos) com Nav2 e `slam_toolbox`.
2. **yahboomcar_\***: Pacotes originais do fabricante Yahboom para o robô físico **ROSMASTER X3** (e variantes Ackermann R2 e menor X1). Fornecem drivers de hardware, comportamentos autônomos baseados em câmera e LiDAR, reconhecimento de voz e visão computacional (MediaPipe).

```mermaid
graph TD
    subgraph Simulação (robodog2)
        gz[Gazebo Fortress] --- |ros_gz_bridge| rbd_nodes[Nós do robodog2]
        rbd_nodes --> nav2[Nav2 / slam_toolbox]
    end

    subgraph Hardware Real (yahboomcar_*)
        arduino[Hardware Arduino] --- |Rosmaster_Lib| bringup[yahboomcar_bringup]
        bringup --> base_node[yahboomcar_base_node]
        base_node --> EKF[robot_localization]
        lidar[LiDAR / Câmera Astra] --> laser_nodes[yahboomcar_laser / astra / visual]
    end
```

---

## 2. Inventário de Pacotes e Módulos

### A. Core de Hardware e Drivers
*   **[yahboomcar_bringup](file:///home/antonio/ros2_ws/src/yahboomcar_bringup)**: Inicialização física e drivers.
    *   *Mcnamu_driver_X3.py*: Peça central. Lê encoders (`/vel_raw`), IMU (`/imu/data_raw`), junta estados (`/joint_states`) e recebe velocidade (`/cmd_vel`).
    *   *Calibração*: Scripts `calibrate_linear_X3` e `calibrate_angular_X3` para ajustar odometria.
    *   *Patrulha básica*: `patrol_a1_X3.py` realiza movimentos básicos (quadrado, triângulo, círculo), pausando se o joystick for ativado.
*   **[yahboomcar_base_node](file:///home/antonio/ros2_ws/src/yahboomcar_base_node)**:
    *   *base_node_X3*: Integra `/vel_raw` para produzir `/odom_raw` usando cinemática de rodas Mecanum (omnidirecional).
*   **[yahboomcar_description](file:///home/antonio/ros2_ws/src/yahboomcar_description)** & **[description_x1](file:///home/antonio/ros2_ws/src/yahboomcar_description_x1)**:
    *   Modelos URDF e malhas STL do robô real para exibição no RViz2.

### B. Navegação e LiDAR
*   **[yahboomcar_nav](file:///home/antonio/ros2_ws/src/yahboomcar_nav)**:
    *   Configurações Nav2 (DWB/TEB) e SLAM (Cartographer/gmapping) para o hardware real.
    *   *scan_filter.py*: Reduz densidade do laser (downsampling de 50%) para gmapping.
*   **[yahboomcar_laser](file:///home/antonio/ros2_ws/src/yahboomcar_laser)**:
    *   *Avoidance*: Desvio reativo baseado em 10 casos (bloqueante usando `sleep()`).
    *   *Tracker*: Seguimento PID do objeto mais próximo.
    *   *Warning*: Gira em direção ao obstáculo e liga o buzzer.
*   **[laserscan_to_point_pulisher](file:///home/antonio/ros2_ws/src/laserscan_to_point_pulisher)**:
    *   Converte `/scan` para `nav_msgs/Path` (lista de poses) para visualização.

### C. Visão, Voz e Interação
*   **[yahboomcar_astra](file:///home/antonio/ros2_ws/src/yahboomcar_astra)**:
    *   *colorHSV*: Detecção interativa de cores HSV usando câmera RGB.
    *   *colorTracker*: Seguimento de objetos com controle PID de distância (via profundidade Astra RGB-D) e centragem.
*   **[yahboomcar_visual](file:///home/antonio/ros2_ws/src/yahboomcar_visual)**:
    *   Visualização e processamento básico: Flip de imagens, conversão do laser em imagem bird's-eye (`laser_to_image`), e realidade aumentada simples com checkerboard (`simple_AR`).
*   **[yahboomcar_mediapipe](file:///home/antonio/ros2_ws/src/yahboomcar_mediapipe)**:
    *   Reconhecimento de mãos, pose, holistic e face mesh. Publica pontos no tópico `/mediapipe/points` em formato normalizado `[0, 1]`.
*   **[yahboomcar_voice_ctrl](file:///home/antonio/ros2_ws/src/yahboomcar_voice_ctrl)**:
    *   Camada de reconhecimento de voz baseada em comandos em inglês para ativar os comportamentos principais (desvio, seguimento, LEDs do carro). *Nota: Contém duplicatas estáticas de código dos pacotes originais.*

### D. Salvamento de Mapas e Mensagens
*   **[yahboomcar_msgs](file:///home/antonio/ros2_ws/src/yahboomcar_msgs)**:
    *   Tipos de mensagens personalizadas como `Target`, `TargetArray`, `ImageMsg`, `Position` e `PointArray`.
*   **[yahboom_web_savmap_interfaces](file:///home/antonio/ros2_ws/src/yahboom_web_savmap_interfaces)** & **[yahboom_app_save_map](file:///home/antonio/ros2_ws/src/yahboom_app_save_map)**:
    *   Serviço `WebSaveMap.srv` e nó em SQLite para salvar mapas na estrutura exigida pela aplicação móvel Yahboom.

---

## 3. Bugs Relevantes e Limitações

Abaixo estão listados os principais problemas identificados nas bases originais que devem ser levados em consideração ou corrigidos ao transitar para o hardware do robodog2:

| Componente | Problema Detectado | Impacto | Resolução Adotada / Necessária |
| :--- | :--- | :--- | :--- |
| **Mcnamu_driver_X3.py** | Nomes errados em `/joint_states` | Robot state publisher pode emitir warnings de incompatibilidade com o URDF | Corrigir nomes de juntas para bater com o URDF do robodog2 |
| **Mcnamu_driver_X3.py** | Sem animação de rodas | Rodas ficam estáticas no RViz | Publicar `state.position` (mesmo que incremental/estimada) |
| **base_node_X3.cpp** | `vy` zerado na odometria | Perda do componente de movimento lateral do Mecanum | Corrigido para repassar o `linear_velocity_y_` real |
| **dwa_nav_params.yaml** | `robot_model_type` como `"differential"` e `max_vel_y: 0.0` | Nav2 ignora o fato de o X3 ser holonômico | Alterar para `"omni"` e habilitar limites e amostragem de `y` |
| **laser_Avoidance_a1_X3.py** | `self.Moving` não inicializado | Crasha se o joystick enviar estado antes do LiDAR | Inicializar `self.Moving = False` no `__init__` |
| **laser_Tracker_4ROS.py** | Typo `self.laserAngle` | Crasha no primeiro callback do scanner | Corrigir para `self.LaserAngle` |
| **colorTracker.py** | Unidades e escalas misturadas (mm vs metros) | PID de distância agia com leituras erradas | Ajustar limites de profundidade (`0.04 < d < 80.0`) e limiar anti-spike |
| **Vários scripts** | Caminhos hardcoded para `/root/...` ou `/home/yahboom/...` | Falhas imediatas ao executar em outras máquinas ou ambientes | Parametrizar caminhos por pacotes ROS 2 ou caminhos de usuário |

---

## 4. O Caminho para Integração no Hardware Real

Para colocar o ciclo autônomo do **robodog2** rodando no robô físico, o plano deve seguir esta sequência estruturada:

### Passo 1: Preparar o `rbd2_bringup` (Pacote de Hardware)
1.  **Rosmaster_Lib**: Garantir que a biblioteca de baixo nível consegue comunicar com a placa controladora Arduino a partir da SBC local (Raspberry Pi).
2.  **Driver Autoral (`rbd2_driver.py`)**: Criar uma versão limpa de `Mcnamu_driver_X3.py` com:
    *   Nomes de juntas corrigidos para o URDF do robodog2.
    *   Atualização visual de rotação de rodas.
    *   Logging padrão de ROS 2 (`self.get_logger()`).
3.  **Launch File (`rbd2_bringup_launch.py`)**: Replicar a cadeia de nós real:
    *   `rbd2_driver` + `rbd2_base_node`
    *   Filtro Madgwick (`imu_filter_madgwick`) com magnetômetro desativado.
    *   Filtro EKF (`robot_localization`) unificando odometria e giroscópio para publicar o TF `odom -> base_footprint`.
    *   `robot_state_publisher` apontando para o URDF real.

### Passo 2: Calibração Física
1.  Executar calibração linear (`calibrate_linear_X3`) para determinar o fator corretivo de deslocamento.
2.  Executar calibração angular (`calibrate_angular_X3`) para corrigir erros de rotação (fator de correção padrão em torno de 0.75).

### Passo 3: Adaptação do Nav2 para Omni
*   Ajustar os arquivos de parâmetros do Nav2 (`rbd_dwa_nav_params.yaml`) para refletir a cinemática omnidirecional:
    ```yaml
    robot_model_type: "omni"
    max_vel_y: 0.2
    vy_samples: 10
    ```

### Passo 4: Execução do Loop de Decisão no Robô
*   Executar o nó `rbd_navega` no robô físico para acionar o motor de patrulha autônoma por pesos em sincronia com o hardware real calibrado.
