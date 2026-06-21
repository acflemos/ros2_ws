# Apostila Técnica: Projeto robodog2 e Plataforma ROSMASTER X3

> [!NOTE]
> **Data de Atualização:** 19 de Junho de 2026  
> **Hardware de Referência:** ROSMASTER X3 (Raspberry Pi 4B)  
> **Ecossistema:** ROS 2 Humble & Gazebo Fortress (v6.17.1)  
> **Status do Projeto:** Em desenvolvimento ativo

---

## 1. Introdução ao Projeto robodog2

### Contextualização e Propósito
O **robodog2** é um sistema robótico de vigilância doméstica projetado para exibir comportamentos autônomos inspirados em instintos caninos. Diferente de patrulhas lineares convencionais, o robodog2 utiliza uma lógica de "vontade própria" baseada em pesos, permitindo que o robô tome decisões sobre qual cômodo visitar de forma orgânica. 

Este projeto representa a evolução técnica do *robodog1* (desenvolvido em ROS1 Noetic), marcando a transição para o ecossistema **ROS2 Humble**. O hardware de referência é o **ROSMASTER X3** da Yahboom. Devido à ausência de suporte oficial do fabricante para simulação em ROS2, o projeto robodog2 foca na criação de um *Digital Twin* robusto no **Gazebo Fortress**, garantindo paridade entre o código simulado e o hardware real.

### Tabela Comparativa de Migração
Abaixo, detalha-se a evolução arquitetural entre as gerações do projeto:

| Característica | robodog1 | robodog2 |
| :--- | :--- | :--- |
| **ROS** | ROS1 Noetic | ROS2 Humble |
| **Hardware** | TurtleBot3 Waffle (Simulado) | ROSMASTER X3 (Raspberry Pi 4B) |
| **Simulação** | Gazebo Classic | Gazebo Fortress (v6.17.1) |
| **Navegação** | move_base + AMCL | Nav2 (AMCL omnidirecional + DWB) |
| **SLAM** | gmapping | slam_toolbox |
| **Build System** | catkin | colcon / ament_python |
| **Status** | Congelado (Referência) | Em desenvolvimento ativo |

---

## 2. Hardware: Plataforma ROSMASTER X3 (Yahboom)

### Componentes do Sistema
A base móvel ROSMASTER X3 é composta por hardware de alta performance para robótica autônoma:
* **Unidade de Processamento:** Raspberry Pi 4B (4 GB RAM).
* **Percepção Espacial:** LiDAR 360° (YDLIDAR X4 ou LDROBOT LD14) para detecção de obstáculos e SLAM.
* **Visão:** Câmera RGB para processamento de imagem e futura integração de IA.
* **Navegação Inercial:** Unidade IMU de 9 eixos (utilizada no hardware físico para estabilização de odometria).
* **Locomoção Mecanum:** O uso de quatro rodas Mecanum confere capacidade **omnidirecional**. Isso permite que o robô execute movimentos de translação lateral (*strafe*) e rotação simultânea, facilitando a navegação em corredores estreitos e ambientes domésticos densos.

### Integração de Driver
No hardware real, a comunicação é gerida pelo pacote `yahboomcar_bringup`. O arquivo **Mcnamu_driver_X3.py** é o componente crítico que traduz comandos de velocidade ROS para a biblioteca `Rosmaster_Lib` (Arduino). Este driver é o equivalente funcional aos plugins de controle utilizados no Gazebo.

---

## 3. Simulação e Digital Twin no Gazebo Fortress

### Integração com Gazebo Fortress
O projeto preenche a lacuna do fabricante ao fornecer suporte ao **Gazebo Fortress (Ignition v6.17.1)**. Um ponto técnico crucial foi a conversão dos arquivos de mundo e modelos do Gazebo Classic para o formato **SDF 1.6**, com o ajuste manual de poses de modelos a partir do bloco dos arquivos originais para garantir fidelidade posicional.

### Arquitetura de Plugins URDF
Para emular o comportamento físico, o modelo URDF utiliza quatro plugins do Fortress:
1. **ignition-gazebo-velocity-control-system:** Subscreve o tópico de comando de velocidade.
2. **ignition-gazebo-odometry-publisher-system:** Publica dados de odometria e as transformações TF (`odom -> base_footprint`).
3. **ignition-gazebo-joint-state-publisher-system:** Monitora o estado das juntas das rodas.
4. **gpu_lidar:** Gera dados de varredura laser sintéticos para o tópico `/scan`.

### Comunicação via Bridge
A ponte entre os mundos ROS e Gazebo é estabelecida pelo `ros_gz_bridge` (configurado em `rbd_x3_bridge.yaml`), seguindo o mapeamento:
* **Gazebo → ROS:** `/clock`, `/joint_states`, `/odom`, `/tf`, `/scan`.
* **ROS → Gazebo:** `/cmd_vel -> /model/rosmaster_x3/cmd_vel`.

### Ambientes de Teste
* **cma_vazio.world:** Residência de 15 cômodos sem mobília para calibração cinemática.
* **cma_moveis.world:** Cenário complexo com 79 modelos de móveis para validação de desvio de obstáculos.

---

## 4. Mapeamento (SLAM) e Navegação Autônoma

### Processo de SLAM
Utiliza-se o **slam_toolbox** para a geração de mapas estáticos e dinâmicos. Os mapas validados no projeto possuem as seguintes especificações:
* **Mapa Vazio (rbd_mapa_vazio):** 485 × 378 pixels.
* **Mapa Mobiliado (rbd_mapa_moveis):** 312 × 374 pixels.
* **Resolução:** 0.05 m/px.

### Pilha de Navegação (Nav2)
A arquitetura Nav2 foi ajustada para a cinemática do X3. O controlador **DWB (Dynamic Window Controller)** foi selecionado especificamente por sua capacidade de lidar com restrições holonômicas e não-holonômicas, permitindo o uso pleno das rodas Mecanum.

### Tuning de Parâmetros Críticos
Para operação em ambientes domésticos, os seguintes parâmetros foram otimizados:
* `inflation_radius`: Raio de expansão de obstáculos para segurança.
* `cost_scaling_factor`: Peso do custo de proximidade.
* `sim_time`: Tempo de projeção de trajetórias para o DWB.
* `acc_lim_theta`: Limites de aceleração angular para evitar oscilações.
* **Partículas AMCL:** Ajuste fino da população de partículas para localização precisa em ambientes com poucos pontos de referência (paredes lisas).

---

## 5. Lógica de Comportamento: Instintos Programados

O robodog2 utiliza um sistema de tomada de decisão baseado em **acumulação de pesos**, simulando necessidades ou "vontades".

### Ciclo do Algoritmo
1. **Incremento:** A cada iteração, os pesos de todas as tarefas disponíveis são aumentados.
2. **Seleção:** A tarefa com o maior peso positivo é selecionada (empates são resolvidos por escolha aleatória).
3. **Execução:** O sistema envia os objetivos (Waypoints) para o Nav2.
4. **Decremento:** Após a conclusão, o peso da tarefa executada é reduzido drasticamente.
5. **Reset de Ciclo:** O ciclo reinicia continuamente.

**Condição especial:** Quando todos os pesos tornam-se negativos, o sistema realiza um reset completo da pilha de prioridades.

### Mecanismos de Segurança
O nó `rbd2_navega` implementa a função **foge_de_parede()**. Este é um comportamento de recuperação (*fallback*) acionado quando o planejador local do Nav2 entra em conflito em cantos apertados, forçando o robô a realizar uma manobra de recuo e reorientação.

---

## 6. Guia do Workspace de Estudo (ros2_ws)

### Organização e Regras de Trabalho
O workspace diferencia o código original do fabricante do código autoral do projeto:

| Pacote | Função | Relação com robodog2 |
| :--- | :--- | :--- |
| **yahboomcar_bringup** | Inicialização hardware real. | Base para o rbd2_bringup. |
| **yahboomcar_laser** | Desvio e seguimento via LiDAR. | Atua como monitor de segurança (*safety monitor*). |
| **yahboomcar_visual** | Visão computacional básica. | Base para futura integração com Jetson Nano. |
| **robodog2** | Pacote âncora do projeto. | Contém simulação, navegação e instintos. |

**Nota de Segurança:** A pasta `codigo_referencia/` deve ser tratada como **somente leitura**. Modificações devem ocorrer apenas nos pacotes com prefixo `robodog2_` localizados em `src/`.

### Aliases de Produtividade e Variáveis
Adicione as seguintes definições ao seu `~/.bash_aliases`:

```bash
# Fix para RViz em Máquinas Virtuais (Evita erro de renderização GLSL)
export OGRE_RTT_MODE=Copy

# Atalhos de Build e Paths
alias rbd2_build='colcon build --symlink-install'
export RBD2_MAPS_SRC=~/ros2_ws/src/robodog2/maps/

# Operação
alias rbd2_sim='ros2 launch robodog2 rbd2_simulador_x3_moveis.launch.py'
alias rbd2_navega='ros2 run robodog2 rbd2_navega'
```

---

## 7. Procedimentos de Uso

### Cenário: Geração de Mapas
1. Lance o simulador em um mundo mobiliado.
2. Execute o `slam_toolbox` via launch file.
3. Utilize o teleop para percorrer a casa.
4. Salve o mapa utilizando a variável `$RBD2_MAPS_SRC` para garantir que o arquivo seja versionado corretamente no pacote.

### Cenário: Navegação Autônoma
1. Execute o launch de simulação (ele carregará o mapa e o Nav2 automaticamente).
2. Inicie o nó de comportamento: `ros2 run robodog2 rbd2_navega`.
3. Monitore os costmaps e trajetórias no RViz (utilize o arquivo de configuração `robodog2.rviz`).

---

## 8. Status do Projeto e Roadmap

### Checklist de Validação Técnica
* [x] Spawn do X3 no Gazebo Fortress sem *flickering* de modelos.
* [x] Teleoperação omnidirecional funcional via bridge.
* [x] Odometria e transformações TF (`odom -> base_footprint`) validadas.
* [x] Mapas versionados para ambientes com e sem móveis.
* [x] Navegação autônoma via Nav2 com DWB validada em simulação.
* [x] Correção de renderização RViz aplicada (`OGRE_RTT_MODE`).

### Próximos Passos (Roadmap)
1. **Transição para Hardware:** Validar o `rbd2_bringup` no robô físico.
2. **Visão Computacional:** Implementar MediaPipe na Jetson Nano para reconhecimento de gestos.
3. **Calibração Dinâmica:** Ajustar os parâmetros de `rbd_tabelas.py` para as dimensões físicas reais da residência.
4. **Fusão Sensorial:** Ativar IMU física para mitigar erros de odometria em superfícies irregulares.
