# yahboomcar_description — Descrição do ROSMASTER X3 (ROS2)

Pacote ROS2 originado do **ROSMASTER X3** (Yahboom). Contém a descrição geométrica do robô (URDF, malhas STL) e launch files para visualização no RViz2.

Este pacote é a **referência ROS2** para a geometria do robô com rodas mecanum — equivalente ao arquivo `urdf/yahboomcar_X3_sim.urdf.xacro` do robodog1 (ROS1), mas estruturado como pacote ROS2 independente.

---

## Estrutura

```
yahboomcar_description/
├── urdf/                  ← Modelos URDF e Xacro
├── meshes/                ← Malhas STL (X3 mecanum + R2 Ackermann + sensores)
├── launch/                ← Launch files de visualização
├── rviz/                  ← Configuração RViz2
└── yahboomcar_description/ ← Módulo Python do pacote
```

---

## Arquivos URDF

| Arquivo | Variante | Descrição |
|---|---|---|
| [urdf/yahboomcar_X3.urdf](urdf/yahboomcar_X3.urdf) | X3 (mecanum) | URDF compilado (gerado pelo SolidWorks). Pronto para uso direto como `robot_description`. Sem namespace. |
| [urdf/yahboomcar_X3.urdf.xacro](urdf/yahboomcar_X3.urdf.xacro) | X3 (mecanum) | Versão parametrizada com suporte a namespace (`ns`). Usa macros `common_link`, `continuous_joint`, `fixed_joint`. **Preferir este para multi-robô.** |
| [urdf/yahboomcar_R2.urdf](urdf/yahboomcar_R2.urdf) | R2 (Ackermann) | URDF compilado do modelo Ackermann (direcção tipo carro). |
| [urdf/yahboomcar_R2.urdf.xacro](urdf/yahboomcar_R2.urdf.xacro) | R2 (Ackermann) | Versão parametrizada do R2. |
| [urdf/yahboomcar_R2_multi.urdf.xacro](urdf/yahboomcar_R2_multi.urdf.xacro) | R2 multi-robô | Variante do R2 para simulação com múltiplos robôs em namespaces separados. |

### Geometria do X3 (mecanum)

- **Corpo (base_link):** massa 0.486 kg, inércia calculada pelo SolidWorks
- **4 rodas mecanum:** massa 0.051 kg cada, joint contínuo em torno do eixo Z
  - Dianteiras (frente): x=+0.08m, y=±0.085m, z=-0.039m
  - Traseiras (trás):    x=-0.08m, y=±0.085m, z≈-0.040m
- **LiDAR (laser_link):** x=0.044m, z=0.11m (LDROBOT LD14)
- **Câmera (camera_link):** x=0.057m, z=0.038m

---

## Malhas STL

| Pasta | Conteúdo |
|---|---|
| [meshes/mecanum/](meshes/mecanum/) | Corpo e 4 rodas mecanum do X3 (`base_link_X3.STL`, `front/back_left/right_wheel_X3.STL`) |
| [meshes/Ackermann/](meshes/Ackermann/) | Corpo, rodas, braços de direção e sensores do R2 |
| [meshes/sensor/](meshes/sensor/) | LiDAR e câmera comuns às duas variantes |
| [meshes/](meshes/) | Malhas avulsas (versões alternativas) |

**NOTA:** Existem malhas duplicadas na subpasta `yahboomcar_description/meshes/` (cópia do módulo Python). Os caminhos `package://yahboomcar_description/meshes/` resolvem para a pasta `meshes/` da raiz do pacote.

---

## Launch files

| Arquivo | Descrição |
|---|---|
| [launch/display_X3.launch.py](launch/display_X3.launch.py) | Lança `robot_state_publisher` + `joint_state_publisher_gui` + `rviz2` para visualizar o X3. Sem hardware real — apenas visualização. |
| [launch/display_R2.launch.py](launch/display_R2.launch.py) | Mesmo que acima, mas para o modelo R2 (Ackermann). |

---

## Comparação com robodog1 (ROS1)

| Aspeto | robodog1 `yahboomcar_X3_sim.urdf.xacro` (ROS1) | `yahboomcar_X3.urdf.xacro` (ROS2) |
|---|---|---|
| Plugins Gazebo | Incluídos no próprio arquivo | **Não incluídos** (apenas geometria) |
| Namespace | Fixo (sem parâmetro) | Parametrizado via `$(arg ns)` |
| Malhas | `package://robodog1/meshes/` | `package://yahboomcar_description/meshes/` |
| Odometria | `libgazebo_ros_planar_move.so` | Gerida pelo `yahboomcar_bringup` (hardware real) |

---

## Uso no robodog2 (próximos passos)

- [ ] Criar `yahboomcar_X3_sim.urdf.xacro` equivalente para ROS2 com plugins Gazebo ROS2
- [ ] Atualizar caminhos `package://yahboomcar_description/meshes/` nos launch files do robodog2
- [ ] Adaptar `yahboomcar_X3.urdf.xacro` para ser incluído no URDF principal do robodog2
- [ ] O parâmetro `ns` do xacro é útil para futuros testes multi-robô no robodog2
