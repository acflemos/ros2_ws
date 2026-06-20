# yahboomcar_description_x1

Descrição URDF/Xacro do robô Yahboomcar X1 e X3 para visualização em RViz.  
Sem nós ROS 2 — apenas recursos estáticos (URDFs, meshes, configuração RViz).

## Ficheiros URDF

| Ficheiro | Robô |
|---------|------|
| `yahboomcar_X1.urdf` / `.xacro` | ROSMaster X1 (diferencial) |
| `yahboomcar_X3.urdf` / `.xacro` | ROSMaster X3 (mecanum) |

## Launch

`display.launch.py`:
- Parâmetro `model` (default: `yahboomcar_X1.urdf`) — escolher X1 ou X3
- Parâmetro `gui` (default: `false`) — abre `joint_state_publisher_gui` para mover juntas
- Lança: `robot_state_publisher` + `joint_state_publisher` + `rviz2`

## Nota para robodog2

O `yahboomcar_description` (já documentado) contém URDFs mais completos com todos os sensores. Este pacote é redundante — os URDFs X3 aqui podem diferir dos de `yahboomcar_description`. Usar sempre `yahboomcar_description` como referência canónica.
