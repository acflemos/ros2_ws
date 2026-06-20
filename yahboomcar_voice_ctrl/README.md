# yahboomcar_voice_ctrl

Versões dos nós principais com controlo por voz adicionado (comandos em inglês via microfone).  
Os ficheiros são cópias de `yahboomcar_astra` e `yahboomcar_linefollow` com lógica de reconhecimento de voz integrada.

## Executáveis

| Executável | Nó base | Comandos de voz adicionados |
|-----------|---------|----------------------------|
| `Voice_Ctrl_Mcnamu_driver_X3` | driver X3 | "Go ahead", "Back off", "Turn left/right", "Red/Green/Blue light", etc. |
| `Voice_Ctrl_follow_line_a1_X3` | linefollow X3 | — |
| `Voice_Ctrl_follow_line_a1_R2` | linefollow R2 | — |
| `Voice_Ctrl_follow_line_4ROS_R2` | linefollow 4ROS R2 | — |
| `Voice_Ctrl_colorHSV` | colorHSV | — |
| `Voice_Ctrl_colorTracker` | colorTracker | — |
| `Voice_Ctrl_Ackman_driver_R2` | driver R2 | — |
| `voice_Ctrl_send_mark` | — | Publica waypoints de navegação por voz |

## Tópicos principais (Voice_Ctrl_Mcnamu_driver_X3)

| Direção | Nome | Tipo |
|---------|------|------|
| Sub | `/cmd_vel` | `geometry_msgs/Twist` |
| Sub | `/RGBLight` | `std_msgs/Int32` |
| Sub | `/Buzzer` | `std_msgs/Bool` |
| Pub | `/imu/data_raw` | `sensor_msgs/Imu` |
| Pub | `/vel_raw` | `geometry_msgs/Twist` |
| Pub | `/voltage` | `std_msgs/Float32` |
| Pub | `/joint_states` | `sensor_msgs/JointState` |

## Nota

Este pacote contém cópias dos ficheiros `astra_common.py`, `colorHSV.py` e `follow_common.py` dos outros pacotes — as correções de bugs feitas nos originais **não estão refletidas** nestas cópias.
