# yahboomcar_base_node

Nó de odometria que integra velocidades brutas dos motores em posição 2D.  
É a ponte entre o driver de hardware (motores/encoders) e o stack de navegação ROS 2.

## Arquitetura

```
Driver hardware          base_node_X3          Nav2 / EKF
(Mcnamu_driver_X3.py) → vel_raw (Twist) → /odom_raw (Odometry)
                                         → TF: odom → base_footprint (opcional)
```

## Executáveis

| Executável | Robô | Cinemática | pub_odom_tf default |
|-----------|------|-----------|---------------------|
| `base_node_X3` | ROSMaster X3 | Omnidirecional (mecanum) | true |
| `base_node_x1` | ROSMaster x1 | Omnidirecional | false |
| `base_node_R2` | ROSMaster R2 | Ackermann (steering) | true |

## Tópicos

| Direção | Nome | Tipo | Notas |
|---------|------|------|-------|
| Sub | `vel_raw` | `geometry_msgs/Twist` | Relativo no X3/R2; absoluto `/vel_raw` no x1 |
| Pub | `/odom_raw` | `nav_msgs/Odometry` | Odometria integrada |
| Pub | TF `odom→base_footprint` | TransformStamped | Só se `pub_odom_tf:=true` |

## Parâmetros

| Parâmetro | Default | Descrição |
|-----------|---------|-----------|
| `linear_scale_x` | 1.0 | Fator de calibração — compensar deslizamento em X |
| `linear_scale_y` | 1.0 | Fator de calibração — compensar deslizamento em Y |
| `wheelbase` | 0.25 | Distância entre eixos em metros (só relevante para R2) |
| `pub_odom_tf` | true/false | Publicar TF `odom→base_footprint` diretamente |

**NOTA:** Se usar `robot_localization` (EKF), definir `pub_odom_tf:=false` — o EKF publica o TF e há conflito se ambos publicarem.

## Diferenças entre modelos

### X3 e x1 — cinemática omnidirecional
```
delta_x = (vx * cos(θ) - vy * sin(θ)) * dt
delta_y = (vx * sin(θ) + vy * cos(θ)) * dt
```
- `linear.x` = velocidade forward (m/s)
- `linear.y` = velocidade lateral (m/s) — válido para mecanum
- `angular.z` = velocidade angular (rad/s)

### R2 — cinemática Ackermann
```
R = wheelbase / tan(steer_angle_deg)
angular_z = vx / R
delta_x = vx * cos(θ) * dt   (sem componente lateral)
```
- `linear.x` = velocidade forward (m/s)
- `linear.y` = **ângulo de steering em graus** (campo reutilizado — diferente do X3!)
- `angular.z` do Twist é IGNORADO (calculado internamente)

## Bugs corrigidos

### base_node_X3.cpp
1. **vy zerado na odometria** — linha original sobrescrevia `linear_velocity_y_` com `0.0`, eliminando informação de movimento lateral do mecanum. Corrigido para publicar `linear_velocity_y_` real.
2. **Código morto** — variáveis `steer_angle` e `MI_PI` copiadas do R2 mas nunca usadas no X3. Removidas.

## Uso no robodog2

Para o X3 mecanum, usar `base_node_X3` como nó de odometria.  
O EKF em `yahboomcar_bringup` funde `/odom_raw` com `/imu/data` para produzir `/odom`.

```bash
ros2 run yahboomcar_base_node base_node_X3 \
  --ros-args -p pub_odom_tf:=false -p linear_scale_x:=1.0 -p linear_scale_y:=1.0
```
