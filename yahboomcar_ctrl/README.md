# yahboomcar_ctrl

Teleoperação do ROSMaster X3/R2 via joystick ou teclado.

## Executáveis

| Executável | Ficheiro | Descrição |
|-----------|---------|-----------|
| `yahboom_joy_X3` | `yahboom_joy_X3.py` | Joystick para X3 (mecanum, suporta linear.y) |
| `yahboom_joy_R2` | `yahboom_joy_R2.py` | Joystick para R2 (Ackermann, linear.y = steering angle) |
| `yahboom_keyboard` | `yahboom_keyboard.py` | Controlo por teclado (terminal) |

## Tópicos

| Direção | Nome | Tipo | Descrição |
|---------|------|------|-----------|
| Sub | `/joy` | `sensor_msgs/Joy` | Input do joystick (nó `joy_node`) |
| Pub | `/cmd_vel` | `geometry_msgs/Twist` | Comandos de velocidade |
| Pub | `/JoyState` | `std_msgs/Bool` | Estado ativo/inativo do joystick |
| Pub | `/RGBLight` | `std_msgs/Int32` | Índice da cor RGB (0-5) |
| Pub | `/Buzzer` | `std_msgs/Bool` | Ativar/desativar buzzer |

## Parâmetros (joy)

| Parâmetro | Default X3 | Default R2 | Descrição |
|-----------|-----------|-----------|-----------|
| `xspeed_limit` | 1.0 | 1.0 | Velocidade máxima forward (m/s) |
| `yspeed_limit` | 1.0 | 5.0 | X3: vel. lateral (m/s) / R2: steering máximo (graus) |
| `angular_speed_limit` | 5.0 | 5.0 | Velocidade angular máxima (rad/s) |

## Mapeamento de botões — Joystick Yahboom (Jetson, user=root)

| Botão/Eixo | X3 | R2 | Função |
|-----------|----|----|--------|
| `axes[9]==1` | ✓ | — | Toggle Joy_active |
| `buttons[9]==1` | — | ✓ | Toggle Joy_active |
| `buttons[7]` | ✓ | ✓ | Ciclar cor RGB |
| `buttons[11]` | ✓ | ✓ | Toggle buzzer |
| `buttons[13]` | ✓ | ✓ | Ciclar marcha linear |
| `buttons[14]` | ✓ | ✓ | Ciclar marcha angular |
| `axes[1]` | ✓ | ✓ | Velocidade forward |
| `axes[0]` | ✓ | — | Velocidade lateral (mecanum) |
| `axes[2]` | ✓ | ✓ | X3: rotação / R2: steering |

**NOTA:** o joystick só envia `cmd_vel` quando `Joy_active=True` (toggle com o botão acima).  
O PC remoto (`user_pc`) não tem este guarda — publica sempre.

## Lógica de marcha

- **Linear**: cicla entre 1/3 → 2/3 → 1.0 do limite configurado
- **Angular**: cicla entre 1/4 → 1/2 → 3/4 → 1.0 do limite configurado
- As velocidades são clampadas aos limites em cada callback

## Teclado (yahboom_keyboard)

```
   u    i    o        frente-esq  frente  frente-dir
   j    k    l   →    rot-esq     parar   rot-dir
   m    ,    .        trás-esq    trás    trás-dir
```

- `t/T`: alternar publicação entre `linear.x` e `linear.y` (modo mecanum)
- `s/S`: toggle de pausa (continua a publicar Twist zeros)
- `q/z`: ±10% em ambas as velocidades
- `w/x`: ±10% só na linear; `e/c`: ±10% só na angular

## Bugs corrigidos

### yahboom_joy_X3.py e yahboom_joy_R2.py
- **user_pc — tipos errados**: `pub_RGBLight.publish(int)` e `pub_Buzzer.publish(bool)` causavam `TypeError` em ROS 2. Corrigido para `Int32(data=...)` e `Bool(data=...)`.

### yahboom_joy_R2.py
- **user_jetson — axes duplicado**: `ylinear_speed` e `angular_speed` usavam ambos `axes[2]`. Para R2 Ackermann, `angular.z` não é publicado (calculado internamente pelo `base_node_R2`), por isso a variável `angular_speed` foi removida do `user_jetson`.

### yahboom_keyboard.py
- **Typo**: `linenar_speed_limit` → `linear_speed_limit`

## Integração com robodog2

O X3 usa `linear.y` para movimento lateral real — confirmar que os limites de velocidade estão sincronizados com os parâmetros do `base_node_X3` e do Nav2 DWB.
