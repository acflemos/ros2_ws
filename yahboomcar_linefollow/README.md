# yahboomcar_linefollow

Seguimento de linha colorida com evitação de obstáculos via LiDAR, usando PID + visão HSV.

## Executáveis

| Executável | Robô | Laser monitoriza |
|-----------|------|-----------------|
| `follow_line_a1_X3` | X3 (câmara invertida) | **Retaguarda** — `abs(angle) > 180° - LaserAngle` |
| `follow_line_a1_R2` | R2 alternativo | Retaguarda (igual ao X3) |
| `follow_line_4ROS_R2` | R2 / 4ROS | **Frente** — `abs(angle) < LaserAngle` |

**NOTA:** O X3 monitoriza a retaguarda porque a câmara está montada virada para trás — por isso o `img_flip=True` faz flip horizontal antes do processamento.

## Tópicos

| Direção | Nome | Tipo | Descrição |
|---------|------|------|-----------|
| Sub | `/JoyState` | `std_msgs/Bool` | Pausa seguimento quando joystick ativo |
| Sub | `/scan` | `sensor_msgs/LaserScan` | Deteção de obstáculos |
| Pub | `/cmd_vel` | `geometry_msgs/Twist` | Comandos de movimento |
| Pub | `/Buzzer` | `std_msgs/Bool` | Alerta sonoro ao detetar obstáculo |
| Pub | `/linefollow/rgb` | `sensor_msgs/Image` | Imagem processada (não usado ativamente) |

## Parâmetros

| Parâmetro | Default | Descrição |
|-----------|---------|-----------|
| `Hmin/Smin/Vmin` | 0/85/126 | Limite inferior do range HSV |
| `Hmax/Smax/Vmax` | 9/253/253 | Limite superior do range HSV |
| `Kp/Ki/Kd` | 60/0/20 | Ganhos PID angular |
| `linear` | 0.18 | Velocidade linear constante (m/s) |
| `LaserAngle` | 30 | Semi-ângulo de monitorização do laser (graus) |
| `ResponseDist` | 0.55 | Distância de paragem por obstáculo (m) |

## Algoritmo

```
1. Captura frame (VideoCapture 0)
2. Se img_flip: flip horizontal (câmara invertida no X3)
3. Aplica máscara HSV → contornos → maior área → centroide (px, py, raio)
4. PID: erro = (px - 320) / 16  →  angular.z
5. Se abs(px - 320) < 40: angular.z = 0 (zona morta)
6. linear.x = constante (parâmetro `linear`)
7. LiDAR: contar pontos na zona de interesse com dist < ResponseDist
8. Se warning > 10: parar + buzzer
```

## Estados interativos

| Tecla | Estado |
|-------|--------|
| Espaço | Iniciar tracking |
| `i` | Carregar HSV do ficheiro |
| `r` | Reset (HSV + PID) |
| `q` | Sair |
| Mouse (drag) | Desenhar ROI para aprender nova cor |

## Ficheiro de calibração

`yahboomcar_linefollow/LineFollowHSV.text`: `55, 73, 218, 125, 253, 255`  
(HSV range para linha amarela/laranja em condições de luz interior)

Caminho hardcoded para `/root/yahboomcar_ros2_ws/...` — deve ser ajustado.

## Bugs conhecidos

- **FPS inválido**: `start = time.time(); end = time.time()` consecutivos → FPS sempre infinito ou erro de divisão por zero.
- **hsv_text hardcoded**: falha em qualquer máquina que não seja a Yahboom original.
- **Thread por frame**: `threading.Thread(target=execute, ...)` cria nova thread a cada tick do timer sem gestão de ciclo de vida.
