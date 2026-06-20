# yahboomcar_astra

Deteção e seguimento de objetos por cor HSV usando a câmara Astra RGB-D.

## Executáveis

| Executável | Nó | Descrição |
|-----------|-----|-----------|
| `colorHSV` | `ColorIdentify` | Deteção de cor + publicação de posição 2D |
| `colorTracker` | `ColorTracker` | Seguimento com PID + profundidade (desativado no launch) |

## Arquitetura

```
Câmara Astra (VideoCapture 0)
        ↓
  colorHSV (ColorIdentify)
  ├─ Deteção HSV → círculo envolvente
  └─ Publica → /Current_point (Position: pixel_x, pixel_y, raio)

Câmara Astra (image_raw depth, 32FC1)
        ↓
  colorTracker (ColorTracker)
  ├─ Subscreve /Current_point
  ├─ Mede profundidade em 5 pontos ao redor do centro
  ├─ PID linear  → manter distância = minDistance
  ├─ PID angular → centrar objeto em x=320
  └─ Publica → /cmd_vel
```

## Tópicos

| Direção | Nó | Nome | Tipo | Descrição |
|---------|-----|------|------|-----------|
| Pub | colorHSV | `/Current_point` | `yahboomcar_msgs/Position` | Posição do objeto (pixel x, y, raio) |
| Pub | colorHSV | `/cmd_vel` | `geometry_msgs/Twist` | Zeros quando sem objeto |
| Sub | colorTracker | `/Current_point` | `yahboomcar_msgs/Position` | Posição do objeto |
| Sub | colorTracker | `/image_raw` | `sensor_msgs/Image` | Depth 32FC1 (metros) |
| Sub | colorTracker | `/JoyState` | `std_msgs/Bool` | Pausa PID quando joystick ativo |
| Pub | colorTracker | `/cmd_vel` | `geometry_msgs/Twist` | Comandos de seguimento |

**NOTA sobre `Position`:** os campos `anglex`/`angley`/`distance` são reutilizados como pixel_x/pixel_y/raio — semântica diferente do nome do tipo.

## Parâmetros (colorTracker)

| Parâmetro | Default | Descrição |
|-----------|---------|-----------|
| `linear_Kp/Ki/Kd` | 3.0 / 0.0 / 1.0 | PID de distância |
| `angular_Kp/Ki/Kd` | 0.5 / 0.0 / 2.0 | PID de centragem horizontal |
| `minDistance` | 1.0 | Distância alvo em metros |

## Estados (colorHSV)

| Estado | Tecla | Descrição |
|--------|-------|-----------|
| `identify` | `i` | Carrega HSV do ficheiro `colorHSV.text` |
| `init` | (mouse) | Desenhar ROI para aprender cor |
| `tracking` | Espaço | Ativo — publica posição do objeto |

## Bugs corrigidos

### colorTracker.py
1. **`distance_` inicializado a `1000.0`** → inflacionava distância medida em 1m. Corrigido para 0.
2. **Range de profundidade errado**: `40 < d < 80000` é válido para mm (16UC1) mas a imagem é decodificada como `32FC1` (metros). Corrigido para `0.04 < d < 80.0`.
3. **Filtro anti-spike em metros**: `abs(prev_dist - dist) > 300` comparava metros com limiar em mm. Corrigido para `> 0.3` metros.

### astra_common.py
- `read_HSV()` usava `"r+"` (read+write) — corrigido para `"r"`.

## Limitações
- `hsv_text` hardcoded para `/root/yahboomcar_ros2_ws/...` — falha em outras máquinas.
- `colorHSV` abre a câmara diretamente via `VideoCapture(0)` em vez de subscrever tópico ROS 2.
- FPS calculado com `start`/`end` consecutivos → sempre mostra valor inválido.
