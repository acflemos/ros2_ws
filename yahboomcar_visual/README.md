# yahboomcar_visual

Captura, processamento e visualização de imagens para o ROSMaster X3.  
Suporta câmara Astra (RGB + Depth), LIDAR bird's-eye view e realidade aumentada com checkerboard.

## Executáveis

| Executável | Descrição |
|-----------|-----------|
| `astra_rgb_image` | Subscreve `/camera/color/image_raw` e exibe com `cv.imshow` |
| `astra_depth_image` | Subscreve `/camera/depth/image_raw` (32FC1) e exibe |
| `astra_image_flip` | Recebe imagem comprimida, faz flip horizontal, republica |
| `astra_color_point` | Sincroniza RGB + Depth (ApproximateTime) para processamento conjunto |
| `pub_image` | Pipe: subscreve `/image_raw`, redimensiona para 640×480, publica `/image` |
| `laser_to_image` | Converte LaserScan em imagem bird's-eye (1600×1200) |
| `simple_AR` | Realidade aumentada com deteção de checkerboard 6×9 e sobreposição 3D |

## Tópicos

| Nó | Sub/Pub | Tópico | Tipo |
|----|---------|--------|------|
| `astra_rgb_image` | Sub | `/camera/color/image_raw` | `sensor_msgs/Image` |
| `astra_depth_image` | Sub | `/camera/depth/image_raw` | `sensor_msgs/Image` (32FC1) |
| `astra_image_flip` | Sub | `/image_raw/compressed` | `CompressedImage` |
| `astra_image_flip` | Pub | `/image_flip/compressed` | `CompressedImage` |
| `astra_color_point` | Sub | `/camera/color/image_raw` + `/camera/depth/image_raw` | sincronizados |
| `pub_image` | Sub | `/image_raw` | `Image` |
| `pub_image` | Pub | `/image` | `Image` |
| `laser_to_image` | Sub | `/scan` | `sensor_msgs/LaserScan` |
| `laser_to_image` | Pub | `/laserImage` | `sensor_msgs/Image` |
| `simple_AR` | Sub | `/Graphics_topic` | `std_msgs/String` |
| `simple_AR` | Pub | `/simpleAR/camera` | `sensor_msgs/Image` |

## Calibração da câmara (astra.yaml)

```
fx=615.50, fy=623.69, cx=365.84, cy=238.78  (640×480)
Distorção plumb_bob: [0.166, -0.160, -0.009, 0.025, 0.0]
```

**NOTA:** valores diferentes dos usados em `yahboomcar_slam` (fx≈517) — câmaras Astra diferentes ou re-calibrações. Confirmar qual corresponde ao hardware real.

## laser_to_image — algoritmo bird's-eye

```
LaserScan → LaserProjection.projectLaser() → PointCloud2
→ projeção: x_img = -y * 80 + 500
            y_img = -x * 80 + 500  (escala: 80 px/m, origem no centro)
→ intensidade = Z normalizado [-2, 2] m → [0, 500]
→ imagem 1600×1200 uint8
```

## simple_AR — realidade aumentada

1. Detecta padrão checkerboard 6×9 com `cv.findChessboardCorners`
2. Refina com `cv.cornerSubPix`
3. Estima pose com `cv.solvePnPRansac`
4. Projeta pontos 3D para imagem com `cv.projectPoints`
5. Desenha um de 12 gráficos pré-definidos sobre o padrão

**Parâmetros câmara:** carregados de `/root/yahboomcar_ros2_ws/.../astra.yaml` — caminho hardcoded.

## Limitações
- `simple_AR` usa `VideoCapture(0)` diretamente (não subscreve tópico ROS 2).
- Todos os caminhos de ficheiro hardcoded para `/root/yahboomcar_ros2_ws/...`.
- Sem launch files para os nós individuais.
- `detection/` (SSD MobileNet v2), `simple_qrcode/` e `Marker/` são scripts standalone não integrados em nós ROS 2.
