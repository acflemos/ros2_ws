# yahboomcar_mediapipe

Deteção de poses humanas, mãos e rostos usando Google MediaPipe, com publicação de landmarks em ROS 2.

## Executáveis (registados como nós ROS 2)

| Executável | Deteção | Pontos publicados |
|-----------|---------|-------------------|
| `01_HandDetector` | Mãos (até 2) | 21 landmarks por mão |
| `02_PoseDetector` | Corpo completo | 33 landmarks corporais |
| `03_Holistic` | Face + Corpo + Mãos | 468 + 33 + 21 + 21 = 543 pontos |
| `04_FaceMesh` | Face mesh (até 2 rostos) | 468 landmarks por rosto |
| `05_FaceEyeDetection` | Rosto + Olhos (Haar) | Publica em `/FaceEyeDetection/image` |

## Scripts standalone (não registados, sem nó ROS 2)

| Ficheiro | Descrição |
|---------|-----------|
| `06_FaceLandmarks.py` | 68 landmarks faciais com DLIB |
| `07_FaceDetection.py` | Bounding box de rosto com MediaPipe |
| `08_Objectron.py` | Deteção 3D de objetos (caixa, cadeira, etc.) |
| `09_VirtualPaint.py` | Pintura virtual com gestos de mão |
| `10_HandCtrl.py` | Controlo de efeitos de imagem por ângulo do polegar |
| `11_GestureRecognition.py` | Reconhecimento de gestos personalizados |

## Tópicos

| Direção | Nome | Tipo | Descrição |
|---------|------|------|-----------|
| Pub | `/mediapipe/points` | `yahboomcar_msgs/PointArray` | Landmarks em coordenadas normalizadas [0,1] |
| Pub | `/FaceEyeDetection/image` | `sensor_msgs/Image` | Frame com deteções desenhadas (só nó 05) |

**NOTA:** Todos os nós usam `VideoCapture(0)` diretamente — não subscrevem tópico ROS 2 de imagem.

## Launch

`mediapipe_points.launch.py` lança:
1. `yahboomcar_point/pub_point` — publicador de pontos 3D
2. `rviz2` com configuração `mediapipe_points.rviz`

## Dependências externas

```
mediapipe   # Google MediaPipe (pip install mediapipe)
opencv-python
numpy
dlib        # só para 06_FaceLandmarks.py
```

## Limitações

- Nós 06-11 não estão registados em `setup.py` — não são lançáveis via `ros2 run`.
- Caminhos Haar cascade hardcoded relativos ao ficheiro fonte.
- Sem parâmetros ROS 2 dinâmicos para ajustar confiança de deteção.
- Coordenadas MediaPipe são normalizadas [0,1] — para usar em controlo robótico precisam de ser convertidas para píxeis.
