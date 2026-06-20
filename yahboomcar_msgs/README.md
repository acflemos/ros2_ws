# yahboomcar_msgs

Mensagens ROS 2 customizadas do Yahboom ROSMaster X3. Este pacote não contém nós — só definições de tipos de mensagem usados pelos outros pacotes do workspace.

## Mensagens

### Target
Alvo detetado por visão computacional com bounding box e metadados.
```
string frame_id       # frame da câmara (ex: "camera_link")
Time stamp            # timestamp da deteção
float32 scores        # confiança [0.0, 1.0]
float32 ptx, pty      # canto superior-esquerdo da bbox em pixels
float32 distw, disth  # dimensões da bbox em pixels
float32 centerx, centery  # centro da bbox em pixels
```
Publicado por: `yahboomcar_visual`, `yahboomcar_KCFTracker`, `yahboomcar_mediapipe`

### TargetArray
Lista de `Target[]` para transmitir múltiplas deteções numa frame (ex: YOLO multi-class).

### ImageMsg
Imagem bruta sem compressão — alternativa leve a `sensor_msgs/Image`.  
**Atenção:** não inclui `encoding` nem `step`; assume BGR contíguo (formato padrão OpenCV).

### Position
Posição de um alvo. **Os campos têm semântica dupla consoante o pacote consumidor:**
```
float32 anglex    # yahboomcar_astra: pixel_x do centroide  | intent original: desvio horizontal em graus
float32 angley    # yahboomcar_astra: pixel_y do centroide  | intent original: desvio vertical em graus
float32 distance  # yahboomcar_astra: raio do círculo (px)  | intent original: distância em metros
```
Em `yahboomcar_astra`, estes campos são reutilizados para transportar a posição 2D do objeto em píxeis
(saída de `colorHSV` → entrada de `colorTracker`). Noutros contextos, a semântica original de ângulos/metros aplica-se.

### PointArray
Lista de `geometry_msgs/Point[]` para publicar obstáculos ou pontos de interesse extraídos de nuvem de pontos.

## Dependências
- `geometry_msgs` — tipo `Point` usado em `PointArray`
- `std_msgs` — incluído mas não usado diretamente nas msgs atuais
- `builtin_interfaces` — tipo `Time` em `Target`

## Uso no robodog2
Para usar estas mensagens em robodog2, adicionar ao `package.xml`:
```xml
<depend>yahboomcar_msgs</depend>
```
E ao `CMakeLists.txt`:
```cmake
find_package(yahboomcar_msgs REQUIRED)
ament_target_dependencies(... yahboomcar_msgs)
```
