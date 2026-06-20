# Imagem bruta sem compressão (alternativa leve a sensor_msgs/Image)
# Nota: não inclui encoding nem step — assume BGR contíguo (OpenCV padrão)

int32 height     # altura em pixels
int32 width      # largura em pixels
int32 channels   # canais de cor: 1=gray, 3=BGR, 4=BGRA
uint8[] data     # dados brutos: height * width * channels bytes
