# Alvo detetado por visão computacional (bounding box + metadados)
# Publicado por: yahboomcar_visual, yahboomcar_KCFTracker, yahboomcar_mediapipe

string frame_id                  # frame da câmara (ex: "camera_link")
builtin_interfaces/Time stamp    # timestamp da deteção
float32 scores                   # confiança da deteção [0.0, 1.0]
float32 ptx                      # canto superior-esquerdo da bbox — pixel X
float32 pty                      # canto superior-esquerdo da bbox — pixel Y
float32 distw                    # largura da bounding box em pixels
float32 disth                    # altura da bounding box em pixels
float32 centerx                  # centro da bbox — pixel X (ptx + distw/2)
float32 centery                  # centro da bbox — pixel Y (pty + disth/2)
