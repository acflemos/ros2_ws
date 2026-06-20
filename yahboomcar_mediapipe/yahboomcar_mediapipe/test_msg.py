# test_msg.py — Arquivo de teste mínimo para importação de mensagens
# ===================================================================
# Verifica apenas que o pacote yahboomcar_msgs está corretamente instalado
# e que os tipos de mensagem podem ser importados. Não testa funcionalidade.
#
# Sem shebang, sem encoding — não é executável diretamente pelo entry point
# ROS2; serve apenas como smoke test de importação.
#
# Relevância para robodog2: útil para verificar setup do ambiente antes
#   de rodar os nós MediaPipe; não requer câmara nem MediaPipe.
from yahboomcar_msgs.msg import *
def main():
    print("pass")