# yahboom_app_save_map_client.py — Cliente de teste do serviço WebSaveMap
# ========================================================================
# Script de teste/debug para o servidor yahboom_app_save_map.py.
# Chama o serviço 'add_three_ints' (nome de serviço errado — deveria ser
# 'yahboomAppSaveMap' para coincidir com o servidor) com mapname="mapssss".
#
# ATENÇÃO — bugs conhecidos:
#   1. Nome do serviço hardcoded como 'add_three_ints' (boilerplate não atualizado)
#      → servidor usa 'yahboomAppSaveMap'; este cliente nunca conecta ao servidor.
#   2. mapname hardcoded como "mapssss" — apenas para teste manual.
#
# Este arquivo é um script de teste/desenvolvimento — não é usado em produção.
# Relevância para robodog2: padrão de cliente assíncrono ROS2 (call_async + spin_once)
#   útil como template para chamar serviços de navegação no robodog2.

from yahboom_web_savmap_interfaces.srv import WebSaveMap
import sys
import rclpy
from rclpy.node import Node


class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(WebSaveMap, 'add_three_ints')       # CHANGE
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = WebSaveMap.Request()                                   # CHANGE

    def send_request(self):
        print(sys.argv[0])
        self.req.mapname = "mapssss"                                      # CHANGE
        self.future = self.cli.call_async(self.req)


def main(args=None):
    rclpy.init(args=args)

    minimal_client = MinimalClientAsync()
    minimal_client.send_request()

    while rclpy.ok():
        rclpy.spin_once(minimal_client)
        if minimal_client.future.done():
            try:
                response = minimal_client.future.result()
            except Exception as e:
                minimal_client.get_logger().info(
                    'Service call failed %r' % (e,))
            else:
                minimal_client.get_logger().info(
                    'Result of add_three_ints: %s' %                                # CHANGE
                    (response.response))  # CHANGE
            break

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
