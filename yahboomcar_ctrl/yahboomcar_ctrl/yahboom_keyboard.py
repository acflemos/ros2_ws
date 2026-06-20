#!/usr/bin/env python
# encoding: utf-8
# Controlo do robô via teclado em terminal (teleop sem joystick).
# Lê input raw do teclado via termios/tty e publica em cmd_vel.

import sys
import select
import termios
import tty

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

msg = """
Control Your SLAM-Bot!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%
t/T : x and y speed switch
s/S : stop keyboard control
space key, k : force stop
anything else : stop smoothly

CTRL-C to quit
"""

# (fator_linear, fator_angular): cada tecla define uma combinação de direção
moveBindings = {
    'i': (1, 0),    # frente
    'o': (1, -1),   # frente + rodar direita
    'j': (0, 1),    # rodar esquerda (só rotação)
    'l': (0, -1),   # rodar direita (só rotação)
    'u': (1, 1),    # frente + rodar esquerda
    ',': (-1, 0),   # trás
    '.': (-1, 1),   # trás + rodar esquerda
    'm': (-1, -1),  # trás + rodar direita
    'I': (1, 0),
    'O': (1, -1),
    'J': (0, 1),
    'L': (0, -1),
    'U': (1, 1),
    'M': (-1, -1),
}

# (fator_vel_linear, fator_vel_angular): ajuste incremental de 10%
speedBindings = {
    'q': (1.1, 1.1), 'Q': (1.1, 1.1),  # ambas +10%
    'z': (.9, .9),   'Z': (.9, .9),    # ambas -10%
    'w': (1.1, 1),   'W': (1.1, 1),    # linear +10%
    'x': (.9, 1),    'X': (.9, 1),     # linear -10%
    'e': (1, 1.1),   'E': (1, 1.1),    # angular +10%
    'c': (1, .9),    'C': (1, .9),     # angular -10%
}


class Yahboom_Keybord(Node):
    def __init__(self, name):
        super().__init__(name)
        self.pub = self.create_publisher(Twist, 'cmd_vel', 1)

        self.declare_parameter("linear_speed_limit", 1.0)
        self.declare_parameter("angular_speed_limit", 5.0)
        # CORRIGIDO: typo "linenar_speed_limit" → "linear_speed_limit"
        self.linear_speed_limit = self.get_parameter("linear_speed_limit").get_parameter_value().double_value
        self.angular_speed_limit = self.get_parameter("angular_speed_limit").get_parameter_value().double_value

        self.settings = termios.tcgetattr(sys.stdin)

    def getKey(self):
        # Lê um caractere do stdin sem bloqueio (timeout 0.1s)
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def vels(self, speed, turn):
        return "currently:\tspeed %s\tturn %s " % (speed, turn)


def main():
    rclpy.init()
    yahboom_keyboard = Yahboom_Keybord("yahboom_keyboard_ctrl")

    xspeed_switch = True  # True=publicar em linear.x, False=publicar em linear.y (tecla t)
    speed = 0.2           # velocidade linear atual
    turn = 1.0            # velocidade angular atual
    x = 0                 # fator de direção linear
    th = 0                # fator de direção angular
    status = 0            # contador para reexibir menu a cada 15 ajustes
    stop = False          # toggle: parar envio de comandos (tecla s)
    count = 0             # contador de ticks sem input válido (>4 → parar suavemente)

    try:
        print(msg)
        print(yahboom_keyboard.vels(speed, turn))
        while True:
            key = yahboom_keyboard.getKey()

            if key == "t" or key == "T":
                xspeed_switch = not xspeed_switch  # alternar entre linear.x e linear.y
            elif key == "s" or key == "S":
                stop = not stop
                print("stop keyboard control: {}".format(not stop))

            if key in moveBindings:
                x = moveBindings[key][0]
                th = moveBindings[key][1]
                count = 0
            elif key in speedBindings:
                speed = speed * speedBindings[key][0]
                turn = turn * speedBindings[key][1]
                count = 0
                if speed > yahboom_keyboard.linear_speed_limit:
                    speed = yahboom_keyboard.linear_speed_limit
                    print("Linear speed limit reached!")
                if turn > yahboom_keyboard.angular_speed_limit:
                    turn = yahboom_keyboard.angular_speed_limit
                    print("Angular speed limit reached!")
                print(yahboom_keyboard.vels(speed, turn))
                if status == 14:
                    print(msg)
                status = (status + 1) % 15
            elif key == ' ':
                x, th = 0, 0  # parar imediatamente
            else:
                count += 1
                if count > 4:
                    x, th = 0, 0  # parar suavemente após 5 ticks sem input
                if key == '\x03':  # CTRL-C
                    break

            twist = Twist()
            if xspeed_switch:
                twist.linear.x = speed * x
            else:
                twist.linear.y = speed * x  # modo mecanum Y
            twist.angular.z = turn * th

            if not stop:
                yahboom_keyboard.pub.publish(twist)
            else:
                yahboom_keyboard.pub.publish(Twist())  # publicar zeros quando parado

    except Exception as e:
        print(e)
    finally:
        yahboom_keyboard.pub.publish(Twist())  # garantir paragem ao sair

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, yahboom_keyboard.settings)
    yahboom_keyboard.destroy_node()
    rclpy.shutdown()
