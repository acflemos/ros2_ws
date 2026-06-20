#!/usr/bin/env python
# encoding: utf-8
# Controlo do ROSMaster R2 via joystick (cinemática Ackermann — steering frontal).
# linear.y é usado como ângulo de steering (graus), não como velocidade lateral.
# Deteta automaticamente Jetson (user=root) ou PC remoto.

import time
import getpass

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from actionlib_msgs.msg import GoalID
from std_msgs.msg import Int32, Bool


class JoyTeleop(Node):
    def __init__(self, name):
        super().__init__(name)
        self.Joy_active = False
        self.Buzzer_active = False
        self.RGBLight_index = 0
        self.cancel_time = time.time()
        self.user_name = getpass.getuser()
        self.linear_Gear = 1
        self.angular_Gear = 1

        self.pub_goal = self.create_publisher(GoalID, "move_base/cancel", 10)
        self.pub_cmdVel = self.create_publisher(Twist, 'cmd_vel', 10)
        self.pub_Buzzer = self.create_publisher(Bool, "Buzzer", 1)
        self.pub_JoyState = self.create_publisher(Bool, "JoyState", 10)
        self.pub_RGBLight = self.create_publisher(Int32, "RGBLight", 10)

        # QoS=1 no R2 (vs 10 no X3) — menos buffer, menor latência para steering
        self.sub_Joy = self.create_subscription(Joy, 'joy', self.buttonCallback, 1)

        self.declare_parameter('xspeed_limit', 1.0)
        self.declare_parameter('yspeed_limit', 5.0)   # R2: yspeed é steering angle (graus)
        self.declare_parameter('angular_speed_limit', 5.0)
        self.xspeed_limit = self.get_parameter('xspeed_limit').get_parameter_value().double_value
        self.yspeed_limit = self.get_parameter('yspeed_limit').get_parameter_value().double_value
        self.angular_speed_limit = self.get_parameter('angular_speed_limit').get_parameter_value().double_value

    def buttonCallback(self, joy_data):
        if not isinstance(joy_data, Joy):
            return
        if self.user_name == "root":
            self.user_jetson(joy_data)
        else:
            self.user_pc(joy_data)

    def user_jetson(self, joy_data):
        # Joystick Yahboom (Jetson). R2 usa buttons[9] para toggle (vs axes[9] no X3).

        # buttons[9]: toggle Joy_active
        if joy_data.buttons[9] == 1:
            self.cancel_nav()

        # buttons[7]: ciclar cor RGB
        if joy_data.buttons[7] == 1:
            rgb_msg = Int32()
            if self.RGBLight_index < 6:
                rgb_msg.data = self.RGBLight_index
                for _ in range(3):
                    self.pub_RGBLight.publish(rgb_msg)
            else:
                self.RGBLight_index = 0
            self.RGBLight_index += 1

        # buttons[11]: toggle buzzer
        if joy_data.buttons[11] == 1:
            self.Buzzer_active = not self.Buzzer_active
            buzzer_msg = Bool()
            buzzer_msg.data = self.Buzzer_active
            for _ in range(3):
                self.pub_Buzzer.publish(buzzer_msg)

        # buttons[13]: ciclar marcha linear
        if joy_data.buttons[13] == 1:
            if self.linear_Gear == 1.0:
                self.linear_Gear = 1.0 / 3
            elif self.linear_Gear == 1.0 / 3:
                self.linear_Gear = 2.0 / 3
            elif self.linear_Gear == 2.0 / 3:
                self.linear_Gear = 1.0

        # buttons[14]: ciclar marcha angular
        if joy_data.buttons[14] == 1:
            if self.angular_Gear == 1.0:
                self.angular_Gear = 1.0 / 4
            elif self.angular_Gear == 1.0 / 4:
                self.angular_Gear = 1.0 / 2
            elif self.angular_Gear == 1.0 / 2:
                self.angular_Gear = 3.0 / 4
            elif self.angular_Gear == 3.0 / 4:
                self.angular_Gear = 1.0

        # axes[1]=velocidade forward, axes[2]=steering angle (Ackermann)
        # CORRIGIDO: código original usava axes[2] para AMBOS ylinear e angular —
        # angular_speed calculado mas nunca publicado (twist.angular.z comentado no R2)
        xlinear_speed = self.filter_data(joy_data.axes[1]) * self.xspeed_limit * self.linear_Gear
        ylinear_speed = self.filter_data(joy_data.axes[2]) * self.yspeed_limit * self.linear_Gear  # steering

        xlinear_speed = max(-self.xspeed_limit, min(self.xspeed_limit, xlinear_speed))
        ylinear_speed = max(-self.yspeed_limit, min(self.yspeed_limit, ylinear_speed))

        if self.Joy_active:
            twist = Twist()
            twist.linear.x = xlinear_speed
            twist.linear.y = ylinear_speed  # interpretado como steering angle pelo base_node_R2
            # angular.z não publicado: base_node_R2 calcula-o internamente a partir do steering
            for _ in range(3):
                self.pub_cmdVel.publish(twist)

    def user_pc(self, joy_data):
        # Joystick genérico ligado ao PC.

        if joy_data.axes[5] == -1:
            self.cancel_nav()

        if joy_data.buttons[5] == 1:
            if self.RGBLight_index < 6:
                # CORRIGIDO: publicar Int32() em vez de int nativo
                self.pub_RGBLight.publish(Int32(data=self.RGBLight_index))
            else:
                self.RGBLight_index = 0
            self.RGBLight_index += 1

        if joy_data.buttons[7] == 1:
            self.Buzzer_active = not self.Buzzer_active
            # CORRIGIDO: publicar Bool() em vez de bool nativo
            self.pub_Buzzer.publish(Bool(data=self.Buzzer_active))

        if joy_data.buttons[9] == 1:
            if self.linear_Gear == 1.0:
                self.linear_Gear = 1.0 / 3
            elif self.linear_Gear == 1.0 / 3:
                self.linear_Gear = 2.0 / 3
            elif self.linear_Gear == 2.0 / 3:
                self.linear_Gear = 1.0

        if joy_data.buttons[10] == 1:
            if self.angular_Gear == 1.0:
                self.angular_Gear = 1.0 / 4
            elif self.angular_Gear == 1.0 / 4:
                self.angular_Gear = 1.0 / 2
            elif self.angular_Gear == 1.0 / 2:
                self.angular_Gear = 3.0 / 4
            elif self.angular_Gear == 3.0 / 4:
                self.angular_Gear = 1.0

        # axes[1]=forward, axes[0]=steering (Y), axes[3]=angular (stick direito)
        xlinear_speed = self.filter_data(joy_data.axes[1]) * self.xspeed_limit * self.linear_Gear
        ylinear_speed = self.filter_data(joy_data.axes[0]) * self.yspeed_limit * self.linear_Gear
        angular_speed = self.filter_data(joy_data.axes[3]) * self.angular_speed_limit * self.angular_Gear

        xlinear_speed = max(-self.xspeed_limit, min(self.xspeed_limit, xlinear_speed))
        ylinear_speed = max(-self.yspeed_limit, min(self.yspeed_limit, ylinear_speed))
        angular_speed = max(-self.angular_speed_limit, min(self.angular_speed_limit, angular_speed))

        twist = Twist()
        twist.linear.x = xlinear_speed
        twist.linear.y = ylinear_speed
        twist.angular.z = angular_speed
        for _ in range(3):
            self.pub_cmdVel.publish(twist)

    def filter_data(self, value):
        if abs(value) < 0.2:
            value = 0
        return value

    def cancel_nav(self):
        now_time = time.time()
        if now_time - self.cancel_time > 1:
            self.Joy_active = not self.Joy_active
            joy_msg = Bool()
            joy_msg.data = self.Joy_active
            for _ in range(3):
                self.pub_JoyState.publish(joy_msg)
                self.pub_cmdVel.publish(Twist())
            self.cancel_time = now_time


def main():
    rclpy.init()
    joy_ctrl = JoyTeleop('joy_ctrl')
    rclpy.spin(joy_ctrl)
