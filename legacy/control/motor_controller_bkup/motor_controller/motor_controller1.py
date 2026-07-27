#!/usr/bin/env python
import rclpy
import math
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Twist
from can_msgs.msg import Frame
from typing import List


class MotorController(Node):

    def __init__(self):
        super().__init__('motor_controller')
        self.declare_parameter('wheel.tread')
        self.declare_parameter('wheel.diameter')
        self.declare_parameter('wheel.gear_ratio')
        self.declare_parameter('publish_timer_loop_duration')
        self.tread = self.get_parameter('wheel.tread').get_parameter_value().double_value
        self.diameter = self.get_parameter('wheel.diameter').get_parameter_value().double_value
        self.gear_ratio = self.get_parameter('wheel.gear_ratio').get_parameter_value().double_value
        publish_timer_loop_duration = self.get_parameter(
            'publish_timer_loop_duration').get_parameter_value().double_value

        buffer_size = 10
        self.twist_sub = self.create_subscription(Twist, 'sub_speed_command', self.twist_callback, buffer_size)
        self.can_pub = self.create_publisher(Frame, 'pub_can', buffer_size)
        self.publish_timer = self.create_timer(publish_timer_loop_duration, self.publish_canframe_callback)
        self.frame_msg = Frame()
        self.frame_msg.header.frame_id = "can0"        # Default can0
        self.frame_msg.id = 0x210                      # MotorControler CAN ID : 0x210
        self.frame_msg.dlc = 8 

    def twist_callback(self, msg):
        rpm = self.toRefRPM(msg.linear.x, msg.angular.z)
        cmd_right = self.toCanCmd(rpm[0])
        cmd_left = self.toCanCmd(rpm[1])
        can_data = cmd_right + cmd_left
                                # Data length
        self.frame_msg.data = can_data

    def publish_canframe_callback(self):
        self.can_pub.publish(self.frame_msg)

#  Velocity -> RPM Calc
#  V_right = (V + tread/2 * w), V_left = (V - tread / 2 * w ) [m/s]
#  w_right = V/r + (d/2r) * w   [rad/s]
#  w_left = V/r - (d/2r) * w    [rad/s]
#  rpm = w * 60 / 2* pi [rpm]
#  rpm = rpm * gear_ratio  [rpm]
    
#     def toRefRPM(self, linear_velocity, angular_velocity):  # Calc Motor ref rad/s
#         wheel_angular_velocities = np.zeros(2)

#     # === (0) ω补偿模型参数（连续三段线性模型） ===
#         #a, b, k, d = 0.03288, 0.03687, 0.36656, 1.48788
#         #y_left, y_right = b - a * d, b + a * d

#     # === (1) 根据拟合结果补偿 angular_velocity → 得到 ω_cmd ===
#  #       omega_des = angular_velocity
#         omega_cmd = angular_velocity
#        # if omega_des < y_left:
#         #    omega_cmd = (omega_des - (b - a * d)) / k - d
#         #elif omega_des > y_right:
#          #   omega_cmd = (omega_des - (b + a * d)) / k + d
#         #else:
#         # 中间线段
#          #   if abs(a) < 1e-6:
#           #      omega_cmd = omega_des  # 退化为原值
#            # else:
#             #    omega_cmd = (omega_des - b) / a

#     # === (2) 用 ω_cmd 和 v 计算目标左右轮角速度 ===
#         wheel_angular_velocities[0] = (linear_velocity / (self.diameter * 0.5)) - (
#                self.tread / self.diameter
#                  )* omega_cmd  # [rad/s]
#         wheel_angular_velocities[1] = (linear_velocity / (self.diameter * 0.5)) + (
#                self.tread / self.diameter
#                 ) * omega_cmd  # [rad/s]

#     # === (3) 左右轮各自的电机补偿（你原来的线性模型） ===
#         a_left = 0.8714179104477613
#         w0_left = 3.5196112015072383
#         a_right = 0.8543792071802537
#         w0_right = 2.6832719807757126

#     # 左轮补偿
#         if abs(wheel_angular_velocities[0]) < 1e-6:
#             wL_cmd = 0.0
#         else:
#             wL_cmd = np.sign(wheel_angular_velocities[0]) * (
#                 abs(wheel_angular_velocities[0]) / a_left + w0_left
#                 )

#     # 右轮补偿
#         if abs(wheel_angular_velocities[1]) < 1e-6:
#             wR_cmd = 0.0
#         else:
#             wR_cmd = np.sign(wheel_angular_velocities[1]) * (
#                 abs(wheel_angular_velocities[1]) / a_right + w0_right
#             )

#     # === (4) 转换为 RPM ===
#         minute_to_second = 60.0
#         rpm_left = (wL_cmd * (minute_to_second / (2.0 * np.pi))) * self.gear_ratio
#         rpm_right = (wR_cmd * (minute_to_second / (2.0 * np.pi))) * self.gear_ratio
#         rpm = np.array([rpm_left, rpm_right], dtype=float)

    
#         return rpm.tolist()

    def toRefRPM(self, linear_velocity, angular_velocity):  # Calc Motor ref rad/s
        wheel_angular_velocities = np.array([(linear_velocity / (self.diameter * 0.5)) + (self.tread / self.diameter) * angular_velocity,  # right[rad/s]
                                             (linear_velocity / (self.diameter * 0.5)) - (self.tread / self.diameter) * angular_velocity])  # left[rad/s]
       
        # wheel_angular_velocities = np.array([10, 10])
        minute_to_second = 60
        rpm = wheel_angular_velocities * (minute_to_second / (2 * math.pi))
        return (rpm * self.gear_ratio).tolist()

   # --- (1.5) 根据拟合结果直接进行补偿（反算应该给电机的输入角速度） ---
     #    a_left = 0.8714179104477613
     #    w0_left = 3.5196112015072383
     #    a_right = 0.8543792071802537
     #    w0_right = 2.6832719807757126
    #     a_left = 0.834
    #     w0_left = 2.76 
    #     a_right = 0.844
    #     w0_right = 2.81

    # # 左轮补偿
    
    #     wL_cmd = np.sign(wheel_angular_velocities[0]) * (
    #     abs(wheel_angular_velocities[0]) / a_left + w0_left)


    #     wR_cmd = np.sign(wheel_angular_velocities[1]) * (
 	#     abs(wheel_angular_velocities[1]) / a_right + w0_right)

    # # --- (2) 将补偿后的角速度转换为RPM ---
    #     minutetosecond = 60.0
    	
    #     rpm_left = (wL_cmd * (minutetosecond / (2.0 * np.pi))) * self.gear_ratio
    	 
    #     rpm_right = (wR_cmd * (minutetosecond / (2.0 * np.pi))) * self.gear_ratio
    	
    #     rpm = np.array([rpm_left, rpm_right], dtype=float)


    #     return rpm.tolist()





    @staticmethod
    def toCanCmd(rpm: float) -> List[int]:
        rounded = round(rpm)
        bytes = rounded.to_bytes(4, "little", signed=True)
        return list(bytes)


def main(args=None):
    rclpy.init(args=args)
    motor_controller = MotorController()
    rclpy.spin(motor_controller)
    motor_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
