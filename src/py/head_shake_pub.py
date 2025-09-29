#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from lx_motor_interfaces.msg import MotorStatus

class MotorController(Node):
    def __init__(self):
        super().__init__('head_controller')

        # 声明参数
        self.declare_parameter('target_position', [3.14159])  # 头部舵机运动范围[2.61825, 3.66492]，舵机零位为3.14159
        self.declare_parameter('threshold', 0.1)    # 目标位置容错率

        # 获取参数
        self.target_positions = self.get_parameter('target_position').get_parameter_value().double_array_value
        self.target_positions = list(self.target_positions) 
        self.threshold = self.get_parameter('threshold').get_parameter_value().double_value

        # 发布者
        self.control_pub = self.create_publisher(
            MotorStatus,
            '/vmr/head_joint_control_2',
            10
        )

        # 订阅者
        self.state_sub = self.create_subscription(
            MotorStatus,
            '/vmr/head_joint_state_2',
            self.state_callback,
            10
        )

        self.reached_flags = [False] * len(self.target_positions)

        # 定时器：周期性发送目标
        self.timer = self.create_timer(0.1, self.send_control_command)

        self.get_logger().info(f'目标位置参数: {self.target_positions}, 阈值: {self.threshold}')

    def send_control_command(self):
        """发布目标位置"""
        msg = MotorStatus()
        msg.mode = [0] * 64
        msg.data = [0.0] * 64

        for i, target in enumerate(self.target_positions):
            msg.mode[i] = 1
            msg.data[i] = target
            
        self.control_pub.publish(msg)

    def state_callback(self, msg: MotorStatus):
        """接收电机状态并计算偏差"""
        for i, target in enumerate(self.target_positions):
            current = msg.data[i]
            error = target - current

            self.get_logger().info(
                f'关节{i}: 当前={current:.2f}, 目标={target:.2f}, 偏差={error:.2f}'
            )

            if abs(error) < self.threshold:
                if not self.reached_flags[i]:
                    self.reached_flags[i] = True
                    self.get_logger().info(f'关节{i} 已到达目标位置!')
            else:
                self.reached_flags[i] = False

        # 检查是否所有关节都到达
        if all(self.reached_flags):
            self.get_logger().info("所有关节均已到达目标位置，程序即将退出")
            exit(0)  # 退出程序

def main(args=None):
    rclpy.init(args=args)
    node = MotorController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

# 运行指令
# python3 head_pose_pub.py --ros-args -p target_position:="[3.1415926]" -p threshold:=0.1
