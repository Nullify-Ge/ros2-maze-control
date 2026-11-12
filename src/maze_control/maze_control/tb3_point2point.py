#!/usr/bin/env python3
"""点到点 P 控制节点。

功能
    以里程计位姿作为反馈量，采用比例控制律驱动 TurtleBot3 由当前位姿驶向固定目标点；
    当距离误差小于到达阈值时输出零速度，实现闭环运动控制。

订阅
    /odom       nav_msgs/msg/Odometry    车辆位置与姿态
发布
    /cmd_vel    geometry_msgs/msg/Twist  速度指令

控制律
    e_d = || p_goal - p ||                      距离误差
    e_t = wrap(atan2(dy, dx) - theta)           航向误差，归一化至 [-pi, pi]
    v   = clamp(Kp_dist * e_d, -max_v, max_v)   线速度
    w   = clamp(Kp_ang  * e_t, -max_w, max_w)   角速度
    e_d < goal_threshold 时输出零速度，并给出一次到达提示。

运行
    ros2 run maze_control tb3_point2point
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import sqrt, atan2, pi, sin, cos


class Point2PointController(Node):
    """点到点比例控制器，目标点在构造时给定。"""

    def __init__(self):
        super().__init__('tb3_point2point')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # 机器人当前位姿
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # 控制参数
        self.Kp_dist = 0.3      # 距离比例增益
        self.Kp_ang = 0.8       # 角度比例增益
        self.max_v = 0.2        # 最大线速度，单位 m/s
        self.max_w = 1.0        # 最大角速度，单位 rad/s
        self.goal_threshold = 0.05  # 到达判定阈值，单位 m

        # 目标点，可按实验要求修改
        self.goal_x = 1.0
        self.goal_y = 0.0

        # 控制循环定时器，周期 0.05 s，对应 20 Hz
        self.timer = self.create_timer(0.05, self.control_loop)

    def odom_callback(self, msg):
        """更新车辆位置，并将姿态四元数解算为航向角。"""
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        # 四元数转航向角 yaw
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.theta = atan2(siny_cosp, cosy_cosp)

    def control_loop(self):
        """按固定周期计算并发布速度指令。"""
        dx = self.goal_x - self.x
        dy = self.goal_y - self.y
        dist_err = sqrt(dx**2 + dy**2)
        theta_des = atan2(dy, dx)
        theta_err = theta_des - self.theta
        # 角度归一化到 [-pi, pi]
        theta_err = atan2(sin(theta_err), cos(theta_err))

        twist = Twist()
        if dist_err > self.goal_threshold:
            twist.linear.x = self.Kp_dist * dist_err
            twist.angular.z = self.Kp_ang * theta_err
            # 速度限幅，防止剧烈震荡
            twist.linear.x = max(-self.max_v, min(self.max_v, twist.linear.x))
            twist.angular.z = max(-self.max_w, min(self.max_w, twist.angular.z))
        else:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.get_logger().info(f"到达目标点({self.goal_x},{self.goal_y})，停止运动")
        self.cmd_pub.publish(twist)


def main():
    rclpy.init()
    node = Point2PointController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
