#!/usr/bin/env python3
"""航点序列跟踪节点。

功能
    在点到点控制的基础上扩展为多点序列：车辆依次驶向预设航点，到达当前航点后自动
    切换至下一航点，全部航点完成后停车。同时将实际行驶轨迹以 nav_msgs/Path 形式发布，
    便于在 RViz2 中观察轨迹形状。

订阅
    /odom        nav_msgs/msg/Odometry     车辆位置与姿态
发布
    /cmd_vel     geometry_msgs/msg/Twist   速度指令
    /trajectory  nav_msgs/msg/Path         实际行驶轨迹（frame_id = odom）

控制律
    与点到点控制一致，仅在到达判定成立时执行航点索引自增；
    航点序列耗尽后发布零速度并置完成标志，避免重复打印。

运行
    ros2 run maze_control tb3_waypoints
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry, Path
from math import sqrt, atan2, sin, cos


class WaypointController(Node):
    """多点序列跟踪控制器。"""

    def __init__(self):
        super().__init__('tb3_waypoints')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.path_pub = self.create_publisher(Path, '/trajectory', 10)

        # 机器人当前位姿
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # 控制参数
        self.Kp_dist = 0.3
        self.Kp_ang = 0.8
        self.max_v = 0.2
        self.max_w = 1.0
        self.threshold = 0.05   # 航点到达切换阈值，单位 m

        # 矩形航点列表，可按实验要求修改
        self.waypoints = [(1.0,0.0), (1.0,1.0), (0.0,1.0), (0.0,0.0)]
        self.wp_index = 0

        # 轨迹发布相关状态
        self.path = Path()
        self.path.header.frame_id = 'odom'
        self.finished = False

        # 控制循环定时器，周期 0.05 s，对应 20 Hz
        self.timer = self.create_timer(0.05, self.control_loop)

    def odom_callback(self, msg):
        """更新车辆位置，并将姿态四元数解算为航向角。"""
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.theta = atan2(siny_cosp, cosy_cosp)

    def record_pose(self):
        """把当前位姿追加到轨迹序列并发布。

        以四元数形式只保留偏航分量（z、w），足以表达平面运动姿态；
        轨迹点数量超过 20000 时丢弃最早的点，防止消息无限增长。
        """
        ps = PoseStamped()
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.header.frame_id = 'odom'
        ps.pose.position.x = self.x
        ps.pose.position.y = self.y
        ps.pose.orientation.z = sin(self.theta / 2.0)
        ps.pose.orientation.w = cos(self.theta / 2.0)
        self.path.poses.append(ps)
        if len(self.path.poses) > 20000:
            self.path.poses.pop(0)
        self.path.header.stamp = ps.header.stamp
        self.path_pub.publish(self.path)

    def control_loop(self):
        """按固定周期计算并发布速度指令，同时推进航点索引。"""
        self.record_pose()
        if self.wp_index >= len(self.waypoints):
            # 全部航点完成：仅打印一次，随后持续输出零速度
            if not self.finished:
                self.get_logger().info("全部航点完成，停车")
                self.finished = True
            self.cmd_pub.publish(Twist())
            return
        gx, gy = self.waypoints[self.wp_index]
        dx = gx - self.x
        dy = gy - self.y
        dist_err = sqrt(dx**2 + dy**2)
        theta_des = atan2(dy, dx)
        theta_err = theta_des - self.theta
        # 角度归一化到 [-pi, pi]
        theta_err = atan2(sin(theta_err), cos(theta_err))

        twist = Twist()
        if dist_err > self.threshold:
            twist.linear.x = self.Kp_dist * dist_err
            twist.angular.z = self.Kp_ang * theta_err
            # 速度限幅，防止剧烈震荡
            twist.linear.x = max(-self.max_v, min(self.max_v, twist.linear.x))
            twist.angular.z = max(-self.max_w, min(self.max_w, twist.angular.z))
        else:
            self.get_logger().info(
                f"到达航点{self.wp_index}:({gx},{gy})，切换下一个航点")
            self.wp_index += 1
        self.cmd_pub.publish(twist)


def main():
    rclpy.init()
    node = WaypointController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
