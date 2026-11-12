#!/usr/bin/env python3
"""反应式避障节点。

功能
    依据雷达前方扇区的最小距离作出反应式决策：前方存在近距障碍时原地转向较空旷
    一侧，前方畅通时保持直行；若某一侧墙过近则施加小幅角速度向外侧修正。
    本节点不依赖地图与路径规划，属于典型的反应式（reactive）避障方法。

订阅
    /scan       sensor_msgs/msg/LaserScan  激光雷达扫描数据
发布
    /cmd_vel    geometry_msgs/msg/Twist    速度指令

决策流程
    1. 未收到任何激光数据            -> 停车（避免无传感器输入时盲目运动）
    2. 处于避让锁存状态              -> 继续转向，直至前方距离大于 1.4 倍安全距离
    3. 前方距离小于安全距离          -> 置避让锁存，向左右较空旷的一侧原地转向
    4. 前方畅通                      -> 直行；某侧墙过近时叠加小角速度修正

实现说明
    (1) 转向锁存：进入避让后须转至前方充分空旷才恢复直行，否则会在阈值附近
        反复切换状态，形成原地抖动。
    (2) 空测量值的语义：空旷区域雷达返回 inf，表示“前方很远”，不能等同于
        “没有数据”；是否收到过扫描帧由 got_scan 标志单独判断。

运行
    ros2 run maze_control tb3_avoid
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


def wrap(a):
    """将角度归一化至 (-pi, pi]。"""
    while a > math.pi:
        a -= 2.0 * math.pi
    while a <= -math.pi:
        a += 2.0 * math.pi
    return a


class Tb3Avoid(Node):
    """基于雷达前方距离的反应式避障节点。"""

    def __init__(self):
        super().__init__('tb3_avoid')

        d = self.declare_parameter
        d('safe_distance', 0.70)    # 前方避让触发距离，单位 m
        d('forward_speed', 0.15)    # 直行线速度，单位 m/s
        d('turn_speed', 0.60)       # 避让角速度，单位 rad/s
        d('front_deg', 20.0)        # 前方扇区半角，单位 度
        d('side_deg', 55.0)         # 侧前方扇区中心角，用于比较左右空旷程度
        d('side_keep', 0.35)        # 侧向最小保持距离，小于该值则向外修正
        d('max_range', 3.5)         # 有效测距上限，单位 m

        g = lambda n: self.get_parameter(n).value
        self.safe = g('safe_distance')
        self.v = g('forward_speed')
        self.w_turn = g('turn_speed')
        self.front_deg = math.radians(g('front_deg'))
        self.side_deg = math.radians(g('side_deg'))
        self.side_keep = g('side_keep')
        self.max_range = g('max_range')

        # ---- 状态量 ----
        self.got_scan = False
        self.front = float('inf')
        self.left = float('inf')
        self.right = float('inf')
        self.avoiding = False       # 转向锁存标志
        self.turn_dir = 1.0         # 转向方向：+1 左转，-1 右转

        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(
            LaserScan, 'scan', self.scan_cb, qos_profile_sensor_data)
        self.create_timer(0.05, self.loop)      # 控制周期 20 Hz
        self.create_timer(1.0, self.report)     # 状态打印周期 1 Hz

        self.get_logger().info(
            '反应式避障已启动：safe=%.2f m, v=%.2f m/s' % (self.safe, self.v))

    def scan_cb(self, msg):
        """按扇区提取前方、左侧、右侧的最小有效距离。"""
        front, left, right = [], [], []

        for i, r in enumerate(msg.ranges):
            # 过滤无效测量：非有限值、超出量程或无效近距值
            if not math.isfinite(r) or r < msg.range_min or r > self.max_range:
                continue
            a = wrap(msg.angle_min + i * msg.angle_increment)
            if abs(a) <= self.front_deg:
                front.append(r)
            elif abs(a - self.side_deg) <= math.radians(20.0):
                left.append(r)
            elif abs(a + self.side_deg) <= math.radians(20.0):
                right.append(r)

        self.front = min(front) if front else float('inf')
        self.left = min(left) if left else float('inf')
        self.right = min(right) if right else float('inf')
        self.got_scan = True

    def loop(self):
        """按固定周期执行决策并发布速度指令。"""
        if not self.got_scan:
            self.stop()
            return

        if self.avoiding:
            # 锁存生效：持续转向，直至前方充分空旷
            if self.front > self.safe * 1.4:
                self.avoiding = False
            else:
                self.publish(0.0, self.turn_dir * self.w_turn)
                return

        if self.front < self.safe:
            # 前方存在近距障碍：向较空旷的一侧原地转向
            self.avoiding = True
            self.turn_dir = 1.0 if self.left >= self.right else -1.0
            self.publish(0.0, self.turn_dir * self.w_turn)
            return

        # 前方畅通：直行，侧向过近时叠加向外修正的角速度
        w = 0.0
        if self.left < self.side_keep:
            w = -0.35       # 左侧过近，向右修正
        elif self.right < self.side_keep:
            w = 0.35        # 右侧过近，向左修正

        self.publish(self.v, w)

    def publish(self, v, w):
        """发布速度指令。"""
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)
        self.pub.publish(msg)

    def stop(self):
        """发布零速度使车辆停止。"""
        self.publish(0.0, 0.0)

    def report(self):
        """周期打印当前决策状态与分区距离。"""
        if not self.got_scan:
            return
        state = '避让' if self.avoiding else '直行'
        self.get_logger().info(
            '【%s】前 %s 左 %s 右 %s m' % (state, _fmt(self.front),
                                          _fmt(self.left), _fmt(self.right)))


def _fmt(x):
    """无有效测距时以 -- 显示。"""
    return '--' if not math.isfinite(x) else '%.2f' % x


def main(args=None):
    rclpy.init(args=args)
    node = Tb3Avoid()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
