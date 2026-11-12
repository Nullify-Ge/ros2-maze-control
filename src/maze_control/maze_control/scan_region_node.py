#!/usr/bin/env python3
"""激光雷达分区测距节点。

功能
    订阅 /scan 后将 360° 扫描点按角度划分为前方、左侧、右侧三个扇区，分别提取
    代表距离并周期打印，为后续避障与墙跟随算法提供分区距离输入。

订阅
    /scan       sensor_msgs/msg/LaserScan  激光雷达扫描数据
输出
    终端日志：前方 / 左侧 / 右侧距离

分区定义（机器人坐标系，0° 为正前方，逆时针为正）
    前方  -15° ~ +15°   取最小值，代表最近障碍
    左侧  +60° ~ +90°   取中值，抑制单点噪声
    右侧  -90° ~ -60°   取中值，抑制单点噪声

实现说明
    TurtleBot3 所用雷达的 angle_min 为 0，扫描角均落在 [0, 2*pi) 区间内，不存在
    负角度。因此负角度扇区须先折算到 [0, 360)，例如右侧 -90°~-60° 对应 270°~300°；
    否则该扇区无法取到任何点，距离将恒等于 range_max。

运行
    ros2 run maze_control scan_region
"""
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class ScanRegionFilter(Node):
    """雷达三分区测距节点。"""

    def __init__(self):
        super().__init__('scan_region_filter_node')
        self.sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data)

        # 扇区角度配置，单位：度，按 [0, 360) 的原始角度书写
        self.front_min_deg = -15
        self.front_max_deg = 15
        self.left_min_deg = 60
        self.left_max_deg = 90
        self.right_min_deg = -90
        self.right_max_deg = -60

        self.get_logger().info(
            "区域雷达节点启动：前方[-15,15]°, 左[60,90]°, 右[-90,-60]°")

    def scan_callback(self, msg: LaserScan):
        """对一帧扫描数据做分区统计并打印。"""
        ranges = np.array(msg.ranges, dtype=np.float64)

        # 有效点：非 nan / inf，且落在雷达量程内
        valid = np.isfinite(ranges) & (ranges > msg.range_min) & (ranges <= msg.range_max)

        # 每个点对应的角度（度），统一折算到 [0, 360)
        deg = np.degrees(msg.angle_min + np.arange(len(ranges)) * msg.angle_increment)
        deg = np.mod(deg, 360.0)

        def in_sector(lo, hi):
            """返回角度区间 [lo, hi] 的掩码；lo < hi 不成立时按跨 0° 处理。"""
            lo %= 360.0
            hi %= 360.0
            if lo <= hi:
                return (deg >= lo) & (deg <= hi)
            return (deg >= lo) | (deg <= hi)

        m_front = in_sector(self.front_min_deg, self.front_max_deg) & valid
        m_left = in_sector(self.left_min_deg, self.left_max_deg) & valid
        m_right = in_sector(self.right_min_deg, self.right_max_deg) & valid

        # 前方取最小值（最近障碍），左右取中值（抗单点噪声）
        front = np.min(ranges[m_front]) if m_front.any() else msg.range_max
        left = np.median(ranges[m_left]) if m_left.any() else msg.range_max
        right = np.median(ranges[m_right]) if m_right.any() else msg.range_max

        self.get_logger().info(
            f"【距离】前方: {front:.2f} m | 左侧: {left:.2f} m | 右侧: {right:.2f} m")


def main(args=None):
    rclpy.init(args=args)
    node = ScanRegionFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
