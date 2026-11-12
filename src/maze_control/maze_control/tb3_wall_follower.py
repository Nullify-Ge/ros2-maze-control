#!/usr/bin/env python3
"""迷宫左墙跟随控制节点。

功能
    订阅激光雷达与里程计数据，按“左侧贴墙”策略（左手定则）驱动 TurtleBot3 由迷宫入口
    沿墙行至出口，到达目标点后自动停车。节点同时发布行驶轨迹，供 RViz2 可视化。

订阅
    /scan        sensor_msgs/msg/LaserScan     激光雷达扫描数据
    /odom        nav_msgs/msg/Odometry         里程计位姿
发布
    /cmd_vel     geometry_msgs/msg/Twist       速度指令
    /trajectory  nav_msgs/msg/Path             实际行驶轨迹（frame_id = odom）

状态机（四段式）
    1. 转角锁存 PIVOT      前方或左前方距离小于 front_danger 时触发，记录目标朝向，
                           原地转至目标朝向（默认 80°）后解锁，并冷却 1 s。
                           转向方向由左右两侧空旷程度比较确定，兼顾 T 型路口与死胡同。
    2. 左墙丢失 REACQUIRE  左侧距离大于 left_lost 时，以低速向左转向重新贴墙；
                           持续超过 1.5 s 仍未找到墙面，判定进入开阔区域并直行。
    3. 沿墙行驶 FOLLOW     对左侧点云做最小二乘直线拟合，得到垂距 d 与车身-墙面夹角
                           psi，控制律为
                               w = kp * (d_smooth - target_left) + k_psi * psi
                           d_smooth 为垂距的一阶平滑值；线速度按转向幅度衰减，
                               v = v_nom * (1 - 0.45 * |w| / w_max)
    4. 终止停车 STOP       到达目标点、越界或进入终止状态后持续输出零速度。

雷达分区（车体坐标系：0° 为正前方，逆时针为正）
    前方    -front_half ~ +front_half          取最小值，最早发现障碍
    左前方  +left_front_min ~ +left_front_max  取最小值，提前感知转角
    左侧    +left_min ~ +left_max              取最小值，用于判断是否存在墙面
    正左方  +perp_min ~ +perp_max              取平均值，代表实际垂距

退出条件
    到达出口：与 goal_x / goal_y 的距离小于 goal_tol（未设置时退化为 x > exit_x）
    越界保护：|x| > bounds 或 |y| > bounds

运行
    ros2 run maze_control tb3_wall_follower
    或将参数交由 launch 文件统一传入，参见 tb3_maze.launch.py
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan


class LeftWallFollower(Node):
    """基于雷达墙面拟合的左墙跟随控制器。"""

    def __init__(self):
        super().__init__('tb3_wall_follower')

        # ---- 可调参数（亦可通过 ros2 param 在线修改）----
        self.declare_parameter('target_left', 0.55)     # 期望左侧墙距，单位 m
        self.declare_parameter('front_danger', 0.45)    # 前方危险距离，单位 m
        self.declare_parameter('left_lost', 1.60)       # 超过该值判定左侧无墙，单位 m
        self.declare_parameter('v_nom', 0.26)           # 直行速度，单位 m/s
        self.declare_parameter('v_slow', 0.08)          # 贴墙与找墙时的低速，单位 m/s
        self.declare_parameter('w_max', 1.50)           # 角速度限幅，单位 rad/s
        self.declare_parameter('w_turn', 1.30)          # 转角角速度，单位 rad/s
        self.declare_parameter('kp', 0.90)              # 墙距误差比例增益
        self.declare_parameter('kd', 0.35)              # 误差变化率增益（抑制超调）
        self.declare_parameter('k_psi', 1.20)           # 航向对齐增益
        self.declare_parameter('alpha', 0.35)           # 墙距一阶平滑系数，取值越小越平滑
        self.declare_parameter('dead_band', 0.04)       # 误差死区，单位 m
        self.declare_parameter('front_half', 20.0)      # 前方扇区半角，单位 度
        self.declare_parameter('left_front_min', 20.0)  # 左前方扇区起始角，单位 度
        self.declare_parameter('left_front_max', 50.0)  # 左前方扇区终止角，单位 度
        self.declare_parameter('left_min', 40.0)        # 左侧扇区起始角，单位 度
        self.declare_parameter('left_max', 90.0)        # 左侧扇区终止角，单位 度
        self.declare_parameter('perp_min', 78.0)        # 正左方扇区起始角，单位 度
        self.declare_parameter('perp_max', 102.0)       # 正左方扇区终止角，单位 度
        self.declare_parameter('goal_x', -999.0)        # 出口坐标 x，-999 表示不启用
        self.declare_parameter('goal_y', -999.0)        # 出口坐标 y
        self.declare_parameter('goal_tol', 0.45)        # 到达判定半径，单位 m
        self.declare_parameter('exit_x', 2.55)          # 旧式出口判定阈值，goal 未设时生效
        self.declare_parameter('bounds', 15.00)         # 越界保护阈值，单位 m

        g = self.get_parameter
        self.target_left = g('target_left').value
        self.front_danger = g('front_danger').value
        self.left_lost = g('left_lost').value
        self.v_nom = g('v_nom').value
        self.v_slow = g('v_slow').value
        self.w_max = g('w_max').value
        self.w_turn = g('w_turn').value
        self.kp = g('kp').value
        self.kd = g('kd').value
        self.k_psi = g('k_psi').value
        self.alpha = g('alpha').value
        self.dead_band = g('dead_band').value
        self.front_half = math.radians(g('front_half').value)
        self.lf_min = math.radians(g('left_front_min').value)
        self.lf_max = math.radians(g('left_front_max').value)
        self.l_min = math.radians(g('left_min').value)
        self.l_max = math.radians(g('left_max').value)
        self.p_min = math.radians(g('perp_min').value)
        self.p_max = math.radians(g('perp_max').value)
        self.goal_x = g('goal_x').value
        self.goal_y = g('goal_y').value
        self.goal_tol = g('goal_tol').value
        self.use_goal = (self.goal_x > -900.0) and (self.goal_y > -900.0)
        self.exit_x = g('exit_x').value
        self.bounds = g('bounds').value

        # ---- 雷达与位姿状态 ----
        self.front = float('inf')
        self.left = float('inf')
        self.left_front = float('inf')
        self.scan_ok = False
        self.lost_since = None      # 左侧丢墙的起始时刻
        self.state = 'INIT'
        self.done = False
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.right = float('inf')

        # ---- 转角锁存状态 ----
        self.pivot_deg = 80.0        # 单次转角的目标角度，单位 度
        self.pivot_tol = 5.0         # 转角角度容差，单位 度
        self.pivot_target = None     # 锁存的目标朝向；None 表示未处于转角过程
        self.pivot_t0 = 0.0          # 转角开始时刻
        self.pivot_cool = 0.0        # 转角结束后的冷却截止时刻，避免立即重复触发
        self.log_tick = 0
        self.path_tick = 0
        self.last_scan = None        # 最近一帧激光数据，供直线拟合使用
        self.wall_d = float('inf')   # 拟合得到的墙面垂距
        self.wall_psi = 0.0          # 拟合得到的车身与墙面夹角
        self.left_smooth = None      # 平滑后的左侧墙距
        self.err_prev = None         # 上一控制周期的误差，用于计算微分项
        self.last_v = 0.0
        self.last_w = 0.0

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        # 行驶轨迹发布，RViz2 中添加 Path 显示即可观察轨迹
        self.path_pub = self.create_publisher(Path, '/trajectory', 10)
        self.path_msg = Path()
        self.path_msg.header.frame_id = 'odom'
        self.create_subscription(LaserScan, '/scan', self.scan_cb,
                                 qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.create_timer(0.1, self.control_loop)      # 控制周期 10 Hz

        if self.use_goal:
            self.get_logger().info(
                '左墙跟随启动：目标墙距 %.2f m，前方危险 %.2f m，出口 (%.2f, %.2f) ± %.2f m'
                % (self.target_left, self.front_danger,
                   self.goal_x, self.goal_y, self.goal_tol))
        else:
            self.get_logger().info(
                '左墙跟随启动：目标墙距 %.2f m，前方危险 %.2f m，出口 x>%.2f'
                % (self.target_left, self.front_danger, self.exit_x))

    # ---------------- 回调函数 ----------------
    def odom_cb(self, msg):
        """更新位姿，并累积发布行驶轨迹。"""
        p = msg.pose.pose.position
        self.x = p.x
        self.y = p.y
        # 由四元数解算偏航角，用于转角锁存的角度判定
        q = msg.pose.pose.orientation
        self.yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                              1.0 - 2.0 * (q.y * q.y + q.z * q.z))

        # 累积轨迹点，每 5 帧（约 6 Hz）发布一次，降低消息频率
        ps = PoseStamped()
        ps.header.frame_id = 'odom'
        ps.header.stamp = msg.header.stamp
        ps.pose = msg.pose.pose
        self.path_msg.header.stamp = msg.header.stamp
        self.path_msg.poses.append(ps)
        self.path_tick += 1
        if self.path_tick % 5 == 0:
            self.path_pub.publish(self.path_msg)

    def scan_cb(self, msg):
        """提取各扇区距离，并保存当前帧供直线拟合使用。"""
        n = len(msg.ranges)
        if n == 0:
            return
        self.front = self.sector_min(msg, -self.front_half, self.front_half)
        self.left_front = self.sector_min(msg, self.lf_min, self.lf_max)
        # 判断“是否存在墙面”使用宽扇区最小值，保证进入转角前即可感知
        self.last_scan = msg
        self.left_wide = self.sector_min(msg, self.l_min, self.l_max)
        # 控制“贴墙距离”使用正左方均值，其结果接近真实垂距，不受偏航角影响
        perp = self.sector_mean(msg, self.p_min, self.p_max)
        self.left = perp if math.isfinite(perp) else self.left_wide
        right_mean = self.sector_mean(msg, math.radians(-102.0), math.radians(-78.0))
        self.right = right_mean if math.isfinite(right_mean) else 0.0
        self.scan_ok = True

    # ---------------- 工具函数 ----------------
    @staticmethod
    def wrap(a):
        """将角度归一化至 (-pi, pi]。"""
        while a > math.pi:
            a -= 2.0 * math.pi
        while a <= -math.pi:
            a += 2.0 * math.pi
        return a

    def sector_min(self, msg, a_min, a_max):
        """返回角度区间 [a_min, a_max] 内所有有效测距的最小值。"""
        best = float('inf')
        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r) or r <= 0.0:
                continue
            ang = self.wrap(msg.angle_min + i * msg.angle_increment)
            if a_min <= ang <= a_max and r < best:
                best = r
        return best

    def sector_mean(self, msg, a_min, a_max):
        """返回角度区间内所有有效测距的平均值，较最小值更接近垂直距离。"""
        vals = []
        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r) or r <= 0.0:
                continue
            ang = self.wrap(msg.angle_min + i * msg.angle_increment)
            if a_min <= ang <= a_max:
                vals.append(r)
        if not vals:
            return float('inf')
        return sum(vals) / len(vals)

    def wall_fit(self):
        """对左侧点云做最小二乘直线拟合。

        Returns
            d    车体原点到墙面直线的距离
            psi  墙面走向与车体 x 轴的夹角，0 表示车身与墙平行，
                 正值表示前方墙面向左偏开
        """
        msg = self.last_scan
        if msg is None:
            return float('inf'), 0.0

        pts = []
        for i, r in enumerate(msg.ranges):
            # 仅使用 0.15~3.0 m 的有效点，避免远距离噪声影响拟合
            if not math.isfinite(r) or r < 0.15 or r > 3.0:
                continue
            ang = self.wrap(msg.angle_min + i * msg.angle_increment)
            if not (math.radians(30.0) <= ang <= math.radians(150.0)):
                continue
            # 极坐标转笛卡尔坐标，得到车体坐标系下的点
            pts.append((r * math.cos(ang), r * math.sin(ang)))

        # 点数不足时放弃拟合，交回距离控制分支处理
        if len(pts) < 8:
            return float('inf'), 0.0

        n = len(pts)
        mx = sum(p[0] for p in pts) / n
        my = sum(p[1] for p in pts) / n
        sxx = sxy = syy = 0.0
        for px, py in pts:
            dx, dy = px - mx, py - my
            sxx += dx * dx
            sxy += dx * dy
            syy += dy * dy

        # 协方差矩阵主方向即墙面走向
        ang = 0.5 * math.atan2(2.0 * sxy, sxx - syy)
        ux, uy = math.cos(ang), math.sin(ang)
        if ux < 0.0:                      # 统一指向 +x，使角度落在 (-90°, 90°]
            ux, uy = -ux, -uy
        psi = math.atan2(uy, ux)

        # 原点到直线的距离等于单位方向向量与质心向量的叉积模
        d = abs(ux * (-my) - uy * (-mx))
        return d, psi

    def now(self):
        """返回当前仿真时刻，单位 s。"""
        return self.get_clock().now().nanoseconds * 1e-9

    def publish(self, v, w):
        """发布速度指令，并记录最近一次输出。"""
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)
        self.cmd_pub.publish(msg)
        self.last_v = float(v)
        self.last_w = float(w)

    def stop(self, reason):
        """输出零速度并记录终止原因，同一原因只打印一次。"""
        self.publish(0.0, 0.0)
        if not self.done:
            self.get_logger().info(reason)
        self.done = True

    # ---------------- 主控制循环 ----------------
    def control_loop(self):
        """按固定周期执行状态机决策并发布速度指令。"""
        if self.done:
            self.publish(0.0, 0.0)
            return

        # 越界保护，防止算法失效时车辆驶出仿真区域
        if abs(self.x) > self.bounds or abs(self.y) > self.bounds:
            self.stop('越界停车：(%.2f, %.2f)' % (self.x, self.y))
            return

        # 到达出口判定：优先使用目标点距离，更换迷宫时只需重新给定坐标
        if self.use_goal:
            dist = math.hypot(self.x - self.goal_x, self.y - self.goal_y)
            if dist < self.goal_tol:
                self.stop('已到达出口：(%.2f, %.2f)，距目标 (%.2f, %.2f) 仅 %.2f m，停车'
                          % (self.x, self.y, self.goal_x, self.goal_y, dist))
                return
        elif self.x > self.exit_x:
            self.stop('已到达出口：(%.2f, %.2f)，停车' % (self.x, self.y))
            return

        if not self.scan_ok:
            self.publish(0.0, 0.0)
            return

        t = self.now()

        # 1. 前方过近：锁存一个转角目标，原地转至目标朝向
        close = min(self.front, self.left_front) < self.front_danger
        if close and self.pivot_target is None and t > self.pivot_cool:
            # 转向方向：左右两侧中较空旷的一侧，可兼顾 T 型路口与死胡同
            turn_dir = -1.0                      # 默认右转
            if math.isfinite(self.left) and math.isfinite(self.right):
                if self.left > self.right + 0.05:
                    turn_dir = 1.0               # 左侧更空旷，改为左转
            self.pivot_target = self.wrap(self.yaw + turn_dir * math.radians(self.pivot_deg))
            self.pivot_t0 = t

        if self.pivot_target is not None:
            self.state = 'TURN_RIGHT'
            self.lost_since = None
            err = self.wrap(self.pivot_target - self.yaw)
            if abs(err) < math.radians(self.pivot_tol) or (t - self.pivot_t0) > 8.0:
                # 转角到位或超时：解除锁存，并设置 1 s 冷却时间
                self.pivot_target = None
                self.pivot_cool = t + 1.0
                self.publish(0.0, 0.0)
            else:
                w = max(-self.w_turn, min(self.w_turn, 2.0 * err))
                self.publish(0.0, w)

        # 2. 左侧丢失：低速左转寻找墙面；长时间未找到则直行通过开阔区域
        elif self.left > self.left_lost:
            if self.lost_since is None:
                self.lost_since = t
            lost_for = t - self.lost_since
            if lost_for > 1.5:
                self.state = 'STRAIGHT'
                self.publish(self.v_nom, 0.0)
            else:
                self.state = 'TURN_LEFT'
                self.publish(self.v_slow, self.w_turn * 0.8)

        # 3. 沿左墙行驶：墙面直线拟合，距离项控制贴墙距离，航向项摆正车身
        else:
            self.lost_since = None
            self.state = 'FOLLOW'

            d, psi = self.wall_fit()
            self.wall_d = d
            self.wall_psi = psi

            if math.isfinite(d):
                if self.left_smooth is None:
                    self.left_smooth = d
                else:
                    self.left_smooth = ((1.0 - self.alpha) * self.left_smooth
                                        + self.alpha * d)
                err = self.left_smooth - self.target_left
            else:
                err = 0.0

            # 距离项：偏离目标墙距时横向修正；航向项：车身与墙不平行时摆正
            w = self.kp * err + self.k_psi * psi
            w = max(-self.w_max, min(self.w_max, w))

            # 线速度随转向幅度衰减，转弯时自动减速以保证过弯稳定
            v = self.v_nom * (1.0 - 0.45 * abs(w) / self.w_max)
            self.publish(v, w)

        # 每 5 个控制周期（约 0.5 s）打印一次状态
        self.log_tick += 1
        if self.log_tick % 5 == 0:
            self.get_logger().info(
                '[%s] 前 %.2f 左 %.2f 拟合墙距 %.2f 夹角 %+.2f rad -> v=%.2f w=%+.2f  @(%.2f, %.2f)'
                % (self.state, self.front, self.left, self.wall_d, self.wall_psi,
                   self.last_v, self.last_w, self.x, self.y))


def main(args=None):
    rclpy.init(args=args)
    node = LeftWallFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 退出前补发一次零速度，避免车辆保持上一次的非零指令。
        # 进程被中断时上下文可能已失效，故此处需容错处理。
        try:
            node.publish(0.0, 0.0)
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
