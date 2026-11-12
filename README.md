# ros2-maze-control

ROS 2 Humble + Gazebo 的差速小车迷宫通行项目：激光雷达分区测距、反应式避障、迷宫左墙跟随。

**课程代码**：(2026-2027-1)-S0610350-01/2/3/4

| 项目 | 说明 |
|---|---|
| 平台 | TurtleBot3 Burger（差速驱动，轮距 0.160 m、轮径 0.066 m） |
| 仿真 | Gazebo 11.10.2 + RViz2 |
| 环境 | Ubuntu 22.04.5 LTS + ROS 2 Humble |
| 语言 | Python 3.10（rclpy） |
| 功能包 | `maze_control`（5 个可执行节点 + 7 套迷宫世界 + 一键 launch） |

---

# maze_control — TurtleBot3 迷宫自主穿越（左墙跟随）

> 课程代码：(2026-2027-1)-S0610350
> 平台：ROS 2 Humble + Gazebo 11 + TurtleBot3 Burger
> 课程：《机器人感知与控制实践》期末综合设计 —— 方向 1「地老鼠走迷宫系统的开发」

---

## 1. 项目简介

本项目在 Gazebo 仿真环境中，用一个差速两轮机器人（TurtleBot3 Burger）完成从迷宫入口到出口的全自主穿越。

系统由四部分组成：

| 模块 | 功能 | 对应任务书要求 |
|---|---|---|
| 环境与模型 | 自建 `.world` 迷宫（墙体带碰撞）、Python launch 一键启动 | 环境与模型搭建（25 分） |
| 运动控制 | 开环直行/原地转、里程计闭环 P 控制点到点、航点序列跟踪 | 运动控制模块（15 分） |
| 雷达处理 | `/scan` 极坐标分区提取前/左/右距离、反应式避障 | 激光雷达数据处理（15 分） |
| 迷宫穿越 | 左墙跟随算法（PD 控距 + 墙面直线拟合 + 转角锁存） | 迷宫左墙跟随（25 分） |

所有算法节点均为自己编写，Gazebo 动力学与传感器插件使用 TurtleBot3 官方模型（`turtlebot3_gazebo` 提供的 `model.sdf`）。

---

## 2. 运行环境

| 项目 | 版本 / 说明 |
|---|---|
| 操作系统 | Ubuntu 22.04 |
| ROS 2 | Humble |
| 仿真器 | Gazebo 11.10 |
| 机器人 | TurtleBot3 Burger |

安装依赖：

```bash
sudo apt install ros-humble-gazebo-ros-pkgs \
                 ros-humble-turtlebot3 \
                 ros-humble-turtlebot3-gazebo \
                 ros-humble-turtlebot3-description \
                 ros-humble-turtlebot3-teleop

echo 'export TURTLEBOT3_MODEL=burger' >> ~/.bashrc
source ~/.bashrc
```

> `TURTLEBOT3_MODEL` 必须设置，官方 launch 文件依赖它；本项目的 launch 内部也会自己设置一次。

---

## 3. 目录结构

```
maze_control/
├── launch/
│   └── tb3_maze.launch.py      # 一键启动：Gazebo + 机器人 + RViz2 + 算法节点
├── maze_control/
│   ├── tb3_point2point.py      # 点到点 P 控制
│   ├── tb3_waypoints.py        # 航点序列跟踪（并发布 /trajectory 供 RViz 画轨迹）
│   ├── scan_region_node.py     # 雷达分区距离提取
│   ├── tb3_avoid.py            # 反应式避障
│   └── tb3_wall_follower.py    # 左墙跟随（迷宫穿越主算法）
├── rviz/
│   └── maze.rviz               # RViz 配置：车模 + 雷达点 + 绿色轨迹
├── worlds/
│   ├── straight_maze.world     # 直线
│   ├── l2_maze.world           # 1 个弯
│   ├── shape7_maze.world       # 1 个弯（另一形状）
│   ├── u_maze.world            # 2 个弯
│   ├── z_maze.world            # 3 个弯
│   ├── l_maze.world            # 2 个弯
│   ├── s_shape_maze.world      # S 型（挑战项，朴素左手定则无法通过）
│   └── zips/                   # 原始压缩包（存档）
├── package.xml
├── setup.py
└── README.md
```

---

## 4. 编译

```bash
cd ~/ros2/ros2maze_ws
colcon build --packages-select maze_control
source install/setup.bash
```

---

## 5. 一键启动

```bash
ros2 launch maze_control tb3_maze.launch.py
```

该命令会依次启动：

1. `gzserver` 加载 `worlds/<world>` 迷宫
2. `gzclient` 打开 Gazebo 可视化
3. `robot_state_publisher` 发布 `/robot_description` 与 TF
4. `spawn_entity` 在迷宫入口生成 TurtleBot3
5. `rviz2` 加载 `rviz/maze.rviz`（车模 + 雷达点 + 绿色轨迹）
6. 8 秒后启动算法节点（左墙跟随）

### 常用参数

| 参数 | 默认值 | 说明 |
|---|---|---|
| `world` | `l2_maze.world` | 迷宫文件名（`worlds/` 下） |
| `x_pose` / `y_pose` | `0.0` / `2.0` | 机器人出生点 |
| `goal_x` / `goal_y` | `4.0` / `7.0` | 出口坐标 |
| `goal_tol` | `0.45` | 距出口多近算到达（m） |
| `v_nom` | `0.22` | 巡线速度（m/s，Burger 上限约 0.22） |
| `rviz` | `true` | 是否启动 RViz |
| `gui` | `true` | 是否启动 gzclient |
| `run` | `true` | 是否启动算法节点 |
| `use_sim_time` | `true` | 使用仿真时间 |

示例（跑 U 型迷宫、关闭 RViz 和 Gazebo 界面做快速验证）：

```bash
ros2 launch maze_control tb3_maze.launch.py \
    world:=u_maze.world x_pose:=0.5 y_pose:=2.0 \
    goal_x:=8.0 goal_y:=6.0 rviz:=false gui:=false
```

---

## 6. 各模块单独运行

先启动仿真（不启算法节点）：

```bash
ros2 launch maze_control tb3_maze.launch.py run:=false
```

### 6.1 点到点 P 控制

```bash
ros2 run maze_control tb3_point2point
```

控制律：`v = Kp_dist · 距离误差`，`w = Kp_ang · 角度误差`，到点后停车。
默认目标 `(1.0, 0.0)`，可通过参数覆盖：

```bash
ros2 run maze_control tb3_point2point --ros-args -p goal_x:=1.0 -p goal_y:=1.0
```

### 6.2 航点序列跟踪

```bash
ros2 run maze_control tb3_waypoints
```

默认走矩形：`(1,0) → (1,1) → (0,1) → (0,0)`，依次经过后停车。
节点同时把走过的位姿发布到 `/trajectory`（`nav_msgs/Path`），RViz 里画成绿色轨迹线。

### 6.3 雷达分区提取

```bash
ros2 run maze_control scan_region
```

输出示例：

```
【距离】前方: 3.17 m | 左侧: 0.88 m | 右侧: 0.88 m
```

分区定义（角度均相对于车身正前方）：

| 区域 | 角度范围 | 取值方式 |
|---|---|---|
| 前方 | −15° ~ +15° | 最小值 |
| 左侧 | 60° ~ 90° | 中值 |
| 右侧 | −90° ~ −60° | 中值 |

> 注意：本机 `/scan` 的 `angle_min = 0`，角度全部落在 `[0, 2π)`，因此代码用
> `deg = (degrees(angle) % 360)` 后再判断扇区，处理跨 0° 的情况。

### 6.4 反应式避障

```bash
ros2 run maze_control tb3_avoid
```

逻辑：前方距离 < `safe_distance` 就停住原地转，转开后再走；转向带锁存，不会在阈值附近抖动。

### 6.5 左墙跟随（迷宫穿越）

```bash
ros2 run maze_control tb3_wall_follower --ros-args -p goal_x:=4.0 -p goal_y:=7.0
```

---

## 7. 迷宫清单与实测结果

测试条件：Ubuntu 22.04 + Gazebo 11，`v_nom = 0.22 m/s`，计时包含约 8 s 的仿真启动等待。

| 迷宫文件 | 弯数 | 用时 | 终点（距目标） |
|---|---|---|---|
| `straight_maze.world` | 0 | 51 s | (7.69, 2.32) |
| `l2_maze.world` | 1 | 51 s | (3.71, 6.68) |
| `shape7_maze.world` | 1 | 54 s | (6.75, 4.63) |
| `u_maze.world` | 2 | 63 s | (7.77, 6.38) |
| `z_maze.world` | 3 | 69 s | (7.76, 7.62) |
| `l_maze.world` | 2 | 66 s | (6.76, 8.38) |
| `s_shape_maze.world` | 3 | —— | 挑战项，见第 9 节 |

---

## 8. 算法说明

### 8.1 左墙跟随控制律

以「保持左侧墙距 `target_left`」为目标，分三步：

1. **测距**：取正左方 78°~102° 扇区测距的**平均值**作为垂距（比最小值更稳）
2. **拟合**：对左侧 30°~150° 的激光点做最小二乘直线拟合，得到车身到墙的垂距 `d`
   与车身与墙的夹角 `psi`
3. **控向**：

```
w = kp · (d − target_left) + k_psi · psi
v = v_nom · (1 − 0.45 · |w| / w_max)      # 转得越猛，走得越慢
```

其中 `kp = 0.90`、`k_psi = 1.20`、`kd = 0.35`（微分抑制超调）、`target_left = 0.55 m`。

### 8.2 转角锁存

前方距离 < `front_danger`（0.45 m）时：

1. 比较左右两侧的空旷程度，**往更空的一侧转**
2. 锁定目标朝向，**原地转到 80° 才解锁**（中途不管前方是否已空）
3. 转完冷却 1 s，防止立刻重复触发

这一改动解决了「转一点点→前方变空→恢复直行→又撞墙」的抖动问题。

### 8.3 算法缺陷分析与改进（拓展项）

| 阶段 | 做法 | 问题 | 改进后 |
|---|---|---|---|
| v1 | 左前方 40°~90° 取最小值控距 | 车一歪斜，测得距离变小 → 误以为贴墙 → 越歪越纠，画龙 | 横向标准差 0.124 m |
| v2 | PD 控制 + 平滑 | 只能抑制、不能消除振荡 | 标准差 0.09 m |
| v3 | 改用正左方均值 | 抖动减小，但偏航时仍有系统偏差 | 标准差 0.07 m |
| v4 | **墙面直线拟合**（距离 + 夹角双项） | —— | **标准差 0.053 m，横向摆动方向反转次数 2 → 0** |

---

## 9. 已知限制

- **S 型迷宫**（`s_shape_maze.world`，含右转口）无法用朴素左手定则通过：算法只判断
  「左侧有没有墙」，不预判前方通路形状，在连续反向弯里会走错方向。
  改进方向：加上「贴墙距离 + 前方通路宽度」的局部决策，或改用右手定则变体 / 局部建图。
- 转弯时是「停-转-走」，比连续弧线转弯慢；`v_nom` 可调到 0.30 缩短演示时间（超出 Burger 规格，报告中已注明）。

---

## 10. 话题接口

| 话题 | 类型 | 方向 | 说明 |
|---|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | 发布 | 速度指令 |
| `/odom` | `nav_msgs/Odometry` | 订阅 | 里程计位姿 |
| `/scan` | `sensor_msgs/LaserScan` | 订阅 | 激光雷达 |
| `/trajectory` | `nav_msgs/Path` | 发布 | 行走轨迹（RViz 显示绿线） |
| `/robot_description` | `std_msgs/String` | 订阅 | URDF 模型 |

---

## 11. 常见问题

| 现象 | 原因 | 解决 |
|---|---|---|
| `Caught exception in launch: 'TURTLEBOT3_MODEL'` | 没设环境变量 | `export TURTLEBOT3_MODEL=burger` |
| Gazebo 里只看到地面、没有迷宫 | 旧 gzserver 没退出，端口冲突 | `pkill -9 gzserver` 后重开 |
| RViz 里车模不动、只有绿线在长 | RViz 没开 `use_sim_time`，TF 被当作过期数据 | launch 里已带该参数；手动起 RViz 要加 `--ros-args -p use_sim_time:=true` |
| Gazebo 报 `Empty string used when setting a required parameter. Key[name]` | `${world}` 标签 `name=""` | 给 world 填上名字 |
| `file 'xxx' was not found in the share directory` | `setup.py` 的 `data_files` 写错或 build 失败 | 看 `CalledProcessError` **上面**几行的真实报错 |

---

## 12. 分工

| 成员 | 负责模块 |
|---|---|
| （待填） | 环境搭建、运动控制、雷达处理、迷宫穿越、README |
| （待填） | 设计报告撰写、结果分析 |

---

## 13. 参考

- TurtleBot3 官方：<https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/>
- ROS 2 Humble 文档：<https://docs.ros.org/en/humble/>
