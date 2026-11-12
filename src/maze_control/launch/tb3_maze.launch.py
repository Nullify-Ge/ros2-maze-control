"""一键启动：Gazebo 迷宫仿真 + RViz2 + 左墙跟随节点

用法（world 只写短名即可，出生点/出口坐标由本文件内的迷宫表自动查）:
    ros2 launch maze_control tb3_maze.launch.py              # 默认 l2（L 型）
    ros2 launch maze_control tb3_maze.launch.py world:=u     # U 型
    ros2 launch maze_control tb3_maze.launch.py world:=z     # Z 型
    ros2 launch maze_control tb3_maze.launch.py world:=l     # 大 L
    ros2 launch maze_control tb3_maze.launch.py world:=s     # S 型（左手定则会失败）

其它开关:
    gui:=false   不开 Gazebo 界面
    rviz:=false  不开 RViz
    run:=false   只搭场景，不启动算法节点
    v_nom:=0.30  改直行速度（默认 0.26）
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction,
    SetEnvironmentVariable, TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# ---------------------------------------------------------------------------
# 迷宫表：短名 -> (world 文件, 出生点 x/y, 出口 x/y, 出口判定半径)
# 起点都是入口、朝 +x；出口坐标取自各 world 文件头的 astar 验证行
# ---------------------------------------------------------------------------
MAZES = {
    'branch':   ('branch_maze.world',   1.00, 1.00, 9.60, 7.90, 0.60),
    'straight': ('straight_maze.world', 0.0, 2.0, 8.0, 2.0, 0.45),
    'l2':       ('l2_maze.world',       0.0, 2.0, 4.0, 7.0, 0.45),
    'shape7':   ('shape7_maze.world',   0.0, 2.0, 7.0, 5.0, 0.45),
    'u':        ('u_maze.world',        0.0, 2.0, 8.0, 6.0, 0.45),
    'z':        ('z_maze.world',        0.0, 2.0, 8.0, 8.0, 0.45),
    'l':        ('l_maze.world',        0.0, 2.0, 7.0, 8.0, 0.45),
    's':        ('s_shape_maze.world',  0.0, 1.0, 7.0, 9.0, 0.45),
}


def _resolve(name):
    """把 world:= 传进来的短名或文件名统一成表里的键。"""
    key = name.strip()
    if key.endswith('.world'):
        key = key[:-len('.world')]
    if key.endswith('_maze'):
        key = key[:-len('_maze')]
    return key


def _setup(context):
    pkg_share = get_package_share_directory('maze_control')
    tb3_share = get_package_share_directory('turtlebot3_gazebo')
    gazebo_share = get_package_share_directory('gazebo_ros')
    rviz_config = os.path.join(pkg_share, 'rviz', 'maze.rviz')

    key = _resolve(LaunchConfiguration('world').perform(context))
    if key not in MAZES:
        raise RuntimeError(
            'unknown maze: %r, available: %s' % (key, ', '.join(sorted(MAZES))))
    world_file, def_x, def_y, def_gx, def_gy, def_tol = MAZES[key]

    def pick(arg, default):
        """命令行显式给了就用命令行的，没给（空串）就用迷宫表里的。"""
        value = LaunchConfiguration(arg).perform(context).strip()
        return value if value else str(default)

    x_pose = pick('x_pose', def_x)
    y_pose = pick('y_pose', def_y)
    goal_x = pick('goal_x', def_gx)
    goal_y = pick('goal_y', def_gy)
    goal_tol = pick('goal_tol', def_tol)

    use_rviz = LaunchConfiguration('rviz')
    use_gui = LaunchConfiguration('gui')
    use_run = LaunchConfiguration('run')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return [
        # turtlebot3 的 launch 都靠这个环境变量找模型
        SetEnvironmentVariable('TURTLEBOT3_MODEL', 'burger'),

        # 1. Gazebo 服务器：加载选定的迷宫 world
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_share, 'launch', 'gzserver.launch.py')),
            launch_arguments={
                'world': PathJoinSubstitution([pkg_share, 'worlds', world_file]),
            }.items(),
        ),

        # 2. Gazebo 客户端（图形界面）
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_share, 'launch', 'gzclient.launch.py')),
            condition=IfCondition(use_gui),
        ),

        # 3. 机器人状态发布（提供 TF 与 /robot_description）
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(tb3_share, 'launch',
                             'robot_state_publisher.launch.py')),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),

        # 4. 在迷宫入口刷出小车，朝 +x
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(tb3_share, 'launch', 'spawn_turtlebot3.launch.py')),
            launch_arguments={'x_pose': x_pose, 'y_pose': y_pose}.items(),
        ),

        # 5. RViz2（必须带 use_sim_time，否则 TF 会被当成过期数据丢掉）
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(use_rviz),
            output='screen',
        ),

        # 6. 左墙跟随节点，等 Gazebo 把车刷出来（约 5~8 s）再启动
        TimerAction(
            period=8.0,
            actions=[
                Node(
                    package='maze_control',
                    executable='tb3_wall_follower',
                    name='tb3_wall_follower',
                    parameters=[{
                        'use_sim_time': use_sim_time,
                        'goal_x': float(goal_x),
                        'goal_y': float(goal_y),
                        'goal_tol': float(goal_tol),
                        'v_nom': ParameterValue(
                            LaunchConfiguration('v_nom'), value_type=float),
                    }],
                    condition=IfCondition(use_run),
                    output='screen',
                ),
            ],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='l2',
                              description='迷宫短名: ' + ', '.join(sorted(MAZES))),
        DeclareLaunchArgument('rviz', default_value='true', description='启动 RViz2'),
        DeclareLaunchArgument('gui', default_value='true', description='启动 Gazebo 界面'),
        DeclareLaunchArgument('run', default_value='true', description='启动左墙跟随节点'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        # 下面四个留空 = 用迷宫表里的坐标；填了则以填的为准
        DeclareLaunchArgument('x_pose', default_value='', description='覆盖出生点 x'),
        DeclareLaunchArgument('y_pose', default_value='', description='覆盖出生点 y'),
        DeclareLaunchArgument('goal_x', default_value='', description='覆盖出口 x'),
        DeclareLaunchArgument('goal_y', default_value='', description='覆盖出口 y'),
        DeclareLaunchArgument('goal_tol', default_value='', description='覆盖判定半径'),
        DeclareLaunchArgument('v_nom', default_value='0.26',
                              description='直行速度 m/s（burger 上限约 0.22，0.3 属超规格）'),
        OpaqueFunction(function=_setup),
    ])
