"""一键启动：Gazebo 仿真 + YOLO 检测 + 抓取规划器。

用法：
  ros2 launch grasp_sim full.launch.py
  ros2 launch grasp_sim full.launch.py confidence:=0.5
  ros2 launch grasp_sim full.launch.py skip_planner:=true   # 调试用：只起 sim + YOLO

依赖：
  - ultralytics 装到系统 python3（不是 conda）
    pip3 install --user ultralytics opencv-python
  - /tmp/ur5.urdf 和 /tmp/ur5.srdf 预生成（首次冷启动见 README 第 4 步）

完全 ROS2 标准做法：
  - YOLO 通过 entry_points='detector' 注册，ros2 run yolo_vision detector 启动
  - planner 通过 install 复制到 share/grasp_sim/scripts/，ros2 run grasp_sim grasp_planner 启动
  - 路径全靠 ROS2 资源查找 API（FindExecutable / FindPackageShare）
  - 不依赖 conda、不硬编码 ~/grasp_ws/
"""
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ---- 参数 ----
    model_arg = DeclareLaunchArgument(
        "model", default_value="best.pt",
        description="YOLO 模型文件名",
    )
    confidence_arg = DeclareLaunchArgument(
        "confidence", default_value="0.5",
        description="YOLO 置信度阈值",
    )
    skip_yolo_arg = DeclareLaunchArgument(
        "skip_yolo", default_value="false",
        description="跳过 YOLO 节点",
    )
    skip_planner_arg = DeclareLaunchArgument(
        "skip_planner", default_value="false",
        description="跳过抓取规划器",
    )
    yolo_delay_arg = DeclareLaunchArgument(
        "yolo_delay", default_value="20.0",
        description="YOLO 启动延时（秒，等控制器 + move_group 就绪）",
    )
    planner_delay_arg = DeclareLaunchArgument(
        "planner_delay", default_value="25.0",
        description="planner 启动延时（秒，等 YOLO 出第一帧 detection）",
    )

    # ---- 1. include sim.launch.py（Gazebo + UR5 + 控制器 + MoveIt）----
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("grasp_sim"), "/launch/sim.launch.py"]
        )
    )

    # ---- 2. YOLO 节点 ----
    # 用 Node 而非 ExecuteProcess：标准 ROS2 节点方式，自动用当前进程 python3
    # 可执行名 'detector' 来自 yolo_vision/setup.py 的 entry_points
    yolo_node = Node(
        package="yolo_vision",
        executable="detector",
        name="yolo_detector",
        output="screen",
        parameters=[{
            "model": LaunchConfiguration("model"),
            "confidence": LaunchConfiguration("confidence"),
        }],
        condition=UnlessCondition(LaunchConfiguration("skip_yolo")),
    )

    # ---- 3. 抓取规划器 ----
    # 用 Node：可执行名 'grasp_planner' 来自 CMakeLists.txt 的 install(PROGRAMS)
    planner_node = Node(
        package="grasp_sim",
        executable="grasp_planner",
        name="grasp_planner",
        output="screen",
        condition=UnlessCondition(LaunchConfiguration("skip_planner")),
    )

    return LaunchDescription([
        model_arg, confidence_arg,
        skip_yolo_arg, skip_planner_arg,
        yolo_delay_arg, planner_delay_arg,
        LogInfo(msg="=== full.launch.py: 启动 sim + YOLO + planner ==="),
        sim_launch,
        # YOLO 延 t=20s 起（控制器 ~t=14s 激活，move_group ~t=12s）
        TimerAction(period=20.0, actions=[yolo_node]),
        # planner 延 t=25s 起（等 YOLO 出第一帧 detection）
        TimerAction(period=25.0, actions=[planner_node]),
    ])
