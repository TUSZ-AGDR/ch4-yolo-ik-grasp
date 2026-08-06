"""Launch Gazebo Fortress + UR5 + ros2_control + RGBD camera + ros_gz_bridge + MoveIt2."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    ExecuteProcess,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ur_moveit_config.launch_common import load_yaml

URDF_PATH = "/tmp/ur5.urdf"
SRDF_PATH = "/tmp/ur5.srdf"
COMBINED_XACRO = "models/ur5_with_gripper.urdf.xacro"


def generate_launch_description():
    pkg_grasp_sim = get_package_share_directory("grasp_sim")
    pkg_ur_description = get_package_share_directory("ur_description")
    # Use source paths so xacro changes take effect without rebuild
    # get_package_share_directory returns .../install/grasp_sim/share/grasp_sim
    ws_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(pkg_grasp_sim))))
    src_grasp_sim = os.path.join(ws_root, "src", "grasp_sim")

    world_path = os.path.join(pkg_grasp_sim, "worlds", "grasp_world.sdf")
    bridge_yaml = os.path.join(pkg_grasp_sim, "config", "gz_bridge.yaml")
    ros2_control_yaml = os.path.join(pkg_grasp_sim, "config", "ros2_control.yaml")
    combined_xacro_path = os.path.join(src_grasp_sim, COMBINED_XACRO)
    srdf_xacro_path = os.path.join(src_grasp_sim, "config", "ur5_gripper.srdf.xacro")

    # Add ROS share dirs to Gazebo resource path so mesh model:// URIs resolve
    ros_share = "/opt/ros/humble/share"
    existing_gz_path = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    gz_resource_path = f"{ros_share}" + (f":{existing_gz_path}" if existing_gz_path else "")

    set_gz_path = SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", gz_resource_path)

    # URDF description as Command substitution (for robot_state_publisher)
    # Uses combined UR5+gripper xacro with unified ros2_control
    robot_description_cmd = Command([
        FindExecutable(name="xacro"), " ",
        combined_xacro_path,
        " name:=ur5",
        " ur_type:=ur5",
        " safety_limits:=false",
        " sim_ignition:=true",
        " sim_gazebo:=false",
        " simulation_controllers:=", ros2_control_yaml,
    ])

    # Generate URDF file for spawn (combined UR5 + gripper)
    generate_urdf = ExecuteProcess(
        cmd=[
            "bash", "-c",
            "xacro " + combined_xacro_path +
            " name:=ur5 ur_type:=ur5 safety_limits:=false" +
            " sim_ignition:=true sim_gazebo:=false" +
            " simulation_controllers:=" + ros2_control_yaml +
            " -o " + URDF_PATH,
        ],
        name="generate_ur5_urdf",
        output="screen",
    )

    # Generate SRDF file alongside URDF (same reason: avoid Command-as-yaml parsing)
    generate_srdf = ExecuteProcess(
        cmd=[
            "bash", "-c",
            "xacro " + srdf_xacro_path + " name:=ur5 -o " + SRDF_PATH,
        ],
        name="generate_ur5_srdf",
        output="screen",
    )

    # Gazebo Sim
    ros2_lib = "/opt/ros/humble/lib"
    ws_lib = os.path.join(ws_root, "install", "gz_ros2_control", "lib")
    gz_plugin_path = f"{ws_lib}:{ros2_lib}"
    gz_sim = ExecuteProcess(
        cmd=["gz", "sim", "-r", world_path],
        name="gz_sim",
        output="screen",
        additional_env={
            "GZ_SIM_RESOURCE_PATH": gz_resource_path,
            "GZ_SIM_SYSTEM_PLUGIN_PATH": gz_plugin_path,
            "LD_LIBRARY_PATH": f"{ws_lib}:{ros2_lib}:{os.environ.get('LD_LIBRARY_PATH', '')}",
        },
    )

    # Robot State Publisher (use_sim_time for Gazebo /clock)
    # Read URDF from file written by generate_urdf above (avoids YAML parsing
    # of inline xacro output when URDF contains "1e-5" / "package://" etc).
    urdf_contents = open(URDF_PATH).read()
    robot_state_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": urdf_contents, "use_sim_time": True}],
    )

    # Spawn UR5 into Gazebo
    spawn_ur5 = ExecuteProcess(
        cmd=[
            "ros2", "run", "ros_gz_sim", "create",
            "-world", "grasp_world",
            "-name", "ur5",
            "-file", URDF_PATH,
            "-x", "0.0",
            "-y", "0.0",
            "-z", "0.0",
        ],
        name="spawn_ur5",
        output="screen",
    )

    # ── MoveIt2 move_group ───────────────────────────────────────
    # robot_description_semantic loaded from pre-generated SRDF file (same
    # reason as URDF: launch framework tries yaml-decoding Command output).
    srdf_contents = open(SRDF_PATH).read()

    # Load MoveIt configs
    # kinematics.yaml loaded as file path so ROS2 expands /** wildcard correctly
    kinematics_params = PathJoinSubstitution(
        [FindPackageShare("grasp_sim"), "config", "kinematics.yaml"]
    )
    joint_limits_yaml = load_yaml("ur_moveit_config", "config/joint_limits.yaml")
    ompl_yaml = load_yaml("ur_moveit_config", "config/ompl_planning.yaml")
    controllers_yaml = load_yaml("ur_moveit_config", "config/controllers.yaml")
    # Remove scaled_joint_trajectory_controller so it doesn't conflict with
    # the spawner at t=14 (double-loading causes JTC configure to fail).
    controllers_yaml.pop("scaled_joint_trajectory_controller", None)
    controllers_yaml["joint_trajectory_controller"]["default"] = True

    ompl_config = {
        "planning_plugin": "ompl_interface/OMPLPlanner",
        "request_adapters": "default_planner_request_adapters/AddTimeOptimalParameterization "
                            "default_planner_request_adapters/FixWorkspaceBounds "
                            "default_planner_request_adapters/FixStartStateBounds "
                            "default_planner_request_adapters/FixStartStateCollision "
                            "default_planner_request_adapters/FixStartStatePathConstraints",
        "start_state_max_bounds_error": 0.1,
    }
    ompl_config.update(ompl_yaml)

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        name="move_group",
        output="screen",
        parameters=[
            {"robot_description": urdf_contents},
            {"robot_description_semantic": srdf_contents},
            {"publish_robot_description_semantic": True},
            kinematics_params,
            {"robot_description_planning": joint_limits_yaml},
            {"move_group": ompl_config},
            {
                "moveit_simple_controller_manager": controllers_yaml,
                "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
            },
            {
                "moveit_manage_controllers": False,
                "trajectory_execution.allowed_execution_duration_scaling": 1.2,
                "trajectory_execution.allowed_goal_duration_margin": 0.5,
                "trajectory_execution.allowed_start_tolerance": 0.01,
                "trajectory_execution.execution_duration_monitoring": False,
            },
            {
                "publish_planning_scene": True,
                "publish_geometry_updates": True,
                "publish_state_updates": True,
                "publish_transforms_updates": True,
            },
            {"use_sim_time": True},
        ],
    )

    # Controller spawners — load controllers into Gazebo plugin's controller_manager
    # JTC manages all 7 joints (arm + gripper) as a single trajectory
    # Set P gain BEFORE robot spawn (gz_ros2_control reads it during URDF load)
    set_p_gain = ExecuteProcess(
        cmd=[
            "bash", "-c",
            "sleep 2 && "
            "ros2 param set /gz_ros2_control position_proportional_gain 5.0 && "
            "echo 'P gain set to 5.0'",
        ],
        name="set_p_gain",
        output="screen",
    )

    spawn_controllers = ExecuteProcess(
        cmd=[
            "bash", "-c",
            "sleep 3 && "
            "ros2 run controller_manager spawner joint_trajectory_controller "
            "--controller-manager-timeout 30 --ros-args -p use_sim_time:=true || true; "
            "ros2 run controller_manager spawner joint_state_broadcaster "
            "--controller-manager-timeout 30 --ros-args -p use_sim_time:=true || true; "
            "sleep 1 && "
            "ros2 service call /controller_manager/switch_controller "
            "controller_manager_msgs/srv/SwitchController "
            "'{activate_controllers: [joint_trajectory_controller, joint_state_broadcaster], strictness: 1}'",
        ],
        name="spawn_controllers",
        output="screen",
    )

    # ros_gz_bridge (config_file reads YAML list of bridges)
    gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="gz_bridge",
        output="screen",
        parameters=[{"config_file": bridge_yaml, "use_sim_time": True}],
    )

    return LaunchDescription([
        set_gz_path,
        generate_urdf,                                            # t=0
        generate_srdf,                                            # t=0
        TimerAction(period=2.0, actions=[gz_sim]),                # t=2
        TimerAction(period=5.0, actions=[set_p_gain]),            # t=5 (before UR5 spawn!)
        TimerAction(period=8.0, actions=[robot_state_pub]),       # t=8
        TimerAction(period=9.0, actions=[spawn_ur5]),             # t=9
        TimerAction(period=12.0, actions=[move_group]),           # t=12 (MoveIt2 IK)
        TimerAction(period=14.0, actions=[spawn_controllers]),    # t=14
        TimerAction(period=16.0, actions=[gz_bridge]),            # t=16
    ])
