#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable

def _get_script_path(script_name):
    try:
        pkg_prefix = get_package_prefix("lab8")
        path = os.path.join(pkg_prefix, "lib", "lab8", script_name)
        if os.path.exists(path):
            return os.path.abspath(path)
    except Exception:
        pass
    _dir = os.path.dirname(os.path.abspath(__file__))
    for _ in range(5):
        s = os.path.join(_dir, "scripts", script_name)
        if os.path.isfile(s):
            return s
        if os.path.basename(_dir) == "lab8" and os.path.isdir(os.path.join(_dir, "scripts")):
            return os.path.join(_dir, "scripts", script_name)
        _dir = os.path.dirname(_dir)
        if not _dir or _dir == os.path.dirname(_dir):
            break
    raise FileNotFoundError(f"Cannot find {script_name}")

def generate_launch_description():
    pkg_tb4_nav = get_package_share_directory("turtlebot4_navigation")
    try:
        pkg_lab8 = get_package_share_directory("lab8")
    except Exception:
        pkg_lab8 = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
    aruco_params = os.path.join(pkg_lab8, "config", "aruco_params.yaml")
    nav2_params  = os.path.join(pkg_lab8, "config", "nav2.yaml")
    slam_params  = os.path.join(pkg_lab8, "config", "slam.yaml")
    slam_launch  = PathJoinSubstitution([pkg_tb4_nav, "launch", "slam.launch.py"])

    return LaunchDescription([
        SetEnvironmentVariable("ROS_DOMAIN_ID", "99"),
        DeclareLaunchArgument("skip_undock", default_value="false"),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([slam_launch]),
            launch_arguments=[("use_sim_time", "false"), ("params", slam_params)]
        ),

        Node(package="nav2_controller",       executable="controller_server",  name="controller_server",  output="screen", parameters=[nav2_params]),
        Node(package="nav2_planner",          executable="planner_server",     name="planner_server",     output="screen", parameters=[nav2_params]),
        Node(package="nav2_behaviors",        executable="behavior_server",    name="behavior_server",    output="screen", parameters=[nav2_params]),
        Node(package="nav2_bt_navigator",     executable="bt_navigator",       name="bt_navigator",       output="screen", parameters=[nav2_params]),
        Node(package="nav2_waypoint_follower",executable="waypoint_follower",  name="waypoint_follower",  output="screen", parameters=[nav2_params]),
        Node(package="nav2_velocity_smoother",executable="velocity_smoother",  name="velocity_smoother",  output="screen", parameters=[nav2_params]),
        ExecuteProcess(
            cmd=["bash", "-c",
                 "sleep 3 && "
                 "ros2 lifecycle set /controller_server configure && ros2 lifecycle set /controller_server activate && "
                 "ros2 lifecycle set /planner_server configure && ros2 lifecycle set /planner_server activate && "
                 "ros2 lifecycle set /behavior_server configure && ros2 lifecycle set /behavior_server activate && "
                 "ros2 lifecycle set /bt_navigator configure && ros2 lifecycle set /bt_navigator activate && "
                 "ros2 lifecycle set /velocity_smoother configure && ros2 lifecycle set /velocity_smoother activate && "
                 "ros2 lifecycle set /waypoint_follower configure && ros2 lifecycle set /waypoint_follower activate && "
                 "echo ALL_NODES_ACTIVE"],
            output="screen",
        ),

        ExecuteProcess(cmd=["python3", _get_script_path("odom_to_tf.py"),          "--ros-args", "-p", "use_sim_time:=false"], output="screen"),
        ExecuteProcess(cmd=["python3", _get_script_path("aruco_detector.py"),      "--ros-args", "--params-file", aruco_params, "-p", "use_sim_time:=false"], output="screen"),
        ExecuteProcess(cmd=["python3", _get_script_path("explore_action_server.py"),"--ros-args", "-p", "use_sim_time:=false"], output="screen"),
        ExecuteProcess(cmd=["python3", _get_script_path("orchestrator.py"),        "--ros-args", "-p", "use_sim_time:=false",
                            "-p", ["skip_undock:=", LaunchConfiguration("skip_undock")]], output="screen"),
    ])
