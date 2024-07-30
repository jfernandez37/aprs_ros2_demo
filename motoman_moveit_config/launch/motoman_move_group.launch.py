from launch import LaunchDescription
from launch.actions import OpaqueFunction

import os

from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from moveit_configs_utils import MoveItConfigsBuilder

from ament_index_python.packages import get_package_share_directory

def launch_setup(context, *args, **kwargs):
    
    urdf = os.path.join(get_package_share_directory("motoman_description"), "urdf/motoman.urdf.xacro")
    rviz_config_file = PathJoinSubstitution([FindPackageShare("motoman_description"), "config", "motoman.rviz",])

        
    moveit_config = (
        MoveItConfigsBuilder("motoman", package_name="motoman_moveit_config")
        .robot_description(file_path=urdf)
        .robot_description_semantic(file_path="config/motoman.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )
        
    # Move group node
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
        ],
    )
    
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[moveit_config.to_dict(),],
    )    

    nodes_to_start = [
        move_group_node,
        rviz_node
    ]

    return nodes_to_start


def generate_launch_description():
    declared_arguments = []

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])