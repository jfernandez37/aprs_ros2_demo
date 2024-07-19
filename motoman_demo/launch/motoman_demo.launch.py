import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

from launch.actions import (
    OpaqueFunction,
)

from launch.actions import IncludeLaunchDescription

from moveit_configs_utils import MoveItConfigsBuilder

def launch_setup(context, *args, **kwargs):

    motoman_description = LaunchDescription([IncludeLaunchDescription(PythonLaunchDescriptionSource([FindPackageShare("motoman_description"), '/launch', '/motoman_bringup.launch.py']))])

    urdf = os.path.join(get_package_share_directory("motoman_description"), "urdf/motoman.urdf.xacro")

    moveit_config = (
        MoveItConfigsBuilder("motoman", package_name="motoman_moveit_config")
        .robot_description(urdf)
        .robot_description_semantic(file_path="config/motoman.srdf")
        .trajectory_execution(file_path="config/controllers.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )
        
    # Nist Competitor node
    motoman_node = Node(
        package="motoman_demo",
        executable="motoman_kitting",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
        ],
    )



    nodes_to_start = [
    	motoman_description,
        motoman_node,
    ]

    return nodes_to_start

def generate_launch_description():
    declared_arguments = []

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
