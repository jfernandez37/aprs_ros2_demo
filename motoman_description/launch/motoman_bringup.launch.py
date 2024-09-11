import os
import xacro
from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.actions import (
    OpaqueFunction,
)
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from ament_index_python.packages import get_package_share_directory

from moveit_configs_utils import MoveItConfigsBuilder

def launch_setup(context, *args, **kwargs):

    urdf = os.path.join(get_package_share_directory('motoman_description'), 'urdf', 'motoman.urdf.xacro')
        
    doc = xacro.process_file(urdf)

    robot_description = {"robot_description": doc.toxml()}

    robot_controllers = PathJoinSubstitution([FindPackageShare("motoman_description"), "config", "motoman_controllers.yaml",])
    rviz_config_file = PathJoinSubstitution([FindPackageShare("motoman_description"), "config", "motoman.rviz",])

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_controllers],
        output="both",
        remappings=[
            ("~/robot_description", "/robot_description"),
        ],
    )
    
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[
            robot_description
        ],
    )

    joint_state_broadcaster = Node(
            package='controller_manager',
            executable='spawner',
            name='joint_state_broadcaster_spawner',
            arguments=[
                'joint_state_broadcaster'
            ]
        )
    
    joint_trajectory_controller = Node(
            package='controller_manager',
            executable='spawner',
            name='controller_spawner',
            # namespace=robot,
            arguments=[
                'joint_trajectory_controller'
            ]
        )
    
    moveit_config = (
        MoveItConfigsBuilder("motoman", package_name="motoman_moveit_config")
        .robot_description(urdf)
        .robot_description_semantic(file_path="config/motoman.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )    
    
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[moveit_config.to_dict(),],
    )    
    
    
    # motoman_gripper_control = Node(
    #     package="motoman_hardware",
    #     executable="motoman_gripper_control.py",
    #     output = 'screen'
    # )    

    nodes_to_start = [
        control_node,
        robot_state_publisher,
        joint_state_broadcaster,
        joint_trajectory_controller,
        # motoman_gripper_control,
        # rviz_node
    ]

    return nodes_to_start


def generate_launch_description():
    declared_arguments = []

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
