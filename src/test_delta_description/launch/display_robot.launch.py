from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import Command, LaunchConfiguration, FindExecutable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import os

def generate_launch_description():
    package_name = 'test_delta_description'
    pkg_share = get_package_share_path(package_name)
    
    # Launch arguments
    default_model_path = pkg_share / 'urdf/delta_robot.urdf'
    default_rviz_config_path = pkg_share / 'rviz/robot.rviz'

    model_arg = DeclareLaunchArgument(
        name='model', 
        default_value=str(default_model_path),
        description='Absolute path to robot urdf file'
    )
    
    rviz_arg = DeclareLaunchArgument(
        name='rvizconfig', 
        default_value=str(default_rviz_config_path),
        description='Absolute path to rviz config file'
    )

    robot_description = ParameterValue(
        Command(['xacro ', LaunchConfiguration('model')]),
        value_type=str
    )

    # Main nodes
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'test_delta_description': robot_description }],
        output='screen'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )

    # Inverse kinematics node - sửa thành ExecuteProcess
    delta_inverse_kinematics_node = Node(
        package='test_delta_description',
        executable='delta_robot_inverse_kinematics',
        name='delta_inverse_kinematics',
        output='screen'
    )

    # Position publisher node - sửa thành ExecuteProcess
    position_publisher_node = Node(
        package='test_delta_description',
        executable='position_publisher',
        name='position_publisher',
        output='screen'
    )

    # Static transform for world to base_link
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'base_link'],
        output='screen'
    )
    
    

    return LaunchDescription([
        model_arg,
        rviz_arg,
        static_tf_node,
        robot_state_publisher_node, 
        rviz_node,
        delta_inverse_kinematics_node,
        position_publisher_node
    ])