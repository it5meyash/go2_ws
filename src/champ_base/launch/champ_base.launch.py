import os
import xacro
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    go2_description_share = get_package_share_directory('go2_description')
    xacro_path = os.path.join(go2_description_share, 'xacro', 'go2_description.xacro')
    robot_description_xml = xacro.process_file(xacro_path).toxml()

    gait_config = PathJoinSubstitution(
        [FindPackageShare('go2_description'), 'config', 'gait.yaml']
    )

    common_params = [
        gait_config,
        {
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'urdf': robot_description_xml,
        },
    ]

    quadruped_controller_node = Node(
        package='champ_base',
        executable='quadruped_controller_node',
        name='quadruped_controller_node',
        output='screen',
        parameters=common_params,
    )

    state_estimation_node = Node(
        package='champ_base',
        executable='state_estimation_node',
        name='state_estimation_node',
        output='screen',
        parameters=common_params,
    )

    message_relay_node = Node(
        package='champ_base',
        executable='message_relay_node',
        name='message_relay_node',
        output='screen',
        parameters=[gait_config, {'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        quadruped_controller_node,
        state_estimation_node,
        message_relay_node,
    ])