import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('carla_vehicle_control')
    default_params = os.path.join(pkg_share, 'config', 'vehicle_params.yaml')
    default_csv_dir = os.path.join(pkg_share, 'config')

    role_arg = DeclareLaunchArgument('role_name', default_value='ego_vehicle')
    route_arg = DeclareLaunchArgument('route', default_value='A')
    params_arg = DeclareLaunchArgument('params_path', default_value=default_params)
    csv_arg = DeclareLaunchArgument('csv_dir', default_value=default_csv_dir)
    hz_arg = DeclareLaunchArgument('control_hz', default_value='10.0')
    save_arg = DeclareLaunchArgument('save_log', default_value='true')

    control_node = Node(
        package='carla_vehicle_control',
        executable='vehicle_control_node',
        name='vehicle_control_node',
        output='screen',
        parameters=[{
            'role_name': LaunchConfiguration('role_name'),
            'route': LaunchConfiguration('route'),
            'params_path': LaunchConfiguration('params_path'),
            'csv_dir': LaunchConfiguration('csv_dir'),
            'control_hz': LaunchConfiguration('control_hz'),
            'save_log': LaunchConfiguration('save_log'),
        }]
    )

    return LaunchDescription([
        role_arg,
        route_arg,
        params_arg,
        csv_arg,
        hz_arg,
        save_arg,
        control_node,
    ])
