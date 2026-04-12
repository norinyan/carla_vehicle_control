import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction, OpaqueFunction, ExecuteProcess
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration


# 你现在确认可用的坐标系（bridge下）：
# y 用正值；A点以 ego_vehicle.json 里的点为准
ROUTE_TO_SPAWN = {
    'A': '105.29,170.06,0.3,0.0,0.0,-0.326128',
    'B': '15.0,112.2,0.3,0.0,0.0,90.224867',
    'C': '199.99,230.60,0.3,0.0,0.0,-91.314404',
}


def _make_spawn_action(context):
    pkg_carla_vehicle_control = get_package_share_directory('carla_vehicle_control')
    pkg_carla_spawn_objects = get_package_share_directory('carla_spawn_objects')

    route = LaunchConfiguration('route').perform(context).strip().upper()
    if route not in ROUTE_TO_SPAWN:
        raise RuntimeError(f"route 参数无效: {route}，可选 A/B/C")

    vehicles_config_file = PathJoinSubstitution([
        pkg_carla_vehicle_control,
        'config',
        'ego_vehicle.json'
    ])

    spawn_objects_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_carla_spawn_objects, 'carla_spawn_objects.launch.py')
        ),
        launch_arguments={
            'objects_definition_file': vehicles_config_file,
            'spawn_point_ego_vehicle': ROUTE_TO_SPAWN[route],
            'spawn_sensors_only': 'False',
        }.items()
    )
    return [spawn_objects_launch]


def generate_launch_description():
    pkg_carla_ros_bridge = get_package_share_directory('carla_ros_bridge')

    host_arg = DeclareLaunchArgument('host', default_value='localhost')
    port_arg = DeclareLaunchArgument('port', default_value='2000')
    timeout_arg = DeclareLaunchArgument('timeout', default_value='10')
    town_arg = DeclareLaunchArgument('town', default_value='Town04')
    delay_arg = DeclareLaunchArgument('spawn_delay', default_value='5.0')
    route_arg = DeclareLaunchArgument('route', default_value='A')

    # 是否可视化参考路径点：默认不画，启动时传 visualize_route:=true 才画
    visualize_arg = DeclareLaunchArgument('visualize_route', default_value='false')
    
    visualize_delay_arg = DeclareLaunchArgument('visualize_delay', default_value='9.0')
    
    carla_bridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_carla_ros_bridge, 'carla_ros_bridge.launch.py')
        ),
        launch_arguments={
            'host': LaunchConfiguration('host'),
            'port': LaunchConfiguration('port'),
            'timeout': LaunchConfiguration('timeout'),
            'town': LaunchConfiguration('town'),
            'synchronous_mode': 'True',
            'synchronous_mode_wait_for_vehicle_control_command': 'True',

            'fixed_delta_seconds': '0.1',
        }.items()
    )

    delayed_spawn = TimerAction(
        period=LaunchConfiguration('spawn_delay'),
        actions=[OpaqueFunction(function=_make_spawn_action)]
    )
    visualize_route = TimerAction(
        period=LaunchConfiguration('visualize_delay'),
        actions=[
            ExecuteProcess(
                cmd=[
                    'python3',
                    '/home/nor/ros2_carla_ws/src/carla_vehicle_control/carla_vehicle_control/scenarios/trajectory_generator.py',
                    '--route',
                    LaunchConfiguration('route'),
                ],
                output='screen',
                condition=IfCondition(LaunchConfiguration('visualize_route'))
            )
        ]
    )

    return LaunchDescription([
            host_arg,
            port_arg,
            timeout_arg,
            town_arg,
            delay_arg,
            route_arg,
            visualize_arg,
            visualize_delay_arg,
            carla_bridge_launch,
            delayed_spawn,
            visualize_route,
    ])