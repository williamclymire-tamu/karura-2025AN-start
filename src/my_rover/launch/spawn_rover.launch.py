from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    xacro_file = PathJoinSubstitution([
        FindPackageShare('my_rover'),
        'urdf',
        'rover.urdf.xacro'
    ])

    robot_description_content = ParameterValue(
        Command(['xacro ', xacro_file]),
        value_type=str
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': use_sim_time
        }]
    )

    robot_controllers = PathJoinSubstitution([
        FindPackageShare('my_rover'),
        'config',
        'rover_controllers.yaml'
    ])

    gzserver_proc = ExecuteProcess(
        cmd=['/usr/bin/gzserver', '--verbose', '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )

    gzclient_proc = ExecuteProcess(
        cmd=['/usr/bin/gzclient'],
        output='screen'
    )

    spawn_rover_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'my_rover',
            '-z', '0.12'   # adjusted for correct wheel-ground contact
        ]
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )

    diff_drive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'diff_drive_controller',
            '--param-file', robot_controllers
        ],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Use simulation clock'),
        gzserver_proc,
        gzclient_proc,
        robot_state_publisher_node,
        spawn_rover_node,
        joint_state_broadcaster_spawner,
        diff_drive_controller_spawner
    ])
