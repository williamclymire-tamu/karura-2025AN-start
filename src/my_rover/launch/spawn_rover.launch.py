from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, ExecuteProcess
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    # Use simulation clock
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Robot state publisher with xacro
    def robot_state_publisher(context):
        robot_description_content = Command([
            'xacro',
            PathJoinSubstitution([
                FindPackageShare('my_rover'),
                'urdf',
                'rover.urdf.xacro'
            ])
        ])
        robot_description = {'robot_description': robot_description_content}
        return [Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[robot_description, {'use_sim_time': use_sim_time}]
        )]

    # Controller YAML file
    robot_controllers = PathJoinSubstitution([
        FindPackageShare('my_rover'),
        'config',
        'rover_controllers.yaml'
    ])

    # Launch Gazebo Classic
    gzserver_proc = ExecuteProcess(
        cmd=['/usr/bin/gzserver', '--verbose', '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )

    gzclient_proc = ExecuteProcess(
        cmd=['/usr/bin/gzclient'],
        output='screen'
    )

    # Spawn the rover
    spawn_rover_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=['-topic', 'robot_description', '-entity', 'my_rover']
    )

    # Controllers
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

    # Build launch description
    ld = LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation clock'),

        gzserver_proc,
        gzclient_proc,
        spawn_rover_node,
        OpaqueFunction(function=robot_state_publisher),
        joint_state_broadcaster_spawner,
        diff_drive_controller_spawner
    ])

    return ld
