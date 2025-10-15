from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # use simulation clock
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # robot description
    xacro_file = PathJoinSubstitution([
        FindPackageShare('my_rover'),
        'urdf',
        'rover.urdf.xacro'
    ])
    
    robot_description_content = ParameterValue(
        Command(['xacro ', xacro_file]),
        value_type=str
    )
    
    # robot state publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': use_sim_time
        }]
    )
    
    # controller YAML file
    robot_controllers = PathJoinSubstitution([
        FindPackageShare('my_rover'),
        'config',
        'rover_controllers.yaml'
    ])
    
    # launch Gazebo Classic
    gzserver_proc = ExecuteProcess(
        cmd=['/usr/bin/gzserver', '--verbose', '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )
    
    gzclient_proc = ExecuteProcess(
        cmd=['/usr/bin/gzclient'],
        output='screen'
    )
    
    # spawn the rover at proper height so wheels touch ground
    spawn_rover_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'my_rover',
            '-z', '0.35' 
        ] 
    )
    
    # controllers
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
    
    # build launch description
    ld = LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true',
                            description='Use simulation clock'),
        gzserver_proc,
        gzclient_proc,
        robot_state_publisher_node,
        spawn_rover_node,
        joint_state_broadcaster_spawner,
        diff_drive_controller_spawner
    ])
    
    return ld