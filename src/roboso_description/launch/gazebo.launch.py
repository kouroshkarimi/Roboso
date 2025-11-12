from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.event_handlers import OnProcessStart

def generate_launch_description():
    # ---------------- Paths ----------------
    pkg_share = FindPackageShare('roboso_description')
    urdf_file = PathJoinSubstitution([pkg_share, 'urdf', 'so101.urdf.xacro'])
    rviz_config = PathJoinSubstitution([pkg_share, 'rviz', 'display.rviz'])
    controllers_yaml = PathJoinSubstitution([pkg_share, 'config', 'so101_controllers.yaml'])

    gazebo_pkg_share = FindPackageShare('ros_gz_sim')
    gazebo_launch_file = PathJoinSubstitution([gazebo_pkg_share, 'launch', 'gz_sim.launch.py'])

    robot_description = ParameterValue(
        Command(['xacro ', urdf_file]), value_type=str
    )

    # ---------------- Arguments ----------------
    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation clock if true'
    )

    # ---------------- Nodes ----------------
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description
        }]
    )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'fixed_frame': 'world'
        }]
    )

    # ---------------- Gazebo ----------------
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gazebo_launch_file]),
        launch_arguments={
            'gz_args': '-r empty.sdf'
        }.items()
    )

    # ---------------- ROS-Gazebo Bridge ----------------
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_bridge',
        output='screen',
        arguments=[
            '/world/default/model/ardurobot/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/cmd_vel@geometry_msgs/msg/Twist[gz.msgs.Twist',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
        ],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # ---------------- Spawn Robot ----------------
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'ardurobot',
            '-x', '0', '-y', '0', '-z', '0',  # small lift above ground
            '-Y', '0'
        ],
        output='screen'
    )

    # ---------------- ROS2 Control ----------------
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[{'use_sim_time': use_sim_time},
                    {'robot_description': robot_description},
                    controllers_yaml],
        output='screen'
    )

    # Spawners for controllers
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    # Ensure controllers start after Gazebo spawn
    start_controllers = RegisterEventHandler(
        OnProcessStart(
            target_action=spawn_entity,
            on_start=[joint_state_broadcaster_spawner, arm_controller_spawner]
        )
    )

    # ---------------- Launch Description ----------------
    return LaunchDescription([
        declare_use_sim_time,
        gz_sim,
        robot_state_publisher,
        rviz2,
        gz_bridge,
        spawn_entity,
        controller_manager,
        start_controllers
    ])
