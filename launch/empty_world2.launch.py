import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution, PathJoinSubstitution, Command
from launch_ros.actions import Node


def generate_launch_description():
    # Directories
    launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')
    ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_turtlebot3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    pkg_turtlebot3_description = get_package_share_directory('turtlebot3_description')
    pkg_turtlebot3_slam_toolbox = get_package_share_directory('turtlebot3_slam_toolbox')


    # Config
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world_file = LaunchConfiguration('world', default='empty.sdf')

    # Set Gazebo resource path
    set_env_vars_resources = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(pkg_turtlebot3_gazebo, 'models')
    )

    # rviz_node = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz2',
    #     output='screen',
    #     arguments=['-d', os.path.join(pkg_turtlebot3_gazebo, 'rviz', 'tb3_gazebo_upd.rviz')],
    #     parameters=[{'use_sim_time': use_sim_time}]
    # )

    # Launch Gazebo
    gazebo_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': [PathJoinSubstitution([
                pkg_turtlebot3_gazebo,
                'worlds',
                world_file
            ]),
            TextSubstitution(text=' -r -v -v1')],
            'on_exit_shutdown': 'true'
        }.items()
    )

    


    # URDF description
    urdf_file = os.path.join(pkg_turtlebot3_description, 'urdf', 'turtlebot3_burger.urdf.xacro')
    robot_description = Command(['xacro ', urdf_file])


    print("###################################")
    print(os.path.join(
                pkg_turtlebot3_gazebo, 'config', 'ekf_tb3_1.yaml'))
    print("#####################################")

    # === Robot 1 ===
    tb3_1 = GroupAction([
        # State publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace='tb3_1',
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': robot_description,
                'frame_prefix': 'tb3_1/'
            }],
            output='screen'
        ),
        
        
        Node(
            package='robot_localization',
            executable='ekf_node',
            namespace='tb3_1',
            name='ekf_filter_node',
            output='screen',
            parameters=[os.path.join(
                pkg_turtlebot3_gazebo, 'config', 'ekf_tb3_1.yaml'),
                {'use_sim_time': use_sim_time}
            ]
        ),
        

        # Spawner
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_file_dir, 'spawn_tb3_1.launch.py')
            ),
            launch_arguments={
                'x_pose': TextSubstitution(text='0.0'),
                'y_pose': TextSubstitution(text='0.0'),
                'robot_namespace': TextSubstitution(text='tb3_1'),
                'robot_name': TextSubstitution(text='tb3_1')
            }.items()
        )
    
    ])

    # === Robot 2 ===
    tb3_2 = GroupAction([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace='tb3_2',
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': robot_description,
                'frame_prefix': 'tb3_2/'
            }],
            output='screen'
        ),
        
        Node(
            package='robot_localization',
            executable='ekf_node',
            namespace='tb3_2',
            name='ekf_filter_node',
            output='screen',
            parameters=[os.path.join(
                pkg_turtlebot3_gazebo, 'config', 'ekf_tb3_2.yaml'),
                {'use_sim_time': use_sim_time}
            ]
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_file_dir, 'spawn_tb3_2.launch.py')
            ),
            launch_arguments={
                'x_pose': TextSubstitution(text='2.0'),
                'y_pose': TextSubstitution(text='0.0'),
                'robot_namespace': TextSubstitution(text='tb3_2'),
                'robot_name': TextSubstitution(text='tb3_2')
            }.items()
        )
    ])
    
    # Odometry bridges - now including pose information
    odom_bridges = GroupAction([
        
        # Node to bridge messages like /cmd_vel and /odom
        Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"
        ],
        output="screen",
        parameters=[
            {'use_sim_time': True},
        ]
        )

    ])
    

    # Assemble launch description
    ld = LaunchDescription()
    ld.add_action(set_env_vars_resources)
    ld.add_action(gazebo_cmd)
    ld.add_action(odom_bridges)
    ld.add_action(tb3_1)
    ld.add_action(tb3_2)
    #ld.add_action(rviz_node)
    
    return ld