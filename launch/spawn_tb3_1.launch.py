import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    # Get the robot model
    TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']
    model_folder = 'turtlebot3_' + TURTLEBOT3_MODEL
    urdf_path = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'models',
        model_folder,
        'model_tb3_1.sdf'
    )

    # Launch arguments
    x_pose_arg = DeclareLaunchArgument('x_pose', default_value='0.0')
    y_pose_arg = DeclareLaunchArgument('y_pose', default_value='0.0')
    z_pose_arg = DeclareLaunchArgument('z_pose', default_value='0.01')
    yaw_angle_arg = DeclareLaunchArgument('yaw_angle', default_value='0.0')
    robot_namespace_arg = DeclareLaunchArgument('robot_namespace', default_value='robot1')
    robot_name_arg = DeclareLaunchArgument('robot_name', default_value=TURTLEBOT3_MODEL)

    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    z_pose = LaunchConfiguration('z_pose')
    yaw_angle = LaunchConfiguration('yaw_angle')
    robot_namespace = LaunchConfiguration('robot_namespace')
    robot_name = LaunchConfiguration('robot_name')

    # Spawn robot in Gazebo
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', robot_name,
            '-file', urdf_path,
            '-x', x_pose,
            '-y', y_pose,
            '-z', z_pose,
            '-Y', yaw_angle
        ],
        namespace=robot_namespace,
        output='screen'
    )

    # Bridge configuration
    bridge_params = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'params',
        'tb3_1_bridge.yaml'
    )

    # Start bridge for tb3
    bridge_tb3 = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        namespace=robot_namespace,
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ],
        output='screen'
    )

    image_bridge_tb3 = Node(
        package='ros_gz_image',
        executable='image_bridge',
        namespace=robot_namespace,
        arguments=['/image_raw'],
        output='screen'
    )


    # Construct launch description
    ld = LaunchDescription()

    # Declare launch arguments
    for arg in [x_pose_arg, y_pose_arg, z_pose_arg, yaw_angle_arg, robot_namespace_arg, robot_name_arg]:
        ld.add_action(arg)

    # Launch actions
    ld.add_action(spawn_robot)
    ld.add_action(bridge_tb3)
    ld.add_action(image_bridge_tb3)

    return ld
