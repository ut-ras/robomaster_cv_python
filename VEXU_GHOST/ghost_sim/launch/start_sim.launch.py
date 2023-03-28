import os
import xacro
from ament_index_python.packages import get_package_share_directory

import launch
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

# Opaque Function hack to allow for better CLI arg parsing
def launch_setup(context, *args, **kwargs):
    ghost_sim_share_dir = get_package_share_directory('ghost_sim')

    # Select URDF Config based on CLI arg "enable_pid"
    enable_pid_param = LaunchConfiguration("enable_pid").perform(context)

    if(enable_pid_param in ['True', "true", True, 1]):
        enable_pid = True
    elif(enable_pid_param in ['False', "false", False, 0]):
        enable_pid = False
    else:
        # For now default to true
        enable_pid = True

    if(bool(enable_pid)):
        filename = "ghost1_sim_pid.urdf"
    else:
        filename = "ghost1_sim_voltage.urdf"

    # Load URDF and process to text
    # urdf_path = os.path.join(ghost_sim_share_dir, "urdf", filename)
    urdf_path = os.path.join(ghost_sim_share_dir, "urdf_robomasters", "sentry_base.urdf")
    doc = xacro.process(urdf_path)
    
    spawn_entity_args = ("-x 0.0 -y 0.0 -z 1.0 -R 0.0 -P 0.0 -Y 0.0 -entity sentry -topic robot_description").split()

    # Node to spawn robot model in Gazebo
    gazebo_ros = Node(
        package = "gazebo_ros",
        executable = "spawn_entity.py",
        output='screen',
        arguments=spawn_entity_args)

    # Node to publish robot joint transforms
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': False}, {"robot_description": doc}])

    return [gazebo_ros, robot_state_publisher]


def generate_launch_description():
    # Load relevant filepaths
    gazebo_ros_share_dir = get_package_share_directory('gazebo_ros')
    ghost_ros_share_dir = get_package_share_directory('ghost_ros')
    ghost_sim_share_dir = get_package_share_directory('ghost_sim')

    home_dir = os.path.expanduser('~')
    ghost_ros_base_dir = os.path.join(home_dir, "VEXU_GHOST", "ghost_ros")

    world_file = os.path.join(ghost_sim_share_dir, "urdf", "spin_up.world")
    rviz_config_path = os.path.join(ghost_ros_share_dir, 'rviz/urdf_config.rviz')

    # Simulator (Doesn't launch Simulator GUI by default, use CLI Arg "sim_gui" for debugging)
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_share_dir,
                         'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': world_file,
            'gui': LaunchConfiguration("sim_gui"),
            'verbose': LaunchConfiguration('verbose'),
            }.items()
    )

    ground_truth_publisher = Node(
        package='ghost_sim',
        executable='ground_truth_pose_publisher',
        name='ground_truth_pose_publisher',
    )

    # Launch RVIZ Display as primary GUI interface
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
    )

    # Joystick (Only launched if joystick CLI arg is set to True)
    joy_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                ghost_sim_share_dir,
                "launch",
                "joystick.launch.py"
            )
        ),
        condition=launch.conditions.IfCondition(LaunchConfiguration("joystick"))
    )

    estimator_node = Node(
        package='ghost_ros',
        executable='ghost_estimator_node',
        name='ghost_estimator_node',
        output='screen',
        parameters=[ghost_ros_base_dir + "/config/ghost_estimator_config.yaml"]
    )

    depth_node = Node(
        package='depth',
        executable='depth_calculations',
        name='depth_calculations'
    )

    state_machine_node = Node(
        package='ghost_ros',
        executable='robot_state_machine_node',
        name='ghost_state_machine_node',
        output='screen',
        parameters=[ghost_ros_base_dir + "/config/ghost_state_machine_config.yaml"]
    )

    sentry_control_node = Node(
        package='sentry_control',
        executable='sentry_control',
        name='sentry_control'
    )

    return LaunchDescription([
        DeclareLaunchArgument(name='enable_pid', default_value='true'),
        DeclareLaunchArgument(name='joystick', default_value='false'),
        DeclareLaunchArgument('sim_gui', default_value='false'),
        DeclareLaunchArgument('verbose', default_value='true'),
        simulation,
        ground_truth_publisher,
        rviz_node,
        joy_launch_description,
        estimator_node,
        depth_node,
        state_machine_node,
        sentry_control_node,
        OpaqueFunction(function = launch_setup)
    ])