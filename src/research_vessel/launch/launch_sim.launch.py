import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from geographic_msgs.msg import GeoPose
from rclpy import *
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit

def generate_launch_description():


    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!

    package_name='research_vessel' #<--- CHANGE ME
    pkg_share = get_package_share_directory(package_name)
    robot_localization_file_path = os.path.join(pkg_share, 'config', 'ekf_with_gps.yaml') 
    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    twist_mux_params = os.path.join(get_package_share_directory(package_name),'config','twist_mux.yaml')
    twist_mux = Node(
            package="twist_mux",
            executable="twist_mux",
            parameters=[twist_mux_params, {'use_sim_time': True}],
            remappings=[('/cmd_vel_out','/diff_cont/cmd_vel_unstamped')]
        )

    gazebo_params_file = os.path.join(get_package_share_directory(package_name),'config','gazebo_params.yaml')

    # Include the Gazebo launch file, provided by the gazebo_ros package
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
                    launch_arguments={'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_file}.items()
             )

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'my_bot'],
                        output='screen')

    #Currently not fused into EKF due to errors in icp when out of range of features.
    #Need to write wrapper to handle restarting the node when too many errors occur.
    laser_scan_matcher = Node(
    package='ros2_laser_scan_matcher',
    executable='laser_scan_matcher',
    name='laser_scan_matcher',
    respawn=True,
    output='screen',
    parameters=[{
        'laser_frame': 'laser_frame',
        'publish_odom': '/scanmatcherout',
        'use_sim_time': True,
        'restart': 1,
        'restart_threshold_mean_error': 0.01}])


  # Start the navsat transform node which converts GPS data into the world coordinate frame
    start_navsat_transform_cmd = Node(
    package='robot_localization',
    executable='navsat_transform_node',
    name='navsat_transform',
    output='screen',
    parameters=[robot_localization_file_path, 
    {'use_sim_time': True}],
    remappings=[
                ('/gps/fix','/gps/out'), 
                ('/imu/data', '/imu/out'),
                ('odometry/filtered', 'odometry/global')])



  # map->odom transform
    start_robot_localization_global_cmd = Node(
    package='robot_localization',
    executable='ekf_node',
    name='ekf_filter_node_map',
    output='screen',
    parameters=[robot_localization_file_path],
    remappings=[('odometry/filtered', 'odometry/global'),
                ('/set_pose', '/initialpose')])


  # odom->base_footprint transform
    start_robot_localization_local_cmd = Node(
    package='robot_localization',
    executable='ekf_node',
    name='ekf_filter_node_odom',
    output='screen',
    parameters=[ robot_localization_file_path],
    remappings=[('odometry/filtered', '/odom'),
                ('/set_pose', '/initialpose')])

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"],
    )
    
    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
    )

    nav = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','navigation_launch.py'
                )])
                , launch_arguments= {'use_sim_time': 'true'}.items()
    )

    delayed_nav = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[nav],
        )
    )

    # Launch them all!
    return LaunchDescription([
        rsp,
        twist_mux,
        gazebo,
        spawn_entity,
        diff_drive_spawner,
        joint_broad_spawner,
        laser_scan_matcher,
        start_robot_localization_global_cmd,
        start_robot_localization_local_cmd,
        start_navsat_transform_cmd,
        delayed_nav
    ])
