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
    # rsp = IncludeLaunchDescription(
    #             PythonLaunchDescriptionSource([os.path.join(
    #                 get_package_share_directory(package_name),'launch','rsp.launch.py'
    #             )]), launch_arguments={'use_sim_time': 'true'}.items()
    # )


    pcl_to_laserscan = Node(
            package="pointcloud_to_laserscan",
            executable="pointcloud_to_laserscan_node",
            parameters=[{'use_sim_time': True},
                        {'range_min': 0.0},
                        {'range_max': 130.0},
                        {'max_height': 0.0},
                        {'min_height': 0.0},
                        ],
            remappings=[('/cloud_in','/cluster_points'),
                        ('/scan','/scan_processed')]
        )

    # gazebo_params_file = os.path.join(get_package_share_directory(package_name),'config','gazebo_params.yaml')

    # # Include the Gazebo launch file, provided by the gazebo_ros package
    # gazebo = IncludeLaunchDescription(
    #             PythonLaunchDescriptionSource([os.path.join(
    #                 get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
    #                 launch_arguments={'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_file}.items()
    #          )

    # # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    # spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
    #                     arguments=['-topic', 'robot_description',
    #                                '-entity', 'my_bot'],
    #                     output='screen')

    #Currently not fused into EKF due to errors in icp when out of range of features.
    #Need to write wrapper to handle restarting the node when too many errors occur.
    # laser_scan_matcher = Node(
    # package='ros2_laser_scan_matcher',
    # executable='laser_scan_matcher',
    # name='laser_scan_matcher',
    # respawn=True,
    # output='screen',
    # parameters=[{
    #     'laser_frame': 'laser_frame',
    #     'publish_odom': '/scanmatcherout',
    #     'use_sim_time': True,
    #     'restart': 1,
    #     'restart_threshold_mean_error': 0.01}])




  # Start the navsat transform node which converts GPS data into the world coordinate frame
    start_navsat_transform_cmd = Node(
    package='robot_localization',
    executable='navsat_transform_node',
    name='navsat_transform',
    output='screen',
    parameters=[robot_localization_file_path, 
    {'use_sim_time': True}],
    remappings=[
                ('gps/fix','wamv/sensors/gps/gps/fix'), 
                ('odometry/filtered', 'odometry/global')])


    rf2o = Node(
    package="rf2o_laser_odometry",
    executable="rf2o_laser_odometry_node",
    name="rf2o_laser_odometry",
    parameters=[{
                'laser_scan_topic' : '/scan_processed',
                'odom_topic' : '/odom_rf2o',
                'publish_tf' : False,
                'base_frame_id' : 'wamv/wamv/base_link',
                'odom_frame_id' : 'odom',
                'init_pose_from_topic' : 'odometry/global',
                'freq' : 20.0}])



  # map->odom transform
    start_robot_localization_global_cmd = Node(
    package='robot_localization',
    executable='ekf_node',
    name='ekf_filter_node_map',
    output='screen',
    parameters=[robot_localization_file_path],
    remappings=[('odometry/filtered', 'odometry/global'),
                ('/set_pose', '/initialpose')])


    map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'])

#   # odom->base_footprint transform
#     start_robot_localization_local_cmd = Node(
#     package='robot_localization',
#     executable='ekf_node',
#     name='ekf_filter_node_odom',
#     output='screen',
#     parameters=[ robot_localization_file_path],
#     remappings=[('odometry/filtered', '/odom'),
#                 ('/set_pose', '/initialpose')])

    lidar_filter = Node(
        package="research_vessel",
        executable="lidar_filter.py",
        name="scan_filter_node",
    )

    topic_relay = Node(
        package='research_vessel',
        executable='topic_relay.py',
        name='odometry_to_pose'
    )

    config_dir_imu = os.path.join(get_package_share_directory('research_vessel'), 'config')
    imu_filter = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter',
        remappings=[('/imu/data_raw','/wamv/sensors/imu/imu/data')],
        output='screen',
        parameters=[os.path.join(config_dir_imu, 'imu_filter.yaml')],
    )

    frame_id        = 'wamv/wamv/base_link/lidar_wamv_sensor'
    filtered_cloud  = 'filtered_cloud'
    stateDim        = 4 # [x,y,v_x,v_y]//,w,h]
    measDim         = 2 # [z_x,z_y,z_w,z_h]
    ctrlDim         = 0
     
    multiple_object_tracking_lidar_params = [
    {"use_sim_time": True},
    {"frame_id": frame_id},
    {"filtered_cloud": filtered_cloud},
    {"stateDim": stateDim},
    {"measDim": measDim},
    {"ctrlDim": ctrlDim},
    ]

    multiple_object_tracking_lidar = Node(
        package='multiple_object_tracking_lidar',
        executable='multiple_object_tracking_lidar_node',
        name='multiple_object_tracking_lidar',
        parameters=multiple_object_tracking_lidar_params,
        output={
            'stdout': 'screen',
            'stderr': 'screen',
        }
    )
    nav = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','navigation.launch.py'
                )])
                , launch_arguments= {'use_sim_time': 'true'}.items()
    )


    # Launch them all!
    return LaunchDescription([
        # rsp,
        # twist_mux,
        # pcl_to_laserscan,
        # gazebo,
        # spawn_entity,
        # diff_drive_spawner,
        # joint_broad_spawner,
        # laser_scan_matcher,
        pcl_to_laserscan,
        imu_filter,
        topic_relay,
        # start_robot_localization_local_cmd,
        rf2o,
        lidar_filter,
        multiple_object_tracking_lidar,
        start_navsat_transform_cmd,
        start_robot_localization_global_cmd,
        map_to_odom,
        
        # baselinks
        nav
    ])
