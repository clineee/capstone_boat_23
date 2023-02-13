Guide to getting the LIDAR 

install dependencies:
`rosdep install --from-path src`
you might need to do other stuff to init rosdep first, google it
to build:
`source /opt/ros/humble/local_setup.bash`
in the main directory (capstone_boat_23)
`colcon build`
`source install/setup.bash`
if you use a different shell (csh, ect) there are different scrupts
to run my stuff:
`ros2 launch multiple_object_tracking_lidar multiple_object_tracking_lidar.launch.py`
`ros2 run lidar_faker faker`

To run anything, you need to do both of the source commands first
