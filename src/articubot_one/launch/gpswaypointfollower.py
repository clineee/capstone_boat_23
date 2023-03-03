from geometry_msgs.msg import PoseStamped
from robot_navigator import BasicNavigator, NavigationResult
import time
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from robot_localization.srv import FromLL
import yaml
import sys

#reads gps waypoints (lat/long) from a yaml file (passed as an argument) 
# in the format [[lat, long],[lat, long]]
def get_gps_points():
    with open(sys.argv[1], 'r') as yaml_data:
        gps_points = yaml.safe_load(yaml_data)
        print(gps_points)
    return gps_points

#converts gps coordinates into the cartesian coordinate frame 
#using the navsat transform node function fromLL and stores them in a list
def convert_LL(gps_points):
    rclpy.init()
    node = Node("my_node")
    client = node.create_client(FromLL, "/fromLL")
    map_points = []
    while gps_points:
        waypoint = gps_points.pop(0)

        request = FromLL.Request()
        request.ll_point.latitude = waypoint[0]
        request.ll_point.longitude = waypoint[1]

        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future)

        if future.result() is not None:
            response = future.result()

            map_points.append([response.map_point.x, response.map_point.y])
        else:
            print("Service call failed")
    node.destroy_node()
    rclpy.shutdown()
    return map_points
#uses nav2 simple commander python api to navigate using the global odom ekf 
#for PoseWithCovarianceStamped instead of amcl
def main():
    gps_points = get_gps_points()
    map_points = convert_LL(gps_points)
    print(map_points)
    rclpy.init()
    nav = BasicNavigator()
    nav.waitUntilNav2Active()
    for points in map_points:
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = nav.get_clock().now().to_msg()
        #pose is not needed because of planner parameter "use_final_approach_orientation: true"
        goal_pose.pose.position.x = points[0]
        goal_pose.pose.position.y = points[1]
        goal_pose.pose.position.z = 0.0
        nav.goToPose(goal_pose)
        #while robot is navigating do something else, in this case read 
        # back distance to goal in meters using distance_remaining feedback
        i = 0
        while not nav.isNavComplete():
            i += 1
            feedback = nav.getFeedback()
            if feedback and i % 5 == 0:
                print('Distance remaining: ' + '{:.2f}'.format(feedback.distance_remaining) + ' meters.')
        result = nav.getResult()
        if result == NavigationResult.SUCCEEDED:
            print('Goal succeeded!')
        elif result == NavigationResult.CANCELED:
            print('Goal was canceled!')
        elif result == NavigationResult.FAILED:
            print('Goal failed!')
        else:
            print('Goal has an invalid return status!')
    nav.lifecycleShutdown()
    exit(0)

if __name__ == "__main__":
    main()


