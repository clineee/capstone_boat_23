import sys
import yaml
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from robot_localization.srv import FromLL, ToLL
from robot_navigator import BasicNavigator, NavigationResult


class GPSPublisher(Node):
    def __init__(self):
        super().__init__('gps_publisher')

        self.publisher = self.create_publisher(
            String,
            '/next_pose',
            10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = "Publishing!"
        self.publisher.publish(msg)


class PathSubscriber(Node):
    def __init__(self):
        super().__init__('path_subscriber')

        self.subscription = self.create_subscription(
            Path,
            '/received_global_plan',
            self.path_callback,
            10
        )
        self.subscription
        self.next_pose = None;

    def path_callback(self, msg):
        global gps_pose
        if len(msg.poses) >= 2 and msg.poses[1] != self.next_pose:
            self.next_pose = msg.poses[1]
            print(self.next_pose)
            gps_pose = self.next_pose


#reads gps waypoints (lat/long) from a yaml file (passed as an argument) 
# in the format [[lat, long],[lat, long]]
def get_gps_points():
    with open(sys.argv[1], 'r') as yaml_data:
        gps_points = yaml.safe_load(yaml_data)
        print(gps_points)
    return gps_points


def convert_from_LL(gps_points):
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
    return map_points

#uses nav2 simple commander python api to navigate using the global odom ekf 
#for PoseWithCovarianceStamped instead of amcl
def main():
    rclpy.init()
    gps_points = get_gps_points()
    map_points = convert_from_LL(gps_points)
    print(map_points)

    global gps_pose
    gps_pose = None

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

        path_subscriber = PathSubscriber()
        gps_publisher = GPSPublisher()

        node = Node("my_node2")
        client = node.create_client(ToLL, "/toLL")
        request = ToLL.Request()

        while not nav.isNavComplete():
            i += 1
            feedback = nav.getFeedback()
            if feedback and i % 5 == 0:
                print('Distance remaining: ' + '{:.2f}'.format(feedback.distance_remaining) + ' meters.')

            rclpy.spin_once(path_subscriber)

            request.map_point.x = gps_pose.pose.position.x
            request.map_point.y = gps_pose.pose.position.y
            request.map_point.z = 0.0
            print("Map position x: ", gps_pose.pose.position.x)
            print("Map position y: ",gps_pose.pose.position.y)

            future = client.call_async(request)
            rclpy.spin_until_future_complete(node, future)

            if future.result() is not None:
                response = future.result()
                print("Lat from Toll ", response.ll_point.latitude)
                print("Long from Toll ",response.ll_point.longitude)
            else:
                print("Service call failed")
                continue
            rclpy.spin_once(gps_publisher)


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
    rclpy.shutdown()
    exit(0)

if __name__ == "__main__":
    main()


