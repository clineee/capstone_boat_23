#ifndef MULTIPLE_OBJECT_TRACKING_LIDAR_HPP
#define MULTIPLE_OBJECT_TRACKING_LIDAR_HPP

#include "rclcpp/rclcpp.hpp"

#include <geometry_msgs/msg/point.hpp>
#include "visualization_msgs/msg/marker.hpp"

#include "opencv2/video/tracking.hpp"

#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"

#include "sensor_msgs/msg/point_cloud2.hpp"
#include <sensor_msgs/msg/imu.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/filter.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

namespace multiple_object_tracking_lidar
{

class MultipleObjectTrackingLidar : public rclcpp::Node{
public:
    MultipleObjectTrackingLidar(
        const rclcpp::NodeOptions& options=rclcpp::NodeOptions()
    );
    MultipleObjectTrackingLidar(
        const std::string& name_space,
        const rclcpp::NodeOptions& options=rclcpp::NodeOptions()
    );
private:
    //add member variables for pitch, roll, yaw
    double pitch;
    double roll;
    double yaw;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr markerPub;
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr objID_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cluster_points;
  	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cluster_centroids;

    std::vector<geometry_msgs::msg::Point> prevClusterCenters;

    std::vector<int> objID; // Output of the data association using KF
                            // measurement.setTo(Scalar(0));

    std::vector<cv::KalmanFilter> kalman_filters;
    rclcpp::Clock::SharedPtr clock_;
    std::string frame_id;
    std::string filtered_cloud;

    static double euclidean_distance(const pcl::PointXYZ &p1, const pcl::PointXYZ &p2);
    static std::pair<int, int> findIndexOfMin(std::vector<std::vector<double>> distMat);
    void publish_cloud(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &pub, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cluster);
    void cloud_cb(const sensor_msgs::msg::PointCloud2::ConstPtr &input);
    void imu_cb(const sensor_msgs::msg::Imu::ConstPtr &input);
    

  	static cv::KalmanFilter init_kf(float x, float y);
  	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> process_point_cloud(const sensor_msgs::msg::PointCloud2::ConstPtr& input);
  	static std::vector<pcl::PointXYZ> findCentroids(const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &cluster_vec);
  	void assign_object_id(const std::vector<pcl::PointXYZ> &kf_preds, const std::vector<pcl::PointXYZ> &cluster_centers);
  	std::vector<pcl::PointXYZ> get_kf_preds();
  	void correct_kfs(const std::vector<pcl::PointXYZ> &clusterCenters);
};

}
#endif //MULTIPLE_OBJECT_TRACKING_LIDAR_HPP
