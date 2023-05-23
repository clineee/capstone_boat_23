#include "multiple_object_tracking_lidar.hpp"

namespace multiple_object_tracking_lidar {

static const rclcpp::Logger LOGGER = rclcpp::get_logger("multiple_object_tracking_lidar");

// constructor - delegate with no namespace
MultipleObjectTrackingLidar::MultipleObjectTrackingLidar(const rclcpp::NodeOptions &options) :
	MultipleObjectTrackingLidar("", options) {}

// main constructor
MultipleObjectTrackingLidar::MultipleObjectTrackingLidar(const std::string &name_space,
														 const rclcpp::NodeOptions &options) :
	Node("MultipleObjectTrackingLidar", name_space, options) {
  RCLCPP_INFO(this->get_logger(), "MultipleObjectTrackingLidar init complete!");

  // init parameters
  this->declare_parameter("frame_id", "map");
  this->declare_parameter("filtered_cloud", "filtered_cloud");

  // override defaults if provided in the launch.py file
  this->get_parameter("frame_id", frame_id);
  this->get_parameter("filtered_cloud", filtered_cloud);

  // Store clock
  clock_ = this->get_clock();

  std::cout << "About to setup callback\n";

  // Create a ROS subscriber for the input point cloud
  auto sub_callback = std::bind(&MultipleObjectTrackingLidar::cloud_cb, this, std::placeholders::_1);

  sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(filtered_cloud, 1, sub_callback);

  // Create a ROS publisher for the output point cloud
  pub_cluster_points = this->create_publisher<sensor_msgs::msg::PointCloud2>("cluster_points", 1);
  pub_cluster_centroids = this->create_publisher<sensor_msgs::msg::PointCloud2>("cluster_centroids", 1);

  // Subscribe to the clustered pointclouds
  objID_pub = this->create_publisher<std_msgs::msg::Int32MultiArray>("obj_id", 1);
}


// callback upon receiving new point cloud data
// this is the main processing function
void MultipleObjectTrackingLidar::cloud_cb(const sensor_msgs::msg::PointCloud2::ConstPtr &input) {
  // (1) Get point cloud data and process it
  // separate input point cloud into clusters using KD-tree
  std::cout << "Callback\n";
  auto cluster_vec = MultipleObjectTrackingLidar::process_point_cloud(input);

  // (2) Find cluster
  // find the centroid of the clusters
  std::cout << "Clustered\n";
  auto cluster_centroids = MultipleObjectTrackingLidar::findCentroids(cluster_vec);

  // (2.5) Create new KFs as needed to match total number of clusters
  std::cout << "KFs\n";
  if (cluster_centroids.size() > kalman_filters.size()) {
	for (auto i = kalman_filters.size(); i < cluster_centroids.size(); i++) {
	  kalman_filters.push_back(init_kf(0, 0));
	}
  }

  // (3) Update KF predictions
  std::cout << "update KFs\n";
  auto kf_preds = get_kf_preds();

  // (4) Assign Object IDs
  std::cout << "pre ObjID\n";
  assign_object_id(kf_preds, cluster_centroids);

  // publish ObjectID message
  std_msgs::msg::Int32MultiArray obj_id_msg;
  for (auto id : objID) {
	obj_id_msg.data.push_back(id);
  }
  objID_pub->publish(obj_id_msg);

  // (5) Update KF state
  correct_kfs(cluster_centroids);

  // (6) Publish data
  std::cout << "Publishing\n";
  // merge all cluster point clouds and publish
  pcl::PointCloud<pcl::PointXYZ>::Ptr merged_clouds(new pcl::PointCloud<pcl::PointXYZ>);
  for (const auto &cloud : cluster_vec) {
	(*merged_clouds) += (*cloud);
  }
  publish_cloud(pub_cluster_points, merged_clouds);

  // merge all centroid points and publish
  pcl::PointCloud<pcl::PointXYZ>::Ptr merged_centers(new pcl::PointCloud<pcl::PointXYZ>);
  for (auto centroid : cluster_centroids) {
	(*merged_centers).points.push_back(centroid);
	std::cout << "Centroid: " << centroid.x << " " << centroid.y << " " << centroid.z << "\n";
  }
  publish_cloud(pub_cluster_centroids, merged_centers);
}


// calculate euclidean distance of two points
double MultipleObjectTrackingLidar::euclidean_distance(const pcl::PointXYZ &p1, const pcl::PointXYZ &p2) {
  return sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) + (p1.z - p2.z) * (p1.z - p2.z));
}


// finds the index of the min value in a matrix
std::pair<int, int> MultipleObjectTrackingLidar::findIndexOfMin(std::vector<std::vector<double>> distMat) {
  std::pair<int, int> minIndex;
  double minVal = std::numeric_limits<double>::max();

  for (int i = 0; i < distMat.size(); i++)
	for (int j = 0; j < distMat.at(0).size(); j++) {
	  if (distMat[i][j] < minVal) {
		minVal = distMat[i][j];
		minIndex = std::make_pair(i, j);
	  }
	}
  return minIndex;
}


// Assign object IDs
// Find the dist from each KF center to point cluster
// Then, assign obj ID by taking min of L2 norm (KF center - cluster center)
// Repeat until all pairs assigned.
void MultipleObjectTrackingLidar::assign_object_id(const std::vector<pcl::PointXYZ> &kf_preds, const std::vector<pcl::PointXYZ> &cluster_centroids) {
  // Find the cluster that is more probable to be belonging to a given KF.
  // reset all assigned object IDs, then reassign
  objID.clear();   // Clear the objID vector
  objID.resize(cluster_centroids.size()); // Allocate default elements so that [i] doesn't segfault.
  // Should be done better

  // first, create matrix of distance from center to KF predicted position
  std::vector<std::vector<double>> distMat;
  for (auto centroid : cluster_centroids) {
	std::vector<double> distVec;
  	for (auto kf_pred : kf_preds) {
	  distVec.push_back(euclidean_distance(kf_pred, centroid));
	}
	distMat.push_back(distVec);
  }

  // debug: print distMat
  for(auto& row : distMat){
	for(auto& val : row){
	  std::cout << val << " ";
	}
	std::cout << "\n";
  }

  std::cout << kf_preds.size() << " " << cluster_centroids.size() << "\n";
  // Associate each KF with its most likely cluster
  for (int clusterCount = 0; clusterCount < objID.size(); clusterCount++) {
	// 1. Find the index of the closest KF point prediction to real cluster center
	std::pair<int, int> minIndex(findIndexOfMin(distMat));
	std::cout << "minIndex: " << minIndex.first << " " << minIndex.second << "\n";

	// 2. objID maps cluster index to KF index
	objID[minIndex.first] = minIndex.second;


	// 3. Set distance values of already identified KFs and Clusters to high number so algorithm doesn't chose them later
	distMat[minIndex.first] = std::vector<double>(kf_preds.size(), 10000.0); // Set the row to a high number.
	for (auto& row : distMat) // set the column to a high number
	{
	  row[minIndex.second] = 10000.0;
	}
  }

  std::cout << "pre drop least " << (cluster_centroids.size() == objID.size()) << "\n";
  // if more KFs than clusters, drop least likely KFs
  /*if(kf_preds.size() > cluster_centroids.size()){
	std::vector<cv::KalmanFilter> new_kfs;
	for(int i = 0; i <= cluster_centroids.size(); i++){
	  new_kfs.push_back(kalman_filters.at(objID[i]));
	}
	kalman_filters = new_kfs;
  }*/
}


void MultipleObjectTrackingLidar::correct_kfs(const std::vector<pcl::PointXYZ> &cluster_centroids) {
  int i = 0;
  for(auto centroid : cluster_centroids){
	cv::Mat measMat = (cv::Mat_<float>(2, 1) << centroid.x, centroid.y);
	kalman_filters.at(objID[i]).correct(measMat);
	i++;
  }
}


std::vector<pcl::PointXYZ> MultipleObjectTrackingLidar::get_kf_preds() {
  // First predict, to update the internal statePre variable
  // then store in ROS Point data structure
  std::vector<pcl::PointXYZ> kf_preds;
  for (auto &kf : kalman_filters) {
	auto pred = kf.predict();        // update internal state

	// convert to point
	pcl::PointXYZ pt(pred.at<float>(0), pred.at<float>(1), pred.at<float>(2));
	kf_preds.push_back(pt);
  }
  return kf_preds;
}

// publish cluster info
void MultipleObjectTrackingLidar::publish_cloud(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &pub,
												const pcl::PointCloud<pcl::PointXYZ>::Ptr& cluster) {
  auto clustermsg = std::make_shared<sensor_msgs::msg::PointCloud2>();
  pcl::toROSMsg(*cluster, *clustermsg);
  clustermsg->header.frame_id = frame_id;
  clustermsg->header.stamp = clock_->now();
  pub->publish(*clustermsg);
}


// Initialize a single kalman filter -- to be called as needed as new clusters are found
cv::KalmanFilter MultipleObjectTrackingLidar::init_kf(float x, float y) {
  // KF constants
  float dvx = 0.01f; // 1.0
  float dvy = 0.01f; // 1.0
  float dx = 1.0f;
  float dy = 1.0f;
  float sigmaP = 0.01;
  float sigmaQ = 0.1;

  // state_dim : [x,y,v_x,v_y]    meas_dim : [x, y]   ctrl_dim : []
  cv::KalmanFilter kf(4, 2, 0, CV_32F);

  // initialize KF matrices and initial state
  kf.transitionMatrix = (cv::Mat_<float>(4, 4) << dx, 0, 1, 0, 0, dy, 0, 1, 0, 0, dvx, 0, 0, 0, 0, dvy);
  cv::setIdentity(kf.measurementMatrix);
  cv::setIdentity(kf.processNoiseCov, cv::Scalar::all(sigmaP));
  cv::setIdentity(kf.measurementNoiseCov, cv::Scalar(sigmaQ));

  // Set initial state
  kf.statePre.at<float>(0) = x;
  kf.statePre.at<float>(1) = y;
  kf.statePre.at<float>(2) = 0; // initial v_x
  kf.statePre.at<float>(3) = 0; // initial v_y

  return kf;
}

// Process the point cloud
// Input: point cloud data
// Output: vector of point cloud clusters
std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> MultipleObjectTrackingLidar::process_point_cloud(const sensor_msgs::msg::PointCloud2::ConstPtr &input) {
  // create placeholder objects
  pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr clustered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);

  // fill point cloud with data
  pcl::fromROSMsg(*input, *input_cloud);

  // adjust point location using IMU data
  // create 3d rotation matrix using Eigen
  // TODO: create subscriber for IMU data in launch function. Store latest roll, pitch, yaw in this.roll, ect. as a float
  // TODO: then, uncomment these lines
  /*Eigen::Affine3f IMU_rotation = Eigen::Affine3f::Identity();

  IMU_rotation.translation << 0.0, 0.0, 0.0;					// do not translate the points
  IMU_rotation.rotate(Eigen::AngleAxisf(this.roll, Eigen::Vector3d::UnitX()) * Eigen::AngleAxisf(this.pitch, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisf(this.yaw, Eigen::Vector3d::UnitZ()));

  pcl::transformPointCloud(*input_cloud, *transformed_cloud, IMU_rotation);*/

  // identify clusters using k-D tree
  tree->setInputCloud(input_cloud);
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance(0.08);
  ec.setMinClusterSize(10);
  ec.setMaxClusterSize(600);
  ec.setSearchMethod(tree);
  ec.setInputCloud(input_cloud);
  //ec.setInputCloud(transformed_cloud);			TODO: when IMU works, switch this and above line

  /* Extract the clusters out of pc and save indices in cluster_indices.*/
  ec.extract(cluster_indices);

  // Vector of cluster point clouds
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cluster_vec;

  // create a point cloud (reference) for each cluster
  // first, iterate clusters
  for (const auto &clusterIdx : cluster_indices) {
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);

	// now, iterate all points in cluster
	for (auto pointIdx : clusterIdx.indices) {
	  cloud_cluster->points.push_back(input_cloud->points[pointIdx]);
	}

	cluster_vec.push_back(cloud_cluster);
  }

  return cluster_vec;
}


// find the centroid for each cluster
std::vector<pcl::PointXYZ> MultipleObjectTrackingLidar::findCentroids(const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &cluster_vec) {
  // to store centroids...
  std::vector<pcl::PointXYZ> cluster_centroids;

  // iterate each cluster
  for (const auto &cluster : cluster_vec) {
	float x = 0.0, y = 0.0, z = 0.0;
	int numPts = 0;

	// find centroid by averaging all points in cluster
	for (auto point : cluster->points) {
	  x += point.x;
	  y += point.y;
	  z += point.z;
	  numPts++;
	}

	pcl::PointXYZ centroid;
	centroid.x = x / (float)numPts;
	centroid.y = y / (float)numPts;
	centroid.z = z / (float)numPts;

	cluster_centroids.push_back(centroid);
  }

  return cluster_centroids;
}

}