#include "multiple_object_tracking_lidar.hpp"

namespace multiple_object_tracking_lidar
{

static const rclcpp::Logger LOGGER = rclcpp::get_logger("multiple_object_tracking_lidar");


// constructor - delegate with no namespace
MultipleObjectTrackingLidar::MultipleObjectTrackingLidar(const rclcpp::NodeOptions& options) :
							 MultipleObjectTrackingLidar("", options) {}


// main constructor
MultipleObjectTrackingLidar::MultipleObjectTrackingLidar(const std::string& name_space,
  														 const rclcpp::NodeOptions& options) :
							 Node("MultipleObjectTrackingLidar", name_space, options)
{
  RCLCPP_INFO(this->get_logger(),"MultipleObjectTrackingLidar init complete!");

  // init parameters
  this->declare_parameter("stateDim", 4);
  this->declare_parameter("measDim", 2);
  this->declare_parameter("ctrlDim", 0);
  this->declare_parameter("frame_id", "map");
  this->declare_parameter("filtered_cloud", "filtered_cloud");

  // override defaults if provided in the launch.py file
  this->get_parameter("stateDim", stateDim);
  this->get_parameter("measDim", measDim);
  this->get_parameter("ctrlDim", ctrlDim);
  this->get_parameter("frame_id", frame_id);
  this->get_parameter("filtered_cloud", filtered_cloud);

  // Store clock
  clock_ = this->get_clock();

  std::cout << "About to setup callback\n";

  // Create a ROS subscriber for the input point cloud
  //std::function<void(const sensor_msgs::msg::PointCloud2::SharedPtr)> sub_callback =
  //	  std::bind(&MultipleObjectTrackingLidar::cloud_cb, this, std::placeholders::_1);
  auto sub_callback = std::bind(&MultipleObjectTrackingLidar::cloud_cb, this, std::placeholders::_1);

  sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(filtered_cloud,1,sub_callback);

  // Create a ROS publisher for the output point cloud
  // TODO: vectorize as needed
  pub_cluster_points= this->create_publisher<sensor_msgs::msg::PointCloud2>("cluster_points", 1);
  pub_cluster_centroids = this->create_publisher<sensor_msgs::msg::PointCloud2>("cluster_centroids", 1);

  // Subscribe to the clustered pointclouds
  // rclcpp::Subscription<sensor_msgs::msg::PointCloud2> c1=this->create_subscription<sensor_msgs::msg::PointCloud2>("ccs", 100, kft);
  objID_pub = this->create_publisher<std_msgs::msg::Int32MultiArray>("obj_id", 1);
  /* Point cloud clustering
   */
  // cc_pos=this->create_publisher<std_msgs::msg::Float32MultiArray>("ccs", 100);//clusterCenter1
  //markerPub = this->create_publisher<visualization_msgs::msg::Marker>("viz", 1);
}


// calculate euclidean distance of two points
double MultipleObjectTrackingLidar::euclidean_distance(geometry_msgs::msg::Point &p1, geometry_msgs::msg::Point &p2) {
  return sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) +
              (p1.z - p2.z) * (p1.z - p2.z));
}


/*
objID: vector containing the IDs of the clusters that should be associated with
each KF_Tracker objID[0] corresponds to KFT0, objID[1] corresponds to KFT1 etc.
*/

// find the index of the min value from the distMat
// TODO: why?
std::pair<int, int> MultipleObjectTrackingLidar::findIndexOfMin(std::vector<std::vector<float>> distMat) {
  std::cout << "findIndexOfMin CALLED\n";	//TODO: remove debug statement

  std::pair<int, int> minIndex;

  // TODO: rename -- what is El?
  float minVal = std::numeric_limits<float>::max();

  for (int i = 0; i < distMat.size(); i++)
    for (int j = 0; j < distMat.at(0).size(); j++) {
      if (distMat[i][j] < minVal) {
        minVal = distMat[i][j];
        minIndex = std::make_pair(i, j);
      }
    }
  return minIndex;
}


// Kalman Filter something ....
// ccs: cluster centers
void MultipleObjectTrackingLidar::kft(const std_msgs::msg::Float32MultiArray ccs) {

  // First predict, to update the internal statePre variable
  std::vector<cv::Mat> pred{KF0.predict(), KF1.predict(), KF2.predict(),
                            KF3.predict(), KF4.predict(), KF5.predict()};

  // Get measurements
  // Extract the position of the clusters from the multiArray. To check if the
  // data coming in, check the .z (every third) coordinate and that will be 0.0
  std::vector<geometry_msgs::msg::Point> clusterCenters;

  // condense cluster centers into vector of points
  for (auto it = ccs.data.begin(); it != ccs.data.end(); it += 3) {
    geometry_msgs::msg::Point pt;
    pt.x = *it;
    pt.y = *(it + 1);
    pt.z = *(it + 2);

    clusterCenters.push_back(pt);
  }

  // iterate over KF predictions
  std::vector<geometry_msgs::msg::Point> KFpredictions;
  for (auto it = pred.begin(); it != pred.end(); it++) {
    geometry_msgs::msg::Point pt;
    pt.x = (*it).at<float>(0);
    pt.y = (*it).at<float>(1);
    pt.z = (*it).at<float>(2);

    KFpredictions.push_back(pt);
  }

  // Find the cluster that is more probable to be belonging to a given KF.
  // reset all assigned object IDs, then reassign
  objID.clear();   // Clear the objID vector
  objID.resize(6); // Allocate default elements so that [i] doesn't segfault.
                   // Should be done better


  // Copy clusterCentres for modifying it and preventing multiple assignments of
  // the same ID
  std::vector<geometry_msgs::msg::Point> copyOfClusterCenters(clusterCenters);
  std::vector<std::vector<float>> distMat;

  for (int filterN = 0; filterN < 6; filterN++) {
    std::vector<float> distVec;
    for (int n = 0; n < 6; n++) {
      distVec.push_back(euclidean_distance(KFpredictions[filterN], copyOfClusterCenters[n]));
    }

    distMat.push_back(distVec);
  }

  // iterate through clusters to TODO: do what?
  for (int clusterCount = 0; clusterCount < 6; clusterCount++) {
    // 1. Find min(distMax)==> (i,j);
    std::pair<int, int> minIndex(findIndexOfMin(distMat));
    std::cout << "Received minIndex=" << minIndex.first << "," << minIndex.second
         << "\n";
    // 2. objID[i]=clusterCenters[j]; counter++
    objID[minIndex.first] = minIndex.second;

    // 3. distMat[i,:]=10000; distMat[:,j]=10000
    distMat[minIndex.first] =
        std::vector<float>(6, 10000.0); // Set the row to a high number.
    for (int row = 0; row < distMat.size();
         row++) // set the column to a high number
    {
      distMat[row][minIndex.second] = 10000.0;
    }
    // 4. if(counter<6) got to 1.
    std::cout << "clusterCount=" << clusterCount << "\n";
  }

  // std::cout<<"Got object IDs"<<"\n";
  // countIDs(objID);// for verif/corner cases

  // display objIDs
  /* DEBUG
    std::cout<<"objID= ";
    for(auto it=objID.begin();it!=objID.end();it++)
        std::cout<<*it<<" ,";
    std::cout<<"\n";
    */

  //TODO: fix MarkerArray
  //visualization_msgs::MarkerArray clusterMarkers;

  for (int i = 0; i < 6; i++) {
    visualization_msgs::msg::Marker m;

    m.id = i;
    m.type = visualization_msgs::msg::Marker::CUBE;
    m.header.frame_id = frame_id;
    m.scale.x = 0.3;
    m.scale.y = 0.3;
    m.scale.z = 0.3;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.color.a = 1.0;
    m.color.r = i % 2 ? 1 : 0;
    m.color.g = i % 3 ? 1 : 0;
    m.color.b = i % 4 ? 1 : 0;

    // geometry_msgs::msg::Point clusterC(clusterCenters.at(objID[i]));
    geometry_msgs::msg::Point clusterC(KFpredictions[i]);
    m.pose.position.x = clusterC.x;
    m.pose.position.y = clusterC.y;
    m.pose.position.z = clusterC.z;

    //TODO: fix MarkerArray
    //clusterMarkers.markers.push_back(m);
  }

  prevClusterCenters = clusterCenters;

  //TODO: fix MarkerArray
  //markerPub.publish(clusterMarkers);

  std_msgs::msg::Int32MultiArray obj_id;
  for (auto it = objID.begin(); it != objID.end(); it++)
    obj_id.data.push_back(*it);
  // Publish the object IDs
  objID_pub->publish(obj_id);
  // convert clusterCenters from geometry_msgs::msg::Point to floats
  std::vector<std::vector<float>> cc;
  for (int i = 0; i < 6; i++) {
    std::vector<float> pt;
    pt.push_back(clusterCenters[objID[i]].x);
    pt.push_back(clusterCenters[objID[i]].y);
    pt.push_back(clusterCenters[objID[i]].z);

    cc.push_back(pt);
  }
  // std::cout<<"cc[5][0]="<<cc[5].at(0)<<"cc[5][1]="<<cc[5].at(1)<<"cc[5][2]="<<cc[5].at(2)<<"\n";
  float meas0[2] = {cc[0].at(0), cc[0].at(1)};
  float meas1[2] = {cc[1].at(0), cc[1].at(1)};
  float meas2[2] = {cc[2].at(0), cc[2].at(1)};
  float meas3[2] = {cc[3].at(0), cc[3].at(1)};
  float meas4[2] = {cc[4].at(0), cc[4].at(1)};
  float meas5[2] = {cc[5].at(0), cc[5].at(1)};

  // The update phase
  cv::Mat meas0Mat = cv::Mat(2, 1, CV_32F, meas0);
  cv::Mat meas1Mat = cv::Mat(2, 1, CV_32F, meas1);
  cv::Mat meas2Mat = cv::Mat(2, 1, CV_32F, meas2);
  cv::Mat meas3Mat = cv::Mat(2, 1, CV_32F, meas3);
  cv::Mat meas4Mat = cv::Mat(2, 1, CV_32F, meas4);
  cv::Mat meas5Mat = cv::Mat(2, 1, CV_32F, meas5);

  // std::cout<<"meas0Mat"<<meas0Mat<<"\n";
  if (!(meas0Mat.at<float>(0, 0) == 0.0f || meas0Mat.at<float>(1, 0) == 0.0f))
    cv::Mat estimated0 = KF0.correct(meas0Mat);
  if (!(meas1[0] == 0.0f || meas1[1] == 0.0f))
    cv::Mat estimated1 = KF1.correct(meas1Mat);
  if (!(meas2[0] == 0.0f || meas2[1] == 0.0f))
    cv::Mat estimated2 = KF2.correct(meas2Mat);
  if (!(meas3[0] == 0.0f || meas3[1] == 0.0f))
    cv::Mat estimated3 = KF3.correct(meas3Mat);
  if (!(meas4[0] == 0.0f || meas4[1] == 0.0f))
    cv::Mat estimated4 = KF4.correct(meas4Mat);
  if (!(meas5[0] == 0.0f || meas5[1] == 0.0f))
    cv::Mat estimated5 = KF5.correct(meas5Mat);

  // Publish the point clouds belonging to each clusters

  // std::cout<<"estimate="<<estimated.at<float>(0)<<","<<estimated.at<float>(1)<<"\n";
  // Point statePt(estimated.at<float>(0),estimated.at<float>(1));
  // std::cout<<"DONE KF_TRACKER\n";
}


// publish cluster info
void MultipleObjectTrackingLidar::publish_cloud(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &pub, pcl::PointCloud<pcl::PointXYZ>::Ptr cluster) {
  auto clustermsg = std::make_shared<sensor_msgs::msg::PointCloud2>();
  pcl::toROSMsg(*cluster, *clustermsg);
  clustermsg->header.frame_id = frame_id;
  clustermsg->header.stamp = clock_->now();
  pub->publish(*clustermsg);
}


// initialize all of the matrices and input states for the KFs
void MultipleObjectTrackingLidar::init_kfs(const std::vector<pcl::PointXYZ>& clusterCentroids) {
  // Initialize 6 Kalman Filters; Assuming 6 max objects in the dataset.
  // Could be made generic by creating a Kalman Filter only when a new object
  // is detected

  float dvx = 0.01f; // 1.0
  float dvy = 0.01f; // 1.0
  float dx = 1.0f;
  float dy = 1.0f;
  float sigmaP = 0.01;
  float sigmaQ = 0.1;

  // initialize KF matrices and initial state
  int i = 0;
  for(auto &kf : kalman_filters){
	// Set matrices
	kf.transitionMatrix = (cv::Mat_<float>(4, 4) << dx, 0, 1, 0, 0, dy, 0, 1, 0, 0, dvx, 0, 0, 0, 0, dvy);
	cv::setIdentity(kf.measurementMatrix);
	cv::setIdentity(kf.processNoiseCov, cv::Scalar::all(sigmaP));
	cv::setIdentity(kf.measurementNoiseCov, cv::Scalar(sigmaQ));

	// Set initial state
	kf.statePre.at<float>(0) = clusterCentroids.at(i).x;
	kf.statePre.at<float>(1) = clusterCentroids.at(i).y;
	kf.statePre.at<float>(2) = 0; // initial v_x
	kf.statePre.at<float>(3) = 0; // initial v_y

	i++;
  }
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
  cv::KalmanFilter kf;

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
std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>  MultipleObjectTrackingLidar::process_point_cloud(const sensor_msgs::msg::PointCloud2::ConstPtr& input){
  // create placeholder objects
  pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr clustered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);

  // fill point cloud with data
  pcl::fromROSMsg(*input, *input_cloud);
  tree->setInputCloud(input_cloud);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance(0.08);
  ec.setMinClusterSize(10);
  ec.setMaxClusterSize(600);
  ec.setSearchMethod(tree);
  ec.setInputCloud(input_cloud);
  /* Extract the clusters out of pc and save indices in cluster_indices.*/
  ec.extract(cluster_indices);

  // Vector of cluster point clouds
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cluster_vec;

  // create a point cloud (reference) for each cluster
  // first, iterate clusters
  for(const auto& clusterIdx : cluster_indices){
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
std::vector<pcl::PointXYZ> MultipleObjectTrackingLidar::findCentroids(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cluster_vec){
  // to store centroids...
  std::vector<pcl::PointXYZ> clusterCentroids;

  // iterate each cluster
  for(const auto& cluster : cluster_vec){
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

	clusterCentroids.push_back(centroid);
  }

  return clusterCentroids;
}

// callback for clustering ==> main processing function
void MultipleObjectTrackingLidar::cloud_cb(const sensor_msgs::msg::PointCloud2::ConstPtr &input)
{
  // separate input point cloud into clusters using KD-tree
  auto cluster_vec = MultipleObjectTrackingLidar::process_point_cloud(input);

  // find the centroid of the clusters
  auto clusterCentroids = MultipleObjectTrackingLidar::findCentroids(cluster_vec);

  // TODO: WIP -- create KFs as new objects are found
  if(clusterCentroids.size() > kalman_filters.size()){
	// init new KFs
	for(int i = kalman_filters.size(); i < clusterCentroids.size(); i++){
	  auto centroid = clusterCentroids.at(i);
	  kalman_filters.push_back(init_kf(centroid.x, centroid.y));
	}
  } else if(clusterCentroids.size() < kalman_filters.size()){
	// TODO: remove KFs corresponding to missing object
  }


  // Ensure at least 6 clusters exist to publish (later clusters may be empty)
  // This creates empty clusters + centroids to publish when fewer than 6 objects are detected
  while (cluster_vec.size() < 6) {
	pcl::PointCloud<pcl::PointXYZ>::Ptr empty_cluster(new pcl::PointCloud<pcl::PointXYZ>);
	empty_cluster->points.push_back(pcl::PointXYZ(0, 0, 0));
	cluster_vec.push_back(empty_cluster);
  }

  while (clusterCentroids.size() < 6) {
	pcl::PointXYZ centroid;
	centroid.x = 0.0;
	centroid.y = 0.0;
	centroid.z = 0.0;

	clusterCentroids.push_back(centroid);
  }

  // If this is the first frame, initialize kalman filters for the clustered objects
  if (firstFrame) {
	MultipleObjectTrackingLidar::init_kfs(clusterCentroids);

	firstFrame = false;

	for (int i = 0; i < 6; i++) {
	  geometry_msgs::msg::Point pt;
	  pt.x = clusterCentroids.at(i).x;
	  pt.y = clusterCentroids.at(i).y;
	  prevClusterCenters.push_back(pt);
	}
  }

  else {
    std_msgs::msg::Float32MultiArray cc;
    for (int i = 0; i < 6; i++) {
      cc.data.push_back(clusterCentroids.at(i).x);
      cc.data.push_back(clusterCentroids.at(i).y);
      cc.data.push_back(clusterCentroids.at(i).z);
    }
    // std::cout<<"6 clusters initialized\n";

    // cc_pos.publish(cc);// Publish cluster mid-points.
    kft(cc);

	// TODO: fix publishers
    int i = 0;
    bool publishedCluster[6];
    for (auto it = objID.begin(); it != objID.end(); it++) { // std::cout<<"Inside the for loop\n";

      switch (i) {
        std::cout << "Inside the switch case\n";
      case 0: {
        publish_cloud(pub_cluster0, cluster_vec[*it]);
        publishedCluster[i] =
            true; // Use this flag to publish only once for a given obj ID
        i++;
        break;
      }
      case 1: {
        publish_cloud(pub_cluster1, cluster_vec[*it]);
        publishedCluster[i] =
            true; // Use this flag to publish only once for a given obj ID
        i++;
        break;
      }
      case 2: {
        publish_cloud(pub_cluster2, cluster_vec[*it]);
        publishedCluster[i] =
            true; // Use this flag to publish only once for a given obj ID
        i++;
        break;
      }
      case 3: {
        publish_cloud(pub_cluster3, cluster_vec[*it]);
        publishedCluster[i] =
            true; // Use this flag to publish only once for a given obj ID
        i++;
        break;
      }
      case 4: {
        publish_cloud(pub_cluster4, cluster_vec[*it]);
        publishedCluster[i] =
            true; // Use this flag to publish only once for a given obj ID
        i++;
        break;
      }

      case 5: {
        publish_cloud(pub_cluster5, cluster_vec[*it]);
        publishedCluster[i] =
            true; // Use this flag to publish only once for a given obj ID
        i++;
        break;
      }
      default:
        break;
      }
    }
  }
}

}
