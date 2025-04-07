/* feel free to change any part of this file, or delete this file. In general,
you can do whatever you want with this template code, including deleting it all
and starting from scratch. The only requirment is to make sure your entire 
solution is contained within the cw2_team_<your_team_number> package */

// include guards, prevent .h file being defined multiple times (linker error)
#ifndef cw2_CLASS_H_
#define cw2_CLASS_H_

// system includes
#include <ros/ros.h>
#include <ros/package.h> 
#include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <sstream>
#include <iomanip>
#include <Eigen/Dense>

// include services from the spawner package 
#include "cw2_world_spawner/Task1Service.h"
#include "cw2_world_spawner/Task2Service.h"
#include "cw2_world_spawner/Task3Service.h"

// TF2 includes
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Scalar.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf/transform_listener.h>
#include <tf2_ros/buffer.h>

// MoveIt includes
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

// PCL includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/transforms.h> 
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h> 
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/statistical_outlier_removal.h>

// OpenCV includes
#include <opencv2/opencv.hpp>

// aliases
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointC;
typedef PointC::Ptr PointCPtr;

struct MultiViewObject {
  float center_x, center_y;
  int point_count;
  std::string shape;
  std::string color;
  int vote_count;
  std::vector<int> point_counts_history;
};

struct ObjectInfo {
  double center_x;
  double center_y;
  double area;
  std::string color;
};
class cw2
{
public:

  /* ----- class member FUNCTIONS ----- */

  // constructor
  cw2(ros::NodeHandle nh);

  // service callbacks for tasks 1, 2, & 3
  bool 
  t1_callback(cw2_world_spawner::Task1Service::Request &request,
    cw2_world_spawner::Task1Service::Response &response);
  bool 
  t2_callback(cw2_world_spawner::Task2Service::Request &request,
    cw2_world_spawner::Task2Service::Response &response);
  bool 
  t3_callback(cw2_world_spawner::Task3Service::Request &request,
    cw2_world_spawner::Task3Service::Response &response);
  // For task 2 and 3
  void
  pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg);

  std::string
  computeColor(double avg_r, double avg_g, double avg_b);
  void 
  colorFilter(const PointCPtr &in_cloud_ptr, PointCPtr &out_cloud_ptr);
  void
  pubFilteredPCMsg (ros::Publisher &pc_pub, PointC &pc);
  void 
  computeObjectCenters(const std::vector<PointCPtr> &clusters);

  void 
  segmentObjects2D(PointCPtr &in_cloud_ptr, std::vector<PointCPtr> &clusters);
  // For Task 2 and 3
  bool
  moveToPoseWithFallback(const geometry_msgs::PoseStamped &goal_pose);

  void segmentObjectsWithOpenCV(PointCPtr &in_cloud_ptr, std::vector<PointCPtr> &clusters);

  std::string classifyObjectByPointCount(size_t num_points);
  void updateMergedObjectsFromSingleView(const std::vector<PointCPtr> &clusters);
  void finalizeVotingResults();
  void mergeNearbyMergedObjectsWithPriorityVoting(float dist_threshold = 0.09);
  std::string classifyShapeWithOpenCV(PointCPtr cluster_cloud, float &estimated_x_mm);

private:

  std::vector<ObjectInfo> detected_objects;
  std::vector<MultiViewObject> merged_objects;

  /* ----- class member FUNCTIONS ----- */

  // Pick-and-place
  bool 
  liftArm(const geometry_msgs::PointStamped &object_point);
  bool
  moveAboveObject(const geometry_msgs::PointStamped &object_point,
                  const std::string &shape_type);
  bool
  adjustGripperYaw(double yaw); 
  bool
  adjustGripperXY(const geometry_msgs::PointStamped &object_point,
                  const std::string &shape_type,
                  double yaw);
  bool
  openGripper();
  bool
  lowerToObject(const geometry_msgs::PointStamped &object_point);
  bool
  closeGripper();
  bool
  liftObject(const geometry_msgs::PointStamped &goal_point);
  bool
  moveAboveBasket(const geometry_msgs::PointStamped &goal_point,
                  const std::string &shape_type);
  bool
  lowerToBasket(const geometry_msgs::PointStamped &goal_point);
  bool
  releaseObject();

  // Cartesian path planning
  bool
  planAndExecuteCartesian(const std::vector<geometry_msgs::Pose> &waypoints,
                          const std::string &action_name);

  // // Pointcloud callback
  // void
  // pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg);

  // Cluster helpers
  geometry_msgs::PoseStamped
  pointStampedToPoseStamped(const geometry_msgs::PointStamped &pt);
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr
  getObjectCluster(const geometry_msgs::PointStamped &object_point);

  // Calculate cluster orientation
  cv::Mat
  clusterToBinaryImage(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cluster,
                      double resolution);
  std::vector<std::vector<cv::Point>>
  extractContours(const cv::Mat &binaryImg);
  std::vector<cv::Point>
  selectSegment(const std::vector<std::vector<cv::Point>> &contours);

  /* ----- class member VARIABLES ----- */

  ros::NodeHandle nh_;
  ros::ServiceServer t1_service_;
  ros::ServiceServer t2_service_;
  ros::ServiceServer t3_service_;

  sensor_msgs::PointCloud2ConstPtr latest_pointcloud_; 

  // MoveIt interfaces
  moveit::planning_interface::MoveGroupInterface arm_group_{"panda_arm"};
  moveit::planning_interface::MoveGroupInterface hand_group_{"hand"};
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;

  // Pointcloud
  ros::Subscriber pc_sub_;  // subscriber listening for pointcloud                  
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_ptr_;  // pointer to pointcloud in PCL format

  pcl::PCLPointCloud2 g_pcl_pc;
  PointCPtr g_cloud_ptr;

  ros::Publisher g_pub_cloud;
  PointCPtr g_cloud_filtered;
  sensor_msgs::PointCloud2 g_cloud_filtered_msg;

  // TF2
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // Reference frames
  std::string base_frame_ = "panda_link0";  // robot base
  std::string camera_frame_ = "color";      // camera

  // Gripper (dimensions in hand.xacro)
  double gripper_open_ = 0.08;       // 80 mm 
  double gripper_closed_ = 0.01;     // 10 mm
  double fingertip_offset = 0.0584;  // 58.4 mm

  // Home pose (when node starts)
  geometry_msgs::PoseStamped home_pose_;

  // MoveIt default goal tolerances
  double def_joint_tol = 0.01;   // 0.01 rad (~0.6 deg) per joint
  double def_pos_tol = 0.01;     // 10 mm
  double def_orient_tol = 0.01;  // 0.01 rad 

  // Camera resolution
  double resolution = 0.0028;  // mm per pixel
  double angle = 0.0;
};

#endif // end of include guard for cw2_CLASS_H_
