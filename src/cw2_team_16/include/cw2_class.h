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
#include <cmath> 
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
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

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
typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointC;
typedef PointC::Ptr PointCPtr;

// ADD COMMENT
struct MultiViewObject {
  std::string shape;
  std::string color;
  float center_x;
  float center_y;
  float center_z;
  float angle_deg;
  int point_count;
  PointCPtr pointcloud;
  float unit_size_x;
};

// ADD COMMENT
struct ObjectInfo {
  double center_x;
  double center_y;
  double area;
  std::string color;
};

struct ShapeInfo {
  std::string shape;
  float angle_deg;
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

private:

  std::vector<ObjectInfo> detected_objects;
  std::vector<MultiViewObject> merged_objects;

  /* ----- class member FUNCTIONS ----- */

  // ROBOT MOTION helpers--------------------------------------------------------
  bool
  moveAboveObject(const geometry_msgs::PointStamped &object_point, 
                  double eef_step,
                  std::vector<double> tolerances);
  bool 
  moveArmUpDown(const geometry_msgs::PoseStamped &target_pose,
                const std::string &action_name,
                double eef_step,
                std::vector<double> tolerances);
  bool
  moveHorizontally(const geometry_msgs::PointStamped &point,
                  const std::string &shape_type,
                  double new_yaw,
                  const std::string &action_name,  
                  double eef_step,
                  std::vector<double> tolerances,
                  double size = 0.0);
  bool
  moveToReadyPose(double velocity_scaling,
                  double accel_scaling,
                  std::vector<double> tolerances); 
  bool
  moveToPoseWithFallback(const geometry_msgs::PoseStamped &goal_pose);
  bool
  adjustYaw(const std::string &shape_type,
            double new_yaw,
            double velocity_scaling,
            double accel_scaling,
            std::vector<double> tolerances); 
  bool
  planAndExecuteCartesian(const std::vector<geometry_msgs::Pose> &waypoints,
                          const std::string &action_name,
                          double eef_step);

  // GRIPPER helper--------------------------------------------------------------
  bool
  toggleGripper(double gripper_width,
                const std::string &action_name,
                double velocity_scaling,
                double accel_scaling);

  // POINTCLOUD helpers-----------------------------------------------------------
  void
  pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg);
  void
  pubFilteredPCMsg (ros::Publisher &pc_pub, PointC &pc);
  PointCPtr
  getObjectCluster(const geometry_msgs::PointStamped &object_point);

  // COMPUTER VISION helpers-------------------------------------------------------
  double
  getOrientationFomCluster(const PointCPtr &cluster);
  

  cv::Mat
  clusterToBinaryImg(const PointCPtr &cluster);
  
  std::vector<std::vector<cv::Point>>
  extractContours(const cv::Mat &binaryImg);
  double
  findActualAngle(const std::string &shape_type,
                  double angle);
  std::string
  computeColor(double avg_r,
              double avg_g,
              double avg_b);
  void 
  colorFilter(const PointCPtr &in_cloud_ptr,
              PointCPtr &out_cloud_ptr);
  void 
  classifyClustersFromMergedCloud(PointCPtr full_cloud);
  cv::Mat convertClusterToImage(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& cluster,
                                      float resolution, cv::Scalar& avg_color);
  ShapeInfo classifyShapeFromImage(const cv::Mat& image);
  geometry_msgs::Point get3DCenter(PointCPtr cloud);
  void publishCentroidMarker(const geometry_msgs::Point& pt, int id,
                                 const std::string& label, float angle_deg);

  // // --------------------------------------------------------------------------
  // IF NEEDED
  // geometry_msgs::PoseStamped
  // pointToPose(const geometry_msgs::PointStamped &point);

  // NOT IN CPP
  // void 
  // segmentObjects2D(PointCPtr &in_cloud_ptr, std::vector<PointCPtr> &clusters);
 

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

  // Pointclouds
  ros::Subscriber pc_sub_;        // subscriber listening for pointcloud  
  ros::Publisher pc_pub;          // publisher 
  PointCPtr cloud_ptr_;           // pointer to pointcloud from ROS messages
  PointCPtr filtered_cloud_ptr_;  //
  sensor_msgs::PointCloud2 filtered_cloud_msg;  // ROS pointcloud message

  // TF2
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // Reference frames
  std::string base_frame_ = "panda_link0";  // robot base
  std::string camera_frame_ = "color";      // camera

  // MODIFY FOR T3 ANY SIZE = TRUE
  // Gripper dimensions 
  double gripper_open_ = 0.08;       // 80 mm 
  double gripper_closed_ = 0.015;    // 38 mm (object width = 40mm)
  double fingertip_offset = 0.0584;  // 58.4 mm

  // Home pose (when node starts)
  geometry_msgs::PoseStamped home_pose_;

  // MoveIt scaling
  double def_vel_scal = 0.5;
  double def_accel_scal = 0.5;

  // MoveIt default goal tolerances
  double def_joint_tol = 0.01;   // 0.01 rad 
  double def_pos_tol = 0.01;     // 10 mm
  double def_orient_tol = 0.01;  // 0.01 rad 
  std::vector<double> def_tolerances = {0.01, 0.01, 0.01};  // [joint, position, orientation]

  // Camera resolution
  double camera_resolution = 0.0028;  // mm per pixel
};

#endif // end of include guard for cw2_CLASS_H_
