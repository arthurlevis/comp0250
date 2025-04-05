/* feel free to change any part of this file, or delete this file. In general,
you can do whatever you want with this template code, including deleting it all
and starting from scratch. The only requirment is to make sure your entire 
solution is contained within the cw2_team_<your_team_number> package */

// include guards, prevent .h file being defined multiple times (linker error)
#ifndef cw2_CLASS_H_
#define cw2_CLASS_H_

// system includes
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>


// include services from the spawner package
#include "cw2_world_spawner/Task1Service.h"
#include "cw2_world_spawner/Task2Service.h"
#include "cw2_world_spawner/Task3Service.h"

// TF2 includes
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Scalar.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

// MoveIt includes
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

// // PCL includes
// #include <pcl_conversions/pcl_conversions.h>
// #include <pcl/common/centroid.h>
// #include <pcl/point_cloud.h>
// #include <pcl/point_types.h>
// #include <pcl/filters/voxel_grid.h>
// #include <pcl/filters/passthrough.h>
// #include <pcl/filters/extract_indices.h>
// #include <pcl/features/normal_3d.h>
// #include <pcl/ModelCoefficients.h>
// #include <pcl/sample_consensus/method_types.h>
// #include <pcl/sample_consensus/model_types.h>
// #include <pcl/search/kdtree.h>
// #include <pcl/segmentation/sac_segmentation.h>
// #include <pcl/segmentation/extract_clusters.h>
// #include <pcl/filters/statistical_outlier_removal.h>

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

  /* ----- class member FUNCTIONS ----- */

  // Pick & place
  bool
  // moveToPreGraspOffset(const geometry_msgs::PointStamped &object_point,
  //                     const std::string &shape_type,
  //                     double offset_z);
  moveToPreGraspOffset(const geometry_msgs::PointStamped &object_point,
                      const std::string &shape_type);
  bool
  openGripper();
  bool
  lowerToObject(const geometry_msgs::PointStamped &object_point);
  bool
  closeGripper();
  bool
  liftObject(double delta_z);
  bool
  // moveToBasketOffset(const geometry_msgs::PointStamped &goal_point,
  //                   double offset_z);
  moveToBasketOffset(const geometry_msgs::PointStamped &goal_point);
  bool
  releaseObject();

  // Cartesian path planning
  bool
  planAndExecuteCartesian(const std::vector<geometry_msgs::Pose> &waypoints,
                        const std::string &action_name);
  

  /* ----- class member VARIABLES ----- */

  ros::NodeHandle nh_;
  ros::ServiceServer t1_service_;
  ros::ServiceServer t2_service_;
  ros::ServiceServer t3_service_;

  // MoveIt interfaces
  moveit::planning_interface::MoveGroupInterface arm_group_{"panda_arm"};
  moveit::planning_interface::MoveGroupInterface hand_group_{"hand"};
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;

  // Robot frames
  std::string base_frame_ = "panda_link0";

  // Gripper (dimensions in hand.xacro)
  double gripper_open_ = 0.08;       // 80 mm 
  double gripper_closed_ = 0.01;     // 10 mm
  double fingertip_offset = 0.0584;  // 58.4 mm

  // Home pose (when node starts)
  geometry_msgs::PoseStamped home_pose_;

  // Default goal tolerances
  double def_joint_tol = 0.01;   // 0.01 rad (~0.6 deg) per joint
  double def_pos_tol = 0.01;     // 10 mm
  double def_orient_tol = 0.01;  // 0.01 rad 
};

#endif // end of include guard for cw2_CLASS_H_
