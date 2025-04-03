/* feel free to change any part of this file, or delete this file. In general,
you can do whatever you want with this template code, including deleting it all
and starting from scratch. The only requirment is to make sure your entire 
solution is contained within the cw2_team_<your_team_number> package */

#include <cw2_class.h> 

///////////////////////////////////////////////////////////////////////////////

cw2::cw2(ros::NodeHandle nh)
{
  /* Class constructor */

  nh_ = nh;
  base_frame_ = "panda_link0";

  // Advertise solutions for coursework tasks
  t1_service_  = nh_.advertiseService("/task1_start", 
    &cw2::t1_callback, this);
  t2_service_  = nh_.advertiseService("/task2_start", 
    &cw2::t2_callback, this);
  t3_service_  = nh_.advertiseService("/task3_start",
    &cw2::t3_callback, this);
  
  arm_group_.setMaxVelocityScalingFactor(0.25);
  hand_group_.setMaxVelocityScalingFactor(0.25); 
  
  // Set planning parameters
  arm_group_.setPlanningTime(15.0);
  arm_group_.setNumPlanningAttempts(1000);
  arm_group_.allowReplanning(true);

  // Set tolerances
  arm_group_.setGoalJointTolerance(0.01);  // m      
  arm_group_.setGoalPositionTolerance(0.01);  // m 
  arm_group_.setGoalOrientationTolerance(0.1);  // rad 

  // // Subscribe to point cloud
  // pointcloud_sub_ = nh_.subscribe("/r200/camera/depth_registered/points", 1, &cw2::pointCloudCallback, this);
  // cloud_ptr_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGBA>>();

  // Get default pose
  default_pose_ = arm_group_.getCurrentPose();

  ROS_INFO("cw2 class initialised");
}

///////////////////////////////////////////////////////////////////////////////

bool
cw2::t1_callback(cw2_world_spawner::Task1Service::Request &request,
  cw2_world_spawner::Task1Service::Response &response) 
{
  /* function which should solve task 1 */

  ROS_INFO("The coursework solving callback for task 1 has been triggered");

  // 0. Extract inputs
  geometry_msgs::PointStamped object_point = request.object_point;
  std::string shape_type = request.shape_type;
  geometry_msgs::PointStamped goal_point = request.goal_point;

  ROS_INFO("Task 1: Shape type = %s", shape_type.c_str());

  ROS_INFO("Basket position: x=%.3f, y=%.3f, z=%.3f", 
    goal_point.point.x, goal_point.point.y, goal_point.point.z);

  // 1. Move the arm to a pre-grasp offset (25 cm above the object)
  ROS_DEBUG("Moving to pre-grasp offset...");
  if (!moveToPreGraspOffset(object_point, shape_type, 0.25)) {
  ROS_ERROR("Failed to move to pre-grasp offset.");
  return true;
  }
  ROS_DEBUG("Reached pre-grasp offset.");

  // 2. Open the gripper
  ROS_DEBUG("Opening gripper...");
  if (!openGripper()) {
  ROS_ERROR("Failed to open gripper.");
  return true;
  }
  ROS_DEBUG("Gripper opened successfully.");

  // 3. Lower the arm to grasp position (lower by 15.5 cm).
  ROS_DEBUG("Lowering arm to grasp the object...");
  if (!lowerToObject(0.13)) {
  ROS_ERROR("Failed to lower to object.");
  return true;
  }
  ROS_DEBUG("Reached grasp position.");

  // 4. Close the gripper to grasp the object.
  ROS_DEBUG("Closing gripper...");
  if (!closeGripper()) {
  ROS_ERROR("Failed to grasp the object.");
  return true;
  }
  ROS_DEBUG("Object grasped successfully.");

  // note: if the gripper fails to hold the shape, the arm will still execute steps 5–7 as successful,
  // potentially placing nothing in the basket.

  // 5. Move the arm to safely lift the object (raise by 15.5 cm).
  ROS_DEBUG("Lifting the object...");
  if (!liftObject(0.13)) {
  ROS_ERROR("Failed to lift the object.");
  return true;
  }
  ROS_DEBUG("Object lifted successfully.");

  // 6. Move the arm to pre-place offset (25 cm above the basket).
  ROS_DEBUG("Moving to pre-place offset...");
  if (!moveToBasketOffset(goal_point, 0.25)) {
  ROS_ERROR("Failed to move to pre-place offset.");
  return true;
  }
  ROS_DEBUG("Reached pre-place offset.");

  // 7. Release object into the basket.
  ROS_DEBUG("Releasing object into the basket...");
  if (!releaseObject()) {
  ROS_ERROR("Failed to release object.");
  return true;
  }
  ROS_DEBUG("Object released successfully.");

  ROS_INFO("Task 1: Pick and Place completed successfully");

  return true;
}

///////////////////////////////////////////////////////////////////////////////

bool
cw2::t2_callback(cw2_world_spawner::Task2Service::Request &request,
  cw2_world_spawner::Task2Service::Response &response)
{
  /* function which should solve task 2 */

  ROS_INFO("The coursework solving callback for task 2 has been triggered");

  return true;
}

///////////////////////////////////////////////////////////////////////////////

bool
cw2::t3_callback(cw2_world_spawner::Task3Service::Request &request,
  cw2_world_spawner::Task3Service::Response &response)
{
  /* function which should solve task 3 */

  ROS_INFO("The coursework solving callback for task 3 has been triggered");

  return true;
}

///////////////////////////////////////////////////////////////////////////////
// Pick & place helper functions

bool
cw2::moveToPreGraspOffset(const geometry_msgs::PointStamped &object_point,
                        const std::string &shape_type,
                        double offset_z)
{
  // Note: think about a solution for T1_ANY_ORIENTATION = true
  // (align the object's & gripper's yaw angle)

  // Calculate pregrasp pose offset according to shape
  geometry_msgs::PoseStamped pre_grasp_pose;
  pre_grasp_pose.header = object_point.header;  // preserve frame_id
  pre_grasp_pose.pose.position = object_point.point; // copy centroid position (x, y, z)
  pre_grasp_pose.pose.position.z += offset_z;  // add vertical offset

  if (shape_type == "nought") {
    pre_grasp_pose.pose.position.y += 0.08;
  }
  else if (shape_type == "cross") {
    pre_grasp_pose.pose.position.x += 0.06;
  }

  // Use fixed, top-down view
  tf2::Quaternion q;
  double roll = 0.0, pitch = M_PI, yaw = M_PI_4 + M_PI_2;  // roll, pitch, yaw
  q.setRPY(roll, pitch, yaw);
  q.normalize();
  pre_grasp_pose.pose.orientation = tf2::toMsg(q);

  // Execute cartesian path using the helper function (defined below)
  std::vector<geometry_msgs::Pose> waypoints;
  waypoints.push_back(pre_grasp_pose.pose);
  if (!planAndExecuteCartesian(waypoints, "moveToPreGraspOffset")) {
    ROS_ERROR("moveToPreGraspOffset: Motion failed after multiple attempts.");
    return false;
  }
  ROS_INFO("moveToPreGraspOffset: Reached pre-grasp offset of %.3f m", offset_z);
  return true;
}

// ----------------------------------------------------------------------------
bool
cw2::openGripper()
{
  ROS_INFO("openGripper: Opening the gripper...");
  double width = gripper_open_;  // predefined
  double eachJoint = width / 2.0;
  
  std::vector<double> gripperJointTargets(2, eachJoint);
  hand_group_.setJointValueTarget(gripperJointTargets);
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  
  if (hand_group_.plan(plan) != moveit::planning_interface::MoveItErrorCode::SUCCESS) {
    ROS_ERROR("openGripper: Opening failed.");
    return false;
  }
  hand_group_.move();
  ros::Duration(0.5).sleep();
  ROS_INFO("openGripper: Gripper opened.");
  return true;
}

// ----------------------------------------------------------------------------
bool
cw2::lowerToObject(double delta_z)
{
  // Get the current arm pose
  geometry_msgs::PoseStamped current_pose = arm_group_.getCurrentPose();
  
  // Lower the arm by delta_z
  current_pose.pose.position.z -= delta_z;

  // Create a waypoint vector for Cartesian path planning
  std::vector<geometry_msgs::Pose> waypoints;
  waypoints.push_back(current_pose.pose);
  
  // Execute the Cartesian path
  if (!planAndExecuteCartesian(waypoints, "lowerToObject")) {
    ROS_ERROR("lowerToObject: Motion failed after multiple attempts.");
    return false;
  }
  
  ROS_INFO("lowerToObject: Reached grasp position.");
  return true;
}

// ----------------------------------------------------------------------------
bool
cw2::closeGripper()
{
  ROS_INFO("closeGripper: Closing the gripper...");
  double width = gripper_closed_;  // predefined
  double eachJoint = width / 2.0;

  std::vector<double> gripperJointTargets(2, eachJoint);
  hand_group_.setJointValueTarget(gripperJointTargets);
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  
  if (hand_group_.plan(plan) != moveit::planning_interface::MoveItErrorCode::SUCCESS) {
    ROS_ERROR("closeGripper: Closing failed.");
    return false;
  }
  hand_group_.move();
  ros::Duration(0.5).sleep();
  ROS_INFO("closeGripper: Gripper closed.");
  return true;
}

// ----------------------------------------------------------------------------
bool
cw2::liftObject(double delta_z)
{
  // Get the current arm pose
  geometry_msgs::PoseStamped current_pose = arm_group_.getCurrentPose();
  
  // Lift the arm by delta_z
  current_pose.pose.position.z += delta_z;

  // Create a waypoint vector for Cartesian path planning
  std::vector<geometry_msgs::Pose> waypoints;
  waypoints.push_back(current_pose.pose);

  // Execute the Cartesian path
  if (!planAndExecuteCartesian(waypoints, "raiseFromObject")) {
    ROS_ERROR("raiseFromObject: Motion failed after multiple attempts.");
    return false;
  }
  
  ROS_INFO("raiseFromObject: Object lifted successfully.");
  return true;
}

// ----------------------------------------------------------------------------
bool
cw2::moveToBasketOffset(const geometry_msgs::PointStamped &goal_point,
                      double offset_z)
{
  // Calculate pre-place pose
  geometry_msgs::PoseStamped pre_place_pose;
  pre_place_pose.header = goal_point.header;  
  pre_place_pose.pose.position = goal_point.point;
  pre_place_pose.pose.position.z = goal_point.point.z + offset_z;
  pre_place_pose.pose.orientation = arm_group_.getCurrentPose().pose.orientation;

  // Create a waypoint vector for Cartesian path planning
  std::vector<geometry_msgs::Pose> waypoints;
  waypoints.push_back(pre_place_pose.pose);
  
  // Execute the Cartesian path
  if (!planAndExecuteCartesian(waypoints, "moveToBasketOffset")) {
    ROS_ERROR("moveToBasketOffset: Motion failed after multiple attempts.");
    return false;
  }
  
  ROS_INFO("moveToBasketOffset: Reached pre-place offset of %.3f m", offset_z);
  return true;
}

// ----------------------------------------------------------------------------
bool
cw2::releaseObject()
{
  ROS_INFO("releaseObject: Releasing object...");
  double width = gripper_open_;  // predefined
  double eachJoint = width / 2.0;

  std::vector<double> gripperJointTargets(2, eachJoint);
  hand_group_.setJointValueTarget(gripperJointTargets);
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  
  if (hand_group_.plan(plan) != moveit::planning_interface::MoveItErrorCode::SUCCESS) {
    ROS_ERROR("releaseObject: Release failed.");
    return false;
  }
  hand_group_.move();
  ros::Duration(0.5).sleep();
  ROS_INFO("releaseObject: Object released.");
  return true;
}


///////////////////////////////////////////////////////////////////////////////
// Cartesian path planning helper functions with:
// - multiple planning attempts with increasing step size
// - multiple execution attempts with pose perturbation
// - adaptive tolerance relaxation for difficult moves
// - enhanced error handling and debugging output

bool
cw2::planAndExecuteCartesian(const std::vector<geometry_msgs::Pose> &waypoints,
                          const std::string &action_name)
{
  moveit_msgs::RobotTrajectory trajectory;
  int max_planning_attempts = 3;
  double eef_step = 0.01;       // Initial step size of 1cm for end effector
  double jump_threshold = 0.0;  // Deprecated, but required by the current overload

  // Outer loop: Try planning with increasingly larger step sizes
  for (int attempt = 1; attempt <= max_planning_attempts; attempt++) {
    
    // Compute path and check if we achieved at least 90% of the requested path
    ROS_INFO_STREAM(action_name << ": Planning attempt " << attempt << " with eef_step = " << eef_step);
    double fraction = arm_group_.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    
    if (fraction >= 0.9) {
      ROS_INFO_STREAM(action_name << ": Cartesian path computed successfully with fraction " << fraction);
      int max_exec_attempts = 3;
      
      // Inner loop: Try executing the planned path multiple times
      for (int exec = 1; exec <= max_exec_attempts; exec++) {
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        plan.trajectory_ = trajectory;

        // Attempt execution of the planned path
        if (arm_group_.execute(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
          ROS_INFO_STREAM(action_name << ": Execution succeeded on attempt " << exec);
          return true;
        } else {
          ROS_WARN_STREAM(action_name << ": Execution attempt " << exec << " failed.");

          // If execution fails, try perturbing the final position slightly
          // This can help overcome local minima in the motion planning
          if (exec < max_exec_attempts && !waypoints.empty()) {
            geometry_msgs::Pose perturbed = waypoints.back();
            perturbed.position.x += 0.005;
            perturbed.position.y += 0.005;
            perturbed.position.z += 0.005;
            ROS_INFO_STREAM(action_name << ": Perturbing final waypoint for retry.");
            
            // Recompute path with perturbed endpoint
            std::vector<geometry_msgs::Pose> perturbed_waypoints = waypoints;
            perturbed_waypoints.back() = perturbed;
            fraction = arm_group_.computeCartesianPath(perturbed_waypoints, eef_step, jump_threshold, trajectory);
            if (fraction < 0.9)
              ROS_WARN_STREAM(action_name << ": Perturbed path computation only achieved fraction " << fraction);
          }
        }
      }

      // If execution still fails, try lowering the tolerance parameters
      ROS_WARN_STREAM(action_name << ": Execution failed after retries. Lowering tolerances...");
      double original_pos_tol = arm_group_.getGoalPositionTolerance();
      double original_orient_tol = arm_group_.getGoalOrientationTolerance();
      
      // Increase tolerances by 50%
      arm_group_.setGoalJointTolerance(original_pos_tol * 1.5);
      arm_group_.setGoalOrientationTolerance(original_orient_tol * 1.5);
      ROS_INFO_STREAM(action_name << ": Lowered tolerances to pos_tol = " 
                      << arm_group_.getGoalPositionTolerance() << ", orient_tol = " 
                      << arm_group_.getGoalOrientationTolerance());
      
      // Try one final execution with lower tolerances
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      plan.trajectory_ = trajectory;
      if (arm_group_.execute(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
        ROS_INFO_STREAM(action_name << ": Execution succeeded with lower tolerances.");
        // Restore the original tolerances
        arm_group_.setGoalJointTolerance(original_pos_tol);
        arm_group_.setGoalOrientationTolerance(original_orient_tol);
        return true;
      } else {
        ROS_ERROR_STREAM(action_name << ": Execution failed even with lower tolerances.");
        arm_group_.setGoalJointTolerance(original_pos_tol);
        arm_group_.setGoalOrientationTolerance(original_orient_tol);
      }
    } else {
      ROS_WARN_STREAM(action_name << ": Cartesian path planning achieved only " << (fraction * 100.0)
                       << "%. Retrying with reduced constraints.");
    }

    // Increase the step size to reduce planning constraints
    eef_step *= 1.5;  
  }

  ROS_ERROR_STREAM(action_name << ": All planning attempts failed.");
  return false;
}