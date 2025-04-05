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
  nh.setParam("/panda_arm_controller/constraints/goal_time", 60.0);

  // Advertise solutions for coursework tasks
  t1_service_  = nh_.advertiseService("/task1_start", 
    &cw2::t1_callback, this);
  t2_service_  = nh_.advertiseService("/task2_start", 
    &cw2::t2_callback, this);
  t3_service_  = nh_.advertiseService("/task3_start",
    &cw2::t3_callback, this);
  
  // Set planning parameters
  arm_group_.setPlanningTime(40.0);
  arm_group_.setNumPlanningAttempts(2000);
  arm_group_.allowReplanning(true);

  // // Motion velocity
  // arm_group_.setMaxVelocityScalingFactor(0.5);
  // hand_group_.setMaxVelocityScalingFactor(0.5); 

  // Default goal tolerances
  arm_group_.setGoalJointTolerance(def_joint_tol);        // 0.01 rad (~0.6 deg) per joint      
  arm_group_.setGoalPositionTolerance(def_pos_tol);       // 10 mm 
  arm_group_.setGoalOrientationTolerance(def_orient_tol); // 0.01 rad

  // Home pose
  home_pose_ = arm_group_.getCurrentPose(); 

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

  // ROS_INFO("Object position: x=%.3f, y=%.3f, z=%.3f", 
  //   object_point.point.x, object_point.point.y, object_point.point.z);
  // ROS_INFO("Task 1: Shape type = %s", shape_type.c_str());
  // ROS_INFO("Basket position: x=%.3f, y=%.3f, z=%.3f", 
  //   goal_point.point.x, goal_point.point.y, goal_point.point.z);
  
  // 1. Move the arm to a pre-grasp offset
  ROS_DEBUG("Moving to pre-grasp offset...");
  if (!moveToPreGraspOffset(object_point, shape_type)) {
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

  // 3. Lower the arm to grasp position
  ROS_DEBUG("Lowering arm to grasp the object...");
  if (!lowerToObject(object_point)) {  
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

  // note: if the gripper fails to hold the shape, the arm will still execute steps 5–7,
  // potentially placing nothing in the basket.

  // 5. Move the arm to lift the object
  ROS_DEBUG("Lifting the object...");
  if (!liftObject(goal_point)) { 
  ROS_ERROR("Failed to lift the object.");
  return true;
  }
  ROS_DEBUG("Object lifted successfully.");

  // 6. Move the arm to pre-place offset
  ROS_DEBUG("Moving to pre-place offset...");
  if (!moveToBasketOffset(goal_point)) {
  ROS_ERROR("Failed to move to pre-place offset.");
  return true;
  }
  ROS_DEBUG("Reached pre-place offset.");

  // 7. Lower the arm to safe release height (Optional)
  ROS_DEBUG("Lowering arm to safe release height...");
  if (!lowerToBasket(goal_point)) {  
  ROS_ERROR("Failed to lower the arm.");
  return true;
  }
  ROS_DEBUG("Reached safe release height.");

  // 8. Release object into the basket.
  ROS_DEBUG("Releasing object into the basket...");
  if (!releaseObject()) {
  ROS_ERROR("Failed to release object.");
  return true;
  }
  ROS_DEBUG("Object released successfully.");

  // 9. Return to home position ("ready" position defined at /panda_moveit_config/config/panda_arm.xacro)
  arm_group_.stop();
  arm_group_.clearPoseTargets();
  arm_group_.setPlanningTime(20.0); 
  arm_group_.setNamedTarget("ready");
  if (arm_group_.move() == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
    ROS_INFO("Returned to 'ready' position successfully.");
  } else {
    ROS_WARN("Failed to move to 'ready' position.");
  }

  ROS_INFO("Task 1 completed successfully.");

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
                          const std::string &shape_type)
{
  // Get the current arm pose
  geometry_msgs::PoseStamped current_pose = arm_group_.getCurrentPose();

  // Calculate pregrasp position
  geometry_msgs::PoseStamped pre_grasp_pose;
  pre_grasp_pose.header = object_point.header; 
  pre_grasp_pose.pose.position.x = object_point.point.x;
  pre_grasp_pose.pose.position.y = object_point.point.y;
  pre_grasp_pose.pose.position.z = current_pose.pose.position.z;  // high enough when "ready"

  // Add x-y offset according to shape (grasping strategy)
  if (shape_type == "nought") {
    pre_grasp_pose.pose.position.y += 0.08;
  }
  else if (shape_type == "cross") {
    pre_grasp_pose.pose.position.x += 0.06;
  }

  // Calculate pregrasp orientation
  tf2::Quaternion q;
  q.setRPY(0.0, M_PI, M_PI_4 + M_PI_2);  // roll, pitch, yaw (top-down view)
  pre_grasp_pose.pose.orientation = tf2::toMsg(q);

  // Set specific tolerances
  arm_group_.setGoalJointTolerance(0.05);       // 0.05 rad (~2.8 deg) per joint
  arm_group_.setGoalPositionTolerance(0.03);    // 30 mm
  arm_group_.setGoalOrientationTolerance(0.07); // 0.07 rad (~4 deg)

  // Create a waypoint vector
  std::vector<geometry_msgs::Pose> waypoints;
  waypoints.push_back(pre_grasp_pose.pose);

  // Execute Cartesian path
  if (!planAndExecuteCartesian(waypoints, "moveToPreGraspOffset")) {
    ROS_ERROR("moveToPreGraspOffset: Motion failed after multiple attempts.");
    
    // Clear constraints
    arm_group_.clearPathConstraints();

    // // Reset default tolerances (redundant, already reset in planAndExecuteCartesian helper)
    // arm_group_.setGoalPositionTolerance(def_joint_tol);
    // arm_group_.setGoalPositionTolerance(def_pos_tol);

    return false;
  }
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
cw2::lowerToObject(const geometry_msgs::PointStamped &object_point)
{
  // Get the current arm pose
  geometry_msgs::PoseStamped current_pose = arm_group_.getCurrentPose();
  
  // Lower the arm (add margin for table height)
  current_pose.pose.position.z = object_point.point.z + fingertip_offset + 0.08;

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
cw2::liftObject(const geometry_msgs::PointStamped &goal_point)
{
  // Get the current arm pose
  geometry_msgs::PoseStamped current_pose = arm_group_.getCurrentPose();

  // Lift the arm high enough to avoid collisions
  current_pose.pose.position.z = goal_point.point.z + 0.33 + fingertip_offset + 0.20;

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

// // ----------------------------------------------------------------------------
bool
cw2::moveToBasketOffset(const geometry_msgs::PointStamped &goal_point)
{

  // Get the current pose
  geometry_msgs::PoseStamped current_pose = arm_group_.getCurrentPose();
  
  // Calculate pre-place pose
  geometry_msgs::PoseStamped pre_place_pose;
  pre_place_pose.header = goal_point.header;  
  pre_place_pose.pose.position.x = goal_point.point.x;
  pre_place_pose.pose.position.y = goal_point.point.y;
  pre_place_pose.pose.position.z = current_pose.pose.position.z; 
  pre_place_pose.pose.orientation = current_pose.pose.orientation;

  // Set specific tolerances
  arm_group_.setGoalJointTolerance(0.05);        // 0.05 rad (~3 deg) per joint
  arm_group_.setGoalPositionTolerance(0.07);     // 60 mm 
  arm_group_.setGoalOrientationTolerance(0.25);  // 0.1 rad (~14 deg)

  // Create a waypoint vector for CARTESIAN path planning
  std::vector<geometry_msgs::Pose> waypoints;
  waypoints.push_back(pre_place_pose.pose);

  // Execute Cartesian paths
  if (!planAndExecuteCartesian(waypoints, "moveToPreGraspOffset")) {
    ROS_ERROR("moveToPreGraspOffset: Motion failed after multiple attempts.");
    
    // Clear constraints
    arm_group_.clearPathConstraints();

    // // Reset default tolerances 
    // arm_group_.setGoalPositionTolerance(def_joint_tol);
    // arm_group_.setGoalPositionTolerance(def_pos_tol);

    return false;
  }
  return true;
}

// ----------------------------------------------------------------------------
bool
cw2::lowerToBasket(const geometry_msgs::PointStamped &goal_point)
{
  // Get the current arm pose
  geometry_msgs::PoseStamped current_pose = arm_group_.getCurrentPose();
  
  // Lower the arm to safe release position
  current_pose.pose.position.z = goal_point.point.z + 0.05 + fingertip_offset + 0.25;

  // Create a waypoint vector for Cartesian path planning
  std::vector<geometry_msgs::Pose> waypoints;
  waypoints.push_back(current_pose.pose);
  
  // Execute the Cartesian path
  if (!planAndExecuteCartesian(waypoints, "lowerToBasket")) {
    ROS_ERROR("lowerToBasket: Motion failed after multiple attempts.");
    return false;
  }
  
  ROS_INFO("lowertoBasket: Object can be safely released.");
  return true;
}

// ----------------------------------------------------------------------------
bool
cw2::releaseObject()
{
  ROS_INFO("releaseObject: Releasing object...");
  double width = gripper_open_; 
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
// Cartesian path planning

/* 
Approach:
- multiple planning attempts with increasing step size
- multiple execution attempts with pose perturbation
- adaptive tolerance relaxation for difficult moves
- enhanced error handling and debugging output

Important parameters:
- eef_step (EE step size); lower for smoother path, higher for faster planning but jerky motion & collision
- path_fraction_threshold (minimum completion)
*/

bool
cw2::planAndExecuteCartesian(const std::vector<geometry_msgs::Pose> &waypoints,
                          const std::string &action_name)
{
  moveit_msgs::RobotTrajectory trajectory;
  int max_planning_attempts = 10;
  double eef_step = 0.002;       // initial EE step size
  double jump_threshold = 0.0; 

  // Outer loop: Try planning with increasingly larger step sizes
  for (int attempt = 1; attempt <= max_planning_attempts; attempt++) {
    
    // Calculate path
    ROS_INFO_STREAM(action_name << ": Planning attempt " << attempt << " with eef_step = " << eef_step);
    double fraction = arm_group_.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    
    // Check if it completed at least 70%
    if (fraction >= 0.9) {
      ROS_INFO_STREAM(action_name << ": Cartesian path computed successfully with fraction " << fraction);
      int max_exec_attempts = 10;
      
      // Inner loop: Try executing the planned path multiple times
      for (int exec = 1; exec <= max_exec_attempts; exec++) {
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        plan.trajectory_ = trajectory;

        if (arm_group_.execute(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
          ROS_INFO_STREAM(action_name << ": Execution succeeded on attempt " << exec);
          return true;
        } else {
          ROS_WARN_STREAM(action_name << ": Execution attempt " << exec << " failed.");

          // If execution fails, try perturbing the final position (to overcome local minima)
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

      // If execution still fails, increase tolerances
      ROS_WARN_STREAM(action_name << ": Execution failed after retries. Increasing tolerances...");
      double current_pos_tol = arm_group_.getGoalPositionTolerance();       
      double current_orient_tol = arm_group_.getGoalOrientationTolerance();

      ROS_INFO("Current tolerances: Pos=%.3fm, Orient=%.3frad", 
        arm_group_.getGoalPositionTolerance(),
        arm_group_.getGoalOrientationTolerance());
      
      arm_group_.setGoalJointTolerance(current_pos_tol * 1.3);           // increase by 30%
      arm_group_.setGoalOrientationTolerance(current_orient_tol * 1.3);  // increase by 30%
      ROS_INFO_STREAM(action_name << ": Lowered tolerances to pos_tol = " 
                      << arm_group_.getGoalPositionTolerance()
                      << ", orient_tol = " 
                      << arm_group_.getGoalOrientationTolerance());
      
      // Try execution with increased tolerances
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      plan.trajectory_ = trajectory;
      if (arm_group_.execute(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
        ROS_INFO_STREAM(action_name << ": Execution succeeded with lower tolerances.");
        // Reset the default tolerances
        arm_group_.setGoalJointTolerance(def_pos_tol);
        arm_group_.setGoalOrientationTolerance(def_pos_tol);
        return true;
      } else {
        ROS_ERROR_STREAM(action_name << ": Execution failed even with lower tolerances.");
        arm_group_.setGoalJointTolerance(def_pos_tol);
        arm_group_.setGoalOrientationTolerance(def_pos_tol);
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