/* feel free to change any part of this file, or delete this file. In general,
you can do whatever you want with this template code, including deleting it all
and starting from scratch. The only requirment is to make sure your entire 
solution is contained within the cw2_team_<your_team_number> package */

#include <cw2_class.h> 

///////////////////////////////////////////////////////////////////////////////

cw2::cw2(ros::NodeHandle nh):
  tf_buffer_(ros::Duration(15)),
  tf_listener_(tf_buffer_),
  cloud_ptr_(new PointC),
  filtered_cloud_ptr_(new PointC)
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
  
  // MoveIt planning parameters
  arm_group_.setPlanningTime(120);            
  arm_group_.setNumPlanningAttempts(2000); 
  arm_group_.allowReplanning(true);     
  
  // MoveIt execution parameters
  arm_group_.setMaxVelocityScalingFactor(def_vel_scal);        // 0.5
  arm_group_.setMaxAccelerationScalingFactor(def_accel_scal);  // 0.5

  // Default goal tolerances
  arm_group_.setGoalJointTolerance(def_joint_tol);        // 0.01 rad per joint      
  arm_group_.setGoalPositionTolerance(def_pos_tol);       // 10 mm 
  arm_group_.setGoalOrientationTolerance(def_orient_tol); // 0.01 rad

  // Home pose
  home_pose_ = arm_group_.getCurrentPose(); 


  // Initialize pointcloud subscriber & publisher
  pc_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>(
    "/r200/camera/depth_registered/points", 1, &cw2::pointCloudCallback, this);
  pc_pub = nh_.advertise<sensor_msgs::PointCloud2> ("cw2/filtered_cloud", 1, true);

  ROS_INFO("cw2 class initialised");
}
///////////////////////////////////////////////////////////////////////////////

bool
cw2::t1_callback(cw2_world_spawner::Task1Service::Request &request,
  cw2_world_spawner::Task1Service::Response &response) 
{
  /* function which should solve task 1 */

  ROS_INFO("The coursework solving callback for task 1 has been triggered");

  // 1. Extract inputs
  geometry_msgs::PointStamped object_point = request.object_point;
  std::string shape_type = request.shape_type;
  geometry_msgs::PointStamped goal_point = request.goal_point;

  // Check if object & basket are in diagonally oppopsite quadrants
  bool is_diagonal = (object_point.point.x * goal_point.point.x < 0) &&  // opposite x signs
                      (object_point.point.y * goal_point.point.y < 0);   // opposite y signs

  // // Check orientation
  // geometry_msgs::PoseStamped current_pose = arm_group_.getCurrentPose();
  // double roll, pitch, yaw;
  // tf2::getEulerYPR(current_pose.pose.orientation, yaw, pitch, roll);
  // ROS_INFO("Roll: %.3f, Pitch: %.3f, Yaw: %.3f (rad)", roll, pitch, yaw);

  // Add return to home for helpers that fail more often

  // 2. Move the arm above the object
  ROS_DEBUG("Moving above the object...");
  if (!moveAboveObject(object_point, 0.004, {0.005, 0.01, 0.005})) {
    ROS_ERROR("Failed to move above the object.");
    return true;
  }

  // 3. Raise the arm to get larger FoV
  geometry_msgs::PoseStamped fov_pose = arm_group_.getCurrentPose();
  fov_pose.pose.position.z = object_point.point.z + 0.75;
  ROS_DEBUG("Lifting the arm...");
  if (!moveArmUpDown(fov_pose, "raiseArmToLargerFOV", 0.002, {0.005, 0.01, 0.005})) {
    ROS_ERROR("Failed to raise the arm.");
    // return true;
  }

  // Allow some time to process the pointcloud
  ros::AsyncSpinner spinner(2);
  spinner.start();
  ros::Duration(1.5).sleep();
  object_point.header.stamp = ros::Time::now();
  ros::Duration(1.5).sleep();
  spinner.stop();
  ROS_WARN("Break Point 2");

  // Convert cloud_ptr_ to base_frame_ first
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(*cloud_ptr_, cloud_msg);
  cloud_msg.header.frame_id = cloud_ptr_->header.frame_id;

  geometry_msgs::TransformStamped transform_stamped;
  try {
    transform_stamped = tf_buffer_.lookupTransform(base_frame_, cloud_msg.header.frame_id, ros::Time(0), ros::Duration(1.0));
  } catch (tf2::TransformException &ex) {
    ROS_WARN("TF lookup failed: %s", ex.what());
    return true;
  }

  cloud_msg.header.stamp = transform_stamped.header.stamp;

  sensor_msgs::PointCloud2 cloud_transformed_msg;
  try {
    tf_buffer_.transform(cloud_msg, cloud_transformed_msg, base_frame_);
  } catch (tf2::TransformException &ex) {
    ROS_WARN("TF transform failed: %s", ex.what());
    return true;
  }

  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
  pcl::fromROSMsg(cloud_transformed_msg, *transformed_cloud);



  colorFilter(transformed_cloud, filtered_cloud_ptr_);
  merged_objects.clear();
  classifyClustersFromMergedCloud(filtered_cloud_ptr_);

  // PointCPtr obj_cluster_notrans = merged_objects.front().pointcloud;
  // PointCPtr obj_cluster(new pcl::PointCloud<pcl::PointXYZRGBA>);

  // std::string cloud_frame = cloud_ptr_->header.frame_id;
  // sensor_msgs::PointCloud2 raw_cloud_msg;
  // pcl::toROSMsg(*obj_cluster_notrans, raw_cloud_msg);
  // raw_cloud_msg.header.frame_id = cloud_frame;

  // geometry_msgs::TransformStamped transform_stamped;
  // try {
  //   transform_stamped = tf_buffer_.lookupTransform(
  //     base_frame_, cloud_frame, ros::Time(0), ros::Duration(1.0));
  // } catch (tf2::TransformException &ex) {
  //   ROS_WARN("TF lookup failed: %s", ex.what());
  //   return true;
  // }

  // raw_cloud_msg.header.stamp = transform_stamped.header.stamp;

  // sensor_msgs::PointCloud2 transformed_cloud_msg;
  // try {
  //   tf_buffer_.transform(raw_cloud_msg, transformed_cloud_msg, base_frame_);
  // } catch (tf2::TransformException &ex) {
  //   ROS_WARN("TF transform failed: %s", ex.what());
  //   return true;
  // }

  // Visulization: Publish obj pointcloud
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr object_cluster = merged_objects.front().pointcloud;
  sensor_msgs::PointCloud2 obj_msg;
  pcl::toROSMsg(*object_cluster, obj_msg);
  obj_msg.header.frame_id = base_frame_;
  obj_msg.header.stamp = ros::Time::now();
  pc_pub.publish(obj_msg);
  ROS_WARN("Published object pointcloud with %d points", (int)object_cluster->size());

  // 5. Calculate orientation from cluster
  // double rect_angle = getOrientationFomCluster(obj_cluster);  // fitted rectangle rotation [-90,0) deg.
  double rect_angle = merged_objects.front().angle_deg;  
  if (rect_angle == 0.0) {  
    moveToReadyPose(0.5, 0.5, {0.02, 0.02, 0.02});
    return true;  
  }
  double actual_angle = findActualAngle(shape_type, rect_angle);  // actual rotation
  double new_yaw = actual_angle * CV_PI / 180.0;                  // convert to radians
  double size = merged_objects.front().unit_size_x;

  ROS_INFO("Rotated rectangle angle: %.2f deg", rect_angle);
  ROS_INFO("Actual object rotation: %.2f deg", actual_angle); 
  
  // 6. Lower the arm to increase dexterity (recommended when object located in corners)
  geometry_msgs::PoseStamped adjust_pose = arm_group_.getCurrentPose();
  adjust_pose.pose.position.z -= 0.40;
  ROS_DEBUG("Lowering the arm...");
  if (!moveArmUpDown(adjust_pose, "lowerArmToAdjust", 0.005, {0.01, 0.01, 0.01})) {
    ROS_ERROR("Failed to lower the arm");
    return true;
  }

  // 7. Adjust x,y positions (grasping strategy)
  ROS_DEBUG("Adjusting XY...");
  if (!moveHorizontally(object_point, shape_type, new_yaw, "adjustArmXY", 0.001, {0.01, 0.005, 0.01}, size)) {
    ROS_ERROR("Failed to adjust XY.");
    moveToReadyPose(0.5, 0.5, {0.02, 0.02, 0.02});
  return true;
  }
  ROS_DEBUG("XY adjusted successfully.");
  
  // 8. Adjust yaw angle (grasping strategy)
  ROS_DEBUG("Adjusting yaw angle...");
  if (!adjustYaw(shape_type, new_yaw, 0.4, 0.4, {0.03, 0.02, 0.05})) {
    ROS_WARN("Arm rotation not or poorly completed.");
  return true;
  }
  ROS_DEBUG("Yaw adjusted successfully.");

  // 9. Open the gripper
  ROS_DEBUG("Opening the gripper...");
  if (!toggleGripper(gripper_open_, "openGripper", 1.0, 1.0)) {
    ROS_ERROR("Failed to open the gripper.");
    return true;
  }

  // 10. Lower the arm to pick the object
  geometry_msgs::PoseStamped pick_pose = arm_group_.getCurrentPose();
  pick_pose.pose.position.z = object_point.point.z + fingertip_offset + 0.08;
  ROS_DEBUG("Lowering the arm...");
  if (!moveArmUpDown(pick_pose, "lowerArmToPick", 0.005, {0.01, 0.01, 0.01})) {
    ROS_ERROR("Failed to lower the arm.");
    return true;
  }

  // 11. Close the gripper
  ROS_DEBUG("Closing the gripper...");
  if (!toggleGripper(gripper_closed_, "closeGripper", 0.5, 0.5)) {
    ROS_ERROR("Failed to close the gripper.");
    return true;
  }

  // note: if the gripper fails to hold the object, the arm will still execute the next steps,
  // potentially placing nothing in the basket.

  // 12. Lift the object at a safe height
  geometry_msgs::PoseStamped lift_pose = arm_group_.getCurrentPose();
  lift_pose.pose.position.z = goal_point.point.z + 0.33 + fingertip_offset + 0.14;
  ROS_DEBUG("Lifting the object...");
  if (!moveArmUpDown(lift_pose, "liftObject", 0.005, {0.01, 0.01, 0.01})) {
    ROS_ERROR("Failed to lift the object.");
    return true;
  }

  // 13. Move the arm above the basket
  if (is_diagonal) {  // check diagonality
    ROS_INFO("Object & basket in diagonally opposite quadrants. Rotating base joint...");
    std::vector<double> joint_target = arm_group_.getCurrentJointValues();
    joint_target[0] -= M_PI_2; // rotate base joint by 90 deg to simplify planning
    arm_group_.setJointValueTarget(joint_target);
    if (!arm_group_.move()) {
      ROS_WARN("Base rotated with Timed_Out error.");
    }
  }
  ROS_DEBUG("Moving above the basket...");
  if (!moveHorizontally(goal_point, shape_type, new_yaw, "moveAboveBasket", 0.005, {0.02, 0.025, 0.25})) {
    ROS_ERROR("Failed to move above the basket.");
    moveToReadyPose(0.25, 0.25, {0.02, 0.02, 0.02});
    return true;
  }

  // 14. Lower the arm to safe release height
  geometry_msgs::PoseStamped release_pose = arm_group_.getCurrentPose();
  release_pose.pose.position.z = goal_point.point.z + 0.05 + fingertip_offset + 0.25;
  ROS_DEBUG("Lowering the object...");
  if (!moveArmUpDown(release_pose, "releasePose", 0.002, {0.025, 0.03, 0.3})) {
    ROS_WARN("Failed to lower the object at safe height."); 
    // return true;  // often acceptable
  }

  // 15. Release object into the basket.
  ROS_DEBUG("Releasing the gripper...");
  if (!toggleGripper(gripper_open_, "releaseObject", 1.0, 1.0)) {
    ROS_ERROR("Failed to release the object.");
    return true;
  }

  // 16. Return to home
  ros::Duration(0.5).sleep();
  ROS_DEBUG("Returning back to 'ready'...");
  if (!moveToReadyPose(0.3, 0.3, {0.02, 0.02, 0.01})) {
    ROS_WARN("Back to 'ready' (ignore time error if successfully reached.)");
    return true;
  }

  ROS_INFO("Task 1 completed successfully.");

  return true;
}

////////////////////////////////////////////////////////////////////////////////////////////////

bool
cw2::t2_callback(cw2_world_spawner::Task2Service::Request &request,
  cw2_world_spawner::Task2Service::Response &response)
{
  /* function which should solve task 2 */

  ROS_INFO("The coursework solving callback for task 2 has been triggered");
  // Define the reference and mystery object poses
  geometry_msgs::PoseStamped reference_pose;
  geometry_msgs::PoseStamped mystery_pose;
  const auto& ref_points = request.ref_object_points;
  reference_pose.pose.position = ref_points[0].point;
  mystery_pose.pose.position = request.mystery_object_point.point;
  mystery_pose.pose.position.z += 0.6; // Adjust height
  reference_pose.pose.position.z += 0.6; // Adjust height

  // === Step 1: Inspect the mystery object ===

  // Move to the top of mystery object pose
  moveToPoseWithFallback(mystery_pose);

  // Wait for the robot to stabilize
  ros::Duration(2.0).sleep();

  // Filter the point cloud to remove color other than red , blue, and purple

  // Segment the point cloud into objects
  colorFilter(cloud_ptr_, filtered_cloud_ptr_);
  merged_objects.clear();
  classifyClustersFromMergedCloud(filtered_cloud_ptr_);

  // Check if any objects were detected
  if (merged_objects.empty()) {
    ROS_WARN("No object detected at mystery point, retry the Task.");
    return true;
  }

  // Print the detected objects
  std::string mystery_shape = merged_objects.front().shape;
  ROS_INFO("Mystery object shape: %s", mystery_shape.c_str());

  // === Step 2: Inspect the reference object ===

  // Move to the top of the reference object
  moveToPoseWithFallback(reference_pose);

  // Wait for the robot to stabilize
  ros::Duration(2.0).sleep();

  // Filter the point cloud to remove color other than red , blue, and purple
  colorFilter(cloud_ptr_, filtered_cloud_ptr_);

  // Segment the point cloud into objects
  merged_objects.clear();
  classifyClustersFromMergedCloud(filtered_cloud_ptr_);

  // Check if any objects were detected
  if (merged_objects.empty()) {
    ROS_ERROR("No object detected at reference point, retry the Task.");
    return true;
  }

  // Print the detected objects
  std::string ref_shape = merged_objects.front().shape;
  ROS_INFO("Reference object shape: %s", ref_shape.c_str());


  // === Step 3: Map mystery shape to index
  if (mystery_shape == "nought") {
    response.mystery_object_num = (ref_shape == "nought") ? 1 : 2;
  } else if (mystery_shape == "cross") {
    response.mystery_object_num = (ref_shape == "nought") ? 2 : 1;
  } else {
    response.mystery_object_num = 0;
  }

  ROS_INFO("Final mystery_object_num = %ld", response.mystery_object_num);

    // 10. Return to home position ("ready" position defined at /panda_moveit_config/config/panda_arm.xacro)
    arm_group_.setMaxVelocityScalingFactor(1); 
    hand_group_.setMaxVelocityScalingFactor(1); 
    arm_group_.setGoalJointTolerance(0.05);     
    arm_group_.setGoalPositionTolerance(0.03);   
    arm_group_.setGoalOrientationTolerance(0.05);
    arm_group_.stop();
    arm_group_.clearPoseTargets();
    arm_group_.setNamedTarget("ready");
    if (arm_group_.move() == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
      ROS_INFO("Returned to 'ready' position successfully.");
    } else {
      ROS_WARN("Failed to move to 'ready' position.");
    }
    arm_group_.setMaxVelocityScalingFactor(0.5); 
    hand_group_.setMaxVelocityScalingFactor(0.5); 
    arm_group_.setGoalJointTolerance(def_joint_tol);  
    arm_group_.setGoalPositionTolerance(def_pos_tol);            
    arm_group_.setGoalOrientationTolerance(def_orient_tol);
  
    ROS_INFO("Task 2 completed successfully.");

  return true;
}


////////////////////////////////////////////////////////////////////////////////////////////////

bool
cw2::t3_callback(cw2_world_spawner::Task3Service::Request &request,
  cw2_world_spawner::Task3Service::Response &response)
{

  ROS_INFO("The coursework solving callback for task 3 has been triggered");

  // Get current arm pose
  geometry_msgs::PoseStamped current_pose = arm_group_.getCurrentPose();

  // 1. Scan objects & basket
  std::vector<geometry_msgs::PoseStamped> camera_view_poses;
  geometry_msgs::PoseStamped pose1, pose2, pose3, pose4, pose5, pose6, pose7, pose8, pose9, pose10;
  
  // === Viewpoint 1: Bottom Right ===
  pose1.pose.position.x = 0.468 - 0.057;
  pose1.pose.position.y = 0.268;
  pose1.pose.position.z = current_pose.pose.position.z;

  // === Viewpoint 2: Bottom Center ===
  pose2 = pose1;
  pose2.pose.position.y = 0.0;

  // === Viewpoint 3: Bottom Left ===
  pose3 = pose1;
  pose3.pose.position.y = -0.268;

  // === Viewpoint 4: Left Center lower ===
  pose4 = pose3;
  pose4.pose.position.x = 0.1;

  // === Viewpoint 5 : Left Center upper ===
  pose5 = pose4;
  pose5.pose.position.x = -0.1;

  // === Viewpoint 6: Top Left ===
  pose6 = pose3;
  pose6.pose.position.x = -0.468 + 0.05;

  // === Viewpoint 7: Top Center ===
  pose7 = pose6;
  pose7.pose.position.y = 0.0;
  
  // === Viewpoint 8: Top Right ===
  pose8 = pose6;
  pose8.pose.position.y = 0.268;

  // === Viewpoint 9: Right Center upper ===
  pose9 = pose8;
  pose9.pose.position.x = -0.1;

  // === Viewpoint 10: Right Center lower ===
  pose10 = pose8;
  pose10.pose.position.x = 0.1;

  camera_view_poses = {pose1, pose2, pose3, pose4, pose5, pose6, pose7, pose8, pose9, pose10};


  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr merged_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);

  for (const auto& pose : camera_view_poses) {
    moveToPoseWithFallback(pose);
    // Wait for the robot to stabilize
    ros::Duration(2.0).sleep();
    ros::spinOnce();
    ros::Duration(0.2).sleep();
    ros::spinOnce();
    ros::Duration(0.2).sleep();

    colorFilter(cloud_ptr_, filtered_cloud_ptr_);
    // Transform point cloud to base_frame
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*filtered_cloud_ptr_, cloud_msg);
    cloud_msg.header.frame_id = cloud_ptr_->header.frame_id;

    geometry_msgs::TransformStamped transform_stamped;

    try {
      transform_stamped = tf_buffer_.lookupTransform(
        base_frame_,
        cloud_ptr_->header.frame_id,
        ros::Time(0),
        ros::Duration(1.0)
      );
    } catch (tf2::TransformException &ex) {
      ROS_WARN("TF lookup failed: %s", ex.what());
      return false;
    }

    cloud_msg.header.stamp = transform_stamped.header.stamp;

    sensor_msgs::PointCloud2 transformed_cloud_msg;
    try {
      tf_buffer_.transform(cloud_msg, transformed_cloud_msg, base_frame_);
    } catch (tf2::TransformException &ex) {
      ROS_WARN("TF transform failed: %s", ex.what());
      continue;
    }

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::fromROSMsg(transformed_cloud_msg, *transformed_cloud);
    // Add to the merged cloud
    *merged_cloud += *transformed_cloud;

  }
  
  // // Publish the merged cloud
  // sensor_msgs::PointCloud2 merged_msg;
  // pcl::toROSMsg(*merged_cloud, merged_msg);
  // merged_msg.header.frame_id = base_frame_;
  // merged_msg.header.stamp = ros::Time::now();
  // pc_pub.publish(merged_msg);
  // ROS_INFO("Published merged cloud with %d points", (int)merged_cloud->size());

  merged_objects.clear();
  classifyClustersFromMergedCloud(merged_cloud);

  double min_dist = std::numeric_limits<double>::max();
  const MultiViewObject* target = nullptr;
  const MultiViewObject* basket_obj = nullptr;
  // print the summary of detected objects
  for (const auto& obj : merged_objects) {
    ROS_INFO("Detected object: Shape=%s, Color=%s, Position=(%.3f, %.3f, %.3f), Angle=%.3f, Point Count=%d, Size=%.3f",
             obj.shape.c_str(), obj.color.c_str(), obj.center_x, obj.center_y, obj.center_z, obj.angle_deg, obj.point_count, obj.unit_size_x);
    if (obj.shape == "basket") {
      basket_obj = &obj;
    }
  }


  // 2.print and count all detected objects 
  int cross_count = 0;
  int nought_count = 0;
  int black_count = 0;

  for (const auto& obj : merged_objects) {
    if (obj.color == "black") {
      black_count++;
    } else if (obj.shape == "cross") {
      cross_count++;
    } else if (obj.shape == "nought") {
      nought_count++;
    }

  }

  ROS_INFO("Count Summary -> Cross: %d, Nought: %d, Black (obstacles): %d",
           cross_count, nought_count, black_count);


  // 3. Print the counts of each object type and find the most common object
  std::vector<std::string> candidate_shapes;
  int most_common_shape_count = 0;

  if (cross_count > nought_count) {
    candidate_shapes.push_back("cross");
    most_common_shape_count = cross_count;
  } else if (nought_count > cross_count) {
    candidate_shapes.push_back("nought");
    most_common_shape_count = nought_count;
  } else if (nought_count == cross_count && cross_count > 0) {
    candidate_shapes.push_back("cross");
    candidate_shapes.push_back("nought");
    most_common_shape_count = cross_count;
    ROS_INFO("Equal number of cross and nought. Considering both.");
  } else {
    ROS_ERROR("No valid cross or nought objects found.");
    return false;
  }

  // Print total number of shapes
  ROS_INFO("Total number of shapes: %d", cross_count + nought_count);

  // service response
  response.total_num_shapes = cross_count + nought_count;
  response.num_most_common_shape = most_common_shape_count;


   // Step 3: Find closest object among candidate shapes
    for (const auto& obj : merged_objects) {
      if (obj.color == "black") continue;
      if (std::find(candidate_shapes.begin(), candidate_shapes.end(), obj.shape) == candidate_shapes.end()) continue;

      double dx = obj.center_x - current_pose.pose.position.x;
      double dy = obj.center_y - current_pose.pose.position.y;
      double dz = obj.center_z - current_pose.pose.position.z;

      double dist = std::sqrt(dx * dx + dy * dy + dz * dz);

      if (dist < min_dist) {
        min_dist = dist;
        target = &obj;
      }
    }

  if (!target) {
    ROS_ERROR("No valid object matched for pickup.");
    return false;
  }

  if (basket_obj) {
    ROS_INFO("Basket position: (%.3f, %.3f, %.3f)", basket_obj->center_x, basket_obj->center_y, basket_obj->center_z);
  } else {
    ROS_WARN("Basket object not found.");
  }
  ROS_INFO("Target to pick: Shape=%s, Position=(%.3f, %.3f, %.3f), Angle=%.3f, Point Count=%d, Size=%.3f",
           target->shape.c_str(), target->center_x, target->center_y, target->center_z, target->angle_deg, target->point_count, target->unit_size_x);

  //Visulization: Publish obj pointcloud
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr object_cluster = target->pointcloud;
  sensor_msgs::PointCloud2 obj_msg;
  pcl::toROSMsg(*object_cluster, obj_msg);
  obj_msg.header.frame_id = base_frame_;
  obj_msg.header.stamp = ros::Time::now();
  pc_pub.publish(obj_msg);
  ROS_WARN("Published object pointcloud with %d points", (int)object_cluster->size());

  geometry_msgs::PointStamped object_point;
  geometry_msgs::PointStamped basket_point;
  
  object_point.point.x = target->center_x;
  object_point.point.y = target->center_y;
  object_point.point.z = target->center_z;
  std::string shape_type = target->shape;
  object_point.header.frame_id = base_frame_;
  PointCPtr obj_cluster = target->pointcloud;
  double size = target->unit_size_x;

  basket_point.point.x = basket_obj->center_x;
  basket_point.point.y = basket_obj->center_y;
  basket_point.point.z = basket_obj->center_z;
  basket_point.header.frame_id = base_frame_;

  // Move the arm above the object
  // Return to home first to avoid plan collisions
  ros::Duration(0.5).sleep();
  ROS_DEBUG("Returning back to 'ready'...");
  if (!moveToReadyPose(0.3, 0.3, {0.02, 0.02, 0.01})) {
    ROS_WARN("Back to 'ready' (ignore time error if successfully reached.)");
    return true;
  }


  ROS_DEBUG("Moving above the object...");
  if (!moveAboveObject(object_point, 0.004, {0.005, 0.01, 0.005})) {
    ROS_ERROR("Failed to move above the object.");
    return true;
  }

  // Calculate orientation from cluster
  // double rect_angle = getOrientationFomCluster(obj_cluster);  // fitted rectangle rotation [-90,0) deg.
  double rect_angle = target->angle_deg;
  // if (rect_angle == 0.0) {
  //   ROS_ERROR("No rotation detected, moving to ready pose.");  
  //   moveToReadyPose(0.5, 0.5, {0.02, 0.02, 0.02});
  //   return true;  
  // }

  double actual_angle = findActualAngle(shape_type, rect_angle);  // actual rotation
  double new_yaw = actual_angle * CV_PI / 180.0;                  // convert to radians

  ROS_INFO("Rotated rectangle angle: %.2f deg", rect_angle);
  ROS_INFO("Actual object rotation: %.2f deg", actual_angle); 
  
  // Lower the arm to increase dexterity (recommended when object located in corners)
  geometry_msgs::PoseStamped adjust_pose = arm_group_.getCurrentPose();
  adjust_pose.pose.position.z -= 0.40;
  ROS_DEBUG("Lowering the arm...");
  if (!moveArmUpDown(adjust_pose, "lowerArmToAdjust", 0.005, {0.01, 0.01, 0.01})) {
    ROS_ERROR("Failed to lower the arm");
    return true;
  }

  // Adjust x,y positions (grasping strategy)
  ROS_DEBUG("Adjusting XY...");
  if (!moveHorizontally(object_point, shape_type, new_yaw, "adjustArmXY", 0.001, {0.01, 0.005, 0.01}, size)) {
    ROS_ERROR("Failed to adjust XY.");
    moveToReadyPose(0.5, 0.5, {0.02, 0.02, 0.02});
  return true;
  }
  ROS_DEBUG("XY adjusted successfully.");
  
  // Adjust yaw angle (grasping strategy)
  ROS_DEBUG("Adjusting yaw angle...");
  if (!adjustYaw(shape_type, new_yaw, 0.4, 0.4, {0.03, 0.02, 0.05})) {
    ROS_WARN("Arm rotation not or poorly completed.");
  return true;
  }
  ROS_DEBUG("Yaw adjusted successfully.");

  // Open the gripper
  ROS_DEBUG("Opening the gripper...");
  if (!toggleGripper(gripper_open_, "openGripper", 1.0, 1.0)) {
    ROS_ERROR("Failed to open the gripper.");
    return true;
  }

  // Lower the arm to pick the object
  geometry_msgs::PoseStamped pick_pose = arm_group_.getCurrentPose();
  pick_pose.pose.position.z = object_point.point.z + fingertip_offset + 0.03;
  ROS_DEBUG("Lowering the arm...");
  if (!moveArmUpDown(pick_pose, "lowerArmToPick", 0.005, {0.01, 0.01, 0.01})) {
    ROS_ERROR("Failed to lower the arm.");
    return true;
  }

  // Close the gripper
  ROS_DEBUG("Closing the gripper...");
  if (!toggleGripper(gripper_closed_, "closeGripper", 0.5, 0.5)) {
    ROS_ERROR("Failed to close the gripper.");
    return true;
  }

  // Lift the object at a safe height
  geometry_msgs::PoseStamped lift_pose = arm_group_.getCurrentPose();
  lift_pose.pose.position.z = basket_point.point.z + 0.33 + fingertip_offset + 0.14;
  ROS_DEBUG("Lifting the object...");
  if (!moveArmUpDown(lift_pose, "liftObject", 0.005, {0.01, 0.01, 0.01})) {
    ROS_ERROR("Failed to lift the object.");
    return true;
  }

  // Move the arm above the basket
  bool is_diagonal = (object_point.point.x * basket_point.point.x < 0) &&  // opposite x signs
                      (object_point.point.y * basket_point.point.y < 0);   // opposite y signs
  if (is_diagonal) {  // check diagonality
    ROS_INFO("Object & basket in diagonally opposite quadrants. Rotating base joint...");
    std::vector<double> joint_target = arm_group_.getCurrentJointValues();
    joint_target[0] -= M_PI_2; // rotate base joint by 90 deg to simplify planning
    arm_group_.setJointValueTarget(joint_target);
    if (!arm_group_.move()) {
      ROS_WARN("Base rotated with Timed_Out error.");
    }
  }
  ROS_DEBUG("Moving above the basket...");
  if (!moveHorizontally(basket_point, shape_type, new_yaw, "moveAboveBasket", 0.005, {0.02, 0.025, 0.25})) {
    ROS_ERROR("Failed to move above the basket.");
    moveToReadyPose(0.25, 0.25, {0.02, 0.02, 0.02});
    return true;
  }

  // Lower the arm to safe release height
  geometry_msgs::PoseStamped release_pose = arm_group_.getCurrentPose();
  release_pose.pose.position.z = basket_point.point.z + 0.05 + fingertip_offset + 0.25;
  ROS_DEBUG("Lowering the object...");
  if (!moveArmUpDown(release_pose, "releasePose", 0.002, {0.025, 0.03, 0.3})) {
    ROS_WARN("Failed to lower the object at safe height."); 
    // return true;  // often acceptable
  }

  // Release object into the basket.
  ROS_DEBUG("Releasing the gripper...");
  if (!toggleGripper(gripper_open_, "releaseObject", 1.0, 1.0)) {
    ROS_ERROR("Failed to release the object.");
    return true;
  }

  // Return to home
  ros::Duration(0.5).sleep();
  ROS_DEBUG("Returning back to 'ready'...");
  if (!moveToReadyPose(0.3, 0.3, {0.02, 0.02, 0.01})) {
    ROS_WARN("Back to 'ready' (ignore time error if successfully reached.)");
    return true;
  }

  ROS_INFO("Task 3 completed successfully.");

  return true;
}

////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////

// ROBOT MOTION helpers

// -------------------------------------------------------------------------------------------
bool
cw2::moveAboveObject(const geometry_msgs::PointStamped &object_point,
                    double eef_step,
                    std::vector<double> tolerances)
{
  // Get the current arm pose
  geometry_msgs::PoseStamped current_pose = arm_group_.getCurrentPose();
  
  // Calculate above_object_pose position
  geometry_msgs::PoseStamped above_object_pose;
  above_object_pose.header = object_point.header; 

  try {
    // Get the camera pose in the same frame as object_point (base frame)
    geometry_msgs::PoseStamped camera_pose;
    camera_pose.header.frame_id = camera_frame_;
    camera_pose.pose.orientation.w = 1.0;  
    camera_pose = tf_buffer_.transform(camera_pose,
                                      object_point.header.frame_id,
                                      ros::Duration(1.0));

    // Calculate position by offseting the arm to center camera about the object
    above_object_pose.pose.position.x = current_pose.pose.position.x + object_point.point.x - camera_pose.pose.position.x;
    above_object_pose.pose.position.y = current_pose.pose.position.y + object_point.point.y - camera_pose.pose.position.y;
    above_object_pose.pose.position.z = current_pose.pose.position.z;

  } catch (tf2::TransformException &ex) {
    ROS_WARN("Failed to transform object point to camera frame: %s", ex.what());
  }

  // Calculate above_object_pose orientation (top-down view)
  tf2::Quaternion q;
  q.setRPY(0.0, M_PI, 3 * M_PI_4);  // roll, pitch, yaw
  above_object_pose.pose.orientation = tf2::toMsg(q);

  // Set specific tolerances
  arm_group_.setGoalJointTolerance(tolerances[0]);
  arm_group_.setGoalPositionTolerance(tolerances[1]);
  arm_group_.setGoalOrientationTolerance(tolerances[2]);

  // Create a waypoint vector
  std::vector<geometry_msgs::Pose> waypoints;
  waypoints.push_back(above_object_pose.pose);

  // Execute Cartesian path
  if (!planAndExecuteCartesian(waypoints, "moveAboveObject", eef_step)) {
    ROS_ERROR("moveAboveObject: Motion failed after multiple attempts.");
    // No need to reset default parameters & tolerances (already reset in helper)
    if (arm_group_.move() != moveit::planning_interface::MoveItErrorCode::SUCCESS) {
      ROS_ERROR("Fallback moveToPose also failed.");
    return false;
    }
  }
  return true;
}

// -------------------------------------------------------------------------------------------
bool 
cw2::moveArmUpDown(const geometry_msgs::PoseStamped &target_pose,
                  const std::string &action_name,
                  double eef_step,
                  std::vector<double> tolerances)
{
  // Set specific tolerances
  arm_group_.setGoalJointTolerance(tolerances[0]);
  arm_group_.setGoalPositionTolerance(tolerances[1]);
  arm_group_.setGoalOrientationTolerance(tolerances[2]);

  // Create a waypoint vector
  std::vector<geometry_msgs::Pose> waypoints;
  waypoints.push_back(target_pose.pose);

  // Execute the Cartesian path
  if (!planAndExecuteCartesian(waypoints, action_name, eef_step)) {
    ROS_ERROR("%s failed after multiple attempts.", action_name.c_str());
    // No need to reset default parameters & tolerances (already reset in helper)
    return false;
  }
  
  ROS_INFO("%s successful.", action_name.c_str());
  return true;
}

// -------------------------------------------------------------------------------------------
bool
cw2::moveHorizontally(const geometry_msgs::PointStamped &point,
                      const std::string &shape_type,
                      double new_yaw,
                      const std::string &action_name,
                      double eef_step,
                      std::vector<double> tolerances,
                      double size)
{

  // Get current pose
  geometry_msgs::PoseStamped current_pose = arm_group_.getCurrentPose();
  current_pose.header = point.header; 

  // Define offsets
  double x_offset = 0.0;
  double y_offset = 0.0;

  // Thresholds 
  bool is_yaw_small = (std::abs(new_yaw) <= 0.088);  // (0.088 rad ~ 5 deg)
  bool is_yaw_pos = (new_yaw >= 0.0);                // counter-clockwise 

  // Define offsets (pick-and-place strategy)
  if (shape_type == "nought") { 
    // size = 0.4
    if (std::abs(size - 0.4) < 1e-3) {
      if (is_yaw_small) {  // no need to account for rotation when angle < 5 deg. (linear offset)
        y_offset = 0.08;  
      } else {             // apply rotated offsets      
        if (is_yaw_pos) {
          x_offset = - 0.08 * sin(new_yaw);  
          y_offset = 0.08 * cos(new_yaw);    // counter-clockwise -> positive y-direction
        } else {
          x_offset = 0.08 * sin(new_yaw);
          y_offset = - 0.08 * cos(new_yaw);  // clockwise -> negative y-direction

        }
      } 
    }
    // size = 0.3
    else if (std::abs(size - 0.3) < 1e-3) {
      if (is_yaw_small) {
        y_offset = 0.07;
      } else {
        if (is_yaw_pos) {
          x_offset = - 0.07 * sin(new_yaw);
          y_offset = 0.07 * cos(new_yaw);
        } else {
          x_offset = 0.07 * sin(new_yaw);
          y_offset = - 0.07 * cos(new_yaw);
        }
      }
    }
    // size = 0.2
    else if (std::abs(size - 0.2) < 1e-3) {
      if (is_yaw_small) {
        y_offset = 0.06;
      } else {
        if (is_yaw_pos) {
          x_offset = - 0.06 * sin(new_yaw);
          y_offset = 0.06 * cos(new_yaw);
        } else {
          x_offset = 0.06 * sin(new_yaw);
          y_offset = - 0.06 * cos(new_yaw);
        }
      }
    }

  } 
  else if (shape_type == "cross") {  // same strategy but with smaller offsets
    // size = 0.4
    if (std::abs(size - 0.4) < 1e-3) {
      if (is_yaw_small) {
        x_offset = 0.055;
      } else {
        if (is_yaw_pos) {
          x_offset = -0.058 * sin(new_yaw);
          y_offset = 0.058 * cos(new_yaw);
        } else {
          x_offset = 0.058 * sin(new_yaw);
          y_offset = - 0.058 * cos(new_yaw);
        }
      }
    }
    // size = 0.3
    else if (std::abs(size - 0.3) < 1e-3) {
      if (is_yaw_small) {
        x_offset = 0.045;
      } else {
        if (is_yaw_pos) {
          x_offset = -0.048 * sin(new_yaw);
          y_offset = 0.048 * cos(new_yaw);
        } else {
          x_offset = 0.048 * sin(new_yaw);
          y_offset = - 0.048 * cos(new_yaw);
        }
      }
    }
    // size = 0.2
    else if (std::abs(size - 0.2) < 1e-3) {
      if (is_yaw_small) {
        x_offset = 0.035;
      } else {
        if (is_yaw_pos) {
          x_offset = -0.038 * sin(new_yaw);
          y_offset = 0.038 * cos(new_yaw);
        } else {
          x_offset = 0.038 * sin(new_yaw);
          y_offset = - 0.038 * cos(new_yaw);
        }
      }
    }
  }

  // Apply the offsets 
  current_pose.pose.position.x = point.point.x + x_offset;
  current_pose.pose.position.y = point.point.y + y_offset;

  // Set specific tolerances
  arm_group_.setGoalJointTolerance(tolerances[0]);
  arm_group_.setGoalPositionTolerance(tolerances[1]);
  arm_group_.setGoalOrientationTolerance(tolerances[2]);

  // Create a waypoint vector for CARTESIAN path planning
  std::vector<geometry_msgs::Pose> waypoints;
  waypoints.push_back(current_pose.pose);

  // Execute Cartesian paths
  if (!planAndExecuteCartesian(waypoints, action_name, eef_step)) {
    ROS_ERROR("%s: Motion failed after multiple attempts.", action_name.c_str());
    // No need to reset default parameters & tolerances (already reset in helper)
    return false;
  }
  return true;
}
// -------------------------------------------------------------------------------------------
bool
cw2::moveToReadyPose(double velocity_scaling,
                    double accel_scaling,
                    std::vector<double> tolerances)  
{
  // ("ready" position defined at /panda_moveit_config/config/panda_arm.xacro)

  bool success = false;

  try {
    // Stop motion & clear targets
    arm_group_.stop();
    arm_group_.clearPoseTargets();
    arm_group_.setNamedTarget("ready");

    // Set velocity & acceleration scaling
    arm_group_.setMaxVelocityScalingFactor(velocity_scaling); 
    arm_group_.setMaxAccelerationScalingFactor(accel_scaling);

    // Set tolerances
    arm_group_.setGoalJointTolerance(tolerances[0]);
    arm_group_.setGoalPositionTolerance(tolerances[1]);
    arm_group_.setGoalOrientationTolerance(tolerances[2]);

    // Joint-space motion
    if (arm_group_.move() == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
      ROS_INFO("Returned to ready.");
      success = true;
    } else {
      ROS_WARN("Returned to ready with Timed_Out error.");
      success = true; 
    }
  }
  catch(const std::exception& e) {
    ROS_ERROR("Exception in moveToReadyPose: %s", e.what());
    success = false;
  }

  // Reset default MoveIt parameters & tolerances (check class constructor)
  arm_group_.setMaxVelocityScalingFactor(def_vel_scal); 
  arm_group_.setMaxAccelerationScalingFactor(def_accel_scal); 

  arm_group_.setGoalJointTolerance(def_joint_tol);  
  arm_group_.setGoalPositionTolerance(def_pos_tol);            
  arm_group_.setGoalOrientationTolerance(def_orient_tol);

  return success;
}

// -------------------------------------------------------------------------------------------
bool cw2::adjustYaw(const std::string &shape_type,
  double new_yaw,
  double velocity_scaling,
  double accel_scaling,
  std::vector<double> tolerances)
{
bool success = false;

try {
  // Get current arm pose
  geometry_msgs::PoseStamped current_pose = arm_group_.getCurrentPose();

  // Threshold
  bool is_yaw_small = (std::abs(new_yaw) <= 0.088);  // (0.088 rad ~ 5 deg)

  // No adjustment when angle < 5 deg.
  if (is_yaw_small) { 
    ROS_INFO("Keeping current orientation");
    return true;  // exit if condition is True
  }

  // For crosses, add 90 deg. to grasp the arm with the correct orientation
  if (shape_type == "cross"){
    new_yaw += M_PI_2;
  }

  // Create new quaternion with the same pitch & roll but new yaw
  tf2::Quaternion q_new;
  q_new.setRPY(0.0, M_PI, 3*M_PI_4 + new_yaw);
  q_new.normalize();

  // Set the new orientation
  current_pose.pose.orientation = tf2::toMsg(q_new);

  // Clear targets & set target pose
  arm_group_.clearPoseTargets();
  arm_group_.setPoseTarget(current_pose);

  // Set velocity & acceleration scaling
  arm_group_.setMaxVelocityScalingFactor(velocity_scaling); 
  arm_group_.setMaxAccelerationScalingFactor(accel_scaling);

  // Set tolerances
  arm_group_.setGoalJointTolerance(tolerances[0]);
  arm_group_.setGoalPositionTolerance(tolerances[1]);
  arm_group_.setGoalOrientationTolerance(tolerances[2]);

  // Joint-space motion
  if (arm_group_.move() == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
    ROS_INFO("Yaw angle adjusted successfully.");
    success = true;
  } else {
    ROS_WARN("Yaw adjusted with Timed_Out error.");
    success = true;  
  }
}
catch(const std::exception& e) {
  ROS_ERROR("Exception in adjustGripperYaw: %s", e.what());
  success = false;
}

// Reset default MoveIt parameters & tolerances
arm_group_.setMaxVelocityScalingFactor(def_vel_scal); 
arm_group_.setMaxAccelerationScalingFactor(def_accel_scal); 

arm_group_.setGoalJointTolerance(def_joint_tol);  
arm_group_.setGoalPositionTolerance(def_pos_tol);            
arm_group_.setGoalOrientationTolerance(def_orient_tol);

return success;
}

// -------------------------------------------------------------------------------------------
bool 
cw2::moveToPoseWithFallback(const geometry_msgs::PoseStamped &target_pose) {
  geometry_msgs::PoseStamped pose_copy = target_pose;
  /* This function is for pure moving without tolerances for picking & placing. */

  // Ensure the target pose has a valid frame ID
  if (pose_copy.header.frame_id.empty()) {
    pose_copy.header.frame_id = "panda_link0";
  }

  // Set a default orientation if none is provided
  const auto& q = pose_copy.pose.orientation;
  if (q.x == 0 && q.y == 0 && q.z == 0 && q.w == 0) {
    tf2::Quaternion q_set;
    q_set.setRPY(0.0, M_PI, M_PI_4 + M_PI_2); // Default orientation (roll, pitch, yaw)
    q_set.normalize();
    pose_copy.pose.orientation = tf2::toMsg(q_set);
  }

  // Add the target pose to the waypoints for Cartesian planning
  std::vector<geometry_msgs::Pose> waypoints;
  waypoints.push_back(pose_copy.pose);
  arm_group_.clearPathConstraints();

  bool success = false;

  // Cartesian path planning
  if (planAndExecuteCartesian(waypoints, "moveToPoseWithFallback", 0.005)) {
    success = true;
  } else {
    // If Cartesian planning fails, fall back to joint-space planning
    ROS_WARN("Cartesian planning failed, switching to joint-space planning...");
    arm_group_.setPoseTarget(pose_copy);
    moveit::planning_interface::MoveGroupInterface::Plan joint_plan;

    if (arm_group_.plan(joint_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
      success = (arm_group_.execute(joint_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    }
  }

  // Check if all planning attempts failed
  if (!success) {
    ROS_ERROR("moveToPoseWithFallback: All planning attempts failed.");
    return false;
  }

  return true;
}

// -------------------------------------------------------------------------------------------
// Cartesian path planning

/* Approach:
  - multiple planning attempts with increasing step size
  - multiple execution attempts with pose perturbation
  - adaptive tolerance relaxation for difficult moves
  - enhanced error handling and debugging output
  
  Important parameters:
  - eef_step (EE step size); lower for smoother path, higher for faster planning but jerky motion & collision
  - path_fraction_threshold (minimum completion) */

bool
cw2::planAndExecuteCartesian(const std::vector<geometry_msgs::Pose> &waypoints,
                            const std::string &action_name,
                            double eef_step)
{
  moveit_msgs::RobotTrajectory trajectory;
  int max_planning_attempts = 20;

  // Outer loop: Try planning with increasingly larger step sizes
  for (int attempt = 1; attempt <= max_planning_attempts; attempt++) {
    
    // Calculate path
    ROS_INFO_STREAM(action_name << ": Planning attempt " << attempt << " with eef_step = " << eef_step);
    moveit_msgs::MoveItErrorCodes error_code;
    double fraction = arm_group_.computeCartesianPath(waypoints,      
                                                      eef_step,       // step size between waypoints
                                                      trajectory,     // output trajectory
                                                      true,           // avoid collisions (optional)
                                                      &error_code);   // error code output (optional)
    
    // Check if it completed at least 60% (motion often fails ~ 60%)
    if (fraction >= 0.60) {
      ROS_INFO_STREAM(action_name << ": Cartesian path computed successfully with fraction " << fraction);
      int max_exec_attempts = 20;
      
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
            perturbed.position.x += 0.01;
            perturbed.position.y += 0.01;
            perturbed.position.z += 0.01;
            ROS_INFO_STREAM(action_name << ": Perturbing final waypoint for retry.");
            
            // Recompute path with perturbed endpoint
            std::vector<geometry_msgs::Pose> perturbed_waypoints = waypoints;
            perturbed_waypoints.back() = perturbed;
            moveit_msgs::MoveItErrorCodes error_code;
            double fraction = arm_group_.computeCartesianPath(perturbed_waypoints,      
                                                              eef_step,       
                                                              trajectory,     
                                                              true,          
                                                              &error_code);   
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
        arm_group_.setGoalJointTolerance(def_joint_tol);
        arm_group_.setGoalPositionTolerance(def_pos_tol);
        arm_group_.setGoalOrientationTolerance(def_orient_tol);
        return true;
      } else {
        ROS_ERROR_STREAM(action_name << ": Execution failed even with lower tolerances.");
        arm_group_.setGoalJointTolerance(def_joint_tol);
        arm_group_.setGoalPositionTolerance(def_pos_tol);
        arm_group_.setGoalOrientationTolerance(def_orient_tol);
      }
    } else {
      ROS_WARN_STREAM(action_name << ": Cartesian path planning achieved only " << (fraction * 100.0)
                       << "%. Retrying with reduced constraints.");
    }

    // Increase the step size to reduce planning constraints
    eef_step *= 1.25;  
  }

  ROS_ERROR_STREAM(action_name << ": All planning attempts failed.");
  return false;
}

////////////////////////////////////////////////////////////////////////////////////////////////

// GRIPPER helper

bool
cw2::toggleGripper(double gripper_width,
                  const std::string &action_name,
                  double velocity_scaling,
                  double accel_scaling)
{
  double width = gripper_width; 
  double eachJoint = width / 2.0;
  
  std::vector<double> gripperJointTargets(2, eachJoint);
  hand_group_.setJointValueTarget(gripperJointTargets);
  moveit::planning_interface::MoveGroupInterface::Plan plan;

  // Set scaling
  hand_group_.setMaxVelocityScalingFactor(velocity_scaling); 
  hand_group_.setMaxAccelerationScalingFactor(accel_scaling); 
  
  bool success = false;

  if (hand_group_.plan(plan) != moveit::planning_interface::MoveItErrorCode::SUCCESS) {
    ROS_ERROR("%s failed.", action_name.c_str());
    success = false;
  }
  hand_group_.move();
  ros::Duration(1).sleep();
  ROS_INFO("%s successful.", action_name.c_str());
  success = true;

  // Reset scaling
  hand_group_.setMaxVelocityScalingFactor(def_vel_scal); 
  hand_group_.setMaxAccelerationScalingFactor(def_accel_scal); 

  return success;
}

////////////////////////////////////////////////////////////////////////////////////////////////

// POINTCLOUD helpers

void
cw2::pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
{
  // Store ROS message
  latest_pointcloud_ = cloud_msg;

  // Convert ROS message to PCL pointcloud pointer
  pcl::fromROSMsg(*cloud_msg, *cloud_ptr_);
}

// ------------------------------------------------------------------------------------------- DELETE
// void
// cw2::pubFilteredPCMsg(ros::Publisher &pc_pub, PointC &pc)
// {
//   // Convert PCL pointcloud to ROS message
//   pcl::toROSMsg(pc, filtered_cloud_msg);

//   // Assing a header
//   filtered_cloud_msg.header.stamp = ros::Time::now();
//   filtered_cloud_msg.header.frame_id = "color";

//   // Publish pointcloud
//   pc_pub.publish(filtered_cloud_msg);
//   // ROS_INFO("Published filtered cloud with %d points", (int)pc.size());
// }



////////////////////////////////////////////////////////////////////////////////////////////////

// COMPUTER VISION helpers

double
cw2::getOrientationFomCluster(const PointCPtr &cluster)
{
  // Convert cluster to binary image
  cv::Mat binary_img = clusterToBinaryImg(cluster);

  // Debug: show binary image
  std::string package_path = ros::package::getPath("cw2_team_16");
  cv::imwrite(package_path + "/debug/binary.png", binary_img);
  ROS_INFO("Saved binary image");

  // Extract contours from binary image
  std::vector<std::vector<cv::Point>> contours = extractContours(binary_img);

  // // Debug: show contour in green
  // cv::Mat ContoursImg;
  // cv::cvtColor(binary_img, ContoursImg, cv::COLOR_GRAY2BGR);
  // cv::drawContours(ContoursImg, contours, -1, cv::Scalar(0,255,0), 1);
  // cv::imwrite(package_path + "/debug/contours.png", ContoursImg);
  // ROS_INFO("Saved image with contours");

  if (contours.size() != 1) {
    ROS_ERROR("None or more than one contour found. Try again.");  // occurs randomly
    return 0.0;
  }

  // Fit a minimum area rectangle to the contour & calculate rotation
  cv::RotatedRect contourRect = cv::minAreaRect(contours[0]); 
  double rect_angle = contourRect.angle;
  ROS_INFO("Rotated rectangle angle: %.2f deg", rect_angle);

  // // Debug: show contour in green & rotated rectangle in red
  // cv::Mat rectImg;
  // cv::cvtColor(binary_img, rectImg, cv::COLOR_GRAY2BGR); 
  // cv::drawContours(rectImg, contours, 0, cv::Scalar(0, 255, 0), 1); 
  // cv::Point2f vertices[4];
  // contourRect.points(vertices);
  // for (int i = 0; i < 4; i++) {
  //   cv::line(rectImg, vertices[i], vertices[(i+1)%4], cv::Scalar(0, 0, 255), 2);
  // }
  // cv::imwrite(package_path + "/debug/rectangle.png", rectImg);
  // ROS_INFO("Saved image with rotated rectangle");

  return rect_angle;
}

// ----------------------------------------------------------------------------
cv::Mat
cw2::clusterToBinaryImg(const PointCPtr &cluster)
{
  // Calculate xy spread of the cluster (bounding box)
  Eigen::Vector4f min_pt, max_pt;
  pcl::getMinMax3D(*cluster, min_pt, max_pt);
  double box_width = (max_pt[0] - min_pt[0]);
  double box_height = (max_pt[1] - min_pt[1]);
  // ROS_INFO("XY bounding box: %.3f m by %.3f m", box_width, box_height);

  // Define resolution
  double resolution = camera_resolution / 2.0;  // 2 pixels per mm

  // Derive image dimensions
  int img_width  = static_cast<int>(std::ceil((box_width) / resolution));
  int img_height = static_cast<int>(std::ceil((box_height) / resolution));
  // ROS_INFO("Image dimensions (width x height): %d x %d pixels", img_width, img_height);

  // Create a blank image (8-bit grayscale)
  cv::Mat binary_img = cv::Mat::zeros(img_height, img_width, CV_8UC1);

  // Determine coordinates of each cluster point
  for (const auto &pt : cluster->points)
  {
    // Project the 3D point into 2D (x-y plane)
    int w = static_cast<int>((pt.x - min_pt[0]) / resolution);  // width
    int h = static_cast<int>((pt.y - min_pt[1]) / resolution);  // height
    
    // Keep coordinates within image bounds
    if (w >= 0 && w < img_width && h >= 0 && h < img_height)
    {
      binary_img.at<uchar>(h, w) = 255;
    }
  }

  // Morphological operations (optional, no particular benefits)
  // cv::erode(binary_img, binary_img, cv::Mat(), cv::Point(-1,-1), 1);
  // cv::morphologyEx(binary_img,
  //                 binary_img,
  //                 cv::MORPH_OPEN,  
  //                 cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2, 2))); 
  cv::bitwise_not(binary_img, binary_img);
  cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
  cv::morphologyEx(binary_img, binary_img, cv::MORPH_OPEN, kernel);
  cv::flip(binary_img, binary_img, 1);
  return binary_img;
}

// ----------------------------------------------------------------------------
std::vector<std::vector<cv::Point>>
cw2::extractContours(const cv::Mat &binary_img)
{
  // Invert the binary image to extract contours more easily
  cv::Mat inverted;
  cv::bitwise_not(binary_img, inverted);

  // Find contours
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(inverted, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  if (contours.size() != 1){
    ROS_INFO("Found %lu contours", contours.size());
  }
  return contours;
}

// ----------------------------------------------------------------------------
double
cw2::findActualAngle(const std::string &shape_type,
                    double angle)
{ 
  /* minAreaRect() returns the minimum area rotated rectangle:
    - 4 corner points ordered clockwise, starting from the point with the lowest y
    - if 2 points have the same lowest y, the rightmost point = starting point
    - returns angle between the line (starting to end point) & the horizontal
    - value always lies between [-90,0). */

  // When T1 any orientation = False, rotated rectangle images show:
  //  angle = -45 deg for crosses (because +shaped)
  //  angle = -90 deg for noughts (because 0 excluded)

  if (shape_type == "cross") {
    angle += 45.0;
  } else if (shape_type == "nought") {
  angle += 90.0;
  }

  // Four-fold symmetry > simplify by normalizing to [-45, 45]
  while (angle > 45.0)
  angle -= 90.0;
  while (angle < -45.0)
  angle += 90.0;

  // Return inverse sign (from debug observations)
  //  - positive angle for counter-clockwise rotation
  //  - negative angle for clockwise rotation 
  return - angle;
}



/////////////////////////////////////////////////////////////////////////////////
// Task 2 & 3 helpers


std::string
cw2::computeColor(double avg_r, double avg_g, double avg_b)
{ 
  // Normalize to [0, 1] range
  double r = avg_r / 255.0;
  double g = avg_g / 255.0;
  double b = avg_b / 255.0;

  // Allowable deviation
  const double tol = 0.1;

  auto within = [](double val, double target, double tol) {
    return std::abs(val - target) < tol;
  };

  if (within(r, 0.1, tol) && within(g, 0.1, tol) && within(b, 0.8, tol)) {
    return "blue";
  } else if (within(r, 0.8, tol) && within(g, 0.1, tol) && within(b, 0.1, tol)) {
    return "red";
  } else if (within(r, 0.8, tol) && within(g, 0.1, tol) && within(b, 0.8, tol)) {
    return "purple";
  } else if (within(r, 0.5, tol) && within(g, 0.2, tol) && within(b, 0.2, tol)) {
    return "darkred";  // basket
  } else if (within(r, 0.1, tol) && within(g, 0.1, tol) && within(b, 0.1, tol)) {
    return "black";  // obstacle
  }

  return "unknown";
}


///////////////////////////////////////////////////////////////////////////////
void 
cw2::colorFilter(const PointCPtr &in_cloud_ptr, PointCPtr &out_cloud_ptr) {
  if (!in_cloud_ptr || in_cloud_ptr->empty()) {
    ROS_WARN("Input point cloud is empty, skipping filtering.");
    out_cloud_ptr.reset(new pcl::PointCloud<pcl::PointXYZRGBA>);  // Ensure output is always valid
    return;
  }

  // Create a new local variable to temporarily store the result, avoiding memory sharing issues
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);

  for (const auto &point : in_cloud_ptr->points) {
    double r = static_cast<double>(point.r);
    double g = static_cast<double>(point.g);
    double b = static_cast<double>(point.b);

    std::string color = computeColor(r, g, b);

    if (color == "blue" || color == "red" || color == "purple" || color == "darkred" || color == "black"){
      temp_cloud->points.push_back(point);
    }
  }

  // Set width, height, and is_dense to avoid PCL crashes
  temp_cloud->width = temp_cloud->points.size();
  temp_cloud->height = 1;
  temp_cloud->is_dense = true;

  // Ensure frame_id is not empty
  // out_cloud_ptr->header.frame_id = "color"; 
  out_cloud_ptr->header.frame_id = base_frame_;
  out_cloud_ptr->header.stamp = ros::Time::now().toNSec();

  ROS_INFO("Filtered point cloud has %zu points", temp_cloud->points.size());

  // Assign to the external smart pointer after completion
  out_cloud_ptr = temp_cloud;  // Safe assignment
}
///////////////////////////////////////////////////////////////////////////////
void 
cw2::classifyClustersFromMergedCloud(PointCPtr full_cloud) {
  if (!full_cloud || full_cloud->empty()) {
    ROS_WARN("Merged cloud is empty.");
    return;
  }
  
  // Remove invalid points
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cleaned(new pcl::PointCloud<pcl::PointXYZRGBA>);
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*full_cloud, *cleaned, indices);

  // Downsample the point cloud
  pcl::VoxelGrid<pcl::PointXYZRGBA> vg;
  vg.setInputCloud(cleaned);
  vg.setLeafSize(0.002f, 0.002f, 0.002f);
  PointCPtr filtered(new pcl::PointCloud<pcl::PointXYZRGBA>);
  vg.filter(*filtered);

  // Remove NaN points again
  std::vector<int> valid_indices;
  pcl::removeNaNFromPointCloud(*filtered, *filtered, valid_indices);

  // Perform clustering
  pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGBA>);
  tree->setInputCloud(filtered);
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZRGBA> ec;
  ec.setClusterTolerance(0.005);
  ec.setMinClusterSize(200);
  ec.setMaxClusterSize(90000);
  ec.setSearchMethod(tree);
  ec.setInputCloud(filtered);
  ec.extract(cluster_indices);

  // Iterate through each cluster
  int cluster_index = 0; // Initialize an index counter
  for (const auto& indices : cluster_indices) {
    PointCPtr cluster_raw(new pcl::PointCloud<pcl::PointXYZRGBA>);
    for (int idx : indices.indices)
      cluster_raw->points.push_back(filtered->points[idx]);

    // Further remove NaN points within the cluster
    PointCPtr cluster(new pcl::PointCloud<pcl::PointXYZRGBA>);
    std::vector<int> valid_indices;
    pcl::removeNaNFromPointCloud(*cluster_raw, *cluster, valid_indices);

    if (cluster->empty()) {
      ROS_WARN("Cluster has no valid points after removing NaNs. Skipping...");
      continue;
    }

    // Convert cluster to image and calculate average color
    cv::Scalar avg_color;
    cv::Mat image = convertClusterToImage(cluster, 0.0015, avg_color);

    std::string color = computeColor(avg_color[2], avg_color[1], avg_color[0]);

    std::string shape;
    ShapeInfo info = classifyShapeFromImage(image);
    double angle_deg = getOrientationFomCluster(cluster);

    if (color == "black") {
      shape = "obstacle";
    } else if (color == "darkred") {
      shape = "basket";
    } else {
      shape = info.shape;
    }
    
    geometry_msgs::Point centroid = get3DCenter(cluster);

    // Determine the size of object
    double size_x;
    if (shape == "nought") {
      if (cluster->size() < 3000) {
        size_x = 0.2;
      } else if (cluster->size() >= 3000 && cluster->size() < 5800) {
        size_x = 0.3;
      } else {
        size_x = 0.4;
      }
    } else if (shape == "cross") {
      if (cluster->size() < 1800) {
        size_x = 0.2;
      } else if (cluster->size() >= 1800 && cluster->size() < 3600) {
        size_x = 0.3;
      } else {
        size_x = 0.4;
      }
    } else {
      size_x = 0.0;
    }

    // Save the cluster information
    MultiViewObject obj;
    obj.shape = shape;
    obj.color = color;
    obj.center_x = centroid.x;
    obj.center_y = centroid.y;
    obj.center_z = centroid.z;
    obj.angle_deg = angle_deg;
    obj.point_count = cluster->size();
    obj.pointcloud = cluster;
    obj.unit_size_x = size_x;

    merged_objects.push_back(obj);

    // Increment the index counter
    ++cluster_index;
  }
}

///////////////////////////////////////////////////////////////////////////////
ShapeInfo 
cw2::classifyShapeFromImage(const cv::Mat& image) {
  ShapeInfo info = {"unknown", 0.0f};

  if (image.empty()) return info;

  // Step 1: Extract the largest contour
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(image.clone(), contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
  if (contours.empty()) return info;

  auto max_it = std::max_element(contours.begin(), contours.end(),
    [](const std::vector<cv::Point>& a, const std::vector<cv::Point>& b) {
      return cv::contourArea(a) < cv::contourArea(b);
  });
  const auto& main_contour = *max_it;

  // Step 2: Fit a minimum bounding rectangle to extract the angle
  cv::RotatedRect rect = cv::minAreaRect(main_contour);
  float angle = rect.angle;
  info.angle_deg = angle;

  // Step 3: Rotate the image back to align it
  cv::Point2f center(image.cols / 2.0f, image.rows / 2.0f);
  cv::Mat rot_mat = cv::getRotationMatrix2D(center, -angle, 1.0);
  cv::Mat rotated;
  cv::warpAffine(image, rotated, rot_mat, image.size(), cv::INTER_NEAREST);

  // Step 4: Analyze the central region to determine the hollow level
  float roi_ratio = 0.3f;
  int img_w = rotated.cols;
  int img_h = rotated.rows;
  int roi_w = static_cast<int>(img_w * roi_ratio);
  int roi_h = static_cast<int>(img_h * roi_ratio);
  int cx = img_w / 2;
  int cy = img_h / 2;
  cv::Rect center_roi(cx - roi_w / 2, cy - roi_h / 2, roi_w, roi_h);
  center_roi &= cv::Rect(0, 0, img_w, img_h);

  cv::Mat center_patch = rotated(center_roi);
  float center_fill = static_cast<float>(cv::countNonZero(center_patch)) / (roi_w * roi_h);

  // Step 5: Identify the shape
  if (center_fill < 0.2f) {
    info.shape = "nought";
  } else if (center_fill > 0.6f) {
    info.shape = "cross";
  } else {
    info.shape = "unknown";
  }

  return info;
}

///////////////////////////////////////////////////////////////////////////////
cv::Mat 
cw2::convertClusterToImage(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& cluster,
                                      float resolution, cv::Scalar& avg_color) {
  if (!cluster || cluster->empty()) return cv::Mat();

  float min_x = FLT_MAX, max_x = -FLT_MAX;
  float min_y = FLT_MAX, max_y = -FLT_MAX;
  int r_sum = 0, g_sum = 0, b_sum = 0;

  for (const auto& pt : cluster->points) {
    min_x = std::min(min_x, pt.x);
    max_x = std::max(max_x, pt.x);
    min_y = std::min(min_y, pt.y);
    max_y = std::max(max_y, pt.y);
    r_sum += pt.r;
    g_sum += pt.g;
    b_sum += pt.b;
  }

  float width = max_x - min_x;
  float height = max_y - min_y;
  int img_w = static_cast<int>(width / resolution) + 1;
  int img_h = static_cast<int>(height / resolution) + 1;

  cv::Mat image = cv::Mat::zeros(img_h, img_w, CV_8UC1);

  for (const auto& pt : cluster->points) {
    int u = static_cast<int>((pt.x - min_x) / resolution);
    int v = static_cast<int>((pt.y - min_y) / resolution);
    if (u >= 0 && u < img_w && v >= 0 && v < img_h) {
      image.at<uchar>(v, u) = 255;
    }
  }

  cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
  cv::dilate(image, image, kernel);

  int num = cluster->points.size();
  avg_color = cv::Scalar(b_sum / num, g_sum / num, r_sum / num);  // OpenCV uses BGR


  return image;
}

///////////////////////////////////////////////////////////////////////////////
geometry_msgs::Point 
cw2::get3DCenter(PointCPtr cloud) {
  float min_x = FLT_MAX, min_y = FLT_MAX, min_z = FLT_MAX;
  float max_x = -FLT_MAX, max_y = -FLT_MAX, max_z = -FLT_MAX;

  for (const auto& pt : cloud->points) {
    if (!pcl::isFinite(pt)) continue;
    min_x = std::min(min_x, pt.x);
    max_x = std::max(max_x, pt.x);
    min_y = std::min(min_y, pt.y);
    max_y = std::max(max_y, pt.y);
    min_z = std::min(min_z, pt.z);
    max_z = std::max(max_z, pt.z);
  }

  geometry_msgs::Point center;
  center.x = (min_x + max_x) / 2.0f;
  center.y = (min_y + max_y) / 2.0f;
  center.z = (min_z + max_z) / 2.0f;

  return center;
}
