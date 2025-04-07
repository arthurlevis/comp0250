/* feel free to change any part of this file, or delete this file. In general,
you can do whatever you want with this template code, including deleting it all
and starting from scratch. The only requirment is to make sure your entire 
solution is contained within the cw2_team_<your_team_number> package */

#include <cw2_class.h> 

///////////////////////////////////////////////////////////////////////////////

cw2::cw2(ros::NodeHandle nh):
  tf_buffer_(ros::Duration(15)),
  tf_listener_(tf_buffer_),
  g_cloud_ptr(new PointC),
  g_cloud_filtered(new PointC),
  cloud_ptr_(new pcl::PointCloud<pcl::PointXYZRGBA>())
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
  
  // NEW: 
  // MoveIt planning parameters (joint-space planning only)
  arm_group_.setPlanningTime(60);            
  arm_group_.setNumPlanningAttempts(2000);
  arm_group_.allowReplanning(true);

  // MoveIt execution parameters (both Cartesian & joint-space planning)
  arm_group_.setMaxVelocityScalingFactor(0.5); 
  hand_group_.setMaxVelocityScalingFactor(0.5); 

  // Default goal tolerances (both Cartesian & joint-space planning)
  arm_group_.setGoalJointTolerance(def_joint_tol);        // 0.01 rad (~0.6 deg) per joint      
  arm_group_.setGoalPositionTolerance(def_pos_tol);       // 10 mm 
  arm_group_.setGoalOrientationTolerance(def_orient_tol); // 0.01 rad

  // Home pose
  home_pose_ = arm_group_.getCurrentPose(); 

  // Initialize pointcloud subscriber & publisher
  pc_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>(
    "/r200/camera/depth_registered/points", 1, &cw2::pointCloudCallback, this);
  g_pub_cloud = nh_.advertise<sensor_msgs::PointCloud2> ("cw2/filtered_cloud", 1, true);
  ROS_INFO("cw2 class initialised");

  ROS_INFO("cw2 class initialised");
}
///////////////////////////////////////////////////////////////////////////////

bool
cw2::t1_callback(cw2_world_spawner::Task1Service::Request &request,
  cw2_world_spawner::Task1Service::Response &response) 
{
  /* function which should solve task 1 */

  ROS_INFO("The coursework solving callback for task 1 has been triggered");

  // i. Extract inputs
  geometry_msgs::PointStamped object_point = request.object_point;
  std::string shape_type = request.shape_type;
  geometry_msgs::PointStamped goal_point = request.goal_point;

  // ii. Lift arm to get larger FoV
  ROS_DEBUG("Lifting the arm...");
  if (!liftArm(object_point)) {
  ROS_ERROR("Failed to lift arm.");
  return true;
  }
  ROS_DEBUG("Lifted the arm.");
  
  // iii. Move the arm to above the object
  ROS_DEBUG("Moving above object...");
  if (!moveAboveObject(object_point, shape_type)) {
  ROS_ERROR("Failed to move above offset.");
  return true;
  }
  ROS_DEBUG("Reached above object.");

  // Allow some time to process updated pointcloud
  ros::Duration(2.0).sleep(); 
  object_point.header.stamp = ros::Time::now();
  ros::spinOnce();
  ros::Duration(0.2).sleep();
  ros::spinOnce();
  ros::Duration(0.2).sleep();

  // --------------------------------------------------------------------------------
  // Calculate orientation from object cluster
  
  // Convert the object point to a PoseStamped message
  geometry_msgs::PoseStamped object_pose = pointStampedToPoseStamped(object_point);

  // Extract object from pointcloud (cluster)
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cluster = getObjectCluster(object_point);
  if (!cluster || cluster->empty()) {
    ROS_ERROR("Failed to extract object cluster.");
    return false;
  }

  // Convert cluster to binary image
  cv::Mat binaryImg = clusterToBinaryImage(cluster, resolution);
  if (binaryImg.empty()) {
    ROS_ERROR("Binary image is empty!");
    return false;
  }

  // Extract contours from the binary image
  std::vector<std::vector<cv::Point>> contours = extractContours(binaryImg);
  ROS_INFO("Extracted %lu contours", contours.size());
  if (contours.empty()) {
    ROS_ERROR("No contours found.");
    return false;
  }

  // Calculate centre in binary image
  cv::Moments m = cv::moments(binaryImg, true);
  cv::Point globalCenter;
  if (m.m00 != 0)
    globalCenter = cv::Point(static_cast<int>(m.m10/m.m00), static_cast<int>(m.m01/m.m00));
  else
    globalCenter = cv::Point(binaryImg.cols/2, binaryImg.rows/2);
  ROS_INFO("Global center (image coordinates): [%d, %d]", globalCenter.x, globalCenter.y);

  // Select longest shape segment
  std::vector<cv::Point> armContour = selectSegment(contours);
  if (armContour.empty()) {
    ROS_ERROR("No segment selected.");
    return false;
  }

  // Calculate orientation from contour
  cv::RotatedRect armRect = cv::minAreaRect(armContour);  // returns rotated rectangle
  angle = armRect.angle;  // returns angle relative to rectangle's width
  if (armRect.size.width < armRect.size.height) {
    angle += 90.0; 
  } 
  ROS_INFO("Detected arm angle: %.2f degrees", angle);

  // Convert angle to radians
  double yaw = angle * CV_PI / 180.0;
  // --------------------------------------------------------------------------------

  // 1. Adjust gripper yaw angle
  ROS_DEBUG("Adjusting gripper's yaw...");
  if (!adjustGripperYaw(yaw)) {
  ROS_ERROR("Failed rotate gripper.");
  return true;
  }
  ROS_DEBUG("Gripper yaw adjusted successfully.");

  // 2. Adjust gripper x,y positions  // Work in Progress
  ROS_DEBUG("Adjusting gripper's XY...");
  if (!adjustGripperXY(shape_type, yaw)) {
  ROS_ERROR("Failed to move gripper.");
  return true;
  }
  ROS_DEBUG("Gripper XY adjusted successfully.");

  // 3. Open the gripper
  ROS_DEBUG("Opening gripper...");
  if (!openGripper()) {
  ROS_ERROR("Failed to open gripper.");
  return true;
  }
  ROS_DEBUG("Gripper opened successfully.");

  // 4. Lower the arm to grasp position
  ROS_DEBUG("Lowering arm to object...");
  if (!lowerToObject(object_point)) {  
  ROS_ERROR("Failed to lower to object.");
  return true;
  }
  ROS_DEBUG("Reached grasp position.");

  // 5. Close the gripper to grasp the object.
  ROS_DEBUG("Closing gripper...");
  if (!closeGripper()) {
  ROS_ERROR("Failed to grasp the object.");
  return true;
  }
  ROS_DEBUG("Object grasped successfully.");

  // note: if the gripper fails to hold the shape, the arm will still execute steps 5–7,
  // potentially placing nothing in the basket.

  // 6. Move the arm to lift the object
  ROS_DEBUG("Lifting the object...");
  if (!liftObject(goal_point)) { 
  ROS_ERROR("Failed to lift the object.");
  return true;
  }
  ROS_DEBUG("Object lifted successfully.");

  // 7. Move the arm to pre-place offset
  ROS_DEBUG("Moving above basket...");
  if (!moveAboveBasket(goal_point, shape_type)) {
  ROS_ERROR("Failed to move above basket.");
  return true;
  }
  ROS_DEBUG("Reached above basket.");

  // 8. Lower the arm to safe release height (Optional)
  ROS_DEBUG("Lowering arm to safe release height...");
  if (!lowerToBasket(goal_point)) {  
  ROS_ERROR("Failed to lower the arm.");
  return true;
  }
  ROS_DEBUG("Reached safe release height.");

  // 9. Release object into the basket.
  ROS_DEBUG("Releasing object into the basket...");
  if (!releaseObject()) {
  ROS_ERROR("Failed to release object.");
  return true;
  }
  ROS_DEBUG("Object released successfully.");

  // 10. Return to home position ("ready" position defined at /panda_moveit_config/config/panda_arm.xacro)
  arm_group_.stop();
  arm_group_.clearPoseTargets();
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



  // === Step 1: Inspect the reference object ===

  // Define the reference and mystery object poses
  geometry_msgs::PoseStamped reference_pose;
  geometry_msgs::PoseStamped mystery_pose;
  const auto& ref_points = request.ref_object_points;
  reference_pose.pose.position = ref_points[0].point;

  // Move to the top of the reference object
  reference_pose.pose.position.z += 0.6; // Adjust height
  moveToPoseWithFallback(reference_pose);

  // Wait for the robot to stabilize
  ros::Duration(2.0).sleep();

  // Filter the point cloud to remove color other than red , blue, and purple
  colorFilter(g_cloud_ptr, g_cloud_filtered);

  // Segment the point cloud into objects
  std::vector<PointCPtr> clusters;
  merged_objects.clear();
  segmentObjectsWithOpenCV(g_cloud_filtered, clusters);
  updateMergedObjectsFromSingleView(clusters);
  mergeNearbyMergedObjectsWithPriorityVoting();

  // Check if any objects were detected
  if (merged_objects.empty()) {
    ROS_ERROR("No object detected at reference point.");
    return true;
  }

  // Print the detected objects (DEBUG)
  std::string ref_shape = merged_objects.front().shape;
  ROS_INFO("Reference object shape: %s", ref_shape.c_str());

  // === Step 2: Inspect the mystery object ===

  // Move to the top of mystery object pose
  mystery_pose.pose.position = request.mystery_object_point.point;
  mystery_pose.pose.position.z += 0.6; // Adjust height
  moveToPoseWithFallback(mystery_pose);

  // Wait for the robot to stabilize
  ros::Duration(2.0).sleep();

  // Filter the point cloud to remove color other than red , blue, and purple
  colorFilter(g_cloud_ptr, g_cloud_filtered);
  clusters.clear();
  merged_objects.clear();
  segmentObjectsWithOpenCV(g_cloud_filtered, clusters);
  updateMergedObjectsFromSingleView(clusters);
  mergeNearbyMergedObjectsWithPriorityVoting();

  if (merged_objects.empty()) {
    ROS_WARN("No object detected at mystery point.");
    response.mystery_object_num = 0;
    return true;
  }

  std::string mystery_shape = merged_objects.front().shape;
  ROS_INFO("Mystery object shape: %s", mystery_shape.c_str());

  // === Step 3: Map mystery shape to index
  if (mystery_shape == "nought") {
    response.mystery_object_num = (ref_shape == "nought") ? 1 : 2;
  } else if (mystery_shape == "cross") {
    response.mystery_object_num = (ref_shape == "nought") ? 2 : 1;
  } else {
    response.mystery_object_num = 0;
  }

  ROS_INFO("Final mystery_object_num = %ld", response.mystery_object_num);





  // // Service response
  // if (merged_objects.empty()) {
  //   ROS_WARN("No objects detected.");
  //   response.mystery_object_num = 0;
  //   return true;
  // }

  // const auto& target = *std::max_element(merged_objects.begin(), merged_objects.end(),
  //                                       [](const MultiViewObject& a, const MultiViewObject& b) {
  //                                         return a.point_count < b.point_count;
  //                                       });

  // if (target.shape == "nought")
  //   response.mystery_object_num = 1;
  // else if (target.shape == "cross")
  //   response.mystery_object_num = 2;

  // ROS_INFO("Mystery object -> shape=%s, color=%s, position=(%.3f, %.3f), result=%ld",
  //         target.shape.c_str(), target.color.c_str(), target.center_x, target.center_y,
  //         response.mystery_object_num);

  // // Move to the detected object
  // geometry_msgs::PoseStamped object_pose;
  // object_pose.pose.position.x = target.center_x;
  // object_pose.pose.position.y = target.center_y;
  // object_pose.pose.position.z = 0.2;

  // moveToPoseWithFallback(object_pose);

  return true;
}


///////////////////////////////////////////////////////////////////////////////

bool
cw2::t3_callback(cw2_world_spawner::Task3Service::Request &request,
  cw2_world_spawner::Task3Service::Response &response)
{

  ROS_INFO("The coursework solving callback for task 3 has been triggered");

    // 1. scan objects and basket
  std::vector<geometry_msgs::PoseStamped> camera_view_poses;
  geometry_msgs::PoseStamped pose1, pose2, pose3, pose4, pose5, pose6, pose7, pose8;
  
  // === Viewpoint 1: Bottom Right ===
  pose1.pose.position.x = 0.468;
  pose1.pose.position.y = 0.265;
  pose1.pose.position.z = 0.603;

  // === Viewpoint 2: Bottom Center ===
  pose2 = pose1;
  pose2.pose.position.y = 0.0;

  // === Viewpoint 3: Bottom Left ===
  pose3 = pose1;
  pose3.pose.position.y = -0.265;

  // === Viewpoint 4: Left Center ===
  pose4 = pose3;
  pose4.pose.position.x = 0;


  // === Viewpoint 5: Top Left ===
  //x=-0.432, y=-0.246, z=0.611
  // pose5.pose.position.x = -0.468;
  // pose5.pose.position.y = -0.265;
  // pose5.pose.position.z = 0.603;
  pose5 = pose3;
  pose5.pose.position.x = -0.468;

  // === Viewpoint 6: Top Center ===
  pose6 = pose5;
  pose6.pose.position.y = 0.0;

  // === Viewpoint 7: Top Right ===
  pose7 = pose5;
  pose7.pose.position.y = 0.265;

  // === Viewpoint 8: Right Center ===
  pose8 = pose7;
  pose8.pose.position.x = 0.0;


  camera_view_poses = {pose1, pose2, pose3, pose4, pose5, pose6, pose7, pose8};

  merged_objects.clear();

  // Note
  // Left Bottom x=0.468, y=-0.265, z=0.603
  // Right Bottom x=0.449, y=0.266, z=0.618

  for (const auto& pose : camera_view_poses) {
    moveToPoseWithFallback(pose);
    ros::Duration(1.5).sleep();
  

    colorFilter(g_cloud_ptr, g_cloud_filtered);
    std::vector<PointCPtr> clusters;
    segmentObjectsWithOpenCV(g_cloud_filtered, clusters);
    updateMergedObjectsFromSingleView(clusters);
  }

  mergeNearbyMergedObjectsWithPriorityVoting();
  finalizeVotingResults();

  // 2.print and count all detected objects 
  int cross_count = 0;
  int nought_count = 0;
  int black_count = 0;
  // int64 total_num_shapes = 0;
  // int64 num_most_common_shape = 0;

  // obj_num = detected_objects.size();
  


  for (const auto& obj : merged_objects) {
    if (obj.color == "black") {
      black_count++;
    } else if (obj.shape == "cross") {
      cross_count++;
    } else if (obj.shape == "nought") {
      nought_count++;
    }

    // ROS_INFO("Merged Object -> Shape: %s | Color: %s | Center: (%.3f, %.3f)",
    //          obj.shape.c_str(), obj.color.c_str(), obj.center_x, obj.center_y);
  }

  ROS_INFO("Merged Summary -> Cross: %d, Nought: %d, Black (obstacles): %d",
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
    ROS_WARN("Equal number of cross and nought. Considering both.");
  } else {
    ROS_ERROR("No valid cross or nought objects found.");
    return false;
  }

  


  // ROS_INFO("Most common shape(s): %s", candidate_shapes[0].c_str());
  // if (candidate_shapes.size() > 1) {
  //   ROS_INFO("Most common shape(s): %s and %s", candidate_shapes[0].c_str(), candidate_shapes[1].c_str());
  // }
  // ROS_INFO("Most common shape count: %d", most_common_shape);


   // Step 3: Find closest object among candidate shapes
  double min_dist = std::numeric_limits<double>::max();
  const MultiViewObject* target = nullptr;

  for (const auto& obj : merged_objects) {
    if (obj.color == "black") continue;
    if (std::find(candidate_shapes.begin(), candidate_shapes.end(), obj.shape) == candidate_shapes.end()) continue;

    double dist = std::hypot(obj.center_x, obj.center_y);
    if (dist < min_dist) {
      min_dist = dist;
      target = &obj;
    }
  }

  if (!target) {
    ROS_ERROR("No valid object matched for pickup.");
    return false;
  }

  // Step 4: Find the basket (only keep the one with most points)
  const MultiViewObject* basket_obj = nullptr;
  int max_basket_points = -1;

  for (const auto& obj : merged_objects) {
    if (obj.shape == "basket" && obj.point_count > max_basket_points) {
      basket_obj = &obj;
      max_basket_points = obj.point_count;
    }
  }

  if (!basket_obj) {
  ROS_WARN("Basket not detected. Using default basket position.");

    basket_obj.center_x = -0.41 - 0.15;
    basket_obj.center_y = -0.36;
    basket_obj.shape = "basket";
    basket_obj.color = "darkred";
  }
  //Shape: basket | Color: darkred | Center: (-0.409, -0.362)


  ROS_INFO("Target to pick: Shape=%s, Position=(%.3f, %.3f)", target->shape.c_str(),
           target->center_x, target->center_y);
  ROS_INFO("Basket position: (%.3f, %.3f)", basket_obj->center_x, basket_obj->center_y);

  // Step 5: Move to the target object
  geometry_msgs::PointStamped pick_point;
  pick_point.point.x = target->center_x - 0.045; // This offset is important
  pick_point.point.y = target->center_y;
  pick_point.point.z = 0.4; // TODO: change

  geometry_msgs::PointStamped drop_point;
  drop_point.point.x = basket_obj->center_x - 0.045; // This offset is important
  drop_point.point.y = basket_obj->center_y;
  drop_point.point.z = 0.04; // TODO: change

  // // Step 6: Execute grasp and drop (skip orientation)
  // if (!liftArm(pick_point)) return false;
  // if (!moveAboveObject(pick_point, target_obj->shape)) return false;
  // ros::Duration(1.0).sleep();

  // if (!openGripper()) return false;
  // if (!lowerToObject(pick_point)) return false;
  // if (!closeGripper()) return false;
  // if (!liftObject(drop_point)) return false;
  // if (!moveAboveBasket(drop_point, target_obj->shape)) return false;
  // if (!lowerToBasket(drop_point)) return false;
  // if (!releaseObject()) return false;


///////////TODO
  // ObjectInfo target_obj;
  // bool basket_found = false;
  // geometry_msgs::PoseStamped place_pose;
  // for (const auto &obj : merged_objects) {
  //   if (obj.shape == "basket") {
  //     place_pose.header.frame_id = "panda_link0";
  //     place_pose.pose.position.x = obj.center_x;
  //     place_pose.pose.position.y = obj.center_y;
  //     place_pose.pose.position.z = 0.15;
  //     basket_found = true;
  //     break;
  //   }
  // }

  // if (!basket_found) {
  //   ROS_ERROR("No basket found to place the object.");
  //   return false;
  // }

  // geometry_msgs::PoseStamped object_point;
  // object_point.pose.position.x = target_obj.center_x;
  // object_point.pose.position.y = target_obj.center_y;
  // object_point.pose.position.z = 0.12;




  // // 1. Move the arm to a pre-grasp offset
  // ROS_DEBUG("Moving to pre-grasp offset...");
  // if (!moveAboveObject(object_point, target_obj.shape)) {
  // ROS_ERROR("Failed to move to pre-grasp offset.");
  // return true;
  // }
  // ROS_DEBUG("Reached pre-grasp offset.");
////////

  // // 2. Open the gripper
  // ROS_DEBUG("Opening gripper...");
  // if (!openGripper()) {
  // ROS_ERROR("Failed to open gripper.");
  // return true;
  // }
  // ROS_DEBUG("Gripper opened successfully.");

  // // 3. Lower the arm to grasp position
  // ROS_DEBUG("Lowering arm to grasp the object...");
  // if (!lowerToObject(object_point)) {  
  // ROS_ERROR("Failed to lower to object.");
  // return true;
  // }
  // ROS_DEBUG("Reached grasp position.");

  // // 4. Close the gripper to grasp the object.
  // ROS_DEBUG("Closing gripper...");
  // if (!closeGripper()) {
  // ROS_ERROR("Failed to grasp the object.");
  // return true;
  // }
  // ROS_DEBUG("Object grasped successfully.");

  // // note: if the gripper fails to hold the shape, the arm will still execute steps 5–7,
  // // potentially placing nothing in the basket.

  // // 5. Move the arm to lift the object
  // ROS_DEBUG("Lifting the object...");
  // if (!liftObject(goal_point)) { 
  // ROS_ERROR("Failed to lift the object.");
  // return true;
  // }
  // ROS_DEBUG("Object lifted successfully.");

  // // 6. Move the arm to pre-place offset
  // ROS_DEBUG("Moving to pre-place offset...");
  // if (!moveToBasketOffset(goal_point)) {
  // ROS_ERROR("Failed to move to pre-place offset.");
  // return true;
  // }
  // ROS_DEBUG("Reached pre-place offset.");

  // // 7. Lower the arm to safe release height (Optional)
  // ROS_DEBUG("Lowering arm to safe release height...");
  // if (!lowerToBasket(goal_point)) {  
  // ROS_ERROR("Failed to lower the arm.");
  // return true;
  // }
  // ROS_DEBUG("Reached safe release height.");

  // // 8. Release object into the basket.
  // ROS_DEBUG("Releasing object into the basket...");
  // if (!releaseObject()) {
  // ROS_ERROR("Failed to release object.");
  // return true;
  // }
  // ROS_DEBUG("Object released successfully.");

  // // 9. Return to home position ("ready" position defined at /panda_moveit_config/config/panda_arm.xacro)
  // // arm_group_.stop();
  // // arm_group_.clearPoseTargets();
  // // arm_group_.setPlanningTime(20.0); 
  // // arm_group_.setNamedTarget("ready");
  // // if (arm_group_.move() == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
  // //   ROS_INFO("Returned to 'ready' position successfully.");
  // // } else {
  // //   ROS_WARN("Failed to move to 'ready' position.");
  // // }



  return true;
}

///////////////////////////////////////////////////////////////////////////////
// Pick & place helper functions

bool
cw2::liftArm(const geometry_msgs::PointStamped &object_point)
{
  // Get the current arm pose
  geometry_msgs::PoseStamped current_pose = arm_group_.getCurrentPose(); 
  
  // Lower the arm (add margin for table height)
  current_pose.pose.position.z = object_point.point.z + 0.75;

  // Create a waypoint vector for Cartesian path planning
  std::vector<geometry_msgs::Pose> waypoints;
  waypoints.push_back(current_pose.pose);
  
  // Execute the Cartesian path
  if (!planAndExecuteCartesian(waypoints, "liftArm")) {
    ROS_ERROR("liftArm: Motion failed after multiple attempts.");
    return false;
  }
  ROS_INFO("liftArm: Arm lifted.");
  return true;
}

// ----------------------------------------------------------------------------
bool
cw2::moveAboveObject(const geometry_msgs::PointStamped &object_point,
                    const std::string &shape_type)
{
  // Get the current arm pose
  geometry_msgs::PoseStamped current_pose = arm_group_.getCurrentPose();

  // Define above_object_pose pose
  geometry_msgs::PoseStamped above_object_pose;
  above_object_pose.header = object_point.header; 

  // Calculate position (centered about object)

  // // T1 ANY ORIENTATION = False
  // above_object_pose.pose.position.x = object_point.point.x;  // for grasping
  // above_object_pose.pose.position.y = object_point.point.y;  // for grasping
  // above_object_pose.pose.position.z = current_pose.pose.position.z;  // high enough when "ready"

  // Add x-y offsets according to shape (grasping strategy)
  // if (shape_type == "nought") {
  //   above_object_pose.pose.position.y += 0.08;
  // }
  // else if (shape_type == "cross") {
  //   above_object_pose.pose.position.x += 0.06;
  // }

  // T1 ANY ORIENTATION = True
  // Transform object_point to camera frame
  geometry_msgs::PointStamped object_cam_frame;
  try {
  object_cam_frame = tf_buffer_.transform(object_point,
                                          camera_frame_,
                                          ros::Duration(1.0));

    // Get the camera pose in the same frame as object_point (base frame)
    geometry_msgs::PoseStamped camera_pose;
    camera_pose.header.frame_id = camera_frame_;
    camera_pose.pose.orientation.w = 1.0;  
    camera_pose = tf_buffer_.transform(camera_pose,
                                      object_point.header.frame_id,
                                      ros::Duration(1.0));

    // Offset EE pose to center camera over object
    above_object_pose.pose.position.x = current_pose.pose.position.x + (object_point.point.x - camera_pose.pose.position.x);
    above_object_pose.pose.position.y = current_pose.pose.position.y + (object_point.point.y - camera_pose.pose.position.y);
    above_object_pose.pose.position.z = current_pose.pose.position.z;

  } catch (tf2::TransformException &ex) {
    ROS_WARN("Failed to transform object point to camera frame: %s", ex.what());
  }

  // Calculate orientation
  tf2::Quaternion q;
  q.setRPY(0.0, M_PI, M_PI_4 + M_PI_2);  // roll, pitch, yaw (top-down view)
  above_object_pose.pose.orientation = tf2::toMsg(q);

  // Set specific tolerances
  arm_group_.setGoalJointTolerance(0.05);       // 0.05 rad (~2.8 deg) per joint
  arm_group_.setGoalPositionTolerance(0.03);    // 30 mm
  arm_group_.setGoalOrientationTolerance(0.07); // 0.07 rad (~4 deg)

  // Create a waypoint vector
  std::vector<geometry_msgs::Pose> waypoints;
  waypoints.push_back(above_object_pose.pose);

  // Execute Cartesian path
  if (!planAndExecuteCartesian(waypoints, "moveAboveObject")) {
    ROS_ERROR("moveAboveObject: Motion failed after multiple attempts.");
    // No need to reset default parameters & tolerances (already reset in helper)
    return false;
  }
  return true;
}

// ----------------------------------------------------------------------------
bool
cw2::adjustGripperYaw(double yaw)
{
  // Get the current arm pose
  geometry_msgs::PoseStamped current_pose = arm_group_.getCurrentPose();

  // Convert current orientation to RPY
  tf2::Quaternion q_current;
  tf2::fromMsg(current_pose.pose.orientation, q_current);
  double roll, pitch, current_yaw;
  tf2::Matrix3x3(q_current).getRPY(roll, pitch, current_yaw);

  // Create new quaternion with same roll/pitch but new yaw
  tf2::Quaternion q_new;
  q_new.setRPY(roll, pitch, yaw);
  q_new.normalize();
  
  // Set the new orientation
  current_pose.pose.orientation = tf2::toMsg(q_new);
  
  // Joint-space planning
  arm_group_.setPoseTarget(current_pose);
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  
  // Plan & execute
  bool success = (arm_group_.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) && 
                 (arm_group_.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS);
  if (!success) {
    ROS_ERROR("Joint-space planning/execution failed.");
    return false;
  }
  return true;
}

// ----------------------------------------------------------------------------
bool 
cw2::adjustGripperXY(const std::string &shape_type,
                    double yaw)
{
  // Get the current arm pose
  geometry_msgs::PoseStamped current_pose = arm_group_.getCurrentPose();

  // Default offsets
  double x_offset = 0.0;
  double y_offset = 0.0;
  
  // Set initial offsets based on shape (grasping strategy)
  if (shape_type == "nought") {
    y_offset = 0.08;  
  }
  else if (shape_type == "cross") {
    x_offset = 0.06;  
  }

  // Adjust x,y offsets to object's yaw
  double rotated_x_offset = x_offset * cos(yaw) - y_offset * sin(yaw);
  double rotated_y_offset = x_offset * sin(yaw) + y_offset * cos(yaw);
  
  // Apply rotated offsets to current pose
  current_pose.pose.position.x += rotated_x_offset;
  current_pose.pose.position.y += rotated_y_offset;

  // Create a waypoint vector for Cartesian path planning
  std::vector<geometry_msgs::Pose> waypoints;
  waypoints.push_back(current_pose.pose);
  
  // Execute the Cartesian path
  if (!planAndExecuteCartesian(waypoints, "adjustGripperXY")) {
    ROS_ERROR("adjustGripperXY: Motion failed after multiple attempts.");
    // No need to reset default parameters & tolerances (already reset in helper)
    return false;
  }
  
  ROS_INFO("adjustGripperXY: Adjusted gripper's x,y positions.");
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
    // No need to reset default parameters & tolerances (already reset in helper)
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
    // No need to reset default parameters & tolerances (already reset in helper)
    return false;
  }
  
  ROS_INFO("raiseFromObject: Object lifted successfully.");
  return true;
}

// // ----------------------------------------------------------------------------
bool
cw2::moveAboveBasket(const geometry_msgs::PointStamped &goal_point,
                    const std::string &shape_type)
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

  // // T1 ANY ORIENTATION = False
  // // Add x-y offsets to place object in centre of basket
  // if (shape_type == "nought") {
  //   pre_place_pose.pose.position.y += 0.08;
  // }
  // else if (shape_type == "cross") {
  //   pre_place_pose.pose.position.x += 0.06;
  // }

  // Set specific tolerances
  arm_group_.setGoalJointTolerance(0.05);        // 0.05 rad (~3 deg) per joint
  arm_group_.setGoalPositionTolerance(0.07);     // 60 mm 
  arm_group_.setGoalOrientationTolerance(0.25);  // 0.1 rad (~14 deg)

  // Create a waypoint vector for CARTESIAN path planning
  std::vector<geometry_msgs::Pose> waypoints;
  waypoints.push_back(pre_place_pose.pose);

  // Execute Cartesian paths
  if (!planAndExecuteCartesian(waypoints, "moveAboveBasket")) {
    ROS_ERROR("moveAboveBasket: Motion failed after multiple attempts.");
    // No need to reset default parameters & tolerances (already reset in helper)
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
    // No need to reset default parameters & tolerances (already reset in helper)
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

///////////////////////////////////////////////////////////////////////////////
// Cluster helpers
geometry_msgs::PoseStamped
cw2::pointStampedToPoseStamped(const geometry_msgs::PointStamped &pt)
{
  geometry_msgs::PoseStamped pose;
  pose.header = pt.header;              
  pose.pose.position = pt.point;        

  // Assign default orientation 
  pose.pose.orientation.x = 0.0;
  pose.pose.orientation.y = 0.0;
  pose.pose.orientation.z = 0.0;
  pose.pose.orientation.w = 1.0;
  return pose;
}

// ----------------------------------------------------------------------------
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr
cw2::getObjectCluster(const geometry_msgs::PointStamped &object_point) 
{
  if (cloud_ptr_->empty()) {
      ROS_ERROR("Input pointcloud empty.");
      return nullptr;
  }

  // Count number of points
  ROS_INFO("Input cloud has %lu points", cloud_ptr_->points.size());

  // // Debug: Check object pointcloud dimensions using Eigen vectors
  // Eigen::Vector4f min_pt, max_pt;
  // pcl::getMinMax3D(*cloud_ptr_, min_pt, max_pt);
  // ROS_INFO("Cloud bounds: \n\tX: [%.3f, %.3f]\n\tY: [%.3f, %.3f]\n\tZ: [%.3f, %.3f]", 
  //         min_pt[0], max_pt[0], min_pt[1], max_pt[1], min_pt[2], max_pt[2]);

  // Create a new point cloud for the cluster
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cluster(
      new pcl::PointCloud<pcl::PointXYZRGBA>);

  // Known object dimensions 
  const double GRID_CELL_SIZE = 0.04;  // 40 mm
  const double TOTAL_SIZE = 0.2;       // 200 mm
  const double HEIGHT = 0.04;          // 40 mm
  const double MARGIN = 0.04;          // allow 40 mm margin

  // Transform object_point to camera frame
  geometry_msgs::PointStamped object_cam_frame;
  try {
    object_cam_frame = tf_buffer_.transform(object_point,
      cloud_ptr_->header.frame_id,
      ros::Duration(1.0));
    ROS_INFO_STREAM("Transform: " << object_cam_frame);
  } catch (tf2::TransformException &ex) {
    ROS_WARN("TF transform failed: %s", ex.what());
    return nullptr;
  };

  // Define the centers
  double center_x = object_cam_frame.point.x;
  double center_y = object_cam_frame.point.y;
  double center_z = object_cam_frame.point.z;

  // Filter x-axis
  pcl::PassThrough<pcl::PointXYZRGBA> pass;
  pass.setInputCloud(cloud_ptr_);
  pass.setFilterFieldName("x");
  double x_min = center_x - (TOTAL_SIZE/2 + MARGIN);
  double x_max = center_x + (TOTAL_SIZE/2 + MARGIN);
  pass.setFilterLimits(x_min, x_max);
  pass.filter(*cluster);
  ROS_INFO("After X filter [%.3f, %.3f]: %lu points", x_min, x_max, cluster->points.size());

  // Filter y-axis
  pass.setInputCloud(cluster);
  pass.setFilterFieldName("y");
  double y_min = center_y - (TOTAL_SIZE/2 + MARGIN);
  double y_max = center_y + (TOTAL_SIZE/2 + MARGIN);
  pass.setFilterLimits(y_min, y_max);
  pass.filter(*cluster);
  ROS_INFO("After Y filter [%.3f, %.3f]: %lu points", y_min, y_max, cluster->points.size());

  // Filter z-axis
  pass.setInputCloud(cluster);
  pass.setFilterFieldName("z");
  double z_max = 0.70;   // camera z-position in base frame = 5cm lower than EE (lifted at 0.75)
  double z_min = 0.64;   // object height = 40 mm (add 20 mm margin)
  pass.setFilterLimits(z_min, z_max);
  pass.filter(*cluster);
  ROS_INFO("After Z filter [%.3f, %.3f]: %lu points", z_min, z_max, cluster->points.size());

  if (cluster->points.size() < 50) {  // Minimum points threshold
      ROS_WARN("Too few points in cluster.");
      return nullptr;
  }

  // // Debug output for resulting cluster bounds
  // pcl::getMinMax3D(*cluster, min_pt, max_pt);
  // ROS_INFO("Filtered cluster bounds: \n\tX: [%.3f, %.3f]\n\tY: [%.3f, %.3f]\n\tZ: [%.3f, %.3f]", 
  //          min_pt[0], max_pt[0], min_pt[1], max_pt[1], min_pt[2], max_pt[2]);

  return cluster;
}

///////////////////////////////////////////////////////////////////////////////
// Calculate orientation from cluster

cv::Mat
cw2::clusterToBinaryImage(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cluster,
                          double resolution)
{
  // Calculate min & max x-y
  Eigen::Vector4f min_pt, max_pt;
  pcl::getMinMax3D(*cluster, min_pt, max_pt);

  double width_m = (max_pt[0] - min_pt[0]);
  double height_m = (max_pt[1] - min_pt[1]);
  ROS_INFO("XY bounding box: %.3f m by %.3f m", width_m, height_m);

  // Calculate image dimensions (in pixels)
  int img_width  = static_cast<int>(std::ceil((max_pt[0] - min_pt[0]) / resolution));
  int img_height = static_cast<int>(std::ceil((max_pt[1] - min_pt[1]) / resolution));

  // Create a blank image (8-bit grayscale)
  cv::Mat binaryImg = cv::Mat::zeros(img_height, img_width, CV_8UC1);

  // Determine coordinates of each cluster point
  for (const auto &pt : cluster->points)
  {
    // Project the 3D point into 2D (x-y plane)
    int u = static_cast<int>((pt.x - min_pt[0]) / resolution);
    int v = static_cast<int>((pt.y - min_pt[1]) / resolution);
    
    // Ensure the coordinates are within image bounds
    if (u >= 0 && u < img_width && v >= 0 && v < img_height)
    {
      binaryImg.at<uchar>(v, u) = 255;
    }
  }

  // // Debug: check image orientation is the same than camera's top-down view
  // std::string package_path = ros::package::getPath("cw2_team_16");
  // cv::imwrite(package_path + "/src/binary.png", binaryImg);
  // ROS_INFO("Saved binary image");
  
  return binaryImg;
}

// ----------------------------------------------------------------------------
std::vector<std::vector<cv::Point>>
cw2::extractContours(const cv::Mat &binaryImg)
{
    // Invert the binary image to extract contours more easily
    cv::Mat inverted;
    cv::bitwise_not(binaryImg, inverted);

    // Find contours
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(inverted, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    // // NEW:
    // // Debug: check contours
    // std::string package_path = ros::package::getPath("cw2_team_16");
    // cv::Mat ContoursImg;
    // cv::cvtColor(binaryImg, ContoursImg, cv::COLOR_GRAY2BGR);
    // cv::drawContours(ContoursImg, contours, -1, cv::Scalar(0,0,255), 2);
    // cv::imwrite(package_path + "/src/contours.png", ContoursImg);
    // ROS_INFO("Saved image with contours");

    return contours;
}

// ----------------------------------------------------------------------------
std::vector<cv::Point>
cw2::selectSegment(const std::vector<std::vector<cv::Point>> &contours)
{
double bestLength = 0;
std::vector<cv::Point> bestContour;

// Find longest segment
for (const auto &contour : contours)
{
  cv::Moments m = cv::moments(contour);
  if (m.m00 <= 0) continue;

    // Fit a rotated rectangle to measure length
    cv::RotatedRect box = cv::minAreaRect(contour);
    float length = std::max(box.size.width, box.size.height);

    if (length > bestLength)
    {
      bestLength = length;
      bestContour = contour;
  }
}

return bestContour;
}

// Task 2 & 3

/////////////////////////////////////////////////////////////////////////////////
bool 
cw2::moveToPoseWithFallback(const geometry_msgs::PoseStamped &target_pose) {geometry_msgs::PoseStamped pose_copy = target_pose;

  if (pose_copy.header.frame_id.empty()) {
    pose_copy.header.frame_id = "panda_link0";
  }

  const auto& q = pose_copy.pose.orientation;
  if (q.x == 0 && q.y == 0 && q.z == 0 && q.w == 0) {
    tf2::Quaternion q_set;
    q_set.setRPY(0.0, M_PI, M_PI_4 + M_PI_2);
    q_set.normalize();
    pose_copy.pose.orientation = tf2::toMsg(q_set);
  }

  std::vector<geometry_msgs::Pose> waypoints;
  waypoints.push_back(pose_copy.pose);
  arm_group_.clearPathConstraints();

  bool success = false;
  if (planAndExecuteCartesian(waypoints, "moveToPoseWithFallback")) {
    success = true;
  } else {
    ROS_WARN("Cartesian planning failed, switching to joint-space planning...");
    arm_group_.setPoseTarget(pose_copy);
    moveit::planning_interface::MoveGroupInterface::Plan joint_plan;

    if (arm_group_.plan(joint_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
      success = (arm_group_.execute(joint_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    }
  }

  if (!success) {
    ROS_ERROR("moveToPoseWithFallback: All planning attempts failed.");
    return false;
  }

  return true;
}

////////////////////////////////////////////////////////////////////////////////
void
cw2::pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr &cloud_input_msg)
{
  latest_pointcloud_ = cloud_input_msg;

  // Convert ROS message to PCL point cloud
  pcl_conversions::toPCL (*cloud_input_msg, g_pcl_pc);
  pcl::fromPCLPointCloud2 (g_pcl_pc, *g_cloud_ptr);

  // Also convert directly to cloud_ptr_ (TODO: This is from Arthur's version and check if there is conflict later)
  pcl::fromROSMsg(*cloud_input_msg, *cloud_ptr_);

  // Filter the point cloud
  pubFilteredPCMsg(g_pub_cloud, *g_cloud_filtered);
}

///////////////////////////////////////////////////////////////////////////////
std::string
cw2::computeColor(double avg_r, double avg_g, double avg_b)
{ 
  // // Blue detection (Target: R=25, G=25, B=204)
  // if (avg_b > 170 && avg_r < 80 && avg_g < 80) {
  //   return "blue";
  // } 
  // // Red detection (Target: R=204, G=25, B=25)
  // else if (avg_r > 170 && avg_g < 80 && avg_b < 80) {
  //   return "red";
  // } 
  // // Purple detection (Target: R=204, G=25, B=204)
  // else if (avg_r > 170 && avg_b > 170 && avg_g < 80) {
  //   return "purple";
  // }
  // // Black detection (Target: R=0, G=0, B=0)
  // else if (avg_r < 80 && avg_g < 80 && avg_b < 80) {
  //   return "black";
  // }
  
  // // Default case
  // return "none";

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
cw2::pubFilteredPCMsg(ros::Publisher &pc_pub, PointC &pc)
{
  pcl::toROSMsg(pc, g_cloud_filtered_msg);
  g_cloud_filtered_msg.header.stamp = ros::Time::now();
  g_cloud_filtered_msg.header.frame_id = "color";

  pc_pub.publish(g_cloud_filtered_msg);
  // ROS_INFO("Published filtered cloud with %d points", (int)pc.size());
}

///////////////////////////////////////////////////////////////////////////////
void 
cw2::colorFilter(const PointCPtr &in_cloud_ptr, PointCPtr &out_cloud_ptr) {
  if (!in_cloud_ptr || in_cloud_ptr->empty()) {
    ROS_WARN("Input point cloud is empty, skipping filtering.");
    out_cloud_ptr.reset(new pcl::PointCloud<pcl::PointXYZRGB>);  // Ensure output is always valid
    return;
  }

  // Create a new local variable to temporarily store the result, avoiding memory sharing issues
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

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
  out_cloud_ptr->header.frame_id = "panda_link0";

  ROS_INFO("Filtered point cloud has %zu points", temp_cloud->points.size());

  // Assign to the external smart pointer after completion
  out_cloud_ptr = temp_cloud;  // Safe assignment
}

///////////////////////////////////////////////////////////////////////////////
// void 
// cw2::computeObjectCenters(const std::vector<PointCPtr> &clusters) {
//   detected_objects.clear(); // Clear old data before computing
//   std::string target_frame = "panda_link0";

//   for (const auto &cluster : clusters) {
//     if (cluster->empty()) continue;

//     // Compute bounding box
//     double min_x = std::numeric_limits<double>::max();
//     double max_x = std::numeric_limits<double>::lowest();
//     double min_y = std::numeric_limits<double>::max();
//     double max_y = std::numeric_limits<double>::lowest();

//     // Accumulate RGB values for color computation
//     double sum_r = 0.0, sum_g = 0.0, sum_b = 0.0;
//     int valid_points = 0;

//     for (const auto &point : cluster->points) {
//       // Update bounding box
//       min_x = std::min(min_x, static_cast<double>(point.x));
//       max_x = std::max(max_x, static_cast<double>(point.x));
//       min_y = std::min(min_y, static_cast<double>(point.y));
//       max_y = std::max(max_y, static_cast<double>(point.y));

//       // Accumulate color values
//       sum_r += static_cast<double>(point.r);
//       sum_g += static_cast<double>(point.g);
//       sum_b += static_cast<double>(point.b);
//       valid_points++;
//     }

//     // Compute center point
//     double center_x = (min_x + max_x) / 2.0;
//     double center_y = (min_y + max_y) / 2.0;

//     // Compute area
//     double area = (max_x - min_x) * (max_y - min_y);

//     // Compute color
//     std::string color = "unknown";
//     if (valid_points > 0) {
//       double avg_r = sum_r / valid_points;
//       double avg_g = sum_g / valid_points;
//       double avg_b = sum_b / valid_points;
//       color = computeColor(avg_r, avg_g, avg_b); // Directly call computeColor
//     }

//     // Transform center point to `base_link`
//     geometry_msgs::PointStamped point_in, point_out;
//     point_in.header.frame_id = latest_pointcloud_->header.frame_id;
//     point_in.header.stamp = latest_pointcloud_->header.stamp;
//     point_in.point.x = center_x;
//     point_in.point.y = center_y;
//     point_in.point.z = 0.0;

//     try {
//       tf_buffer_.transform(point_in, point_out, target_frame, ros::Duration(1.0));
//       center_x = point_out.point.x;
//       center_y = point_out.point.y;
//     } catch (tf2::TransformException &ex) {
//       ROS_WARN("Could not transform object center to %s: %s", target_frame.c_str(), ex.what());
//     }

//     // Store in detected_objects
//     detected_objects.push_back({center_x, center_y, area, color});
//   }

//   ROS_INFO("Detected %zu objects.", detected_objects.size());
// }
void 
cw2::computeObjectCenters(const std::vector<PointCPtr> &clusters) {
  detected_objects.clear(); // Clear old data before computing
  std::string target_frame = "panda_link0";

  for (const auto &cluster : clusters) {
    if (cluster->empty()) continue;

    // === Compute centroid ===
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cluster, centroid);

    double center_x = static_cast<double>(centroid[0]);
    double center_y = static_cast<double>(centroid[1]);

    // === Accumulate RGB values for color computation ===
    double sum_r = 0.0, sum_g = 0.0, sum_b = 0.0;
    int valid_points = 0;

    for (const auto &point : cluster->points) {
      sum_r += static_cast<double>(point.r);
      sum_g += static_cast<double>(point.g);
      sum_b += static_cast<double>(point.b);
      valid_points++;
    }

    std::string color = "unknown";
    if (valid_points > 0) {
      double avg_r = sum_r / valid_points;
      double avg_g = sum_g / valid_points;
      double avg_b = sum_b / valid_points;
      color = computeColor(avg_r, avg_g, avg_b);
    }

    // === Transform center point to target frame ===
    geometry_msgs::PointStamped point_in, point_out;
    point_in.header.frame_id = latest_pointcloud_->header.frame_id;
    point_in.header.stamp = latest_pointcloud_->header.stamp;
    point_in.point.x = center_x;
    point_in.point.y = center_y;
    point_in.point.z = 0.0;  // z ignored here

    try {
      tf_buffer_.transform(point_in, point_out, target_frame, ros::Duration(1.0));
      center_x = point_out.point.x;
      center_y = point_out.point.y;
    } catch (tf2::TransformException &ex) {
      ROS_WARN("Could not transform object center to %s: %s", target_frame.c_str(), ex.what());
    }

    // === Store the result ===
    detected_objects.push_back({center_x, center_y, static_cast<int>(cluster->size()), color});
  }

  ROS_INFO("Detected %zu objects.", detected_objects.size());
}

///////////////////////////////////////////////////////////////////////////////
void
cw2::segmentObjectsWithOpenCV(PointCPtr &in_cloud_ptr, std::vector<PointCPtr> &clusters) {
  if (!in_cloud_ptr || in_cloud_ptr->empty()) {
    ROS_WARN("Input point cloud is empty, skipping OpenCV segmentation.");
    return;
  }

  // ==== Step 1: Compute point cloud bounding box ====
  float min_x = FLT_MAX, max_x = -FLT_MAX;
  float min_y = FLT_MAX, max_y = -FLT_MAX;
  for (const auto &pt : in_cloud_ptr->points) {
    min_x = std::min(min_x, pt.x);
    max_x = std::max(max_x, pt.x);
    min_y = std::min(min_y, pt.y);
    max_y = std::max(max_y, pt.y);
  }

  // Add padding
  float padding = 0.02f; // 2cm padding
  min_x -= padding;
  max_x += padding;
  min_y -= padding;
  max_y += padding;

  float roi_width  = max_x - min_x;
  float roi_height = max_y - min_y;
  float roi_span   = std::max(roi_width, roi_height); // to make square image

  const int img_size = 1024;
  const float scale = img_size / roi_span; // m -> px

  // ==== Step 2: Project point cloud to image ====
  cv::Mat image = cv::Mat::zeros(img_size, img_size, CV_8UC1);
  std::vector<std::vector<int>> pixel_to_point_indices(img_size * img_size);  // map pixel to point indices

  for (size_t i = 0; i < in_cloud_ptr->points.size(); ++i) {
    const auto &pt = in_cloud_ptr->points[i];
    int px = static_cast<int>((pt.x - min_x) * scale);
    int py = static_cast<int>((pt.y - min_y) * scale);

    if (px >= 0 && px < img_size && py >= 0 && py < img_size) {
      image.at<uchar>(py, px) = 255;
      pixel_to_point_indices[py * img_size + px].push_back(i);
    }
  }

  // ==== Step 3: Morphological dilation ====
  // int dilation_size = (in_cloud_ptr->points.size() < 5000) ? 2 : 1;
  int dilation_size = 3;
  cv::Mat dilated;
  cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT,
                      cv::Size(2 * dilation_size + 1, 2 * dilation_size + 1));
  cv::dilate(image, dilated, kernel);

  // ==== Step 4: Connected components ====
  cv::Mat labels;
  int num_labels = cv::connectedComponents(dilated, labels, 8, CV_32S);

  // ==== Step 5: Map back to point cloud ====
  clusters.clear();
  clusters.resize(num_labels - 1);
  for (int i = 0; i < num_labels - 1; ++i) {
      clusters[i] = PointCPtr(new pcl::PointCloud<pcl::PointXYZRGB>);
  }

  for (int y = 0; y < img_size; ++y) {
    for (int x = 0; x < img_size; ++x) {
      int label = labels.at<int>(y, x);
      if (label == 0) continue;

      const auto &indices = pixel_to_point_indices[y * img_size + x];
      for (int idx : indices) {
        clusters[label - 1]->points.push_back(in_cloud_ptr->points[idx]);
      }
    }
  }

  for (size_t i = 0; i < clusters.size(); ++i) {
    clusters[i]->width = clusters[i]->points.size();
    clusters[i]->height = 1;
    clusters[i]->is_dense = true;
    ROS_DEBUG("Cluster %zu has %zu points", i, clusters[i]->size());
  }

  bool raw_ok = cv::imwrite("/home/hxy/Desktop/projection_raw.png", image);

  bool dilated_ok = cv::imwrite("/home/hxy/Desktop/projection_dilated.png", dilated);


  ROS_DEBUG("Saved raw projection image: %s", raw_ok ? "YES" : "NO");
  ROS_DEBUG("Saved dilated image: %s", dilated_ok ? "YES" : "NO");

}

std::string 
cw2::classifyObjectByPointCount(size_t num_points) {
  if (num_points > 20000) return "nought";
  if (num_points > 8000)  return "cross";
  return "unknown";
}

void 
cw2::updateMergedObjectsFromSingleView(const std::vector<PointCPtr> &clusters) {
  computeObjectCenters(clusters);

  int basket_index = -1;
  int max_basket_points = -1;

  for (size_t i = 0; i < detected_objects.size(); ++i) {
    const auto& obj = detected_objects[i];

    std::string shape;
    if (obj.color == "darkred") {
      shape = "basket";
      // If the current object is the largest basket, keep it
      if ((int)clusters[i]->points.size() > max_basket_points) {
        max_basket_points = clusters[i]->points.size();
        basket_index = i;
      }
      continue;  // Skip for now, will be added later
    }
    else if (obj.color == "black") {
      shape = "obstacle";
    }
    else {
      shape = classifyObjectByPointCount(clusters[i]->points.size());
    }

    // Check if it matches an existing merged object
    bool matched = false;
    for (auto& merged : merged_objects) {
      double dist = std::hypot(merged.center_x - obj.center_x,
                               merged.center_y - obj.center_y);
      if (dist < 0.03) {
        if (shape == merged.shape) merged.vote_count++;
        else merged.vote_count--;
        matched = true;
        break;
      }
    }

    if (!matched) {
      merged_objects.push_back({
        obj.center_x,
        obj.center_y,
        (int)clusters[i]->points.size(),
        shape,
        obj.color,
        1,
        {(int)clusters[i]->points.size()}
      });
    }
  }

  // Handle the basket (only keep the largest one)
  if (basket_index != -1) {
    const auto& basket_obj = detected_objects[basket_index];
    merged_objects.push_back({
      basket_obj.center_x,
      basket_obj.center_y,
      (int)clusters[basket_index]->points.size(),
      "basket",
      basket_obj.color,
      1,
      {(int)clusters[basket_index]->points.size()}
    });
  }
}


void 
cw2::mergeNearbyMergedObjectsWithPriorityVoting(float dist_threshold) {
  std::vector<MultiViewObject> merged_final;

  for (const auto& current : merged_objects) {
    bool merged = false;

    for (auto& final : merged_final) {
      double dist = std::hypot(current.center_x - final.center_x,
                               current.center_y - final.center_y);
      if (dist < dist_threshold) {
        int original_final_count = final.point_count;
        final.point_count += current.point_count;

        final.point_counts_history.insert(final.point_counts_history.end(),
          current.point_counts_history.begin(),
          current.point_counts_history.end());
        final.vote_count += current.vote_count;

        if (current.point_count > original_final_count) {
          final.center_x = current.center_x;
          final.center_y = current.center_y;
        }

        // Shape logic
        if (current.shape == "nought" || final.shape == "nought") {
          final.shape = "nought";
        } else if (current.shape == "cross" || final.shape == "cross") {
          final.shape = "cross";
        }

        // Color logic
        if (final.color == "unknown" && current.color != "unknown") {
          final.color = current.color;
        }

        merged = true;
        break;
      }
    }

    if (!merged) {
      merged_final.push_back(current);
    }
  }

  int max_basket_idx = -1;
  int max_basket_points = -1;

  for (size_t i = 0; i < merged_final.size(); ++i) {
    if (merged_final[i].shape == "basket" && merged_final[i].point_count > max_basket_points) {
      max_basket_points = merged_final[i].point_count;
      max_basket_idx = i;
    }
  }

  // Remove other baskets, keeping only the largest one
  if (max_basket_idx != -1) {
    std::vector<MultiViewObject> filtered;
    for (size_t i = 0; i < merged_final.size(); ++i) {
      if (merged_final[i].shape != "basket" || i == max_basket_idx) {
        filtered.push_back(merged_final[i]);
      }
    }
    merged_final = filtered;
  }

  merged_objects = merged_final;
}

void 
cw2::finalizeVotingResults() {
  ROS_INFO("==== Voting Results ====");

  for (const auto& obj : merged_objects) {
    if (obj.shape == "unknown") {
      continue;  // Skip unknown shapes
    }

    std::ostringstream oss;
    for (size_t i = 0; i < obj.point_counts_history.size(); ++i) {
      oss << obj.point_counts_history[i];
      if (i != obj.point_counts_history.size() - 1)
        oss << " + ";
    }

    ROS_INFO_STREAM("Merged Object -> Shape: " << obj.shape
                    << " | Color: " << obj.color
                    << " | Center: (" << std::fixed << std::setprecision(3)
                    << obj.center_x << ", " << obj.center_y << ")");
  }
}
