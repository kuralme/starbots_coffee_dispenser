#include "pick_and_place.hpp"

PickAndPlace::PickAndPlace(const rclcpp::NodeOptions &node_options)
    : Node("starbots_delivery_service_server", node_options),
      obj_pose_received_(false), goal_poses_received_(false)
{
  RCLCPP_INFO(LOGGER, "Initializing Starbots UR3e Coffee Dispenser...");
  this->declare_parameter("automatically_declare_parameters_from_overrides",
                          true);

  move_group_node_ =
      rclcpp::Node::make_shared("ur3e_move_group_node", node_options);

  // start move_group node in a seperate thread and spin it
  auto move_group_executor =
      std::make_shared<rclcpp::executors::MultiThreadedExecutor>(rclcpp::ExecutorOptions(), 2);
  move_group_executor->add_node(move_group_node_);
  std::thread([move_group_executor]()
              { move_group_executor->spin(); })
      .detach();

  // initialize move_group interfaces
  move_group_robot_ = std::make_shared<MoveGroupInterface>(
      move_group_node_, PLANNING_GROUP_ROBOT);
  move_group_gripper_ = std::make_shared<MoveGroupInterface>(
      move_group_node_, PLANNING_GROUP_GRIPPER);

  // get initial state of robot and gripper
  move_group_robot_->startStateMonitor();
  joint_model_group_robot_ =
      move_group_robot_->getCurrentState()->getJointModelGroup(
          PLANNING_GROUP_ROBOT);
  joint_model_group_gripper_ =
      move_group_gripper_->getCurrentState()->getJointModelGroup(
          PLANNING_GROUP_GRIPPER);

  RCLCPP_INFO(LOGGER, "Planning Frame: %s",
              move_group_robot_->getPlanningFrame().c_str());
  RCLCPP_INFO(LOGGER, "End Effector Link: %s",
              move_group_robot_->getEndEffectorLink().c_str());
  RCLCPP_INFO(LOGGER, "Available Planning Groups:");
  std::vector<std::string> group_names =
      move_group_robot_->getJointModelGroupNames();
  for (long unsigned int i = 0; i < group_names.size(); i++)
  {
    RCLCPP_INFO(LOGGER, "Group %ld: %s", i, group_names[i].c_str());
  }

  // get current state of robot and gripper
  robot_current_state_ = move_group_robot_->getCurrentState(10);
  robot_current_state_->copyJointGroupPositions(joint_model_group_robot_,
                                                joint_group_positions_robot_);
  gripper_current_state_ = move_group_gripper_->getCurrentState(10);
  gripper_current_state_->copyJointGroupPositions(
      joint_model_group_gripper_, joint_group_positions_gripper_);
  move_group_robot_->setStartStateToCurrentState();
  move_group_gripper_->setStartStateToCurrentState();
  move_group_robot_->setPlanningTime(20.0);

  // Prepare ROS2 communiation
  rclcpp::CallbackGroup::SharedPtr callback_group;
  callback_group = move_group_node_->create_callback_group(
      rclcpp::CallbackGroupType::Reentrant);
  rclcpp::SubscriptionOptions sub_options;
  sub_options.callback_group = callback_group;

  objpose_sub_ = move_group_node_->create_subscription<
      starbots_detection_msgs::msg::DetectedObjects>(
      "/cup_detected", 10,
      std::bind(&PickAndPlace::objectDetectionCallback, this, _1),
      sub_options);
  holepose_sub_ = move_group_node_->create_subscription<
      starbots_detection_msgs::msg::DetectedCupholders>(
      "/cup_holder_detected", 100,
      std::bind(&PickAndPlace::holeDetectionCallback, this, _1), sub_options);
  octo_client_ =
      move_group_node_->create_client<std_srvs::srv::Empty>("/clear_octomap");

  createSceneObjects();
  gotoPredefined("quick_pick");
  RCLCPP_INFO(LOGGER, "UR3e ready for coffee delivery");
}
PickAndPlace::~PickAndPlace()
{
  RCLCPP_INFO(LOGGER, "Class Terminated: UR3e Coffee Dispenser");
}

void PickAndPlace::objectDetectionCallback(
    const starbots_detection_msgs::msg::DetectedObjects::SharedPtr msg)
{
  if (!obj_pose_received_)
  {
    obj_position_ = msg->position;
    obj_radius_ = msg->width / 2.;
    obj_thickness_ = msg->thickness;
    obj_height_ = msg->height;
    obj_pose_received_ = true;

    // RCLCPP_INFO(LOGGER, "===========================");
    // RCLCPP_INFO(LOGGER, "Cup detected");
    // RCLCPP_INFO(LOGGER, "Position: (%.2f, %.2f, %.2f)", obj_position_.x,
    //             obj_position_.y, obj_position_.z);
    // RCLCPP_INFO(LOGGER, "Radius: %.4f", obj_radius_);
    // RCLCPP_INFO(LOGGER, "Height: %.2f", obj_height_);
    // RCLCPP_INFO(LOGGER, "===========================");
  }
}
void PickAndPlace::holeDetectionCallback(
    const starbots_detection_msgs::msg::DetectedCupholders::SharedPtr msg)
{
  if (!goal_poses_received_)
  {
    for (const auto &cupholder : msg->cup_holders)
    {
      goal_poses_.push_back(cupholder.position);

      // RCLCPP_INFO(LOGGER, "===========================");
      // RCLCPP_INFO(LOGGER, "Cupholder ID: %u", cupholder.cupholder_id);
      // RCLCPP_INFO(LOGGER, "Position: (%.2f, %.2f, %.2f)", cupholder.position.x, cupholder.position.y, cupholder.position.z);
      // RCLCPP_INFO(LOGGER, "Radius: %.2f", cupholder.radius);
      // RCLCPP_INFO(LOGGER, "Height: %.2f", cupholder.height);
      // }
      // RCLCPP_INFO(LOGGER, "===========================");
      goal_poses_received_ = true;
    }
  }
}

void PickAndPlace::gotoPredefined(std::string pose_name)
{
  // Move robot(joints) to predefined home configuration
  RCLCPP_INFO(LOGGER, "Going to '%s' Pose...", pose_name.c_str());
  RCLCPP_INFO(LOGGER, "Preparing Joint Value Trajectory...");
  move_group_robot_->setNamedTarget(pose_name);
  move_group_robot_->setStartStateToCurrentState();

  MoveGroupInterface::Plan kinematics_trajectory_plan;
  bool plan_success_robot_ =
      (move_group_robot_->plan(kinematics_trajectory_plan) ==
       moveit::core::MoveItErrorCode::SUCCESS);
  if (plan_success_robot_)
  {
    size_t num_points =
        kinematics_trajectory_plan.trajectory_.joint_trajectory.points.size();
    if (num_points > 1)
    {
      move_group_robot_->execute(kinematics_trajectory_plan);
    }
    else
    {
      RCLCPP_INFO(LOGGER, "Already at the '%s' pose.", pose_name.c_str());
    }
  }
}
bool PickAndPlace::executeKinematicsPlan(float pos_x, float pos_y, float pos_z)
{
  move_group_robot_->clearPoseTargets();
  robot_current_state_ = move_group_robot_->getCurrentState(10);
  move_group_robot_->setStartStateToCurrentState();

  geometry_msgs::msg::Pose target_pose;
  target_pose.position.x = pos_x;
  target_pose.position.y = pos_y;
  target_pose.position.z = pos_z;
  target_pose.orientation.x = -1.0;
  target_pose.orientation.y = 0.0;
  target_pose.orientation.z = 0.0;
  target_pose.orientation.w = 0.0;
  move_group_robot_->setPoseTarget(target_pose);

  MoveGroupInterface::Plan kinematics_trajectory_plan;
  bool plan_success_robot_ =
      (move_group_robot_->plan(kinematics_trajectory_plan) ==
       moveit::core::MoveItErrorCode::SUCCESS);

  if (plan_success_robot_)
  {
    size_t num_points =
        kinematics_trajectory_plan.trajectory_.joint_trajectory.points.size();
    RCLCPP_INFO(LOGGER, "Trajectory has %zu points", num_points);
    if (num_points > 3)
    {
      move_group_robot_->execute(kinematics_trajectory_plan);
      return true;
    }
    else
    {
      RCLCPP_WARN(LOGGER, "Trajectory too short — nothing to execute.");
      return false;
    }
  }
  else
  {
    RCLCPP_ERROR(LOGGER, "Planning failed — no execution.");
    return false;
  }
}
void PickAndPlace::executeCartesianPlan(float x_delta, float y_delta, float z_delta)
{
  move_group_robot_->clearPoseTargets();
  robot_current_state_ = move_group_robot_->getCurrentState(10);
  move_group_robot_->setStartStateToCurrentState();
  geometry_msgs::msg::Pose target_pose = move_group_robot_->getCurrentPose().pose;

  std::vector<geometry_msgs::msg::Pose> cartesian_waypoints;
  cartesian_waypoints.push_back(target_pose);
  tf2::Quaternion q;
  q.setRPY(M_PI, 0, 0);
  target_pose.orientation.x = q.x();
  target_pose.orientation.y = q.y();
  target_pose.orientation.z = q.z();
  target_pose.orientation.w = q.w();
  target_pose.position.x += x_delta;
  target_pose.position.y += y_delta;
  target_pose.position.z += z_delta;
  cartesian_waypoints.push_back(target_pose);

  RCLCPP_INFO(LOGGER, "Plan & Execute Cartesian Trajectory...");
  const double jump_threshold_ = 0.0;
  const double end_effector_step_ = 0.01;
  moveit_msgs::msg::RobotTrajectory cartesian_trajectory_plan_;
  double plan_fraction_robot_ = move_group_robot_->computeCartesianPath(
      cartesian_waypoints, end_effector_step_, jump_threshold_,
      cartesian_trajectory_plan_);

  // Check plan: 0.0 to 1.0 = success and -1.0 = failure
  if (plan_fraction_robot_ >= 0.0)
  {
    move_group_robot_->execute(cartesian_trajectory_plan_);
    RCLCPP_INFO(LOGGER, "Cartesian Trajectory Success !");
  }
  else
  {
    RCLCPP_INFO(LOGGER, "Cartesian Trajectory Planning Failed !");
  }
  cartesian_waypoints.clear();
}

void PickAndPlace::executeGripperPlan(std::string pose_name)
{
  MoveGroupInterface::Plan gripper_trajectory_plan;
  move_group_gripper_->setNamedTarget(pose_name);

  bool plan_success_gripper_ =
      (move_group_gripper_->plan(gripper_trajectory_plan) ==
       moveit::core::MoveItErrorCode::SUCCESS);

  if (plan_success_gripper_)
  {
    move_group_gripper_->execute(gripper_trajectory_plan);
    RCLCPP_INFO(LOGGER, "Gripper Action Command Success !");
  }
  else
  {
    RCLCPP_INFO(LOGGER, "Gripper Action Command Failed !");
  }
}
void PickAndPlace::closeGripperIncremental()
{
  float gripper_value = 0.4;
  const float target_value = 0.0;
  const float step_large = 0.07;
  const float step_medium = 0.02;
  const float step_small = 0.003;

  while (gripper_value > target_value)
  {
    joint_group_positions_gripper_[0] = gripper_value;
    move_group_gripper_->setJointValueTarget(joint_group_positions_gripper_);

    MoveGroupInterface::Plan plan;
    bool success = move_group_gripper_->plan(plan) ==
                   moveit::core::MoveItErrorCode::SUCCESS;

    if (success)
    {
      move_group_gripper_->execute(plan);
    }
    else
    {
      RCLCPP_ERROR(LOGGER, "Failed to plan gripper motion at value: %.3f",
                   gripper_value);
      break;
    }

    // Dynamically reduce step size for precision
    if (gripper_value > 0.1)
      gripper_value -= step_large;
    else if (gripper_value > 0.03)
      gripper_value -= step_medium;
    else
      gripper_value -= step_small;

    std::this_thread::sleep_for(std::chrono::milliseconds(300));
  }
}

void PickAndPlace::attachObject()
{
  // Add the cup to the planning scene
  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = primitive.CYLINDER;
  primitive.dimensions.resize(2);
  primitive.dimensions[primitive.CYLINDER_HEIGHT] = obj_height_;
  primitive.dimensions[primitive.CYLINDER_RADIUS] = obj_radius_;

  geometry_msgs::msg::Pose obj_pose;
  obj_pose.position = obj_position_;
  moveit_msgs::msg::CollisionObject collision_object;
  collision_object.header.frame_id = move_group_robot_->getPlanningFrame();
  collision_object.id = "coffee_cup";
  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(obj_pose);
  collision_object.operation = collision_object.ADD;

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  planning_scene_interface.applyCollisionObject(collision_object);

  // Attach the cup to gripper
  moveit_msgs::msg::AttachedCollisionObject attached_object;
  attached_object.link_name = move_group_robot_->getEndEffectorLink();
  attached_object.object = collision_object;
  attached_object.touch_links = {"rg2_gripper_right_thumb",
                                 "rg2_gripper_left_thumb"};
  planning_scene_interface.applyAttachedCollisionObject(attached_object);

  RCLCPP_INFO(LOGGER, "Cup attached to gripper");
}
void PickAndPlace::detachObject()
{
  // Detach the cup from the gripper
  moveit_msgs::msg::AttachedCollisionObject detach_object;
  detach_object.link_name = move_group_robot_->getEndEffectorLink();
  detach_object.object.id = "coffee_cup";
  detach_object.object.operation = moveit_msgs::msg::CollisionObject::REMOVE;

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  planning_scene_interface.applyAttachedCollisionObject(detach_object);

  // Remove the cup from the world
  planning_scene_interface.removeCollisionObjects({"coffee_cup"});
  RCLCPP_INFO(LOGGER, "Removed cup from planning scene");

  RCLCPP_INFO(LOGGER, "Cup detached from gripper");
}
void PickAndPlace::createSceneObjects()
{
  // Create coffee counter part that isnt visible in pointcloud
  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions = {0.65, 0.55, 0.05};
  geometry_msgs::msg::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.x = 0.2;
  box_pose.position.y = -0.2;
  box_pose.position.z = -0.05;

  moveit_msgs::msg::CollisionObject collision_object;
  collision_object.header.frame_id = move_group_robot_->getPlanningFrame();
  collision_object.id = "coffee_counter_partial";
  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  planning_scene_interface.applyCollisionObject(collision_object);
}

void PickAndPlace::clearOctomap()
{
  auto request = std::make_shared<std_srvs::srv::Empty::Request>();
  if (!octo_client_->wait_for_service(std::chrono::seconds(2)))
  {
    RCLCPP_WARN(LOGGER, "clear_octomap service not available");
    return;
  }
  auto future = octo_client_->async_send_request(request);
  RCLCPP_INFO(LOGGER, "Octomap cleared!");
}
void PickAndPlace::createTrajectoryConstraint()
{
  // Constraint for planning trajectory
  shape_msgs::msg::SolidPrimitive box;
  box.type = shape_msgs::msg::SolidPrimitive::BOX;
  box.dimensions = {2.0, 2.0, 2.0};
  geometry_msgs::msg::Pose box_pose;
  box_pose.position.x = -0.1;
  box_pose.position.y = 0.3;
  box_pose.orientation.w = 1.0;
  displayBoxConstraint(box_pose, box.dimensions);

  moveit_msgs::msg::PositionConstraint box_constraint;
  box_constraint.header.frame_id = move_group_robot_->getPlanningFrame();
  box_constraint.link_name = move_group_robot_->getEndEffectorLink();
  box_constraint.constraint_region.primitives.push_back(box);
  box_constraint.constraint_region.primitive_poses.push_back(box_pose);
  box_constraint.weight = 1.0;

  path_constraints_.position_constraints.push_back(box_constraint);
  move_group_robot_->setPathConstraints(path_constraints_);
  move_group_robot_->setPlanningTime(20.0);
}
void PickAndPlace::createOrientationConstraint()
{
  // "Gripper pointing down" orientation constraint
  moveit_msgs::msg::OrientationConstraint ocm;
  ocm.link_name = move_group_robot_->getEndEffectorLink();
  ocm.header.frame_id = move_group_robot_->getPlanningFrame();
  tf2::Quaternion q;
  q.setRPY(M_PI, 0, 0);
  ocm.orientation.x = q.x();
  ocm.orientation.y = q.y();
  ocm.orientation.z = q.z();
  ocm.orientation.w = q.w();
  ocm.absolute_x_axis_tolerance = 0.1;
  ocm.absolute_y_axis_tolerance = 0.1;
  ocm.absolute_z_axis_tolerance = M_PI;
  ocm.weight = 1.0;
  path_constraints_.orientation_constraints.push_back(ocm);

  // Changed planner for better orientation constrained planning
  move_group_robot_->setPlannerId("KPIECEkConfigDefault");
  move_group_robot_->setPathConstraints(path_constraints_);
  RCLCPP_INFO(LOGGER, "Applied upright orientation constraint (Z down)");
}
void PickAndPlace::clearOrientationConstraints()
{
  // move_group_robot_->clearPathConstraints();
  path_constraints_.orientation_constraints.clear();
  move_group_robot_->setPathConstraints(path_constraints_);
  move_group_robot_->setPlannerId("BiTRRTkConfigDefault");
  RCLCPP_INFO(LOGGER, "Cleared Orientation constraints");
}
void PickAndPlace::displayBoxConstraint(
    const geometry_msgs::msg::Pose &pose,
    const rosidl_runtime_cpp::BoundedVector<double, 3, std::allocator<double>>
        &dimensions)
{
  // Publish trajectory space constraint marker
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = move_group_robot_->getPlanningFrame();
  marker.header.stamp = move_group_node_->now();
  marker.ns = "/";
  marker.id = 1;

  marker.type = visualization_msgs::msg::Marker::CUBE;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.lifetime = rclcpp::Duration::from_seconds(0.0);

  marker.color.a = 0.2;
  marker.pose = pose;
  marker.scale.x = dimensions.at(0);
  marker.scale.y = dimensions.at(1);
  marker.scale.z = dimensions.at(2);

  constraint_marker_pub_->publish(marker);
}