#include "pick_and_place.hpp"

PickAndPlace::PickAndPlace(const rclcpp::NodeOptions &node_options)
    : Node("starbots_delivery_service_server", node_options),
      goal_poses_received_(false) {

  RCLCPP_INFO(LOGGER, "Initializing Starbots UR3e Coffee Dispenser...");
  this->declare_parameter("automatically_declare_parameters_from_overrides",
                          true);

  move_group_node_ =
      rclcpp::Node::make_shared("ur3e_move_group_node", node_options);

  // start move_group node in a seperate thread and spin it
  auto move_group_executor =
      std::make_shared<rclcpp::executors::MultiThreadedExecutor>(
          rclcpp::ExecutorOptions(), 2);
  move_group_executor->add_node(move_group_node_);
  std::thread([move_group_executor]() {
    move_group_executor->spin();
  }).detach();

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
  for (long unsigned int i = 0; i < group_names.size(); i++) {
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
  move_group_robot_->setPlanningTime(15.0);

  // Prepare ROS2 communiation
  holepose_sub_ = move_group_node_->create_subscription<
      starbots_detection_msgs::msg::DetectedCupholders>(
      "/cup_holder_detected", 100,
      std::bind(&PickAndPlace::holeDetectionCallback, this, _1));
  octo_client_ =
      move_group_node_->create_client<std_srvs::srv::Empty>("/clear_octomap");

  // Hardcoded coffee cup position
  cup_position_.x = 0.222; // guess 0.23
  cup_position_.y = 0.334; // guess 0.33
  cup_position_.z = 0.07;

  createSceneObjects();
  if (gotoPredefined("quick_pick")) {
    RCLCPP_INFO(LOGGER, "UR3e ready for coffee delivery");
  } else {
    RCLCPP_ERROR(LOGGER, "UR3e not ready for coffee delivery!");
  }
}
PickAndPlace::~PickAndPlace() {
  RCLCPP_INFO(LOGGER, "Class Terminated: UR3e Coffee Dispenser");
}

void PickAndPlace::holeDetectionCallback(
    const starbots_detection_msgs::msg::DetectedCupholders::SharedPtr msg) {
  if (!goal_poses_received_) {
    for (const auto &cupholder : msg->cup_holders) {
      goal_poses_.push_back(cupholder.position);
      // RCLCPP_INFO(LOGGER, "===========================");
      // RCLCPP_INFO(LOGGER, "Cupholder ID: %u", cupholder.cupholder_id);
      // RCLCPP_INFO(LOGGER, "Position: (%.2f, %.2f, %.2f)",
      // cupholder.position.x, cupholder.position.y,
      // cupholder.position.z);
      // RCLCPP_INFO(LOGGER, "Radius: %.2f", cupholder.radius);
      // RCLCPP_INFO(LOGGER, "Height: %.2f", cupholder.height);
    }
    //   RCLCPP_INFO(LOGGER, "===========================");
    goal_poses_received_ = true;
  }
}
bool PickAndPlace::gotoPredefined(std::string pose_name) {
  // Move robot(joints) to predefined configuration
  RCLCPP_INFO(LOGGER, "Going to '%s' Pose...", pose_name.c_str());
  move_group_robot_->setNamedTarget(pose_name);
  move_group_robot_->setStartStateToCurrentState();

  MoveGroupInterface::Plan kinematics_trajectory_plan;
  bool plan_success_robot_ =
      (move_group_robot_->plan(kinematics_trajectory_plan) ==
       moveit::core::MoveItErrorCode::SUCCESS);
  if (plan_success_robot_) {
    size_t num_points =
        kinematics_trajectory_plan.trajectory_.joint_trajectory.points.size();
    if (num_points > 1) {
      if (move_group_robot_->execute(kinematics_trajectory_plan) ==
          moveit::core::MoveItErrorCode::SUCCESS)
        return true;
      else
        return false;
    } else {
      RCLCPP_INFO(LOGGER, "Already at the '%s' pose.", pose_name.c_str());
      return true;
    }
  } else {
    return false;
  }
}
bool PickAndPlace::executeKinematicsPlan(float pos_x, float pos_y,
                                         float pos_z) {

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
  bool plan_success_robot =
      (move_group_robot_->plan(kinematics_trajectory_plan) ==
       moveit::core::MoveItErrorCode::SUCCESS);

  if (plan_success_robot) {
    size_t num_points =
        kinematics_trajectory_plan.trajectory_.joint_trajectory.points.size();
    RCLCPP_INFO(LOGGER, "Trajectory has %zu points", num_points);
    if (num_points > 3) {
      move_group_robot_->execute(kinematics_trajectory_plan);
      move_group_robot_->clearPoseTargets();
      return true;
    } else {
      RCLCPP_WARN(LOGGER, "Trajectory too short — nothing to execute.");
      move_group_robot_->clearPoseTargets();
      return false;
    }
  } else {
    RCLCPP_WARN(LOGGER, "Planning failed — no execution.");
    move_group_robot_->clearPoseTargets();
    return false;
  }
}
void PickAndPlace::executeCartesianPlan(float x_delta, float y_delta,
                                        float z_delta) {

  move_group_robot_->clearPoseTargets();
  robot_current_state_ = move_group_robot_->getCurrentState(10);
  move_group_robot_->setStartStateToCurrentState();
  geometry_msgs::msg::Pose target_pose =
      move_group_robot_->getCurrentPose().pose;

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

  const double jump_threshold = 0.0;
  const double end_effector_step = 0.01;
  const bool collision_aware = true;
  moveit_msgs::msg::RobotTrajectory cartesian_trajectory_plan;
  double plan_fraction_robot_ = move_group_robot_->computeCartesianPath(
      cartesian_waypoints, end_effector_step, jump_threshold,
      cartesian_trajectory_plan, collision_aware);

  // Check plan: 0.0 to 1.0 = success and -1.0 = failure
  if (plan_fraction_robot_ >= 0.0) {
    move_group_robot_->execute(cartesian_trajectory_plan);
  } else {
    cartesian_waypoints.clear();
    // throw std::runtime_error("Cartesian planning failed !");
  }
  cartesian_waypoints.clear();
}
void PickAndPlace::executeGripperPlan(std::string pose_name) {
  MoveGroupInterface::Plan gripper_trajectory_plan;
  move_group_gripper_->setNamedTarget(pose_name);

  bool plan_success_gripper_ =
      (move_group_gripper_->plan(gripper_trajectory_plan) ==
       moveit::core::MoveItErrorCode::SUCCESS);

  if (plan_success_gripper_) {
    move_group_gripper_->execute(gripper_trajectory_plan);
    RCLCPP_INFO(LOGGER, "Gripper Action Command Success !");
  } else {
    RCLCPP_INFO(LOGGER, "Gripper Action Command Failed !");
  }
}
void PickAndPlace::closeGripperIncremental() {
  float gripper_value = 0.4;
  const float target_value = -0.1; // ~gripper_grasp
  const float step_large = 0.2;
  const float step_medium = 0.06;

  while (gripper_value > target_value) {
    // Dynamically reduce step size for precision
    if (gripper_value > 0.1)
      gripper_value -= step_large;
    else if (gripper_value > 0.05)
      gripper_value -= step_medium;

    joint_group_positions_gripper_[0] = gripper_value;
    move_group_gripper_->setJointValueTarget(joint_group_positions_gripper_);

    MoveGroupInterface::Plan plan;
    bool success = move_group_gripper_->plan(plan) ==
                   moveit::core::MoveItErrorCode::SUCCESS;

    if (success) {
      move_group_gripper_->execute(plan);
    } else {
      RCLCPP_ERROR(LOGGER, "Failed to plan gripper motion at value: %.3f",
                   gripper_value);
      break;
    }
  }
}

void PickAndPlace::attachGripperObject(const std::string &object_id) {
  // Add the cup to the planning scene
  geometry_msgs::msg::Pose cup_pose;
  cup_pose.position = cup_position_;
  std::vector<double> cup_dimensions = {0.11, 0.04}; // {height, width}
  moveit_msgs::msg::CollisionObject coffee_cup =
      createCollisionObject(object_id, "cylinder", cup_dimensions, cup_pose);

  // Attach the cup to gripper
  moveit_msgs::msg::AttachedCollisionObject attached_object;
  attached_object.link_name = move_group_robot_->getEndEffectorLink();
  attached_object.object = coffee_cup;
  attached_object.touch_links = {"rg2_gripper_right_thumb",
                                 "rg2_gripper_left_thumb"};
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  planning_scene_interface.applyAttachedCollisionObject(attached_object);

  RCLCPP_INFO(LOGGER, "Cup attached to gripper");
}
void PickAndPlace::detachGripperObject(const std::string &object_id) {
  // Detach the cup from the gripper
  moveit_msgs::msg::AttachedCollisionObject detach_object;
  detach_object.link_name = move_group_robot_->getEndEffectorLink();
  detach_object.object.id = object_id;
  detach_object.object.operation = moveit_msgs::msg::CollisionObject::REMOVE;

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  planning_scene_interface.applyAttachedCollisionObject(detach_object);

  // Remove the cup from the world
  planning_scene_interface.removeCollisionObjects({object_id});
  RCLCPP_INFO(LOGGER, "Removed cup from planning scene");

  RCLCPP_INFO(LOGGER, "Cup detached from gripper");
}
void PickAndPlace::createSceneObjects() {
  // Create scene objects arent visible in pointcloud
  geometry_msgs::msg::Pose counter_pose;
  counter_pose.orientation.w = 1.0;
  counter_pose.position.x = 0.25;
  counter_pose.position.y = 0.30;
  counter_pose.position.z = -0.03;
  std::vector<double> counter_dimensions = {0.68, 1.6,
                                            0.06}; // {length, width, height}
  const moveit_msgs::msg::CollisionObject coffee_counter =
      createCollisionObject("coffee_counter", "box", counter_dimensions,
                            counter_pose);

  geometry_msgs::msg::Pose clamp1_pose;
  clamp1_pose.orientation.w = 1.0;
  clamp1_pose.position.x = -0.08;
  clamp1_pose.position.y = 0.6;
  clamp1_pose.position.z = -0.03;
  std::vector<double> clamp1_dim = {0.06, 0.02,
                                    0.12}; // {length, width, height}
  const moveit_msgs::msg::CollisionObject clamp1 =
      createCollisionObject("clamp1", "box", clamp1_dim, clamp1_pose);

  geometry_msgs::msg::Pose clamp2_pose;
  tf2::Quaternion q;
  q.setRPY(0, 0, -M_PI / 4.);
  clamp2_pose.orientation.x = q.x();
  clamp2_pose.orientation.y = q.y();
  clamp2_pose.orientation.z = q.z();
  clamp2_pose.orientation.w = q.w();
  clamp2_pose.position.x = -0.08;
  clamp2_pose.position.y = 0.15;
  clamp2_pose.position.z = -0.03;
  std::vector<double> clamp2_dimentions = {0.06, 0.02,
                                           0.12}; // {length, width, height}
  const moveit_msgs::msg::CollisionObject clamp2 =
      createCollisionObject("clamp2", "box", clamp2_dimentions, clamp2_pose);

  geometry_msgs::msg::Pose wall_pose;
  wall_pose.orientation.w = 1.0;
  wall_pose.position.x = 0.0;
  wall_pose.position.y = -0.45;
  wall_pose.position.z = 0.0;
  std::vector<double> wall_dim = {1.5, 0.1, 1.6}; // {length, width, height}
  const moveit_msgs::msg::CollisionObject wall =
      createCollisionObject("wall", "box", wall_dim, wall_pose);

  geometry_msgs::msg::Pose machine_pose;
  machine_pose.orientation.w = 1.0;
  machine_pose.position.x = 0.3;
  machine_pose.position.y = 0.95;
  machine_pose.position.z = 0.14;
  std::vector<double> machine_dim = {0.5, 0.22,
                                     0.25}; // {length, width, height}
  const moveit_msgs::msg::CollisionObject coffee_machine =
      createCollisionObject("coffee_machine", "box", machine_dim, machine_pose);

  geometry_msgs::msg::Pose cupdisp_pose;
  cupdisp_pose.orientation.w = 1.0;
  cupdisp_pose.position.x = 0.25;
  cupdisp_pose.position.y = -0.33;
  cupdisp_pose.position.z = 0.55;
  std::vector<double> cuprack_dim = {0.5, 0.07}; // {height, radius}
  const moveit_msgs::msg::CollisionObject cup_dispenser = createCollisionObject(
      "cup_dispenser", "cylinder", cuprack_dim, cupdisp_pose);

  std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
  collision_objects.push_back(wall);
  collision_objects.push_back(coffee_counter);
  collision_objects.push_back(clamp1);
  collision_objects.push_back(clamp2);
  collision_objects.push_back(coffee_machine);
  collision_objects.push_back(cup_dispenser);

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  planning_scene_interface.addCollisionObjects(collision_objects);
  // Wait for MoveGroup to recieve and process the collision object message.
  rclcpp::sleep_for(std::chrono::seconds(1));
}
moveit_msgs::msg::CollisionObject PickAndPlace::createCollisionObject(
    const std::string &object_id,
    const std::string &type,               // Object type: "box" or "cylinder"
    const std::vector<double> &dimensions, // Dimensions of the object
    const geometry_msgs::msg::Pose &pose)  // Pose of the object
{
  moveit_msgs::msg::CollisionObject collision_object;
  shape_msgs::msg::SolidPrimitive primitive;

  // Ensure that dimensions are correctly converted
  if (type == "box") {
    primitive.type = primitive.BOX;
    if (dimensions.size() == 3) {
      // Assuming dimensions are {length, width, height}
      primitive.dimensions.resize(3);
      primitive.dimensions[0] = dimensions[0]; // length
      primitive.dimensions[1] = dimensions[1]; // width
      primitive.dimensions[2] = dimensions[2]; // height
    } else {
      RCLCPP_ERROR(
          LOGGER, "Box requires exactly 3 dimensions: {length, width, height}");
      return collision_object;
    }
  } else if (type == "cylinder") {
    primitive.type = primitive.CYLINDER;
    if (dimensions.size() == 2) {
      // Assuming dimensions are {height, radius}
      primitive.dimensions.resize(2);
      primitive.dimensions[primitive.CYLINDER_HEIGHT] = dimensions[0];
      primitive.dimensions[primitive.CYLINDER_RADIUS] = dimensions[1];
    } else {
      RCLCPP_ERROR(LOGGER,
                   "Cylinder requires exactly 2 dimensions: {height, radius}");
      return collision_object;
    }
  } else {
    RCLCPP_ERROR(LOGGER, "Unsupported object type: %s", type.c_str());
    return collision_object;
  }

  // Set up collision object properties
  collision_object.header.frame_id = move_group_robot_->getPlanningFrame();
  collision_object.id = object_id;
  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(pose);
  collision_object.operation = collision_object.ADD;

  RCLCPP_INFO(LOGGER, "Added %s into the scene", object_id.c_str());
  return collision_object;
}
void PickAndPlace::clearOctomap() {
  auto request = std::make_shared<std_srvs::srv::Empty::Request>();
  if (!octo_client_->wait_for_service(std::chrono::seconds(2))) {
    RCLCPP_WARN(LOGGER, "clear_octomap service not available");
    return;
  }
  auto future = octo_client_->async_send_request(request);
  RCLCPP_INFO(LOGGER, "Octomap cleared!");
}
void PickAndPlace::createOrientationConstraint() {
  moveit_msgs::msg::OrientationConstraint ocm;
  ocm.link_name = move_group_robot_->getEndEffectorLink();
  ocm.header.frame_id = move_group_robot_->getPlanningFrame();

  // Enforce Z-axis down (gripper pointing down)
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
void PickAndPlace::clearOrientationConstraints() {
  // move_group_robot_->clearPathConstraints();
  path_constraints_.orientation_constraints.clear();
  move_group_robot_->setPathConstraints(path_constraints_);
  move_group_robot_->setPlannerId("BiTRRTkConfigDefault");
  RCLCPP_INFO(LOGGER, "Cleared Orientation constraints");
}