#include <chrono>
#include <cmath>
#include <memory>
#include <thread>
#include <vector>

#include <custom_msgs/msg/detected_objects.hpp>
#include <custom_msgs/msg/detected_surfaces.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_msgs/msg/position_constraint.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <visualization_msgs/msg/marker.hpp>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_node");
static const std::string PLANNING_GROUP_ROBOT = "ur_manipulator";
static const std::string PLANNING_GROUP_GRIPPER = "gripper";

using namespace std::chrono_literals;

class PickAndPlace {
public:
  PickAndPlace(rclcpp::Node::SharedPtr base_node_) : base_node_(base_node_) {
    RCLCPP_INFO(LOGGER, "Initializing Class: Pick And Place...");

    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);

    move_group_node_ =
        rclcpp::Node::make_shared("move_group_node", node_options);
    // start move_group node in a new executor thread and spin it
    executor_.add_node(move_group_node_);
    std::thread([this]() { this->executor_.spin(); }).detach();

    // initialize move_group interfaces
    move_group_robot_ = std::make_shared<MoveGroupInterface>(
        move_group_node_, PLANNING_GROUP_ROBOT);
    move_group_gripper_ = std::make_shared<MoveGroupInterface>(
        move_group_node_, PLANNING_GROUP_GRIPPER);

    // get initial state of robot and gripper
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
    // more efficient method than std::copy() method used in the docs
    for (long unsigned int i = 0; i < group_names.size(); i++) {
      RCLCPP_INFO(LOGGER, "Group %ld: %s", i, group_names[i].c_str());
    }

    // get current state of robot and gripper
    current_state_robot_ = move_group_robot_->getCurrentState(10);
    current_state_robot_->copyJointGroupPositions(joint_model_group_robot_,
                                                  joint_group_positions_robot_);
    current_state_gripper_ = move_group_gripper_->getCurrentState(10);
    current_state_gripper_->copyJointGroupPositions(
        joint_model_group_gripper_, joint_group_positions_gripper_);
    move_group_robot_->setStartStateToCurrentState();
    move_group_gripper_->setStartStateToCurrentState();

    objpose_sub_ = move_group_node_
                       ->create_subscription<custom_msgs::msg::DetectedObjects>(
                           "/object_detected", 10,
                           std::bind(&PickAndPlace::objectDetectionCallback,
                                     this, std::placeholders::_1));
    marker_pub_ =
        move_group_node_->create_publisher<visualization_msgs::msg::Marker>(
            "/constraint_marker", rclcpp::QoS(10).transient_local());

    // Set box constraint as planning workspace
    createTrajectoryConstraint();

    RCLCPP_INFO(LOGGER, "Class Initialized: Pick And Place");
  }

  ~PickAndPlace() { RCLCPP_INFO(LOGGER, "Class Terminated: Pick And Place"); }

  void execute() {

    while (!obj_pose_received_) {
      RCLCPP_WARN(LOGGER, "Object Pose not received yet!");
      std::this_thread::sleep_for(std::chrono::milliseconds(3000));
    }
    float objX = obj_pose_.position.x;
    float objY = obj_pose_.position.y;
    float objZ = obj_pose_.position.z + .3;

    RCLCPP_INFO(LOGGER, "Planning and Executing Pick And Place...");

    RCLCPP_INFO(LOGGER, "Preparing Pregrasp Position...");
    RCLCPP_INFO(LOGGER, "Setting Goal Position: [%.3f, %.3f, %.3f]", objX, objY,
                objZ);
    setupPoseTarget(objX, objY, objZ, -1.000, 0.000, 0.000, 0.000);
    RCLCPP_INFO(LOGGER, "Planning Goal Pose Trajectory...");
    planTrajectoryKinematics();
    RCLCPP_INFO(LOGGER, "Executing Goal Pose Trajectory...");
    executeTrajectoryKinematics();

    RCLCPP_INFO(LOGGER, "Opening Gripper...");
    RCLCPP_INFO(LOGGER, "Preparing Gripper Value...");
    move_group_gripper_->setNamedTarget("gripper_open");
    RCLCPP_INFO(LOGGER, "Planning Gripper Action...");
    planTrajectoryGripper();
    RCLCPP_INFO(LOGGER, "Executing Gripper Action...");
    executeTrajectoryGripper();
    RCLCPP_INFO(LOGGER, "Gripper Opened");

    // Approach to grasping position
    RCLCPP_INFO(LOGGER, "Approaching to grasp...");
    RCLCPP_INFO(LOGGER, "Preparing Cartesian Trajectory...");
    setupWaypointTarget(+0.000, +0.000, -0.300);
    RCLCPP_INFO(LOGGER, "Planning Cartesian Trajectory...");
    planTrajectoryCartesian();
    RCLCPP_INFO(LOGGER, "Executing Cartesian Trajectory...");
    executeTrajectoryCartesian();

    // Incrementally close the gripper and pick the cup
    closeGripperIncremental();
    attachObject();

    RCLCPP_INFO(LOGGER, "Retreating...");
    RCLCPP_INFO(LOGGER, "Preparing Cartesian Trajectory...");
    setupWaypointTarget(+0.000, +0.000, +0.300);
    RCLCPP_INFO(LOGGER, "Planning Cartesian Trajectory...");
    planTrajectoryCartesian();
    RCLCPP_INFO(LOGGER, "Executing Cartesian Trajectory...");
    executeTrajectoryCartesian();

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    RCLCPP_INFO(LOGGER, "================================================");
    RCLCPP_INFO(LOGGER, "Going to the Dropping Position...");
    createOrientationConstraint();

    RCLCPP_INFO(LOGGER, "Preparing kinematic trajectory...");
    setupPoseTarget(-0.370, 0.000, -0.220, -1.000, 0.000, 0.000, 0.000);
    RCLCPP_INFO(LOGGER, "Planning Goal Pose Trajectory...");
    planTrajectoryKinematics();
    RCLCPP_INFO(LOGGER, "Executing Goal Pose Trajectory...");
    executeTrajectoryKinematics();

    clearConstraints();
    move_group_robot_->setPathConstraints(ws_constraints_);
    RCLCPP_INFO(LOGGER, "Reapplied workspace constraints");
    RCLCPP_INFO(LOGGER, "================================================");
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    // Open the gripper and drop the box
    RCLCPP_INFO(LOGGER, "Opening Gripper...");
    RCLCPP_INFO(LOGGER, "Preparing Gripper Value...");
    move_group_gripper_->setNamedTarget("gripper_open");
    RCLCPP_INFO(LOGGER, "Planning Gripper Action...");
    planTrajectoryGripper();
    RCLCPP_INFO(LOGGER, "Executing Gripper Action...");
    executeTrajectoryGripper();
    RCLCPP_INFO(LOGGER, "Gripper Opened");
    detachObject();

    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
  }
  void gotoHome() {
    // Move robot(joints) to predefined home configuration
    RCLCPP_INFO(LOGGER, "Going to Home Pose...");
    RCLCPP_INFO(LOGGER, "Preparing Joint Value Trajectory...");
    move_group_robot_->setNamedTarget("home");
    RCLCPP_INFO(LOGGER, "Planning Joint Value Trajectory...");
    planTrajectoryKinematics();
    RCLCPP_INFO(LOGGER, "Executing Joint Value Trajectory...");
    executeTrajectoryKinematics();
  }

private:
  void objectDetectionCallback(
      const custom_msgs::msg::DetectedObjects::SharedPtr msg) {
    if (!obj_pose_received_) {
      obj_pose_.position = msg->position;
      obj_radius_ = msg->width / 2.;
      obj_thickness_ = msg->thickness;
      obj_height_ = msg->height;
      obj_pose_received_ = true;
    }
  }

  void setupJointTarget(float angle0, float angle1, float angle2, float angle3,
                        float angle4, float angle5) {
    // set the joint values for each joint of robot arm
    joint_group_positions_robot_[0] = angle0; // Shoulder Pan
    joint_group_positions_robot_[1] = angle1; // Shoulder Lift
    joint_group_positions_robot_[2] = angle2; // Elbow
    joint_group_positions_robot_[3] = angle3; // Wrist 1
    joint_group_positions_robot_[4] = angle4; // Wrist 2
    joint_group_positions_robot_[5] = angle5; // Wrist 3
    move_group_robot_->setJointValueTarget(joint_group_positions_robot_);
  }

  void setupPoseTarget(float pos_x, float pos_y, float pos_z, float quat_x,
                       float quat_y, float quat_z, float quat_w) {
    // set the pose values for end effector of robot arm
    target_pose_robot_.position.x = pos_x;
    target_pose_robot_.position.y = pos_y;
    target_pose_robot_.position.z = pos_z;
    target_pose_robot_.orientation.x = quat_x;
    target_pose_robot_.orientation.y = quat_y;
    target_pose_robot_.orientation.z = quat_z;
    target_pose_robot_.orientation.w = quat_w;
    move_group_robot_->setPoseTarget(target_pose_robot_);
  }

  void planTrajectoryKinematics() {
    plan_success_robot_ =
        (move_group_robot_->plan(kinematics_trajectory_plan_) ==
         moveit::core::MoveItErrorCode::SUCCESS);

    if (plan_success_robot_) {
      RCLCPP_INFO(LOGGER, "Robot Kinematics Trajectory Success !");
    } else {
      RCLCPP_ERROR(LOGGER, "Robot Kinematics Trajectory Failed !");
    }
    auto traj_points =
        kinematics_trajectory_plan_.trajectory_.joint_trajectory.points.size();
    RCLCPP_INFO(LOGGER, "Planned trajectory points: %ld", traj_points);
  }

  void executeTrajectoryKinematics() {
    if (plan_success_robot_) {
      size_t num_points = kinematics_trajectory_plan_.trajectory_
                              .joint_trajectory.points.size();
      RCLCPP_INFO(LOGGER, "Trajectory has %zu points", num_points);
      if (num_points > 1) {
        move_group_robot_->execute(kinematics_trajectory_plan_);
        RCLCPP_INFO(LOGGER, "Executed trajectory.");
      } else {
        RCLCPP_WARN(LOGGER, "Trajectory too short — nothing to execute.");
      }
    } else {
      RCLCPP_ERROR(LOGGER, "Planning failed — no execution.");
    }
  }

  void setupWaypointTarget(float x_delta, float y_delta, float z_delta) {
    // initially set target pose to current pose of the robot
    target_pose_robot_ = move_group_robot_->getCurrentPose().pose;
    cartesian_waypoints_.push_back(target_pose_robot_);

    // add the desired pose to the target waypoints vector
    target_pose_robot_.position.x += x_delta;
    target_pose_robot_.position.y += y_delta;
    target_pose_robot_.position.z += z_delta;
    cartesian_waypoints_.push_back(target_pose_robot_);
  }

  void planTrajectoryCartesian() {
    plan_fraction_robot_ = move_group_robot_->computeCartesianPath(
        cartesian_waypoints_, end_effector_step_, jump_threshold_,
        cartesian_trajectory_plan_);
  }

  void executeTrajectoryCartesian() {
    if (plan_fraction_robot_ >= 0.0) {
      // 0.0 to 1.0 = success and -1.0 = failure
      move_group_robot_->execute(cartesian_trajectory_plan_);
      RCLCPP_INFO(LOGGER, "Robot Cartesian Trajectory Success !");
    } else {
      RCLCPP_INFO(LOGGER, "Robot Cartesian Trajectory Failed !");
    }
    // clear cartesian waypoints vector
    cartesian_waypoints_.clear();
  }

  bool executeGripperPlan() {
    return move_group_gripper_->plan(gripper_trajectory_plan_) ==
               moveit::core::MoveItErrorCode::SUCCESS &&
           move_group_robot_->execute(gripper_trajectory_plan_) ==
               moveit::core::MoveItErrorCode::SUCCESS;
  }
  void planTrajectoryGripper() {
    plan_success_gripper_ =
        (move_group_gripper_->plan(gripper_trajectory_plan_) ==
         moveit::core::MoveItErrorCode::SUCCESS);
  }
  void closeGripperIncremental() {
    float gripper_value = 0.4;
    const float target_value = 0.01; // ~gripper_grasp
    const float step_large = 0.06;
    const float step_medium = 0.01;
    const float step_small = 0.003;

    RCLCPP_INFO(LOGGER, "Incremental Gripper Closing Started...");

    while (gripper_value > target_value) {
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

      // Dynamically reduce step size for precision
      if (gripper_value > 0.1)
        gripper_value -= step_large;
      else if (gripper_value > 0.03)
        gripper_value -= step_medium;
      else
        gripper_value -= step_small;

      std::this_thread::sleep_for(std::chrono::milliseconds(300));
    }

    RCLCPP_INFO(LOGGER, "Incremental Gripper Closing Completed.");
  }
  void executeTrajectoryGripper() {
    if (plan_success_gripper_) {
      move_group_gripper_->execute(gripper_trajectory_plan_);
      RCLCPP_INFO(LOGGER, "Gripper Action Command Success !");
    } else {
      RCLCPP_INFO(LOGGER, "Gripper Action Command Failed !");
    }
  }

  void attachObject() {

    // Add the cup to the planning scene
    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = primitive.CYLINDER;
    primitive.dimensions.resize(2);
    primitive.dimensions[primitive.CYLINDER_HEIGHT] = obj_height_;
    primitive.dimensions[primitive.CYLINDER_RADIUS] = obj_radius_;

    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = move_group_robot_->getPlanningFrame();
    collision_object.id = "coffee_cup";
    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(obj_pose_);
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
  void detachObject() {

    // Detach the cup from the gripper
    moveit_msgs::msg::AttachedCollisionObject detach_object;
    detach_object.link_name = move_group_robot_->getEndEffectorLink();
    detach_object.object.id = "coffee_cup";
    detach_object.object.operation = moveit_msgs::msg::CollisionObject::REMOVE;

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    planning_scene_interface.applyAttachedCollisionObject(detach_object);

    // Remove the cup from the world
    // planning_scene_interface.removeCollisionObjects({"coffee_cup"});

    RCLCPP_INFO(LOGGER, "Cup detached from gripper");
  }

  void createTrajectoryConstraint() {
    // Constraint for planning trajectory
    shape_msgs::msg::SolidPrimitive box;
    box.type = shape_msgs::msg::SolidPrimitive::BOX;
    box.dimensions = {1.2, 1.0, 2.0};
    geometry_msgs::msg::Pose box_pose;
    box_pose.position.x = -0.1;
    box_pose.position.y = 0.3;
    box_pose.orientation.w = 1.0;
    displayBox(box_pose, box.dimensions);

    moveit_msgs::msg::PositionConstraint box_constraint;
    box_constraint.header.frame_id = move_group_robot_->getPlanningFrame();
    box_constraint.link_name = move_group_robot_->getEndEffectorLink();
    box_constraint.constraint_region.primitives.emplace_back(box);
    box_constraint.constraint_region.primitive_poses.emplace_back(box_pose);
    box_constraint.weight = 1.0;

    ws_constraints_.position_constraints.emplace_back(box_constraint);
    move_group_robot_->setStartStateToCurrentState();
    move_group_robot_->setPathConstraints(ws_constraints_);
  }
  void createOrientationConstraint() {
    // "Gripper pointing down" orientation constraint
    moveit_msgs::msg::OrientationConstraint ocm;
    ocm.link_name = move_group_robot_->getEndEffectorLink();
    ocm.header.frame_id = move_group_robot_->getPlanningFrame();
    ocm.orientation.x = -1.0;
    ocm.orientation.y = 0.0;
    ocm.orientation.z = 0.0;
    ocm.orientation.w = 0.0;
    ocm.absolute_x_axis_tolerance = 0.1;
    ocm.absolute_y_axis_tolerance = 0.1;
    ocm.absolute_z_axis_tolerance = M_PI;
    ocm.weight = 1.0;

    moveit_msgs::msg::Constraints constraints;
    constraints.orientation_constraints.push_back(ocm);
    move_group_robot_->setStartStateToCurrentState();
    move_group_robot_->setPathConstraints(constraints);

    RCLCPP_INFO(LOGGER, "Applied upright orientation constraint (Z down)");
  }
  void clearConstraints() {
    move_group_robot_->clearPathConstraints();
    RCLCPP_INFO(LOGGER, "Cleared path constraints");
  }
  void displayBox(
      const geometry_msgs::msg::Pose &pose,
      const rosidl_runtime_cpp::BoundedVector<double, 3, std::allocator<double>>
          &dimensions) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = move_group_robot_->getPlanningFrame();
    marker.header.stamp = move_group_node_->now();
    marker.ns = "/";
    marker.id = 1;

    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.lifetime = rclcpp::Duration::from_seconds(0.0);

    marker.color.a = 0.5;
    marker.pose = pose;
    marker.scale.x = dimensions.at(0);
    marker.scale.y = dimensions.at(1);
    marker.scale.z = dimensions.at(2);

    marker_pub_->publish(marker);
  }

  using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;
  using JointModelGroup = moveit::core::JointModelGroup;
  using RobotStatePtr = moveit::core::RobotStatePtr;
  using Plan = MoveGroupInterface::Plan;
  using Pose = geometry_msgs::msg::Pose;
  using RobotTrajectory = moveit_msgs::msg::RobotTrajectory;

  rclcpp::Node::SharedPtr base_node_;
  rclcpp::Node::SharedPtr move_group_node_;
  rclcpp::executors::SingleThreadedExecutor executor_;

  std::shared_ptr<MoveGroupInterface> move_group_robot_;
  std::shared_ptr<MoveGroupInterface> move_group_gripper_;
  const JointModelGroup *joint_model_group_robot_;
  const JointModelGroup *joint_model_group_gripper_;
  rclcpp::Subscription<custom_msgs::msg::DetectedObjects>::SharedPtr
      objpose_sub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  geometry_msgs::msg::Pose obj_pose_;
  moveit_msgs::msg::Constraints ws_constraints_;
  float obj_radius_, obj_thickness_, obj_height_;
  bool obj_pose_received_ = false;

  // declare trajectory planning variables for robot and gripper
  std::vector<double> joint_group_positions_robot_;
  RobotStatePtr current_state_robot_;
  Plan kinematics_trajectory_plan_;
  Pose target_pose_robot_;
  bool plan_success_robot_ = false;
  std::vector<double> joint_group_positions_gripper_;
  RobotStatePtr current_state_gripper_;
  Plan gripper_trajectory_plan_;
  bool plan_success_gripper_ = false;

  // declare cartesian trajectory planning variables for robot
  std::vector<Pose> cartesian_waypoints_;
  RobotTrajectory cartesian_trajectory_plan_;
  const double jump_threshold_ = 0.0;
  const double end_effector_step_ = 0.01;
  double plan_fraction_robot_ = 0.0;
}; // class PickAndPlace

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  std::shared_ptr<rclcpp::Node> base_node =
      std::make_shared<rclcpp::Node>("pick_and_place");

  PickAndPlace pick_and_place_trajectory_node(base_node);
  pick_and_place_trajectory_node.gotoHome();
  pick_and_place_trajectory_node.execute();
  pick_and_place_trajectory_node.gotoHome();
  RCLCPP_INFO(LOGGER, "Pick And Place Execution Complete");

  rclcpp::shutdown();
  return 0;
}